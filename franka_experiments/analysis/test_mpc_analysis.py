#!/usr/bin/env python3
"""Self-test of mpc_analysis.py and its helpers in analysis_utils.py (no pytest needed).

Synthetic MPC runs with a known truth: a 1 kHz plant that carries out the commanded q̈ with
a chosen latency, delay and gain plus a white disturbance, the MPC's q_des and plan
predicted from its state, an oscillation in u or in the shield's correction, costs built so
that η and α are known, multipliers with a slack in use, and old or broken runs.

    python3 test_mpc_analysis.py        (prints every check, exits 1 on any failure)
"""

from __future__ import annotations

import contextlib
import io
import os
import sys
import tempfile
import traceback
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import analysis_utils as u  # noqa: E402
import mpc_analysis  # noqa: E402

DT, N, H = 0.05, 10, 0.001       # MPC step, horizon, robot_state period
T0 = 12.0
Q_HOME = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.578, 0.785])
RHO = SIGMA = 1000.0


# ── Synthetic run ────────────────────────────────────────────────────────────

def write(df: pd.DataFrame, path: Path):
    df.to_csv(path, index=False)


def joint_frame(t, values, names, recv=None):
    cols = {"timestamp": t, "recv_time": t if recv is None else recv}
    for name, column in zip(names, values.T):
        cols[name] = column
    return pd.DataFrame(cols)


def make_mpc_run(root: Path, name: str, *, controller="mpc", seconds=60.0, latency=0.0, plant_delay=0.0,
                 gain=1.0, disturbance=0.05, oscillate_joint=None, shield_joint=None, plan_noise=0.0,
                 costs=True, drop_cycle=None, files=None, task_hz=None, compensate=False, seed=0):
    """Write a run folder; returns its path. Joints are 0-based in the arguments.

    compensate: the MPC predicts its state to when u_k arrives (input_delay = latency), as
    mpc_commander does with input_delay_s; else it plans as if u_k acted at once.
    """
    rng = np.random.default_rng(seed)
    run = root / controller / name
    run.mkdir(parents=True)
    K = int(round(seconds / DT))
    t_k = T0 + DT * np.arange(K)
    e = rng.normal(0.0, 1.0, (K, 7))
    U = np.zeros((K, 7))
    for k in range(1, K):
        U[k] = 0.8 * U[k - 1] + e[k]
    if oscillate_joint is not None:
        U[:, oscillate_joint] = 2.0 * np.sin(2 * np.pi * 3.0 * (t_k - T0)) + 0.1 * e[:, oscillate_joint]
    tau = t_k + latency                                        # u_k reaches the controller
    cmd_t, cmd_u = tau, U
    if shield_joint is not None:                               # the shield, at 200 Hz
        cmd_t = T0 + 0.005 * np.arange(int(seconds / 0.005))
        cmd_u = np.nan_to_num(u.zoh(tau, U, cmd_t, 1.0))
        cmd_u[:, shield_joint] += 1.5 * np.sign(np.sin(2 * np.pi * 2.0 * (cmd_t - T0) + 0.3))
    w = rng.normal(0.0, disturbance, (K, 7))                   # white, held over each MPC step

    n = K * int(round(DT / H))
    t = T0 + H * np.arange(n)
    a = gain * np.nan_to_num(u.zoh(cmd_t, cmd_u, t + 0.5 * H - plant_delay, 1.0))
    # The disturbance changes with the command, latency after each cycle
    a += w[np.clip(np.floor((t - T0 - latency) / DT + 1e-9).astype(int), 0, K - 1)]
    dq = np.vstack([np.zeros(7), H * np.cumsum(a, axis=0)])[:n]
    q = Q_HOME + np.vstack([np.zeros(7), np.cumsum(H * dq + 0.5 * H * H * a, axis=0)])[:n]
    write(joint_frame(t, np.column_stack([np.column_stack([q[:, j], dq[:, j], 0 * q[:, j]]) for j in range(7)]),
                      [f"{c}_{j}" for j in range(1, 8) for c in ("q", "dq", "tau")]), run / "experiment_robot_state.csv")

    i_k = np.round((t_k - T0) / H).astype(int)
    delay = latency if compensate else 0.0
    a_prev = np.vstack([np.zeros(7), U[:-1]])                 # in force at t_k: u_{k-1}
    q0 = q[i_k] + delay * dq[i_k] + 0.5 * delay ** 2 * a_prev
    dq0 = dq[i_k] + delay * a_prev
    q_hat, dq_hat = q0 + DT * dq0 + 0.5 * DT ** 2 * U, dq0 + DT * U
    qq, dd, plan = q0.copy(), dq0.copy(), np.zeros((K, N, 3))
    for j in range(N):
        uj = U[np.minimum(np.arange(K) + j, K - 1)] + (plan_noise * rng.normal(size=(K, 7)) if j else 0.0)
        qq, dd = qq + DT * dd + 0.5 * DT ** 2 * uj, dd + DT * uj
        plan[:, j] = u.fr3_tcp_positions(qq)

    keep = np.ones(K, bool)
    if drop_cycle is not None:
        keep[drop_cycle] = False
    human = (t_k - T0 > 0.2 * seconds) & (t_k - T0 < 0.4 * seconds)
    binding = human & (np.arange(K) % 2 == 0)
    slack = np.where(binding & (np.arange(K) % 40 == 0), 0.01, 0.0)
    diag = {"timestamp": t_k, "recv_time": t_k + 0.0005, "status": "solved", "solved": 1, "fallback": 0,
            "iterations": 40, "solve_time_ms": np.where(np.arange(K) % 100 == 7, 2.0, 0.3), "cost": -5.0,
            "human_rows": np.where(human, 4, 0), "human_binding": binding.astype(int),
            "human_clearance": np.where(human, 0.0, np.nan), "max_slack": slack, "accel_saturation": 0.0}
    if costs:
        # ℓ falls linearly and V is the sum of the next N ℓ plus T: η = 1 and α = (ℓ_k − ℓ_{k+N}) / ℓ_k > 0,
        # except one cycle where V jumps at a phase switch
        ell = 2.0 - np.arange(K + N) / (K + N)
        V = 0.1 + np.array([ell[k:k + N].sum() for k in range(K)])
        V[K // 2] += 5.0
        diag.update(input_delay=delay, cost_full=V, stage_cost=ell[:K], terminal_cost=0.1,
                    lambda_human=np.where(slack > 0, RHO + SIGMA * slack, np.where(binding, 0.5 * RHO, 0.0)),
                    lambda_accel=np.where(np.arange(K) % 4 == 0, 2.0, 0.0), lambda_position=0.0, lambda_velocity=0.0)
    diag = pd.DataFrame(diag)[keep]
    q_des = joint_frame(t_k, np.column_stack([np.column_stack([q_hat[:, j], dq_hat[:, j], U[:, j]]) for j in range(7)]),
                        [f"{c}_{j}" for j in range(1, 8) for c in ("q_d", "dq_d", "qdd")], recv=t_k + latency)[keep]
    cols = {"timestamp": t_k, "recv_time": t_k + latency, "step_s": DT}
    for j in range(N):
        for a_i, axis in enumerate("xyz"):
            cols[f"{axis}{j}"] = plan[:, j, a_i]
    plan_df = pd.DataFrame(cols)[keep]
    nom = joint_frame(tau, U, [f"qddot_nom_{j}" for j in range(1, 8)])
    events = pd.DataFrame({"timestamp": [T0, t_k[K // 2]], "recv_time": [T0, t_k[K // 2]],
                           "event": ["phase", "phase"], "value": ["MOVE_TO_PICK", "WAIT_AT_PICK"]})
    out = {"mpc_diagnostics": diag, "q_des": q_des, "mpc_plan": plan_df, "qddot_nom": nom, "task_events": events}
    if shield_joint is not None:
        out["qddot_safe"] = joint_frame(cmd_t, cmd_u, [f"qddot_safe_{j}" for j in range(1, 8)])
    if task_hz is not None:                                    # the task reference moves at task_hz
        t_task = T0 + 0.01 * np.arange(int(seconds / 0.01))
        out["task"] = pd.DataFrame({"timestamp": t_task, "recv_time": t_task, "ref_x": 0.3, "ref_y": 0.0,
                                    "ref_z": 0.4 + 0.005 * np.sin(2 * np.pi * task_hz * (t_task - T0)),
                                    "s": 0.0, "s_dot": 0.0, "ee_err": 0.0})
    for key, df in out.items():
        if files is None or key in files:
            write(df, run / f"experiment_{key}.csv")
    if files is not None and "robot_state" not in files:
        (run / "experiment_robot_state.csv").unlink()
    return run


def analyse(root: Path, run: str, *extra):
    code, out = quiet(mpc_analysis.main, [run, "--log-dir", str(root / "logs"), "--out-dir", str(root / "graphs"),
                                          *extra])
    out_dir = root / "graphs" / run / "mpc_analysis"
    read = lambda f: pd.read_csv(out_dir / f) if (out_dir / f).is_file() else None
    return code, out, out_dir, {f: read(f"summary_{f}.csv") for f in ("residuals", "acceleration", "oscillation", "mpc")}


# ── Checks ───────────────────────────────────────────────────────────────────

CHECKS = []


def check(fn):
    CHECKS.append(fn)
    return fn


def quiet(fn, *a, **kw):
    """Run fn with its prints captured; returns (result, output)."""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
        try:
            result = fn(*a, **kw)
        except SystemExit as exc:
            result = f"SystemExit({exc.code})"
    return result, buf.getvalue()


@check
def tcp_kinematics_match_pinocchio_on_the_urdf():
    # fr3_hand_tcp in fr3_link0, by Pinocchio on the URDF the MPC loads
    refs = {(0, 0, 0, 0, 0, 0, 0): [0.088, 0.0, 0.8226],
            tuple(Q_HOME): [0.308490202, 0.0, 0.487490708],
            (0.5, -0.3, 0.4, -1.8, -0.6, 2.1, -0.4): [0.385719, 0.40759394, 0.66295429]}
    q = np.array(list(refs))
    np.testing.assert_allclose(u.fr3_tcp_positions(q), np.array(list(refs.values())), atol=2e-6)


@check
def zoh_holds_each_sample_until_the_next_and_not_too_long():
    t = np.array([0.0, 1.0, 2.0])
    y = np.array([[1.0], [2.0], [3.0]])
    out = u.zoh(t, y, np.array([-0.1, 0.0, 0.5, 1.99, 2.4, 3.5]), max_age=1.0)[:, 0]
    np.testing.assert_allclose(out[1:5], [1.0, 1.0, 2.0, 3.0])
    assert np.isnan(out[0]) and np.isnan(out[5])


@check
def evaluation_grid_gives_the_same_derivative_as_every_sample():
    t = np.arange(2000) * H
    y = np.sin(7 * t)
    full = u.local_poly_derivative(t, y, 0.05)
    t_eval = t[100:1900:7]
    np.testing.assert_allclose(u.local_poly_derivative(t, y, 0.05, t_eval=t_eval), full[100:1900:7], atol=1e-12)


@check
def cross_correlation_test_tells_a_residual_that_depends_on_u():
    rng = np.random.default_rng(1)
    n = 3000
    uu = np.zeros(n)
    for k in range(1, n):
        uu[k] = 0.9 * uu[k - 1] + rng.normal()
    independent = rng.normal(size=n)
    dependent = 0.5 * uu + 0.3 * u.lagged(uu, 1) + rng.normal(size=n)
    independent[::50] = np.nan                                # missing cycles
    _, _, _, p_ind = u.cross_correlation_test(independent, uu, 10)
    r, _, _, p_dep = u.cross_correlation_test(dependent, uu, 10)
    assert p_ind > 0.05 and p_dep < 1e-6, (p_ind, p_dep)
    assert r[0] > 0.3                                         # after prewhitening, at the lag that matters


@check
def oscillation_index_finds_a_regular_oscillation_and_not_noise():
    rng = np.random.default_rng(2)
    t = np.arange(6000) * 0.01
    slow = 3.0 * np.sin(2 * np.pi * 0.1 * t)                  # the task's motion: removed by the high-pass
    period, r, _ = u.oscillation_index(slow + np.sin(2 * np.pi * 2.5 * t) + 0.3 * rng.normal(size=t.size), 0.01)
    assert abs(period - 0.4) < 0.01 and r > 1.0, (period, r)
    _, r_noise, _ = u.oscillation_index(slow + rng.normal(size=t.size), 0.01)
    assert not r_noise > 1.0, r_noise


@check
def ideal_plant_gives_white_residuals_independent_of_u():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        make_mpc_run(root / "logs", "20261024_100000", disturbance=0.2)
        code, out, out_dir, s = analyse(root, "mpc/20261024_100000")
        assert code == 0, out
        res = s["residuals"]
        joints = res[res.signal.str.startswith("joint")]
        assert joints.white.sum() >= 6 and joints.uncorrelated_with_u.sum() >= 6, joints[["p_white", "p_xcorr"]]
        assert (joints.rms_command < 1e-12).all() and (joints.rms_state < 1e-12).all()
        m = s["mpc"].iloc[0]
        assert m.fk_check_max_m < 1e-9
        acc = s["acceleration"]
        assert (acc.delay_ms == 0).all() and np.allclose(acc.gain, 1.0, atol=0.02), acc[["delay_ms", "gain"]]
        assert len(list(out_dir.glob("mpc_residual_joint_*.png"))) == 7


@check
def late_command_and_slow_plant_show_in_the_residuals_and_the_delay():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        make_mpc_run(root / "logs", "20261024_110000", latency=0.015, plant_delay=0.010, gain=0.9, costs=False)
        code, out, _, s = analyse(root, "mpc/20261024_110000")
        assert code == 0, out
        joints = s["residuals"][s["residuals"].signal.str.startswith("joint")]
        assert not joints.uncorrelated_with_u.any(), joints.p_xcorr
        assert (joints.rms_command > 0.3 * joints.rms).all() and (joints.rms_plant > 0.1 * joints.rms).all()
        acc = s["acceleration"]
        # The command log is stamped when u_k was sent: only the plant's own 10 ms remain
        assert (acc.delay_ms == 10).all() and np.allclose(acc.gain, 0.9, atol=0.02), acc
        assert "no cost and multiplier columns" in out


@check
def compensated_latency_leaves_residuals_independent_of_u():
    """The late command of the check above, with the MPC predicting its state to when u_k arrives."""
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        make_mpc_run(root / "logs", "20261024_115000", latency=0.015, disturbance=0.2, compensate=True)
        code, out, _, s = analyse(root, "mpc/20261024_115000")
        assert code == 0 and "input delay (15 ms)" in out and "from lag 2" in out, out
        joints = s["residuals"][s["residuals"].signal.str.startswith("joint")]
        # The disturbance during the delay is in two residuals (MA(1)): white from lag 2, never tied to u
        assert (joints.rho1 > 0.3).all() and joints.white.sum() >= 5, joints[["rho1", "p_white"]]
        assert joints.uncorrelated_with_u.all(), joints.p_xcorr
        assert (joints.rms_command < 1e-9).all() and (joints.rms_state < 1e-9).all()
        assert s["mpc"].iloc[0].fk_check_max_m < 1e-9


@check
def oscillations_in_u_and_in_the_shield_are_found():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        # u oscillates at the task's own 3 Hz, the shield's correction at 2 Hz on its own
        make_mpc_run(root / "logs", "20261024_120000", controller="mpc_cbf", oscillate_joint=3, shield_joint=1,
                     task_hz=3.0)
        code, out, _, s = analyse(root, "mpc_cbf/20261024_120000")
        assert code == 0, out
        o = s["oscillation"].set_index(["signal", "joint"])
        u4 = o.loc[("u (MPC q̈)", 4)]
        assert abs(u4.period_s - 1 / 3) < 0.02 and u4.clear and u4.in_task_reference, u4
        sh2 = o.loc[("shield correction", 2)]
        assert abs(sh2.period_s - 0.5) < 0.02 and sh2.clear and not sh2.in_task_reference, sh2
        assert not (o.loc["shield correction"].drop(index=2).regularity > 1).any()
        # The rest is coloured noise: at most weakly regular, never clear; q̇ of joint 2 carries the shield's 2 Hz
        own = o[o.clear & ~o.in_task_reference].drop(index="task reference p̈", errors="ignore")
        assert set(own.index) == {("shield correction", 2), ("measured q̇", 2)}, own
        acc = s["acceleration"]
        assert (acc.command == "qddot_safe").all() and (acc.delay_ms == 0).all()
        assert np.allclose(acc.gain, 1.0, atol=0.02), acc
        # The shield's correction is the command part of the residual, not the plant's
        j2 = s["residuals"].set_index("signal").loc["joint 2"]
        assert j2.rms_command > 5 * j2.rms_plant, j2


@check
def cost_ratio_multipliers_decrease_and_timing_are_what_was_built():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        make_mpc_run(root / "logs", "20261024_130000", drop_cycle=300, task_hz=0.0)   # reference fixed
        code, out, out_dir, s = analyse(root, "mpc/20261024_130000")
        assert code == 0, out
        m = s["mpc"].iloc[0]
        assert abs(m.eta_median - 1.0) < 1e-9 and m.eta_in_band > 0.95
        # α = (ℓ_k − ℓ_{k+N}) / ℓ_k = (N / (K + N)) / ℓ_k > 0, except where V jumped once (fixed reference)
        K = 1200
        assert m.alpha_fixed_up == 1 and m.alpha_human_up == 0 and m.alpha_moving_cycles == 0
        assert m.alpha_human_cycles > 200 and m.alpha_fixed_cycles > 800
        assert N / (K + N) / 2.0 - 1e-9 <= m.alpha_human_min <= N / (K + N) / 1.0 + 1e-9
        assert abs(m.lambda_accel_binding_share - 0.25) < 0.01
        assert m.kkt_slack_consistent_share == 1.0 and abs(m.lambda_human_over_rho_max_held - 0.5) < 1e-9
        assert m.missed_cycles == 1 and m.solve_ms_p50 == 0.3 and m.solve_ms_max == 2.0
        for plot in ("mpc_cost_ratio", "mpc_multipliers", "mpc_cost_decrease", "mpc_realtime", "mpc_plan_churn"):
            assert (out_dir / f"{plot}.png").is_file(), plot


@check
def replanning_shows_as_churn():
    """The plan is what then got commanded, so without replanning the churn is the disturbance's
    one-step effect only; replanning the inputs after the first adds to it."""
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        churn = {}
        for name, noise in (("20261024_140000", 0.0), ("20261024_140001", 1.0)):
            make_mpc_run(root / "logs", name, plan_noise=noise, costs=False)
            code, out, _, s = analyse(root, f"mpc/{name}")
            assert code == 0, out
            churn[noise] = s["mpc"].iloc[0]
        assert churn[1.0].churn_head_median_mm > 5 * churn[0.0].churn_head_median_mm
        assert churn[1.0].horizon_rms_last_mm > churn[1.0].horizon_rms_first_mm


@check
def partial_old_and_broken_runs_fail_gracefully():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        logs = root / "logs"
        # No robot_state: no residuals nor executed q̈, the rest still runs
        make_mpc_run(logs, "20261024_150000", files=("mpc_diagnostics", "q_des", "mpc_plan", "qddot_nom"))
        code, out, _, s = analyse(root, "mpc/20261024_150000")
        assert code == 0 and "prediction residuals skipped" in out and s["residuals"] is None, out
        # Empty plan file (the first runs of 2026-10-21)
        run = make_mpc_run(logs, "20261024_160000", costs=False)
        (run / "experiment_mpc_plan.csv").write_text("")
        code, out, _, _ = analyse(root, "mpc/20261024_160000")
        assert code == 0 and "plan consistency skipped" in out, out
        # Too short to analyse, and missing runs or folders
        make_mpc_run(logs, "20261024_170000", seconds=1.0)
        code, out, _, _ = analyse(root, "mpc/20261024_170000")
        assert code == "SystemExit(1)" and "fewer than" in out, out
        code, out = quiet(mpc_analysis.main, ["19990101_000000", "--log-dir", str(logs)])
        assert code == "SystemExit(1)" and "not found" in out, out
        code, out = quiet(mpc_analysis.main, ["--log-dir", str(root / "nowhere")])
        assert code == "SystemExit(1)" and "does not exist" in out, out
        # Without a name: the latest run of an MPC controller, not of another one
        (logs / "cbf" / "20991231_235959").mkdir(parents=True)
        code, out = quiet(mpc_analysis.main, ["--log-dir", str(logs), "--out-dir", str(root / "graphs")])
        assert "mpc/20261024_170000" in out, out


def main():
    failed = 0
    for fn in CHECKS:
        try:
            fn()
            print(f"ok    {fn.__name__}")
        except Exception:
            failed += 1
            print(f"FAIL  {fn.__name__}")
            traceback.print_exc()
    print(f"\n{len(CHECKS) - failed} passed, {failed} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())