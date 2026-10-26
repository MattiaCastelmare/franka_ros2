#!/usr/bin/env python3
"""A-posteriori diagnostics of the MPC (mpc_commander) on one run's logs.

The MPC predicts with a model (the double integrator q̈ = u) and plans a horizon it then
executes only one step of. As with the Kalman filter's innovations, what it predicted is
held against what happened:

1. Prediction residuals, the MPC's innovations. q_des holds the plan's next state, so
   e = x(t0 + dt) − x̂(t0 + dt | t_k) is the model's one-step error, per joint and at the
   TCP; t0 = t_k + input_delay is where the plan starts (logged per solve; 0 in older logs,
   whose MPC planned from the measured state as if u_k acted at once). With a delay the
   predictions overlap by it, so an exact model still leaves an MA(1) residual: whiteness is
   then tested from lag 2. If the model holds, e is zero-mean, white (ACF + Box-Pierce, as whiteness_test.py)
   and uncorrelated with the inputs that produced it: Ljung's cross-correlation test, after
   prewhitening u with an AR model of it (closed-loop inputs are far from white). The error
   is split in three parts that add up to it, each computed from the logs:
     state    the MPC's prediction to t0 + dt redone from the logged state and q̈ in force,
              against its own (state read, delay, q̈ in force it assumed);
     command  what the q̈ in force over [t_k, t0 + dt] (the shield's output, or u_k only once
              it arrives) moves the joint, against the model's (q̈ in force until t0, u_k after);
     plant    the rest: the torque loop not carrying out the q̈ it was sent.
   Along the horizon, e_j = p_TCP(t_k + j·dt) − p̂_j|k mixes model error and replanning.
2. Executed acceleration, the Savitzky-Golay check. q̈ from the measured q̇ (Savitzky-Golay
   derivative) against the q̈ sent to the torque controller (qddot_safe behind a shield,
   else qddot_nom), passed through the same filter, with the delay and gain that best align
   them. A negative delay means the logged command stamps lag what the controller got.
3. Plan consistency. plan_{k+1}[j] against plan_k[j+1]: how much each solve changes the path
   the previous one planned (churn), with and without human rows. Oscillations: the index of
   Thornhill, Huang & Zhang (2003) (regularity of the ACF zero crossings) on u, on the
   measured q̇ and on the shield's correction, with their spectra. Clear: r ≥ 2 at ≥ 1.25 Hz
   (high-passed coloured noise alone reaches r > 1 below that); one with the period of a
   regular oscillation of the task reference's own acceleration is the task's, not the loop's.
4. Achieved vs design cost (Patwardhan & Shah 2002; Schäfer & Cinar 2004). What the MPC
   expected to pay over its horizon, V_k minus the terminal term, against what the closed
   loop paid over the same N cycles, Σ ℓ_{k+j}: η = (V_k − T_k) / Σ ℓ_{k+j} ≈ 1 when the
   plans come true; η < 1 is an optimistic MPC (model error, a person it did not foresee).
5. Multipliers: how often each row family binds and how hard (cost per unit of bound), and
   λ_human against ρ: the slack penalty is exact only below it, and a row on its slack
   must have λ = ρ + σ·s (KKT).
6. Real time: solve time and loop latency against dt, statuses, fallbacks, missed cycles.
7. Decrease of V (Grüne & Pannek, a-posteriori suboptimality estimate):
   α_k = (V_k − V_{k+1}) / ℓ_k. α_k ≥ α > 0 on every cycle bounds the closed-loop cost by
   V/α for a regulation MPC. It is checked where the problem is one: cycles whose task
   reference stays fixed over the whole horizon, with no human rows; there α < 0 (V went
   up) is the model or the shield. With a moving reference, or a person, V may go up by
   design, and those cycles are reported apart. One QP per cycle: a diagnostic, not a proof.
Points 4, 5 and 7 need the cost and multiplier columns of MpcDiagnostics (from 2026-10-24).

The MPC's TCP is recomputed from q with analysis_utils.fr3_tcp_positions, checked on every
run against the plan's first point (the TCP of q_des, computed by the MPC with Pinocchio).
Plan point j (0-based) is the state at t0 + (j + 1)·dt.

Outputs, in <out-dir>/<controller>/<run>/mpc_analysis/:
  mpc_residual_joint_<j>.png      one-step residual: time | distribution | ACF | xcorr with u
  mpc_residual_tcp_<axis>.png     the same at the TCP (no xcorr)
  mpc_residual_horizon.png        rms and p95 of ‖e_j‖ along the horizon
  mpc_acceleration_joint_<j>.png  commanded vs executed q̈
  mpc_plan_churn.png, mpc_oscillation_psd.png, mpc_realtime.png
  mpc_cost_ratio.png, mpc_multipliers.png, mpc_cost_decrease.png   (points 4, 5, 7)
  summary_residuals.csv, summary_acceleration.csv, summary_oscillation.csv, summary_mpc.csv

Usage:
  python3 mpc_analysis.py [RUN] [--max-lag 20] [--window-s 0.05] [--rho 1000] [--sigma 1000]
                          [--log-dir DIR] [--out-dir DIR]
RUN: a run folder name or <controller>/<run>; the latest mpc or mpc_cbf run if omitted.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")   # files only: runs without a display
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from numpy.lib.stride_tricks import sliding_window_view
from scipy.signal import savgol_filter, welch
from scipy.stats import chi2, norm

from analysis_utils import (
    AXES, GRAPH_DIR, LOG_DIR, acf_gapped, cross_correlation_test, effective_size, fail,
    fr3_tcp_positions, high_pass, interp_within, local_poly_derivative, match, on_grid,
    oscillation_index, portmanteau, read_log, resolve_run, warn, zoh,
)

JOINTS = 7
CONTROLLERS = ("mpc", "mpc_cbf")
MIN_SOLVES = 50
STATE_MAX_GAP_S = 0.01      # robot_state is ~1 kHz: interpolate across a few dropped samples at most
SUBSTEPS = 20               # samples of the command within one MPC step (residual split)
ACCEL_GRID_S = 0.005        # executed q̈ evaluated every 5 ms
COMMAND_GRID_S = 0.001      # the commanded q̈ held and filtered on a 1 kHz grid
# Candidate delays of the executed q̈; below 0: the logged command stamps lag the controller
DELAYS_S = np.round(np.arange(-0.050, 0.200 + 1e-9, ACCEL_GRID_S), 3)
SIGNAL_GRID_S = 0.01        # q̇ and the shield's correction at 100 Hz for the oscillation index
DETREND_S = 1.0             # high-pass before the oscillation index: the task's own motion is slower
TASK_PERIOD_TOL = 0.2       # an oscillation within 20% of a period of the task reference is the task's
# A clear oscillation: regularity r ≥ OSC_CLEAR_R at a frequency ≥ OSC_MIN_HZ. Thornhill's r > 1 is
# met by coloured noise too once high-passed (it becomes a band around the filter's corner,
# 0.6–1.5 Hz here; r up to ~1.6 for u and ~4 for q̇ in the self-test and on Gazebo runs)
OSC_CLEAR_R, OSC_MIN_HZ = 2.0, 1.25
LAMBDA_TOL = 1e-3           # a multiplier above this: its family binds
ETA_BAND = (0.8, 1.25)
REF_FIXED_M = 1e-4          # [m] the task reference counts as fixed below this displacement
FK_TOL_M = 1e-6
COLORS = plt.get_cmap("tab10").colors


# ── Loading ──────────────────────────────────────────────────────────────────

def joint_columns(df: pd.DataFrame, pattern: str) -> np.ndarray:
    return df[[pattern.format(i) for i in range(1, JOINTS + 1)]].to_numpy(dtype=float)


def load_run(run: Path) -> dict:
    """The run's MPC logs as arrays; the optional ones are None when not logged."""
    diag = read_log(run, "mpc_diagnostics", ("status",))
    if diag is None or len(diag) < MIN_SOLVES:
        fail(f"fewer than {MIN_SOLVES} MPC solves logged in '{run}' (not an MPC run, or a very short one).")
    d = dict(diag=diag, t=diag["timestamp"].to_numpy(dtype=float), plan=None, q_des=None,
             state=None, nom=None, safe=None, task=None, phases=[])
    plan = read_log(run, "mpc_plan")
    if plan is not None and "x0" in plan.columns:
        n_points = sum(1 for c in plan.columns if c[:1] == "x" and c[1:].isdigit())
        d["plan"] = dict(t=plan["timestamp"].to_numpy(dtype=float), p=np.stack(
            [plan[[f"x{j}", f"y{j}", f"z{j}"]].to_numpy(dtype=float) for j in range(n_points)], axis=1))
        d["dt"] = float(np.nanmedian(plan["step_s"]))
    else:
        d["dt"] = float(np.median(np.diff(d["t"])))
    q_des = read_log(run, "q_des")
    if q_des is not None:
        d["q_des"] = dict(t=q_des["timestamp"].to_numpy(dtype=float), recv=q_des["recv_time"].to_numpy(dtype=float),
                          q=joint_columns(q_des, "q_d_{}"), dq=joint_columns(q_des, "dq_d_{}"),
                          u=joint_columns(q_des, "qdd_{}"))
    state = read_log(run, "robot_state")
    if state is not None:
        d["state"] = dict(t=state["timestamp"].to_numpy(dtype=float), q=joint_columns(state, "q_{}"),
                          dq=joint_columns(state, "dq_{}"))
    for key, name in (("nom", "qddot_nom"), ("safe", "qddot_safe")):
        df = read_log(run, name)
        if df is not None and len(df) > 1:
            d[key] = dict(t=df["timestamp"].to_numpy(dtype=float), u=joint_columns(df, name + "_{}"))
    task = read_log(run, "task")
    if task is not None and len(task) > 1:
        d["task"] = dict(t=task["timestamp"].to_numpy(dtype=float), p=task[["ref_x", "ref_y", "ref_z"]].to_numpy(dtype=float))
    events = read_log(run, "task_events", ("event", "value"))
    if events is not None:
        phases = events[events["event"] == "phase"]
        d["phases"] = list(zip(phases["timestamp"].to_numpy(dtype=float), phases["value"]))
    return d


def diag_at(d: dict, t: np.ndarray, column: str) -> np.ndarray:
    """A diagnostics column at the solves stamped t (NaN where no solve matches)."""
    k = match(d["t"], t, 0.25 * d["dt"])
    values = d["diag"][column].to_numpy(dtype=float) if column in d["diag"] else np.full(len(d["t"]), np.nan)
    return np.where(k >= 0, values[np.maximum(k, 0)], np.nan)


def shade(ax, t, mask, color="orange", label=None):
    """Shade the x ranges where mask holds (e.g. human rows in use)."""
    if np.any(mask):
        ax.fill_between(t, 0, 1, where=mask, transform=ax.get_xaxis_transform(), color=color,
                        alpha=0.15, linewidth=0, label=label)


def phase_lines(ax, d, t0):
    for t, _ in d["phases"]:
        ax.axvline(t - t0, color="gray", linewidth=0.6, alpha=0.4)


def save(fig, path):
    fig.tight_layout()
    fig.savefig(path, dpi=300, bbox_inches="tight")
    plt.close(fig)


def p_text(p, short=False):
    if not np.isfinite(p):
        return "n/a"
    if p < 0.001:
        return "<.001" if short else "< 0.001"
    return f"{p:.3f}" if short else f"= {p:.3f}"


# ── 1. Prediction residuals ─────────────────────────────────────────────────

def residual_stats(t, y, period, max_lag, first_lag=1):
    """Mean, spread, bias in units of its standard error and whiteness of one residual series.

    first_lag 2: with an input delay τ consecutive predictions (τ + dt long) overlap by τ, so
    even an exact model leaves the disturbance of the overlap in two residuals (MA(1)). The
    Box-Pierce test then starts at lag 2, each lag's variance (1 + 2ρ₁²)/n (Bartlett) instead
    of 1/n: Q is divided by 1 + 2ρ₁².
    """
    ok = np.isfinite(y)
    t, y = t[ok], y[ok]
    rho, pairs = acf_gapped(t, y, max_lag, period)
    q, p, dof = portmanteau(rho[first_lag - 1:], pairs[first_lag - 1:])
    if first_lag == 2 and np.isfinite(q):
        q /= 1.0 + 2.0 * rho[0] ** 2
        p = float(chi2.sf(q, dof))
    n_eff = effective_size(len(y), rho)
    std = float(np.std(y, ddof=1))
    return dict(n=int(len(y)), mean=float(np.mean(y)), std=std, rms=float(np.sqrt(np.mean(y ** 2))),
                n_eff=n_eff, bias_z=float(np.mean(y) / (std / np.sqrt(n_eff))) if std > 0 else np.nan,
                rho1=rho[0], q=q, dof=dof, first_lag=first_lag, p_white=p,
                white=bool(np.isfinite(p) and p >= 0.05)), rho, pairs


def command_in_force(d, t):
    """The q̈ the torque controller had last received at the times t (the shield's output behind
    a CBF, else the MPC's), NaN where none; None without a command log."""
    cmd = d["safe"] or d["nom"]
    return None if cmd is None else zoh(cmd["t"], cmd["u"], t, 3.0 * d["dt"])


def command_effect(d, t, u, delay, a_prev):
    """Δq the q̈ in force over [t_k, t0 + dt] adds to the MPC's prediction, which held a_prev
    until t0 = t_k + delay (when u_k reaches the controller) and u_k after:
    ∫ (delay + dt − s)(a_cmd(t_k + s) − a_model(s)) ds, sampled SUBSTEPS times."""
    if command_in_force(d, t[:1]) is None:
        return np.full_like(u, np.nan)
    span = delay + d["dt"]                      # (n,): from t_k to the predicted state
    dq = np.zeros_like(u)
    for i in range(SUBSTEPS):
        s = (i + 0.5) * span / SUBSTEPS
        a_model = np.where((s < delay)[:, None], a_prev, u)
        dq += ((span - s) * span / SUBSTEPS)[:, None] * (command_in_force(d, t + s) - a_model)
    return dq


def analyse_residuals(d, args, out_dir, t0):
    """Point 1. Returns (rows of summary_residuals.csv, scalar metrics)."""
    qd, st, dt = d["q_des"], d["state"], d["dt"]
    if qd is None or st is None:
        warn("no q_des or robot_state log: prediction residuals skipped.")
        return [], {}
    t, u = qd["t"], qd["u"]
    delay = np.nan_to_num(diag_at(d, t, "input_delay"))   # the plan starts at t0 = t + delay (0: older logs)
    q_meas = interp_within(st["t"], st["q"], t + delay + dt, STATE_MAX_GAP_S)
    e = q_meas - qd["q"]
    # The three parts. state: the MPC's prediction redone from the logged state at t and the
    # logged q̈ in force (until t0), against its own; command: the q̈ in force against the
    # model's; plant: the rest, the torque loop
    q0 = interp_within(st["t"], st["q"], t, STATE_MAX_GAP_S)
    dq0 = interp_within(st["t"], st["dq"], t, STATE_MAX_GAP_S)
    a_prev = command_in_force(d, t)
    a_prev = np.zeros_like(u) if a_prev is None else np.nan_to_num(a_prev)
    tau = delay[:, None]
    q_t0, dq_t0 = q0 + tau * dq0 + 0.5 * tau ** 2 * a_prev, dq0 + tau * a_prev
    e_state = (q_t0 + dt * dq_t0 + 0.5 * dt ** 2 * u) - qd["q"]
    e_command = command_effect(d, t, u, delay, a_prev)
    e_plant = e - e_state - e_command
    first_lag = 2 if np.median(delay) > 0 else 1     # see residual_stats
    _, E = on_grid(t, e, dt)
    k, U = on_grid(t, u, dt)
    _, P = on_grid(t, e_plant, dt)

    cmd_name = "qddot_safe" if d["safe"] else "qddot_nom"
    print(f"\n1. Prediction residuals: e = q(t0 + dt) − q̂(t0 + dt | t), t0 = t + input delay "
          f"({1e3 * np.median(delay):.0f} ms), {int(np.isfinite(e).all(axis=1).sum())} solves; parts: state, "
          f"command ({cmd_name} in force), plant (torque loop)")
    if first_lag == 2:
        print("  (with an input delay the predictions overlap: whiteness tested from lag 2)")
    print(f"  {'joint':5s} {'rms':>6s} {'bias z':>6s} {'ACF1':>5s} {'white? p':>8s} {'xcorr u p':>9s} | "
          f"{'state':>6s} {'command':>7s} {'plant':>6s} [mrad]  {'plant xcorr u p':>15s}")
    rows = []
    for j in range(JOINTS):
        s, rho, pairs = residual_stats(t, e[:, j], dt, args.max_lag, first_lag)
        if s["n"] < MIN_SOLVES:
            continue
        r, xpairs, S, p_x = cross_correlation_test(E[:, j], U[:, j], args.max_lag)
        _, _, _, p_plant = cross_correlation_test(P[:, j], U[:, j], args.max_lag)
        rms = lambda x: float(np.sqrt(np.nanmean(x ** 2)))
        s.update(signal=f"joint {j + 1}", unit="rad", xcorr_s=S, p_xcorr=p_x,
                 uncorrelated_with_u=bool(np.isfinite(p_x) and p_x >= 0.05),
                 rms_state=rms(e_state[:, j]), rms_command=rms(e_command[:, j]),
                 rms_plant=rms(e_plant[:, j]), p_xcorr_plant=p_plant)
        rows.append(s)
        print(f"  {j + 1:<5d} {1e3 * s['rms']:6.3f} {s['bias_z']:+6.1f} {s['rho1']:5.2f} "
              f"{p_text(s['p_white'], True):>8s} {p_text(p_x, True):>9s} | {1e3 * s['rms_state']:6.3f} "
              f"{1e3 * s['rms_command']:7.3f} {1e3 * s['rms_plant']:6.3f}         {p_text(p_plant, True):>15s}")
        plot_residual(f"Joint {j + 1}", t - t0, e[:, j], 1e3, "mrad", rho, pairs, s, r, xpairs,
                      out_dir / f"mpc_residual_joint_{j + 1}.png")

    metrics = {}
    plan = d["plan"]
    if plan is not None:
        kp = match(plan["t"], t, 0.25 * dt)
        p_hat = np.where((kp >= 0)[:, None], plan["p"][np.maximum(kp, 0), 0], np.nan)
        fk_err = np.linalg.norm(fr3_tcp_positions(qd["q"]) - p_hat, axis=1)
        metrics["fk_check_max_m"] = float(np.nanmax(fk_err)) if np.isfinite(fk_err).any() else np.nan
        if not metrics["fk_check_max_m"] <= FK_TOL_M:
            warn(f"the TCP of q_des differs from the plan's first point by up to {metrics['fk_check_max_m']:.2e} m: "
                 "is the MPC's ee_frame fr3_hand_tcp? TCP residuals are off by that much.")
        e_p = fr3_tcp_positions(q_meas) - p_hat
        line = []
        for a, axis in enumerate(AXES):
            s, rho, pairs = residual_stats(t, e_p[:, a], dt, args.max_lag, first_lag)
            if s["n"] < MIN_SOLVES:
                continue
            s.update(signal=f"TCP {axis}", unit="m")
            rows.append(s)
            line.append(f"{axis} {1e3 * s['rms']:.2f} mm (bias z {s['bias_z']:+.1f}, ACF1 {s['rho1']:.2f}, "
                        f"white p {p_text(s['p_white'], True)})")
            plot_residual(f"TCP {axis.upper()}", t - t0, e_p[:, a], 1e3, "mm", rho, pairs, s, None, None,
                          out_dir / f"mpc_residual_tcp_{axis}.png")
        print(f"  TCP: {'; '.join(line)}   [FK check {metrics['fk_check_max_m']:.0e} m]")
        metrics.update(analyse_horizon(d, out_dir))
    return rows, metrics


def analyse_horizon(d, out_dir):
    """‖p_TCP(t_k + j·dt) − p̂_j|k‖ along the horizon, all solves / without / with human rows."""
    plan, st, dt = d["plan"], d["state"], d["dt"]
    n_pts = plan["p"].shape[1]
    delay = np.nan_to_num(diag_at(d, plan["t"], "input_delay"))
    err = np.full((len(plan["t"]), n_pts), np.nan)
    for j in range(n_pts):
        q = interp_within(st["t"], st["q"], plan["t"] + delay + (j + 1) * dt, STATE_MAX_GAP_S)
        err[:, j] = np.linalg.norm(fr3_tcp_positions(q) - plan["p"][:, j], axis=1)
    human = diag_at(d, plan["t"], "human_rows") > 0
    horizon_ms = 1e3 * dt * np.arange(1, n_pts + 1)
    fig, ax = plt.subplots(figsize=(9, 5))
    metrics, parts = {}, []
    for label, mask, color in (("all solves", np.ones(len(err), bool), "black"),
                               ("no human rows", ~human, "tab:blue"), ("human rows in use", human, "tab:orange")):
        e = err[mask]
        e = e[np.isfinite(e).all(axis=1)]
        if len(e) < MIN_SOLVES:
            continue
        rms, p95 = np.sqrt(np.mean(e ** 2, axis=0)), np.percentile(e, 95, axis=0)
        ax.plot(horizon_ms, 1e3 * rms, "o-", color=color, label=f"rms, {label} ({len(e)})")
        ax.plot(horizon_ms, 1e3 * p95, "--", color=color, alpha=0.7, label=f"p95, {label}")
        parts.append(f"{label} {1e3 * rms[0]:.2f} → {1e3 * rms[-1]:.1f} mm")
        if label == "all solves":
            metrics.update(horizon_rms_first_mm=1e3 * rms[0], horizon_rms_last_mm=1e3 * rms[-1],
                           horizon_p95_last_mm=1e3 * p95[-1])
    ax.set_title("MPC plan vs execution along the horizon\n(model error and replanning together)", fontsize=13)
    ax.set_xlabel("Prediction time ahead [ms]", fontsize=12)
    ax.set_ylabel(r"$\|p_{TCP}(t_k + j\,dt) - \hat p_{j|k}\|$ [mm]", fontsize=12)
    ax.legend(fontsize=9)
    ax.grid(True, linestyle="--", alpha=0.6)
    save(fig, out_dir / "mpc_residual_horizon.png")
    print(f"  Horizon, rms from {horizon_ms[0]:.0f} to {horizon_ms[-1]:.0f} ms ahead: {'; '.join(parts)}")
    return metrics


def plot_residual(name, t, y, scale, unit, rho, pairs, s, r, xpairs, path):
    panels = 4 if r is not None else 3
    fig, axes = plt.subplots(1, panels, figsize=(6 * panels, 5),
                             gridspec_kw={"width_ratios": [3, 1, 1.4, 1.4][:panels]})
    ok = np.isfinite(y)
    ys = scale * y[ok]
    ax = axes[0]
    ax.plot(t[ok], ys, color="purple", linewidth=0.8, alpha=0.8, label=f"Residual ({s['n']} solves)")
    ax.axhline(0, color="black", linewidth=1.5)
    half_ci = 1.96 * scale * s["std"] / np.sqrt(s["n_eff"])
    ax.axhline(scale * s["mean"], color="red", linestyle="--",
               label=f"Mean {scale * s['mean']:+.3f} {unit} (95% CI ±{half_ci:.3f}, n_eff = {s['n_eff']:.0f})")
    ax.set_title(f"MPC one-step residual: {name}", fontsize=14)
    ax.set_xlabel("Elapsed Time [s]", fontsize=12)
    ax.set_ylabel(f"x(t+dt) − x̂(t+dt | t) [{unit}]", fontsize=12)
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, linestyle="--", alpha=0.5)

    ax = axes[1]
    spread = 1.4826 * np.median(np.abs(ys - np.median(ys)))
    lim = 6 * (spread if spread > 0 else scale * s["std"])
    bins = np.linspace(np.median(ys) - lim, np.median(ys) + lim, 61)
    ax.hist(np.clip(ys, bins[0], bins[-1]), bins=bins, color="purple", alpha=0.7, density=True)
    xs = np.linspace(bins[0], bins[-1], 300)
    ax.plot(xs, norm.pdf(xs, scale * s["mean"], scale * s["std"]), color="black", linewidth=1.2,
            label="Gaussian, same mean/std")
    ax.set_title("Distribution", fontsize=12)
    ax.set_xlabel(f"Residual [{unit}]", fontsize=12)
    ax.legend(fontsize=8)
    ax.grid(True, linestyle="--", alpha=0.5)

    ax = axes[2]
    lags = np.arange(1, len(rho) + 1)
    band = np.where(pairs > 0, 1.96 / np.sqrt(np.maximum(pairs, 1)), np.nan)
    if s["first_lag"] == 2:     # MA(1) allowed: Bartlett's band from lag 2
        band[1:] *= np.sqrt(1.0 + 2.0 * rho[0] ** 2)
    ax.fill_between(lags, -band, band, color="gray", alpha=0.3,
                    label="95% band, white noise" if s["first_lag"] == 1 else "95% band, MA(1)")
    ax.vlines(lags, 0, rho, color="purple", linewidth=2)
    ax.plot(lags, rho, "o", color="purple", markersize=4)
    ax.axhline(0, color="black", linewidth=1)
    ax.set_title(f"Autocorrelation: {'white' if s['white'] else 'NOT white'}\n"
                 f"Box-Pierce Q = {s['q']:.0f} (lags {s['first_lag']}..{s['first_lag'] + s['dof'] - 1}), "
                 f"p {p_text(s['p_white'])}", fontsize=12)
    ax.set_xlabel("Lag [solves]", fontsize=12)
    ax.set_ylim(min(-0.3, np.nanmin(rho) - 0.05), 1.0)
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, linestyle="--", alpha=0.5)

    if r is not None:
        ax = axes[3]
        lags = np.arange(len(r))
        band = np.where(xpairs > 0, 1.96 / np.sqrt(np.maximum(xpairs, 1)), np.nan)
        ax.fill_between(lags, -band, band, color="gray", alpha=0.3, label="95% band, independent")
        ax.vlines(lags, 0, r, color="teal", linewidth=2)
        ax.plot(lags, r, "o", color="teal", markersize=4)
        ax.axhline(0, color="black", linewidth=1)
        verdict = "independent of u" if s["uncorrelated_with_u"] else "DEPENDS on u"
        ax.set_title(f"Cross-correlation with $u_{{k-\\tau}}$ (prewhitened): {verdict}\n"
                     f"S = {s['xcorr_s']:.0f} (L = {len(r)}), p {p_text(s['p_xcorr'])}", fontsize=12)
        ax.set_xlabel(r"Lag $\tau$ [solves]", fontsize=12)
        ax.set_ylim(min(-0.5, np.nanmin(r) - 0.05), max(0.5, np.nanmax(r) + 0.05))
        ax.legend(fontsize=8, loc="upper right")
        ax.grid(True, linestyle="--", alpha=0.5)
    save(fig, path)


# ── 2. Executed acceleration ────────────────────────────────────────────────

def filtered_command(cmd, t_f, window_s, max_age):
    """The commanded q̈ as the executed one is measured: held, integrated into a velocity and
    differentiated with the same Savitzky-Golay window, on the uniform grid t_f.

    Comparing the executed q̈ (smoothed by the derivative) with the raw steps of the command
    would bias the gain low and blur the delay: both now go through the same linear filter,
    so a delay applied to the command commutes with it. NaN within a window of a stale command.
    """
    h = t_f[1] - t_f[0]
    c = zoh(cmd["t"], cmd["u"], t_f, max_age)
    stale = ~np.isfinite(c).all(axis=1)
    v = np.cumsum(np.nan_to_num(c) * h, axis=0)
    width = 2 * int(round(0.5 * window_s / h)) + 1          # local_poly_derivative's window in samples
    out = savgol_filter(v, width, 2, deriv=1, delta=h, axis=0)
    out[np.convolve(stale, np.ones(width), mode="same") > 0] = np.nan
    return out


def align(t_g, a_meas, t_f, c_f):
    """For each candidate delay d, a_meas(t) against the filtered command at t − d: the best d
    overall and per joint, with rms, correlation and least-squares gain at it."""
    shifted = lambda d: interp_within(t_f, c_f, t_g - d, 2.0 * (t_f[1] - t_f[0]))
    rms_all, per_joint = [], []
    for d in DELAYS_S:
        e = a_meas - shifted(d)
        ok = np.isfinite(e).all(axis=1)
        rms_all.append(np.sqrt(np.mean(e[ok] ** 2)) if ok.sum() > MIN_SOLVES else np.inf)
        per_joint.append(np.sqrt(np.nanmean(e ** 2, axis=0)))
    best = int(np.argmin(rms_all))
    per_joint = np.array(per_joint)
    out = []
    for j in range(JOINTS):
        bj = int(np.nanargmin(per_joint[:, j])) if np.isfinite(per_joint[:, j]).any() else best
        c = shifted(DELAYS_S[bj])[:, j]
        ok = np.isfinite(c) & np.isfinite(a_meas[:, j])
        a, cj = a_meas[ok, j], c[ok]
        out.append(dict(delay_ms=1e3 * DELAYS_S[bj], rms=float(np.sqrt(np.mean((a - cj) ** 2))),
                        corr=float(np.corrcoef(a, cj)[0, 1]) if np.std(a) > 0 and np.std(cj) > 0 else np.nan,
                        gain=float(a @ cj / (cj @ cj)) if cj @ cj > 0 else np.nan))
    return 1e3 * DELAYS_S[best], float(rms_all[best]), out


def analyse_acceleration(d, args, out_dir, t0):
    """Point 2. Returns (rows of summary_acceleration.csv, scalar metrics)."""
    st = d["state"]
    cmd = d["safe"] or d["nom"]
    if st is None or cmd is None:
        warn("no robot_state or commanded q̈ log: executed acceleration skipped.")
        return [], {}
    t_g = np.arange(max(d["t"][0], st["t"][0]), min(d["t"][-1], st["t"][-1]), ACCEL_GRID_S)
    a_meas = local_poly_derivative(st["t"], st["dq"], args.window_s, t_eval=t_g)
    max_age = 3.0 * d["dt"]
    margin = DELAYS_S.max() + args.window_s
    t_f = np.arange(t_g[0] - margin, t_g[-1] + margin, COMMAND_GRID_S)
    cmd_name = "qddot_safe" if d["safe"] else "qddot_nom"
    c_f = filtered_command(cmd, t_f, args.window_s, max_age)
    delay, rms_all, stats = align(t_g, a_meas, t_f, c_f)
    nominal = None
    if d["safe"] and d["nom"]:
        nominal = align(t_g, a_meas, t_f, filtered_command(d["nom"], t_f, args.window_s, max_age))
    print(f"\n2. Executed acceleration: Savitzky-Golay q̈ from the measured q̇ (window {1e3 * args.window_s:.0f} ms) "
          f"vs {cmd_name} through the same filter; best delay over all joints {delay:.0f} ms (rms {rms_all:.3f} rad/s²)")
    if nominal:
        print(f"   vs the MPC's own qddot_nom: best delay {nominal[0]:.0f} ms (rms {nominal[1]:.3f} rad/s²)")
    print(f"  {'joint':5s} {'rms [rad/s²]':>12s} {'corr':>5s} {'delay':>7s} {'gain':>5s}"
          + ("   | vs qddot_nom: rms, corr, gain" if nominal else ""))
    rows = []
    for j, s in enumerate(stats):
        row = dict(joint=j + 1, command=cmd_name, **s)
        extra = ""
        if nominal:
            n = nominal[2][j]
            row.update(rms_vs_nom=n["rms"], corr_vs_nom=n["corr"], gain_vs_nom=n["gain"], delay_ms_vs_nom=n["delay_ms"])
            extra = f"   | {n['rms']:.3f}, {n['corr']:.2f}, {n['gain']:.2f}"
        rows.append(row)
        print(f"  {j + 1:<5d} {s['rms']:12.3f} {s['corr']:5.2f} {s['delay_ms']:4.0f} ms {s['gain']:5.2f}{extra}")
        plot_acceleration(j, t_g - t0, a_meas[:, j], cmd, (t_f - t0, c_f[:, j]), s, delay, max_age,
                          args.window_s, cmd_name, t0, out_dir / f"mpc_acceleration_joint_{j + 1}.png")
    return rows, dict(accel_delay_ms=delay, accel_rms=rms_all,
                      accel_corr_median=float(np.nanmedian([s["corr"] for s in stats])),
                      accel_gain_median=float(np.nanmedian([s["gain"] for s in stats])))


def plot_acceleration(j, t, a, cmd, filtered, s, delay_ms, max_age, window_s, cmd_name, t0, path):
    c = zoh(cmd["t"], cmd["u"], t + t0, max_age)[:, j]
    t_f, c_f = filtered
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), gridspec_kw={"height_ratios": [1, 1]})
    # Zoom: the 4 s with the largest command changes
    dc = np.nan_to_num(np.abs(np.diff(c, prepend=c[0])))
    n_win = max(int(4.0 / (t[1] - t[0])), 1) if len(t) > 1 else 1
    centre = int(np.argmax(np.convolve(dc, np.ones(n_win), mode="same"))) if len(dc) > n_win else len(t) // 2
    zoom = slice(max(centre - n_win // 2, 0), centre + n_win // 2)
    for ax, sl, title in ((ax1, slice(None), "whole run"), (ax2, zoom, "4 s zoom")):
        ax.step(t[sl], c[sl], where="post", color="blue", linewidth=1.5, alpha=0.5,
                label=f"Commanded ({cmd_name}, held)")
        if sl is zoom and len(t[sl]):
            in_zoom = (t_f >= t[sl][0]) & (t_f <= t[sl][-1])
            ax.plot(t_f[in_zoom], c_f[in_zoom], "--", color="blue", linewidth=1.0,
                    label="Commanded, through the same Savitzky-Golay filter")
        ax.plot(t[sl], a[sl], color="red", linewidth=1.0, label=f"Executed (Savitzky-Golay of q̇, {1e3 * window_s:.0f} ms)")
        ax.set_ylabel(f"$\\ddot q_{j + 1}$ [rad/s$^2$]", fontsize=12)
        ax.grid(True, linestyle="--", alpha=0.6)
        ax.set_title(title, fontsize=11)
    ax1.legend(fontsize=10, loc="upper right")
    ax2.legend(fontsize=9, loc="upper right")
    ax2.set_xlabel("Elapsed Time [s]", fontsize=12)
    fig.suptitle(f"Commanded vs executed acceleration: joint {j + 1}\nrms {s['rms']:.3f} rad/s², corr {s['corr']:.2f}, "
                 f"best-fit delay {s['delay_ms']:.0f} ms (all joints {delay_ms:.0f} ms), gain {s['gain']:.2f}", fontsize=13)
    save(fig, path)


# ── 3. Plan consistency and oscillations ────────────────────────────────────

def analyse_churn(d, out_dir, t0):
    plan, dt = d["plan"], d["dt"]
    if plan is None or len(plan["t"]) < MIN_SOLVES:
        warn("no MPC plan logged: plan consistency skipped.")
        return {}
    t, P = plan["t"], plan["p"]
    nxt = np.flatnonzero(np.abs(np.diff(t) - dt) < 0.25 * dt)     # consecutive solves
    churn = np.linalg.norm(P[nxt + 1, :-1] - P[nxt, 1:], axis=2)    # (pairs, N − 1)
    head, worst = churn[:, 0], churn.max(axis=1)
    human = diag_at(d, t[nxt + 1], "human_rows") > 0
    binding = diag_at(d, t[nxt + 1], "human_binding") > 0
    m = dict(churn_head_median_mm=1e3 * np.median(head), churn_head_p95_mm=1e3 * np.percentile(head, 95),
             churn_max_p95_mm=1e3 * np.percentile(worst, 95), churn_max_mm=1e3 * worst.max())
    parts = []
    for label, mask in (("no human rows", ~human), ("human rows", human)):
        if mask.sum() >= 10:
            m[f"churn_head_p95_{label.replace(' ', '_')}_mm"] = 1e3 * np.percentile(head[mask], 95)
            parts.append(f"{label} p95 {1e3 * np.percentile(head[mask], 95):.2f} mm ({mask.sum()} solves)")
    print(f"\n3. Plan consistency: ‖plan_k+1[j] − plan_k[j+1]‖ over {len(nxt)} consecutive solves")
    print(f"  first shared point (t_k + 2dt): median {m['churn_head_median_mm']:.2f} mm, p95 "
          f"{m['churn_head_p95_mm']:.2f} mm; whole path: p95 {m['churn_max_p95_mm']:.2f} mm, "
          f"max {m['churn_max_mm']:.1f} mm")
    if parts:
        print(f"  {'; '.join(parts)}")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 5), gridspec_kw={"width_ratios": [3, 1]})
    ts = t[nxt + 1] - t0
    shade(ax1, ts, human, "orange", "human rows in use")
    shade(ax1, ts, binding, "red", "a human row binding")
    ax1.plot(ts, 1e3 * worst, color="gray", linewidth=0.8, label="largest along the path")
    ax1.plot(ts, 1e3 * head, color="purple", linewidth=1.0, label="first shared point (t + 2dt)")
    phase_lines(ax1, d, t0)
    ax1.set_yscale("symlog", linthresh=0.1)
    ax1.set_ylim(bottom=0.0)
    ax1.set_title("Plan churn: how far each solve moves the path the previous one planned", fontsize=13)
    ax1.set_xlabel("Elapsed Time [s]", fontsize=12)
    ax1.set_ylabel("[mm]", fontsize=12)
    ax1.legend(fontsize=9, loc="upper right")
    ax1.grid(True, linestyle="--", alpha=0.5)
    steps = 1e3 * dt * np.arange(2, churn.shape[1] + 2)
    for label, mask, color in (("no human rows", ~human, "tab:blue"), ("human rows", human, "tab:orange")):
        if mask.sum() >= 10:
            ax2.plot(steps, 1e3 * np.median(churn[mask], axis=0), "o-", color=color, label=f"median, {label}")
            ax2.plot(steps, 1e3 * np.percentile(churn[mask], 95, axis=0), "--", color=color, label=f"p95, {label}")
    ax2.set_title("Along the path", fontsize=12)
    ax2.set_xlabel("Time ahead of t_k [ms]", fontsize=12)
    ax2.set_ylabel("[mm]", fontsize=12)
    ax2.legend(fontsize=8)
    ax2.grid(True, linestyle="--", alpha=0.5)
    save(fig, out_dir / "mpc_plan_churn.png")
    return m


def analyse_oscillations(d, out_dir):
    """Oscillation index and spectrum of u, of the measured q̇ and of the shield's correction.

    The task's own motion (point-to-point moves of ~1 s) is regular too: the same index on
    the task reference's TCP acceleration tells those apart. An oscillation within
    TASK_PERIOD_TOL of a regular one of the reference is the task's, the others the loop's.
    """
    dt, signals = d["dt"], []
    t_g = np.arange(d["t"][0], d["t"][-1], SIGNAL_GRID_S)
    if d["task"] is not None:
        p = interp_within(d["task"]["t"], d["task"]["p"], t_g, 0.1)
        p = pd.DataFrame(p).interpolate(limit_direction="both").to_numpy()
        signals.append(("task reference p̈", "m/s²", SIGNAL_GRID_S, list(AXES),
                        savgol_filter(p, 11, 3, deriv=2, delta=SIGNAL_GRID_S, axis=0)))
    labels = [f"joint {j + 1}" for j in range(JOINTS)]
    if d["q_des"] is not None:
        _, U = on_grid(d["q_des"]["t"], d["q_des"]["u"], dt)
        signals.append(("u (MPC q̈)", "rad/s²", dt, labels, U))
    if d["state"] is not None:
        signals.append(("measured q̇", "rad/s", SIGNAL_GRID_S, labels,
                        interp_within(d["state"]["t"], d["state"]["dq"], t_g, STATE_MAX_GAP_S)))
    if d["safe"] and d["nom"]:
        corr = zoh(d["safe"]["t"], d["safe"]["u"], t_g, 3 * dt) - zoh(d["nom"]["t"], d["nom"]["u"], t_g, 3 * dt)
        signals.append(("shield correction", "rad/s²", SIGNAL_GRID_S, labels, corr))
    if not signals:
        return [], {}
    rows, task_periods = [], []
    fig, axes = plt.subplots(1, len(signals), figsize=(7 * len(signals), 5), squeeze=False)
    for ax, (name, unit, period, columns, Y) in zip(axes[0], signals):
        for j, label in enumerate(columns):
            T, r, rms = oscillation_index(Y[:, j], period, DETREND_S)
            regular = bool(np.isfinite(r) and r > 1)
            if name.startswith("task") and regular:
                task_periods.append(T)
            in_task = regular and any(abs(T - Tt) <= TASK_PERIOD_TOL * Tt for Tt in task_periods)
            clear = regular and r >= OSC_CLEAR_R and 1.0 / T >= OSC_MIN_HZ
            rows.append(dict(signal=name, joint=j + 1, label=label, period_s=T, regularity=r,
                             rms_high_passed=rms, unit=unit, regular=regular, clear=clear,
                             in_task_reference=in_task))
            x = np.nan_to_num(high_pass(Y[:, j], period, DETREND_S))
            f, pxx = welch(x, fs=1.0 / period, nperseg=min(len(x), int(round(10.0 / period))))
            ax.semilogy(f[1:], pxx[1:], color=COLORS[j], linewidth=2 if clear else 1, label=label
                        + (f" ({1 / T:.2f} Hz, r {r:.1f}{', clear' if clear else ''})" if regular else ""))
        ax.set_title(f"{name}, high-passed ({DETREND_S:.0f} s)", fontsize=12)
        ax.set_xlabel("Frequency [Hz]", fontsize=12)
        ax.set_ylabel(f"PSD [({unit})²/Hz]", fontsize=12)
        ax.legend(fontsize=8)
        ax.grid(True, which="both", linestyle="--", alpha=0.5)
    fig.suptitle("Oscillations: spectra and Thornhill regularity r (> 1: a regular oscillation)", fontsize=13)
    save(fig, out_dir / "mpc_oscillation_psd.png")
    describe = lambda r: f"{r['signal']} {r['label']}: {1 / r['period_s']:.2f} Hz (r {r['regularity']:.1f}, rms {r['rms_high_passed']:.3g} {r['unit']})"
    own = lambda r: not r["signal"].startswith("task") and not r["in_task_reference"]
    loop = [describe(r) for r in rows if r["clear"] and own(r)]
    weak = sum(r["regular"] and not r["clear"] and own(r) for r in rows)
    task = [describe(r) for r in rows if r["regular"] and r["in_task_reference"] and not r["signal"].startswith("task")]
    print(f"  clear oscillations (r ≥ {OSC_CLEAR_R:g}, ≥ {OSC_MIN_HZ:g} Hz), not the task's: "
          f"{'; '.join(loop) if loop else 'none'} [{weak} weaker with r > 1 in summary_oscillation.csv]")
    if task_periods:
        print(f"  with the period of the task's own motion ({', '.join(f'{T:.2f}' for T in task_periods)} s): "
              f"{'; '.join(task) if task else 'none'}")
    return rows, dict(oscillations=len(loop))


# ── 4, 5, 7. Cost, multipliers, decrease of V ───────────────────────────────

def cost_series(d):
    """V, its terminal part and ℓ on the solve grid, or None when not logged."""
    diag = d["diag"]
    cols = ["cost_full", "stage_cost", "terminal_cost"]
    if not all(c in diag for c in cols) or not np.isfinite(diag["cost_full"]).any():
        return None
    k, G = on_grid(d["t"], diag[cols].to_numpy(dtype=float), d["dt"])
    t_grid = d["t"][0] + d["dt"] * np.arange(len(G))
    _, H = on_grid(d["t"], diag[["human_rows", "human_binding"]].to_numpy(dtype=float), d["dt"])
    return dict(t=t_grid, V=G[:, 0], l=G[:, 1], T=G[:, 2], human=H[:, 0] > 0, binding=H[:, 1] > 0,
                human_rows=H[:, 0])


def analyse_cost_ratio(d, c, n_steps, out_dir, t0):
    """Point 4: η_k = (V_k − T_k) / Σ_{j<N} ℓ_{k+j}."""
    J = np.full(len(c["l"]), np.nan)
    if len(c["l"]) >= n_steps:
        J[:len(c["l"]) - n_steps + 1] = sliding_window_view(c["l"], n_steps).sum(axis=1)
    eta = (c["V"] - c["T"]) / J
    ok = np.isfinite(eta) & (J > 0)
    if ok.sum() < MIN_SOLVES:
        warn("too few windows of N consecutive solves: cost ratio skipped.")
        return {}
    e = eta[ok]
    inside = np.mean((e >= ETA_BAND[0]) & (e <= ETA_BAND[1]))
    m = dict(eta_median=float(np.median(e)), eta_p25=float(np.percentile(e, 25)),
             eta_p75=float(np.percentile(e, 75)), eta_in_band=float(inside))
    parts = []
    for label, mask in (("no human rows", ok & ~c["human"]), ("human rows", ok & c["human"])):
        if mask.sum() >= 10:
            m[f"eta_median_{label.replace(' ', '_')}"] = float(np.median(eta[mask]))
            parts.append(f"{label} {np.median(eta[mask]):.2f}")
    print(f"\n4. Achieved vs design cost: η = (V_k − terminal) / Σ ℓ over the next {n_steps} solves, "
          f"{ok.sum()} windows")
    print(f"  η median {m['eta_median']:.2f} (IQR {m['eta_p25']:.2f}–{m['eta_p75']:.2f}), "
          f"in [{ETA_BAND[0]}, {ETA_BAND[1]}] {100 * inside:.0f}%" + (f"; median {', '.join(parts)}" if parts else ""))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    ts = c["t"] - t0
    ax1.semilogy(ts, c["V"] - c["T"], color="blue", linewidth=1, label="Design: $V_k$ − terminal")
    ax1.semilogy(ts, J, color="red", linewidth=1, label=f"Achieved: Σ ℓ over the next {n_steps} solves")
    shade(ax1, ts, c["human"], "orange", "human rows in use")
    phase_lines(ax1, d, t0)
    ax1.set_ylabel("Cost", fontsize=12)
    ax1.set_title("Achieved vs design cost of the MPC", fontsize=14)
    ax1.legend(fontsize=9, loc="upper right")
    ax1.grid(True, which="both", linestyle="--", alpha=0.5)
    ax2.axhspan(*ETA_BAND, color="green", alpha=0.12, label=f"[{ETA_BAND[0]}, {ETA_BAND[1]}]")
    ax2.plot(ts, eta, color="purple", linewidth=1, label=f"η (median {m['eta_median']:.2f})")
    ax2.axhline(1.0, color="black", linewidth=1)
    shade(ax2, ts, c["human"], "orange")
    phase_lines(ax2, d, t0)
    ax2.set_ylim(0, max(2.0, min(5.0, np.nanpercentile(eta[ok], 99) * 1.1)))
    ax2.set_xlabel("Elapsed Time [s]", fontsize=12)
    ax2.set_ylabel("η = design / achieved", fontsize=12)
    ax2.legend(fontsize=9, loc="upper right")
    ax2.grid(True, linestyle="--", alpha=0.5)
    save(fig, out_dir / "mpc_cost_ratio.png")
    return m


def analyse_multipliers(d, args, out_dir, t0):
    """Point 5."""
    diag = d["diag"]
    fams = [("lambda_human", "human rows", "cost/m"), ("lambda_accel", "q̈ box", "cost/(rad/s²)"),
            ("lambda_position", "joint position", "cost/rad"), ("lambda_velocity", "joint velocity", "cost/(rad/s)")]
    if not all(f in diag for f, _, _ in fams) or not np.isfinite(diag["lambda_human"]).any():
        return {}
    solved = diag["solved"].to_numpy(dtype=float) == 1
    print(f"\n5. Multipliers over {solved.sum()} solved cycles (max |λ| per family; binding: λ > {LAMBDA_TOL:g})")
    m = {}
    fig, axes = plt.subplots(len(fams), 1, figsize=(12, 10), sharex=True)
    ts = d["t"] - t0
    for ax, (col, name, unit) in zip(axes, fams):
        lam = diag[col].to_numpy(dtype=float)
        active = solved & (lam > LAMBDA_TOL)
        share = active.sum() / max(solved.sum(), 1)
        m[f"{col}_binding_share"] = float(share)
        if active.any():
            m[f"{col}_p95"] = float(np.percentile(lam[active], 95))
            m[f"{col}_max"] = float(lam[active].max())
            print(f"  {name:15s} binding {100 * share:5.1f}% | p50 {np.median(lam[active]):9.3g}  "
                  f"p95 {m[f'{col}_p95']:9.3g}  max {m[f'{col}_max']:9.3g}  [{unit}]")
        else:
            print(f"  {name:15s} never binding")
        y = lam / args.rho if col == "lambda_human" else lam
        ax.plot(ts[active], y[active], ".", markersize=2, color="purple")
        if col == "lambda_human":
            ax.axhline(1.0, color="red", linestyle="--", label="ρ: the slack takes over")
            ax.set_ylabel("λ_human / ρ", fontsize=11)
            shade(ax, ts, d["diag"]["human_binding"].to_numpy(dtype=float) > 0, "orange", "a human row binding")
            ax.legend(fontsize=8, loc="upper right")
        else:
            ax.set_ylabel(f"{name}\n[{unit}]", fontsize=10)
        ax.grid(True, linestyle="--", alpha=0.5)
    axes[0].set_title("Lagrange multipliers of the MPC rows (largest per family, binding cycles)", fontsize=13)
    axes[-1].set_xlabel("Elapsed Time [s]", fontsize=12)
    save(fig, out_dir / "mpc_multipliers.png")

    lam_h = diag["lambda_human"].to_numpy(dtype=float)
    slack = diag["max_slack"].to_numpy(dtype=float)
    held = solved & (lam_h > LAMBDA_TOL) & (slack <= 1e-4)
    on_slack = solved & (slack > 1e-4)
    m["lambda_human_over_rho_max_held"] = float(lam_h[held].max() / args.rho) if held.any() else np.nan
    line = (f"  human rows held without slack: max λ/ρ {m['lambda_human_over_rho_max_held']:.2f} "
            f"({held.sum()} cycles); on their slack: {on_slack.sum()} cycles")
    if on_slack.any():
        kkt = np.abs(lam_h[on_slack] - (args.rho + args.sigma * slack[on_slack])) / (args.rho + args.sigma * slack[on_slack])
        m["kkt_slack_consistent_share"] = float(np.mean(kkt < 0.02))
        line += f", λ = ρ + σ·s within 2% in {100 * m['kkt_slack_consistent_share']:.0f}%"
    print(line)
    return m


def reference_fixed(d, t, span):
    """Per time in t: the task reference stays within REF_FIXED_M of where it is over [t, t + span]
    (a regulation problem over the whole horizon); past the end of the log, as far as it goes.
    All False without a task log."""
    if d["task"] is None:
        return np.zeros(len(t), bool)
    offsets = np.linspace(0.0, span, 11)
    p = np.stack([interp_within(d["task"]["t"], d["task"]["p"], t + o, 0.1) for o in offsets], axis=1)
    moved = np.linalg.norm(p - p[:, :1], axis=2)
    return np.isfinite(moved[:, 0]) & np.all(~(moved >= REF_FIXED_M), axis=1)


def analyse_decrease(d, c, n_steps, out_dir, t0):
    """Point 7: α_k = (V_k − V_{k+1}) / ℓ_k on consecutive solves, in three classes of cycle.

    The certificate is for regulation: with the reference fixed over the whole horizon and no
    human rows, V should not go up (α ≥ 0). A moving reference raises V whenever it pulls
    away faster than the robot follows, and a person changes the problem itself.
    """
    V, l = c["V"], c["l"]
    alpha = np.full(len(V), np.nan)
    alpha[:-1] = (V[:-1] - V[1:]) / l[:-1]
    ok = np.isfinite(alpha) & (l > 0)
    ok[-1] = False
    if ok.sum() < MIN_SOLVES:
        warn("too few consecutive solves with a cost: decrease of V skipped.")
        return {}
    a = alpha[ok]
    human = c["human"] | np.r_[c["human"][1:], False]                  # rows at k or at k + 1
    fixed = reference_fixed(d, c["t"], (n_steps + 1) * d["dt"]) & ~human
    classes = (("fixed", "fixed reference, no human rows", ok & fixed),
               ("moving", "moving reference, no human rows", ok & ~fixed & ~human),
               ("human", "human rows", ok & human))
    m = dict(alpha_median=float(np.median(a)), alpha_p5=float(np.percentile(a, 5)),
             alpha_nonneg_share=float(np.mean(a >= 0)))
    print(f"\n7. Decrease of V: α = (V_k − V_k+1) / ℓ_k over {ok.sum()} consecutive solves: α ≥ 0 in "
          f"{100 * m['alpha_nonneg_share']:.1f}%, median {m['alpha_median']:.2f}, p5 {m['alpha_p5']:.2f}")
    for key, name, mask in classes:
        m[f"alpha_{key}_cycles"] = int(mask.sum())
        m[f"alpha_{key}_up"] = int(np.sum(alpha[mask] < 0))
        if mask.any():
            m[f"alpha_{key}_nonneg_share"] = float(np.mean(alpha[mask] >= 0))
            m[f"alpha_{key}_min"] = float(np.min(alpha[mask]))
            print(f"  {name:32s} {mask.sum():5d} cycles: α ≥ 0 in {100 * m[f'alpha_{key}_nonneg_share']:5.1f}%, "
                  f"median {np.median(alpha[mask]):6.2f}, min {m[f'alpha_{key}_min']:7.2f}")
        else:
            print(f"  {name:32s}     0 cycles" + (" (no task log)" if key == "fixed" and d["task"] is None else ""))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 5), gridspec_kw={"width_ratios": [3, 1]})
    ts = c["t"] - t0
    shade(ax1, ts, c["human"], "orange", "human rows in use")
    shade(ax1, ts, fixed, "green", "reference fixed over the horizon")
    ax1.plot(ts[ok], np.clip(alpha[ok], -3, 2), ".", markersize=2, color="purple", label="α (clipped to [−3, 2])")
    up_fixed = ok & fixed & (alpha < 0)
    ax1.plot(ts[up_fixed], np.clip(alpha[up_fixed], -3, 2), "x", color="red", markersize=5,
             label="V up with the reference fixed and no human rows")
    ax1.axhline(0, color="black", linewidth=1)
    phase_lines(ax1, d, t0)
    ax1.set_title("Decrease of the optimal cost: α = (V_k − V_{k+1}) / ℓ_k  (grey lines: task phases)", fontsize=13)
    ax1.set_xlabel("Elapsed Time [s]", fontsize=12)
    ax1.set_ylabel("α", fontsize=12)
    ax1.legend(fontsize=8, loc="lower right")
    ax1.grid(True, linestyle="--", alpha=0.5)
    bins = np.linspace(-3, 2, 51)
    for key, name, mask in classes:
        if mask.any():
            ax2.hist(np.clip(alpha[mask], -3, 2), bins=bins, alpha=0.5, label=f"{name} ({mask.sum()})",
                     color={"fixed": "green", "moving": "gray", "human": "orange"}[key])
    ax2.axvline(0, color="black", linewidth=1)
    ax2.set_title("α by class of cycle", fontsize=12)
    ax2.set_xlabel("α", fontsize=12)
    ax2.legend(fontsize=7)
    ax2.grid(True, linestyle="--", alpha=0.5)
    save(fig, out_dir / "mpc_cost_decrease.png")
    return m


# ── 6. Real time ────────────────────────────────────────────────────────────

def analyse_realtime(d, out_dir, t0):
    diag, dt = d["diag"], d["dt"]
    solve = diag["solve_time_ms"].to_numpy(dtype=float)
    iters = diag["iterations"].to_numpy(dtype=float)
    gaps = np.diff(d["t"])
    m = dict(solves=len(diag), solved_share=float(np.mean(diag["solved"] == 1)),
             fallbacks=int(np.nansum(diag["fallback"])), solve_ms_p50=float(np.nanmedian(solve)),
             solve_ms_p95=float(np.nanpercentile(solve, 95)), solve_ms_p99=float(np.nanpercentile(solve, 99)),
             solve_ms_max=float(np.nanmax(solve)), iterations_max=float(np.nanmax(iters)),
             missed_cycles=int(np.sum(gaps > 1.5 * dt)), cycle_gap_max_ms=1e3 * float(np.max(gaps)))
    latency = None
    if d["q_des"] is not None:
        latency = 1e3 * (d["q_des"]["recv"] - d["q_des"]["t"])
        m.update(loop_ms_p50=float(np.nanmedian(latency)), loop_ms_p95=float(np.nanpercentile(latency, 95)),
                 loop_ms_max=float(np.nanmax(latency)), loop_over_dt=int(np.sum(latency > 1e3 * dt)))
    statuses = diag["status"].value_counts().to_dict() if "status" in diag else {}
    print(f"\n6. Real time over {m['solves']} cycles (dt {1e3 * dt:.0f} ms)")
    print(f"  solve {m['solve_ms_p50']:.2f} ms median, p95 {m['solve_ms_p95']:.2f}, p99 {m['solve_ms_p99']:.2f}, "
          f"max {m['solve_ms_max']:.2f}; iterations max {m['iterations_max']:.0f}; "
          f"statuses {', '.join(f'{k} {v}' for k, v in statuses.items())}; fallbacks {m['fallbacks']}")
    line = f"  missed cycles {m['missed_cycles']} (largest gap {m['cycle_gap_max_ms']:.0f} ms)"
    if latency is not None:
        line += (f"; loop latency (stamp → q_des received) {m['loop_ms_p50']:.1f} ms median, p95 "
                 f"{m['loop_ms_p95']:.1f}, max {m['loop_ms_max']:.1f}, over dt {m['loop_over_dt']}")
    print(line)

    fig, axes = plt.subplots(1, 3, figsize=(18, 5), gridspec_kw={"width_ratios": [3, 1, 1]})
    ts = d["t"] - t0
    ax = axes[0]
    shade(ax, ts, diag["human_rows"].to_numpy(dtype=float) > 0, "orange", "human rows in use")
    ax.semilogy(ts, solve, ".", markersize=2, color="purple", label="QP solve")
    if latency is not None:
        ax.semilogy(d["q_des"]["t"] - t0, latency, ".", markersize=2, color="teal", label="loop latency")
    ax.axhline(1e3 * dt, color="red", linestyle="--", label=f"dt = {1e3 * dt:.0f} ms")
    ax.set_title("MPC timing per cycle", fontsize=13)
    ax.set_xlabel("Elapsed Time [s]", fontsize=12)
    ax.set_ylabel("[ms]", fontsize=12)
    ax.legend(fontsize=8, loc="upper right")
    ax.grid(True, which="both", linestyle="--", alpha=0.5)
    for ax, values, name, color in ((axes[1], solve, "QP solve time", "purple"),
                                    (axes[2], latency, "Loop latency", "teal")):
        if values is None:
            ax.axis("off")
            continue
        v = values[np.isfinite(values)]
        ax.hist(v, bins=60, color=color, alpha=0.7)
        ax.axvline(np.percentile(v, 99), color="black", linestyle=":", label=f"p99 {np.percentile(v, 99):.2f} ms")
        ax.set_title(name, fontsize=12)
        ax.set_xlabel("[ms]", fontsize=12)
        ax.legend(fontsize=8)
        ax.grid(True, linestyle="--", alpha=0.5)
    save(fig, out_dir / "mpc_realtime.png")
    return m


# ── Main ────────────────────────────────────────────────────────────────────

def main(argv=None):
    parser = argparse.ArgumentParser(description="A-posteriori diagnostics of the MPC on one run.")
    parser.add_argument("run_name", nargs="?", default=None,
                        help="run folder name or <controller>/<run>; the latest mpc/mpc_cbf run if omitted")
    parser.add_argument("--max-lag", type=int, default=20, help="autocorrelation lags [solves]")
    parser.add_argument("--window-s", type=float, default=0.05,
                        help="Savitzky-Golay window for the executed q̈ [s] (default 0.05: one MPC step)")
    parser.add_argument("--rho", type=float, default=1000.0, help="slack_linear of the run (config/mpc.yaml)")
    parser.add_argument("--sigma", type=float, default=1000.0, help="slack_quadratic of the run (config/mpc.yaml)")
    parser.add_argument("--log-dir", default=LOG_DIR)
    parser.add_argument("--out-dir", default=GRAPH_DIR)
    args = parser.parse_args(argv)
    if args.max_lag < 1 or args.window_s <= 0.0:
        fail("--max-lag must be at least 1 and --window-s positive.")

    run = resolve_run(args.log_dir, args.run_name, controllers=CONTROLLERS)
    name = run.relative_to(Path(args.log_dir).expanduser()).as_posix()
    print(f"Analysing run: {name}")
    d = load_run(run)
    out_dir = Path(args.out_dir).expanduser() / name / "mpc_analysis"
    out_dir.mkdir(parents=True, exist_ok=True)
    t0 = d["t"][0]
    n_steps = d["plan"]["p"].shape[1] if d["plan"] is not None else 10
    print(f"  {len(d['t'])} solves over {d['t'][-1] - t0:.1f} s, dt {1e3 * d['dt']:.0f} ms, N {n_steps}; "
          f"q̈ to the torque controller: {'qddot_safe (shielded)' if d['safe'] else 'qddot_nom'}")

    metrics = dict(run=name)
    rows_res, m = analyse_residuals(d, args, out_dir, t0)
    metrics.update(m)
    rows_acc, m = analyse_acceleration(d, args, out_dir, t0)
    metrics.update(m)
    metrics.update(analyse_churn(d, out_dir, t0))
    rows_osc, m = analyse_oscillations(d, out_dir)
    metrics.update(m)
    c = cost_series(d)
    if c is None:
        print("\n4, 5, 7: no cost and multiplier columns in this run's diagnostics (logged from 2026-10-24 on).")
    else:
        metrics.update(analyse_cost_ratio(d, c, n_steps, out_dir, t0))
        metrics.update(analyse_multipliers(d, args, out_dir, t0))
    metrics.update(analyse_realtime(d, out_dir, t0))
    if c is not None:
        metrics.update(analyse_decrease(d, c, n_steps, out_dir, t0))

    for rows, file in ((rows_res, "summary_residuals.csv"), (rows_acc, "summary_acceleration.csv"),
                       (rows_osc, "summary_oscillation.csv"), ([metrics], "summary_mpc.csv")):
        if rows:
            pd.DataFrame(rows).to_csv(out_dir / file, index=False)
    plots = len(list(out_dir.glob("*.png")))
    print(f"\nSaved {plots} plots and the summaries in: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())