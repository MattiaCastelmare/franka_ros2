#!/usr/bin/env python3
"""Self-test of analysis_utils.py, kf_compare.py and whiteness_test.py (no pytest needed).

Synthetic logs with a known truth: the filter's velocity delayed by a known amount, white
or correlated innovations, consistent NIS, a looped bag, dropped frames, invalid keypoints
logged as 0, a heartbeat before engagement, empty files, single-arm and old-format runs.

    python3 test_kf_analysis.py        (prints every check, exits 1 on the first failure)
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
from scipy.signal import savgol_filter

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kf_compare  # noqa: E402
import analysis_utils as u  # noqa: E402
import whiteness_test  # noqa: E402

PERIOD = 1.0 / 30.0
T_START = 1789637008.0


# ── Synthetic run ────────────────────────────────────────────────────────────

def _truth(t):
    """Smooth arm motion: position (N, 4, 3) and its exact velocity."""
    tt = t - T_START
    pos, vel = np.empty((len(t), 4, 3)), np.empty((len(t), 4, 3))
    for i in range(4):
        for a in range(3):
            pos[:, i, a], vel[:, i, a] = 0.3 * a, 0.0
            # A slow reach (0.25-0.35 Hz) and a quicker hand gesture (0.8-0.95 Hz)
            for amp, f, ph in ((0.06 + 0.03 * i, 0.25 + 0.05 * a, 0.7 * i + a),
                               (0.03, 0.8 + 0.05 * i, 1.3 * a + i)):
                w = 2 * np.pi * f
                pos[:, i, a] += amp * np.sin(w * tt + ph)
                vel[:, i, a] += amp * w * np.cos(w * tt + ph)
    return pos, vel


def make_run(root: Path, name: str, *, dual=True, n=1500, kf_delay=0.1, innovations="white",
             loop=False, drop=0.1, old_format=False, seed=0):
    """Write a run folder; returns its path."""
    rng = np.random.default_rng(seed)
    run = root / name
    run.mkdir(parents=True)
    t_all = T_START + np.arange(n) * PERIOD
    kept = rng.random(n) > drop                          # dropped frames
    kept[:5] = True
    t = t_all[kept]
    heartbeat = T_START - 2.0 + np.arange(60) * PERIOD    # state rows before engagement
    for side in (("left", "right") if dual else (None,)):
        prefix = f"experiment_{side}_" if side else "experiment_"
        pos, vel = _truth(t)
        raw = pos + rng.normal(0, 0.003, pos.shape)
        raw[rng.random((len(t), 4)) < 0.05] = np.nan            # missing keypoints
        _, vel_delayed = _truth(t - kf_delay)
        kf_vel = vel_delayed + rng.normal(0, 0.01, vel.shape)
        valid = np.ones((len(t), 4), dtype=int)
        valid[:10] = 0                                           # not valid yet: logged as 0
        measured = (np.isfinite(raw).all(axis=-1) & (valid == 1)).astype(int)

        cols_raw = {"timestamp": t}
        if not old_format:
            cols_raw["recv_time"] = t + 0.03
        for i, kp in enumerate(u.KEYPOINTS):
            for a, ax in enumerate(u.AXES):
                cols_raw[f"{kp}_{ax}"] = raw[:, i, a]
        df_raw = pd.DataFrame(cols_raw)

        st_t = np.r_[heartbeat, t]
        cols_st = {"timestamp": st_t}
        if not old_format:
            cols_st["recv_time"] = st_t + 0.03
        zeros = np.zeros(len(heartbeat))
        for i, kp in enumerate(u.KEYPOINTS):
            for a, ax in enumerate(u.AXES):
                cols_st[f"{kp}_{ax}"] = np.r_[zeros, np.where(valid[:, i] == 1, pos[:, i, a], 0.0)]
            for a, ax in enumerate(u.AXES):
                cols_st[f"{kp}_v{ax}"] = np.r_[zeros, np.where(valid[:, i] == 1, kf_vel[:, i, a], 0.0)]
        for i in range(4):
            cols_st[f"vis_{i}"] = np.r_[zeros + 0.9, np.full(len(t), 0.9)]
            cols_st[f"measured_{i}"] = np.r_[zeros, measured[:, i]].astype(int)
            cols_st[f"valid_{i}"] = np.r_[zeros, valid[:, i]].astype(int)
            cols_st[f"age_{i}"] = np.r_[zeros - 1, np.zeros(len(t))]
        df_st = pd.DataFrame(cols_st)

        cols_dg = {"timestamp": t}
        if not old_format:
            cols_dg["recv_time"] = t + 0.03
        for i, kp in enumerate(u.KEYPOINTS):
            if innovations == "white":
                inn = rng.normal(0, 0.01, (len(t), 3))
            else:                                               # AR(1), phi = 0.7, per frame
                inn = np.zeros((len(t), 3))
                e = rng.normal(0, 0.01, (len(t), 3))
                for k in range(1, len(t)):
                    steps = int(round((t[k] - t[k - 1]) / PERIOD))
                    inn[k] = 0.7**steps * inn[k - 1] + e[k]
            nis = rng.chisquare(3, len(t))
            gated = rng.random(len(t)) < 0.01
            upd = measured[:, i] == 1
            inn[~upd & ~gated] = 0.0                            # no update: zero innovation
            inn[gated] *= 8.0
            nis[gated] = 30.0
            nis[~upd & ~gated] = np.nan
            meas_i = measured[:, i].copy()
            meas_i[gated] = 0
            measured[:, i] = meas_i
            for a, ax in enumerate(u.AXES):
                cols_dg[f"{kp}_inn_{ax}"] = inn[:, a]
            cols_dg[f"{kp}_p_trace"] = np.where(valid[:, i] == 1, 0.05 + np.exp(-np.arange(len(t)) / 30), 0.0)
            if not old_format:
                cols_dg[f"{kp}_nis"] = nis
                cols_dg[f"{kp}_gated"] = gated.astype(int)
        df_dg = pd.DataFrame(cols_dg)
        # The state's measured flags must agree with the gating written in the diagnostics
        for i in range(4):
            df_st[f"measured_{i}"] = np.r_[zeros, measured[:, i]].astype(int)

        if loop:   # the bag restarts: a second, shorter pass over the same stamps
            df_raw = pd.concat([df_raw, df_raw.iloc[:200]], ignore_index=True)
            df_st = pd.concat([df_st, df_st.iloc[:260]], ignore_index=True)
            df_dg = pd.concat([df_dg, df_dg.iloc[:200]], ignore_index=True)
        df_raw.to_csv(run / f"{prefix}human_raw.csv", index=False)
        df_st.to_csv(run / f"{prefix}human_state.csv", index=False)
        if not old_format or dual:
            df_dg.to_csv(run / f"{prefix}kf_diagnostics.csv", index=False)
    return run


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
def local_poly_derivative_is_savitzky_golay():
    rng = np.random.default_rng(1)
    t = np.arange(400) * PERIOD
    y = np.sin(3 * t) + rng.normal(0, 0.01, t.size)
    ours = u.local_poly_derivative(t, y, window_s=0.5)
    ref = savgol_filter(y, 15, 2, deriv=1, delta=PERIOD)
    np.testing.assert_allclose(ours[7:-7], ref[7:-7], rtol=1e-9, atol=1e-9)
    assert np.isnan(ours[:3]).all()                    # no one-sided fit at the edges


@check
def local_poly_derivative_handles_uneven_stamps_and_gaps():
    rng = np.random.default_rng(2)
    t = np.sort(rng.uniform(0, 10, 300))
    t = t[(t < 4) | (t > 5)]                            # a 1 s gap
    y = 0.3 + 0.8 * t - 0.05 * t**2                      # exact for a quadratic fit
    d = u.local_poly_derivative(t, y, window_s=0.5)
    ok = np.isfinite(d)
    np.testing.assert_allclose(d[ok], 0.8 - 0.1 * t[ok], atol=1e-9)
    near_gap = (t > 3.9) & (t < 4.0) | (t > 5.0) & (t < 5.1)
    assert np.isnan(d[near_gap]).all()                  # no slope from one side of a gap


@check
def savitzky_golay_attenuates_fast_motion_as_theory_says():
    """The reference is a low-pass: for a sinusoid of frequency f and window w it returns
    the velocity times 3 (sin x - x cos x) / x^3, x = pi f w (window w = 2h)."""
    t = np.arange(3000) * PERIOD
    for f in (0.3, 0.6, 1.0, 1.5):
        d = u.local_poly_derivative(t, np.sin(2 * np.pi * f * t), window_s=0.5)
        gain = np.nanmax(np.abs(d)) / (2 * np.pi * f)
        x = np.pi * f * 0.5
        assert abs(gain - 3 * (np.sin(x) - x * np.cos(x)) / x**3) < 0.01, (f, gain)


@check
def interpolation_never_bridges_a_gap():
    t = np.array([0.0, 0.1, 0.2, 1.0, 1.1])
    y = np.array([0.0, 1.0, 2.0, 10.0, 11.0])[:, None]
    out = u.interp_within(t, y, np.array([0.05, 0.2, 0.6, 1.05, 2.0]), max_gap=0.15)[:, 0]
    np.testing.assert_allclose(out[[0, 1, 3]], [0.5, 2.0, 10.5])
    assert np.isnan(out[2]) and np.isnan(out[4])


@check
def match_finds_the_same_frame_only():
    t = np.array([0.0, 1.0, 2.0])
    np.testing.assert_array_equal(u.match(t, np.array([-1.0, 0.1, 0.6, 2.0, 2.4]), 0.25),
                                  [-1, 0, -1, 2, -1])


@check
def autocorrelation_pairs_by_stamps_despite_dropped_frames():
    rng = np.random.default_rng(3)
    n = 20000
    y = np.zeros(n)
    e = rng.normal(size=n)
    for k in range(1, n):
        y[k] = 0.6 * y[k - 1] + e[k]
    t = np.arange(n) * PERIOD
    kept = rng.random(n) > 0.3                           # 30 % of the frames dropped
    rho, pairs = u.acf_gapped(t[kept], y[kept], 3, PERIOD)
    np.testing.assert_allclose(rho, [0.6, 0.36, 0.216], atol=0.03)
    naive = np.corrcoef(y[kept][:-1], y[kept][1:])[0, 1]  # index lag 1 mixes lags 1, 2, 3...
    assert naive < 0.55, naive


@check
def portmanteau_accepts_white_and_rejects_ar1():
    rng = np.random.default_rng(4)
    t = np.arange(3000) * PERIOD
    white = rng.normal(size=t.size)
    _, p_white, _ = u.portmanteau(*u.acf_gapped(t, white, 20, PERIOD))
    ar = np.zeros(t.size)
    for k in range(1, t.size):
        ar[k] = 0.3 * ar[k - 1] + white[k]
    _, p_ar, _ = u.portmanteau(*u.acf_gapped(t, ar, 20, PERIOD))
    assert p_white > 0.05 and p_ar < 1e-6, (p_white, p_ar)


@check
def nis_interval_is_chi2():
    lo, hi = u.nis_interval(3, 1)
    assert abs(lo - 0.216) < 1e-3 and abs(hi - 9.348) < 1e-3
    lo, hi = u.nis_interval(3, 2000)
    assert 2.85 < lo < 3.0 < hi < 3.15


@check
def a_looped_bag_keeps_the_same_longest_pass_in_every_file():
    with tempfile.TemporaryDirectory() as tmp:
        run = make_run(Path(tmp), "20261009_091036", loop=True)
        paths = {"human_raw.csv": run / "experiment_left_human_raw.csv",
                 "human_state.csv": run / "experiment_left_human_state.csv"}
        logs, out = quiet(u.load_arm, paths)
        assert "restart" in out
        for df in logs.values():
            t = df["timestamp"].to_numpy()
            assert np.all(np.diff(t) > 0)
        assert len(logs["human_raw.csv"]) > 1000


@check
def invalid_keypoints_are_nan_not_zero():
    with tempfile.TemporaryDirectory() as tmp:
        run = make_run(Path(tmp), "20261011_182441")
        st = u.state_arrays(u.read_numeric_csv(run / "experiment_right_human_state.csv"))
        assert np.isnan(st["vel"][:70]).all()          # heartbeat + first 10 frames
        assert np.isfinite(st["vel"][80:]).all()


@check
def kf_compare_finds_the_known_delay_on_one_time_axis():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        make_run(root / "logs", "20261011_182441", kf_delay=0.1, loop=True)
        code, out = quiet(kf_compare.main, ["20261011_182441", "--log-dir", str(root / "logs"),
                                            "--out-dir", str(root / "graphs")])
        assert code == 0, out
        out_dir = root / "graphs" / "20261011_182441" / "savitzky_golay"
        for side in ("left", "right"):
            s = pd.read_csv(out_dir / f"summary_{side}.csv")
            xyz = s[s.axis == "xyz"]
            # Within half a frame period: the resolution a 30 Hz log has on a delay
            assert np.all(np.abs(xyz.delay_ms - 100.0) <= 0.5e3 * PERIOD), xyz.delay_ms.tolist()
            assert np.all(xyz.rms_err_at_delay < 0.5 * xyz.rms_err), xyz[["rms_err", "rms_err_at_delay"]]
        assert len(list(out_dir.glob("kf_validation_*.png"))) == 24


@check
def whiteness_test_tells_white_from_correlated_innovations():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        for name, kind in (("20261011_000001", "white"), ("20261011_000002", "ar1")):
            make_run(root / "logs", name, innovations=kind, seed=5)
            code, out = quiet(whiteness_test.main, [name, "--log-dir", str(root / "logs"),
                                                    "--out-dir", str(root / "graphs")])
            assert code == 0, out
            s = pd.concat(pd.read_csv(f) for f in (root / "graphs" / name / "whiteness_test").glob("summary_*.csv"))
            if kind == "white":
                assert s.white.mean() >= 0.8, s[["keypoint", "axis", "p_white"]]
                assert (s.nis_lo <= s.nis_mean).all() and (s.nis_mean <= s.nis_hi * 1.5).all()
            else:
                assert not s.white.any(), s[["keypoint", "axis", "p_white"]]
                np.testing.assert_allclose(s.rho1, 0.7, atol=0.06)
            assert (s.gate_rate < 0.03).all()
        plots = list((root / "graphs" / "20261011_000001" / "whiteness_test").glob("*.png"))
        assert len(plots) == 2 * (4 + 12 + 4), len(plots)   # trace, axes, NIS


@check
def old_and_broken_runs_fail_gracefully():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        logs = root / "logs"
        # Single-arm, no filter diagnostics (August-September format)
        make_run(logs, "20260821_162400", dual=False, old_format=True)
        code, out = quiet(kf_compare.main, ["20260821_162400", "--log-dir", str(logs),
                                            "--out-dir", str(root / "g")])
        assert code == 0 and "arm:" in out, out
        code, out = quiet(whiteness_test.main, ["20260821_162400", "--log-dir", str(logs),
                                                "--out-dir", str(root / "g")])
        assert code == "SystemExit(1)" and "no kf_diagnostics" in out, out
        # Dual-arm, diagnostics without nis / gated (16 September format)
        make_run(logs, "20260916_175158", old_format=True)
        code, out = quiet(whiteness_test.main, ["20260916_175158", "--log-dir", str(logs),
                                                "--out-dir", str(root / "g")])
        assert code == 0 and "no NIS" in out, out
        # Header-only files
        empty = logs / "20260916_165907"
        empty.mkdir()
        for f in ("human_raw", "human_state", "kf_diagnostics"):
            (empty / f"experiment_left_{f}.csv").write_text("timestamp,shoulder_x\n")
        for script in (kf_compare, whiteness_test):
            code, out = quiet(script.main, ["20260916_165907", "--side", "left", "--log-dir", str(logs),
                                            "--out-dir", str(root / "g")])
            assert code == 1 and "no data" in out, out
        # Missing run, missing log folder
        code, out = quiet(kf_compare.main, ["19990101_000000", "--log-dir", str(logs)])
        assert code == "SystemExit(1)" and "not found" in out, out
        code, out = quiet(whiteness_test.main, ["--log-dir", str(root / "nowhere")])
        assert code == "SystemExit(1)" and "does not exist" in out, out


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