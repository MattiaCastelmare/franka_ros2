#!/usr/bin/env python3
"""Kalman filter velocity against an offline Savitzky-Golay reference, per keypoint and axis.

The reference is the derivative of a centred quadratic least-squares fit to the raw
positions over a fixed time window (Savitzky-Golay), computed on the actual stamps: dropped
frames and gaps are handled and nothing is interpolated across a gap. The reference sees
the future and the filter does not, so the filter lags it: next to the rms difference, each
keypoint gets the delay that best aligns the filter's velocity with the reference.

The reference is itself a low-pass filter: a sinusoid of frequency f comes out scaled by
3 (sin x - x cos x) / x^3, x = pi f w (window w): -8 % at 0.6 Hz and -22 % at 1 Hz with the
default 0.5 s. On fast hand motion a shorter --window-s keeps it honest (and noisier).

By default the reference is built only from the raw samples the filter accepted
(--reference measured): an outlier the gate rejected would otherwise make the reference
itself spike. Both curves share one time origin, the first frame logged in either file.

Outputs, in <out-dir>/<run>/savitzky_golay/:
  kf_validation_<side>_<keypoint>_<axis>.png   both velocities on one time axis
  summary_<side>.csv                           per keypoint and axis: n, rms, bias, corr, delay

Usage:
  python3 kf_compare.py [RUN] [--side left|right|both] [--window-s 0.5]
                        [--reference measured|all] [--log-dir DIR] [--out-dir DIR]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")   # files only: runs without a display
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from analysis_utils import (
    AXES, GAP_FACTOR, GRAPH_DIR, KEYPOINTS, LOG_DIR, MATCH_FACTOR, fail, find_arms,
    frame_period, interp_within, load_arm, local_poly_derivative, match, raw_arrays,
    resolve_run, state_arrays, warn,
)

DELAYS_S = np.round(np.arange(-0.10, 0.50 + 1e-9, 0.005), 3)   # candidate filter delays [s]
FILES = ("human_raw.csv", "human_state.csv")


def best_delay(t_kf, v_kf, t_ref, v_ref, max_gap):
    """The shift d minimising rms |v_kf(t + d) - v_ref(t)| over the reference samples.

    v_kf, v_ref: (N, k). Shifts that keep fewer than half of the zero-shift samples are not
    considered. Returns (d, rms at d), NaN if nothing overlaps.
    """
    n0 = np.isfinite(interp_within(t_kf, v_kf, t_ref, max_gap) - v_ref).all(axis=1).sum()
    best_d, best_rms = np.nan, np.inf
    for d in DELAYS_S:
        e = interp_within(t_kf, v_kf, t_ref + d, max_gap) - v_ref
        ok = np.isfinite(e).all(axis=1)
        if n0 == 0 or ok.sum() < 0.5 * n0:
            continue
        rms = float(np.sqrt(np.mean(np.sum(e[ok] ** 2, axis=1))))
        if rms < best_rms:
            best_d, best_rms = float(d), rms
    return best_d, (best_rms if np.isfinite(best_rms) else np.nan)


def analyse_arm(label, logs, args, out_dir):
    raw, st = raw_arrays(logs["human_raw.csv"]), state_arrays(logs["human_state.csv"])
    if len(raw["t"]) < 3 or len(st["t"]) < 3:
        warn(f"{label}: fewer than 3 frames logged, skipped.")
        return []
    period = frame_period(raw["t"])
    if not np.isfinite(period):
        warn(f"{label}: fewer than two distinct stamps, skipped.")
        return []
    max_gap = GAP_FACTOR * period
    t0 = min(raw["t"][0], st["t"][0])   # one origin for both curves

    ref_pos = raw["pos"].copy()
    if args.reference == "measured":
        k = match(st["t"], raw["t"], MATCH_FACTOR * period)
        measured = np.where((k >= 0)[:, None], st["measured"][np.maximum(k, 0)], False)
        if not measured.any():
            warn(f"{label}: no 'measured' flag set in human_state: reference from all raw samples.")
        else:
            ref_pos[~measured] = np.nan

    print(f"\n{label}: {len(raw['t'])} raw frames, period {1e3 * period:.1f} ms, "
          f"SG window {args.window_s:.2f} s, reference = {args.reference} samples")
    print(f"  {'keypoint':9s} {'n':>5s}  {'rms |v_KF - v_SG|':>17s}  {'bias x/y/z [m/s]':>22s}  "
          f"{'corr x/y/z':>15s}  {'delay':>7s}  {'rms at delay':>12s}")
    rows = []
    for i, kp in enumerate(KEYPOINTS):
        v_ref = local_poly_derivative(raw["t"], ref_pos[:, i], args.window_s)
        v_kf = st["vel"][:, i]
        v_kf_at = interp_within(st["t"], v_kf, raw["t"], max_gap)
        e = v_kf_at - v_ref
        ok = np.isfinite(e).all(axis=1)
        if ok.sum() < 10:
            print(f"  {kp:9s} {int(ok.sum()):5d}  (too few samples where both velocities exist)")
            continue
        delay, rms_at_delay = best_delay(st["t"], v_kf, raw["t"], v_ref, max_gap)
        rms3 = float(np.sqrt(np.mean(np.sum(e[ok] ** 2, axis=1))))
        stats = []
        for a, axis in enumerate(AXES):
            ea = e[ok, a]
            corr = (float(np.corrcoef(v_kf_at[ok, a], v_ref[ok, a])[0, 1])
                    if np.std(v_kf_at[ok, a]) > 0 and np.std(v_ref[ok, a]) > 0 else np.nan)
            d_a, r_a = best_delay(st["t"], v_kf[:, [a]], raw["t"], v_ref[:, [a]], max_gap)
            stats.append(dict(side=label, keypoint=kp, axis=axis, n=int(ok.sum()),
                              rms_err=float(np.sqrt(np.mean(ea**2))), bias=float(np.mean(ea)),
                              corr=corr, delay_ms=1e3 * d_a, rms_err_at_delay=r_a))
            plot_axis(label, kp, axis, raw["t"] - t0, v_ref[:, a], st["t"] - t0, v_kf[:, a],
                      stats[-1], args.window_s, out_dir)
        rows += stats
        rows.append(dict(side=label, keypoint=kp, axis="xyz", n=int(ok.sum()), rms_err=rms3,
                         bias=np.nan, corr=np.nan, delay_ms=1e3 * delay,
                         rms_err_at_delay=rms_at_delay))
        biases = " ".join(f"{s['bias']:+.3f}" for s in stats)
        corrs = " ".join(f"{s['corr']:.2f}" for s in stats)
        print(f"  {kp:9s} {int(ok.sum()):5d}  {rms3:13.3f} m/s  {biases:>22s}  {corrs:>15s}  "
              f"{1e3 * delay:5.0f} ms  {rms_at_delay:8.3f} m/s")
    return rows


def plot_axis(label, kp, axis, t_ref, v_ref, t_kf, v_kf, stats, window_s, out_dir):
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(t_ref, v_ref, label=f"Offline Reference (Savitzky-Golay, {window_s:.2f} s)",
            color="blue", linewidth=2, alpha=0.7)
    ax.plot(t_kf, v_kf, label="Real-Time Estimate (Kalman Filter)", color="red", linewidth=1.5)
    ax.set_title(f"Velocity Validation: {label.capitalize()} {kp.capitalize()} (Axis {axis.upper()})\n"
                 f"rms |v_KF - v_SG| = {stats['rms_err']:.3f} m/s, bias {stats['bias']:+.3f} m/s, "
                 f"best-fit delay {stats['delay_ms']:.0f} ms (n = {stats['n']})", fontsize=13)
    ax.set_xlabel("Elapsed Time [s]", fontsize=12)
    ax.set_ylabel(f"Velocity $v_{axis}$ [m/s]", fontsize=12)
    ax.legend(fontsize=11)
    ax.grid(True, linestyle="--", alpha=0.6)
    fig.savefig(out_dir / f"kf_validation_{label}_{kp}_{axis}.png", dpi=300, bbox_inches="tight")
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Validate Kalman filter velocities against Savitzky-Golay.")
    parser.add_argument("run_name", nargs="?", default=None,
                        help="experiment folder name; the latest run if omitted")
    parser.add_argument("--side", choices=["left", "right", "both"], default="both",
                        help="which arm to analyse (default: both)")
    parser.add_argument("--window-s", type=float, default=0.5,
                        help="Savitzky-Golay window [s] (default 0.5: 15 frames at 30 Hz)")
    parser.add_argument("--reference", choices=["measured", "all"], default="measured",
                        help="raw samples of the reference (default: those the filter accepted)")
    parser.add_argument("--log-dir", default=LOG_DIR)
    parser.add_argument("--out-dir", default=GRAPH_DIR)
    args = parser.parse_args(argv)
    if args.window_s <= 0.0:
        fail("--window-s must be positive.")

    run = resolve_run(args.log_dir, args.run_name)
    print(f"Analysing run: {run.name}")
    arms = find_arms(run, args.side, FILES)
    if not arms:
        fail(f"no {' + '.join(FILES)} pair for side '{args.side}' in '{run}'.")
    out_dir = Path(args.out_dir).expanduser() / run.name / "savitzky_golay"
    out_dir.mkdir(parents=True, exist_ok=True)

    plots = 0
    for label, paths in arms:
        logs = load_arm(paths)
        if logs is None:
            warn(f"{label}: logs unreadable or empty, skipped.")
            continue
        rows = analyse_arm(label, logs, args, out_dir)
        if rows:
            pd.DataFrame(rows).to_csv(out_dir / f"summary_{label}.csv", index=False)
            plots += sum(r["axis"] != "xyz" for r in rows)
    if plots:
        print(f"\nSaved {plots} plots and the summaries in: {out_dir}")
    else:
        print("No plot generated: check the logs of this run.")
    return 0 if plots else 1


if __name__ == "__main__":
    raise SystemExit(main())