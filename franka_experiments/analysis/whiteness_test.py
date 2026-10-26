#!/usr/bin/env python3
"""Innovation whiteness and NIS consistency of the human_tracker Kalman filter, per keypoint.

If the filter's model matches the data, the innovations of its accepted updates are a
zero-mean WHITE sequence (uncorrelated from one update to the next) and the normalised
innovation squared NIS = y^T S^-1 y is chi^2 with 3 degrees of freedom. On a run's logs:
- whiteness: sample autocorrelation of each innovation axis at lags 1..L, pairing updates
  by their stamps (dropped frames never shift a lag), with the 95% band +-1.96/sqrt(n_l)
  and the Box-Pierce portmanteau test Q = sum n_l rho_l^2 ~ chi^2_L (p < 0.05: not white);
- bias: mean innovation in units of its standard error, using the effective sample size
  of a correlated sequence, n_eff = n / (1 + 2 sum rho_l);
- consistency (logs with NIS): NIS against the chi^2_3 95% interval, its average against
  chi^2_3n / n (Bar-Shalom; it assumes independent NIS, so with correlated innovations
  the interval is narrower than it should be), its median against chi^2_3's (2.37) and
  the gate rate;
- convergence: trace(P), drawn only while the filter runs. It adds m^2 (position) and
  (m/s)^2 (velocity), so the velocity block dominates it.
Only frames with an accepted update enter the innovation statistics: a frame with no
update logs a zero innovation, a gated one the innovation it rejected (drawn apart).

Outputs, in <out-dir>/<run>/whiteness_test/:
  kf_covariance_trace_<side>_<keypoint>.png
  kf_whiteness_<side>_<keypoint>_<axis>.png   innovations | distribution | autocorrelation
  kf_nis_<side>_<keypoint>.png                 (logs with NIS only)
  summary_<side>.csv

Usage:
  python3 whiteness_test.py [RUN] [--side left|right|both] [--max-lag 20] [--gate 11.34]
                            [--log-dir DIR] [--out-dir DIR]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")   # files only: runs without a display
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.stats import chi2, kurtosis, norm

from analysis_utils import (
    AXES, GRAPH_DIR, KEYPOINTS, LOG_DIR, MATCH_FACTOR, acf_gapped, diag_arrays, effective_size,
    fail, find_arms, frame_period, load_arm, match, nis_interval, portmanteau, resolve_run,
    state_arrays, warn,
)

CHI2_3_MEDIAN = float(chi2.median(3))
MIN_UPDATES = 30


def update_masks(diag, st, period):
    """(accepted, rejected): per frame and keypoint, a real update and a gated measurement.

    Accepted = the filter counted the keypoint as measured (human_state) and logged a
    non-zero innovation: the first measurement after a reset initialises the filter with
    no innovation. Without human_state, the nis / gated columns decide (or, in the oldest
    logs, a non-zero innovation).
    """
    nonzero = np.isfinite(diag["inn"]).all(axis=-1) & ~(diag["inn"] == 0.0).all(axis=-1)
    if st is not None:
        k = match(st["t"], diag["t"], MATCH_FACTOR * period)
        measured = np.where((k >= 0)[:, None], st["measured"][np.maximum(k, 0)], False)
        accepted = measured & nonzero
    elif diag["gated"] is not None:
        accepted = nonzero & ~diag["gated"]
    else:
        accepted = nonzero
    rejected = diag["gated"] & nonzero if diag["gated"] is not None else nonzero & ~accepted
    return accepted, rejected & ~accepted


def analyse_arm(label, logs, args, out_dir):
    diag = diag_arrays(logs["kf_diagnostics.csv"])
    st = state_arrays(logs["human_state.csv"]) if "human_state.csv" in logs else None
    if len(diag["t"]) < 3:
        warn(f"{label}: fewer than 3 frames logged, skipped.")
        return [], 0
    period = frame_period(diag["t"])
    if not np.isfinite(period):
        warn(f"{label}: fewer than two distinct stamps, skipped.")
        return [], 0
    t0 = diag["t"][0] if st is None else min(diag["t"][0], st["t"][0])
    accepted, rejected = update_masks(diag, st, period)
    has_nis = diag["nis"] is not None

    print(f"\n{label}: {len(diag['t'])} frames, period {1e3 * period:.1f} ms, lags 1..{args.max_lag} "
          f"({1e3 * args.max_lag * period:.0f} ms)" + ("" if has_nis else ", no NIS in these logs"))
    print(f"  {'keypoint':9s} {'updates':>7s} {'gated':>6s} | {'ACF lag 1 x/y/z':>16s} | "
          f"{'white? p x/y/z':>20s} | {'bias z x/y/z':>17s} | {'NIS mean [95% band]':>22s} {'med/2.37':>8s}")
    rows, plots = [], 0
    for i, kp in enumerate(KEYPOINTS):
        plots += plot_trace(label, kp, diag["t"] - t0, diag["trace"][:, i], out_dir)
        acc, rej = accepted[:, i], rejected[:, i]
        n, n_rej = int(acc.sum()), int(rej.sum())
        if n < MIN_UPDATES:
            print(f"  {kp:9s} {n:7d} {n_rej:6d} | too few accepted updates for the tests")
            continue
        t_acc, y_acc = diag["t"][acc], diag["inn"][acc, i]
        nis_row = nis_stats(diag["nis"][:, i], acc, rej, args.gate) if has_nis else {}
        stats = []
        for a, axis in enumerate(AXES):
            y = y_acc[:, a]
            rho, pairs = acf_gapped(t_acc, y, args.max_lag, period)
            q, p, dof = portmanteau(rho, pairs)
            n_eff = effective_size(n, rho)
            std = float(np.std(y, ddof=1))
            stats.append(dict(
                side=label, keypoint=kp, axis=axis, n_updates=n, n_gated=n_rej,
                gate_rate=n_rej / max(n + n_rej, 1), mean=float(np.mean(y)), std=std,
                n_eff=n_eff, bias_z=float(np.mean(y) / (std / np.sqrt(n_eff))) if std > 0 else np.nan,
                rho1=rho[0], rho2=rho[1] if args.max_lag > 1 else np.nan, q=q, dof=dof, p_white=p,
                white=bool(np.isfinite(p) and p >= 0.05), excess_kurtosis=float(kurtosis(y)), **nis_row))
            plot_whiteness(label, kp, axis, t_acc - t0, y, diag["t"][rej] - t0,
                           diag["inn"][rej, i, a], rho, pairs, stats[-1], out_dir)
            plots += 1
        if has_nis:
            plots += plot_nis(label, kp, diag["t"] - t0, diag["nis"][:, i], acc, rej, nis_row,
                              args.gate, period, out_dir)
        rows += stats
        nis_txt = (f"{nis_row['nis_mean']:6.2f} [{nis_row['nis_lo']:.2f}, {nis_row['nis_hi']:.2f}]"
                   f" {nis_row['nis_median'] / CHI2_3_MEDIAN:8.2f}" if has_nis else f"{'-':>22s} {'-':>8s}")
        acf_txt = " ".join(f"{s['rho1']:5.2f}" for s in stats)
        p_txt = " ".join(_p_text(s["p_white"], short=True) for s in stats)
        z_txt = " ".join(f"{s['bias_z']:+5.1f}" for s in stats)
        print(f"  {kp:9s} {n:7d} {100 * n_rej / max(n + n_rej, 1):5.1f}% | {acf_txt:>16s} | "
              f"{p_txt:>20s} | {z_txt:>17s} | {nis_txt}")
    if rows:
        white = sum(r["white"] for r in rows)
        print(f"  -> innovations white (Box-Pierce p >= 0.05) on {white} of {len(rows)} axes; "
              f"|bias z| > 2 on {sum(abs(r['bias_z']) > 2 for r in rows)} of {len(rows)}")
    return rows, plots


def _p_text(p, short=False):
    if not np.isfinite(p):
        return "n/a"
    if p < 0.001:
        return "<.001" if short else "< 0.001"
    return f"{p:.3f}" if short else f"= {p:.3f}"


def nis_stats(nis, acc, rej, gate):
    """NIS of the accepted updates and of all measurements (gated ones included)."""
    acc_v = nis[acc & np.isfinite(nis)]
    all_v = nis[(acc | rej) & np.isfinite(nis)]
    if not acc_v.size:
        return dict(nis_mean=np.nan, nis_lo=np.nan, nis_hi=np.nan, nis_median=np.nan,
                    nis_mean_accepted=np.nan, above_gate=np.nan)
    lo, hi = nis_interval(3, all_v.size)
    return dict(nis_mean=float(np.mean(all_v)), nis_lo=lo, nis_hi=hi,
                nis_median=float(np.median(all_v)), nis_mean_accepted=float(np.mean(acc_v)),
                above_gate=float(np.mean(all_v > gate)) if gate > 0 else np.nan)


def plot_trace(label, kp, t, trace, out_dir):
    if not np.isfinite(trace).any():
        return 0
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.semilogy(t, trace, color="green", linewidth=2)
    ax.set_title(f"Uncertainty Convergence: Trace {label.capitalize()} {kp.capitalize()}", fontsize=14)
    ax.set_xlabel("Elapsed Time [s]", fontsize=12)
    ax.set_ylabel("trace(P)  [m$^2$ + (m/s)$^2$]", fontsize=12)
    ax.grid(True, which="both", linestyle="--", alpha=0.6)
    fig.savefig(out_dir / f"kf_covariance_trace_{label}_{kp}.png", dpi=300, bbox_inches="tight")
    plt.close(fig)
    return 1


def plot_whiteness(label, kp, axis, t, y, t_rej, y_rej, rho, pairs, s, out_dir):
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 5),
                                        gridspec_kw={"width_ratios": [3, 1, 1.4]})
    ax1.plot(t, y, color="purple", linewidth=1, alpha=0.8, label=f"Innovation ({s['n_updates']} updates)")
    if len(t_rej):
        ax1.plot(t_rej, y_rej, "x", color="red", markersize=4, alpha=0.7,
                 label=f"Rejected by the gate ({len(t_rej)})")
    ax1.axhline(0, color="black", linewidth=1.5)
    half_ci = 1.96 * s["std"] / np.sqrt(s["n_eff"])
    ax1.axhline(s["mean"], color="red", linestyle="--",
                label=f"Mean {s['mean']:+.4f} m (95% CI ±{half_ci:.4f}, n_eff = {s['n_eff']:.0f})")
    ax1.set_title(f"Innovations: {label.capitalize()} {kp.capitalize()} (Axis {axis.upper()})", fontsize=14)
    ax1.set_xlabel("Elapsed Time [s]", fontsize=12)
    ax1.set_ylabel("Innovation [m]", fontsize=12)
    ax1.legend(loc="upper right", fontsize=9)
    ax1.grid(True, linestyle="--", alpha=0.5)

    spread = 1.4826 * np.median(np.abs(y - np.median(y)))
    lim = 6 * (spread if spread > 0 else s["std"])
    bins = np.linspace(np.median(y) - lim, np.median(y) + lim, 61)
    ax2.hist(np.clip(y, bins[0], bins[-1]), bins=bins, color="purple", alpha=0.7, density=True)
    xs = np.linspace(bins[0], bins[-1], 300)
    ax2.plot(xs, norm.pdf(xs, s["mean"], s["std"]), color="black", linewidth=1.2, label="Gaussian, same mean/std")
    ax2.set_title(f"Distribution (excess kurtosis {s['excess_kurtosis']:.1f})", fontsize=12)
    ax2.set_xlabel("Innovation [m]", fontsize=12)
    ax2.legend(fontsize=8)
    ax2.grid(True, linestyle="--", alpha=0.5)

    lags = np.arange(1, len(rho) + 1)
    band = np.where(pairs > 0, 1.96 / np.sqrt(np.maximum(pairs, 1)), np.nan)
    ax3.fill_between(lags, -band, band, color="gray", alpha=0.3, label="95% band, white noise")
    ax3.vlines(lags, 0, rho, color="purple", linewidth=2)
    ax3.plot(lags, rho, "o", color="purple", markersize=4)
    ax3.axhline(0, color="black", linewidth=1)
    verdict = "white" if s["white"] else "NOT white"
    p_txt = _p_text(s["p_white"])
    ax3.set_title(f"Autocorrelation: {verdict}\nBox-Pierce Q = {s['q']:.0f} (L = {s['dof']}), p {p_txt}",
                  fontsize=12)
    ax3.set_xlabel("Lag [frames]", fontsize=12)
    ax3.set_ylim(min(-0.3, np.nanmin(rho) - 0.05), 1.0)
    ax3.legend(fontsize=8, loc="upper right")
    ax3.grid(True, linestyle="--", alpha=0.5)

    fig.tight_layout()
    fig.savefig(out_dir / f"kf_whiteness_{label}_{kp}_{axis}.png", dpi=300, bbox_inches="tight")
    plt.close(fig)


def plot_nis(label, kp, t, nis, acc, rej, s, gate, period, out_dir):
    acc_ok, rej_ok = acc & np.isfinite(nis) & (nis > 0), rej & np.isfinite(nis) & (nis > 0)
    if not acc_ok.any():
        return 0
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5), gridspec_kw={"width_ratios": [3, 1]})
    lo1, hi1 = nis_interval(3, 1)
    ax1.axhspan(lo1, hi1, color="green", alpha=0.12, label=f"chi$^2_3$ 95% interval [{lo1:.2f}, {hi1:.2f}]")
    ax1.semilogy(t[acc_ok], nis[acc_ok], ".", color="purple", markersize=2, alpha=0.6, label="NIS, accepted")
    if rej_ok.any():
        ax1.semilogy(t[rej_ok], nis[rej_ok], "x", color="red", markersize=4, label="NIS, gated")
    window = max(int(round(1.0 / period)), 1)   # ~1 s moving average over the measurements
    meas = acc_ok | rej_ok
    avg = pd.Series(nis[meas]).rolling(window, min_periods=max(window // 2, 1)).mean().to_numpy()
    ax1.semilogy(t[meas], avg, color="black", linewidth=1.5, label="Moving average (~1 s)")
    ax1.axhline(3.0, color="green", linestyle="--", linewidth=1, label="E[NIS] = 3")
    if gate > 0:
        ax1.axhline(gate, color="red", linestyle=":", linewidth=1.2, label=f"Gate {gate:g}")
    ax1.set_title(f"NIS: {label.capitalize()} {kp.capitalize()}   mean {s['nis_mean']:.2f} "
                  f"(consistent: [{s['nis_lo']:.2f}, {s['nis_hi']:.2f}]), "
                  f"median/2.37 = {s['nis_median'] / CHI2_3_MEDIAN:.2f}", fontsize=13)
    ax1.set_xlabel("Elapsed Time [s]", fontsize=12)
    ax1.set_ylabel("NIS", fontsize=12)
    ax1.legend(loc="lower right", fontsize=8)
    ax1.grid(True, which="both", linestyle="--", alpha=0.5)

    top = max(15.0, gate * 1.2)
    bins = np.linspace(0, top, 61)
    ax2.hist(np.clip(nis[meas], 0, top), bins=bins, density=True, color="purple", alpha=0.7)
    xs = np.linspace(1e-3, top, 300)
    ax2.plot(xs, chi2.pdf(xs, 3), color="black", linewidth=1.5, label="chi$^2_3$")
    ax2.set_title(f"Distribution ({100 * s['above_gate']:.1f}% above the gate)", fontsize=12)
    ax2.set_xlabel("NIS", fontsize=12)
    ax2.legend(fontsize=9)
    ax2.grid(True, linestyle="--", alpha=0.5)
    fig.tight_layout()
    fig.savefig(out_dir / f"kf_nis_{label}_{kp}.png", dpi=300, bbox_inches="tight")
    plt.close(fig)
    return 1


def main(argv=None):
    parser = argparse.ArgumentParser(description="Whiteness and consistency tests of the Kalman filter innovations.")
    parser.add_argument("run_name", nargs="?", default=None,
                        help="experiment folder name; the latest run if omitted")
    parser.add_argument("--side", choices=["left", "right", "both"], default="both",
                        help="which arm to analyse (default: both)")
    parser.add_argument("--max-lag", type=int, default=20, help="autocorrelation lags [frames]")
    parser.add_argument("--gate", type=float, default=11.34,
                        help="the filter's Mahalanobis gate (kf_mahalanobis_threshold)")
    parser.add_argument("--log-dir", default=LOG_DIR)
    parser.add_argument("--out-dir", default=GRAPH_DIR)
    args = parser.parse_args(argv)
    if args.max_lag < 1:
        fail("--max-lag must be at least 1.")

    run = resolve_run(args.log_dir, args.run_name)
    print(f"Analysing run: {run.name}")
    arms = find_arms(run, args.side, ("kf_diagnostics.csv",))
    if not arms:
        fail(f"no kf_diagnostics.csv for side '{args.side}' in '{run}' "
             f"(runs before the filter diagnostics were logged have none).")
    out_dir = Path(args.out_dir).expanduser() / run.name / "whiteness_test"
    out_dir.mkdir(parents=True, exist_ok=True)

    total = 0
    for label, paths in arms:
        state = paths["kf_diagnostics.csv"].with_name(
            paths["kf_diagnostics.csv"].name.replace("kf_diagnostics", "human_state"))
        if state.is_file():
            paths = dict(paths, **{"human_state.csv": state})
        else:
            warn(f"{label}: no human_state log: accepted updates told apart by the nis/gated columns.")
        logs = load_arm(paths)
        if logs is None:
            warn(f"{label}: logs unreadable or empty, skipped.")
            continue
        rows, plots = analyse_arm(label, logs, args, out_dir)
        total += plots
        if rows:
            pd.DataFrame(rows).to_csv(out_dir / f"summary_{label}.csv", index=False)
    if total:
        print(f"\nSaved {total} plots and the summaries in: {out_dir}")
    else:
        print("No plot generated: check the logs of this run.")
    return 0 if total else 1


if __name__ == "__main__":
    raise SystemExit(main())