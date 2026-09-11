#!/usr/bin/env python3
"""Timing figure for the Safe-RL deployment node (rl_policy_commander).

Turns an ``rl_policy_run_*.csv`` into the determinism evidence the sim-to-real
roadmap asks for: the control-loop period distribution (jitter) and the ONNX
inference latency, both against the nominal period.

    python3 scripts/plot_rl_timing.py                       # newest CSV found
    python3 scripts/plot_rl_timing.py --csv <file> --out figs/timing.png
    python3 scripts/plot_rl_timing.py --no-plot             # stats only

Only the *commanding* ticks are used for the statistics — a gated tick (warm-up,
stale input) publishes zeros without running inference, so including them would
report an inference time that never happened.
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
from pathlib import Path

import numpy as np


def _default_csv() -> str:
    """Newest ``rl_policy_run_*.csv`` from the repo log dir, then ``~``."""
    roots = [Path(os.path.realpath(__file__)).parents[1] / 'franka_logs',
             Path.home() / 'franka_logs']
    hits = []
    for r in roots:
        hits += glob.glob(str(r / 'rl_policy_run_*.csv'))
    return max(hits, key=os.path.getmtime) if hits else ''


def load(path: str):
    with open(path) as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise SystemExit(f'{path} has no data rows')
    t    = np.array([float(r['time']) for r in rows])
    tick = np.array([float(r['tick_ms']) for r in rows])
    inf  = np.array([float(r['infer_ms']) for r in rows])
    cmd  = np.array([[float(r[f'qddot_nom_{i}']) for i in range(1, 8)]
                     for r in rows])
    commanding = np.any(cmd != 0.0, axis=1)
    return t, tick, inf, commanding


def stats(name: str, x: np.ndarray, unit: str = 'ms') -> str:
    return (f'  {name:<18} mean {np.mean(x):7.3f} {unit}   '
            f'std {np.std(x):6.3f}   p50 {np.percentile(x, 50):7.3f}   '
            f'p99 {np.percentile(x, 99):7.3f}   max {np.max(x):7.3f}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default=_default_csv())
    ap.add_argument('--out', default=None, help='output image (default: alongside CSV)')
    ap.add_argument('--nominal-ms', type=float, default=10.0,
                    help='nominal control period [ms] (default 100 Hz)')
    ap.add_argument('--no-plot', action='store_true')
    args = ap.parse_args()

    if not args.csv or not os.path.isfile(args.csv):
        raise SystemExit('No rl_policy_run_*.csv found — pass --csv explicitly.')

    t, tick, inf, commanding = load(args.csv)
    # Drop the first sample: tick_ms is 0 until a previous tick exists.
    keep = commanding.copy()
    keep[0] = False
    if not keep.any():
        raise SystemExit(f'{args.csv} contains no commanding ticks')
    tick_c, inf_c = tick[keep], inf[keep]

    print(f'{args.csv}\n  rows {len(t)}  commanding {keep.sum()}  '
          f'duration {t[-1] - t[0]:.1f} s  nominal {args.nominal_ms:.2f} ms')
    print(stats('loop period', tick_c))
    print(stats('|period − nominal|', np.abs(tick_c - args.nominal_ms)))
    print(stats('ONNX inference', inf_c))
    over = float(np.mean(tick_c > 1.5 * args.nominal_ms)) * 100.0
    print(f'  ticks > 1.5x nominal: {over:.2f} %')

    if args.no_plot:
        return

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    out = args.out or os.path.splitext(args.csv)[0] + '_timing.png'
    fig, ax = plt.subplots(3, 1, figsize=(10, 9))

    ax[0].plot(t[keep], tick_c, lw=0.6)
    ax[0].axhline(args.nominal_ms, color='k', ls='--', lw=1,
                  label=f'nominal {args.nominal_ms:.1f} ms')
    ax[0].set_ylabel('loop period [ms]')
    ax[0].set_title('rl_policy_commander — control-loop period')
    ax[0].legend(loc='upper right')
    ax[0].grid(alpha=0.3)

    ax[1].hist(tick_c - args.nominal_ms, bins=80)
    ax[1].set_xlabel('period − nominal [ms]')
    ax[1].set_ylabel('count')
    ax[1].set_title('jitter distribution')
    ax[1].grid(alpha=0.3)

    ax[2].plot(t[keep], inf_c, lw=0.6)
    ax[2].set_xlabel('time [s]')
    ax[2].set_ylabel('inference [ms]')
    ax[2].set_title(f'ONNX inference latency (p99 = '
                    f'{np.percentile(inf_c, 99):.3f} ms)')
    ax[2].grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f'  figure → {out}')


if __name__ == '__main__':
    main()
