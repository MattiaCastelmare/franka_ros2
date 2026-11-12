#!/usr/bin/env python3
"""QP-performance study — are late QP ticks the ones that overlap a constraint rebuild?

  python3 -I qpperf_late_tick_overlap.py <run>_raw.npz [...]

Input: the per-call rows written by `qpperf_executor_replay.py` (sim t, wall start, wall duration,
thread CPU, TID) for `qp` and `rebuild`. Episode window sim [10.55, 40.56] s (S6B manifest).
For each QP tick:
  * late    = the sim-time gap to the previous tick is >= 15 ms (1.5 periods);
  * overlap = some rebuild ran (in wall time) between the end of the previous tick and the end of
    this one;
  * wait    = wall duration - thread CPU (time runnable-but-not-running or blocked: GIL, core).
Prints P(late | overlap), P(late | no overlap) and the median tick wait with and without a rebuild
running DURING the tick. Read-only; numpy only.
"""
import sys

import numpy as np

T0, T1 = 10.55, 40.56
for f in sys.argv[1:]:
    Z = np.load(f)
    q, b = Z['qp'], Z['rebuild']
    q = q[(q[:, 0] >= T0) & (q[:, 0] <= T1)]
    b = b[(b[:, 0] >= T0 - 0.1) & (b[:, 0] <= T1 + 0.1)]
    gap = np.r_[np.nan, np.diff(q[:, 0])]
    end = q[:, 1] + q[:, 2]
    prev_end = np.r_[np.nan, end[:-1]]
    bs, be = b[:, 1], b[:, 1] + b[:, 2]
    ov = np.array([np.isfinite(s) and bool(np.any((bs < e) & (be > s))) for s, e in zip(prev_end, end)])
    during = np.array([bool(np.any((bs < e) & (be > s))) for s, e in zip(q[:, 1], end)])
    late = gap >= 0.015
    wait = 1e3 * (q[:, 2] - q[:, 3])

    def p(mask):
        return float(late[mask].mean()) if mask.any() else float('nan')

    def med(mask):
        return float(np.median(wait[mask])) if mask.any() else float('nan')

    print(f"{f.split('/')[-1]}: ticks {len(q)}, late {int(late.sum())}; "
          f"P(late|rebuild in interval) {p(ov):.3f} (n={int(ov.sum())}), "
          f"P(late|none) {p(~ov):.3f} (n={int((~ov).sum())}); "
          f"tick wait p50 {med(during):.2f} ms with a rebuild during the tick (n={int(during.sum())}) "
          f"vs {med(~during):.2f} ms without; "
          f"rebuild wait p50 {1e3 * np.median(b[:, 2] - b[:, 3]):.2f} ms, CPU p50 {1e3 * np.median(b[:, 3]):.2f} ms")
