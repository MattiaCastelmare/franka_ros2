#!/usr/bin/env python3
"""R1 — attribute the per-tick output differences between executor modes to the constraint
snapshot each tick used (scheduling), not to the control equations.

  python3 -I r1_snapshot_attribution.py <seq_reference.npz> <run>_raw.npz [...]

For each tick in the episode (sim [10.55, 40.56]): the output deviation from the sequential,
schedule-ideal reference in the same 10 ms slot, and whether the tick used a constraint snapshot
built IN ITS OWN 20 ms rebuild period ('fresh', as the reference does: the rebuild timer is created
before the tick timer and the reference runs it first) or one built in the PREVIOUS period ('stale').
Freshness is decided in node time: the snapshot's build-end node time >= the tick slot's 20 ms grid
point. Prints, per run, the share of stale ticks and the deviation p50 / p95 for fresh vs stale.
"""
import sys

import numpy as np

T0, T1 = 10.55, 40.56
ref = np.load(sys.argv[1])['pub']
R = {}
for row in ref:
    if T0 <= row[0] <= T1:
        R.setdefault(int(np.floor((row[0] + 5e-4) / 0.01)), row[1:8])
for f in sys.argv[2:]:
    Z = np.load(f)
    ti, pub = Z['tick_in'], Z['pub']
    # each command belongs to the latest tick that STARTED before it was published (wall time);
    # pairing by index would shift after any tick that returns without publishing
    idx = np.searchsorted(ti[:, 0], pub[:, 0], side='right') - 1
    keep = idx >= 0
    ti, pub = ti[idx[keep]], pub[keep]
    m = (ti[:, 1] >= T0) & (ti[:, 1] <= T1)
    ti, pub = ti[m], pub[m]
    print(f"  ({f.split('/')[-1]}: {len(Z['tick_in'])} ticks, {len(Z['pub'])} commands)")
    slot = np.floor((ti[:, 1] + 5e-4) / 0.01).astype(int)
    grid20 = np.floor((ti[:, 1] + 5e-4) / 0.02) * 0.02
    fresh = ti[:, 8] >= grid20 - 5e-4
    dev = np.array([np.max(np.abs(pub[i, 2:9] - R[s])) if s in R else np.nan for i, s in enumerate(slot)])
    ok = np.isfinite(dev)

    def q(mask, p):
        x = dev[mask & ok]
        return float(np.percentile(x, p)) if x.size else float('nan')
    print(f"{f.split('/')[-1]:28s} ticks {int(ok.sum())}  stale {100 * (~fresh[ok]).mean():5.1f} %  "
          f"dev fresh p50/p95 {q(fresh, 50):.4f}/{q(fresh, 95):.4f}  stale p50/p95 {q(~fresh, 50):.4f}/{q(~fresh, 95):.4f}  "
          f"share of dev>0.05 on stale ticks {100 * np.mean(~fresh[ok & (dev > 0.05)]) if (ok & (dev > 0.05)).any() else float('nan'):.1f} %")
