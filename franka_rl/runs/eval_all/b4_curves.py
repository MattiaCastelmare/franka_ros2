"""b4 training curves from the stdout logs: rollout success, collision rate, mean d_min, CBF-active fraction per 250k block."""
import re, sys, numpy as np
runs = sys.argv[1:]
keys = ['success_rate', 'collision_rate', 'mean_surface_dist', 'ep_rew_mean']
for r in runs:
    rows, cur = [], {}
    for line in open(f'runs/sac_{r}_train.log', errors='ignore'):
        m = re.match(r'\|\s+(\w+)\s+\|\s+([-\d.e+]+)\s+\|', line)
        if not m: continue
        k, v = m.group(1), float(m.group(2))
        if k == 'total_timesteps': cur['t'] = v
        if k in keys: cur[k] = v
        if k == 'total_timesteps' and len(cur) > 1: rows.append(dict(cur))
    t0 = rows[0]['t']
    out = []
    for lo in np.arange(0, 1.5e6, 250e3):
        blk = [x for x in rows if lo <= x['t'] - t0 < lo + 250e3]
        f = lambda k: np.nanmean([x.get(k, np.nan) for x in blk]) if blk else np.nan
        out.append(f"{f('success_rate'):.2f}/{100*f('collision_rate'):.2f}%/{100*f('mean_surface_dist'):.0f}")
    print(f'{r:14s} ' + '  '.join(out))
