#!/usr/bin/env python3
"""One figure per ball pass from a LIVE recording: q̈_nom, q̈_safe, realised accel, q̇.

    python3 scripts/pass_plot.py <live bag> <replay bag of it (for pass timing)> <truth.npz> <out.png>
"""
import os, sys
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ball_throw_eval import read_bag, _stamp, passes, DIST, QDD_SAFE, QDD_NOM
from smoothness_report import load

live, rep, truth, out = sys.argv[1:5]
T = np.load(truth); tt, ok, P = T['t'], T['ok'], T['p']
rows = []
for tp, recv, m in read_bag(rep, {DIST}):
    rows.append((_stamp(m), np.array([[l.closest_point_robot.x, l.closest_point_robot.y, l.closest_point_robot.z]
                                      for l in m.links if l.valid])))
ps = passes(tt, ok, P, np.array([r[0] for r in rows]), [r[1] for r in rows], 0.6, 1.5)
safe, nom, js, *_ = load(live)
ts = np.array([s[0] for s in safe]); Qs = np.array([s[1] for s in safe])
tn = np.array([s[0] for s in nom]); Qn = np.array([s[1] for s in nom])
tj = np.array([j[0] for j in js]); V = np.array([j[2] for j in js])
g = np.arange(tj[0], tj[-1], 0.01); Vg = np.array([np.interp(g, tj, V[:, i]) for i in range(7)]).T
A = np.gradient(Vg, 0.01, axis=0); k = 5; A = np.stack([np.convolve(A[:, i], np.ones(k) / k, 'same') for i in range(7)], 1)
n = len(ps); fig, ax = plt.subplots(3, n, figsize=(3.2 * n, 7), sharex=True, squeeze=False)
for c, p in enumerate(ps):
    t0 = p['tc']
    for row, (t, Y, ttl) in enumerate([(ts, Qs, 'q̈_safe'), (g, A, 'realised accel'), (g, Vg, 'q̇')]):
        m = (t > t0 - 0.8) & (t < t0 + 0.6)
        for j in range(7): ax[row, c].plot(t[m] - t0, Y[m, j], lw=1, label=f'j{j+1}')
        if row == 0:
            mm = (tn > t0 - 0.8) & (tn < t0 + 0.6)
            ax[0, c].set_title(f'pass {c+1} v={p["v"]:.1f} m/s d={p["dmin"]*100:.0f}cm', fontsize=8)
        ax[row, c].axvline(0, color='k', lw=.5); ax[row, c].set_ylabel(ttl, fontsize=8)
ax[0, 0].legend(fontsize=6, ncol=2)
plt.tight_layout(); plt.savefig(out, dpi=80); print('wrote', out)
