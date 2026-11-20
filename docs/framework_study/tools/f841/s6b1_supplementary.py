#!/usr/bin/env python3
"""S6B.1 — supplementary checks on the reconstruction (numpy only; host or container; read-only).

Usage:
  python3 -I s6b1_supplementary.py <cell_dir> <s6b1_reconstruction.npz> <out.json>

Computes:
  * every truth/reconstruction disagreement on matched samples, with its instant and cause hint;
  * the sample spacing of the measured joint stream and a Lipschitz bound on how far the clearance
    can dip between two consecutive samples (static obstacle: |d gap / dt| <= |v_cp|);
  * ground_truth_node's own obstacle-pose pairing age t - t_pose (truth.npz), overall and right
    after each truth hole;
  * the reconstruction-only spans (bag samples the truth does not have).
"""
import json
import os
import sys

import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import h3_moving as H3  # noqa: E402   (F-8-41 harness correction; reduces exactly to S6B.1 for a static ball)
CD, NPZ, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
T = np.load(f'{CD}/truth.npz')
Z = np.load(NPZ)
man = json.load(open(f'{CD}/run_manifest_sim.json'))
T0, T1 = float(man['episode']['t_start_sim']), float(man['episode']['t_end_sim'])
t, G, gm, mi, cp = Z['t'], Z['gap'], Z['gmin'], Z['truth_index'], Z['cp']
V_OBS = H3.scenario_speed_bound(man['scenario_resolved'])      # F-8-41: 0.0 for a static scene
R = {'v_obs_max_mps': V_OBS}

b = (mi >= 0) & np.isfinite(gm)
tg = T['gap'][mi[b], :, 0].min(axis=1)
d = np.abs(gm[b] - tg)
bad = np.where(d > 0)[0]
R['disagreements'] = [dict(t=float(t[b][i]), recon=float(gm[b][i]), truth=float(tg[i]),
                           truth_obstacle_z=float(T['p'][mi[b][i], 0, 2]),
                           truth_t_pose=float(T['t_pose'][mi[b][i]])) for i in bad]
eb = b & (t >= T0) & (t <= T1)
dd = np.abs(gm[eb] - T['gap'][mi[eb], :, 0].min(axis=1))
R['episode_min_gap_abs_diff_excluding_placement_instant'] = dict(
    n=int(eb.sum()), max=float(np.sort(dd)[-2]) if dd.size > 1 else None, n_nonzero=int((dd > 0).sum()))
# F-8-41 (C3): with a moving ball, recon (causal bag pose) and truth (its own pose) may use poses taken
# |dt_pose| apart; the min gap may then differ by up to v_obs_max * |dt_pose|. Static: excess == |d|.
dtp = np.abs((t[eb] - Z['pose_age'][eb]) - T['t_pose'][mi[eb]])
ex = H3.pose_pairing_excess(dd, dtp, V_OBS)
R['episode_min_gap_excess_over_pose_bound'] = dict(
    n=int(eb.sum()), max=float(np.sort(ex)[-2]) if ex.size > 1 else None, n_positive=int((ex > 0).sum()),
    dt_pose_ms_max=float(1e3 * dtp.max()) if dtp.size else None, v_obs_max_mps=V_OBS)

e = (t >= T0) & (t <= T1) & np.isfinite(gm)
dt = np.diff(t[e])
v = np.linalg.norm(np.diff(cp[e], axis=0), axis=2) / dt[:, None]
vmax = float(np.nanmax(v))
# F-8-41 (C2): relative speed (cp + ball) between samples, and the causal pose's staleness per sample
lb, dip = H3.continuous_lower_bound(gm[e], Z['pose_age'][e], float(dt.max()), vmax, V_OBS)
R['sampling'] = dict(n=int(e.sum()), dt_ms_max=float(1e3 * dt.max()), n_dt_2ms=int((dt > 0.0015).sum()),
                     t_of_2ms_intervals=[float(x) for x in t[e][:-1][dt > 0.0015]],
                     cp_speed_max_mps=vmax, v_obs_max_mps=V_OBS,
                     pose_age_ms_max=float(1e3 * np.nanmax(Z['pose_age'][e])),
                     max_intersample_dip_bound_m=dip,
                     min_sampled_gap=float(np.nanmin(gm[e])),
                     continuous_lower_bound_m=lb)

tt, tp = T['t'], T['t_pose']
te = (tt >= T0) & (tt <= T1)
age = 1e3 * (tt[te] - tp[te])
R['truth_pose_pairing_age_ms'] = dict(
    p50=float(np.percentile(age, 50)), p99=float(np.percentile(age, 99)), max=float(age.max()),
    min=float(age.min()), n=int(te.sum()), n_negative=int((age < -0.5).sum()), n_positive=int((age > 0.5).sum()))
holes_end = tt[1:][np.diff(tt) > 0.0021]
after = []
for h in holes_end[(holes_end >= T0) & (holes_end <= T1)]:
    i = int(np.searchsorted(tt, h))
    j = i
    while j < tt.size and tt[j] - tp[j] < -0.0005:
        j += 1
    after.append(dict(hole_end=float(h), age_ms_first=float(1e3 * (tt[i] - tp[i])),
                      n_negative_after=int(j - i), lasts_ms=float(1e3 * (tt[min(j, tt.size - 1)] - tt[i]))))
R['truth_pose_pairing_after_holes'] = after

ro = e & (mi < 0)
idx = np.where(ro)[0]
spans, s0 = [], idx[0] if idx.size else None
for a, c in zip(idx[:-1], idx[1:]):
    if c != a + 1:
        spans.append([float(t[s0]), float(t[a]), int(a - s0 + 1)])
        s0 = c
if idx.size:
    spans.append([float(t[s0]), float(t[idx[-1]]), int(idx[-1] - s0 + 1)])
R['reconstruction_only_spans'] = [dict(first=a, last=c, n=n,
                                       recon_min_gap=float(np.nanmin(gm[(t >= a) & (t <= c)])))
                                  for a, c, n in spans]
json.dump(R, open(OUT, 'w'), indent=1)
print(json.dumps(R, indent=1))
