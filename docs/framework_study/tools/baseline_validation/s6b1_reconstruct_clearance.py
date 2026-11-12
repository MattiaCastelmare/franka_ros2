#!/usr/bin/env python3
"""S6B.1 — independent reconstruction of the robot–obstacle sphere clearance from the ROS bag.

Read-only on the recording. Runs inside the `franka_ros2` container (needs rosbag2_py, pinocchio and
the INSTALLED franka_experiments / franka_simulation_experiments, which S4/S6B verified equal to
the source at f708991).

Usage:
  python3 -B s6b1_reconstruct_clearance.py <cell_dir> <out_dir>

Method (production geometry, no re-implementation):
  * measured joints: /NS_1/franka/joint_states from the cell bag, mapped BY NAME to fr3_joint1..7,
    time = header.stamp (the same time base ground_truth_node stores in truth.npz);
  * kinematics + control points: simexp.truth_geometry.TruthGeometry on the run's fr3_rsp.urdf with
    the installed fr3_complete.yaml (exactly ground_truth_node's inputs; the node's launch params
    are in the S6B evidence `container_tmp/tmp__launch_params_gloxv0uu`);
  * obstacle: /sim/truth/obstacle_poses from the bag, causal (latest header stamp <= t), radius
    from the scenario (obstacles_json); compared with the scenario p0 and with truth.npz `p`;
  * gap: simexp.truth_geometry.surface_gaps (= ||cp - c|| - r_obs - r_cp, signed).
No interpolation: a gap is computed only at instants where a measured joint sample exists.

Outputs (out_dir): s6b1_reconstruction.json (all numbers quoted in S6B1_CLEARANCE_AUDIT.md) and
s6b1_reconstruction.npz (t, q, per-CP gaps, min gap, CP labels, match index to truth).
"""
import json
import os
import sys

import numpy as np

sys.dont_write_bytecode = True
CD, OUT = sys.argv[1], sys.argv[2]
os.makedirs(OUT, exist_ok=True)

import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

sys.path.insert(0, '/ros2_ws/install/franka_simulation_experiments/lib/franka_simulation_experiments')
from simexp.truth_geometry import (TruthGeometry, cp_label, load_perception_config,  # noqa: E402
                                   surface_gaps)
import simexp.truth_geometry as tgmod  # noqa: E402

ARM = [f'fr3_joint{i}' for i in range(1, 8)]
CFG = '/ros2_ws/install/franka_experiments/share/franka_experiments/config/fr3_complete.yaml'
D_SAFE = 0.10
R = {'cell_dir': CD, 'truth_geometry_module': tgmod.__file__, 'perception_config': CFG}


def pct(x, ps=(50, 95, 99)):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if not x.size:
        return None
    d = {f'p{p}': float(np.percentile(x, p)) for p in ps}
    d.update(n=int(x.size), min=float(x.min()), max=float(x.max()), mean=float(x.mean()))
    return d


def hs(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


# ------------------------------------------------------------------ inputs
man = json.load(open(os.path.join(CD, 'run_manifest_sim.json')))
T0, T1 = float(man['episode']['t_start_sim']), float(man['episode']['t_end_sim'])
R['episode_sim'] = [T0, T1]
TZ = np.load(os.path.join(CD, 'truth.npz'))
tt, tq, tcp, tgap, tp = TZ['t'], TZ['q'], TZ['cp'], TZ['gap'], TZ['p']
labels = [str(x) for x in TZ['labels']]
r_cp, r_obs = np.asarray(TZ['cp_radius'], float), np.asarray(TZ['obstacle_radius'], float)

rd = rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=os.path.join(CD, 'bag'), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in rd.get_all_topics_and_types()}
R['bag_topics_joint_like'] = sorted(k for k in types if 'joint' in k)
js_t, js_rx, js_q, orders = [], [], [], {}
op_t, op_rx, op_p = [], [], []
while rd.has_next():
    topic, raw, t_ns = rd.read_next()
    if topic == '/NS_1/franka/joint_states':
        m = deserialize_message(raw, get_message(types[topic]))
        d = {n: i for i, n in enumerate(m.name)}
        if not all(n in d for n in ARM):
            continue
        orders[tuple(m.name)] = orders.get(tuple(m.name), 0) + 1
        js_t.append(hs(m)); js_rx.append(t_ns * 1e-9)
        js_q.append([m.position[d[n]] for n in ARM])
    elif topic == '/sim/truth/obstacle_poses':
        m = deserialize_message(raw, get_message(types[topic]))
        op_t.append(hs(m)); op_rx.append(t_ns * 1e-9)
        op_p.append([[p.position.x, p.position.y, p.position.z] for p in m.poses])
js_t, js_rx, js_q = np.array(js_t), np.array(js_rx), np.array(js_q)
op_t, op_rx, op_p = np.array(op_t), np.array(op_rx), np.array(op_p, float)
R['joint_name_orders'] = {' '.join(k): v for k, v in orders.items()}

# ------------------------------------------------------------------ joint-state stream checks
o = np.argsort(js_t, kind='stable')
R['joint_states'] = dict(
    n_total=int(js_t.size), header_monotonic=bool(np.all(np.diff(js_t) >= 0)),
    n_duplicate_stamps=int(np.sum(np.diff(js_t[o]) == 0)),
    rx_minus_header_ms=pct(1e3 * (js_rx - js_t)))
e = (js_t >= T0) & (js_t <= T1)
dj = np.diff(js_t[e])
R['joint_states'].update(
    n_episode=int(e.sum()), dt_ms=pct(1e3 * dj), max_dt_ms=float(1e3 * dj.max()),
    n_dt_gt_2ms=int((dj > 0.0021).sum()), expected_at_1khz=int(round((T1 - T0) * 1000)) + 1)

# ------------------------------------------------------------------ obstacle pose
oe = op_t >= T0 - 0.01
R['obstacle'] = dict(
    n_msgs=int(op_t.size), n_poses_per_msg=sorted({int(len(x)) for x in op_p}) if op_p.ndim == 3 else None,
    scenario_p0=[0.55, -0.25, 0.449], scenario_radius=0.05, truth_radius=r_obs.tolist())
placed = np.where(op_p[:, 0, 2] > -10.0)[0]
R['obstacle']['first_placed_stamp'] = float(op_t[placed[0]])
pp = op_p[placed, 0, :]
R['obstacle']['placed_mean'] = pp.mean(0).tolist()
R['obstacle']['placed_max_dev_from_p0_m'] = float(np.max(np.linalg.norm(pp - np.array([0.55, -0.25, 0.449]), axis=1)))
ope = (op_t >= T0) & (op_t <= T1)
R['obstacle']['dt_ms_episode'] = pct(1e3 * np.diff(op_t[ope]))
R['obstacle']['max_dt_ms_episode'] = float(1e3 * np.diff(op_t[ope]).max())
te = (tt >= T0) & (tt <= T1)
R['obstacle']['truth_p_max_dev_from_p0_m'] = float(np.nanmax(np.linalg.norm(tp[te, 0, :] - np.array([0.55, -0.25, 0.449]), axis=1)))

# ------------------------------------------------------------------ geometry
pc = load_perception_config(CFG)
geo = TruthGeometry(open(os.path.join(CD, 'fr3_rsp.urdf')).read(), pc)
radii = r_obs


def recon(q):
    cps = geo.control_points(q)
    return cps


cps0 = recon(js_q[0])
lab0 = [cp_label(c) for c in cps0]
R['geometry'] = dict(labels_reconstructed=lab0, labels_truth=labels, labels_equal=lab0 == labels,
                     cp_radius_reconstructed=[float(c['radius']) for c in cps0],
                     cp_radius_truth=r_cp.tolist())

# (A) method check: FK of truth.npz's OWN q must reproduce truth.npz's cp and gap exactly
idxA = np.where(te)[0]
errA_cp, errA_gap = 0.0, 0.0
for i in idxA:
    cps = recon(tq[i])
    P = np.array([c['point'] for c in cps])
    errA_cp = max(errA_cp, float(np.abs(P - tcp[i]).max()))
    g = surface_gaps(cps, tp[i], radii)
    errA_gap = max(errA_gap, float(np.abs(g - tgap[i]).max()))
R['check_A_same_q'] = dict(n=int(idxA.size), max_abs_cp_err_m=errA_cp, max_abs_gap_err_m=errA_gap)

# (B) independent reconstruction on EVERY measured joint sample (whole run, causal obstacle pose)
sel = np.where((js_t >= T0 - 1.0) & (js_t <= T1 + 0.5))[0]
G = np.full((sel.size, len(lab0)), np.nan)
Pcp = np.full((sel.size, len(lab0), 3), np.nan)
pose_age = np.full(sel.size, np.nan)
for k, i in enumerate(sel):
    j = np.searchsorted(op_t, js_t[i], side='right') - 1
    if j < 0:
        continue
    pose_age[k] = js_t[i] - op_t[j]
    c = op_p[j, 0, :]
    if c[2] < -10.0:          # parked below the floor before placement: not an obstacle in the scene
        continue
    cps = recon(js_q[i])
    Pcp[k] = np.array([cc['point'] for cc in cps])
    G[k] = surface_gaps(cps, c[None, :], radii)[:, 0]
rt, rq = js_t[sel], js_q[sel]
gmin = np.nanmin(G, axis=1)

# match to truth samples by identical header stamp
key = {round(float(t), 6): i for i, t in enumerate(tt)}
mi = np.array([key.get(round(float(t), 6), -1) for t in rt])
both = (mi >= 0) & np.isfinite(gmin)
ee = (rt >= T0) & (rt <= T1)
dq = np.abs(rq[both] - tq[mi[both]]).max(axis=1)
dg = np.abs(G[both] - tgap[mi[both], :, 0])
dgm = np.abs(gmin[both] - tgap[mi[both], :, 0].min(axis=1))
R['check_B_independent'] = dict(
    n_recon_episode=int((ee & np.isfinite(gmin)).sum()), n_truth_episode=int(te.sum()),
    n_matched_episode=int((both & ee).sum()),
    n_recon_without_truth_episode=int((ee & np.isfinite(gmin) & (mi < 0)).sum()),
    n_truth_without_bag_episode=int(te.sum() - (both & ee).sum()),
    max_abs_q_diff_rad=float(dq.max()), n_q_diff_nonzero=int((dq > 0).sum()),
    gap_abs_diff_all_cp_m=pct(dg.ravel()), min_gap_abs_diff_m=pct(dgm),
    obstacle_pose_age_ms=pct(1e3 * pose_age[ee]))

# ------------------------------------------------------------------ truth holes and the bag inside them
dt_truth = np.diff(tt)
holes = []
for i in np.where(dt_truth > 0.0021)[0]:
    a, b = float(tt[i]), float(tt[i + 1])
    inside = (rt > a) & (rt < b)
    gi = gmin[inside]
    h = dict(after_truth_t=a, before_truth_t=b, gap_ms=1e3 * (b - a), in_episode=bool(b >= T0 and a <= T1),
             truth_gap_edges=[float(tgap[i].min()), float(tgap[i + 1].min())],
             bag_samples_inside=int(inside.sum()),
             bag_max_dt_ms_inside=float(1e3 * np.diff(np.r_[a, rt[inside], b]).max()) if inside.any() else 1e3 * (b - a),
             recon_min_gap_inside=float(np.nanmin(gi)) if gi.size and np.isfinite(gi).any() else None,
             recon_min_t_inside=float(rt[inside][np.nanargmin(gi)]) if gi.size and np.isfinite(gi).any() else None,
             recon_min_cp_inside=lab0[int(np.nanargmin(G[inside][np.nanargmin(gi)]))] if gi.size and np.isfinite(gi).any() else None)
    holes.append(h)
R['truth_holes_gt_2ms'] = holes
R['truth_dt_episode_ms'] = pct(1e3 * np.diff(tt[te]))

# ------------------------------------------------------------------ clearance on the reconstruction
er = ee & np.isfinite(gmin)
k = np.where(er)[0][np.nanargmin(gmin[er])]
dtr = np.diff(rt[er])
R['recon_clearance'] = dict(
    min_gap=float(gmin[k]), t_min=float(rt[k]), cp_min=lab0[int(np.nanargmin(G[k]))],
    time_below_dsafe_s=float(np.sum(dtr[gmin[er][:-1] < D_SAFE])),
    time_below_0_s=float(np.sum(dtr[gmin[er][:-1] < 0])),
    n_samples_below_dsafe=int((gmin[er] < D_SAFE).sum()), n_samples_below_0=int((gmin[er] < 0).sum()))
# truth-only min for reference
R['truth_clearance'] = dict(min_gap=float(tgap[te].min()), t_min=float(tt[te][np.argmin(tgap[te].min(axis=(1, 2)))]))

# passes: local minima of the reconstructed min gap below 0.25 m, >= 2 s apart
passes = []
cand = np.where(er & (gmin < 0.25))[0]
for i in cand:
    lo, hi = np.searchsorted(rt, rt[i] - 1.0), np.searchsorted(rt, rt[i] + 1.0)
    w = gmin[lo:hi]
    if gmin[i] == np.nanmin(w) and (not passes or rt[i] - rt[passes[-1]] > 2.0):
        passes.append(i)
pl = []
for n, i in enumerate(passes, 1):
    lo, hi = rt[i] - 1.0, rt[i] + 1.0
    wt = (tt >= lo) & (tt <= hi)
    wr = (rt >= lo) & (rt <= hi)
    tmin = float(tgap[wt].min()) if wt.any() else None
    hh = [h for h in holes if h['before_truth_t'] >= lo and h['after_truth_t'] <= hi]
    pl.append(dict(n=n, t=float(rt[i]), recon_min_gap=float(gmin[i]), cp=lab0[int(np.nanargmin(G[i]))],
                   truth_min_gap_pm1s=tmin, truth_sample_at_recon_min=bool(mi[i] >= 0),
                   bag_max_dt_ms_pm1s=float(1e3 * np.diff(rt[wr]).max()),
                   truth_holes_pm1s=[[round(h['after_truth_t'], 3), round(h['gap_ms'], 1)] for h in hh],
                   recon_below_dsafe_pm1s=int((gmin[wr] < D_SAFE).sum())))
R['passes'] = pl

json.dump(R, open(os.path.join(OUT, 's6b1_reconstruction.json'), 'w'), indent=1, default=float)
np.savez_compressed(os.path.join(OUT, 's6b1_reconstruction.npz'), t=rt, q=rq, gap=G, gmin=gmin, cp=Pcp,
                    truth_index=mi, labels=np.asarray(lab0), pose_age=pose_age, js_rx=js_rx[sel])
print(json.dumps(R, indent=1, default=float))
