"""E3 H2 — container step (ROS sourced). Read-only on the cell; writes ONLY into <out_dir>.

Extracts what the host-side metrics need and cannot get from s6b_series.npz:
  * every /cbf/per_link_distances message: receipt (sim), capture stamp, valid entries, min d;
  * the joint state (position AND velocity, mapped by NAME: /NS_1/franka/joint_states is
    published in a scrambled order) just before and just after the obstacle's birth;
  * a replay of real_time_distance's robot self-mask + depth gate at the birth and after it,
    with the node's own MaskBuilder, the cell's own perception config, meshes, camera
    extrinsics and the camera intrinsics of the cell's own world.sdf. The depth image is not
    recorded, so the ball is rendered analytically (ray-sphere); the arm's depth is the
    mask's own z-buffer. A ball pixel is DELETED when it lies inside the dilated exclusion
    mask and less than depth_gate_bias + depth_gate_tol in front of the arm (distance_engine).

Usage: python3 -B e3_h2_bag.py <cell_dir> <analysis_dir_with_s6b1_npz> <out_dir>
"""
import json
import math
import os
import re
import sys

import numpy as np

CELL, ADIR, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
man = json.load(open(os.path.join(CELL, 'run_manifest_sim.json')))
T0 = man['episode']['t_start_sim']
if T0 is None:
    json.dump({'error': 'no episode start (t_start_sim is null)'},
              open(os.path.join(OUT, 'h2_bag.json'), 'x'), indent=1)
    sys.exit(0)
T0 = float(T0)
JN = [f'fr3_joint{i}' for i in range(1, 8)]

import rosbag2_py  # noqa: E402
from franka_msgs.msg import MultiLinkDistance  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

rd = rosbag2_py.SequentialReader()
rd.open(rosbag2_py.StorageOptions(uri=os.path.join(CELL, 'bag'), storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'))
rd.set_filter(rosbag2_py.StorageFilter(topics=['/cbf/per_link_distances', '/NS_1/franka/joint_states']))
pld, js = [], []
while rd.has_next():
    topic, raw, t_ns = rd.read_next()
    tr = t_ns * 1e-9
    if topic == '/cbf/per_link_distances':
        m = deserialize_message(raw, MultiLinkDistance)
        ds = [lk.distance for lk in m.links if lk.valid and math.isfinite(lk.distance)]
        pld.append((tr, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, len(ds),
                    min(ds) if ds else math.inf))
    elif abs(tr - T0) < 0.3:
        m = deserialize_message(raw, JointState)
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        d = {n: (m.position[i], m.velocity[i] if i < len(m.velocity) else math.nan)
             for i, n in enumerate(m.name)}
        if all(n in d for n in JN):
            js.append((st, [d[n][0] for n in JN], [d[n][1] for n in JN]))
pld.sort(key=lambda x: x[1])
js.sort()
P = np.array(pld, float) if pld else np.zeros((0, 4))

R = {'t0': T0, 'n_pld': len(pld)}
pre = P[(P[:, 0] < T0) & (P[:, 0] >= T0 - 1.0)] if len(P) else P
R['pre_birth_1s'] = dict(msgs=int(len(pre)), nonempty=int((pre[:, 2] > 0).sum()) if len(pre) else 0,
                         last_capture_age_s=float(T0 - pre[-1, 1]) if len(pre) else None,
                         rate_hz=float(len(pre)))
post = P[P[:, 1] >= T0] if len(P) else P
ne = post[post[:, 2] > 0] if len(post) else post
R['first_nonempty_after_birth'] = (dict(capture_minus_t0=float(ne[0, 1] - T0),
                                        receipt_minus_t0=float(ne[0, 0] - T0), d_min=float(ne[0, 3]),
                                        empty_frames_before=int(((post[:, 1] < ne[0, 1])).sum()))
                                   if len(ne) else None)
bef = [x for x in js if x[0] <= T0]
aft = [x for x in js if x[0] >= T0]
R['joint_state_before_birth'] = dict(stamp=bef[-1][0], q=bef[-1][1], qdot=bef[-1][2]) if bef else None
R['joint_state_after_birth'] = dict(stamp=aft[0][0], q=aft[0][1], qdot=aft[0][2]) if aft else None

# ---------------------------------------------------------------- self-mask replay
import pinocchio as pin  # noqa: E402
import trimesh  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402

from franka_experiments.utils.config import load_config_file  # noqa: E402
from franka_experiments.utils.distance_utils import load_extrinsics  # noqa: E402
from franka_experiments.utils.mask_builder import MaskBuilder  # noqa: E402

cfg = load_config_file(man['files']['perception_config'])
R_base, t_base = load_extrinsics(man['files']['camera_extrinsics'])
sdf = open(man['files']['world']).read()
cam = re.search(r'<sensor name="depth" type="depth_camera">(.*?)</sensor>', sdf, re.S).group(1)
hfov = float(re.search(r'<horizontal_fov>([0-9.eE+-]+)</horizontal_fov>', cam).group(1))
W = int(re.search(r'<width>(\d+)</width>', cam).group(1))
H = int(re.search(r'<height>(\d+)</height>', cam).group(1))
fx = (W / 2.0) / math.tan(hfov / 2.0)
K = np.array([[fx, 0, W / 2.0], [0, fx, H / 2.0], [0, 0, 1.0]])
obs = man['scenario_resolved']['obstacles']
# F-8-41 (C4): ONE obstacle, any ANALYTIC law (static, linear, oscillate, wander, ballistic); the ball is
# rendered at its law position at each probe (law time = t - t0, as scenario_obstacles_system.cpp runs it;
# s6b1_reconstruct_clearance.py checks that against the bag's /sim/truth/obstacle_poses). Static: p0.
if len(obs) != 1:
    raise SystemExit('mask replay implemented for ONE obstacle only')
sys.path.insert(0, '/ros2_ws/install/franka_simulation_experiments/lib/franka_simulation_experiments')
from simexp import obstacle_laws as OL  # noqa: E402
BALL_SPEC = obs[0]
BALL_C, BALL_R = np.array(OL.position(BALL_SPEC, 0.0), float), float(obs[0]['radius'])
gate = float(cfg['mask']['depth_gate_bias_m']) + float(cfg['mask']['depth_gate_tol_m'])
np.random.seed(0)   # the node's trimesh.sample is unseeded: equivalent up to sample density (Inferred)
base = get_package_share_directory(cfg['meshes'].get('package', 'franka_description'))
n_s = int(cfg['meshes'].get('sample_points_per_link', 300))
samples = {ln: np.asarray(trimesh.load(os.path.join(base, rel), force='mesh').sample(n_s))
           for ln, rel in cfg['meshes']['files'].items()}
mask_cfg = cfg['mask']
if bool(cfg['distance'].get('ssb_enabled', False)):          # real_time_distance.__init__, verbatim rule
    mask_cfg = dict(mask_cfg)
    mask_cfg['model_points_stride'] = max(1, int(cfg['distance'].get('ssb_model_points_stride', 8)))
EE_LINK = cfg['robot'].get('ee_link', 'fr3_link8')         # real_time_distance: robot_cfg['ee_link']
mb = MaskBuilder(link_mesh_samples=samples, R_base=R_base, t_base=t_base, ee_link=EE_LINK,
                 mask_cfg=mask_cfg)
mb.set_intrinsics(K)
model = pin.buildModelFromUrdf(os.path.join(CELL, 'fr3_rsp.urdf'))
data = model.createData()
arm = [model.getJointId(n) for n in JN]
z = np.load(os.path.join(ADIR, 's6b1_reconstruction.npz'))
tq, Q = z['t'], z['q']
uu, vv = np.meshgrid(np.arange(W), np.arange(H))
dr = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones_like(uu, float)], -1)
a = (dr * dr).sum(-1)


def render_ball(center):
    c = R_base.T @ (center - t_base)
    b = -2 * (dr @ c)
    cc = c @ c - BALL_R ** 2
    disc = b * b - 4 * a * cc
    bz = np.where(disc >= 0, (-b - np.sqrt(np.maximum(disc, 0))) / (2 * a), np.inf)
    return bz, np.isfinite(bz)


rows = []
np.seterr(invalid='ignore')   # inf - inf where neither the arm nor the ball covers a pixel
for dt in np.round(np.arange(0.0, 1.001, 0.05), 3):
    i = int(np.searchsorted(tq, T0 + dt))
    if i >= len(tq):
        break
    qf = pin.neutral(model)
    for k, jid in enumerate(arm):
        qf[model.joints[jid].idx_q] = Q[i, k]
    pin.forwardKinematics(model, data, qf)
    pin.updateFramePlacements(model, data)
    tr = {ln: (np.asarray(data.oMf[model.getFrameId(ln)].rotation),
               np.asarray(data.oMf[model.getFrameId(ln)].translation)) for ln in samples}
    mb.rebuild(tr, (H, W))
    ball_c = np.array(OL.position(BALL_SPEC, float(tq[i]) - T0), float)     # F-8-41 (C4)
    ball_Z, ball_px = render_ball(ball_c)
    zr, excl = mb.robot_depth, mb.search_exclusion_mask
    hidden = ball_px & np.isfinite(zr) & (zr < ball_Z - 0.02) & mb.robot_mask
    vis = ball_px & ~hidden
    in_mask = vis & excl
    kept = in_mask & (ball_Z < zr - gate)
    deleted = in_mask & ~kept
    dz = (zr - ball_Z)[in_mask]
    rows.append(dict(dt=float(dt), t_q=float(tq[i]), ball_center=ball_c.tolist(),
                     ball_px=int(ball_px.sum()), hidden_by_arm=int(hidden.sum()),
                     in_exclusion_mask=int(in_mask.sum()), deleted_by_self_mask=int(deleted.sum()),
                     visible=int(((vis & ~excl) | kept).sum()),
                     min_arm_minus_ball_depth_in_mask_m=float(dz.min()) if dz.size else None))
R['mask_replay'] = dict(camera=dict(W=W, H=H, hfov=hfov, fx=fx, extrinsics=man['files']['camera_extrinsics']),
                        ee_link=EE_LINK, ssb_enabled=bool(cfg['distance'].get('ssb_enabled', False)),
                        ball=dict(center=BALL_C.tolist(), radius=BALL_R, law=BALL_SPEC.get('law', 'static')),
                        depth_gate_m=gate,
                        sample_points_per_link=n_s, probes=rows)
np.savez_compressed(os.path.join(OUT, 'h2_bag.npz'), pld=P)
with open(os.path.join(OUT, 'h2_bag.json'), 'x') as f:
    json.dump(R, f, indent=1)
print(json.dumps({k: R[k] for k in ('pre_birth_1s', 'first_nonempty_after_birth')}, indent=1))
print('mask replay at birth:', rows[0] if rows else None)
