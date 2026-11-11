#!/usr/bin/env python3
"""Simulated robot, camera and ground-truth geometry (chapter 12 §3, §7). Container, read-only.

Production code: `utils.gz_robot_description` (`process_xacro`, `build_descriptions`,
`lump_inertials`, `rpy_to_matrix`, `kdl_tip_link`, `ARMATURE`), `utils.gz_world` (`matrix_to_rpy`,
`camera_link_pose`, `hfov_from_fx`, `R_LINK_FROM_OPTICAL`), `simexp.truth_geometry.TruthGeometry`,
`simexp.scenario_catalog.resolve/enforce_start_clearance/_spawn`, with the SOURCE `fr3_complete.yaml`
and `fr3_control.yaml`. Pinocchio builds the models; xacro runs `franka_description` as the launch does.

  1. rpy_to_matrix and matrix_to_rpy are inverses (Rz Ry Rx)                                        (NUM)
  2. (12.1) camera link: R_link R_LINK_FROM_OPTICAL = R_opt, so the rendered image is the optical frame;
     (12.2) hfov = 2 atan(W / 2 fx)                                                                 (NUM)
  3. lump_inertials = the parallel-axis theorem                                                    (NUM)
  4. the simulated arm: total mass and g(q) unchanged by armature + rigid fingers (fingers at
     0.04 m); M(q) gains the armature on its diagonal plus transverse disk terms (F-4-9 context)  (NUM)
  5. world -> base -> fr3_link0 is the identity, so the Gazebo world frame = base = fr3_link0       (FACT)
  6. truth control points at Q0; every scenario starts outside d_safe after enforce_start_clearance,
     and which ones had to be shifted                                                               (NUM/INFO)
"""
import os
import sys
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common6 as C6  # noqa: E402
import numpy as np  # noqa: E402
import pinocchio as pin  # noqa: E402
import yaml  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402
from simexp import scenario_catalog as S, truth_geometry as TG  # noqa: E402
from utils import gz_robot_description as GRD, gz_world as GW  # noqa: E402

R = C6.C.Report('check_sim_geometry (chapter 12 §3, §7)')
rng = np.random.default_rng(6)

# 1
err = 0.0
for Rm in Rotation.random(200, random_state=6).as_matrix():
    err = max(err, float(np.max(np.abs(GRD.rpy_to_matrix(GW.matrix_to_rpy(Rm)) - Rm))))
R.below('1. max |rpy_to_matrix(matrix_to_rpy(R)) - R| over 200 random rotations', err, 1e-12)

# 2
e2 = 0.0
for Rm in Rotation.random(50, random_state=7).as_matrix():
    _, rpy = GW.camera_link_pose(Rm, np.zeros(3))
    e2 = max(e2, float(np.max(np.abs(GRD.rpy_to_matrix(rpy) @ GW.R_LINK_FROM_OPTICAL - Rm))))
R.below('2. camera link: max |R_link R_LINK_FROM_OPTICAL - R_opt|', e2, 1e-12)
R.close('2. hfov_from_fx(848, 424) = 2 atan(1) = pi/2', GW.hfov_from_fx(848, 424.0), np.pi / 2, 1e-15, kind='NUM')
R.fact('2. R_LINK_FROM_OPTICAL maps optical z (view axis) to link x (Gazebo camera axis)',
       np.allclose(GW.R_LINK_FROM_OPTICAL @ np.array([0, 0, 1.0]), [1, 0, 0]))

# 3
bodies = []
for _ in range(3):
    pts, w = rng.normal(size=(40, 3)) * 0.1 + rng.normal(size=3) * 0.2, rng.uniform(0.1, 1.0, 40)
    m = w.sum(); c = (w[:, None] * pts).sum(0) / m; d = pts - c
    I = sum(wi * (di @ di * np.eye(3) - np.outer(di, di)) for wi, di in zip(w, d))
    bodies.append((m, c, I, pts, w))
m, c, I = GRD.lump_inertials([b[:3] for b in bodies])
P = np.vstack([b[3] for b in bodies]); W = np.concatenate([b[4] for b in bodies]); D = P - c
I_direct = sum(wi * (di @ di * np.eye(3) - np.outer(di, di)) for wi, di in zip(W, D))
R.below('3. lump_inertials vs the inertia of the merged point cloud (max abs)', float(np.max(np.abs(I - I_direct))), 1e-12)

# 4
xml0 = GRD.process_xacro('fr3', hand=True)
Q0 = np.array(S.load()['q0'], float)
gz, rsp = GRD.build_descriptions(xml0, initial_q=Q0)
m0, m1 = pin.buildModelFromXML(xml0), pin.buildModelFromXML(rsp)
d0, d1 = m0.createData(), m1.createData()
R.close('4. total mass: xacro vs simulated [kg]', sum(i.mass for i in m1.inertias), sum(i.mass for i in m0.inertias),
        1e-9, kind='NUM')
idx0 = [m0.joints[m0.getJointId(f'fr3_joint{i}')].idx_q for i in range(1, 8)]
idx1 = [m1.joints[m1.getJointId(f'fr3_joint{i}')].idx_q for i in range(1, 8)]
iv0 = [m0.joints[m0.getJointId(f'fr3_joint{i}')].idx_v for i in range(1, 8)]
iv1 = [m1.joints[m1.getJointId(f'fr3_joint{i}')].idx_v for i in range(1, 8)]
fing = [m0.joints[m0.getJointId(j)].idx_q for j in GRD.FINGER_JOINTS if m0.existJointName(j)]
eg, dM_diag, dM_off = 0.0, [], 0.0
for _ in range(5):
    qa = rng.uniform(-1.5, 1.5, 7)
    q0 = pin.neutral(m0); q0[idx0] = qa; q0[fing] = GRD.FINGER_OPEN_M
    q1 = pin.neutral(m1); q1[idx1] = qa
    g0 = pin.computeGeneralizedGravity(m0, d0, q0)[iv0]
    g1 = pin.computeGeneralizedGravity(m1, d1, q1)[iv1]
    eg = max(eg, float(np.max(np.abs(g0 - g1))))
    M0 = pin.crba(m0, d0, q0); M0 = np.triu(M0) + np.triu(M0, 1).T
    M1 = pin.crba(m1, d1, q1); M1 = np.triu(M1) + np.triu(M1, 1).T
    dM = M1[np.ix_(iv1, iv1)] - M0[np.ix_(iv0, iv0)]
    dM_diag.append(np.diag(dM) - np.array(GRD.ARMATURE))
    dM_off = max(dM_off, float(np.max(np.abs(dM - np.diag(np.diag(dM))))))
R.below('4. max |g_sim(q) - g_xacro(q)| over 5 random q (fingers at 0.04 m) [N·m]', eg, 1e-9)
dd = np.array(dM_diag)
A = np.array(GRD.ARMATURE)
lo_b = np.array([0.5 * A[j + 1:].sum() for j in range(7)])      # (12.16): a disk on link k adds
hi_b = np.array([A[j + 1:].sum() for j in range(7)])            # I_a,k (1 + (z_j . z_k)^2) / 2 to M_jj
R.fact(f'4. diag(M_sim - M_xacro) - ARMATURE per joint, min/max over 5 q: '
       f'{np.round(dd.min(0), 4).tolist()} / {np.round(dd.max(0), 4).tolist()}; within the disk bounds '
       f'[sum_(k>j) I_a,k / 2, sum_(k>j) I_a,k]; max off-diagonal change {dM_off:.4f} kg·m²',
       bool(np.all(dd >= lo_b - 1e-9) and np.all(dd <= hi_b + 1e-9)))
R.fact(f"4. kdl_tip_link(gazebo urdf) = {GRD.kdl_tip_link(gz)!r}", GRD.kdl_tip_link(gz) == 'fr3_hand_tcp')

# 5
root = ET.fromstring(gz)
def origin(jn):
    j = next(e for e in root.findall('joint') if e.get('name') == jn)
    o = j.find('origin')
    return (j.find('parent').get('link'), j.find('child').get('link'),
            np.fromstring((o.get('xyz') if o is not None else '0 0 0'), sep=' '),
            np.fromstring((o.get('rpy') if o is not None else '0 0 0'), sep=' '))
wb, bl = origin('world_to_base'), origin('fr3_base_joint')
R.fact(f'5. gazebo urdf: {wb[0]}->{wb[1]} xyz {wb[2].tolist()} rpy {wb[3].tolist()}; '
       f'{bl[0]}->{bl[1]} xyz {bl[2].tolist()} rpy {bl[3].tolist()}',
       wb[:2] == ('world', 'base') and bl[:2] == ('base', 'fr3_link0')
       and not np.any(wb[2]) and not np.any(wb[3]) and not np.any(bl[2]) and not np.any(bl[3]))
stack = C6.src('franka_experiments/launch/torque_control_stack.launch.py', 'py')
R.fact('5. torque_control_stack publishes world -> base as the identity (world_to_robot_base_tf)',
       "'--x', '0', '--y', '0', '--z', '0'," in stack and "'--frame-id', 'world'," in stack)
R.fact('5. the RSP copy has no world link (build_descriptions strips it)',
       'world' not in [e.get('name') for e in ET.fromstring(rsp).findall('link')])

# 6
pc = yaml.safe_load(open(os.path.join(C6.C.CONFIG, 'fr3_complete.yaml')))
d_safe = float(yaml.safe_load(open(os.path.join(C6.C.CONFIG, 'fr3_control.yaml')))['params']['d_safe'])
geo = TG.TruthGeometry(rsp, pc)
cps = geo.control_points(Q0)
R.fact(f'6. {len(cps)} truth control points at Q0 (gz_cbf_stack sets truth_n_cp = 11): '
       + ', '.join(f"{TG.cp_label(c)} r={c['radius']:.3f}" for c in cps), len(cps) == 11)
R.close('6. flange (fr3_link8) z at Q0 vs scenarios.yaml flange_q0 [m]', float(np.linalg.norm(
    geo.flange(Q0) - np.asarray(S.load()['flange_q0']))), 0.0, 1e-3, kind='NUM')
worst, shifted = np.inf, []
cfg = S.load()
for n in S.all_names(cfg):
    r = S.resolve(n, 0, cfg)
    if not r['obstacles']:
        continue
    q = np.asarray(r['q0'], float)
    cq = geo.control_points(q)
    r2 = S.enforce_start_clearance(r, cq, d_safe)
    g = TG.surface_gaps(cq, [S._spawn(o) for o in r2['obstacles']], [o['radius'] for o in r2['obstacles']])
    worst = min(worst, float(g.min()) - d_safe)
    shifted += [(n, s['cp'], round(s['gap_before'], 4), round(s['shift_m'], 4)) for s in r2['start_shift']]
R.fact(f'6. after enforce_start_clearance every scenario starts with min gap - d_safe >= 0 (worst {worst:+.4f} m, '
       f'd_safe {d_safe} from the SOURCE fr3_control.yaml)', worst >= 0.0)
R.info(f'6. scenarios shifted at the start (name, CP, gap before, shift): {shifted}')
sys.exit(R.finish())
