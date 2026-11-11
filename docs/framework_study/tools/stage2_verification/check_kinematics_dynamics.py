#!/usr/bin/env python3
"""Kinematics and dynamics checks for chapter 04 (preserved from the Stage 2 `check_kin.py`).

Verifies, against the baseline source (`franka_experiments.utils.kinematics`,
`franka_experiments.utils.distance_utils`):
  [A] model facts: the no-hand and hand models, base frame = fr3_link0 = identity, masses,
      fr3_hand / fr3_hand_tcp placement relative to fr3_link8                       (04 §2)
  [B] over 25 random states (seed 0): control-point FK (kinematics vs perception), J (4.7)-(4.8),
      J-dot (4.8), the acceleration identity (4.6), the rigid-point J_P (4.4) and J-dot_P (4.5),
      Pinocchio's LWA J-dot = d/dt J_LWA                                             (04 §3.4, §4.3)
  [C] dynamics on the hand model, 10 random states: data.M fill, data.C = 0, nle - g = C(q,qd) qd,
      and the torque the code computes (4.10) against RNEA                           (04 §4.5)

Needs: ROS 2 Humble environment with xacro and franka_description (ament index), Pinocchio,
numpy, PyYAML. Run inside the `franka_ros2` container (see README.md).
The random draws are in the same order and with the same seed as the original run, so the
printed error magnitudes are directly comparable with chapter 04.
"""
import os
import sys

sys.dont_write_bytecode = True   # read-only: never write __pycache__ into the repository
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common as C  # noqa: E402

C.use_source_package()
import numpy as np  # noqa: E402
import pinocchio as pin  # noqa: E402
from franka_experiments.utils.kinematics import (  # noqa: E402
    CBFKinematics, build_urdf_no_hand, generate_urdf_from_xacro, load_pinocchio_model,
    resolve_arm_joint_ids)
from franka_experiments.utils.distance_utils import (  # noqa: E402
    define_control_points, define_robot_segments)

R = C.Report('check_kinematics_dynamics (chapter 04)')
R.info(f'pinocchio {pin.__version__} (Stage 2 reported 4.0.0)')
cfg = C.load_yaml(os.path.join(C.CONFIG, 'fr3_complete.yaml'))

# ── [A] models ──────────────────────────────────────────────────────────────────────────────
p_nohand = build_urdf_no_hand()
try:
    m = pin.buildModelFromUrdf(p_nohand)
finally:
    os.unlink(p_nohand)
R.fact('[A] no-hand model is 7-DoF', m.nq == 7 and m.nv == 7, f'nq={m.nq} nv={m.nv}')
for nm in ('base', 'fr3_link0'):
    fr = m.frames[m.getFrameId(nm)]
    R.fact(f'[A] frame {nm} has identity placement on joint 0',
           fr.parentJoint == 0 and fr.placement.isIdentity(), f'parentJoint={fr.parentJoint}')
mh, dh = load_pinocchio_model(generate_urdf_from_xacro())
R.fact('[A] hand model is 9-DoF', mh.nq == 9 and mh.nv == 9, f'nq={mh.nq} nv={mh.nv}')
R.close('[A] total mass no-hand [kg]', sum(i.mass for i in m.inertias), 17.891, 5e-4)
R.close('[A] total mass hand [kg]', sum(i.mass for i in mh.inertias), 18.604, 5e-4)
pin.framesForwardKinematics(mh, dh, pin.neutral(mh))
o8 = dh.oMf[mh.getFrameId('fr3_link8')]
rel_h = o8.actInv(dh.oMf[mh.getFrameId('fr3_hand')])
rel_t = o8.actInv(dh.oMf[mh.getFrameId('fr3_hand_tcp')])
R.close('[A] fr3_hand in fr3_link8: translation [m]', rel_h.translation, [0, 0, 0], 1e-9)
R.close('[A] fr3_hand in fr3_link8: rpy [rad] (basis of F-2-9)',
        pin.rpy.matrixToRpy(rel_h.rotation), [0, 0, -np.pi / 4], 1e-9)
R.close('[A] fr3_hand_tcp in fr3_link8: translation [m]', rel_t.translation, [0, 0, 0.1034], 1e-9)

# ── [B] derivatives, 25 random states ───────────────────────────────────────────────────────
rng = np.random.default_rng(0)
lo, hi = m.lowerPositionLimit, m.upperPositionLimit
kin = CBFKinematics(m)
kin.configure_control_points(cfg['robot'], cfg['distance'])
labels = list(kin._control_points)
R.fact('[B] 11 control points configured', len(labels) == 11, ', '.join(labels))
d = m.createData()


def positions(q):
    """Control points as perception builds them (distance_utils.define_control_points)."""
    pin.forwardKinematics(m, d, q)
    pin.updateFramePlacements(m, d)
    tf = {n: (d.oMf[m.getFrameId(n)].rotation.copy(), d.oMf[m.getFrameId(n)].translation.copy())
          for n in cfg['robot']['segment_links']}
    return np.array([c['point'] for c in define_control_points(tf, cfg['robot'], cfg['distance'])]), tf


worst = dict(pos=0.0, J=0.0, Jd=0.0, acc=0.0, Jrig=0.0, Jdrig=0.0, lwad=0.0)
eps = 1e-6
for trial in range(25):
    q = lo + (hi - lo) * rng.random(7)
    v = rng.normal(0, 1.0, 7)
    a = rng.normal(0, 3.0, 7)
    kin.update(q, v)
    P_kin = np.array([kin.control_point_position(lb) for lb in labels])
    P_per, _ = positions(q)
    worst['pos'] = max(worst['pos'], np.abs(P_kin - P_per).max())
    J = np.array([kin.control_point_jacobian(lb)[0] for lb in labels])
    Jd = np.array([kin.control_point_jacobian(lb)[1] for lb in labels])
    fd = np.empty_like(J)
    for k in range(7):
        e = np.zeros(7)
        e[k] = eps
        fd[:, :, k] = (positions(q + e)[0] - positions(q - e)[0]) / (2 * eps)
    worst['J'] = max(worst['J'], np.abs(J - fd).max())
    kin.update(q + v * eps, v)
    Jp = np.array([kin.control_point_jacobian(lb)[0] for lb in labels])
    kin.update(q - v * eps, v)
    Jm = np.array([kin.control_point_jacobian(lb)[0] for lb in labels])
    worst['Jd'] = max(worst['Jd'], np.abs(Jd - (Jp - Jm) / (2 * eps)).max())
    # (4.6) along q(t) = q + v t + a t^2 / 2:  p'' = J a + J-dot v
    h = 1e-4
    P = lambda tt: positions(q + v * tt + 0.5 * a * tt * tt)[0]  # noqa: E731
    acc_fd = (P(h) - 2 * P(0) + P(-h)) / h ** 2
    acc_an = np.einsum('cij,j->ci', J, a) + np.einsum('cij,j->ci', Jd, v)
    worst['acc'] = max(worst['acc'], np.abs(acc_fd - acc_an).max())
    # (4.4)/(4.5) rigid point at a random offset on fr3_link5
    kin.update(q, v)
    fid = m.getFrameId('fr3_link5')
    off = rng.normal(0, 0.1, 3)
    Jr, Jrd = kin.point_jacobian(fid, kin.data.oMf[fid].act(off))

    def prig(qq):
        pin.forwardKinematics(m, d, qq)
        pin.updateFramePlacements(m, d)
        return d.oMf[fid].act(off)

    fdr = np.column_stack([(prig(q + np.eye(7)[k] * eps) - prig(q - np.eye(7)[k] * eps)) / (2 * eps)
                           for k in range(7)])
    worst['Jrig'] = max(worst['Jrig'], np.abs(Jr - fdr).max())

    def jrig(qq):
        kin.update(qq, v)
        return kin.point_jacobian(fid, kin.data.oMf[fid].act(off))[0]

    worst['Jdrig'] = max(worst['Jdrig'], np.abs(Jrd - (jrig(q + v * eps) - jrig(q - v * eps)) / (2 * eps)).max())
    # Pinocchio convention: getFrameJacobianTimeVariation(LWA) = d/dt J_LWA
    kin.update(q, v)
    J6d = pin.getFrameJacobianTimeVariation(m, kin.data, fid, pin.LOCAL_WORLD_ALIGNED)

    def j6(qq):
        dd = m.createData()
        pin.computeJointJacobians(m, dd, qq)
        pin.updateFramePlacements(m, dd)
        return pin.getFrameJacobian(m, dd, fid, pin.LOCAL_WORLD_ALIGNED)

    worst['lwad'] = max(worst['lwad'], np.abs(J6d - (j6(q + v * eps) - j6(q - v * eps)) / (2 * eps)).max())

reported = dict(pos=1.1e-16, J=1.6e-10, Jd=2.3e-10, acc=1.9e-7, Jrig=1.2e-10, Jdrig=1.8e-10, lwad=2.9e-10)
bounds = dict(pos=1e-12, J=1e-8, Jd=1e-8, acc=1e-5, Jrig=1e-8, Jdrig=1e-8, lwad=1e-8)
names = dict(pos='[B] control-point FK, kinematics vs perception [m]',
             J='[B] J (4.8) vs central difference of perception positions',
             Jd='[B] J-dot (4.8) vs central difference of J along qd',
             acc='[B] (4.6) p\'\' = J qdd + J-dot qd vs second difference (h=1e-4)',
             Jrig='[B] rigid-point J_P (4.4) vs central difference',
             Jdrig='[B] rigid-point J-dot_P (4.5) vs central difference',
             lwad='[B] Pinocchio LWA J-dot vs d/dt J_LWA')
for k in worst:
    R.below(names[k], worst[k], bounds[k], note=f'reported {reported[k]:.1e}')
q = 0.5 * (lo + hi)
P_per, tf = positions(q)
segs = define_robot_segments(tf, cfg['robot'], cfg['distance'])
R.below('[B] last capsule end = fingertip control point [m]', np.abs(segs[-1]['p1'] - P_per[-1]).max(),
        1e-12, kind='FACT')

# ── [C] dynamics, hand model ────────────────────────────────────────────────────────────────
acc = {'lower_tri_filled': [], 'C_zero': [], 'nle_g_vs_Cv': [], 'code_vs_true_tau': [], 'tau_max': []}
jids = resolve_arm_joint_ids(mh)
for trial in range(10):
    qf = pin.neutral(mh)
    qa = lo + (hi - lo) * rng.random(7)
    va = rng.normal(0, 1, 7)
    aa = rng.normal(0, 3, 7)
    vf = np.zeros(mh.nv)
    af = np.zeros(mh.nv)
    for k, j in enumerate(jids):
        qf[mh.joints[j].idx_q] = qa[k]
        vf[mh.joints[j].idx_v] = va[k]
        af[mh.joints[j].idx_v] = aa[k]
    pin.computeAllTerms(mh, dh, qf, vf)
    M = np.array(dh.M)
    acc['lower_tri_filled'].append(not np.allclose(np.tril(M, -1), 0))
    acc['C_zero'].append(np.allclose(dh.C, 0))
    nle_g = np.array(dh.nle) - np.array(dh.g)
    Cm = pin.computeCoriolisMatrix(mh, mh.createData(), qf, vf)
    acc['nle_g_vs_Cv'].append(np.abs(nle_g - Cm @ vf).max())
    tau_rnea = pin.rnea(mh, mh.createData(), qf, vf, af)
    g = pin.computeGeneralizedGravity(mh, mh.createData(), qf)
    vi = [mh.joints[j].idx_v for j in jids]
    tau_code = M[np.ix_(vi, vi)] @ aa + nle_g[vi]          # (4.10), what qddot_to_torque computes
    tau_true = (tau_rnea - g)[vi]                          # M a + C v, arm rows, fingers at rest
    acc['code_vs_true_tau'].append(np.abs(tau_code - tau_true).max())
    acc['tau_max'].append(np.abs(tau_true).max())
R.fact('[C] data.M lower triangle is filled after computeAllTerms', all(acc['lower_tri_filled']),
       'all 10 states' if all(acc['lower_tri_filled']) else 'NOT filled in some state')
R.fact('[C] data.C is identically 0 after computeAllTerms', all(acc['C_zero']))
R.below('[C] nle - g vs computeCoriolisMatrix @ qd', max(acc['nle_g_vs_Cv']), 1e-10, note='reported 1.5e-14')
R.below('[C] (4.10) code torque vs rnea - g [N m]', max(acc['code_vs_true_tau']), 1e-10,
        note=f'reported 1.3e-14; torques up to {max(acc["tau_max"]):.1f} N m (reported 13.6)')
sys.exit(R.finish())
