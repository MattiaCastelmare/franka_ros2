#!/usr/bin/env python3
"""Nominal-command shaping and IK identities (chapter 06 §4, §6, §7).

  1. (6.8) damped pseudoinverse as coded, J+ = solve(J J^T + l2 I, J)^T, equals J^T (J J^T + l2 I)^-1   (NUM)
  2. (6.10) the null-space "projector" N = I - J+ J built from the DAMPED J+ is not exact:
     the singular values of J N are l2 s/(s^2 + l2) <= sqrt(l2)/2. Bound at lambda_sq_min / max    (NUM)
  3. state governor (utils/state_governor.py): at q_dot = 0 the velocity margin uses the UPPER
     envelope only. For joint 4 below its upper reference (inside the filter's position box)
     the target weight with q_dot = 0 vs q_dot = -1e-6 rad/s, as a function of the distance to
     q_ref,up                                                                                 (NUM)
  4. RL commander, F-3-10: the obstacle centre of a held entry (p_human = 0) is -r n
     (rl_policy.obstacle_centre), and the finite-difference v_obs jump it causes
     |p_h,prev| / dt                                                                          (NUM)

Needs: Python 3, numpy, PyYAML. Host. Read-only.
"""
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common3 as C3  # noqa: E402

C = C3.C
C.use_source_package()
import numpy as np  # noqa: E402
from franka_experiments.utils.cbf_hard_limits import FR3_VEL_Q_REF_UPPER  # noqa: E402
from franka_experiments.utils.rl_policy import obstacle_centre  # noqa: E402
from franka_experiments.utils.state_governor import governor_from_params  # noqa: E402

R = C.Report('check_nominal_shaping (chapter 06 §4, §6, §7)')
rng = np.random.default_rng(7)

# 1-2) damped pseudoinverse and its null-space leakage
L2_MIN, L2_MAX = 1e-4, 5e-2          # pentagon_qddot_commander declare_parameter defaults
worst = 0.0
for _ in range(200):
    J = rng.normal(size=(6, 7)) * rng.uniform(0.05, 1.0)
    for l2 in (L2_MIN, L2_MAX):
        Jp_code = np.linalg.solve(J @ J.T + l2 * np.eye(6), J).T
        Jp_math = J.T @ np.linalg.inv(J @ J.T + l2 * np.eye(6))
        worst = max(worst, float(np.abs(Jp_code - Jp_math).max()))
R.below('(6.8) solve(JJ^T + l2 I, J)^T = J^T (JJ^T + l2 I)^-1 (200 random J, both l2)', worst, 1e-10)
worst = 0.0
for _ in range(200):
    J = rng.normal(size=(6, 7))
    l2 = float(rng.uniform(L2_MIN, L2_MAX))
    Jp = J.T @ np.linalg.inv(J @ J.T + l2 * np.eye(6))
    s = np.linalg.svd(J, compute_uv=False)
    got = np.sort(np.linalg.svd(J @ (np.eye(7) - Jp @ J), compute_uv=False))[-6:]
    exp = np.sort(l2 * s / (s * s + l2))
    worst = max(worst, float(np.abs(got - exp).max()))
R.below('(6.10) singular values of J(I - J+J) = l2 s / (s^2 + l2)', worst, 1e-10)
R.close('(6.10) worst-case leakage gain sqrt(l2)/2 at lambda_sq_min / lambda_sq_max',
        [np.sqrt(L2_MIN) / 2, np.sqrt(L2_MAX) / 2], [0.005, 0.1118], 1e-4, kind='NUM',
        note='m/s^2 of task acceleration per rad/s^2 of null-space command, at the worst singular value')

# 3) state governor at q_dot = 0
P = C3.params_namespace('hw_default')
gov = governor_from_params(P)
R.fact('state governor ON in the hardware default', gov is not None)
q = C.Q_HOME.copy()
qref = FR3_VEL_Q_REF_UPPER[3]
q4_box = qref - float(P.position_margin_rad)
rows = []
for dq in (0.30, 0.20, 0.10, 0.05, 0.02, 0.0):
    qq = q.copy()
    qq[3] = qref - dq
    out = []
    for qd4 in (0.0, -1e-6):
        g = governor_from_params(P)
        qd = np.zeros(7)
        qd[3] = qd4
        out.append(g.weight(q=qq, qdot=qd, dt=0.0).target)
    rows.append((dq, *out))
    R.info(f'joint 4 at q_ref,up - {dq:.2f} rad: target w (q_dot=0) = {out[0]:.3f}, '
           f'(q_dot4 = -1e-6) = {out[1]:.3f}')
w_at_box = [r for r in rows if abs(r[0] - float(P.position_margin_rad)) < 1e-12][0]
R.fact(f'asymmetry: at the filter position-box edge (q_ref,up - position_margin_rad = {q4_box:.4f} rad) '
       f'a STOPPED joint 4 gives w = {w_at_box[1]:.3f}, the same joint with q_dot4 = -1e-6 gives '
       f'{w_at_box[2]:.3f}', w_at_box[1] < w_at_box[2] - 0.1,
       'velocity_margin picks the upper envelope when q_dot >= 0 (np.where(qd >= 0.0, up, -lo))')

# 4) RL: held entry centre and v_obs spike
r_obs = 0.08
n = np.array([0.531, 0.0, 0.848])
c_bad = obstacle_centre(np.zeros(3), n, r_obs)
R.close('F-3-10: obstacle centre of a held entry = -r n [m]', c_bad, -r_obs * n, 1e-12, kind='NUM')
R.close('F-3-10: its distance from the base origin [m]', np.linalg.norm(c_bad), r_obs * np.linalg.norm(n), 1e-12,
        kind='NUM')
p_prev = np.array([0.52, 0.0, 0.49])                # the true point 0.21 m in front of the fingertip
c_prev = obstacle_centre(p_prev, n, r_obs)
for hz in (90.0, 100.0):
    R.info(f'F-3-10: finite-difference v_obs jump |c_bad - c_prev| * {hz:.0f} Hz = '
           f'{np.linalg.norm(c_bad - c_prev) * hz:.1f} m/s (|p_h,prev| = {np.linalg.norm(p_prev):.3f} m)')
R.close('F-3-10: the jump equals |p_h,prev| / dt when n is unchanged [m/s at 100 Hz]',
        np.linalg.norm(c_bad - c_prev) * 100.0, np.linalg.norm(p_prev) * 100.0, 1e-9, kind='NUM')
sys.exit(R.finish())
