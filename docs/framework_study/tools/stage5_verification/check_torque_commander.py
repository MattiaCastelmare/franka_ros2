#!/usr/bin/env python3
"""pentagon_torque_commander (Pipeline C's nominal): orientation-loop sign (chapter 10 §3.1).

The node needs a ROS graph, so its orientation law (L398–431) is TRANSCRIBED here and checked
against the source text; the dynamics is idealised (the commanded angular acceleration is
realised exactly, world frame).

  1. the source computes e_rot = ½·vee(R_dᵀR − RᵀR_d) and adds +K_rot·e_rot to the angular
     acceleration, while e_pos = p_d − p                                                      (FACT)
  2. for R = R_d·exp([θ]×), e_rot = sin-like(θ) ≈ +θ: the rotation from DESIRED to CURRENT
     (opposite sign to e_pos's desired − current), expressed in R_d's frame                   (NUM)
  3. the resulting loop θ̈ = +K_p θ − K_d θ̇ is unstable: growth rate (−K_d + √(K_d² + 4K_p))/2;
     a 0.01 rad perturbation grows, while the pentagon_qddot_commander form −R_d·log(R_dᵀR)
     decays                                                                     (NUM, illustrative)
  4. no trajectory acceleration feedforward: ẍ_cmd = K_p e + K_d ė (contrast 06 (6.5))          (FACT)

Needs: Python 3, numpy. Host. Read-only.
"""
import math
import os
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common5 as C5  # noqa: E402

C = C5.C
import numpy as np  # noqa: E402

R = C.Report('check_torque_commander (chapter 10 §3.1)')
S = C5.src('franka_experiments/franka_experiments/nodes/pentagon_torque_commander.py')
R.fact('1. e_rot = ½(R_err[2,1] − R_err[1,2], …) with R_err = R_dᵀR',
       'np.dot(self._R_des.T, R_cur, out=self._R_err_buf)' in S
       and 'self._e_rot_buf[0] = 0.5 * (self._R_err_buf[2, 1] - self._R_err_buf[1, 2])' in S)
R.fact('1. angular acceleration = +kp_rot·e_rot + kd_rot·(−J_ang q̇); position error = p_d − p_ee',
       'np.multiply(self.kp_rot, self._e6_buf[3:], out=self._xddot6_buf[3:])' in S
       and 'np.negative(self._e6_dot_buf[3:], out=self._e6_dot_buf[3:])' in S
       and 'np.subtract(p_d, p_ee, out=self._scratch3_buf)' in S)
R.fact('4. no a_d term: xddot6[:3] = kp·e + kd·ė only', 'a_d' not in S.split('def _timer_cb')[1])
KP, KD = 20.0, 5.0                                   # node defaults kp_rot, kd_rot


def hat(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def expm(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3) + hat(w)
    K = hat(w / th)
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def logm(Rm):
    c = max(-1.0, min(1.0, (np.trace(Rm) - 1) / 2))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    return th / (2 * math.sin(th)) * np.array([Rm[2, 1] - Rm[1, 2], Rm[0, 2] - Rm[2, 0], Rm[1, 0] - Rm[0, 1]])


def e_code(Rd, Rc):
    E = Rd.T @ Rc
    return 0.5 * np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])


def e_fixed(Rd, Rc):                                 # pentagon_qddot_commander L884-886
    return -(Rd @ logm(Rd.T @ Rc))


Rd = expm(np.array([0.3, -0.2, 0.5]))
th = np.array([0.0, 0.01, 0.0])
R.close('2. e_rot(code) at R = R_d·exp([θ]) ≈ +θ (desired→current, R_d frame)', e_code(Rd, Rd @ expm(th)), th, 1e-6,
        kind='NUM')


def sim(err, T=2.0, dt=1e-3):
    Rc, w = Rd @ expm(np.array([0.004, 0.01, -0.003])), np.zeros(3)
    for _ in range(int(T / dt)):
        alpha = KP * err(Rd, Rc) - KD * w            # ideal tracking of the commanded angular accel
        w = w + alpha * dt
        Rc = expm(w * dt) @ Rc                       # world-frame angular velocity
    return float(np.linalg.norm(logm(Rd.T @ Rc)))


g = (-KD + math.sqrt(KD ** 2 + 4 * KP)) / 2
e1, e2 = sim(e_code), sim(e_fixed)
R.info(f'3. unstable root of θ̈ = K_p θ − K_d θ̇: {g:.3f} s⁻¹ (×{math.exp(2 * g):.0f} in 2 s)')
R.fact(f'3. 2 s from a 0.011 rad error: code sign → {e1:.3f} rad (grows); fixed sign → {e2:.2e} rad (decays)',
       e1 > 0.5 and e2 < 1e-3, 'illustrative: ideal tracking, no translation, no saturation')
sys.exit(R.finish())
