"""Model-based reaching baseline: the robot's commander law, in the sim.

Same structure and gains as ``pentagon_qddot_commander`` (franka_experiments):

    ẍ = [ kp·sat(p* − p) − kd·ṗ ;  kp_rot·e_R − kd_rot·ω ]  − J̇q̇
    q̈_task = J⁺_λ ẍ                       adaptive damped least squares
    q̈_null = N·( −k_null·(q − q₀) − d_null·q̇ ),   N = I − J⁺_λ J
    q̈_nom  = q̈_task + q̈_null

Paired with ``env.shield: true`` and the obstacle rows ON it is the
model-based chain (nominal controller → CBF → torque) the RL policy is
compared against. Differences from the robot, on purpose:

* the target is a fixed point, not a moving path reference, so the position
  error is saturated at ``e_max`` instead (bounded approach speed
  ≈ kp·e_max/kd, the job the path's timing law does on the robot);
* the orientation reference is the pose at reset, held;
* it reads the true MuJoCo state (no estimator), like every policy here.

It returns the normalised action a = q̈_nom / q̈_max that env.step expects.
"""
from __future__ import annotations

import mujoco
import numpy as np


class CartesianPDBaseline:
    def __init__(self, env, kp=20.0, kd=9.0, kp_rot=5.0, kd_rot=6.0, k_null=5.0,
                 d_null=2.0, e_max=0.10, lambda_sq_min=1e-4, lambda_sq_max=5e-2,
                 manip_thr=0.05):
        self.env = env
        self.kp, self.kd, self.kp_rot, self.kd_rot = kp, kd, kp_rot, kd_rot
        self.k_null, self.d_null, self.e_max = k_null, d_null, e_max
        self.l_min, self.l_max, self.manip_thr = lambda_sq_min, lambda_sq_max, manip_thr
        m = env.model
        self._jp = np.zeros((3, m.nv))
        self._jr = np.zeros((3, m.nv))
        self.reset()

    def reset(self):
        """Call right after env.reset(): latches the posture and orientation references."""
        e = self.env
        self.q0 = e._q
        self.R0 = e.data.site_xmat[e._ee_site].reshape(3, 3).copy()
        self._J_prev = None

    def _jac(self):
        e = self.env
        mujoco.mj_jacSite(e.model, e.data, self._jp, self._jr, e._ee_site)
        return np.vstack([self._jp[:, e._dadr], self._jr[:, e._dadr]])

    def __call__(self, obs=None):
        e = self.env
        q, qd = e._q, e._qdot
        J = self._jac()
        Jdqd = (np.zeros(6) if self._J_prev is None
                else ((J - self._J_prev) / e.dt) @ qd)
        self._J_prev = J

        v = J @ qd
        ep = e._target - e._ee_pos()
        n = float(np.linalg.norm(ep))
        if n > self.e_max:
            ep *= self.e_max / n
        R = e.data.site_xmat[e._ee_site].reshape(3, 3)
        er = 0.5 * sum(np.cross(R[:, i], self.R0[:, i]) for i in range(3))
        xdd = np.concatenate([self.kp * ep - self.kd * v[:3],
                              self.kp_rot * er - self.kd_rot * v[3:]]) - Jdqd

        w = float(np.sqrt(max(np.linalg.det(J @ J.T), 0.0)))
        f = float(np.clip(1.0 - w / self.manip_thr, 0.0, 1.0)) ** 2
        lam = self.l_min + (self.l_max - self.l_min) * f
        Jp = J.T @ np.linalg.inv(J @ J.T + lam * np.eye(6))
        N = np.eye(len(q)) - Jp @ J
        qdd = Jp @ xdd + N @ (-self.k_null * (q - self.q0) - self.d_null * qd)
        return np.clip(qdd / e.qddot_max, -1.0, 1.0).astype(np.float32)
