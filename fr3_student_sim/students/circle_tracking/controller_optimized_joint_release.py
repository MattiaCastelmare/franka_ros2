"""Optimized circle followed by latched damping near a joint-position limit.

The guard uses measured q and dq only, with a linear lookahead towards each
physical position bound. It does not use the collision detector or true wall
force. RELEASE disables all tracking, posture and mass objectives, and keeps
only joint damping. Gazebo continues to compensate gravity.
"""

import numpy as np

from fr3_student_sim.fr3_model import Q_MAX, Q_MIN

from controller_optimized import Controller as OptimizedController


class Controller(OptimizedController):
    """Keep optimized unchanged until the joint guard selects RELEASE once."""

    def __init__(self, robot, params, dt):
        super().__init__(robot, params, dt)
        release = params.get('joint_release', {})
        self.joint_margin = float(release.get('margin', 0.10))
        self.lookahead = float(release.get('lookahead', 0.10))
        # The existing runner stops within 0.05 rad of a physical bound.
        if (not np.isfinite(self.joint_margin) or self.joint_margin <= 0.05
                or np.any(2.0 * self.joint_margin >= Q_MAX - Q_MIN)):
            raise ValueError('joint_release.margin must exceed 0.05 rad and fit the joint ranges')
        if not np.isfinite(self.lookahead) or self.lookahead < 0.0:
            raise ValueError('joint_release.lookahead must be finite and nonnegative')
        damping = np.asarray(
            release.get('damping', [30.0, 30.0, 30.0, 20.0, 10.0, 10.0, 5.0]),
            dtype=float)
        if damping.ndim == 0:
            damping = np.full(7, float(damping))
        if (damping.shape != (7,) or not np.all(np.isfinite(damping))
                or np.any(damping <= 0.0)):
            raise ValueError('joint_release.damping must be a positive scalar or seven positive values')
        self.release_damping = damping.copy()  # [N m s/rad], direct torque damping.
        self.release_start_time = None
        self.release_joint = 0  # One-based joint number after the first trigger.

    def _joint_margins(self, q, dq):
        """Return current and linearly predicted distances to physical limits.

        Motion away from a bound does not enlarge the current safety margin.
        Lookahead is a trigger heuristic, not a guaranteed stopping distance.
        """
        lower = q - Q_MIN
        upper = Q_MAX - q
        predicted_lower = lower - self.lookahead * np.maximum(-dq, 0.0)
        predicted_upper = upper - self.lookahead * np.maximum(dq, 0.0)
        return np.minimum(lower, upper), np.minimum(predicted_lower, predicted_upper)

    def update(self, t, q, dq):
        """Return optimized torque or pure release damping, without gravity."""
        q, dq = np.asarray(q, dtype=float), np.asarray(dq, dtype=float)
        if (q.shape != (7,) or dq.shape != (7,)
                or not np.all(np.isfinite(q)) or not np.all(np.isfinite(dq))):
            raise ValueError('joint release requires finite seven-joint q and dq')
        margin, predicted = self._joint_margins(q, dq)
        if self.release_start_time is None and np.any(predicted <= self.joint_margin):
            self.release_start_time = float(t)
            self.release_joint = int(np.argmin(predicted)) + 1

        active = self.release_start_time is not None
        if active:
            torque = -self.release_damping * dq
            self._log_release(q, dq)
        else:
            torque = super().update(t, q, dq)

        self.log.update({
            'release_active': float(active),
            'release_start_time': self.release_start_time if active else np.nan,
            'release_joint': float(self.release_joint),
            'tau_release': torque.copy() if active else np.zeros(7),
            'joint_limit_margin': margin.copy(),
            'joint_limit_predicted_margin': predicted.copy(),
        })
        # No print in this critical path, and no rearming or done=True. The
        # existing runner duration and safety checks still terminate the run.
        return torque

    def _log_release(self, q, dq):
        """Keep mass and velocity diagnostics without evaluating a gradient."""
        J = self.robot.jacobian_pos(q)
        M = self.robot.mass_matrix(q)
        normal = self.mass_direction
        J_n = normal @ J
        inverse_mass = float(J_n @ np.linalg.solve(M, J_n))
        self.log = {
            'circle_center': self.reference.center.copy(),
            'p_initial': self.p_initial.copy(),
            'mass_kg': 1.0 / inverse_mass if inverse_mass > 0.0 else np.inf,
            'tau_mass': np.zeros(7),
            'normal_speed': float(-normal @ (J @ dq)),
            'normal_speed_ref': np.nan,
        }
        # No active task reference or acceleration command exists in RELEASE.
        for key in ('p_ref', 'v_ref', 'a_ref', 'position_error', 'velocity_error'):
            self.log[key] = np.full(3, np.nan)
        for key in ('qdd_command', 'qdd_null', 'dq_null', 'mass_gradient'):
            self.log[key] = np.full(7, np.nan)
