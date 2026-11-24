"""Circular position tracking with a postural objective in the nullspace.

For G(q) = -0.5 * ||q - q0||^2, its gradient is q0 - q. The preferred
acceleration is kp_nullspace * (q0 - q) - kd_nullspace * dq, projected by
N = I - J_pinv @ J. Gravity is compensated by the simulator.
"""

import numpy as np

from fr3_sim.controllers.circle_tracking import DampedTrackingController as DampedCircleController
from fr3_sim.methods.circle_reference import CircleReference, CircularLapReference


class Controller(DampedCircleController):
    """Track the circle while attracting redundant motion towards initial_q."""

    def __init__(self, robot, params, dt):
        super().__init__(robot, params, dt)
        self.q0 = np.asarray(params['initial_q'], dtype=float).copy()
        if self.q0.shape != (7,) or not np.all(np.isfinite(self.q0)):
            raise ValueError('initial_q must contain seven finite joint angles')
        tracking = params.get('tracking', {})
        self.kp_nullspace = float(tracking.get('kp_nullspace', 25.0))
        if not np.isfinite(self.kp_nullspace) or self.kp_nullspace < 0.0:
            raise ValueError('kp_nullspace must be finite and nonnegative')
        # The parent already validates an explicit kd_nullspace. Only the
        # baseline's default differs: 10 [1/s], rather than the damping variant's 4.
        self.kd_nullspace = float(tracking.get('kd_nullspace', 10.0))
        if isinstance(self.reference, CircleReference):
            options = params.get('circle', {}).get('lap_return')
            if options is not None:
                self.reference = CircularLapReference(self.reference, self.q0, options)

    def update(self, t, q, dq):
        """Add the projected posture term; the parent supplies damping once."""
        reference = self.reference
        if isinstance(reference, CircularLapReference):
            reference.advance(t, q, dq)
        tau_damped = super().update(t, q, dq)
        J = self.robot.jacobian_pos(q)
        N = np.eye(7) - np.linalg.pinv(J) @ J
        qdd_posture = self.kp_nullspace * (N @ (self.q0 - np.asarray(q, dtype=float)))

        # Both logs contain the complete nullspace contribution, not only damping.
        self.log['qdd_null'] = self.log['qdd_null'] + qdd_posture
        self.log['qdd_command'] = self.log['qdd_command'] + qdd_posture
        if isinstance(reference, CircularLapReference):
            self.log.update(reference.diagnostics(t))
            if reference.failure_reason:
                self.failure_reason = reference.failure_reason
                self.done = True
        return tau_damped + self.robot.mass_matrix(q) @ qdd_posture
