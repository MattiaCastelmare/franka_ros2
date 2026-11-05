"""Circular tracking with nullspace velocity damping.

Reuse the minimal controller without changing its Cartesian tracking law:
    N = I - J_pinv J
    qdd_null = -kd_nullspace N dq
    tau = tau_minimal + M(q) qdd_null

``tracking.kd_nullspace`` is in 1/s. Zero recovers the minimal controller.
"""

import numpy as np

from controller import Controller as MinimalCircleController


class Controller(MinimalCircleController):
    """Add only velocity damping in the position Jacobian's null space."""

    def __init__(self, robot, params, dt):
        super().__init__(robot, params, dt)
        self.kd_nullspace = float(params.get('tracking', {}).get('kd_nullspace', 4.0))
        if not np.isfinite(self.kd_nullspace) or self.kd_nullspace < 0.0:
            raise ValueError('kd_nullspace must be finite and nonnegative')

    def update(self, t, q, dq):
        tau_minimal = super().update(t, q, dq)
        J = self.robot.jacobian_pos(q)
        N = np.eye(7) - np.linalg.pinv(J) @ J
        dq_null = N @ dq
        qdd_null = -self.kd_nullspace * dq_null

        self.log['qdd_command'] = self.log['qdd_command'] + qdd_null
        self.log['dq_null'] = dq_null.copy()
        self.log['qdd_null'] = qdd_null.copy()
        return tau_minimal + self.robot.mass_matrix(q) @ qdd_null
