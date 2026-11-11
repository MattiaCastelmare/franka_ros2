"""Circular position tracking and nullspace damping used by the baseline.

Robotics 2: 13_CartesianControl, slides 8 and 11; and
02_KinematicRedundancy_2, slides 7 and 13.
Gravity is already compensated by the simulator.
"""

import numpy as np

from fr3_sim.controllers.base import StudentController

from fr3_sim.methods.circle_reference import CircleReference


class PositionTrackingController(StudentController):
    """Track the fingertip position with minimum-norm joint acceleration."""

    def __init__(self, robot, params, dt):
        super().__init__(robot, params, dt)
        self.p_initial = robot.position(params['initial_q'])
        circle = params.get('circle', {})
        self.reference = CircleReference(
            self.p_initial,
            radius=circle.get('radius', 0.04),
            period=circle.get('period', 8.0),
            ramp=circle.get('ramp', 1.0),
        )
        tracking = params.get('tracking', {})
        self.kp_position = float(tracking.get('kp_position', 100.0))
        self.kd_position = float(tracking.get('kd_position', 20.0))

    def update(self, t, q, dq):
        """Return joint torques without gravity compensation."""
        p_ref, v_ref, a_ref = self.reference.sample(t)
        p = self.robot.position(q)
        J = self.robot.jacobian_pos(q)
        position_error = p_ref - p
        velocity_error = v_ref - J @ dq

        # Cartesian PD with acceleration feedforward (lecture 13, slide 11).
        a = (a_ref + self.kp_position * position_error
             + self.kd_position * velocity_error)

        # Minimum-norm solution (lecture 02, slide 7), with qdd_0 = 0.
        J_pinv = np.linalg.pinv(J)
        qdd_command = J_pinv @ (a - self.robot.jdot_dq(q, dq))
        torque = self.robot.mass_matrix(q) @ qdd_command + self.robot.coriolis(q, dq)

        # Each array is a snapshot: the runner retains these values until saving.
        self.log = {
            'p_ref': p_ref.copy(),
            'v_ref': v_ref.copy(),
            'a_ref': a_ref.copy(),
            'circle_center': self.reference.center.copy(),
            'p_initial': self.p_initial.copy(),
            'position_error': position_error.copy(),
            'velocity_error': velocity_error.copy(),
            'qdd_command': qdd_command.copy(),
        }
        return torque


class DampedTrackingController(PositionTrackingController):
    """Add only velocity damping in the position Jacobian's null space."""

    def __init__(self, robot, params, dt):
        super().__init__(robot, params, dt)
        self.kd_nullspace = float(params.get('tracking', {}).get('kd_nullspace', 4.0))
        if not np.isfinite(self.kd_nullspace) or self.kd_nullspace < 0.0:
            raise ValueError('kd_nullspace must be finite and nonnegative')
        normal = np.asarray(params['wall_geometry']['normal'], dtype=float)
        if normal.shape != (3,) or not np.all(np.isfinite(normal)):
            raise ValueError('wall normal must contain three finite values')
        length = np.linalg.norm(normal)
        if not np.isfinite(length) or length <= 0.0:
            raise ValueError('wall normal must have a finite, nonzero norm')
        self.mass_direction = normal / length

    def update(self, t, q, dq):
        tau_minimal = super().update(t, q, dq)
        J = self.robot.jacobian_pos(q)
        N = np.eye(7) - np.linalg.pinv(J) @ J
        dq_null = N @ dq
        qdd_null = -self.kd_nullspace * dq_null
        M = self.robot.mass_matrix(q)

        self.log['qdd_command'] = self.log['qdd_command'] + qdd_null
        self.log['dq_null'] = dq_null.copy()
        self.log['qdd_null'] = qdd_null.copy()

        # m_n = 1 / (n.T J M^-1 J.T n), reusing the tracking matrices.
        # This adds no mass-gradient torque or mass-minimization objective.
        J_n = self.mass_direction @ J
        inverse_mass = float(J_n @ np.linalg.solve(M, J_n))
        self.log['mass_kg'] = 1.0 / inverse_mass if inverse_mass > 0.0 else np.inf
        return tau_minimal + M @ qdd_null
