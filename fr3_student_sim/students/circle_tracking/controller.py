"""Position tracking with the Cartesian inverse-dynamics law from the lectures.

Robotics 2: 13_CartesianControl, slides 8 and 11; and
02_KinematicRedundancy_2, slides 7 and 13.
Gravity is already compensated by the simulator.
"""

import numpy as np

from fr3_student_sim.controller_base import StudentController
from reference import CircleReference


class Controller(StudentController):
    """Track the fingertip position, with velocity damping in the null space."""

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
        self.kd_nullspace = float(tracking.get('kd_nullspace', 4.0))

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

        # Redundant inverse kinematics (lecture 02, slide 7), damping only.
        J_pinv = np.linalg.pinv(J)
        N = np.eye(7) - J_pinv @ J
        qdd_command = (
            J_pinv @ (a - self.robot.jdot_dq(q, dq))
            - self.kd_nullspace * (N @ dq)
        )
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
