"""Straight position tracking with projected directional-mass minimization.

Only the fingertip's three position coordinates are controlled. Orientation
is free. Gravity compensation, torque saturation, collision monitoring and
the existing safety stops remain the runner's responsibility.
"""

import numpy as np

from fr3_student_sim.controller_base import StudentController
from utils.mass_objective import DirectionalMassObjective, projected_mass_torque
from straight_line_reference import StraightLineReference


class Controller(StudentController):
    """Keep Cartesian tracking, nullspace damping and mass objective explicit."""

    def __init__(self, robot, params, dt):
        super().__init__(robot, params, dt)
        self.p_initial = robot.position(params['initial_q'])
        geometry = params['wall_geometry']  # Static geometry supplied by the runner.
        mass = params.get('mass_minimization', {})
        self.mass_objective = DirectionalMassObjective(
            robot, geometry['normal'], mass.get('gradient_step', 1e-4))
        self.mass_gain = float(mass.get('gain', 0.01))
        if not np.isfinite(self.mass_gain) or self.mass_gain < 0.0:
            raise ValueError('mass_minimization.gain must be finite and nonnegative')

        point = np.asarray(geometry['point'], dtype=float)
        if point.shape != (3,) or not np.all(np.isfinite(point)):
            raise ValueError('wall point must contain three finite values')
        normal = self.mass_objective.normal
        wall_distance = float(normal @ (self.p_initial - point))
        line = params.get('line', {})
        self.reference = StraightLineReference(
            self.p_initial, -normal, wall_distance,
            approach_speed=line.get('approach_speed', 0.02),
            ramp_time=line.get('ramp_time', 0.5),
            penetration=line.get('penetration', 0.001))
        self.hold_time = float(line.get('hold_time', 0.2))
        if not np.isfinite(self.hold_time) or self.hold_time < 0.0:
            raise ValueError('line.hold_time must be finite and nonnegative')
        self.end_time = self.reference.move_duration + self.hold_time

        tracking = params.get('tracking', {})
        self.kp_position = float(tracking.get('kp_position', 100.0))
        self.kd_position = float(tracking.get('kd_position', 20.0))
        self.kd_nullspace = float(tracking.get('kd_nullspace', 4.0))
        for name in ('kp_position', 'kd_position', 'kd_nullspace'):
            value = getattr(self, name)
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f'tracking.{name} must be finite and nonnegative')

    def update(self, t, q, dq):
        """Return requested total torque [N m], without gravity compensation."""
        p_ref, v_ref, a_ref = self.reference.sample(t)
        p = self.robot.position(q)
        J = self.robot.jacobian_pos(q)
        M = self.robot.mass_matrix(q)
        position_error = p_ref - p
        velocity = J @ dq
        velocity_error = v_ref - velocity
        a = (a_ref + self.kp_position * position_error
             + self.kd_position * velocity_error)

        # The same acceleration law and nullspace damping as the circle variant.
        J_pinv = np.linalg.pinv(J)
        N = np.eye(M.shape[0]) - J_pinv @ J
        dq_null = N @ dq
        qdd_null = -self.kd_nullspace * dq_null
        qdd_tracking = J_pinv @ (a - self.robot.jdot_dq(q, dq)) + qdd_null
        tau_tracking = M @ qdd_tracking + self.robot.coriolis(q, dq)

        # The direction and contact point are fixed; no true external force is used.
        mass_kg, gradient = self.mass_objective.evaluate(q)
        tau_mass = projected_mass_torque(M, J, gradient, self.mass_gain)
        qdd_mass = np.linalg.solve(M, tau_mass)
        self.log = {
            'p_ref': p_ref.copy(),
            'v_ref': v_ref.copy(),
            'a_ref': a_ref.copy(),
            'position_error': position_error.copy(),
            'velocity_error': velocity_error.copy(),
            'qdd_command': qdd_tracking + qdd_mass,
            'dq_null': dq_null.copy(),
            'qdd_null': qdd_null.copy(),
            'mass_kg': mass_kg,
            'mass_gradient': gradient.copy(),
            'tau_mass': tau_mass.copy(),  # Requested component, before total-torque clipping.
            'normal_speed': float(self.reference.direction @ velocity),
            'normal_speed_ref': float(self.reference.direction @ v_ref),
        }
        # End at a scheduled time, regardless of detector output or actual contact.
        self.done = bool(t >= self.end_time)
        return tau_tracking + tau_mass
