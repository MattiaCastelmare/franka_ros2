"""Perpendicular full-online mass reduction with one constrained QP."""
import numpy as np

from fr3_sim.controllers.base import StudentController
from fr3_sim.methods.cartesian_tracking import CartesianAccelerationTracking
from fr3_sim.methods.hierarchical_qp import QPInfeasibleError, WeightedAccelerationQP
from fr3_sim.methods.joint_constraints import JointLimits, vector
from fr3_sim.methods.mass_objective import DirectionalMassObjective
from fr3_sim.methods.perpendicular_reference import PerpendicularApproachReference
from fr3_sim.simulation.fr3_model import Q_MAX, Q_MIN


class Controller(StudentController):
    """Track the approach and reduce mass using current-state braking bounds."""

    def __init__(self, robot, params, dt):
        super().__init__(robot, params, dt)
        wall = params['wall_geometry']
        self.reference = PerpendicularApproachReference(
            robot.position(params['initial_q']), wall['point'], wall['normal'],
            **params.get('approach', {}),
        )
        tracking = params.get('tracking', {})
        self.tracking = CartesianAccelerationTracking(tracking.get('kp_position', 100.0),
                                                      tracking.get('kd_position', 20.0))
        self.null_damping = float(tracking.get('nullspace_damping', 12.0))
        self.ramp_duration = float(tracking.get('startup_ramp_duration', 0.1))
        mass = params['mass_minimization']
        self.mass_objective = DirectionalMassObjective(robot, mass['direction'])
        self.mass_gain = float(mass.get('gain', 4.0))
        for name, value in (('tracking.nullspace_damping', self.null_damping),
                            ('tracking.startup_ramp_duration', self.ramp_duration),
                            ('mass_minimization.gain', self.mass_gain)):
            if not np.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        self.limits = JointLimits(params.get('safety', {}))
        self.qp = WeightedAccelerationQP(7, params.get('qp', {}),
                                         tracking_weight=tracking.get('qp_weight', 1000.0))
        self.failure_damping = vector(params.get('failure_damping', [30, 30, 30, 20, 10, 10, 5]),
                                      7, 'failure_damping', positive=True)
        self.failure_reason = ''

    def update(self, t, q, dq):
        q, dq = np.asarray(q, dtype=float), np.asarray(dq, dtype=float)
        ref = self.reference
        p_ref, v_ref, a_ref = ref.sample(t)
        p, J = self.robot.position(q), self.robot.jacobian_pos(q)
        M, c = self.robot.mass_matrix(q), self.robot.coriolis(q, dq)
        gravity = self.robot.gravity(q)
        M_inv = np.linalg.solve(M, np.eye(7))
        W = J @ M_inv @ J.T
        if np.linalg.eigvalsh(W)[0] <= 1e-8:
            raise ValueError('Cartesian inertia is singular at this configuration')
        Jbar = np.linalg.solve(W, J @ M_inv).T
        N = np.eye(7) - Jbar @ J

        ep, ev = p_ref - p, v_ref - J @ dq
        acceleration = self.tracking.acceleration(a_ref, ep, ev)
        target = acceleration - self.robot.jdot_dq(q, dq)
        mass, gradient = self.mass_objective.evaluate(q, J=J, M_inv=M_inv)
        phase = (1.0 if self.ramp_duration == 0 else
                 float(np.clip(t / self.ramp_duration, 0.0, 1.0)))
        startup_scale = phase**2 * (3.0 - 2.0 * phase)
        tau_mass = -startup_scale * self.mass_gain * (N.T @ gradient)
        preference = Jbar @ target - self.null_damping * (N @ dq) + M_inv @ tau_mass

        lower, upper = self.limits.braking_bounds(q, dq)
        # Limit the published torque and the nominal motor torque including gravity.
        halfwidth = self.limits.torque - 0.5 * np.abs(gravity)
        if np.any(halfwidth <= 0):
            raise QPInfeasibleError('no nonzero-width command/motor torque intersection')
        try:
            result = self.qp.solve(J, target, preference, M, c + 0.5 * gravity,
                                   lower, upper, halfwidth)
        except QPInfeasibleError as exc:
            self.failure_reason = str(exc)
            self.done = True
            tau = np.clip(-self.failure_damping * dq + 0.5 * gravity, -halfwidth, halfwidth)
            tau -= 0.5 * gravity
            u = M_inv @ (tau - c)
            status, iterations, milliseconds, violation = -1, 0, 0.0, np.nan
        else:
            u = result.acceleration
            tau = M @ u + c
            status, iterations = result.status, result.iterations
            milliseconds, violation = result.milliseconds, result.max_violation

        trial_complete = not self.done and t >= ref.end_time
        if trial_complete:
            self.done = True
        distance = float(ref.direction @ (p_ref - ref.start_position))
        self.log = {
            'p_ref': p_ref, 'v_ref': v_ref, 'a_ref': a_ref,
            'position_error': ep, 'velocity_error': ev, 'tracking_acceleration': acceleration,
            'mass_kg': mass, 'mass_gradient': gradient, 'tau_mass': tau_mass,
            'startup_scale': startup_scale, 'normal_speed': float(-self.mass_objective.normal @ (J @ dq)),
            'normal_speed_ref': float(-self.mass_objective.normal @ v_ref),
            'p_initial': ref.start_position, 'approach_distance_ref': distance,
            'reference_depth': distance - ref.gap, 'approach_complete': float(t >= ref.motion_end_time),
            'trial_complete': float(trial_complete), 'joint_limit_margin': np.minimum(q - Q_MIN, Q_MAX - q),
            'qdd_lower': lower, 'qdd_upper': upper, 'qdd_command': u,
            'task_acceleration_error': J @ u - target, 'tau_motor_predicted': tau + gravity,
            'qp_status': status, 'qp_iterations': iterations, 'qp_ms': milliseconds,
            'qp_constraint_violation': violation,
        }
        return tau
