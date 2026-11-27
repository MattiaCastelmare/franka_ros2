"""Cartesian tracking and online mass reduction with one constrained QP."""
import numpy as np

from fr3_sim.controllers.base import StudentController
from fr3_sim.methods.cartesian_tracking import CartesianAccelerationTracking
from fr3_sim.methods.hierarchical_qp import QPInfeasibleError, WeightedAccelerationQP
from fr3_sim.methods.joint_constraints import JointLimits, vector
from fr3_sim.methods.mass_objective import DirectionalMassObjective
from fr3_sim.simulation.fr3_model import Q_MAX, Q_MIN


class WeightedQPTrackingController(StudentController):
    """Track a supplied reference with current-state joint braking bounds."""

    def __init__(self, robot, params, dt, reference, *,
                 default_damping=10.0, default_ramp=0.5, default_mass_gain=0.1):
        super().__init__(robot, params, dt)
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('control period must be positive and finite')
        self.reference = reference
        tracking = params.get('tracking', {})
        self.tracking = CartesianAccelerationTracking(tracking.get('kp_position', 100.0),
                                                      tracking.get('kd_position', 20.0))
        self.null_damping = float(tracking.get('nullspace_damping', default_damping))
        self.startup_ramp_duration = float(tracking.get('startup_ramp_duration', default_ramp))
        self.q_posture = None
        self.posture_gain = 0.0
        mass = params['mass_minimization']
        self.mass_objective = DirectionalMassObjective(robot, mass['direction'])
        self.mass_gain = float(mass.get('gain', default_mass_gain))
        for name, value in (('tracking.nullspace_damping', self.null_damping),
                            ('tracking.startup_ramp_duration', self.startup_ramp_duration),
                            ('mass_minimization.gain', self.mass_gain)):
            if not np.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        safety = params.get('safety', {})
        self.limits = JointLimits(safety)
        self.braking_buffer = vector(safety.get('braking_buffer', 0.0), 7,
                                     'safety.braking_buffer', nonnegative=True)
        if np.any(2 * self.braking_buffer >= self.limits.q_upper - self.limits.q_lower):
            raise ValueError('safety.braking_buffer must fit within the configured joint range')
        self.qp = WeightedAccelerationQP(7, params.get('qp', {}),
                                         tracking_weight=tracking.get('qp_weight', 1000.0))
        self.failure_damping = vector(params.get('failure_damping', [30, 30, 30, 20, 10, 10, 5]),
                                      7, 'failure_damping', positive=True)
        self.failure_reason = ''

    def update(self, t, q, dq):
        q, dq = np.asarray(q, dtype=float), np.asarray(dq, dtype=float)
        p_ref, v_ref, a_ref = self.reference.sample(t)
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
        ramp = self.startup_ramp_duration
        phase = 1.0 if ramp == 0 else float(np.clip(t / ramp, 0.0, 1.0))
        startup_scale = phase**2 * (3.0 - 2.0 * phase)
        tau_mass = -startup_scale * self.mass_gain * (N.T @ gradient)
        preference = Jbar @ target - self.null_damping * (N @ dq) + M_inv @ tau_mass
        if self.q_posture is not None:
            preference += self.posture_gain * (N @ (self.q_posture - q))

        lower, upper = self.limits.braking_bounds(q, dq, position_buffer=self.braking_buffer)
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

        self.log = {
            'p_ref': p_ref, 'v_ref': v_ref, 'a_ref': a_ref,
            'position_error': ep, 'velocity_error': ev, 'tracking_acceleration': acceleration,
            'mass_kg': mass, 'mass_gradient': gradient, 'tau_mass': tau_mass,
            'startup_scale': startup_scale, 'normal_speed': float(-self.mass_objective.normal @ (J @ dq)),
            'normal_speed_ref': float(-self.mass_objective.normal @ v_ref),
            'joint_limit_margin': np.minimum(q - Q_MIN, Q_MAX - q),
            'qdd_lower': lower, 'qdd_upper': upper, 'qdd_command': u,
            'task_acceleration_error': J @ u - target, 'tau_motor_predicted': tau + gravity,
            'qp_status': status, 'qp_iterations': iterations, 'qp_ms': milliseconds,
            'qp_constraint_violation': violation,
        }
        return tau
