"""Natural-inertia Cartesian impedance with prioritized constrained redundancy.

Robotics 2, 15_ImpedanceControl, slide 13, extended to a redundant position
task. This is not a proof of passivity for the variable-inertia, sampled HQP
closed loop (see also slide 14). The observer affects safety predictions only;
it is never subtracted from the impedance command.
"""

import numpy as np

from fr3_sim.controllers.base import StudentController

from fr3_sim.methods.hierarchical_qp import HierarchicalAccelerationQP, QPInfeasibleError
from fr3_sim.methods.joint_constraints import JointConstraints, vector
from fr3_sim.methods.mass_objective import DirectionalMassObjective, projected_mass_torque


class ImpedanceHQPController(StudentController):
    """Controller independent of trajectory shape and environment geometry.

    ``reference.sample(t)`` supplies world position, velocity and acceleration.
    The fixed world mass direction concerns the controlled point's inertia,
    not an assumption about the unknown point where external torques act.
    """

    def __init__(self, robot, params, dt, reference):
        super().__init__(robot, params, dt)
        if not np.isfinite(dt) or dt <= 0:
            raise ValueError('control period must be positive and finite')
        self.reference = reference
        self.monitor = None
        impedance = params.get('impedance', {})
        self.stiffness = vector(impedance.get('stiffness', 600.0), 3,
                                'impedance.stiffness', positive=True)
        self.damping = vector(impedance.get('damping', 120.0), 3,
                              'impedance.damping', positive=True)
        # A scalar keeps M N k symmetric positive semidefinite. Arbitrary
        # diagonal acceleration gains would not generally remain dissipative.
        self.null_damping = float(impedance.get('nullspace_damping', 10.0))
        if not np.isfinite(self.null_damping) or self.null_damping < 0:
            raise ValueError('impedance.nullspace_damping must be a nonnegative finite scalar')
        self.posture_gain = float(impedance.get('posture_gain', 25.0))
        self.startup_ramp_duration = float(impedance.get('startup_ramp_duration', 0.5))
        if not np.isfinite(self.startup_ramp_duration) or self.startup_ramp_duration < 0:
            raise ValueError('impedance.startup_ramp_duration must be finite and nonnegative')
        self.q_posture = params.get('q_posture_ref')
        self.q_posture = (None if self.q_posture is None else
                          vector(self.q_posture, 7, 'q_posture_ref'))
        mass = params.get('mass_minimization', {})
        # Explicit task direction: changing the object needs no force reconstruction.
        self.mass_objective = DirectionalMassObjective(robot, mass['direction'])
        self.mass_gain = float(mass.get('gain', 0.1))
        if not np.isfinite(self.mass_gain) or self.mass_gain < 0:
            raise ValueError('mass_minimization.gain must be finite and nonnegative')
        if not np.isfinite(self.posture_gain) or self.posture_gain < 0:
            raise ValueError('impedance.posture_gain must be finite and nonnegative')
        self.constraints = JointConstraints(params.get('safety', {}))
        if self.q_posture is not None and (np.any(self.q_posture < self.constraints.q_lower)
                                           or np.any(self.q_posture > self.constraints.q_upper)):
            raise ValueError('q_posture_ref must respect configured position margins')
        self.qp = HierarchicalAccelerationQP(7, params.get('qp', {}))
        estimate = params.get('disturbance_estimate', {})
        self.snapshot_options = {name: float(estimate.get(name, default)) for name, default in
                                 (('warmup_s', 0.2), ('max_gap_s', 0.01), ('max_age_s', 0.01))}
        self.error_floor = vector(estimate.get('acceleration_error', 0.5), 7,
                                  'disturbance_estimate.acceleration_error', nonnegative=True)
        self.invalid_error = vector(estimate.get('invalid_acceleration_error', 1.0), 7,
                                    'disturbance_estimate.invalid_acceleration_error', positive=True)
        self.lag_reserve = float(estimate.get('lag_reserve_factor', 1.0))
        if not np.isfinite(self.lag_reserve) or self.lag_reserve < 0:
            raise ValueError('disturbance_estimate.lag_reserve_factor must be nonnegative')
        self.failure_damping = vector(params.get('failure_damping', [30, 30, 30, 20, 10, 10, 5]),
                                      7, 'failure_damping', positive=True)
        self.last_t = None
        self.last_residual = None
        self.last_residual_t = None
        self.failure_reason = ''

    def bind_collision_monitor(self, monitor):
        if not monitor.enabled:
            raise ValueError('impedance HQP requires observer:=true')
        self.monitor = monitor

    def _estimate(self, t, M_inv):
        if self.monitor is None:
            raise RuntimeError('bind_collision_monitor must be called before update')
        snapshot = self.monitor.snapshot(t, **self.snapshot_options)
        residual = snapshot.residual.copy() if snapshot.valid else np.zeros(7)
        uncertainty = self.error_floor.copy() if snapshot.valid else self.invalid_error.copy()
        # The derivative only enlarges an error reserve; it never extrapolates
        # or deconvolves r and never becomes an actuator compensation term.
        if snapshot.valid and self.last_residual is not None:
            elapsed = snapshot.timestamp - self.last_residual_t
            if elapsed > 0:
                rate = np.abs((residual - self.last_residual) / elapsed)
                lag = (snapshot.filter_time_constant_s + max(snapshot.age_s, 0)
                       + self.constraints.delay)
                uncertainty += self.lag_reserve * (np.abs(M_inv) @ (lag * rate))
        self.last_residual = residual.copy() if snapshot.valid else None
        self.last_residual_t = snapshot.timestamp if snapshot.valid else None
        return snapshot, residual, uncertainty

    def update(self, t, q, dq):
        q, dq = np.asarray(q, float), np.asarray(dq, float)
        p_ref, v_ref, a_ref = self.reference.sample(t)
        p, J = self.robot.position(q), self.robot.jacobian_pos(q)
        M, c = self.robot.mass_matrix(q), self.robot.coriolis(q, dq)
        gravity = self.robot.gravity(q)
        M_inv = np.linalg.solve(M, np.eye(7))
        W = J @ M_inv @ J.T
        # Do not silently regularize a singular task into another impedance.
        if np.linalg.eigvalsh(W)[0] <= 1e-8:
            raise ValueError('Cartesian inertia is singular at this configuration')
        Lambda = np.linalg.solve(W, np.eye(3))
        Jbar = M_inv @ J.T @ Lambda
        N = np.eye(7) - Jbar @ J
        ep, ev = p_ref - p, v_ref - J @ dq
        impedance_force = self.stiffness * ep + self.damping * ev
        task_target = a_ref - self.robot.jdot_dq(q, dq) + W @ impedance_force
        posture = np.zeros(7) if self.q_posture is None else self.posture_gain * (self.q_posture - q)
        mass_kg, gradient = self.mass_objective.evaluate(q, M=M, J=J)
        tau_mass = projected_mass_torque(M, J, gradient, self.mass_gain)
        # Activate secondary attraction smoothly at RUN startup. Damping,
        # Cartesian impedance and every safety constraint remain active.
        # Scale preferences before the QPs, never their constrained output.
        phase = (1.0 if self.startup_ramp_duration == 0.0 else
                 float(np.clip(t / self.startup_ramp_duration, 0.0, 1.0)))
        startup_scale = phase * phase * (3.0 - 2.0 * phase)
        posture = startup_scale * posture
        tau_mass = startup_scale * tau_mass
        secondary_target = (Jbar @ task_target
                            + N @ (posture - self.null_damping * dq)
                            + M_inv @ tau_mass)
        snapshot, residual, uncertainty = self._estimate(t, M_inv)
        elapsed = self.dt if self.last_t is None else float(t) - self.last_t
        self.last_t = float(t)
        # An observed scheduling gap lengthens the prediction interval.
        horizon = max(self.dt, elapsed)
        bounds = self.constraints.bounds(q, dq, M_inv, c, self.tau_applied,
                                          residual, uncertainty, horizon)
        log = {
            'p_ref': p_ref.copy(), 'v_ref': v_ref.copy(), 'a_ref': a_ref.copy(),
            'position_error': ep.copy(), 'velocity_error': ev.copy(),
            'mass_kg': mass_kg, 'mass_gradient': gradient.copy(),
            'startup_scale': startup_scale,
            'tau_mass': tau_mass.copy(), 'impedance_force': impedance_force.copy(),
            'normal_speed': float(-self.mass_objective.normal @ (J @ dq)),
            'normal_speed_ref': float(-self.mass_objective.normal @ v_ref),
            'observer_valid': float(snapshot.valid),
            'observer_timestamp': np.nan if snapshot.timestamp is None else snapshot.timestamp,
            'observer_age': snapshot.age_s,
            'observer_sample_interval': (np.nan if snapshot.sample_interval_s is None
                                         else snapshot.sample_interval_s),
            'residual_used': residual.copy(),
            'acceleration_uncertainty': uncertainty.copy(),
            'joint_limit_margin': bounds.margin.copy(),
            'qdd_lower': bounds.lower.copy(), 'qdd_upper': bounds.upper.copy(),
        }
        try:
            if snapshot.reason not in ('ok', 'initializing', 'warming_up'):
                raise QPInfeasibleError('invalid disturbance snapshot: ' + snapshot.reason)
            if elapsed > self.snapshot_options['max_gap_s']:
                raise QPInfeasibleError(f'controller sample gap: {elapsed:.6g} s')
            # Bound both the runner's command and nominal total motor torque.
            # |tau|<=T AND |tau+g|<=T is exactly
            # |M u+c+g/2| <= T-|g|/2. Gravity is not added to the output.
            torque_halfwidth = self.constraints.torque - 0.5 * np.abs(gravity)
            if np.any(torque_halfwidth <= 0):
                raise QPInfeasibleError('no nonzero-width command/motor torque intersection')
            result = self.qp.solve(J, task_target, secondary_target, M, c + 0.5 * gravity,
                                   bounds.lower, bounds.upper, torque_halfwidth)
        except QPInfeasibleError as exc:
            # No stale command or silent relaxation of safety constraints.
            # This bounded damping is a termination fallback, not a safety proof.
            self.failure_reason = str(exc)
            self.done = True
            torque_lower = np.maximum(-self.constraints.torque,
                                      -self.constraints.torque - gravity)
            torque_upper = np.minimum(self.constraints.torque,
                                      self.constraints.torque - gravity)
            if np.any(torque_lower > torque_upper):
                raise RuntimeError(self.failure_reason)
            tau = np.clip(-self.failure_damping * dq, torque_lower, torque_upper)
            u = M_inv @ (tau - c)
            log.update({'qp_primary_status': -1.0, 'qp_secondary_status': -1.0,
                        'qp_constraint_violation': float('nan'),
                        'qp_primary_iterations': 0.0, 'qp_secondary_iterations': 0.0,
                        'qp_primary_ms': 0.0, 'qp_secondary_ms': 0.0})
        else:
            u = result.acceleration
            tau = M @ u + c
            log.update({'qp_primary_status': result.primary_status,
                        'qp_secondary_status': result.secondary_status,
                        'qp_constraint_violation': result.max_violation,
                        'qp_primary_iterations': result.primary_iterations,
                        'qp_secondary_iterations': result.secondary_iterations,
                        'qp_primary_ms': result.primary_ms, 'qp_secondary_ms': result.secondary_ms})
        log.update({'qdd_command': u.copy(),
                    'qdd_predicted': u + bounds.disturbance,
                    'tau_motor_predicted': tau + gravity,
                    'task_acceleration_error': J @ u - task_target})
        self.log = log
        return tau
