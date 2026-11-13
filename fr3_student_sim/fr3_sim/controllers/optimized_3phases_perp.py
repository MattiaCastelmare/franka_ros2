"""Prepare impact posture, blend to compliance, then hold compliant contact.

Only the objectives and trial schedule differ from optimized_hqp_perp.
Reference/initialization, observer and numerical helpers are reused.
Phase-specific update orchestration is local; optimized_hqp is unchanged.
No true contact force or wall stiffness is available to this controller.
"""
from __future__ import annotations

import numpy as np

from fr3_sim.methods.hierarchical_qp import QPInfeasibleError
from fr3_sim.controllers.optimized_hqp_perp import Controller as PerpendicularController


class Controller(PerpendicularController):
    """Three monotone phases: approach=0, transition=1, confirmed contact=2."""

    def __init__(self, robot, params, dt):
        super().__init__(robot, params, dt)
        options = params.get('three_phases', {})
        defaults = {
            'transition_distance': 0.010,
            'contact_distance': 0.002,
            'contact_confirmation_s': 0.003,
            'contact_hold_duration': 0.15,
            'contact_timeout': 0.50,
            'contact_penetration': 0.0,
            'stiffness_normal': 300.0,
            'stiffness_tangent': 600.0,
            'damping_normal': 60.0,
            'damping_tangent': 120.0,
        }
        unknown = set(options) - set(defaults)
        if unknown:
            raise ValueError(f'unknown three_phases settings: {sorted(unknown)}')
        for name, default in defaults.items():
            raw = options.get(name, default)
            if isinstance(raw, (bool, np.bool_)):
                raise ValueError(f'three_phases.{name} must be a scalar, not boolean')
            value = float(raw)
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f'three_phases.{name} must be finite and nonnegative')
            setattr(self, name, value)
        if self.transition_distance <= self.contact_distance:
            raise ValueError('transition_distance must exceed contact_distance')
        if self.contact_confirmation_s <= 0.0 or self.contact_hold_duration <= 0.0:
            raise ValueError('contact confirmation and hold durations must be positive')
        if self.contact_timeout <= self.contact_confirmation_s:
            raise ValueError('contact_timeout must exceed contact_confirmation_s')
        if min(self.stiffness_normal, self.stiffness_tangent,
               self.damping_normal, self.damping_tangent) <= 0.0:
            raise ValueError('contact stiffness and damping must be positive')
        normal = self.reference.normal
        normal_projector = np.outer(normal, normal)
        tangent_projector = np.eye(3) - normal_projector
        self.contact_stiffness = (self.stiffness_normal * normal_projector
                                  + self.stiffness_tangent * tangent_projector)
        self.contact_damping = (self.damping_normal * normal_projector
                                + self.damping_tangent * tangent_projector)
        # Project the initial tool position onto the plane; an optional positive
        # depth moves this equilibrium inward. It never follows the 10 mm
        # nominal reference into the wall by accident.
        self.contact_p_ref = (self.reference.start_position
                              + self.reference.direction
                              * (self.reference.gap + self.contact_penetration))
        self.phase = 0
        self.contact_blend = 0.0
        self.contact_time = None
        self._contact_candidate_since = None
        self._phase_log = {}

    def _tracking_objectives(self, t, p_ref, v_ref, a_ref, p, velocity, W, snapshot):
        nominal = self.cartesian_tracking.acceleration(a_ref, p_ref - p, v_ref - velocity)
        gap = self.reference.gap + float(self.reference.normal @ (p - self.reference.start_position))
        # Distance, not nominal crossing time, anticipates the impact. Retain
        # the greatest blend reached, so rebound/noise never restarts the
        # posture search or switches the robot back to stiff tracking.
        progress = float(np.clip((self.transition_distance - gap)
                                 / (self.transition_distance - self.contact_distance), 0.0, 1.0))
        self.contact_blend = max(self.contact_blend, progress**2 * (3.0 - 2.0 * progress))
        residual_contact = (snapshot.valid and gap <= self.contact_distance
                            and bool(np.any(np.abs(snapshot.residual)
                                            > self.monitor.detector.threshold)))
        if self.contact_time is None:
            if residual_contact:
                if self._contact_candidate_since is None:
                    self._contact_candidate_since = float(t)
                if t - self._contact_candidate_since + 1e-12 >= self.contact_confirmation_s:
                    self.contact_time = float(t)
            else:
                self._contact_candidate_since = None
        self.phase = (2 if self.contact_time is not None else
                      1 if self.contact_blend > 0.0 else 0)
        force = (self.contact_stiffness @ (self.contact_p_ref - p)
                 - self.contact_damping @ velocity)
        contact_acceleration = W @ force
        activation = 1.0 - self.contact_blend
        acceleration = activation * nominal + self.contact_blend * contact_acceleration
        self._phase_log = {
            'phase': float(self.phase),
            'contact_blend': self.contact_blend,
            'wall_gap': gap,
            'contact_detected': float(self.contact_time is not None),
            'contact_latched': float(self.contact_time is not None),
            'contact_candidate': float(residual_contact),
            'contact_time': np.nan if self.contact_time is None else self.contact_time,
            'contact_p_ref': self.contact_p_ref.copy(),
            'contact_position_error': self.contact_p_ref - p,
            'nominal_tracking_acceleration': nominal.copy(),
            'contact_acceleration': contact_acceleration.copy(),
            'impedance_force': force.copy(),
        }
        return acceleration, activation

    def update(self, t, q, dq):
        """Keep all phase-specific orchestration in this experiment module."""
        q, dq = np.asarray(q, float), np.asarray(dq, float)
        p_ref, v_ref, a_ref = self.reference.sample(t)
        p, J = self.robot.position(q), self.robot.jacobian_pos(q)
        M, c = self.robot.mass_matrix(q), self.robot.coriolis(q, dq)
        gravity = self.robot.gravity(q)
        M_inv = np.linalg.solve(M, np.eye(7))
        W = J @ M_inv @ J.T
        # Keep the existing rank check for dynamically consistent redundancy.
        if np.linalg.eigvalsh(W)[0] <= 1e-8:
            raise ValueError('Cartesian inertia is singular at this configuration')
        Lambda = np.linalg.solve(W, np.eye(3))
        Jbar = M_inv @ J.T @ Lambda
        N = np.eye(7) - Jbar @ J
        velocity = J @ dq
        ep, ev = p_ref - p, v_ref - velocity
        # Blend phase objectives locally, before the unchanged numerical HQP.
        snapshot, residual, uncertainty = self._estimate(t, M_inv)
        tracking_acceleration, secondary_scale = self._tracking_objectives(
            t, p_ref, v_ref, a_ref, p, velocity, W, snapshot)
        if not np.isfinite(secondary_scale) or not 0.0 <= secondary_scale <= 1.0:
            raise ValueError('secondary objective activation must be in [0, 1]')
        task_target = tracking_acceleration - self.robot.jdot_dq(q, dq)
        posture = np.zeros(7) if self.q_posture is None else self.posture_gain * (self.q_posture - q)
        if secondary_scale > 0.0:
            mass_kg, gradient = self.mass_objective.evaluate(q, M=M, J=J, M_inv=M_inv)
        else:
            # Keep mass diagnostics without the analytical derivative work once
            # an experiment has disabled its mass/posture preference.
            mass_kg = 1.0 / float(self.mass_objective.normal @ W @ self.mass_objective.normal)
            gradient = np.zeros(7)
        # The torque projector is N.T; reuse the same dynamically consistent
        # projector instead of solving for it again in the mass utility.
        tau_mass = -self.mass_gain * (N.T @ gradient)
        # Activate secondary attraction smoothly at RUN startup. Damping,
        # Cartesian tracking and every safety constraint remain active.
        # Scale preferences before the QPs, never their constrained output.
        phase = (1.0 if self.startup_ramp_duration == 0.0 else
                 float(np.clip(t / self.startup_ramp_duration, 0.0, 1.0)))
        startup_scale = phase * phase * (3.0 - 2.0 * phase)
        posture = startup_scale * secondary_scale * posture
        tau_mass = startup_scale * secondary_scale * tau_mass
        secondary_target = (Jbar @ task_target
                            + N @ (posture - self.null_damping * dq)
                            + M_inv @ tau_mass)
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
            'secondary_scale': secondary_scale,
            'tau_mass': tau_mass.copy(), 'tracking_acceleration': tracking_acceleration.copy(),
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
        if self.constraints.soft_enabled:
            log.update({
                'soft_lower': bounds.soft_lower.copy(), 'soft_upper': bounds.soft_upper.copy(),
                'soft_scale': bounds.soft_scale.copy(),
                'soft_slack_lower': np.full((3, 7), np.nan),
                'soft_slack_upper': np.full((3, 7), np.nan),
                'soft_slack_max': np.nan, 'soft_slack_normalized_max': np.nan,
                'qp_safety_status': -1.0, 'qp_safety_iterations': 0.0, 'qp_safety_ms': 0.0,
            })
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
            preferences = ({'soft_lower': bounds.soft_lower, 'soft_upper': bounds.soft_upper,
                            'soft_scale': bounds.soft_scale}
                           if self.constraints.soft_enabled else {})
            result = self.qp.solve(J, task_target, secondary_target, M, c + 0.5 * gravity,
                                   bounds.lower, bounds.upper, torque_halfwidth, **preferences)
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
            if self.constraints.soft_enabled:
                used = np.maximum(result.soft_slack_lower, result.soft_slack_upper)
                log.update({
                    'soft_slack_lower': result.soft_slack_lower.copy(),
                    'soft_slack_upper': result.soft_slack_upper.copy(),
                    'soft_slack_max': float(np.max(used)),
                    'soft_slack_normalized_max': float(np.max(used / bounds.soft_scale)),
                    'qp_safety_status': result.safety_status,
                    'qp_safety_iterations': result.safety_iterations,
                    'qp_safety_ms': result.safety_ms,
                })
        log.update({'qdd_command': u.copy(),
                    'qdd_predicted': u + bounds.disturbance,
                    'tau_motor_predicted': tau + gravity,
                    'task_acceleration_error': J @ u - task_target})
        self.log = log
        distance = float(self.reference.direction @
                         (self.log['p_ref'] - self.reference.start_position))
        self.log.update(self._phase_log)
        self.log.update({
            'p_initial': self.reference.start_position.copy(),
            'approach_distance_ref': distance,
            'reference_depth': distance - self.reference.gap,
            'approach_complete': float(t >= self.reference.motion_end_time),
            'trial_complete': 0.0,
        })
        if not self.done:
            if self.contact_time is not None:
                end_time = max(self.reference.motion_end_time, self.contact_time) + self.contact_hold_duration
                if t >= end_time:
                    self.log['trial_complete'] = 1.0
                    self.done = True
            elif t >= self.reference.motion_end_time + self.contact_timeout:
                self.failure_reason = 'contact not confirmed before three-phase timeout'
                self.done = True
        return tau
