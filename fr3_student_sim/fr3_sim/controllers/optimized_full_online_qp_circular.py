"""Online mass descent for one circle, then a physical return to initial_q."""
from __future__ import annotations

import numpy as np

from fr3_sim.controllers.weighted_qp_tracking import WeightedQPTrackingController
from fr3_sim.methods.circle_reference import CircleReference, CircularLapReference
from fr3_sim.methods.joint_constraints import vector


class Controller(WeightedQPTrackingController):
    """Alternate a pure online circle and a constrained return to initial_q.

    Both phases use the same QP with joint braking and torque limits.
    """

    def __init__(self, robot, params, dt):
        online_params = dict(params)
        online_params.pop('q_posture_ref', None)
        online_params['tracking'] = dict(params.get('tracking', {}))
        online_params['tracking']['posture_gain'] = 0.0
        # Older circular YAMLs can supply their hard position margin and OSQP settings.
        qp_settings = dict(params.get('qp', {}))
        qp_settings.pop('primary_regularization', None)
        qp_settings.pop('equality_tolerance', None)
        online_params['qp'] = qp_settings
        safety = params.get('safety', {})
        legacy_soft = safety.get('soft_constraints', {})
        if legacy_soft.get('enabled', False):
            hard_margin = legacy_soft.get('hard_position_margin', 0.06)
            buffer = np.asarray(safety.get('position_margin', 0.15)) - np.asarray(hard_margin)
            online_params['safety'] = dict(safety, position_margin=hard_margin, braking_buffer=buffer)
        circle = params.get('circle', {})
        reference = CircleReference(robot.position(params['initial_q']),
                                    radius=circle.get('radius', 0.04),
                                    period=circle.get('period', 4.0),
                                    ramp=circle.get('ramp', 1.0))
        super().__init__(robot, online_params, dt, reference)
        self.q0 = vector(params['initial_q'], 7, 'initial_q')
        if np.any(self.q0 < self.limits.q_lower) or np.any(self.q0 > self.limits.q_upper):
            raise ValueError('initial_q must respect configured hard position margins')
        options = circle.get('lap_return', {})
        self.reference = CircularLapReference(self.reference, self.q0, options)
        self._tracking_mass_gain = self.mass_gain
        self._tracking_damping = self.null_damping
        # Apply the same smooth ramp on the local clock of each phase.
        # The adapter scales the objectives using elapsed time in this phase.
        self._ramp_duration = self.startup_ramp_duration
        self.startup_ramp_duration = 0.0

    @property
    def lap_index(self):
        return self.reference.lap_index

    def update(self, t, q, dq):
        reference = self.reference
        reference.advance(t, q, dq)

        elapsed = float(t) - reference.started_at
        ramp = reference.transition_duration if reference.returning else self._ramp_duration
        phase = 1.0 if ramp == 0 else float(np.clip(elapsed / ramp, 0.0, 1.0))
        scale = phase * phase * (3.0 - 2.0 * phase)
        return_damping = 2.0 * np.sqrt(reference.posture_gain)
        if reference.returning:
            self.mass_gain = (1.0 - scale) * self._tracking_mass_gain
            self.q_posture = self.q0
            self.posture_gain = scale * reference.posture_gain
            self.null_damping = self._tracking_damping + scale * (return_damping - self._tracking_damping)
        else:
            self.mass_gain = scale * self._tracking_mass_gain
            self.q_posture = None
            self.posture_gain = 0.0
            self.null_damping = (self._tracking_damping if self.lap_index == 1 else
                                 return_damping + scale * (self._tracking_damping - return_damping))

        tau = super().update(t, q, dq)
        if reference.failure_reason and not self.done:
            self.failure_reason = reference.failure_reason
            self.done = True
        self.log.update(reference.diagnostics(t))
        self.log['startup_scale'] = scale
        return tau
