"""Online mass descent for one circle, then a physical return to initial_q."""
from __future__ import annotations

import numpy as np

from fr3_sim.controllers.optimized_hqp import Controller as CircularController
from fr3_sim.methods.circle_reference import CircularLapReference
from fr3_sim.methods.joint_constraints import vector


class Controller(CircularController):
    """Alternate a pure online circle and a constrained return to initial_q.

    Both phases use the existing HQP update and absolute observer time.
    Only the reference clock and secondary preferences change between them.
    """

    def __init__(self, robot, params, dt):
        online_params = dict(params)
        online_params.pop('q_posture_ref', None)
        online_params['tracking'] = dict(params.get('tracking', {}))
        online_params['tracking']['posture_gain'] = 0.0
        super().__init__(robot, online_params, dt)
        self.q0 = vector(params['initial_q'], 7, 'initial_q')
        if (np.any(self.q0 < self.constraints.hard_q_lower)
                or np.any(self.q0 > self.constraints.hard_q_upper)):
            raise ValueError('initial_q must respect configured hard position margins')
        options = params.get('circle', {}).get('lap_return', {})
        self.reference = CircularLapReference(self.reference, self.q0, options)
        self._tracking_mass_gain = self.mass_gain
        self._tracking_damping = self.null_damping
        # Apply the same smooth ramp on the local clock of each phase.
        # The parent still receives absolute time for observer/gap checks.
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
