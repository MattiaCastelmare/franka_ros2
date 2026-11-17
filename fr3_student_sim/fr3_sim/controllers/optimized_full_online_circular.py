"""Online mass descent for one circle, then a physical return to initial_q."""
from __future__ import annotations

import numpy as np

from fr3_sim.controllers.optimized_hqp import Controller as CircularController
from fr3_sim.methods.joint_constraints import vector


class _LapReference:
    """Restart the circle clock; brake smoothly before holding its start."""

    def __init__(self, circle):
        self.circle = circle
        self.started_at = 0.0
        self.returning = False
        self.transition_duration = 0.2

    def sample(self, t):
        if not self.returning:
            return self.circle.sample(t - self.started_at)
        elapsed = t - self.started_at
        duration = self.transition_duration
        if elapsed >= duration:
            return self.circle.sample(-1.0)
        u = float(np.clip(elapsed / duration, 0.0, 1.0))
        c0, c1, c2, c3, c4, c5 = self._braking
        p = c0 + u * (c1 + u * (c2 + u * (c3 + u * (c4 + u * c5))))
        v = (c1 + u * (2*c2 + u * (3*c3 + u * (4*c4 + u * 5*c5)))) / duration
        a = (2*c2 + u * (6*c3 + u * (12*c4 + u * 20*c5))) / duration**2
        return p, v, a

    def start_return(self, t):
        # Match p/v/a at entry, then reach the starting point at rest.
        p, v, a = self.sample(t)
        duration = self.transition_duration
        delta = self.circle.start_position - p
        velocity, acceleration = duration * v, duration**2 * a
        self._braking = (p, velocity, 0.5 * acceleration,
                         10*delta - 6*velocity - 1.5*acceleration,
                         -15*delta + 8*velocity + 1.5*acceleration,
                         6*delta - 3*velocity - 0.5*acceleration)
        self.returning = True
        self.started_at = float(t)

    def lap_complete(self, t):
        elapsed = t - self.started_at
        ramp = self.circle.ramp
        if ramp > 0 and elapsed < ramp:
            u = elapsed / ramp
            phase_time = ramp * (u**3 - 0.5 * u**4)
        else:
            phase_time = elapsed - 0.5 * ramp
        return phase_time >= self.circle.period


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
        self.reference = _LapReference(self.reference)
        self.lap_index = 1
        self._tracking_mass_gain = self.mass_gain
        self._tracking_damping = self.null_damping
        # Apply the same smooth ramp on the local clock of each phase.
        # The parent still receives absolute time for observer/gap checks.
        self._ramp_duration = self.startup_ramp_duration
        self.startup_ramp_duration = 0.0
        options = params.get('circle', {}).get('lap_return', {})
        for name, default in (('posture_gain', 25.0), ('position_tolerance', 0.001),
                              ('velocity_tolerance', 0.005), ('timeout', 5.0),
                              ('transition_duration', 0.2)):
            value = float(options.get(name, default))
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f'circle.lap_return.{name} must be positive and finite')
            setattr(self, '_return_' + name, value)
        self.reference.transition_duration = self._return_transition_duration

    def update(self, t, q, dq):
        reference = self.reference
        error = float(np.max(np.abs(self.q0 - np.asarray(q))))
        speed = float(np.max(np.abs(dq)))
        if reference.returning:
            if (t - reference.started_at >= reference.transition_duration
                    and error <= self._return_position_tolerance
                    and speed <= self._return_velocity_tolerance):
                reference.returning = False
                reference.started_at = float(t)
                self.lap_index += 1
        elif reference.lap_complete(t):
            reference.start_return(t)

        elapsed = float(t) - reference.started_at
        ramp = reference.transition_duration if reference.returning else self._ramp_duration
        phase = 1.0 if ramp == 0 else float(np.clip(elapsed / ramp, 0.0, 1.0))
        scale = phase * phase * (3.0 - 2.0 * phase)
        return_damping = 2.0 * np.sqrt(self._return_posture_gain)
        if reference.returning:
            self.mass_gain = (1.0 - scale) * self._tracking_mass_gain
            self.q_posture = self.q0
            self.posture_gain = scale * self._return_posture_gain
            self.null_damping = self._tracking_damping + scale * (return_damping - self._tracking_damping)
        else:
            self.mass_gain = scale * self._tracking_mass_gain
            self.q_posture = None
            self.posture_gain = 0.0
            self.null_damping = (self._tracking_damping if self.lap_index == 1 else
                                 return_damping + scale * (self._tracking_damping - return_damping))

        tau = super().update(t, q, dq)
        if reference.returning and elapsed >= self._return_timeout and not self.done:
            self.failure_reason = 'return to initial_q timed out'
            self.done = True
        self.log.update({'startup_scale': scale, 'lap_index': float(self.lap_index),
                         'lap_returning': float(reference.returning), 'lap_time': elapsed,
                         'q0_error': error, 'q0_speed': speed})
        return tau
