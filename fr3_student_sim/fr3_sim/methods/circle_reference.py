"""Circular position, velocity and acceleration references without ROS dependencies."""
from __future__ import annotations

import numpy as np


def _integrated_ramp(u):
    return u**3 - 0.5 * u**4


def _finite_scalar(value, name: str) -> float:
    """Reject arrays and non-finite values with a useful configuration error."""
    try:
        scalar = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be a finite scalar') from exc
    if scalar.ndim != 0 or not np.isfinite(scalar):
        raise ValueError(f'{name} must be a finite scalar')
    return float(scalar)


class CircleReference:
    """A vertical x-z circle beginning at its -x point.

    The circle's center is ``start_position + [radius, 0, 0]``. Angular
    velocity increases from zero to ``2*pi/period`` with a cubic smoothstep
    over ``ramp`` seconds. Integrating that ramp gives the phase, so position,
    velocity and acceleration remain continuous at both ramp boundaries.

    Negative times hold the starting point with zero velocity and acceleration.
    With ``ramp=0``, constant angular velocity begins immediately at ``t=0``;
    that explicit mode does not provide a smooth start.
    """

    def __init__(self, start_position, radius=0.04, period=8.0, ramp=1.0):
        try:
            start = np.asarray(start_position, dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValueError('start_position must contain three finite coordinates') from exc
        if start.shape != (3,) or not np.all(np.isfinite(start)):
            raise ValueError('start_position must contain three finite coordinates')
        self.start_position = start.copy()
        self.radius = _finite_scalar(radius, 'radius')
        self.period = _finite_scalar(period, 'period')
        self.ramp = _finite_scalar(ramp, 'ramp')
        if self.radius <= 0:
            raise ValueError('radius must be positive')
        if self.period <= 0:
            raise ValueError('period must be positive')
        if self.ramp < 0:
            raise ValueError('ramp must be nonnegative')
        self._omega = 2.0 * np.pi / self.period
        with np.errstate(over='ignore', invalid='ignore'):
            self.center = self.start_position + np.array([self.radius, 0.0, 0.0])
        if not np.isfinite(self._omega) or not np.all(np.isfinite(self.center)):
            raise ValueError('circle parameters exceed the finite numerical range')

    def sample(self, t):
        """Return independent ``(position, velocity, acceleration)`` arrays.

        Units are metres, metres/second and metres/second squared. Time is in
        simulation seconds; all vectors use the robot base frame.
        """
        t = _finite_scalar(t, 't')
        if t < 0 or (t == 0 and self.ramp > 0):
            return self.start_position.copy(), np.zeros(3), np.zeros(3)

        if self.ramp > 0 and t < self.ramp:
            u = t / self.ramp
            theta = self._omega * self.ramp * _integrated_ramp(u)
            speed = self._omega * (3.0 * u ** 2 - 2.0 * u ** 3)
            acceleration = self._omega / self.ramp * (6.0 * u - 6.0 * u ** 2)
        else:
            theta = self._omega * (t - 0.5 * self.ramp)
            speed, acceleration = self._omega, 0.0

        with np.errstate(over='ignore', invalid='ignore'):
            sine, cosine = np.sin(theta), np.cos(theta)
            radial = np.array([-cosine, 0.0, sine])
            tangent = np.array([sine, 0.0, cosine])
            p_ref = self.center + self.radius * radial
            v_ref = self.radius * speed * tangent
            a_ref = self.radius * (acceleration * tangent - speed * speed * radial)
        if not all(np.all(np.isfinite(vector)) for vector in (p_ref, v_ref, a_ref)):
            raise ValueError('reference at t exceeds the finite numerical range')
        return p_ref.copy(), v_ref.copy(), a_ref.copy()


class CircularLapReference:
    """Share the braking reference and restart schedule between circular controllers."""

    def __init__(self, circle, initial_q, options):
        if not isinstance(options, dict):
            raise ValueError('circle.lap_return must be a mapping')
        self.circle = circle
        self.center = circle.center
        self.initial_q = initial_q
        self.started_at = 0.0
        self.returning = False
        self.lap_index = 1
        self.failure_reason = ''
        for name, default in (('posture_gain', 25.0), ('position_tolerance', 0.001),
                              ('velocity_tolerance', 0.005), ('timeout', 5.0),
                              ('transition_duration', 0.2)):
            value = _finite_scalar(options.get(name, default), f'circle.lap_return.{name}')
            if value <= 0:
                raise ValueError(f'circle.lap_return.{name} must be positive')
            setattr(self, name, value)
        duration = options.get('duration')
        self.duration = (None if duration is None else
                         _finite_scalar(duration, 'circle.lap_return.duration'))
        if self.duration is not None and not self.transition_duration <= self.duration <= self.timeout:
            raise ValueError('circle.lap_return.duration must be between transition_duration and timeout')

        if circle.period >= 0.5 * circle.ramp:
            self.lap_duration = circle.period + 0.5 * circle.ramp
        else:
            # Invert the integrated phase when a full turn fits inside the ramp.
            target = circle.period / circle.ramp
            lo, hi = 0.0, 1.0
            for _ in range(52):
                mid = 0.5 * (lo + hi)
                if _integrated_ramp(mid) < target:
                    lo = mid
                else:
                    hi = mid
            self.lap_duration = hi * circle.ramp

    def sample(self, t):
        elapsed = t - self.started_at
        if not self.returning:
            return self.circle.sample(elapsed)
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
        # Match position, velocity and acceleration entering the braking phase.
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

    def advance(self, t, q, dq):
        self.q0_error = float(np.max(np.abs(self.initial_q - q)))
        self.q0_speed = float(np.max(np.abs(dq)))
        if self.returning:
            elapsed = t - self.started_at
            settled = (self.q0_error <= self.position_tolerance
                       and self.q0_speed <= self.velocity_tolerance)
            if self.duration is None:
                if elapsed >= self.transition_duration and settled:
                    self._restart(t)
                elif elapsed >= self.timeout:
                    self.failure_reason = 'return to initial_q timed out'
            else:
                restart_at = self.started_at + self.duration
                if t >= restart_at - 1e-12:
                    if settled:
                        self._restart(restart_at)
                    else:
                        self.failure_reason = 'return to initial_q did not settle by the scheduled restart'
        else:
            return_at = self.started_at + self.lap_duration
            if t >= return_at - 1e-12:
                self.start_return(t if self.duration is None else return_at)

    def _restart(self, started_at):
        self.returning = False
        self.started_at = float(started_at)
        self.lap_index += 1

    def diagnostics(self, t):
        return {'lap_index': float(self.lap_index), 'lap_returning': float(self.returning),
                'lap_time': t - self.started_at, 'q0_error': self.q0_error,
                'q0_speed': self.q0_speed}
