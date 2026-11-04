"""Circular position, velocity and acceleration references without ROS dependencies."""
from __future__ import annotations

import numpy as np


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
            theta = self._omega * self.ramp * (u ** 3 - 0.5 * u ** 4)
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
