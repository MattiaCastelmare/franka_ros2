"""Finite straight displacement with zero reference speed at both endpoints."""

import numpy as np


class RetreatReference:
    """Quintic position reference, held at its endpoint after ``duration`` seconds."""

    def __init__(self, start_position, displacement, duration):
        self.start_position = np.asarray(start_position, dtype=float).copy()
        self.displacement = np.asarray(displacement, dtype=float).copy()
        for name, vector in (('start_position', self.start_position),
                             ('displacement', self.displacement)):
            if vector.shape != (3,) or not np.all(np.isfinite(vector)):
                raise ValueError(f'{name} must contain three finite values')
        self.duration = float(duration)
        if not np.isfinite(self.duration) or self.duration <= 0.0:
            raise ValueError('retreat.duration must be finite and positive')

    def sample(self, t):
        """Return position [m], velocity [m/s] and acceleration [m/s^2]."""
        t = float(t)
        if not np.isfinite(t):
            raise ValueError('reference time must be finite')
        if t <= 0.0:
            return self.start_position.copy(), np.zeros(3), np.zeros(3)
        if t >= self.duration:
            return self.start_position + self.displacement, np.zeros(3), np.zeros(3)

        u = t / self.duration
        s = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
        s_dot = 30.0 * u**2 * (1.0 - u)**2 / self.duration
        s_ddot = 60.0 * u * (1.0 - u) * (1.0 - 2.0 * u) / self.duration**2
        return (self.start_position + s * self.displacement,
                s_dot * self.displacement, s_ddot * self.displacement)
