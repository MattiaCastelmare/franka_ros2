"""Cartesian acceleration feedback shared by every tracking experiment."""
from __future__ import annotations

import numpy as np


def _gain(value, name):
    """Copy a positive finite scalar or diagonal three-axis gain."""
    try:
        gain = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be a positive finite scalar or 3-vector') from exc
    if gain.ndim == 0:
        gain = np.full(3, float(gain))
    if gain.shape != (3,) or not np.all(np.isfinite(gain)) or np.any(gain <= 0.0):
        raise ValueError(f'{name} must be a positive finite scalar or 3-vector')
    return gain.copy()


class CartesianAccelerationTracking:
    """Apply acceleration feedforward and Cartesian position/velocity feedback.

    ``kp_position`` is in s^-2 and ``kd_position`` in s^-1. Scalars apply to
    all three world axes; 3-vectors specify diagonal gains. The requested
    acceleration is independent of robot inertia, trajectory shape and
    secondary posture or mass objectives.
    """

    def __init__(self, kp_position=100.0, kd_position=20.0):
        self.kp_position = _gain(kp_position, 'tracking.kp_position')
        self.kd_position = _gain(kd_position, 'tracking.kd_position')

    def acceleration(self, a_ref, position_error, velocity_error):
        """Return ``a_ref + Kp * position_error + Kd * velocity_error``."""
        return (a_ref + self.kp_position * position_error
                + self.kd_position * velocity_error)
