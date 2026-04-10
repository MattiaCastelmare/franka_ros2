"""Pure math helpers: minimum-jerk, cosine ramp, clamp, low-pass filter."""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np


def min_jerk(tau: float) -> Tuple[float, float]:
    """Minimum-jerk 5th-order polynomial for smooth interpolation.

    Parameters
    ----------
    tau : float
        Normalised time in [0, 1].

    Returns
    -------
    s : float
        Position blend factor (0 → 1).
    sdot : float
        Normalised velocity ds/dτ.
    """
    tau = float(np.clip(tau, 0.0, 1.0))
    t2 = tau * tau
    t3 = t2 * tau
    t4 = t3 * tau
    t5 = t4 * tau
    s = 10.0 * t3 - 15.0 * t4 + 6.0 * t5
    sdot = 30.0 * t2 - 60.0 * t3 + 30.0 * t4
    return s, sdot


def cosine_ramp(t: float, duration: float) -> float:
    """Cosine ramp envelope: 0 → 1 over *duration* seconds.

    Returns 1.0 if *duration* ≤ 0 or *t* ≥ *duration*.
    Returns 0.0 if *t* ≤ 0.
    """
    if duration <= 0.0 or t >= duration:
        return 1.0
    if t <= 0.0:
        return 0.0
    return 0.5 * (1.0 - math.cos(math.pi * t / duration))


def clamp_joints(qdot: np.ndarray, qdot_max: float) -> np.ndarray:
    """Per-element clamp to ``[-qdot_max, +qdot_max]``."""
    return np.clip(qdot, -qdot_max, qdot_max)


def lpf(prev: np.ndarray, current: np.ndarray, alpha: float) -> np.ndarray:
    """First-order low-pass filter: ``out = α·prev + (1−α)·current``."""
    return alpha * prev + (1.0 - alpha) * current


def rate_limit(prev: "np.ndarray", current: "np.ndarray", max_delta: float) -> "np.ndarray":
    """Clamp per-element change between consecutive commands."""
    import numpy as np
    delta = current - prev
    return prev + np.clip(delta, -max_delta, max_delta)
