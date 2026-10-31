"""The obstacle laws of the ScenarioObstacles system, in Python (truth checks). No rclpy.

Same definitions as src/scenario_obstacles_system.cpp: position at episode time
t >= 0 for the analytic laws, and the explicit-Euler pursuit of a target
sequence (cbf_scenarios.Sphere.step).
"""
from __future__ import annotations

import numpy as np


def _axis(spec: dict) -> np.ndarray:
    """The oscillation axis, normalised as ScenarioObstacles::Configure does.

    The plugin normalises once at configure time and refuses a zero axis, so a
    caller who writes an unnormalised ``axis`` gets the same trajectory from
    both sides. Doing it here too is what keeps this a mirror rather than an
    approximation.
    """
    a = np.asarray(spec.get('axis', (1.0, 0.0, 0.0)), float)
    n = float(np.linalg.norm(a))
    if n < 1e-9:
        raise ValueError('oscillate needs a non-zero axis')
    return a / n


def position(spec: dict, t: float) -> np.ndarray:
    p0 = np.asarray(spec['p0'], float)
    law = spec.get('law', 'static')
    if law == 'static':
        return p0
    v0 = np.asarray(spec.get('v0', (0, 0, 0)), float)
    if law == 'linear':
        return p0 + v0 * t
    if law == 'ballistic':
        g = np.asarray(spec.get('gravity', (0, 0, -9.81)), float)
        return p0 + v0 * t + 0.5 * g * t * t
    if law == 'oscillate':
        w = 2.0 * np.pi / float(spec['period'])
        return p0 + _axis(spec) * (float(spec['amplitude']) * np.sin(w * t))
    raise ValueError(f'{law} is not analytic (pursuit needs the target, see pursue())')


def velocity(spec: dict, t: float) -> np.ndarray:
    law = spec.get('law', 'static')
    v0 = np.asarray(spec.get('v0', (0, 0, 0)), float)
    if law == 'static':
        return np.zeros(3)
    if law == 'linear':
        return v0
    if law == 'ballistic':
        return v0 + np.asarray(spec.get('gravity', (0, 0, -9.81)), float) * t
    if law == 'oscillate':
        w = 2.0 * np.pi / float(spec['period'])
        return _axis(spec) * (float(spec['amplitude']) * w * np.cos(w * t))
    raise ValueError(law)


def pursue(p0, speed: float, targets, h) -> np.ndarray:
    """Positions after each step k: p_{k+1} = p_k + speed * unit(target_k - p_k) * h_k."""
    p = np.asarray(p0, float).copy()
    out = []
    for tgt, hk in zip(np.asarray(targets, float), np.broadcast_to(h, (len(targets),))):
        d = tgt - p
        n = np.linalg.norm(d)
        if n > 1e-9:
            p = p + speed * d / n * hk
        out.append(p.copy())
    return np.array(out)
