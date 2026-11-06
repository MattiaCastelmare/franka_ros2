"""The obstacle laws of the ScenarioObstacles system, in Python (truth checks). No rclpy.

Same definitions as src/scenario_obstacles_system.cpp: position at episode time
t >= 0 for the analytic laws, and the explicit-Euler pursuit of a target
sequence (cbf_scenarios.Sphere.step).

``wander`` is the quasi-random tour of a box (one incommensurate sinusoid per
world axis). It is analytic for exactly this reason: the plugin and this module
are the sim and the truth of ONE motion, and no seeded RNG is bit-identical
across C++ and numpy. Variety between seeds comes from the catalog's
``jitter_m`` on ``p0``, which translates the whole tour.
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


def _phase(spec: dict) -> float:
    """``phase_deg`` in radians (0 = the ball is born at p0, the swing's centre).

    The Gazebo side is scenario_obstacles_system.cpp's ``ob.phase``, parsed from
    the same ``<phase_deg>``: these two are the truth and the sim of ONE motion,
    so a change here is a change there.
    """
    return float(spec.get('phase_deg', 0.0)) * np.pi / 180.0


#: default frequency multipliers of ``wander``: 1, the golden ratio and the
#: silver ratio. Mirrored in gz_world.WANDER_RATIOS and in
#: scenario_obstacles_system.cpp; the three must agree, because this module is
#: the TRUTH of the motion the plugin runs, not an approximation of it.
WANDER_RATIOS = (1.0, 1.6180339887498949, 2.4142135623730951)


def _wander(spec: dict):
    """``(box, w_i, phase_i)`` of the wander law, validated as the plugin does."""
    period = float(spec.get('period', 0.0))
    if period <= 0.0:
        raise ValueError('wander needs a positive period')
    box = np.asarray(spec.get('box', (0.0, 0.0, 0.0)), float)
    ratios = np.asarray(spec.get('ratios', WANDER_RATIOS), float)
    ph = np.asarray(spec.get('phase_deg_xyz', (0.0, 0.0, 0.0)), float) * np.pi / 180.0
    if box.shape != (3,) or ratios.shape != (3,) or ph.shape != (3,):
        raise ValueError('wander needs 3-vectors for box, ratios and phase_deg_xyz')
    return box, (2.0 * np.pi / period) * ratios, ph


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
        return p0 + _axis(spec) * (float(spec['amplitude']) * np.sin(w * t + _phase(spec)))
    if law == 'wander':
        # One INDEPENDENT sinusoid per world axis: the three frequencies are
        # incommensurate, so the ball tours the box p0 +/- box without ever
        # repeating. Bounded by construction, which is why no clamp is needed.
        box, w3, ph = _wander(spec)
        return p0 + box * np.sin(w3 * t + ph)
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
        return _axis(spec) * (float(spec['amplitude']) * w * np.cos(w * t + _phase(spec)))
    if law == 'wander':
        box, w3, ph = _wander(spec)
        return box * w3 * np.cos(w3 * t + ph)
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
