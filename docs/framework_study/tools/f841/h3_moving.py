"""F-8-41 (harness only): the obstacle-motion quantities the validity and clearance checks need.

Every function reduces EXACTLY to the static case when every obstacle's law is 'static': the speed
bound is 0.0, so every corrected bound equals the sealed H2 formula bit for bit (tested against the
sealed E3 H2 runs in test_f841.py).

Speed bound. The checks need an upper bound on |v_obs| that holds at EVERY instant of the run. For the
analytic laws it is computed from the scenario, not measured:
  static      0
  linear      |v0|
  oscillate   amplitude * 2*pi/period
  wander      || box_i * 2*pi/period * ratio_i ||   (each axis peaks at box_i*w_i; the three frequencies
              are incommensurate, so the sum of squares is the supremum: approached, never exceeded)
Laws without a time-independent bound (ballistic: speed grows with t; pursuit: the target's motion)
are refused: a check that cannot be bounded must not silently pass.
"""
import math

import numpy as np

WANDER_RATIOS = (1.0, 1.6180339887498949, 2.4142135623730951)   # = simexp.obstacle_laws.WANDER_RATIOS


def speed_bound(spec):
    """Upper bound on |v| of one obstacle over all time [m/s]."""
    law = spec.get('law', 'static')
    if law == 'static':
        return 0.0
    if law == 'linear':
        return float(np.linalg.norm(np.asarray(spec.get('v0', (0.0, 0.0, 0.0)), float)))
    if law == 'oscillate':
        period = float(spec['period'])
        if period <= 0.0:
            raise ValueError('oscillate needs a positive period')
        return abs(float(spec['amplitude'])) * 2.0 * math.pi / period
    if law == 'wander':
        period = float(spec.get('period', 0.0))
        if period <= 0.0:
            raise ValueError('wander needs a positive period')
        box = np.asarray(spec.get('box', (0.0, 0.0, 0.0)), float)
        ratios = np.asarray(spec.get('ratios', WANDER_RATIOS), float)
        return float(np.linalg.norm(np.abs(box) * (2.0 * math.pi / period) * ratios))
    raise ValueError(f'law {law!r} has no time-independent speed bound: the moving-obstacle checks refuse it')


def scenario_speed_bound(scenario_resolved):
    """max over the scenario's obstacles of speed_bound (0.0 for a static scene)."""
    obs = scenario_resolved.get('obstacles') or []
    return max([speed_bound(o) for o in obs] or [0.0])


def g_min(d_safe, v_ref, tau_budget, v_obs_max):
    """V4: a gap g0 at the birth cannot close to d_safe within tau_budget when
    g0 >= d_safe + (v_ref + v_obs_max) * tau_budget.
    d/dt ||p_cp - c|| >= -|v_cp - v_obs| >= -(|v_cp| + |v_obs|) (triangle inequality), so the closing
    speed of any control point on the ball is at most v_ref + v_obs_max."""
    return d_safe + (v_ref + v_obs_max) * tau_budget


def continuous_lower_bound(gmin, pose_age, dt_max, v_cp_max, v_obs_max):
    """Lower bound on the TRUE clearance over the whole sampled interval.

    gmin[k]     reconstructed min gap at joint sample k, computed with the latest obstacle pose at or
                before t_k (causal); pose_age[k] = t_k - t_pose(k) >= 0 is that pose's age.
    (1) pose staleness: the ball moved at most v_obs_max * pose_age[k] since its pose, and the distance
        to a sphere is 1-Lipschitz in its centre, so gap_true(t_k) >= gmin[k] - v_obs_max * pose_age[k];
    (2) between samples: |d gap / dt| <= v_cp_max + v_obs_max, so within dt_max/2 of a sample the gap
        dips at most (v_cp_max + v_obs_max) * dt_max / 2.
    With v_obs_max = 0 this is exactly the sealed static formula min(gmin) - v_cp_max * dt_max / 2."""
    gmin = np.asarray(gmin, float)
    age = np.asarray(pose_age, float)
    if v_obs_max == 0.0:
        corrected = gmin
    else:
        if not np.all(np.isfinite(age[np.isfinite(gmin)])) or np.any(age[np.isfinite(gmin)] < 0):
            raise ValueError('pose_age must be finite and >= 0 wherever a gap was reconstructed')
        corrected = gmin - v_obs_max * age
    dip = (v_cp_max + v_obs_max) * dt_max / 2.0
    return float(np.nanmin(corrected) - dip), float(dip)


def pose_pairing_excess(d_abs, dt_pose_abs, v_obs_max):
    """R13 for a moving obstacle: |gap_recon - gap_truth| on a matched sample can differ only because the
    two used poses taken |dt_pose| apart, so |d| <= v_obs_max * |dt_pose| (1-Lipschitz in the centre).
    Returns |d| - v_obs_max * |dt_pose| per sample (<= EXACT expected). Static: exactly |d|."""
    d_abs = np.asarray(d_abs, float)
    if v_obs_max == 0.0:
        return d_abs
    return d_abs - v_obs_max * np.asarray(dt_pose_abs, float)
