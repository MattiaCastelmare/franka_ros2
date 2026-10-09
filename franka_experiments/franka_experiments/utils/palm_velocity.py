#!/usr/bin/env python3
"""Palm velocity: the W75 estimator and the uncertainty of its constant-velocity prediction.

W75 (validated in Stage 5C and ROS streaming, frozen 15 September):
  EWL    exponentially weighted linear regression, tau 0.20 s, last 5 fresh samples (<= 0.40 s)
  SURDE  soft mixture of linear fits over N = 3, 4, 5, 7, 9, 12 samples (<= 0.80 s),
         white noise sigma 2 mm, weights from the derivative-sum cost (temperature from K fits)
  fusion 0.75 SURDE + 0.25 EWL; a velocity is held for at most 0.10 s without new samples.
Only fresh measured palms enter the histories (never predicted ones).
"""

import math
from collections import deque

import numpy as np

SOURCE_NONE, SOURCE_UPDATED, SOURCE_HOLD = 0, 1, 2
BANK = (3, 4, 5, 7, 9, 12)


# ------------------------------------------------------------ EWL
def weighted_linear_velocity(times, positions, tau_s):
    times, positions = np.asarray(times, dtype=float), np.asarray(positions, dtype=float)
    if len(times) < 3:
        return None

    x = times - times[-1]
    weights = np.exp(x / float(tau_s))
    sw = float(np.sum(weights))
    if sw <= 0.0:
        return None

    x_mean = float(np.sum(weights * x) / sw)
    p_mean = np.sum(weights[:, None] * positions, axis=0) / sw
    xc = x - x_mean
    denominator = float(np.sum(weights * xc * xc))
    if denominator <= 1e-12:
        return None

    return np.asarray(np.sum(weights[:, None] * xc[:, None] * (positions - p_mean), axis=0) / denominator,
                      dtype=float)


# ------------------------------------------------------------ SURDE
def slope_filter(times):
    """Row of the least-squares slope (linear fit with intercept): velocity = row @ positions."""
    times = np.asarray(times, dtype=float)
    if len(times) < 2:
        return None

    x = times - times[-1]
    xc = x - np.mean(x)
    denominator = float(np.dot(xc, xc))
    if not np.isfinite(denominator) or denominator <= 1e-12:
        return None
    return np.asarray(xc / denominator, dtype=float)


def direct_derivative_sum_filter(times, n0):
    """Row of the sum of the last n0 finite differences (positions[i] - positions[i-1]) / dt."""
    times = np.asarray(times, dtype=float)
    N = len(times)
    if n0 < 1 or N < n0 + 1:
        return None

    s = np.zeros(N, dtype=float)
    for i in range(N - n0, N):
        dt = float(times[i] - times[i - 1])
        if dt <= 1e-9:
            return None
        s[i] += 1.0 / dt
        s[i - 1] -= 1.0 / dt
    return s


def surde_candidates(times, positions, sigma_mm):
    """One linear fit per window N of BANK: velocity, cost against the direct derivatives, and the
    noise terms (Q = sigma^2 I) used by the temperature."""
    times, positions = np.asarray(times, dtype=float), np.asarray(positions, dtype=float)
    if len(times) < BANK[0]:
        return None

    sigma2 = (float(sigma_mm) / 1000.0) ** 2
    n0 = BANK[0] - 1
    rows = []
    for N in [N for N in BANK if N <= len(times)]:
        v, s = slope_filter(times[-N:]), direct_derivative_sum_filter(times[-N:], n0)
        if v is None or s is None:
            continue

        velocity, direct_sum = v @ positions[-N:], s @ positions[-N:]
        # noise terms of the two filters (Q = sigma^2 I), used by the temperature
        Cvs = sigma2 * float(np.dot(v, s))
        Vw = sigma2 * float(np.dot(v, v))
        Sw = sigma2 * float(np.dot(s, s))
        costs_xyz = n0 * velocity ** 2 + 2.0 * Cvs - 2.0 * velocity * direct_sum
        rows.append({'N': int(N), 'velocity': np.asarray(velocity, dtype=float),
                     'cost': float(np.sum(costs_xyz)), 'Vw': Vw, 'Sw': Sw, 'Cvs': Cvs})

    return {'rows': rows, 'n0': n0} if rows else None


def surde_temperature(candidates):
    rows = candidates['rows']
    K = len(rows)
    if K <= 1:
        return np.inf

    Vw, Sw, Cvs = float(rows[0]['Vw']), float(rows[0]['Sw']), float(rows[0]['Cvs'])
    n0 = int(candidates['n0'])
    nu_axis = 2.0 * n0 ** 2 * Vw ** 2 - 8.0 * n0 * Vw * Cvs + 4.0 * Vw * Sw + 4.0 * Cvs ** 2
    nu_xyz = 3.0 * max(nu_axis, 0.0)
    if nu_xyz <= 1e-30:
        return 1e-15

    return max(math.sqrt(nu_xyz / (2.0 * math.log(K))), 1e-15)


def surde_soft(times, positions, sigma_mm):
    """Fits weighted by N * exp(-cost / T)."""
    candidates = surde_candidates(times, positions, sigma_mm)
    if candidates is None:
        return None

    rows = candidates['rows']
    costs = np.asarray([row['cost'] for row in rows], dtype=float)
    Ns = np.asarray([row['N'] for row in rows], dtype=float)
    velocities = np.stack([row['velocity'] for row in rows], axis=0)

    if len(rows) == 1:
        weights = np.ones(1, dtype=float)
    else:
        logw = np.log(Ns) - costs / surde_temperature(candidates)
        logw -= np.max(logw)
        weights = np.exp(logw)
        total = float(np.sum(weights))
        if not np.isfinite(total) or total <= 0.0:
            weights = np.zeros(len(rows), dtype=float)
            weights[int(np.argmin(costs))] = 1.0
        else:
            weights /= total

    return np.asarray(np.sum(weights[:, None] * velocities, axis=0), dtype=float)


# ------------------------------------------------------------ W75
class _Channel:
    """History of fresh palms -> velocity, held for HOLD_S after the last update."""

    HOLD_S = 0.10

    def __init__(self, maxlen, max_age_s, min_samples, estimate):
        self.history = deque(maxlen=maxlen)
        self.max_age_s, self.min_samples, self.estimate = max_age_s, min_samples, estimate
        self.velocity = self.updated_at = None

    def reset(self):
        self.history.clear()
        self.velocity = self.updated_at = None

    def update(self, now, palm, fresh, position_available):
        """-> (available, velocity, source, age_s, updated)"""
        while self.history and now - self.history[0][0] > self.max_age_s:
            self.history.popleft()

        updated = False
        if fresh:
            self.history.append((now, palm.copy()))
            if len(self.history) >= self.min_samples:
                tt = np.asarray([item[0] for item in self.history], dtype=float)
                pp = np.asarray([item[1] for item in self.history], dtype=float)
                velocity = self.estimate(tt, pp)
                if velocity is not None:
                    self.velocity, self.updated_at, updated = velocity.copy(), now, True

        if position_available and self.velocity is not None and self.updated_at is not None:
            age = now - self.updated_at
            if -1e-9 <= age <= self.HOLD_S + 1e-9:
                return (True, self.velocity.copy(), (SOURCE_UPDATED if updated else SOURCE_HOLD),
                        float(age), updated)

        return False, None, SOURCE_NONE, np.nan, updated


class W75VelocityEstimator:

    def __init__(self):
        self.ewl = _Channel(5, 0.40, 3, lambda t, p: weighted_linear_velocity(t, p, 0.20))
        self.surde = _Channel(12, 0.80, BANK[0], lambda t, p: surde_soft(t, p, 2.0))

    def reset(self):
        self.ewl.reset()
        self.surde.reset()

    def update(self, now, palm, fresh, position_available):
        ewl_ok, ewl_v, ewl_source, ewl_age, ewl_updated = self.ewl.update(
            now, palm, fresh, position_available)
        surde_ok, surde_v, surde_source, surde_age, surde_updated = self.surde.update(
            now, palm, fresh, position_available)

        if ewl_ok and surde_ok:
            return {'available': True, 'velocity': 0.25 * ewl_v + 0.75 * surde_v,
                    'source': SOURCE_UPDATED if (ewl_updated or surde_updated) else SOURCE_HOLD,
                    'age_s': max(float(ewl_age), float(surde_age))}
        if surde_ok:
            return {'available': True, 'velocity': surde_v.copy(), 'source': int(surde_source),
                    'age_s': float(surde_age)}
        if ewl_ok:
            return {'available': True, 'velocity': ewl_v.copy(), 'source': int(ewl_source),
                    'age_s': float(ewl_age)}
        return {'available': False, 'velocity': np.zeros(3, dtype=float), 'source': SOURCE_NONE,
                'age_s': np.nan}


# ------------------------------------------------------------ prediction uncertainty
# q97.5 of the 3-D palm error [mm] of the W75 constant-velocity prediction, calibrated offline
# per speed class and horizon.
PREDICTION_HORIZONS_MS = np.asarray((33, 67, 100, 133, 167, 200, 250, 300), dtype=float)
PREDICTION_Q975_MM = {
    'slow': np.asarray((15.143, 20.815, 30.395, 37.848, 49.529, 66.295, 84.938, 108.248), dtype=float),
    'low': np.asarray((25.957, 40.220, 61.148, 84.904, 91.217, 104.230, 139.686, 179.249), dtype=float),
    'medium': np.asarray((48.072, 69.596, 90.289, 106.856, 106.349, 134.713, 182.454, 233.515), dtype=float),
    'fast': np.asarray((48.820, 86.009, 99.605, 128.562, 162.389, 205.623, 246.683, 296.686), dtype=float),
}
PREDICTION_CHI3_Q975_RADIUS = 3.057515920563  # q97.5 radius of a 3-D standard normal


def prediction_speed_class(speed):
    return 'slow' if speed < 0.10 else 'low' if speed < 0.30 else 'medium' if speed < 0.60 else 'fast'


def prediction_q975_error_m(age_s, speed):
    """q97.5 error [m] at this prediction age (0-300 ms), interpolated; nan outside."""
    age_ms = float(age_s) * 1000.0
    if not np.isfinite(age_ms) or not np.isfinite(speed) or age_ms < 0.0 or age_ms > 300.0 + 1e-9:
        return np.nan

    # monotone in the horizon, starting from 0 mm at 0 ms
    values = np.maximum.accumulate(PREDICTION_Q975_MM[prediction_speed_class(float(speed))])
    horizons = np.concatenate((np.asarray([0.0]), PREDICTION_HORIZONS_MS))
    errors = np.concatenate((np.asarray([0.0]), values))
    return float(np.interp(age_ms, horizons, errors)) / 1000.0
