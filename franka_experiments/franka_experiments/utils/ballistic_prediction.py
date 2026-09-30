"""Predicted impact points of fast tracked objects, as extra obstacle rows.

WHY THIS EXISTS
---------------
On the 2026-09-30 hardware throws (rosbag/ball_throws_3, 3.4-4.0 m/s) the arm
executed exactly what the CBF commanded — 4-16 ms lag, gain ~0.9 — and the
command was already at the acceleration box as soon as the filter acted. What
it lacked was TIME: the filter acted ~100-120 ms before closest approach,
because a barrier on the CURRENT gap only binds once the ball is near. A
ballistic fit of the ball's last 100 ms already predicted the miss distance to
~0.1 m 250-350 ms before closest approach, while the ball was still 0.7-1.0 m
away — beyond the 0.7 m publish band, so the CBF never saw it.

WHAT IT DOES
------------
For every confirmed track that is moving fast and knows it (|v| − k·σ >=
min_speed, the same test cbf_state_rows._fast_track applies), propagate
p(τ) = p + v τ + ½ g τ² over ``horizon_s`` and, for every control point c with
capsule radius r, find the predicted surface gap

    g(τ) = |p(τ) − c| − r − object_radius

and its minimum g* at τ*. When g* < ``publish_gap_m`` and the object is still
approaching (τ* >= ``min_time_s``), the control point gets one more obstacle
hit at the PREDICTED position p(τ*) with gap g*. It is appended to the
control point's ``extras``, so it goes out as one more rank row
(``link#k.r``, see perception_msgs.labelled_links) and the CBF builds an
ordinary HOCBF row from it — no consumer change. Its cluster id is
``PREDICTED_CLUSTER_ID`` so no track velocity is attached: the row describes a
place, not a moving thing, and the motion is already in where the place is.

Gravity is applied when the track's IMM says ballistic (mode_prob >= 0.5,
which is also its neutral start) or when the track has no IMM and
``assume_ballistic`` is set.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Iterable, List

import numpy as np

from franka_experiments.utils.distance_engine import ObstacleHit

#: Cluster id of a predicted hit. -1 already means "unknown cluster".
PREDICTED_CLUSTER_ID = -2
_G = np.array([0.0, 0.0, -9.81])


@dataclass(frozen=True)
class PredictionConfig:
    enabled: bool = False
    horizon_s: float = 0.5
    step_s: float = 0.01
    min_speed: float = 1.0
    k_sigma: float = 2.0
    min_frames: int = 3
    object_radius_m: float = 0.035
    publish_gap_m: float = 0.30
    min_time_s: float = 0.03
    assume_ballistic: bool = True

    @classmethod
    def from_dict(cls, d) -> 'PredictionConfig':
        d = dict(d or {})
        known = {k: d[k] for k in cls.__dataclass_fields__ if k in d}
        return cls(**known)


def _is_fast(track, cfg: PredictionConfig) -> bool:
    if int(getattr(track, 'frames_seen', 0)) < cfg.min_frames:
        return False
    v = np.asarray(track.velocity, dtype=float)
    s = float(np.linalg.norm(v))
    if not np.isfinite(s) or s < cfg.min_speed:
        return False
    u = v / s
    C = np.asarray(track.velocity_cov, dtype=float)
    sig = float(np.sqrt(max(float(u @ C @ u), 0.0)))
    return s - cfg.k_sigma * sig >= cfg.min_speed


def _ballistic(track, cfg: PredictionConfig) -> bool:
    mp = getattr(track, 'mode_prob', None)
    if mp is not None:
        return float(np.asarray(mp)[1]) >= 0.5
    return cfg.assume_ballistic


def predict_trajectory(track, cfg: PredictionConfig) -> tuple:
    """(taus, positions) of the predicted path, base frame."""
    taus = np.arange(0.0, cfg.horizon_s + 1e-9, cfg.step_s)
    p = np.asarray(track.position, dtype=float)
    v = np.asarray(track.velocity, dtype=float)
    P = p + np.outer(taus, v)
    if _ballistic(track, cfg):
        P = P + 0.5 * np.outer(taus ** 2, _G)
    return taus, P


def add_predicted_hits(cp_results: List, tracks: Iterable, cfg: PredictionConfig) -> tuple:
    """``(cp_results', n_hits)``: a copy of the control-point results with the
    predicted hits appended to ``extras``. Results are never mutated in place.
    """
    if not cfg.enabled:
        return cp_results, 0
    paths = [predict_trajectory(t, cfg) for t in tracks if _is_fast(t, cfg)]
    if not paths:
        return cp_results, 0
    out, n = [], 0
    for r in cp_results:
        c = np.asarray(r.point, dtype=float)
        best = None
        for taus, P in paths:
            g = np.linalg.norm(P - c, axis=1) - float(r.radius) - cfg.object_radius_m
            i = int(np.argmin(g))
            if taus[i] < cfg.min_time_s or g[i] >= cfg.publish_gap_m:
                continue
            if best is None or g[i] < best[0]:
                best = (float(g[i]), P[i])
        if best is None:
            out.append(r)
            continue
        gap, p_hit = best
        d = c - p_hit
        nd = float(np.linalg.norm(d))
        if nd < 1e-9:
            out.append(r)
            continue
        hit = ObstacleHit(distance=max(gap, 0.0), point=p_hit, direction=d / nd,
                          cluster_id=PREDICTED_CLUSTER_ID, range_m=None)
        out.append(replace(r, extras=list(getattr(r, 'extras', ())) + [hit]))
        n += 1
    return out, n
