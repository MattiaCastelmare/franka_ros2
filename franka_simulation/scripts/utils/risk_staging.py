"""Risk staging utilities (shared by controller + blender).

Purpose
-------
Provide a *single* place for the 30/20/10/5 cm staging logic so that:
- distance-based gates are consistent across nodes
- filters and constraint enforcement can be risk-scaled without duplicating logic

This module is numpy-only (real-time friendly, no ROS imports).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def _smoothstep01(x: float) -> float:
    x = float(np.clip(float(x), 0.0, 1.0))
    return float(3.0 * x * x - 2.0 * x * x * x)


def ramp_down_distance(*, d: float, d_hi: float, d_lo: float) -> float:
    """Return 0 when d>=d_hi, 1 when d<=d_lo (smoothstep in between)."""
    d_hi = float(d_hi)
    d_lo = float(d_lo)
    if d_hi <= d_lo + 1e-12:
        return 1.0 if float(d) <= float(d_lo) else 0.0
    t = (float(d_hi) - float(d)) / (float(d_hi) - float(d_lo))
    return _smoothstep01(t)


@dataclass
class RiskStaging:
    d_eff: float
    w_far: float
    w_mid: float
    w_near: float
    w_total: float
    stop_gate: bool


def compute_risk_staging(
    *,
    d_eff: float,
    d_far: float,
    d_mid: float,
    d_near: float,
    d_stop: float,
) -> RiskStaging:
    """Compute staging weights from an effective distance $d_{eff}$.

    Assumes: d_far > d_mid > d_near > d_stop.

    The weights are overlapping-but-normalized so that:
      0 <= w_far,w_mid,w_near <= 1
      0 <= w_total <= 1
    and w_total increases as distance decreases.
    """
    d_eff = float(d_eff)

    d_far = float(d_far)
    d_mid = float(d_mid)
    d_near = float(d_near)
    d_stop = float(d_stop)

    # Monotonic guard (do not crash if YAML has a typo).
    d_far = max(d_far, d_mid + 1e-6)
    d_mid = max(d_mid, d_near + 1e-6)
    d_near = max(d_near, d_stop + 1e-6)

    r_far = ramp_down_distance(d=d_eff, d_hi=d_far, d_lo=d_mid)
    r_mid = ramp_down_distance(d=d_eff, d_hi=d_mid, d_lo=d_near)
    r_near = ramp_down_distance(d=d_eff, d_hi=d_near, d_lo=d_stop)

    w_near = float(np.clip(r_near, 0.0, 1.0))
    w_mid = float(np.clip(r_mid * (1.0 - w_near), 0.0, 1.0))
    w_far = float(np.clip(r_far * (1.0 - w_mid - w_near), 0.0, 1.0))

    w_total = float(np.clip(w_far + w_mid + w_near, 0.0, 1.0))
    stop_gate = bool(d_eff <= d_stop)

    return RiskStaging(
        d_eff=float(d_eff),
        w_far=float(w_far),
        w_mid=float(w_mid),
        w_near=float(w_near),
        w_total=float(w_total),
        stop_gate=bool(stop_gate),
    )
