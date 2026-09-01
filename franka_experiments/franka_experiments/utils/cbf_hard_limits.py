"""Hard state-limit shielding for the acceleration-level CBF filter.

Pure-numpy helpers (no ROS, no Pinocchio) used by ``cbf_safety_filter``:

* :func:`hard_accel_box` — per-joint q̈ bounds that keep BOTH the joint
  velocity AND the joint position inside their limits, built as an
  intersection of three boxes:

    1. static accel/decel authority  ±q̈_max
    2. one-step velocity bound       |q̇ + q̈·Δt| ≤ v_cap
    3. position-aware velocity cap   v_cap shrinks near q_min/q_max with the
       trapezoid braking curve  v = √(2·η·a·h)  (h = distance to the limit)

  The braking-curve cap is the classical viability bound: approaching a
  position limit at speed v is admissible only if v² ≤ 2·η·a_max·h, i.e. the
  joint can still stop before the limit using a fraction η of its decel
  authority.  Riding the curve demands exactly η·a_max of deceleration, so
  with η < 1 the demanded braking never exceeds the static box and the two
  bounds cannot structurally conflict.

* :func:`apply_slew_limit` — intersects a box with the accel-continuity box
  |q̈ − q̈_prev| ≤ Δ.  Guarantees the returned box is non-empty: if the safety
  box lies entirely outside the slew box, the output pins q̈ to the nearest
  slew edge — i.e. the command moves toward the safety box at the maximum
  rate the continuity limit allows.  This is the deliberate priority order:
  continuity (firmware `torque_discontinuity` reflex) > instantaneous
  saturation of the state bound, with the position/velocity margins sized to
  absorb the (one/two tick) transient.

* :func:`workspace_face_rows` — HOCBF row ingredients for a Cartesian
  axis-aligned box on one robot point (the EE): one row per near face.
"""

from __future__ import annotations

import numpy as np


def hard_accel_box(
    q: np.ndarray,
    qdot: np.ndarray,
    *,
    acc_lb: np.ndarray,      # (n,) static lower accel bound (< 0)
    acc_ub: np.ndarray,      # (n,) static upper accel bound (> 0)
    qdot_max: np.ndarray,    # (n,) official per-joint |q̇| limit
    v_margin: float,         # fraction of qdot_max actually allowed (e.g. 0.9)
    q_min: np.ndarray,
    q_max: np.ndarray,
    q_margin: float,         # [rad] stay this far from the position limits
    brake_eta: float,        # fraction of decel authority for the braking curve
    dt: float,               # one-step horizon (nominal QP period)
) -> tuple[np.ndarray, np.ndarray]:
    """Per-joint q̈ box enforcing velocity AND position limits. Returns (lb, ub).

    Feasibility guard: if lb > ub (conflicting demands, e.g. diving toward the
    lower position limit while already at −v_cap), the box collapses onto lb —
    the bound that encodes "brake away from the lower limit / don't exceed
    −q̈_max" — matching the pre-existing velocity-box guard semantics.
    """
    # Distance to the (margin-shrunk) position limits, floored at 0 so a
    # config already past the margin yields v_cap = 0 (full stop demand),
    # never a NaN.
    h_up = np.maximum(q_max - q_margin - q, 0.0)
    h_lo = np.maximum(q - q_margin - q_min, 0.0)

    # Decel authority available for the braking curve (symmetric authority =
    # min of the two static bounds, scaled by η < 1 so riding the curve never
    # saturates the static box).
    a_auth = brake_eta * np.minimum(np.abs(acc_lb), np.abs(acc_ub))

    v_cap = v_margin * qdot_max
    v_ub = np.minimum(v_cap, np.sqrt(2.0 * a_auth * h_up))
    v_lb = np.maximum(-v_cap, -np.sqrt(2.0 * a_auth * h_lo))

    ub = np.minimum(acc_ub, (v_ub - qdot) / dt)
    lb = np.maximum(acc_lb, (v_lb - qdot) / dt)
    ub = np.maximum(ub, lb)          # feasibility guard (priority to lb)
    return lb, ub


def apply_slew_limit(
    lb: np.ndarray,
    ub: np.ndarray,
    qddot_prev: np.ndarray,
    delta: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Intersect (lb, ub) with the continuity box qddot_prev ± delta.

    Always returns a non-empty box: where the two boxes are disjoint the
    result is pinned to the slew edge nearest to the safety box, so the
    command approaches the (temporarily unreachable) safety bound at the
    maximum admissible accel rate instead of jumping.
    """
    lo_s = qddot_prev - delta
    hi_s = qddot_prev + delta
    lo = np.maximum(lb, lo_s)
    hi = np.minimum(ub, hi_s)

    # Disjoint cases: safety box entirely above / below the slew box.
    above = lb > hi_s                # must accelerate up faster than allowed
    below = ub < lo_s                # must accelerate down faster than allowed
    lo = np.where(above, hi_s, np.where(below, lo_s, lo))
    hi = np.where(above, hi_s, np.where(below, lo_s, hi))
    return lo, hi


def workspace_face_rows(
    p: np.ndarray,        # (3,) constrained point, world/base frame
    Jp: np.ndarray,       # (3, nv) point Jacobian
    Jpd_qd: np.ndarray,   # (3,)   J̇p @ q̇  (drift term)
    ws_min: np.ndarray,   # (3,)
    ws_max: np.ndarray,   # (3,)
    margin: float,        # [m] barrier zero this far inside each face
    horizon: float,       # [m] emit a row only when h < horizon
) -> list[tuple[np.ndarray, float, float, str]]:
    """HOCBF row ingredients for an axis-aligned Cartesian box.

    For each of the 6 faces, barrier h = distance inside the (margin-shrunk)
    box along that axis.  Only faces with h < horizon yield a row (the linear
    HOCBF self-deactivates at large h anyway; the horizon merely caps row
    count).  Returns tuples ``(a_row (nv,), h, jdq, label)`` with the same
    semantics as the obstacle rows:  a·q̈ + jdq ≥ −k1·(a·q̇) − k0·h.
    """
    rows: list[tuple[np.ndarray, float, float, str]] = []
    axes = ('x', 'y', 'z')
    for k in range(3):
        # Lower face:  h = p_k − ws_min_k − margin,  ḣ = +ṗ_k
        h_lo = float(p[k] - ws_min[k] - margin)
        if h_lo < horizon:
            rows.append((Jp[k].copy(), h_lo, float(Jpd_qd[k]),
                         f'ws:{axes[k]}-'))
        # Upper face:  h = ws_max_k − margin − p_k,  ḣ = −ṗ_k
        h_hi = float(ws_max[k] - margin - p[k])
        if h_hi < horizon:
            rows.append((-Jp[k], h_hi, float(-Jpd_qd[k]),
                         f'ws:{axes[k]}+'))
    return rows
