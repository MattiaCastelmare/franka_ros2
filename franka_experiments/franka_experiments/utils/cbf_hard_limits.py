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
    relax_dt: float | None = None,   # approach horizon; None → dt (legacy)
) -> tuple[np.ndarray, np.ndarray]:
    """Per-joint q̈ box enforcing velocity AND position limits. Returns (lb, ub).

    Feasibility guard: if lb > ub (conflicting demands, e.g. diving toward the
    lower position limit while already at −v_cap), the box collapses onto lb —
    the bound that encodes "brake away from the lower limit / don't exceed
    −q̈_max" — matching the pre-existing velocity-box guard semantics.

    ``relax_dt`` decouples APPROACHING a cap from being OVER it:

    * below the cap the allowed q̈ is ``(v_bound − q̇)/relax_dt``. A longer
      horizon means a smaller allowed acceleration, so the bound starts acting
      well before the cap instead of only in the final ``q̈_max·dt`` sliver.
      With dt = 10 ms that sliver is 0.17 rad/s on joint5 — measured on
      hardware, joint5 ramped from 39% to 70% of its limit with the box never
      once engaging (``vbite`` empty on every log line) and the firmware
      aborted on ``joint_velocity_violation`` before it ever bit.
    * once past the cap the one-step ``dt`` is used again, so braking authority
      is never softened when it is actually needed.

    ``None`` reproduces the legacy one-step behaviour exactly.
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

    # Approaching a bound → relax_dt (early, gentle). Already past it → dt
    # (one step, full authority). np.where keeps this branch-free and per-joint.
    rdt = dt if relax_dt is None else float(relax_dt)
    dt_ub = np.where(v_ub >= qdot, rdt, dt)
    dt_lb = np.where(v_lb <= qdot, rdt, dt)

    ub = np.minimum(acc_ub, (v_ub - qdot) / dt_ub)
    lb = np.maximum(acc_lb, (v_lb - qdot) / dt_lb)
    ub = np.maximum(ub, lb)          # feasibility guard (priority to lb)
    return lb, ub


def position_velocity_accel_box(
    q: np.ndarray,
    qdot: np.ndarray,
    *,
    acc_lb: np.ndarray,
    acc_ub: np.ndarray,
    qdot_max: np.ndarray,
    v_margin: float,
    q_min: np.ndarray,
    q_max: np.ndarray,
    q_margin: float,
    brake_eta: float,
    dt: float,
    relax_dt: float | None = None,
    out_lb: np.ndarray = None,   # (n,) OUTPUT buffer, written in place
    out_ub: np.ndarray,      # (n,) OUTPUT buffer, written in place
) -> tuple[np.ndarray, np.ndarray]:
    """:func:`hard_accel_box` with the calling convention the QP loop needs.

    Same box, but written into the caller's pre-allocated buffers and returning
    the ``(ratio, bite)`` diagnostics that :func:`velocity_accel_box` returned —
    so ``cbf_safety_filter`` can swap one for the other without touching its
    CBFDIAG / VELHI logging.

    The math is NOT duplicated here: this delegates to :func:`hard_accel_box`,
    which is the version covered by test_cbf_hard_constraints. It therefore
    allocates a handful of (n,) temporaries per call — negligible next to the
    sparse matrix ``build_osqp_A`` builds on the same tick, and worth it to keep
    a single tested implementation of the braking curve.

    Returns:
        ``(ratio, bite)``. *ratio* is ``|q̇| / q̇_max`` per joint. *bite* marks
        joints whose box was tightened by the position or velocity bound rather
        than by the static accel box — note this is BROADER than the velocity-
        only *bite* it replaces, since a joint approaching a position limit now
        also lights up.
    """
    lb, ub = hard_accel_box(
        q, qdot, acc_lb=acc_lb, acc_ub=acc_ub, qdot_max=qdot_max,
        v_margin=v_margin, q_min=q_min, q_max=q_max, q_margin=q_margin,
        brake_eta=brake_eta, dt=dt, relax_dt=relax_dt)
    out_lb[:] = lb
    out_ub[:] = ub
    ratio = np.abs(qdot) / qdot_max
    bite = (ub < acc_ub - 1e-9) | (lb > acc_lb + 1e-9)
    return ratio, bite


def apply_slew_limit(
    lb: np.ndarray,
    ub: np.ndarray,
    qddot_prev: np.ndarray,
    delta: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Intersect (lb, ub) with the continuity box qddot_prev ± delta.

    LIVE in cbf_safety_filter._qp_tick since the command-discontinuity failure:
    the QP solution can jump when the 50 Hz constraint snapshot moves, and the
    arm cannot track a step (measured trk_err ~9.5 rad/s² after a 2.4x jump in
    dnorm inside one 60 ms window).

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


# TODO[LEGACY]: used only by test/test_cbf_hard_constraints.py; workspace rows reverted in 4d4d450 | confidence: medium | superseded-by: none | flagged: 2026-09-01
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


def velocity_accel_box(
    qdot: np.ndarray,
    *,
    acc_lb: np.ndarray,      # (n,) static lower accel bound (< 0)
    acc_ub: np.ndarray,      # (n,) static upper accel bound (> 0)
    qdot_max: np.ndarray,    # (n,) official per-joint |qdot| limit
    v_margin: float,         # fraction of qdot_max actually allowed
    dt: float,               # one-step horizon (nominal QP period)
    out_lb: np.ndarray,      # (n,) OUTPUT buffer, written in place
    out_ub: np.ndarray,      # (n,) OUTPUT buffer, written in place
) -> tuple[np.ndarray, np.ndarray]:
    """Tighten the per-joint qddot box so |qdot| cannot exceed v_margin*qdot_max.

    MOVED here from ``CBFSafetyFilter._update_velocity_box`` in Phase 3. The
    arithmetic and its ordering are unchanged; only the destination of the two
    box arrays became an explicit output-buffer argument, so the node keeps
    owning its pre-allocated storage and this stays allocation-neutral on the
    100 Hz path (the two diagnostic arrays were freshly allocated before and
    still are).

    One-step bound: after one dt the velocity qdot + qddot*dt must stay within
    +/-v_margin*qdot_max, intersected with the static decel box::

        qddot_ub = min( +decel,  (+v_margin*qdot_max - qdot)/dt )
        qddot_lb = max( -decel,  (-v_margin*qdot_max - qdot)/dt )

    Anti-asymmetry (by design): near +qdot_max the upper bound collapses toward
    0 while the lower bound stays at -decel, so full braking authority AWAY from
    an obstacle is always retained; only the velocity-increasing direction is
    curtailed.

    SUPERSEDED in cbf_safety_filter by position_velocity_accel_box, which adds
    the position-limit braking curve on top of this. Kept: it is the
    velocity-only box, still the right choice for a caller that has no position
    limits to enforce, and franka_sim carries its own copy of both.

    Args:
        qdot: (n,) measured joint velocities [rad/s].
        acc_lb: (n,) static lower acceleration bound (negative).
        acc_ub: (n,) static upper acceleration bound (positive).
        qdot_max: (n,) official per-joint velocity limit [rad/s].
        v_margin: Fraction of *qdot_max* actually allowed, in (0, 1].
        dt: One-step horizon [s] (the nominal QP period, not the measured one).
        out_lb: (n,) buffer receiving the lower qddot bound; written in place.
        out_ub: (n,) buffer receiving the upper qddot bound; written in place.

    Returns:
        ``(ratio, bite)`` diagnostics: *ratio* is ``|qdot| / qdot_max`` per
        joint, *bite* is a boolean mask marking joints whose box was tightened
        by the velocity bound rather than by the static decel box. Both are
        DIAGNOSTIC ONLY and are never read by the QP.
    """
    vmax   = v_margin * qdot_max
    ub_vel = (vmax - qdot) / dt
    lb_vel = (-vmax - qdot) / dt
    ub = np.minimum(acc_ub, ub_vel)
    lb = np.maximum(acc_lb, lb_vel)
    # Feasibility guard: if already past v_margin*qdot_max by more than one
    # tick's decel authority, the velocity bound would invert the box (lb > ub).
    # Clamp ub UP to lb (NOT lb down — that could exceed -decel and violate the
    # accel limit) → forces qddot = -decel, i.e. hardest legal braking.
    ub = np.maximum(ub, lb)

    ratio = np.abs(qdot) / qdot_max
    bite  = (ub < acc_ub - 1e-9) | (lb > acc_lb + 1e-9)

    out_lb[:] = lb
    out_ub[:] = ub
    return ratio, bite
