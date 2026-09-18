"""Lateral evasion: what to do when backing off is no longer physically possible.

WHY THIS EXISTS
---------------
Every obstacle row in this filter demands the same thing: accelerate the control
point ALONG n̂, away from the obstacle. That is the right answer while it is
achievable, and it is the only answer the filter currently knows.

It stops being achievable, and the failure is arithmetic rather than a matter of
tuning. The barrier closes at ``ḣ = aᵀq̇ − v_obs``. To arrest that closing rate
the robot must produce ``aᵀq̈ > 0``, and the acceleration box bounds how much:

    a_max = Σⱼ max(aⱼ·lb_j, aⱼ·ub_j)        [m/s²] along n̂

so the shortest distance in which the robot can null a closing rate ``ḣ < 0`` is
``ḣ²/(2·a_max)``. When that exceeds the remaining gap ``h``, NO admissible
acceleration keeps the barrier positive. The QP does not fail loudly at that
point — it pays slack, reports a violated row, and the arm backs off as hard as
it can into a collision it was never able to prevent. That is precisely the
"fast obstacle" case: a hand moving at 1.5 m/s covers 5 cm in the 33 ms between
depth frames, and no gain retunes the robot's inertia.

But "cannot stop the gap from closing" is NOT the same as "cannot avoid the
obstacle". A human whose hand is about to be hit does not push back along the
line of approach; they move the hand SIDEWAYS, out of the swept volume. That
option exists here too, it is usually far cheaper in acceleration than braking
along n̂, and until now nothing in the filter could ask for it — because asking
for it requires knowing WHICH WAY the obstacle is going, and the scalar residual
``aᵀq̇ − ḋ`` contains no direction at all. The tracked 3D velocity does.

This module is that missing piece, and it is the reason the tracker earns its
place beyond a better-conditioned ``v_obs``.

THE ESCAPE DIRECTION
--------------------
Let ``d = p_robot − p_obs`` and let the obstacle travel along ``v̂``. Decompose

    d_∥ = (d·v̂)v̂        along the line of travel
    d_⊥ = d − d_∥        perpendicular offset from that line

``‖d_⊥‖`` IS the miss distance: the closest the obstacle will come to the
control point if both keep going. So the direction that increases the miss
distance fastest is ``d̂_⊥`` — perpendicular to the obstacle's velocity, pointing
away from its path. That is "sideways" made precise, and note it is NOT n̂: for
a glancing approach the two are nearly orthogonal, and moving along n̂ barely
changes the miss distance at all.

The degenerate case ``‖d_⊥‖ → 0`` is a head-on collision course, where every
perpendicular direction increases the miss distance at the same first-order
rate and geometry has no preference. There the choice is made on the robot's
side instead: the perpendicular direction the arm can actually accelerate along
hardest, i.e. the one maximising ``‖êᵀJ_p‖``. That is the top eigenvector of a
2×2 matrix and costs nothing.

HOW IT ENTERS THE QP — AND WHY NOT AS A CONSTRAINT
--------------------------------------------------
As a BIAS on ``q̈_nom``, exactly like ``tangential_bias``, never as a row.

A row would be a hard demand, and adding hard demands to a QP that is already
infeasible enough to be paying slack is how a filter starts fighting itself: the
escape row and the barrier row would both be relaxed by the same shared slack,
and the solver would trade safety for evasion with no way to say which it
preferred. A bias moves the OBJECTIVE instead. Every barrier stays exactly as
binding as it was, the robot's limits are untouched, and the QP spends the part
of ``q̈`` that the barrier has no opinion about — which is free, and which today
sits at zero — on getting out of the way. The worst case for a wrong escape
direction is therefore a slightly worse tracking error, never a violated
barrier.

It is also why this is separate from ``tangential_bias`` rather than folded into
it. That function answers "the barrier blocks one direction, use the others",
from the arm's own intent; this one answers "the barrier cannot be satisfied at
all, get out of the swept volume", from the obstacle's measured velocity.
Different trigger, different data, different gain, and they add.

Pure numpy, no ROS, no Pinocchio — the Jacobian arrives as an array.
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np


def normal_brake_authority(a: np.ndarray, acc_lb: np.ndarray,
                           acc_ub: np.ndarray, *, eta: float = 1.0) -> float:
    """[m/s²] the largest ``aᵀq̈`` the acceleration box still allows.

    The box is per-joint and independent, so the maximum of a linear form over
    it is separable and EXACT — no optimisation, no bound:

        max{ aᵀq̈ : lb ≤ q̈ ≤ ub } = Σⱼ max(aⱼ·lbⱼ, aⱼ·ubⱼ)

    This is the robot's real ability to push the control point along n̂ IN THIS
    CONFIGURATION, which is the whole point: near a singularity ``‖a‖`` collapses
    and the same joint box buys a fraction of the Cartesian authority it buys in
    a well-conditioned pose. A constant "max deceleration" would be blind to
    exactly the situation that matters.

    Args:
        a: (nv,) the row direction ``n̂ᵀJ_p``.
        acc_lb / acc_ub: (nv,) the acceleration box actually handed to the QP —
            pass the VELOCITY-TIGHTENED box, not the static limits, or the
            authority is overstated for a joint already near its speed limit.
        eta: fraction of the box reserved for this. < 1 because the same
            acceleration budget also has to serve every other row and the
            tracking objective; claiming all of it would declare the situation
            feasible right up to the point where it is only feasible if the
            robot abandons everything else.

    Returns:
        A non-negative authority. Exactly 0.0 when the row has no leverage,
        which the caller must read as "cannot brake at all" rather than
        dividing by it.
    """
    a = np.asarray(a, dtype=np.float64).ravel()
    lb = np.asarray(acc_lb, dtype=np.float64).ravel()
    ub = np.asarray(acc_ub, dtype=np.float64).ravel()
    return max(float(eta) * float(np.sum(np.maximum(a * lb, a * ub))), 0.0)


def evasion_urgency(h: float, h_dot: float, brake_authority: float, *,
                    engage_ratio: float = 0.7, h_floor: float = 0.01) -> float:
    """How badly the normal direction is losing, as a weight in ``[0, 1]``.

        stop = ḣ² / (2·a_max)        distance needed to null the closing rate
        r    = stop / max(h, h_floor)

    ``r < 1`` means the robot can still stop the gap from closing before it
    reaches zero; ``r ≥ 1`` means it cannot, whatever the QP does. The weight
    ramps over ``[engage_ratio, 1]`` as a SMOOTHSTEP rather than switching at
    ``r = 1``:

    * a hard switch on a 30 Hz vision-derived quantity chatters, and the thing
      being switched is a bias on ``q̈_nom`` — the arm would jerk sideways and
      back at whatever rate the noise crosses the threshold. That is the exact
      failure ``tangential_bias`` v1 was rewritten for, and this module is not
      going to reintroduce it one file over.
    * engaging slightly BEFORE the situation is provably hopeless is the whole
      value: an evasion started at ``r = 0.7`` has room to work, one started at
      ``r = 1.0`` starts at the moment it stops mattering.

    Args:
        h: [m] barrier value — the surface gap minus ``d_safe``. Values ≤ 0
          (already inside the safety distance) saturate the weight, which is the
          intended reading: there is nothing left to brake into.
        h_dot: [m/s] the barrier's rate, ``aᵀq̇ − v_obs``. POSITIVE means
            separating; only the closing half produces any urgency at all, so a
            fast obstacle moving AWAY never triggers evasion.
        brake_authority: [m/s²] from :func:`normal_brake_authority`.
        engage_ratio: where the ramp starts.
        h_floor: [m] floor on the denominator, so a barrier at exactly zero
            gives a large ratio rather than an infinity.

    Returns:
        ``0.0`` when the obstacle is receding, when the robot has ample braking
        distance, or when there is no authority to reason about — and ``1.0``
        when braking demonstrably cannot succeed.
    """
    if h_dot >= 0.0:
        return 0.0                       # separating: nothing to escape from
    if brake_authority <= 1e-9:
        # No leverage along n̂ at all. Braking is not merely insufficient, it is
        # unavailable — the most urgent case there is, and the one where moving
        # sideways is the only option left.
        return 1.0
    stop = (h_dot * h_dot) / (2.0 * brake_authority)
    r = stop / max(float(h), float(h_floor))
    lo = float(engage_ratio)
    if r <= lo:
        return 0.0
    if r >= 1.0:
        return 1.0
    s = (r - lo) / (1.0 - lo)
    return float(s * s * (3.0 - 2.0 * s))          # smoothstep


def escape_direction(
    d_vec: np.ndarray,
    v_obs_vec: np.ndarray,
    Jp: Optional[np.ndarray] = None,
    *,
    v_min: float = 0.05,
    perp_min: float = 0.02,
) -> Optional[np.ndarray]:
    """(3,) unit direction that grows the miss distance fastest, or ``None``.

    See the module docstring for the derivation. ``None`` means "no meaningful
    lateral exists", and the caller must then contribute no bias at all —
    falling back to the normal retreat the barrier already demands, i.e. to
    today's behaviour.

    Args:
        d_vec: (3,) ``p_robot − p_obs``, base frame.
        v_obs_vec: (3,) obstacle velocity, base frame, from the track.
        Jp: (3, nv) point Jacobian, used ONLY to break the head-on tie by
            choosing the perpendicular direction the arm can accelerate along
            hardest. ``None`` falls back to a deterministic geometric choice.
        v_min: [m/s] below this the obstacle is not meaningfully moving and
            "perpendicular to its velocity" is a direction fitted to noise.
        perp_min: [m] below this offset the approach is head-on and the tie is
            broken on the robot's side instead of the geometry's.

    Returns:
        A unit vector, or ``None``.
    """
    d = np.asarray(d_vec, dtype=np.float64).ravel()
    v = np.asarray(v_obs_vec, dtype=np.float64).ravel()
    if d.size != 3 or v.size != 3 or not (np.all(np.isfinite(d))
                                          and np.all(np.isfinite(v))):
        return None
    v_norm = float(np.linalg.norm(v))
    if v_norm < float(v_min):
        return None
    v_hat = v / v_norm

    d_perp = d - float(d @ v_hat) * v_hat
    n_perp = float(np.linalg.norm(d_perp))
    if n_perp >= float(perp_min):
        return d_perp / n_perp

    # ── Head-on: every perpendicular direction is geometrically equivalent ──
    # Pick the one the arm can actually move along. Build an orthonormal basis
    # (u1, u2) of the plane ⊥ v̂ and maximise ‖(c₁u₁ + c₂u₂)ᵀJp‖ over unit c,
    # which is the top eigenvector of the 2×2 Gram matrix M Mᵀ with
    # M = [u₁ᵀJp ; u₂ᵀJp]. Closed form, no SVD, deterministic.
    u1 = np.cross(v_hat, np.array([0.0, 0.0, 1.0]))
    if float(np.linalg.norm(u1)) < 1e-6:
        u1 = np.cross(v_hat, np.array([0.0, 1.0, 0.0]))
    u1 /= np.linalg.norm(u1)
    u2 = np.cross(v_hat, u1)
    u2 /= np.linalg.norm(u2)
    if Jp is None:
        e = u1
    else:
        M = np.vstack([u1 @ np.asarray(Jp, dtype=np.float64),
                       u2 @ np.asarray(Jp, dtype=np.float64)])   # (2, nv)
        w, V = np.linalg.eigh(M @ M.T)
        c = V[:, int(np.argmax(w))]
        e = c[0] * u1 + c[1] * u2
        n_e = float(np.linalg.norm(e))
        if n_e < 1e-9:
            e = u1
        else:
            e = e / n_e
    # Sign: the eigenvector's is arbitrary, so resolve it toward whatever tiny
    # perpendicular offset does exist (keep going the way we are already off
    # the line), and deterministically otherwise — an unresolved sign would
    # flip frame to frame and the arm would shake instead of moving.
    ref = d_perp if n_perp > 1e-12 else np.array([1.0, 1.0, 1.0])
    if float(e @ ref) < 0.0:
        e = -e
    elif float(e @ ref) == 0.0 and float(e[np.argmax(np.abs(e))]) < 0.0:
        e = -e
    return e


def evasion_bias(rows, *, gain: float, max_bias: float,
                 min_leverage: float = 1e-3) -> np.ndarray:
    """(nv,) acceleration bias to ADD to ``q̈_nom``, or an exact zero vector.

    Args:
        rows: iterable of ``(g, w)`` where ``g = êᵀJ_p`` is the escape direction
            expressed in JOINT space (nv,) and ``w ∈ [0, 1]`` is that control
            point's urgency from :func:`evasion_urgency`.
        gain: [rad/s²] the bias magnitude at full urgency for a single control
            point. This is an ACCELERATION, not a dimensionless factor, so it
            can be read against the joint acceleration box directly.
        max_bias: [rad/s²] norm cap on the total. Several control points on the
            same link facing the same obstacle all contribute, and without a cap
            their sum scales with how finely the arm happens to be discretised
            into control points — a modelling detail that must not change how
            hard the robot swerves.
        min_leverage: rows whose ``‖g‖`` is below this are dropped: the escape
            direction exists in Cartesian space but the arm has no way to move
            along it from this configuration, and normalising a near-null vector
            would turn numerical noise into a full-strength command.

    Returns:
        ``np.zeros(nv)`` exactly when nothing is engaged, so a caller EMA-ing
        this decays cleanly to zero on disengagement.
    """
    # The accumulator is sized from the FIRST row, whatever its urgency, so the
    # return shape is (nv,) for any non-empty input. Sizing it from the first
    # ENGAGED row instead returned a zero-length array when every row was idle,
    # which is a different type for the same meaning — and a caller adding it to
    # q̈_nom would broadcast-fail or, worse, silently produce an empty result.
    bias = None
    for g, w in rows:
        g = np.asarray(g, dtype=np.float64).ravel()
        if bias is None:
            bias = np.zeros(g.size)
        if w <= 0.0:
            continue
        n_g = float(np.linalg.norm(g))
        if n_g < min_leverage or not np.isfinite(n_g):
            continue
        bias += float(w) * (g / n_g)
    if bias is None:
        return np.zeros(0)                     # no rows at all
    bias *= float(gain)
    n_b = float(np.linalg.norm(bias))
    if max_bias > 0.0 and n_b > max_bias:
        bias *= max_bias / n_b
    return bias
