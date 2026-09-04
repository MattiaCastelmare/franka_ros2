"""Extra HOCBF rows for the QP: self-collision and joint position limits.

Both were previously handled — badly — outside the QP:

* self-collision was a post-solve VETO that replaced the command wholesale, so
  the arm stopped dead instead of steering around the problem;
* joint position limits were a per-joint BOX clip, a hard wall that saturates
  one joint at the last moment. Measured on hardware: joint4 sitting exactly on
  its braking curve had its box collapse to ``lb = 0``, the commander kept
  asking for -4 rad/s², and the 4 rad/s² clip on that single joint produced
  ``dnorm=10.989`` with ``dq_ort=10.941`` — the whole deviation dumped
  orthogonally to anything the CBF was doing, a command the arm could not track
  (``trk_err=8.428``), ending in a ``joint_velocity_violation`` reflex.

As HOCBF rows both engage GRADUALLY and the QP spreads the correction across
joints in the least-squares sense, exactly as it already does for obstacles.
The velocity/position box stays underneath as the hard floor — a row can be
relaxed by the slack, a box cannot.

Row convention (identical to the obstacle rows in cbf_safety_filter):

    aᵀ q̈ + s  ≥  −k1·(aᵀ q̇) − k0·h − ċ

so a builder returns ``(a, h, jdq, label)`` and the node appends it to the same
``rows_a / rows_h / rows_jdq / rows_link`` lists.

Everything here is pure numpy plus the caller's kinematics object; no ROS.
"""

from __future__ import annotations

from typing import List, NamedTuple, Optional, Sequence, Tuple

import numpy as np

from franka_experiments.utils.self_collision import (
    Capsule,
    segment_segment_closest,
)

Row = Tuple[np.ndarray, float, float, str]


# ── Joint position limits ────────────────────────────────────────────────────

def joint_limit_rows(
    q: np.ndarray,
    q_min: np.ndarray,
    q_max: np.ndarray,
    margin: float,
    horizon: float,
    nv: int,
    qdot: Optional[np.ndarray] = None,
    a_auth: Optional[np.ndarray] = None,
    lead: float = 1.5,
) -> List[Row]:
    """HOCBF rows keeping each joint inside ``[q_min+margin, q_max-margin]``.

    Two barriers per joint:

        upper:  h = q_max − margin − q ,  ḣ = −q̇  ⇒  a = −e_j
        lower:  h = q − margin − q_min ,  ḣ = +q̇  ⇒  a = +e_j

    ``ċ = 0`` exactly: ḧ = ∓q̈ has no drift term, unlike the Cartesian rows
    where J̇q̇ appears.

    A row is emitted only while ``h < horizon``; beyond that the linear HOCBF is
    non-binding anyway and the row would only inflate ``n_c`` (and with it OSQP
    ``setup()`` churn). Rows come out in a deterministic order — joint index,
    then upper before lower — so the QP's sparsity pattern is stable frame to
    frame and the warm start stays valid.

    VELOCITY-AWARE HORIZON (``qdot`` and ``a_auth`` given). A fixed distance
    horizon has a failure mode that was measured on hardware: the position
    braking curve in ``hard_accel_box`` starts clipping a joint as soon as

        h < q̇² / (2·a_auth)

    which for joint2 at 1.02 rad/s is 0.337 rad — BEYOND the 0.30 rad row
    horizon. So the hard, non-negotiable box clipped the joint while the soft,
    negotiable row did not yet exist. The QP could not trade that joint against
    the others, so it dumped the correction orthogonally instead: ``dq_ort``
    reached 19.5 rad/s² with ``vbite`` stuck on joint2 and ``s[qlim]`` at zero
    — the row was never even asked. That is the same shape as the joint4
    incident in this module's header, and it ends the same way.

    With ``qdot``/``a_auth`` the horizon becomes ``max(horizon, lead·q̇²/(2·a))``
    in the direction of travel only — a joint moving AWAY from a limit gets no
    inflation — so the row is guaranteed to exist before the box bites, and
    ``lead`` (> 1) sets how far ahead. The row still self-deactivates through
    ``−k0·h``; this only controls when it is present at all.

    ``qdot=None`` reproduces the fixed-horizon behaviour exactly.
    """
    rows: List[Row] = []
    vel_aware = qdot is not None and a_auth is not None
    for j in range(nv):
        # Stopping distance in the direction of travel, or 0 when the joint is
        # moving away from that limit (no inflation for a receding joint).
        if vel_aware:
            v = float(qdot[j])
            aa = max(float(a_auth[j]), 1e-9)
            stop_up = (v * v) / (2.0 * aa) if v > 0.0 else 0.0
            stop_lo = (v * v) / (2.0 * aa) if v < 0.0 else 0.0
            hor_up = max(horizon, lead * stop_up)
            hor_lo = max(horizon, lead * stop_lo)
        else:
            hor_up = hor_lo = horizon

        h_up = float(q_max[j] - margin - q[j])
        if h_up < hor_up:
            a = np.zeros(nv)
            a[j] = -1.0
            rows.append((a, h_up, 0.0, f'q{j + 1}+'))
        h_lo = float(q[j] - margin - q_min[j])
        if h_lo < hor_lo:
            a = np.zeros(nv)
            a[j] = 1.0
            rows.append((a, h_lo, 0.0, f'q{j + 1}-'))
    return rows


# ── Retreat cap ──────────────────────────────────────────────────────────────
#
# Every other row here bounds a barrier's rate from BELOW. These two helpers
# build the missing upper bound: how fast the filter is allowed to run away.
#
# The HOCBF row demands  n̂ᵀJ q̈ ≥ −k1·ḣ − k0·h̄ − ċ  and says nothing about the
# other side, so once h̄ < 0 the demanded floor grows with k0·|h̄| and the QP
# keeps accelerating the arm away for as long as the violation lasts. Fast
# retreat next to a person is its own hazard — and the arm is left with a large
# velocity and a large tracking error once the obstacle is gone.
#
# A cap row is the SAME direction n̂ᵀJ bounded from above, stored with a = −n̂ᵀJ
# so the node's generic ``G = [−A | −e_group]`` assembly turns it into an upper
# bound with no special case anywhere in the QP path.


def retreat_cap_speed(
    v_obs: float,
    h_bar: float,
    *,
    base: float,
    obs_gain: float,
    depth_gain: float,
    depth_speed_ref: float,
    engage_gap: float,
    max_speed: float,
) -> float:
    """[m/s] fastest separation rate a control point may be given.

        ramp   = clip(max(v_obs,0) / depth_speed_ref, 0, 1)
        relief = max_speed · clip(h̄ / engage_gap, 0, 1)
        v_cap  = min( max_speed,
                      base
                      + obs_gain·max(v_obs,0)
                      + depth_gain·max(−h̄,0)·ramp
                      + relief )

    The governing idea is PROPORTIONALITY: how fast the robot may back away is
    set by how fast the obstacle is coming in, not by how alarming the geometry
    looks. A person stepping slowly toward the arm should be answered by the arm
    stepping slowly away; anything else is a hazard of its own.

    Read as a function of ``h̄`` the cap is V-shaped, with its minimum exactly at
    the barrier:

    * ``h̄ ≥ engage_gap`` — ``relief`` saturates the cap at ``max_speed``. The
      arm is doing its task, not avoiding, and a cap here would throttle
      ordinary motion along an arbitrary obstacle normal. That was measured:
      ``retreat=+0.161/0.150`` with the nearest obstacle 0.389 m away and no
      avoidance happening at all. ``relief`` is a RAMP rather than an on/off
      gate on purpose — a gate would have to sit either above the distance
      where a fast approach first engages the barrier (useless) or below it
      (a step in the cap at the worst moment). ``engage_gap ≤ 0`` disables the
      relief entirely, so the cap applies at full strength everywhere — the
      pre-ramp behaviour, kept reachable from configuration.
    * ``h̄ = 0`` — ``relief`` is zero and the cap is purely proportional:
      ``base + obs_gain·v_obs``. This is where avoidance happens and where the
      shaping is meant to be tightest.
    * ``h̄ < 0`` — the depth term escalates, gated by ``ramp``.

    The terms:

    * ``base`` — the creep, not a retreat speed. It is what remains when the
      obstacle is stationary, and its only job is to let the arm walk out of a
      violation that nothing is making worse. Keep it small: it is the one term
      that is NOT proportional to anything.
    * ``obs_gain·v_obs`` — the proportional term, and the one that should
      dominate. At ``obs_gain = 1`` the arm may separate exactly as fast as the
      obstacle closes, which holds the gap without outrunning anything: at the
      barrier the constraint is then simply ``ḣ = n̂ᵀJq̇ − v_obs ≤ base``.
      ``v_obs`` is clamped to the approaching half upstream, so a receding
      obstacle contributes nothing.
    * ``depth_gain·max(−h̄,0)·ramp`` — escalation for a violated barrier, scaled
      by ``ramp``. Without ``ramp`` this is the hole in the proportionality: a
      slow obstacle that ends up deep inside ``d_safe`` authorises a fast
      retreat purely because the geometry is bad — exactly the "l'ostacolo si
      muove lentamente ma il robot si allontana troppo velocemente" case. With
      it, depth can only amplify an approach that is actually happening.
      ``depth_speed_ref`` is the closing speed at which the escalation reaches
      full strength; ``≤ 0`` disables the ramp and restores the older,
      speed-independent behaviour.
    * ``max_speed`` — absolute ceiling regardless of the above.

    The clamps are what make the escalation temporary: the instant the obstacle
    leaves ``d_safe`` and stops closing, both extra terms vanish and the cap is
    ``base`` plus whatever ``relief`` the distance now buys.

    This bound is SOFT — it has its own slack, priced well below the barrier's,
    so a genuine emergency still overrules it. Being proportional is a shaping
    policy, not a safety guarantee; the barrier remains the guarantee.
    """
    v = max(float(v_obs), 0.0)
    h = float(h_bar)
    ref = float(depth_speed_ref)
    gap = float(engage_gap)
    ramp = 1.0 if ref <= 0.0 else min(v / ref, 1.0)
    # gap <= 0 means NO relief, i.e. the cap applies at full strength at every
    # distance — the pre-ramp behaviour. Same convention as depth_speed_ref
    # above: a non-positive value disables the refinement rather than
    # reinterpreting it.
    relief = 0.0 if gap <= 0.0 else float(max_speed) * min(max(h / gap, 0.0), 1.0)
    return min(float(max_speed),
               float(base)
               + float(obs_gain) * v
               + float(depth_gain) * max(-h, 0.0) * ramp
               + relief)


def retreat_cap_rhs(
    A_cap: np.ndarray,      # (n, nv) the STORED rows, i.e. −n̂ᵀJ
    qdot: np.ndarray,       # (nv,) measured joint velocity
    cap_v: np.ndarray,      # (n,) per-row cap from retreat_cap_speed
    horizon: float,         # [s] enforcement horizon T
) -> np.ndarray:
    """RHS of the cap rows for ``G x ≤ h_qp``.

    With the row stored as ``a = −n̂ᵀJ``, the node's ``G[:, :nv] = −A`` gives
    ``+n̂ᵀJ q̈ − s ≤ h_qp``, and the one-step bound

        n̂ᵀJ (q̇ + q̈·T) ≤ v_cap

    is exactly ``h_qp = (v_cap − n̂ᵀJ q̇) / T``, with ``n̂ᵀJ q̇ = −(A_cap q̇)``.

    ``horizon`` is symmetric on purpose — see the parameter docs. Uses the RAW
    ``q̇``: this is a measured velocity, not the noisy derivative the barrier's
    ``k1`` term filters.
    """
    return (cap_v + A_cap @ qdot) / float(horizon)


def link_speed_cap(clearance: float, *, v_max: float, reaction_s: float) -> float:
    """[m/s] fastest a control point may travel, given the room it has left.

        v_allow = min( v_max, clearance / reaction_s )

    Two different bounds, and the tighter wins:

    * ``v_max`` is the flat ceiling — no part of the robot moves faster than
      this, whatever the configuration. The joint velocity box cannot express
      it: a folded or near-singular arm decouples joint speed from task speed
      in both directions, so modest ``q̇`` can whip a link and vice versa.
    * ``clearance / reaction_s`` is the GEOMETRIC bound, and the reason this
      exists at all. The barrier is rebuilt at 50 Hz from ~30 Hz perception,
      so between two evaluations a point travels ``v·Δt`` blind. Let it move
      faster than its own remaining gap divided by that blind time and it can
      cross the barrier BETWEEN samples — the constraint was never violated at
      any instant the QP looked at, and the links interpenetrate anyway. This
      is plain discretisation tunnelling, and no amount of k0/k1 tuning fixes
      it: the fix is to forbid the speed that makes the gap jumpable.

    ``reaction_s`` is that worst-case blind time: perception period + QP period
    + actuation lag, not the QP period alone.
    """
    return min(float(v_max), max(float(clearance), 0.0) / float(reaction_s))


def link_speed_row(
    Jp: np.ndarray,         # (3, nv) point Jacobian
    qdot: np.ndarray,       # (nv,)
    v_allow: float,         # [m/s] from link_speed_cap
    activate_frac: float,   # emit only above this fraction of v_allow
    label: str,
) -> Optional[Tuple[np.ndarray, float, str]]:
    """One task-space speed row for a control point, or ``None``.

    ``‖ṗ‖ ≤ v_allow`` is a norm bound, not linear. Linearised along the
    direction the point is ACTUALLY travelling,

        v̂ᵀJp q̇ ≤ v_allow ,      v̂ = Jp q̇ / ‖Jp q̇‖

    which is exact for the current motion (``v̂ᵀJp q̇ = ‖ṗ‖``) and tightens the
    one direction that matters. Returned as ``a = −v̂ᵀJp`` so the node's generic
    ``G = [−A | −e]`` turns it into an UPPER bound, exactly like a retreat-cap
    row — and it shares that row's RHS builder, :func:`retreat_cap_rhs`.

    ``None`` when the point is slower than ``activate_frac·v_allow``: below that
    the row is non-binding and would only inflate ``n_c`` (and OSQP ``setup()``
    churn) on every control point at once. Same horizon-gate tradeoff every
    other row family here makes.

    Returns:
        ``(a, v_allow, label)`` or ``None``.
    """
    v = Jp @ qdot
    speed = float(np.linalg.norm(v))
    if speed < activate_frac * v_allow or speed < 1e-9:
        return None
    v_hat = v / speed
    return (-(v_hat @ Jp), float(v_allow), label)


# ── Obstacle-velocity feedforward ────────────────────────────────────────────
#
# The HOCBF row already carries the obstacle's closing speed through the k1
# anticipation term (``ḣ = n̂ᵀJq̇ − v_obs``). These two terms are a SECOND,
# separately tunable use of the same estimate, ported from the Berdini
# implementation:
#
#   (a) a braking-distance tightening of the barrier itself,
#           h_eff = h − min( v_app² / (2·a_obs) , brake_max )
#       i.e. the barrier is measured not to where the obstacle IS but to where
#       it could still be after decelerating at a_obs. Note this is the
#       OBSTACLE's stopping distance under an ASSUMED deceleration — it is not
#       the robot's braking capability, which the joint accel box owns.
#
#   (b) a direct feedforward on the row's right-hand side, with its own gain
#       k_ff, so the "how hard do I react to a closing obstacle" lever is no
#       longer welded to k1 (which also sets the barrier's damping and cannot be
#       raised without making the whole row oscillate — measured on hardware at
#       k1 = 10.5, that single term was 159 % of h_qp's entire swing).
#
# Both are ZERO when v_app is zero, so the flag-off path is exact.


def velocity_feedforward_terms(
    v_app: float,
    *,
    decel: float,        # [m/s²] assumed obstacle deceleration a_obs
    gain: float,         # [-]    k_ff on the RHS feedforward
    brake_max: float,    # [m]    clamp on the braking-distance tightening
) -> Tuple[float, float]:
    """``(h_brake, b_ff)`` for one obstacle row.

    Args:
        v_app: closing speed of the obstacle along n̂ [m/s]. Negative values
            (receding) contribute nothing — the caller already clamps, and this
            clamps again so the function is safe to call on a raw estimate.
        decel: assumed obstacle deceleration ``a_obs`` [m/s²].
        gain: ``k_ff``.
        brake_max: hard cap on ``h_brake`` [m].

    Returns:
        ``h_brake`` — subtract it from the barrier: ``h_eff = h − h_brake``.
        ``b_ff``   — the RHS contribution, ALREADY SIGNED for this repo's
        convention. The QP row is assembled as ``G x ≤ h_qp`` with
        ``G[:, :nv] = −A``, i.e. ``aᵀq̈ + s ≥ −h_qp``, so a LARGER ``h_qp``
        is a LOOSER row. A closing obstacle must tighten, hence ``b_ff ≤ 0``
        and the caller adds it: ``h_qp += b_ff``. (In the ``aᵀq̈ + s ≥ b``
        convention of the source implementation this is the stated
        ``b += k_ff·v_app``; the sign flip is the convention, not a change.)

    ``brake_max`` is not optional hygiene. ``v_app`` is capped at
    ``obstacle_velocity_max`` = 2.0 m/s upstream, so an unclamped ``h_brake``
    reaches ``2²/(2·4) = 0.5 m`` — more than three times ``d_safe``. One depth
    artefact would then drive the barrier deeply negative and the QP would
    answer with a maximal retreat, which is the failure this filter has already
    been bitten by twice.
    """
    v = max(float(v_app), 0.0)
    h_brake = min(v * v / (2.0 * float(decel)), float(brake_max))
    return h_brake, -float(gain) * v


# ── Risk-weighted slack ──────────────────────────────────────────────────────
#
# The QP relaxes a family with ONE slack variable shared by every row in it:
#
#     aᵢᵀq̈ + s_g ≥ bᵢ        for every row i of family g,   cost ½ρ_g s_g²
#
# so when rows conflict the QP buys the same relief for all of them at once. A
# row 3 cm from contact and a row 40 cm away are relaxed by exactly the same
# amount. Weighting the slack COLUMN,
#
#     aᵢᵀq̈ + s_g / wᵢ ≥ bᵢ
#
# makes the relief each row receives inversely proportional to its criticality:
# a critical row (large w) gets little relief per unit of paid slack, a distant
# one gets a lot, so the QP's cheapest way out of a conflict is to bend the
# distant row and hold the near one. The slack stays SHARED — this is weighted
# sharing, not independent per-row relaxation, which would need one slack
# variable per row and hence a decision vector that resizes with n_c.
#
# NORMALISATION — the part that is easy to get wrong. Writing the column as
# −1/wᵢ makes the CRITICAL row's relief shrink while the distant row keeps
# today's. That sounds safer and is a trap: the shared slack is the max over
# wᵢ·rᵢ, so it inflates by up to w_max and its ½ρs² cost by up to w_max², and
# the obstacle family then out-prices ‖q̈ − q̈_nom‖² and distorts the solution
# far beyond "redistribute relaxation". Measured on the 4-row replay with the
# accel box saturated: s_obs went 3.0 → 15.0 for the same conflict.
#
# So the caller writes the column as −w_max/wᵢ instead. Identical RATIO between
# rows — the redistribution is the same — but the most critical row keeps
# EXACTLY today's column (−1.0) and the shared slack keeps today's magnitude,
# so no retuning of rho_slack is implied. The distant rows are the ones that
# move, gaining up to w_max× more relief, which is precisely the intent:
# relaxation is what a distant row is for.
#
# The weight uses the logistic that Flacco, Kröger, De Luca and Khatib use for
# the repulsive-vector magnitude in the depth-space collision-avoidance work
# ("A depth space approach to human-robot collision avoidance", ICRA 2012, and
# the JIRS 2015 journal version), where
#
#     V(d) = V_max / (1 + exp( α·(2d/ρ − 1) ))
#
# with ρ the distance of influence and α the shape. The same σ(·) is reused
# here as a criticality score rather than as a repulsion magnitude — same
# geometry, same knee at d = ρ/2, same saturation behaviour at both ends. It is
# borrowed for its shape and its provenance, not re-derived: a sigmoid is what
# keeps the weight continuous (so the QP's active set does not chatter as a
# control point drifts across a threshold) and bounded at both ends (so no
# single measurement can drive the weight to 0 or ∞).


def joint_limit_risk_margin(
    a: np.ndarray,          # the row's direction, ±e_j from joint_limit_rows
    h: float,               # the row's barrier value [rad]
    qdot: np.ndarray,       # (nv,) measured joint velocity
    a_auth: np.ndarray,     # (nv,) braking authority, eta·min(|acc_lb|,|acc_ub|)
) -> float:
    """Criticality ARGUMENT for a joint-limit row: margin left after braking.

        h_score = h − q̇²/(2·a_auth)      when the joint is CLOSING on this
        h_score = h                        limit, otherwise unchanged

    Scoring a joint-limit row on the raw ``h`` weights it far too late, and the
    numbers say so. The position braking curve in ``hard_accel_box`` starts
    clipping at ``h = q̇²/(2·a_auth)``; for the logged joint2 at 1.02 rad/s that
    is 0.335 rad, where a logistic with ``rho = 0.40`` scores only ``w = 1.07``
    — essentially neutral. The box was already fighting the joint while the
    weighting still considered it uninteresting.

    The post-braking margin fixes the ordering without a second parameter: at
    that same 0.335 rad the score is exactly 0.0, i.e. "no room left after
    stopping", and the weight saturates. A joint sitting at 0.335 rad with zero
    velocity scores 0.335 and stays neutral — which is correct, it is not in
    trouble. That is the difference between "near a limit" and "closing on a
    limit", and only the second is what the request was about.

    Direction matters: a joint moving AWAY from the limit this row guards gets
    no penalty. ``a`` is ``−e_j`` for an upper-limit row and ``+e_j`` for a
    lower one, so the closing speed is ``−a_j·q̇_j`` and only its positive part
    counts.

    This is the joint-space twin of the obstacle braking term in
    :func:`velocity_feedforward_terms`, and deliberately so — same physics, same
    shape, different units.
    """
    j = int(np.argmax(np.abs(a)))
    closing = -float(a[j]) * float(qdot[j])
    if closing <= 0.0:
        return float(h)
    aa = max(float(a_auth[j]), 1e-9)
    return float(h) - (closing * closing) / (2.0 * aa)


def risk_slack_weight(
    h: float,
    *,
    w_max: float,    # weight at h <= 0; 1.0 disables the weighting
    rho: float,      # [m] distance of influence — the knee sits at h = rho/2
    alpha: float,    # sigmoid shape; larger = sharper transition
) -> float:
    """Criticality weight for one row's slack column. Returns w in [1, w_max].

        sigma(h) = 1 / (1 + exp( alpha·(2h/rho − 1) ))
        w(h)     = 1 + (w_max − 1)·sigma(h)

    Bounded BY CONSTRUCTION at both ends, which is the reason for the affine
    wrapper rather than using sigma directly:

    * ``h ≥ rho`` ⇒ ``sigma ≈ 0`` ⇒ ``w ≈ 1`` — a distant row, which the caller
      turns into the LARGEST slack multiplier (``w_max/w ≈ w_max``): the row
      that can afford to yield is the one that yields;
    * ``h ≤ 0``   ⇒ ``sigma ≈ 1`` ⇒ ``w ≈ w_max`` — a violated row, multiplier
      ``w_max/w ≈ 1``, i.e. EXACTLY today's slack column. The row that matters
      is left as it is rather than being tightened, which is what keeps the
      family's slack magnitude — and therefore its ½ρs² cost — unchanged;
    * ``w ≥ 1`` always, so the multiplier ``w_max/w`` lies in ``[1, w_max]``.
      Note what this claims and what it does not: relaxation is redistributed
      onto the rows that can afford it, and no row that matters is loosened —
      but distant rows genuinely do yield further than they do today. That is
      the mechanism, not a side effect.

    A useful interaction: when the Phase-1 feedforward is on, ``h`` already
    carries the obstacle's braking distance, so the criticality score becomes
    velocity-aware for free — a fast approach raises the weight before the
    geometry alone would.
    """
    z = float(alpha) * (2.0 * float(h) / float(rho) - 1.0)
    # exp overflows to inf for z >> 0, which is the correct limit (sigma -> 0);
    # guard only the warning, not the value.
    sigma = 1.0 / (1.0 + np.exp(np.clip(z, -700.0, 700.0)))
    return 1.0 + (float(w_max) - 1.0) * float(sigma)


# ── Self-collision ───────────────────────────────────────────────────────────

class SelfCollisionRowBuilder:
    """HOCBF rows for capsule pairs that are closing on each other.

    The barrier is the SURFACE gap between two capsules,

        h = ‖p_a − p_b‖ − r_a − r_b − margin

    with ``p_a``/``p_b`` the closest points of the two segments. Both points are
    attached to moving links, so the relative Jacobian is what matters:

        ḣ = n̂ᵀ(ṗ_a − ṗ_b) = n̂ᵀ(J_a − J_b) q̇   ⇒   a = n̂ᵀ(J_a − J_b)
        ċ = n̂ᵀ(J̇_a − J̇_b) q̇

    This is the part the old post-solve veto could not do: it only knew the gap,
    not which joint motion changes it, so it had no way to ask the QP for a
    *different* command — only to refuse the one it had.

    Pair selection is the caller's (see ``self_collision.build_capsule_pairs``
    with the exclusion list from Franka's own SRDF). Only pairs whose gap is
    below ``horizon`` produce a row, so a normal pose contributes nothing.
    """

    def __init__(
        self,
        capsules: Sequence[Capsule],
        pairs: Sequence[Tuple[int, int]],
        frame_ids: Sequence[int],
        arm_v_ids: Sequence[int],
        margin: float = 0.0,
        horizon: float = 0.15,
        max_rows: int = 4,
        lead_s: float = 0.0,
        release: float = 1.0,
        gap_vel_alpha: float = 0.6,
    ):
        if not capsules:
            raise ValueError(
                'no capsules — the URDF was built without with_sc:=true')
        self._caps = list(capsules)
        self._pairs = [(int(i), int(j)) for i, j in pairs]
        self._fids = [int(f) for f in frame_ids]
        # The *_sc model carries the hand and its finger joints, so Pinocchio's
        # nv exceeds the 7 the QP controls. Jacobian rows must be projected onto
        # the arm columns. Sound because the finger joints are held at zero
        # velocity, so they contribute nothing to ḣ — but the SLICE has to
        # happen after the full-width product, never before.
        self._arm_v = np.asarray(arm_v_ids, dtype=np.intp)
        self._margin = float(margin)
        self._horizon = float(horizon)
        self._max_rows = int(max_rows)
        self._lead_s = float(lead_s)
        self._release = float(release)
        self._gap_vel_alpha = float(gap_vel_alpha)
        # Per-pair closing-speed tracking and engaged set, both keyed by the
        # pair's index in self._pairs — a fixed, finite key set (24 pairs), so
        # neither dict can grow without bound.
        self._gap_prev: dict = {}    # k -> (gap, stamp) at the last rebuild
        self._gap_vel: dict = {}     # k -> EMA'd closing speed [m/s], >0 = closing
        self._engaged: set = set()   # k of the pairs that produced a row last time
        self.last_lead = 0.0         # DIAGNOSTIC: largest anticipation used [m]

        n_cap = len(self._caps)
        self._loc_p1 = np.array([c.p1 for c in self._caps], dtype=np.float64)
        self._loc_p2 = np.array([c.p2 for c in self._caps], dtype=np.float64)
        self._radius = np.array([c.radius for c in self._caps])
        # Pre-allocated world endpoints, refilled each call.
        self._w1 = np.zeros((n_cap, 3))
        self._w2 = np.zeros((n_cap, 3))
        self.last_min_gap = float('inf')
        self.last_pair = ''

    @property
    def n_pairs(self) -> int:
        return len(self._pairs)

    @property
    def n_capsules(self) -> int:
        return len(self._caps)

    def describe(self) -> str:
        return (f'{len(self._caps)} capsules, {len(self._pairs)} pairs, '
                f'margin={self._margin:.3f} m, horizon={self._horizon:.3f} m, '
                f'max_rows={self._max_rows}')

    def _world_endpoints(self, kin) -> None:
        """Place every capsule using the *_sc frame placements from *kin*."""
        for c, fid in enumerate(self._fids):
            oMf = kin.data.oMf[fid]
            R, t = oMf.rotation, oMf.translation
            self._w1[c] = R @ self._loc_p1[c] + t
            self._w2[c] = R @ self._loc_p2[c] + t

    def _closing_speed(self, k: int, gap: float, stamp) -> float:
        """EMA'd rate at which pair *k*'s gap is shrinking [m/s], >0 = closing.

        Finite difference of the gap between rebuilds, exactly the shape
        ``ConstraintBuilder._obstacle_speed`` already uses for the perceived
        obstacle — and for the same reason: the quantity that decides how early
        a row must engage is a RATE, and the Jacobian-based ``ḣ`` is only
        available for pairs whose Jacobians we have already paid for, which is
        the very set this is deciding. At the 50 Hz rebuild a 0.5 m/s closure
        moves the gap 10 mm per step, three orders of magnitude above the
        geometry's numerical noise, so the difference is well conditioned.

        Returns 0.0 (no anticipation, i.e. the pre-existing fixed horizon)
        whenever the timing is unusable: first sight of the pair, no stamp, or
        a Δt outside a sane window.
        """
        prev = self._gap_prev.get(k)
        self._gap_prev[k] = (gap, stamp)
        if prev is None or stamp is None or prev[1] is None:
            return 0.0
        dt = stamp - prev[1]
        if not (1e-4 < dt < 0.5):
            return 0.0                      # duplicate frame, or a clock jump
        v_raw = (prev[0] - gap) / dt        # >0 when the surfaces approach
        v = (self._gap_vel_alpha * self._gap_vel.get(k, 0.0)
             + (1.0 - self._gap_vel_alpha) * v_raw)
        self._gap_vel[k] = v
        return v

    def build(self, kin, v_full: np.ndarray, stamp=None) -> List[Row]:
        """Rows for the pairs currently inside the (velocity-aware) horizon.

        *kin* must be a CBFKinematics already updated at the current q, q̇ with
        ``with_jdot=True`` — the caller owns that call so FK is paid once per
        tick no matter how many consumers need it.

        *v_full* is the FULL-width velocity of that model (nv, including any
        finger joints), not the 7-vector: the J̇ product needs every column.
        Returned rows are 7-wide, projected onto ``arm_v_ids``.

        *stamp* is the joint state's timestamp [s], used only to differentiate
        the gap for the anticipation term. ``None`` disables anticipation and
        restores the fixed-horizon behaviour exactly.

        ENGAGEMENT, and why it is not just ``gap < horizon``
        ---------------------------------------------------
        A fixed 0.08 m horizon gives a pair closing at 0.5 m/s about 160 ms of
        warning. The HOCBF row it then produces demands
        ``aᵀq̈ >= −k1·(aᵀq̇) − k0·h̄``, which at that speed and that gap is a
        large deceleration appearing in ONE tick — a step in q̈_safe, a torque
        spike through M·q̈, and the measured velocity overshooting into
        ``joint_velocity_violation``. The arm stopping "because of a pose where
        it would self-collide" is that step, not the collision.

        So the horizon is widened by how fast the pair is actually closing,
        ``horizon + lead_s · v_close``: at 0.5 m/s and lead_s = 0.35 the row
        appears at 0.26 m instead of 0.08 m, while h̄ is still large and
        ``−k0·h̄`` still leaves the row slack. The constraint then tightens
        CONTINUOUSLY over ~350 ms instead of stepping. A pair that is not
        closing keeps exactly the old horizon, so a normal pose still produces
        no rows.

        Strictly conservative in both directions: ``v_close`` is clamped to the
        approaching half, so the effective horizon is never SMALLER than the
        configured one, and the hysteresis below only ever keeps a row alive
        longer. Neither can remove a constraint that the old code would have
        emitted.
        """
        self._world_endpoints(kin)
        self.last_min_gap = float('inf')
        self.last_pair = ''
        self.last_lead = 0.0

        # Cheap scan first: distance only, no Jacobians. Jacobians cost a
        # Pinocchio call each and are paid solely for the pairs that make a row.
        near: List[Tuple[float, int, np.ndarray, np.ndarray]] = []
        for k, (i, j) in enumerate(self._pairs):
            pa, pb, d = segment_segment_closest(
                self._w1[i], self._w2[i], self._w1[j], self._w2[j])
            gap = d - self._radius[i] - self._radius[j]
            if gap < self.last_min_gap:
                self.last_min_gap = gap
                self.last_pair = f'{self._caps[i].body[-6:]}/{self._caps[j].body[-6:]}'

            lead = self._lead_s * max(self._closing_speed(k, gap, stamp), 0.0)
            if lead > self.last_lead:
                self.last_lead = lead
            horizon_eff = self._horizon + lead
            # Hysteresis: a pair that already has a row keeps it until the gap
            # opens past release·horizon_eff. Without it a pair sitting near
            # the boundary adds and removes its row on alternate rebuilds —
            # each time a discontinuity in the constraint set, and (because
            # n_c changes) an OSQP setup() with the warm start thrown away.
            limit = horizon_eff * self._release if k in self._engaged else horizon_eff
            if gap - self._margin < limit:
                near.append((gap, k, pa, pb))

        if not near:
            self._engaged.clear()
            return []
        # Cap the row count: the closest pairs are the ones that matter, and an
        # unbounded n_c would churn OSQP's factorization.
        near.sort(key=lambda t: t[0])
        self._engaged = {k for _, k, _, _ in near[: self._max_rows]}
        rows: List[Row] = []
        for gap, k, pa, pb in near[: self._max_rows]:
            i, j = self._pairs[k]
            delta = pa - pb
            norm = float(np.linalg.norm(delta))
            if norm < 1e-9:
                continue                     # exactly coincident: no normal
            n_w = delta / norm

            Ja, Jad = kin.point_jacobian(self._fids[i], pa)
            Jb, Jbd = kin.point_jacobian(self._fids[j], pb)
            # Full-width first, then project: slicing the Jacobian before the
            # product would silently drop the coupling through the wrist.
            a_full = n_w @ (Ja - Jb)
            a = np.ascontiguousarray(a_full[self._arm_v], dtype=np.float64)
            jdq = float(n_w @ ((Jad - Jbd) @ v_full))
            h = float(gap - self._margin)
            if not (np.all(np.isfinite(a)) and np.isfinite(h) and np.isfinite(jdq)):
                continue
            rows.append((a, h, jdq,
                         f'sc:{self._caps[i].body[-6:]}/{self._caps[j].body[-6:]}'))
        return rows


# ── Construction helper ──────────────────────────────────────────────────────

def build_self_collision_builder(
    urdf_path: str,
    resolve_frame_id,
    arm_v_ids: Sequence[int],
    exclude_pairs: Optional[Sequence[str]] = None,
    margin: float = 0.0,
    horizon: float = 0.15,
    max_rows: int = 4,
    lead_s: float = 0.0,
    release: float = 1.0,
    gap_vel_alpha: float = 0.6,
) -> SelfCollisionRowBuilder:
    """Parse the ``*_sc`` capsules from *urdf_path* and wire up a builder.

    ``exclude_pairs`` should carry Franka's own SRDF ``reason="Never"`` list —
    without it the builder checks 47 pairs, 23 of which the manufacturer states
    cannot collide at any configuration, and several of those (link1/link3 above
    all) interpenetrate by centimetres at the home pose purely because every
    capsule radius already includes ``safety_distance``.
    """
    from franka_experiments.utils.self_collision import (
        build_capsule_pairs, parse_sc_capsules)

    with open(urdf_path) as fh:
        capsules = parse_sc_capsules(fh.read())
    if not capsules:
        raise RuntimeError(
            f'{urdf_path} has no *_sc links — generated without with_sc:=true?')
    pairs = build_capsule_pairs(capsules, list(exclude_pairs or []))
    if not pairs:
        raise RuntimeError('every capsule pair was excluded')

    fids = []
    for c in capsules:
        fid = resolve_frame_id(c.frame)
        if fid is None:
            raise RuntimeError(f'frame "{c.frame}" not in the Pinocchio model')
        fids.append(int(fid))

    return SelfCollisionRowBuilder(capsules, pairs, fids, arm_v_ids,
                                   margin=margin, horizon=horizon,
                                   max_rows=max_rows, lead_s=lead_s,
                                   release=release, gap_vel_alpha=gap_vel_alpha)


# ═════════════════════════════════════════════════════════════════════════════
#  The constraint set
# ═════════════════════════════════════════════════════════════════════════════
#
# Everything above builds ONE family of rows. This assembles all of them into
# the matrices the QP solves, and it is stateful: the barrier smoothing, the
# per-track obstacle-velocity filter and the frame counters have to persist
# between rebuilds.
#
# Runs at the CONSTRAINT rate (50 Hz), never on the QP tick, which is why it may
# allocate freely. The QP tick only reads the snapshot it returns.

FR3_JOINTS = [f'fr3_joint{i}' for i in range(1, 8)]
# Keys as they appear in franka_description/robots/fr3/joint_limits.yaml
FR3_JOINT_KEYS = [f'joint{i}' for i in range(1, 8)]

# One slack per CONSTRAINT FAMILY. A single shared slack couples families that
# have nothing to do with each other and are not even in the same units: on
# hardware a joint-limit row (radians) drove the shared slack to 3.37, and that
# same 3.37 relaxed every SELF-COLLISION row (metres) until the firmware had to
# fire self_collision_avoidance_violation on its own. Separate slacks let one
# family yield without disarming the others.
NV         = 7
G_OBS, G_SC, G_QLIM, G_SING, G_CAP, G_SPD = 0, 1, 2, 3, 4, 5
N_SLACK = 6
GROUP_NAMES = ('obs', 'sc', 'qlim', 'sing', 'cap', 'spd')
NX = NV + N_SLACK   # [qddot(7), s_obs, s_sc, s_qlim, s_sing, s_cap, s_spd]

# h̄ stored on a retreat-cap row. Cap rows are not barriers — their RHS is
# overwritten in the QP tick — but they live in the same h_bar array, which is
# ALSO read by `n_active_cps` (count of h̄ < 0) and by the argmin that picks
# which row labels the CBFDIAG line. A large positive sentinel keeps them out of
# both without needing a mask in either place, and without inf ever entering the
# arithmetic.
CAP_H_SENTINEL = 1.0e3


# ── Immutable snapshots (atomic-swap shared state) ───────────────────────────

class JointSnap(NamedTuple):
    q:     np.ndarray   # (NV,)
    qdot:  np.ndarray   # (NV,)
    stamp: float


class NomSnap(NamedTuple):
    qddot: np.ndarray   # (NV,)
    stamp: float


class Obstacle(NamedTuple):
    link: str
    d:    float
    pr:   np.ndarray    # closest point on robot, world frame
    ph:   np.ndarray    # closest point on human, world frame
    conf: float


class ObstacleSnap(NamedTuple):
    items: tuple        # tuple[_Obstacle, ...]
    stamp: float        # RECEIPT time [s] (node clock). Every staleness check
                        # uses this and only this — distance_timeout is tuned
                        # against transport age and its semantics are unchanged.
    t_cap: float        # CAPTURE time [s]: MultiLinkDistance.header.stamp, i.e.
                        # the depth frame this measurement came from. Used ONLY
                        # to differentiate d in _obstacle_speed. Receipt time
                        # carries transport + executor-scheduling jitter, and
                        # dividing Δd by a jittered Δt puts that jitter straight
                        # into v_obs — which now drives the k1 anticipation
                        # term, the retreat cap AND (with the Phase-1 flag on)
                        # a term that SQUARES it. Falls back to `stamp` when the
                        # header stamp is missing or implausible.


class ConstraintSnap(NamedTuple):
    A:         np.ndarray  # (n_c, NV)     aᵢ = n̂ᵢᵀ Jᵢ rows
    h_bar:     np.ndarray  # (n_c,)        barrier values h̄ᵢ
    jdot_qdot: np.ndarray  # (n_c,)        ċᵢ = n̂ᵢᵀ(J̇ᵢ q̇) at the snapshot q̇
    G:         np.ndarray  # (n_c, NV+1)   prebuilt [−A | −1] for the QP
    t_dist:    float       # stamp of the distance data the geometry is based on
    links:     tuple       # (n_c,) link names, DIAGNOSTIC only — not used by QP
    d_obs_min: float       # [m] closest OBSTACLE surface gap this snapshot, or
                           # +inf when no obstacle row was built. NOT min(h_bar):
                           # h_bar also holds joint-limit rows (radians) and
                           # self-collision rows, so its argmin is meaningless as
                           # a distance. Published on cbf_status for the
                           # commander's phase governor.
    group:     np.ndarray  # (n_c,) constraint family per row → its slack column
    d_sc_min:  float       # [m] closest self-collision capsule gap, or +inf
    v_obs:     np.ndarray  # (n_c,) obstacle closing speed along n̂ [m/s], >= 0.
                           # Zero for self-collision and joint-limit rows: both
                           # sides of those are the robot's own, already in q̇.
    b_ff:      np.ndarray  # (n_c,) additive RHS feedforward, or None when
                           # enable_velocity_feedforward is off. None (not a
                           # zeros array) so the QP tick can skip the add
                           # entirely and stay bit-identical to the pre-Phase-1
                           # expression. Non-zero only on OBSTACLE rows, and
                           # already signed to TIGHTEN (≤ 0).
    cap_v:     np.ndarray  # (n_cap,) [m/s] the rate each RATE-CAP row allows.
                           # Two families share this block: retreat caps (a
                           # separation rate along n̂) and task-space speed caps
                           # (a point's own speed along its direction of
                           # travel). Both are stored with a negated row, so the
                           # generic G = [−A | −e] turns them into UPPER bounds,
                           # and both take their RHS from retreat_cap_rhs.
    n_cap:     int         # how many rate-cap rows. Always the LAST n_cap rows,
                           # so the QP tick addresses them with a SLICE (a view)
                           # rather than a boolean mask (a copy) — the only
                           # reason their position in the snapshot is pinned.
                           # 0 disables the whole branch.
    n_rtr:     int         # of those, how many are RETREAT caps. They come
                           # first in the block, so retreat is [-n_cap:][:n_rtr]
                           # and task-space speed is [-n_cap:][n_rtr:]. Keeps
                           # the two diagnostics apart without a per-tick mask.



def build_optional_row_builders(P, kin, log):
    """Construct the self-collision and singularity row builders, or neither.

    Both are OPTIONAL by design and both fail soft: the obstacle rows and the
    hard state box are the primary protection, so a URDF that will not build or
    a frame that does not resolve must degrade to "that family is off" and log
    it — never take the safety filter down at boot, where it would leave the
    arm with no filter at all.

    Returns the keyword arguments :class:`ConstraintBuilder` expects, so the
    caller writes ``ConstraintBuilder(..., **build_optional_row_builders(...))``
    and never sees the six pieces of self-collision plumbing (the capsule model
    carries the hand and fingers, so its q/v vectors are longer than the arm's
    and need an index map built once).

    Pinocchio and ``utils.kinematics`` are imported HERE, not at module scope:
    ``kinematics`` pulls in rclpy, and everything else in this module is pure
    numpy and must stay importable — and testable — with no ROS installed.
    """
    import pinocchio as pin

    from franka_experiments.utils.cbf_singularity import SingularityRowBuilder
    from franka_experiments.utils.kinematics import (
        CBFKinematics, build_urdf_with_sc)

    sc_rows = sc_kin = sc_qi = sc_vi = sc_q = sc_v = sing_rows = None
    _sc_sd = P.self_collision_safety_distance
    _sc_exclude = [str(x) for x in (P.self_collision_exclude_pairs or [])]
    if P.self_collision_rows_enabled:
        try:
            sc_urdf = build_urdf_with_sc(
                hand=True,
                safety_distance=None if _sc_sd is None else float(_sc_sd))
            sc_kin = CBFKinematics(pin.buildModelFromUrdf(sc_urdf))
            # The sc model carries the hand + fingers, so its q/v vectors
            # are longer than 7. Map the arm joints into it once.
            sc_q = pin.neutral(sc_kin.model)
            sc_v = np.zeros(sc_kin.model.nv)
            sc_qi = [sc_kin.model.joints[
                sc_kin.model.getJointId(j)].idx_q for j in FR3_JOINTS]
            sc_vi = [sc_kin.model.joints[
                sc_kin.model.getJointId(j)].idx_v for j in FR3_JOINTS]
            sc_rows = build_self_collision_builder(
                sc_urdf, sc_kin.resolve_frame_id,
                arm_v_ids=sc_vi,
                exclude_pairs=_sc_exclude, margin=P.self_collision_row_margin,
                horizon=P.self_collision_row_horizon, max_rows=P.self_collision_max_rows,
                lead_s=P.self_collision_row_lead_s,
                release=P.self_collision_row_release,
                gap_vel_alpha=P.self_collision_gap_vel_alpha)
            log.info(
                f'self-collision rows: {sc_rows.describe()}')
        except Exception as exc:
            # The obstacle rows and the hard box are the primary protection;
            # a failure here must not take the safety filter down at boot.
            sc_rows = None
            log.error(
                f'self-collision rows DISABLED — setup failed: {exc}')
    else:
        log.warn('self-collision rows DISABLED by parameter')

    if P.singularity_rows_enabled:
        try:
            _sg_fid = kin.resolve_frame_id(P.singularity_frame)
            if _sg_fid is None:
                raise KeyError(f'frame {P.singularity_frame!r} not in the CBF model')
            sing_rows = SingularityRowBuilder(
                kin.model, _sg_fid,
                sigma_floor=P.singularity_sigma_floor, horizon=P.singularity_row_horizon, eps=P.singularity_grad_eps,
                rot_scale=P.singularity_rot_scale, min_leverage=P.singularity_min_leverage,
                label=f'sing:{P.singularity_frame}')
            log.info(
                f'singularity row: {sing_rows.describe()}')
        except Exception as exc:
            # Same policy as the self-collision block: a setup failure here
            # must not take the safety filter down at boot.
            sing_rows = None
            log.error(
                f'singularity row DISABLED — setup failed: {exc}')
    else:
        log.warn('singularity row DISABLED by parameter')


    return dict(sc_rows=sc_rows, sc_kin=sc_kin, sc_qi=sc_qi, sc_vi=sc_vi,
                sc_q=sc_q, sc_v=sc_v, sing_rows=sing_rows)


class ConstraintBuilder:
    """Assembles every CBF row into one immutable :class:`ConstraintSnap`.

    Owns the state that must survive between rebuilds: the asymmetric barrier
    smoothing (``_h_smooth``), the per-track obstacle closing-speed filter
    (``_obs_vel``) and its frame counters, the "this joint-limit row has been
    binding forever" counter, and the resolved Pinocchio frame ids.

    Everything else comes from ``P`` (the parameter namespace) or per call, so
    the same builder can be driven by a replay harness with no ROS at all.

    ``diag_*`` attributes are read by the node's CBFDIAG line and by nothing
    else; they are attributes rather than return values because several are
    maxima accumulated across the per-row loop.
    """

    def __init__(self, P, kin, *, q_min, q_max, acc_lb, acc_ub,
                 sc_rows=None, sc_kin=None, sc_qi=None, sc_vi=None,
                 sc_q=None, sc_v=None, sing_rows=None, logger=None):
        self._P, self._kin, self._log = P, kin, logger
        self._q_min, self._q_max = q_min, q_max
        self._lb, self._ub = acc_lb, acc_ub
        # Braking authority per joint, byte-for-byte the expression
        # hard_accel_box uses, so the joint-limit row horizon and the box agree
        # on where the braking curve starts. Static — computed once.
        self._a_auth = P.position_brake_eta * np.minimum(np.abs(acc_lb),
                                                         np.abs(acc_ub))
        self._sc_rows, self._sc_kin = sc_rows, sc_kin
        self._sc_qi, self._sc_vi = sc_qi, sc_vi
        self._sc_q, self._sc_v = sc_q, sc_v
        self._sing_rows = sing_rows
        self._h_smooth, self._obs_vel, self._obs_frames = {}, {}, {}
        self._qlim_stuck, self._fid_cache = {}, {}
        self.diag_h_hold = self.diag_v_obs = 0.0
        self.diag_vapp = self.diag_hbrake = 0.0
        self.diag_sigma = float('nan')
        self.diag_w = self.diag_wq = None

    def build(self, js, obs, now):
        if now - obs.stamp > self._P.distance_timeout:
            return None

        # with_jdot=True: also run computeJointJacobiansTimeVariation so that
        # point_jacobian() can return J̇p — needed for the J̇q̇ term of d̈
        # (h̄ has relative degree 2). One extra O(nv) backward pass at 50 Hz.
        self._kin.update(js.q, js.qdot, with_jdot=True)
        rows_a, rows_h, rows_jdq = [], [], []
        # Label per kept row (DIAGNOSTIC only, never read by the QP). Perception
        # now sends one entry per CONTROL POINT, so several rows share a
        # robot_link_name; the #k suffix counts occurrences in arrival order so
        # CBFDIAG can name which CP on a link is driving the constraint.
        rows_link     = []
        rows_group    = []          # constraint family per row (G_OBS/G_SC/G_QLIM)
        rows_vobs     = []          # obstacle closing speed along n̂ [m/s]
        link_seen: dict[str, int] = {}
        d_sc_min      = np.inf      # closest self-collision surface gap [m]
        n_weak        = 0           # obstacles dropped this tick for low leverage
        min_a_dropped = np.inf      # smallest ‖a‖ among the dropped ones (debug)
        d_obs_min     = np.inf      # closest obstacle gap, obstacle rows only
        # Retreat-cap rows are collected here and spliced in as the TRAILING
        # rows of the snapshot (see the block after the self-collision rows), so
        # the mask is a contiguous tail slice instead of a per-append flag on
        # every one of the four row builders.
        cap_a, cap_val, cap_grp, cap_link = [], [], [], []
        # (Jp, clearance, label) per kept obstacle control point. The task-space
        # speed rows are built from these AFTER the self-collision pass, so they
        # can fold d_sc_min into each point's clearance.
        spd_pts: list[tuple[np.ndarray, float, str]] = []
        # Phase-1 RHS feedforward, one entry per OBSTACLE row in append order.
        # Scattered onto the full-length vector via grp == G_OBS at the end, so
        # it survives any future reordering of the row builders.
        obs_bff: list[float] = []
        # Phase-2 slack weights, same per-OBSTACLE-row-in-append-order contract
        # as obs_bff and scattered the same way.
        obs_w: list[float] = []
        # Same contract for the JOINT-LIMIT family: one weight per emitted row,
        # in append order, scattered by family at the end.
        qlim_w: list[float] = []
        self.diag_h_hold = 0.0     # largest recovery held back this rebuild [m]
        self.diag_v_obs  = 0.0     # fastest approaching obstacle this rebuild
        self.diag_vapp   = 0.0     # largest v_app the feedforward USED
        self.diag_hbrake = 0.0     # largest braking-distance tightening [m]

        for ob in obs.items:
            # obstacle_horizon is a COMPUTATIONAL cutoff, NOT a safety gate: the
            # linear HOCBF row self-deactivates at large d (its lower bound
            # −k0·h̄ becomes very negative), and the k1·ḣ̄ + jdq terms let a fast
            # approach engage the row gradually from afar — so activation is now
            # continuous. We skip only obstacles so far that engaging them would
            # need a physically unrealistic approach speed (at 1.2 m with the
            # current k0/k1 that is ≈ −2.4 m/s, beyond human motion), purely to
            # bound/stabilize n_c and avoid OSQP setup() churn. Replaces the old
            # d_safe+cbf_activation_margin step gate.
            if ob.d < d_obs_min:
                d_obs_min = ob.d    # BEFORE the horizon/leverage filters: the
                                    # governor wants the true closest distance,
                                    # not the closest one that produced a row.
            if ob.d > self._P.cbf_obstacle_horizon or ob.conf < self._P.min_confidence:
                continue

            # delta/‖delta‖ gives the unit normal n̂ (obstacle → control point).
            # ‖delta‖ itself is the CENTRE-to-obstacle distance and is NOT the
            # barrier argument — see the h assignment below.
            delta   = ob.pr - ob.ph
            d_ctr   = float(np.linalg.norm(delta))
            if d_ctr < 1e-8:
                continue
            n_w = delta / d_ctr

            fid = self._frame_id(ob.link)
            if fid is None:
                continue

            # point_jacobian returns (Jp, J̇p): Jp is the 3×7 position Jacobian
            # (same matrix as the old point_jacobian_pos), J̇p feeds jdq below.
            Jp, Jpd = self._kin.point_jacobian(fid, ob.pr)
            a      = (n_w @ Jp).astype(np.float64)      # (NV,)  aᵢ = n̂ᵀ Jp
            a_norm = float(np.linalg.norm(a))
            # Drop constraints with too little leverage on q̈ along THIS
            # obstacle's normal. Replaces the old cond(Jp)>1e5 test: cond(Jp)
            # measured the conditioning of the whole 3×7 map, not the leverage
            # of the specific row aᵢ the QP actually uses — a high cond only
            # weakens ‖aᵢ‖ when n̂ aligns with the ill-conditioned singular
            # direction, so it dropped direction-OK constraints and was an
            # indirect proxy for the weak case (see CBF review notes).
            if a_norm < self._P.cbf_min_leverage:
                n_weak       += 1
                min_a_dropped = min(min_a_dropped, a_norm)
                continue

            # Barrier argument is the SURFACE gap published by perception
            # (ob.d = LinkDistance.distance), not ‖pr − ph‖.  pr is the control
            # point on the segment AXIS, so ‖pr − ph‖ omits both the capsule
            # radius (0.05 m) and the pixel-dilation margin DistanceEngine
            # already subtracted (0.014 m at Z = 0.5 m, ~0.11 m at Z = 2 m).
            # Using it made the filter believe it was 6–16 cm farther from the
            # obstacle than perception reported — an optimistic bias in exactly
            # the unsafe direction, and a direct cause of contact while the CBF
            # still read a positive h̄.  ob.d is finite by construction: the
            # publisher sets valid=False otherwise and _on_distances drops it.
            #
            # a and jdq are UNCHANGED and remain exact: d_surface = d_ctr − r −
            # margin with r constant, so ḋ_surface = ḋ_ctr = n̂ᵀJp q̇.  Only the
            # zeroth-order term of the HOCBF moves.
            # Asymmetric smoothing of the barrier — obstacle rows ONLY.
            #
            # Obstacle h comes from the camera at ~30 Hz while the QP runs at
            # 100 Hz, so it sits frozen for a tick or two and then jumps:
            # measured, d_min repeated its exact value on 7 of 26 consecutive
            # diagnostic lines and then moved 24 mm, which k0 = 25 turns into a
            # 0.60 rad/s² step in the constraint — 31% of h_qp's whole range in
            # one tick. That staircase is the jerk felt on approach.
            #
            # Self-collision and joint-limit rows are NOT smoothed and do not
            # need to be: their h is built from joint states, which arrive far
            # faster than the QP and never staircase.
            #
            # Asymmetric, in the same spirit as distance_engine's own LPF one
            # layer upstream: a CLOSER measurement is taken instantly (no lag
            # where it would cost safety), while RECOVERY is rate-limited. So
            # h_eff <= the raw measurement at every instant — strictly more
            # conservative than the unsmoothed value, never less.
            #
            # EMA rather than a rate limit: a rate limit leaves a corner at both
            # ends of the ramp, and sizing it is awkward — 0.5 m/s allows 10 mm
            # per rebuild, larger than the 9 mm rises actually seen, so it never
            # engaged at all. The EMA has no corner and its effect scales with
            # the step. Same shape distance_engine already uses on its own
            # moving-away branch.
            #
            # An earlier attempt extrapolated h forward with ḣ = aᵀq̇ instead.
            # That assumes a static obstacle, so when the human is the one
            # moving the prediction is wrong and the correction at the next
            # frame is BIGGER than the step it was meant to remove — on the
            # logged sequence 0.124 -> 0.112 -> 0.121 it made the worst step
            # grow from 12 mm to 13 mm. Rate-limiting cannot do that.
            h_raw  = ob.d - self._P.d_safe
            lbl    = f'{ob.link}#{link_seen.get(ob.link, 0)}'
            h_prev = self._h_smooth.get(lbl)
            if h_prev is None or h_raw <= h_prev:
                h = h_raw                       # closer, or first sight
            else:
                h = self._P.cbf_h_recovery_alpha * h_prev + (1.0 - self._P.cbf_h_recovery_alpha) * h_raw
            self._h_smooth[lbl] = h
            self.diag_h_hold = max(self.diag_h_hold, h_raw - h)

            # Obstacle velocity along n̂ (>0 = closing). CONSERVATIVE CLAMP: only
            # the approaching half is used. A positive v_obs makes ḣ more
            # negative, so the constraint demands MORE retreat — the term can
            # only ever tighten the QP, never loosen it. Trusting the receding
            # half would mean relaxing a barrier on a 30 Hz vision estimate.
            v_o = 0.0
            if self._P.obstacle_velocity_enabled:
                v_o = max(self._obstacle_speed(
                    lbl, ob.d, obs.t_cap, float(a @ js.qdot)), 0.0)
                if v_o > self.diag_v_obs:
                    self.diag_v_obs = v_o
            rows_vobs.append(v_o)

            # ── Phase 1: obstacle-velocity feedforward ──────────────────────
            # (a) tightens the barrier by the obstacle's stopping distance,
            # (b) adds a separately-gained RHS term. Both computed HERE, at the
            # 50 Hz rebuild, so the 100 Hz QP tick pays nothing for them.
            # v_app is the already-clamped, already-EMA'd closing speed — the
            # same signal the k1 term and the retreat cap use, deliberately not
            # a second filter on the same physical quantity.
            #
            # ORDERING IS DELIBERATE: this runs AFTER `self._h_smooth[lbl] = h`
            # above, so the EMA state keeps tracking the MEASURED barrier. Were
            # the braked value stored instead, next frame's smoothing would
            # start from an already-tightened h and the term would compound
            # frame over frame into an unbounded drift.
            b_ff_i = 0.0
            if self._P.enable_velocity_feedforward and self._obs_frames.get(lbl, 0) >= self._P.velocity_feedforward_min_frames:
                h_brake, b_ff_i = velocity_feedforward_terms(
                    v_o, decel=self._P.obstacle_decel_assumed, gain=self._P.velocity_feedforward_gain,
                    brake_max=self._P.velocity_braking_margin_max)
                h -= h_brake
                if v_o > self.diag_vapp:
                    self.diag_vapp = v_o
                if h_brake > self.diag_hbrake:
                    self.diag_hbrake = h_brake

            # ċᵢ = n̂ᵀ(J̇p q̇): centripetal/Coriolis part of d̈ that does NOT
            # depend on q̈ (the relative-degree-2 term previously omitted).
            # Frozen at this snapshot's q̇ (js.qdot); the QP refreshes only aᵀq̇.
            jdq = float(n_w @ (Jpd @ js.qdot))          # scalar ċᵢ

            if np.all(np.isfinite(a)) and np.isfinite(h) and np.isfinite(jdq):
                rows_a.append(a)
                rows_h.append(h)
                rows_jdq.append(jdq)
                rows_group.append(G_OBS)
                # Appended INSIDE the finite guard, in lockstep with rows_a, so
                # obs_bff can never drift out of alignment with the obstacle
                # rows. (rows_vobs above is appended OUTSIDE the guard — a
                # pre-existing misalignment, left untouched in this phase and
                # reported separately.)
                obs_bff.append(b_ff_i)
                # Criticality weight from the barrier value that ACTUALLY goes
                # into the QP — i.e. after the Phase-1 braking tightening, so
                # with that flag on the weight is velocity-aware for free.
                obs_w.append(risk_slack_weight(
                    h, w_max=self._P.slack_weight_max, rho=self._P.slack_weight_rho,
                    alpha=self._P.slack_weight_alpha) if self._P.enable_weighted_slack else 1.0)
                k = link_seen.get(ob.link, 0)
                link_seen[ob.link] = k + 1
                rows_link.append(f'{ob.link}#{k}')

                # Retreat cap for THIS control point, on the same normal. One
                # per kept obstacle row, so the cap set follows the barrier set
                # exactly and adds no activation churn of its own. Stored with
                # −a: the generic G = [−A | −e] then yields +aᵀq̈, an upper
                # bound, while every other row keeps its lower one.
                # One cap row per kept obstacle row, ALWAYS — the distance
                # dependence lives in the cap VALUE (retreat_cap_speed's relief
                # term saturates it at max_speed far from the barrier), not in
                # whether the row exists. Gating the row instead would add
                # activation churn to n_c and put a step in the bound at
                # whatever distance the gate sat.
                if self._P.retreat_cap_enabled:
                    cap_a.append(-a)
                    cap_val.append(self._retreat_cap(v_o, h))
                    cap_grp.append(G_CAP)
                    cap_link.append(f'cap:{ob.link}#{k}')
                if self._P.link_speed_rows_enabled:
                    spd_pts.append((Jp, ob.d, f'spd:{ob.link}#{k}'))

        # ── Joint-limit rows ────────────────────────────────────────────────
        # Same HOCBF convention: aᵀq̈ + s ≥ −k1(aᵀq̇) − k0·h − ċ, with ċ ≡ 0
        # because ḧ = ∓q̈ carries no drift term.
        if self._P.joint_limit_rows_enabled:
            for a, h, jdq, lbl in joint_limit_rows(
                    js.q, self._q_min, self._q_max,
                    self._P.joint_limit_row_margin, self._P.joint_limit_row_horizon, NV,
                    qdot=js.qdot if self._P.joint_limit_row_velocity_horizon else None,
                    a_auth=self._a_auth if self._P.joint_limit_row_velocity_horizon else None,
                    lead=self._P.joint_limit_row_horizon_lead):
                rows_a.append(a)
                rows_h.append(h)
                rows_jdq.append(jdq)
                rows_group.append(G_QLIM)
                rows_vobs.append(0.0)
                rows_link.append(lbl)
                # Criticality weight for THIS row, in radians. Same logistic
                # and same normalisation as the obstacle family, its own rho —
                # but scored on the margin left AFTER BRAKING, not on the raw h.
                # Raw h weights far too late: at the distance where the box
                # already clips joint2 (0.335 rad at 1.02 rad/s) a rho = 0.40
                # logistic scores only 1.07, essentially neutral. The
                # post-braking margin is 0.0 there and saturates the weight,
                # while a joint parked at the same distance stays neutral. That
                # is the difference between "near a limit" and "closing on a
                # limit", and only the second is a problem.
                qlim_w.append(risk_slack_weight(
                    joint_limit_risk_margin(a, h, js.qdot, self._a_auth),
                    w_max=self._P.slack_weight_max, rho=self._P.slack_weight_rho_qlim,
                    alpha=self._P.slack_weight_alpha)
                    if (self._P.enable_weighted_slack and self._P.slack_weight_rho_qlim > 0.0) else 1.0)

        # ── Singularity row ─────────────────────────────────────────────────
        # At most one row (σ_min is a single scalar barrier). Emitted only while
        # σ_min − floor < horizon, so a well-conditioned pose contributes
        # nothing and n_c stays stable.
        if self._sing_rows is not None:
            try:
                for a, h, jdq, lbl in self._sing_rows.build(
                        js.q, js.qdot, js.stamp):
                    rows_a.append(a)
                    rows_h.append(h)
                    rows_jdq.append(jdq)
                    rows_group.append(G_SING)
                    rows_vobs.append(0.0)   # nothing external moves this barrier
                    rows_link.append(lbl)
                self.diag_sigma = self._sing_rows.last_sigma
            except Exception as exc:
                self._log.error(
                    f'singularity row skipped this tick: {exc}',
                    throttle_duration_sec=2.0)

        # ── Self-collision rows ─────────────────────────────────────────────
        # Needs its own FK pass: self._kin is the hand-less model and has no
        # *_sc capsule frames. One extra Pinocchio update at the 50 Hz
        # constraint rate, not on the QP tick.
        if self._sc_rows is not None:
            try:
                for k_, (iq, iv) in enumerate(zip(self._sc_qi, self._sc_vi)):
                    self._sc_q[iq] = js.q[k_]
                    self._sc_v[iv] = js.qdot[k_]
                self._sc_kin.update(self._sc_q, self._sc_v, with_jdot=True)
                for a, h, jdq, lbl in self._sc_rows.build(self._sc_kin, self._sc_v,
                                                          stamp=js.stamp):
                    rows_a.append(a)
                    rows_h.append(h)
                    rows_jdq.append(jdq)
                    rows_group.append(G_SC)
                    rows_vobs.append(0.0)   # both bodies are the robot's own
                    rows_link.append(lbl)
                d_sc_min = self._sc_rows.last_min_gap
            except Exception as exc:
                self._log.error(
                    f'self-collision rows skipped this tick: {exc}',
                    throttle_duration_sec=2.0)

        if n_weak > 0:
            # Previously this drop was silent (no log/counter). Throttled so a
            # persistently weak-leverage obstacle can't spam the log.
            self._log.warn(
                f'{n_weak} obstacle(s) dropped: CBF leverage ‖a‖ < cbf_min_leverage='
                f'{self._P.cbf_min_leverage:.3g} (min ‖a‖={min_a_dropped:.3g})',
                throttle_duration_sec=2.0)

        # A joint-limit row that binds tick after tick means the trajectory is
        # asking for a pose outside the usable range — the QP will pay slack
        # forever and the arm will drift off the path. That is a commander/
        # workspace problem the filter cannot fix, so say so out loud rather
        # than let it show up as a mysterious tracking error.
        for a_, h_, _, lbl_ in zip(rows_a, rows_h, rows_jdq, rows_link):
            if lbl_.startswith('q') and h_ < 0.02:
                self._qlim_stuck[lbl_] = self._qlim_stuck.get(lbl_, 0) + 1
                if self._qlim_stuck[lbl_] % 100 == 0:
                    self._log.warn(
                        f'joint-limit row {lbl_} has been binding for '
                        f'{self._qlim_stuck[lbl_]} ticks (h={h_:+.4f} m). The '
                        f'commanded trajectory needs this joint past the row '
                        f'barrier — lower joint_limit_row_margin or fix the '
                        f'path; the QP is paying slack every tick.')
            elif lbl_ in self._qlim_stuck:
                del self._qlim_stuck[lbl_]

        # ── Task-space speed rows ───────────────────────────────────────────
        # Built here, after the self-collision pass, because each point's
        # clearance is min(its own obstacle gap, the closest self-collision
        # gap): a near self-collision is a whole-arm geometric event and must
        # slow every control point, not only the pair that reported it.
        for Jp_i, clear_i, lbl_i in spd_pts:
            row = link_speed_row(
                Jp_i, js.qdot,
                link_speed_cap(min(clear_i, d_sc_min),
                               v_max=self._P.link_speed_max,
                               reaction_s=self._P.link_speed_reaction_s),
                self._P.link_speed_activate_frac, lbl_i)
            if row is not None:
                a_s, v_s, lbl_s = row
                cap_a.append(a_s)
                cap_val.append(v_s)
                cap_grp.append(G_SPD)
                cap_link.append(lbl_s)

        # ── Rate-cap rows (TRAILING, so the QP addresses them by slice) ─────
        # Retreat caps and task-space speed caps share one tail block: both are
        # UPPER bounds of the form aᵀq̇ ≤ v, stored with a negated row and given
        # their RHS by the same retreat_cap_rhs. Only the slack family differs,
        # and rows_group already carries that.
        # Appended after the stuck-joint-limit bookkeeping above on purpose:
        # that loop keys off the label prefix, and these carry 'cap:'/'spd:'.
        n_cap = len(cap_a)
        if n_cap:
            rows_a.extend(cap_a)
            rows_h.extend([CAP_H_SENTINEL] * n_cap)   # not a barrier — see const
            rows_jdq.extend([0.0] * n_cap)            # RHS is overwritten in the QP
            rows_group.extend(cap_grp)
            rows_vobs.extend([0.0] * n_cap)           # v_obs is folded into cap_v
            rows_link.extend(cap_link)

        if not rows_a:
            return None

        A     = np.vstack(rows_a)
        h_bar = np.array(rows_h,   dtype=np.float64)
        jdq_v = np.array(rows_jdq, dtype=np.float64)    # (n_c,) ċᵢ
        n_c   = A.shape[0]
        # [−A | −e_group]: aᵀq̈ + s_family ≥ b. Only the row's OWN family column
        # is −1; the other slack columns stay 0, so a row can never be relaxed
        # by another family's slack.
        grp   = np.asarray(rows_group, dtype=np.intp)
        G     = np.zeros((n_c, NX))
        G[:, :NV] = -A
        # Slack column. Phase 2 replaces the literal −1.0 with −1/wᵢ on obstacle
        # rows, which is the WHOLE of the weighted-slack change: NX, P, N_SLACK,
        # build_osqp_A and setup() are all untouched, and the QP tick does not
        # even know it happened. With the flag off the literal is written
        # unchanged, so G is bit-identical to the pre-Phase-2 build.
        # m is the slack MULTIPLIER: the row reads aᵀq̈ + mᵢ·s_g ≥ bᵢ, so a
        # larger m means more relief per unit of paid slack. m = w_max/wᵢ, NOT
        # 1/wᵢ — see risk_slack_weight's normalisation note: 1/wᵢ inflates the
        # shared slack and its ½ρs² cost by up to w_max², which would make the
        # obstacle family out-price the tracking objective. With w_max/wᵢ the
        # most critical row keeps exactly today's multiplier of 1.0 and only the
        # distant rows move. Non-obstacle rows stay at 1.0 untouched.
        m_row = None
        self.diag_w = self.diag_wq = None
        if self._P.enable_weighted_slack:
            m_row = np.ones(n_c, dtype=np.float64)
            for fam, wl, diag in ((G_OBS, obs_w, 'obs'), (G_QLIM, qlim_w, 'qlim')):
                idx_w = np.flatnonzero(grp == fam)
                if idx_w.size != len(wl):
                    m_row = None
                    self._log.error(
                        f'weighted slack disabled this tick: {idx_w.size} '
                        f'{diag} rows vs {len(wl)} weights',
                        throttle_duration_sec=2.0)
                    break
                if idx_w.size:
                    w_arr = np.asarray(wl, dtype=np.float64)
                    m_row[idx_w] = self._P.slack_weight_max / w_arr
                    if fam == G_OBS:
                        self.diag_w = w_arr
                    else:
                        self.diag_wq = w_arr
            if m_row is None:
                self.diag_w = self.diag_wq = None
        if m_row is None:
            G[np.arange(n_c), NV + grp] = -1.0
        else:
            G[np.arange(n_c), NV + grp] = -m_row
        # Phase-1 RHS feedforward, scattered onto the OBSTACLE rows by family
        # rather than by position, so it stays correct if the row builders are
        # ever reordered. None when the flag is off: the QP tick then skips the
        # add entirely and its expression is bit-identical to pre-Phase-1.
        b_ff = None
        if self._P.enable_velocity_feedforward:
            b_ff = np.zeros(n_c, dtype=np.float64)
            idx  = np.flatnonzero(grp == G_OBS)
            if idx.size == len(obs_bff):
                b_ff[idx] = obs_bff
            else:
                # Cannot happen with the current builders; if it ever does,
                # degrade to no feedforward rather than misalign the rows.
                self._log.error(
                    f'velocity feedforward disabled this tick: {idx.size} '
                    f'obstacle rows vs {len(obs_bff)} feedforward terms',
                    throttle_duration_sec=2.0)
                b_ff = None
        return ConstraintSnap(A, h_bar, jdq_v, G, obs.stamp,
                                    tuple(rows_link), float(d_obs_min),
                                    grp, float(d_sc_min),
                                    np.asarray(rows_vobs, dtype=np.float64),
                                    b_ff,
                                    np.asarray(cap_val, dtype=np.float64),
                                    int(n_cap),
                                    int(sum(1 for g in cap_grp if g == G_CAP)))

    def _retreat_cap(self, v_obs: float, h_bar: float) -> float:
        """[m/s] fastest separation rate this control point may be given.

        Thin binding of this node's parameters onto
        :func:`~franka_experiments.utils.cbf_state_rows.retreat_cap_speed`,
        which owns the formula and is where it is tested.
        """
        return retreat_cap_speed(
            v_obs, h_bar,
            base=self._P.retreat_cap_base_speed, obs_gain=self._P.retreat_cap_obstacle_gain,
            depth_gain=self._P.retreat_cap_depth_gain, depth_speed_ref=self._P.retreat_cap_depth_speed_ref,
            engage_gap=self._P.retreat_cap_engage_gap, max_speed=self._P.retreat_cap_max_speed)

    def _obstacle_speed(self, lbl: str, d_now: float, stamp: float,
                        adotq: float) -> float:
        """Component of the OBSTACLE's velocity along n̂, in m/s, >0 = closing.

        The gap closes at  ḋ = n̂ᵀ(ṗ_robot − ṗ_obs) = aᵀq̇ − v_obs, so the
        obstacle's contribution is the RESIDUAL

            v_obs = aᵀq̇ − ḋ_measured

        Estimated this way, not by differentiating ``closest_point_human``:
        that point is the argmin over depth pixels and the engine's own
        docstring calls it memoryless — it jumps between surface patches frame
        to frame, so its derivative is mostly artefact. ``d`` is the signal the
        engine already conditions, with an asymmetric LPF and an approach-spike
        rejection on top.

        Splitting it this way also puts each half on the clock it deserves:
        aᵀq̇ stays fresh at the QP rate (joint states are fast and clean), while
        only the noisy 30 Hz half is filtered.

        ``stamp`` is the depth frame's CAPTURE time (``_ObstacleSnap.t_cap``),
        not the receipt time it used to be. Δd was being divided by a Δt that
        carried transport and executor-scheduling jitter, so that jitter went
        straight into v_obs — and v_obs now drives three consumers (the k1
        anticipation term, the retreat cap, and the Phase-1 braking term, which
        squares it). Same arithmetic, correct clock.

        Returns 0.0 until two perception frames have been seen.
        """
        self._obs_frames[lbl] = self._obs_frames.get(lbl, 0) + 1
        prev = self._obs_vel.get(lbl)
        if prev is None:
            self._obs_vel[lbl] = (d_now, stamp, 0.0)
            return 0.0
        d_prev, t_prev, v_prev = prev
        dt = stamp - t_prev
        if dt <= 1e-4:
            return v_prev                  # same perception frame — hold
        d_dot = (d_now - d_prev) / dt
        v_raw = adotq - d_dot
        # Cap at a physically plausible approach speed, same scale the distance
        # engine uses for its own spike rejection. A depth artefact can
        # otherwise fabricate metres per second out of one bad frame.
        v_raw = float(np.clip(v_raw, -self._P.obstacle_velocity_max, self._P.obstacle_velocity_max))
        v = self._P.obstacle_velocity_alpha * v_prev + (1.0 - self._P.obstacle_velocity_alpha) * v_raw
        self._obs_vel[lbl] = (d_now, stamp, v)
        return v

    def _frame_id(self, link: str) -> int | None:
        if link not in self._fid_cache:
            self._fid_cache[link] = self._kin.resolve_frame_id(link)
        return self._fid_cache[link]
