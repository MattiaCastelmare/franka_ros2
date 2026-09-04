"""Assembling the OSQP problem matrices for the acceleration-level CBF filter.

OWNS
----
The translation from CBF constraint rows into the exact sparse structures the
native OSQP interface expects:

* :func:`build_osqp_A`      — the constraint matrix ``A = [ G ; I ]``
* :func:`build_osqp_bounds` — the matching ``(l, u)`` bound vectors
* :func:`tangential_bias`   — nudges the QP's target sideways around a close
  obstacle/self-collision row instead of only backing straight off it

DOES NOT OWN
------------
* Where the CBF rows come from (barrier values, Jacobians) — the node builds
  those from perception and Pinocchio.
* The cost matrix ``P`` / linear term ``q`` — owned by the node, which
  pre-allocates them once.
* Solving, warm-starting, or interpreting the solution.

Hot-path note: :func:`build_osqp_A` allocates a sparse matrix per call and runs
on the QP tick, exactly as it did as a node static method — the Phase 3 move is
allocation-neutral. :func:`build_osqp_bounds` returns the caller's own box
arrays unchanged when there are no CBF rows, so the ``n_c == 0`` path stays
allocation-free.
"""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
import scipy.sparse as sparse

from franka_experiments.utils.cbf_state_rows import G_OBS, G_SC, retreat_cap_rhs


def build_osqp_A(G: Optional[np.ndarray], nv: int,
                 n_slack: int = 1) -> sparse.csc_matrix:
    """Build the native-OSQP constraint matrix ``A = [ G ; I ]``.

    MOVED here from ``CBFSafetyFilter._osqp_A`` in Phase 3; body unchanged
    except that the joint count is now an argument instead of a module global.

    Rows ``0..n_c-1`` are the CBF constraints (``[-A | -e_group]``, the slack
    column being the one belonging to that row's family); the trailing
    ``nv + n_slack`` rows are the identity block carrying the box bounds (joint
    qddot limits plus ``slack >= 0`` for each family). The CBF block is stored with a FULL (dense)
    sparsity pattern — every entry is an explicit structural nonzero, including
    zeros — so the pattern is invariant for a given ``n_c``. That is what lets
    ``prob.update(Ax=...)`` stay valid across ticks even when a Jacobian entry
    passes through zero; a plain ``csc_matrix(G)`` would drop those zeros and
    silently change the pattern.

    Args:
        G: ``(n_c, nv + 1)`` CBF row block, or ``None`` when no CBF row is
            active (``n_c == 0``), in which case only the identity block is
            returned.
        nv: Number of joint variables.
        n_slack: Number of slack variables appended after them, so the decision
            vector is ``nv + n_slack`` long. One slack per CONSTRAINT FAMILY,
            not one overall: a single shared slack lets a badly-violated row of
            one kind relax every row of every other kind. Measured on hardware —
            a joint-limit row drove s to 3.37, which relaxed the self-collision
            rows by the same 3.37 until the firmware had to fire
            self_collision_avoidance_violation itself.

    Returns:
        The CSC constraint matrix. ``l <= A x <= u`` is set by
        :func:`build_osqp_bounds`, which uses the same row order.
    """
    n_x = nv + n_slack
    box = sparse.identity(n_x, format='csc')
    if G is None:                       # n_c == 0 → box bounds only
        return box
    if G.shape[1] != n_x:
        raise ValueError(
            f'G has {G.shape[1]} columns, expected nv + n_slack = {n_x}')
    n_c  = G.shape[0]
    rows = np.repeat(np.arange(n_c), n_x)
    cols = np.tile(np.arange(n_x), n_c)
    cbf  = sparse.csc_matrix((G.ravel(), (rows, cols)), shape=(n_c, n_x))
    return sparse.vstack([cbf, box], format='csc')


def build_osqp_bounds(
    G: Optional[np.ndarray],
    h_qp: Optional[np.ndarray],
    box_lb: np.ndarray,
    box_ub: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Build the ``(l, u)`` bounds for ``l <= A x <= u``.

    MOVED here from ``CBFSafetyFilter._osqp_lu`` in Phase 3; body unchanged
    except that the two box arrays are now arguments instead of node attributes.

    CBF rows are one-sided (``-inf <= G x <= h_qp``); the identity block carries
    the box (``box_lb <= x <= box_ub``). Row order matches :func:`build_osqp_A`.

    Args:
        G: ``(n_c, nv + 1)`` CBF row block, or ``None`` when ``n_c == 0``.
        h_qp: ``(n_c,)`` upper bounds for the CBF rows, or ``None`` when
            ``n_c == 0``.
        box_lb: ``(nv + 1,)`` lower box bound, owned by the caller.
        box_ub: ``(nv + 1,)`` upper box bound, owned by the caller.

    Returns:
        ``(l, u)``. When ``G is None`` these are the caller's own ``box_lb`` /
        ``box_ub`` arrays returned by reference — not copies — exactly as
        before, so the no-constraint path allocates nothing.
    """
    if G is None:
        return box_lb, box_ub
    n_c = G.shape[0]
    l = np.concatenate([np.full(n_c, -np.inf), box_lb])
    u = np.concatenate([h_qp,                  box_ub])
    return l, u


def build_row_rhs(con, qdot, qdot_cbf, *, k0: float, k1: float,
                  retreat_horizon: float, speed_horizon: float):
    """The right-hand side ``h_qp`` of every CBF row, for one QP tick.

    Three kinds of row share one array, and they do NOT share a formula:

    * BARRIER rows (obstacle, self-collision, joint-limit, singularity) —
      the HOCBF ``h_qp = k1·ḣ + k0·h̄ + ċ``, with ``ḣ = aᵀq̇ − v_obs``. ``aᵀq̇``
      is refreshed here at the QP rate; ``ċ`` and ``h̄`` are carried from the
      50 Hz snapshot. The velocity feeding ``k1`` is the lightly smoothed
      ``qdot_cbf``, because it is a DERIVATIVE of a measured signal that k1
      multiplies straight into the bound — unfiltered it accounted for 159 % of
      this term's whole swing on hardware and the command oscillated.
    * the Phase-1 feedforward ``b_ff``, added only when the flag put it in the
      snapshot; ``None`` there means the expression is bit-identical to the
      pre-Phase-1 build, which is what the regression asserts.
    * RATE-CAP rows (retreat caps, then task-space speed caps) — these are not
      barriers at all. They are stored with a negated ``a`` so the generic
      ``G = [−A | −e]`` turns them into UPPER bounds, and their RHS is the
      one-step ``(v_cap − aᵀq̇)/T``. They occupy the TRAILING ``n_cap`` rows,
      retreat first, which is the only reason the snapshot pins their position:
      it lets this address them by slice (a view) instead of by mask (a copy).
      They use the RAW ``qdot`` — a measured velocity, not a noisy derivative.

    Positive indices throughout. Negative-index slices read more naturally but
    have a trap: with no speed rows the retreat slice would end at index 0 and
    silently select nothing, leaving those rows on the barrier's RHS.

    Args:
        con: the :class:`~franka_experiments.utils.cbf_state_rows.ConstraintSnap`.
        qdot: (nv,) measured joint velocity, fresh this tick.
        qdot_cbf: (nv,) lightly smoothed velocity, for the ``k1`` term only.
        k0, k1: linear class-K gains, shared by every barrier family.
        retreat_horizon, speed_horizon: [s] enforcement horizons of the two
            rate-cap families.

    Returns:
        ``(h_qp, (retreat_rate, retreat_cap, link_speed, link_speed_cap))`` —
        the bound vector, and the four numbers CBFDIAG reports about the caps
        (0.0 when that family has no rows this tick, never a stale value).
    """
    h_qp = k1 * (con.A @ qdot_cbf - con.v_obs) + k0 * con.h_bar + con.jdot_qdot
    if con.b_ff is not None:
        h_qp += con.b_ff

    rtr = rtr_cap = spd = spd_cap = 0.0
    if con.n_cap:
        n_c, k, r = con.A.shape[0], con.n_cap, con.n_rtr
        t0 = n_c - k
        rate = -(con.A[t0:] @ qdot)      # aᵀq̇ on the cap rows, >0 = the
                                         # bounded direction of that family
        if r:
            h_qp[t0:t0 + r] = retreat_cap_rhs(con.A[t0:t0 + r], qdot,
                                              con.cap_v[:r], retreat_horizon)
            rtr, rtr_cap = float(np.max(rate[:r])), float(np.min(con.cap_v[:r]))
        if k > r:
            h_qp[t0 + r:] = retreat_cap_rhs(con.A[t0 + r:], qdot,
                                            con.cap_v[r:], speed_horizon)
            spd, spd_cap = float(np.max(rate[r:])), float(np.min(con.cap_v[r:]))
    return h_qp, (rtr, rtr_cap, spd, spd_cap)


# Shape constants for tangential_bias's two internal blends. Not exposed as
# ROS parameters: they set the WIDTH of a smoothing transition, not a gain —
# retuning them changes how far into "nominal has no sideways intent" the
# blend starts, not how hard the steering pushes. cbf_tangential_gain and
# cbf_tangential_filter_alpha are the knobs meant for hardware tuning.
_TB_BLEND_FRAC = 0.15   # q̈_nom's orthogonal part must reach this fraction of
                        # ‖q̈_nom‖ before it alone drives the bias direction —
                        # below it the qdot fallback is blended in smoothly.
_TB_DRIFT_REF  = 0.05   # [rad/s] orthogonal q̇ magnitude at which the qdot
                        # fallback reaches full confidence. Below it the
                        # fallback is faded toward zero rather than normalised
                        # to a full-scale, noise-driven direction.


def pad_rows_to_block(G: Optional[np.ndarray], h_qp: Optional[np.ndarray],
                      block: int) -> Tuple[Optional[np.ndarray],
                                           Optional[np.ndarray]]:
    """Round the CBF row block up to a multiple of ``block`` with inert rows.

    OSQP's sparsity pattern is a function of the ROW COUNT, so every change in
    ``n_c`` forces the node to build a new ``osqp.OSQP()`` and factorize from
    scratch — and n_c changes constantly: perception publishes a varying number
    of control points, and rows engage and disengage as the arm moves (22 → 29
    → 30 → 34 → 36 → 37 across a few hundred ms of one hardware log). At 100 Hz
    in a Python node that is a per-tick allocation plus factorization spike and
    the GC pressure that comes with it, which is how the QP thread ends up
    starving the executor's IO thread — the "joint state stale → braking on
    last known q̇" that immediately preceded a ``joint_velocity_violation`` on
    hardware.

    Quantising the row count to a block absorbs that churn: n_c anywhere in
    33..48 becomes 48, one pattern, one ``setup()``, ``update()`` thereafter,
    and the warm start survives. Only crossing a block boundary re-setups.

    The padding rows are INERT by construction: all-zero on the joint columns
    with a single ``−1`` in the LAST slack column and ``+inf`` on the right,
    i.e. ``−s <= +inf`` for a slack that is already constrained ``>= 0``.
    Always satisfied, never active, contributes nothing to the KKT system —
    and non-zero, so it raises no zero-row question in OSQP's scaling. Which
    slack it leans on is arbitrary precisely because the row is never active.
    It cannot change the solution, only the shape of the problem the solver is
    handed.

    Args:
        G: ``(n_c, nv + n_slack)`` row block, or ``None`` when ``n_c == 0``.
        h_qp: ``(n_c,)`` right-hand side, or ``None``.
        block: quantisation step. ``<= 1`` disables padding and returns the
            caller's own arrays unchanged.

    Returns:
        ``(G, h_qp)`` — the caller's arrays by reference when no padding is
        needed (already a multiple, ``block <= 1``, or ``n_c == 0``), so the
        common case allocates nothing.
    """
    if G is None or h_qp is None or block <= 1:
        return G, h_qp
    n_c = G.shape[0]
    n_pad = -(-n_c // block) * block         # ceil(n_c / block) * block
    if n_pad == n_c:
        return G, h_qp
    G_out = np.zeros((n_pad, G.shape[1]), dtype=np.float64)
    G_out[:n_c] = G
    G_out[n_c:, -1] = -1.0
    h_out = np.empty(n_pad, dtype=np.float64)
    h_out[:n_c] = h_qp
    h_out[n_c:] = np.inf
    return G_out, h_out


def tangential_bias(qddot_nom: np.ndarray, qdot: np.ndarray, con, *,
                    gain: float, engage_margin: float,
                    max_bias: float = None) -> np.ndarray:
    """The sideways nudge to ADD to q̈_nom, so the QP steers AROUND a close
    obstacle/self-collision instead of only backing straight off it.

    The QP itself already guarantees safety: every row's hard inequality
    ``aᵢᵀq̈ >= h_qpᵢ`` is untouched by this function. What it does not
    guarantee is that the FREE part of q̈ — the part orthogonal to every active
    ``aᵢ``, which the barrier has no opinion about — actually gets used to go
    AROUND the obstacle rather than sitting at zero while the normal component
    is clipped to the barrier. That reads as "the arm only ever backs
    straight off", because ``argmin ‖q̈ − q̈_nom‖`` under a single active
    constraint is exactly the projection of ``q̈_nom`` onto the constraint
    hyperplane — no tangential term in the objective, no tangential motion in
    the solution, however open the sideways direction is.

    For each OBSTACLE/SELF-COLLISION row inside ``engage_margin`` this blends
    two sources of "which way is sideways", both built as ``v − (v·â)â`` so
    BOTH are orthogonal to ``â`` BY CONSTRUCTION (up to float rounding): the
    result can only ever ADD tangential motion, never reduce the
    normal-direction retreat the barrier demands.

    * q̈_nom's OWN component orthogonal to â — amplifying the sideways intent
      the nominal command already has near that obstacle, not inventing one.
    * q̇'s (filtered) component orthogonal to â, as a fallback for when q̈_nom
      has none — "keep curving the way the arm is already drifting" rather
      than inventing an arbitrary left/right choice.

    Both blends are SMOOTHSTEPS, not hard switches, on purpose — a hard
    ``if perp_norm < eps`` branch measured as the dominant source of the
    "oscillates too much" report: 100 Hz noise straddling the threshold made
    the bias FLIP between two unrelated directions tick to tick, and
    normalising a near-zero, mostly-noise ``qdot`` component to a full-scale
    direction turned tiny drift into a full-strength kick in a near-random
    direction. Here:

    * the q̈_nom-vs-qdot blend ramps over ``_TB_BLEND_FRAC · ‖q̈_nom‖`` instead
      of snapping at one point;
    * the qdot fallback carries its own CONFIDENCE, ``‖q̇_perp‖ / _TB_DRIFT_REF``
      clamped to ``[0, 1]`` — a barely-moving arm contributes a barely-there
      bias instead of a full-scale one in whatever direction rounding error
      happens to point.

    Rows accumulate additively (a row's tangent plane may have a nonzero
    component along a DIFFERENT row's â when two obstacles are close
    together), and the total is norm-clamped to ``max_bias`` — bounding the
    worst case where several rows reinforce each other, independent of how
    large ``q̈_nom`` itself gets during a tracking-error spike.

    This function is STATELESS and re-evaluated fresh every call: it does not
    smooth across ticks. Tick-to-tick smoothing (the other half of "too
    oscillatory") is the CALLER's job — see the EMA in
    ``CBFSafetyFilter._qp_tick`` / ``cbf_tangential_filter_alpha`` — kept out
    of this function so it stays a pure, unit-testable map from one snapshot
    to one bias vector.

    Args:
        qddot_nom: (nv,) the tracking command this bias will be added to.
        qdot: (nv,) joint velocity — pass the FILTERED copy
            (``CBFSafetyFilter._qdot_cbf``), not the raw measurement: this
            function normalises it to extract a direction, which amplifies
            noise even more than the k1 term ``_qdot_cbf`` already exists for.
        con: the :class:`~franka_experiments.utils.cbf_state_rows.ConstraintSnap`.
        gain: dimensionless multiplier on the accumulated tangential term.
            0.0 (or no engaged row) returns an exact zero vector.
        engage_margin: [m] a row only contributes once its barrier value
            ``h̄ᵢ`` is below this — far rows add no bias and no cost.
        max_bias: [rad/s²] cap on ``‖bias‖``, or ``None``/``<= 0`` for no cap.

    Returns:
        (nv,) the bias to ADD to ``q̈_nom`` — NOT ``q̈_nom`` itself. Exactly
        ``np.zeros_like(qddot_nom)`` whenever nothing is engaged, so a caller
        EMA-ing this return value decays cleanly to zero on disengagement.
    """
    if gain <= 0.0 or con is None or con.A.shape[0] == 0:
        return np.zeros_like(qddot_nom)
    mask = ((con.group == G_OBS) | (con.group == G_SC)) & (con.h_bar < engage_margin)
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return np.zeros_like(qddot_nom)
    nom_norm = float(np.linalg.norm(qddot_nom))
    if nom_norm < 1e-9:
        return np.zeros_like(qddot_nom)   # no direction to amplify or scale to
    bias = np.zeros_like(qddot_nom)
    for i in idx:
        a = con.A[i]
        a2 = float(a @ a)
        if a2 < 1e-12:
            continue
        a_hat = a / np.sqrt(a2)

        qn_perp = qddot_nom - float(qddot_nom @ a_hat) * a_hat
        qn_perp_norm = float(np.linalg.norm(qn_perp))
        s = float(np.clip(qn_perp_norm / (_TB_BLEND_FRAC * nom_norm), 0.0, 1.0))
        s = s * s * (3.0 - 2.0 * s)                       # smoothstep(s)

        v_perp = qdot - float(qdot @ a_hat) * a_hat
        v_perp_norm = float(np.linalg.norm(v_perp))
        if v_perp_norm > 1e-9:
            conf = float(np.clip(v_perp_norm / _TB_DRIFT_REF, 0.0, 1.0))
            fallback = (v_perp / v_perp_norm) * (conf * nom_norm)
        else:
            fallback = np.zeros_like(qddot_nom)

        perp = s * qn_perp + (1.0 - s) * fallback
        w = float(np.clip((engage_margin - con.h_bar[i]) / engage_margin, 0.0, 1.0))
        bias += w * perp
    bias *= gain
    if max_bias is not None and max_bias > 0.0:
        bias_norm = float(np.linalg.norm(bias))
        if bias_norm > max_bias:
            bias *= max_bias / bias_norm
    return bias
