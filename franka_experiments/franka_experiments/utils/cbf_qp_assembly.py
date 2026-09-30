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
                  retreat_horizon: float, speed_horizon: float,
                  zoned: bool = True):
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
        k0, k1: linear class-K gains, shared by every barrier family — UNLESS
            the snapshot carries per-row gains (``con.k0_row``/``k1_row``, set
            by the zone ladder), in which case these are the fallback only.
        zoned: honour the snapshot's per-row gains when it has them. False
            forces the scalar expression, which is how the regression proves
            that a flags-off build is bit-identical: it evaluates BOTH and
            compares, so the two paths must be selectable on one snapshot.
        retreat_horizon, speed_horizon: [s] enforcement horizons of the two
            rate-cap families.

    Returns:
        ``(h_qp, (retreat_rate, retreat_cap, link_speed, link_speed_cap))`` —
        the bound vector, and the four numbers CBFDIAG reports about the caps
        (0.0 when that family has no rows this tick, never a stale value).
    """
    # Per-row gains when the zone ladder put them in the snapshot, scalars
    # otherwise. Both branches are the SAME expression: numpy broadcasts a
    # length-n_c vector against the length-n_c operands elementwise exactly as
    # it broadcasts a scalar, so this is a change of gain schedule and not a
    # change of formula. That matters for the bit-identity claim — there is no
    # second code path to keep in sync, only a different multiplicand.
    kk0, kk1 = k0, k1
    if zoned:
        if getattr(con, 'k0_row', None) is not None:
            kk0 = con.k0_row
        if getattr(con, 'k1_row', None) is not None:
            kk1 = con.k1_row
    h_qp = kk1 * (con.A @ qdot_cbf - con.v_obs) + kk0 * con.h_bar + con.jdot_qdot
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


# ═════════════════════════════════════════════════════════════════════════════
#  Never "no solution": the fallback ladder
# ═════════════════════════════════════════════════════════════════════════════
#
# Every CBF row in this filter carries a slack column — obstacle, self-
# collision, joint-limit, singularity, retreat-cap and speed-cap families all
# have one — so the row block can never make the problem infeasible on its own.
# The only hard constraints are the identity block: the joint acceleration box
# (velocity- and position-tightened) and ``s >= 0``. That box is built with a
# feasibility guard (``hard_accel_box`` collapses onto ``lb`` when the demands
# conflict) and the slew box is pinned to a non-empty interval, so ``lb <= ub``
# always holds and the QP always HAS a solution.
#
# What can still happen is that OSQP does not FIND it within its budget — an
# ill-scaled instance after a Jacobian degenerates, a max_iter hit, a
# "solved inaccurate" — and until now that was reported upward as "QP not
# solved" and answered with open-loop braking. The controller must never see
# that as the first resort, so the failure is walked down a ladder, each rung
# strictly simpler and the last one closed-form:
#
#   level 0  the full problem, accepted when solved; a SOLVED_INACCURATE
#            iterate is also accepted, clipped into the hard box (it is a
#            feasible-up-to-tolerance point of the same problem, and clipping
#            can only move it INTO the box)
#   level 1  the same objective with every CBF row DROPPED — box only, a
#            throwaway OSQP on an identity constraint matrix, which is a
#            projection onto a box and converges in a handful of iterations
#   level 2  pure braking inside the box, ``clip(-k_brake * qdot, lb, ub)``,
#            which needs no solver at all
#
# Level 1 is "the barrier is off for this tick" and level 2 is "brake", and
# both are reported as a safety-chain fault upstream — but both are FINITE
# commands inside every hard limit, which is the property B4 asks for.

OSQP_LEVEL_FULL, OSQP_LEVEL_BOX, OSQP_LEVEL_BRAKE = 0, 1, 2
LEVEL_NAMES = ('full', 'box-only', 'brake')


def accept_iterate(x, status_val: int, box_lb: np.ndarray, box_ub: np.ndarray,
                   nv: int, *, solved: int, inaccurate: int):
    """The decision vector to use from one OSQP solve, or ``None``.

    ``solved`` and ``inaccurate`` are both taken, with the joint block clipped
    into the hard box (see the comment in the body: OSQP's constraint
    tolerance is not zero, and the box is the one constraint that must hold
    exactly). Anything else — max_iter, primal/dual infeasible, non-finite —
    is ``None`` and the caller walks down the ladder.
    """
    if x is None or not np.all(np.isfinite(x)):
        return None
    if status_val not in (solved, inaccurate):
        return None
    # Clip the joint block into the hard box in BOTH accepted cases. OSQP
    # satisfies its constraints only to eps_abs (1e-3 by default), so even a
    # SOLVED iterate can sit a hair outside the box; the box is the one
    # constraint that must hold exactly (it is what keeps the firmware reflex
    # from firing), and clipping can only move the point INTO it. The rows
    # are soft, so nothing is lost by it.
    y = np.array(x, dtype=np.float64, copy=True)
    np.clip(y[:nv], box_lb[:nv], box_ub[:nv], out=y[:nv])
    return y


def box_only_solve(P_csc, qvec: np.ndarray, box_lb: np.ndarray, box_ub: np.ndarray,
                   *, max_iter: int):
    """Level 1: the same objective over the hard box alone.

    A fresh OSQP instance every time — this rung fires rarely and must not
    share state with the warm-started full problem, whose iterate is exactly
    what just failed. Returns the decision vector or ``None`` (which, with a
    consistent box, does not happen — but the caller still has level 2).
    """
    import osqp
    n = qvec.size
    prob = osqp.OSQP()
    prob.setup(P=P_csc, q=qvec, A=sparse.identity(n, format='csc'),
               l=box_lb, u=box_ub, warm_start=False, max_iter=int(max_iter),
               eps_abs=1e-6, eps_rel=1e-6, verbose=False)
    res = prob.solve()
    x = res.x
    if (res.info.status_val not in (osqp.constant('OSQP_SOLVED'),
                                    osqp.constant('OSQP_SOLVED_INACCURATE'))
            or x is None or not np.all(np.isfinite(x))):
        return None
    y = np.array(x, dtype=np.float64, copy=True)
    np.clip(y, box_lb, box_ub, out=y)
    return y


def braking_command(qdot: np.ndarray, acc_lb: np.ndarray, acc_ub: np.ndarray,
                    *, k_brake: float) -> np.ndarray:
    """Level 2: ``clip(-k_brake * qdot, lb, ub)`` — closed form, always finite,
    always inside the box handed in. Pass the TIGHTENED box, not the static
    limits, so the braking step also honours the velocity/position caps."""
    return np.clip(-float(k_brake) * np.asarray(qdot, dtype=np.float64),
                   acc_lb, acc_ub)
