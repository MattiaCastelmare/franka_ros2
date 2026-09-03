"""Assembling the OSQP problem matrices for the acceleration-level CBF filter.

OWNS
----
The translation from CBF constraint rows into the exact sparse structures the
native OSQP interface expects:

* :func:`build_osqp_A`      — the constraint matrix ``A = [ G ; I ]``
* :func:`build_osqp_bounds` — the matching ``(l, u)`` bound vectors

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

from franka_experiments.utils.cbf_state_rows import retreat_cap_rhs


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
