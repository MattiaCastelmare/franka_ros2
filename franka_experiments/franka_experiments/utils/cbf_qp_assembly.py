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
