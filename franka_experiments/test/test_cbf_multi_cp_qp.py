"""Feasibility of the CBF QP under simultaneous multi-control-point activation.

Since perception publishes one LinkDistance per CONTROL POINT rather than one
per link, the QP routinely carries ~11 CBF rows instead of ~5, and several of
them can be violated (h̄ < 0) at once.  This module checks that widening the
constraint set cannot empty it.

Exercises the real ``build_osqp_A`` / ``build_osqp_bounds`` (numpy + scipy
only).  OSQP itself is not imported: it reports primal infeasibility exactly
when the feasible set is empty, so a feasibility LP over the assembled
``l <= A x <= u`` answers the question without the solver — and runs in CI
without a sourced workspace.

Run with pytest, or directly:  python3 test_cbf_multi_cp_qp.py
"""

import numpy as np
from scipy.optimize import linprog

from franka_experiments.utils.cbf_hard_limits import velocity_accel_box
from franka_experiments.utils.cbf_qp_assembly import (
    build_osqp_A,
    build_osqp_bounds,
)

NV       = 7
K0, K1   = 25.0, 10.5        # fr3_control.yaml
D_SAFE   = 0.20
SLACK_UB = 1e6               # cbf_safety_filter._box_ub[-1]

# Static accel box from fr3_control.yaml joint_limits col[3].
QDDOT_MAX = np.array([6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0])
QDOT_MAX  = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])


def _box(qdot=None, v_margin=0.9, dt=0.01):
    """(lb, ub) over [qddot (7), slack (1)], optionally velocity-tightened.

    Delegates to the production velocity_accel_box rather than restating its
    arithmetic — a local reimplementation silently omitted its `ub = max(ub, lb)`
    feasibility guard and reported an empty box that the real filter never
    produces.
    """
    lb, ub = -QDDOT_MAX.copy(), QDDOT_MAX.copy()
    if qdot is not None:
        velocity_accel_box(qdot, acc_lb=-QDDOT_MAX, acc_ub=QDDOT_MAX,
                           qdot_max=QDOT_MAX, v_margin=v_margin, dt=dt,
                           out_lb=lb, out_ub=ub)
    return np.append(lb, 0.0), np.append(ub, SLACK_UB)


def _rows(rng, n_c, qdot, h_bar=None):
    """Assemble (G, h_qp) for n_c control points, as _update_constraints does."""
    # a = n̂ᵀJp: unit normal times a position Jacobian of realistic scale.
    A = np.vstack([
        (rng.normal(size=3) / np.linalg.norm(rng.normal(size=3) + 1e-9))
        @ rng.normal(scale=0.4, size=(3, NV))
        for _ in range(n_c)
    ])
    if h_bar is None:
        # d in [0, 1.2] (clamped surface gap up to the obstacle horizon).
        h_bar = rng.uniform(0.0, 1.2, size=n_c) - D_SAFE
    jdq  = rng.normal(scale=2.0, size=n_c)
    h_qp = K1 * (A @ qdot) + K0 * h_bar + jdq
    G = np.empty((n_c, NV + 1))
    G[:, :NV], G[:, -1] = -A, -1.0
    return G, h_qp


def _feasible(G, h_qp, box_lb, box_ub):
    """True iff {x : l <= A_qp x <= u} is non-empty, via a phase-1 LP."""
    A_qp = build_osqp_A(G, NV).toarray()
    l, u = build_osqp_bounds(G, h_qp, box_lb, box_ub)

    # l <= Ax <= u  →  [A; -A] x <= [u; -l], dropping the infinite rows.
    ub_ok = np.isfinite(u)
    lb_ok = np.isfinite(l)
    A_ub  = np.vstack([A_qp[ub_ok], -A_qp[lb_ok]])
    b_ub  = np.concatenate([u[ub_ok], -l[lb_ok]])

    res = linprog(np.zeros(NV + 1), A_ub=A_ub, b_ub=b_ub,
                  bounds=[(None, None)] * (NV + 1), method='highs')
    return res.status == 0


# ── Feasibility is preserved as control points pile up ───────────────────────

def test_feasible_for_every_row_count():
    """1..11 simultaneously active control points, randomised geometry."""
    rng = np.random.default_rng(20260902)
    for n_c in range(1, 12):
        for _ in range(40):
            qdot = rng.uniform(-1.0, 1.0, size=NV) * QDOT_MAX * 0.5
            G, h_qp = _rows(rng, n_c, qdot)
            assert _feasible(G, h_qp, *_box(qdot)), \
                f'infeasible at n_c={n_c}'


def test_feasible_when_every_control_point_is_breached():
    """Worst case: all 11 CPs at zero gap, so every h̄ = -d_safe at once."""
    rng = np.random.default_rng(7)
    for _ in range(60):
        qdot = rng.uniform(-1.0, 1.0, size=NV) * QDOT_MAX * 0.9
        G, h_qp = _rows(rng, 11, qdot, h_bar=np.full(11, -D_SAFE))
        assert _feasible(G, h_qp, *_box(qdot))


def test_adding_rows_never_removes_feasibility():
    """Growing 5 rows (old per-link pooling) to 11 (per-CP) stays feasible.

    Structural, not luck: one slack s appears with coefficient -1 in EVERY CBF
    row, so s = max_i(-h_qp_i - a_i^T qddot) satisfies all rows at once for any
    qddot in the box. The set can only empty if that s exceeds its 1e6 cap.
    """
    rng = np.random.default_rng(99)
    for _ in range(50):
        qdot = rng.uniform(-1.0, 1.0, size=NV) * QDOT_MAX * 0.5
        G11, h11 = _rows(rng, 11, qdot)
        box = _box(qdot)
        assert _feasible(G11[:5], h11[:5], *box)      # old pooled row count
        assert _feasible(G11,     h11,     *box)      # per-CP row count


def test_required_slack_stays_far_below_its_cap():
    """The 1e6 slack cap is the only way feasibility could be lost — quantify
    how much headroom the realistic worst case leaves."""
    rng = np.random.default_rng(3)
    worst = 0.0
    for _ in range(400):
        qdot = rng.uniform(-1.0, 1.0, size=NV) * QDOT_MAX
        _, h_qp = _rows(rng, 11, qdot, h_bar=np.full(11, -D_SAFE))
        worst = max(worst, float(np.max(-h_qp)))      # s needed at qddot = 0
    assert worst < SLACK_UB / 1000.0, worst           # >= 3 orders of headroom


def test_velocity_box_stays_nonempty_at_saturation():
    """Even past the velocity margin the box keeps lb <= ub (braking-only)."""
    qdot = 0.95 * QDOT_MAX                    # beyond v_margin = 0.9
    lb, ub = _box(qdot)
    assert np.all(lb <= ub), 'velocity_accel_box must not invert the box'
    # Past the margin by more than one tick of decel authority, the guard
    # collapses the box onto lb = -qddot_max: hardest legal braking, nothing else.
    assert np.all(ub[:NV] < 0.0), 'saturated joints must be forced to decelerate'
    assert np.allclose(ub[:NV], lb[:NV])
    rng = np.random.default_rng(11)
    G, h_qp = _rows(rng, 11, qdot)
    assert _feasible(G, h_qp, lb, ub)


# ── Row order / assembly sanity for the widened constraint set ───────────────

def test_assembled_shapes_track_row_count():
    rng = np.random.default_rng(5)
    qdot = np.zeros(NV)
    for n_c in (1, 5, 11):
        G, h_qp = _rows(rng, n_c, qdot)
        A_qp = build_osqp_A(G, NV)
        l, u = build_osqp_bounds(G, h_qp, *_box())
        assert A_qp.shape == (n_c + NV + 1, NV + 1)
        assert l.shape == u.shape == (n_c + NV + 1,)
        # CBF block is one-sided; the identity block carries the box.
        assert np.all(np.isneginf(l[:n_c]))
        assert np.allclose(u[:n_c], h_qp)


def test_cbf_block_keeps_a_full_sparsity_pattern():
    """Structural zeros must stay explicit, or prob.update(Ax=...) breaks when
    a Jacobian entry passes through zero — worth re-checking now that far more
    rows are pushed each tick."""
    G = np.zeros((11, NV + 1))
    G[:, -1] = -1.0                      # every qddot coefficient exactly zero
    A_qp = build_osqp_A(G, NV)
    assert A_qp[:11, :].nnz == 11 * (NV + 1)


if __name__ == '__main__':
    import pytest
    raise SystemExit(pytest.main([__file__, '-v']))
