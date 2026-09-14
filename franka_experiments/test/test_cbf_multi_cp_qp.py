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


# ── Per-family slack ─────────────────────────────────────────────────────────
#
# One shared slack couples constraint families that are not even in the same
# units. Measured on hardware: a joint-limit row (radians) drove the shared
# slack to 3.37, and that same 3.37 relaxed every self-collision row (metres)
# by the same amount until the firmware fired its own
# self_collision_avoidance_violation. These tests pin the separation.

G_OBS, G_SC, G_QLIM = 0, 1, 2
N_SLACK = 3
NX = NV + N_SLACK
RHO = {G_OBS: 1000.0, G_SC: 20000.0, G_QLIM: 200.0}


def _grouped_G(A, groups):
    """[-A | -e_group]: only the row's own family column carries the -1."""
    n_c = A.shape[0]
    G = np.zeros((n_c, NX))
    G[:, :NV] = -A
    G[np.arange(n_c), NV + np.asarray(groups)] = -1.0
    return G


def _solve_row_closed_form(c, rho, lb_j, ub_j):
    """Exact optimum of  min ½qdd² + ½rho·s²  s.t.  qdd + s >= c,  qdd in box.

    Solved in closed form rather than handed to SLSQP: the realistic prices
    (rho up to 2e4) against a demand of ~500 are badly enough conditioned that
    the solver reports "positive directional derivative" and the assertion would
    be about the solver, not about the QP. Stationarity of
    ½qdd² + ½rho(c-qdd)² gives qdd = rho·c/(1+rho); clip it, and the slack takes
    up whatever the box could not.
    """
    if c <= 0.0:
        return 0.0, 0.0                      # already satisfied at rest
    qdd = float(np.clip(rho * c / (1.0 + rho), lb_j, ub_j))
    return qdd, max(c - qdd, 0.0)


def test_a_violated_family_does_not_relax_the_others():
    """The regression, on the exact shape that failed on hardware.

    Row A: a joint-limit row demanding far more than the box can deliver.
    Row B: a self-collision row, easily satisfiable on its own.
    They sit on different joints, so with per-family slacks the problem
    decouples and each family pays for itself.
    """
    box_lb, box_ub = _box()
    c_qlim, c_sc = 500.0, 0.5            # = -h_qp for each row

    qdd4, s_qlim = _solve_row_closed_form(c_qlim, RHO[G_QLIM],
                                          box_lb[3], box_ub[3])
    qdd6, s_sc = _solve_row_closed_form(c_sc, RHO[G_SC],
                                        box_lb[5], box_ub[5])

    assert abs(qdd4 - box_ub[3]) < 1e-9, 'joint4 saturates its box'
    assert s_qlim > 400.0, 'the impossible row absorbs a huge slack'
    # The self-collision row is untouched by that: it still commands a real
    # acceleration instead of being covered by borrowed slack.
    assert abs(qdd6 - c_sc) < 1e-3, (
        f'the sc row must still demand ~{c_sc} rad/s², got {qdd6:.4f}')
    assert s_sc < 1e-3, f'and pay almost no slack, got {s_sc:.5f}'


def test_shared_slack_would_have_disarmed_the_self_collision_row():
    """Contrast: one shared slack, same data, and the sc row commands NOTHING.

    With a single s the joint-limit row drives it to ~496. That same 496 already
    satisfies  qdd6 + s >= 0.5  at qdd6 = 0, so the self-collision row asks for
    no acceleration at all — which is why the firmware ended up firing
    self_collision_avoidance_violation on its own.
    """
    box_lb, box_ub = _box()
    c_qlim, c_sc = 500.0, 0.5
    # The dominant row sets the shared slack.
    qdd4, s_shared = _solve_row_closed_form(c_qlim, 1000.0, box_lb[3], box_ub[3])
    assert s_shared > 400.0, s_shared
    # The sc row is then satisfied for free.
    assert 0.0 + s_shared >= c_sc, 'shared slack covers the sc row entirely'
    qdd6_shared = max(c_sc - s_shared, 0.0)
    assert qdd6_shared == 0.0, 'the sc row demands nothing — disarmed'

    # Per-family, the same row demands a real acceleration.
    qdd6_grouped, _ = _solve_row_closed_form(c_sc, RHO[G_SC],
                                             box_lb[5], box_ub[5])
    assert qdd6_grouped > 0.4, (qdd6_grouped, qdd6_shared)


def test_every_family_stays_feasible_with_its_own_slack():
    rng = np.random.default_rng(5)
    box_lb, box_ub = _box()
    for _ in range(60):
        n_c = int(rng.integers(1, 12))
        A = rng.normal(scale=0.4, size=(n_c, NV))
        groups = rng.integers(0, N_SLACK, size=n_c)
        h_qp = rng.normal(scale=5.0, size=n_c)
        G = _grouped_G(A, groups)
        assert _feasible_nx(G, h_qp, box_lb, box_ub)


def _feasible_nx(G, h_qp, box_lb, box_ub):
    lb = np.concatenate([box_lb[:NV], np.zeros(N_SLACK)])
    ub = np.concatenate([box_ub[:NV], np.full(N_SLACK, 1e6)])
    A_qp = build_osqp_A(G, NV, N_SLACK).toarray()
    l, u = build_osqp_bounds(G, h_qp, lb, ub)
    ok_u, ok_l = np.isfinite(u), np.isfinite(l)
    A_ub = np.vstack([A_qp[ok_u], -A_qp[ok_l]])
    b_ub = np.concatenate([u[ok_u], -l[ok_l]])
    res = linprog(np.zeros(NX), A_ub=A_ub, b_ub=b_ub,
                  bounds=[(None, None)] * NX, method='highs')
    return res.status == 0


def test_assembled_shape_carries_all_three_slacks():
    A = np.zeros((4, NV))
    G = _grouped_G(A, [G_OBS, G_SC, G_QLIM, G_OBS])
    A_qp = build_osqp_A(G, NV, N_SLACK)
    assert A_qp.shape == (4 + NX, NX)
    # Each row touches exactly one slack column.
    block = G[:, NV:]
    assert np.array_equal((block != 0).sum(axis=1), np.ones(4))
    assert block[0, G_OBS] == -1.0 and block[1, G_SC] == -1.0
    assert block[2, G_QLIM] == -1.0 and block[3, G_OBS] == -1.0
