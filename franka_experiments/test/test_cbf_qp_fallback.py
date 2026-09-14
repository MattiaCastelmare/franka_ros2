"""Phase 3a/3b: the QP always answers, and the box is never negotiable.

* the fallback ladder in cbf_qp_assembly — a forced OSQP failure (max_iter = 1)
  still yields a finite command inside every hard limit, and each rung is
  exercised on its own;
* the identity block is the ONLY hard constraint, carries no slack column,
  and the solution integrated over one tick stays inside the velocity limit
  even when adversarial rows demand otherwise.

Needs OSQP (runs in the ROS container, skips elsewhere).
"""

import numpy as np
import pytest

osqp = pytest.importorskip('osqp')
sparse = pytest.importorskip('scipy.sparse')

from franka_experiments.utils.cbf_hard_limits import hard_accel_box     # noqa: E402
from franka_experiments.utils.cbf_qp_assembly import (                   # noqa: E402
    accept_iterate, box_only_solve, braking_command, build_osqp_A,
    build_osqp_bounds)
from franka_experiments.utils.cbf_state_rows import NV, NX, N_SLACK      # noqa: E402

SOLVED = osqp.constant('OSQP_SOLVED')
INACC = osqp.constant('OSQP_SOLVED_INACCURATE')
MAXIT = osqp.constant('OSQP_MAX_ITER_REACHED')
QD = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])
QDD = np.array([6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0])
Q_MIN = np.array([-2.9007, -1.8361, -2.9007, -3.0770, -2.8763, 0.4398, -3.0508])
Q_MAX = np.array([2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508])
DT = 0.01


def _P():
    P = np.eye(NX)
    P[NV:, NV:] *= 1000.0
    return sparse.csc_matrix(P)


def _box(qdot, q=None):
    q = 0.5 * (Q_MIN + Q_MAX) if q is None else q
    lb, ub = hard_accel_box(q, qdot, acc_lb=-QDD, acc_ub=QDD, qdot_max=QD, v_margin=0.9,
                            q_min=Q_MIN, q_max=Q_MAX, q_margin=0.05, brake_eta=0.6,
                            dt=DT, relax_dt=0.10, clip_to_limits=True)
    return (np.concatenate([lb, np.zeros(N_SLACK)]),
            np.concatenate([ub, np.full(N_SLACK, 1e6)]))


def _rows(rng, n, scale=50.0):
    """n adversarial obstacle-family rows demanding a huge aᵀq̈."""
    G = np.zeros((n, NX))
    A = rng.normal(size=(n, NV))
    G[:, :NV] = -A
    G[:, NV] = -1.0
    h = -scale * np.ones(n)          # aᵀq̈ + s ≥ scale
    return G, h


# ── accept_iterate ───────────────────────────────────────────────────────────

def test_a_solved_iterate_is_kept_inside_the_box():
    x = np.arange(NX, dtype=float)
    lb, ub = _box(np.zeros(NV))
    y = accept_iterate(x, SOLVED, lb, ub, NV, solved=SOLVED, inaccurate=INACC)
    np.testing.assert_array_equal(y[:NV], np.clip(x[:NV], lb[:NV], ub[:NV]))
    np.testing.assert_array_equal(y[NV:], x[NV:])


def test_an_inaccurate_iterate_is_clipped_into_the_box():
    lb, ub = _box(np.zeros(NV))
    x = np.concatenate([100.0 * np.ones(NV), np.zeros(N_SLACK)])
    y = accept_iterate(x, INACC, lb, ub, NV, solved=SOLVED, inaccurate=INACC)
    assert np.all(y[:NV] <= ub[:NV] + 1e-12) and np.all(y[:NV] >= lb[:NV] - 1e-12)
    np.testing.assert_array_equal(y[:NV], ub[:NV])


def test_max_iter_and_nonfinite_are_rejected():
    lb, ub = _box(np.zeros(NV))
    assert accept_iterate(np.zeros(NX), MAXIT, lb, ub, NV, solved=SOLVED, inaccurate=INACC) is None
    x = np.zeros(NX); x[0] = np.nan
    assert accept_iterate(x, SOLVED, lb, ub, NV, solved=SOLVED, inaccurate=INACC) is None
    assert accept_iterate(None, SOLVED, lb, ub, NV, solved=SOLVED, inaccurate=INACC) is None


def test_inaccurate_is_rejected_when_the_flag_disables_it():
    lb, ub = _box(np.zeros(NV))
    assert accept_iterate(np.zeros(NX), INACC, lb, ub, NV, solved=SOLVED, inaccurate=-999) is None


# ── box_only_solve ───────────────────────────────────────────────────────────

def test_box_only_is_the_projection_of_the_nominal_onto_the_box():
    rng = np.random.default_rng(0)
    for _ in range(10):
        qdot = rng.uniform(-0.9, 0.9, NV) * QD
        lb, ub = _box(qdot)
        nom = rng.normal(size=NV) * 20.0
        q = np.zeros(NX); q[:NV] = -nom
        x = box_only_solve(_P(), q, lb, ub, max_iter=4000)
        assert x is not None and np.all(np.isfinite(x))
        np.testing.assert_allclose(x[:NV], np.clip(nom, lb[:NV], ub[:NV]), atol=1e-3)
        assert np.all(x[:NV] <= ub[:NV] + 1e-9) and np.all(x[:NV] >= lb[:NV] - 1e-9)


# ── braking_command ──────────────────────────────────────────────────────────

def test_braking_is_inside_the_tightened_box_and_opposes_the_velocity():
    qdot = np.array([2.0, -2.0, 0.1, 0.0, 4.0, -3.0, 1.0])
    lb, ub = _box(qdot)
    b = braking_command(qdot, lb[:NV], ub[:NV], k_brake=3.0)
    assert np.all(b <= ub[:NV] + 1e-12) and np.all(b >= lb[:NV] - 1e-12)
    assert np.all(b * qdot <= 1e-12)


# ── The ladder end to end: a forced failure never leaves the box ────────────

def test_forced_solver_failure_still_yields_a_finite_in_box_command():
    """max_iter = 1 makes OSQP report MAX_ITER on the full problem; the ladder
    must then answer from level 1 with a finite vector inside the box."""
    rng = np.random.default_rng(1)
    qdot = 0.85 * QD * np.sign(rng.normal(size=NV))
    lb, ub = _box(qdot)
    G, h = _rows(rng, 6)
    nom = rng.normal(size=NV) * 5.0
    qv = np.zeros(NX); qv[:NV] = -nom
    l, u = build_osqp_bounds(G, h, lb, ub)
    prob = osqp.OSQP()
    prob.setup(P=_P(), q=qv, A=build_osqp_A(G, NV, N_SLACK), l=l, u=u,
               warm_start=True, max_iter=1, verbose=False, check_termination=1)
    res = prob.solve()
    x = accept_iterate(res.x, res.info.status_val, lb, ub, NV, solved=SOLVED, inaccurate=INACC)
    assert x is None, res.info.status
    x = box_only_solve(_P(), qv, lb, ub, max_iter=4000)
    assert x is not None and np.all(np.isfinite(x))
    assert np.all(x[:NV] <= ub[:NV] + 1e-9) and np.all(x[:NV] >= lb[:NV] - 1e-9)
    # And the last rung, should even that fail, is closed form.
    b = braking_command(qdot, lb[:NV], ub[:NV], k_brake=3.0)
    assert np.all(np.isfinite(b))


# ── 3b: the box is hard, has no slack, and the one-tick integral holds ──────

def test_the_identity_block_carries_the_box_and_no_slack_column():
    rng = np.random.default_rng(2)
    G, h = _rows(rng, 4)
    lb, ub = _box(np.zeros(NV))
    A = build_osqp_A(G, NV, N_SLACK).toarray()
    l, u = build_osqp_bounds(G, h, lb, ub)
    I = A[4:, :]
    np.testing.assert_array_equal(I, np.eye(NX))
    np.testing.assert_array_equal(l[4:], lb)
    np.testing.assert_array_equal(u[4:], ub)
    # a joint row of the identity block touches no slack column
    assert not I[:NV, NV:].any()


def test_adversarial_rows_never_push_the_one_tick_velocity_past_the_limit():
    """Rows demanding a huge acceleration, priced expensively, with the joints
    already near their velocity limit: the solution must satisfy
    |q̇ + q̈·dt| ≤ v_margin·q̇max, joint by joint, on every instance."""
    rng = np.random.default_rng(3)
    for trial in range(40):
        qdot = rng.uniform(0.7, 0.9, NV) * QD * np.sign(rng.normal(size=NV))
        q = rng.uniform(Q_MIN + 0.3, Q_MAX - 0.3)
        lb, ub = _box(qdot, q)
        G, h = _rows(rng, int(rng.integers(1, 12)), scale=200.0)
        nom = rng.normal(size=NV) * 30.0
        qv = np.zeros(NX); qv[:NV] = -nom
        l, u = build_osqp_bounds(G, h, lb, ub)
        prob = osqp.OSQP()
        prob.setup(P=_P(), q=qv, A=build_osqp_A(G, NV, N_SLACK), l=l, u=u,
                   warm_start=True, max_iter=20000, verbose=False,
                   eps_abs=1e-5, eps_rel=1e-5)
        res = prob.solve()
        x = accept_iterate(res.x, res.info.status_val, lb, ub, NV, solved=SOLVED, inaccurate=INACC)
        assert x is not None, res.info.status
        qdd = x[:NV]
        assert np.all(qdd <= ub[:NV] + 1e-12) and np.all(qdd >= lb[:NV] - 1e-12)
        v_next = qdot + qdd * DT
        assert np.all(np.abs(v_next) <= 0.9 * QD + 1e-9), (trial, np.abs(v_next) / QD)
        assert np.all(np.abs(qdd) <= QDD + 1e-12)


def test_the_guard_hole_the_clip_closes_is_real():
    """Joint 7 diving at 4.7 rad/s toward a position limit 0.3 rad away: the
    unclipped box demands >100 rad/s² against a 17 rad/s² limit; clipped, it
    demands exactly the limit."""
    q = 0.5 * (Q_MIN + Q_MAX); q[6] = Q_MIN[6] + 0.3
    qdot = np.zeros(NV); qdot[6] = -0.9 * QD[6]
    kw = dict(acc_lb=-QDD, acc_ub=QDD, qdot_max=QD, v_margin=0.9, q_min=Q_MIN,
              q_max=Q_MAX, q_margin=0.05, brake_eta=0.6, dt=DT, relax_dt=0.10)
    lb0, ub0 = hard_accel_box(q, qdot, **kw)
    assert lb0[6] > 100.0 and ub0[6] == lb0[6]
    lb1, ub1 = hard_accel_box(q, qdot, clip_to_limits=True, **kw)
    assert lb1[6] == QDD[6] and ub1[6] == QDD[6]
    # every other joint is untouched by the clip
    np.testing.assert_array_equal(lb1[:6], lb0[:6])
    np.testing.assert_array_equal(ub1[:6], ub0[:6])
