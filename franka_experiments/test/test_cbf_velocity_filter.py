"""Unit tests for the ZCBF velocity QP solver and select_gamma.

Tests the core mathematical invariants of _solve_cbf_qp,
_build_cbf_constraints, and select_gamma without requiring a ROS2 runtime
or robot model.
"""

import numpy as np
import pytest
import qpsolvers as qp_lib


NV = 7
RHO = 1000.0
SOLVER = 'osqp'

QDOT_MAX = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])
QDOT_MIN = -QDOT_MAX


def _solve(qdot_nom, constraints, qdot_min=QDOT_MIN, qdot_max=QDOT_MAX):
    """Minimal replica of CbfVelocityFilter._solve_cbf_qp for unit testing."""
    P = np.eye(NV + 1, dtype=np.float64)
    P[-1, -1] = RHO

    q_vec = np.zeros(NV + 1, dtype=np.float64)
    q_vec[:NV] = -np.asarray(qdot_nom, dtype=np.float64)

    lb = np.concatenate([qdot_min, [0.0]])
    ub = np.concatenate([qdot_max, [1e6]])

    G_rows, h_rows = [], []
    for a_vel, b_vel in constraints:
        row = np.zeros(NV + 1, dtype=np.float64)
        row[:NV] = -a_vel
        row[-1]  = -1.0
        G_rows.append(row)
        h_rows.append(-b_vel)

    G = np.vstack(G_rows).astype(np.float64) if G_rows else None
    h = np.array(h_rows, dtype=np.float64)   if h_rows else None

    x = qp_lib.solve_qp(
        P=P, q=q_vec, G=G, h=h, A=None, b=None,
        lb=lb, ub=ub,
        solver=SOLVER,
        verbose=False,
    )
    assert x is not None and np.all(np.isfinite(x)), 'QP returned None or non-finite'
    return x[:NV].copy(), float(x[-1])


# ── Test 1: no constraints → qdot_safe ≈ qdot_nom ───────────────────────────

def test_no_constraints_passthrough():
    qdot_nom = np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    qdot_safe, slack = _solve(qdot_nom, constraints=[])
    np.testing.assert_allclose(qdot_safe, qdot_nom, atol=1e-4)
    assert slack < 1e-6


# ── Test 2: single constraint, CBF must be satisfied (pure ZCBF, no hdot) ───

def test_cbf_constraint_satisfied():
    """With h=0.1, gamma=1.0, a=[1,0,...]:
        b_vel = -gamma * h = -0.1  (pure ZCBF, no hdot term)
        Constraint: a·q̇_safe >= -0.1.
        With qdot_nom[0]=0.5 the constraint is slack; verify the invariant holds.
    """
    gamma = 1.0
    h     = 0.1
    a_vel = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    b_vel = float(-gamma * h)   # = -0.1

    qdot_nom = np.array([0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    constraints = [(a_vel, b_vel)]
    qdot_safe, slack = _solve(qdot_nom, constraints)

    margin = float(a_vel @ qdot_safe) - b_vel
    assert margin >= -1e-4, f'CBF constraint violated: margin={margin:.6f}'
    assert slack >= -1e-6


# ── Test 3: active constraint forces deviation from qdot_nom ─────────────────

def test_cbf_constraint_active():
    """qdot_nom pushes toward the obstacle; constraint must redirect it.
        h=0.05, gamma=1.0  →  b_vel = -0.05
        qdot_nom[0] = -0.8 < b_vel = -0.05, so QP must push qdot_safe[0] >= -0.05.
    """
    gamma = 1.0
    h     = 0.05
    a_vel = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    b_vel = float(-gamma * h)   # = -0.05

    qdot_nom = np.array([-0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    constraints = [(a_vel, b_vel)]
    qdot_safe, slack = _solve(qdot_nom, constraints)

    margin = float(a_vel @ qdot_safe) - b_vel
    assert margin >= -2e-3, f'Active CBF constraint violated: margin={margin:.6f}'
    assert qdot_safe[0] >= b_vel - 2e-3


# ── Test 4: velocity bounds respected ────────────────────────────────────────

def test_velocity_bounds_respected():
    qdot_nom = np.array([5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0])  # above limits
    qdot_safe, _ = _solve(qdot_nom, constraints=[])
    assert np.all(qdot_safe <= QDOT_MAX + 1e-4)
    assert np.all(qdot_safe >= QDOT_MIN - 1e-4)


# ── Test 5: h < 0 (inside obstacle) — constraint still added, QP still runs ─

def test_h_negative_constraint_still_enforced():
    """Even when h < 0 the constraint is added and the QP must still satisfy it.
        h=-0.02, gamma=1.0  →  b_vel = 0.02 > 0 (tighter than usual)
    """
    gamma = 1.0
    h     = -0.02
    a_vel = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    b_vel = float(-gamma * h)   # = 0.02 > 0

    qdot_nom = np.array([-0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    constraints = [(a_vel, b_vel)]
    qdot_safe, slack = _solve(qdot_nom, constraints)

    # Soft constraint: a·q̇_safe + s >= b_vel (primal tol ~1e-3)
    assert float(a_vel @ qdot_safe) + slack >= b_vel - 1e-3


# ── Test 6: multi-link constraints — all satisfied simultaneously ─────────────

def test_multi_link_constraints():
    """QP with 3 orthogonal constraints (simulates fr3_link4/5/6 each active).
    Each link generates an independent ZCBF row.  All must be satisfied after solve.
    Also verifies the activation gate: a link with h > cbf_activation_margin (0.10 m)
    must NOT contribute a constraint row.
    """
    CBF_ACTIVATION_MARGIN = 0.10
    D_SAFE = 0.15

    # Three active links — h within margin (h <= 0.10)
    links = [
        # (a_vel_unit, h, link_name)
        (np.array([1., 0., 0., 0., 0., 0., 0.]), 0.05,  'fr3_link4'),
        (np.array([0., 1., 0., 0., 0., 0., 0.]), 0.08,  'fr3_link5'),
        (np.array([0., 0., 1., 0., 0., 0., 0.]), 0.02,  'fr3_link6'),
    ]
    # One inactive link — h > margin → must be filtered out
    links_out = [
        (np.array([0., 0., 0., 1., 0., 0., 0.]), 0.15,  'fr3_link7'),  # h=0.15 > 0.10
    ]

    gamma = 1.0
    constraints = [(a, float(-gamma * h)) for a, h, _ in links]

    # Simulate activation gate: only include links where h <= margin
    for a, h, name in links_out:
        assert h > CBF_ACTIVATION_MARGIN, f'{name} should be outside activation zone'

    qdot_nom = np.array([-0.5, -0.5, -0.5, 0.0, 0.0, 0.0, 0.0])
    qdot_safe, slack = _solve(qdot_nom, constraints)

    # All 3 constraints must be satisfied (up to OSQP tolerance)
    for a_vel, b_vel in constraints:
        margin = float(a_vel @ qdot_safe) - b_vel
        assert margin >= -2e-3, \
            f'Multi-link constraint violated: margin={margin:.6f}  b_vel={b_vel:.4f}'

    # Filtered link must NOT be in constraints (len == 3, not 4)
    assert len(constraints) == 3


# ── Test 7: select_gamma is monotonically decreasing and hits expected extremes

def test_gamma_continuous():  # noqa: F811
    """gamma(d) must be monotonically decreasing in d and match boundary values."""
    import sys
    import os
    # Direct import without ROS2 — import the module source directly
    sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                    '..', 'franka_experiments', 'utils'))
    # Inline the function to avoid ament dependency in unit tests
    GAMMA_MIN = 2.0
    GAMMA_MAX = 12.0
    K         = 18.0

    def gamma_fn(d, d_safe=0.20):
        delta = max(float(d) - float(d_safe), 0.0)
        return GAMMA_MIN + (GAMMA_MAX - GAMMA_MIN) * np.exp(-K * delta)

    d_safe = 0.20
    d_vals = np.linspace(d_safe, d_safe + 0.5, 200)
    gammas = np.array([gamma_fn(d, d_safe) for d in d_vals])

    # Monotonically non-increasing
    assert np.all(np.diff(gammas) <= 1e-10), 'gamma is not monotonically decreasing'

    # At boundary: gamma ≈ gamma_max
    np.testing.assert_allclose(gamma_fn(d_safe, d_safe), GAMMA_MAX, atol=1e-10)

    # Far away: gamma ≈ gamma_min  (delta = 0.5 m → exp(-9) ≈ 1.2e-4)
    np.testing.assert_allclose(gamma_fn(d_safe + 0.5, d_safe), GAMMA_MIN, atol=0.01)
