"""Retreat authority: how fast a control point CAN back away, exactly.

Every other row in the filter bounds the separation rate from one side; none of
them says what the joint boxes physically allow along n̂. These four closed
forms do, and everything downstream that decides "can this point outrun the
obstacle at all" (lateral evasion, the latency budget) rests on them being the
EXACT optimum of the LP over the box — so the first tests check them against a
real LP solver, not against a hand-derived number.

Pure numpy + scipy.
"""

import numpy as np
from scipy.optimize import linprog

from franka_experiments.utils.cbf_state_rows import (
    retreat_accel_available,
    retreat_displacement_reachable,
    retreat_speed_available,
    retreat_speed_reachable,
)

NV = 7
# Official FR3, franka_description/robots/fr3/joint_limits.yaml
QD = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])
QDD = np.array([6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0])
BOX = dict(qdot_max=QD, acc_lb=-QDD, acc_ub=QDD)


def _lp_max(a, lb, ub):
    """max aᵀx s.t. lb ≤ x ≤ ub, by an actual LP solve."""
    res = linprog(-a, bounds=list(zip(lb, ub)), method='highs')
    assert res.success, res.message
    return -res.fun


# ── The velocity maximum is the exact LP optimum ─────────────────────────────

def test_identity_jacobian_along_one_axis_is_that_joints_limit():
    """Jp = [I₃ | 0] and n̂ = x̂: only joint 1 moves the point along x, so the
    fastest retreat is exactly its velocity limit."""
    Jp = np.zeros((3, NV)); Jp[:, :3] = np.eye(3)
    a = np.array([1.0, 0.0, 0.0]) @ Jp
    assert retreat_speed_available(a, QD) == QD[0]


def test_identity_jacobian_along_a_diagonal_sums_the_weighted_limits():
    Jp = np.zeros((3, NV)); Jp[:, :3] = np.eye(3)
    n = np.array([1.0, 1.0, 1.0]) / np.sqrt(3.0)
    a = n @ Jp
    assert np.isclose(retreat_speed_available(a, QD), np.sum(np.abs(n) * QD[:3]))


def test_matches_a_real_lp_solve_on_random_rows():
    rng = np.random.default_rng(0)
    for _ in range(50):
        a = rng.normal(size=NV)
        assert np.isclose(retreat_speed_available(a, QD), _lp_max(a, -QD, QD))


def test_asymmetric_box_matches_the_lp_and_can_only_shrink():
    """The position braking curve makes the velocity box one-sided near a
    limit. The closed form must track the LP there too, and a box that is a
    subset of the symmetric one can never buy MORE retreat."""
    rng = np.random.default_rng(1)
    for _ in range(30):
        a = rng.normal(size=NV)
        lo = -QD * rng.uniform(0.0, 1.0, NV)
        hi = QD * rng.uniform(0.0, 1.0, NV)
        v = retreat_speed_available(a, hi, qdot_min=lo)
        assert np.isclose(v, _lp_max(a, lo, hi))
        assert v <= retreat_speed_available(a, QD) + 1e-12


def test_sign_of_the_normal_does_not_matter():
    """Stored as n̂ᵀJ or as −n̂ᵀJ (the retreat-cap convention), same speed."""
    rng = np.random.default_rng(2)
    a = rng.normal(size=NV)
    assert retreat_speed_available(a, QD) == retreat_speed_available(-a, QD)


def test_the_velocity_margin_scales_the_answer_linearly():
    """Pass the box the filter ENFORCES (velocity_box_margin·qdot_max): the
    estimate must follow it, not the datasheet."""
    rng = np.random.default_rng(3)
    a = rng.normal(size=NV)
    assert np.isclose(retreat_speed_available(a, 0.9 * QD),
                      0.9 * retreat_speed_available(a, QD))


def test_a_null_row_has_no_retreat():
    assert retreat_speed_available(np.zeros(NV), QD) == 0.0
    assert retreat_accel_available(np.zeros(NV), -QDD, QDD) == 0.0


# ── Near a singularity the authority collapses ───────────────────────────────

def test_near_singular_jacobian_collapses_the_retreat():
    """A rank-1 Jacobian Jp = u vᵀ can only move the point along u. A normal
    orthogonal to u has a = 0 exactly; one at a small angle ε has a ∝ ε, so
    the available retreat collapses LINEARLY with the singular value — which
    is the amplification bound the singularity row exists for."""
    rng = np.random.default_rng(4)
    u = np.array([0.0, 0.0, 1.0])
    v = rng.normal(size=NV); v /= np.linalg.norm(v)
    Jp = np.outer(u, v)
    n_perp = np.array([1.0, 0.0, 0.0])
    assert retreat_speed_available(n_perp @ Jp, QD) == 0.0
    for eps in (1e-1, 1e-2, 1e-3):
        n = np.array([np.cos(eps), 0.0, np.sin(eps)])
        v_eps = retreat_speed_available(n @ Jp, QD)
        assert np.isclose(v_eps, np.sin(eps) * np.sum(np.abs(v) * QD))
    # And it is continuous in the singular value itself.
    healthy = retreat_speed_available(u @ Jp, QD)
    assert np.isclose(retreat_speed_available(u @ (1e-3 * Jp), QD), 1e-3 * healthy)


# ── The acceleration maximum ─────────────────────────────────────────────────

def test_acceleration_maximum_matches_the_lp():
    rng = np.random.default_rng(5)
    for _ in range(30):
        a = rng.normal(size=NV)
        lb = -QDD * rng.uniform(0.2, 1.0, NV)
        ub = QDD * rng.uniform(0.2, 1.0, NV)
        assert np.isclose(retreat_accel_available(a, lb, ub), _lp_max(a, lb, ub))


# ── Reachable speed over a horizon ───────────────────────────────────────────

def test_nothing_changes_instantly():
    rng = np.random.default_rng(6)
    a = rng.normal(size=NV)
    qdot = rng.normal(size=NV) * 0.3
    assert np.isclose(retreat_speed_reachable(a, qdot, 0.0, **BOX), float(a @ qdot))


def test_from_rest_the_speed_ramps_at_the_box_acceleration_then_saturates():
    rng = np.random.default_rng(7)
    a = rng.normal(size=NV)
    v_avail = retreat_speed_available(a, QD)
    a_avail = retreat_accel_available(a, -QDD, QDD)
    q0 = np.zeros(NV)
    t_sat = v_avail / a_avail
    for t in (0.25 * t_sat, 0.5 * t_sat):
        assert np.isclose(retreat_speed_reachable(a, q0, t, **BOX), a_avail * t)
    for t in (t_sat, 2.0 * t_sat, 100.0):
        assert np.isclose(retreat_speed_reachable(a, q0, t, **BOX), v_avail)


def test_reachable_speed_is_monotone_in_time_and_bounded_by_v_avail():
    rng = np.random.default_rng(8)
    a = rng.normal(size=NV)
    qdot = rng.normal(size=NV)
    ts = np.linspace(0.0, 3.0, 61)
    vs = [retreat_speed_reachable(a, qdot, t, **BOX) for t in ts]
    assert np.all(np.diff(vs) >= -1e-12)
    assert max(vs) <= retreat_speed_available(a, QD) + 1e-12


def test_a_point_already_closing_is_still_closing_after_a_short_horizon():
    """The honest number: starting at aᵀq̇ = −1 m/s with 4 m/s² of authority,
    after 0.1 s the point still moves toward the obstacle at 0.6 m/s. The
    function must report that, not clamp it away."""
    a = np.zeros(NV); a[0] = 1.0                     # 1 m/rad on joint 1
    qdot = np.zeros(NV); qdot[0] = -1.0
    v = retreat_speed_reachable(a, qdot, 0.1, qdot_max=QD,
                                acc_lb=np.full(NV, -4.0), acc_ub=np.full(NV, 4.0))
    assert np.isclose(v, -1.0 + 0.4)


# ── Reachable displacement ───────────────────────────────────────────────────

def test_displacement_is_the_integral_of_the_reachable_speed():
    """Numerical integration of the speed profile must reproduce the closed
    form, across the saturation knee and from a closing start."""
    rng = np.random.default_rng(9)
    for _ in range(10):
        a = rng.normal(size=NV)
        qdot = rng.normal(size=NV) * 0.5
        for T in (0.05, 0.3, 2.0):
            ts = np.linspace(0.0, T, 4001)
            vs = np.array([retreat_speed_reachable(a, qdot, t, **BOX) for t in ts])
            s_num = np.trapz(vs, ts)
            assert np.isclose(retreat_displacement_reachable(a, qdot, T, **BOX),
                              s_num, atol=1e-6)


def test_displacement_at_zero_horizon_is_zero_and_grows_from_rest_as_half_a_t2():
    rng = np.random.default_rng(10)
    a = rng.normal(size=NV)
    assert retreat_displacement_reachable(a, np.zeros(NV), 0.0, **BOX) == 0.0
    a_avail = retreat_accel_available(a, -QDD, QDD)
    t = 0.01
    assert np.isclose(retreat_displacement_reachable(a, np.zeros(NV), t, **BOX),
                      0.5 * a_avail * t * t)


def test_no_acceleration_authority_means_coasting():
    a = np.zeros(NV); a[2] = 1.0
    qdot = np.zeros(NV); qdot[2] = 0.3
    zero = np.zeros(NV)
    assert np.isclose(retreat_displacement_reachable(
        a, qdot, 0.5, qdot_max=QD, acc_lb=zero, acc_ub=zero), 0.15)
