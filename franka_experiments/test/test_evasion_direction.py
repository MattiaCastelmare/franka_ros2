"""Escape direction: retreat while the obstacle can be outrun, step aside when
it cannot, and rotate smoothly between the two.

Pure numpy. The directions are checked on an analytic Jacobian where the
achievable speeds can be computed by brute force, and the bias on the builder
through the real ConstraintBuilder (see test_cbf_outrun_evasion for the
bit-identity half).
"""

import numpy as np
import pytest

from franka_experiments.utils.cbf_state_rows import retreat_speed_available
from franka_experiments.utils.evasion_direction import (
    blend_weight, escape_direction, evasion_bias, lateral_direction, outrun_ratio)

NV = 7
QD = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])
KW = dict(margin=0.8, ramp_start=0.6, v_min=0.15)


def _jp(seed=0):
    rng = np.random.default_rng(seed)
    return 0.3 * rng.normal(size=(3, NV))


# ── Ratio and ramp ───────────────────────────────────────────────────────────

def test_ratio_is_zero_for_a_receding_or_static_obstacle():
    assert outrun_ratio(0.0, 1.0, margin=0.8) == 0.0
    assert outrun_ratio(-2.0, 1.0, margin=0.8) == 0.0


def test_ratio_is_infinite_with_no_authority():
    assert outrun_ratio(0.1, 0.0, margin=0.8) == float('inf')


def test_ratio_is_one_exactly_at_the_margin():
    assert np.isclose(outrun_ratio(0.8, 1.0, margin=0.8), 1.0)


def test_blend_is_a_continuous_ramp_from_ramp_start_to_one():
    rs = np.linspace(0.0, 1.5, 3001)
    ws = np.array([blend_weight(r, ramp_start=0.6) for r in rs])
    assert ws[rs <= 0.6].max() == 0.0
    assert ws[rs >= 1.0].min() == 1.0
    assert np.all(np.diff(ws) >= 0.0)
    assert np.abs(np.diff(ws)).max() < 0.01          # no step anywhere
    assert blend_weight(float('inf'), ramp_start=0.6) == 1.0


# ── The lateral direction ────────────────────────────────────────────────────

def test_lateral_direction_is_orthogonal_to_the_obstacle_velocity():
    Jp = _jp()
    v = np.array([0.4, -0.9, 0.2])
    e = lateral_direction(v, Jp, QD, d_vec=np.array([0.1, 0.3, 0.05]))
    assert e is not None
    assert np.isclose(np.linalg.norm(e), 1.0)
    assert abs(float(e @ v)) < 1e-9


def test_lateral_direction_is_the_fastest_achievable_one_in_the_plane():
    """Brute force over a fine circle ⟂ v̂: nothing in the plane beats it by
    more than the sampling resolution."""
    Jp = _jp(1)
    v = np.array([1.0, 0.2, -0.3])
    e = lateral_direction(v, Jp, QD, d_vec=np.array([0.0, 0.2, 0.1]))
    f_e = retreat_speed_available(e @ Jp, QD)
    v_hat = v / np.linalg.norm(v)
    u1 = np.cross(v_hat, [0, 0, 1.0]); u1 /= np.linalg.norm(u1)
    u2 = np.cross(v_hat, u1)
    best = max(retreat_speed_available((np.cos(t) * u1 + np.sin(t) * u2) @ Jp, QD)
               for t in np.linspace(0, np.pi, 3600, endpoint=False))
    assert f_e >= 0.995 * best


def test_lateral_direction_points_away_from_the_obstacle_path():
    """The point is already on one side of the obstacle's line of travel; the
    escape must not send it across."""
    Jp = _jp(2)
    v = np.array([0.0, -1.0, 0.0])
    d = np.array([0.3, 0.5, 0.0])          # robot is at +x of the path
    e = lateral_direction(v, Jp, QD, d_vec=d)
    d_perp = d - float(d @ (v / np.linalg.norm(v))) * (v / np.linalg.norm(v))
    assert float(e @ d_perp) > 0.0


def test_lateral_direction_is_deterministic_head_on():
    Jp = _jp(3)
    v = np.array([0.0, -1.0, 0.0])
    d = np.array([0.0, 0.5, 0.0])          # exactly on the path
    e1 = lateral_direction(v, Jp, QD, d_vec=d)
    e2 = lateral_direction(v, Jp, QD, d_vec=d)
    np.testing.assert_array_equal(e1, e2)


def test_a_slow_obstacle_has_no_lateral_direction():
    assert lateral_direction(np.array([0.0, 0.05, 0.0]), _jp(), QD, d_vec=np.ones(3)) is None


def test_no_leverage_in_the_plane_gives_none():
    """A rank-1 Jacobian along v̂: nothing in the plane ⟂ v̂ moves the point."""
    v = np.array([0.0, 0.0, 1.0])
    Jp = np.outer(v, np.ones(NV))
    assert lateral_direction(v, Jp, QD, d_vec=np.array([0.1, 0.0, 0.5])) is None


# ── The escape direction ─────────────────────────────────────────────────────

def test_slow_approach_escapes_along_the_normal():
    """Outrunnable: escape == +n̂ (the normal retreat) to machine precision,
    w == 0, and the ratio is reported."""
    Jp = _jp(4)
    n = np.array([0.0, 1.0, 0.0])
    v = 0.2 * n                                    # closing at 0.2 m/s along n̂
    e, w, r = escape_direction(n, v, Jp, QD, d_vec=0.3 * n, **KW)
    np.testing.assert_allclose(e, n, atol=1e-12)
    assert w == 0.0
    assert 0.0 < r < 0.6


def test_approach_at_three_times_v_avail_escapes_orthogonally_to_v_obs():
    Jp = _jp(5)
    n = np.array([0.0, 1.0, 0.0])
    v_avail = retreat_speed_available(n @ Jp, QD)
    v = 3.0 * v_avail * n
    e, w, r = escape_direction(n, v, Jp, QD, d_vec=0.3 * n + np.array([0.02, 0, 0]), **KW)
    assert w == 1.0
    assert r > 1.0
    assert abs(float(e @ v)) < 1e-9
    assert np.isclose(np.linalg.norm(e), 1.0)


def test_the_escape_direction_is_continuous_across_the_threshold():
    """Sweep the closing speed through the whole ramp: no jump in ê anywhere,
    and it ends up orthogonal to v_obs."""
    Jp = _jp(6)
    n = np.array([0.0, 1.0, 0.0])
    v_avail = retreat_speed_available(n @ Jp, QD)
    d = 0.3 * n + np.array([0.02, 0.0, 0.01])
    prev = None
    for s in np.linspace(0.2, 1.5, 1301):
        e, w, r = escape_direction(n, s * v_avail * n, Jp, QD, d_vec=d, **KW)
        if prev is not None:
            assert np.linalg.norm(e - prev) < 0.02, (s, np.linalg.norm(e - prev))
        prev = e
    assert abs(float(prev @ n)) < 1e-9


def test_zero_velocity_escapes_along_the_normal_with_zero_weight():
    n = np.array([1.0, 0.0, 0.0])
    e, w, r = escape_direction(n, np.zeros(3), _jp(), QD, d_vec=0.2 * n, **KW)
    np.testing.assert_array_equal(e, n)
    assert w == 0.0 and r == 0.0


# ── The bias ─────────────────────────────────────────────────────────────────

def test_zero_closing_speed_gives_exact_zeros():
    b = evasion_bias(np.array([1.0, 0, 0]), _jp(), 0.0, 1.0, gain=1.0, accel=1.0,
                     v_ref=0.5, max_bias=3.0)
    assert b.shape == (NV,) and not b.any()
    b = evasion_bias(np.array([1.0, 0, 0]), _jp(), -0.5, 1.0, gain=1.0, accel=1.0,
                     v_ref=0.5, max_bias=3.0)
    assert not b.any()


def test_outrunnable_gives_exact_zeros():
    b = evasion_bias(np.array([1.0, 0, 0]), _jp(), 0.8, 0.0, gain=1.0, accel=1.0,
                     v_ref=0.5, max_bias=3.0)
    assert not b.any()


def test_bias_pushes_the_point_along_the_escape_direction():
    Jp = _jp(7)
    e = np.array([0.0, 0.0, 1.0])
    b = evasion_bias(e, Jp, 1.0, 1.0, gain=1.0, accel=1.0, v_ref=0.5, max_bias=100.0)
    acc = Jp @ b
    assert float(acc @ e) > 0.9 * np.linalg.norm(acc)


def test_bias_is_norm_capped_and_scales_with_the_ramps():
    Jp = _jp(8)
    e = np.array([0.0, 0.0, 1.0])
    big = evasion_bias(e, Jp, 5.0, 1.0, gain=10.0, accel=10.0, v_ref=0.5, max_bias=2.0)
    assert np.isclose(np.linalg.norm(big), 2.0)
    half_v = evasion_bias(e, Jp, 0.25, 1.0, gain=1.0, accel=1.0, v_ref=0.5, max_bias=100.0)
    full_v = evasion_bias(e, Jp, 0.5, 1.0, gain=1.0, accel=1.0, v_ref=0.5, max_bias=100.0)
    np.testing.assert_allclose(2.0 * half_v, full_v)
    half_w = evasion_bias(e, Jp, 0.5, 0.5, gain=1.0, accel=1.0, v_ref=0.5, max_bias=100.0)
    np.testing.assert_allclose(2.0 * half_w, full_v)
