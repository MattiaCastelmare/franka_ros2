"""The SSM closed forms of ISO 10218-2:2025 Annex L (utils/iso_ssm).

These pin the four properties the rest of the ISO layer relies on:

* ``ssm_speed_cap`` is the EXACT inverse of ``protective_separation``. The QP
  row builder and the independent monitor both consume the cap; if it drifted
  from the separation formula, the monitor would trip on speeds the filter had
  deliberately allowed, or worse, not trip on ones it had not.
* inside ``C + Z_d + Z_r`` the cap is exactly 0.0 — the stop region, not a
  small positive speed that rounds to one.
* the monotonicities a reader would assume: slower with a faster human, faster
  with more room and with more braking authority.
* no NaN anywhere on the boundary cases, because a NaN cap propagates into a QP
  row and OSQP does not return from that gracefully.
"""

import math

import numpy as np
import pytest

from franka_experiments.utils.iso_ssm import (
    pfl_speed, protective_separation, ssm_speed_cap, stopping_distance)

KW = dict(t_r=0.10, a_s=1.0, c=0.10, z_d=0.06, z_r=0.01)


# ── stopping_distance ────────────────────────────────────────────────────────

def test_stopping_distance_is_reaction_plus_braking():
    assert stopping_distance(2.0, t_r=0.1, a_s=4.0) == pytest.approx(
        2.0 * 0.1 + 2.0 ** 2 / (2 * 4.0))


def test_a_receding_part_does_not_shrink_the_stopping_distance():
    assert stopping_distance(-3.0, t_r=0.1, a_s=1.0) == 0.0


# ── the inversion ────────────────────────────────────────────────────────────

def test_the_cap_is_the_exact_inverse_of_the_separation_formula():
    rng = np.random.default_rng(20250914)
    for _ in range(400):
        v_h = float(rng.uniform(0.0, 3.0))
        kw = dict(t_r=float(rng.uniform(0.01, 0.5)),
                  a_s=float(rng.uniform(0.2, 10.0)),
                  c=float(rng.uniform(0.0, 0.9)),
                  z_d=float(rng.uniform(0.0, 0.2)),
                  z_r=float(rng.uniform(0.0, 0.05)))
        d = float(rng.uniform(0.0, 3.0))
        v = ssm_speed_cap(d, v_h, v_max=1e6, **kw)
        if v <= 0.0:
            # Stop region: no positive speed satisfies d >= S_p, so S_p at
            # v = 0 must already be at least d.
            assert protective_separation(0.0, v_h, **kw) >= d - 1e-9
            continue
        assert protective_separation(v, v_h, **kw) == pytest.approx(d, abs=1e-9)


def test_the_cap_is_clipped_by_v_max():
    assert ssm_speed_cap(50.0, 0.0, v_max=1.3, **KW) == pytest.approx(1.3)


# ── the stop region ──────────────────────────────────────────────────────────

def test_inside_c_plus_z_the_cap_is_exactly_zero():
    floor = KW['c'] + KW['z_d'] + KW['z_r']
    for d in (0.0, 0.5 * floor, floor - 1e-6, floor):
        assert ssm_speed_cap(d, 0.0, v_max=5.0, **KW) == 0.0
        assert ssm_speed_cap(d, 2.0, v_max=5.0, **KW) == 0.0


def test_just_outside_the_floor_the_cap_is_positive_but_small():
    floor = KW['c'] + KW['z_d'] + KW['z_r']
    v = ssm_speed_cap(floor + 0.01, 0.0, v_max=5.0, **KW)
    assert 0.0 < v < 0.15


# ── monotonicity ─────────────────────────────────────────────────────────────

def test_the_cap_decreases_with_the_human_approach_speed():
    caps = [ssm_speed_cap(1.0, v_h, v_max=5.0, **KW)
            for v_h in (0.0, 0.5, 1.0, 1.6, 2.0, 3.0)]
    assert all(a > b for a, b in zip(caps, caps[1:]))


def test_the_cap_increases_with_distance():
    caps = [ssm_speed_cap(d, 2.0, v_max=5.0, **KW)
            for d in (0.2, 0.5, 1.0, 2.0, 4.0)]
    assert all(a < b for a, b in zip(caps, caps[1:]))


def test_the_cap_increases_with_braking_authority():
    kw = {k: v for k, v in KW.items() if k != 'a_s'}
    caps = [ssm_speed_cap(1.0, 2.0, a_s=a, v_max=5.0, **kw)
            for a in (0.5, 1.0, 2.0, 5.0, 10.0)]
    assert all(a < b for a, b in zip(caps, caps[1:]))


def test_separation_grows_with_robot_speed():
    sp = [protective_separation(v, 2.0, **KW) for v in (0.0, 0.1, 0.5, 1.0, 2.0)]
    assert all(a < b for a, b in zip(sp, sp[1:]))


def test_a_receding_obstacle_does_not_shrink_the_separation():
    assert (protective_separation(1.0, -5.0, **KW)
            == pytest.approx(protective_separation(1.0, 0.0, **KW)))


# ── numerical hygiene ────────────────────────────────────────────────────────

def test_no_nan_on_the_boundary_cases():
    cases = [
        dict(d=0.0, v_h=0.0),
        dict(d=0.0, v_h=2.0),
        dict(d=1e9, v_h=0.0),
        dict(d=1e9, v_h=2.0),
    ]
    for case in cases:
        for a_s in (1e-9, 1e-3, 1.0, 1e6):
            kw = {k: v for k, v in KW.items() if k != 'a_s'}
            v = ssm_speed_cap(case['d'], case['v_h'], a_s=a_s, v_max=5.0, **kw)
            assert math.isfinite(v) and v >= 0.0
            assert math.isfinite(protective_separation(v, case['v_h'],
                                                       a_s=a_s, **kw))


def test_a_zero_deceleration_does_not_divide_by_zero():
    assert math.isfinite(stopping_distance(1.0, t_r=0.1, a_s=0.0))
    assert math.isfinite(protective_separation(1.0, 1.0, t_r=0.1, a_s=0.0,
                                               c=0.1, z_d=0.0, z_r=0.0))


# ── pfl_speed ────────────────────────────────────────────────────────────────

def test_pfl_speed_reproduces_the_worked_example_in_the_docstring():
    # Annex M hands-and-fingers, quasi-static: F=140 N, k=75 N/mm, m_H=0.6 kg,
    # FR3 M=17.8 kg with no payload -> m_R = 8.9 kg.  v_PFL ~ 0.68 m/s.
    assert pfl_speed(140.0, 75000.0, 8.9, 0.6) == pytest.approx(0.682, abs=5e-4)


def test_a_payload_lowers_the_pfl_speed():
    bare = pfl_speed(140.0, 75000.0, 8.9, 0.6)
    loaded = pfl_speed(140.0, 75000.0, 8.9 + 3.0, 0.6)
    assert loaded < bare


def test_a_stiffer_body_region_lowers_the_pfl_speed():
    assert pfl_speed(140.0, 150000.0, 8.9, 0.6) < pfl_speed(140.0, 75000.0, 8.9, 0.6)


def test_the_reduced_mass_is_dominated_by_the_lighter_body():
    # m_H = 0.6 kg against a 8.9 kg arm: mu is within 10 % of m_H, so doubling
    # the ARM mass barely moves v_PFL — which is why the annex's M/2
    # simplification survives at all.
    a = pfl_speed(140.0, 75000.0, 8.9, 0.6)
    b = pfl_speed(140.0, 75000.0, 17.8, 0.6)
    assert abs(a - b) / a < 0.05
