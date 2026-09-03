"""Task-space speed rows: no control point may outrun its own clearance.

The joint velocity box bounds ``q̇`` per joint and nothing else. A folded or
near-singular arm decouples joint speed from task speed in BOTH directions, and
the failure this was written for shows the gap plainly: every joint under 0.31
of its velocity limit while link5 and the hand closed to a 2.9 cm gap, with the
QP reporting ``n_act=0`` throughout.

The row bounds ``‖ṗ_i‖`` directly, by the tighter of

* a flat ceiling ``link_speed_max`` — nothing on the robot moves faster, ever;
* ``clearance_i / reaction_s`` — the GEOMETRIC bound. Barriers are rebuilt at
  50 Hz from ~30 Hz perception, so a point travels ``v·Δt`` blind between two
  evaluations. Faster than its own remaining gap over that blind time and it
  crosses the barrier BETWEEN samples: no constraint the QP ever looked at was
  violated, and the links interpenetrate anyway. That is discretisation
  tunnelling, and the tests below pin the property that forbids it.

Pure numpy.
"""

import numpy as np

from franka_experiments.utils.cbf_state_rows import (
    link_speed_cap,
    link_speed_row,
    retreat_cap_rhs,
)

NV = 7
CAP_KW = dict(v_max=0.8, reaction_s=0.20)


def _jp(rng):
    """A stand-in for a 3×nv point Jacobian."""
    return rng.normal(size=(3, NV))


# ── The cap value ────────────────────────────────────────────────────────────

def test_far_from_everything_the_flat_ceiling_rules():
    assert link_speed_cap(2.0, **CAP_KW) == 0.8


def test_close_up_the_geometry_rules():
    """10 cm of room over a 0.20 s blind window is 0.5 m/s, not 0.8."""
    assert np.isclose(link_speed_cap(0.10, **CAP_KW), 0.5)


def test_the_cap_is_the_tighter_of_the_two_everywhere():
    for d in np.linspace(0.0, 1.0, 21):
        assert np.isclose(link_speed_cap(d, **CAP_KW), min(0.8, d / 0.20))


def test_zero_clearance_forbids_motion():
    """Touching means no speed is admissible — the row demands a full stop of
    that point, and the slack (not this function) decides what it costs."""
    assert link_speed_cap(0.0, **CAP_KW) == 0.0


def test_negative_clearance_is_treated_as_touching():
    """A gap can go negative once perception and the capsule model disagree.
    Propagating that sign would turn the cap NEGATIVE and demand motion; it
    must clamp to the touching case instead."""
    assert link_speed_cap(-0.05, **CAP_KW) == 0.0


def test_the_cap_is_exactly_the_no_tunnelling_speed():
    """The property the whole row exists for, stated directly: a point held at
    its cap cannot cover its own clearance within the blind window."""
    for clearance in (0.02, 0.05, 0.10, 0.30):
        v = link_speed_cap(clearance, **CAP_KW)
        assert v * 0.20 <= clearance + 1e-12


# ── The row ──────────────────────────────────────────────────────────────────

def test_no_row_for_a_slow_point():
    """Below activate_frac·v_allow the row is non-binding, and emitting one per
    control point at all times would only churn n_c and OSQP setup()."""
    rng = np.random.default_rng(0)
    Jp = _jp(rng)
    qdot = np.zeros(NV)
    assert link_speed_row(Jp, qdot, 0.8, 0.5, 'spd:x') is None


def test_no_row_for_a_stationary_point_even_at_zero_cap():
    """v̂ is undefined at zero speed. The guard must come before the division,
    or a parked arm produces NaNs in the QP."""
    rng = np.random.default_rng(1)
    assert link_speed_row(_jp(rng), np.zeros(NV), 0.0, 0.5, 'spd:x') is None


def test_row_direction_is_the_point_s_own_travel():
    """‖ṗ‖ ≤ v is a norm bound, not linear. Linearised along the direction the
    point is ACTUALLY moving, the row is exact for the current motion:
    v̂ᵀJp q̇ = ‖ṗ‖."""
    rng = np.random.default_rng(2)
    Jp = _jp(rng)
    qdot = rng.normal(size=NV)
    speed = float(np.linalg.norm(Jp @ qdot))

    a, v_allow, _ = link_speed_row(Jp, qdot, 0.3 * speed, 0.5, 'spd:x')
    # Stored negated (see the module docstring), so −aᵀq̇ is the bounded rate.
    assert np.isclose(-float(a @ qdot), speed)


def test_row_reports_back_the_cap_it_was_given():
    rng = np.random.default_rng(3)
    Jp = _jp(rng)
    qdot = rng.normal(size=NV)
    _, v_allow, label = link_speed_row(Jp, qdot, 0.01, 0.5, 'spd:link5#0')
    assert v_allow == 0.01
    assert label == 'spd:link5#0'


def test_rhs_makes_the_one_step_speed_land_on_the_cap():
    """End to end through the shared RHS builder: a q̈ sitting exactly on the
    row's bound must bring ‖ṗ‖ to v_allow after one horizon."""
    rng = np.random.default_rng(4)
    Jp = _jp(rng)
    qdot = rng.normal(size=NV)
    speed = float(np.linalg.norm(Jp @ qdot))
    v_allow = 0.4 * speed
    T = 0.10

    a, _, _ = link_speed_row(Jp, qdot, v_allow, 0.5, 'spd:x')
    u = retreat_cap_rhs(a[None, :], qdot, np.array([v_allow]), T)[0]

    # G[:, :NV] = -a, so the row is (-a)ᵀq̈ ≤ u; on the bound with s = 0:
    qddot = (-a) * (u / float(a @ a))
    v_hat = (Jp @ qdot) / speed
    assert np.isclose(float(v_hat @ Jp @ (qdot + qddot * T)), v_allow)


def test_the_row_slows_a_point_that_is_over_its_geometric_cap():
    """The failure shape, in one assertion: a point moving fast with very
    little room left. The row must demand DECELERATION along its travel."""
    rng = np.random.default_rng(5)
    Jp = _jp(rng)
    qdot = rng.normal(size=NV)
    speed = float(np.linalg.norm(Jp @ qdot))
    clearance = 0.03                                   # 3 cm, as logged
    v_allow = link_speed_cap(clearance, **CAP_KW)
    assert v_allow < speed, 'the fixture must actually be over the cap'

    a, _, _ = link_speed_row(Jp, qdot, v_allow, 0.5, 'spd:x')
    u = retreat_cap_rhs(a[None, :], qdot, np.array([v_allow]), 0.10)
    # The bound on (-a)ᵀq̈ is negative ⇒ every admissible q̈ has a component
    # AGAINST the current direction of travel.
    assert u[0] < 0.0
