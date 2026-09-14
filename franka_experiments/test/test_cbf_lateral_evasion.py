"""Lateral evasion: when braking cannot work, and which way to step aside.

The claim being tested is a physical one, so the tests are about the physics
rather than about numbers pulled from a run:

* the trigger is the ROBOT'S OWN acceleration box, not a tuned threshold — a
  closing rate the box can null produces no evasion however fast the obstacle
  is, and one it cannot produces evasion however slow;
* the escape direction maximises the MISS DISTANCE, which is perpendicular to
  the obstacle's velocity — and is NOT n̂, a distinction that only exists
  because the estimate is now a 3D vector;
* it is a BIAS, so no barrier row and no bound may move when it engages.

Pure numpy for the formulas, plus the real ConstraintBuilder for the wiring.
"""

import numpy as np

from franka_experiments.utils.cbf_evasion import (
    escape_direction, evasion_bias, evasion_urgency, normal_brake_authority)
from _cbf_builder_harness import NV, make_builder, make_obstacle, run

ACC = np.array([6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0])
PR, PH = (0.5, 0.0, 0.5), (0.5, -0.25, 0.5)      # n̂ = +y


# ── Braking authority ───────────────────────────────────────────────────────

def test_authority_is_the_exact_maximum_of_the_linear_form_over_the_box():
    """Separable and exact — no bound, no optimisation. Checked against a brute
    force over the box vertices, which is where the maximum of a linear form
    lives."""
    rng = np.random.default_rng(0)
    for _ in range(50):
        a = rng.normal(size=NV)
        got = normal_brake_authority(a, -ACC, ACC)
        best = max(float(a @ np.where(m, ACC, -ACC))
                   for m in np.ndindex(*(2,) * NV))
        assert np.isclose(got, best)


def test_authority_is_never_negative_and_zero_for_a_null_row():
    assert normal_brake_authority(np.zeros(NV), -ACC, ACC) == 0.0
    assert normal_brake_authority(np.ones(NV), -ACC, ACC) > 0.0


def test_eta_scales_the_authority_linearly():
    a = np.ones(NV)
    assert np.isclose(normal_brake_authority(a, -ACC, ACC, eta=0.5),
                      0.5 * normal_brake_authority(a, -ACC, ACC))


def test_a_weaker_box_gives_less_authority():
    """Configuration matters: near a singularity the same joint box buys a
    fraction of the Cartesian authority. A constant "max deceleration" would be
    blind to exactly the case that matters."""
    a = np.ones(NV)
    assert normal_brake_authority(a, -0.1 * ACC, 0.1 * ACC) < \
           normal_brake_authority(a, -ACC, ACC)


# ── Urgency ─────────────────────────────────────────────────────────────────

def test_a_separating_obstacle_never_triggers_evasion():
    for h_dot in (0.0, 0.5, 5.0):
        assert evasion_urgency(0.05, h_dot, 2.0) == 0.0


def test_ample_braking_distance_gives_zero_urgency():
    """h = 1 m, closing at 0.1 m/s, 2 m/s² available: stopping distance 2.5 mm."""
    assert evasion_urgency(1.0, -0.1, 2.0) == 0.0


def test_urgency_saturates_when_braking_demonstrably_cannot_succeed():
    """h = 5 cm, closing at 1.5 m/s, 2 m/s²: needs 56 cm to stop."""
    assert evasion_urgency(0.05, -1.5, 2.0) == 1.0


def test_no_leverage_at_all_is_maximally_urgent():
    """Braking is not merely insufficient, it is unavailable — and that is
    exactly when stepping aside is the only option left."""
    assert evasion_urgency(0.05, -0.3, 0.0) == 1.0


def test_urgency_is_monotone_and_continuous_in_the_closing_rate():
    """A hard switch on a 30 Hz vision quantity would chatter, and what is being
    switched is a bias on q̈_nom — the arm would jerk sideways and back at the
    noise rate. That is the failure the tangential bias was rewritten for."""
    w = np.array([evasion_urgency(0.10, -v, 2.0)
                  for v in np.linspace(0.0, 2.0, 401)])
    assert np.all(np.diff(w) >= -1e-12), 'urgency must be monotone'
    assert w[0] == 0.0 and w[-1] == 1.0
    # A switch would put ZERO samples strictly between 0 and 1 and a single
    # step of 1.0. The ramp is quadratic in the closing rate (r ~ hdot^2), so
    # it is narrow in v by construction -- the property that matters is that it
    # is a ramp at all, spread over many samples.
    interior = int(((w > 0.0) & (w < 1.0)).sum())
    assert interior >= 10, f'only {interior} samples inside the ramp'
    assert w.max(initial=0.0) == 1.0 and float(np.abs(np.diff(w)).max()) < 0.15


def test_urgency_is_monotone_in_the_remaining_gap():
    w = [evasion_urgency(h, -0.8, 2.0) for h in np.linspace(0.01, 1.0, 200)]
    assert np.all(np.diff(w) <= 1e-12)


def test_more_braking_authority_lowers_the_urgency():
    assert evasion_urgency(0.10, -0.8, 8.0) < evasion_urgency(0.10, -0.8, 2.0)


def test_a_zero_gap_does_not_divide_by_zero():
    assert np.isfinite(evasion_urgency(0.0, -0.5, 2.0))
    assert evasion_urgency(-0.2, -0.5, 2.0) == 1.0


# ── Escape direction ────────────────────────────────────────────────────────

def test_the_escape_is_perpendicular_to_the_obstacle_velocity():
    """That is what maximises the miss distance: moving ALONG the obstacle's
    travel changes the closest approach not at all."""
    d = np.array([0.0, 0.3, 0.1])
    v = np.array([1.0, 0.0, 0.0])
    e = escape_direction(d, v)
    assert abs(float(e @ v)) < 1e-12
    assert np.isclose(np.linalg.norm(e), 1.0)


def test_the_escape_points_AWAY_from_the_obstacle_s_path():
    """Sign test. Moving along it must INCREASE the miss distance, so it has to
    have a positive component on the existing perpendicular offset."""
    v = np.array([1.0, 0.0, 0.0])
    for off in ([0.0, 0.3, 0.0], [0.0, -0.3, 0.0], [0.0, 0.1, -0.25]):
        d = np.array([0.7, 0.0, 0.0]) + np.array(off)
        assert float(escape_direction(d, v) @ np.array(off)) > 0.0


def test_the_escape_is_NOT_the_barrier_normal_on_a_glancing_approach():
    """The distinction the whole feature rests on. For an obstacle passing by,
    n̂ and the escape are nearly orthogonal — backing off along n̂ barely changes
    the miss distance, which is why the existing rows cannot express this."""
    d = np.array([0.9, 0.05, 0.0])          # mostly along the travel axis
    v = np.array([-1.0, 0.0, 0.0])          # obstacle coming straight down it
    n_hat = d / np.linalg.norm(d)
    e = escape_direction(d, v)
    assert abs(float(e @ n_hat)) < 0.2, 'escape collapsed onto the normal'


def test_a_slow_obstacle_yields_no_escape_direction():
    """Below v_min "perpendicular to its velocity" is a direction fitted to
    noise, and the normal retreat the barrier already demands is the whole
    response."""
    assert escape_direction(np.array([0.0, 0.3, 0.0]),
                            np.array([0.0, 0.01, 0.0]), v_min=0.15) is None


def test_a_head_on_approach_still_produces_a_direction():
    """d_perp ≈ 0: every perpendicular is geometrically equivalent, so the tie
    goes to the direction the arm can actually accelerate along. It must still
    return SOMETHING — returning None here would leave the most dangerous
    geometry with no evasion at all."""
    d = np.array([0.0, 0.4, 0.0])
    v = np.array([0.0, -1.0, 0.0])          # straight at the control point
    Jp = np.vstack([np.linspace(0.4, 0.1, NV),
                    np.linspace(0.05, 0.02, NV),
                    np.linspace(0.02, 0.3, NV)])
    e = escape_direction(d, v, Jp)
    assert e is not None
    assert abs(float(e @ v)) < 1e-9 and np.isclose(np.linalg.norm(e), 1.0)


def test_the_head_on_tie_break_picks_the_direction_the_arm_can_move():
    """The x row of this Jacobian has ~10x the leverage of the z row, so the
    escape must come out along x."""
    d = np.array([0.0, 0.4, 0.0])
    v = np.array([0.0, -1.0, 0.0])
    Jp = np.zeros((3, NV))
    Jp[0] = 0.5
    Jp[2] = 0.05
    e = escape_direction(d, v, Jp)
    assert abs(e[0]) > 0.9, e


def test_the_head_on_direction_is_deterministic_across_calls():
    """An unresolved eigenvector sign would flip frame to frame and the arm
    would shake instead of moving."""
    d = np.array([0.0, 0.4, 0.0])
    v = np.array([0.0, -1.0, 0.0])
    Jp = np.vstack([np.linspace(0.4, 0.1, NV), np.zeros(NV),
                    np.linspace(0.02, 0.3, NV)])
    e = [escape_direction(d, v, Jp) for _ in range(20)]
    for x in e[1:]:
        np.testing.assert_array_equal(x, e[0])


def test_garbage_input_returns_none_rather_than_a_direction():
    for d, v in ((np.array([np.nan, 0, 0]), np.array([1.0, 0, 0])),
                 (np.array([0.1, 0, 0]), np.array([np.inf, 0, 0])),
                 (np.zeros(2), np.array([1.0, 0, 0]))):
        assert escape_direction(d, v) is None


# ── The bias ────────────────────────────────────────────────────────────────

def test_no_engaged_row_gives_an_exact_zero():
    assert evasion_bias([], gain=1.5, max_bias=3.0).size == 0
    b = evasion_bias([(np.ones(NV), 0.0)], gain=1.5, max_bias=3.0)
    np.testing.assert_array_equal(b, np.zeros(NV))


def test_the_bias_points_along_the_escape_row():
    g = np.array([1.0, 0, 0, 0, 0, 0, 0.0])
    b = evasion_bias([(g, 1.0)], gain=1.5, max_bias=3.0)
    assert np.allclose(b, 1.5 * g)


def test_the_bias_scales_with_urgency():
    g = np.ones(NV)
    lo = evasion_bias([(g, 0.25)], gain=1.5, max_bias=99.0)
    hi = evasion_bias([(g, 0.75)], gain=1.5, max_bias=99.0)
    assert np.isclose(np.linalg.norm(hi), 3.0 * np.linalg.norm(lo))


def test_the_cap_bounds_the_sum_over_control_points():
    """Several control points on one link face one obstacle. Without the cap the
    swerve would scale with how finely the arm happens to be discretised — a
    modelling detail that must not change how hard the robot moves."""
    rows = [(np.ones(NV), 1.0)] * 8
    assert np.linalg.norm(evasion_bias(rows, gain=1.5, max_bias=3.0)) <= 3.0 + 1e-12


def test_a_row_with_no_leverage_is_dropped_not_normalised():
    """The escape exists in Cartesian space but the arm cannot move along it
    from this pose. Normalising a near-null vector turns rounding error into a
    full-strength command."""
    b = evasion_bias([(np.full(NV, 1e-9), 1.0)], gain=1.5, max_bias=3.0)
    np.testing.assert_array_equal(b, np.zeros(NV))


# ── Wiring into the real snapshot ───────────────────────────────────────────

def _con(enable, *, v, d=0.20, qdot=0.0, **over):
    b = make_builder(obstacle_velocity_source='tracker',
                     enable_lateral_evasion=enable, **over)
    return run(b, [make_obstacle(d=d, pr=PR, ph=PH, v=v, frames_seen=20)],
               n_frames=5, qdot=qdot)


FAST = (0.9, 1.4, 0.0)      # closing hard along +y, travelling mostly in x


def test_flag_off_produces_no_bias_and_bit_identical_rows():
    off = _con(False, v=FAST)
    assert off.esc_bias is None and off.esc_w_max == 0.0
    on = _con(True, v=FAST)
    np.testing.assert_array_equal(on.A, off.A)
    np.testing.assert_array_equal(on.h_bar, off.h_bar)
    np.testing.assert_array_equal(on.G, off.G)
    np.testing.assert_array_equal(on.jdot_qdot, off.jdot_qdot)


def test_a_fast_unbrakeable_obstacle_engages_the_evasion():
    con = _con(True, v=FAST, d=0.16)
    assert con.esc_w_max > 0.0
    assert con.esc_bias is not None and np.linalg.norm(con.esc_bias) > 0.0


def test_a_slow_obstacle_the_robot_can_brake_for_does_not():
    con = _con(True, v=(0.0, 0.05, 0.0), d=0.9)
    assert con.esc_w_max == 0.0 and con.esc_bias is None


def test_a_receding_obstacle_never_engages_however_fast():
    con = _con(True, v=(2.0, -2.0, 0.0), d=0.16)
    assert con.esc_w_max == 0.0 and con.esc_bias is None


def test_a_track_less_obstacle_never_engages():
    """No velocity means no direction. The feature is inert without the tracker,
    which is why it is safe to ship the flag alongside 'residual' mode."""
    b = make_builder(obstacle_velocity_source='residual',
                     enable_lateral_evasion=True)
    con = run(b, [make_obstacle(d=0.16, pr=PR, ph=PH)], n_frames=5)
    assert con.esc_bias is None


def test_the_bias_is_capped_as_configured():
    con = _con(True, v=(2.0, 2.0, 0.0), d=0.05, lateral_evasion_max_bias=1.0)
    assert np.linalg.norm(con.esc_bias) <= 1.0 + 1e-12


def test_lower_authority_engages_the_evasion_earlier():
    """The trigger is the robot's limits, so crediting it less of its own box
    must make it give up on braking sooner."""
    strong = _con(True, v=(0.6, 0.9, 0.0), d=0.30, lateral_evasion_authority=1.0)
    weak = _con(True, v=(0.6, 0.9, 0.0), d=0.30, lateral_evasion_authority=0.05)
    assert weak.esc_w_max >= strong.esc_w_max
