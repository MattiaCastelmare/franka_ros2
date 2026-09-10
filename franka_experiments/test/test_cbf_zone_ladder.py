"""The gap ladder: four rungs, no steps, and a hold that is not a freeze.

Three separable claims, tested separately:

* the LADDER itself (utils.cbf_zones) is a smooth partition of the gap axis
  whose blended gains never leave the convex hull of the configured rungs;
* the BUILDER schedules only obstacle rows and only their gains and slack
  column — A, h_bar and jdot_qdot are byte-for-byte what they were, whatever
  rung the row is on, because the ladder changes how hard the barrier pushes
  and never where the barrier is;
* with the flag OFF the snapshot carries no per-row gains at all, so
  build_row_rhs evaluates the same scalar expression it always did. That is
  the regression the whole design hangs on: the ladder must be provably inert
  until it is switched on.
"""

import numpy as np
import pytest

from _cbf_builder_harness import make_builder, make_obstacle, run
from franka_experiments.utils.cbf_qp_assembly import build_row_rhs
from franka_experiments.utils.cbf_zones import (
    ZoneLadder, ZONE_NAMES, ZONE_HOLD, ZONE_NOTICE, ladder_from_params,
    smoothstep)

NV = 7


def _ladder(**kw):
    return ZoneLadder(**kw)


# ── The ladder in isolation ──────────────────────────────────────────────────

def test_the_weights_are_a_partition_of_unity_at_every_gap():
    z = _ladder()
    for d in np.linspace(-0.05, 0.60, 1301):
        w = z.weights(float(d))
        assert w.min() >= 0.0
        # Exactly 1.0, not approximately: the weights are built by differencing
        # cumulative ramps precisely so this holds in floating point. A sum
        # that drifts is a blended gain that is not a convex combination of the
        # rungs, i.e. a gain nobody configured.
        assert w.sum() == pytest.approx(1.0, abs=1e-12)


def test_the_blended_gain_never_leaves_the_configured_range():
    z = _ladder()
    lo0, hi0 = float(z.k0.min()), float(z.k0.max())
    lo1, hi1 = float(z.k1.min()), float(z.k1.max())
    for d in np.linspace(-0.05, 0.60, 1301):
        g = z.gains(float(d))
        assert lo0 - 1e-12 <= g.k0 <= hi0 + 1e-12
        assert lo1 - 1e-12 <= g.k1 <= hi1 + 1e-12
        assert 0.0 < g.slack_m <= 1.0


def test_the_gains_fall_monotonically_as_the_gap_opens():
    """Closer is never gentler. A non-monotone schedule would mean some gap has
    a softer response than a gap further out, which is not a ladder."""
    z = _ladder()
    ds = np.linspace(0.0, 0.50, 2001)
    k0 = np.array([z.gains(float(d)).k0 for d in ds])
    k1 = np.array([z.gains(float(d)).k1 for d in ds])
    m = np.array([z.gains(float(d)).slack_m for d in ds])
    assert np.all(np.diff(k0) <= 1e-12)
    assert np.all(np.diff(k1) <= 1e-12)
    assert np.all(np.diff(m) >= -1e-12)


def test_the_schedule_has_no_steps():
    """The anti-jerk property, stated as a number.

    A hard switch at the active/priority boundary would put a step of
    (50-25)*h_bar into the row's bound; the measured correlation between the
    commanded radial acceleration and -h_qp is 0.9999, so that step lands in
    the joints. Over a 1 mm change in gap the gain must move by far less than
    the rung difference.
    """
    z = _ladder()
    ds = np.arange(0.0, 0.50, 0.001)
    k0 = np.array([z.gains(float(d)).k0 for d in ds])
    step = float(np.max(np.abs(np.diff(k0))))
    # 25 units of k0 spread over a 40 mm band, with a smoothstep whose peak
    # slope is 1.5x the mean: ~0.94 per mm. Anything near the 25-unit rung
    # difference would mean a switch survived somewhere.
    assert step < 1.5
    assert step < 0.1 * (float(z.k0.max()) - float(z.k0.min()))


def test_each_rung_is_flat_across_its_middle():
    """"20 to 10 behaves exactly as today" has to be literally true, not
    approximately: the active rung carries k0_cbf/k1_cbf themselves, and with
    a 40 mm blend band it carries them unblended from 0.18 down to 0.12."""
    z = _ladder()
    for d in (0.18, 0.15, 0.12):
        g = z.gains(d)
        assert g.k0 == pytest.approx(25.0)
        assert g.k1 == pytest.approx(10.5)
        assert g.slack_m == pytest.approx(1.0)


def test_a_missing_gap_reads_as_far_away_and_never_as_the_hold_rung():
    """A NaN must not stop the robot. Staleness is handled by the node's own
    guards; this module must not become a second, quieter one."""
    z = _ladder()
    w = z.weights(float('nan'))
    assert w[ZONE_NOTICE] == 1.0
    assert w[ZONE_HOLD] == 0.0


def test_boundaries_that_do_not_descend_are_rejected_at_construction():
    with pytest.raises(ValueError):
        _ladder(d_active=0.35)          # active above notice
    with pytest.raises(ValueError):
        _ladder(d_hold=0.10, d_priority=0.10)   # not strict


def test_an_overwide_blend_band_is_clamped_and_says_so():
    """Overlapping bands would break the partition of unity, so the constructor
    clamps rather than trusting the caller."""
    z = _ladder(blend_m=0.5)
    assert z.blend_clamped
    assert z.blend_m == pytest.approx(0.05)     # narrowest boundary gap
    for d in np.linspace(0.0, 0.4, 401):
        assert z.weights(float(d)).sum() == pytest.approx(1.0, abs=1e-12)


def test_smoothstep_is_flat_at_both_ends():
    """The reason it is not a linear ramp: a slope jump in a gain that
    multiplies h_bar is a jerk in the command."""
    eps = 1e-6
    assert smoothstep(0.0, 0.0, 1.0) == 0.0
    assert smoothstep(1.0, 0.0, 1.0) == 1.0
    assert smoothstep(eps, 0.0, 1.0) < eps          # slope -> 0 at the bottom
    assert 1.0 - smoothstep(1.0 - eps, 0.0, 1.0) < eps
    assert smoothstep(0.5, 0.0, 1.0) == pytest.approx(0.5)


# ── The task switch ──────────────────────────────────────────────────────────

def test_the_task_is_switched_off_only_inside_the_hold_rung():
    z = _ladder()
    assert z.task_weight(0.30, 0.01) == pytest.approx(1.0)
    assert z.task_weight(0.15, 0.01) == pytest.approx(1.0)
    z.reset_task()
    assert z.task_weight(0.08, 0.01) == pytest.approx(0.5)   # priority cut
    z.reset_task()
    assert z.task_weight(0.03, 0.01) == pytest.approx(0.0)   # hold


def test_the_task_falls_instantly_and_climbs_over_resume_s():
    """Asymmetric on purpose. Tightening is a decision about this frame's
    measurement; relaxing is a claim that the danger has passed, and a 5 cm
    boundary read by a sensor with 4 cm of calibration spread crosses itself on
    noise. Rate limiting only the climb turns that chatter into one decision."""
    z = _ladder(resume_s=0.5)
    assert z.task_weight(0.03, 0.01) == pytest.approx(0.0)   # one tick, no ramp
    seen = [z.task_weight(0.40, 0.01) for _ in range(60)]
    assert seen[0] == pytest.approx(0.02)                    # 0.01 / 0.5
    assert all(b >= a for a, b in zip(seen, seen[1:]))       # monotone climb
    assert seen[-1] == pytest.approx(1.0)
    # and it took the configured time, not one tick
    assert seen[10] < 0.5


def test_a_flicker_across_the_hold_boundary_does_not_flicker_the_task():
    """The failure this prevents: the measured depth dropout took a control
    point 0.242 -> 0.077 -> 0.253 m in three frames. Without the asymmetry the
    task would come back on the very next tick and the arm would be commanded
    a full trajectory step out of a single bad frame."""
    z = _ladder(resume_s=0.5)
    z.task_weight(0.24, 0.01)
    z.task_weight(0.03, 0.01)               # one bad frame
    back = z.task_weight(0.25, 0.01)        # measurement recovers immediately
    assert back == pytest.approx(0.02)      # still essentially off
    assert back < 0.1


# ── The builder ──────────────────────────────────────────────────────────────

def _rows(con):
    return (con.A, con.h_bar, con.jdot_qdot, con.v_obs)


def _assert_geometry_identical(a, b):
    np.testing.assert_array_equal(a.A, b.A)
    np.testing.assert_array_equal(a.h_bar, b.h_bar)
    np.testing.assert_array_equal(a.jdot_qdot, b.jdot_qdot)
    np.testing.assert_array_equal(a.v_obs, b.v_obs)


def _build(d, **kw):
    b = make_builder(**kw)
    return run(b, [make_obstacle(d=d)], n_frames=3)


def test_flag_off_carries_no_per_row_gains_at_all():
    """None, not an array of k0_cbf. build_row_rhs then evaluates the scalar
    expression rather than an elementwise one that merely agrees with it."""
    con = _build(0.25)
    assert con.k0_row is None
    assert con.k1_row is None
    assert con.zone_row is None


def test_flag_off_and_flag_on_agree_bit_for_bit_on_the_geometry():
    """Whatever rung the row lands on, the ladder never moves the barrier —
    only how hard it pushes and how expensive it is to ignore."""
    for d in (0.45, 0.28, 0.22, 0.15, 0.09, 0.04):
        off = _build(d)
        on = _build(d, enable_zone_ladder=True)
        _assert_geometry_identical(off, on)


def test_on_the_active_rung_even_the_bound_is_bit_identical():
    """The user-visible contract of the middle rung: 20 to 10 cm behaves
    exactly as it does today, down to the floating-point bound."""
    off = _build(0.15)
    on = _build(0.15, enable_zone_ladder=True)
    np.testing.assert_array_equal(off.G, on.G)
    qd = np.full(NV, 0.05)
    h_off, _ = build_row_rhs(off, qd, qd, k0=25.0, k1=10.5,
                             retreat_horizon=0.15, speed_horizon=0.10)
    h_on, _ = build_row_rhs(on, qd, qd, k0=25.0, k1=10.5,
                            retreat_horizon=0.15, speed_horizon=0.10)
    np.testing.assert_array_equal(h_off, h_on)


def test_the_zoned_and_scalar_paths_are_the_same_expression():
    """`zoned=False` on a snapshot that HAS gains must reproduce the scalar
    result exactly. If it did not, the two paths would have drifted and the
    bit-identity argument above would be about a different code path than the
    one that ships."""
    on = _build(0.05, enable_zone_ladder=True)
    qd = np.full(NV, 0.05)
    a, _ = build_row_rhs(on, qd, qd, k0=25.0, k1=10.5,
                         retreat_horizon=0.15, speed_horizon=0.10, zoned=False)
    manual = 10.5 * (on.A @ qd - on.v_obs) + 25.0 * on.h_bar + on.jdot_qdot
    np.testing.assert_array_equal(a, manual)


def test_only_obstacle_rows_are_scheduled():
    """Self-collision and joint-limit rows keep the scalar gains: their
    'distance' is a capsule gap or a radian, and the rungs are calibrated in
    obstacle metres. Scheduling them off an obstacle ladder would be a category
    error that happens to type-check."""
    from franka_experiments.utils.cbf_state_rows import G_OBS
    b = make_builder(enable_zone_ladder=True, joint_limit_rows_enabled=True)
    # Park the joints near their upper limit so the joint-limit family actually
    # produces rows; at the harness default q = 0.1 rad it produces none and
    # the assertion below would pass vacuously.
    con = run(b, [make_obstacle(d=0.04)], n_frames=3, q=2.70)
    other = np.flatnonzero(con.group != G_OBS)
    assert other.size > 0, 'test needs a non-obstacle row to be meaningful'
    np.testing.assert_allclose(con.k0_row[other], 25.0)
    np.testing.assert_allclose(con.k1_row[other], 10.5)
    # and the obstacle row on the hold rung really did move, so the assertion
    # above is about selectivity rather than about nothing happening at all.
    obs = np.flatnonzero(con.group == G_OBS)
    assert float(con.k0_row[obs][0]) == pytest.approx(50.0)


def test_the_notice_rung_lowers_the_gain_that_multiplies_a_derivative():
    """k1 is the noise amplifier and the phase-margin killer: it multiplies a
    DERIVATIVE of a measured signal, and against the arm's own velocity the
    loop crosses unity gain at omega = k1 (1.67 Hz at the shipped 10.5, where
    the hardware log rings at 1.9 Hz). On the outer rung it must be lower."""
    con = _build(0.28, enable_zone_ladder=True)
    from franka_experiments.utils.cbf_state_rows import G_OBS
    obs = np.flatnonzero(con.group == G_OBS)
    assert obs.size
    assert float(con.k1_row[obs][0]) == pytest.approx(7.0)
    assert float(con.k0_row[obs][0]) == pytest.approx(12.0)


def test_the_notice_rung_engages_further_out_than_the_shipped_gains():
    """The 'notices it earlier, pushes gently' property, as arithmetic rather
    than as prose. The row binds where k1*hdot + k0*h_bar changes sign, i.e. at
    d = d_safe + (k1/k0)*|hdot|, and the notice ratio 7/12 is larger than the
    shipped 10.5/25."""
    z = _ladder()
    hdot = 0.12                                  # arm walking in at 12 cm/s
    d_bind_shipped = 0.15 + (10.5 / 25.0) * hdot
    d_bind_notice = 0.15 + (7.0 / 12.0) * hdot
    assert d_bind_notice > d_bind_shipped
    # and it lands INSIDE the notice rung, so the schedule is self-consistent:
    # the gains that put the engagement there are the gains in force there.
    assert z.gains(d_bind_notice).k0 == pytest.approx(12.0)
    assert d_bind_notice < z.bounds[3]


def test_the_notice_rung_shrinks_the_step_a_fabricated_velocity_makes():
    """The measured artefact: on a genuinely STATIC obstacle v_obs ran as a
    square wave 0.000 -> 0.085 m/s, and each edge is k1*0.085 of step in a row
    that was binding on every tick. Two of those edges are the two largest
    single-tick jumps in the hardware log."""
    artefact = 0.085
    shipped = 10.5 * artefact
    notice = 7.0 * artefact
    assert shipped == pytest.approx(0.8925, abs=1e-4)
    assert notice < 0.7 * shipped


def test_the_inner_rungs_make_a_violation_more_expensive():
    """'Maximum priority' as a number the QP can act on: the row reads
    a^T qddot + m*s >= b, so m < 1 forces more slack for the same violation and
    the cost is rho*s^2/2, i.e. quadratic in m."""
    from franka_experiments.utils.cbf_state_rows import G_OBS, NV as _NV
    def slack_col(d):
        con = _build(d, enable_zone_ladder=True)
        i = int(np.flatnonzero(con.group == G_OBS)[0])
        return -float(con.G[i, con.A.shape[1] + G_OBS])
    assert slack_col(0.15) == pytest.approx(1.0)     # active: untouched
    assert slack_col(0.08) == pytest.approx(0.5)     # priority
    assert slack_col(0.03) == pytest.approx(0.25)    # hold
    # never zero: a row the arm physically cannot satisfy must stay relaxable,
    # which is the B4 "never deadlock" requirement.
    assert slack_col(0.0) > 0.0


def test_the_zone_multiplier_never_loosens_what_criticality_decided():
    """The two multipliers pull opposite ways on the same axis, which is why
    multiplying them is coherent rather than merely convenient.

    Criticality is w_max/w in [1, w_max]: it LOOSENS distant rows and saturates
    at 1 for a violated one. The ladder is in [m_hold, 1]: it TIGHTENS close
    rows and is exactly 1 outside the priority rung. Each is inert where the
    other works. The invariant that matters is one-sided: adding the ladder can
    never make a row looser than criticality alone had made it.
    """
    from franka_experiments.utils.cbf_state_rows import G_OBS
    def slack_col(d, **kw):
        con = _build(d, **kw)
        i = int(np.flatnonzero(con.group == G_OBS)[0])
        return -float(con.G[i, con.A.shape[1] + G_OBS])
    for d in (0.45, 0.28, 0.22, 0.15, 0.09, 0.03):
        crit = slack_col(d, enable_weighted_slack=True)
        both = slack_col(d, enable_weighted_slack=True, enable_zone_ladder=True)
        assert both <= crit + 1e-12, f'ladder loosened the row at d={d}'
    # Outside the priority rung the ladder is inert, so the two agree exactly.
    for d in (0.45, 0.28, 0.22, 0.15):
        assert (slack_col(d, enable_weighted_slack=True)
                == pytest.approx(slack_col(d, enable_weighted_slack=True,
                                           enable_zone_ladder=True)))


def test_ladder_from_params_returns_none_when_the_flag_is_off():
    class Q:
        enable_zone_ladder = False
    assert ladder_from_params(Q()) is None


# ── Boundaries relative to d_safe ────────────────────────────────────────────

def _shipped_ratios():
    import os
    import yaml
    path = os.path.join(os.path.dirname(__file__), '..', 'config', 'fr3_control.yaml')
    with open(path) as f:
        p = yaml.safe_load(f)['params']
    return {k: p[k] for k in ('zone_r_notice', 'zone_r_active', 'zone_r_priority',
                              'zone_r_hold', 'zone_blend_r')}


def test_the_boundaries_scale_with_d_safe():
    """Lowering d_safe must move the whole ladder with the barrier. With
    absolute boundaries d_safe = 0.03 left the hold rung at 0.05 m, OUTSIDE the
    barrier, and the robot could not be brought closer whatever d_safe said."""
    from _cbf_builder_harness import make_params
    r = _shipped_ratios()
    for ds in (0.03, 0.10, 0.15, 0.20):
        z = ladder_from_params(make_params(enable_zone_ladder=True, d_safe=ds, **r))
        assert z.bounds == pytest.approx(tuple(
            r[k] * ds for k in ('zone_r_hold', 'zone_r_priority',
                                'zone_r_active', 'zone_r_notice')))
        assert z.blend_m == pytest.approx(r['zone_blend_r'] * ds)
        assert not z.blend_clamped


def test_shipped_ratios_put_d_safe_at_two_thirds_of_the_outer_rung():
    r = _shipped_ratios()
    assert 1.0 / r['zone_r_notice'] == pytest.approx(2.0 / 3.0)
    # ...and at d_safe = 0.20 they reproduce the hardware-designed ladder.
    from _cbf_builder_harness import make_params
    z = ladder_from_params(make_params(enable_zone_ladder=True, d_safe=0.20, **r))
    assert z.bounds == pytest.approx((0.05, 0.10, 0.20, 0.30))
    assert z.blend_m == pytest.approx(0.04)


def test_a_small_d_safe_no_longer_cuts_the_task_far_outside_the_barrier():
    """The measured complaint: at d_safe = 0.03 the task was already halved at
    ~0.12 m by the absolute priority rung."""
    from _cbf_builder_harness import make_params
    z = ladder_from_params(make_params(enable_zone_ladder=True, d_safe=0.03,
                                       **_shipped_ratios()))
    assert z.task_weight(0.10, 0.01) == pytest.approx(1.0)
    assert z.task_weight(0.05, 0.01) == pytest.approx(1.0)
    z.reset_task()
    # hold boundary 0.0075 m, blend band +-0.003 m: 0.004 is below the band
    assert z.task_weight(0.004, 0.01) == pytest.approx(0.0)


def test_a_relative_ladder_needs_a_positive_d_safe():
    from _cbf_builder_harness import make_params
    with pytest.raises(ValueError, match='d_safe'):
        ladder_from_params(make_params(enable_zone_ladder=True, d_safe=0.0))


# ── What "stop" actually commands ────────────────────────────────────────────

def _nominal_after_the_switch(d, *, nom, qdot, k_brake=5.0, resume_s=0.5):
    """The blend the node performs at STEP 3a, isolated so it can be asserted
    without a ROS graph. Kept identical to the node's expression on purpose: if
    they ever diverge this test is the thing that stops being true."""
    z = _ladder(resume_s=resume_s)
    w = z.task_weight(d, 0.01)
    return w * nom + (1.0 - w) * (-k_brake * qdot), w


def test_the_hold_rung_commands_a_deceleration_and_not_a_coast():
    """The trap this is here to mark.

    A zero nominal makes the QP minimise ‖q̈‖², whose unconstrained solution is
    q̈ = 0 — "hold this velocity". The arm would COAST into the obstacle at
    whatever speed it already had, which is the same failure the joint-state
    stale branch documents. So the task switch fades the nominal into
    -k_brake*qdot, and the test of that is a sign: the commanded acceleration
    must oppose the current velocity.
    """
    qdot = np.full(NV, 0.05)
    nom = np.array([2.0, -1.0, 0.5, 0.0, 0.3, -0.2, 0.1])
    far, w_far = _nominal_after_the_switch(0.35, nom=nom, qdot=qdot)
    np.testing.assert_allclose(far, nom)
    assert w_far == pytest.approx(1.0)

    held, w_held = _nominal_after_the_switch(0.03, nom=nom, qdot=qdot)
    assert w_held == pytest.approx(0.0)
    # It is not zero...
    assert np.linalg.norm(held) > 0.0
    # ...and it points against the motion, i.e. it sheds speed.
    assert float(held @ qdot) < 0.0
    np.testing.assert_allclose(held, -5.0 * qdot)


def test_the_priority_rung_gives_way_without_giving_up():
    """Half authority, not none: the trajectory yields but the arm is still
    being driven somewhere, which is what keeps the priority rung distinct from
    the hold rung rather than being a cliff at the priority boundary."""
    qdot = np.full(NV, 0.05)
    nom = np.array([2.0, -1.0, 0.5, 0.0, 0.3, -0.2, 0.1])
    eff, w = _nominal_after_the_switch(0.08, nom=nom, qdot=qdot)
    assert w == pytest.approx(0.5)
    assert 0.0 < np.linalg.norm(eff) < np.linalg.norm(nom)


def test_the_barrier_rows_are_untouched_by_the_task_switch():
    """The claim that makes 'hold' safe: the task is what gives way, the
    barrier is not. The switch lives entirely in the OBJECTIVE, so the rows a
    hold-rung snapshot carries are the same rows any other snapshot carries."""
    deep = _build(0.03, enable_zone_ladder=True)
    off = _build(0.03)
    _assert_geometry_identical(off, deep)
    # and the row still demands retreat: h_bar is negative that far inside
    # d_safe, so k0*h_bar drives the bound negative whatever the task is doing.
    from franka_experiments.utils.cbf_state_rows import G_OBS
    i = int(np.flatnonzero(deep.group == G_OBS)[0])
    assert deep.h_bar[i] < 0.0
