"""enable_outrun_evasion on the real ConstraintBuilder: inert unless a closing
obstacle cannot be outrun, and never a change to any QP row.

Same regression shape as the tracker-source and feedforward tests: with the
flag OFF the snapshot is bit-identical to today's; with the flag ON but nothing
closing faster than the point can retreat, still bit-identical AND the bias
field is None; only a genuinely un-outrunnable approach produces a bias — and
even then A, h_bar, jdot_qdot and G are untouched, because the bias lives in
the objective. Drives the shipped builder through _cbf_builder_harness.
"""

import numpy as np

from _cbf_builder_harness import (NV, QD, make_builder, make_obstacle, run)
from franka_experiments.utils.cbf_state_rows import retreat_speed_available

PR = (0.5, 0.0, 0.5)
PH = (0.5, -0.25, 0.5)
N_HAT = np.array([0.0, 1.0, 0.0])          # obstacle → control point


def _rows_identical(a, b):
    np.testing.assert_array_equal(a.A, b.A)
    np.testing.assert_array_equal(a.h_bar, b.h_bar)
    np.testing.assert_array_equal(a.jdot_qdot, b.jdot_qdot)
    np.testing.assert_array_equal(a.G, b.G)
    np.testing.assert_array_equal(a.v_obs, b.v_obs)


def _v_avail_at_the_cp():
    """The harness Jacobian at the q the runs use, so a test can pick a closing
    speed relative to what the point can actually outrun."""
    b = make_builder()
    b._kin.update(np.full(NV, 0.1), np.zeros(NV))
    Jp, _ = b._kin.point_jacobian(0, np.asarray(PR))
    return retreat_speed_available(N_HAT @ Jp, 0.9 * QD)


def _run(flag, v, **kw):
    # obstacle_velocity_max lifted out of the way: the trigger now reads the
    # CONDITIONED closing speed, which is clamped like every other estimate in
    # this filter, and these tests are about the outrun logic rather than
    # about that clamp. (On the robot the clamp does apply, so an obstacle
    # closing faster than obstacle_velocity_max is judged as closing at it.)
    kw.setdefault('obstacle_velocity_max', 20.0)
    b = make_builder(obstacle_velocity_source='tracker', enable_outrun_evasion=flag, **kw)
    return run(b, [make_obstacle(pr=PR, ph=PH, v=(0.0, v, 0.0), frames_seen=20,
                                 cov=np.eye(3) * 1e-4, track_id=3)],
               n_frames=5, qdot=0.05)


def test_flag_off_is_bit_identical_and_carries_no_bias():
    v = 3.0 * _v_avail_at_the_cp()
    off = _run(False, v)
    assert off.outrun_bias is None
    _rows_identical(off, _run(False, v))


def test_zero_velocity_with_the_flag_on_is_bit_identical_to_flag_off():
    off = _run(False, 0.0)
    on = _run(True, 0.0)
    _rows_identical(off, on)
    assert on.outrun_bias is None


def test_a_static_obstacle_without_a_track_gives_no_bias():
    b = make_builder(obstacle_velocity_source='tracker', enable_outrun_evasion=True)
    con = run(b, [make_obstacle(pr=PR, ph=PH)], n_frames=5, qdot=0.05)
    assert con.outrun_bias is None


def test_an_outrunnable_approach_gives_no_bias_and_identical_rows():
    """Closing at a third of v_avail: below the ramp, nothing is added."""
    v = 0.3 * _v_avail_at_the_cp()
    off, on = _run(False, v), _run(True, v)
    _rows_identical(off, on)
    assert on.outrun_bias is None
    assert on.v_obs[0] > 0.0, 'fixture: the obstacle must be seen as closing'


def test_an_un_outrunnable_approach_biases_the_target_but_not_the_rows():
    v = 3.0 * _v_avail_at_the_cp()
    off, on = _run(False, v), _run(True, v)
    _rows_identical(off, on)                       # the rows never move
    assert on.outrun_bias is not None
    assert on.outrun_bias.shape == (NV,)
    assert np.linalg.norm(on.outrun_bias) > 0.0
    assert np.linalg.norm(on.outrun_bias) <= 3.0 + 1e-9   # outrun_evasion_max_bias


def test_the_bias_moves_the_point_perpendicular_to_the_obstacle_velocity():
    """Full urgency: the Cartesian push J_p·bias is (nearly) orthogonal to the
    obstacle's velocity — sideways, not backwards."""
    v = 3.0 * _v_avail_at_the_cp()
    on = _run(True, v)
    on_builder = make_builder()
    on_builder._kin.update(np.full(NV, 0.1), np.zeros(NV))
    Jp, _ = on_builder._kin.point_jacobian(0, np.asarray(PR))
    push = Jp @ on.outrun_bias
    cos = float(push @ N_HAT) / np.linalg.norm(push)
    assert abs(cos) < 0.2, cos


def test_the_bias_grows_continuously_with_the_closing_speed():
    va = _v_avail_at_the_cp()
    norms = [np.linalg.norm(_run(True, s * va).outrun_bias)
             if _run(True, s * va).outrun_bias is not None else 0.0
             for s in np.linspace(0.3, 1.6, 27)]
    assert norms[0] == 0.0 and norms[-1] > 0.0
    assert np.all(np.diff(norms) >= -1e-9)
    assert max(np.diff(norms)) < 0.6 * norms[-1]      # no step


# ── The gates that keep noise from becoming motion ──────────────────────────

def test_no_bias_beyond_the_engage_gap():
    """The outrun test compares a speed against a speed and never looks at the
    gap, so ungated it fired anywhere inside cbf_obstacle_horizon (1.2 m).
    Stepping aside from something a metre away is noise with a direction."""
    v = 3.0 * _v_avail_at_the_cp()
    far = run(make_builder(obstacle_velocity_source='tracker', enable_outrun_evasion=True,
                           obstacle_velocity_max=20.0, outrun_evasion_engage_gap=0.30),
              [make_obstacle(d=0.90, pr=PR, ph=PH, v=(0.0, v, 0.0), frames_seen=20)],
              n_frames=5, qdot=0.05)
    assert far.outrun_bias is None


def test_the_engage_gap_fades_instead_of_switching():
    """A step in the gate is a step in q̈_nom. The magnitude must grow
    continuously as the obstacle comes in, from zero at the gap."""
    v = 3.0 * _v_avail_at_the_cp()
    gap = 0.30
    mags = []
    for h in np.linspace(gap, 0.02, 30):
        con = run(make_builder(obstacle_velocity_source='tracker',
                               enable_outrun_evasion=True,
                               obstacle_velocity_max=20.0,
                               outrun_evasion_engage_gap=gap),
                  [make_obstacle(d=0.15 + h, pr=PR, ph=PH, v=(0.0, v, 0.0), frames_seen=20)],
                  n_frames=5, qdot=0.05)
        mags.append(0.0 if con.outrun_bias is None else float(np.linalg.norm(con.outrun_bias)))
    mags = np.array(mags)
    assert mags[0] < 1e-12 and mags[-1] > 0.0
    assert np.all(np.diff(mags) >= -1e-9), 'must grow monotonically as h shrinks'
    assert np.max(np.diff(mags)) < 0.25 * mags[-1], 'no step anywhere in the fade'


def test_a_weak_leverage_point_is_not_tripped_by_a_small_closing_speed():
    """v_avail collapses with the leverage of the row (0.10-0.20 m/s on the
    link4 control points, measured), so an unfloored ratio reads estimation
    noise as 'cannot be outrun'. The floor says a point that cannot retreat at
    0.3 m/s cannot escape sideways at 0.3 m/s either."""
    from franka_experiments.utils.evasion_direction import outrun_ratio
    weak = 0.12
    # what a conditioned tracker still reports on a static scene, worst case
    assert outrun_ratio(0.20, weak, margin=0.8) > 1.0            # would trip
    assert outrun_ratio(0.20, weak, margin=0.8, v_avail_floor=0.30) < 1.0
    # a genuinely fast obstacle still trips it, floor or no floor
    assert outrun_ratio(1.2, weak, margin=0.8, v_avail_floor=0.30) > 1.0
    # and the floor never LOOSENS the test on a point that does have authority
    for va in (0.5, 1.0, 2.5):
        assert (outrun_ratio(0.9, va, margin=0.8, v_avail_floor=0.30)
                == outrun_ratio(0.9, va, margin=0.8))


def test_the_trigger_is_clamped_like_every_other_estimate():
    """The outrun ratio now reads the CONDITIONED closing speed, so it inherits
    obstacle_velocity_max: the filter never acts on a speed it does not
    believe anywhere else. Same input, clamp low enough, and the trigger
    stops firing."""
    v = 3.0 * _v_avail_at_the_cp()
    assert _run(True, v, obstacle_velocity_max=20.0).outrun_bias is not None
    assert _run(True, v, obstacle_velocity_max=0.30).outrun_bias is None
