"""obstacle_velocity_source: the switch, and the proof the default path is inert.

The requirement that dominates this file is the same one that dominated
test_cbf_velocity_feedforward: **with the flag at its default the QP rows must
be bit-identical to the pre-change build.** Here that is not arithmetic
identity of a term that happens to be zero — it is the stronger claim that a
message carrying a full obstacle track changes NOTHING while the source is
'residual'. So the tests compare two builders driven with identical inputs
except for the presence of track data, with np.testing.assert_array_equal.

The second half checks the 'tracker' path against a hand-computed projection,
and pins the sign — which is the one error that would silently invert the whole
feature.

Drives the real ConstraintBuilder through _cbf_builder_harness. No ROS.
"""

import numpy as np
import pytest

from _cbf_builder_harness import (NV, make_builder, make_obstacle, run)

# n̂ = (pr − ph)/‖·‖ = +y for this geometry, so a velocity along +y is CLOSING.
PR = (0.5, 0.0, 0.5)
PH = (0.5, -0.25, 0.5)
N_HAT = np.array([0.0, 1.0, 0.0])


def _snaps(source, **ob_kw):
    b = make_builder(obstacle_velocity_source=source)
    return run(b, [make_obstacle(pr=PR, ph=PH, **ob_kw)], n_frames=5, qdot=0.05)


def _assert_identical(a, b):
    np.testing.assert_array_equal(a.A, b.A)
    np.testing.assert_array_equal(a.h_bar, b.h_bar)
    np.testing.assert_array_equal(a.jdot_qdot, b.jdot_qdot)
    np.testing.assert_array_equal(a.G, b.G)
    np.testing.assert_array_equal(a.v_obs, b.v_obs)


# ── THE regression: 'residual' ignores the track completely ─────────────────

def test_residual_mode_is_bit_identical_with_and_without_a_track():
    """A message from the tracker and a message from a publisher that has never
    heard of tracking must produce the SAME QP, byte for byte, while the source
    is 'residual'. Anything less means the default path has been changed by a
    feature that is supposed to be off."""
    plain = _snaps('residual')
    tracked = _snaps('residual', v=(0.0, 0.9, 0.0), frames_seen=20,
                     cov=np.eye(3) * 0.04, track_id=7)
    _assert_identical(plain, tracked)


def test_residual_mode_is_bit_identical_over_a_whole_sequence():
    """One frame could agree by luck: the residual estimator is STATEFUL (its
    EMA and its per-label frame counter), so the check has to survive the state
    evolving."""
    for n in range(1, 12):
        b0, b1 = (make_builder(obstacle_velocity_source='residual'),
                  make_builder(obstacle_velocity_source='residual'))
        a = run(b0, [make_obstacle(pr=PR, ph=PH)], n_frames=n, qdot=0.05)
        c = run(b1, [make_obstacle(pr=PR, ph=PH, v=(0.0, 1.2, 0.0),
                                   frames_seen=30, cov=np.eye(3) * 0.09)],
                n_frames=n, qdot=0.05)
        _assert_identical(a, c)


def test_residual_mode_is_bit_identical_on_a_MOVING_obstacle():
    """The interesting case: the gap actually changes, so the residual's finite
    difference is doing real work and any accidental cross-talk from the track
    would show up in h_bar and v_obs."""
    def moving(k, v=None, **kw):
        return [make_obstacle(d=0.40 - 0.01 * k, pr=PR, ph=PH, v=v, **kw)]

    b0, b1 = (make_builder(obstacle_velocity_source='residual'),
              make_builder(obstacle_velocity_source='residual'))
    a = run(b0, lambda k: moving(k), n_frames=10, qdot=0.05)
    c = run(b1, lambda k: moving(k, v=(0.0, 0.8, 0.0), frames_seen=15,
                                 cov=np.eye(3) * 0.01), n_frames=10, qdot=0.05)
    _assert_identical(a, c)
    assert a.v_obs[0] > 0.0, 'fixture assumption: the residual must be active'


def test_the_two_sources_actually_differ_on_the_same_input():
    """Guards the guards above: if 'tracker' produced the same numbers as
    'residual', every identity test in this file would pass vacuously."""
    kw = dict(v=(0.0, 0.9, 0.0), frames_seen=20)
    r = _snaps('residual', **kw)
    t = _snaps('tracker', **kw)
    assert not np.allclose(r.v_obs, t.v_obs), (r.v_obs, t.v_obs)


# ── The 'tracker' path ──────────────────────────────────────────────────────

def test_tracker_mode_reports_the_projection_onto_n_hat():
    v = np.array([0.3, 0.8, -0.2])
    con = _snaps('tracker', v=tuple(v), frames_seen=20)
    assert np.isclose(con.v_obs[0], float(N_HAT @ v)), con.v_obs


def test_a_receding_track_gives_exactly_zero():
    """THE sign test, and the conservative clamp in one. n̂ points obstacle →
    control point, so a velocity along −n̂ is receding; only the approaching
    half may ever reach the QP. Trusting the receding half would mean RELAXING
    a barrier on a vision estimate."""
    con = _snaps('tracker', v=(0.0, -1.5, 0.0), frames_seen=20)
    assert con.v_obs[0] == 0.0


def test_a_purely_lateral_track_gives_exactly_zero():
    """A velocity orthogonal to n̂ changes the gap at first order not at all,
    and must contribute nothing to the barrier's rate — however fast it is."""
    con = _snaps('tracker', v=(2.0, 0.0, 2.0), frames_seen=20)
    assert abs(con.v_obs[0]) < 1e-12


def test_a_young_track_is_not_trusted():
    """frames_seen counts UPDATES, and below the gate the answer is 0.0 — the
    static-obstacle assumption, not a guess."""
    for n in (0, 1, 2):
        assert _snaps('tracker', v=(0.0, 1.0, 0.0), frames_seen=n).v_obs[0] == 0.0
    assert _snaps('tracker', v=(0.0, 1.0, 0.0), frames_seen=3).v_obs[0] > 0.0


def test_no_track_at_all_gives_zero_not_a_crash():
    """The defaults path: a publisher that knows nothing about tracking."""
    assert _snaps('tracker').v_obs[0] == 0.0


def test_tracker_mode_still_honours_the_speed_clamp():
    con = _snaps('tracker', v=(0.0, 9.0, 0.0), frames_seen=20)
    assert con.v_obs[0] == 2.0        # obstacle_velocity_max


def test_tracker_mode_has_no_lag_on_a_step():
    """The whole point. The residual needs several frames to climb to a new
    closing speed because of its 0.7 EMA; the tracked value is available in
    full on the first frame past the gate, because the smoothing already
    happened upstream."""
    v = (0.0, 1.0, 0.0)
    seq = [run(make_builder(obstacle_velocity_source='tracker'),
               [make_obstacle(pr=PR, ph=PH, v=v, frames_seen=20)],
               n_frames=n, qdot=0.0).v_obs[0] for n in (1, 2, 3, 8)]
    assert all(np.isclose(x, 1.0) for x in seq), seq


def test_a_nan_in_the_track_is_rejected_rather_than_propagated():
    """A NaN reaching h_bar would poison the whole QP row block."""
    con = _snaps('tracker', v=(0.0, float('nan'), 0.0), frames_seen=20)
    assert con.v_obs[0] == 0.0
    assert np.all(np.isfinite(con.h_bar))


# ── The switch does not leak state between modes ────────────────────────────

def test_switching_source_does_not_hand_the_tracker_the_residual_s_memory():
    """The two estimators must not share a filter. If 'tracker' advanced the
    residual's per-label EMA (or read it), switching at runtime would produce a
    transient built from the other estimator's history."""
    b = make_builder(obstacle_velocity_source='tracker')
    run(b, [make_obstacle(pr=PR, ph=PH, v=(0.0, 1.0, 0.0), frames_seen=20)],
        n_frames=10)
    assert b._obs_vel == {}, 'tracker mode advanced the residual filter state'


def test_disabling_obstacle_velocity_zeroes_both_sources():
    for src in ('residual', 'tracker'):
        b = make_builder(obstacle_velocity_source=src,
                         obstacle_velocity_enabled=False)
        con = run(b, [make_obstacle(pr=PR, ph=PH, v=(0.0, 1.0, 0.0),
                                    frames_seen=20)], n_frames=5)
        assert con.v_obs[0] == 0.0


# ── The residual floor (Phase 1 reactivity fix) ─────────────────────────────
#
# A track is the velocity of a cluster CENTROID. On the hand bags a reaching
# hand measured 0.22-0.27 m/s on the residual and 0.003 m/s on the projected
# track, and wherever there is no usable track at all the tracker's answer is
# exactly 0. With the floor on, the QP gets the LARGER of the two closing
# speeds; with it off, the tracker path is bit-identical to before.

def _moving(k, **kw):
    return [make_obstacle(d=0.40 - 0.01 * k, pr=PR, ph=PH, **kw)]


def test_floor_off_is_bit_identical_to_tracker_only():
    a = run(make_builder(obstacle_velocity_source='tracker',
                         obstacle_velocity_residual_floor=False),
            lambda k: _moving(k, v=(0.0, 0.1, 0.0), frames_seen=20), n_frames=10, qdot=0.05)
    b = run(make_builder(obstacle_velocity_source='tracker'),
            lambda k: _moving(k, v=(0.0, 0.1, 0.0), frames_seen=20), n_frames=10, qdot=0.05)
    _assert_identical(a, b)


def test_floor_supplies_the_residual_when_there_is_no_track():
    """No track at all (the scene guard dropped the person): tracker-only says
    0, the floor must say exactly what 'residual' mode would have said."""
    trk = run(make_builder(obstacle_velocity_source='tracker',
                           obstacle_velocity_residual_floor=True),
              lambda k: _moving(k), n_frames=10, qdot=0.05)
    res = run(make_builder(obstacle_velocity_source='residual'),
              lambda k: _moving(k), n_frames=10, qdot=0.05)
    only = run(make_builder(obstacle_velocity_source='tracker'),
               lambda k: _moving(k), n_frames=10, qdot=0.05)
    assert only.v_obs[0] == 0.0, 'fixture: tracker-only must be blind here'
    assert res.v_obs[0] > 0.0
    np.testing.assert_array_equal(trk.v_obs, res.v_obs)
    np.testing.assert_array_equal(trk.h_bar, res.h_bar)


def test_floor_takes_the_larger_of_the_two_and_never_the_smaller():
    """A slow track under a fast residual: the residual wins. A fast track over
    a slow residual: the track wins. Neither can pull the other down."""
    slow_track = run(make_builder(obstacle_velocity_source='tracker',
                                  obstacle_velocity_residual_floor=True),
                     lambda k: _moving(k, v=(0.0, 0.05, 0.0), frames_seen=20),
                     n_frames=10, qdot=0.05)
    res = run(make_builder(obstacle_velocity_source='residual'),
              lambda k: _moving(k), n_frames=10, qdot=0.05)
    assert np.isclose(slow_track.v_obs[0], res.v_obs[0])
    assert slow_track.v_obs[0] > 0.05

    fast_track = run(make_builder(obstacle_velocity_source='tracker',
                                  obstacle_velocity_residual_floor=True),
                     lambda k: _moving(k, v=(0.0, 1.5, 0.0), frames_seen=20),
                     n_frames=10, qdot=0.05)
    assert np.isclose(fast_track.v_obs[0], 1.5)


def test_floor_can_only_tighten_relative_to_tracker_only():
    """Row by row, over a whole sequence: v_obs with the floor >= v_obs without
    it, and the barrier value is untouched by the floor itself."""
    rng = np.random.default_rng(0)
    for trial in range(5):
        v = float(rng.uniform(0.0, 1.0))
        kw = dict(v=(0.0, v, 0.0), frames_seen=20)
        a = run(make_builder(obstacle_velocity_source='tracker'),
                lambda k: _moving(k, **kw), n_frames=8, qdot=0.05)
        b = run(make_builder(obstacle_velocity_source='tracker',
                             obstacle_velocity_residual_floor=True),
                lambda k: _moving(k, **kw), n_frames=8, qdot=0.05)
        assert np.all(b.v_obs >= a.v_obs - 1e-12)
        np.testing.assert_array_equal(a.A, b.A)
        np.testing.assert_array_equal(a.h_bar, b.h_bar)


def test_floor_is_inert_in_residual_mode():
    a = run(make_builder(obstacle_velocity_source='residual'),
            lambda k: _moving(k, v=(0.0, 0.9, 0.0), frames_seen=20), n_frames=8, qdot=0.05)
    b = run(make_builder(obstacle_velocity_source='residual',
                         obstacle_velocity_residual_floor=True),
            lambda k: _moving(k, v=(0.0, 0.9, 0.0), frames_seen=20), n_frames=8, qdot=0.05)
    _assert_identical(a, b)


# ── Conditioning of the tracked velocity (deadband + median) ────────────────
#
# Measured on the real centroid sequences of rosbag/arm_complex: a scene whose
# obstacles do not move produces a tracked speed with a p90 of 0.19-0.25 m/s
# and excursions past 1.6 m/s, which k1 turns into ~3 rad/s² of step in the
# row's right-hand side every tick. These two turn that into silence without
# touching what a real approach reports.

def _v_obs(v, *, n_frames=6, **over):
    b = make_builder(obstacle_velocity_source='tracker', **over)
    con = run(b, lambda k: [make_obstacle(pr=PR, ph=PH, frames_seen=20,
                                          v=(0.0, v(k) if callable(v) else v, 0.0))],
              n_frames=n_frames, qdot=0.0)
    return float(con.v_obs[0])


def test_deadband_silences_a_tracked_speed_below_it():
    """The measured noise floor of a STATIC scene must reach the QP as exactly
    zero — not as something small, as nothing: a moving v_obs moves the k1
    term, the retreat cap and the evasion trigger."""
    for v in (0.0, 0.05, 0.10, 0.149):
        assert _v_obs(v, obstacle_velocity_track_deadband=0.15) == 0.0, v


def test_deadband_subtracts_rather_than_gates():
    """A gate would step by the whole deadband as the estimate crossed it.
    Subtracting keeps the map continuous, so nothing jumps."""
    db = 0.15
    for v in (0.16, 0.3, 1.0):
        assert np.isclose(_v_obs(v, obstacle_velocity_track_deadband=db), v - db)
    # continuity across the threshold
    lo = _v_obs(0.1499, obstacle_velocity_track_deadband=db)
    hi = _v_obs(0.1501, obstacle_velocity_track_deadband=db)
    assert abs(hi - lo) < 1e-3


def test_the_median_rejects_a_one_frame_excursion():
    """A wander excursion lasts a frame or two; a real approach does not."""
    spike = lambda k: 1.5 if k == 4 else 0.0
    assert _v_obs(spike, obstacle_velocity_median=3, n_frames=5) == 0.0
    # ...and without the median the same spike goes straight through
    assert np.isclose(_v_obs(spike, obstacle_velocity_median=1, n_frames=5), 1.5)


def test_the_median_passes_a_sustained_approach():
    assert np.isclose(_v_obs(0.8, obstacle_velocity_median=3), 0.8)


def test_both_off_is_bit_identical_to_the_unconditioned_path():
    kw = dict(v=(0.0, 0.4, 0.0), frames_seen=20)
    a = run(make_builder(obstacle_velocity_source='tracker'),
            [make_obstacle(pr=PR, ph=PH, **kw)], n_frames=6, qdot=0.05)
    b = run(make_builder(obstacle_velocity_source='tracker',
                         obstacle_velocity_median=1,
                         obstacle_velocity_track_deadband=0.0),
            [make_obstacle(pr=PR, ph=PH, **kw)], n_frames=6, qdot=0.05)
    _assert_identical(a, b)


def test_the_conditioning_only_touches_the_tracker_path():
    """The residual floor is deliberately NOT deadbanded: it is the quiet one
    (p99 0.017-0.029 m/s on the same static scenes) and the reactive one."""
    con = run(make_builder(obstacle_velocity_source='tracker',
                           obstacle_velocity_residual_floor=True,
                           obstacle_velocity_track_deadband=0.15),
              lambda k: [make_obstacle(d=0.40 - 0.01 * k, pr=PR, ph=PH,
                                       v=(0.0, 0.05, 0.0), frames_seen=20)],
              n_frames=10, qdot=0.05)
    assert con.v_obs[0] > 0.0, 'the residual must still reach the QP'


# ── Median on the consumed closing speed ────────────────────────────────────
#
# The hardware log this comes from had a STATIC obstacle and a v_obs peaking
# at 0.298 m/s with a tick-to-tick p95 of 0.215, which k1 turns into 2.26
# rad/s² of step in a row that was binding on every tick. A rate limit was
# tried first and removed: the artefact steps are only marginally faster than
# a limb could physically move, so a physically honest bound never binds.

def _drive_v(seq, dt=1.0 / 30.0, **over):
    """Feed a per-FRAME sequence of tracked speeds, return what the QP got."""
    import _cbf_builder_harness as H
    b = make_builder(obstacle_velocity_source='tracker', **over)
    out = []
    for k, v in enumerate(seq):
        con = b.build(H.make_js(q=0.1, qdot=0.0, stamp=k * dt),
                      H.make_obs([make_obstacle(pr=PR, ph=PH, frames_seen=20,
                                                v=(0.0, v, 0.0))], stamp=k * dt),
                      k * dt)
        out.append(float(con.v_obs[0]))
    return np.array(out)


def test_the_median_rejects_an_isolated_excursion_on_the_final_estimate():
    """This is the noise this filter is made of: one or two frames high, then
    back down. A median removes it outright; an average would fold it in."""
    # A 5-median rejects up to TWO outliers in any window of five, which is
    # the shape of this noise; a third high sample inside the same window is
    # no longer an excursion and must be believed.
    seq = [0.02, 0.03, 0.30, 0.01, 0.02, 0.03, 0.28, 0.30, 0.02, 0.01, 0.02]
    lim = _drive_v(seq, obstacle_velocity_median=5)
    raw = _drive_v(seq, obstacle_velocity_median=1)
    assert raw.max() >= 0.30
    assert lim.max() < 0.10, lim
    # ...and a genuine sustained rise inside the same series still gets through
    assert _drive_v(seq + [0.30] * 5, obstacle_velocity_median=5).max() >= 0.28


def test_the_median_passes_a_sustained_approach_in_full():
    for v in (0.2, 0.5, 1.0):
        lim = _drive_v([v] * 10, obstacle_velocity_median=5)
        assert np.isclose(lim[-1], v, atol=1e-9), (v, lim[-1])


def test_the_median_lag_is_two_frames_whatever_the_speed():
    """(N−1)/2 frames, independent of the approach speed — the property a rate
    limit does not have (its lag grows with the speed, which is backwards)."""
    for v in (0.2, 1.0, 2.0):
        lim = _drive_v([0.0] * 4 + [v] * 8, obstacle_velocity_median=5)
        first_full = int(np.argmax(np.isclose(lim, v)))
        assert first_full - 4 == 2, (v, first_full)


def test_median_one_disables_it_exactly():
    seq = [0.0, 0.9, 0.0, 0.9, 0.1]
    np.testing.assert_allclose(_drive_v(seq, obstacle_velocity_median=1), seq)


def test_the_median_is_a_time_window_not_a_rebuild_window():
    """The builder runs at 50 Hz on a 30 Hz stream, so two rebuilds can carry
    the same frame. Keyed on the capture stamp, the same frame counts once."""
    import _cbf_builder_harness as H
    b = make_builder(obstacle_velocity_source='tracker', obstacle_velocity_median=3)
    out = []
    for k, (t_cap, v) in enumerate([(0.00, 0.0), (0.00, 0.0), (0.033, 0.6),
                                    (0.033, 0.6), (0.066, 0.6), (0.066, 0.6)]):
        con = b.build(H.make_js(q=0.1, qdot=0.0, stamp=k * 0.02),
                      H.make_obs([make_obstacle(pr=PR, ph=PH, frames_seen=20,
                                                v=(0.0, v, 0.0))],
                                 stamp=k * 0.02, t_cap=t_cap), k * 0.02)
        out.append(float(con.v_obs[0]))
    # three DISTINCT frames seen (0.0, 0.6, 0.6) -> the median is already 0.6
    assert np.isclose(out[-1], 0.6), out
