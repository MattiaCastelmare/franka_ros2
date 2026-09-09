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
