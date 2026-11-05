"""The static-scene hold and the two continuity gates: identity, not arithmetic.

WHY THIS FILE EXISTS
--------------------
Measured on ``datasets/experiments_irim2.zip`` (the DIAG lab, 157.6 s, the arm
on a 0.25 m circle, no contact and no protective stop):

* the QP carried a MEDIAN of 46 obstacle rows, p95 67 — the cell's own
  surfaces, not the task's obstacle;
* ``LinkDistance.obstacle_velocity`` reached 7.78 m/s on the wire and the
  conditioned ``v_obs`` reached 3.11 m/s, where the fastest real motion in the
  room was a hand;
* a row was violated (h < 0) on 23.1 % of QP ticks and in **90.6 %** of those
  the geometric gap was still wider than ``d_safe``: the barrier was crossed by
  the velocity-derived margins, not by geometry;
* ``‖q̈_safe − q̈_nom‖`` had a median of 1.64 rad/s² against a nominal of 1.85,
  and 16 % of ticks pushed AGAINST the task.

So the properties below are the contract of the fix, not a number:

1. with the flags at their defaults the rows are BIT-IDENTICAL to the previous
   build, including on a message carrying a full obstacle track;
2. a static-classified track loses its VELOCITY, never its row: ``A``, the
   geometric part of ``h`` and the gains are untouched;
3. sustained motion releases a held track within ``release_hits`` frames, and
   an isolated spike does not release it;
4. a young track's velocity is withheld until it has been SEEN for
   ``obstacle_velocity_min_age_s``;
5. the residual floor may not overrule a track until it has persisted.

Drives the real ConstraintBuilder through ``_cbf_builder_harness``. No ROS.
"""

import numpy as np
import pytest

from _cbf_builder_harness import make_builder, make_obstacle, run

from franka_experiments.utils.static_scene import StaticSceneHold, hold_from_params

# n̂ = (pr − ph)/‖·‖ = +y, so a velocity along +y is CLOSING.
PR = (0.5, 0.0, 0.5)
PH = (0.5, -0.25, 0.5)
RATE = 90.0                      # the shipped camera profile
DT = 1.0 / RATE

STATIC_ON = dict(enable_static_scene_hold=True, obstacle_velocity_source='tracker',
                 enable_vobs_in_hdot=True, enable_velocity_standoff=True,
                 enable_velocity_feedforward=True, enable_uncertainty_margin=True,
                 obstacle_velocity_min_frames=3, velocity_feedforward_min_frames=3,
                 static_scene_window_s=0.2, static_scene_min_samples=8)


# ── 1. the classifier itself ────────────────────────────────────────────────

def _feed(hold, v_of_frame, *, n, t0=0.0, tid=4):
    for k in range(n):
        hold.observe(tid, v_of_frame(k), t0 + k * DT)
    return hold


def _hold(**kw):
    kw.setdefault('window_s', 0.2)
    kw.setdefault('min_samples', 8)
    return StaticSceneHold(**kw)


def test_a_wall_is_static_once_it_has_a_window_of_history():
    h = _hold()
    rng = np.random.default_rng(3)
    # A static cluster's centroid jitters; the measured noise floor on a room
    # blob was 0.198 m/s with ZERO mean, which is what the medians key on.
    _feed(h, lambda k: rng.normal(0.0, 0.02, 3), n=4)
    assert not h.is_static(4), 'cannot be called scene before a full window'
    _feed(h, lambda k: rng.normal(0.0, 0.02, 3), n=40, t0=4 * DT)
    assert h.is_static(4)


def test_isolated_merge_spikes_do_not_un_static_a_wall():
    """The phantom is made of one-or-two-frame excursions (a cluster merge steps
    the centroid). A MEAN would let two of them in the same direction through;
    a median does not."""
    h = _hold()

    def v(k):
        return np.array([0.0, 3.0, 0.0]) if k % 17 == 0 else np.zeros(3)

    _feed(h, v, n=60)
    assert h.is_static(4)


def test_a_walking_person_is_never_static():
    h = _hold()
    _feed(h, lambda k: np.array([0.0, 0.30, 0.0]), n=60)
    assert not h.is_static(4)


def test_a_standing_person_who_starts_moving_is_released_in_three_frames():
    h = _hold(release_speed_m_s=0.25, release_hits=3)
    _feed(h, lambda k: np.zeros(3), n=40)
    assert h.is_static(4)
    t = 40 * DT
    h.observe(4, (0.0, 0.5, 0.0), t)
    assert h.is_static(4), 'one frame of motion is a spike, not a release'
    h.observe(4, (0.0, 0.5, 0.0), t + DT)
    assert h.is_static(4)
    h.observe(4, (0.0, 0.5, 0.0), t + 2 * DT)
    assert not h.is_static(4), '3 consecutive frames = 33 ms at 90 Hz'
    # ... and it must then earn a FRESH window, not inherit the old verdict.
    _feed(h, lambda k: np.zeros(3), n=5, t0=t + 3 * DT)
    assert not h.is_static(4)


def test_irregular_capture_stamps_still_classify():
    """THE bug the DIAG bag found. The samples are trimmed to the window, so
    the span between the oldest surviving sample and the newest is at most the
    window and, with a real camera's irregular stamps, essentially never
    exactly it. Testing that span against the window rejected almost
    everything: on the bag the classifier held 0.6 % of the rows whose track
    was quiet, while 23 of its 31 tracks had a median speed under the
    threshold. 'Enough history' is measured from when the uninterrupted
    history started instead."""
    h = _hold(window_s=0.3, min_samples=8)
    rng = np.random.default_rng(11)
    t = 0.0
    for _ in range(80):
        t += DT * float(rng.uniform(0.6, 1.8))        # jittered capture stamps
        h.observe(4, rng.normal(0.0, 0.02, 3), t)
    assert h.is_static(4)


def test_a_released_track_must_earn_a_whole_window_again():
    """The release truncates the samples; without its own 'uninterrupted
    history' mark, min_samples alone would let 8 quiet frames (89 ms at 90 Hz)
    re-classify a track that was moving a moment ago."""
    h = _hold(window_s=0.3, min_samples=8, release_speed_m_s=0.25, release_hits=3)
    t = 0.0
    for _ in range(60):
        h.observe(4, np.zeros(3), t)
        t += DT
    assert h.is_static(4)
    for _ in range(3):
        h.observe(4, (0.0, 0.5, 0.0), t)
        t += DT
    assert not h.is_static(4)
    for _ in range(12):                               # 12 quiet frames = 0.13 s
        h.observe(4, np.zeros(3), t)
        t += DT
    assert not h.is_static(4), 'min_samples is met, the window is not'
    for _ in range(20):
        h.observe(4, np.zeros(3), t)
        t += DT
    assert h.is_static(4)


def test_an_unidentified_obstacle_is_never_static():
    h = _hold()
    for k in range(60):
        h.observe(0, np.zeros(3), k * DT)       # track_id 0 = no confirmed track
    assert not h.is_static(0)
    assert h.n_tracks == 0


def test_the_same_capture_stamp_twice_is_one_sample():
    """The consumer rebuilds faster than the camera delivers: a repeated stamp
    must not count as evidence, or the window fills in half the time."""
    h = _hold()
    for k in range(40):
        h.observe(4, np.zeros(3), k * DT)
        h.observe(4, np.zeros(3), k * DT)       # same frame, seen twice
    assert h.is_static(4)
    h2 = _hold()
    for k in range(4):
        for _ in range(10):
            h2.observe(4, np.zeros(3), k * DT)
    assert not h2.is_static(4), '4 frames of history, whatever the rebuild rate'


def test_age_is_wall_time_not_frames_and_pruning_bounds_the_memory():
    h = _hold()
    _feed(h, lambda k: np.zeros(3), n=10)
    assert h.age_s(4, 9 * DT) == pytest.approx(9 * DT, abs=1e-9)
    assert h.age_s(99, 1.0) == 0.0                    # unknown id
    h.prune(10.0)
    assert h.n_tracks == 0


def test_hold_from_params_is_none_when_the_flag_is_off():
    class P:
        enable_static_scene_hold = False
    assert hold_from_params(P) is None
    P.enable_static_scene_hold = True
    assert isinstance(hold_from_params(P), StaticSceneHold)


# ── 2. the rows: the default path is inert ──────────────────────────────────

def _assert_identical(a, b):
    np.testing.assert_array_equal(a.A, b.A)
    np.testing.assert_array_equal(a.h_bar, b.h_bar)
    np.testing.assert_array_equal(a.jdot_qdot, b.jdot_qdot)
    np.testing.assert_array_equal(a.G, b.G)
    np.testing.assert_array_equal(a.v_obs, b.v_obs)


def _run(flags, *, v=(0.0, 0.0, 0.0), frames_seen=30, track_id=9, n_frames=40):
    b = make_builder(**flags)
    ob = make_obstacle(d=0.25, pr=PR, ph=PH, v=v, frames_seen=frames_seen,
                       cov=np.eye(3) * 0.01, track_id=track_id, cp_label='fr3_link5#0')
    return run(b, [ob], n_frames=n_frames, dt=DT, qdot=0.0), b


def test_with_the_flags_off_a_tracked_static_obstacle_is_bit_identical():
    """The regression this repo holds every feature to: OFF changes nothing."""
    base = dict(obstacle_velocity_source='tracker', enable_vobs_in_hdot=True,
                enable_velocity_standoff=True, enable_velocity_feedforward=True,
                obstacle_velocity_min_frames=3, velocity_feedforward_min_frames=3)
    a, _ = _run(base)
    c, _ = _run(dict(base, enable_static_scene_hold=False,
                     obstacle_velocity_min_age_s=0.0,
                     obstacle_velocity_floor_min_frames=1))
    _assert_identical(a, c)


# ── 3. a held track loses the velocity, never the row ───────────────────────

def test_a_scene_track_keeps_its_row_and_loses_its_velocity_claim():
    """A surface reporting a spiky phantom velocity: with the hold ON its
    tightenings go away and the GEOMETRY of the row does not move."""
    def spiky(k):
        v = (0.0, 2.5, 0.0) if k % 13 == 0 else (0.0, 0.0, 0.0)
        return [make_obstacle(d=0.25, pr=PR, ph=PH, v=v, frames_seen=30,
                              cov=np.eye(3) * 0.09, track_id=11,
                              cp_label='fr3_link5#0')]

    off = make_builder(**{**STATIC_ON, 'enable_static_scene_hold': False})
    on = make_builder(**STATIC_ON)
    a = run(off, spiky, n_frames=40, dt=DT, qdot=0.0)
    c = run(on, spiky, n_frames=40, dt=DT, qdot=0.0)
    # the row direction is geometry: identical
    np.testing.assert_allclose(a.A, c.A, atol=0, rtol=0)
    # the velocity claim and every tightening it buys: gone
    assert abs(float(c.v_obs[0])) <= 1e-12 < abs(float(a.v_obs[0]))
    assert on.diag_static_rows >= 1 and on.diag_static_tracks >= 1
    assert off.diag_static_rows == 0
    # h is LOOSER by exactly the margins the velocity bought, never tighter
    assert float(c.h_bar[0]) > float(a.h_bar[0])
    assert (on.diag_hbrake, on.diag_hunc) == (0.0, 0.0)
    # The standoff decays rather than stepping (by design: tightening is
    # instant, releasing is EMA'd), so what is left on the last rebuild is the
    # tail of the spikes that arrived BEFORE the track was classified.
    assert on.diag_hstand < 0.25 * off.diag_hstand
    assert off.diag_hbrake > 0.0


def test_a_genuinely_moving_obstacle_is_untouched_by_the_hold():
    """The safety direction: the hold must not be able to silence a real
    approach. Same flags, a track closing at 0.4 m/s."""
    moving = lambda k: [make_obstacle(  # noqa: E731
        d=0.40 - 0.2 * k * DT, pr=PR, ph=PH, v=(0.0, 0.4, 0.0), frames_seen=30,
        cov=np.eye(3) * 0.01, track_id=12, cp_label='fr3_link5#0')]
    off = make_builder(**{**STATIC_ON, 'enable_static_scene_hold': False})
    on = make_builder(**STATIC_ON)
    a = run(off, moving, n_frames=40, dt=DT, qdot=0.0)
    c = run(on, moving, n_frames=40, dt=DT, qdot=0.0)
    _assert_identical(a, c)
    assert on.diag_static_rows == 0
    assert abs(float(c.v_obs[0])) > 0.1


# ── 4. the age gate (P1a) ───────────────────────────────────────────────────

def test_a_moving_track_ages_so_its_velocity_is_eventually_admitted():
    """The age gate must not be able to withhold a MOVING obstacle's velocity
    forever: the first version of the release path reset the track's first-seen
    stamp, so a track that kept moving re-entered the release branch every
    frame, never aged, and never got its anticipation back."""
    h = StaticSceneHold(window_s=0.2, min_samples=8, release_speed_m_s=0.25,
                        release_hits=3)
    for k in range(40):
        h.observe(4, (0.0, 0.5, 0.0), k * DT)
    assert not h.is_static(4)
    assert h.age_s(4, 39 * DT) == pytest.approx(39 * DT, abs=1e-9)


def test_a_young_track_claims_nothing_and_an_aged_one_claims_again():
    """8 updates at 90 Hz is 89 ms, shorter than the measured median track
    life: the gate that means something is in SECONDS."""
    flags = dict(STATIC_ON, enable_static_scene_hold=False,
                 obstacle_velocity_min_age_s=0.20)
    young, b_young = _run(flags, v=(0.0, 0.4, 0.0), n_frames=5)   # 56 ms of age
    assert abs(float(young.v_obs[0])) <= 1e-12
    assert b_young.diag_young_rows >= 1
    aged, b_aged = _run(flags, v=(0.0, 0.4, 0.0), n_frames=40)    # 0.43 s
    assert abs(float(aged.v_obs[0])) > 0.1
    # The counter is per REBUILD, so by the last one nothing is young any more.
    assert b_aged.diag_young_rows == 0


# ── 5. the residual-floor persistence gate (P1b) ────────────────────────────

def test_the_residual_floor_must_persist_before_it_overrules_a_track():
    """An argmin hop is one frame of apparent closing. The floor exists for a
    limb, which holds for hundreds of milliseconds."""
    flags = dict(obstacle_velocity_source='tracker', enable_vobs_in_hdot=False,
                 obstacle_velocity_residual_floor=True,
                 obstacle_velocity_residual_floor_gap=0.0,
                 obstacle_velocity_min_frames=1, obstacle_velocity_alpha=0.0,
                 obstacle_velocity_floor_min_frames=4)

    def hop(k):
        # The gap takes one big step (a hop to a nearer surface patch) and
        # then sits still: the residual reports a large closing on ONE frame.
        d = 0.25 if k < 10 else 0.20
        return [make_obstacle(d=d, pr=PR, ph=PH, v=(0.0, 0.0, 0.0),
                              frames_seen=30, cov=np.eye(3) * 0.01,
                              track_id=13, cp_label='fr3_link5#0')]

    gated = make_builder(**flags)
    plain = make_builder(**{**flags, 'obstacle_velocity_floor_min_frames': 1})
    a = run(gated, hop, n_frames=11, dt=DT, qdot=0.0)
    c = run(plain, hop, n_frames=11, dt=DT, qdot=0.0)
    assert float(c.v_obs[0]) > 0.5, 'the ungated floor passes the hop'
    assert float(a.v_obs[0]) <= 1e-12, 'the gated floor holds it'
    assert gated.diag_floor_held >= 1
