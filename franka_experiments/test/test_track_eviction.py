"""TrackManager(evict_stale_tentative=True): a full table must not refuse a new
object while it holds stale tentative clutter — and must never evict a track
that is confirmed or still being hit.

Why it exists: on the 2026-09-30 ball throws the table sat at max_tracks for
the whole run and 85 % of new clusters were refused (cap=), so the ball often
got no track at all.
"""

import numpy as np

from franka_experiments.utils.obstacle_tracker import TrackManager

DT = 1.0 / 90.0


def _mgr(evict, **kw):
    kw.setdefault('max_tracks', 3)
    kw.setdefault('confirm_hits', 3)
    kw.setdefault('confirm_window', 5)
    kw.setdefault('max_missed', 15)
    return TrackManager(evict_stale_tentative=evict, **kw)


def _fill_with_flicker(m):
    """Three one-frame clusters, then a frame without them: three stale
    tentative tracks occupying the whole table."""
    m.step([np.array([x, 0.0, 1.0]) for x in (0.0, 1.0, 2.0)], DT)
    m.step([], DT)


def test_off_refuses_the_birth_as_before():
    m = _mgr(False)
    _fill_with_flicker(m)
    m.step([np.array([5.0, 0.0, 1.0])], DT)
    assert m.n_over_capacity == 1
    assert not any(np.allclose(t.x[:3], [5.0, 0.0, 1.0]) for t in m.tracks)


def test_on_gives_the_new_object_a_slot():
    m = _mgr(True)
    _fill_with_flicker(m)
    m.step([np.array([5.0, 0.0, 1.0])], DT)
    assert m.n_over_capacity == 0 and m.n_evicted == 1
    assert len(m.tracks) == 3
    assert any(np.allclose(t.x[:3], [5.0, 0.0, 1.0], atol=1e-6) for t in m.tracks)


def test_confirmed_tracks_are_never_evicted():
    m = _mgr(True)
    pts = [np.array([x, 0.0, 1.0]) for x in (0.0, 1.0, 2.0)]
    for _ in range(4):                       # confirm all three
        m.step(pts, DT)
    m.step([], DT)                           # all three now stale — but confirmed
    ids = {t.track_id for t in m.tracks}
    m.step([np.array([5.0, 0.0, 1.0])], DT)
    assert {t.track_id for t in m.tracks} == ids
    assert m.n_evicted == 0 and m.n_over_capacity == 1


def test_a_tentative_track_still_being_hit_is_never_evicted():
    m = _mgr(True)
    ball = np.array([0.0, 0.0, 1.0])
    m.step([ball, np.array([1.0, 0.0, 1.0]), np.array([2.0, 0.0, 1.0])], DT)
    ball_id = next(t.track_id for t in m.tracks if np.allclose(t.x[:3], ball))
    # The ball keeps being seen (tentative, 2 hits < 3); the clutter does not.
    # A new cluster must take a CLUTTER slot, not the ball's.
    m.step([ball, np.array([5.0, 0.0, 1.0])], DT)
    assert ball_id in {t.track_id for t in m.tracks}
    assert m.n_evicted == 1
