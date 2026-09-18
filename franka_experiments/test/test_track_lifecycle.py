"""Why tracks die, and the threshold that stops a two-frame track being believed.

A hardware run with one static and one moving obstacle produced 1727 tracks in
91 s — 19 births per second — and the track behind the closest control point
persisted for a MEDIAN of 70 ms, about two perception frames. A Kalman filter
that lives two frames has estimated nothing: its velocity is dominated by the
initial covariance and the process noise, so it reports its own prior and the
consumer reads that as an obstacle moving.

Two changes, and these tests pin both:

* ``obstacle_velocity_min_frames`` 3 -> 8, so a track has to survive before it
  is believed. That is a threshold on the SYMPTOM.
* lifecycle counters on ``TrackManager``, so the next run says which of
  ``max_missed`` / the gates / ``cluster_min_points`` is actually killing the
  tracks. That is what names the disease.

The counters have to be trustworthy to be worth acting on, so what is asserted
here is that each one moves for its OWN reason and not for another's.
"""

import os

import numpy as np
import pytest
import yaml

from franka_experiments.utils.obstacle_tracker import TrackManager

CONFIG = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..',
                      'config', 'fr3_control.yaml')
COMPLETE = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..',
                        'config', 'fr3_complete.yaml')


def mgr(**kw):
    kw.setdefault('confirm_hits', 3)
    kw.setdefault('confirm_window', 5)
    kw.setdefault('max_missed', 5)
    return TrackManager(**kw)


def feed(m, seq):
    """seq: list of per-frame lists of 3-vectors."""
    for frame in seq:
        m.step([np.asarray(p, dtype=float) for p in frame], dt=1 / 30.0)


# ── the threshold ────────────────────────────────────────────────────────────

def test_the_shipped_threshold_outlives_the_birth_rule():
    """3 matched the tracker's M-of-N birth rule, so a track was trusted the
    instant it was confirmed. That only holds if a confirmed track then lives."""
    with open(CONFIG) as fh:
        p = yaml.safe_load(fh)['params']
    assert p['obstacle_velocity_min_frames'] >= 8
    # and it must stay inside the declared range
    from franka_experiments.utils.config import CBF_PARAM_SPEC
    kind, bounds = CBF_PARAM_SPEC['obstacle_velocity_min_frames']
    assert kind == 'int'
    assert p['obstacle_velocity_min_frames'] <= bounds['maximum']


def test_the_residual_floor_is_still_on_so_the_gate_is_not_a_blindfold():
    """Raising the gate costs latency on a genuinely new obstacle. It is paid
    for by v_obs = max(tracker, residual): the residual needs no identity and
    no history, so the gap is covered."""
    with open(CONFIG) as fh:
        p = yaml.safe_load(fh)['params']
    assert p['obstacle_velocity_residual_floor'] is True


# ── the counters ─────────────────────────────────────────────────────────────

def test_a_steadily_tracked_object_is_born_once_and_never_reaped():
    m = mgr()
    feed(m, [[[0.5, 0.0, 0.5 + 0.01 * i]] for i in range(30)])
    d = m.death_stats()
    assert d['births'] == 1
    assert d['reaped'] == 0 and d['reaped_young'] == 0
    assert d['alive'] == 1 and d['confirmed'] == 1


def test_a_flickering_object_births_repeatedly_and_dies_young():
    """cluster_min_points too high looks like this: the cluster itself appears
    and disappears, so no track ever reaches confirmation."""
    m = mgr(max_missed=1)
    feed(m, [([[0.5, 0.0, 0.5]] if i % 3 == 0 else []) for i in range(30)])
    d = m.death_stats()
    assert d['births'] >= 3
    assert d['reaped_young'] > d['reaped'], d


def test_a_mature_track_that_disappears_is_reaped_as_mature():
    m = mgr(max_missed=2)
    feed(m, [[[0.5, 0.0, 0.5]] for _ in range(10)])       # confirm it
    feed(m, [[] for _ in range(6)])                        # then lose it
    d = m.death_stats()
    assert d['reaped'] == 1 and d['reaped_young'] == 0
    assert d['mean_life'] >= 3


def test_max_missed_decides_whether_an_occlusion_kills_a_track():
    """The first candidate cause: tracks reaped while merely occluded."""
    def survives(max_missed, gap):
        m = mgr(max_missed=max_missed)
        feed(m, [[[0.5, 0.0, 0.5]] for _ in range(10)])
        feed(m, [[] for _ in range(gap)])
        return m.death_stats()['reaped'] + m.death_stats()['reaped_young'] == 0
    assert survives(5, 3) and not survives(1, 3)


def test_a_tight_gate_starts_a_new_track_instead_of_updating_one():
    """The second candidate: clusters land outside every gate, so `unassociated`
    climbs with `births` while the object never actually left."""
    tight = mgr(gate_max_m=0.01, max_missed=99)
    feed(tight, [[[0.5, 0.0, 0.5 + 0.05 * i]] for i in range(10)])
    wide = mgr(gate_max_m=1.0, max_missed=99)
    feed(wide, [[[0.5, 0.0, 0.5 + 0.05 * i]] for i in range(10)])
    assert tight.death_stats()['births'] > wide.death_stats()['births']
    assert tight.death_stats()['unassociated'] > wide.death_stats()['unassociated']
    assert wide.death_stats()['births'] == 1


def test_over_capacity_is_counted_separately_from_a_tight_gate():
    """Conflating the two would hide a saturated tracker behind a tight gate."""
    m = mgr(max_tracks=2, max_missed=99)
    feed(m, [[[0.1 * k, 0.0, 0.5] for k in range(6)] for _ in range(5)])
    d = m.death_stats()
    assert d['births'] == 2
    assert d['over_capacity'] > 0
    assert d['unassociated'] >= d['over_capacity']


def test_the_counters_are_cumulative_and_never_rewind():
    m = mgr(max_missed=1)
    feed(m, [([[0.5, 0.0, 0.5]] if i % 3 == 0 else []) for i in range(20)])
    a = m.death_stats()
    feed(m, [([[0.5, 0.0, 0.5]] if i % 3 == 0 else []) for i in range(20)])
    b = m.death_stats()
    for k in ('births', 'reaped', 'reaped_young', 'unassociated'):
        assert b[k] >= a[k], k


def test_mean_life_counts_updates_not_wall_time():
    """A track coasting through an occlusion ages without accumulating the
    evidence its velocity is supposed to rest on."""
    m = mgr(max_missed=10)
    feed(m, [[[0.5, 0.0, 0.5]] for _ in range(4)])   # 4 real updates
    feed(m, [[] for _ in range(12)])                  # then only coasting
    assert m.death_stats()['mean_life'] == pytest.approx(4.0)


# ── the capacity invariant ───────────────────────────────────────────────────

def test_the_track_table_is_at_least_as_large_as_the_cluster_table():
    """An arithmetic precondition, not a tuning preference.

    At max_tracks = 12 against max_clusters = 16, four clusters per frame could
    never be given a track BY CONSTRUCTION. Measured on a 134 s hardware run:
    1579 of 2205 unassociated clusters (72 %) were dropped for want of a slot,
    so the twelve slots went to whichever clusters arrived first — fragments
    included — and a real obstacle appearing afterwards got no velocity at all.
    """
    with open(COMPLETE) as fh:
        t = yaml.safe_load(fh)['tracking']
    assert t['max_tracks'] >= t['max_clusters'], (
        f"max_tracks={t['max_tracks']} < max_clusters={t['max_clusters']}: "
        f"{t['max_clusters'] - t['max_tracks']} clusters per frame can never "
        f"be tracked")


def test_a_saturated_table_reports_it_rather_than_failing_quietly():
    """`cap` is what made the real cause visible. Without it, saturation and a
    tight gate are indistinguishable: both look like 'clusters are not being
    tracked'."""
    m = mgr(max_tracks=2, max_missed=99)
    feed(m, [[[0.3 * k, 0.0, 0.5] for k in range(5)] for _ in range(4)])
    d = m.death_stats()
    assert d['over_capacity'] > 0
    assert d['births'] == 2
    # and with room, the same scene saturates nothing
    m2 = mgr(max_tracks=8, max_missed=99)
    feed(m2, [[[0.3 * k, 0.0, 0.5] for k in range(5)] for _ in range(4)])
    assert m2.death_stats()['over_capacity'] == 0
    assert m2.death_stats()['births'] == 5


def test_raising_the_table_lets_the_later_clusters_be_tracked():
    """The point of the change: an obstacle that appears after the table filled
    used to get nothing."""
    early = [[0.3 * k, 0.0, 0.5] for k in range(4)]
    late = early + [[2.0, 0.0, 0.5]]        # a fifth object, appearing later
    for n, expect in ((4, False), (8, True)):
        m = mgr(max_tracks=n, max_missed=99)
        feed(m, [early] * 5 + [late] * 5)
        tracked = any(abs(t.x[0] - 2.0) < 0.1 for t in m.tracks)
        assert tracked is expect, f'max_tracks={n}'
