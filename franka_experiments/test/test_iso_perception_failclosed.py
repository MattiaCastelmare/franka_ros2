"""Fail-closed perception: the three silent "no constraint" paths (Step 7).

Each of these was a place where perception produced *absence of a constraint*
where it should have produced *a fault*, and absence is indistinguishable from
safety to everything downstream:

1. **The contact-regime publish hole.** A frame in which every control point is
   closer than ``min_thresh`` published only an empty heartbeat, so the QP ran
   with zero obstacle rows at the closest the arm ever gets to something.
2. **The unbounded hold.** A control point with no finite measurement held its
   last smoothed distance forever, and the barrier kept building a row on it. A
   stale row is worse than no row: it looks like knowledge.
3. **The dead confidence gate.** ``LinkDistance.confidence`` was 1.0
   unconditionally, so the filter's ``min_confidence`` threshold was inert — a
   threshold every entry passes by construction is not a threshold.

The fourth, the empty-frame run in ``cbf_safety_filter``, is tested through
``_empty_frame_fault`` directly.

Fault behaviour is **[R]** for a RATED safety function under ISO 13849-1:2023.
This chain is not one, so failing closed here is **[E]** good practice and not
compliance — see SAFETY.md.
"""

import math
import types

import numpy as np
import pytest

from franka_experiments.utils.distance_engine import ControlPointResult, DistanceEngine
from franka_experiments.utils.perception_msgs import build_cp_messages, find_pt_confidence

THRESH = {'min_thresh': 0.08, 'max_thresh': 0.7}
LINKS = ['fr3_link5', 'fr3_link8']


class _Log:
    def __init__(self):
        self.lines = []

    def warning(self, m):
        self.lines.append(m)

    warn = info = error = debug = lambda self, *a, **k: None


def _cp(d, seg=0, cp=0, link='fr3_link5', direction=(0.0, 1.0, 0.0)):
    return ControlPointResult(
        point=np.array([0.5, 0.0, 0.5]), seg_idx=seg, cp_idx=cp, radius=0.05,
        start_link=link, end_link=link, distance=float(d),
        direction=None if direction is None else np.asarray(direction, float),
        closest_obstacle_point=np.array([0.5, -0.25, 0.5]),
        closest_pixel=(10, 10))


def _msgs(results, n_pts=500):
    from builtin_interfaces.msg import Time
    return build_cp_messages(
        cp_results=results, n_pts=n_pts, stamp=Time(), frame_id='fr3_link0',
        segment_links=LINKS, thresholds=THRESH, fallback=2.0, zones={})


# ── 1. the contact-regime entries survive, flagged ───────────────────────────

def test_a_contact_regime_entry_is_published_valid_and_flagged():
    _, mld = _msgs([_cp(0.02)])
    assert len(mld.links) == 1
    ld = mld.links[0]
    assert ld.valid is True, 'the closest measurement in the frame must not be dropped'
    assert ld.distance == pytest.approx(0.02)
    assert ld.confidence == pytest.approx(0.5), \
        'below min_thresh is flagged, not trusted and not dropped'


def test_a_zero_gap_is_still_a_measurement():
    _, mld = _msgs([_cp(0.0)])
    assert mld.links[0].valid is True
    assert mld.links[0].confidence == pytest.approx(0.5)


def test_no_row_is_dropped_at_the_shipped_min_confidence():
    """The roadmap's own acceptance check for Step 7 change 3.

    find_pt_confidence floors its pixel-count term at 0.2 and its range term at
    1.0 below 2 m, and the per-CP band ends at max_thresh = 0.7 m — so every
    entry in that band scores exactly 0.2 or better and the filter's
    `ob.conf < min_confidence` gate drops nothing. Populating the field
    therefore cannot cost a row; it only makes the gate live for the future.
    """
    import yaml, os
    cfg = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                       '..', 'config', 'fr3_control.yaml')
    with open(cfg) as fh:
        min_conf = float(yaml.safe_load(fh)['params']['min_confidence'])
    for n_pts in (1, 4, 50, 100, 500, 5000):
        for d in (0.0, 0.05, 0.08, 0.3, 0.7):
            _, mld = _msgs([_cp(d)], n_pts=n_pts)
            conf = mld.links[0].confidence
            assert not conf < min_conf, (
                f'd={d} n_pts={n_pts} scores {conf} < min_confidence={min_conf}: '
                f'this row would now be dropped and was not before')


def test_the_flag_stays_above_the_shipped_min_confidence():
    """0.5 has to clear the filter's gate, or flagging becomes dropping by
    another route."""
    import yaml, os
    cfg = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                       '..', 'config', 'fr3_control.yaml')
    with open(cfg) as fh:
        assert 0.5 > float(yaml.safe_load(fh)['params']['min_confidence'])


# ── 3. the confidence gate is live again ─────────────────────────────────────

def test_confidence_is_the_real_figure_not_a_constant():
    _, many = _msgs([_cp(0.30)], n_pts=500)
    _, few = _msgs([_cp(0.30)], n_pts=10)
    assert many.links[0].confidence > few.links[0].confidence
    assert many.links[0].confidence == pytest.approx(find_pt_confidence(0.30, 500))
    assert few.links[0].confidence == pytest.approx(find_pt_confidence(0.30, 10))


def test_a_far_entry_scores_lower_than_a_near_one():
    _, near = _msgs([_cp(0.5)])
    _, far = _msgs([_cp(3.5)])
    assert far.links[0].confidence < near.links[0].confidence


def test_the_multi_distance_path_is_unchanged():
    """The legacy topic keeps its band: only the per-link safety topic moved."""
    md, _ = _msgs([_cp(0.02)])
    assert md.distances[0].valid is False
    md, _ = _msgs([_cp(0.30)])
    assert md.distances[0].valid is True


# ── 2. the hold is bounded ───────────────────────────────────────────────────

def _engine(hold_s, log=None):
    return DistanceEngine({'min_depth_m': 0.2, 'max_depth_m': 4.0,
                           'lpf_alpha': 0.5,
                           'iso_distance_hold_max_s': hold_s}, logger=log)


def test_a_held_value_expires_and_is_published_invalid():
    log = _Log()
    e = _engine(0.10, log)
    out = e._lpf_pass([_cp(0.30)], dt=0.033)
    assert out[0].distance == pytest.approx(0.30)
    # Four dropouts at 33 ms = 132 ms > 100 ms.
    for i in range(4):
        out = e._lpf_pass([_cp(float('inf'))], dt=0.033)
    assert out[0].direction is None, 'the hold must expire'
    assert out[0].distance == pytest.approx(0.30), \
        'the distance is still carried; it is the DIRECTION that invalidates it'
    assert any('hold expired' in m for m in log.lines)


def test_a_held_value_inside_the_bound_is_still_held():
    e = _engine(0.10)
    e._lpf_pass([_cp(0.30)], dt=0.033)
    out = e._lpf_pass([_cp(float('inf'))], dt=0.033)
    assert out[0].direction is not None
    assert out[0].distance == pytest.approx(0.30)


def test_an_expired_hold_becomes_an_invalid_link_distance():
    """The expiry rides the path build_cp_messages already has for a missing
    direction — no new invalidation route, and that path is already tested."""
    _, mld = _msgs([_cp(0.30, direction=None)])
    assert mld.links[0].valid is False


def test_a_finite_measurement_clears_the_hold_clock():
    e = _engine(0.10)
    e._lpf_pass([_cp(0.30)], dt=0.033)
    for _ in range(3):
        e._lpf_pass([_cp(float('inf'))], dt=0.033)
    e._lpf_pass([_cp(0.28)], dt=0.033)       # obstacle is back
    out = e._lpf_pass([_cp(float('inf'))], dt=0.033)
    assert out[0].direction is not None, 'the clock must restart, not resume'


def test_a_zero_bound_disables_it_and_restores_the_old_behaviour():
    e = _engine(0.0)
    e._lpf_pass([_cp(0.30)], dt=0.033)
    for _ in range(200):
        out = e._lpf_pass([_cp(float('inf'))], dt=0.033)
    assert out[0].direction is not None
    assert out[0].distance == pytest.approx(0.30)


def test_the_bound_still_bites_without_frame_stamps():
    """dt=None is the legacy path; the hold must still expire, on frame count."""
    e = _engine(0.10)
    e._lpf_pass([_cp(0.30)], dt=None)
    for _ in range(10):
        out = e._lpf_pass([_cp(float('inf'))], dt=None)
    assert out[0].direction is None


# ── 4. the empty-frame run ───────────────────────────────────────────────────

from franka_experiments.nodes.cbf_safety_filter import CBFSafetyFilter


class _FilterStub:
    def __init__(self, **over):
        defaults = dict(iso_enabled=True, iso_empty_frame_max_s=0.20,
                        zone_r_active=1.0, d_safe=0.10)
        defaults.update(over)
        self.P = types.SimpleNamespace(**defaults)
        self._empty_close = True
        self._empty_since = 0.0
        self._empty_faulted = False

    def get_logger(self):
        return types.SimpleNamespace(error=lambda *a, **k: None)


def test_an_empty_run_after_a_close_frame_faults():
    f = _FilterStub()
    assert CBFSafetyFilter._empty_frame_fault(f, 0.10) is False
    assert CBFSafetyFilter._empty_frame_fault(f, 0.30) is True


def test_an_empty_run_after_a_far_frame_does_not_fault():
    f = _FilterStub()
    f._empty_close = False
    assert CBFSafetyFilter._empty_frame_fault(f, 10.0) is False


def test_the_empty_run_fault_is_inert_with_the_flag_off():
    f = _FilterStub(iso_enabled=False)
    assert CBFSafetyFilter._empty_frame_fault(f, 10.0) is False


def test_no_empty_run_no_fault():
    f = _FilterStub()
    f._empty_since = None
    assert CBFSafetyFilter._empty_frame_fault(f, 10.0) is False
