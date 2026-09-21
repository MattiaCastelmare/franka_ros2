"""The EE trajectory overlay: what gets drawn, and what deliberately does not.

The overlay answers one question — is the end effector on the path the commander
asked for — so the tests are about the ways a polyline can lie: a line drawn
across a gap the arm never travelled, a trace that silently stops growing
because it hit its cap, two curves decimated differently so identical paths
render as a sawtooth.
"""

from __future__ import annotations

import numpy as np
import pytest

from franka_experiments.utils.trajectory_trace import (
    ID_ACTUAL,
    ID_DESIRED,
    ID_DEVIATION,
    ID_LABEL,
    TraceBuffer,
    trace_markers,
)


def _line(n, step=0.01, start=(0.0, 0.0, 0.0), axis=0):
    pts = []
    p = np.array(start, dtype=float)
    for _ in range(n):
        pts.append(p.copy())
        p[axis] += step
    return pts


# ── TraceBuffer ──────────────────────────────────────────────────────────────

def test_first_point_is_always_kept():
    b = TraceBuffer(min_spacing_m=0.5)
    assert b.add((1.0, 2.0, 3.0))
    assert len(b) == 1


def test_spacing_filter_drops_points_too_close_to_the_last_kept_one():
    b = TraceBuffer(min_spacing_m=0.01)
    b.add((0.0, 0.0, 0.0))
    assert not b.add((0.005, 0.0, 0.0))
    assert not b.add((0.009, 0.0, 0.0))
    assert b.add((0.011, 0.0, 0.0))
    assert len(b) == 2


def test_spacing_is_measured_from_the_last_KEPT_point():
    """Otherwise a slow approach never accumulates anything.

    Measuring from the last OFFERED point would reset the budget on every
    sample, so an arm creeping at 0.1 mm per tick would be dropped forever
    however far it eventually travelled.
    """
    b = TraceBuffer(min_spacing_m=0.01)
    b.add((0.0, 0.0, 0.0))
    for i in range(1, 20):
        b.add((0.001 * i, 0.0, 0.0))
    assert len(b) == 2
    assert np.allclose(b.last, (0.010, 0.0, 0.0))


def test_a_teleport_cuts_the_trace_instead_of_extending_it():
    b = TraceBuffer(min_spacing_m=0.0, jump_reset_m=0.10)
    for p in _line(5, step=0.02):
        b.add(p)
    assert len(b) == 5
    b.add((5.0, 0.0, 0.0))
    assert len(b) == 1, 'the trace must restart, not bridge the gap'
    assert b.cuts == 1


def test_the_jump_test_wins_over_the_spacing_test():
    """A teleport also passes the spacing test; cutting has to come first."""
    b = TraceBuffer(min_spacing_m=0.01, jump_reset_m=0.10)
    b.add((0.0, 0.0, 0.0))
    b.add((1.0, 0.0, 0.0))
    assert len(b) == 1 and b.cuts == 1


def test_jump_reset_zero_disables_the_cut():
    b = TraceBuffer(min_spacing_m=0.0, jump_reset_m=0.0)
    b.add((0.0, 0.0, 0.0))
    b.add((9.0, 0.0, 0.0))
    assert len(b) == 2 and b.cuts == 0


def test_cap_drops_the_oldest_and_keeps_growing():
    b = TraceBuffer(max_points=10, min_spacing_m=0.0, jump_reset_m=0.0)
    for p in _line(25, step=0.001):
        b.add(p)
    assert len(b) == 10
    assert np.allclose(b.last, (0.024, 0.0, 0.0)), 'the TIP must survive'
    assert np.allclose(b.points[0], (0.015, 0.0, 0.0))


def test_non_finite_and_malformed_points_are_refused():
    b = TraceBuffer()
    assert not b.add((0.0, float('nan'), 0.0))
    assert not b.add((0.0, float('inf'), 0.0))
    assert not b.add((0.0, 0.0))
    assert len(b) == 0


# ── Timestamps: the trail the image overlay draws ────────────────────────────

def test_expire_drops_the_old_end_and_keeps_the_recent_one():
    b = TraceBuffer(min_spacing_m=0.0, jump_reset_m=0.0)
    for i, p in enumerate(_line(10, step=0.01)):
        b.add(p, t=float(i))            # one point per second
    assert b.expire(now=9.0, ttl=3.0) == 6
    assert len(b) == 4
    assert b.times == [6.0, 7.0, 8.0, 9.0]
    assert np.allclose(b.last, (0.09, 0.0, 0.0))


def test_expire_is_a_no_op_when_nothing_is_old_enough():
    b = TraceBuffer(min_spacing_m=0.0, jump_reset_m=0.0)
    b.add((0.0, 0.0, 0.0), t=1.0)
    assert b.expire(now=1.5, ttl=3.0) == 0
    assert len(b) == 1


def test_a_disabled_trail_is_not_an_empty_one():
    """``ttl <= 0`` disables expiry; clearing instead would show nothing.

    A trail length of zero is a configuration mistake, and a viewer that
    silently drew an empty overlay would be the hardest way to notice it.
    """
    b = TraceBuffer(min_spacing_m=0.0, jump_reset_m=0.0)
    for i, p in enumerate(_line(5, step=0.01)):
        b.add(p, t=float(i))
    assert b.expire(now=1e6, ttl=0.0) == 0
    assert len(b) == 5


def test_expire_cannot_outlive_a_cut():
    """A cut drops the timestamps with the points, not just the points.

    Leave the two lists out of step and every subsequent expiry cuts the wrong
    prefix — the trail then loses its HEAD, which is the one part that matters.
    """
    b = TraceBuffer(min_spacing_m=0.0, jump_reset_m=0.10)
    for i, p in enumerate(_line(5, step=0.01)):
        b.add(p, t=float(i))
    b.add((5.0, 0.0, 0.0), t=5.0)       # teleport → cut
    assert len(b) == 1 and b.times == [5.0]
    assert b.expire(now=5.0, ttl=1.0) == 0
    assert len(b) == 1


def test_the_tip_survives_a_point_the_spacing_filter_rejected():
    """Standing still must not make the marker disappear.

    While the arm holds position every sample fails the spacing test, so a
    drawer reading ``last`` would keep the marker at the last STORED vertex and,
    once the trail expires, nowhere at all — precisely when someone is looking
    at where it stopped.
    """
    b = TraceBuffer(min_spacing_m=0.01, jump_reset_m=0.0)
    b.add((0.0, 0.0, 0.0), t=0.0)
    assert not b.add((0.003, 0.0, 0.0), t=1.0)
    assert np.allclose(b.tip, (0.003, 0.0, 0.0))
    assert b.tip_time == 1.0
    b.expire(now=10.0, ttl=1.0)
    assert len(b) == 0
    assert np.allclose(b.tip, (0.003, 0.0, 0.0)), 'the tip is not a vertex'


def test_a_refused_point_does_not_move_the_tip():
    b = TraceBuffer()
    b.add((0.1, 0.0, 0.0), t=1.0)
    b.add((0.1, float('nan'), 0.0), t=2.0)
    assert np.allclose(b.tip, (0.1, 0.0, 0.0))
    assert b.tip_time == 1.0


def test_clear_forgets_the_tip_too():
    b = TraceBuffer()
    b.add((0.1, 0.0, 0.0), t=1.0)
    b.clear()
    assert b.tip is None and len(b) == 0 and b.times == []


def test_the_cap_drops_points_and_their_timestamps_together():
    b = TraceBuffer(max_points=3, min_spacing_m=0.0, jump_reset_m=0.0)
    for i, p in enumerate(_line(6, step=0.001)):
        b.add(p, t=float(i))
    assert len(b) == 3 and b.times == [3.0, 4.0, 5.0]


def test_invalid_configuration_is_rejected():
    with pytest.raises(ValueError):
        TraceBuffer(max_points=1)
    with pytest.raises(ValueError):
        TraceBuffer(min_spacing_m=-0.1)


def test_identical_paths_decimate_identically():
    """Both traces use one spacing, so a perfect follow draws as one curve.

    Decimating the two independently would put their vertices at different arc
    lengths, and two identical curves would render as a sawtooth between them —
    a deviation display inventing the deviation.
    """
    a = TraceBuffer(min_spacing_m=0.01, jump_reset_m=0.0)
    b = TraceBuffer(min_spacing_m=0.01, jump_reset_m=0.0)
    for p in _line(200, step=0.001):
        a.add(p)
        b.add(p)
    assert len(a) == len(b)
    assert np.allclose(np.array(a.points), np.array(b.points))


# ── Markers ──────────────────────────────────────────────────────────────────

def _stamp():
    from builtin_interfaces.msg import Time
    return Time()


def _by_id(arr):
    return {m.id: m for m in arr.markers}


def test_marker_array_carries_the_two_traces_with_their_colours():
    from visualization_msgs.msg import Marker
    des, act = _line(4), _line(4, start=(0.0, 0.02, 0.0))
    m = _by_id(trace_markers(frame_id='fr3_link0', stamp=_stamp(),
                             desired=des, actual=act))
    d, a = m[ID_DESIRED], m[ID_ACTUAL]
    assert d.action == Marker.ADD and a.action == Marker.ADD
    assert d.type == Marker.LINE_STRIP
    assert len(d.points) == 4 and len(a.points) == 4
    assert d.color.r > 0.8 and d.color.g < 0.2 and d.color.b < 0.2, 'desired red'
    assert a.color.b > 0.8 and a.color.r < 0.2, 'actual blue'
    assert d.header.frame_id == 'fr3_link0'


def test_the_two_widths_are_equal_by_default():
    """Nesting a thin line in a thick one hides one of them at every overlap."""
    m = _by_id(trace_markers(frame_id='f', stamp=_stamp(),
                             desired=_line(3), actual=_line(3)))
    assert m[ID_DESIRED].scale.x == m[ID_ACTUAL].scale.x


def test_a_trace_with_fewer_than_two_points_is_deleted_not_drawn():
    from visualization_msgs.msg import Marker
    m = _by_id(trace_markers(frame_id='f', stamp=_stamp(),
                             desired=_line(1), actual=_line(5)))
    assert m[ID_DESIRED].action == Marker.DELETE
    assert m[ID_DESIRED].points == []
    assert m[ID_ACTUAL].action == Marker.ADD


def test_deviation_segment_joins_the_two_tips():
    from visualization_msgs.msg import Marker
    des = _line(3)                                   # tip at (0.02, 0, 0)
    act = _line(3, start=(0.0, 0.05, 0.0))           # tip at (0.02, 0.05, 0)
    m = _by_id(trace_markers(frame_id='f', stamp=_stamp(),
                             desired=des, actual=act))
    seg = m[ID_DEVIATION]
    assert seg.action == Marker.ADD and len(seg.points) == 2
    assert seg.points[0].x == pytest.approx(0.02)
    assert seg.points[1].y == pytest.approx(0.05)


def test_label_reports_the_error_in_millimetres():
    des = _line(3)
    act = _line(3, start=(0.0, 0.05, 0.0))
    m = _by_id(trace_markers(frame_id='f', stamp=_stamp(),
                             desired=des, actual=act))
    assert m[ID_LABEL].text == '50 mm'


def test_label_is_suppressed_while_the_arm_is_tracking():
    """A 1 mm label parked on the trace forever is noise, not information."""
    from visualization_msgs.msg import Marker
    des = _line(3)
    act = _line(3, start=(0.0, 0.0005, 0.0))
    m = _by_id(trace_markers(frame_id='f', stamp=_stamp(),
                             desired=des, actual=act,
                             label_min_error_m=0.002))
    assert m[ID_LABEL].action == Marker.DELETE


def test_deviation_and_label_are_deleted_when_one_side_has_no_points():
    from visualization_msgs.msg import Marker
    m = _by_id(trace_markers(frame_id='f', stamp=_stamp(),
                             desired=[], actual=_line(4)))
    assert m[ID_DEVIATION].action == Marker.DELETE
    assert m[ID_LABEL].action == Marker.DELETE


def test_ids_are_stable_so_a_republish_replaces_rather_than_accumulates():
    a = _by_id(trace_markers(frame_id='f', stamp=_stamp(),
                             desired=_line(3), actual=_line(3)))
    b = _by_id(trace_markers(frame_id='f', stamp=_stamp(),
                             desired=_line(9), actual=_line(9)))
    assert set(a) == set(b) == {ID_DESIRED, ID_ACTUAL, ID_DEVIATION, ID_LABEL}
    assert {m.ns for m in a.values()} == {m.ns for m in b.values()}
