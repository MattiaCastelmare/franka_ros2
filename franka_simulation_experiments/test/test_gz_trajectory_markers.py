"""The Gazebo marker request: its field names are real, and the trail is redrawn when it moves.

`gz_trajectory_markers` talks to Gazebo's /marker service by shelling out a
protobuf TEXT-FORMAT string, which is the one wire format with no schema check on
this side. A misspelt field is not an exception and not even an empty marker:
protobuf stops at the typo and ign-transport sends what it parsed so far, so the
marker is created and listed while every field after the typo is dropped —
including `point`, which is last. The line then never appears and nothing in the
logs says why. That is how `size` (the field the proto does NOT have; it is
`scale`) got in and survived a by-hand check.

So the request's field names are pinned against marker.proto itself.
"""
import os
import re

import pytest
from franka_experiments.utils.trajectory_trace import TraceBuffer

import gz_trajectory_markers as M

#: Shipped with ignition-msgs; absent on a machine without the Gazebo headers.
PROTO = '/usr/include/ignition/msgs8/ignition/msgs/marker.proto'

CIRCLE = [(0.4, 0.25, 0.45), (0.4, 0.0, 0.70), (0.4, -0.25, 0.45)]


def _marker_proto_fields(path: str) -> set:
    """Every top-level field name of `message Marker`, read from the proto."""
    text = open(path).read()
    start = re.search(r'^message\s+Marker\s*$', text, re.M)
    body = text[start.end():]
    end = re.search(r'^\}', body, re.M)
    body = body[:end.start()]
    # `Header header = 1;`, `repeated Vector3d point = 11;` — but not the enum
    # members (`LINE_STRIP = 5;`), which have no type token before the name.
    return set(re.findall(r'^\s*(?:repeated\s+)?[A-Za-z_][\w.]*\s+(\w+)\s*=\s*\d+\s*;',
                          body, re.M))


def _top_level_fields(req: str) -> set:
    """The field names the request sets at the top level, brace-depth 0."""
    out, depth, i = set(), 0, 0
    while i < len(req):
        c = req[i]
        if c in '{[':
            depth += 1
        elif c in '}]':
            depth -= 1
        elif depth == 0:
            m = re.match(r'([A-Za-z_]\w*)\s*:', req[i:])
            if m:
                out.add(m.group(1))
                i += m.end()
                continue
        i += 1
    return out


@pytest.mark.skipif(not os.path.exists(PROTO), reason='ignition-msgs headers not installed')
def test_every_field_in_the_request_exists_in_marker_proto():
    req = M.marker_request(CIRCLE, ns='t', marker_id=1, rgba=(1, 0, 0, 1))
    assert _top_level_fields(req) <= _marker_proto_fields(PROTO)


@pytest.mark.skipif(not os.path.exists(PROTO), reason='ignition-msgs headers not installed')
def test_the_proto_has_scale_and_not_size():
    """The exact confusion that produced a silently empty marker."""
    fields = _marker_proto_fields(PROTO)
    assert 'scale' in fields
    assert 'size' not in fields


def test_no_scale_is_sent():
    """`scale` MULTIPLIES the points — it is not a line width.

    Measured in Gazebo Fortress: the same 0.25 m circle renders when no scale is
    sent and renders as nothing at all with `scale: 0.004`, because the points
    are multiplied down to a 1 mm dot. The marker is still listed in
    /marker/list either way, so only the picture shows the difference. Sending
    no scale is therefore load-bearing, not an omission to tidy up.
    """
    req = M.marker_request(CIRCLE, ns='t', marker_id=1, rgba=(1, 0, 0, 1))
    assert 'scale' not in req


def test_request_carries_every_point_and_the_right_type():
    req = M.marker_request(CIRCLE, ns='ee', marker_id=3, rgba=(0, 0, 1, 1))
    assert req[req.index('point: ['):].count('{x:') == len(CIRCLE)
    assert 'type: LINE_STRIP' in req and 'visibility: GUI' in req
    assert 'action: ADD_MODIFY' in req and 'ns: "ee"' in req and 'id: 3' in req
    # the colour goes in twice, as ambient and as diffuse, or the line renders black
    assert req.count('b: 1') == 2


def test_delete_is_sent_as_its_own_action():
    """An expiring trail ends below two vertices and must be removed.

    A LINE_STRIP that is merely not re-sent keeps its last geometry on screen
    forever, which is exactly what a trail is meant to prevent.
    """
    req = M.delete_request(ns='ee', marker_id=2)
    assert 'action: DELETE_MARKER' in req
    assert 'ns: "ee"' in req and 'id: 2' in req
    assert 'point' not in req


@pytest.mark.skipif(not os.path.exists(PROTO), reason='ignition-msgs headers not installed')
def test_the_delete_request_also_only_uses_real_fields():
    assert _top_level_fields(M.delete_request(ns='ee', marker_id=2)) <= _marker_proto_fields(PROTO)


def test_signature_of_an_empty_trace_is_distinct():
    assert M.trace_signature(TraceBuffer()) == (0,)


def test_signature_changes_when_the_trail_slides_at_constant_length():
    """The case a length-only check would miss.

    In steady state the trail gains a vertex at the tip and loses one to expiry
    in the same period: the count never moves while the curve moves entirely. A
    redraw keyed on length alone would freeze the picture.
    """
    trace = TraceBuffer(min_spacing_m=0.0, jump_reset_m=0.0)
    for i in range(5):
        trace.add((0.01 * i, 0.0, 0.0), float(i))
    before = M.trace_signature(trace)
    assert trace.expire(now=5.0, ttl=2.5) > 0        # drop the old end
    for i in range(5, 7):
        trace.add((0.01 * i, 0.0, 0.0), float(i))    # and grow at the tip
    after = M.trace_signature(trace)
    assert len(before) == len(after)
    assert before != after


def test_the_trail_empties_once_nothing_is_added():
    """`trail_seconds` after the last point the trace is gone, not merely short."""
    trace = TraceBuffer(min_spacing_m=0.0, jump_reset_m=0.0)
    for i in range(10):
        trace.add((0.01 * i, 0.0, 0.0), 0.1 * i)
    trace.expire(now=0.9 + 2.0 + 1e-6, ttl=2.0)
    assert len(trace) == 0
    assert M.trace_signature(trace) == (0,)
