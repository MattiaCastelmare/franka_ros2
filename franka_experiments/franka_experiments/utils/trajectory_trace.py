"""Accumulate two EE traces and render them as RViz markers.

Used by ``nodes/trajectory_visualization_node`` (RViz markers) and by
``nodes/trajectory_overlay_node`` (the same two traces drawn on the camera
image). Split out for the same reason ``utils/visualization.py`` is split out of
``real_time_distance``: the decisions worth testing — when a point is kept, when
the trace is cut, how long it may grow, when it expires — are pure functions of
the numbers, and testing them should not require a running graph.

WHAT THE OVERLAY IS FOR
-----------------------
Red is ``P(s)``, the path the commander is asking for. Blue is where the end
effector actually is. Both come from ``pentagon_qddot_commander``, computed on
the same tick from the same forward kinematics, so a gap between them is a
tracking error and nothing else — not a timestamp offset between two pipelines,
which at 0.1 m/s draws 3 mm of fictitious deviation for every 33 ms of skew.

The two lines are the same width on purpose. Nesting a thin line inside a thick
one looks tidy until you realise the thick one wins the depth test everywhere
they overlap, so one of the two is simply never visible. Equal widths z-fight
where the traces coincide, which reads as a single shimmering line — an honest
rendering of "these are the same curve" — and separate cleanly where they do
not. The deviation segment and the error label exist so the answer does not
depend on reading a shimmer.
"""

from __future__ import annotations

import bisect
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

#: Marker ids inside the namespace. Fixed so each republish REPLACES its
#: predecessor instead of piling up: RViz keys markers on (ns, id).
ID_DESIRED, ID_ACTUAL, ID_DEVIATION, ID_LABEL = 0, 1, 2, 3


class TraceBuffer:
    """A polyline that grows at the tip and is cut when the source teleports.

    Every stored vertex carries the timestamp it was offered with, so a viewer
    that wants a trail of fixed DURATION rather than fixed length can drop the
    old end with :meth:`expire`. The markers node never calls it and is
    unaffected; the image overlay calls it on every frame, which is what makes
    the two curves read as two short comets chasing each other instead of two
    ever-growing scribbles over the robot.

    Args:
        max_points: hard cap on stored vertices; the oldest are dropped. A
            LINE_STRIP is re-serialised on every publish, so this is the knob
            that bounds both the message size and the render cost, not a
            cosmetic history length.
        min_spacing_m: a new point is kept only this far from the last kept one.
            At 100 Hz and 0.1 m/s consecutive samples are 1 mm apart, so without
            this the cap above is reached in a few seconds of a trajectory whose
            shape needs a hundredth of those points to be legible.
        jump_reset_m: a step larger than this CUTS the trace instead of
            extending it. A commander restart, or a run that begins with the arm
            somewhere else, would otherwise be drawn as a straight line across
            space the end effector never travelled — a line that looks exactly
            like a catastrophic deviation. 0 disables the cut.
    """

    def __init__(self, *, max_points: int = 4000, min_spacing_m: float = 0.002,
                 jump_reset_m: float = 0.10) -> None:
        if max_points < 2:
            raise ValueError(f'max_points must be >= 2, got {max_points}')
        if min_spacing_m < 0.0:
            raise ValueError(f'min_spacing_m must be >= 0, got {min_spacing_m}')
        self.max_points = int(max_points)
        self.min_spacing_m = float(min_spacing_m)
        self.jump_reset_m = float(jump_reset_m)
        self._pts: List[np.ndarray] = []
        self._t: List[float] = []
        self.cuts = 0            #: how many times the trace was cut, DIAGNOSTIC
        #: Last VALID point offered and its timestamp, whether or not it was
        #: stored. The spacing filter rejects everything while the arm holds
        #: still, so a drawer that took its marker from ``last`` would watch the
        #: dot vanish the moment the motion stops — exactly when the operator is
        #: looking at where it stopped.
        self.tip: Optional[np.ndarray] = None
        self.tip_time: float = float('-inf')

    def __len__(self) -> int:
        return len(self._pts)

    @property
    def points(self) -> List[np.ndarray]:
        """The stored vertices, oldest first. Live list — do not mutate."""
        return self._pts

    @property
    def times(self) -> List[float]:
        """Timestamp of each stored vertex, same order. Live list."""
        return self._t

    @property
    def last(self) -> Optional[np.ndarray]:
        return self._pts[-1] if self._pts else None

    def clear(self) -> None:
        self._pts.clear()
        self._t.clear()
        self.tip = None
        self.tip_time = float('-inf')

    def add(self, p: Sequence[float], t: float = 0.0) -> bool:
        """Offer a point stamped *t*. Returns True when it was stored.

        Order of the three tests matters. The jump check comes BEFORE the
        spacing check because a teleport is also, trivially, far enough away to
        pass the spacing test, and cutting has to win over extending.

        A rejected point still updates :attr:`tip`: "too close to the last one
        to be worth a vertex" is a statement about the POLYLINE, not about where
        the end effector is.
        """
        q = np.asarray(p, dtype=float)
        if q.shape != (3,) or not np.all(np.isfinite(q)):
            return False
        self.tip = q
        self.tip_time = float(t)
        if self._pts:
            d = float(np.linalg.norm(q - self._pts[-1]))
            if self.jump_reset_m > 0.0 and d > self.jump_reset_m:
                self._pts.clear()
                self._t.clear()
                self.cuts += 1
            elif d < self.min_spacing_m:
                return False
        self._pts.append(q)
        self._t.append(float(t))
        if len(self._pts) > self.max_points:
            cut = len(self._pts) - self.max_points
            del self._pts[:cut]
            del self._t[:cut]
        return True

    def expire(self, now: float, ttl: float) -> int:
        """Drop vertices older than *ttl* seconds. Returns how many went.

        Only the OLD end is dropped, and only by time: the stamps are
        non-decreasing (one publisher, one clock), so the vertices to remove are
        always a prefix and bisect finds the boundary without walking the list.

        Strictly older: the vertex sitting exactly on the boundary is kept, and
        the fade renders it at alpha zero. That is what makes the tail end
        transparent instead of ending on the last visible step.

        ``ttl <= 0`` disables expiry rather than clearing the trace — a trail
        length of zero is a configuration mistake, and a viewer that silently
        showed nothing would be the hardest possible way to notice it.
        """
        if ttl <= 0.0 or not self._t:
            return 0
        cut = bisect.bisect_left(self._t, float(now) - float(ttl))
        if cut <= 0:
            return 0
        del self._pts[:cut]
        del self._t[:cut]
        return cut


def _points(xs: Iterable[Sequence[float]]):
    from geometry_msgs.msg import Point
    out = []
    for p in xs:
        pt = Point()
        pt.x, pt.y, pt.z = float(p[0]), float(p[1]), float(p[2])
        out.append(pt)
    return out


def _strip(frame_id, stamp, ns, mid, pts, width, rgba):
    from visualization_msgs.msg import Marker
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = ns
    m.id = mid
    m.type = Marker.LINE_STRIP
    # Fewer than two vertices is not a degenerate line, it is no line: RViz
    # warns on the first and draws nothing either way, so say DELETE and mean it.
    m.action = Marker.ADD if len(pts) >= 2 else Marker.DELETE
    m.pose.orientation.w = 1.0
    m.scale.x = float(width)
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    m.points = _points(pts) if m.action == Marker.ADD else []
    return m


def trace_markers(
    *,
    frame_id: str,
    stamp,
    desired: Sequence[Sequence[float]],
    actual: Sequence[Sequence[float]],
    ns: str = 'ee_trace',
    desired_width: float = 0.002,
    actual_width: float = 0.002,
    desired_rgba: Tuple[float, float, float, float] = (0.90, 0.10, 0.10, 1.0),
    actual_rgba: Tuple[float, float, float, float] = (0.10, 0.35, 0.95, 1.0),
    deviation: bool = True,
    deviation_rgba: Tuple[float, float, float, float] = (1.0, 0.75, 0.0, 0.9),
    deviation_width: float = 0.004,
    label: bool = True,
    label_height: float = 0.03,
    label_min_error_m: float = 0.002,
):
    """Build the MarkerArray for one frame.

    ``deviation`` adds a segment between the two current tips and ``label`` puts
    the distance between them, in millimetres, at its midpoint. Both are about
    the same question — how far off is it RIGHT NOW — which the two traces
    answer only where they happen to be separated by more than a line width.

    ``label_min_error_m`` keeps the text out of the way while the arm is
    tracking: below it the label is deleted rather than drawn at 1 mm, which
    would otherwise sit on the trace forever announcing that nothing is wrong.
    """
    from visualization_msgs.msg import Marker, MarkerArray

    arr = MarkerArray()
    arr.markers.append(_strip(frame_id, stamp, ns, ID_DESIRED, desired,
                              desired_width, desired_rgba))
    arr.markers.append(_strip(frame_id, stamp, ns, ID_ACTUAL, actual,
                              actual_width, actual_rgba))

    p_d = np.asarray(desired[-1], float) if len(desired) else None
    p_a = np.asarray(actual[-1], float) if len(actual) else None
    err = (float(np.linalg.norm(p_d - p_a))
           if p_d is not None and p_a is not None else None)

    seg = _strip(frame_id, stamp, ns, ID_DEVIATION,
                 [p_d, p_a] if (deviation and err is not None) else [],
                 deviation_width, deviation_rgba)
    arr.markers.append(seg)

    txt = Marker()
    txt.header.frame_id = frame_id
    txt.header.stamp = stamp
    txt.ns = ns
    txt.id = ID_LABEL
    txt.type = Marker.TEXT_VIEW_FACING
    show = label and err is not None and err >= label_min_error_m
    txt.action = Marker.ADD if show else Marker.DELETE
    if show:
        mid = 0.5 * (p_d + p_a)
        txt.pose.position.x = float(mid[0])
        txt.pose.position.y = float(mid[1])
        txt.pose.position.z = float(mid[2]) + label_height
        txt.pose.orientation.w = 1.0
        txt.scale.z = float(label_height)
        txt.color.r, txt.color.g, txt.color.b, txt.color.a = deviation_rgba
        txt.text = f'{err * 1000.0:.0f} mm'
    arr.markers.append(txt)
    return arr
