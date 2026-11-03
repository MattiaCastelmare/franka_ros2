#!/usr/bin/env python3
"""gz_trajectory_markers: the desired and the true EE path, drawn inside Gazebo.

SIM only; passive. The two trails ``trajectory_overlay_node`` draws on the real
robot's camera image, drawn instead in the Gazebo 3-D scene:

* **red**  — ``P(s)``, the Cartesian point ``pentagon_qddot_commander`` is
  asking for at this instant (``/NS_1/ee_desired``);
* **blue** — where the end effector actually is (``/NS_1/ee_actual``).

Both die of old age after ``trail_seconds``, so what you watch is two short
comets chasing each other, not two ever-growing scribbles over the robot. Where
the blue one sits on the red one the arm is tracking; where it bulges away it is
not.

The accumulation is ``franka_experiments.utils.trajectory_trace.TraceBuffer``,
the same class and the same ``trail_seconds`` default (2 s) the real robot's
overlay uses — not a reimplementation. It is what decides when a point is worth
a vertex (``min_spacing_m``), when the trail is CUT rather than extended
(``jump_reset_m``: a commander restart would otherwise draw a straight line
across space the end effector never travelled), and when a vertex has expired.

WHY BOTH POINTS COME FROM THE COMMANDER
---------------------------------------
Same reason the real overlay does it: the commander publishes both from one tick
and one forward-kinematics evaluation. Re-deriving the true point from TF here
would compare two pipelines with different latencies — the TF tree is fed by a
30 Hz republisher, so at 0.1 m/s the blue line would trail the red one by ~3 mm
of pure skew, indistinguishable from the tracking error this node exists to show.

Vertices are aged by the MESSAGE stamp, not by arrival, for the same reason: the
commander stamps from the clock this node reads, so a vertex's age stays honest
even when a redraw is late.

NO PER-VERTEX FADE
------------------
The image overlay fades its trail with ``trail_alphas``. It cannot be done here:
``ignition.msgs.Marker`` carries ONE ``material`` for the whole strip and has no
per-point colour, so the trail is a hard window — full colour inside
``trail_seconds``, gone outside it. Faking the fade would mean one marker and
one ``ign service`` process per segment, and at ~230 ms per call that costs more
than the redraw period.

FRAME
-----
The points are in ``fr3_link0`` (the commander's ``ee_track_frame``) and the
markers are in the Gazebo world frame. ``gz_fr3_plant.launch.py`` spawns the
model with ``ros_gz_sim create`` and no ``-x/-y/-z``, i.e. at the world origin
with identity orientation, and the FR3 URDF roots at ``base`` — coincident with
``fr3_link0``. So the two frames are the same and the points are passed through
unchanged. ``frame_offset_xyz`` exists for a world that moves the robot.

HOW THE MARKERS GET THERE
-------------------------
Gazebo's ``MarkerManager`` GUI plugin serves ``/marker`` over ign-transport,
which has no Python bindings here, so each redraw shells out to ``ign service``.
Two consequences the implementation has to respect:

* MarkerManager never sends its ``ignition.msgs.Empty`` reply, so the CLI always
  reports "Service call timed out" and returns 0 — while the marker IS drawn.
  The call is therefore fire-and-forget: the exit status says nothing and is not
  read, and ``--timeout`` is low only to keep the process short-lived.
* a call costs ~230 ms of process startup and ign-transport discovery, far more
  than a tick. It runs detached, never on the executor, and a redraw is SKIPPED
  while the previous one is still in flight rather than queued behind it.

Because the GUI plugin owns the markers, this node needs ``gui:=true`` and the
same ``IGN_PARTITION`` as the server, and it draws nothing in a headless run.

Parameters
----------
  ee_desired_topic / ee_actual_topic   PointStamped, from the commander
  trail_seconds    how long a vertex lives (default 2.0, as on the real robot)
  redraw_hz        how often the two polylines are re-sent (default 5)
  min_spacing_m / max_points / jump_reset_m   passed to TraceBuffer
  ns / desired_id / actual_id          marker namespace and ids
  frame_offset_xyz added to every point: fr3_link0 -> world, if a world moves it
"""
import os
import subprocess
import sys

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.time import Time

from franka_experiments.utils.trajectory_trace import TraceBuffer

#: ``ignition.msgs.Marker.Type.LINE_STRIP`` / ``Visibility.GUI``, by name: the
#: text-format parser takes the enum labels.
_LINE_STRIP = 'LINE_STRIP'
_GUI = 'GUI'


def marker_request(points, *, ns: str, marker_id: int, rgba,
                   marker_type: str = _LINE_STRIP) -> str:
    """The ``ignition.msgs.Marker`` text-format request for one polyline.

    Every field name here is one the proto actually has (``ignition/msgs/marker.proto``:
    action, ns, id, type, visibility, scale, material, point). A misspelt one is
    a silent, measured failure mode, not an exception: protobuf stops at the bad
    field and ign-transport sends the message it parsed SO FAR, so the marker is
    created and listed while every field after the typo — ``point`` among them,
    since it is last — is dropped, and the line never appears. ``size`` instead
    of ``scale`` fails exactly this way. Hence the field names are pinned by
    ``test_gz_trajectory_markers``.

    NO ``scale`` IS SENT, DELIBERATELY. It is tempting to read it as a line
    width; it is not. Gazebo applies it as the visual's scale, i.e. it MULTIPLIES
    the point coordinates, so a ``scale`` of 0.004 shrank a 0.25 m circle to a
    1 mm dot that was measured to render as nothing at all — markers present in
    ``/marker/list``, no line on screen. Omitted, the points are used as given.
    Fortress draws a LINE_STRIP one pixel wide and nothing here changes that.
    """
    r, g, b, a = rgba
    mat = (f'ambient: {{r: {r:g}, g: {g:g}, b: {b:g}, a: {a:g}}}, '
           f'diffuse: {{r: {r:g}, g: {g:g}, b: {b:g}, a: {a:g}}}')
    pts = ', '.join(f'{{x: {x:.6g}, y: {y:.6g}, z: {z:.6g}}}' for x, y, z in points)
    return (f'action: ADD_MODIFY, ns: "{ns}", id: {int(marker_id)}, '
            f'type: {marker_type}, visibility: {_GUI}, '
            f'material: {{{mat}}}, point: [{pts}]')


def delete_request(*, ns: str, marker_id: int) -> str:
    """Remove one marker.

    Needed because an expiring trail ENDS at fewer than two vertices, and a
    LINE_STRIP that is simply not re-sent keeps its last geometry on screen. Stop
    publishing instead of deleting and the final 2 s of motion stay painted over
    the robot for the rest of the run — the one thing a trail is supposed to
    prevent.
    """
    return f'action: DELETE_MARKER, ns: "{ns}", id: {int(marker_id)}'


def trace_signature(trace) -> tuple:
    """What has to change before a redraw is worth 230 ms of process.

    Length alone is not enough: a trail in steady state gains a vertex at the
    tip and loses one to expiry in the same period, so the count is unchanged
    while the curve has moved entirely. The timestamps of the two ends do change,
    and they are what the buffer is keyed on.
    """
    n = len(trace)
    if n == 0:
        return (0,)
    return (n, trace.times[0], trace.times[-1])


class GzTrajectoryMarkers(Node):
    def __init__(self):
        super().__init__('gz_trajectory_markers')
        dp = self.declare_parameter
        des_topic = str(dp('ee_desired_topic', '/NS_1/ee_desired').value)
        act_topic = str(dp('ee_actual_topic', '/NS_1/ee_actual').value)
        self._ns = str(dp('ns', 'ee_trajectory').value)
        self._des_id = int(dp('desired_id', 1).value)
        self._act_id = int(dp('actual_id', 2).value)
        self._ttl = float(dp('trail_seconds', 2.0).value)
        self._des_rgba = [float(v) for v in dp('desired_rgba', [0.9, 0.1, 0.1, 1.0]).value]
        self._act_rgba = [float(v) for v in dp('actual_rgba', [0.1, 0.35, 0.95, 1.0]).value]
        self._offset = [float(v) for v in dp('frame_offset_xyz', [0.0, 0.0, 0.0]).value]
        redraw_hz = float(dp('redraw_hz', 5.0).value)
        self._timeout_ms = int(dp('service_timeout_ms', 50).value)

        buf = dict(max_points=int(dp('max_points', 4000).value),
                   min_spacing_m=float(dp('min_spacing_m', 0.002).value),
                   jump_reset_m=float(dp('jump_reset_m', 0.10).value))
        self._des = TraceBuffer(**buf)
        self._act = TraceBuffer(**buf)
        self._sent: dict = {}
        self._inflight: list = []

        self.create_subscription(PointStamped, des_topic, self._on_desired, 10)
        self.create_subscription(PointStamped, act_topic, self._on_actual, 10)
        self.create_timer(1.0 / max(redraw_hz, 0.1), self._redraw)
        self.get_logger().info(
            f'Drawing Gazebo markers ns="{self._ns}": desired(id {self._des_id}, red) '
            f'from {des_topic}, actual(id {self._act_id}, blue) from {act_topic}; '
            f'trail {self._ttl:g} s, redraw {redraw_hz:g} Hz, '
            f'spacing {buf["min_spacing_m"] * 1e3:.0f} mm. '
            f'Needs gui:=true — MarkerManager is a GUI plugin.')

    # ── Inputs ──────────────────────────────────────────────────────────────

    def _stamp_s(self, msg) -> float:
        """Seconds for the trail clock, from the message when it carries one.

        The commander stamps from the same ROS clock this node reads, so using
        its stamp keeps a vertex's age honest even when a redraw is late. An
        unstamped message falls back to now, rather than to t=0, which would
        expire the whole history the instant it arrived.
        """
        t = Time.from_msg(msg.header.stamp).nanoseconds
        return (t if t > 0 else self.get_clock().now().nanoseconds) * 1e-9

    def _point(self, msg) -> tuple:
        return (msg.point.x + self._offset[0],
                msg.point.y + self._offset[1],
                msg.point.z + self._offset[2])

    def _on_desired(self, msg) -> None:
        self._des.add(self._point(msg), self._stamp_s(msg))

    def _on_actual(self, msg) -> None:
        self._act.add(self._point(msg), self._stamp_s(msg))

    # ── Output ──────────────────────────────────────────────────────────────

    def _send(self, req: str) -> None:
        """Hand one marker request to ``ign service`` and do not wait for it.

        stdout/stderr go to DEVNULL because the CLI always prints its timeout
        (MarkerManager does not reply) and that line is not a failure.
        """
        self._inflight.append(subprocess.Popen(
            ['ign', 'service', '-s', '/marker',
             '--reqtype', 'ignition.msgs.Marker', '--reptype', 'ignition.msgs.Empty',
             '--timeout', str(self._timeout_ms), '--req', req],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))

    def _redraw(self) -> None:
        self._inflight = [p for p in self._inflight if p.poll() is None]
        if self._inflight:
            return                      # a redraw is still in flight
        now = self.get_clock().now().nanoseconds * 1e-9
        for trace, mid, rgba in ((self._des, self._des_id, self._des_rgba),
                                 (self._act, self._act_id, self._act_rgba)):
            trace.expire(now, self._ttl)
            sig = trace_signature(trace)
            if sig == self._sent.get(mid):
                continue
            self._sent[mid] = sig
            if len(trace) >= 2:
                self._send(marker_request(trace.points, ns=self._ns,
                                          marker_id=mid, rgba=rgba))
            else:
                self._send(delete_request(ns=self._ns, marker_id=mid))


def main():
    rclpy.init()
    node = GzTrajectoryMarkers()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
