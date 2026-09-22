#!/usr/bin/env python3
"""TrajectoryVisualizationNode — the desired path and the real one, side by side.

Draws two polylines as RViz markers:

* **red**  — ``P(s)``, the Cartesian path ``pentagon_qddot_commander`` is asking
  for at this instant;
* **blue** — where the end effector actually is.

Where they coincide the arm is tracking. Where they separate it is not, and the
amber segment between the two tips plus a millimetre label say by how much.

WHY THE POINTS COME FROM THE COMMANDER
--------------------------------------
Both topics are published by the commander, from the same tick and the same
forward-kinematics evaluation. Deriving either one here — the actual from TF,
the desired from ``q_des_state`` — would have been less code and a worse answer:

* TF and the commander are two pipelines with different latencies. The TF tree
  is fed by a 30 Hz republisher, so at 0.1 m/s the blue line would trail the red
  one by ~3 mm of pure skew. Small, and indistinguishable from the thing this
  node exists to measure.
* ``q_des_state`` carries ``q_d``, the JOINT reference, which the commander's
  anti-windup snaps onto the measured state whenever the Cartesian error grows
  past ``hard_reset_thr``. A red line drawn from ``FK(q_d)`` would therefore
  jump onto the blue one exactly when tracking is worst.

Subscriptions
-------------
  <ee_desired_topic>   PointStamped — P(s), the commanded Cartesian point
  <ee_actual_topic>    PointStamped — the measured EE position

Published
---------
  <marker_topic>       MarkerArray  — four markers in one namespace:
                                      the two traces, the deviation segment and
                                      the error label

Rendering runs on its own timer, not on the subscriptions: the commander ticks
at 100 Hz and a LINE_STRIP is re-serialised whole on every publish, so pushing a
MarkerArray per incoming point would spend most of the CPU re-sending a curve
nobody's eye can follow. The traces accumulate in the callbacks; the timer draws.
"""

from __future__ import annotations

import threading

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray

from franka_experiments.utils.cbf_utils import load_robot_config
from franka_experiments.utils.node_runtime import run_node_main
from franka_experiments.utils.params import declare_float, declare_int, declare_str
from franka_experiments.utils.trajectory_trace import TraceBuffer, trace_markers


class TrajectoryVisualizationNode(Node):
    """Accumulate the two EE traces and republish them as markers."""

    def __init__(self):
        super().__init__('trajectory_visualization')
        self.done = False

        cfg = load_robot_config('control')
        topics = cfg['topics']

        des_topic = declare_str(
            self, 'ee_desired_topic', topics.get('ee_desired', '/NS_1/ee_desired'))
        act_topic = declare_str(
            self, 'ee_actual_topic', topics.get('ee_actual', '/NS_1/ee_actual'))
        mrk_topic = declare_str(
            self, 'marker_topic',
            topics.get('trajectory_markers', '/NS_1/trajectory_markers'))

        rate = declare_float(self, 'publish_rate_hz', 20.0,
                             positive=True, maximum=200.0)
        # 5 mm: wide enough to read at a metre without leaning into the
        # screen, narrow enough that a 1 cm deviation is still a visible gap
        # rather than two lines touching. Below this the traces are there but
        # nobody sees them, which is the same as not drawing them.
        self._w_des = declare_float(self, 'desired_line_width', 0.005,
                                    positive=True, maximum=0.05)
        self._w_act = declare_float(self, 'actual_line_width', 0.005,
                                    positive=True, maximum=0.05)
        self._label_min = declare_float(self, 'label_min_error_m', 0.002,
                                        minimum=0.0, maximum=1.0)
        max_points = declare_int(self, 'max_points', 4000,
                                 minimum=2, maximum=200000)
        spacing = declare_float(self, 'min_point_spacing_m', 0.002,
                                minimum=0.0, maximum=1.0)
        jump = declare_float(self, 'jump_reset_m', 0.10,
                             minimum=0.0, maximum=10.0)

        # One buffer each, same settings: the two curves are meant to be
        # compared, and decimating them differently would put the red and blue
        # vertices at different arc lengths — which shows up as a sawtooth
        # between two curves that are in fact identical.
        self._des = TraceBuffer(max_points=max_points, min_spacing_m=spacing,
                                jump_reset_m=jump)
        self._act = TraceBuffer(max_points=max_points, min_spacing_m=spacing,
                                jump_reset_m=jump)
        self._frame = ''
        self._lock = threading.Lock()

        self._pub = self.create_publisher(MarkerArray, mrk_topic, 1)
        self.create_subscription(PointStamped, des_topic,
                                 self._on_desired, 10)
        self.create_subscription(PointStamped, act_topic,
                                 self._on_actual, 10)
        self.create_timer(1.0 / rate, self._draw)

        self.get_logger().info(
            f'trajectory_visualization ready\n'
            f'  desired (red)  ← {des_topic}\n'
            f'  actual  (blue) ← {act_topic}\n'
            f'  markers        → {mrk_topic} at {rate:.0f} Hz\n'
            f'  widths {self._w_des:.3f}/{self._w_act:.3f} m, '
            f'spacing {spacing:.3f} m, cap {max_points} pts, '
            f'cut above {jump:.2f} m')

    # ── Subscriptions ────────────────────────────────────────────────────

    @staticmethod
    def _xyz(msg: PointStamped):
        return (msg.point.x, msg.point.y, msg.point.z)

    def _on_desired(self, msg: PointStamped) -> None:
        with self._lock:
            # The frame comes from the publisher rather than from a parameter of
            # this node: the commander is the one that knows which frame it
            # computed in, and a second opinion here could only ever disagree.
            self._frame = msg.header.frame_id
            self._des.add(self._xyz(msg))

    def _on_actual(self, msg: PointStamped) -> None:
        with self._lock:
            if not self._frame:
                self._frame = msg.header.frame_id
            self._act.add(self._xyz(msg))

    # ── Render ───────────────────────────────────────────────────────────

    def _draw(self) -> None:
        with self._lock:
            frame = self._frame
            # Copies: trace_markers walks both lists while the callbacks may be
            # appending to them, and a list that grows mid-iteration is exactly
            # the kind of intermittent failure that never reproduces.
            des = list(self._des.points)
            act = list(self._act.points)
        if not frame or (not des and not act):
            return
        self._pub.publish(trace_markers(
            frame_id=frame, stamp=self.get_clock().now().to_msg(),
            desired=des, actual=act,
            desired_width=self._w_des, actual_width=self._w_act,
            label_min_error_m=self._label_min))

    def request_stop(self) -> None:
        self.done = True


def main(args=None):
    run_node_main(TrajectoryVisualizationNode, args=args)


if __name__ == '__main__':
    main()
