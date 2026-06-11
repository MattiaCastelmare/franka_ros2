#!/usr/bin/env python3
"""Smoke test for the decoupled cbf_safety_filter (run inside the container).

Feeds synthetic /joint_states (100 Hz), /qddot_nom (50 Hz) and
/cbf/per_link_distances (30 Hz, one close obstacle), then checks:

  1. qddot_safe published at ~ qp_rate_hz and all-finite
  2. CBF active (output deviates from passthrough when obstacle close)
  3. distance stream stopped -> filter falls back to passthrough
  4. qddot_nom stopped -> braking fallback (output ~ -k_brake * qdot = 0 here)
"""

import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from franka_msgs.msg import MultiLinkDistance, LinkDistance

from franka_experiments.nodes.cbf_safety_filter import CBFSafetyFilter, FR3_JOINTS


class Stimulus(Node):
    def __init__(self):
        super().__init__('stimulus')
        self.q = np.array([0.0, -0.4, 0.0, -1.8, 0.0, 1.6, 0.8])
        # joint1 rotation accelerates the obstacle point (x=0.3, y=0) in +y,
        # i.e. toward the human at y=+0.26 -> the CBF constraint must bind
        self.qddot_nom = np.array([5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.send_dist = True
        self.send_nom = True
        self.received = []
        self.t_recv = []

        self.pub_js = self.create_publisher(JointState, '/NS_1/joint_states', 10)
        self.pub_nom = self.create_publisher(Float64MultiArray, '/NS_1/qddot_nom', 10)
        self.pub_dist = self.create_publisher(
            MultiLinkDistance, '/cbf/per_link_distances',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            Float64MultiArray, '/NS_1/qddot_safe', self._on_safe, 10)

        self.create_timer(0.01, self._tick_js)
        self.create_timer(0.02, self._tick_nom)
        self.create_timer(1.0 / 30.0, self._tick_dist)

    def _on_safe(self, msg):
        self.received.append(np.array(msg.data))
        self.t_recv.append(time.monotonic())

    def _tick_js(self):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(FR3_JOINTS)
        m.position = self.q.tolist()
        m.velocity = [0.0] * 7
        self.pub_js.publish(m)

    def _tick_nom(self):
        if not self.send_nom:
            return
        m = Float64MultiArray()
        m.data = self.qddot_nom.tolist()
        self.pub_nom.publish(m)

    def _tick_dist(self):
        if not self.send_dist:
            return
        ld = LinkDistance()
        ld.robot_link_name = 'fr3_link5'
        ld.distance = 0.26          # inside activation band (d_safe+margin=0.35)
        ld.closest_point_robot = Point(x=0.3, y=0.0, z=0.6)
        ld.closest_point_human = Point(x=0.3, y=0.26, z=0.6)
        ld.confidence = 0.9
        ld.valid = True
        m = MultiLinkDistance()
        m.header.stamp = self.get_clock().now().to_msg()
        m.links = [ld]
        self.pub_dist.publish(m)


def main():
    rclpy.init()
    flt = CBFSafetyFilter()
    stim = Stimulus()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(flt)
    ex.add_node(stim)

    def run(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            ex.spin_once(timeout_sec=0.02)

    failures = []

    # Phase 1: CBF active
    run(3.0)
    n1 = len(stim.received)
    if n1 < 10:
        failures.append(f'phase1: only {n1} outputs')
    else:
        dt = np.diff(stim.t_recv[-min(200, n1):])
        rate = 1.0 / np.mean(dt)
        out = np.array(stim.received[-50:])
        dev = np.abs(out - stim.qddot_nom).max()
        print(f'phase1: n={n1}  rate={rate:.0f} Hz  finite={np.isfinite(out).all()}  '
              f'max|out-nom|={dev:.3f}')
        if not (150 <= rate <= 260):
            failures.append(f'phase1: rate {rate:.0f} Hz not ~200')
        if not np.isfinite(out).all():
            failures.append('phase1: non-finite output')
        if dev < 1e-3:
            failures.append('phase1: CBF constraint did not bind (expected clipping)')

    # Phase 2: stop distances -> passthrough after distance_timeout (0.5 s)
    stim.send_dist = False
    run(1.5)
    out = np.array(stim.received[-20:])
    dev = np.abs(out - stim.qddot_nom).max()
    print(f'phase2: max|out-nom|={dev:.2e} (expect ~0, passthrough)')
    if dev > 1e-4:
        failures.append(f'phase2: not passthrough after stale distances (dev={dev:.2e})')

    # Phase 3: stop nominal -> braking fallback (qdot=0 -> output ~0)
    stim.send_nom = False
    run(1.5)
    out = np.array(stim.received[-20:])
    print(f'phase3: max|out|={np.abs(out).max():.2e} (expect ~0, braking with qdot=0)')
    if np.abs(out).max() > 1e-4:
        failures.append('phase3: braking fallback not applied')

    flt.destroy_node()
    stim.destroy_node()
    rclpy.shutdown()

    if failures:
        print('FAIL:\n  ' + '\n  '.join(failures))
        raise SystemExit(1)
    print('SMOKE TEST PASSED')


if __name__ == '__main__':
    main()
