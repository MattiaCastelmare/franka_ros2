#!/usr/bin/env python3
"""Smoke test for the decoupled cbf_safety_filter (run inside the container).

Feeds synthetic /joint_states (100 Hz), /qddot_nom (50 Hz) and
/cbf/per_link_distances (30 Hz, one close obstacle), then checks:

  1. qddot_safe published at ~ qp_rate_hz and all-finite
  2. CBF active (output deviates from passthrough when obstacle close)
  3. distance stream stopped -> BRAKING fallback, not passthrough
  4. qddot_nom stopped -> braking fallback (output ~ -k_brake * qdot = 0 here)
"""

import threading
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

        # joint_states_FAST, matching topics['joint_states_fast'] in
        # fr3_control.yaml. The filter stopped subscribing to /NS_1/joint_states
        # when that key was added — that topic is a 30 Hz Python republisher of
        # cached values — and this script kept publishing to the old one, so the
        # filter never saw a joint state, `_qp_tick` returned early on every
        # tick and the run failed with an empty output array instead of saying
        # what was wrong.
        self.pub_js = self.create_publisher(
            JointState, '/NS_1/franka/joint_states', 10)
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

    # A real spin, on its own thread. `spin_once` in a loop services ONE work
    # item per call, and with two nodes, three timers and two subscriptions
    # between them it could not keep the 100 Hz joint-state timer fed, let
    # alone deliver the output messages this script counts.
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()

    def run(sec):
        time.sleep(sec)

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
        # Against the CONFIGURED rate rather than a literal: this said
        # "~200 Hz" long after qp_rate_hz became 100, so the script could only
        # ever fail on a correctly running filter.
        want = float(flt.P.qp_rate_hz)
        if not (0.75 * want <= rate <= 1.3 * want):
            failures.append(f'phase1: rate {rate:.0f} Hz not ~{want:.0f}')
        if not np.isfinite(out).all():
            failures.append('phase1: non-finite output')
        if dev < 1e-3:
            failures.append('phase1: CBF constraint did not bind (expected clipping)')

    # Phase 2: stop distances -> BRAKING after distance_timeout (0.5 s).
    #
    # This used to expect PASSTHROUGH, and passthrough is what the node did
    # when the script was written. It no longer does, deliberately: a failure
    # of the channel that feeds the barrier must degrade toward more
    # conservative, so the nominal becomes -k_brake*q̇ and `fault` is raised
    # (see the `distance stale` branch of _qp_tick). With q̇ = 0 here that is
    # an output of ~0 against a nominal of 5.0 — which the old assertion read
    # as a failure, so this script could only pass on the unsafe behaviour.
    stim.send_dist = False
    run(1.5)
    if not stim.received:
        print('FAIL:\n  no qddot_safe at all — check the topic names against '
              'the `topics:` block of fr3_control.yaml')
        raise SystemExit(1)
    out = np.array(stim.received[-20:])
    print(f'phase2: max|out|={np.abs(out).max():.2e} '
          f'(expect ~0: braking with qdot=0, NOT passthrough)')
    if np.abs(out).max() > 1e-4:
        failures.append('phase2: no braking fallback after stale distances '
                        f'(max|out|={np.abs(out).max():.2e})')

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
