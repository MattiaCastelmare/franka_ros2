#!/usr/bin/env python3
"""scenario_commander: cbf_scenarios' nominal laws on /NS_1/qddot_nom (spec Phase 4).

SIM only; mutually exclusive with pentagon_qddot_commander (the stack runs with
motion_source:=none). Laws: ``simexp.scenario_laws``, bit-equal to
cbf_scenarios.run_scenario's. Before the episode it holds Q0 with the same joint
PD, which also removes the few-mrad sag Gazebo leaves at spawn. From the
episode start (/sim/episode) it runs the scenario law on episode time.

Runs at ``rate_hz`` (default 300 = qp_rate_hz): run_scenario evaluates its
nominal on every QP tick. Reads the fast joint states, like every safety-path node.
"""
import json
import os
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from simexp.scenario_laws import NominalLaw  # noqa: E402

ARM = [f'fr3_joint{i}' for i in range(1, 8)]


class ScenarioCommander(Node):
    def __init__(self):
        super().__init__('scenario_commander', namespace='NS_1')
        spec = json.loads(self.declare_parameter('nominal_json', '{}').value)
        q0 = [float(x) for x in self.declare_parameter(
            'q0', [0.0, -0.70, 0.0, -2.35, 0.0, 1.57, 0.70]).value]
        rate = float(self.declare_parameter('rate_hz', 300.0).value)
        self.law = NominalLaw(spec, q0)
        self.q = self.qd = None
        self.state, self.t_start = 0.0, None
        self.pub = self.create_publisher(Float64MultiArray, '/NS_1/qddot_nom', 10)
        self.pub_des = self.create_publisher(PointStamped, '/NS_1/ee_desired', 10)
        self.pub_act = self.create_publisher(PointStamped, '/NS_1/ee_actual', 10)
        self.create_subscription(JointState, '/NS_1/franka/joint_states', self._js, QoSProfile(depth=1))
        self.create_subscription(Float64MultiArray, '/sim/episode', self._ep, 10)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f'SIM scenario commander: {spec} at {rate:.0f} Hz, holds Q0 until '
                               f'the episode starts')

    def _js(self, msg):
        d = {n: i for i, n in enumerate(msg.name)}
        if all(n in d for n in ARM):
            self.q = np.array([msg.position[d[n]] for n in ARM])
            self.qd = np.array([msg.velocity[d[n]] for n in ARM])

    def _ep(self, msg):
        if len(msg.data) >= 2:
            self.state = msg.data[0]
            self.t_start = msg.data[1] if msg.data[1] >= 0 else None

    def _tick(self):
        if self.q is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        p_ref = flange = None
        if self.state >= 1.0 and self.t_start is not None and now >= self.t_start:
            qdd, p_ref, flange = self.law(now - self.t_start, self.q, self.qd)
        else:
            qdd = self.law.hold(self.q, self.qd)
        self.pub.publish(Float64MultiArray(data=[float(x) for x in qdd]))
        if p_ref is not None:
            stamp = self.get_clock().now().to_msg()
            for pub, p in ((self.pub_des, p_ref), (self.pub_act, flange)):
                m = PointStamped()
                m.header.stamp = stamp
                m.header.frame_id = 'fr3_link0'
                m.point.x, m.point.y, m.point.z = (float(v) for v in p)
                pub.publish(m)


def main():
    rclpy.init()
    node = ScenarioCommander()
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
