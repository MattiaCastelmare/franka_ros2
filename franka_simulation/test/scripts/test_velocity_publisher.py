#!/usr/bin/env python3
"""Velocity pipeline test publisher.

Publishes sinusoidal joint-velocity commands to validate the
velocity simulation pipeline (sim_velocity.launch.py).

Pipeline under test:
    this_node → /fr3_velocity_controller/commands → fr3_velocity_controller → Gazebo

Usage:
    ros2 run franka_simulation test_velocity_publisher
    # or via test launch:
    ros2 launch franka_simulation test_velocity_pipeline.launch.py
"""

import sys
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

_N_JOINTS = 7
_CMD_TOPIC = '/fr3_velocity_controller/commands'


class VelocityTestPublisher(Node):
    def __init__(self):
        super().__init__('velocity_test_publisher')

        self.declare_parameter('rate_hz',     50.0)
        self.declare_parameter('duration_s',  30.0)
        self.declare_parameter('amplitude',    0.15)   # rad/s
        self.declare_parameter('frequency',    0.1)    # Hz — one full cycle per 10 s
        self.declare_parameter('active_joint', 0)      # 0-based joint index to move

        self._rate_hz     = self.get_parameter('rate_hz').value
        self._duration_s  = self.get_parameter('duration_s').value
        self._amplitude   = self.get_parameter('amplitude').value
        self._frequency   = self.get_parameter('frequency').value
        self._active_joint = int(self.get_parameter('active_joint').value)

        self._pub = self.create_publisher(Float64MultiArray, _CMD_TOPIC, 10)
        self._t0  = self.get_clock().now()
        self._timer = self.create_timer(1.0 / self._rate_hz, self._cb)
        self._done = False

        self.get_logger().info(
            f'Velocity test publisher started\n'
            f'  topic     : {_CMD_TOPIC}\n'
            f'  rate      : {self._rate_hz} Hz\n'
            f'  duration  : {self._duration_s} s\n'
            f'  amplitude : {self._amplitude} rad/s\n'
            f'  joint     : {self._active_joint + 1}')

    def _cb(self):
        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9

        if t >= self._duration_s:
            if not self._done:
                self._done = True
                self._publish_zeros()
                self.get_logger().info('Test complete — published zeros, shutting down')
                self._timer.cancel()
                rclpy.shutdown()
            return

        qdot = [0.0] * _N_JOINTS
        qdot[self._active_joint] = self._amplitude * math.sin(
            2.0 * math.pi * self._frequency * t)

        msg = Float64MultiArray()
        msg.data = qdot
        self._pub.publish(msg)

        if int(t) != int(t - 1.0 / self._rate_hz):
            remaining = self._duration_s - t
            self.get_logger().info(
                f't={t:.1f}s  qdot[{self._active_joint}]={qdot[self._active_joint]:.4f} rad/s'
                f'  (remaining: {remaining:.0f} s)')

    def _publish_zeros(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * _N_JOINTS
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._publish_zeros()
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
