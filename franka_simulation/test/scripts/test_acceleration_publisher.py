#!/usr/bin/env python3
"""Acceleration pipeline test publisher.

Publishes sinusoidal joint-acceleration commands to validate the
acceleration simulation pipeline (sim_acceleration.launch.py).

Pipeline under test:
    this_node → /sim_acceleration_bridge/accel_cmd
              → sim_acceleration_bridge (integrator: q̈→q̇)
              → /fr3_velocity_controller/commands
              → fr3_velocity_controller → Gazebo

The acceleration bridge node subscribes on its private topic ~/accel_cmd,
which resolves to /sim_acceleration_bridge/accel_cmd.

Usage:
    ros2 run franka_simulation test_acceleration_publisher
    # or via test launch:
    ros2 launch franka_simulation test_acceleration_pipeline.launch.py
"""

import sys
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

_N_JOINTS = 7
_CMD_TOPIC = '/sim_acceleration_bridge/accel_cmd'


class AccelerationTestPublisher(Node):
    def __init__(self):
        super().__init__('acceleration_test_publisher')

        self.declare_parameter('rate_hz',      50.0)
        self.declare_parameter('duration_s',   30.0)
        self.declare_parameter('amplitude',     0.3)    # rad/s²
        self.declare_parameter('frequency',     0.1)    # Hz
        self.declare_parameter('active_joint',  0)

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
            f'Acceleration test publisher started\n'
            f'  topic     : {_CMD_TOPIC}\n'
            f'  rate      : {self._rate_hz} Hz\n'
            f'  duration  : {self._duration_s} s\n'
            f'  amplitude : {self._amplitude} rad/s²\n'
            f'  joint     : {self._active_joint + 1}\n'
            f'  NOTE: sim_acceleration_bridge must be running to integrate commands')

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

        qddot = [0.0] * _N_JOINTS
        qddot[self._active_joint] = self._amplitude * math.sin(
            2.0 * math.pi * self._frequency * t)

        msg = Float64MultiArray()
        msg.data = qddot
        self._pub.publish(msg)

        if int(t) != int(t - 1.0 / self._rate_hz):
            self.get_logger().info(
                f't={t:.1f}s  qddot[{self._active_joint}]={qddot[self._active_joint]:.4f} rad/s²')

    def _publish_zeros(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * _N_JOINTS
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AccelerationTestPublisher()
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
