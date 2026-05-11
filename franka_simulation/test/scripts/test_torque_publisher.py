#!/usr/bin/env python3
"""Torque pipeline test publisher.

Publishes joint-torque commands to validate the torque simulation pipeline
(sim_torque.launch.py).

Pipeline under test:
    this_node → /fr3_effort_controller/commands → fr3_effort_controller → Gazebo

IMPORTANT — gravity in Gazebo:
    Gazebo simulates gravity internally via the physics engine.
    The robot will NOT fall when publishing zeros (Gazebo equilibrium).
    Send only the task torques — do NOT add gravity compensation.
    This is different from the real hardware where rt_torque_controller
    adds g(q) from Pinocchio.

Test phases:
  Phase 0 (0–10 s)  : publish zeros — robot should hold current pose in Gazebo
  Phase 1 (10–30 s) : small sinusoidal torque on joint 1 — robot should oscillate
  Phase 2 (30– s)   : zeros again — should stop

Usage:
    ros2 run franka_simulation test_torque_publisher
    # or via test launch:
    ros2 launch franka_simulation test_torque_pipeline.launch.py
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

_N_JOINTS = 7
_CMD_TOPIC = '/fr3_effort_controller/commands'

# Conservative amplitude — FR3 joint1 max torque is ~87 Nm
_PHASE1_AMPLITUDE = 2.0   # Nm
_PHASE1_FREQUENCY = 0.1   # Hz


class TorqueTestPublisher(Node):
    def __init__(self):
        super().__init__('torque_test_publisher')

        self.declare_parameter('rate_hz',          100.0)
        self.declare_parameter('zeros_duration_s',  10.0)
        self.declare_parameter('active_duration_s', 20.0)
        self.declare_parameter('amplitude',    _PHASE1_AMPLITUDE)
        self.declare_parameter('frequency',    _PHASE1_FREQUENCY)
        self.declare_parameter('active_joint',  0)

        self._rate_hz          = self.get_parameter('rate_hz').value
        self._zeros_duration_s = self.get_parameter('zeros_duration_s').value
        self._active_duration_s = self.get_parameter('active_duration_s').value
        self._amplitude        = self.get_parameter('amplitude').value
        self._frequency        = self.get_parameter('frequency').value
        self._active_joint     = int(self.get_parameter('active_joint').value)

        self._total_duration = self._zeros_duration_s + self._active_duration_s

        self._pub  = self.create_publisher(Float64MultiArray, _CMD_TOPIC, 10)
        self._t0   = self.get_clock().now()
        self._timer = self.create_timer(1.0 / self._rate_hz, self._cb)
        self._done = False

        self.get_logger().warn(
            'TORQUE TEST: Gazebo manages gravity — publishing zeros = robot at equilibrium')
        self.get_logger().info(
            f'Torque test publisher started\n'
            f'  topic      : {_CMD_TOPIC}\n'
            f'  rate       : {self._rate_hz} Hz\n'
            f'  Phase 0    : zeros for {self._zeros_duration_s:.0f} s (gravity equilibrium test)\n'
            f'  Phase 1    : ±{self._amplitude:.1f} Nm on joint {self._active_joint + 1}'
            f' for {self._active_duration_s:.0f} s\n'
            f'  Phase 2    : zeros (stop)')

    def _cb(self):
        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9

        if t >= self._total_duration:
            if not self._done:
                self._done = True
                self._publish_zeros()
                self.get_logger().info('Test complete — published zeros, shutting down')
                self._timer.cancel()
                rclpy.shutdown()
            return

        tau = [0.0] * _N_JOINTS

        if t >= self._zeros_duration_s:
            tr = t - self._zeros_duration_s
            tau[self._active_joint] = self._amplitude * math.sin(
                2.0 * math.pi * self._frequency * tr)

        msg = Float64MultiArray()
        msg.data = tau
        self._pub.publish(msg)

        if int(t) != int(t - 1.0 / self._rate_hz):
            phase = 'ZEROS (equilibrium)' if t < self._zeros_duration_s else 'ACTIVE (sinusoidal)'
            self.get_logger().info(
                f't={t:.1f}s [{phase}]  tau[{self._active_joint}]={tau[self._active_joint]:.3f} N·m')

    def _publish_zeros(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * _N_JOINTS
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TorqueTestPublisher()
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
