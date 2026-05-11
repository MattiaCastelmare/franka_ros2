#!/usr/bin/env python3
"""Sinusoidal joint-velocity trajectory for the velocity pipeline.

Publishes to:
    /fr3_velocity_controller/commands  (std_msgs/Float64MultiArray, 7 joints)

Each joint follows:
    q̇[i] = A[i] * sin(ω*t + φ[i])

with conservative amplitudes and a smooth startup ramp so Gazebo does not jerk.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

_N = 7
_RATE_HZ = 100.0
_RAMP_S   = 2.0   # seconds to ramp from 0 → full amplitude

# Amplitudes [rad/s] — well below FR3 limits (2.62 / 5.26)
_AMP   = [0.20, 0.15, 0.15, 0.12, 0.10, 0.10, 0.08]
_OMEGA = 0.4   # rad/s  (~0.06 Hz)
# Phase offsets give coordinated, visually interesting motion
_PHASE = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]


class VelocityCircleTrajectory(Node):
    def __init__(self):
        super().__init__('velocity_circle_trajectory')
        self._pub = self.create_publisher(
            Float64MultiArray, '/fr3_velocity_controller/commands', 10)
        self._t = 0.0
        self._dt = 1.0 / _RATE_HZ
        self._timer = self.create_timer(self._dt, self._step)
        self.get_logger().info('velocity_circle_trajectory started')

    def _step(self):
        ramp = min(self._t / _RAMP_S, 1.0)
        msg = Float64MultiArray()
        msg.data = [
            ramp * _AMP[i] * math.sin(_OMEGA * self._t + _PHASE[i])
            for i in range(_N)
        ]
        self._pub.publish(msg)
        self._t += self._dt


def main(args=None):
    rclpy.init(args=args)
    node = VelocityCircleTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send zero before shutdown to stop the robot
        stop = Float64MultiArray()
        stop.data = [0.0] * _N
        node._pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
