#!/usr/bin/env python3
"""Figure-8 joint-velocity trajectory for the velocity pipeline.

Publishes to:
    /fr3_velocity_controller/commands  (std_msgs/Float64MultiArray, 7 joints)

Uses two frequencies (ω₁, 2·ω₁) with per-joint phases to produce a
Lissajous/figure-8 pattern clearly visible in RViz/Gazebo.

    q̇[i] = A[i] * sin(ω₁*t + φ[i])          (odd joints)
    q̇[i] = A[i] * sin(2*ω₁*t + φ[i])        (even joints)
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

_N = 7
_RATE_HZ = 100.0
_RAMP_S   = 2.0

_AMP   = [0.18, 0.14, 0.14, 0.12, 0.10, 0.10, 0.08]
_OMEGA = 0.35   # base frequency [rad/s]
_PHASE = [0.0, math.pi/4, math.pi/2, 3*math.pi/4, math.pi, 5*math.pi/4, 3*math.pi/2]


class VelocityFigure8Trajectory(Node):
    def __init__(self):
        super().__init__('velocity_figure8_trajectory')
        self._pub = self.create_publisher(
            Float64MultiArray, '/fr3_velocity_controller/commands', 10)
        self._t = 0.0
        self._dt = 1.0 / _RATE_HZ
        self._timer = self.create_timer(self._dt, self._step)
        self.get_logger().info('velocity_figure8_trajectory started')

    def _step(self):
        ramp = min(self._t / _RAMP_S, 1.0)
        msg = Float64MultiArray()
        data = []
        for i in range(_N):
            # Alternate between ω and 2ω per joint for figure-8 shape
            omega = _OMEGA if i % 2 == 0 else 2 * _OMEGA
            data.append(ramp * _AMP[i] * math.sin(omega * self._t + _PHASE[i]))
        msg.data = data
        self._pub.publish(msg)
        self._t += self._dt


def main(args=None):
    rclpy.init(args=args)
    node = VelocityFigure8Trajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Float64MultiArray()
        stop.data = [0.0] * _N
        node._pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
