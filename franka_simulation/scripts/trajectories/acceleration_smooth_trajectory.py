#!/usr/bin/env python3
"""Smooth sinusoidal acceleration trajectory for the acceleration pipeline.

Publishes to:
    /sim_acceleration_bridge/accel_cmd  (std_msgs/Float64MultiArray, 7 joints)

The sim_acceleration_bridge integrates these commands into velocity setpoints
for fr3_velocity_controller.

Design rationale — why sinusoidal, not ramp-up/steady:
    A constant or one-directional acceleration accumulates velocity monotonically
    until the bridge clamp is hit, then stays clamped → the robot "runs away".
    A zero-mean sinusoid  a(t) = A·sin(ωt)  integrates to an oscillating velocity
    v(t) ≈ −A/ω · cos(ωt) + C  that stays bounded without relying on clamping.

Startup envelope: first RAMP_S seconds scale from 0 → 1 to avoid the initial jerk
that would occur if a full sinusoid started abruptly.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

_N       = 7
_RATE_HZ = 100.0
_RAMP_S  = 4.0   # smooth startup envelope [s]

# Sinusoid frequency — slow enough to see the motion, fast enough to be demo-friendly
_OMEGA = 0.25    # rad/s  →  period ≈ 25 s

# Peak accelerations [rad/s²]: conservative, well inside FR3 limits.
# Resulting peak velocity ≈ AMP/ω — with ω=0.25: ~0.4–0.8 rad/s, well below limits.
_AMP   = [0.10, 0.08, 0.08, 0.06, 0.05, 0.05, 0.04]
# Phase offsets spread motion across joints for visual interest
_PHASE = [0.0, math.pi/3, 2*math.pi/3, math.pi, 4*math.pi/3, 5*math.pi/3, 0.0]


class AccelerationSmoothTrajectory(Node):
    def __init__(self):
        super().__init__('acceleration_smooth_trajectory')
        self._pub = self.create_publisher(
            Float64MultiArray, '/sim_acceleration_bridge/accel_cmd', 10)
        self._t = 0.0
        self._dt = 1.0 / _RATE_HZ
        self._timer = self.create_timer(self._dt, self._step)
        self.get_logger().info(
            f'acceleration_smooth_trajectory started | ω={_OMEGA} rad/s | '
            f'peak_vel ≈ {max(_AMP)/_OMEGA:.2f} rad/s'
        )

    def _step(self):
        # Raised-cosine startup envelope: 0 → 1 over RAMP_S seconds, then 1
        envelope = 0.5 * (1.0 - math.cos(math.pi * min(self._t, _RAMP_S) / _RAMP_S))

        msg = Float64MultiArray()
        msg.data = [
            envelope * _AMP[i] * math.sin(_OMEGA * self._t + _PHASE[i])
            for i in range(_N)
        ]
        self._pub.publish(msg)
        self._t += self._dt


def main(args=None):
    rclpy.init(args=args)
    node = AccelerationSmoothTrajectory()
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
