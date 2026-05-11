#!/usr/bin/env python3
"""Stable torque trajectory with implicit gravity compensation for the torque pipeline.

Publishes to:
    /fr3_effort_controller/commands  (std_msgs/Float64MultiArray, 7 joints)

Subscribes to:
    /joint_states  (sensor_msgs/JointState)

--- Why gravity compensation is required here ---
JointGroupEffortController sends raw torque commands to the Gazebo actuators.
Gazebo physics applies gravity independently.  Publishing τ=0 means zero motor
torque, so gravity pulls the robot down.  To hold a pose the commanded torque
must counteract gravity.

--- Strategy: joint-space PD around home position + sinusoidal perturbation ---
    τ[i] = Kp[i]·(q_home[i] − q[i])   ← spring restoring toward home
           − Kd[i]·dq[i]               ← damping (no oscillation)
           + A[i]·sin(ω·t + φ[i])      ← small sinusoidal perturbation

The PD term acts as an implicit gravity compensator: whenever gravity pulls a
joint away from home, the spring pushes it back.  With appropriate Kp the robot
holds position stably and the sinusoidal perturbation creates smooth oscillation
around equilibrium — safe and student-readable.

FR3 home pose (Gazebo initial values from xacro):
    q = [0, −π/4, 0, −3π/4, 0, π/2, π/4]
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

_N = 7
_RATE_HZ = 100.0
_RAMP_S  = 5.0   # startup ramp for the PD + perturbation [s]

# FR3 home joint positions [rad] (must match xacro initial_position values)
_Q_HOME = [0.0, -math.pi/4, 0.0, -3*math.pi/4, 0.0, math.pi/2, math.pi/4]

# Joint-space stiffness [Nm/rad] — hold pose against gravity without oscillation.
# Values tuned for FR3 inertia: higher for proximal (heavy) joints.
_KP = [60.0, 80.0, 50.0, 80.0, 30.0, 25.0, 15.0]

# Damping [Nm·s/rad] — set ≈ 2√(Kp·I_eff) for critical damping; here conservative.
_KD = [8.0, 10.0, 6.0, 10.0, 4.0, 3.0, 2.0]

# Sinusoidal perturbation: small amplitudes, slow frequency → gentle oscillation
_OMEGA = 0.20    # rad/s  →  period ≈ 31 s
_AMP   = [1.5, 1.0, 0.8, 0.5, 0.3, 0.3, 0.2]   # [Nm] — well below rated torques
_PHASE = [0.0, math.pi/3, 2*math.pi/3, math.pi, 4*math.pi/3, 5*math.pi/3, 0.0]

_JOINT_NAMES = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
    'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]


class TorqueSineTrajectory(Node):
    def __init__(self):
        super().__init__('torque_sine_trajectory')

        self._pub = self.create_publisher(
            Float64MultiArray, '/fr3_effort_controller/commands', 10)

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub = self.create_subscription(
            JointState, '/joint_states', self._cb_js, best_effort)

        self._q  = list(_Q_HOME)   # current position (seed = home)
        self._dq = [0.0] * _N      # current velocity
        self._js_idx: list[int] | None = None

        self._t = 0.0
        self._dt = 1.0 / _RATE_HZ
        self._timer = self.create_timer(self._dt, self._step)

        self.get_logger().info(
            'torque_sine_trajectory started | PD hold + sinusoidal perturbation'
        )

    def _cb_js(self, msg: JointState):
        if self._js_idx is None:
            try:
                self._js_idx = [msg.name.index(n) for n in _JOINT_NAMES]
            except ValueError as e:
                self.get_logger().warn(
                    f'joint_states missing joint: {e}', throttle_duration_sec=5.0)
                return
        if len(msg.position) > max(self._js_idx):
            self._q = [msg.position[i] for i in self._js_idx]
        if len(msg.velocity) > max(self._js_idx):
            self._dq = [msg.velocity[i] for i in self._js_idx]

    def _step(self):
        # Startup ramp: first RAMP_S seconds smoothly engage the controller
        ramp = 0.5 * (1.0 - math.cos(math.pi * min(self._t, _RAMP_S) / _RAMP_S))

        msg = Float64MultiArray()
        tau = []
        for i in range(_N):
            pd  = _KP[i] * (_Q_HOME[i] - self._q[i]) - _KD[i] * self._dq[i]
            sin = _AMP[i] * math.sin(_OMEGA * self._t + _PHASE[i])
            tau.append(ramp * (pd + sin))
        msg.data = tau
        self._pub.publish(msg)
        self._t += self._dt


def main(args=None):
    rclpy.init(args=args)
    node = TorqueSineTrajectory()
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
