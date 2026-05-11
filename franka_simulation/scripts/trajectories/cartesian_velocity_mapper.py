#!/usr/bin/env python3
"""Cartesian circle trajectory → joint velocity commands.

Generates a horizontal circle in the XY plane (constant Z) and maps it to
joint velocities via the damped-least-squares Jacobian pseudoinverse.

Publishes to:
    /fr3_velocity_controller/commands  (Float64MultiArray, 7 joints)

Subscribes to:
    /joint_states  (for current q and the real-time Jacobian)

--- Parameters (edit to customise) ---
    RADIUS   circle radius [m]          default 0.05
    OMEGA    angular velocity [rad/s]   default 0.30
    KP_CART  corrective gain [1/s]      default 1.5
    DLS_LAM  DLS damping                default 0.05

--- How it works ---
    1. At startup, read q from /joint_states → compute FK → set circle centre.
    2. Each step: compute desired position/velocity on the circle.
    3. Add corrective velocity  Kp·(x_des − x_cur)  to stay on track.
    4. Solve DLS: qdot = J†(q) · xdot_cmd   (translational 3-DOF only).
    5. Clamp to FR3 velocity limits and publish.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from utils.fr3_kinematics import jacobian, dls_qdot, QDOT_MAX, Q_HOME

# ── Tunable parameters ────────────────────────────────────────────────────────
RADIUS   = 0.05    # m   — circle radius (5 cm is safe and visible)
OMEGA    = 0.30    # rad/s  →  period ≈ 21 s
KP_CART  = 1.5     # 1/s — corrective gain to keep EE on the circle
DLS_LAM  = 0.05    # DLS base damping
RAMP_S   = 3.0     # startup envelope duration [s]
VEL_FRAC = 0.60    # fraction of joint velocity limits to use
# ─────────────────────────────────────────────────────────────────────────────

_N = 7
_JOINT_NAMES = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
    'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]
_QDOT_LIM = VEL_FRAC * QDOT_MAX


class CartesianVelocityMapper(Node):
    def __init__(self):
        super().__init__('cartesian_velocity_mapper')

        self._pub = self.create_publisher(
            Float64MultiArray, '/fr3_velocity_controller/commands', 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self._sub = self.create_subscription(
            JointState, '/joint_states', self._cb_js, qos)

        self._q   = Q_HOME.copy()
        self._idx: list | None = None
        self._center: np.ndarray | None = None  # computed from first FK

        self._t  = 0.0
        self._dt = 0.01   # 100 Hz
        self._timer = self.create_timer(self._dt, self._step)

        self.get_logger().info(
            f'cartesian_velocity_mapper | R={RADIUS} m  ω={OMEGA} rad/s  '
            f'period={2*math.pi/OMEGA:.1f} s'
        )

    def _cb_js(self, msg: JointState):
        if self._idx is None:
            try:
                self._idx = [msg.name.index(n) for n in _JOINT_NAMES]
            except ValueError:
                return
        if len(msg.position) > max(self._idx):
            self._q = np.array([msg.position[i] for i in self._idx])

    def _step(self):
        # Compute FK once we have joint states; set circle centre on first call
        J, p_cur = jacobian(self._q)
        if self._center is None:
            self._center = p_cur.copy()
            self.get_logger().info(
                f'Circle centre set to EE home position: '
                f'[{p_cur[0]:.3f}, {p_cur[1]:.3f}, {p_cur[2]:.3f}] m'
            )

        # Startup ramp: scale commands from 0 → 1 over RAMP_S seconds
        ramp = 0.5 * (1.0 - math.cos(math.pi * min(self._t, RAMP_S) / RAMP_S))

        # Desired position and velocity on the circle
        x_des = self._center + np.array([
            RADIUS * math.cos(OMEGA * self._t),
            RADIUS * math.sin(OMEGA * self._t),
            0.0,
        ])
        xdot_des = np.array([
            -RADIUS * OMEGA * math.sin(OMEGA * self._t),
             RADIUS * OMEGA * math.cos(OMEGA * self._t),
             0.0,
        ])

        # Corrective velocity: pulls EE back onto circle if it drifts
        xdot_cmd = ramp * (xdot_des + KP_CART * (x_des - p_cur))

        # Joint velocity via DLS pseudoinverse
        qdot = dls_qdot(J, xdot_cmd, lam=DLS_LAM)

        # Safety clamp to velocity limits
        qdot = np.clip(qdot, -_QDOT_LIM, _QDOT_LIM)

        msg = Float64MultiArray()
        msg.data = qdot.tolist()
        self._pub.publish(msg)
        self._t += self._dt


def main(args=None):
    rclpy.init(args=args)
    node = CartesianVelocityMapper()
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
