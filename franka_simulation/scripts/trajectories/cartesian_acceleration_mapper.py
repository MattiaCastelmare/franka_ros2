#!/usr/bin/env python3
"""Cartesian circle trajectory → stable joint acceleration commands.

Publishes to:
    /sim_acceleration_bridge/accel_cmd  (Float64MultiArray, 7 joints)

Subscribes to:
    /joint_states  (position + velocity)

--- Why the previous J̇-based approach was unstable ---
The formula  q̈ = J†·(ẍ_des − J̇·q̇)  requires numerically differentiating the
Jacobian.  At 100 Hz this estimate is noisy; multiplied by q̇ it produces large
spikes that the bridge integrates into runaway velocity.

--- Closed-loop Cartesian PD + feedforward (this implementation) ---
Instead of relying on J̇, we compute a task-space acceleration command that
includes position and velocity feedback:

    ẍ_cmd = ẍ_des + Kp_c·(x_des − x_cur) + Kd_c·(ẋ_des − ẋ_cur)

    q̈ = J†(q) · ẍ_cmd           (DLS pseudoinverse)

Properties:
  • ẍ_des is the circle's centripetal acceleration (feedforward — smooth tracking)
  • Kp_c·(x_des − x_cur) corrects drift — if the EE wanders, this pulls it back
  • Kd_c·(ẋ_des − ẋ_cur) damps overshoot — prevents oscillation around the path
  • No J̇ needed → no numerical noise

The bridge integrates q̈ → q̇ with built-in velocity clamping, so residual errors
remain bounded.

--- Tunable parameters ---
    RADIUS   circle radius [m]               default 0.05
    OMEGA    angular velocity [rad/s]         default 0.25
    KP_C     Cartesian position gain [1/s²]   default 20.0
    KD_C     Cartesian velocity gain [1/s]    default 8.0
    DLS_LAM  DLS base damping                 default 0.05
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from utils.fr3_kinematics import jacobian, dls_qdot, Q_HOME

# ── Tunable parameters ────────────────────────────────────────────────────────
RADIUS  = 0.05    # m   — circle radius
OMEGA   = 0.25    # rad/s  →  period ≈ 25 s
KP_C    = 20.0    # 1/s²  — Cartesian position error gain
KD_C    = 8.0     # 1/s   — Cartesian velocity error gain
DLS_LAM = 0.05    # DLS base damping
RAMP_S  = 4.0     # startup ramp [s]
# Conservative joint acceleration clamp [rad/s²]
QDDOT_MAX = np.array([5.0, 5.0, 5.0, 5.0, 8.0, 8.0, 8.0])
# ─────────────────────────────────────────────────────────────────────────────

_N = 7
_JOINT_NAMES = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
    'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]


class CartesianAccelerationMapper(Node):
    def __init__(self):
        super().__init__('cartesian_acceleration_mapper')

        self._pub = self.create_publisher(
            Float64MultiArray, '/sim_acceleration_bridge/accel_cmd', 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self._sub = self.create_subscription(
            JointState, '/joint_states', self._cb_js, qos)

        self._q  = Q_HOME.copy()
        self._dq = np.zeros(_N)
        self._idx: list | None = None
        self._center: np.ndarray | None = None

        self._t  = 0.0
        self._dt = 0.01   # 100 Hz
        self._timer = self.create_timer(self._dt, self._step)

        self.get_logger().info(
            f'cartesian_acceleration_mapper | R={RADIUS} m  ω={OMEGA} rad/s  '
            f'Kp={KP_C}  Kd={KD_C}'
        )

    def _cb_js(self, msg: JointState):
        if self._idx is None:
            try:
                self._idx = [msg.name.index(n) for n in _JOINT_NAMES]
            except ValueError:
                return
        idx = self._idx
        if len(msg.position) > max(idx):
            self._q  = np.array([msg.position[i] for i in idx])
        if len(msg.velocity) > max(idx):
            self._dq = np.array([msg.velocity[i] for i in idx])

    def _step(self):
        J, x_cur = jacobian(self._q)

        # Place circle so EE is exactly on it at t=0 → no initial position error
        if self._center is None:
            # EE starts at center + [R, 0, 0], so center = x_cur − [R, 0, 0]
            self._center = x_cur - np.array([RADIUS, 0.0, 0.0])
            self.get_logger().info(
                f'Circle centre: '
                f'[{self._center[0]:.3f}, {self._center[1]:.3f}, {self._center[2]:.3f}] m  '
                f'(EE start: [{x_cur[0]:.3f}, {x_cur[1]:.3f}, {x_cur[2]:.3f}] m)'
            )

        t = self._t

        # ── Reference trajectory (circle in XY plane) ────────────────────────
        x_des = self._center + np.array([
            RADIUS * math.cos(OMEGA * t),
            RADIUS * math.sin(OMEGA * t),
            0.0,
        ])
        xdot_des = np.array([
            -RADIUS * OMEGA * math.sin(OMEGA * t),
             RADIUS * OMEGA * math.cos(OMEGA * t),
             0.0,
        ])
        xddot_des = np.array([                         # centripetal feedforward
            -RADIUS * OMEGA**2 * math.cos(OMEGA * t),
            -RADIUS * OMEGA**2 * math.sin(OMEGA * t),
             0.0,
        ])

        # ── Closed-loop Cartesian task-space acceleration ─────────────────────
        x_err   = x_des - x_cur
        xdot_cur = J @ self._dq                        # estimated EE velocity
        xdot_err = xdot_des - xdot_cur

        xddot_cmd = xddot_des + KP_C * x_err + KD_C * xdot_err

        # ── Map to joint acceleration via DLS ────────────────────────────────
        qddot = dls_qdot(J, xddot_cmd, lam=DLS_LAM)

        # Startup ramp + clamp
        ramp = 0.5 * (1.0 - math.cos(math.pi * min(t, RAMP_S) / RAMP_S))
        qddot = np.clip(ramp * qddot, -QDDOT_MAX, QDDOT_MAX)

        msg = Float64MultiArray()
        msg.data = qddot.tolist()
        self._pub.publish(msg)
        self._t += self._dt


def main(args=None):
    rclpy.init(args=args)
    node = CartesianAccelerationMapper()
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
