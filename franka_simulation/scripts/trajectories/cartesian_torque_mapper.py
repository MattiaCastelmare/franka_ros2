#!/usr/bin/env python3
"""Cartesian torque controller with analytic gravity compensation.

Publishes to:
    /fr3_effort_controller/commands  (Float64MultiArray, 7 joints)

Subscribes to:
    /joint_states  (position + velocity)

--- Control law ---
    τ = G(q)                        ← analytic gravity compensation (URDF masses)
      + Kp·(q_home − q)             ← joint-space spring toward HOME
      − Kd·q̇                        ← joint damping
      + ramp · Jᵀ·F_cart(t)        ← small sinusoidal Cartesian overlay

G(q) is exact (computed from URDF link masses), so the robot holds its pose
under gravity.  The joint-space PD adds stiffness against perturbations.
The Cartesian overlay creates a slow, visible EE motion without destabilising
the pose.

--- Parameters ---
    F_AMP   Cartesian force amplitude [N]    default 3.0
    OMEGA   oscillation frequency [rad/s]    default 0.20  (period ≈ 31 s)
    KP      joint position gain [Nm/rad]     per-joint array
    KD      joint velocity gain [Nm·s/rad]   per-joint array
    RAMP_S  startup ramp [s]                 default 5.0
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from utils.fr3_kinematics import jacobian, gravity_torques, Q_HOME

# ── Tunable parameters ────────────────────────────────────────────────────────
F_AMP  = 3.0     # N   — peak Cartesian force amplitude
OMEGA  = 0.20    # rad/s  →  period ≈ 31 s
RAMP_S = 5.0     # startup ramp [s] — long ramp to avoid jerk on engage

# Joint-space PD gains — stiffness to resist perturbations from HOME
KP = np.array([40.0, 55.0, 35.0, 55.0, 20.0, 15.0, 10.0])   # [Nm/rad]
KD = np.array([ 6.0,  8.0,  5.0,  8.0,  3.0,  2.0,  1.5])   # [Nm·s/rad]

# Maximum torque clamp [Nm] — inside FR3 rated torques
TAU_MAX = np.array([87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0])
# ─────────────────────────────────────────────────────────────────────────────

_N = 7
_JOINT_NAMES = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
    'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]


class CartesianTorqueMapper(Node):
    def __init__(self):
        super().__init__('cartesian_torque_mapper')

        self._pub = self.create_publisher(
            Float64MultiArray, '/fr3_effort_controller/commands', 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self._sub = self.create_subscription(
            JointState, '/joint_states', self._cb_js, qos)

        self._q   = Q_HOME.copy()
        self._dq  = np.zeros(_N)
        self._idx: list | None = None

        self._t  = 0.0
        self._dt = 0.01   # 100 Hz
        self._timer = self.create_timer(self._dt, self._step)

        self.get_logger().info(
            f'cartesian_torque_mapper | F_amp={F_AMP} N  ω={OMEGA} rad/s  '
            f'period={2*math.pi/OMEGA:.1f} s'
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
        t = self._t

        # Startup ramp: engage controller smoothly
        ramp = 0.5 * (1.0 - math.cos(math.pi * min(t, RAMP_S) / RAMP_S))

        # ── τ_gravity: analytic gravity compensation ──────────────────────────
        G = gravity_torques(self._q)

        # ── Joint-space PD: spring toward HOME + damping ──────────────────────
        tau_pd = KP * (Q_HOME - self._q) - KD * self._dq

        # ── Cartesian overlay: slow sinusoidal force in world X ───────────────
        #   F_cart(t) = [A·sin(ωt), 0, 0]
        F_cart = np.array([
            F_AMP * math.sin(OMEGA * t),
            0.0,
            0.0,
        ])
        J, _ = jacobian(self._q)
        tau_cart = J.T @ F_cart

        # ── Compose: G always active; PD + Cartesian ramp up ─────────────────
        tau = G + ramp * (tau_pd + tau_cart)
        tau = np.clip(tau, -TAU_MAX, TAU_MAX)

        msg = Float64MultiArray()
        msg.data = tau.tolist()
        self._pub.publish(msg)
        self._t += self._dt


def main(args=None):
    rclpy.init(args=args)
    node = CartesianTorqueMapper()
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
