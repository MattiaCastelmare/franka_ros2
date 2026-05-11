#!/usr/bin/env python3
"""Acceleration bridge for Gazebo simulation.

Integrates joint acceleration commands into velocity commands forwarded to
fr3_velocity_controller.  This lets external nodes (e.g. CBF filters that
work at acceleration level) command the simulated robot without needing a
dedicated acceleration-level controller in Gazebo.

Integration step:
    q̇(t+1) = clamp(q̇(t) + q̈_cmd · dt,  -qdot_max,  +qdot_max)

Subscriptions:
    ~/accel_cmd   (std_msgs/Float64MultiArray)  — joint acceleration commands [rad/s²]
    /joint_states (sensor_msgs/JointState)      — current joint velocities

Publications:
    /fr3_velocity_controller/commands  (std_msgs/Float64MultiArray)  — velocity setpoints

Parameters:
    rate_hz        (float, default 100.0)  — control loop rate
    qdot_max       (list[float], 7 values) — per-joint velocity limits [rad/s]
    accel_cmd_topic (str)  — override subscription topic
    vel_cmd_topic   (str)  — override publication topic
    joint_names     (list[str])  — expected joint ordering
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

_JOINT_NAMES = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
    'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]
_N = len(_JOINT_NAMES)

# FR3 velocity limits from URDF (conservative, per Franka docs)
_QDOT_MAX_DEFAULT = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]

# Low-pass filter coefficient for incoming acceleration commands.
# y[k] = (1-ALPHA)*y[k-1] + ALPHA*u[k]
# ALPHA=0.2 → time constant ≈ 4 steps ≈ 40 ms at 100 Hz — smooths spikes
_ACCEL_LP_ALPHA = 0.2


class SimAccelBridge(Node):
    def __init__(self):
        super().__init__('sim_acceleration_bridge')

        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('qdot_max', _QDOT_MAX_DEFAULT)
        self.declare_parameter('accel_cmd_topic', '~/accel_cmd')
        self.declare_parameter('vel_cmd_topic', '/fr3_velocity_controller/commands')
        self.declare_parameter('joint_names', _JOINT_NAMES)

        rate_hz = self.get_parameter('rate_hz').value
        self._qdot_max = np.array(self.get_parameter('qdot_max').value, dtype=float)
        accel_topic = self.get_parameter('accel_cmd_topic').value
        vel_topic   = self.get_parameter('vel_cmd_topic').value
        self._joint_names = list(self.get_parameter('joint_names').value)

        if len(self._qdot_max) != _N:
            self.get_logger().error(f'qdot_max must have {_N} entries, got {len(self._qdot_max)}')
            raise ValueError('bad qdot_max length')

        self._dt = 1.0 / rate_hz
        self._qdot = np.zeros(_N)
        self._qddot_cmd = np.zeros(_N)       # raw incoming command
        self._qddot_filt = np.zeros(_N)      # low-pass filtered command
        self._js_index: list[int] | None = None  # mapping joint_states → our ordering

        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._sub_accel = self.create_subscription(
            Float64MultiArray, accel_topic, self._cb_accel, 10)
        self._sub_js = self.create_subscription(
            JointState, '/joint_states', self._cb_js, best_effort_qos)

        self._pub_vel = self.create_publisher(Float64MultiArray, vel_topic, 10)
        self._timer = self.create_timer(self._dt, self._step)

        self.get_logger().info(
            f'sim_acceleration_bridge ready | rate={rate_hz} Hz | '
            f'accel_in={accel_topic} → vel_out={vel_topic}'
        )

    def _cb_accel(self, msg: Float64MultiArray):
        if len(msg.data) != _N:
            self.get_logger().warn(
                f'accel_cmd has {len(msg.data)} values, expected {_N} — ignored',
                throttle_duration_sec=2.0,
            )
            return
        raw = np.array(msg.data, dtype=float)
        # Low-pass filter: smooth spikes before integration
        self._qddot_filt = (1.0 - _ACCEL_LP_ALPHA) * self._qddot_filt + _ACCEL_LP_ALPHA * raw
        self._qddot_cmd = self._qddot_filt

    def _cb_js(self, msg: JointState):
        if self._js_index is None:
            try:
                self._js_index = [msg.name.index(n) for n in self._joint_names]
            except ValueError as e:
                self.get_logger().warn(f'joint_states missing joint: {e}', throttle_duration_sec=5.0)
                return
        if len(msg.velocity) < max(self._js_index) + 1:
            return
        self._qdot = np.array([msg.velocity[i] for i in self._js_index], dtype=float)

    def _step(self):
        # Euler integration with symmetric velocity clamp
        qdot_next = np.clip(
            self._qdot + self._qddot_cmd * self._dt,
            -self._qdot_max,
             self._qdot_max,
        )
        out = Float64MultiArray()
        out.data = qdot_next.tolist()
        self._pub_vel.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SimAccelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
