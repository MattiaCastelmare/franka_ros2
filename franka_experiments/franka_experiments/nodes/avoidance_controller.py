#!/usr/bin/env python3

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from franka_msgs.msg import HumanRobotDistance

from franka_experiments.utils.constants import FR3_JOINT_NAMES, NUM_JOINTS
from franka_experiments.utils.kinematics import generate_urdf_from_xacro
from franka_experiments.utils.simulation_imports import build_reduced_pinocchio_model_from_urdf

_EE_FRAME = "fr3_hand"
_OUTPUT_TOPIC = "/NS_1/avoidance_qdot"


class AvoidanceController(Node):

    def __init__(self):
        super().__init__("human_avoidance_controller")

        self.declare_parameter("control_rate", 100.0)
        self.declare_parameter("human_distance_topic", "/human_robot/closest_distance")
        self.declare_parameter("k_rep", 0.3)
        self.declare_parameter("max_joint_vel", 0.25)
        self.declare_parameter("d_influence", 0.5)
        self.declare_parameter("lpf_alpha", 0.75)
        self.declare_parameter("max_accel_local", 3.0)

        self._control_rate = float(self.get_parameter("control_rate").value)
        self._human_distance_topic = str(self.get_parameter("human_distance_topic").value)
        self._k_rep = float(self.get_parameter("k_rep").value)
        self._max_joint_vel = float(self.get_parameter("max_joint_vel").value)
        self._d_influence = float(self.get_parameter("d_influence").value)
        self._lpf_alpha = float(self.get_parameter("lpf_alpha").value)
        self._max_accel_local = float(self.get_parameter("max_accel_local").value)
        self._dt = 1.0 / self._control_rate

        self.q = None
        self.human_distance = None
        self.pin_ok = False
        self._qdot_prev = np.zeros(7)
        self._qdot_cmd_prev = np.zeros(7)

        self._init_pinocchio()

        self.create_subscription(JointState, "/NS_1/joint_states", self._joint_state_cb, 10)
        self.create_subscription(
            HumanRobotDistance,
            self._human_distance_topic,
            self._human_distance_cb,
            10,
        )

        self.pub = self.create_publisher(
            Float64MultiArray, _OUTPUT_TOPIC, qos_profile_sensor_data
        )

        self.create_timer(1.0 / self._control_rate, self._control_loop)

        self.get_logger().info(
            f"Human Avoidance Controller ready — k_rep={self._k_rep}, "
            f"max_vel={self._max_joint_vel}, d_influence={self._d_influence}, "
            f"output='{_OUTPUT_TOPIC}'"
        )

    def _init_pinocchio(self):
        try:
            urdf_xml = generate_urdf_from_xacro()
            self.model, self.data = build_reduced_pinocchio_model_from_urdf(urdf_xml)
            frame_names = [self.model.frames[i].name for i in range(self.model.nframes)]
            if _EE_FRAME not in frame_names:
                self.get_logger().error(
                    f"EE frame '{_EE_FRAME}' not found. Available: {frame_names}"
                )
                return
            self._ee_frame_id = self.model.getFrameId(_EE_FRAME)
            self.pin_ok = True
            self.get_logger().info(f"Pinocchio ready — {self.model.nq} DoF, EE frame '{_EE_FRAME}'")
        except Exception as e:
            self.get_logger().error(f"Pinocchio init failed: {e}")

    def _joint_state_cb(self, msg: JointState):
        if not msg.name:
            return
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        try:
            self.q = np.array(
                [msg.position[name_to_idx[jn]] for jn in FR3_JOINT_NAMES], dtype=float
            )
        except KeyError:
            return

    def _human_distance_cb(self, msg: HumanRobotDistance):
        self.human_distance = msg

    def _publish_zero(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * NUM_JOINTS
        self.pub.publish(msg)
        self._qdot_prev = np.zeros(7)
        self._qdot_cmd_prev = np.zeros(7)

    def _control_loop(self):
        if not self.pin_ok or not isinstance(self.q, np.ndarray):
            self._publish_zero()
            return

        msg = self.human_distance
        if msg is None or not msg.valid:
            self._publish_zero()
            return

        direction = np.array([msg.direction.x, msg.direction.y, msg.direction.z], dtype=float)
        norm = np.linalg.norm(direction)
        if norm < 1e-9 or not np.isfinite(norm):
            self._publish_zero()
            return

        direction /= norm

        d = float(msg.distance)
        s = float(np.clip((self._d_influence - d) / self._d_influence, 0.0, 1.0))

        if s == 0.0:
            self._publish_zero()
            return

        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)

        J_full = pin.computeFrameJacobian(
            self.model, self.data, self.q, self._ee_frame_id, pin.LOCAL_WORLD_ALIGNED
        )
        J_t = J_full[:3, :]

        v_cart = self._k_rep * s * direction
        qdot_raw = J_t.T @ v_cart

        # 1. Clamp velocità raw
        qdot_clamped = np.clip(qdot_raw, -self._max_joint_vel, self._max_joint_vel)

        # 2. Low-pass filter
        qdot_lpf = self._lpf_alpha * qdot_clamped + (1.0 - self._lpf_alpha) * self._qdot_prev
        self._qdot_prev = qdot_lpf.copy()

        # 3. Rate limiter locale
        max_delta = self._max_accel_local * self._dt
        qdot_rate_limited = np.zeros(7)
        for i in range(7):
            delta = qdot_lpf[i] - self._qdot_cmd_prev[i]
            if delta > max_delta:
                qdot_rate_limited[i] = self._qdot_cmd_prev[i] + max_delta
            elif delta < -max_delta:
                qdot_rate_limited[i] = self._qdot_cmd_prev[i] - max_delta
            else:
                qdot_rate_limited[i] = qdot_lpf[i]
        self._qdot_cmd_prev = qdot_rate_limited.copy()

        # 4. Final clamp
        qdot_final = np.clip(qdot_rate_limited, -self._max_joint_vel, self._max_joint_vel)

        out = Float64MultiArray()
        out.data = qdot_final.tolist()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
