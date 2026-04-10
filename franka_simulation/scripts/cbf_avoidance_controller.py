#!/usr/bin/env python3

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from utils.ros_setup import init_pinocchio_only, make_joint_state_callback
from franka_msgs.msg import HumanRobotDistance


class CBFAvoidance(Node):

    def __init__(self):
        super().__init__("cbf_avoidance_controller")

        # ===== Robot =====
        self.joint_names = [
            "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
            "fr3_joint5", "fr3_joint6", "fr3_joint7",
        ]
        self.n_dof = len(self.joint_names)
        self.q = None

        self.pin_ok, self.model, self.data = init_pinocchio_only(self)

        if not self.pin_ok:
            self.get_logger().error("Pinocchio initialization failed")
            return

        # ===== Parameters =====
        self.d_activate = 0.45
        self.k_rep = 0.5
        self.max_vel = 0.5

        # ===== Obstacle =====
        self.last_distance = None
        self.last_direction = None
        self.last_link_name = None
        self.last_valid = False

        # ===== ROS =====
        self.create_subscription(
            JointState,
            "/joint_states",
            make_joint_state_callback(controller=self, joint_names=self.joint_names),
            10,
        )

        self.create_subscription(
            HumanRobotDistance,
            "/human_robot/closest_distance",
            self.distance_callback,
            10,
        )

        self.pub = self.create_publisher(Float64MultiArray, "/avoidance/simple_vel", 10)


        self.create_timer(0.01, self.control_loop)

        self.get_logger().info("Simple avoidance READY")

    def distance_callback(self, msg):
        self.last_valid = bool(msg.valid)

        if not self.last_valid:
            qdot = np.zeros(self.n_dof)
            msg = Float64MultiArray()
            msg.data = qdot.tolist()
            self.pub.publish(msg)
            return

        self.last_distance = float(msg.distance)
        self.get_logger().info("DISTANCE MESSAGE RECEIVED")
        self.last_direction = np.array([
            msg.direction.x,
            msg.direction.y,
            msg.direction.z,
        ], dtype=float)
        self.last_link_name = msg.robot_link_name

    def control_loop(self):
        if not self.pin_ok or self.q is None:
            return

        if not self.last_valid:
            qdot = np.zeros(self.n_dof)
            msg = Float64MultiArray()
            msg.data = qdot.tolist()
            self.pub.publish(msg)
            return

        if self.last_distance is None or self.last_direction is None or self.last_link_name is None:
            return

        d = float(self.last_distance)

        if d > self.d_activate:
            qdot = np.zeros(self.n_dof)
        else:
            n = self.last_direction.copy()
            n_norm = np.linalg.norm(n)

            if n_norm < 1e-9:
                qdot = np.zeros(self.n_dof)
            else:
                n = n / n_norm

                try:
                    frame_id = self.model.getFrameId(self.last_link_name)
                except Exception:
                    self.get_logger().warn(f"Frame '{self.last_link_name}' not found")
                    qdot = np.zeros(self.n_dof)
                else:
                    pin.forwardKinematics(self.model, self.data, self.q)
                    pin.updateFramePlacements(self.model, self.data)

                    J = pin.computeFrameJacobian(
                        self.model,
                        self.data,
                        self.q,
                        frame_id,
                        pin.LOCAL_WORLD_ALIGNED,
                    )[:3, :]

                    strength = self.k_rep * max(self.d_activate - d, 0.0) / self.d_activate
                    v = strength * n
                    qdot = J.T @ v
                    qdot = np.clip(qdot, -self.max_vel, self.max_vel)
                    
        msg = Float64MultiArray()
        msg.data = qdot.tolist()
        self.pub.publish(msg)

        self.get_logger().info(
            f"d = {d:.3f}, link = {self.last_link_name}",
            throttle_duration_sec=0.5,
        )


def main(args=None):
    rclpy.init(args=args)
    node = CBFAvoidance()
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