#!/usr/bin/env python3

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory

from utils import velocity_blender_ros_helpers as vh
from utils import velocity_blender_pipeline as pipeline
from utils.velocity_blender_state import BlenderRuntimeState


class SimpleVelocityBlender(Node):

    def __init__(self):
        super().__init__("velocity_control_blender")

        # Nomi giunti
        self.joint_names = [
            "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
            "fr3_joint5", "fr3_joint6", "fr3_joint7",
        ]
        self.n_dof = 7

        # Map joint_name -> index once to keep JointState parsing O(n).
        self._joint_name_to_i = vh.build_name_to_index(self.joint_names)

        # ===== PARAMETRI (da ROS/YAML) =====
        # Nota: per una vista completa/ordinata dei parametri, guarda:
        #   franka_simulation/config/velocity_blender_params.yaml
        vh.declare_velocity_blender_parameters(self)
        vh.load_velocity_blender_parameters(self, self)

        # Flag: if influence_distance <= 0, force pure tracking (no avoidance/CBF/emergency).
        self.avoidance_disabled = (float(self.influence_distance) <= 0.0)

        # ===== RUNTIME STATE =====
        self._rt = BlenderRuntimeState(n_dof=int(self.n_dof))
        self._reactive_log_wall = 0.0

        # ===== SUBSCRIBERS =====
        self.create_subscription(
            JointState, "/joint_states",
            self.joint_state_cb, 10
        )

        self.create_subscription(
            JointTrajectory, "/velocity_blender/trajectory",
            self.trajectory_cb,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE
            )
        )

        self.create_subscription(
            Float64MultiArray, "/avoidance/velocity",
            self.avoidance_cb, 10
        )

        # Coherent (d_closest, j_row_closest)
        self.create_subscription(
            Float64MultiArray, "/avoidance/closest_constraint",
            self.closest_constraint_cb, 10
        )

        # TODO: confirm this topic exists in the controller if renamed elsewhere
        self.create_subscription(
            Float64MultiArray, "/avoidance/constraints",
            self.constraints_cb, 10
        )

        self.create_subscription(
            String,
            "/avoidance/closest_hazard",
            self.closest_hazard_cb,
            10,
        )

        self.create_subscription(
            String,
            "/avoidance/hazard",
            self.hazard_cb,
            10,
        )

        # External pause control (typically from an interactive test node)
        self.create_subscription(
            Bool,
            "/velocity_blender/pause",
            self.pause_cb,
            10,
        )

        # ===== PUBLISHER =====
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            "/fr3_velocity_controller/commands",
            10
        )

        # ===== TIMER =====
        period = float(self.control_period_s)
        if period <= 0.0:
            period = 0.01
        self.create_timer(period, self.control_loop)

        logger = self.get_logger()
        for line in (
            "✅ Simple Velocity Blender (ḋ-constrained) started",
            f"   Kp={self.kp}, max_vel={self.max_vel}, safety_margin={self.safety_margin}, influence_distance={self.influence_distance}",
            f"   use_avoidance_velocity={self.use_avoidance_velocity}, avoidance_normal_only={self.avoidance_normal_only}, "
            f"null_boost_max={self.null_boost_max}, avoidance_ratio_max={self.avoidance_ratio_max}",
            f"   d_dot_min_far={self.d_dot_min_far}, d_dot_min_close={self.d_dot_min_close}, "
            f"avoidance_tangent_weight={self.avoidance_tangent_weight}, normal_correction_max={self.normal_correction_max}",
            f"   d_dot_push_gain={self.d_dot_push_gain}, d_dot_push_max={self.d_dot_push_max}, "
            f"avoidance_weight_max={self.avoidance_weight_max}",
            f"   tangent_escape_enable={self.tangent_escape_enable}, tangent_escape_speed={self.tangent_escape_speed}, "
            f"tangent_escape_err_min={self.tangent_escape_err_min}",
            f"   pause_enable={self.pause_enable} (topic: /velocity_blender/pause)",
        ):
            logger.info(line)

        # Runtime diagnostics live in BlenderRuntimeState

    # ======================================================================
    # CALLBACKS
    # ======================================================================

    def joint_state_cb(self, msg: JointState):
        vh.on_joint_state(self._rt, msg, self._joint_name_to_i)

    def trajectory_cb(self, msg: JointTrajectory):
        vh.on_trajectory(self._rt, msg, self.joint_names, self.n_dof, self.get_logger())

    def avoidance_cb(self, msg: Float64MultiArray):
        vh.on_avoidance(self._rt, msg, self.n_dof)

        # Reactive test: enforce normal-only avoidance when no trajectory is active.
        try:
            if bool(getattr(self, "reactive_enable", False)) and (
                (not bool(self._rt.active)) or (len(self._rt.trajectory_points) == 0)
            ):
                j = np.array(self._rt.closest_j_row, dtype=float).reshape(-1)
                jn2 = float(j @ j)
                if jn2 > 1e-9:
                    qdot = np.array(self._rt.qdot_avoid, dtype=float).reshape(-1)
                    qdot_n = (float(j @ qdot) / float(jn2)) * j
                    self._rt.qdot_avoid = np.array(qdot_n, dtype=float).reshape(-1)

                    now = float(time.time())
                    if (now - float(self._reactive_log_wall)) >= 1.0:
                        self._reactive_log_wall = now
                        d_dot = float(j @ np.array(self._rt.qdot_avoid, dtype=float).reshape(-1))
                        if d_dot < -1e-6:
                            self.get_logger().warn(
                                f"[REACTIVE-NORMAL] d_dot={d_dot:.5f} (negative, check dir sign)"
                            )
                        else:
                            self.get_logger().debug(
                                f"[REACTIVE-NORMAL] d_dot={d_dot:.5f}"
                            )
        except Exception:
            pass

    def closest_constraint_cb(self, msg: Float64MultiArray):
        vh.on_closest_constraint(self._rt, msg, self.n_dof)

    def constraints_cb(self, msg: Float64MultiArray):
        vh.on_constraints(self._rt, msg, self.n_dof)

    def closest_hazard_cb(self, msg: String):
        vh.on_closest_hazard(self._rt, msg)

    def hazard_cb(self, msg: String):
        vh.on_hazard(self._rt, msg)

    def pause_cb(self, msg: Bool):
        vh.on_pause(self._rt, msg, self.pause_enable)

    # ======================================================================
    # CONTROL LOOP
    # ======================================================================

    def control_loop(self):
        """Loop di controllo principale."""
        now = float(time.time())
        qdot = pipeline.step(rt=self._rt, params=self, now_wall=now, logger=self.get_logger())
        if qdot is not None:
            self.publish_velocity(np.array(qdot, dtype=float).reshape(int(self.n_dof)))

    # ======================================================================
    # UTILITIES
    # ======================================================================

    def publish_velocity(self, qdot: np.ndarray):
        """Pubblica comando di velocità sui giunti."""
        self.cmd_pub.publish(Float64MultiArray(data=qdot.tolist()))


def main(args=None):
    rclpy.init(args=args)
    node = SimpleVelocityBlender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ferma il robot (best-effort): under ros2 launch the context may already be shutting down.
        try:
            if rclpy.ok():
                msg = Float64MultiArray()
                msg.data = [0.0] * 7
                node.cmd_pub.publish(msg)
        except Exception:
            pass

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
