#!/usr/bin/env python3
"""
FR3 Safe Online Avoidance Test — FINAL CLEAN VERSION (USER CONFIRM)
==================================================================

• HOME in joint space via JointTrajectory
• Waypoints cartesiani via MoveToPose
• Tool orientation fixed (tool-down)
• Robot stops at each waypoint
• User must press ENTER to continue
• No blocking spin
• Pinocchio FK only
• Deterministic state machine

Author: Maurizio
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from enum import Enum
from dataclasses import dataclass
from typing import List, Optional
import numpy as np
import threading

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import PlanningScene, MoveItErrorCodes

from franka_simulation.action import MoveToPose

# Pinocchio
import pinocchio as pin
from ament_index_python.packages import get_package_share_directory
import subprocess
import os


# ======================================================
# STATE MACHINE
# ======================================================

class State(Enum):
    INIT = 0
    GOING_HOME = 1
    READY = 2
    SENDING_GOAL = 3
    EXECUTING = 4
    WAIT_USER = 5
    COMPLETED = 6
    FAILED = 7


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    name: str
    critical: bool = False


# ======================================================
# HOME CONFIG (JOINT SPACE)
# ======================================================

HOME_JOINT_POSITION = np.array([
    0.0,
   -0.785398,
    0.0,
   -2.35619,
    0.0,
    1.5708,
    0.785398
])


# ======================================================
# MAIN NODE
# ======================================================

class SafeAvoidanceTest(Node):

    def __init__(self):
        super().__init__("safe_avoidance_test")

        # -------------------------------
        # State
        # -------------------------------
        self.state = State.INIT
        self.waypoints: List[Waypoint] = []
        self.wp_index = 0
        self.current_wp: Optional[Waypoint] = None
        self.user_ok = False

        # -------------------------------
        # Robot state
        # -------------------------------
        self.q = np.zeros(7)
        self.qd = np.zeros(7)
        self.avoidance_vel = np.zeros(7)
        self.blended_vel = np.zeros(7)

        # -------------------------------
        # Pinocchio
        # -------------------------------
        self._init_pinocchio()

        # -------------------------------
        # ROS interfaces
        # -------------------------------
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.create_subscription(Float64MultiArray, "/avoidance/velocity", self._avoid_cb, 10)
        self.create_subscription(Float64MultiArray, "/fr3_velocity_controller/commands", self._cmd_cb, 10)
        self.create_subscription(PlanningScene, "/obstacle_scene", self._scene_cb, 10)

        self.joint_traj_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )

        self.action_client = ActionClient(self, MoveToPose, "move_to_pose")

        # -------------------------------
        # Timers
        # -------------------------------
        self.monitor_timer = self.create_timer(0.1, self._monitor)
        self.debug_timer = self.create_timer(0.5, self._debug)

        # -------------------------------
        # Keyboard listener (NON BLOCKING)
        # -------------------------------
        threading.Thread(
            target=self._keyboard_listener,
            daemon=True
        ).start()

        self.get_logger().info("✅ SafeAvoidanceTest node ready")


    # ======================================================
    # PINOCCHIO
    # ======================================================

    def _init_pinocchio(self):
        xacro = os.path.join(
            get_package_share_directory("franka_description"),
            "robots", "fr3", "fr3.urdf.xacro"
        )

        urdf = subprocess.check_output([
            "xacro", xacro,
            "ros2_control:=false",
            "hand:=true",
            "arm_id:=fr3"
        ]).decode()

        model_full = pin.buildModelFromXML(urdf)
        locked = [model_full.getJointId(n) for n in model_full.names if "finger" in n]
        self.model = pin.buildReducedModel(model_full, locked, pin.neutral(model_full))
        self.data = self.model.createData()
        self.ee_frame = self.model.getFrameId("fr3_link8")

        self.get_logger().info("✔ Pinocchio model loaded")


    # ======================================================
    # CALLBACKS
    # ======================================================

    def _js_cb(self, msg: JointState):
        names = [
            "fr3_joint1", "fr3_joint2", "fr3_joint3",
            "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"
        ]
        try:
            self.q = np.array([msg.position[msg.name.index(n)] for n in names])
            self.qd = np.array([msg.velocity[msg.name.index(n)] for n in names])
        except ValueError:
            return

    def _avoid_cb(self, msg: Float64MultiArray):
        if len(msg.data) == 7:
            self.avoidance_vel = np.array(msg.data)

    def _cmd_cb(self, msg: Float64MultiArray):
        if len(msg.data) == 7:
            self.blended_vel = np.array(msg.data)

    def _scene_cb(self, msg: PlanningScene):
        pass


    # ======================================================
    # FK
    # ======================================================

    def ee_position(self) -> np.ndarray:
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.ee_frame].translation


    # ======================================================
    # HOME
    # ======================================================

    def send_home(self):
        self.get_logger().info("🏠 Moving to HOME (joint space)")
        traj = JointTrajectory()
        traj.joint_names = [
            "fr3_joint1", "fr3_joint2", "fr3_joint3",
            "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"
        ]

        pt = JointTrajectoryPoint()
        pt.positions = HOME_JOINT_POSITION.tolist()
        pt.time_from_start.sec = 4

        traj.points.append(pt)
        self.joint_traj_pub.publish(traj)
        self.state = State.GOING_HOME


    # ======================================================
    # WAYPOINT EXECUTION
    # ======================================================

    def send_next_waypoint(self):
        if self.wp_index >= len(self.waypoints):
            self.state = State.COMPLETED
            self.get_logger().info("🏁 All waypoints completed")
            return

        wp = self.waypoints[self.wp_index]
        self.current_wp = wp
        self.state = State.SENDING_GOAL

        goal = MoveToPose.Goal()
        goal.pose_target.header.frame_id = "world"
        goal.pose_target.pose.position.x = wp.x
        goal.pose_target.pose.position.y = wp.y
        goal.pose_target.pose.position.z = wp.z

        # Tool-down orientation
        goal.pose_target.pose.orientation.x = 1.0
        goal.pose_target.pose.orientation.y = 0.0
        goal.pose_target.pose.orientation.z = 0.0
        goal.pose_target.pose.orientation.w = 0.0

        goal.max_velocity_scaling_factor = 0.06

        self.get_logger().info(f"➡ Sending waypoint {wp.name}")
        self.action_client.wait_for_server()
        self.action_client.send_goal_async(goal).add_done_callback(
            self._goal_response_cb
        )


    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("❌ Goal rejected")
            self.state = State.FAILED
            return

        goal_handle.get_result_async().add_done_callback(self._result_cb)
        self.state = State.EXECUTING


    def _result_cb(self, future):
        result = future.result().result
        if result.result.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error("❌ Execution failed")
            self.state = State.FAILED
            return

        pos = self.ee_position()
        wp = self.current_wp
        err = np.linalg.norm(pos - np.array([wp.x, wp.y, wp.z]))

        self.get_logger().info(f"✅ {wp.name} reached | error = {err:.4f} m")
        self.get_logger().info("⏸ Robot fermo — attendendo conferma utente (ENTER)")

        self.user_ok = False
        self.state = State.WAIT_USER


    # ======================================================
    # MONITOR
    # ======================================================

    def _monitor(self):
        if self.state == State.INIT:
            self.send_home()

        elif self.state == State.GOING_HOME:
            if np.linalg.norm(self.q - HOME_JOINT_POSITION) < 0.02:
                self.get_logger().info("✅ HOME reached")
                self.state = State.READY

        elif self.state == State.READY:
            self.send_next_waypoint()

        elif self.state == State.WAIT_USER:
            if self.user_ok:
                self.wp_index += 1
                self.state = State.READY


    def _debug(self):
        if self.state != State.EXECUTING:
            return

        tv = np.linalg.norm(self.blended_vel - self.avoidance_vel)
        av = np.linalg.norm(self.avoidance_vel)
        bv = np.linalg.norm(self.blended_vel)

        self.get_logger().info(
            f"vel | tracking={tv:.3f} avoid={av:.3f} blended={bv:.3f}"
        )


    # ======================================================
    # KEYBOARD (ASYNC)
    # ======================================================

    def _keyboard_listener(self):
        while rclpy.ok():
            input("\n👉 Premi ENTER per autorizzare il prossimo waypoint...\n")
            self.user_ok = True


# ======================================================
# MAIN
# ======================================================

def main():
    rclpy.init()
    node = SafeAvoidanceTest()

    node.waypoints = [
        Waypoint(0.30, 0.0, 0.45, "WP0_HOME"),
        Waypoint(0.20, -0.65, 0.40, "WP1_RED_APPROACH"),
        Waypoint(0.30, 0.45, 0.30, "WP2_OTHER_SIDE"),
        Waypoint(0.20, -0.35, 0.40, "WP3_BACK"),
        Waypoint(0.10, 0.10, 0.50, "WP4_OTHER_SIDE_AGAIN"),
        Waypoint(0.30, -0.55, 0.30, "WP5_GAP_CENTER", True),
        Waypoint(0.30, 0.55, 0.45, "WP6_EXIT_GAP"),
        Waypoint(0.40, -0.55, 0.40, "WP7_YELLOW_APPROACH", True),
        Waypoint(0.40, 0.50, 0.55, "WP8_YELLOW_OVERHEAD"),
        Waypoint(0.50, -0.50, 0.25, "WP9_DIAGONAL"),
        Waypoint(0.40, 0.50, 0.20, "WP10_FAR_CORNER"),
        Waypoint(0.30, 0.50, 0.30, "WP11_FINAL_HOME"),
        Waypoint(0.30, 0.60, 0.40, "WP12_SAFE"),
        Waypoint(0.30, 0.0, 0.45, "WP13_HOME_RETURN"),
    ]

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
