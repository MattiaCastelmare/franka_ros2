#!/usr/bin/env python3
"""
FR3 Avoidance Minimal Trajectory Sender + Logger
===============================================

SECTIONS REMOVED (WHY):
- Large state machine with interactive waits/pauses (not needed for minimal sender).
- Stack parameter checks and service calls (avoid reading controller params).
- IK fallback and MoveIt execution tracking (no safety/IK logic in this node).
- TF2 / PlanningScene obstacle parsing and nearest-obstacle logic (avoid external scene logic).
- Pinocchio FK and EE error monitoring (not required for minimal trajectory sender).
- Keyboard listener and pause control (no user interaction).
- Stall/timeout logic and best-effort policies (controller handles safety).

This node ONLY:
- Sends a HOME joint trajectory once
- Sends Cartesian waypoints via MoveToPose, gated by user ENTER
- Logs SAFE vs AVOIDANCE and closest hazards/distances
- Logs velocity norms: |cmd|, |avoid|, |track≈|
"""

import time
import threading
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from franka_simulation.action import MoveToPose


@dataclass
class Waypoint:
    x: float
    y: float
    z: float
    name: str
    critical: bool = False


HOME_JOINT_POSITION = np.array(
    [0.0, -0.785398, 0.0, -2.35619, 0.0, 1.5708, 0.785398], dtype=float
)


# ======================================================
# MAIN NODE
# ======================================================

class SafeAvoidanceTest(Node):
    def __init__(self):
        super().__init__("safe_avoidance_test")

        self.waypoints: List[Waypoint] = []
        self.wp_index = -1
        self._last_publish_wall = 0.0
        self.last_sent_wp_name = "(none)"
        self._home_sent = False

        # User confirmation (non-blocking)
        self._confirm_request = threading.Event()
        self._confirm_received = threading.Event()

        self.avoidance_vel = np.zeros(7, dtype=float)
        self.cmd_vel = np.zeros(7, dtype=float)
        self.closest_d = float("inf")
        self.closest_j_row = np.zeros(7, dtype=float)
        self.closest_hazard = "none"
        self._closest_stamp_wall: Optional[float] = None

        # Parameters (minimal)
        self.declare_parameter("log_rate_hz", 5.0)
        self.declare_parameter("waypoint_dwell_s", 0.8)
        self.declare_parameter("avoidance_active_distance_m", 0.35)
        self._rate_hz = float(self.get_parameter("log_rate_hz").value)
        self._waypoint_dwell_s = float(self.get_parameter("waypoint_dwell_s").value)
        self._avoidance_active_distance_m = float(self.get_parameter("avoidance_active_distance_m").value)

        # ROS interfaces
        self.create_subscription(Float64MultiArray, "/avoidance/velocity", self._avoid_cb, 10)
        self.create_subscription(Float64MultiArray, "/fr3_velocity_controller/commands", self._cmd_cb, 10)
        self.create_subscription(Float64MultiArray, "/avoidance/closest_constraint", self._closest_constraint_cb, 10)
        self.create_subscription(String, "/avoidance/closest_hazard", self._closest_hazard_cb, 10)

        self.velocity_blender_traj_pub = self.create_publisher(
            JointTrajectory, "/velocity_blender/trajectory", 10
        )

        self.action_client = ActionClient(self, MoveToPose, "move_to_pose")

        self.log_timer = self.create_timer(1.0 / max(1e-3, self._rate_hz), self._log_status)
        self.send_timer = self.create_timer(0.1, self._send_next_if_due)

        self.get_logger().info("✅ Minimal avoidance test node ready")

        threading.Thread(target=self._keyboard_listener, daemon=True).start()


    def _pick_coherent_safety(self, now_wall: Optional[float] = None) -> Tuple[float, str, float, bool]:
        """Return (d_min, hazard, j_norm, coherent) from closest_* topics."""
        try:
            now = float(now_wall) if now_wall is not None else time.time()
            stamp = self._closest_stamp_wall
            coherent = (stamp is not None) and ((now - float(stamp)) <= 0.5)
            if coherent:
                d = float(self.closest_d)
                haz = str(self.closest_hazard or "none")
                jn = float(np.linalg.norm(self.closest_j_row))
                return d, haz, jn, True
            return float("inf"), "none", 0.0, False
        except Exception:
            return float("inf"), "none", 0.0, False


    def _send_next_if_due(self):
        now = time.time()
        if (now - self._last_publish_wall) < self._waypoint_dwell_s:
            return

        if not self._home_sent:
            self._publish_home()
            self._home_sent = True
            self._request_user_confirm()
            self._last_publish_wall = now
            return

        if not self._confirm_received.is_set():
            return

        self._confirm_received.clear()

        if self.wp_index < 0:
            self.wp_index = 0

        if self.wp_index >= len(self.waypoints):
            return

        wp = self.waypoints[self.wp_index]
        if self._send_waypoint(wp):
            self.wp_index += 1
            self._last_publish_wall = now
            self._request_user_confirm()


    # ======================================================
    # STATUS LINE FORMATTING
    # ======================================================

    def _publish_home(self):
        traj = JointTrajectory()
        traj.joint_names = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ]
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in HOME_JOINT_POSITION]
        pt.time_from_start.sec = 3
        traj.points = [pt]
        self.velocity_blender_traj_pub.publish(traj)
        self.last_sent_wp_name = "HOME"
        self.get_logger().info("➡ Sent HOME joint trajectory")

    def _send_waypoint(self, wp: Waypoint) -> bool:
        if not self.action_client.server_is_ready():
            self.get_logger().warn("⏳ Action server 'move_to_pose' not ready yet")
            return False

        goal = MoveToPose.Goal()
        goal.pose_target.header.frame_id = "world"
        goal.pose_target.pose.position.x = float(wp.x)
        goal.pose_target.pose.position.y = float(wp.y)
        goal.pose_target.pose.position.z = float(wp.z)

        # Tool-down orientation
        goal.pose_target.pose.orientation.x = 1.0
        goal.pose_target.pose.orientation.y = 0.0
        goal.pose_target.pose.orientation.z = 0.0
        goal.pose_target.pose.orientation.w = 0.0

        self.action_client.send_goal_async(goal)
        self.last_sent_wp_name = str(wp.name)
        self.get_logger().info(f"➡ Sent waypoint {wp.name}")
        return True


    # ======================================================
    # SMALL UTILITIES (LOGGING)
    # ======================================================

    def _avoid_cb(self, msg: Float64MultiArray):
        if len(msg.data) == 7:
            self.avoidance_vel = np.array(msg.data, dtype=float)

    def _cmd_cb(self, msg: Float64MultiArray):
        if len(msg.data) == 7:
            self.cmd_vel = np.array(msg.data, dtype=float)

    def _closest_constraint_cb(self, msg: Float64MultiArray):
        if len(msg.data) >= 8:
            self.closest_d = float(msg.data[0])
            self.closest_j_row = np.array(msg.data[1:8], dtype=float)
            self._closest_stamp_wall = time.time()

    def _closest_hazard_cb(self, msg: String):
        if msg.data:
            self.closest_hazard = str(msg.data)
            self._closest_stamp_wall = time.time()

    def _log_status(self):
        now = time.time()
        d, haz, jn, coherent = self._pick_coherent_safety(now_wall=now)
        haz = str(haz or "none")

        qdot_avoid = self.avoidance_vel
        qdot_cmd = self.cmd_vel
        qdot_track = qdot_cmd - qdot_avoid

        av = float(np.linalg.norm(qdot_avoid))
        bv = float(np.linalg.norm(qdot_cmd))
        tv = float(np.linalg.norm(qdot_track))

        d_dot_cmd = 0.0
        d_dot_track = 0.0
        try:
            if coherent and (float(jn) > 1e-6):
                d_dot_cmd = float(np.array(self.closest_j_row, dtype=float).reshape(7) @ np.array(qdot_cmd, dtype=float).reshape(7))
                d_dot_track = float(np.array(self.closest_j_row, dtype=float).reshape(7) @ np.array(qdot_track, dtype=float).reshape(7))
        except Exception:
            d_dot_cmd = 0.0
            d_dot_track = 0.0

        if coherent:
            active = (haz != "none") or (
                (d < float(self._avoidance_active_distance_m)) and (av > 1e-3) and (jn > 1e-6)
            )
        else:
            active = (d < float(self._avoidance_active_distance_m)) and (av > 1e-3) and (jn > 1e-6)

        tag = "[AVOIDANCE]" if active else "[SAFE]"
        wp_name = self.last_sent_wp_name

        self.get_logger().info(
            f"{tag} wp={wp_name} d={d:.3f} haz={haz} "
            f"|cmd|={bv:.3f} |avoid|={av:.3f} |track≈|={tv:.3f} "
            f"d_dot_cmd={d_dot_cmd:.4f} d_dot_track={d_dot_track:.4f} "
            f"src={'closest' if coherent else 'stale'}"
        )


    def _keyboard_listener(self):
        while rclpy.ok():
            try:
                self._confirm_request.wait(timeout=0.2)
            except Exception:
                continue

            if not self._confirm_request.is_set():
                continue

            self._confirm_request.clear()
            try:
                input("\n👉 Premi ENTER per autorizzare il prossimo step...\n")
            except Exception:
                self.get_logger().warn("⚠️ stdin non disponibile: auto-continue")

            self._confirm_received.set()


    def _request_user_confirm(self):
        self._confirm_received.clear()
        self._confirm_request.set()


    # (No keyboard/IK/obstacle logic in minimal node)


# ======================================================
# MAIN
# ======================================================

def main():
    rclpy.init()
    node = SafeAvoidanceTest()

    # Cartesian waypoints in frame "world".
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
        Waypoint(0.40, -0.50, 0.25, "WP9_DIAGONAL"),
        Waypoint(0.40, 0.50, 0.20, "WP10_FAR_CORNER"),
        Waypoint(0.30, 0.50, 0.30, "WP11_FINAL_HOME"),
        Waypoint(0.30, 0.60, 0.40, "WP12_SAFE"),
        Waypoint(0.30, 0.0, 0.45, "WP13_HOME_RETURN"),
    ]

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        # In some ROS2 builds, Ctrl-C during message conversion can raise a RuntimeError.
        # Treat it as a best-effort shutdown (diagnostic-only behavior).
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
