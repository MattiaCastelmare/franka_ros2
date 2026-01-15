#!/usr/bin/env python3
"""Safe avoidance demo node (installable).

This is the installable version of the waypoint-based test previously living under
`franka_simulation/test/avoidance_test.py`.

Key features:
- HOME in joint space via JointTrajectory
- Waypoints cartesian via MoveToPose
- Monitors true execution using Pinocchio FK
- Reads avoidance diagnostics and prints compact, readable logs
- Can run fully automatically (no ENTER) via `require_user_confirm:=false`

"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from enum import Enum
from dataclasses import dataclass
from typing import List, Optional
import numpy as np
import threading
import time
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import PlanningScene, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

from franka_simulation.action import MoveToPose

import pinocchio as pin
from ament_index_python.packages import get_package_share_directory
import subprocess
import os


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


HOME_JOINT_POSITION = np.array([
    0.0,
    -0.785398,
    0.0,
    -2.35619,
    0.0,
    1.5708,
    0.785398,
])


class SafeAvoidanceTest(Node):

    def __init__(self):
        super().__init__("safe_avoidance_test")

        self.state = State.INIT
        self.waypoints: List[Waypoint] = []
        self.wp_index = 0
        self.current_wp: Optional[Waypoint] = None

        self.user_ok = False
        self._confirm_request = threading.Event()
        self._confirm_received = threading.Event()
        self._waiting_reason = None

        self.q = np.zeros(7)
        self.qd = np.zeros(7)
        self.avoidance_vel = np.zeros(7)
        self.blended_vel = np.zeros(7)
        self.min_dist = float("inf")
        self.j_row = np.zeros(7)
        self.hazard = "none"

        self._last_err = None
        self._last_err_wall = None
        self._stall_start_wall = None

        self._goal_result_received = False
        self._exec_start_wall = None
        self._reach_entered_wall = None

        self._last_blender_traj_wall = None
        self._last_blender_traj_points = 0
        self._last_blender_traj_span_rad = float("nan")
        self._traj_wait_started_wall = None
        self._fallback_used_for_current_goal = False

        self._ik_fallback_future = None
        self._ik_fallback_start_wall = None
        self._ik_fallback_target = None

        self._obstacles = {}
        self._obstacles_seen_once = False

        self._last_log_active = None
        self._last_log_hazard = None
        self._no_obstacles_warn_wall = 0.0

        self._init_pinocchio()

        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.create_subscription(Float64MultiArray, "/avoidance/velocity", self._avoid_cb, 10)
        self.create_subscription(Float64MultiArray, "/avoidance/min_distance", self._min_dist_cb, 10)
        self.create_subscription(Float64MultiArray, "/avoidance/jacobian", self._jac_cb, 10)
        self.create_subscription(Float64MultiArray, "/fr3_velocity_controller/commands", self._cmd_cb, 10)
        self.create_subscription(PlanningScene, "/obstacle_scene", self._scene_cb, 10)
        self.create_subscription(String, "/avoidance/hazard", self._hazard_cb, 10)
        self.create_subscription(JointTrajectory, "/velocity_blender/trajectory", self._blender_traj_cb, 10)

        self.joint_traj_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10,
        )
        self.velocity_blender_traj_pub = self.create_publisher(
            JointTrajectory,
            "/velocity_blender/trajectory",
            10,
        )

        self.action_client = ActionClient(self, MoveToPose, "move_to_pose")
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        self.monitor_timer = self.create_timer(0.1, self._monitor)
        self.debug_timer = self.create_timer(0.2, self._debug)

        self.declare_parameter("log_every_n", 5)
        self._log_every_n = int(self.get_parameter("log_every_n").value)
        self._debug_tick = 0

        self.declare_parameter("reach_tolerance_m", 0.03)
        self.declare_parameter("settle_time_s", 0.4)
        self.declare_parameter("max_exec_time_s", 25.0)
        self.declare_parameter("qd_settle_threshold", 0.05)
        self.declare_parameter("cmd_settle_threshold", 0.02)

        self._reach_tol = float(self.get_parameter("reach_tolerance_m").value)
        self._settle_time_s = float(self.get_parameter("settle_time_s").value)
        self._max_exec_time_s = float(self.get_parameter("max_exec_time_s").value)
        self._qd_settle_thr = float(self.get_parameter("qd_settle_threshold").value)
        self._cmd_settle_thr = float(self.get_parameter("cmd_settle_threshold").value)

        self.declare_parameter("traj_wait_timeout_s", 2.0)
        self.declare_parameter("fallback_traj_time_s", 4.0)
        self._traj_wait_timeout_s = float(self.get_parameter("traj_wait_timeout_s").value)
        self._fallback_traj_time_s = float(self.get_parameter("fallback_traj_time_s").value)

        # MoveIt execution speed scaling (important for avoidance demos)
        self.declare_parameter("moveit_velocity_scaling", 0.06)
        self.declare_parameter("moveit_acceleration_scaling", 0.06)
        self._moveit_vel_scale = float(self.get_parameter("moveit_velocity_scaling").value)
        self._moveit_acc_scale = float(self.get_parameter("moveit_acceleration_scaling").value)

        self.declare_parameter("fallback_stuck_min_dist_m", 0.25)
        self._fallback_stuck_min_dist_m = float(self.get_parameter("fallback_stuck_min_dist_m").value)

        self.declare_parameter("confirm_after_home", False)
        self._confirm_after_home = bool(self.get_parameter("confirm_after_home").value)

        self.declare_parameter("require_user_confirm", False)
        self._require_user_confirm = bool(self.get_parameter("require_user_confirm").value)

        self.declare_parameter("use_ansi", False)
        self._use_ansi = bool(self.get_parameter("use_ansi").value)
        self.declare_parameter("status_compact", True)
        self._status_compact = bool(self.get_parameter("status_compact").value)

        if self._require_user_confirm or self._confirm_after_home:
            threading.Thread(target=self._keyboard_listener, daemon=True).start()

        self.get_logger().info("✅ SafeAvoidanceTest node ready")

    @staticmethod
    def _fmt_xyz(x: float, y: float, z: float, prec: int = 3) -> str:
        return f"({x:+.{prec}f},{y:+.{prec}f},{z:+.{prec}f})"

    def _status_line(self, wp: str, err_m: float, d_min_m: float, haz: str, cmd_norm: float, avoid_norm: float, near_txt: str) -> str:
        def f3(x: float) -> str:
            if x is None or (isinstance(x, float) and (math.isnan(x) or math.isinf(x))):
                return "nan"
            return f"{x:.3f}"

        haz = (haz or "none")
        if len(haz) > 40:
            haz = haz[:39] + "…"
        return (
            f"[SAFE] {wp} "
            f"err={f3(err_m)}m d={f3(d_min_m)}m haz={haz} "
            f"|cmd|={f3(cmd_norm)} |avoid|={f3(avoid_norm)} "
            f"near={near_txt}"
        )

    def _get_nearest_obstacle(self, p_world: np.ndarray):
        if not self._obstacles:
            return None, float("inf")
        best_id = None
        best_d = float("inf")
        for oid, o in self._obstacles.items():
            pose = o.get("pose")
            if pose is None:
                continue
            c = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
            d = float(np.linalg.norm(p_world - c))
            if d < best_d:
                best_d = d
                best_id = oid
        return best_id, best_d

    def _init_pinocchio(self):
        xacro_file = os.path.join(
            get_package_share_directory("franka_description"),
            "robots",
            "fr3",
            "fr3.urdf.xacro",
        )
        urdf = subprocess.check_output(
            [
                "xacro",
                xacro_file,
                "ros2_control:=false",
                "hand:=true",
                "arm_id:=fr3",
            ]
        ).decode()

        model_full = pin.buildModelFromXML(urdf)
        locked = [model_full.getJointId(n) for n in model_full.names if "finger" in n]
        self.model = pin.buildReducedModel(model_full, locked, pin.neutral(model_full))
        self.data = self.model.createData()

        ee_candidates = ["fr3_hand_tcp", "fr3_link8"]
        self.ee_frame = None
        for fname in ee_candidates:
            try:
                fid = self.model.getFrameId(fname)
                if fid is not None and int(fid) >= 0:
                    self.ee_frame = fid
                    self.get_logger().info(f"✔ Using EE frame: {fname}")
                    break
            except Exception:
                continue
        if self.ee_frame is None:
            self.ee_frame = self.model.getFrameId(self.model.frames[-1].name)
            self.get_logger().warn(
                f"⚠️ Could not find fr3_hand_tcp/fr3_link8; using fallback frame: {self.model.frames[self.ee_frame].name}"
            )

        self.get_logger().info("✔ Pinocchio model loaded")

    def ee_position(self) -> np.ndarray:
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.ee_frame].translation

    def _js_cb(self, msg: JointState):
        names = [
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ]
        try:
            idx = [msg.name.index(n) for n in names]
            self.q = np.array([msg.position[i] for i in idx], dtype=float)
            if hasattr(msg, "velocity") and len(msg.velocity) >= len(msg.name):
                self.qd = np.array([msg.velocity[i] for i in idx], dtype=float)
            else:
                self.qd = np.zeros(7, dtype=float)
        except (ValueError, IndexError):
            return

    def _avoid_cb(self, msg: Float64MultiArray):
        if len(msg.data) == 7:
            self.avoidance_vel = np.array(msg.data)

    def _min_dist_cb(self, msg: Float64MultiArray):
        if len(msg.data) > 0:
            self.min_dist = float(msg.data[0])

    def _jac_cb(self, msg: Float64MultiArray):
        if len(msg.data) == 7:
            self.j_row = np.array(msg.data, dtype=float)
        else:
            self.j_row = np.zeros(7)

    def _cmd_cb(self, msg: Float64MultiArray):
        if len(msg.data) == 7:
            self.blended_vel = np.array(msg.data)

    def _hazard_cb(self, msg: String):
        if msg.data:
            self.hazard = str(msg.data)

    def _blender_traj_cb(self, msg: JointTrajectory):
        self._last_blender_traj_wall = time.time()
        self._last_blender_traj_points = len(msg.points)
        span = float("nan")
        try:
            if len(msg.points) >= 2:
                p0 = np.array(msg.points[0].positions, dtype=float)
                p1 = np.array(msg.points[-1].positions, dtype=float)
                if p0.shape == (7,) and p1.shape == (7,):
                    span = float(np.linalg.norm(p1 - p0))
        except Exception:
            span = float("nan")
        self._last_blender_traj_span_rad = span

    def _scene_cb(self, msg: PlanningScene):
        now_wall = time.time()
        for co in msg.world.collision_objects:
            if not co.primitives or not co.primitive_poses:
                continue
            oid = co.id
            pose = co.primitive_poses[0]
            prim = co.primitives[0]
            frame_id = co.header.frame_id
            self._obstacles[oid] = {
                "frame_id": frame_id,
                "pose": pose,
                "primitive": prim,
                "stamp_wall": now_wall,
            }
        if not self._obstacles_seen_once and self._obstacles:
            self._obstacles_seen_once = True

    def send_home(self):
        self.get_logger().info("🏠 Moving to HOME (joint space)")
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
        pt.positions = HOME_JOINT_POSITION.tolist()
        pt.time_from_start.sec = 4
        traj.points.append(pt)
        self.velocity_blender_traj_pub.publish(traj)
        self.joint_traj_pub.publish(traj)
        self.state = State.GOING_HOME

    def send_next_waypoint(self):
        if self.wp_index >= len(self.waypoints):
            self.state = State.COMPLETED
            self.get_logger().info("🏁 All waypoints completed")
            return

        wp = self.waypoints[self.wp_index]
        self.current_wp = wp
        self.state = State.SENDING_GOAL

        self._goal_result_received = False
        self._exec_start_wall = time.time()
        self._reach_entered_wall = None

        self._traj_wait_started_wall = time.time()
        self._fallback_used_for_current_goal = False

        goal = MoveToPose.Goal()
        goal.pose_target.header.frame_id = "world"
        goal.pose_target.pose.position.x = wp.x
        goal.pose_target.pose.position.y = wp.y
        goal.pose_target.pose.position.z = wp.z

        goal.pose_target.pose.orientation.x = 1.0
        goal.pose_target.pose.orientation.y = 0.0
        goal.pose_target.pose.orientation.z = 0.0
        goal.pose_target.pose.orientation.w = 0.0

        goal.max_velocity_scaling_factor = float(self._moveit_vel_scale)
        goal.max_acceleration_scaling_factor = float(self._moveit_acc_scale)

        self.get_logger().info(f"➡ Sending waypoint {wp.name}")
        self.action_client.wait_for_server()
        self.action_client.send_goal_async(goal).add_done_callback(self._goal_response_cb)

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
        self._goal_result_received = True

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
            self.user_ok = True
            self._confirm_received.set()

    def _request_user_confirm(self, reason: str):
        self._waiting_reason = str(reason)
        self.user_ok = False
        self._confirm_received.clear()
        self._confirm_request.set()

    def _monitor(self):
        if self.state == State.INIT:
            self.send_home()

        elif self.state == State.GOING_HOME:
            if np.linalg.norm(self.q - HOME_JOINT_POSITION) < 0.02:
                self.get_logger().info("✅ HOME reached")
                if self._confirm_after_home:
                    self._request_user_confirm(reason="start waypoints")
                    self.state = State.WAIT_USER
                else:
                    self.state = State.READY

        elif self.state == State.READY:
            self.send_next_waypoint()

        elif self.state == State.WAIT_USER:
            if self._confirm_received.is_set():
                self._confirm_received.clear()
                self.user_ok = False
                if self._waiting_reason == "after waypoint":
                    self.wp_index += 1
                self._waiting_reason = None
                self.state = State.READY

        elif self.state == State.EXECUTING:
            if self.current_wp is None:
                return

            now = time.time()
            if self._exec_start_wall is None:
                self._exec_start_wall = now

            ee = self.ee_position()
            target = np.array([self.current_wp.x, self.current_wp.y, self.current_wp.z], dtype=float)
            err = float(np.linalg.norm(ee - target))

            if (now - self._exec_start_wall) > self._max_exec_time_s:
                self.get_logger().error(
                    f"⏱️ Timeout while executing {self.current_wp.name}: err={err:.3f}m hazard={self.hazard} d_min={self.min_dist:.3f}"
                )
                self.state = State.FAILED
                return

            cmd_norm = float(np.linalg.norm(self.blended_vel))
            qd_norm = float(np.linalg.norm(self.qd))
            settled = (cmd_norm < self._cmd_settle_thr) and (qd_norm < self._qd_settle_thr)

            if err <= self._reach_tol and settled:
                if self._reach_entered_wall is None:
                    self._reach_entered_wall = now
                elif (now - self._reach_entered_wall) >= self._settle_time_s:
                    self.get_logger().info(
                        f"✅ {self.current_wp.name} reached | err={err:.4f}m hazard={self.hazard} d_min={self.min_dist:.3f}"
                    )
                    if self._require_user_confirm:
                        self._request_user_confirm(reason="after waypoint")
                        self.state = State.WAIT_USER
                    else:
                        self.wp_index += 1
                        self.state = State.READY
            else:
                self._reach_entered_wall = None

    def _debug(self):
        if self.state != State.EXECUTING:
            return
        self._debug_tick += 1

        qdot_avoid = self.avoidance_vel
        qdot_cmd = self.blended_vel

        av = float(np.linalg.norm(qdot_avoid))
        bv = float(np.linalg.norm(qdot_cmd))

        d = float(self.min_dist)
        wp = self.current_wp.name if self.current_wp else "(none)"

        ee = self.ee_position()
        if self.current_wp is not None:
            target = np.array([self.current_wp.x, self.current_wp.y, self.current_wp.z], dtype=float)
            err = float(np.linalg.norm(ee - target))
        else:
            err = float("nan")

        near_id, near_d = self._get_nearest_obstacle(ee)
        near_txt = f"{near_id}@{near_d:.3f}m" if near_id is not None else "(none)"

        active = (d < 0.30) and (av > 1e-3)
        haz = (self.hazard or "none")

        event = (self._last_log_active is None) or (active != self._last_log_active) or (haz != self._last_log_hazard)
        periodic = (self._log_every_n <= 1) or ((self._debug_tick % self._log_every_n) == 0)
        if not (event or periodic):
            return

        if (
            (not self._obstacles_seen_once)
            and (self._exec_start_wall is not None)
            and ((time.time() - self._exec_start_wall) > 2.0)
            and ((time.time() - self._no_obstacles_warn_wall) > 10.0)
        ):
            self._no_obstacles_warn_wall = time.time()
            self.get_logger().warn("⚠️ No obstacles received on /obstacle_scene yet — avoidance will not activate.")

        self.get_logger().info(
            self._status_line(
                wp=str(wp),
                err_m=float(err),
                d_min_m=float(d),
                haz=str(haz),
                cmd_norm=float(bv),
                avoid_norm=float(av),
                near_txt=str(near_txt),
            )
        )

        self._last_log_active = active
        self._last_log_hazard = haz


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
        Waypoint(0.40, -0.50, 0.25, "WP9_DIAGONAL"),
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
