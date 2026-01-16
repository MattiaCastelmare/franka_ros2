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
import time
import math

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import PlanningScene, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import GetParameters

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

        # User confirmation synchronization (avoid consuming ENTER too early).
        self._confirm_request = threading.Event()
        self._confirm_received = threading.Event()
        self._waiting_reason = None  # str

        # -------------------------------
        # Robot state
        # -------------------------------
        self.q = np.zeros(7)
        self.qd = np.zeros(7)
        self.avoidance_vel = np.zeros(7)
        self.blended_vel = np.zeros(7)
        self.min_dist = float("inf")
        self.j_row = np.zeros(7)
        self.hazard = "none"

        # Progress / stall detection
        self._last_err = None
        self._last_err_wall = None
        self._stall_start_wall = None

        # Execution tracking (because MoveToPose is PLANNING ONLY and returns early)
        self._goal_result_received = False
        self._exec_start_wall = None
        self._reach_entered_wall = None

        # Trajectory reception tracking (what the blender should follow)
        self._last_blender_traj_wall = None
        self._last_blender_traj_points = 0
        self._last_blender_traj_span_rad = float('nan')
        self._traj_wait_started_wall = None
        self._fallback_used_for_current_goal = False

        # Async IK fallback state (avoid blocking the executor)
        self._ik_fallback_future = None
        self._ik_fallback_start_wall = None
        self._ik_fallback_target = None

        # -------------------------------
        # Obstacles (from obstacle_synchronizer / PlanningScene)
        # -------------------------------
        # {id: {frame_id: str, pose: Pose, primitive: SolidPrimitive, stamp_wall: float}}
        self._obstacles = {}
        self._obstacles_seen_once = False

        # Logging helpers (keep output informative but not noisy)
        self._last_log_wall = 0.0
        self._last_log_active = None
        self._last_log_hazard = None
        self._no_obstacles_warn_wall = 0.0

        # -------------------------------
        # Pinocchio
        # -------------------------------
        self._init_pinocchio()

        # -------------------------------
        # ROS interfaces
        # -------------------------------
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.create_subscription(Float64MultiArray, "/avoidance/velocity", self._avoid_cb, 10)
        self.create_subscription(Float64MultiArray, "/avoidance/min_distance", self._min_dist_cb, 10)
        self.create_subscription(Float64MultiArray, "/avoidance/jacobian", self._jac_cb, 10)
        self.create_subscription(Float64MultiArray, "/fr3_velocity_controller/commands", self._cmd_cb, 10)
        self.create_subscription(PlanningScene, "/obstacle_scene", self._scene_cb, 10)
        self.create_subscription(String, "/avoidance/hazard", self._hazard_cb, 10)

        # Observe what trajectory actually reaches the velocity blender topic
        self.create_subscription(
            JointTrajectory,
            "/velocity_blender/trajectory",
            self._blender_traj_cb,
            10,
        )

        self.joint_traj_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )

        # In this stack, actual motion is performed by velocity_control_blender via /velocity_blender/trajectory.
        # Some launch configurations do not start joint_trajectory_controller at all, so we also publish HOME here.
        self.velocity_blender_traj_pub = self.create_publisher(
            JointTrajectory,
            "/velocity_blender/trajectory",
            10,
        )

        # Pause control for the velocity blender (ensures the robot truly stops between waypoints)
        self._pause_pub = self.create_publisher(Bool, "/velocity_blender/pause", 10)
        self._paused_last = None  # Optional[bool]

        self.action_client = ActionClient(self, MoveToPose, "move_to_pose")

        # IK service (fallback when MoveIt planning does not provide a usable trajectory)
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        # -------------------------------
        # Timers
        # -------------------------------
        self.monitor_timer = self.create_timer(0.1, self._monitor)
        self.debug_timer = self.create_timer(0.2, self._debug)

        # Debug / logging settings
        self.declare_parameter("log_every_n", 5)  # every N debug ticks (0.2s * N)
        self._log_every_n = int(self.get_parameter("log_every_n").value)
        self._debug_tick = 0

        # Reach/settle criteria (EE space)
        self.declare_parameter("reach_tolerance_m", 0.03)
        self.declare_parameter("settle_time_s", 0.4)
        # If <= 0: disable hard timeout (recommended for interactive demos where avoidance may slow down).
        self.declare_parameter("max_exec_time_s", 0.0)
        self.declare_parameter("qd_settle_threshold", 0.05)   # rad/s
        self.declare_parameter("cmd_settle_threshold", 0.02)  # rad/s

        self._reach_tol = float(self.get_parameter("reach_tolerance_m").value)
        self._settle_time_s = float(self.get_parameter("settle_time_s").value)
        self._max_exec_time_s = float(self.get_parameter("max_exec_time_s").value)
        self._qd_settle_thr = float(self.get_parameter("qd_settle_threshold").value)
        self._cmd_settle_thr = float(self.get_parameter("cmd_settle_threshold").value)

        # If no valid trajectory appears after sending a goal, compute IK and publish a simple trajectory.
        self.declare_parameter("traj_wait_timeout_s", 2.0)
        self.declare_parameter("fallback_traj_time_s", 4.0)
        self._traj_wait_timeout_s = float(self.get_parameter("traj_wait_timeout_s").value)
        self._fallback_traj_time_s = float(self.get_parameter("fallback_traj_time_s").value)

        # MoveIt execution speed scaling (important for avoidance demos)
        self.declare_parameter("moveit_velocity_scaling", 0.06)
        self.declare_parameter("moveit_acceleration_scaling", 0.06)
        self._moveit_vel_scale = float(self.get_parameter("moveit_velocity_scaling").value)
        self._moveit_acc_scale = float(self.get_parameter("moveit_acceleration_scaling").value)

        # Optional: sanity-check that key parameters are actually loaded by the running stack.
        # This is very useful when you have multiple workspaces (e.g., /home/... vs /ros2_ws/...).
        self.declare_parameter("stack_param_check_enable", True)
        self.declare_parameter("stack_param_check_delay_s", 2.0)
        self._stack_param_check_enable = bool(self.get_parameter("stack_param_check_enable").value)
        self._stack_param_check_delay_s = float(self.get_parameter("stack_param_check_delay_s").value)
        self._stack_param_checked = False
        self._stack_param_check_start_wall = time.time()

        # Safety: do NOT trigger the IK fallback just because the robot is "stuck" when close to obstacles.
        # The fallback trajectory ignores collisions and can cause impacts.
        # We only allow the "stuck"-based fallback when we are clearly far from obstacles.
        self.declare_parameter("fallback_stuck_min_dist_m", 0.25)
        self._fallback_stuck_min_dist_m = float(self.get_parameter("fallback_stuck_min_dist_m").value)

        # UX: optionally require ENTER after reaching HOME before starting WP0
        self.declare_parameter("confirm_after_home", True)
        self._confirm_after_home = bool(self.get_parameter("confirm_after_home").value)

        # UX: require ENTER after each waypoint (interactive mode). If false, auto-advance.
        self.declare_parameter("require_user_confirm", True)
        self._require_user_confirm = bool(self.get_parameter("require_user_confirm").value)

        # Timeout behavior: avoid freezing the whole demo.
        # - "wait_user": pause + wait for ENTER, then advance to next waypoint (best-effort)
        # - "advance": auto-advance to next waypoint
        # - "ignore": keep waiting forever (not recommended)
        # - "fail": old behavior (State.FAILED)
        # Default: do NOT block the demo on timeout; advance best-effort.
        self.declare_parameter("timeout_action", "advance")
        self._timeout_action = str(self.get_parameter("timeout_action").value)

        # Best-effort stall handling (avoid infinite hang without declaring failure)
        self.declare_parameter("stall_best_effort_enable", True)
        self.declare_parameter("stall_best_effort_time_s", 6.0)
        # When stalled too long: either "advance" (default) or "wait_user".
        self.declare_parameter("stall_action", "advance")
        self.declare_parameter("stall_cmd_threshold", 0.01)
        self.declare_parameter("stall_err_threshold_m", 0.05)
        self._stall_best_effort_enable = bool(self.get_parameter("stall_best_effort_enable").value)
        self._stall_best_effort_time_s = float(self.get_parameter("stall_best_effort_time_s").value)
        self._stall_action = str(self.get_parameter("stall_action").value)
        self._stall_cmd_threshold = float(self.get_parameter("stall_cmd_threshold").value)
        self._stall_err_threshold_m = float(self.get_parameter("stall_err_threshold_m").value)

        # Logging: when to consider avoidance "active" (for SAFE/AVOIDANCE flag)
        self.declare_parameter("avoidance_active_distance_m", 0.35)
        self._avoidance_active_distance_m = float(self.get_parameter("avoidance_active_distance_m").value)

        # Logging style
        self.declare_parameter("use_ansi", False)
        self._use_ansi = bool(self.get_parameter("use_ansi").value)
        self.declare_parameter("status_compact", True)
        self._status_compact = bool(self.get_parameter("status_compact").value)

        # -------------------------------
        # Keyboard listener (NON BLOCKING)
        # -------------------------------
        # Start it only if interactive confirmation is enabled.
        if self._require_user_confirm or self._confirm_after_home:
            threading.Thread(target=self._keyboard_listener, daemon=True).start()

        self.get_logger().info("✅ SafeAvoidanceTest node ready")

        if self._stack_param_check_enable:
            self.create_timer(0.2, self._stack_param_check_tick)

        # Ensure we start unpaused so HOME can execute immediately.
        self._set_blender_pause(False)


    def _set_blender_pause(self, paused: bool):
        """Best-effort pause control for velocity_control_blender."""
        try:
            p = bool(paused)
            if self._paused_last is not None and (bool(self._paused_last) == p):
                return
            msg = Bool()
            msg.data = p
            self._pause_pub.publish(msg)
            self._paused_last = p
        except Exception:
            pass


    def _stack_param_check_tick(self):
        """One-shot param dump from the running stack (avoidance + blender)."""
        if self._stack_param_checked:
            return
        if (time.time() - self._stack_param_check_start_wall) < self._stack_param_check_delay_s:
            return

        def fetch(node_name: str, keys: List[str]):
            cli = self.create_client(GetParameters, f"/{node_name}/get_parameters")
            if not cli.wait_for_service(timeout_sec=0.5):
                return None
            req = GetParameters.Request()
            req.names = list(keys)
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=1.0)
            if fut.result() is None:
                return None
            out = {}
            for k, v in zip(keys, fut.result().values):
                try:
                    if v.type == v.TYPE_DOUBLE:
                        out[k] = float(v.double_value)
                    elif v.type == v.TYPE_INTEGER:
                        out[k] = int(v.integer_value)
                    elif v.type == v.TYPE_BOOL:
                        out[k] = bool(v.bool_value)
                    elif v.type == v.TYPE_STRING:
                        out[k] = str(v.string_value)
                    elif v.type == v.TYPE_DOUBLE_ARRAY:
                        out[k] = [float(x) for x in v.double_array_value]
                    elif v.type == v.TYPE_INTEGER_ARRAY:
                        out[k] = [int(x) for x in v.integer_array_value]
                    elif v.type == v.TYPE_STRING_ARRAY:
                        out[k] = [str(x) for x in v.string_array_value]
                    else:
                        out[k] = "(unhandled)"
                except Exception:
                    out[k] = "(error)"
            return out

        avoid_keys = [
            "influence_distance",
            "aggressive_distance",
            "safety_margin",
            "nullspace_gain",
            "tangential_gain",
            "aggressive_gain_scale",
            "max_joint_velocity",
        ]
        blend_keys = [
            "max_vel",
            "kp",
            "influence_distance",
            "safety_margin",
            "avoidance_weight_max",
            "d_dot_min_close",
            "d_dot_push_gain",
            "d_dot_push_max",
        ]

        avoid = fetch("online_avoidance_controller", avoid_keys)
        blend = fetch("velocity_control_blender", blend_keys)

        if avoid is None:
            self.get_logger().warn(
                "⚠️ Param check: /online_avoidance_controller/get_parameters not available. "
                "Are you running the full stack (launch) in the same ROS_DOMAIN_ID?"
            )
        else:
            self.get_logger().info(f"[PARAM] online_avoidance_controller: {avoid}")

        if blend is None:
            self.get_logger().warn(
                "⚠️ Param check: /velocity_control_blender/get_parameters not available. "
                "Is the blender node running?"
            )
        else:
            self.get_logger().info(f"[PARAM] velocity_control_blender: {blend}")

        self.get_logger().info(
            f"[PARAM] avoidance_test MoveIt scaling: vel={self._moveit_vel_scale}, acc={self._moveit_acc_scale}"
        )

        self._stack_param_checked = True


    # ======================================================
    # STATUS LINE FORMATTING
    # ======================================================

    def _status_line(
        self,
        wp: str,
        err_m: float,
        d_min_m: float,
        haz: str,
        cmd_norm: float,
        track_norm: float,
        avoid_norm: float,
        near_txt: str,
        avoidance_active: bool,
        use_ansi: bool,
    ) -> str:
        # Keep it short and greppable.
        # Example:
        # [SAFE] WP5 err=0.042 d=0.118 haz=external:box |cmd|=0.082 |avoid|=0.031 near=obstacle_box@0.221
        def f3(x: float) -> str:
            if x is None or (isinstance(x, float) and (math.isnan(x) or math.isinf(x))):
                return "nan"
            return f"{x:.3f}"

        haz = (haz or "none")
        if len(haz) > 40:
            haz = haz[:39] + "…"
        if use_ansi:
            RESET = "\033[0m"
            RED = "\033[31m"
            GREEN = "\033[32m"
        else:
            RESET = ""
            RED = ""
            GREEN = ""

        if avoidance_active:
            tag = f"{RED}[AVOIDANCE]{RESET}"
        else:
            tag = f"{GREEN}[SAFE]{RESET}"

        return (
            f"{tag} {wp} "
            f"err={f3(err_m)}m d={f3(d_min_m)}m haz={haz} "
            f"|cmd|={f3(cmd_norm)} |track|={f3(track_norm)} |avoid|={f3(avoid_norm)} "
            f"near={near_txt}"
        )


    # ======================================================
    # SMALL UTILITIES (LOGGING)
    # ======================================================

    @staticmethod
    def _fmt_xyz(x: float, y: float, z: float, prec: int = 3) -> str:
        return f"({x:+.{prec}f},{y:+.{prec}f},{z:+.{prec}f})"

    @staticmethod
    def _fmt_dim(dim: List[float], prec: int = 3) -> str:
        if dim is None:
            return "(?)"
        try:
            return f"({dim[0]:.{prec}f},{dim[1]:.{prec}f},{dim[2]:.{prec}f})"
        except Exception:
            return "(?)"

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

        # IMPORTANT:
        # Our waypoint goals are expressed for fr3_hand_tcp (see motion_server + MoveToPose usage).
        # If we monitor fr3_link8 instead, the Z error will include the tool/TCP offset and the
        # test will think it's "stalled" even when the robot correctly reached the goal.
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
            # Very defensive fallback (should not happen)
            self.ee_frame = self.model.getFrameId(self.model.frames[-1].name)
            self.get_logger().warn(
                f"⚠️ Could not find fr3_hand_tcp/fr3_link8; using fallback frame: {self.model.frames[self.ee_frame].name}"
            )

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
            idx = [msg.name.index(n) for n in names]
            self.q = np.array([msg.position[i] for i in idx], dtype=float)

            # velocity can be missing/short depending on broadcaster config
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
        # Track whether a *new* trajectory actually arrives to the blender.
        self._last_blender_traj_wall = time.time()
        self._last_blender_traj_points = len(msg.points)

        # Detect degenerate trajectories (all points ~ identical), which would make the blender stop.
        span = float('nan')
        try:
            if len(msg.points) >= 2:
                p0 = np.array(msg.points[0].positions, dtype=float)
                p1 = np.array(msg.points[-1].positions, dtype=float)
                if p0.shape == (7,) and p1.shape == (7,):
                    span = float(np.linalg.norm(p1 - p0))
        except Exception:
            span = float('nan')
        self._last_blender_traj_span_rad = span

        # Log a short summary (rate-limited implicitly by incoming messages)
        try:
            dur_s = 0.0
            if msg.points:
                last = msg.points[-1].time_from_start
                dur_s = float(last.sec) + float(last.nanosec) * 1e-9
            jn = len(getattr(msg, 'joint_names', []) or [])
            span_txt = ""
            if np.isfinite(self._last_blender_traj_span_rad):
                span_txt = f" span≈{self._last_blender_traj_span_rad:.3f}rad"
            self.get_logger().info(
                f"📥 /velocity_blender/trajectory received: points={len(msg.points)} joint_names={jn} duration≈{dur_s:.2f}s{span_txt}"
            )
        except Exception:
            self.get_logger().info(
                f"📥 /velocity_blender/trajectory received: points={len(msg.points)}"
            )

    def _scene_cb(self, msg: PlanningScene):
        # Store latest obstacles for lightweight context (e.g. nearest obstacle).
        # Do NOT spam obstacle lists in the logs.
        now_wall = time.time()

        for co in msg.world.collision_objects:
            if not co.primitives or not co.primitive_poses:
                continue

            oid = co.id
            pose = co.primitive_poses[0]
            prim = co.primitives[0]
            frame_id = co.header.frame_id

            prev = self._obstacles.get(oid)
            self._obstacles[oid] = {
                "frame_id": frame_id,
                "pose": pose,
                "primitive": prim,
                "stamp_wall": now_wall,
            }

        if not self._obstacles_seen_once and self._obstacles:
            self._obstacles_seen_once = True


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
        # Allow motion during HOME
        self._set_blender_pause(False)
        traj = JointTrajectory()
        traj.joint_names = [
            "fr3_joint1", "fr3_joint2", "fr3_joint3",
            "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"
        ]

        pt = JointTrajectoryPoint()
        pt.positions = HOME_JOINT_POSITION.tolist()
        pt.time_from_start.sec = 4

        traj.points.append(pt)

        # Preferred path: publish to velocity blender (used by the simulation stack)
        self.velocity_blender_traj_pub.publish(traj)

        # Backward-compatible path: if a joint trajectory controller is running, publish there too.
        self.joint_traj_pub.publish(traj)

        self.get_logger().info(
            "   ↳ Published HOME trajectory to /velocity_blender/trajectory (and also to /joint_trajectory_controller/joint_trajectory if available)"
        )
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

        # Reset execution trackers
        self._goal_result_received = False
        self._exec_start_wall = time.time()
        self._reach_entered_wall = None

        # Track whether a new trajectory is actually delivered to the blender
        self._traj_wait_started_wall = time.time()
        self._fallback_used_for_current_goal = False

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

        goal.max_velocity_scaling_factor = float(self._moveit_vel_scale)
        goal.max_acceleration_scaling_factor = float(self._moveit_acc_scale)

        # Allow motion while executing a waypoint
        self._set_blender_pause(False)

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

        # IMPORTANT:
        # The server is PLANNING ONLY and returns SUCCESS after publishing a trajectory.
        # We must not mark the waypoint as reached here; instead we continue monitoring
        # the actual robot motion until EE error is within tolerance and the robot is settled.
        self._goal_result_received = True


    # ======================================================
    # MONITOR
    # ======================================================

    def _monitor(self):
        if self.state == State.INIT:
            self.send_home()

        elif self.state == State.GOING_HOME:
            if np.linalg.norm(self.q - HOME_JOINT_POSITION) < 0.02:
                self.get_logger().info("✅ HOME reached")
                if self._confirm_after_home:
                    # Optional confirmation before starting waypoint execution.
                    self._request_user_confirm(reason="start waypoints")
                    self.state = State.WAIT_USER
                else:
                    self.state = State.READY

        elif self.state == State.READY:
            self.send_next_waypoint()

        elif self.state == State.WAIT_USER:
            # Only advance once a fresh confirmation has been received.
            if self._confirm_received.is_set():
                self._confirm_received.clear()
                self.user_ok = False

                # Resume execution
                self._set_blender_pause(False)

                # If we were waiting after a waypoint, advance to the next one.
                # If we were waiting after HOME, start from WP0 without incrementing.
                if self._waiting_reason in ("after waypoint", "timeout waypoint", "stalled waypoint"):
                    self.wp_index += 1

                self._waiting_reason = None
                self.state = State.READY

        elif self.state == State.EXECUTING:
            # Wait for actual reaching (EE error + settle), not just action result.
            if self.current_wp is None:
                return

            now = time.time()
            if self._exec_start_wall is None:
                self._exec_start_wall = now

            ee = self.ee_position()
            target = np.array([self.current_wp.x, self.current_wp.y, self.current_wp.z], dtype=float)
            err = float(np.linalg.norm(ee - target))

            # ----------------------------------------------------------
            # Fallback: if no valid trajectory shows up on /velocity_blender/trajectory,
            # publish a simple IK-based joint trajectory so the robot keeps moving.
            # This unblocks cases where MoveIt planning succeeds but doesn't produce
            # a usable trajectory for the blender (or publishing fails).
            # ----------------------------------------------------------
            if (
                (not self._fallback_used_for_current_goal)
                and (self._traj_wait_started_wall is not None)
                and ((now - self._traj_wait_started_wall) >= self._traj_wait_timeout_s)
            ):
                traj_is_missing = (
                    self._last_blender_traj_wall is None
                    or self._last_blender_traj_wall < self._traj_wait_started_wall
                )
                traj_is_trivial = (
                    (self._last_blender_traj_points <= 1)
                    or (np.isfinite(self._last_blender_traj_span_rad) and self._last_blender_traj_span_rad < 0.01)
                )

                # Also consider "stuck" case: command is ~0 while far from target.
                cmd_norm_now = float(np.linalg.norm(self.blended_vel))
                # NOTE: If we're close to obstacles, the blender may intentionally slow/stop.
                # In that case, starting an IK fallback would be unsafe.
                far_from_obstacles = (float(self.min_dist) >= self._fallback_stuck_min_dist_m)
                no_hazard = (str(self.hazard or "none") == "none")
                stuck_now = (err > 0.05) and (cmd_norm_now < 1e-3) and far_from_obstacles and no_hazard

                if traj_is_missing or traj_is_trivial or stuck_now:
                    # Kick off async IK request (do not block the timer callback)
                    self._start_ik_fallback_request(
                        target_xyz=target,
                        orientation_xyzw=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
                        frame_id="world",
                        duration_s=self._fallback_traj_time_s,
                    )
                    # Avoid spamming start attempts
                    self._traj_wait_started_wall = None

            # Process async IK fallback if one is in flight
            self._process_ik_fallback_if_ready(now_wall=now)

            # Timeout guard
            if (self._max_exec_time_s is not None) and (float(self._max_exec_time_s) > 0.0):
                if (now - self._exec_start_wall) > float(self._max_exec_time_s):
                    self.get_logger().warn(
                        f"⏱️ Timeout while executing {self.current_wp.name} (best-effort): "
                        f"err={err:.3f}m, hazard={self.hazard}, d_min={self.min_dist:.3f}. "
                        f"timeout_action={self._timeout_action}"
                    )

                    action = (self._timeout_action or "wait_user").strip().lower()
                    if action == "fail":
                        self.state = State.FAILED
                        return
                    if action == "ignore":
                        # Keep waiting indefinitely
                        return

                    # Either auto-advance or wait for user
                    if action == "wait_user":
                        self._set_blender_pause(True)
                        if self._require_user_confirm:
                            self.get_logger().info(
                                "⏸ Timeout reached — robot paused. Premi ENTER per passare al waypoint successivo (best-effort)."
                            )
                            self._request_user_confirm(reason="timeout waypoint")
                            self.state = State.WAIT_USER
                        else:
                            self.wp_index += 1
                            self.state = State.READY
                        return

                    # Default: advance (best-effort)
                    self._set_blender_pause(True)
                    self.get_logger().warn(
                        "⏭️ Timeout reached — advancing to next waypoint (best-effort)."
                    )
                    self.wp_index += 1
                    self.state = State.READY
                    return

            # Settle criteria
            cmd_norm = float(np.linalg.norm(self.blended_vel))
            qd_norm = float(np.linalg.norm(self.qd))
            settled = (cmd_norm < self._cmd_settle_thr) and (qd_norm < self._qd_settle_thr)

            if err <= self._reach_tol and settled:
                if self._reach_entered_wall is None:
                    self._reach_entered_wall = now
                elif (now - self._reach_entered_wall) >= self._settle_time_s:
                    self.get_logger().info(
                        f"✅ {self.current_wp.name} reached (measured) | "
                        f"target={self._fmt_xyz(float(target[0]), float(target[1]), float(target[2]), prec=3)} "
                        f"ee={self._fmt_xyz(float(ee[0]), float(ee[1]), float(ee[2]), prec=3)} "
                        f"err={err:.4f} m | hazard={self.hazard} d_min={self.min_dist:.3f}"
                    )
                    if self._require_user_confirm:
                        self.get_logger().info("⏸ Robot fermo — attendendo conferma utente (ENTER)")
                        # Enforce pause so we are truly stopped while waiting.
                        self._set_blender_pause(True)
                        self._request_user_confirm(reason="after waypoint")
                        self.state = State.WAIT_USER
                    else:
                        # Auto-advance to next waypoint.
                        self.wp_index += 1
                        self._waiting_reason = None
                        self.state = State.READY
            else:
                self._reach_entered_wall = None

            # Best-effort stall handling: if we're far from target and the robot is not moving for too long,
            # do NOT fail; pause and ask the user to continue to the next waypoint.
            if self._stall_best_effort_enable and (self._stall_best_effort_time_s > 0.0):
                cmd_norm_now = float(np.linalg.norm(self.blended_vel))
                far = float(err) > float(self._stall_err_threshold_m)
                slow = cmd_norm_now < float(self._stall_cmd_threshold)
                if far and slow:
                    if self._stall_start_wall is None:
                        self._stall_start_wall = now
                    elif (now - float(self._stall_start_wall)) >= float(self._stall_best_effort_time_s):
                        self.get_logger().warn(
                            f"[BEST-EFFORT] Stalled at {self.current_wp.name}: err={err:.3f}m, |cmd|={cmd_norm_now:.3f}. "
                            "Best-effort advance policy triggered."
                        )
                        action = (self._stall_action or "advance").strip().lower()
                        if action == "wait_user":
                            self._set_blender_pause(True)
                            if self._require_user_confirm:
                                self._request_user_confirm(reason="stalled waypoint")
                                self.state = State.WAIT_USER
                            else:
                                self.wp_index += 1
                                self.state = State.READY
                        else:
                            # Default: auto-advance
                            self._set_blender_pause(True)
                            self.get_logger().warn(
                                "⏭️ Stalled too long — advancing to next waypoint (best-effort)."
                            )
                            self.wp_index += 1
                            self.state = State.READY
                        return
                else:
                    self._stall_start_wall = None


    def _debug(self):
        # Log only while executing a waypoint
        if self.state != State.EXECUTING:
            return

        self._debug_tick += 1

        # What we can observe directly:
        # - qdot_avoid: output of online_avoidance_controller
        # - qdot_cmd: actual command sent to ros2_control (after blending)
        # We cannot observe the internal "pure tracking" command directly, but a useful proxy is:
        #   qdot_track_proxy ≈ qdot_cmd - qdot_avoid
        # This makes it easy to see if avoidance is actually contributing near obstacles.
        qdot_avoid = self.avoidance_vel
        qdot_cmd = self.blended_vel
        qdot_track_proxy = qdot_cmd - qdot_avoid

        av = float(np.linalg.norm(qdot_avoid))
        bv = float(np.linalg.norm(qdot_cmd))
        tv = float(np.linalg.norm(qdot_track_proxy))

        jn = float(np.linalg.norm(self.j_row))
        d = float(self.min_dist)

        # Simple "avoidance active" heuristic (for SAFE/AVOIDANCE flag)
        # Prefer hazard label when available; fall back to thresholds.
        haz_now = str(self.hazard or "none")
        active = (
            (haz_now != "none")
            or ((d < float(self._avoidance_active_distance_m)) and (av > 1e-3) and (jn > 1e-6))
        )

        # Alignment between avoidance and command (cosine similarity)
        if av > 1e-9 and bv > 1e-9:
            cos_ac = float(np.dot(qdot_avoid, qdot_cmd) / (av * bv))
        else:
            cos_ac = float('nan')

        if self._use_ansi:
            RESET = "\033[0m"
            RED = "\033[31m"
            YELLOW = "\033[33m"
            GREEN = "\033[32m"
            CYAN = "\033[36m"
        else:
            RESET = ""
            RED = ""
            YELLOW = ""
            GREEN = ""
            CYAN = ""
        wp = self.current_wp.name if self.current_wp else "(none)"

        # Target / actual end-effector position
        ee = self.ee_position()
        if self.current_wp is not None:
            target = np.array([self.current_wp.x, self.current_wp.y, self.current_wp.z], dtype=float)
            err = float(np.linalg.norm(ee - target))
        else:
            target = None
            err = float("nan")

        # Add obstacle context: nearest obstacle center (rough, but very useful)
        near_id, near_d = self._get_nearest_obstacle(ee)
        near_txt = f"{near_id}@{near_d:.3f}m" if near_id is not None else "(none)"
        # Smart logging policy:
        # - periodic (every N ticks)
        # - immediate if avoidance becomes active/inactive or hazard label changes
        now_wall = time.time()
        haz = haz_now
        event = (
            (self._last_log_active is None)
            or (active != self._last_log_active)
            or (haz != self._last_log_hazard)
        )
        periodic = (self._log_every_n <= 1) or ((self._debug_tick % self._log_every_n) == 0)
        if not (event or periodic):
            return

        # Warn (rate-limited) if we never received obstacles: avoidance cannot work.
        if (
            (not self._obstacles_seen_once)
            and (self._exec_start_wall is not None)
            and ((now_wall - self._exec_start_wall) > 2.0)
            and ((now_wall - self._no_obstacles_warn_wall) > 10.0)
        ):
            self._no_obstacles_warn_wall = now_wall
            self.get_logger().warn(
                "⚠️ No obstacles received on /obstacle_scene yet — avoidance will not activate. "
                "Check obstacle_synchronizer + TF frames."
            )

        # Light color coding (only for the distance)
        if np.isfinite(d) and d <= 0.08:
            d_col = RED
        elif np.isfinite(d) and d <= 0.30:
            d_col = YELLOW
        else:
            d_col = GREEN

        act_flag = "ACTIVE" if active else "-"

        # Keep hazard compact
        haz_disp = haz
        if len(haz_disp) > 28:
            haz_disp = haz_disp[:27] + "…"

        # Ratios help answer: "is avoidance actually doing something?"
        # - a_over_cmd close to 0: avoidance negligible
        # - a_over_cmd large: avoidance dominates command
        # - a_over_track large: avoidance dominates estimated tracking component
        eps = 1e-9
        a_over_cmd = av / (bv + eps)
        a_over_track = av / (tv + eps)

        if self._status_compact:
            self.get_logger().info(
                self._status_line(
                    wp=str(wp),
                    err_m=float(err),
                    d_min_m=float(d),
                    haz=str(haz_disp),
                    cmd_norm=float(bv),
                    track_norm=float(tv),
                    avoid_norm=float(av),
                    near_txt=str(near_txt),
                    avoidance_active=bool(active),
                    use_ansi=bool(self._use_ansi),
                )
            )
        else:
            self.get_logger().info(
                f"{(RED + 'AVOIDANCE' + RESET) if active else (GREEN + 'SAFE' + RESET)} {wp} "
                f"err={err:.3f}m "
                f"d_min={d_col}{d:.3f}{RESET}m "
                f"|cmd|={bv:.3f} "
                f"|track≈|={tv:.3f} "
                f"|avoid|={av:.3f} "
                f"a/c={a_over_cmd:.2f} a/t={a_over_track:.2f} cos(a,cmd)={cos_ac:+.2f} "
                f"{act_flag} "
                f"hazard={haz_disp} "
                f"near={near_txt}"
            )

        self._last_log_wall = now_wall
        self._last_log_active = active
        self._last_log_hazard = haz

        # Stall diagnostics: robot not making progress while far from target.
        now_wall = time.time()
        if np.isfinite(err) and (target is not None):
            # Track error trend
            if self._last_err is None:
                self._last_err = err
                self._last_err_wall = now_wall
                self._stall_start_wall = None
            else:
                improving = (self._last_err - err) > 1e-3
                slow_cmd = bv < 0.01
                far = err > 0.05

                if far and slow_cmd and (not improving):
                    if self._stall_start_wall is None:
                        self._stall_start_wall = now_wall
                    elif (now_wall - self._stall_start_wall) > 2.0:
                        self.get_logger().warn(
                            f"{YELLOW}[STALL?]{RESET} "
                            f"wp={wp} err={err:.3f}m |cmd|={bv:.3f}rad/s d_min={d:.3f} "
                            f"avoid_active={active} hazard={self.hazard} near={near_txt} "
                            f"target={self._fmt_xyz(float(target[0]), float(target[1]), float(target[2]), prec=3)} "
                            f"ee={self._fmt_xyz(float(ee[0]), float(ee[1]), float(ee[2]), prec=3)}"
                        )
                        # Prevent spamming warnings
                        self._stall_start_wall = now_wall
                else:
                    self._stall_start_wall = None

                # Update last error occasionally
                if (now_wall - (self._last_err_wall or now_wall)) > 0.5:
                    self._last_err = err
                    self._last_err_wall = now_wall


    # ======================================================
    # KEYBOARD (ASYNC)
    # ======================================================

    def _keyboard_listener(self):
        # Prompt ONLY when requested by the state machine (prevents early ENTER from being reused).
        while rclpy.ok():
            try:
                self._confirm_request.wait(timeout=0.2)
            except Exception:
                continue

            if not self._confirm_request.is_set():
                continue

            # Clear the request before prompting to avoid double-prompts.
            self._confirm_request.clear()

            # Print prompt and wait for ENTER.
            try:
                input("\n👉 Premi ENTER per autorizzare il prossimo step...\n")
            except Exception:
                # If stdin is unavailable, do not block the state machine forever.
                self.get_logger().warn("⚠️ stdin non disponibile: auto-continue")

            self.user_ok = True
            self._confirm_received.set()


    def _request_user_confirm(self, reason: str):
        """Request a fresh ENTER confirmation from the user."""
        self._waiting_reason = str(reason)
        self.user_ok = False
        self._confirm_received.clear()

        # Ask the keyboard thread to prompt.
        self._confirm_request.set()


    # ======================================================
    # FALLBACK IK -> JOINT TRAJECTORY
    # ======================================================
    def _start_ik_fallback_request(
        self,
        target_xyz: np.ndarray,
        orientation_xyzw: np.ndarray,
        frame_id: str,
        duration_s: float,
    ):
        """Start an async IK request; publishing happens when the future completes."""

        if self._ik_fallback_future is not None:
            return

        if not self.ik_client.service_is_ready():
            self.get_logger().warn("⏳ /compute_ik not ready yet; cannot start IK fallback")
            return

        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(target_xyz[0])
        ps.pose.position.y = float(target_xyz[1])
        ps.pose.position.z = float(target_xyz[2])
        ps.pose.orientation.x = float(orientation_xyzw[0])
        ps.pose.orientation.y = float(orientation_xyzw[1])
        ps.pose.orientation.z = float(orientation_xyzw[2])
        ps.pose.orientation.w = float(orientation_xyzw[3])

        req = GetPositionIK.Request()
        req.ik_request.group_name = "fr3_arm"
        req.ik_request.pose_stamped = ps
        req.ik_request.avoid_collisions = False
        req.ik_request.ik_link_name = "fr3_hand_tcp"
        req.ik_request.timeout.sec = 1
        req.ik_request.timeout.nanosec = 0

        # Seed with current joint state
        try:
            req.ik_request.robot_state.joint_state.name = [
                "fr3_joint1", "fr3_joint2", "fr3_joint3",
                "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7",
            ]
            req.ik_request.robot_state.joint_state.position = self.q.tolist()
        except Exception:
            pass

        self._ik_fallback_future = self.ik_client.call_async(req)
        self._ik_fallback_start_wall = time.time()
        self._ik_fallback_target = {
            "duration_s": float(duration_s),
        }

        self.get_logger().warn(
            "⚠️ No usable trajectory observed (or stuck). Started async IK fallback request..."
        )

    def _process_ik_fallback_if_ready(self, now_wall: float):
        """If the async IK future completed (or timed out), handle it."""

        if self._ik_fallback_future is None:
            return

        # Timeout guard
        if self._ik_fallback_start_wall is not None and (now_wall - self._ik_fallback_start_wall) > 2.0:
            self.get_logger().error("❌ IK fallback timed out")
            self._ik_fallback_future = None
            self._ik_fallback_start_wall = None
            self._ik_fallback_target = None
            return

        if not self._ik_fallback_future.done():
            return

        try:
            resp = self._ik_fallback_future.result()
        except Exception as e:
            self.get_logger().error(f"❌ IK fallback call failed: {e}")
            self._ik_fallback_future = None
            self._ik_fallback_start_wall = None
            self._ik_fallback_target = None
            return

        self._ik_fallback_future = None
        self._ik_fallback_start_wall = None

        if resp.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"❌ IK fallback failed: error_code={resp.error_code.val}")
            self._ik_fallback_target = None
            return

        name_to_pos = dict(zip(resp.solution.joint_state.name, resp.solution.joint_state.position))
        joint_names = [
            "fr3_joint1", "fr3_joint2", "fr3_joint3",
            "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7",
        ]
        try:
            q_goal = np.array([name_to_pos[n] for n in joint_names], dtype=float)
        except KeyError:
            self.get_logger().error("❌ IK fallback returned unexpected joint set")
            self._ik_fallback_target = None
            return

        secs = 4.0
        try:
            if self._ik_fallback_target and "duration_s" in self._ik_fallback_target:
                secs = float(self._ik_fallback_target["duration_s"])
        except Exception:
            pass
        secs = max(0.5, secs)

        traj = JointTrajectory()
        traj.joint_names = joint_names

        p0 = JointTrajectoryPoint()
        p0.positions = self.q.tolist()
        p0.time_from_start.sec = 0
        p0.time_from_start.nanosec = 0

        p1 = JointTrajectoryPoint()
        p1.positions = q_goal.tolist()
        p1.time_from_start.sec = int(secs)
        p1.time_from_start.nanosec = int((secs - int(secs)) * 1e9)

        traj.points = [p0, p1]
        self.velocity_blender_traj_pub.publish(traj)
        self._fallback_used_for_current_goal = True
        self.get_logger().info(
            f"✅ Fallback trajectory published to blender (2 points, {secs:.1f}s)"
        )

        self._ik_fallback_target = None


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
