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

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import PlanningScene, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

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
        self._last_obstacles_log_wall = 0.0
        self._obstacles_seen_once = False

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

        self.declare_parameter("log_obstacles_every_s", 5.0)
        self._log_obstacles_every_s = float(self.get_parameter("log_obstacles_every_s").value)

        self.declare_parameter("log_table_header_every_n", 20)
        self._log_table_header_every_n = int(self.get_parameter("log_table_header_every_n").value)

        # Reach/settle criteria (EE space)
        self.declare_parameter("reach_tolerance_m", 0.03)
        self.declare_parameter("settle_time_s", 0.4)
        self.declare_parameter("max_exec_time_s", 25.0)
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

        # UX: optionally require ENTER after reaching HOME before starting WP0
        self.declare_parameter("confirm_after_home", False)
        self._confirm_after_home = bool(self.get_parameter("confirm_after_home").value)

        # -------------------------------
        # Keyboard listener (NON BLOCKING)
        # -------------------------------
        # The listener only prompts when _confirm_request is set.
        threading.Thread(target=self._keyboard_listener, daemon=True).start()

        self.get_logger().info("✅ SafeAvoidanceTest node ready")


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
        # Store latest obstacles so debug log can display their poses.
        # obstacle_synchronizer sets: collision_obj.id = link.name
        now_wall = time.time()
        updated = False

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

            # Detect meaningful changes to trigger a one-time info log
            if prev is None:
                updated = True
            else:
                try:
                    dx = abs(prev["pose"].position.x - pose.position.x)
                    dy = abs(prev["pose"].position.y - pose.position.y)
                    dz = abs(prev["pose"].position.z - pose.position.z)
                    if (dx + dy + dz) > 1e-6:
                        updated = True
                except Exception:
                    updated = True

        if not self._obstacles_seen_once and self._obstacles:
            self._obstacles_seen_once = True
            updated = True

        # Avoid spamming: log summary on first receive and then periodically.
        if updated or ((now_wall - self._last_obstacles_log_wall) >= self._log_obstacles_every_s):
            self._last_obstacles_log_wall = now_wall
            self._log_obstacles_summary()


    def _log_obstacles_summary(self):
        if not self._obstacles:
            return

        # ANSI colors (works well in most terminals)
        RESET = "\033[0m"
        CYAN = "\033[36m"
        GRAY = "\033[90m"

        lines = []
        for oid in sorted(self._obstacles.keys()):
            o = self._obstacles[oid]
            pose = o.get("pose")
            prim = o.get("primitive")
            frame_id = o.get("frame_id", "")

            if pose is None or prim is None:
                continue

            p = pose.position
            dims = list(getattr(prim, "dimensions", []))
            lines.append(
                f"- {oid} frame={frame_id} pos={self._fmt_xyz(p.x, p.y, p.z)} size={self._fmt_dim(dims)}"
            )

        if not lines:
            return

        self.get_logger().info(
            f"{CYAN}[OBSTACLES]{RESET} {len(lines)} objects from /obstacle_scene\n"
            + "\n".join(lines)
            + f"\n{GRAY}(If a box is missing here, avoidance won't see it; if it is present here but not in Gazebo, it's a spawn/physics issue.){RESET}"
        )


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

                # If we were waiting after a waypoint, advance to the next one.
                # If we were waiting after HOME, start from WP0 without incrementing.
                if self._waiting_reason == "after waypoint":
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
                stuck_now = (err > 0.05) and (cmd_norm_now < 1e-3)

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
            if (now - self._exec_start_wall) > self._max_exec_time_s:
                self.get_logger().error(
                    f"⏱️ Timeout while executing {self.current_wp.name}: "
                    f"err={err:.3f}m, hazard={self.hazard}, d_min={self.min_dist:.3f}"
                )
                self.state = State.FAILED
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
                    self.get_logger().info("⏸ Robot fermo — attendendo conferma utente (ENTER)")
                    self._request_user_confirm(reason="after waypoint")
                    self.state = State.WAIT_USER
            else:
                self._reach_entered_wall = None


    def _debug(self):
        # Log only while executing a waypoint
        if self.state != State.EXECUTING:
            return

        self._debug_tick += 1
        if self._log_every_n > 1 and (self._debug_tick % self._log_every_n) != 0:
            return

        # What we can observe directly:
        # - avoidance_vel: output of online_avoidance_controller
        # - blended_vel: actual command sent to ros2_control
        # A good proxy for "tracking component" is the residual blended-avoidance.
        # (Not perfect, but very informative to see when avoidance influences commands.)
        qdot_avoid = self.avoidance_vel
        qdot_cmd = self.blended_vel
        qdot_track_proxy = qdot_cmd - qdot_avoid

        av = float(np.linalg.norm(qdot_avoid))
        bv = float(np.linalg.norm(qdot_cmd))
        tv = float(np.linalg.norm(qdot_track_proxy))

        jn = float(np.linalg.norm(self.j_row))
        d = float(self.min_dist)
        d_dot = float(self.j_row @ qdot_cmd) if jn > 1e-9 else float("nan")

        # Simple "avoidance active" heuristic
        active = (d < 0.30) and (av > 1e-3) and (jn > 1e-6)

        # Alignment between avoidance and command (cosine similarity)
        if av > 1e-9 and bv > 1e-9:
            cos_ab = float(np.dot(qdot_avoid, qdot_cmd) / (av * bv))
        else:
            cos_ab = float("nan")

        # ANSI colors (works well in most terminals)
        RESET = "\033[0m"
        BOLD = "\033[1m"
        RED = "\033[31m"
        YELLOW = "\033[33m"
        GREEN = "\033[32m"
        CYAN = "\033[36m"
        GRAY = "\033[90m"

        # Distance color coding (tuned to defaults d_safe=0.08, d_infl=0.30)
        if np.isfinite(d) and d <= 0.08:
            d_col = RED
        elif np.isfinite(d) and d <= 0.30:
            d_col = YELLOW
        else:
            d_col = GREEN

        active_col = GREEN if active else GRAY
        wp = self.current_wp.name if self.current_wp else "(none)"

        # Velocity magnitudes: highlight avoidance when it dominates
        if av > max(0.05, 0.8 * bv):
            av_col = RED
        elif av > 0.02:
            av_col = YELLOW
        else:
            av_col = GRAY

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

        # Compact table header occasionally
        if self._log_table_header_every_n > 0 and (self._debug_tick % self._log_table_header_every_n) == 0:
            self.get_logger().info(
                f"{CYAN}[AVOID]{RESET} "
                "wp                act  d_min   err    |cmd|  |avoid| ratio  cos   hazard             near_obs"
            )

            if target is not None:
                self.get_logger().info(
                    f"{CYAN}[AVOID]{RESET} "
                    f"target={self._fmt_xyz(float(target[0]), float(target[1]), float(target[2]), prec=3)} "
                    f"ee={self._fmt_xyz(float(ee[0]), float(ee[1]), float(ee[2]), prec=3)}"
                )

        ratio = (av / bv) if bv > 1e-9 else float("nan")
        act_flag = "Y" if active else "-"

        haz = (self.hazard or "none")
        if len(haz) > 18:
            haz_disp = haz[:17] + "…"
        else:
            haz_disp = haz

        self.get_logger().info(
            f"{CYAN}[AVOID]{RESET} "
            f"{wp:<16} "
            f"{active_col}{act_flag}{RESET}   "
            f"{d_col}{d:5.3f}{RESET} "
            f"{err:5.3f} "
            f"{bv:5.3f} "
            f"{av_col}{av:6.3f}{RESET} "
            f"{ratio:5.2f} "
            f"{cos_ab:5.2f} "
            f"{haz_disp:<18} "
            f"{near_txt}"
        )

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
                            f"err={err:.3f}m but |cmd|={bv:.3f}rad/s. "
                            f"d_min={d:.3f} active={active} hazard={self.hazard}. "
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
