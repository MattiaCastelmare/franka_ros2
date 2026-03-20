#!/usr/bin/env python3
"""
HUMAN-ROBOT DISTANCE ESTIMATOR  —  Depth-Space Approach
=========================================================
Estimates the minimum distance between detected human body parts and the
robot's kinematic chain.

Pipeline (inspired by Flacco, Kroeger, De Luca, Khatib — Depth Space Approach)
--------
1. Subscribe to 2-D human landmarks (``HumanPose2D``) from MediaPipe.
   → landmarks are used ONLY to localise the human in the image (ROI).
2. Build a bounding-box ROI in the depth image around visible landmarks.
3. Extract valid 3-D points (in camera frame) from the depth image inside ROI.
   → these are the "human obstacle" measurements; no single-landmark noise.
4. Subscribe to ``/joint_states`` → Pinocchio FK → robot capsule geometry.
5. Transform capsule segments from base frame to camera frame.
6. Compute minimum 3-D distance (in camera frame) between human depth pixels
   and every robot capsule axis (vectorised per capsule).
7. Surface distance = axis-distance − capsule-radius (≥ 0).
8. Publish ``HumanRobotDistance`` on ``/human_robot/closest_distance``.

Why camera frame for the distance computation?
----------------------------------------------
- The robot's position in camera frame is EXACT (from FK + extrinsics).
- The human's position comes from depth pixels — already expressed in camera
  frame by simple back-projection; no additional transform needed.
- Working in camera frame avoids accumulating extrinsic noise twice.

Convention
----------
``direction`` points **FROM the human TOWARD the robot** (escape / repulsion
direction) in the robot base frame, matching ``online_avoidance_controller``.
"""

from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import CameraInfo, Image, JointState
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Float32

from franka_msgs.msg import HumanRobotDistance
from franka_simulation.msg import HumanPose2D

from franka_experiments.utils.constants import FR3_JOINT_NAMES, NUM_JOINTS
from franka_experiments.utils.kinematics import (
    generate_urdf_from_xacro,
    load_pinocchio_model,
)
from franka_experiments.utils.logging_utils import ThrottledLogger, vec_to_str
from franka_experiments.utils.simulation_imports import (
    build_joint_to_joint_capsules,
    build_reduced_pinocchio_model_from_urdf,
    iter_world_capsule_segments,
)

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from franka_experiments.utils.camera_yaml import load_camera_info_yaml

try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False


# ── Capsule radii (same default as franka_simulation avoidance_params) ────
_DEFAULT_CAPSULE_RADII: List[float] = [0.15, 0.12, 0.13]

# Safety zone thresholds [m]
_ZONE_CRITICAL = 0.10
_ZONE_DANGER   = 0.20
_ZONE_WARNING  = 0.40

# Number of robot capsules (ordered by capsule_idx) to skip from distance
# calculations.  The first two capsules are fr3_cap_0 (link0→joint1) and
# fr3_cap_1 (joint1→joint2) — both near the base and rarely relevant for
# safety.  Exclusion is based on the integer capsule_idx field emitted by
# iter_world_capsule_segments, NOT on the parent-name string which differs
# from the old "fr3_link0/fr3_link1" assumption.
_N_EXCLUDED_CAPSULES: int = 2

# Significant landmarks for ROI definition (avoids noisy face/foot details)
_LANDMARK_WHITELIST: frozenset = frozenset([
    0,          # nose            (head proximity)
    11, 12,     # shoulders
    13, 14,     # elbows
    15, 16,     # wrists          (highest danger near robot)
    23, 24,     # hips
    25, 26,     # knees
    27, 28,     # ankles
])


def _classify_zone(distance: float) -> str:
    """Map distance [m] to safety zone string."""
    if distance <= _ZONE_CRITICAL:
        return "critical"
    if distance <= _ZONE_DANGER:
        return "danger"
    if distance <= _ZONE_WARNING:
        return "warning"
    return "safe"


# ══════════════════════════════════════════════════════════════════════════
# Node
# ══════════════════════════════════════════════════════════════════════════

class HumanDistanceEstimator(Node):
    """Estimate minimum human-robot distance via depth-space approach."""

    def __init__(self):
        super().__init__("human_distance_estimator")

        # ── Parameters ────────────────────────────────────────────────
        self._declare_params()
        self._load_params()

        # ── Pinocchio model + capsules (from franka_simulation) ───────
        self._pin_ok = False
        self._model = None
        self._data = None
        self._capsules: Dict[str, list] = {}
        self._frame_ids: Dict[str, int] = {}
        self._init_pinocchio()

        # ── Mutable state ─────────────────────────────────────────────
        self._q: Optional[np.ndarray] = None
        self._landmarks: Optional[HumanPose2D] = None
        self._depth_image: Optional[np.ndarray] = None
        self._depth_K: Optional[np.ndarray] = None   # 3×3 depth intrinsics
        self._rgb_K: Optional[np.ndarray] = None      # 3×3 RGB intrinsics
        self._depth_width: int = 0
        self._depth_height: int = 0

        # Debug overlay state
        self._raw_rgb_image: Optional[np.ndarray] = None    # BGR — from raw RGB topic
        self._debug_rgb_image: Optional[np.ndarray] = None  # BGR — annotated (legacy)
        self._last_debug_pub_time: float = 0.0

        # Camera extrinsic transforms (4×4 homogeneous):
        #   T_base_cam : camera optical frame → robot base frame
        #   T_cam_base : robot base frame → camera optical frame  (inverse)
        self._T_base_cam: Optional[np.ndarray] = None
        self._T_cam_base: Optional[np.ndarray] = None
        self._init_camera_extrinsic()

        # Pre-load intrinsics from YAML (overridden by CameraInfo if available)
        self._load_intrinsics_from_yaml()

        # ── Staleness tracking (generation counters) ──────────────────
        self._lm_gen: int = 0
        self._depth_gen: int = 0
        self._q_gen: int = 0
        self._last_computed_gen: Tuple[int, int, int] = (-1, -1, -1)
        self._last_result: Optional[Dict] = None

        # ── FK cache ──────────────────────────────────────────────────
        self._last_q_for_fk: Optional[np.ndarray] = None
        self._cached_segments: Optional[List[Dict]] = None

        # ── Self-mask cache ────────────────────────────────────────────
        # Keyed on _q_gen: mask is rebuilt only when joints change.
        self._self_mask_cache: Optional[np.ndarray] = None
        self._self_mask_gen: int = -1      # _q_gen at last mask build
        self._morph_kernel: Optional[np.ndarray] = None
        self._morph_kernel_size: int = -1  # cached kernel side-length

        # ── Timing metrics (exponential moving average, α = 0.1) ──────
        _A = 0.1
        self._timing_alpha: float = _A
        self._t_mask_avg:   float = 0.0   # [s] self-mask build time
        self._t_search_avg: float = 0.0   # [s] distance-search time
        self._t_render_avg: float = 0.0   # [s] debug-render time
        self._mask_cache_hits:   int = 0
        self._mask_cache_misses: int = 0

        # ── Logging ───────────────────────────────────────────────────
        self._tlog = ThrottledLogger(self.get_logger(), period_s=1.0)

        # ── Subscriptions ─────────────────────────────────────────────
        self.create_subscription(
            JointState, self._joint_state_topic, self._joint_state_cb, 10)
        self.create_subscription(
            HumanPose2D, self._landmarks_topic, self._landmarks_cb, 10)

        depth_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Image, self._depth_image_topic, self._depth_image_cb, depth_qos)
        self.create_subscription(
            CameraInfo, self._depth_info_topic, self._depth_info_cb, 10)
        self.create_subscription(
            CameraInfo, self._rgb_info_topic, self._rgb_info_cb, 10)
        self.create_subscription(
            Image, self._rgb_image_topic, self._rgb_image_cb, depth_qos)
        self.create_subscription(
            Image, self._debug_image_topic, self._debug_image_cb, depth_qos)

        # ── Publishers ────────────────────────────────────────────────
        self._pub = self.create_publisher(
            HumanRobotDistance, self._output_topic, 10)
        self._pub_debug_image = self.create_publisher(
            Image, self._debug_output_topic, 10)
        self._pub_debug_method = self.create_publisher(
            Image, self._debug_method_topic, 10)
        self._pub_distance_scalar = self.create_publisher(
            Float32, self._distance_scalar_topic, 10)

        # ── Timer ─────────────────────────────────────────────────────
        self.create_timer(1.0 / self._rate, self._compute_and_publish)

        self.get_logger().info(
            f"Human Distance Estimator READY  "
            f"rate={self._rate} Hz  output={self._output_topic}  "
            f"whitelist={self._use_whitelist}"
        )
        if self._debug_disable_filters:
            self.get_logger().warn(
                "*** debug_disable_filters=True ***  "
                "All filter thresholds are relaxed/bypassed for raw observation.  "
                "Set debug_disable_filters:=false to restore normal operation."
            )

    # ================================================================
    # Parameters
    # ================================================================

    def _declare_params(self) -> None:
        self.declare_parameter("rate", 30.0)
        self.declare_parameter("output_topic", "/human_robot/closest_distance")
        self.declare_parameter("joint_state_topic", "/NS_1/joint_states")
        self.declare_parameter("landmarks_topic", "/human_pose/landmarks")
        self.declare_parameter("depth_image_topic",
                               "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("depth_info_topic",
                               "/camera/camera/depth/camera_info")
        self.declare_parameter("rgb_image_topic",
                               "/camera/camera/color/image_raw")
        self.declare_parameter("rgb_info_topic",
                               "/camera/camera/color/camera_info")

        self.declare_parameter("min_visibility", 0.5)
        self.declare_parameter("depth_min_m", 0.15)
        self.declare_parameter("depth_max_m", 5.0)
        self.declare_parameter("depth_scale", 0.001)
        # Minimum visible landmarks needed to define a valid human ROI
        self.declare_parameter("min_valid_landmarks", 1)
        # Minimum valid depth pixels inside the ROI to attempt distance
        self.declare_parameter("min_human_depth_pixels", 10)
        # ROI expansion margin around landmark bounding box [px, depth image]
        self.declare_parameter("human_roi_margin_px", 60)
        # Stride for ROI depth sampling (higher = faster, coarser)
        self.declare_parameter("human_roi_stride", 4)

        # Landmark whitelist: when True only _LANDMARK_WHITELIST IDs are used.
        self.declare_parameter("use_landmark_whitelist", False)

        self.declare_parameter("num_debug_distances", 3)
        self.declare_parameter("debug_image_topic", "/human_pose/image")
        self.declare_parameter("debug_output_topic",
                               "/human_robot/distance_debug_image")
        self.declare_parameter("enable_debug_image", True)
        self.declare_parameter("debug_image_rate", 5.0)
        self.declare_parameter("debug_publish_only_if_subscribed", True)
        # Publish debug frame even when no valid distance (default: True)
        self.declare_parameter("debug_draw_no_valid_distance", True)
        # Overlay mode: when True (default) draw the closest LANDMARK → robot
        # segment instead of the closest depth point → robot segment.
        # The depth-based distance on /human_robot/closest_distance is unaffected.
        self.declare_parameter("debug_use_landmark_overlay", True)

        # ── Relaxed-filter debug mode ──────────────────────────────────
        # When True (default for testing): bypass/relax all filter thresholds
        # so that almost nothing is discarded.  Allows observing "raw" behaviour
        # before tightening filters one by one.  Does NOT change the algorithm;
        # only changes when data is considered valid.
        self.declare_parameter("debug_disable_filters", True)

        # ── Robot-centric depth-space pipeline ─────────────────────────
        # Number of control points sampled along each robot capsule axis.
        # Placed at t_k = (k+1)/(N+1), k=0..N-1 (interior, never at endpoints).
        self.declare_parameter("robot_control_points_per_segment", 5)
        # Half-side of the fallback square surveillance window [px] used when
        # the adaptive window is disabled (use_adaptive_surveillance_window=False).
        self.declare_parameter("surveillance_half_window_px", 30)
        # Metric radius [m] of the adaptive surveillance sphere around each CP.
        # Projected to pixels as half_w = fx * rho_m / z_cp, so the window
        # shrinks with distance: a CP 1 m away with rho=0.4 m → ~250 px at fx=640.
        self.declare_parameter("surveillance_radius_m", 0.30)
        # When True (default) use the depth-adaptive window; when False fall back
        # to the fixed surveillance_half_window_px.
        self.declare_parameter("use_adaptive_surveillance_window", True)
        # Number of closest candidates to use for the legacy top-K centroid
        # selection (kept for reference; not used in the main pipeline).
        self.declare_parameter("local_best_k", 5)
        # Draw ALL robot control point projections on the debug image.
        self.declare_parameter("debug_draw_all_robot_control_points", True)
        # Use the robot-centric depth-space pipeline as the MAIN distance source.
        # When False the legacy landmark-ROI pipeline is used instead.
        self.declare_parameter("debug_use_robot_centric_depth_space", True)

        # ── Robot self-mask (prevents robot from appearing as obstacle) ─
        # Enable/disable the self-mask computation.
        self.declare_parameter("enable_robot_self_mask", True)
        # Minimum painted radius [px] for each sampled capsule point, regardless
        # of depth-projected radius.  Conservative default catches thick links.
        self.declare_parameter("robot_self_mask_radius_px", 15)
        # Extra dilation margin [px] added on top of the painted circles.
        # Increase to add safety margin around the robot body.
        self.declare_parameter("robot_self_mask_dilate_px", 8)
        # Number of points sampled along each capsule axis for mask painting.
        # More samples → smoother coverage for curved/long capsules.
        self.declare_parameter("robot_self_mask_samples_per_segment", 20)
        # Overlay the self-mask on the final debug image (magenta tint).
        self.declare_parameter("robot_self_mask_draw_on_debug", True)

        # ── Internal debug topic ────────────────────────────────────────
        # Topic name for the internal-method debug image.
        self.declare_parameter("debug_method_topic", "/debug_method")
        # Enable the /debug_method publisher.
        self.declare_parameter("enable_debug_method_image", True)

        # ── Distance scalar (for PlotJuggler / rqt_plot) ──────────────
        self.declare_parameter("distance_scalar_topic",
                               "/human_robot/distance_value")

        self.declare_parameter("cam_extrinsic.tx", 0.0)
        self.declare_parameter("cam_extrinsic.ty", 0.0)
        self.declare_parameter("cam_extrinsic.tz", 0.0)
        self.declare_parameter("cam_extrinsic.qx", 0.0)
        self.declare_parameter("cam_extrinsic.qy", 0.0)
        self.declare_parameter("cam_extrinsic.qz", 0.0)
        self.declare_parameter("cam_extrinsic.qw", 1.0)

    def _load_params(self) -> None:
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self._rate = float(p("rate"))
        self._output_topic = str(p("output_topic"))
        self._joint_state_topic = str(p("joint_state_topic"))
        self._landmarks_topic = str(p("landmarks_topic"))
        self._depth_image_topic = str(p("depth_image_topic"))
        self._depth_info_topic = str(p("depth_info_topic"))
        self._rgb_image_topic = str(p("rgb_image_topic"))
        self._rgb_info_topic = str(p("rgb_info_topic"))
        self._min_visibility = float(p("min_visibility"))
        self._depth_min_m = float(p("depth_min_m"))
        self._depth_max_m = float(p("depth_max_m"))
        self._depth_scale = float(p("depth_scale"))
        self._min_valid_landmarks = int(p("min_valid_landmarks"))
        self._min_human_depth_pixels = int(p("min_human_depth_pixels"))
        self._human_roi_margin_px = int(p("human_roi_margin_px"))
        self._human_roi_stride = max(1, int(p("human_roi_stride")))
        self._use_whitelist = bool(p("use_landmark_whitelist"))
        self._num_debug_distances = int(p("num_debug_distances"))
        self._debug_image_topic = str(p("debug_image_topic"))
        self._debug_output_topic = str(p("debug_output_topic"))
        self._enable_debug_image = bool(p("enable_debug_image"))
        self._debug_image_rate = float(p("debug_image_rate"))
        self._debug_pub_only_if_sub = bool(p("debug_publish_only_if_subscribed"))
        self._debug_draw_no_valid = bool(p("debug_draw_no_valid_distance"))
        self._debug_use_landmark_overlay = bool(p("debug_use_landmark_overlay"))
        self._debug_disable_filters      = bool(p("debug_disable_filters"))
        self._robot_cps_per_seg        = max(1, int(p("robot_control_points_per_segment")))
        self._surv_half_window_px      = max(1, int(p("surveillance_half_window_px")))
        self._surveillance_radius_m    = max(0.05, float(p("surveillance_radius_m")))
        self._use_adaptive_surv_window = bool(p("use_adaptive_surveillance_window"))
        self._local_best_k             = max(1, int(p("local_best_k")))
        self._debug_draw_all_cps    = bool(p("debug_draw_all_robot_control_points"))
        self._debug_robot_centric   = bool(p("debug_use_robot_centric_depth_space"))
        self._enable_self_mask      = bool(p("enable_robot_self_mask"))
        self._self_mask_radius_px   = max(1, int(p("robot_self_mask_radius_px")))
        self._self_mask_dilate_px   = max(0, int(p("robot_self_mask_dilate_px")))
        self._self_mask_samples     = max(2, int(p("robot_self_mask_samples_per_segment")))
        self._self_mask_draw        = bool(p("robot_self_mask_draw_on_debug"))
        self._debug_method_topic    = str(p("debug_method_topic"))
        self._enable_debug_method   = bool(p("enable_debug_method_image"))
        self._distance_scalar_topic = str(p("distance_scalar_topic"))

    # ================================================================
    # Initialization helpers
    # ================================================================

    def _init_pinocchio(self) -> None:
        """Load Pinocchio model and build capsules from franka_simulation."""
        try:
            urdf_xml = generate_urdf_from_xacro()
            self._model, self._data = build_reduced_pinocchio_model_from_urdf(
                urdf_xml)
            self._capsules = build_joint_to_joint_capsules(
                model=self._model,
                capsule_radii=_DEFAULT_CAPSULE_RADII,
            )
            self._frame_ids = {}
            self._pin_ok = True
            self.get_logger().info(
                f"Pinocchio + {len(self._capsules)} capsules ready "
                f"(build_joint_to_joint_capsules)"
            )
        except Exception as e:
            self.get_logger().error(f"Pinocchio/capsule init failed: {e}")

    def _init_camera_extrinsic(self) -> None:
        """Build 4×4 T_base_cam from YAML file, falling back to params.

        T_base_cam : transforms points FROM camera optical frame TO robot base.
        T_cam_base : inverse — transforms FROM base TO camera frame.
        """
        loaded_from_yaml = False
        tx = ty = tz = qx = qy = qz = 0.0
        qw = 1.0

        try:
            pkg_share = get_package_share_directory("franka_experiments")
            yaml_path = os.path.join(pkg_share, "config",
                                     "camera_extrinsics.yaml")
            if os.path.isfile(yaml_path):
                with open(yaml_path, "r") as f:
                    data = yaml.safe_load(f)
                tr = data["translation"]
                rot = data["rotation"]
                tx = float(tr["x"])
                ty = float(tr["y"])
                tz = float(tr["z"])
                qx = float(rot["x"])
                qy = float(rot["y"])
                qz = float(rot["z"])
                qw = float(rot["w"])
                loaded_from_yaml = True
                self.get_logger().info(
                    f"Loaded camera extrinsics from {yaml_path}\n"
                    f"   t=[{tx:.4f}, {ty:.4f}, {tz:.4f}]  "
                    f"q=[{qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}]"
                )
            else:
                self.get_logger().warn(
                    f"camera_extrinsics.yaml not found at {yaml_path} "
                    f"— falling back to ROS parameters"
                )
        except Exception as e:
            self.get_logger().warn(
                f"Failed to load camera_extrinsics.yaml: {e} "
                f"— falling back to ROS parameters"
            )

        if not loaded_from_yaml:
            p = lambda n: float(self.get_parameter(n).value)  # noqa: E731
            tx = p("cam_extrinsic.tx")
            ty = p("cam_extrinsic.ty")
            tz = p("cam_extrinsic.tz")
            qx = p("cam_extrinsic.qx")
            qy = p("cam_extrinsic.qy")
            qz = p("cam_extrinsic.qz")
            qw = p("cam_extrinsic.qw")

        q_norm = float(np.sqrt(qx**2 + qy**2 + qz**2 + qw**2))
        if q_norm < 1e-9:
            self.get_logger().warn(
                "Camera extrinsic quaternion is zero — T_base_cam left as None"
            )
            return
        qx, qy, qz, qw = qx/q_norm, qy/q_norm, qz/q_norm, qw/q_norm

        R = np.array([
            [1-2*(qy**2+qz**2), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
            [2*(qx*qy+qz*qw),   1-2*(qx**2+qz**2), 2*(qy*qz-qx*qw)],
            [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),   1-2*(qx**2+qy**2)],
        ], dtype=float)

        T = np.eye(4, dtype=float)
        T[:3, :3] = R
        T[:3, 3] = [tx, ty, tz]
        self._T_base_cam = T

        T_inv = np.eye(4, dtype=float)
        T_inv[:3, :3] = R.T
        T_inv[:3, 3] = -R.T @ np.array([tx, ty, tz], dtype=float)
        self._T_cam_base = T_inv

    def _load_intrinsics_from_yaml(self) -> None:
        """Pre-load camera intrinsics from YAML config files."""
        try:
            pkg_share = get_package_share_directory("franka_experiments")
        except Exception:
            self.get_logger().warn(
                "Cannot resolve franka_experiments share dir "
                "— skipping YAML intrinsics loading"
            )
            return

        # RGB intrinsics
        rgb_path = os.path.join(pkg_share, "config", "rgb_intrinsics.yaml")
        if os.path.isfile(rgb_path):
            try:
                data = load_camera_info_yaml(rgb_path)
                if data is not None:
                    k = data.get("k", [])
                    if len(k) == 9:
                        self._rgb_K = np.array(k, dtype=float).reshape(3, 3)
                        self.get_logger().info(
                            f"RGB intrinsics from YAML: "
                            f"fx={self._rgb_K[0,0]:.1f}  "
                            f"fy={self._rgb_K[1,1]:.1f}  "
                            f"cx={self._rgb_K[0,2]:.1f}  "
                            f"cy={self._rgb_K[1,2]:.1f}"
                        )
            except Exception as e:
                self.get_logger().warn(f"Failed to parse {rgb_path}: {e}")
        else:
            self.get_logger().warn(
                f"rgb_intrinsics.yaml not found at {rgb_path}")

        # Depth intrinsics
        depth_path = os.path.join(pkg_share, "config",
                                  "depth_intrinsics.yaml")
        if os.path.isfile(depth_path):
            try:
                data = load_camera_info_yaml(depth_path)
                if data is not None:
                    k = data.get("k", [])
                    if len(k) == 9:
                        self._depth_K = np.array(k, dtype=float).reshape(3, 3)
                        w = int(data.get("width", 0))
                        h = int(data.get("height", 0))
                        if w > 0 and h > 0 and self._depth_width == 0:
                            self._depth_width = w
                            self._depth_height = h
                        self.get_logger().info(
                            f"Depth intrinsics from YAML: "
                            f"fx={self._depth_K[0,0]:.1f}  "
                            f"fy={self._depth_K[1,1]:.1f}  "
                            f"cx={self._depth_K[0,2]:.1f}  "
                            f"cy={self._depth_K[1,2]:.1f}  "
                            f"({w}\u00d7{h})"
                        )
            except Exception as e:
                self.get_logger().warn(f"Failed to parse {depth_path}: {e}")
        else:
            self.get_logger().warn(
                f"depth_intrinsics.yaml not found at {depth_path}")

    # ================================================================
    # Subscription callbacks
    # ================================================================

    def _joint_state_cb(self, msg: JointState) -> None:
        try:
            name_map = {str(n): int(i) for i, n in enumerate(msg.name)}
            q = np.array(
                [msg.position[name_map[jn]] for jn in FR3_JOINT_NAMES],
                dtype=float,
            )
            self._q = q
            self._q_gen += 1
        except (KeyError, ValueError, IndexError):
            pass

    def _landmarks_cb(self, msg: HumanPose2D) -> None:
        self._landmarks = msg
        self._lm_gen += 1

    def _depth_image_cb(self, msg: Image) -> None:
        try:
            if msg.encoding in ("16UC1", "mono16"):
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                    msg.height, msg.width)
            elif msg.encoding == "32FC1":
                arr = np.frombuffer(msg.data, dtype=np.float32).reshape(
                    msg.height, msg.width)
            else:
                return
            self._depth_image = arr
            self._depth_height = int(msg.height)
            self._depth_width = int(msg.width)
            self._depth_gen += 1
        except Exception:
            pass

    def _depth_info_cb(self, msg: CameraInfo) -> None:
        try:
            self._depth_K = np.array(msg.k, dtype=float).reshape(3, 3)
        except Exception:
            pass

    def _rgb_info_cb(self, msg: CameraInfo) -> None:
        try:
            self._rgb_K = np.array(msg.k, dtype=float).reshape(3, 3)
        except Exception:
            pass

    def _rgb_image_cb(self, msg: Image) -> None:
        """Store latest raw RGB frame as BGR numpy array (base for debug overlay)."""
        if not _HAS_CV2:
            return
        try:
            if msg.encoding == "rgb8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                self._raw_rgb_image = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "bgr8":
                self._raw_rgb_image = np.frombuffer(
                    msg.data, dtype=np.uint8,
                ).reshape(msg.height, msg.width, 3).copy()
            else:
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                self._raw_rgb_image = arr.copy()
        except Exception:
            pass

    def _debug_image_cb(self, msg: Image) -> None:
        """Store latest annotated RGB image as BGR numpy array."""
        if not _HAS_CV2:
            return
        try:
            if msg.encoding == "rgb8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                self._debug_rgb_image = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            elif msg.encoding == "bgr8":
                self._debug_rgb_image = np.frombuffer(
                    msg.data, dtype=np.uint8,
                ).reshape(msg.height, msg.width, 3).copy()
            else:
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                self._debug_rgb_image = arr.copy()
        except Exception:
            pass

    # ================================================================
    # FK helpers
    # ================================================================

    def _run_fk(self) -> None:
        """Run FK with current joint state (updates self._data in place)."""
        q_full = pin.neutral(self._model)
        for k, jname in enumerate(FR3_JOINT_NAMES):
            jid = self._model.getJointId(jname)
            idx_q = self._model.joints[jid].idx_q
            q_full[idx_q] = self._q[k]
        pin.forwardKinematics(self._model, self._data, q_full)
        pin.updateFramePlacements(self._model, self._data)

    def _get_world_segments(self) -> List[Dict]:
        return iter_world_capsule_segments(
            capsules=self._capsules,
            frame_ids=self._frame_ids,
            data=self._data,
        )

    def _get_world_segments_cached(self) -> Optional[List[Dict]]:
        """Return cached capsule world segments, rerunning FK only if q changed."""
        if self._q is None:
            return None
        q_unchanged = (
            self._last_q_for_fk is not None
            and np.array_equal(self._q, self._last_q_for_fk)
        )
        if not q_unchanged:
            self._run_fk()
            self._cached_segments = self._get_world_segments()
            self._last_q_for_fk = self._q.copy()
        return self._cached_segments

    # ================================================================
    # Filter helpers — normal vs. relaxed (debug_disable_filters)
    # ================================================================
    # Each helper has one job: decide whether a piece of data is accepted.
    # Normal mode: applies the configured ROS parameter thresholds.
    # Relaxed mode: accepts everything that is physically valid (non-NaN, > 0).
    # To re-enable a specific filter individually, move its check out of the
    # relaxed branch — all other helpers remain relaxed.

    def _landmark_passes_filters(self, lm_id: int, visibility: float) -> bool:
        """Return True if landmark passes whitelist + visibility checks.

        Relaxed: both checks bypassed — any landmark ID is accepted regardless
        of its visibility score.
        """
        if self._debug_disable_filters:
            return True
        if self._use_whitelist and lm_id not in _LANDMARK_WHITELIST:
            return False
        return visibility >= self._min_visibility

    def _depth_range(self) -> Tuple[float, float]:
        """Return (d_min, d_max) [m] for depth validity.

        Relaxed: (0.0, +inf) — any positive finite depth is accepted.
        Normal:  (depth_min_m, depth_max_m) as configured.
        """
        if self._debug_disable_filters:
            return (0.0, float("inf"))
        return (self._depth_min_m, self._depth_max_m)

    def _effective_roi_stride(self) -> int:
        """ROI depth sampling stride.  Relaxed: 1 (full resolution)."""
        return 1 if self._debug_disable_filters else self._human_roi_stride

    def _effective_min_depth_pixels(self) -> int:
        """Minimum valid depth pixels to compute distance.  Relaxed: 1."""
        return 1 if self._debug_disable_filters else self._min_human_depth_pixels

    def _effective_min_valid_landmarks(self) -> int:
        """Minimum landmarks needed to build a ROI.  Relaxed: 1."""
        return 1 if self._debug_disable_filters else self._min_valid_landmarks

    def _sample_landmark_depth(self, ud_i: int, vd_i: int) -> Optional[float]:
        """Median depth [m] sampled in a neighbourhood around (ud_i, vd_i).

        Normal mode:  3×3 window.
        Relaxed mode: tries 3×3 first; expands to 15×15 if no valid depth found.
        Returns None only when truly no valid depth exists in the window.
        """
        d_min, d_max = self._depth_range()

        def _try_window(half: int) -> Optional[float]:
            r0 = max(0, vd_i - half)
            r1 = min(self._depth_height, vd_i + half + 1)
            c0 = max(0, ud_i - half)
            c1 = min(self._depth_width,  ud_i + half + 1)
            patch  = self._depth_image[r0:r1, c0:c1].astype(np.float32)
            z_vals = patch.flatten() * float(self._depth_scale)
            valid  = z_vals[
                (z_vals > d_min) & (z_vals < d_max) & np.isfinite(z_vals)]
            return float(np.median(valid)) if len(valid) > 0 else None

        z_m = _try_window(1)          # always try 3×3 first
        if z_m is not None:
            return z_m
        if self._debug_disable_filters:
            z_m = _try_window(7)      # expand to 15×15 in relaxed mode
        return z_m

    # ================================================================
    # Phase A — Human ROI from MediaPipe landmarks
    # ================================================================

    def _build_human_depth_roi_from_landmarks(
        self,
    ) -> Optional[Tuple[int, int, int, int]]:
        """Compute a bounding-box ROI (u_min, u_max, v_min, v_max) in the
        depth image space from visible MediaPipe landmarks.

        The ROI is expanded by ``_human_roi_margin_px`` on each side so that
        body parts between skeletal keypoints are also covered.

        Returns None if fewer than ``min_valid_landmarks`` pass the
        visibility filter.
        """
        lm = self._landmarks
        if lm is None:
            return None
        if self._depth_width == 0 or self._depth_height == 0:
            return None

        # Scale factors from RGB landmark space → depth image space.
        # If depth and RGB resolutions differ we compensate proportionally.
        if (self._rgb_K is not None
                and lm.image_width > 0 and lm.image_height > 0):
            scale_u = float(self._depth_width) / float(lm.image_width)
            scale_v = float(self._depth_height) / float(lm.image_height)
        else:
            scale_u = 1.0
            scale_v = 1.0

        us: List[float] = []
        vs: List[float] = []
        for i in range(len(lm.ids)):
            lm_id = int(lm.ids[i])
            if not self._landmark_passes_filters(lm_id, float(lm.visibility[i])):
                continue
            u_d = lm.u[i] * scale_u
            v_d = lm.v[i] * scale_v
            if 0 <= u_d < self._depth_width and 0 <= v_d < self._depth_height:
                us.append(u_d)
                vs.append(v_d)

        min_lm = self._effective_min_valid_landmarks()
        if len(us) < min_lm:
            self._tlog.warn(
                f"ROI: only {len(us)} landmark(s) in image bounds "
                f"(need {min_lm}, total={len(lm.ids)}) — no ROI"
            )
            return None

        m = self._human_roi_margin_px
        u_min = max(0, int(min(us)) - m)
        u_max = min(self._depth_width - 1,  int(max(us)) + m)
        v_min = max(0, int(min(vs)) - m)
        v_max = min(self._depth_height - 1, int(max(vs)) + m)

        if u_max <= u_min or v_max <= v_min:
            return None

        return (u_min, u_max, v_min, v_max)

    # ================================================================
    # Phase B — Human 3-D points from depth ROI (camera frame)
    # ================================================================

    def _get_human_pts_cam_in_roi(
        self,
        roi: Tuple[int, int, int, int],
    ) -> Optional[np.ndarray]:
        """Back-project valid depth pixels in ``roi`` to 3-D camera frame.

        Sampling uses stride ``_human_roi_stride`` for efficiency.
        Filtering: depth zero, NaN, and out of [depth_min_m, depth_max_m].

        Returns np.ndarray of shape (M, 3) in camera frame, or None if
        fewer than ``_min_human_depth_pixels`` valid pixels are found.
        """
        depth = self._depth_image
        if depth is None:
            return None

        K = self._depth_K
        if K is None:
            return None

        fx_d = float(K[0, 0])
        fy_d = float(K[1, 1])
        cx_d = float(K[0, 2])
        cy_d = float(K[1, 2])
        if fx_d < 1e-6 or fy_d < 1e-6:
            return None

        u_min, u_max, v_min, v_max = roi
        stride = self._effective_roi_stride()

        # Extract ROI patch and convert to metres
        patch = depth[v_min:v_max:stride, u_min:u_max:stride]
        z_m = patch.astype(np.float32) * float(self._depth_scale)

        # Build pixel-coordinate grids aligned with the strided patch
        vs_arr = np.arange(v_min, v_max, stride, dtype=np.float32)
        us_arr = np.arange(u_min, u_max, stride, dtype=np.float32)

        # Guard against shape mismatch (border rounding)
        nr = min(len(vs_arr), z_m.shape[0])
        nc = min(len(us_arr), z_m.shape[1])
        vs_arr = vs_arr[:nr]
        us_arr = us_arr[:nc]
        z_m    = z_m[:nr, :nc]

        uu, vv = np.meshgrid(us_arr, vs_arr)  # both (nr, nc)

        # Validity mask: positive, finite, within depth range
        d_min, d_max = self._depth_range()
        valid = (z_m > d_min) & (z_m < d_max) & np.isfinite(z_m)

        if not np.any(valid):
            self._tlog.warn(
                f"ROI depth: zero valid pixels after range filter "
                f"[{d_min:.2f}, {d_max:.2f}] m — no distance"
            )
            return None

        z_v = z_m[valid]
        u_v = uu[valid]
        v_v = vv[valid]

        min_px = self._effective_min_depth_pixels()
        if len(z_v) < min_px:
            self._tlog.warn(
                f"ROI depth: {len(z_v)} valid pixel(s) < min {min_px} "
                f"(stride={stride}, range=[{d_min:.2f},{d_max:.2f}] m)"
            )
            return None

        # Back-project to camera frame (Z-forward optical convention)
        x_cam = (u_v - cx_d) * z_v / fx_d
        y_cam = (v_v - cy_d) * z_v / fy_d

        return np.column_stack([x_cam, y_cam, z_v])  # (M, 3)

    # ================================================================
    # Phase C — Depth-space distance (capsules in camera frame)
    # ================================================================

    def _compute_min_distance_depth_space(
        self,
        segments: List[Dict],
        human_pts_cam: np.ndarray,
    ) -> Optional[Dict]:
        """Compute minimum distance between human depth pixels and robot capsules.

        All geometry is expressed in the CAMERA frame:
        - Capsule segments are transformed from base → camera via T_cam_base.
        - Human points are already in camera frame (from depth back-projection).

        For each capsule segment [p0, p1] with radius r:
          C[i] = p0 + clip(t[i], 0, 1) · (p1-p0)   (closest axis point)
          d_surf[i] = max(||H[i] - C[i]|| - r, 0)
        Global minimum over all (human-pixel, capsule) pairs.

        The closest human point and robot axis point are then converted back
        to the base frame for output in the ROS message.
        """
        if not segments:
            return None
        if human_pts_cam is None or len(human_pts_cam) == 0:
            return None

        # Pre-extract camera-frame transform components (used per segment)
        T_cb = self._T_cam_base
        T_bc = self._T_base_cam
        if T_cb is None or T_bc is None:
            return None

        R_cb = T_cb[:3, :3]  # base → camera rotation
        t_cb = T_cb[:3, 3]
        R_bc = T_bc[:3, :3]  # camera → base rotation
        t_bc = T_bc[:3, 3]

        H = human_pts_cam  # (M, 3) in camera frame

        best_dist        = float("inf")
        best_name        = ""
        best_p_robot_cam = np.zeros(3)
        best_p_human_cam = np.zeros(3)

        for seg in segments:
            p0_base = np.asarray(seg["p0"], dtype=float)
            p1_base = np.asarray(seg["p1"], dtype=float)
            radius  = float(seg["radius"])
            name    = str(seg.get("parent", ""))

            # Skip the first two robot capsules (fr3_cap_0, fr3_cap_1)
            if self._segment_is_excluded(seg):
                continue

            # Transform capsule endpoints to camera frame
            p0_cam = R_cb @ p0_base + t_cb
            p1_cam = R_cb @ p1_base + t_cb

            # Skip segments entirely behind the camera
            if p0_cam[2] <= 0.0 and p1_cam[2] <= 0.0:
                continue

            ab  = p1_cam - p0_cam
            ab2 = float(ab @ ab)

            # Closest point on segment axis for each human point (vectorised)
            if ab2 < 1e-12:
                C = np.tile(p0_cam, (len(H), 1))
            else:
                ts = np.clip((H - p0_cam) @ ab / ab2, 0.0, 1.0)  # (M,)
                C  = p0_cam + ts[:, None] * ab                     # (M, 3)

            dists      = np.linalg.norm(H - C, axis=1)       # (M,)
            surf_dists = np.maximum(dists - radius, 0.0)      # (M,)

            idx = int(np.argmin(surf_dists))
            if surf_dists[idx] < best_dist:
                best_dist        = float(surf_dists[idx])
                best_name        = name
                best_p_robot_cam = C[idx].copy()
                best_p_human_cam = H[idx].copy()

        if not np.isfinite(best_dist) or best_dist > 5.0:
            return None

        # Floor near-zero to avoid jitter at contact
        best_dist = max(best_dist, 0.01)

        # Convert closest points back to robot base frame for output
        p_robot_base = R_bc @ best_p_robot_cam + t_bc
        p_human_base = R_bc @ best_p_human_cam + t_bc

        # Direction: FROM human TOWARD robot, in base frame
        diff_base = p_robot_base - p_human_base
        diff_norm = float(np.linalg.norm(diff_base))
        direction = (diff_base / diff_norm
                     if diff_norm > 1e-9
                     else np.array([0., 0., 1.]))

        # Confidence: more depth pixels + shorter distance → higher confidence
        n_pts      = len(H)
        lm_conf    = float(np.clip(n_pts / 500.0, 0.2, 1.0))
        dist_conf  = (1.0 if best_dist < 2.0
                      else float(np.clip(1.0 - (best_dist - 2.0) / 3.0,
                                         0.3, 1.0)))
        confidence = float(np.clip(lm_conf * dist_conf, 0.0, 1.0))

        return {
            "robot_link_name":     best_name,
            "distance":            best_dist,
            "direction":           direction,
            "closest_point_robot": p_robot_base,
            "closest_point_human": p_human_base,
            "zone":                _classify_zone(best_dist),
            "confidence":          confidence,
            "n_valid_pts":         n_pts,
        }

    # ================================================================
    # Segment exclusion
    # ================================================================

    @staticmethod
    def _segment_is_excluded(seg: Dict) -> bool:
        """Return True if this capsule should be skipped in distance calculations.

        Exclusion is based on ``capsule_idx`` (integer, 0-indexed, sorted over
        the capsule dict keys in ``iter_world_capsule_segments``).  The first
        ``_N_EXCLUDED_CAPSULES`` capsules are always excluded:

          capsule_idx=0 → fr3_cap_0  (fr3_link0 → fr3_joint1)
          capsule_idx=1 → fr3_cap_1  (fr3_joint1 → fr3_joint2)

        This is robust to any renaming of the "parent" string field.
        """
        return int(seg.get("capsule_idx", 0)) < _N_EXCLUDED_CAPSULES

    # ================================================================
    # Robot-centric depth-space pipeline  (Flacco-inspired, CP variant)
    # ================================================================

    def _build_robot_control_points(
        self,
        segments: List[Dict],
    ) -> List[Dict]:
        """Generate N control points per admitted robot capsule axis.

        Points are placed at evenly-spaced interior positions along each
        segment:  t_k = (k+1) / (N+1)  for k = 0 … N-1.

        Capsules with ``capsule_idx < _N_EXCLUDED_CAPSULES`` are skipped
        (fr3_cap_0 and fr3_cap_1, i.e. the base and first joint capsule).

        Returns a list of dicts:
            pt_base  — np.ndarray (3,) in robot base frame
            link     — str, parent link name
            radius   — float, capsule radius [m]
            seg_idx  — int, index in the segments list
            cp_idx   — int, 0 … N-1
        """
        n = self._robot_cps_per_seg
        ts = [(k + 1) / (n + 1) for k in range(n)]
        ctrl_pts: List[Dict] = []
        for seg_idx, seg in enumerate(segments):
            if self._segment_is_excluded(seg):
                continue
            name = str(seg.get("parent", ""))
            p0  = np.asarray(seg["p0"], dtype=float)
            p1  = np.asarray(seg["p1"], dtype=float)
            rad = float(seg["radius"])
            for cp_idx, t in enumerate(ts):
                ctrl_pts.append({
                    "pt_base": p0 + t * (p1 - p0),
                    "link":    name,
                    "radius":  rad,
                    "seg_idx": seg_idx,
                    "cp_idx":  cp_idx,
                })
        return ctrl_pts

    def _project_point_to_depth_pixel(
        self,
        p_base: np.ndarray,
    ) -> Optional[Tuple[int, int]]:
        """Project a 3-D base-frame point to a depth-image pixel (u, v).

        Returns None if behind the camera or if depth intrinsics / extrinsics
        are unavailable.
        """
        if self._T_cam_base is None or self._depth_K is None:
            return None

        p_cam = self._T_cam_base[:3, :3] @ p_base + self._T_cam_base[:3, 3]
        if p_cam[2] <= 0.01:
            return None

        fx = float(self._depth_K[0, 0])
        fy = float(self._depth_K[1, 1])
        cx = float(self._depth_K[0, 2])
        cy = float(self._depth_K[1, 2])
        if fx < 1e-6 or fy < 1e-6:
            return None

        u = int(round(fx * p_cam[0] / p_cam[2] + cx))
        v = int(round(fy * p_cam[1] / p_cam[2] + cy))
        return (u, v)

    def _collect_depth_candidates_around_control_point(
        self,
        px_d: int,
        py_d: int,
        robot_mask: Optional[np.ndarray] = None,
        half_w: Optional[int] = None,
    ) -> Tuple[Optional[np.ndarray], int]:
        """Return 3-D camera-frame obstacle points in the surveillance window.

        The window is a square of side (2*half_w + 1) centred on (px_d, py_d).
        ``half_w`` defaults to self._surv_half_window_px if not provided.
        Depth values are filtered using ``_depth_range()``.  Pixels inside
        ``robot_mask`` are excluded (robot self-mask).

        Returns:
            (pts_cam, n_masked)
              pts_cam  — (M, 3) float array in camera frame, or None if empty
              n_masked — number of pixels rejected by the robot self-mask
        """
        if self._depth_image is None or self._depth_K is None:
            return None, 0

        h = half_w if half_w is not None else self._surv_half_window_px
        x_min = max(0, px_d - h)
        x_max = min(self._depth_width  - 1, px_d + h)
        y_min = max(0, py_d - h)
        y_max = min(self._depth_height - 1, py_d + h)

        window = self._depth_image[y_min:y_max + 1, x_min:x_max + 1]

        depth_min_m, depth_max_m = self._depth_range()
        raw_min = max(1, int(depth_min_m / self._depth_scale))
        # depth_max_m can be +inf when debug_disable_filters=True; in that case
        # accept every raw value that is physically positive and finite.
        if math.isfinite(depth_max_m):
            raw_max    = int(depth_max_m / self._depth_scale)
            valid_mask = (window > 0) & (window >= raw_min) & (window <= raw_max)
        else:
            valid_mask = window >= raw_min  # raw_min ≥ 1 → window > 0 implied

        # ── Robot self-mask: exclude pixels belonging to the robot ─────
        n_masked = 0
        if robot_mask is not None:
            robot_window = robot_mask[y_min:y_max + 1, x_min:x_max + 1]
            n_before  = int(np.count_nonzero(valid_mask))
            valid_mask = valid_mask & ~robot_window
            n_masked  = n_before - int(np.count_nonzero(valid_mask))

        ys, xs = np.where(valid_mask)
        if len(ys) == 0:
            return None, n_masked

        # Back-project to camera frame
        z_vals = window[ys, xs].astype(float) * self._depth_scale
        fx = float(self._depth_K[0, 0])
        fy = float(self._depth_K[1, 1])
        cx = float(self._depth_K[0, 2])
        cy = float(self._depth_K[1, 2])

        u_abs = (xs + x_min).astype(float)
        v_abs = (ys + y_min).astype(float)
        x_cam = (u_abs - cx) * z_vals / fx
        y_cam = (v_abs - cy) * z_vals / fy

        return np.stack([x_cam, y_cam, z_vals], axis=1), n_masked  # (M, 3)

    def _build_robot_self_mask(
        self,
        segments: List[Dict],
    ) -> Optional[np.ndarray]:
        """Build a boolean depth-image mask covering the entire robot body.

        Uses ALL capsule segments (including fr3_cap_0/fr3_cap_1, which are
        excluded from distance calculations but still visible in the depth
        image) so that the complete visible robot silhouette is masked.

        For each sampled point along each capsule axis:
          1. Project to depth-image pixel (u, v) via depth intrinsics.
          2. Compute apparent radius in pixels:
               r_proj = fx_d * capsule_radius / z_cam
             Take max(r_proj, self._self_mask_radius_px) as the painted radius.
          3. Paint a filled circle on the mask.
        After painting, dilate by self._self_mask_dilate_px pixels.

        This is a pragmatic approach: projected radii are geometrically exact
        for a pinhole camera; the minimum floor and dilation add a safety margin.

        Returns: boolean (H, W) ndarray, or None if prerequisites missing.
        """
        if not self._enable_self_mask or not _HAS_CV2:
            return None
        if self._depth_image is None or self._depth_K is None:
            return None
        if self._T_cam_base is None:
            return None

        R_cb = self._T_cam_base[:3, :3]   # base → camera
        t_cb = self._T_cam_base[:3, 3]
        fx   = float(self._depth_K[0, 0])
        fy   = float(self._depth_K[1, 1])
        cx   = float(self._depth_K[0, 2])
        cy   = float(self._depth_K[1, 2])

        mask_u8 = np.zeros((self._depth_height, self._depth_width),
                           dtype=np.uint8)
        n         = self._self_mask_samples
        r_px_floor = self._self_mask_radius_px

        for seg in segments:
            p0     = np.asarray(seg["p0"], dtype=float)
            p1     = np.asarray(seg["p1"], dtype=float)
            radius = float(seg["radius"])

            for k in range(n):
                t_k    = k / (n - 1) if n > 1 else 0.5
                p_base = p0 + t_k * (p1 - p0)
                p_cam  = R_cb @ p_base + t_cb

                if p_cam[2] <= 0.01:
                    continue

                z   = p_cam[2]
                u   = int(round(fx * p_cam[0] / z + cx))
                v   = int(round(fy * p_cam[1] / z + cy))

                # Geometrically exact projected capsule radius, floored to minimum
                r_proj = int(round(fx * radius / z))
                r_px   = max(r_px_floor, r_proj)

                # Draw even if centre is slightly outside image (circle extends in)
                if (-r_px <= u < self._depth_width  + r_px and
                        -r_px <= v < self._depth_height + r_px):
                    cv2.circle(mask_u8, (u, v), r_px, 255, -1)

        # Dilate for a conservative safety margin around the painted silhouette.
        # The kernel is cached so getStructuringElement runs only when the
        # dilation parameter changes (typically never at runtime).
        if self._self_mask_dilate_px > 0:
            k_size = 2 * self._self_mask_dilate_px + 1
            if self._morph_kernel is None or self._morph_kernel_size != k_size:
                self._morph_kernel      = cv2.getStructuringElement(
                    cv2.MORPH_ELLIPSE, (k_size, k_size))
                self._morph_kernel_size = k_size
            mask_u8 = cv2.dilate(mask_u8, self._morph_kernel)

        return mask_u8 > 0   # boolean array

    def _maybe_rebuild_robot_self_mask(
        self,
        segments: List[Dict],
    ) -> Optional[np.ndarray]:
        """Return the robot self-mask, rebuilding it only when joints changed.

        The mask is keyed on ``_q_gen``.  As long as the joint-state generation
        counter has not advanced since the last build, the cached boolean array
        is returned immediately (O(1)).  A full rebuild is triggered only when
        the robot moved (``_q_gen`` changed), which is the only thing that can
        alter the projected capsule silhouette.

        Cache-hit:  returns ``_self_mask_cache`` without any computation.
        Cache-miss: calls ``_build_robot_self_mask``, stores result + gen.
        """
        if not self._enable_self_mask or not _HAS_CV2:
            return None
        if (self._self_mask_cache is not None
                and self._self_mask_gen == self._q_gen):
            self._mask_cache_hits += 1
            return self._self_mask_cache
        # Rebuild needed: new joint configuration
        mask = self._build_robot_self_mask(segments)
        self._self_mask_cache  = mask
        self._self_mask_gen    = self._q_gen
        self._mask_cache_misses += 1
        return mask

    def _compute_surveillance_window_px(self, cp_cam: np.ndarray) -> int:
        """Convert the metric surveillance radius to depth-image pixels.

        Uses the pinhole formula:  half_w = fx * rho_m / z_cp

        This makes the window depth-adaptive: a CP close to the camera gets a
        larger pixel window (same metric footprint), and a distant CP gets a
        smaller one — exactly as required by the Flacco et al. formulation.

        The result is clamped to [5, 300] to avoid degenerate cases.
        """
        if self._depth_K is None:
            return self._surv_half_window_px  # fallback
        z = max(float(cp_cam[2]), 0.05)
        fx = float(self._depth_K[0, 0])
        half_w = int(math.ceil(fx * self._surveillance_radius_m / z))
        return max(5, min(half_w, 300))

    def _find_local_obstacle_minimum(
        self,
        obs_cam: np.ndarray,
        cp_cam: np.ndarray,
        radius: float,
    ) -> Tuple[np.ndarray, float, int, int]:
        """Find the true local obstacle minimum with progressive contraction.

        This replaces the top-K centroid approach with a near-to-far scan that
        prunes candidates as soon as they cannot improve the current minimum —
        exactly the contraction property of the Flacco et al. depth-space method.

        Algorithm:
          1. Compute 3-D Euclidean distance from every candidate to cp_cam.
          2. Sort candidates near → far (ascending 3-D distance).
          3. Process in order:
               d_eff = max(d3d - radius, 0)
               if d_eff < best_d_eff  →  update best
               if d3d - radius ≥ best_d_eff  →  STOP (contraction: all remaining
               candidates are at least as far, so d_eff ≥ best_d_eff)
          4. Return the true argmin candidate (not a centroid).

        The contraction works because once best_d_eff is set, any candidate with
        d3d ≥ best_d_eff + radius satisfies d_eff ≥ best_d_eff and cannot win.
        Processing near-to-far makes this early exit hit as soon as possible.

        Returns: (obs_point_cam, d_eff_min, n_evaluated, n_total)
          obs_point_cam — (3,) winning obstacle point in camera frame
          d_eff_min     — minimum surface distance [m]
          n_evaluated   — candidates examined before early exit
          n_total       — total candidates in window (before pruning)
        """
        n_total = len(obs_cam)

        # Fast path: single candidate — skip argsort entirely.
        if n_total == 1:
            d3d   = float(np.linalg.norm(obs_cam[0] - cp_cam))
            d_eff = max(d3d - radius, 0.0)
            return obs_cam[0], d_eff, 1, 1

        dists    = np.linalg.norm(obs_cam - cp_cam, axis=1)   # (M,)
        order    = np.argsort(dists)                           # near → far

        best_d_eff = float("inf")
        best_i     = int(order[0])
        n_eval     = 0

        for i in order:
            d3d = float(dists[i])
            # Contraction: if even the surface point (closest possible d_eff)
            # of this candidate cannot beat the current best, stop.
            if d3d - radius >= best_d_eff:
                break
            n_eval += 1
            d_eff = max(d3d - radius, 0.0)
            if d_eff < best_d_eff:
                best_d_eff = d_eff
                best_i     = int(i)
                # Perfect early exit: contact (d_eff == 0) cannot be beaten.
                if best_d_eff == 0.0:
                    break

        return obs_cam[best_i], best_d_eff, n_eval, n_total

    def _select_local_obstacle_robust(
        self,
        obs_cam: np.ndarray,
        cp_cam: np.ndarray,
        radius: float,
    ) -> Tuple[np.ndarray, float, int]:
        """Choose a robust local obstacle point via top-K centroid.

        Strategy:
          1. Compute surface distance for every candidate: d_eff = max(||p - cp|| - r, 0).
          2. Sort candidates by d_eff; keep the K closest (K = self._local_best_k,
             clamped to the actual number of candidates).
          3. Compute the 3-D centroid of those K points.
          4. Return the centroid as the obstacle point, together with the
             recomputed surface distance from cp_cam to the centroid.

        This is much less sensitive to single-pixel noise than the raw argmin
        while still reacting quickly when a limb enters the surveillance window.

        Returns: (obs_cam_robust, d_eff_robust, k_used)
          obs_cam_robust — (3,) centroid in camera frame
          d_eff_robust   — surface distance [m] to the centroid
          k_used         — actual number of candidates averaged
        """
        dists      = np.linalg.norm(obs_cam - cp_cam, axis=1)        # (M,)
        surf_dists = np.maximum(dists - radius, 0.0)                  # (M,)

        k = min(self._local_best_k, len(obs_cam))
        if k < len(obs_cam):
            # np.argpartition is O(M): avoids full sort
            top_k_idx = np.argpartition(surf_dists, k)[:k]
        else:
            top_k_idx = np.arange(len(obs_cam))

        centroid = obs_cam[top_k_idx].mean(axis=0)                    # (3,)
        d_to_centroid = float(np.linalg.norm(centroid - cp_cam))
        d_eff_robust  = max(d_to_centroid - radius, 0.0)

        return centroid, d_eff_robust, int(k)

    def _compute_min_distance_robot_centric_depth_space(
        self,
        segments: List[Dict],
    ) -> Optional[Dict]:
        """Robot-centric depth-space distance pipeline.

        For every robot control point (link ≥ fr3_link2):
          1. Project the CP to the depth image.
          2. Collect all valid depth pixels inside a square surveillance window.
          3. Back-project to camera frame and compute surface distance:
                 d_eff = max(||obs_cam − cp_cam|| − radius, 0)
          4. Keep the global minimum across all CPs.

        Returns a result dict compatible with the legacy pipeline:
            robot_link_name      — link of the winning CP
            distance             — minimum surface distance [m]
            direction            — unit vector FROM obstacle TOWARD robot (base)
            closest_point_robot  — winner CP in robot base frame
            closest_point_human  — closest obstacle point in robot base frame
            zone                 — safety zone string
            confidence           — float in [0, 1]
            n_valid_pts          — int, depth pixels in winning window
            winner_cp            — the winning CP dict (for debug overlay)

        Returns None when insufficient data (no depth image, intrinsics, or
        transform) or when no valid obstacle point is found within 5 m.
        """
        if self._depth_image is None or self._depth_K is None:
            return None
        if self._T_base_cam is None or self._T_cam_base is None:
            return None

        ctrl_pts = self._build_robot_control_points(segments)
        if not ctrl_pts:
            return None

        R_cb = self._T_cam_base[:3, :3]   # base → camera
        t_cb = self._T_cam_base[:3, 3]
        R_bc = self._T_base_cam[:3, :3]   # camera → base
        t_bc = self._T_base_cam[:3, 3]

        # ── Self-mask: cached across cycles while robot pose is unchanged ──
        _t0_mask  = time.perf_counter()
        robot_mask = self._maybe_rebuild_robot_self_mask(segments)
        _t_mask   = time.perf_counter() - _t0_mask

        best_dist          = float("inf")
        best_cp            = None
        best_obs_cam       = None
        best_n_pts         = 0
        best_n_eval        = 0
        best_half_w        = 0
        total_masked       = 0
        n_cps_evaluated    = 0
        n_cps_no_cands     = 0
        sum_half_w         = 0

        _t0_search = time.perf_counter()
        for cp in ctrl_pts:
            p_base = cp["pt_base"]
            radius = cp["radius"]

            # Project CP to depth image
            px_d = self._project_point_to_depth_pixel(p_base)
            if px_d is None:
                continue
            px, py = px_d
            if not (0 <= px < self._depth_width and 0 <= py < self._depth_height):
                continue

            # CP in camera frame (needed for adaptive window and distance)
            cp_cam = R_cb @ p_base + t_cb

            # ── Adaptive surveillance window ───────────────────────────
            if self._use_adaptive_surv_window:
                half_w = self._compute_surveillance_window_px(cp_cam)
            else:
                half_w = self._surv_half_window_px
            sum_half_w += half_w

            # Gather obstacle candidates (self-mask applied inside)
            obs_cam, n_masked = self._collect_depth_candidates_around_control_point(
                px, py, robot_mask=robot_mask, half_w=half_w)
            total_masked += n_masked
            n_cps_evaluated += 1

            if obs_cam is None:
                n_cps_no_cands += 1
                continue

            # ── True local minimum with progressive contraction ────────
            obs_min, d_eff, n_eval, n_total = self._find_local_obstacle_minimum(
                obs_cam, cp_cam, radius)

            if d_eff < best_dist:
                best_dist    = d_eff
                best_cp      = cp
                best_obs_cam = obs_min
                best_n_pts   = n_total
                best_n_eval  = n_eval
                best_half_w  = half_w

        _t_search = time.perf_counter() - _t0_search

        # ── Update timing EMAs ─────────────────────────────────────────
        _a = self._timing_alpha
        self._t_mask_avg   = (1.0 - _a) * self._t_mask_avg   + _a * _t_mask
        self._t_search_avg = (1.0 - _a) * self._t_search_avg + _a * _t_search

        if best_cp is None or not np.isfinite(best_dist) or best_dist > 5.0:
            return None

        # Floor near-zero to avoid jitter at contact
        best_dist = max(best_dist, 0.01)

        # Convert to base frame
        cp_base  = best_cp["pt_base"].copy()
        obs_base = R_bc @ best_obs_cam + t_bc

        # Direction: FROM obstacle TOWARD robot CP (in base frame)
        diff      = cp_base - obs_base
        diff_norm = float(np.linalg.norm(diff))
        direction = (diff / diff_norm
                     if diff_norm > 1e-9
                     else np.array([0., 0., 1.]))

        # Confidence: scales with candidate count and proximity
        lm_conf   = float(np.clip(best_n_pts / 500.0, 0.2, 1.0))
        dist_conf = (1.0 if best_dist < 2.0
                     else float(np.clip(1.0 - (best_dist - 2.0) / 3.0,
                                        0.3, 1.0)))
        confidence = float(np.clip(lm_conf * dist_conf, 0.0, 1.0))

        avg_half_w = (sum_half_w // n_cps_evaluated
                      if n_cps_evaluated > 0 else 0)

        return {
            "robot_link_name":     best_cp["link"],
            "distance":            best_dist,
            "direction":           direction,
            "closest_point_robot": cp_base,
            "closest_point_human": obs_base,
            "zone":                _classify_zone(best_dist),
            "confidence":          confidence,
            "n_valid_pts":         best_n_pts,
            "n_masked_total":      total_masked,
            "n_cps_total":         len(ctrl_pts),
            "n_cps_evaluated":     n_cps_evaluated,
            "n_cps_no_cands":      n_cps_no_cands,
            "winner_n_eval":       best_n_eval,
            "winner_half_w":       best_half_w,
            "avg_half_w":          avg_half_w,
            "winner_cp":           best_cp,
            "robot_mask":          robot_mask,
            "ctrl_pts":            ctrl_pts,   # reused by debug overlay
        }

    # ================================================================
    # 3-D → pixel projection  (for debug overlay)
    # ================================================================

    def _project_to_rgb_pixel(
        self, p_base: np.ndarray,
    ) -> Optional[Tuple[int, int]]:
        """Project a 3-D base-frame point to an RGB image pixel (u, v).

        Returns None if behind the camera or calibration unavailable.
        """
        if self._T_cam_base is None or self._rgb_K is None:
            return None

        p_cam = self._T_cam_base[:3, :3] @ p_base + self._T_cam_base[:3, 3]
        if p_cam[2] <= 0.01:
            return None

        fx = float(self._rgb_K[0, 0])
        fy = float(self._rgb_K[1, 1])
        cx = float(self._rgb_K[0, 2])
        cy = float(self._rgb_K[1, 2])

        u = int(round(fx * p_cam[0] / p_cam[2] + cx))
        v = int(round(fy * p_cam[1] / p_cam[2] + cy))
        return (u, v)

    # ── legacy alias (keep compatibility with any external callers) ───
    def _project_to_pixel(self, p_base: np.ndarray) -> Optional[Tuple[int, int]]:
        return self._project_to_rgb_pixel(p_base)

    # ================================================================
    # Diagnostics helpers
    # ================================================================

    def _invalid_reason(self) -> str:
        """Return a short string describing why distance computation failed."""
        if not self._pin_ok:
            return "pinocchio_not_ready"
        if self._q is None:
            return "no_joint_states"
        if self._depth_image is None:
            return "no_depth_image"
        if self._depth_K is None:
            return "no_depth_intrinsics"
        if self._T_base_cam is None:
            return "no_camera_extrinsic"
        if self._debug_robot_centric:
            return "no_obstacle_in_surveillance_windows"
        if self._landmarks is None:
            return "no_landmarks"
        return "insufficient_depth_in_roi"

    # ================================================================
    # Landmark diagnostic  (visualisation only — does not touch distance)
    # ================================================================

    def _find_closest_landmark_diagnostic(
        self,
        segments: List[Dict],
    ) -> Optional[Dict]:
        """Find the MediaPipe landmark whose 3-D position is closest to the robot.

        For each valid landmark the depth image is sampled at the landmark pixel
        (3×3 neighbourhood median) to reconstruct a 3-D camera-frame point.
        The surface distance to every capsule is then computed with the same
        geometry used in _compute_min_distance_depth_space.

        Returns a dict with overlay fields, or None when unavailable:
          landmark_index        — index into lm.ids / lm.u / lm.v
          landmark_id           — MediaPipe landmark ID
          landmark_pixel        — (u, v) in RGB/debug image pixel space
          closest_robot_link    — capsule parent name
          closest_robot_base    — np.ndarray (3,) in robot base frame
          landmark_distance     — surface distance [m] (diagnostic only)

        This is PURELY diagnostic and does NOT modify _last_result or the
        published HumanRobotDistance message.
        """
        lm = self._landmarks
        if lm is None or not segments:
            return None
        if self._depth_image is None or self._depth_K is None:
            return None
        if self._T_cam_base is None or self._T_base_cam is None:
            return None

        K     = self._depth_K
        fx_d  = float(K[0, 0])
        fy_d  = float(K[1, 1])
        cx_d  = float(K[0, 2])
        cy_d  = float(K[1, 2])
        if fx_d < 1e-6 or fy_d < 1e-6:
            return None

        # Scale: landmarks are in RGB-image pixel space → depth pixel space
        if self._rgb_K is not None and lm.image_width > 0 and lm.image_height > 0:
            scale_u = float(self._depth_width)  / float(lm.image_width)
            scale_v = float(self._depth_height) / float(lm.image_height)
        else:
            scale_u = 1.0
            scale_v = 1.0

        R_cb = self._T_cam_base[:3, :3]   # base → camera rotation
        t_cb = self._T_cam_base[:3, 3]
        R_bc = self._T_base_cam[:3, :3]   # camera → base rotation
        t_bc = self._T_base_cam[:3, 3]

        best_dist    = float("inf")
        best_lm_info = None

        for i in range(len(lm.ids)):
            lm_id = int(lm.ids[i])
            if not self._landmark_passes_filters(lm_id, float(lm.visibility[i])):
                continue

            # Landmark pixel in depth-image space (for depth sampling)
            u_d  = lm.u[i] * scale_u
            v_d  = lm.v[i] * scale_v
            ud_i = int(round(u_d))
            vd_i = int(round(v_d))
            if not (0 <= ud_i < self._depth_width and
                    0 <= vd_i < self._depth_height):
                self._tlog.warn(
                    f"lm #{lm_id}: pixel ({ud_i},{vd_i}) outside depth image "
                    f"({self._depth_width}×{self._depth_height}) — skipped"
                )
                continue

            # Sample depth near landmark pixel (expanded search in relaxed mode)
            z_m = self._sample_landmark_depth(ud_i, vd_i)
            if z_m is None:
                self._tlog.warn(
                    f"lm #{lm_id}: no valid depth at ({ud_i},{vd_i})"
                    + (" even in 15×15 window"
                       if self._debug_disable_filters else "")
                    + " — skipped"
                )
                continue

            # Back-project to camera frame using depth intrinsics
            p_lm_cam = np.array([
                (u_d - cx_d) * z_m / fx_d,
                (v_d - cy_d) * z_m / fy_d,
                z_m,
            ], dtype=float)

            # Find closest capsule surface point (same geometry as main pipeline)
            cap_best_dist      = float("inf")
            cap_best_robot_cam = np.zeros(3)
            cap_best_name      = ""

            for seg in segments:
                p0_base = np.asarray(seg["p0"], dtype=float)
                p1_base = np.asarray(seg["p1"], dtype=float)
                radius  = float(seg["radius"])
                name    = str(seg.get("parent", ""))

                # Skip the first two robot capsules (same filter as robot-centric pipeline)
                if self._segment_is_excluded(seg):
                    continue

                p0_cam = R_cb @ p0_base + t_cb
                p1_cam = R_cb @ p1_base + t_cb
                if p0_cam[2] <= 0.0 and p1_cam[2] <= 0.0:
                    continue

                ab  = p1_cam - p0_cam
                ab2 = float(ab @ ab)
                if ab2 < 1e-12:
                    C = p0_cam.copy()
                else:
                    t_p = float(np.clip((p_lm_cam - p0_cam) @ ab / ab2, 0.0, 1.0))
                    C   = p0_cam + t_p * ab

                d_surf = max(float(np.linalg.norm(p_lm_cam - C)) - radius, 0.0)
                if d_surf < cap_best_dist:
                    cap_best_dist      = d_surf
                    cap_best_robot_cam = C.copy()
                    cap_best_name      = name

            if cap_best_dist < best_dist:
                best_dist = cap_best_dist
                # Landmark pixel in RGB/debug image coords (lm.u, lm.v are RGB)
                lm_px_rgb    = (int(round(lm.u[i])), int(round(lm.v[i])))
                p_robot_base = R_bc @ cap_best_robot_cam + t_bc
                best_lm_info = {
                    "landmark_index":     i,
                    "landmark_id":        lm_id,
                    "landmark_pixel":     lm_px_rgb,
                    "closest_robot_link": cap_best_name,
                    "closest_robot_base": p_robot_base,
                    "landmark_distance":  best_dist,
                    "landmark_point_cam": p_lm_cam,   # 3-D in camera frame
                }

        return best_lm_info

    # ================================================================
    # Debug overlay
    # ================================================================

    def _draw_debug_overlay(
        self,
        result: Optional[Dict],
        stamp,
        lm_diag: Optional[Dict] = None,
        ctrl_pts: Optional[List[Dict]] = None,
        winner_cp: Optional[Dict] = None,
    ) -> None:
        """Publish debug image with distance overlay on the raw RGB frame.

        Always draws an info panel (top-left).

        When ctrl_pts are provided (control-points debug mode):
          - GRAY  small circles : all robot CPs projected onto image
          - CYAN  large circle  : winner CP (closest to landmark)
          - ORANGE circle       : chosen landmark
          - YELLOW line         : landmark → winner CP
          - mid-segment label   : "<link>: X.XX m [CP]"
          - info panel          : lm id, link, cp index, d_euclid, radius, d_eff

        Fallback (no ctrl_pts):
          - depth-based overlay (closest depth point → robot capsule point)
        """
        if not _HAS_CV2:
            return
        # Use the raw RGB frame as base so that no MediaPipe skeleton annotation
        # bleeds into the output.  Fall back to the annotated debug image only
        # when the raw frame has not arrived yet (topic not publishing).
        image = self._raw_rgb_image if self._raw_rgb_image is not None \
            else self._debug_rgb_image
        if image is None:
            return

        h_img, w_img = image.shape[:2]
        font     = cv2.FONT_HERSHEY_SIMPLEX
        col_text = (255, 255, 255)
        col_bg   = (30,  30,  30)

        overlay = None

        def _ensure_overlay():
            nonlocal overlay
            if overlay is None:
                overlay = image.copy()
            return overlay

        # ── Info panel (always) ────────────────────────────────────────
        f_scale = 0.60
        f_thick = 1
        pad     = 8

        if winner_cp is not None and lm_diag is not None:
            # Full control-points debug mode
            lines = [
                f"lm #{lm_diag['landmark_id']} \u2192 "
                f"{winner_cp['link']} [cp{winner_cp['cp_idx']}]",
                f"d_euclid: {winner_cp['dist_euclid']:.2f} m",
                f"r_cap:    {winner_cp['radius']:.2f} m",
                f"d_eff:    {winner_cp['dist_eff']:.2f} m",
            ]
            if result is not None:
                lines.append(f"depth:    {result['distance']:.2f} m")
        elif result is not None:
            lines = [
                f"Nearest {result['robot_link_name']}",
                f"depth: {result['distance']:.2f} m",
            ]
            if lm_diag is not None:
                lines.append(
                    f"lm #{lm_diag['landmark_id']}: "
                    f"{lm_diag['landmark_distance']:.2f} m [diag]"
                )
        elif lm_diag is not None:
            lines = [
                f"lm #{lm_diag['landmark_id']} \u2192 "
                f"{lm_diag['closest_robot_link']}",
                f"{lm_diag['landmark_distance']:.2f} m [lm]",
            ]
        else:
            lines = ["No valid distance"]

        sizes   = [cv2.getTextSize(l, font, f_scale, f_thick) for l in lines]
        panel_w = max(s[0][0] for s in sizes) + 2 * pad
        line_h  = sizes[0][0][1]
        gap     = 6
        panel_h = len(lines) * line_h + (len(lines) - 1) * gap + 2 * pad

        _ensure_overlay()
        cv2.rectangle(overlay, (6, 6), (6 + panel_w, 6 + panel_h),
                      col_bg, -1)
        y = 6 + pad + line_h
        for line in lines:
            cv2.putText(overlay, line, (6 + pad, y),
                        font, f_scale, col_text, f_thick, cv2.LINE_AA)
            y += line_h + gap

        # ── Geometric overlay ──────────────────────────────────────────
        # Control-points mode: draw all CPs, highlight winner, landmark line.
        # Depth fallback: draw depth closest point → robot capsule point.

        if ctrl_pts:
            # Draw all control points (3 per admitted link)
            _ensure_overlay()
            for cp in ctrl_pts:
                px_cp = self._project_to_rgb_pixel(cp["pt_base"])
                if (px_cp is None
                        or not (0 <= px_cp[0] < w_img)
                        or not (0 <= px_cp[1] < h_img)):
                    continue
                is_winner = (
                    winner_cp is not None
                    and cp["seg_idx"] == winner_cp["seg_idx"]
                    and cp["cp_idx"]  == winner_cp["cp_idx"]
                )
                if is_winner:
                    cv2.circle(overlay, px_cp, 9,
                               (0, 255, 255), -1, cv2.LINE_AA)  # CYAN winner
                else:
                    cv2.circle(overlay, px_cp, 4,
                               (180, 180, 180), -1, cv2.LINE_AA)  # GRAY normal

        # Draw landmark and line landmark → winner CP
        if lm_diag is not None:
            px_lm = lm_diag["landmark_pixel"]
            in_lm = (0 <= px_lm[0] < w_img and 0 <= px_lm[1] < h_img)
            if in_lm:
                _ensure_overlay()
                cv2.circle(overlay, px_lm, 7,
                           (0, 128, 255), -1, cv2.LINE_AA)  # orange: landmark
            if winner_cp is not None:
                px_wc = self._project_to_rgb_pixel(winner_cp["pt_base"])
                if (in_lm and px_wc is not None
                        and 0 <= px_wc[0] < w_img
                        and 0 <= px_wc[1] < h_img):
                    _ensure_overlay()
                    cv2.line(overlay, px_lm, px_wc,
                             (0, 255, 255), 2, cv2.LINE_AA)
                    mid_label = (f"{winner_cp['link']}: "
                                 f"{winner_cp['dist_eff']:.2f} m [CP]")
                    mid_x = (px_lm[0] + px_wc[0]) // 2
                    mid_y = (px_lm[1] + px_wc[1]) // 2
                    f2    = 0.55
                    (mw, mh), mbase = cv2.getTextSize(
                        mid_label, font, f2, f_thick)
                    cv2.rectangle(
                        overlay,
                        (mid_x - 2, mid_y - mh - 4),
                        (mid_x + mw + 2, mid_y + mbase + 2),
                        col_bg, -1,
                    )
                    cv2.putText(
                        overlay, mid_label, (mid_x, mid_y),
                        font, f2, col_text, f_thick, cv2.LINE_AA,
                    )
        elif result is not None:
            # Depth-based fallback (no lm_diag, no ctrl_pts)
            p_human = result.get("closest_point_human")
            p_robot = result.get("closest_point_robot")
            px_human = (self._project_to_rgb_pixel(p_human)
                        if p_human is not None else None)
            px_robot = (self._project_to_rgb_pixel(p_robot)
                        if p_robot is not None else None)
            if (px_human is not None and px_robot is not None
                    and 0 <= px_human[0] < w_img and 0 <= px_human[1] < h_img
                    and 0 <= px_robot[0] < w_img and 0 <= px_robot[1] < h_img):
                _ensure_overlay()
                cv2.line(overlay, px_human, px_robot,
                         (0, 255, 255), 2, cv2.LINE_AA)
                cv2.circle(overlay, px_human, 7,
                           (0, 255, 0), -1, cv2.LINE_AA)    # green: human
                cv2.circle(overlay, px_robot, 6,
                           (0, 0, 255), -1, cv2.LINE_AA)    # red: robot

        # Publish annotated frame (or raw if nothing was drawn)
        frame_to_pub = overlay if overlay is not None else image

        out = Image()
        out.header.stamp    = stamp
        out.header.frame_id = "camera_color_optical_frame"
        out.height   = h_img
        out.width    = w_img
        out.encoding = "bgr8"
        out.step     = w_img * 3
        out.data     = frame_to_pub.tobytes()
        self._pub_debug_image.publish(out)

    # ================================================================
    # 2-D image debug — robot control points
    # ================================================================

    def _build_robot_debug_control_points(
        self,
        segments: List[Dict],
    ) -> List[Dict]:
        """Generate exactly 3 control points per admitted robot capsule.

        Samples t = (0.15, 0.50, 0.85) along each segment axis.
        Capsules with capsule_idx < _N_EXCLUDED_CAPSULES are skipped (same
        exclusion as the robot-centric pipeline).

        Returns a list of dicts — one per sample point:
            pt_base  — np.ndarray (3,) in robot base frame
            link     — str, parent link name
            radius   — float, capsule radius [m]
            seg_idx  — int, index in the segments list
            cp_idx   — int, 0 / 1 / 2  (position along segment)

        NOTE: these points are for visual debug only.  The real distance
        computation uses the full analytical capsule (axis + radius),
        NOT these discrete samples.
        """
        _TS = (0.15, 0.50, 0.85)
        ctrl_pts: List[Dict] = []
        for seg_idx, seg in enumerate(segments):
            if self._segment_is_excluded(seg):
                continue
            name = str(seg.get("parent", ""))
            p0  = np.asarray(seg["p0"], dtype=float)
            p1  = np.asarray(seg["p1"], dtype=float)
            rad = float(seg["radius"])
            for cp_idx, t in enumerate(_TS):
                ctrl_pts.append({
                    "pt_base": p0 + t * (p1 - p0),
                    "link":    name,
                    "radius":  rad,
                    "seg_idx": seg_idx,
                    "cp_idx":  cp_idx,
                })
        return ctrl_pts

    def _find_closest_robot_control_point_to_landmark(
        self,
        lm_diag: Dict,
        ctrl_pts: List[Dict],
    ) -> Optional[Dict]:
        """Return the control point with minimum surface distance to the landmark.

        dist_eff = max(||p_lm_base - pt_base|| − radius, 0.0)

        The landmark 3-D position comes from lm_diag["landmark_point_cam"]
        (camera frame), transformed to base frame via T_base_cam.

        Returns a copy of the winner dict augmented with:
            dist_euclid — float, raw Euclidean distance [m]
            dist_eff    — float, surface distance [m] (after radius subtraction)
            p_lm_base   — np.ndarray (3,), landmark in base frame
        Returns None when ctrl_pts is empty or the transform is unavailable.
        """
        if not ctrl_pts:
            return None
        p_lm_cam = lm_diag.get("landmark_point_cam")
        if p_lm_cam is None or self._T_base_cam is None:
            return None

        R_bc      = self._T_base_cam[:3, :3]
        t_bc      = self._T_base_cam[:3, 3]
        p_lm_base = R_bc @ np.asarray(p_lm_cam, dtype=float) + t_bc

        best_eff = float("inf")
        winner   = None
        for cp in ctrl_pts:
            d_euclid = float(np.linalg.norm(p_lm_base - cp["pt_base"]))
            d_eff    = max(d_euclid - cp["radius"], 0.0)
            if d_eff < best_eff:
                best_eff = d_eff
                winner   = {
                    **cp,
                    "dist_euclid": d_euclid,
                    "dist_eff":    d_eff,
                    "p_lm_base":   p_lm_base,
                }
        return winner

    # ================================================================
    # Robot-centric debug image
    # ================================================================

    def _make_depth_debug_base_image(self) -> Optional[np.ndarray]:
        """Build a colorised BGR image from the current depth frame.

        Normalises valid (> 0) depth values across the frame, inverts the
        scale so that near objects appear bright, then applies COLORMAP_JET.
        Invalid pixels are set to black.  Returns None when no depth is
        available or cv2 is not importable.
        """
        if not _HAS_CV2 or self._depth_image is None:
            return None

        d       = self._depth_image.astype(np.float32)
        valid   = d > 0
        gray    = np.zeros((self._depth_height, self._depth_width),
                           dtype=np.uint8)
        if np.any(valid):
            d_min = float(d[valid].min())
            d_max = float(d[valid].max())
            if d_max > d_min:
                # Map valid depths to 0–255, then invert so closer = brighter
                norm = np.clip(
                    255.0 * (d[valid] - d_min) / (d_max - d_min), 0, 255
                ).astype(np.uint8)
                gray[valid] = 255 - norm   # invert: near → bright

        colorised = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
        colorised[~valid] = (0, 0, 0)     # invalid pixels → black
        return colorised

    def _draw_robot_centric_debug_overlay(
        self,
        result: Optional[Dict],
        ctrl_pts: Optional[List[Dict]],
        stamp,
        robot_mask: Optional[np.ndarray] = None,
    ) -> None:
        """Publish robot-centric debug image on the DEPTH colorised frame.

        Everything lives in depth-image pixel space, so mask, control-point
        projections, obstacle point, and distance line are all consistent.

        Layer order (bottom → top):
          1. Colorised depth image (JET colourmap, near = bright)
          2. MAGENTA semi-transparent tint — robot self-mask
          3. GREEN small circles           — admitted control points
          4. YELLOW large circle           — winner control point
          5. RED circle                    — closest obstacle pixel
          6. CYAN rectangle                — surveillance window of winner CP
          7. WHITE line + mid label        — winner CP → obstacle point
          8. Info panel (top-left, dark background)

        All projections use depth intrinsics (_depth_K) via
        _project_point_to_depth_pixel(), which is exactly the same
        projection used to build the self-mask and the surveillance windows.
        """
        if not _HAS_CV2:
            return

        base = self._make_depth_debug_base_image()
        if base is None:
            return

        h_img, w_img = base.shape[:2]   # == depth_height, depth_width
        font     = cv2.FONT_HERSHEY_SIMPLEX
        col_text = (255, 255, 255)
        col_bg   = (20,  20,  20)
        f_scale  = 0.55
        f_thick  = 1
        pad      = 7

        overlay = base.copy()

        # ── Layer 2: robot self-mask (magenta tint, 40 % opacity) ─────
        if self._self_mask_draw and robot_mask is not None:
            # The mask is already in depth-image space — no resize needed.
            alpha   = 0.40
            magenta = np.array([200, 0, 200], dtype=np.float32)  # BGR
            idx     = robot_mask   # boolean (H, W)
            overlay[idx] = np.clip(
                overlay[idx].astype(np.float32) * (1.0 - alpha) +
                magenta * alpha,
                0, 255,
            ).astype(np.uint8)
            # Contour for clear boundary
            contours, _ = cv2.findContours(
                robot_mask.astype(np.uint8) * 255,
                cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(overlay, contours, -1, (220, 0, 220), 1,
                             cv2.LINE_AA)

        # ── Layer 3+4: control point projections (depth pixel space) ──
        winner_cp = result.get("winner_cp") if result is not None else None
        winner_px_d: Optional[Tuple[int, int]] = None

        if ctrl_pts and self._debug_draw_all_cps:
            for cp in ctrl_pts:
                px_d = self._project_point_to_depth_pixel(cp["pt_base"])
                if (px_d is None
                        or not (0 <= px_d[0] < w_img)
                        or not (0 <= px_d[1] < h_img)):
                    continue
                is_winner = (
                    winner_cp is not None
                    and cp["seg_idx"] == winner_cp["seg_idx"]
                    and cp["cp_idx"]  == winner_cp["cp_idx"]
                )
                if is_winner:
                    winner_px_d = px_d
                    cv2.circle(overlay, px_d, 9,
                               (0, 230, 230), -1, cv2.LINE_AA)   # YELLOW winner
                    cv2.circle(overlay, px_d, 11,
                               (255, 255, 255), 1, cv2.LINE_AA)  # white ring
                else:
                    cv2.circle(overlay, px_d, 4,
                               (0, 200, 0), -1, cv2.LINE_AA)     # GREEN normal

        # ── Layer 5+6: obstacle point + surveillance window ───────────
        obs_px_d: Optional[Tuple[int, int]] = None
        if result is not None:
            p_obs = result.get("closest_point_human")
            if p_obs is not None:
                obs_px_d = self._project_point_to_depth_pixel(p_obs)

            # Surveillance window around winner CP
            if winner_px_d is not None:
                h_win = self._surv_half_window_px
                rx0 = max(0, winner_px_d[0] - h_win)
                ry0 = max(0, winner_px_d[1] - h_win)
                rx1 = min(w_img - 1, winner_px_d[0] + h_win)
                ry1 = min(h_img - 1, winner_px_d[1] + h_win)
                cv2.rectangle(overlay, (rx0, ry0), (rx1, ry1),
                              (180, 180, 0), 1, cv2.LINE_AA)     # dark yellow box

            if (obs_px_d is not None
                    and 0 <= obs_px_d[0] < w_img
                    and 0 <= obs_px_d[1] < h_img):
                cv2.circle(overlay, obs_px_d, 7,
                           (0, 0, 255), -1, cv2.LINE_AA)         # RED: obstacle
                cv2.circle(overlay, obs_px_d, 9,
                           (255, 255, 255), 1, cv2.LINE_AA)      # white ring

        # ── Layer 7: distance line + mid label ────────────────────────
        if (winner_px_d is not None and obs_px_d is not None
                and 0 <= winner_px_d[0] < w_img
                and 0 <= winner_px_d[1] < h_img
                and 0 <= obs_px_d[0] < w_img
                and 0 <= obs_px_d[1] < h_img
                and result is not None):
            cv2.line(overlay, winner_px_d, obs_px_d,
                     (255, 255, 255), 2, cv2.LINE_AA)
            mid_label = (f"d={result['distance']:.3f}m "
                         f"[{result['robot_link_name']}]")
            mid_x = (winner_px_d[0] + obs_px_d[0]) // 2
            mid_y = (winner_px_d[1] + obs_px_d[1]) // 2
            f2 = 0.48
            (mw, mh), mbase = cv2.getTextSize(mid_label, font, f2, f_thick)
            cv2.rectangle(overlay,
                          (mid_x - 2, mid_y - mh - 4),
                          (mid_x + mw + 2, mid_y + mbase + 2),
                          col_bg, -1)
            cv2.putText(overlay, mid_label, (mid_x, mid_y),
                        font, f2, col_text, f_thick, cv2.LINE_AA)

        # ── Layer 8: info panel ───────────────────────────────────────
        mask_px = (int(np.count_nonzero(robot_mask))
                   if robot_mask is not None else 0)
        n_cps   = len(ctrl_pts) if ctrl_pts else 0

        if result is not None:
            n_cps_tot  = result.get("n_cps_total",    n_cps)
            n_cps_eval = result.get("n_cps_evaluated", "?")
            n_no_cands = result.get("n_cps_no_cands",  "?")
            n_eval     = result.get("winner_n_eval",   "?")
            win_hw     = result.get("winner_half_w",   "?")
            avg_hw_val = result.get("avg_half_w",      "?")
            avg_hw_str = (f"{avg_hw_val:.0f}" if isinstance(avg_hw_val, float)
                          else str(avg_hw_val))
            lines = [
                f"DEPTH-RC  {result['robot_link_name']}",
                f"d_eff:   {result['distance']:.3f} m",
                f"zone:    {result['zone']}",
                f"conf:    {result['confidence']:.2f}",
                f"CPs tot/eval: {n_cps_tot}/{n_cps_eval}",
                f"no-cands CPs: {n_no_cands}",
                f"win cands/eval: {result.get('n_valid_pts','?')}/{n_eval}",
                f"win_hw: {win_hw}px  avg_hw: {avg_hw_str}px",
                f"masked: {result.get('n_masked_total','?')}  mask_px: {mask_px}",
                f"method: true-min + contraction",
            ]
        else:
            lines = [
                "DEPTH-RC  no distance",
                f"CPs: {n_cps}  mask_px: {mask_px}",
            ]

        sizes   = [cv2.getTextSize(ln, font, f_scale, f_thick) for ln in lines]
        panel_w = max(s[0][0] for s in sizes) + 2 * pad
        line_h  = sizes[0][0][1]
        gap     = 5
        panel_h = len(lines) * line_h + (len(lines) - 1) * gap + 2 * pad
        cv2.rectangle(overlay, (6, 6), (6 + panel_w, 6 + panel_h), col_bg, -1)
        y_txt = 6 + pad + line_h
        for ln in lines:
            cv2.putText(overlay, ln, (6 + pad, y_txt),
                        font, f_scale, col_text, f_thick, cv2.LINE_AA)
            y_txt += line_h + gap

        # ── Publish ───────────────────────────────────────────────────
        out = Image()
        out.header.stamp    = stamp
        out.header.frame_id = "camera_depth_optical_frame"
        out.height   = h_img
        out.width    = w_img
        out.encoding = "bgr8"
        out.step     = w_img * 3
        out.data     = overlay.tobytes()
        self._pub_debug_image.publish(out)

    # ================================================================
    # Distance scalar publisher  (PlotJuggler / rqt_plot)
    # ================================================================

    def _publish_distance_scalar(self, result: Optional[Dict]) -> None:
        """Publish the minimum distance as std_msgs/Float32.

        Publishes the measured distance [m] when valid, −1.0 otherwise.
        Subscribe to ``/human_robot/distance_value`` in PlotJuggler or
        rqt_plot to get a live time-series chart.
        """
        scalar = Float32()
        scalar.data = float(result["distance"]) if result is not None else -1.0
        self._pub_distance_scalar.publish(scalar)

    # ================================================================
    # Debug-image gating
    # ================================================================

    def _should_publish_debug_image(self, now_sec: float) -> bool:
        """Return True if a debug image should be published this cycle."""
        if not self._enable_debug_image:
            return False
        if self._debug_image_rate > 0.0:
            if (now_sec - self._last_debug_pub_time) < (
                    1.0 / self._debug_image_rate):
                return False
        if self._debug_pub_only_if_sub:
            if self._pub_debug_image.get_subscription_count() == 0:
                return False
        return True

    # ================================================================
    # Timer callback — main pipeline
    # ================================================================

    def _compute_and_publish(self) -> None:
        """Main timer loop — depth-space distance pipeline.

        Stage 1  DISTANCE PIPELINE (skipped when inputs unchanged):
          A. Build human ROI from MediaPipe landmarks in depth image space.
          B. Extract human 3-D points in camera frame from depth ROI.
          C. Transform robot capsule segments to camera frame.
          D. Compute minimum surface distance (vectorised, per capsule).

        Stage 2  PUBLISH HumanRobotDistance (always, even using cached result).

        Stage 3  DEBUG IMAGE (rate-limited, gated on subscribers).
        """
        now    = self.get_clock().now().to_msg()
        t_wall = time.time()

        # ── Stage 1: DISTANCE PIPELINE ────────────────────────────────
        current_gen = (self._lm_gen, self._depth_gen, self._q_gen)
        if current_gen != self._last_computed_gen:
            segments = self._get_world_segments_cached()

            if self._debug_robot_centric:
                # ── Robot-centric depth-space pipeline (primary) ───────
                # No landmarks required: project robot CPs to depth image
                # and scan local surveillance windows for the nearest obstacle.
                if segments:
                    self._last_result = (
                        self._compute_min_distance_robot_centric_depth_space(
                            segments))
                else:
                    self._last_result = None
            else:
                # ── Legacy landmark-ROI pipeline ───────────────────────
                human_roi = self._build_human_depth_roi_from_landmarks()
                human_pts_cam = None
                if human_roi is not None:
                    human_pts_cam = self._get_human_pts_cam_in_roi(human_roi)
                if human_pts_cam is not None and segments:
                    self._last_result = self._compute_min_distance_depth_space(
                        segments, human_pts_cam)
                else:
                    self._last_result = None

            self._last_computed_gen = current_gen

        result = self._last_result

        # ── Stage 2: PUBLISH HumanRobotDistance ───────────────────────
        msg = HumanRobotDistance()
        msg.header.stamp    = now
        msg.header.frame_id = "base"

        if result is None:
            msg.valid      = False
            msg.distance   = 999.0
            msg.confidence = 0.0
            msg.zone       = "safe"
        else:
            msg.valid               = True
            msg.robot_link_name     = result["robot_link_name"]
            msg.distance            = float(result["distance"])
            msg.direction           = Vector3(
                x=float(result["direction"][0]),
                y=float(result["direction"][1]),
                z=float(result["direction"][2]),
            )
            msg.closest_point_robot = Point(
                x=float(result["closest_point_robot"][0]),
                y=float(result["closest_point_robot"][1]),
                z=float(result["closest_point_robot"][2]),
            )
            msg.zone       = result["zone"]
            msg.confidence = float(result["confidence"])

        self._pub.publish(msg)

        # ── Stage 3: Scalar ───────────────────────────────────────────
        self._publish_distance_scalar(result)

        # ── Stage 4: DEBUG IMAGE PIPELINE ─────────────────────────────
        if self._should_publish_debug_image(t_wall):
            if result is not None or self._debug_draw_no_valid:
                if self._debug_robot_centric:
                    # Robot-centric overlay: ctrl_pts and robot_mask are taken
                    # from the result dict (built during Stage 1) to avoid
                    # redundant recomputation.  Fall back to rebuilding ctrl_pts
                    # only when there is no valid result (e.g. no obstacle found
                    # but debug_draw_no_valid_distance is True).
                    if result is not None:
                        ctrl_pts   = result.get("ctrl_pts")
                        robot_mask = result.get("robot_mask")
                    else:
                        ctrl_pts   = (self._build_robot_control_points(
                            self._cached_segments)
                            if self._cached_segments else None)
                        robot_mask = self._self_mask_cache  # use cached mask

                    _t0_render = time.perf_counter()
                    self._draw_robot_centric_debug_overlay(
                        result, ctrl_pts, now, robot_mask=robot_mask)
                    _t_render = time.perf_counter() - _t0_render
                    _a = self._timing_alpha
                    self._t_render_avg = (
                        (1.0 - _a) * self._t_render_avg + _a * _t_render)
                else:
                    # Legacy landmark-based overlay
                    lm_diag   = None
                    ctrl_pts  = None
                    winner_cp = None
                    if self._cached_segments:
                        lm_diag  = self._find_closest_landmark_diagnostic(
                            self._cached_segments)
                        ctrl_pts = self._build_robot_debug_control_points(
                            self._cached_segments)
                        if lm_diag is not None and ctrl_pts:
                            winner_cp = (
                                self._find_closest_robot_control_point_to_landmark(
                                    lm_diag, ctrl_pts))
                    self._draw_debug_overlay(
                        result, now,
                        lm_diag=lm_diag,
                        ctrl_pts=ctrl_pts,
                        winner_cp=winner_cp,
                    )
                self._last_debug_pub_time = t_wall

        # ── Throttled log ─────────────────────────────────────────────
        if self._tlog.due(t_wall):
            if self._debug_robot_centric and self._cached_segments:
                # Log exclusion and CP count once per throttle period
                n_segs_total    = len(self._cached_segments)
                n_segs_excluded = sum(
                    1 for s in self._cached_segments
                    if self._segment_is_excluded(s))
                n_cps = self._robot_cps_per_seg * (n_segs_total - n_segs_excluded)
                mask_px = 0
                cached_mask = self._self_mask_cache
                if cached_mask is not None:
                    mask_px = int(np.count_nonzero(cached_mask))
                self._tlog.info(
                    f"RC pipeline: {n_segs_total} capsules total, "
                    f"{n_segs_excluded} excluded (fr3_cap_0..{n_segs_excluded-1}), "
                    f"{n_cps} CPs active | "
                    f"self-mask: r_px={self._self_mask_radius_px} "
                    f"dilate={self._self_mask_dilate_px} "
                    f"covered={mask_px}px"
                )
                # ── Timing report ──────────────────────────────────────
                total_ops = self._mask_cache_hits + self._mask_cache_misses
                hit_pct   = (100.0 * self._mask_cache_hits / total_ops
                             if total_ops > 0 else 0.0)
                self._tlog.info(
                    f"TIMING [ms] mask={self._t_mask_avg*1e3:.1f} "
                    f"search={self._t_search_avg*1e3:.1f} "
                    f"render={self._t_render_avg*1e3:.1f} | "
                    f"mask-cache hits={self._mask_cache_hits} "
                    f"miss={self._mask_cache_misses} "
                    f"({hit_pct:.0f}% reuse)"
                )
            if result is not None:
                ph = result["closest_point_human"]
                pr = result["closest_point_robot"]
                self._tlog.info(
                    f"winner={result['robot_link_name']}  "
                    f"d={result['distance']:.3f}m  zone={result['zone']}  "
                    f"CPs={result.get('n_cps_total','?')} eval={result.get('n_cps_evaluated','?')} "
                    f"no-cands={result.get('n_cps_no_cands','?')}  "
                    f"win-cands={result.get('n_valid_pts','?')} n_eval={result.get('winner_n_eval','?')}  "
                    f"win_hw={result.get('winner_half_w','?')}px avg_hw={result.get('avg_half_w','?')}px  "
                    f"masked_total={result.get('n_masked_total','?')}"
                )
                self._tlog.info(
                    f"  robot_pt=[{pr[0]:.3f} {pr[1]:.3f} {pr[2]:.3f}]  "
                    f"obs_pt=[{ph[0]:.3f} {ph[1]:.3f} {ph[2]:.3f}]"
                )
            else:
                self._tlog.warn("No valid human-robot distance")


def main(args=None):
    rclpy.init(args=args)
    node = HumanDistanceEstimator()
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
