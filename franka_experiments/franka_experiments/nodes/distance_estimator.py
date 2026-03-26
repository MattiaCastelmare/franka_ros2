#!/usr/bin/env python3
"""
HUMAN-ROBOT DISTANCE ESTIMATOR  —  Depth-Space Approach
=========================================================
Direct implementation of Flacco, Kroeger, De Luca, Khatib (2015),
"A Depth Space Approach to Human-Robot Collision Avoidance."

Pipeline
--------
Step 1  Forward kinematics → robot capsule control points (CPs).
Step 2  Project each CP to the depth image (pinhole model, Eq. 5).
Step 3  Collect surveillance-window obstacle candidates (Eq. 10–11).
Step 4  Frustum correction per obstacle pixel (Eq. 8, optional).
Step 5  Depth-space distance Eq. (7) with gray-area handling (Section 3.3).
Step 6  Minimum distance with contraction (Eq. 12, Section 3.4.1).
Step 7  Hybrid aggregation — D_min distance, V_mean direction (Eq. 15, Section 3.4.3).
Step 8  Self-mask (Section 3.5) + depth gate (extension not in paper).
Step 9  Publish HumanRobotDistance on /human_robot/closest_distance.

Convention
----------
``direction`` points FROM the obstacle TOWARD the robot (repulsion direction)
in the robot base frame, matching ``online_avoidance_controller``.
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
# iter_world_capsule_segments, NOT on the parent-name string.
_N_EXCLUDED_CAPSULES: int = 2

# Control points per capsule (keyed on capsule_idx).
# Capsules 0-1 are excluded from distance calculations; their entry is 0.
# Any capsule_idx not present in the dict falls back to _DEFAULT_CPS_PER_SEG.
_DEFAULT_CPS_PER_SEG: int = 5
_CPS_PER_CAPSULE: Dict[int, int] = {
    0: 0,   # fr3_cap_0 — excluded
    1: 0,   # fr3_cap_1 — excluded
    2: 0,   # link 3
    3: 5,   # link 4
    4: 5,   # link 5
    5: 5,   # link 6
    6: 5,   # link 7
}


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
    """Estimate minimum human-robot distance via the Flacco et al. depth-space approach."""

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
        self._depth_image: Optional[np.ndarray] = None
        self._depth_K: Optional[np.ndarray] = None   # 3×3 depth intrinsics
        self._depth_width: int = 0
        self._depth_height: int = 0

        # Debug overlay state
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
        self._depth_gen: int = 0
        self._q_gen: int = 0
        self._last_computed_gen: Tuple[int, int] = (-1, -1)
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

        depth_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Image, self._depth_image_topic, self._depth_image_cb, depth_qos)
        self.create_subscription(
            CameraInfo, self._depth_info_topic, self._depth_info_cb, 10)

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
            f"frustum_correction={self._use_frustum_correction}  "
            f"hybrid_aggregation={self._use_hybrid_aggregation}"
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
        self.declare_parameter("depth_image_topic",
                               "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("depth_info_topic",
                               "/camera/camera/depth/camera_info")

        self.declare_parameter("depth_min_m", 0.15)
        self.declare_parameter("depth_max_m", 5.0)
        self.declare_parameter("depth_scale", 0.001)

        self.declare_parameter("num_debug_distances", 3)
        self.declare_parameter("debug_output_topic",
                               "/human_robot/distance_debug_image")
        self.declare_parameter("enable_debug_image", True)
        self.declare_parameter("debug_image_rate", 5.0)
        self.declare_parameter("debug_publish_only_if_subscribed", True)
        self.declare_parameter("debug_draw_no_valid_distance", True)

        # ── Relaxed-filter debug mode ──────────────────────────────────
        # When True: bypass/relax all filter thresholds so that almost nothing
        # is discarded.  Does NOT change the algorithm; only changes when data
        # is considered valid.
        self.declare_parameter("debug_disable_filters", True)

        # ── Robot-centric depth-space pipeline ─────────────────────────
        # Number of control points sampled along each robot capsule axis.
        # Placed at t_k = (k+1)/(N+1), k=0..N-1 (interior, never at endpoints).
        self.declare_parameter("robot_control_points_per_segment", 5)
        # Half-side of the fallback square surveillance window [px].
        self.declare_parameter("surveillance_half_window_px", 30)
        # Metric radius [m] of the adaptive surveillance sphere around each CP.
        self.declare_parameter("surveillance_radius_m", 0.30)
        # When True (default) use the depth-adaptive window.
        self.declare_parameter("use_adaptive_surveillance_window", True)
        self.declare_parameter("local_best_k", 5)
        self.declare_parameter("debug_draw_all_robot_control_points", True)

        # ── Paper Eq. (8) — Frustum correction ─────────────────────────
        # When True (default), apply pixel-frustum correction to each obstacle
        # pixel before computing the depth-space distance.
        self.declare_parameter("use_frustum_correction", True)

        # ── Paper Eq. (15) — Hybrid aggregation ────────────────────────
        # When True (default), use V_mean (mean of unit vectors over surveillance
        # window) as the repulsion direction; distance stays D_min.
        # When False, use the argmin direction only (for comparison).
        self.declare_parameter("use_hybrid_aggregation", True)

        # ── Robot self-mask  (Section 3.5) ─────────────────────────────
        self.declare_parameter("enable_robot_self_mask", True)
        # Minimum painted radius [px] for each sampled capsule point.
        self.declare_parameter("robot_self_mask_radius_px", 15)
        # Extra dilation margin [px] added on top of the painted circles.
        self.declare_parameter("robot_self_mask_dilate_px", 8)
        # Minimum depth difference [m] between a candidate and the robot surface.
        # Extension to the paper: rejects candidates at or behind the robot.
        self.declare_parameter("robot_self_mask_depth_gate_m", 0.10)
        # Points sampled along each capsule axis for mask painting.
        self.declare_parameter("robot_self_mask_samples_per_segment", 20)
        self.declare_parameter("robot_self_mask_draw_on_debug", True)

        # ── Internal debug topic ────────────────────────────────────────
        self.declare_parameter("debug_method_topic", "/debug_method")
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
        self._depth_image_topic = str(p("depth_image_topic"))
        self._depth_info_topic = str(p("depth_info_topic"))
        self._depth_min_m = float(p("depth_min_m"))
        self._depth_max_m = float(p("depth_max_m"))
        self._depth_scale = float(p("depth_scale"))
        self._num_debug_distances = int(p("num_debug_distances"))
        self._debug_output_topic = str(p("debug_output_topic"))
        self._enable_debug_image = bool(p("enable_debug_image"))
        self._debug_image_rate = float(p("debug_image_rate"))
        self._debug_pub_only_if_sub = bool(p("debug_publish_only_if_subscribed"))
        self._debug_draw_no_valid = bool(p("debug_draw_no_valid_distance"))
        self._debug_disable_filters      = bool(p("debug_disable_filters"))
        self._robot_cps_per_seg        = max(1, int(p("robot_control_points_per_segment")))
        self._surv_half_window_px      = max(1, int(p("surveillance_half_window_px")))
        self._surveillance_radius_m    = max(0.05, float(p("surveillance_radius_m")))
        self._use_adaptive_surv_window = bool(p("use_adaptive_surveillance_window"))
        self._local_best_k             = max(1, int(p("local_best_k")))
        self._debug_draw_all_cps    = bool(p("debug_draw_all_robot_control_points"))
        self._use_frustum_correction = bool(p("use_frustum_correction"))
        self._use_hybrid_aggregation = bool(p("use_hybrid_aggregation"))
        self._enable_self_mask      = bool(p("enable_robot_self_mask"))
        self._self_mask_radius_px   = max(1, int(p("robot_self_mask_radius_px")))
        self._self_mask_dilate_px   = max(0, int(p("robot_self_mask_dilate_px")))
        self._self_mask_depth_gate_m = float(p("robot_self_mask_depth_gate_m"))
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
        """Pre-load depth camera intrinsics from YAML config file."""
        try:
            pkg_share = get_package_share_directory("franka_experiments")
        except Exception:
            self.get_logger().warn(
                "Cannot resolve franka_experiments share dir "
                "— skipping YAML intrinsics loading"
            )
            return

        depth_path = os.path.join(pkg_share, "config", "depth_intrinsics.yaml")
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
    # Filter helpers
    # ================================================================

    def _depth_range(self) -> Tuple[float, float]:
        """Return (d_min, d_max) [m] for depth validity.

        Relaxed: (0.0, +inf) — any positive finite depth is accepted.
        Normal:  (depth_min_m, depth_max_m) as configured.
        """
        if self._debug_disable_filters:
            return (0.0, float("inf"))
        return (self._depth_min_m, self._depth_max_m)

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
        """
        return int(seg.get("capsule_idx", 0)) < _N_EXCLUDED_CAPSULES

    # ================================================================
    # Robot control points  (Step 1 — Eq. 5)
    # ================================================================

    def _build_robot_control_points(
        self,
        segments: List[Dict],
    ) -> List[Dict]:
        """Generate N control points per admitted robot capsule axis.

        Points are placed at evenly-spaced interior positions along each
        segment:  t_k = (k+1) / (N+1)  for k = 0 … N-1.

        Capsules with ``capsule_idx < _N_EXCLUDED_CAPSULES`` are skipped.

        Returns a list of dicts:
            pt_base  — np.ndarray (3,) in robot base frame
            link     — str, parent link name
            radius   — float, capsule radius [m]
            seg_idx  — int, index in the segments list
            cp_idx   — int, 0 … N-1
        """
        ctrl_pts: List[Dict] = []
        for seg_idx, seg in enumerate(segments):
            if self._segment_is_excluded(seg):
                continue
            cap_idx = int(seg.get("capsule_idx", seg_idx))
            n = _CPS_PER_CAPSULE.get(cap_idx, _DEFAULT_CPS_PER_SEG)
            if n == 0:
                continue
            ts = [(k + 1) / (n + 1) for k in range(n)]
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

    # ================================================================
    # CP → depth-image pixel projection  (Eq. 5)
    # ================================================================

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

    # ================================================================
    # Frustum correction  (Eq. 8)
    # ================================================================

    def _apply_frustum_correction(
        self,
        obs_uv: np.ndarray,   # (M, 2) int pixel coords of obstacle pixels
        cp_uv:  np.ndarray,   # (2,)   pixel coords of CP projection
    ) -> np.ndarray:          # (M, 2) float corrected sub-pixel coords
        """Pixel frustum correction — Eq. (8) of Flacco et al. 2015.

        For each obstacle pixel (o_bar_x, o_bar_y) and CP projection (px, py):
          o_hat_x = clip(px, o_bar_x, o_bar_x + 1)
          o_hat_y = clip(py, o_bar_y, o_bar_y + 1)

        The result is the sub-pixel position within the obstacle pixel's unit
        square that is closest to the CP projection.  Returns float (M, 2).
        """
        px = float(cp_uv[0])
        py = float(cp_uv[1])
        o_bar = obs_uv.astype(float)           # (M, 2)
        o_hat_x = np.clip(px, o_bar[:, 0], o_bar[:, 0] + 1.0)
        o_hat_y = np.clip(py, o_bar[:, 1], o_bar[:, 1] + 1.0)
        return np.stack([o_hat_x, o_hat_y], axis=1)   # (M, 2)

    # ================================================================
    # Depth-space distance  (Eq. 7 + Section 3.3 gray area)
    # ================================================================

    def _depth_space_distance(
        self,
        obs_uv_z: np.ndarray,   # (M, 3): [u_px, v_px, depth_m] for each candidate
        cp_uv_z:  np.ndarray,   # (3,):   [u_px, v_px, depth_m] for the CP
    ) -> np.ndarray:            # (M,) raw depth-space distances (before radius subtraction)
        """Vectorised Eq. (7) from Flacco et al. 2015 with gray-area handling.

        For each candidate i:
          do = obs_z[i]; dp = cp_z
          Gray area (Section 3.3): if do <= dp, substitute do = dp.
          vx = (ox - cx) * do/fx  -  (px - cx) * dp/fx
          vy = (oy - cy) * do/fy  -  (py - cy) * dp/fy
          vz = do - dp
          D_depth[i] = sqrt(vx^2 + vy^2 + vz^2)

        Uses self._depth_K for fx, fy, cx, cy.
        Returns raw distances — caller subtracts capsule radius.
        """
        K = self._depth_K
        fx = float(K[0, 0])
        fy = float(K[1, 1])
        cx = float(K[0, 2])
        cy = float(K[1, 2])

        ox = obs_uv_z[:, 0]        # (M,)
        oy = obs_uv_z[:, 1]        # (M,)
        do = obs_uv_z[:, 2].copy() # (M,) — copy so gray-area substitution is local

        px = float(cp_uv_z[0])
        py = float(cp_uv_z[1])
        dp = float(cp_uv_z[2])

        # Gray-area handling (Section 3.3): obstacle at or behind CP depth
        gray = do <= dp
        do[gray] = dp

        vx = (ox - cx) * do / fx - (px - cx) * dp / fx
        vy = (oy - cy) * do / fy - (py - cy) * dp / fy
        vz = do - dp

        return np.sqrt(vx * vx + vy * vy + vz * vz)   # (M,)

    # ================================================================
    # Surveillance window candidates  (Eq. 10–11)
    # ================================================================

    def _collect_depth_candidates_around_control_point(
        self,
        px_d: int,
        py_d: int,
        robot_mask: Optional[np.ndarray] = None,
        half_w: Optional[int] = None,
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], int]:
        """Return 3-D camera-frame obstacle points and depth-space coords.

        The window is a square of side (2*half_w + 1) centred on (px_d, py_d).
        ``half_w`` defaults to self._surv_half_window_px if not provided.
        Depth values are filtered using ``_depth_range()``.  Pixels inside
        ``robot_mask`` are excluded (robot self-mask, Section 3.5).

        Returns:
            (pts_cam, pts_uv_z, n_masked)
              pts_cam  — (M, 3) float array in camera frame, or None if empty
              pts_uv_z — (M, 3) depth-space coords [u_px, v_px, z_m], None if empty
              n_masked — number of pixels rejected by the robot self-mask
        """
        if self._depth_image is None or self._depth_K is None:
            return None, None, 0

        h = half_w if half_w is not None else self._surv_half_window_px
        x_min = max(0, px_d - h)
        x_max = min(self._depth_width  - 1, px_d + h)
        y_min = max(0, py_d - h)
        y_max = min(self._depth_height - 1, py_d + h)

        window = self._depth_image[y_min:y_max + 1, x_min:x_max + 1]

        depth_min_m, depth_max_m = self._depth_range()
        raw_min = max(1, int(depth_min_m / self._depth_scale))
        if math.isfinite(depth_max_m):
            raw_max    = int(depth_max_m / self._depth_scale)
            valid_mask = (window > 0) & (window >= raw_min) & (window <= raw_max)
        else:
            valid_mask = window >= raw_min   # raw_min ≥ 1 → window > 0 implied

        # ── Robot self-mask: exclude pixels belonging to the robot ─────
        n_masked = 0
        if robot_mask is not None:
            robot_window = robot_mask[y_min:y_max + 1, x_min:x_max + 1]
            n_before  = int(np.count_nonzero(valid_mask))
            valid_mask = valid_mask & ~robot_window
            n_masked  = n_before - int(np.count_nonzero(valid_mask))

        ys, xs = np.where(valid_mask)
        if len(ys) == 0:
            return None, None, n_masked

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

        pts_cam  = np.stack([x_cam, y_cam, z_vals], axis=1)          # (M, 3)
        pts_uv_z = np.stack([u_abs, v_abs, z_vals], axis=1)          # (M, 3)
        return pts_cam, pts_uv_z, n_masked

    # ================================================================
    # Robot self-mask  (Section 3.5)
    # ================================================================

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

                r_proj = int(round(fx * radius / z))
                r_px   = max(r_px_floor, r_proj)

                if (-r_px <= u < self._depth_width  + r_px and
                        -r_px <= v < self._depth_height + r_px):
                    cv2.circle(mask_u8, (u, v), r_px, 255, -1)

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

        Cache-hit:  returns ``_self_mask_cache`` without any computation.
        Cache-miss: calls ``_build_robot_self_mask``, stores result + gen.
        """
        if not self._enable_self_mask or not _HAS_CV2:
            return None
        if (self._self_mask_cache is not None
                and self._self_mask_gen == self._q_gen):
            self._mask_cache_hits += 1
            return self._self_mask_cache
        mask = self._build_robot_self_mask(segments)
        self._self_mask_cache  = mask
        self._self_mask_gen    = self._q_gen
        self._mask_cache_misses += 1
        return mask

    # ================================================================
    # Adaptive surveillance window  (Eq. 10–11)
    # ================================================================

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

    # ================================================================
    # Local minimum with progressive contraction  (Eq. 12, Section 3.4.1)
    # ================================================================

    def _find_local_obstacle_minimum(
        self,
        obs_cam: np.ndarray,
        cp_cam: np.ndarray,
        radius: float,
    ) -> Tuple[np.ndarray, float, int, int]:
        """Find the true local obstacle minimum with progressive contraction.

        Algorithm:
          1. Compute 3-D Euclidean distance from every candidate to cp_cam.
          2. Sort candidates near → far (ascending 3-D distance).
          3. Process in order:
               d_eff = max(d3d - radius, 0)
               if d_eff < best_d_eff  →  update best
               if d3d - radius ≥ best_d_eff  →  STOP (contraction)
          4. Return the true argmin candidate.

        Returns: (obs_point_cam, d_eff_min, n_evaluated, n_total)
        """
        n_total = len(obs_cam)

        if n_total == 1:
            d3d   = float(np.linalg.norm(obs_cam[0] - cp_cam))
            d_eff = max(d3d - radius, 0.0)
            return obs_cam[0], d_eff, 1, 1

        dists    = np.linalg.norm(obs_cam - cp_cam, axis=1)
        order    = np.argsort(dists)

        best_d_eff = float("inf")
        best_i     = int(order[0])
        n_eval     = 0

        for i in order:
            d3d = float(dists[i])
            if d3d - radius >= best_d_eff:
                break
            n_eval += 1
            d_eff = max(d3d - radius, 0.0)
            if d_eff < best_d_eff:
                best_d_eff = d_eff
                best_i     = int(i)
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

        Returns: (obs_cam_robust, d_eff_robust, k_used)
        """
        dists      = np.linalg.norm(obs_cam - cp_cam, axis=1)
        surf_dists = np.maximum(dists - radius, 0.0)

        k = min(self._local_best_k, len(obs_cam))
        if k < len(obs_cam):
            top_k_idx = np.argpartition(surf_dists, k)[:k]
        else:
            top_k_idx = np.arange(len(obs_cam))

        centroid = obs_cam[top_k_idx].mean(axis=0)
        d_to_centroid = float(np.linalg.norm(centroid - cp_cam))
        d_eff_robust  = max(d_to_centroid - radius, 0.0)

        return centroid, d_eff_robust, int(k)

    # ================================================================
    # Main depth-space pipeline  (Steps 2–7)
    # ================================================================

    def _compute_min_distance_robot_centric_depth_space(
        self,
        segments: List[Dict],
    ) -> Optional[Dict]:
        """Robot-centric depth-space distance pipeline (Flacco et al. 2015).

        For every robot control point (Step 1 result):
          Step 2  Project CP to depth image (Eq. 5).
          Step 3  Collect surveillance-window candidates (Eq. 10–11).
          Step 4  Apply frustum correction (Eq. 8, if enabled).
          Step 5  Compute depth-space distance (Eq. 7 + gray-area Section 3.3).
          Step 6  d_eff = max(D_depth - radius, 0); track global minimum.
        After loop:
          Step 7  Hybrid aggregation (Eq. 15): D_min distance, V_mean direction.

        Returns a result dict or None when insufficient data or no obstacle found.

        Result dict keys:
            robot_link_name      — link of the winning CP
            distance             — D_min surface distance [m]  (Eq. 15)
            direction            — unit vector FROM obstacle TOWARD robot (base)
                                   V_mean over surveillance window (Eq. 15),
                                   or argmin vector when use_hybrid_aggregation=False
            closest_point_robot  — winner CP in robot base frame
            closest_point_human  — closest obstacle point in robot base frame
            zone                 — safety zone string
            confidence           — float in [0, 1]
            n_valid_pts          — depth pixels in winning window
            winner_cp            — winning CP dict (for debug overlay)
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

        # ── Step 8a: Self-mask (cached across cycles) ─────────────────
        _t0_mask   = time.perf_counter()
        robot_mask = self._maybe_rebuild_robot_self_mask(segments)
        _t_mask    = time.perf_counter() - _t0_mask

        best_dist          = float("inf")
        best_cp            = None
        best_obs_cam       = None          # argmin obs point (camera frame)
        best_obs_cam_all   = None          # all obs in winner window (camera frame)
        best_n_pts         = 0
        best_half_w        = 0
        total_masked       = 0
        n_cps_evaluated    = 0
        n_cps_no_cands     = 0
        sum_half_w         = 0

        _t0_search = time.perf_counter()

        for cp in ctrl_pts:
            p_base = cp["pt_base"]
            radius = cp["radius"]

            # ── Step 2: Project CP to depth image (Eq. 5) ─────────────
            px_d = self._project_point_to_depth_pixel(p_base)
            if px_d is None:
                continue
            px, py = px_d
            if not (0 <= px < self._depth_width and 0 <= py < self._depth_height):
                continue

            cp_cam = R_cb @ p_base + t_cb

            # ── Adaptive surveillance window ──────────────────────────
            if self._use_adaptive_surv_window:
                half_w = self._compute_surveillance_window_px(cp_cam)
            else:
                half_w = self._surv_half_window_px
            sum_half_w += half_w

            # ── Step 3: Collect candidates (Eq. 10–11) ────────────────
            obs_cam, obs_uv_z, n_masked = (
                self._collect_depth_candidates_around_control_point(
                    px, py, robot_mask=robot_mask, half_w=half_w))
            total_masked += n_masked
            n_cps_evaluated += 1

            if obs_cam is None or obs_uv_z is None:
                n_cps_no_cands += 1
                continue

            # ── Step 8b: Depth gate (extension to Section 3.5) ────────
            z_robot = float(cp_cam[2])
            z_threshold = z_robot - self._self_mask_depth_gate_m
            keep = obs_cam[:, 2] < z_threshold
            obs_cam  = obs_cam[keep]
            obs_uv_z = obs_uv_z[keep]
            if obs_cam.shape[0] == 0:
                n_cps_no_cands += 1
                continue

            # ── Step 4: Frustum correction (Eq. 8, optional) ──────────
            cp_uv = np.array([float(px), float(py)])
            if self._use_frustum_correction:
                obs_uv_corr = self._apply_frustum_correction(
                    obs_uv_z[:, :2].astype(int), cp_uv)
                obs_uv_z_dist = np.stack(
                    [obs_uv_corr[:, 0], obs_uv_corr[:, 1], obs_uv_z[:, 2]],
                    axis=1)
            else:
                obs_uv_z_dist = obs_uv_z

            # ── Step 5: Depth-space distance (Eq. 7 + gray area) ──────
            cp_uv_z  = np.array([float(px), float(py), z_robot])
            d_depth  = self._depth_space_distance(obs_uv_z_dist, cp_uv_z)  # (M,)

            # ── Step 6: Surface distance; track global minimum ─────────
            d_eff_all = np.maximum(d_depth - radius, 0.0)
            idx_min   = int(np.argmin(d_eff_all))
            d_eff     = float(d_eff_all[idx_min])

            if d_eff < best_dist:
                best_dist        = d_eff
                best_cp          = cp
                best_obs_cam     = obs_cam[idx_min].copy()
                best_obs_cam_all = obs_cam          # full window for V_mean
                best_n_pts       = len(obs_cam)
                best_half_w      = half_w

        _t_search = time.perf_counter() - _t0_search

        # ── Update timing EMAs ─────────────────────────────────────────
        _a = self._timing_alpha
        self._t_mask_avg   = (1.0 - _a) * self._t_mask_avg   + _a * _t_mask
        self._t_search_avg = (1.0 - _a) * self._t_search_avg + _a * _t_search

        if best_cp is None or not np.isfinite(best_dist) or best_dist > 5.0:
            return None

        best_dist = max(best_dist, 0.01)   # floor near-zero to avoid jitter

        cp_base  = best_cp["pt_base"].copy()
        obs_base = R_bc @ best_obs_cam + t_bc

        # ── Step 7: Hybrid aggregation — Eq. (15) ─────────────────────
        # D_hybrid = D_min  (already in best_dist)
        # V_hybrid = normalised mean of unit vectors obs_i → CP (base frame)
        if self._use_hybrid_aggregation and best_obs_cam_all is not None:
            obs_base_all = (R_bc @ best_obs_cam_all.T).T + t_bc   # (M, 3)
            vecs  = cp_base[None, :] - obs_base_all                # (M, 3)
            norms = np.linalg.norm(vecs, axis=1, keepdims=True)    # (M, 1)
            valid = norms[:, 0] > 1e-9
            if np.any(valid):
                unit_vecs = vecs[valid] / norms[valid]             # (K, 3)
                v_mean    = unit_vecs.mean(axis=0)
                v_norm    = float(np.linalg.norm(v_mean))
                direction = (v_mean / v_norm
                             if v_norm > 1e-9 else np.array([0., 0., 1.]))
            else:
                direction = np.array([0., 0., 1.])
        else:
            # Argmin-only direction (use_hybrid_aggregation=False)
            diff      = cp_base - obs_base
            diff_norm = float(np.linalg.norm(diff))
            direction = (diff / diff_norm
                         if diff_norm > 1e-9 else np.array([0., 0., 1.]))

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
            "winner_half_w":       best_half_w,
            "avg_half_w":          avg_half_w,
            "winner_cp":           best_cp,
            "robot_mask":          robot_mask,
            "ctrl_pts":            ctrl_pts,   # reused by debug overlay
        }

    # ================================================================
    # Diagnostics
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
        return "no_obstacle_in_surveillance_windows"

    # ================================================================
    # Robot-centric debug image
    # ================================================================

    def _make_depth_debug_base_image(self) -> Optional[np.ndarray]:
        """Build a colorised BGR image from the current depth frame.

        Normalises valid (> 0) depth values across the frame, inverts the
        scale so that near objects appear bright, then applies COLORMAP_JET.
        Invalid pixels are set to black.
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
                norm = np.clip(
                    255.0 * (d[valid] - d_min) / (d_max - d_min), 0, 255
                ).astype(np.uint8)
                gray[valid] = 255 - norm   # invert: near → bright

        colorised = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
        colorised[~valid] = (0, 0, 0)
        return colorised

    def _draw_robot_centric_debug_overlay(
        self,
        result: Optional[Dict],
        ctrl_pts: Optional[List[Dict]],
        stamp,
        robot_mask: Optional[np.ndarray] = None,
    ) -> None:
        """Publish robot-centric debug image on the DEPTH colorised frame.

        Layer order (bottom → top):
          1. Colorised depth image (JET colourmap, near = bright)
          2. MAGENTA semi-transparent tint — robot self-mask
          3. GREEN small circles           — admitted control points
          4. YELLOW large circle           — winner control point
          5. RED circle                    — closest obstacle pixel
          6. CYAN rectangle                — surveillance window of winner CP
          7. WHITE line + mid label        — winner CP → obstacle point
          8. Info panel (top-left, dark background)
        """
        if not _HAS_CV2:
            return

        base = self._make_depth_debug_base_image()
        if base is None:
            return

        h_img, w_img = base.shape[:2]
        font     = cv2.FONT_HERSHEY_SIMPLEX
        col_text = (255, 255, 255)
        col_bg   = (20,  20,  20)
        f_thick  = 1

        overlay = base.copy()

        # ── Layer 2: robot self-mask (magenta tint, 40 % opacity) ─────
        if self._self_mask_draw and robot_mask is not None:
            alpha   = 0.40
            magenta = np.array([200, 0, 200], dtype=np.float32)
            idx     = robot_mask
            overlay[idx] = np.clip(
                overlay[idx].astype(np.float32) * (1.0 - alpha) +
                magenta * alpha,
                0, 255,
            ).astype(np.uint8)
            contours, _ = cv2.findContours(
                robot_mask.astype(np.uint8) * 255,
                cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(overlay, contours, -1, (220, 0, 220), 1,
                             cv2.LINE_AA)

        # ── Layer 3+4: control point projections ──────────────────────
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
                               (0, 230, 230), -1, cv2.LINE_AA)
                    cv2.circle(overlay, px_d, 11,
                               (255, 255, 255), 1, cv2.LINE_AA)
                else:
                    cv2.circle(overlay, px_d, 4,
                               (0, 200, 0), -1, cv2.LINE_AA)

        # ── Layer 5+6: obstacle point + surveillance window ───────────
        obs_px_d: Optional[Tuple[int, int]] = None
        if result is not None:
            p_obs = result.get("closest_point_human")
            if p_obs is not None:
                obs_px_d = self._project_point_to_depth_pixel(p_obs)

            if winner_px_d is not None:
                h_win = self._surv_half_window_px
                rx0 = max(0, winner_px_d[0] - h_win)
                ry0 = max(0, winner_px_d[1] - h_win)
                rx1 = min(w_img - 1, winner_px_d[0] + h_win)
                ry1 = min(h_img - 1, winner_px_d[1] + h_win)
                cv2.rectangle(overlay, (rx0, ry0), (rx1, ry1),
                              (180, 180, 0), 1, cv2.LINE_AA)

            if (obs_px_d is not None
                    and 0 <= obs_px_d[0] < w_img
                    and 0 <= obs_px_d[1] < h_img):
                cv2.circle(overlay, obs_px_d, 7,
                           (0, 0, 255), -1, cv2.LINE_AA)
                cv2.circle(overlay, obs_px_d, 9,
                           (255, 255, 255), 1, cv2.LINE_AA)

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
        """Main timer loop — Flacco et al. depth-space pipeline.

        Stage 1  DISTANCE PIPELINE (skipped when inputs unchanged):
          Robot-centric depth-space pipeline (Steps 1–8 of the paper).
          No landmarks required.

        Stage 2  PUBLISH HumanRobotDistance (always, even using cached result).

        Stage 3  SCALAR (Float32 for PlotJuggler).

        Stage 4  DEBUG IMAGE (rate-limited, gated on subscribers).
        """
        now    = self.get_clock().now().to_msg()
        t_wall = time.time()

        # ── Stage 1: DISTANCE PIPELINE ────────────────────────────────
        current_gen = (self._depth_gen, self._q_gen)
        if current_gen != self._last_computed_gen:
            segments = self._get_world_segments_cached()
            if segments:
                self._last_result = (
                    self._compute_min_distance_robot_centric_depth_space(
                        segments))
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

        # ── Stage 4: DEBUG IMAGE ──────────────────────────────────────
        if self._should_publish_debug_image(t_wall):
            if result is not None or self._debug_draw_no_valid:
                if result is not None:
                    ctrl_pts   = result.get("ctrl_pts")
                    robot_mask = result.get("robot_mask")
                else:
                    ctrl_pts   = (self._build_robot_control_points(
                        self._cached_segments)
                        if self._cached_segments else None)
                    robot_mask = self._self_mask_cache

                _t0_render = time.perf_counter()
                self._draw_robot_centric_debug_overlay(
                    result, ctrl_pts, now, robot_mask=robot_mask)
                _t_render = time.perf_counter() - _t0_render
                _a = self._timing_alpha
                self._t_render_avg = (
                    (1.0 - _a) * self._t_render_avg + _a * _t_render)
                self._last_debug_pub_time = t_wall

        # ── Throttled log ─────────────────────────────────────────────
        if self._tlog.due(t_wall):
            if self._cached_segments:
                n_segs_total    = len(self._cached_segments)
                n_segs_excluded = sum(
                    1 for s in self._cached_segments
                    if self._segment_is_excluded(s))
                n_cps = sum(
                    _CPS_PER_CAPSULE.get(int(s.get("capsule_idx", i)),
                                         _DEFAULT_CPS_PER_SEG)
                    for i, s in enumerate(self._cached_segments)
                    if not self._segment_is_excluded(s)
                )
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
                    f"CPs={result.get('n_cps_total','?')} "
                    f"eval={result.get('n_cps_evaluated','?')} "
                    f"no-cands={result.get('n_cps_no_cands','?')}  "
                    f"win-cands={result.get('n_valid_pts','?')}  "
                    f"win_hw={result.get('winner_half_w','?')}px "
                    f"avg_hw={result.get('avg_half_w','?')}px  "
                    f"masked_total={result.get('n_masked_total','?')}"
                )
                self._tlog.info(
                    f"  robot_pt=[{pr[0]:.3f} {pr[1]:.3f} {pr[2]:.3f}]  "
                    f"obs_pt=[{ph[0]:.3f} {ph[1]:.3f} {ph[2]:.3f}]"
                )
            else:
                self._tlog.warn(
                    f"No valid distance — {self._invalid_reason()}")


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
