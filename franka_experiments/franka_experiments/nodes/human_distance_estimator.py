#!/usr/bin/env python3
"""
HUMAN-ROBOT DISTANCE ESTIMATOR
================================
Estimates the minimum distance between detected human body parts and the
robot's kinematic chain.

Pipeline
--------
1. Subscribe to 2-D human landmarks (``HumanPose2D``) from MediaPipe.
2. Subscribe to aligned depth image to back-project landmarks to 3-D.
3. Subscribe to ``/joint_states`` to run Pinocchio FK and obtain link
   positions in the robot base frame.
4. For each valid 3-D landmark, compute the distance to every robot link
   segment and pick the global minimum.
5. Publish a ``HumanRobotDistance`` message on
   ``/human_robot/closest_distance`` — fully compatible with the
   ``human_avoidance_controller`` node in this same package.

Convention
----------
``direction`` points **FROM the human TOWARD the robot** (i.e. the
*escape* / repulsion direction), expressed in the robot base frame.
This matches the convention of ``online_avoidance_controller`` where
``direction = p_seg − p_box``.
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import CameraInfo, Image, JointState
from geometry_msgs.msg import Point, Vector3

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
# The values here are ONLY used as a fallback; the real source of truth is
# build_joint_to_joint_capsules() from franka_simulation.
_DEFAULT_CAPSULE_RADII: List[float] = [0.15, 0.12, 0.13]

# Safety zone thresholds [m]
_ZONE_CRITICAL = 0.10
_ZONE_DANGER = 0.20
_ZONE_WARNING = 0.40


def _classify_zone(distance: float) -> str:
    """Map distance [m] to safety zone string."""
    if distance <= _ZONE_CRITICAL:
        return "critical"
    if distance <= _ZONE_DANGER:
        return "danger"
    if distance <= _ZONE_WARNING:
        return "warning"
    return "safe"


def _point_to_segment_closest(
    p: np.ndarray, a: np.ndarray, b: np.ndarray,
) -> Tuple[np.ndarray, float]:
    """Closest point on segment *a*–*b* to point *p* and parameter *t*."""
    ab = b - a
    ab2 = float(ab @ ab)
    if ab2 < 1e-12:
        return a.copy(), 0.0
    t = float(np.clip((p - a) @ ab / ab2, 0.0, 1.0))
    return a + t * ab, t


# ══════════════════════════════════════════════════════════════════════════
# Node
# ══════════════════════════════════════════════════════════════════════════

class HumanDistanceEstimator(Node):
    """Estimate minimum human-robot distance from 2-D pose + depth."""

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
        self._depth_K: Optional[np.ndarray] = None  # 3×3 depth intrinsics
        self._rgb_K: Optional[np.ndarray] = None     # 3×3 RGB intrinsics
        self._depth_width: int = 0
        self._depth_height: int = 0

        # Debug overlay state
        self._debug_rgb_image: Optional[np.ndarray] = None  # BGR

        # Camera extrinsic: SE3 from camera_optical → robot_base.
        # Updated from parameter at init; can be overridden by TF later.
        self._T_base_cam: Optional[np.ndarray] = None  # 4×4 homogeneous
        self._T_cam_base: Optional[np.ndarray] = None  # 4×4 inverse
        self._init_camera_extrinsic()

        # Pre-load intrinsics from YAML (overridden by CameraInfo if available)
        self._load_intrinsics_from_yaml()

        # ── Logging ───────────────────────────────────────────────────
        self._tlog = ThrottledLogger(self.get_logger(), period_s=1.0)

        # ── Subscriptions ─────────────────────────────────────────────
        self.create_subscription(
            JointState,
            self._joint_state_topic,
            self._joint_state_cb,
            10,
        )
        self.create_subscription(
            HumanPose2D,
            self._landmarks_topic,
            self._landmarks_cb,
            10,
        )

        # Depth
        depth_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Image,
            self._depth_image_topic,
            self._depth_image_cb,
            depth_qos,
        )
        self.create_subscription(
            CameraInfo,
            self._depth_info_topic,
            self._depth_info_cb,
            10,
        )

        # RGB camera info (for back-projection of landmarks)
        self.create_subscription(
            CameraInfo,
            self._rgb_info_topic,
            self._rgb_info_cb,
            10,
        )

        # RGB image (not consumed in V1, but subscribed for completeness)
        self.create_subscription(
            Image,
            self._rgb_image_topic,
            self._rgb_image_cb,
            depth_qos,
        )

        # Debug overlay image source (annotated pose image preferred)
        self.create_subscription(
            Image,
            self._debug_image_topic,
            self._debug_image_cb,
            depth_qos,
        )

        # ── Publisher ─────────────────────────────────────────────────
        self._pub = self.create_publisher(
            HumanRobotDistance,
            self._output_topic,
            10,
        )
        self._pub_debug_image = self.create_publisher(
            Image,
            self._debug_output_topic,
            10,
        )

        # ── Timer ─────────────────────────────────────────────────────
        self.create_timer(1.0 / self._rate, self._compute_and_publish)

        self.get_logger().info(
            f"🟢 Human Distance Estimator READY  "
            f"rate={self._rate} Hz  output={self._output_topic}"
        )

    # ================================================================
    # Parameters
    # ================================================================

    def _declare_params(self) -> None:
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("output_topic",
                               "/human_robot/closest_distance")
        self.declare_parameter("joint_state_topic", "/NS_1/joint_states")
        self.declare_parameter("landmarks_topic",
                               "/human_pose/landmarks")
        self.declare_parameter("depth_image_topic",
                               "/camera/camera/depth/image_rect_raw")
        self.declare_parameter("depth_info_topic",
                               "/camera/camera/depth/camera_info")
        self.declare_parameter("rgb_image_topic",
                               "/camera/camera/color/image_raw")
        self.declare_parameter("rgb_info_topic",
                               "/camera/camera/color/camera_info")

        # Minimum visibility score to consider a landmark valid.
        self.declare_parameter("min_visibility", 0.5)
        # Min/max valid depth [m] — reject noise / saturated pixels.
        self.declare_parameter("depth_min_m", 0.15)
        self.declare_parameter("depth_max_m", 5.0)
        # Depth scale factor (RealSense: 0.001 → mm→m).
        self.declare_parameter("depth_scale", 0.001)
        # Minimum number of valid 3-D landmarks required to publish.
        self.declare_parameter("min_valid_landmarks", 3)

        # Debug distance overlay parameters.
        self.declare_parameter("num_debug_distances", 3)
        self.declare_parameter("debug_image_topic", "/human_pose/image")
        self.declare_parameter("debug_output_topic",
                               "/human_robot/distance_debug_image")

        # Camera extrinsic: rotation quaternion + translation
        # (camera_optical_frame → robot base frame).
        # Defaults from config/camera_extrinsics.yaml.
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
        self._num_debug_distances = int(p("num_debug_distances"))
        self._debug_image_topic = str(p("debug_image_topic"))
        self._debug_output_topic = str(p("debug_output_topic"))

    # ================================================================
    # Initialization helpers
    # ================================================================

    def _init_pinocchio(self) -> None:
        """Load Pinocchio model and build capsules from franka_simulation."""
        try:
            urdf_xml = generate_urdf_from_xacro()
            self._model, self._data = build_reduced_pinocchio_model_from_urdf(
                urdf_xml
            )
            self._capsules = build_joint_to_joint_capsules(
                model=self._model,
                capsule_radii=_DEFAULT_CAPSULE_RADII,
            )
            # frame_ids left empty — capsules store their own IDs
            self._frame_ids = {}
            self._pin_ok = True
            self.get_logger().info(
                f"✓ Pinocchio + {len(self._capsules)} capsules from "
                f"franka_simulation (build_joint_to_joint_capsules)"
            )
        except Exception as e:
            self.get_logger().error(f"Pinocchio/capsule init failed: {e}")

    def _init_camera_extrinsic(self) -> None:
        """Build 4×4 T_base_cam from YAML file, falling back to params."""
        loaded_from_yaml = False
        tx = ty = tz = qx = qy = qz = 0.0
        qw = 1.0

        # --- Try YAML file first ----------------------------------------
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
                    f"\U0001f4f7 Loaded camera extrinsics from {yaml_path}\n"
                    f"   t=[{tx:.4f}, {ty:.4f}, {tz:.4f}]  "
                    f"q=[{qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}]"
                )
            else:
                self.get_logger().warn(
                    f"camera_extrinsics.yaml not found at {yaml_path} "
                    f"\u2014 falling back to ROS parameters"
                )
        except Exception as e:
            self.get_logger().warn(
                f"Failed to load camera_extrinsics.yaml: {e} "
                f"\u2014 falling back to ROS parameters"
            )

        # --- Fallback: ROS parameters -----------------------------------
        if not loaded_from_yaml:
            p = lambda n: float(self.get_parameter(n).value)  # noqa: E731
            tx = p("cam_extrinsic.tx")
            ty = p("cam_extrinsic.ty")
            tz = p("cam_extrinsic.tz")
            qx = p("cam_extrinsic.qx")
            qy = p("cam_extrinsic.qy")
            qz = p("cam_extrinsic.qz")
            qw = p("cam_extrinsic.qw")
            self.get_logger().info(
                f"\U0001f4f7 Using camera extrinsics from ROS parameters  "
                f"t=[{tx:.4f}, {ty:.4f}, {tz:.4f}]  "
                f"q=[{qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}]"
            )

        # --- Quaternion → 4×4 homogeneous -------------------------------
        q_norm = float(np.sqrt(qx**2 + qy**2 + qz**2 + qw**2))
        if q_norm < 1e-9:
            self.get_logger().warn(
                "Camera extrinsic quaternion is zero \u2014 "
                "T_base_cam left as None"
            )
            self._T_base_cam = None
            return
        qx, qy, qz, qw = qx / q_norm, qy / q_norm, qz / q_norm, qw / q_norm
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx**2 + qy**2)],
        ], dtype=float)
        T = np.eye(4, dtype=float)
        T[:3, :3] = R
        T[:3, 3] = [tx, ty, tz]
        self._T_base_cam = T

        # Precompute inverse for 3-D → pixel projection
        T_inv = np.eye(4, dtype=float)
        T_inv[:3, :3] = R.T
        T_inv[:3, 3] = -R.T @ np.array([tx, ty, tz], dtype=float)
        self._T_cam_base = T_inv

    def _load_intrinsics_from_yaml(self) -> None:
        """Pre-load camera intrinsics from YAML config files.

        These serve as initial values and will be overridden by live
        CameraInfo messages once the camera driver publishes them.
        """
        try:
            pkg_share = get_package_share_directory("franka_experiments")
        except Exception:
            self.get_logger().warn(
                "Cannot resolve franka_experiments share dir \u2014 "
                "skipping YAML intrinsics loading"
            )
            return

        # \u2500\u2500 RGB intrinsics \u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500
        rgb_path = os.path.join(pkg_share, "config", "rgb_intrinsics.yaml")
        if os.path.isfile(rgb_path):
            try:
                data = load_camera_info_yaml(rgb_path)
                if data is not None:
                    k = data.get("k", [])
                    if len(k) == 9:
                        self._rgb_K = np.array(k, dtype=float).reshape(3, 3)
                        self.get_logger().info(
                            f"\U0001f4f7 RGB intrinsics from YAML: "
                            f"fx={self._rgb_K[0, 0]:.1f}  "
                            f"fy={self._rgb_K[1, 1]:.1f}  "
                            f"cx={self._rgb_K[0, 2]:.1f}  "
                            f"cy={self._rgb_K[1, 2]:.1f}"
                        )
                    else:
                        self.get_logger().warn(
                            f"RGB YAML has 'k' with {len(k)} elements (need 9)"
                        )
                else:
                    self.get_logger().warn(
                        f"No valid CameraInfo document in {rgb_path}"
                    )
            except Exception as e:
                self.get_logger().warn(f"Failed to parse {rgb_path}: {e}")
        else:
            self.get_logger().warn(
                f"rgb_intrinsics.yaml not found at {rgb_path}"
            )

        # \u2500\u2500 Depth intrinsics \u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500\u2500
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
                            f"\U0001f4f7 Depth intrinsics from YAML: "
                            f"fx={self._depth_K[0, 0]:.1f}  "
                            f"fy={self._depth_K[1, 1]:.1f}  "
                            f"cx={self._depth_K[0, 2]:.1f}  "
                            f"cy={self._depth_K[1, 2]:.1f}  "
                            f"({w}\u00d7{h})"
                        )
                    else:
                        self.get_logger().warn(
                            f"Depth YAML has 'k' with {len(k)} elements (need 9)"
                        )
                else:
                    self.get_logger().warn(
                        f"No valid CameraInfo document in {depth_path}"
                    )
            except Exception as e:
                self.get_logger().warn(
                    f"Failed to parse {depth_path}: {e}"
                )
        else:
            self.get_logger().warn(
                f"depth_intrinsics.yaml not found at {depth_path}"
            )

    # ================================================================
    # Subscription callbacks
    # ================================================================

    def _joint_state_cb(self, msg: JointState) -> None:
        """Extract 7 arm joint positions from JointState."""
        try:
            name_map = {str(n): int(i) for i, n in enumerate(msg.name)}
            q = np.array(
                [msg.position[name_map[jn]] for jn in FR3_JOINT_NAMES],
                dtype=float,
            )
            self._q = q
        except (KeyError, ValueError, IndexError):
            pass

    def _landmarks_cb(self, msg: HumanPose2D) -> None:
        self._landmarks = msg

    def _depth_image_cb(self, msg: Image) -> None:
        """Decode raw 16-bit depth image."""
        try:
            if msg.encoding in ("16UC1", "mono16"):
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                    msg.height, msg.width,
                )
            elif msg.encoding == "32FC1":
                arr = np.frombuffer(msg.data, dtype=np.float32).reshape(
                    msg.height, msg.width,
                )
            else:
                return
            self._depth_image = arr
            self._depth_height = int(msg.height)
            self._depth_width = int(msg.width)
        except Exception:
            pass

    def _depth_info_cb(self, msg: CameraInfo) -> None:
        """Extract 3×3 intrinsic matrix K from depth CameraInfo."""
        try:
            self._depth_K = np.array(msg.k, dtype=float).reshape(3, 3)
        except Exception:
            pass

    def _rgb_info_cb(self, msg: CameraInfo) -> None:
        """Extract 3×3 intrinsic matrix K from RGB CameraInfo."""
        try:
            self._rgb_K = np.array(msg.k, dtype=float).reshape(3, 3)
        except Exception:
            pass

    def _rgb_image_cb(self, msg: Image) -> None:
        """Placeholder — RGB image not consumed in V1."""
        pass

    def _debug_image_cb(self, msg: Image) -> None:
        """Store latest annotated / debug RGB image (converted to BGR)."""
        if not _HAS_CV2:
            return
        try:
            if msg.encoding in ("rgb8",):
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                self._debug_rgb_image = cv2.cvtColor(
                    arr, cv2.COLOR_RGB2BGR)
            elif msg.encoding in ("bgr8",):
                self._debug_rgb_image = np.frombuffer(
                    msg.data, dtype=np.uint8,
                ).reshape(msg.height, msg.width, 3).copy()
            else:
                # Best-effort: assume 3-channel uint8
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                self._debug_rgb_image = arr.copy()
        except Exception:
            pass

    # ================================================================
    # 3-D back-projection
    # ================================================================

    def _backproject_landmarks(self) -> List[np.ndarray]:
        """Back-project 2-D landmarks to 3-D points in robot base frame.

        Uses the depth image and camera intrinsics/extrinsics.

        Returns a list of (3,) arrays in robot base frame.
        Invalid landmarks are skipped.
        """
        lm = self._landmarks
        if lm is None:
            return []

        depth = self._depth_image
        if depth is None:
            return []

        # Use depth intrinsics for back-projection (the depth image defines
        # the coordinate system we sample from).
        K = self._depth_K
        if K is None:
            return []

        T = self._T_base_cam
        if T is None:
            return []

        # RGB→depth pixel scaling (landmarks are in RGB pixel coords;
        # the depth image may have a different resolution).
        rgb_K = self._rgb_K
        if rgb_K is not None and rgb_K[0, 0] > 0 and K[0, 0] > 0:
            scale_u = float(self._depth_width) / float(lm.image_width) \
                if lm.image_width > 0 else 1.0
            scale_v = float(self._depth_height) / float(lm.image_height) \
                if lm.image_height > 0 else 1.0
        else:
            scale_u = 1.0
            scale_v = 1.0

        fx, fy = float(K[0, 0]), float(K[1, 1])
        cx, cy = float(K[0, 2]), float(K[1, 2])
        if fx < 1e-6 or fy < 1e-6:
            return []

        R_bc = T[:3, :3]
        t_bc = T[:3, 3]

        points_base: List[np.ndarray] = []
        n_lm = len(lm.ids)
        for i in range(n_lm):
            if lm.visibility[i] < self._min_visibility:
                continue

            # Map from RGB pixel space → depth pixel space
            u_d = lm.u[i] * scale_u
            v_d = lm.v[i] * scale_v
            ui = int(round(u_d))
            vi = int(round(v_d))

            if ui < 0 or ui >= self._depth_width:
                continue
            if vi < 0 or vi >= self._depth_height:
                continue

            # Sample depth (use a small window for robustness)
            z_raw = self._sample_depth(depth, ui, vi)
            if z_raw <= 0:
                continue

            z_m = float(z_raw) * self._depth_scale
            if z_m < self._depth_min_m or z_m > self._depth_max_m:
                continue

            # Back-project to camera optical frame (Z forward)
            x_cam = (u_d - cx) * z_m / fx
            y_cam = (v_d - cy) * z_m / fy
            p_cam = np.array([x_cam, y_cam, z_m], dtype=float)

            # Transform to robot base frame
            p_base = R_bc @ p_cam + t_bc
            points_base.append(p_base)

        return points_base

    @staticmethod
    def _sample_depth(
        depth: np.ndarray, u: int, v: int, half_win: int = 2,
    ) -> float:
        """Sample depth at (u, v) using a median filter over a small window.

        Returns 0 if no valid depth is found.
        """
        h, w = depth.shape[:2]
        v0 = max(0, v - half_win)
        v1 = min(h, v + half_win + 1)
        u0 = max(0, u - half_win)
        u1 = min(w, u + half_win + 1)
        patch = depth[v0:v1, u0:u1].ravel()
        valid = patch[patch > 0]
        if len(valid) == 0:
            return 0.0
        return float(np.median(valid))

    # ================================================================
    # 3-D → pixel projection
    # ================================================================

    def _project_to_pixel(
        self, p_base: np.ndarray,
    ) -> Optional[Tuple[int, int]]:
        """Project a 3-D point (robot base frame) to an RGB pixel (u, v).

        Returns ``None`` if the point is behind the camera or the
        required calibration data is unavailable.
        """
        if self._T_cam_base is None or self._rgb_K is None:
            return None

        # Transform from base frame to camera optical frame
        p_cam = (self._T_cam_base[:3, :3] @ p_base
                 + self._T_cam_base[:3, 3])
        if p_cam[2] <= 0.01:          # behind camera
            return None

        fx = float(self._rgb_K[0, 0])
        fy = float(self._rgb_K[1, 1])
        cx = float(self._rgb_K[0, 2])
        cy = float(self._rgb_K[1, 2])

        u = int(round(fx * p_cam[0] / p_cam[2] + cx))
        v = int(round(fy * p_cam[1] / p_cam[2] + cy))
        return (u, v)

    # ================================================================
    # Distance computation
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
        """Materialise capsule segments in world frame via franka_simulation."""
        return iter_world_capsule_segments(
            capsules=self._capsules,
            frame_ids=self._frame_ids,
            data=self._data,
        )

    def _compute_min_distance(
        self, human_pts: List[np.ndarray],
    ) -> Optional[Dict]:
        """Compute minimum distance between human 3-D points and robot capsules.

        Uses the *same* capsule geometry as franka_simulation's
        ``iter_world_capsule_segments`` — no local duplication.

        Returns a dict with all fields needed for the output message, or
        ``None`` if computation is not possible.
        """
        if not self._pin_ok or self._q is None:
            return None
        if len(human_pts) < self._min_valid_landmarks:
            return None

        # FK with current joint state
        self._run_fk()

        # Build world-frame capsule segments via franka_simulation
        segments = self._get_world_segments()
        if not segments:
            return None

        best_dist = float("inf")
        best_link_name = ""
        best_p_robot = np.zeros(3)
        best_p_human = np.zeros(3)

        for seg in segments:
            p0 = np.array(seg["p0"], dtype=float).reshape(3)
            p1 = np.array(seg["p1"], dtype=float).reshape(3)
            radius = float(seg["radius"])
            parent = str(seg.get("parent", ""))

            for p_h in human_pts:
                closest_on_seg, _ = _point_to_segment_closest(p_h, p0, p1)
                raw_dist = float(np.linalg.norm(p_h - closest_on_seg))
                surface_dist = max(raw_dist - radius, 0.0)

                if surface_dist < best_dist:
                    best_dist = surface_dist
                    best_link_name = parent
                    best_p_robot = closest_on_seg.copy()
                    best_p_human = p_h.copy()

        if not np.isfinite(best_dist) or best_dist > 900.0:
            return None

        # Direction: FROM human TOWARD robot  (p_robot − p_human)
        diff = best_p_robot - best_p_human
        diff_norm = float(np.linalg.norm(diff))
        if diff_norm < 1e-9:
            direction = np.array([0.0, 0.0, 1.0])
        else:
            direction = diff / diff_norm

        # Confidence heuristic
        n_valid = len(human_pts)
        landmark_conf = float(np.clip(n_valid / 15.0, 0.2, 1.0))
        depth_conf = 1.0 if best_dist < 3.0 else float(
            np.clip(1.0 - (best_dist - 3.0) / 2.0, 0.3, 1.0)
        )
        confidence = float(np.clip(landmark_conf * depth_conf, 0.0, 1.0))

        zone = _classify_zone(best_dist)

        return {
            "robot_link_name": best_link_name,
            "distance": best_dist,
            "direction": direction,
            "closest_point_robot": best_p_robot,
            "closest_point_human": best_p_human,
            "zone": zone,
            "confidence": confidence,
        }

    # ================================================================
    # Debug overlay
    # ================================================================

    def _draw_min_distance_debug_overlay(
        self,
        result: Optional[Dict],
        stamp,
    ) -> None:
        """Always publish a debug image if a source frame is available.

        When *result* is valid, draws the global minimum distance:
        - green circle on the closest human point,
        - red circle on the closest robot capsule point,
        - yellow line between them,
        - white label with link name and distance.

        When *result* is ``None``, publishes the bare frame with a
        small ``NO VALID DISTANCE`` label.
        """
        if not _HAS_CV2:
            return
        image = self._debug_rgb_image
        if image is None:
            return

        overlay = image.copy()
        h_img, w_img = overlay.shape[:2]

        font = cv2.FONT_HERSHEY_SIMPLEX
        col_text = (255, 255, 255)   # white
        col_bg   = (40, 40, 40)      # dark background

        if result is not None:
            p_human = result.get("closest_point_human")
            p_robot = result.get("closest_point_robot")

            draw_ok = False
            if p_human is not None and p_robot is not None:
                px_human = self._project_to_pixel(p_human)
                px_robot = self._project_to_pixel(p_robot)

                if (px_human is not None and px_robot is not None
                        and 0 <= px_human[0] < w_img
                        and 0 <= px_human[1] < h_img
                        and 0 <= px_robot[0] < w_img
                        and 0 <= px_robot[1] < h_img):
                    # Colours (BGR)
                    col_human = (0, 255, 0)       # green
                    col_robot = (0, 0, 255)       # red
                    col_line  = (0, 255, 255)     # yellow

                    # Line
                    cv2.line(overlay, px_human, px_robot,
                             col_line, 2, cv2.LINE_AA)
                    # Circles
                    cv2.circle(overlay, px_human, 6,
                               col_human, -1, cv2.LINE_AA)
                    cv2.circle(overlay, px_robot, 6,
                               col_robot, -1, cv2.LINE_AA)

                    # Text label at midpoint
                    label = (f"{result['robot_link_name']}: "
                             f"{result['distance']:.3f} m")
                    mid_x = (px_human[0] + px_robot[0]) // 2
                    mid_y = (px_human[1] + px_robot[1]) // 2

                    f_scale = 0.55
                    f_thick = 1
                    (tw, th), baseline = cv2.getTextSize(
                        label, font, f_scale, f_thick)
                    cv2.rectangle(
                        overlay,
                        (mid_x - 2, mid_y - th - 4),
                        (mid_x + tw + 2, mid_y + baseline + 2),
                        col_bg, -1,
                    )
                    cv2.putText(
                        overlay, label, (mid_x, mid_y),
                        font, f_scale, col_text, f_thick, cv2.LINE_AA,
                    )
                    draw_ok = True

            # If projection failed, still publish the bare frame
            # (draw_ok remains False — nothing extra drawn)
        else:
            # No valid distance — add small info label
            label = "NO VALID DISTANCE"
            f_scale = 0.50
            f_thick = 1
            (tw, th), baseline = cv2.getTextSize(
                label, font, f_scale, f_thick)
            cv2.rectangle(
                overlay,
                (8, 8),
                (8 + tw + 6, 8 + th + baseline + 6),
                col_bg, -1,
            )
            cv2.putText(
                overlay, label, (11, 8 + th + 2),
                font, f_scale, col_text, f_thick, cv2.LINE_AA,
            )

        # Publish as sensor_msgs/Image (bgr8)
        out = Image()
        out.header.stamp = stamp
        out.header.frame_id = "camera_color_optical_frame"
        out.height = h_img
        out.width = w_img
        out.encoding = "bgr8"
        out.step = w_img * 3
        out.data = overlay.tobytes()
        self._pub_debug_image.publish(out)

    # ================================================================
    # Timer callback
    # ================================================================

    def _compute_and_publish(self) -> None:
        """Main timer loop: back-project → distance → publish."""
        now = self.get_clock().now().to_msg()
        t_wall = time.time()
        do_log = False #self._tlog.due(t_wall)

        # ── Input availability flags ──────────────────────────────
        has_q = self._q is not None
        has_lm = self._landmarks is not None
        has_depth = self._depth_image is not None
        has_depth_K = self._depth_K is not None
        has_rgb_K = self._rgb_K is not None
        has_T = self._T_base_cam is not None

        n_lm_total = len(self._landmarks.ids) if has_lm else 0

        if do_log:
            status = (
                f"q={'✓' if has_q else '✗'}  "
                f"lm={'✓' if has_lm else '✗'}({n_lm_total})  "
                f"depth={'✓' if has_depth else '✗'}  "
                f"depth_K={'✓' if has_depth_K else '✗'}  "
                f"rgb_K={'✓' if has_rgb_K else '✗'}  "
                f"T_cam={'✓' if has_T else '✗'}  "
                f"pin={'✓' if self._pin_ok else '✗'}"
            )

        # Back-project landmarks to 3-D in robot base frame
        human_pts = self._backproject_landmarks()

        # Compute minimum distance
        result = self._compute_min_distance(human_pts)

        if result is None:
            # Publish an invalid measurement so the controller knows
            msg = HumanRobotDistance()
            msg.header.stamp = now
            msg.header.frame_id = "base"
            msg.valid = False
            msg.distance = 999.0
            msg.confidence = 0.0
            msg.zone = "safe"
            self._pub.publish(msg)

            # Always publish debug image (with "NO VALID DISTANCE" label)
            self._draw_min_distance_debug_overlay(None, now)

            if do_log:
                reasons: list[str] = []
                if not self._pin_ok:
                    reasons.append("pinocchio_not_ready")
                if not has_q:
                    reasons.append("no_joint_states")
                if not has_lm:
                    reasons.append("no_landmarks")
                if not has_depth:
                    reasons.append("no_depth_image")
                if not has_depth_K:
                    reasons.append("no_depth_intrinsics")
                if not has_T:
                    reasons.append("no_camera_extrinsic")
                if has_lm and len(human_pts) < self._min_valid_landmarks:
                    reasons.append(
                        f"too_few_3d_pts({len(human_pts)}"
                        f"<{self._min_valid_landmarks})"
                    )
                self._tlog.warn(
                    f"[HUMAN-DIST] \u2717 INVALID  inputs: {status}  "
                    f"3d_pts={len(human_pts)}  "
                    f"reasons=[{', '.join(reasons)}]"
                )
            return

        # Build output message
        msg = HumanRobotDistance()
        msg.header.stamp = now
        msg.header.frame_id = "base"
        msg.robot_link_name = result["robot_link_name"]
        msg.distance = float(result["distance"])
        msg.direction = Vector3(
            x=float(result["direction"][0]),
            y=float(result["direction"][1]),
            z=float(result["direction"][2]),
        )
        msg.valid = True
        msg.closest_point_robot = Point(
            x=float(result["closest_point_robot"][0]),
            y=float(result["closest_point_robot"][1]),
            z=float(result["closest_point_robot"][2]),
        )
        msg.zone = result["zone"]
        msg.confidence = float(result["confidence"])

        self._pub.publish(msg)

        # ── Debug distance overlay (global minimum only) ──────────
        self._draw_min_distance_debug_overlay(result, now)

        # ── Throttled debug log ───────────────────────────────────
        if self._tlog.due(time.time()):
            self._tlog.info(
                f"[HUMAN-DIST-DEBUG] valid_pts={len(human_pts)} "
                f"d={result['distance']:.3f}m "
                f"link={result['robot_link_name']} "
                f"zone={result['zone']}"
            )

        if do_log:
            p_rob = result["closest_point_robot"]
            self._tlog.info(
                f"[HUMAN-DIST] \u2713 d={result['distance']:.3f}m  "
                f"link={result['robot_link_name']}  "
                f"zone={result['zone']}  "
                f"conf={result['confidence']:.2f}  "
                f"3d_pts={len(human_pts)}/{n_lm_total}  "
                f"p_rob=[{p_rob[0]:.2f},{p_rob[1]:.2f},{p_rob[2]:.2f}]  "
                f"inputs: {status}"
            )


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
