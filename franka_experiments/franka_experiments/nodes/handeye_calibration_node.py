#!/usr/bin/env python3
"""Hand–eye calibration node for Franka FR3 + RealSense + AprilTag.

Setup
-----
* **Camera** is rigidly **fixed in the world** (static mount).
* **AprilTag** is rigidly **attached to the robot end-effector**.

Solves for two unknown transforms:

* **X** = T_B_C  (``base → camera_color_optical_frame``),
* **Y** = T_E_T  (``fr3_link8 → tag36h11:0``).

Given per-sample measurements:

* ``T_B_E(i)``  — robot FK (``base → fr3_link8``  via URDF chain),
* ``T_C_T(i)``  — AprilTag detection (``camera_color_optical_frame → tag36h11:0``).

Loop-closure equation
---------------------
For every sample *i*::

    X · T_C_T(i)  =  T_B_E(i) · Y

Both sides represent the chain ``base → tag`` via two paths:
``base → camera → tag`` on the left, ``base → EE → tag`` on the right.

The error transform for sample *i* is::

    E_i  =  inv( X · T_C_T(i) ) · ( T_B_E(i) · Y )

which should be close to the identity when X and Y are correct.

Estimation
----------
The node jointly estimates X and Y (12 DOF: 6 per SE(3) transform)
by minimising the sum of squared SE(3) log-map residuals from E_i
using ``scipy.optimize.least_squares`` (Levenberg–Marquardt).

Initial guess:

* ``X_init`` = median of ``T_B_E(i) · inv(T_C_T(i))`` (assuming ``Y ≈ I``),
* ``Y_init`` = ``I``  (identity — tag approximately at EE origin).

Pipeline modes
--------------
**Manual mode** (default)::

    ros2 run franka_experiments handeye_calibration_node

    Move the robot by hand, press ENTER to capture, 'q' to solve.

**Automatic mode** (velocity-based, requires Pinocchio)::

    ros2 run franka_experiments handeye_calibration_node \\
        --ros-args -p auto_motion:=true -p num_samples:=30

    The node autonomously moves the robot through random Cartesian
    waypoints using Jacobian-based resolved-rate velocity control
    (publishing ``std_msgs/Float64MultiArray`` on ``tracking_qdot``),
    capturing calibration samples at each waypoint after stabilisation.
    Requires Pinocchio and the ``franka_experiments`` utility modules.

**Reload a saved dataset**::

    ros2 run franka_experiments handeye_calibration_node \\
        --ros-args -p dataset_file:=dataset_handeye.yaml

    If the file exists, samples are loaded and solved immediately.

The pipeline includes:

1. Dataset acquisition (manual or automatic)
2. Dataset save to ``dataset_handeye.yaml``
3. Outlier filtering (MAD on loop-closure error)
4. Joint nonlinear SE(3) estimation of X and Y (scipy, 12 DOF)
5. Held-out validation
6. Result output to ``camera_extrinsics.yaml`` (X) and
   ``ee_tag_extrinsics.yaml`` (Y)
7. Optional static TF broadcast for RViz inspection
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time as _time
from typing import Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

import tf2_ros

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
    _HAS_APRILTAG_MSGS = True
except ImportError:
    _HAS_APRILTAG_MSGS = False

try:
    import cv2
except ImportError:
    print(
        '[handeye_calibration_node] ERROR: OpenCV not found.\n'
        '  Install with:  pip install opencv-python  (or opencv-contrib-python)\n'
        '  or:  sudo apt install python3-opencv'
    )
    sys.exit(1)

# ── Optional: scipy for nonlinear estimation ─────────────────────────────
try:
    from scipy.optimize import least_squares as _scipy_least_squares  # noqa: F401
    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False

# ── Optional: Pinocchio velocity control for automatic acquisition ───────
_HAS_VELOCITY_CONTROL = False
try:
    from std_msgs.msg import Float64MultiArray
    from franka_experiments.utils.constants import NUM_JOINTS, AUTO_SENTINEL
    from franka_experiments.utils.ros import resolve_tracking_topic
    from franka_experiments.utils.math_utils import (
        min_jerk, cosine_ramp, clamp_joints, lpf, rate_limit,
    )
    from franka_experiments.utils.kinematics import (
        generate_urdf_from_xacro,
        load_pinocchio_model,
        resolve_frame_id,
        resolve_arm_joint_ids,
        compute_ee_fk,
        compute_arm_jacobian,
        dls_solve,
        JointStateManager,
    )
    from franka_experiments.utils.trajectory import (
        sample_waypoints,
        sample_single_waypoint,
    )
    from franka_experiments.utils.logging_utils import ThrottledLogger
    _HAS_VELOCITY_CONTROL = True
except ImportError:
    pass


# ═══════════════════════════════════════════════════════════════════════════
# Helper functions
# ═══════════════════════════════════════════════════════════════════════════

def transform_to_matrix(tf_stamped) -> np.ndarray:
    """Convert a ``geometry_msgs/TransformStamped`` to a 4×4 homogeneous matrix."""
    t = tf_stamped.transform.translation
    q = tf_stamped.transform.rotation  # x, y, z, w
    T = np.eye(4)
    T[:3, 3] = [t.x, t.y, t.z]
    T[:3, :3] = _quat_to_rot(q.x, q.y, q.z, q.w)
    return T


def _quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Quaternion (x, y, z, w) → 3×3 rotation matrix."""
    # normalise
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def _rot_to_quat(R: np.ndarray):
    """3×3 rotation matrix → normalised quaternion (qx, qy, qz, qw)."""
    # Shepperd's method
    tr = np.trace(R)
    if tr > 0:
        s = 0.5 / math.sqrt(tr + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return qx / n, qy / n, qz / n, qw / n


def matrix_to_translation_quaternion(T: np.ndarray):
    """4×4 homogeneous → (tx, ty, tz), (qx, qy, qz, qw)."""
    tx, ty, tz = T[0, 3], T[1, 3], T[2, 3]
    qx, qy, qz, qw = _rot_to_quat(T[:3, :3])
    return (tx, ty, tz), (qx, qy, qz, qw)


def invert_matrix(T: np.ndarray) -> np.ndarray:
    """Invert a 4×4 rigid-body transform (faster than np.linalg.inv)."""
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def rotation_error_deg(R1: np.ndarray, R2: np.ndarray) -> float:
    """Geodesic rotation error between two 3×3 rotation matrices, in degrees."""
    R_rel = R1.T @ R2
    cos_angle = (np.trace(R_rel) - 1.0) / 2.0
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    return math.degrees(math.acos(float(cos_angle)))


def _mat_to_rvec_tvec(T: np.ndarray):
    """Split 4×4 into Rodrigues rvec (3×1) and tvec (3×1) for OpenCV."""
    rvec, _ = cv2.Rodrigues(T[:3, :3])
    tvec = T[:3, 3].reshape(3, 1)
    return rvec, tvec


def _rvec_tvec_to_mat(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """Rodrigues rvec (3×1) + tvec (3×1) → 4×4 homogeneous."""
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    return T


def _axis_angle_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues formula: unit axis + angle (rad) → 3×3 rotation matrix."""
    K = np.array([
        [0.0, -axis[2], axis[1]],
        [axis[2], 0.0, -axis[0]],
        [-axis[1], axis[0], 0.0],
    ], dtype=np.float64)
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * (K @ K)


def _so3_log(R: np.ndarray) -> np.ndarray:
    """SO(3) logarithmic map: 3×3 rotation → 3-vector (angle × axis).

    Returns a 3-vector whose direction is the rotation axis and whose
    norm is the rotation angle in *radians* (range [0, π]).  Numerically
    stable for small and near-π angles.
    """
    cos_angle = (np.trace(R) - 1.0) / 2.0
    cos_angle = float(np.clip(cos_angle, -1.0, 1.0))
    angle = math.acos(cos_angle)

    if angle < 1e-8:
        # Near identity: first-order Taylor expansion of log.
        return np.array([
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ], dtype=np.float64) * 0.5

    if angle > math.pi - 1e-6:
        # Near π: use the symmetric part to extract the axis.
        S = (R + R.T) / 2.0 - np.eye(3)
        col = int(np.argmax(np.diag(R)))
        axis = S[:, col]
        axis_norm = np.linalg.norm(axis)
        if axis_norm < 1e-12:
            return np.zeros(3, dtype=np.float64)
        return axis / axis_norm * angle

    # General case: axis from skew-symmetric part.
    factor = angle / (2.0 * math.sin(angle))
    return np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ], dtype=np.float64) * factor


def _slerp_so3(R_from: np.ndarray, R_to: np.ndarray, alpha: float) -> np.ndarray:
    """Geodesic SLERP between two SO(3) rotation matrices.

    alpha=0 → R_from,  alpha=1 → R_to.
    Uses the SO(3) exponential map:  R_from ⊕ alpha · log(R_from^T · R_to).
    Relies on ``_so3_log`` and ``_axis_angle_rotation`` defined above.
    """
    alpha = float(np.clip(alpha, 0.0, 1.0))
    w = _so3_log(R_from.T @ R_to)          # rotation vector (angle × axis)
    angle = float(np.linalg.norm(w))
    if angle < 1e-8:
        return R_from.copy()
    axis = w / angle
    return R_from @ _axis_angle_rotation(axis, alpha * angle)


# ═══════════════════════════════════════════════════════════════════════════
# Calibration node
# ═══════════════════════════════════════════════════════════════════════════

# Minimum Euclidean distance (metres) between the tool position of a new
# sample and ALL previously captured samples.  Samples closer than this
# threshold are rejected to ensure spatial diversity.
_MIN_SAMPLE_POSITION_DISTANCE_M = 0.02

# ── Acceptability thresholds for calibration verdict ─────────────────────
ACCEPTABLE_TEST_TRANSLATION_MM = 10.0   # max mean test translation error (mm)
ACCEPTABLE_TEST_ROTATION_DEG = 5.0      # max mean test rotation error (deg)


class HandEyeCalibrationNode(Node):
    """Interactive hand-eye calibration node (keyboard-driven or automatic).

    Camera is fixed in the world.  AprilTag is on the end-effector.
    Jointly estimates X = T_B_C and Y = T_E_T from the loop-closure
    equation  ``X · T_C_T(i) = T_B_E(i) · Y``.
    """

    def __init__(self) -> None:
        super().__init__('handeye_calibration_node')

        # ── Declare parameters with defaults ──────────────────────────
        self.declare_parameter('robot_base_frame', 'fr3_link0')
        self.declare_parameter('robot_tool_frame', 'fr3_link8')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('tag_frame', 'tag36h11:0')
        self.declare_parameter('output_parent', 'base')
        self.declare_parameter('output_child', 'camera_color_optical_frame')
        self.declare_parameter('tf_timeout_s', 2.0)
        self.declare_parameter('visualize', False)
        self.declare_parameter('yaml_style', 'dict')

        # ── Pipeline parameters ───────────────────────────────────────
        self.declare_parameter('auto_motion', True)
        self.declare_parameter('dataset_file', 'dataset_handeye.yaml')
        self.declare_parameter('num_samples', 30)
        self.declare_parameter('settle_time_s', 3.0)
        self.declare_parameter('tag_stability_samples', 2)
        self.declare_parameter('tag_stability_max_spread_m', 0.01)
        self.declare_parameter('tag_stability_max_spread_deg', 3.0)
        self.declare_parameter('validation_ratio', 0.2)
        self.declare_parameter('rotation_weight', 1.0)
        self.declare_parameter('translation_weight', 1.0)
        self.declare_parameter('refine', True)
        self.declare_parameter('velocity_scaling', 0.1)
        self.declare_parameter('workspace_center_x', 0.4)
        self.declare_parameter('workspace_center_y', 0.0)
        self.declare_parameter('workspace_center_z', 0.35)
        self.declare_parameter('workspace_extent', 0.15)
        self.declare_parameter('max_tilt_deg', 35.0)

        # ── Auto-motion velocity control parameters ───────────────────
        self.declare_parameter('tracking_topic', '')
        self.declare_parameter('joint_state_topic', '__auto__')
        self.declare_parameter('ee_frame', 'fr3_link8')
        self.declare_parameter('rate_hz', 200.0)
        self.declare_parameter('workspace_bounds',
                               [0.30, 0.55, -0.15, 0.15, 0.10, 0.50])
        self.declare_parameter('num_waypoints', 80)
        self.declare_parameter('min_distance', 0.05)
        self.declare_parameter('segment_duration_s', 5.0)
        self.declare_parameter('hold_time_s', 3.0)
        self.declare_parameter('kp_cart', 2.0)
        self.declare_parameter('damping', 0.02)
        self.declare_parameter('qdot_max', 0.15)
        self.declare_parameter('lpf_alpha', 0.8)
        self.declare_parameter('qdot_acc_max', 5.0)       # rad/s² — joint accel limit
        self.declare_parameter('orient_slerp_dur_s', 0.5) # s — R_des transition time
        self.declare_parameter('warmup_s', 2.0)
        self.declare_parameter('ramp_s', 2.0)
        self.declare_parameter('capture_pos_tol_m', 0.02)
        self.declare_parameter('min_safe_z_m', 0.30)
        self.declare_parameter('orientation_excitation_enabled', True)
        self.declare_parameter('orientation_kp', 1.0)
        self.declare_parameter('capture_rot_tol_deg', 5.0)
        self.declare_parameter('orientation_roll_deg_max', 8.0)
        self.declare_parameter('orientation_pitch_deg_max', 8.0)
        self.declare_parameter('orientation_yaw_deg_max', 12.0)
        self.declare_parameter('orientation_confidence_threshold', 0.5)

        self._base_frame: str = self.get_parameter('robot_base_frame').value
        self._tool_frame: str = self.get_parameter('robot_tool_frame').value
        self._cam_frame: str = self.get_parameter('camera_frame').value
        self._tag_frame: str = self.get_parameter('tag_frame').value
        self._out_parent: str = self.get_parameter('output_parent').value
        self._out_child: str = self.get_parameter('output_child').value
        self._tf_timeout = Duration(
            seconds=self.get_parameter('tf_timeout_s').value)
        self._visualize: bool = self.get_parameter('visualize').value
        self._yaml_style: str = self.get_parameter('yaml_style').value

        self._auto_motion: bool = self.get_parameter('auto_motion').value
        self._dataset_file: str = self.get_parameter('dataset_file').value
        self._num_samples: int = self.get_parameter('num_samples').value
        self._settle_time_s: float = self.get_parameter('settle_time_s').value
        self._tag_stab_n: int = self.get_parameter(
            'tag_stability_samples').value
        self._tag_stab_m: float = self.get_parameter(
            'tag_stability_max_spread_m').value
        self._tag_stab_deg: float = self.get_parameter(
            'tag_stability_max_spread_deg').value
        self._validation_ratio: float = self.get_parameter(
            'validation_ratio').value
        self._rotation_weight: float = self.get_parameter(
            'rotation_weight').value
        self._translation_weight: float = self.get_parameter(
            'translation_weight').value
        self._refine: bool = self.get_parameter('refine').value
        self._velocity_scaling: float = self.get_parameter(
            'velocity_scaling').value
        self._ws_cx: float = self.get_parameter('workspace_center_x').value
        self._ws_cy: float = self.get_parameter('workspace_center_y').value
        self._ws_cz: float = self.get_parameter('workspace_center_z').value
        self._ws_ext: float = self.get_parameter('workspace_extent').value
        self._max_tilt_deg: float = self.get_parameter('max_tilt_deg').value
        self._capture_pos_tol_m: float = self.get_parameter(
            'capture_pos_tol_m').value
        self._min_safe_z_m: float = self.get_parameter(
            'min_safe_z_m').value
        self._orientation_excitation_enabled: bool = self.get_parameter(
            'orientation_excitation_enabled').value
        self._orientation_kp: float = self.get_parameter(
            'orientation_kp').value
        self._capture_rot_tol_deg: float = self.get_parameter(
            'capture_rot_tol_deg').value
        self._orient_roll_max_deg: float = self.get_parameter(
            'orientation_roll_deg_max').value
        self._orient_pitch_max_deg: float = self.get_parameter(
            'orientation_pitch_deg_max').value
        self._orient_yaw_max_deg: float = self.get_parameter(
            'orientation_yaw_deg_max').value
        self._orient_confidence_threshold: float = self.get_parameter(
            'orientation_confidence_threshold').value
        # Adaptive perturbation amplitude scale (reduced on low confidence)
        self._orient_perturbation_scale: float = 1.0

        # ── TF listener ───────────────────────────────────────────────
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Static TF broadcaster (for --visualize) ──────────────────
        self._tf_broadcaster: Optional[tf2_ros.StaticTransformBroadcaster] = None
        if self._visualize:
            self._tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # ── AprilTag /detections subscriber (simple tag detection) ────
        self._tag_last_seen_time: float = 0.0
        self._TAG_RECENCY_S: float = 1.0  # max age for a detection (s)
        if _HAS_APRILTAG_MSGS:
            self._sub_detections = self.create_subscription(
                AprilTagDetectionArray, '/detections',
                self._detections_cb, 10)
            self.get_logger().info(
                'Subscribed to /detections for simple tag detection.')
        else:
            self.get_logger().warn(
                'apriltag_msgs not found — /detections subscriber '
                'disabled.  Install: sudo apt install '
                'ros-humble-apriltag-msgs')

        # ── Sample storage ────────────────────────────────────────────
        self._samples_base_tool: list[np.ndarray] = []   # T_B_E(i)
        self._samples_cam_tag: list[np.ndarray] = []     # T_C_T(i)

        # ── Lifecycle ─────────────────────────────────────────────────
        self.done = False
        self._input_thread: Optional[threading.Thread] = None

        self.get_logger().info('=' * 64)
        self.get_logger().info('Hand-eye calibration node ready.')
        self.get_logger().info('  Geometry  : camera FIXED in world, '
                               'tag ON end-effector')
        self.get_logger().info(f'  base frame  : {self._base_frame}')
        self.get_logger().info(f'  tool frame  : {self._tool_frame}')
        self.get_logger().info(f'  camera frame: {self._cam_frame}')
        self.get_logger().info(f'  tag frame   : {self._tag_frame}')
        self.get_logger().info(f'  visualize   : {self._visualize}')
        self.get_logger().info(f'  auto_motion : {self._auto_motion}')
        self.get_logger().info(f'  refine      : {self._refine}')
        self.get_logger().info(f'  rot_weight  : {self._rotation_weight}')
        self.get_logger().info(f'  trans_weight: {self._translation_weight}')
        self.get_logger().info(f'  num_samples : {self._num_samples}')
        self.get_logger().info(f'  capture_tol : {self._capture_pos_tol_m * 1000:.1f} mm')
        self.get_logger().info(f'  min_safe_z  : {self._min_safe_z_m:.3f} m')
        self.get_logger().info(f'  dataset_file: {self._dataset_file}')
        self.get_logger().info(f'  output       : X → {self._out_parent} → '
                               f'{self._out_child}  (camera_extrinsics.yaml)')
        self.get_logger().info(f'               : Y → {self._tool_frame} → '
                               f'{self._tag_frame}  (ee_tag_extrinsics.yaml)')

        # ── Startup TF connectivity check ─────────────────────────────
        self._startup_tf_check()

        # ── Try loading an existing dataset ───────────────────────────
        dataset_loaded = False
        if not self._auto_motion:
            ds_path = self._resolve_dataset_path()
            if ds_path and os.path.isfile(ds_path):
                if self._load_dataset(ds_path):
                    self.get_logger().info(
                        f'Loaded {len(self._samples_base_tool)} samples '
                        f'from {ds_path}')
                    dataset_loaded = True

        # ── Start acquisition ─────────────────────────────────────────
        if self._auto_motion:
            if not _HAS_VELOCITY_CONTROL:
                self.get_logger().error(
                    'auto_motion=True but Pinocchio / franka_experiments '
                    'velocity-control utilities are not available.  '
                    'Install pinocchio (pip install pin) and ensure '
                    'franka_experiments is built, or set auto_motion:=false.')
                self.done = True
                return
            self._init_velocity_auto_motion()
        elif dataset_loaded and len(self._samples_base_tool) >= 3:
            self.get_logger().info(
                'Dataset loaded.  Type "q" + ENTER to solve, or ENTER '
                'to capture more samples.')
            self._input_thread = threading.Thread(
                target=self._keyboard_loop, daemon=True)
            self._input_thread.start()
        else:
            self.get_logger().info(
                'Press ENTER to capture a sample.  '
                'Type "q" + ENTER to solve.')
            self._input_thread = threading.Thread(
                target=self._keyboard_loop, daemon=True)
            self._input_thread.start()

        self.get_logger().info('=' * 64)

    # ══════════════════════════════════════════════════════════════════
    # Startup TF connectivity check
    # ══════════════════════════════════════════════════════════════════

    def _startup_tf_check(self) -> None:
        """Probe required TF chains at startup and log diagnostics.

        Waits a few seconds for the TF buffer to fill, then attempts:

        1. ``robot_base_frame → robot_tool_frame``  (robot FK),
        2. ``camera_frame → tag_frame``             (AprilTag detection).

        Failures are logged as warnings with troubleshooting hints.
        The node does NOT abort — a failed tag lookup is expected when
        the tag is not yet visible at startup.
        """
        self.get_logger().info('')
        self.get_logger().info('── TF connectivity check ──')

        # Give the TF buffer time to receive initial transforms.
        _time.sleep(2.0)

        probe_timeout = Duration(seconds=3.0)

        # 1) Robot FK: base → tool
        try:
            tf_bt = self._tf_buffer.lookup_transform(
                self._base_frame, self._tool_frame,
                Time(), probe_timeout)
            t = tf_bt.transform.translation
            self.get_logger().info(
                f'  ✓ {self._base_frame} → {self._tool_frame}  '
                f'[{t.x:.3f}, {t.y:.3f}, {t.z:.3f}]')
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'  ✗ {self._base_frame} → {self._tool_frame}  FAILED: {e}')
            self.get_logger().warn(
                '    Troubleshooting:')
            self.get_logger().warn(
                '    • Is the robot driver running?  '
                '(ros2 launch franka_bringup …)')
            self.get_logger().warn(
                '    • Does "ros2 topic echo /tf --once" show '
                f'"{self._base_frame}" and "{self._tool_frame}"?')
            self.get_logger().warn(
                '    • If your URDF root is "base" and the robot '
                'publishes from "fr3_link0", make sure the '
                '"base → fr3_link0" static TF exists '
                '(robot_state_publisher).')

        # 2) Camera → Tag (AprilTag detection)
        try:
            tf_ct = self._tf_buffer.lookup_transform(
                self._cam_frame, self._tag_frame,
                Time(), probe_timeout)
            t = tf_ct.transform.translation
            self.get_logger().info(
                f'  ✓ {self._cam_frame} → {self._tag_frame}  '
                f'[{t.x:.3f}, {t.y:.3f}, {t.z:.3f}]')
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'  ✗ {self._cam_frame} → {self._tag_frame}  '
                f'FAILED: {e}')
            self.get_logger().warn(
                '    Troubleshooting:')
            self.get_logger().warn(
                '    • Is the RealSense node running?  '
                '(ros2 launch realsense2_camera rs_launch.py)')
            self.get_logger().warn(
                '    • Is apriltag_ros running and detecting tags?  '
                '(ros2 launch apriltag_ros …)')
            self.get_logger().warn(
                '    • Is the tag visible to the camera right now?  '
                'This is OK if you will move the robot first.')

        # 3) Sanity: warn if output_child != camera_frame
        if self._out_child != self._cam_frame:
            self.get_logger().warn(
                f'  ⚠ output_child ("{self._out_child}") ≠ '
                f'camera_frame ("{self._cam_frame}").  '
                f'The saved X = T_base_cam uses camera_frame for '
                f'the maths, so output_child should usually match '
                f'camera_frame for consistency.')

        self.get_logger().info('── end TF check ──')
        self.get_logger().info('')

    # ══════════════════════════════════════════════════════════════════
    # AprilTag /detections subscriber
    # ══════════════════════════════════════════════════════════════════

    def _detections_cb(self, msg) -> None:
        """Callback for ``/detections`` (AprilTagDetectionArray).

        Updates ``_tag_last_seen_time`` when the target tag
        (family containing ``36h11``, id ``0``) is present.
        """
        for det in msg.detections:
            if '36h11' in det.family and det.id == 0:
                self._tag_last_seen_time = _time.monotonic()
                return

    def _is_tag_detected_recently(self) -> bool:
        """Return ``True`` if the target tag was detected recently.

        Conditions:
        1. A ``/detections`` message reported family containing
           ``36h11`` with id ``0``.
        2. That detection is no older than ``_TAG_RECENCY_S`` (1.0 s).
        """
        if self._tag_last_seen_time == 0.0:
            return False
        age = _time.monotonic() - self._tag_last_seen_time
        return age <= self._TAG_RECENCY_S

    # ══════════════════════════════════════════════════════════════════
    # Pose generation (for automatic acquisition)
    # ══════════════════════════════════════════════════════════════════

    def generate_calibration_poses(
        self, num_poses: int = 30,
    ) -> list[np.ndarray]:
        """Generate diverse SE(3) end-effector poses in base frame.

        Covers a configurable workspace volume with randomised tilts
        around a downward-pointing base orientation.  Uses farthest-point
        sampling to maximise spatial + orientational diversity.

        Parameters
        ----------
        num_poses : int
            Number of target poses to return.

        Returns
        -------
        list[np.ndarray]
            List of 4×4 homogeneous transforms (base frame).
        """
        rng = np.random.RandomState(42)  # reproducible

        cx, cy, cz = self._ws_cx, self._ws_cy, self._ws_cz
        ext = self._ws_ext
        max_tilt_rad = math.radians(self._max_tilt_deg)

        x_lo, x_hi = cx - ext, cx + ext
        y_lo, y_hi = cy - ext, cy + ext
        z_lo, z_hi = max(0.08, cz - ext), cz + ext

        # Base orientation: gripper pointing downward (EE z → -z_base).
        R_down = np.array(
            [[1.0, 0.0, 0.0],
             [0.0, -1.0, 0.0],
             [0.0, 0.0, -1.0]], dtype=np.float64)

        # Generate many candidates, then subsample.
        n_candidates = max(num_poses * 4, 120)
        candidates: list[np.ndarray] = []

        for _ in range(n_candidates):
            px = rng.uniform(x_lo, x_hi)
            py = rng.uniform(y_lo, y_hi)
            pz = rng.uniform(z_lo, z_hi)

            # Random tilt axis + angle
            axis = rng.randn(3)
            anorm = np.linalg.norm(axis)
            if anorm < 1e-8:
                axis = np.array([0.0, 0.0, 1.0])
            else:
                axis = axis / anorm
            angle = rng.uniform(0.0, max_tilt_rad)

            R_tilt = _axis_angle_rotation(axis, angle)
            R = R_tilt @ R_down

            T = np.eye(4, dtype=np.float64)
            T[:3, :3] = R
            T[:3, 3] = [px, py, pz]
            candidates.append(T)

        # ── Farthest-point sampling for maximum diversity ─────────────
        selected: list[int] = [0]
        remaining = set(range(1, len(candidates)))

        while len(selected) < min(num_poses, len(candidates)):
            best_idx = -1
            best_dist = -1.0
            for r in remaining:
                min_d = float('inf')
                for s in selected:
                    dt = float(np.linalg.norm(
                        candidates[r][:3, 3] - candidates[s][:3, 3]))
                    dr = math.radians(rotation_error_deg(
                        candidates[r][:3, :3], candidates[s][:3, :3]))
                    d = dt + 0.3 * dr  # blend position + orientation
                    if d < min_d:
                        min_d = d
                if min_d > best_dist:
                    best_dist = min_d
                    best_idx = r
            if best_idx < 0:
                break
            selected.append(best_idx)
            remaining.discard(best_idx)

        poses = [candidates[i] for i in selected]
        self.get_logger().info(
            f'Generated {len(poses)} calibration poses '
            f'(from {n_candidates} candidates, farthest-point sampling)')
        return poses

    # ══════════════════════════════════════════════════════════════════
    # Velocity-based automatic acquisition
    # ══════════════════════════════════════════════════════════════════

    def _init_velocity_auto_motion(self) -> None:
        """Set up Pinocchio, velocity publisher, and timer for auto-motion.

        Replaces the old action-client-based automatic acquisition with
        an internal Jacobian-based resolved-rate velocity controller.
        The robot follows random Cartesian waypoints, pausing at each
        to capture a calibration sample once the tag detection is stable.
        """
        # ── Read velocity-control parameters ──────────────────────────
        default_topic = resolve_tracking_topic()
        tracking_raw: str = self.get_parameter('tracking_topic').value
        self._auto_tracking_topic = (
            tracking_raw if tracking_raw else default_topic)

        js_topic: str = self.get_parameter('joint_state_topic').value
        ee_frame_name: str = self.get_parameter('ee_frame').value
        self._auto_rate_hz: float = self.get_parameter('rate_hz').value
        self._auto_warmup_s: float = self.get_parameter('warmup_s').value
        self._auto_ramp_s: float = self.get_parameter('ramp_s').value

        self._auto_workspace_bounds: list[float] = list(
            self.get_parameter('workspace_bounds').value)
        self._auto_num_waypoints: int = self.get_parameter(
            'num_waypoints').value
        self._auto_min_dist: float = self.get_parameter(
            'min_distance').value
        self._auto_seg_dur: float = self.get_parameter(
            'segment_duration_s').value
        self._auto_hold_time_s: float = self.get_parameter(
            'hold_time_s').value

        self._auto_kp: float = self.get_parameter('kp_cart').value
        self._auto_damping: float = self.get_parameter('damping').value
        self._auto_qdot_max: float = self.get_parameter('qdot_max').value
        self._auto_lpf_alpha: float = self.get_parameter('lpf_alpha').value
        self._auto_qdot_acc_max: float = self.get_parameter('qdot_acc_max').value
        self._auto_orient_slerp_dur: float = self.get_parameter(
            'orient_slerp_dur_s').value

        # ── Validate workspace bounds ─────────────────────────────────
        wb = self._auto_workspace_bounds
        if len(wb) != 6:
            self.get_logger().error(
                'workspace_bounds must have 6 elements '
                '[xmin, xmax, ymin, ymax, zmin, zmax]')
            self.done = True
            return
        for i in range(3):
            if wb[2 * i] >= wb[2 * i + 1]:
                self.get_logger().error(
                    f'workspace_bounds: min >= max on axis {i}')
                self.done = True
                return

        # ── Enforce min_safe_z_m on workspace z bounds ────────────────
        if self._min_safe_z_m > wb[5]:  # z_max
            self.get_logger().error(
                f'min_safe_z_m ({self._min_safe_z_m:.3f}) is above '
                f'workspace z_max ({wb[5]:.3f}) — no valid z range. '
                f'Increase workspace z_max or lower min_safe_z_m.')
            self.done = True
            return
        if wb[4] < self._min_safe_z_m:
            self.get_logger().warn(
                f'Clamping workspace z_min from {wb[4]:.3f} to '
                f'min_safe_z_m={self._min_safe_z_m:.3f}')
            wb[4] = self._min_safe_z_m
            self._auto_workspace_bounds = wb
        self.get_logger().info(
            f'Minimum safe Z enforced: {self._min_safe_z_m:.2f} m '
            f'(workspace z range: [{wb[4]:.3f}, {wb[5]:.3f}])')

        # ── Load Pinocchio model ──────────────────────────────────────
        self.get_logger().info('Generating URDF via xacro …')
        try:
            urdf_xml = generate_urdf_from_xacro()
        except Exception as exc:
            self.get_logger().error(f'Failed to generate URDF: {exc}')
            self.done = True
            return

        self.get_logger().info('Building Pinocchio model …')
        self._auto_pin_model, self._auto_pin_data = \
            load_pinocchio_model(urdf_xml)

        try:
            self._auto_ee_frame_id = resolve_frame_id(
                self._auto_pin_model, ee_frame_name)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            self.done = True
            return

        try:
            self._auto_pin_joint_ids = resolve_arm_joint_ids(
                self._auto_pin_model)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            self.done = True
            return

        self.get_logger().info(
            f'EE frame: "{ee_frame_name}" '
            f'(id={self._auto_ee_frame_id})')

        # ── Joint state manager ───────────────────────────────────────
        self._auto_js_mgr = JointStateManager(
            self, self._auto_pin_model, self._auto_pin_joint_ids,
            topic_param=js_topic)

        # ── Generate random waypoints ─────────────────────────────────
        self._auto_rng = np.random.default_rng()
        self._auto_waypoints = sample_waypoints(
            self._auto_num_waypoints, self._auto_workspace_bounds,
            self._auto_min_dist, self._auto_rng)

        # ── State machine variables ───────────────────────────────────
        self._auto_wp_idx: int = 0
        self._auto_phase: str = 'move'   # move | hold | settle
        self._auto_seg_start: float = 0.0
        self._auto_seg_p_start: Optional[np.ndarray] = None
        self._auto_hold_start: float = 0.0
        self._auto_settle_start: float = 0.0
        self._auto_traj_started: bool = False
        self._auto_stopping: bool = False
        self._auto_stop_end: float = 0.0

        # Reference EE orientation (captured on first active tick)
        self._auto_R_ref: Optional[np.ndarray] = None
        # Per-waypoint desired orientation (R_ref with perturbation)
        self._auto_R_des: Optional[np.ndarray] = None
        # Reference quaternion (x,y,z,w) stored alongside R_ref
        self._auto_q_ref: Optional[tuple] = None

        # Statistics
        self._auto_n_ok: int = 0
        self._auto_n_fail_tag: int = 0
        self._auto_n_fail_capture: int = 0

        # Low-pass filter state
        self._auto_qdot_prev = np.zeros(NUM_JOINTS)
        # Separate LPF for angular velocity command (smooths orientation transitions)
        self._auto_v_ang_filt = np.zeros(3)
        # Rate-limiter state (separate from LPF to allow independent tuning)
        self._auto_qdot_rl_prev = np.zeros(NUM_JOINTS)
        # SLERP state for smooth R_des transitions between waypoints
        self._auto_R_des_from: Optional[np.ndarray] = None  # start of SLERP
        self._auto_R_des_slerp_start: float = 0.0           # tr when SLERP began

        # ── Velocity publisher ────────────────────────────────────────
        self._pub_vel = self.create_publisher(
            Float64MultiArray, self._auto_tracking_topic, 10)
        self._auto_zero_msg = Float64MultiArray()
        self._auto_zero_msg.data = [0.0] * NUM_JOINTS

        # ── Timer ─────────────────────────────────────────────────────
        period = 1.0 / self._auto_rate_hz
        self._auto_timer = self.create_timer(period, self._auto_timer_cb)
        self._auto_t0 = self.get_clock().now()

        # ── Throttled logger ──────────────────────────────────────────
        self._auto_tlog = ThrottledLogger(self.get_logger())

        # ── Startup log ───────────────────────────────────────────────
        self.get_logger().info(
            f'Velocity-based auto-motion initialised\n'
            f'  tracking topic : {self._auto_tracking_topic}\n'
            f'  ee frame       : {ee_frame_name}\n'
            f'  rate           : {self._auto_rate_hz} Hz\n'
            f'  warmup / ramp  : {self._auto_warmup_s} / '
            f'{self._auto_ramp_s} s\n'
            f'  workspace      : '
            f'x=[{wb[0]:.2f}, {wb[1]:.2f}]  '
            f'y=[{wb[2]:.2f}, {wb[3]:.2f}]  '
            f'z=[{wb[4]:.2f}, {wb[5]:.2f}]\n'
            f'  waypoints      : {self._auto_num_waypoints}\n'
            f'  target samples : {self._num_samples}\n'
            f'  segment_dur    : {self._auto_seg_dur} s\n'
            f'  hold_time      : {self._auto_hold_time_s} s\n'
            f'  Kp             : {self._auto_kp}\n'
            f'  damping        : {self._auto_damping}\n'
            f'  qdot_max       : {self._auto_qdot_max} rad/s\n'
            f'  lpf_alpha      : {self._auto_lpf_alpha}\n'
            f'  capture_tol    : {self._capture_pos_tol_m * 1000:.1f} mm\n'
            f'  capture_rot_tol: {self._capture_rot_tol_deg:.1f} deg\n'
            f'  orient_excite  : {self._orientation_excitation_enabled}\n'
            f'  orient_roll_max: {self._orient_roll_max_deg:.1f} deg\n'
            f'  orient_pitch_max: {self._orient_pitch_max_deg:.1f} deg\n'
            f'  orient_yaw_max : {self._orient_yaw_max_deg:.1f} deg\n'
            f'  orientation_kp : {self._orientation_kp}\n'
            f'  min_safe_z     : {self._min_safe_z_m:.3f} m')

        for i, wp in enumerate(self._auto_waypoints):
            self.get_logger().info(
                f'  waypoint[{i:2d}] : '
                f'[{wp[0]:.4f}, {wp[1]:.4f}, {wp[2]:.4f}]')

    # ------------------------------------------------------------------
    # Auto-motion timer callback
    # ------------------------------------------------------------------

    def _auto_timer_cb(self) -> None:  # noqa: C901
        """Timer callback: velocity-based waypoint tracking + sample capture.

        State machine: warmup → move → hold → settle → [capture] → next.
        Publishes 7-DOF joint velocities via resolved-rate Cartesian control.
        """
        # ── Clean shutdown: publish zeros, then solve ────────────────
        if self._auto_stopping:
            try:
                self._pub_vel.publish(self._auto_zero_msg)
            except Exception:
                pass
            if _time.monotonic() >= self._auto_stop_end:
                self.get_logger().info('Auto-motion stop complete.')
                self._auto_timer.cancel()
                self._auto_js_mgr.cancel_discovery()
                self._print_auto_summary()
                self._solve_and_save()
                self.done = True
            return

        t = (self.get_clock().now() - self._auto_t0).nanoseconds * 1e-9

        # ── Warmup: publish zeros ─────────────────────────────────────
        if t < self._auto_warmup_s:
            self._pub_vel.publish(self._auto_zero_msg)
            return

        tr = t - self._auto_warmup_s
        envelope = cosine_ramp(tr, self._auto_ramp_s)

        # ── Need joint state ──────────────────────────────────────────
        js = self._auto_js_mgr
        if js.q is None or js.q_full is None:
            self._pub_vel.publish(self._auto_zero_msg)
            self._auto_qdot_prev = np.zeros(NUM_JOINTS)
            self._auto_v_ang_filt = np.zeros(3)
            return

        js_age = (self.get_clock().now() - js.stamp).nanoseconds * 1e-9
        if js_age > 0.1:
            self._pub_vel.publish(self._auto_zero_msg)
            self._auto_qdot_prev = np.zeros(NUM_JOINTS)
            self._auto_v_ang_filt = np.zeros(3)
            return

        # ── Forward kinematics + Jacobian ─────────────────────────────
        q_full = js.q_full.copy()
        oMee = compute_ee_fk(
            self._auto_pin_model, self._auto_pin_data,
            q_full, self._auto_ee_frame_id)
        p_ee = np.array(oMee.translation).copy()
        R_ee = np.array(oMee.rotation).copy()  # 3×3 current orientation

        # Full 6×7 geometric Jacobian (translation + rotation rows).
        J_arm = compute_arm_jacobian(
            self._auto_pin_model, self._auto_pin_data,
            q_full, self._auto_ee_frame_id, self._auto_pin_joint_ids)

        # ── First active tick: start trajectory ───────────────────────
        if not self._auto_traj_started:
            self._auto_traj_started = True
            self._auto_seg_start = tr
            self._auto_seg_p_start = p_ee.copy()
            self._auto_phase = 'move'

            # Store the initial EE orientation as q_ref / R_ref.
            if self._orientation_excitation_enabled and self._auto_R_ref is None:
                self._auto_R_ref = R_ee.copy()
                self._auto_q_ref = _rot_to_quat(R_ee)
                self.get_logger().info(
                    f'Reference EE orientation stored as q_ref = '
                    f'({self._auto_q_ref[0]:.4f}, {self._auto_q_ref[1]:.4f}, '
                    f'{self._auto_q_ref[2]:.4f}, {self._auto_q_ref[3]:.4f})')
                # Generate first perturbed orientation for waypoint 0
                self._auto_R_des = self._generate_orientation_perturbation()
                self.get_logger().info(
                    'Orientation excitation ENABLED — small perturbations '
                    f'around q_ref (roll±{self._orient_roll_max_deg}°, '
                    f'pitch±{self._orient_pitch_max_deg}°, '
                    f'yaw±{self._orient_yaw_max_deg}°).')

            wp = self._auto_waypoints[0]
            self.get_logger().info(
                f'Auto-motion started from '
                f'[{p_ee[0]:.4f}, {p_ee[1]:.4f}, {p_ee[2]:.4f}]\n'
                f'  Moving to waypoint [0/'
                f'{len(self._auto_waypoints) - 1}]  '
                f'target=[{wp[0]:.4f}, {wp[1]:.4f}, {wp[2]:.4f}]')

        # ── Compute interpolated R_des via SO(3) SLERP ────────────────
        # Smooths the orientation target change at each waypoint transition,
        # preventing the instantaneous e_rot jump that causes qdot spikes.
        if (self._orientation_excitation_enabled
                and self._auto_R_des is not None
                and self._auto_R_des_from is not None
                and self._auto_orient_slerp_dur > 0.0):
            slerp_alpha = float(np.clip(
                (tr - self._auto_R_des_slerp_start) / self._auto_orient_slerp_dur,
                0.0, 1.0))
            _R_des_now = _slerp_so3(
                self._auto_R_des_from, self._auto_R_des, slerp_alpha)
            if slerp_alpha >= 1.0:
                self._auto_R_des_from = None   # SLERP complete
        else:
            _R_des_now = self._auto_R_des      # no active SLERP

        # ── Check if enough samples collected ─────────────────────────
        if len(self._samples_base_tool) >= self._num_samples:
            self.get_logger().info(
                f'Collected {self._num_samples} target samples — stopping.')
            self._request_auto_stop()
            return

        # ── Waypoints exhausted ───────────────────────────────────────
        if self._auto_wp_idx >= len(self._auto_waypoints):
            self._request_auto_stop()
            return

        # ── State: MOVE ───────────────────────────────────────────────
        p_target = self._auto_waypoints[self._auto_wp_idx]
        p_d = p_target.copy()
        v_d = np.zeros(3)

        if self._auto_phase == 'move':
            seg_elapsed = tr - self._auto_seg_start
            tau = (seg_elapsed / self._auto_seg_dur
                   if self._auto_seg_dur > 0 else 1.0)

            if tau >= 1.0:
                tau = 1.0
                self._auto_phase = 'hold'
                self._auto_hold_start = tr

            s, sdot = min_jerk(tau)
            p_start = self._auto_seg_p_start
            p_d = p_start + s * (p_target - p_start)
            if self._auto_phase == 'move':
                v_d = ((sdot / self._auto_seg_dur)
                       * (p_target - p_start))
            else:
                v_d = np.zeros(3)

        # ── Runtime z safety clamp on desired position ────────────────
        if p_d[2] < self._min_safe_z_m:
            p_d[2] = self._min_safe_z_m

        # ── State: HOLD (position regulation at waypoint) ─────────────
        if self._auto_phase == 'hold':
            p_d = p_target
            v_d = np.zeros(3)
            hold_elapsed = tr - self._auto_hold_start
            if hold_elapsed >= self._auto_hold_time_s:
                self._auto_phase = 'settle'
                self._auto_settle_start = tr

        # ── State: SETTLE (simple /detections-based capture) ──────────
        if self._auto_phase == 'settle':
            p_d = p_target
            v_d = np.zeros(3)
            ee_err = float(np.linalg.norm(p_ee - p_target))
            settle_elapsed = tr - self._auto_settle_start

            # Orientation error for capture gating.
            if _R_des_now is not None:
                ee_rot_err_deg = rotation_error_deg(_R_des_now, R_ee)
            else:
                ee_rot_err_deg = 0.0

            tag_ok = self._is_tag_detected_recently()

            # Timeout: give up after 5 s of settling.
            if settle_elapsed > 5.0:
                if not tag_ok:
                    self.get_logger().warn(
                        f'  ✗ Waypoint [{self._auto_wp_idx}]: '
                        f'tag NOT detected — skipping')
                    self._auto_n_fail_tag += 1
                else:
                    self.get_logger().warn(
                        f'  ✗ Waypoint [{self._auto_wp_idx}]: '
                        f'settle TIMEOUT after {settle_elapsed:.1f} s '
                        f'(ee_err={ee_err * 1000:.1f} mm, '
                        f'rot_err={ee_rot_err_deg:.1f} deg) — skipping')
                    self._auto_n_fail_capture += 1
                self._auto_advance_waypoint(tr, p_ee)

            # Tag detected → check EE convergence, then capture.
            elif tag_ok:
                if ee_err >= self._capture_pos_tol_m:
                    pass  # Keep regulating — EE position not converged.
                elif ee_rot_err_deg >= self._capture_rot_tol_deg:
                    pass  # Keep regulating — orientation not converged.
                else:
                    # All conditions met → capture.
                    n_before = len(self._samples_base_tool)
                    self._capture_sample()
                    if len(self._samples_base_tool) > n_before:
                        self._auto_n_ok += 1
                        self.get_logger().info(
                            f'  ✓ Sample captured '
                            f'({self._auto_n_ok}/{self._num_samples}) '
                            f'ee_err={ee_err * 1000:.1f} mm '
                            f'rot_err={ee_rot_err_deg:.1f} deg')
                    else:
                        self._auto_n_fail_capture += 1
                        self.get_logger().warn(
                            f'  ✗ Waypoint [{self._auto_wp_idx}]: '
                            f'capture rejected by diversity check')
                    self._auto_advance_waypoint(tr, p_ee)

        # ── Cartesian control law → joint velocities ──────────────────
        e_pos = p_d - p_ee
        v_lin = v_d + self._auto_kp * e_pos

        # Orientation control: track R_des (perturbed around R_ref).
        if self._orientation_excitation_enabled and _R_des_now is not None:
            # Orientation error: log-map of R_des · R_ee^{-1} → 3-vector
            # (expressed in world frame, matching LOCAL_WORLD_ALIGNED
            # Jacobian convention).  Uses SLERP-interpolated _R_des_now so
            # e_rot changes smoothly across waypoint transitions.
            R_err = _R_des_now @ R_ee.T  # R_des · R_ee^{-1}
            e_rot = _so3_log(R_err)             # 3-vector, radians
            v_ang_raw = self._orientation_kp * e_rot
            # Low-pass the angular velocity command: prevents the instantaneous
            # v_ang jump when R_des changes at each waypoint transition.
            # alpha=0.92 at 200Hz gives ~50ms time constant — fast enough to
            # track orientation, slow enough to keep qdot steps within Franka's
            # acceleration limit (kMaxJointAcceleration=10 rad/s² @ 1kHz).
            self._auto_v_ang_filt = lpf(self._auto_v_ang_filt, v_ang_raw, 0.92)
            v_ang = self._auto_v_ang_filt

            # Stack into 6D twist and use full 6×7 Jacobian.
            twist_cmd = np.concatenate([v_lin, v_ang])  # (6,)
            J_ctrl = J_arm                               # 6×7
        else:
            # Fallback: position-only control (no orientation regulation).
            # Reset filter so it starts clean if orientation is re-enabled.
            self._auto_v_ang_filt = np.zeros(3)
            twist_cmd = v_lin                             # (3,)
            J_ctrl = J_arm[:3, :]                         # 3×7

        _max_qdelta = self._auto_qdot_acc_max / self._auto_rate_hz

        qdot_raw = dls_solve(J_ctrl, twist_cmd, self._auto_damping)
        if qdot_raw is None:
            qdot_raw = dls_solve(
                J_ctrl, 0.5 * twist_cmd, self._auto_damping,
                damping_boost=0.1)
            if qdot_raw is None:
                # Ramp qdot to zero instead of hard-zeroing: avoids the
                # acceleration spike that occurs when DLS succeeds again
                # on the next tick and qdot jumps from 0 to a finite value.
                qdot_zero = rate_limit(
                    self._auto_qdot_rl_prev,
                    np.zeros(NUM_JOINTS), _max_qdelta)
                self._auto_qdot_rl_prev = qdot_zero.copy()
                self._auto_qdot_prev = qdot_zero.copy()
                self._auto_v_ang_filt *= 0.9
                msg = Float64MultiArray()
                msg.data = qdot_zero.tolist()
                self._pub_vel.publish(msg)
                return

        qdot_scaled = envelope * qdot_raw
        qdot_clamped = clamp_joints(qdot_scaled, self._auto_qdot_max)
        qdot_filt = lpf(
            self._auto_qdot_prev, qdot_clamped, self._auto_lpf_alpha)
        self._auto_qdot_prev = qdot_filt.copy()

        # Hard acceleration limit (C¹ continuity guarantee on qdot).
        # max_delta = acc_max / rate_hz  →  |q̈| ≤ qdot_acc_max [rad/s²].
        qdot_out = rate_limit(self._auto_qdot_rl_prev, qdot_filt, _max_qdelta)
        self._auto_qdot_rl_prev = qdot_out.copy()

        msg = Float64MultiArray()
        msg.data = qdot_out.tolist()
        self._pub_vel.publish(msg)

        # ── Throttled log (~1 Hz) ─────────────────────────────────────
        if self._auto_tlog.due(t):
            n_cap = len(self._samples_base_tool)
            rot_str = ''
            if self._orientation_excitation_enabled and _R_des_now is not None:
                rot_err_d = rotation_error_deg(_R_des_now, R_ee)
                rot_str = f' |e_rot|={rot_err_d:.1f}°'
            self._auto_tlog.info(
                f'[t={t:.1f}s {self._auto_phase} '
                f'wp={self._auto_wp_idx}/'
                f'{len(self._auto_waypoints) - 1} '
                f'samples={n_cap}/{self._num_samples}] '
                f'|e_pos|={float(np.linalg.norm(e_pos)):.4f}'
                f'{rot_str}')

    # ------------------------------------------------------------------
    # Auto-motion helpers
    # ------------------------------------------------------------------

    def _generate_orientation_perturbation(self) -> np.ndarray:
        """Generate a small random orientation perturbation around q_ref.

        Returns the perturbed desired rotation matrix R_des = R_ref @ R_delta.
        Perturbation bounds (in degrees) are taken from ROS parameters:
          roll  ∈ [-orientation_roll_deg_max,  +orientation_roll_deg_max]
          pitch ∈ [-orientation_pitch_deg_max, +orientation_pitch_deg_max]
          yaw   ∈ [-orientation_yaw_deg_max,   +orientation_yaw_deg_max]

        If the AprilTag detection confidence is low, perturbation amplitude
        is reduced by 50 %.
        """
        scale = self._orient_perturbation_scale

        roll_max  = math.radians(self._orient_roll_max_deg)  * scale
        pitch_max = math.radians(self._orient_pitch_max_deg) * scale
        yaw_max   = math.radians(self._orient_yaw_max_deg)   * scale

        rng = self._auto_rng

        d_roll  = float(rng.uniform(-roll_max,  roll_max))
        d_pitch = float(rng.uniform(-pitch_max, pitch_max))
        d_yaw   = float(rng.uniform(-yaw_max,   yaw_max))

        # Compose R_delta = Rz(yaw) · Ry(pitch) · Rx(roll)  (intrinsic ZYX)
        cx, sx = math.cos(d_roll),  math.sin(d_roll)
        cy, sy = math.cos(d_pitch), math.sin(d_pitch)
        cz, sz = math.cos(d_yaw),   math.sin(d_yaw)

        Rx = np.array([[1, 0,  0],
                        [0, cx, -sx],
                        [0, sx,  cx]], dtype=np.float64)
        Ry = np.array([[ cy, 0, sy],
                        [  0, 1,  0],
                        [-sy, 0, cy]], dtype=np.float64)
        Rz = np.array([[cz, -sz, 0],
                        [sz,  cz, 0],
                        [ 0,   0, 1]], dtype=np.float64)

        R_delta = Rz @ Ry @ Rx

        # q_target = q_ref ⊗ q_delta  →  R_target = R_ref @ R_delta
        R_target = self._auto_R_ref @ R_delta

        # Convert delta to quaternion for norm check logging
        qx, qy, qz, qw = _rot_to_quat(R_delta)
        q_norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)

        self.get_logger().debug(
            f'Orientation perturbation: '
            f'Δroll={math.degrees(d_roll):.2f}° '
            f'Δpitch={math.degrees(d_pitch):.2f}° '
            f'Δyaw={math.degrees(d_yaw):.2f}° '
            f'(scale={scale:.2f})  '
            f'|q_delta|={q_norm:.6f}')

        return R_target

    def _check_tag_confidence_and_adapt(self) -> None:
        """Reduce perturbation amplitude if AprilTag confidence is low.

        If the tag has not been detected recently, halve the perturbation
        scale.  When the tag is detected again, restore full scale.
        """
        if self._is_tag_detected_recently():
            if self._orient_perturbation_scale < 1.0:
                self._orient_perturbation_scale = 1.0
                self.get_logger().info(
                    'AprilTag detected — orientation perturbation '
                    'scale restored to 1.0')
        else:
            if self._orient_perturbation_scale >= 1.0:
                self._orient_perturbation_scale = 0.5
                self.get_logger().warn(
                    'AprilTag NOT detected — reducing orientation '
                    'perturbation scale to 0.5')

    def _auto_advance_waypoint(
        self, tr: float, p_ee: np.ndarray,
    ) -> None:
        """Advance to the next waypoint, generate more if needed, or stop."""
        self._auto_wp_idx += 1

        # If all waypoints visited, generate more or stop.
        if self._auto_wp_idx >= len(self._auto_waypoints):
            if len(self._samples_base_tool) >= self._num_samples:
                self.get_logger().info('All target samples collected.')
                self._request_auto_stop()
                return
            # Generate a fresh batch and append.
            self.get_logger().info(
                f'Exhausted waypoints with '
                f'{len(self._samples_base_tool)}/{self._num_samples} '
                f'samples — generating {self._auto_num_waypoints} more.')
            new_wps = sample_waypoints(
                self._auto_num_waypoints,
                self._auto_workspace_bounds,
                self._auto_min_dist,
                self._auto_rng)
            self._auto_waypoints.extend(new_wps)

        if self._auto_wp_idx < len(self._auto_waypoints):
            self._auto_phase = 'move'
            self._auto_seg_start = tr
            self._auto_seg_p_start = p_ee.copy()

            # Generate a new orientation perturbation for this waypoint.
            # Save the current R_des as the SLERP start so the transition
            # is smooth (no instantaneous jump in orientation error).
            if (self._orientation_excitation_enabled
                    and self._auto_R_ref is not None):
                self._check_tag_confidence_and_adapt()
                self._auto_R_des_from = (
                    self._auto_R_des.copy()
                    if self._auto_R_des is not None else None)
                self._auto_R_des_slerp_start = tr
                self._auto_R_des = self._generate_orientation_perturbation()

            wp = self._auto_waypoints[self._auto_wp_idx]
            self.get_logger().info(
                f'  → Waypoint [{self._auto_wp_idx}/'
                f'{len(self._auto_waypoints) - 1}]  '
                f'target=[{wp[0]:.4f}, {wp[1]:.4f}, {wp[2]:.4f}]')
        else:
            self._request_auto_stop()

    def _request_auto_stop(
        self, stop_duration_s: float = 0.5,
    ) -> None:
        """Enter stopping phase: publish zeros, then solve."""
        if self._auto_stopping:
            return
        self._auto_stopping = True
        self._auto_stop_end = _time.monotonic() + stop_duration_s
        self.get_logger().info(
            f'Auto-motion stopping: zero velocities for '
            f'{stop_duration_s:.1f} s …')

    def _print_auto_summary(self) -> None:
        """Log the automatic-acquisition summary."""
        self.get_logger().info('')
        self.get_logger().info('═══ Automatic acquisition summary ═══')
        self.get_logger().info(
            f'  Waypoints visited     : {self._auto_wp_idx}')
        self.get_logger().info(
            f'  Successful captures   : {self._auto_n_ok}')
        self.get_logger().info(
            f'  Tag detection failures : {self._auto_n_fail_tag}')
        self.get_logger().info(
            f'  Capture rejections    : {self._auto_n_fail_capture}')
        self.get_logger().info('═' * 40)

    def _wait_tag_stable(self) -> bool:
        """Return ``True`` if the AprilTag detection is visible and stable."""
        detections: list[np.ndarray] = []
        for _ in range(self._tag_stab_n):
            _time.sleep(0.2)
            try:
                tf_msg = self._tf_buffer.lookup_transform(
                    self._cam_frame, self._tag_frame, Time(),
                    self._tf_timeout)
                detections.append(transform_to_matrix(tf_msg))
            except Exception:
                continue

        if len(detections) < self._tag_stab_n:
            return False

        ref = detections[0]
        for d in detections[1:]:
            dt = float(np.linalg.norm(d[:3, 3] - ref[:3, 3]))
            dr = rotation_error_deg(ref[:3, :3], d[:3, :3])
            if dt > self._tag_stab_m or dr > self._tag_stab_deg:
                return False
        return True

    # ══════════════════════════════════════════════════════════════════
    # Dataset I/O
    # ══════════════════════════════════════════════════════════════════

    def _resolve_dataset_path(self) -> Optional[str]:
        """Resolve ``dataset_file`` to an absolute path."""
        ds = self._dataset_file
        if os.path.isabs(ds):
            return ds

        # Try installed config directory.
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory('franka_experiments')
            return os.path.join(pkg, 'config', ds)
        except Exception:
            pass

        # Try source tree.
        candidate = os.path.join(
            os.path.dirname(__file__), '..', '..', 'config', ds)
        return os.path.realpath(candidate)

    def _save_dataset(self, path: Optional[str] = None) -> None:
        """Save collected samples to ``dataset_handeye.yaml``."""
        import yaml as _yaml

        if path is None:
            path = self._resolve_dataset_path()
        if path is None:
            self.get_logger().warn('Cannot determine dataset path — skip save.')
            return

        n = len(self._samples_base_tool)
        if n == 0:
            return

        samples = []
        for i in range(n):
            samples.append({
                'T_base_tool': self._samples_base_tool[i].tolist(),
                'T_cam_tag': self._samples_cam_tag[i].tolist(),
            })

        data = {
            'num_samples': n,
            'robot_base_frame': self._base_frame,
            'robot_tool_frame': self._tool_frame,
            'camera_frame': self._cam_frame,
            'tag_frame': self._tag_frame,
            'samples': samples,
        }

        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w') as f:
                _yaml.dump(data, f, default_flow_style=None, sort_keys=False)
            self.get_logger().info(
                f'Dataset saved → {path}  ({n} samples)')
        except Exception as e:
            self.get_logger().warn(f'Failed to save dataset: {e}')

    def _load_dataset(self, path: str) -> bool:
        """Load samples from a previously saved ``dataset_handeye.yaml``."""
        import yaml as _yaml

        try:
            with open(path, 'r') as f:
                data = _yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(f'Failed to load dataset {path}: {e}')
            return False

        samples = data.get('samples', [])
        if not samples:
            self.get_logger().warn(f'No samples in {path}.')
            return False

        for s in samples:
            T_bt = np.array(s['T_base_tool'], dtype=np.float64)
            T_ct = np.array(s['T_cam_tag'], dtype=np.float64)
            if T_bt.shape != (4, 4) or T_ct.shape != (4, 4):
                self.get_logger().warn('Skipping malformed sample in dataset.')
                continue
            self._samples_base_tool.append(T_bt)
            self._samples_cam_tag.append(T_ct)

        return len(self._samples_base_tool) > 0

    # ══════════════════════════════════════════════════════════════════
    # Initial-guess computation
    # ══════════════════════════════════════════════════════════════════

    def _compute_x_init(
        self, indices: Optional[list[int]] = None,
    ) -> np.ndarray:
        """Initial X = T_B_C from median of T_B_E(i) · inv(T_C_T(i)),
        assuming Y ≈ I (tag approximately at EE origin).

        Picks the candidate whose translation is closest to the
        component-wise median translation.
        """
        if indices is None:
            indices = list(range(len(self._samples_base_tool)))
        candidates = [
            self._samples_base_tool[i] @ invert_matrix(
                self._samples_cam_tag[i])
            for i in indices
        ]
        translations = np.array(
            [c[:3, 3] for c in candidates], dtype=np.float64)
        median_t = np.median(translations, axis=0)
        dists = np.linalg.norm(translations - median_t, axis=1)
        return candidates[int(np.argmin(dists))].copy()

    def _compute_y_init(
        self, X: np.ndarray, indices: Optional[list[int]] = None,
    ) -> np.ndarray:
        """Initial Y = T_E_T from median of inv(T_B_E(i)) · X · T_C_T(i).

        Given X, each sample yields a candidate for Y.  We pick the
        one closest to the median translation.
        """
        if indices is None:
            indices = list(range(len(self._samples_base_tool)))
        candidates = [
            invert_matrix(self._samples_base_tool[i]) @ X
            @ self._samples_cam_tag[i]
            for i in indices
        ]
        translations = np.array(
            [c[:3, 3] for c in candidates], dtype=np.float64)
        median_t = np.median(translations, axis=0)
        dists = np.linalg.norm(translations - median_t, axis=1)
        return candidates[int(np.argmin(dists))].copy()

    # ══════════════════════════════════════════════════════════════════
    # Dataset filtering
    # ══════════════════════════════════════════════════════════════════

    def _filter_samples(self) -> None:
        """Remove outlier samples using MAD on loop-closure error.

        Computes initial guesses ``X_init`` and ``Y_init`` (median-based),
        evaluates the per-sample loop-closure error norm, and rejects
        samples whose combined error exceeds ``3 · MAD`` from the median.
        """
        n = len(self._samples_base_tool)
        if n < 5:
            return

        X_init = self._compute_x_init()
        # Use estimated Y_init (not identity) to avoid wrongly rejecting
        # valid samples when the tag is not aligned with the EE frame.
        Y_init = self._compute_y_init(X_init)

        # Per-sample combined error norm.
        errors = np.empty(n, dtype=np.float64)
        for i in range(n):
            left = X_init @ self._samples_cam_tag[i]
            right = self._samples_base_tool[i] @ Y_init
            E_i = invert_matrix(left) @ right
            dt = float(np.linalg.norm(E_i[:3, 3]))
            dr_rad = math.radians(
                rotation_error_deg(np.eye(3), E_i[:3, :3]))
            errors[i] = dt + 0.1 * dr_rad  # blended metric

        median_err = float(np.median(errors))
        mad = float(np.median(np.abs(errors - median_err)))
        threshold = max(median_err + 3.0 * mad, 0.020)

        keep = [i for i in range(n) if errors[i] <= threshold]
        n_rejected = n - len(keep)

        if n_rejected > 0:
            self.get_logger().info(
                f'Filtering: rejected {n_rejected} outlier sample(s) '
                f'(MAD={mad * 1000:.1f} mm-equiv, '
                f'threshold={threshold * 1000:.1f} mm-equiv)')
            self._samples_base_tool = [
                self._samples_base_tool[i] for i in keep]
            self._samples_cam_tag = [
                self._samples_cam_tag[i] for i in keep]
        else:
            self.get_logger().info('Filtering: no outliers detected.')

    # ══════════════════════════════════════════════════════════════════
    # Keyboard loop (manual mode)
    # ══════════════════════════════════════════════════════════════════

    def _keyboard_loop(self) -> None:
        """Read stdin in a loop: ENTER → capture, 'q' → solve & quit."""
        try:
            while rclpy.ok() and not self.done:
                line = input()  # blocks until ENTER
                cmd = line.strip().lower()
                if cmd in ('q', 'quit', 'exit'):
                    self._solve_and_save()
                    self.done = True
                    return
                # Any other input (including empty ENTER) → capture
                self._capture_sample()
        except EOFError:
            # stdin closed (e.g. piped input exhausted)
            self.get_logger().warn(
                'stdin closed — solving with current samples.')
            self._solve_and_save()
            self.done = True

    # ══════════════════════════════════════════════════════════════════
    # Capture one sample
    # ══════════════════════════════════════════════════════════════════

    def _capture_sample(self) -> None:
        """Look up both TFs and store the pair if valid.

        Captures time-consistent ``T_B_E`` and ``T_C_T`` by looking up
        the robot FK at the same timestamp as the tag detection.
        """
        # 1) T_C_T first (latest) — its stamp drives the robot lookup
        try:
            tf_cam_tag = self._tf_buffer.lookup_transform(
                self._cam_frame, self._tag_frame, Time(), self._tf_timeout)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF lookup FAILED ({self._cam_frame} → '
                f'{self._tag_frame}): {e}  — is the tag visible?')
            return

        # 2) T_B_E at same timestamp for time-consistency
        tag_stamp = tf_cam_tag.header.stamp
        tag_time = Time(seconds=tag_stamp.sec,
                        nanoseconds=tag_stamp.nanosec)
        try:
            tf_base_tool = self._tf_buffer.lookup_transform(
                self._base_frame, self._tool_frame, tag_time,
                self._tf_timeout)
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self.get_logger().warn(
                'Robot TF not available at tag stamp '
                '— falling back to latest.')
            try:
                tf_base_tool = self._tf_buffer.lookup_transform(
                    self._base_frame, self._tool_frame, Time(),
                    self._tf_timeout)
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().warn(
                    f'TF lookup FAILED ({self._base_frame} → '
                    f'{self._tool_frame}): {e}')
                return

        # T_B_E(i): base → end-effector, from robot FK (URDF TF chain).
        T_base_tool = transform_to_matrix(tf_base_tool)
        # T_C_T(i): camera → tag, from AprilTag detection.
        T_cam_tag = transform_to_matrix(tf_cam_tag)

        # ── Diversity check: reject if position too close to ANY existing ──
        p_new = T_base_tool[:3, 3]
        for prev in self._samples_base_tool:
            dist = float(np.linalg.norm(p_new - prev[:3, 3]))
            if dist < _MIN_SAMPLE_POSITION_DISTANCE_M:
                self.get_logger().warn(
                    f'Sample REJECTED: tool position too close to '
                    f'an existing sample '
                    f'(< {_MIN_SAMPLE_POSITION_DISTANCE_M * 100:.1f} cm).')
                return

        self._samples_base_tool.append(T_base_tool)
        self._samples_cam_tag.append(T_cam_tag)

        n = len(self._samples_base_tool)
        t = T_base_tool[:3, 3]
        self.get_logger().info(
            f'Sample {n} captured  |  '
            f'tool @ [{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}]')

    # ══════════════════════════════════════════════════════════════════
    # Solve pipeline  (filter → init → optimise → validate → output)
    # ══════════════════════════════════════════════════════════════════

    def _solve_and_save(self) -> None:
        """Full pipeline: filter → solve X,Y → validate → save.

        X = T_B_C  (base → camera),  Y = T_E_T  (end-effector → tag).
        """

        # ── Dataset statistics ────────────────────────────────────────
        self._print_dataset_statistics()

        # ── Save raw dataset to disk ──────────────────────────────────
        self._save_dataset()

        # ── Outlier filtering ─────────────────────────────────────────
        self._filter_samples()

        # ── Check minimum count ───────────────────────────────────────
        n = len(self._samples_base_tool)
        if n < 3:
            self.get_logger().error(
                f'Need at least 3 samples to solve (have {n}).  Aborting.')
            self._print_final_summary(None, None, None, None)
            return
        if n < 10:
            self.get_logger().warn(
                f'Only {n} samples — results may be inaccurate.  '
                f'Recommend ≥15 for a reliable calibration.')

        self.get_logger().info(f'Solving hand-eye with {n} samples …')
        self.get_logger().info(
            '  Equation: X · T_C_T(i) = T_B_E(i) · Y')
        self.get_logger().info(
            '  X = T_B_C  (base → camera)')
        self.get_logger().info(
            '  Y = T_E_T  (end-effector → tag)')

        # ── Degeneracy detection ──────────────────────────────────────
        # Relative EE motions must span all three rotation axes.
        axes: list[np.ndarray] = []
        for i in range(n - 1):
            R_rel = (invert_matrix(self._samples_base_tool[i + 1])
                     @ self._samples_base_tool[i])[:3, :3]
            angle = math.acos(
                float(np.clip((np.trace(R_rel) - 1.0) / 2.0, -1.0, 1.0)))
            if angle > 1e-6:
                rvec, _ = cv2.Rodrigues(R_rel)
                axes.append(rvec.flatten() / np.linalg.norm(rvec))
        if len(axes) >= 3:
            _, sigmas, _ = np.linalg.svd(
                np.array(axes, dtype=np.float64), full_matrices=False)
            cond = sigmas[0] / max(sigmas[-1], 1e-12)
            if sigmas[-1] < 0.1 or cond > 50.0:
                self.get_logger().warn(
                    'WARNING: robot poses are poorly conditioned for '
                    f'hand-eye calibration (σ=[{sigmas[0]:.3f}, '
                    f'{sigmas[1]:.3f}, {sigmas[2]:.3f}], '
                    f'cond={cond:.1f}). Add more diverse orientations.')
            else:
                self.get_logger().info(
                    f'Pose diversity OK  (σ=[{sigmas[0]:.3f}, '
                    f'{sigmas[1]:.3f}, {sigmas[2]:.3f}], '
                    f'cond={cond:.1f})')
        else:
            self.get_logger().warn(
                'WARNING: not enough distinct relative motions to assess '
                'pose diversity.')

        # ── Initial guess ─────────────────────────────────────────────
        # X = T_B_C (base → camera), Y = T_E_T (end-effector → tag).
        X_init = self._compute_x_init()
        # Compute Y_init from X_init (not identity).
        Y_init = self._compute_y_init(X_init)

        mt, st, mr, sr = self._compute_residuals(X_init, Y_init)
        self.get_logger().info('')
        self.get_logger().info(
            f'Initial guess (median X, estimated Y):  '
            f'transl {mt * 1000:.2f}±{st * 1000:.2f} mm, '
            f'rot {mr:.3f}±{sr:.3f} deg')

        # ── Train / test split for true held-out validation ───────────
        all_indices = list(range(n))
        n_test = (max(1, int(n * self._validation_ratio))
                  if (self._validation_ratio > 0.0 and n >= 5) else 0)
        n_train = n - n_test

        if n_test > 0:
            rng = np.random.RandomState(123)
            perm = rng.permutation(n)
            test_indices = sorted(perm[:n_test].tolist())
            train_indices = sorted(perm[n_test:].tolist())
            self.get_logger().info(
                f'Train/test split: {n_train} train, {n_test} test')
        else:
            train_indices = all_indices
            test_indices = []

        # ── Nonlinear joint estimation (on TRAIN set only) ────────────
        if self._refine and _HAS_SCIPY:
            self.get_logger().info('')
            self.get_logger().info(
                'Running nonlinear SE(3) joint optimisation (12 DOF) …')
            X_best, Y_best = self._solve_nonlinear(
                X_init, Y_init, sample_indices=train_indices)
        elif not _HAS_SCIPY:
            self.get_logger().warn(
                'scipy not available — using initial guess only.  '
                'Install with: pip install scipy')
            X_best, Y_best = X_init, Y_init
        else:
            self.get_logger().info(
                'refine=False — using initial guess only.')
            X_best, Y_best = X_init, Y_init

        # ── Train residuals ───────────────────────────────────────────
        mt, st, mr, sr = self._compute_residuals(
            X_best, Y_best, indices=train_indices)
        self.get_logger().info('')
        self.get_logger().info('═══ Train residuals ═══')
        self.get_logger().info(
            f'  Translation error: '
            f'{mt * 1000:.2f} ± {st * 1000:.2f} mm')
        self.get_logger().info(
            f'  Rotation error   : '
            f'{mr:.3f} ± {sr:.3f} deg')
        self.get_logger().info('═' * 42)

        # ── Test residuals (true held-out validation) ─────────────────
        if test_indices:
            mt_t, st_t, mr_t, sr_t = self._compute_residuals(
                X_best, Y_best, indices=test_indices)
            self.get_logger().info('')
            self.get_logger().info(
                '─── Validation (held-out test set) ───')
            self.get_logger().info(
                f'  Train: {n_train} samples, '
                f'Test: {n_test} samples')
            self.get_logger().info(
                f'  Translation error: '
                f'{mt_t * 1000:.2f} ± {st_t * 1000:.2f} mm')
            self.get_logger().info(
                f'  Rotation error   : '
                f'{mr_t:.3f} ± {sr_t:.3f} deg')
            self.get_logger().info('─' * 42)

        # ── Print X = T_B_C ──────────────────────────────────────────
        (tx, ty, tz), (qx, qy, qz, qw) = \
            matrix_to_translation_quaternion(X_best)

        self.get_logger().info('')
        self.get_logger().info('═' * 64)
        self.get_logger().info(
            f'X = T_B_C  ({self._out_parent} → {self._out_child})')
        self.get_logger().info('═' * 64)
        self.get_logger().info(
            f'  translation: [{tx:.6f}, {ty:.6f}, {tz:.6f}]')
        self.get_logger().info(
            f'  quaternion : [{qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f}]')
        self.get_logger().info('')
        self.get_logger().info('4×4 homogeneous matrix:')
        for row in range(4):
            vals = '  '.join(
                f'{X_best[row, col]:+.6f}' for col in range(4))
            self.get_logger().info(f'  [{vals}]')

        # ── Print Y = T_E_T ──────────────────────────────────────────
        (ytx, yty, ytz), (yqx, yqy, yqz, yqw) = \
            matrix_to_translation_quaternion(Y_best)

        self.get_logger().info('')
        self.get_logger().info('═' * 64)
        self.get_logger().info(
            f'Y = T_E_T  ({self._tool_frame} → {self._tag_frame})')
        self.get_logger().info('═' * 64)
        self.get_logger().info(
            f'  translation: [{ytx:.6f}, {yty:.6f}, {ytz:.6f}]')
        self.get_logger().info(
            f'  quaternion : [{yqx:.6f}, {yqy:.6f}, {yqz:.6f}, {yqw:.6f}]')
        self.get_logger().info('')
        self.get_logger().info('4×4 homogeneous matrix:')
        for row in range(4):
            vals = '  '.join(
                f'{Y_best[row, col]:+.6f}' for col in range(4))
            self.get_logger().info(f'  [{vals}]')
        self.get_logger().info('')

        # ── Compute verdict BEFORE writing files ─────────────────────
        # Prefer test-set errors; fall back to train residuals.
        if test_indices:
            summary_mt_mm = mt_t * 1000
            summary_mr_deg = mr_t
            summary_source = 'test'
        else:
            summary_mt_mm = mt * 1000
            summary_mr_deg = mr
            summary_source = 'train'

        t_ok = summary_mt_mm < ACCEPTABLE_TEST_TRANSLATION_MM
        r_ok = summary_mr_deg < ACCEPTABLE_TEST_ROTATION_DEG
        is_acceptable = t_ok and r_ok

        # ── Write YAML only if calibration is ACCEPTABLE ──────────────
        if is_acceptable:
            self._write_yaml_transform(
                X_best, self._out_parent, self._out_child,
                'camera_extrinsics.yaml',
                'X = T_B_C (base → camera)')

            self._write_yaml_transform(
                Y_best, self._tool_frame, self._tag_frame,
                'ee_tag_extrinsics.yaml',
                'Y = T_E_T (end-effector → tag)')
        else:
            self.get_logger().warn(
                'Calibration NOT acceptable — '
                'extrinsics YAML files were NOT saved.  '
                'Previous calibration files (if any) are preserved.')

        # ── Optional: publish TFs for RViz inspection ─────────────────
        if self._visualize and is_acceptable:
            self._publish_calibration_tf(X_best, Y_best)

        # ── Final concise summary ─────────────────────────────────────
        self._print_final_summary(
            X_best, Y_best, summary_mt_mm, summary_mr_deg,
            error_source=summary_source,
            is_acceptable=is_acceptable)

    # ══════════════════════════════════════════════════════════════════
    # Final summary (always printed)
    # ══════════════════════════════════════════════════════════════════

    def _print_final_summary(
        self,
        X: Optional[np.ndarray],
        Y: Optional[np.ndarray],
        translation_error_mm: Optional[float],
        rotation_error_deg: Optional[float],
        error_source: str = 'test',
        is_acceptable: Optional[bool] = None,
    ) -> None:
        """Print a compact, human-readable calibration summary.

        Always called at the end of ``_solve_and_save``.  Output has
        three numbered sections:

        1. Calibration parameters (X and Y),
        2. Mean error (translation + rotation),
        3. Calibration verdict (ACCEPTABLE / NOT ACCEPTABLE).

        Parameters
        ----------
        X, Y : np.ndarray | None
            Final estimated transforms.  ``None`` when calibration failed.
        translation_error_mm, rotation_error_deg : float | None
            Mean residual errors.  ``None`` when calibration failed.
        error_source : str
            ``'test'`` if errors come from the held-out test set,
            ``'train'`` if no test set was available (fallback).
        is_acceptable : bool | None
            Pre-computed verdict.  ``None`` when calibration failed
            entirely (treated as NOT ACCEPTABLE).
        """
        log = self.get_logger().info
        sep = '=' * 60

        log('')
        log(sep)
        log('HAND-EYE CALIBRATION FINAL SUMMARY')
        log(sep)

        # ── 1) Calibration parameters ─────────────────────────────────
        log('')
        log('1) Calibration parameters')
        if X is not None:
            (tx, ty, tz), (qx, qy, qz, qw) = \
                matrix_to_translation_quaternion(X)
            log(f'X = T_B_C ({self._out_parent} -> {self._out_child})')
            log(f'translation: [{tx:.6f}, {ty:.6f}, {tz:.6f}]')
            log(f'quaternion : [{qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f}]')
        else:
            log('X = T_B_C : not available (calibration failed)')

        log('')
        if Y is not None:
            (ytx, yty, ytz), (yqx, yqy, yqz, yqw) = \
                matrix_to_translation_quaternion(Y)
            log(f'Y = T_E_T ({self._tool_frame} -> {self._tag_frame})')
            log(f'translation: [{ytx:.6f}, {yty:.6f}, {ytz:.6f}]')
            log(f'quaternion : [{yqx:.6f}, {yqy:.6f}, {yqz:.6f}, {yqw:.6f}]')
        else:
            log('Y = T_E_T : not available (calibration failed)')

        # ── 2) Mean error ─────────────────────────────────────────────
        log('')
        log('2) Mean error')
        if (translation_error_mm is not None
                and rotation_error_deg is not None):
            source_label = ('test set' if error_source == 'test'
                            else 'train set (no test split available)')
            log(f'translation error: {translation_error_mm:.2f} mm')
            log(f'rotation error   : {rotation_error_deg:.3f} deg')
            log(f'(source: {source_label})')
        else:
            log('not available (calibration failed)')

        # ── 3) Calibration verdict ────────────────────────────────────
        log('')
        log('3) Calibration verdict')
        # Determine verdict string.
        if is_acceptable is None:
            # Calibration failed entirely (< 3 samples, etc.).
            verdict = 'NOT ACCEPTABLE'
        elif is_acceptable:
            verdict = 'ACCEPTABLE'
        else:
            verdict = 'NOT ACCEPTABLE'

        log(verdict)

        if verdict == 'NOT ACCEPTABLE':
            # Explain which threshold was exceeded (if errors available).
            if (translation_error_mm is not None
                    and rotation_error_deg is not None):
                if translation_error_mm >= ACCEPTABLE_TEST_TRANSLATION_MM:
                    log(f'  (translation >= '
                        f'{ACCEPTABLE_TEST_TRANSLATION_MM:.1f} mm '
                        f'threshold)')
                if rotation_error_deg >= ACCEPTABLE_TEST_ROTATION_DEG:
                    log(f'  (rotation >= '
                        f'{ACCEPTABLE_TEST_ROTATION_DEG:.1f} deg '
                        f'threshold)')
            elif X is None:
                log('  (calibration did not produce valid results)')
            log('')
            log('Estimated extrinsics were NOT saved.')
            log('Please repeat the calibration.')

        log('')
        log(sep)
        log('')

    # ══════════════════════════════════════════════════════════════════
    # Nonlinear joint estimation  (scipy, 12 DOF)
    # ══════════════════════════════════════════════════════════════════

    def _solve_nonlinear(
        self,
        X_init: np.ndarray,
        Y_init: np.ndarray,
        sample_indices: Optional[list[int]] = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Jointly estimate X = T_B_C and Y = T_E_T via least squares.

        Minimises::

            Σ_i  || log( E_i ) ||²

        where::

            E_i  =  inv( X · T_C_T(i) ) · ( T_B_E(i) · Y )

        Decision variables: 12 parameters
        ``[rvec_X(3), tvec_X(3), rvec_Y(3), tvec_Y(3)]``
        using axis-angle (Rodrigues) + translation parameterisation.

        Parameters
        ----------
        X_init, Y_init : np.ndarray
            Initial 4×4 guesses for T_B_C and T_E_T.
        sample_indices : list[int] | None
            If given, optimise using only these samples.

        Returns
        -------
        (X_opt, Y_opt) : tuple[np.ndarray, np.ndarray]
            Optimised 4×4 transforms.
        """
        if not _HAS_SCIPY:
            return X_init, Y_init

        from scipy.optimize import least_squares

        indices = (sample_indices if sample_indices is not None
                   else list(range(len(self._samples_base_tool))))
        n = len(indices)
        if n < 3:
            return X_init, Y_init

        A_samples = [self._samples_base_tool[i] for i in indices]
        B_samples = [self._samples_cam_tag[i] for i in indices]

        # Pack initial guess into 12-D vector.
        rx0, tx0 = _mat_to_rvec_tvec(X_init)
        ry0, ty0 = _mat_to_rvec_tvec(Y_init)
        x0 = np.concatenate([
            rx0.flatten(), tx0.flatten(),
            ry0.flatten(), ty0.flatten(),
        ])

        def residuals(x: np.ndarray) -> np.ndarray:
            X = _rvec_tvec_to_mat(
                x[0:3].reshape(3, 1), x[3:6].reshape(3, 1))
            Y = _rvec_tvec_to_mat(
                x[6:9].reshape(3, 1), x[9:12].reshape(3, 1))
            res = np.empty(6 * n, dtype=np.float64)
            for i in range(n):
                # E_i = inv(X · B_i) · (A_i · Y)  should ≈ I
                left = X @ B_samples[i]
                right = A_samples[i] @ Y
                E_i = invert_matrix(left) @ right
                rv, _ = cv2.Rodrigues(E_i[:3, :3])
                res[6 * i:6 * i + 3] = self._rotation_weight * rv.flatten()
                res[6 * i + 3:6 * i + 6] = self._translation_weight * E_i[:3, 3]
            return res

        # Use 'trf' method with Huber loss for robustness against
        # AprilTag detection noise and residual outliers.
        result = least_squares(
            residuals, x0, method='trf', loss='huber',
            max_nfev=5000, ftol=1e-12, xtol=1e-12)

        X_opt = _rvec_tvec_to_mat(
            result.x[0:3].reshape(3, 1),
            result.x[3:6].reshape(3, 1))
        Y_opt = _rvec_tvec_to_mat(
            result.x[6:9].reshape(3, 1),
            result.x[9:12].reshape(3, 1))

        # Report cost improvement.
        cost_before = float(np.sum(residuals(x0) ** 2))
        cost_after = float(np.sum(result.fun ** 2))
        self.get_logger().info(
            f'  Optimisation cost: {cost_before:.6f} → {cost_after:.6f}  '
            f'({result.nfev} evaluations, status={result.status})')

        # Report residuals.
        mt, st, mr, sr = self._compute_residuals(
            X_opt, Y_opt, indices=indices)
        self.get_logger().info(
            f'  Optimised residuals: '
            f'transl {mt * 1000:.2f}±{st * 1000:.2f} mm, '
            f'rot {mr:.3f}±{sr:.3f} deg')

        return X_opt, Y_opt

    # ══════════════════════════════════════════════════════════════════
    # Residual computation
    # ══════════════════════════════════════════════════════════════════

    def _compute_residuals(
        self,
        T_base_cam: np.ndarray,
        T_ee_tag: np.ndarray,
        indices: Optional[list[int]] = None,
    ) -> tuple[float, float, float, float]:
        """Compute loop-closure residuals for candidates X, Y.

        For every sample *i*, compute::

            E_i = inv( X · T_C_T(i) ) · ( T_B_E(i) · Y )

        ``E_i`` should be close to identity.  We extract:

        * translation-norm  ``||t_{E_i}||``  (metres),
        * rotation angle  ``angle(R_{E_i})``  (degrees).

        Returns
        -------
        (mean_t, std_t, mean_r, std_r)
            Mean/std of translation errors (m) and rotation errors (deg).
        """
        if indices is None:
            indices = list(range(len(self._samples_base_tool)))

        t_errors: list[float] = []
        r_errors: list[float] = []
        for i in indices:
            left = T_base_cam @ self._samples_cam_tag[i]
            right = self._samples_base_tool[i] @ T_ee_tag
            E_i = invert_matrix(left) @ right
            dt = float(np.linalg.norm(E_i[:3, 3]))
            dr = rotation_error_deg(np.eye(3), E_i[:3, :3])
            t_errors.append(dt)
            r_errors.append(dr)

        if not t_errors:
            return 0.0, 0.0, 0.0, 0.0

        t_arr = np.array(t_errors, dtype=np.float64)
        r_arr = np.array(r_errors, dtype=np.float64)
        return (float(np.mean(t_arr)), float(np.std(t_arr)),
                float(np.mean(r_arr)), float(np.std(r_arr)))

    # ══════════════════════════════════════════════════════════════════
    # Validation (held-out samples)
    # ══════════════════════════════════════════════════════════════════

    def _validate(
        self,
        T_base_cam: np.ndarray,
        T_ee_tag: np.ndarray,
    ) -> None:
        """Evaluate calibration on held-out samples.

        Computes ``E_i = inv(X · T_C_T(i)) · (T_B_E(i) · Y)``
        on a randomly chosen test subset and reports the residuals.
        """
        n = len(self._samples_base_tool)
        n_test = max(1, int(n * self._validation_ratio))
        if n_test >= n:
            self.get_logger().warn(
                'Not enough samples for validation.')
            return

        rng = np.random.RandomState(123)
        perm = rng.permutation(n)
        test_indices = sorted(perm[:n_test].tolist())

        # Evaluate on test samples using the already-solved X, Y.
        t_errors: list[float] = []
        r_errors: list[float] = []
        for i in test_indices:
            left = T_base_cam @ self._samples_cam_tag[i]
            right = self._samples_base_tool[i] @ T_ee_tag
            E_i = invert_matrix(left) @ right
            dt = float(np.linalg.norm(E_i[:3, 3]))
            dr = rotation_error_deg(np.eye(3), E_i[:3, :3])
            t_errors.append(dt)
            r_errors.append(dr)

        t_arr = np.array(t_errors, dtype=np.float64)
        r_arr = np.array(r_errors, dtype=np.float64)

        self.get_logger().info('')
        self.get_logger().info(
            '─── Validation (held-out samples) ───')
        self.get_logger().info(
            f'  Train: {n - n_test} samples, '
            f'Test: {n_test} samples')
        self.get_logger().info(
            f'  Translation error: '
            f'{float(np.mean(t_arr)) * 1000:.2f} ± '
            f'{float(np.std(t_arr)) * 1000:.2f} mm')
        self.get_logger().info(
            f'  Rotation error   : '
            f'{float(np.mean(r_arr)):.3f} ± '
            f'{float(np.std(r_arr)):.3f} deg')
        self.get_logger().info('─' * 42)

    # ══════════════════════════════════════════════════════════════════
    # Dataset statistics / logging
    # ══════════════════════════════════════════════════════════════════

    def _print_dataset_statistics(self) -> None:
        """Log summary statistics of the collected dataset."""
        n = len(self._samples_base_tool)
        if n == 0:
            self.get_logger().info('Dataset: empty (no samples)')
            return

        translations = np.array(
            [T[:3, 3] for T in self._samples_base_tool],
            dtype=np.float64)

        self.get_logger().info('')
        self.get_logger().info('═══ Dataset Statistics ═══')
        self.get_logger().info(f'  Samples: {n}')
        self.get_logger().info('  Tool position range:')
        self.get_logger().info(
            f'    X: [{translations[:, 0].min():.3f}, '
            f'{translations[:, 0].max():.3f}] m')
        self.get_logger().info(
            f'    Y: [{translations[:, 1].min():.3f}, '
            f'{translations[:, 1].max():.3f}] m')
        self.get_logger().info(
            f'    Z: [{translations[:, 2].min():.3f}, '
            f'{translations[:, 2].max():.3f}] m')

        span = (translations.max(axis=0) - translations.min(axis=0))
        self.get_logger().info(
            f'  Translation span: [{span[0]:.3f}, '
            f'{span[1]:.3f}, {span[2]:.3f}] m')

        if n >= 2:
            # Pairwise rotation diversity (sample up to 200 pairs).
            pair_angles: list[float] = []
            max_pairs = min(n * (n - 1) // 2, 200)
            rng = np.random.RandomState(0)
            if n * (n - 1) // 2 <= max_pairs:
                for i in range(n):
                    for j in range(i + 1, n):
                        pair_angles.append(rotation_error_deg(
                            self._samples_base_tool[i][:3, :3],
                            self._samples_base_tool[j][:3, :3]))
            else:
                for _ in range(max_pairs):
                    i, j = rng.choice(n, size=2, replace=False)
                    pair_angles.append(rotation_error_deg(
                        self._samples_base_tool[i][:3, :3],
                        self._samples_base_tool[j][:3, :3]))
            pa = np.array(pair_angles)
            self.get_logger().info(
                f'  Pairwise rotation: '
                f'[{pa.min():.1f}, {pa.max():.1f}] deg  '
                f'(mean {pa.mean():.1f}°)')

        self.get_logger().info('═' * 30)

    # ══════════════════════════════════════════════════════════════════
    # YAML output
    # ══════════════════════════════════════════════════════════════════

    def _write_yaml_transform(
        self,
        T: np.ndarray,
        parent_frame: str,
        child_frame: str,
        filename: str,
        label: str,
    ) -> None:
        """Write a single rigid transform to a YAML file.

        For ``camera_extrinsics.yaml`` the configured ``yaml_style``
        is used.  For other files ``'dict'`` style is always used.
        """
        import yaml as _yaml

        (tx, ty, tz), (qx, qy, qz, qw) = \
            matrix_to_translation_quaternion(T)
        tx_r = round(float(tx), 6)
        ty_r = round(float(ty), 6)
        tz_r = round(float(tz), 6)
        qx_r = round(float(qx), 6)
        qy_r = round(float(qy), 6)
        qz_r = round(float(qz), 6)
        qw_r = round(float(qw), 6)

        # Decide style: yaml_style only for camera_extrinsics.yaml
        style = (self._yaml_style
                 if filename == 'camera_extrinsics.yaml' else 'dict')

        if style == 'arrays':
            data = {
                'parent_frame': parent_frame,
                'child_frame': child_frame,
                'translation': [tx_r, ty_r, tz_r],
                'quaternion': [qx_r, qy_r, qz_r, qw_r],
            }
        elif style == 'nested':
            data = {
                'camera_extrinsics': {
                    'parent_frame': parent_frame,
                    'child_frame': child_frame,
                    'translation': [tx_r, ty_r, tz_r],
                    'quaternion': [qx_r, qy_r, qz_r, qw_r],
                }
            }
        else:  # 'dict' (default) — compatible with launch static TF
            data = {
                'parent_frame': parent_frame,
                'child_frame': child_frame,
                'translation': {'x': tx_r, 'y': ty_r, 'z': tz_r},
                'rotation': {'x': qx_r, 'y': qy_r, 'z': qz_r, 'w': qw_r},
            }
        if style not in ('dict',):
            self.get_logger().warn(
                f'yaml_style={style!r}: output NOT compatible with '
                f'the launch file static TF publisher (use "dict").')

        # Attempt to find the installed config directory.
        yaml_path: Optional[str] = None
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory('franka_experiments')
            yaml_path = os.path.join(pkg, 'config', filename)
        except Exception:
            pass

        # Also try the source-tree path (useful during development).
        source_path: Optional[str] = None
        candidate = os.path.join(
            os.path.dirname(__file__), '..', '..', 'config', filename)
        resolved = os.path.realpath(candidate)
        config_dir = os.path.dirname(resolved)
        if os.path.isdir(config_dir):
            source_path = resolved

        # ── Hardcoded ROS workspace source-tree path (always written) ──
        _WS_CONFIG_DIR = (
            '/ros2_ws/src/franka_experiments/config')
        git_path = os.path.realpath(
            os.path.join(_WS_CONFIG_DIR, filename))

        # Collect unique paths (normalised via realpath).
        write_paths: list[str] = []
        seen: set[str] = set()
        for p in (yaml_path, source_path, git_path):
            if p is None:
                continue
            rp = os.path.realpath(p)
            if rp not in seen:
                seen.add(rp)
                write_paths.append(rp)

        written_to: list[str] = []

        for path in write_paths:
            try:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                with open(path, 'w') as f:
                    _yaml.dump(
                        data, f,
                        default_flow_style=False, sort_keys=False)
                written_to.append(path)
            except Exception as e:
                self.get_logger().warn(f'Could not write {path}: {e}')

        if written_to:
            for p in written_to:
                self.get_logger().info(f'YAML written → {p}  ({label})')
        else:
            self.get_logger().warn(
                f'Could not write {filename} automatically.  Copy the '
                f'values above into franka_experiments/config/{filename} '
                f'manually.')

        # Always print a copy-pasteable block.
        yaml_str = _yaml.dump(
            data, default_flow_style=False, sort_keys=False)
        self.get_logger().info('─' * 64)
        self.get_logger().info(f'{filename} content  ({label}):')
        for line in yaml_str.splitlines():
            self.get_logger().info(f'  {line}')
        self.get_logger().info('─' * 64)

    # ══════════════════════════════════════════════════════════════════
    # TF visualisation  (X and Y)
    # ══════════════════════════════════════════════════════════════════

    def _publish_calibration_tf(
        self,
        X: np.ndarray,
        Y: np.ndarray,
    ) -> None:
        """Publish calibrated transforms as static TFs for RViz.

        Publishes:

        * ``base → camera_link_calibrated``  (X = T_B_C)
        * ``tool → tag_calibrated``          (Y = T_E_T)
        """
        if self._tf_broadcaster is None:
            return
        from geometry_msgs.msg import TransformStamped as TFMsg

        stamp = self.get_clock().now().to_msg()
        msgs: list = []

        # ── X: base → camera_calibrated ──────────────────────────────
        (tx, ty, tz), (qx, qy, qz, qw) = \
            matrix_to_translation_quaternion(X)
        msg_x = TFMsg()
        msg_x.header.stamp = stamp
        msg_x.header.frame_id = self._out_parent
        msg_x.child_frame_id = self._out_child + '_calibrated'
        msg_x.transform.translation.x = float(tx)
        msg_x.transform.translation.y = float(ty)
        msg_x.transform.translation.z = float(tz)
        msg_x.transform.rotation.x = float(qx)
        msg_x.transform.rotation.y = float(qy)
        msg_x.transform.rotation.z = float(qz)
        msg_x.transform.rotation.w = float(qw)
        msgs.append(msg_x)

        # ── Y: tool → tag_calibrated ─────────────────────────────────
        (ytx, yty, ytz), (yqx, yqy, yqz, yqw) = \
            matrix_to_translation_quaternion(Y)
        msg_y = TFMsg()
        msg_y.header.stamp = stamp
        msg_y.header.frame_id = self._tool_frame
        msg_y.child_frame_id = self._tag_frame + '_calibrated'
        msg_y.transform.translation.x = float(ytx)
        msg_y.transform.translation.y = float(yty)
        msg_y.transform.translation.z = float(ytz)
        msg_y.transform.rotation.x = float(yqx)
        msg_y.transform.rotation.y = float(yqy)
        msg_y.transform.rotation.z = float(yqz)
        msg_y.transform.rotation.w = float(yqw)
        msgs.append(msg_y)

        self._tf_broadcaster.sendTransform(msgs)
        self.get_logger().info(
            f'Published static TF: {self._out_parent} → '
            f'{self._out_child}_calibrated  (X = T_B_C)')
        self.get_logger().info(
            f'Published static TF: {self._tool_frame} → '
            f'{self._tag_frame}_calibrated  (Y = T_E_T)')

    # ══════════════════════════════════════════════════════════════════
    # Graceful stop (run_node_main compatibility)
    # ══════════════════════════════════════════════════════════════════

    def request_stop(self) -> None:
        """Called by run_node_main on KeyboardInterrupt."""
        if self._auto_motion and hasattr(self, '_auto_stopping'):
            # Let the timer callback handle safe stop → solve.
            self.get_logger().info(
                'Interrupted — stopping auto-motion safely.')
            self._request_auto_stop()
        else:
            self.get_logger().info(
                'Interrupted — solving with current samples.')
            self._solve_and_save()
            self.done = True


# ═══════════════════════════════════════════════════════════════════════════
# Entry point
# ═══════════════════════════════════════════════════════════════════════════

def main(args=None):
    from franka_experiments.utils.ros import run_node_main
    run_node_main(HandEyeCalibrationNode, args=args)


if __name__ == '__main__':
    main()
