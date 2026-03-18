#!/usr/bin/env python3
"""
CAPSULE OVERLAY NODE
====================
Projects the FR3 robot capsules (from ``franka_simulation``) onto the
camera image as a 2-D overlay, producing a debug image that shows
exactly where the distance estimator "thinks" the robot links are.

Pipeline
--------
1. Subscribe to ``/joint_states`` to track the robot configuration.
2. Subscribe to the annotated camera image
   (``/human_pose/image`` by default — already has MediaPipe skeleton).
3. On each image callback:
   a. Run Pinocchio FK with the latest *q*.
   b. Materialise capsule segments via ``iter_world_capsule_segments()``.
   c. Transform each segment endpoint base→camera using T_cam_base.
   d. Project to 2-D via the RGB intrinsic matrix K.
   e. Draw each capsule (axis line + radius circles) on the image.
4. Publish the overlay image on ``/human_pose/capsule_overlay``.

All capsule geometry comes **directly** from ``franka_simulation`` —
no local definitions.
"""

from __future__ import annotations

import os
import time
from typing import Dict, List, Optional

import cv2
import numpy as np
import pinocchio as pin

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import Image, JointState

from cv_bridge import CvBridge

from franka_experiments.utils.constants import FR3_JOINT_NAMES
from franka_experiments.utils.kinematics import generate_urdf_from_xacro
from franka_experiments.utils.logging_utils import ThrottledLogger
from franka_experiments.utils.simulation_imports import (
    build_joint_to_joint_capsules,
    build_reduced_pinocchio_model_from_urdf,
    iter_world_capsule_segments,
)

import yaml
from ament_index_python.packages import get_package_share_directory
from franka_experiments.utils.camera_yaml import load_camera_info_yaml

# ── Default capsule radii (same as franka_simulation) ─────────────────────
_DEFAULT_CAPSULE_RADII: List[float] = [0.15, 0.12, 0.13]

# ── Rendering constants ──────────────────────────────────────────────────
# Mirroring make_capsule_markers: CYLINDER body + 2 SPHERE caps.
_CAP_COLOR_BGR = (0, 0, 255)       # bright red – sphere caps (start/end)
_BODY_COLOR_BGR = (0, 0, 180)      # darker red – cylinder body
_CAP_EDGE_BGR = (255, 255, 255)    # white edge around caps
_CAPSULE_ALPHA = 0.40              # alpha-blending factor
_N_CIRCLE_PTS = 20                 # angular samples per circle
_N_HEMI_RINGS = 4                  # latitude rings for hemisphere caps

# ── Joint debug overlay constants ─────────────────────────────────────
_JOINT_CIRCLE_RADIUS = 6
_JOINT_COLOR_BGR = (0, 255, 0)     # green filled circle
_JOINT_LABEL_BGR = (255, 255, 255) # white text label


class CapsuleOverlayNode(Node):
    """Project robot capsules onto the camera image as 2-D overlay."""

    def __init__(self):
        super().__init__("capsule_overlay_node")

        self._bridge = CvBridge()

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter("joint_state_topic", "/NS_1/joint_states")
        self.declare_parameter("image_topic", "/human_pose/image")
        self.declare_parameter("output_topic", "/human_pose/capsule_overlay")
        self._joint_state_topic = str(
            self.get_parameter("joint_state_topic").value
        )
        self._image_topic = str(self.get_parameter("image_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)

        # ── Pinocchio + capsules from franka_simulation ───────────────
        self._pin_ok = False
        self._model = None
        self._data = None
        self._capsules: Dict[str, list] = {}
        self._frame_ids: Dict[str, int] = {}
        self._init_pinocchio()

        # ── Camera matrices ───────────────────────────────────────────
        self._K_rgb: Optional[np.ndarray] = None       # 3×3
        self._T_cam_base: Optional[np.ndarray] = None   # 4×4  camera←base
        self._load_camera_params()

        # ── Mutable state ─────────────────────────────────────────────
        self._q: Optional[np.ndarray] = None
        self._tlog = ThrottledLogger(self.get_logger(), period_s=2.0)
        self._tlog_proj = ThrottledLogger(self.get_logger(), period_s=2.0)
        self._tlog_base = ThrottledLogger(self.get_logger(), period_s=1.0)

        # ── Subscriptions ─────────────────────────────────────────────
        self.create_subscription(
            JointState, self._joint_state_topic,
            self._joint_state_cb, 10,
        )
        self.create_subscription(
            Image, self._image_topic,
            self._image_cb, 10,
        )

        # ── Publisher ─────────────────────────────────────────────────
        self._pub = self.create_publisher(Image, self._output_topic, 10)

        self.get_logger().info(
            f"🟢 Capsule Overlay Node READY  "
            f"image={self._image_topic}  output={self._output_topic}"
        )

    # ================================================================
    # Init helpers
    # ================================================================

    def _init_pinocchio(self) -> None:
        """Load Pinocchio model and build capsules from franka_simulation."""
        try:
            urdf_xml = generate_urdf_from_xacro()
            self._model, self._data = build_reduced_pinocchio_model_from_urdf(
                urdf_xml,
            )
            self._capsules = build_joint_to_joint_capsules(
                model=self._model,
                capsule_radii=_DEFAULT_CAPSULE_RADII,
            )
            self._frame_ids = {}
            self._pin_ok = True
            self.get_logger().info(
                f"✓ Pinocchio + {len(self._capsules)} capsules loaded"
            )
        except Exception as e:
            self.get_logger().error(f"Pinocchio/capsule init failed: {e}")

    def _load_camera_params(self) -> None:
        """Load camera intrinsics + extrinsics from YAML config files."""
        try:
            pkg_share = get_package_share_directory("franka_experiments")
        except Exception:
            self.get_logger().error(
                "Cannot resolve franka_experiments share dir"
            )
            return

        # ── RGB intrinsics ────────────────────────────────────────────
        rgb_path = os.path.join(pkg_share, "config", "rgb_intrinsics.yaml")
        if os.path.isfile(rgb_path):
            try:
                data = load_camera_info_yaml(rgb_path)
                if data is not None:
                    k = data.get("k", [])
                    if len(k) == 9:
                        self._K_rgb = np.array(k, dtype=float).reshape(3, 3)
                        self.get_logger().info(
                            f"📷 RGB K loaded: fx={self._K_rgb[0,0]:.1f} "
                            f"fy={self._K_rgb[1,1]:.1f}"
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
                self.get_logger().warn(f"Failed to parse rgb_intrinsics: {e}")

        # ── Extrinsics (base→camera) ─────────────────────────────────
        ext_path = os.path.join(
            pkg_share, "config", "camera_extrinsics.yaml"
        )
        if os.path.isfile(ext_path):
            try:
                with open(ext_path, "r") as f:
                    data = yaml.safe_load(f)
                tr = data["translation"]
                rot = data["rotation"]
                tx, ty, tz = (
                    float(tr["x"]), float(tr["y"]), float(tr["z"]),
                )
                qx, qy, qz, qw = (
                    float(rot["x"]), float(rot["y"]),
                    float(rot["z"]), float(rot["w"]),
                )

                # Build T_base_cam (base → camera optical frame)
                q_norm = float(
                    np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
                )
                if q_norm < 1e-9:
                    self.get_logger().warn("Extrinsic quat is zero")
                    return
                qx /= q_norm; qy /= q_norm; qz /= q_norm; qw /= q_norm
                R = np.array([
                    [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                    [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                    [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
                ], dtype=float)
                T_base_cam = np.eye(4, dtype=float)
                T_base_cam[:3, :3] = R
                T_base_cam[:3, 3] = [tx, ty, tz]

                # Invert: T_cam_base = T_base_cam^{-1}
                R_inv = R.T
                t_inv = -R_inv @ np.array([tx, ty, tz])
                T_cam_base = np.eye(4, dtype=float)
                T_cam_base[:3, :3] = R_inv
                T_cam_base[:3, 3] = t_inv
                self._T_cam_base = T_cam_base

                self.get_logger().info(
                    f"📷 T_cam_base loaded from {ext_path}"
                )
            except Exception as e:
                self.get_logger().warn(
                    f"Failed to parse camera_extrinsics: {e}"
                )

    # ================================================================
    # Callbacks
    # ================================================================

    def _joint_state_cb(self, msg: JointState) -> None:
        try:
            name_map = {str(n): int(i) for i, n in enumerate(msg.name)}
            q = np.array(
                [msg.position[name_map[jn]] for jn in FR3_JOINT_NAMES],
                dtype=float,
            )
            self._q = q
        except (KeyError, ValueError, IndexError):
            pass

    def _image_cb(self, msg: Image) -> None:
        """Receive image, overlay capsules, publish."""
        # Gate: need everything
        if (
            self._q is None
            or not self._pin_ok
            or self._K_rgb is None
            or self._T_cam_base is None
        ):
            t_w = time.time()
            if self._tlog.due(t_w):
                self._tlog.warn(
                    f"[OVERLAY] waiting: q={'✓' if self._q is not None else '✗'}  "
                    f"pin={'✓' if self._pin_ok else '✗'}  "
                    f"K={'✓' if self._K_rgb is not None else '✗'}  "
                    f"T={'✓' if self._T_cam_base is not None else '✗'}"
                )
            # Pass through the image unmodified
            self._pub.publish(msg)
            return

        # Decode image
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            self._pub.publish(msg)
            return

        # FK
        self._run_fk()

        # ── Joint debug overlay ────────────────────────────────────
        # Project each FR3 joint frame onto the image so we can
        # visually verify extrinsic / intrinsic calibration.
        R_cb = self._T_cam_base[:3, :3]
        t_cb = self._T_cam_base[:3, 3]
        K = self._K_rgb
        n_projected = 0
        z_cam_j1: Optional[float] = None

        for idx, jname in enumerate(FR3_JOINT_NAMES):
            try:
                fid = self._model.getFrameId(jname)
                p_base = self._data.oMf[fid].translation.copy()
            except Exception:
                continue
            p_cam = R_cb @ p_base + t_cb
            if idx == 0:
                z_cam_j1 = float(p_cam[2])
            uv = self._project(p_cam, K)
            if uv[0] is None:
                continue
            u, v = int(uv[0]), int(uv[1])
            label = f"J{idx + 1}"
            cv2.circle(
                frame, (u, v),
                _JOINT_CIRCLE_RADIUS, _JOINT_COLOR_BGR, -1,
            )
            cv2.putText(
                frame, label, (u + 5, v - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                _JOINT_LABEL_BGR, 1, cv2.LINE_AA,
            )
            n_projected += 1

        t_now = time.time()
        if self._tlog.due(t_now):
            self._tlog.info(
                f"[JOINT DEBUG] projected={n_projected}  "
                f"z_cam_J1={z_cam_j1:.2f}" if z_cam_j1 is not None
                else f"[JOINT DEBUG] projected={n_projected}  z_cam_J1=N/A"
            )

        # ── Numerical projection debug ─────────────────────────────
        # Diagnose extrinsic / intrinsic correctness by logging the
        # full transform chain for 3 representative joints.
        t_dbg = time.time()
        if self._tlog_proj.due(t_dbg):
            img_h, img_w = frame.shape[:2]
            log = self.get_logger()
            log.info("─" * 30 + " PROJ DEBUG " + "─" * 30)
            det_R = float(np.linalg.det(R_cb))
            log.info(
                f"[PROJ DEBUG] det(R_cb)={det_R:.6f}  "
                f"t_cb=({t_cb[0]:+.4f}, {t_cb[1]:+.4f}, {t_cb[2]:+.4f})"
            )
            if abs(det_R - 1.0) > 0.01:
                log.warn(
                    f"⚠ det(R_cb)={det_R:.6f} ≠ 1  → R is not a proper rotation!"
                )
            debug_points = [
                ("J1 (base)",  FR3_JOINT_NAMES[0]),
                ("J4 (mid)",   FR3_JOINT_NAMES[3]),
                ("J7 (EE)",    FR3_JOINT_NAMES[6]),
            ]
            for label, fname in debug_points:
                self._debug_projection(
                    label, fname, R_cb, t_cb, K, img_h, img_w,
                )
            log.info("─" * 30 + " /PROJ DEBUG " + "─" * 29)

        # ── Base frame debug (origin + axes + cluster) ─────────
        self._draw_base_debug(frame, R_cb, t_cb, K)

        # Get world-frame capsule segments from franka_simulation
        segments = iter_world_capsule_segments(
            capsules=self._capsules,
            frame_ids=self._frame_ids,
            data=self._data,
        )

        # Prepare alpha overlay — draw all capsules onto a copy, then blend.
        # Rendering order per capsule: body first, then caps on top.
        # This mirrors make_capsule_markers: 1 CYLINDER + 2 SPHEREs.
        overlay = frame.copy()

        for seg in segments:
            p0_base = np.array(seg["p0"], dtype=float).reshape(3)
            p1_base = np.array(seg["p1"], dtype=float).reshape(3)
            radius = float(seg["radius"])

            # Build local orthonormal frame for this capsule
            frame_ax = self._capsule_frame(p0_base, p1_base)
            if frame_ax is None:
                continue
            e_ax, e_u, e_v = frame_ax

            # ── 1. Cylinder body ──────────────────────────────────────
            body_pts = self._project_cylinder(
                p0_base, p1_base, radius, e_u, e_v,
                R_cb, t_cb, K,
            )
            if body_pts is not None and len(body_pts) >= 3:
                hull = cv2.convexHull(
                    np.array(body_pts, dtype=np.int32),
                )
                cv2.fillConvexPoly(overlay, hull, _BODY_COLOR_BGR)

            # ── 2. Start cap (sphere at p0) ───────────────────────────
            cap0_pts = self._project_hemisphere(
                p0_base, -e_ax, radius, e_u, e_v,
                R_cb, t_cb, K,
            )
            if cap0_pts is not None and len(cap0_pts) >= 3:
                hull = cv2.convexHull(
                    np.array(cap0_pts, dtype=np.int32),
                )
                cv2.fillConvexPoly(overlay, hull, _CAP_COLOR_BGR)
                cv2.polylines(
                    overlay, [hull], True, _CAP_EDGE_BGR, 1,
                )

            # ── 3. End cap (sphere at p1) ─────────────────────────────
            cap1_pts = self._project_hemisphere(
                p1_base, e_ax, radius, e_u, e_v,
                R_cb, t_cb, K,
            )
            if cap1_pts is not None and len(cap1_pts) >= 3:
                hull = cv2.convexHull(
                    np.array(cap1_pts, dtype=np.int32),
                )
                cv2.fillConvexPoly(overlay, hull, _CAP_COLOR_BGR)
                cv2.polylines(
                    overlay, [hull], True, _CAP_EDGE_BGR, 1,
                )

        # Alpha-blend the capsule overlay onto the original frame
        cv2.addWeighted(
            overlay, _CAPSULE_ALPHA, frame, 1.0 - _CAPSULE_ALPHA, 0, frame,
        )

        # Publish overlay
        try:
            out_msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            out_msg.header = msg.header
            self._pub.publish(out_msg)
        except Exception:
            self._pub.publish(msg)

    # ================================================================
    # Helpers
    # ================================================================

    def _run_fk(self) -> None:
        """Run FK with latest joint state."""
        q_full = pin.neutral(self._model)
        for k, jname in enumerate(FR3_JOINT_NAMES):
            jid = self._model.getJointId(jname)
            idx_q = self._model.joints[jid].idx_q
            q_full[idx_q] = self._q[k]
        pin.forwardKinematics(self._model, self._data, q_full)
        pin.updateFramePlacements(self._model, self._data)

    @staticmethod
    def _project(
        p_cam: np.ndarray, K: np.ndarray,
    ) -> tuple:
        """Pinhole projection.  Returns (u, v) or (None, None)."""
        if p_cam[2] <= 0.01:
            return None, None
        fx, fy = float(K[0, 0]), float(K[1, 1])
        cx, cy = float(K[0, 2]), float(K[1, 2])
        u = fx * p_cam[0] / p_cam[2] + cx
        v = fy * p_cam[1] / p_cam[2] + cy
        return float(u), float(v)

    # ── Base frame debug helper ──────────────────────────────────────────

    def _draw_base_debug(
        self,
        frame: np.ndarray,
        R_cb: np.ndarray,
        t_cb: np.ndarray,
        K: np.ndarray,
    ) -> None:
        """Draw the robot base frame origin, XYZ axes and cluster points.

        Visualises the Pinocchio base frame projected onto the camera
        image to verify extrinsic calibration and FK base alignment.
        """
        try:
            # ── Define points in BASE frame ──────────────────────
            base_pts = {
                "O":  np.array([0.0,   0.0,   0.0]),
                "Px": np.array([0.10,  0.0,   0.0]),
                "Py": np.array([0.0,   0.10,  0.0]),
                "Pz": np.array([0.0,   0.0,   0.10]),
                "P1": np.array([0.05,  0.05,  0.0]),
                "P2": np.array([0.05, -0.05,  0.0]),
                "P3": np.array([-0.05, 0.05,  0.0]),
                "P4": np.array([-0.05,-0.05,  0.0]),
            }

            # ── Transform to CAMERA frame ────────────────────────
            cam = {}
            for name, p_base in base_pts.items():
                cam[name] = R_cb @ p_base + t_cb

            # ── Project to pixel coordinates ─────────────────────
            px = {}  # name → (u_int, v_int) | None
            for name, p_cam in cam.items():
                if float(p_cam[2]) <= 0.0:
                    px[name] = None
                    continue
                uv = self._project(p_cam, K)
                if uv[0] is not None:
                    px[name] = (int(uv[0]), int(uv[1]))
                else:
                    px[name] = None

            o_px = px.get("O")
            if o_px is None:
                self.get_logger().warn(
                    "[BASE_DEBUG] origin not projectable — "
                    f"z_cam={float(cam['O'][2]):+.4f}"
                )
                return

            # ── 1) Origin — big red filled circle + label ────────
            cv2.circle(frame, o_px, 10, (0, 0, 255), -1)
            cv2.putText(
                frame, "BASE", (o_px[0] + 12, o_px[1] + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA,
            )

            # ── 2) X axis — red line O→Px ────────────────────────
            if px.get("Px") is not None:
                cv2.line(frame, o_px, px["Px"], (0, 0, 255), 3)
                cv2.putText(
                    frame, "X", (px["Px"][0] + 5, px["Px"][1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA,
                )

            # ── 3) Y axis — green line O→Py ──────────────────────
            if px.get("Py") is not None:
                cv2.line(frame, o_px, px["Py"], (0, 255, 0), 3)
                cv2.putText(
                    frame, "Y", (px["Py"][0] + 5, px["Py"][1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA,
                )

            # ── 4) Z axis — blue line O→Pz ───────────────────────
            if px.get("Pz") is not None:
                cv2.line(frame, o_px, px["Pz"], (255, 0, 0), 3)
                cv2.putText(
                    frame, "Z", (px["Pz"][0] + 5, px["Pz"][1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1, cv2.LINE_AA,
                )

            # ── 5) Yellow cluster points around origin ───────────
            for name in ("P1", "P2", "P3", "P4"):
                if px.get(name) is not None:
                    cv2.circle(frame, px[name], 5, (0, 255, 255), -1)

            # ── 6) Throttled numerical logging (1 s) ─────────────
            t_base = time.time()
            if self._tlog_base.due(t_base):
                o_b = base_pts["O"]
                o_c = cam["O"]
                log = self.get_logger()
                log.info("[BASE_DEBUG]")
                log.info(
                    f"  base_xyz = [{o_b[0]:+.4f}, {o_b[1]:+.4f}, "
                    f"{o_b[2]:+.4f}]"
                )
                log.info(
                    f"  cam_xyz  = [{o_c[0]:+.4f}, {o_c[1]:+.4f}, "
                    f"{o_c[2]:+.4f}]"
                )
                log.info(f"  uv       = [{o_px[0]}, {o_px[1]}]")
                for axis_name in ("Px", "Py", "Pz"):
                    c = cam[axis_name]
                    p = px.get(axis_name)
                    px_str = f"[{p[0]}, {p[1]}]" if p else "N/A"
                    log.info(
                        f"  {axis_name}_cam = [{c[0]:+.4f}, "
                        f"{c[1]:+.4f}, {c[2]:+.4f}]  "
                        f"px={px_str}"
                    )

        except Exception as e:
            self.get_logger().warn(f"[BASE_DEBUG] exception: {e}")

    # ── Projection debug helper ─────────────────────────────────────────

    def _debug_projection(
        self,
        label: str,
        frame_name: str,
        R_cb: np.ndarray,
        t_cb: np.ndarray,
        K: np.ndarray,
        img_h: int,
        img_w: int,
    ) -> None:
        """Log the full base→camera→pixel chain for one Pinocchio frame.

        Prints base coords, camera coords, z_cam, pixel coords, and
        emits warnings when the geometry looks suspicious.
        """
        log = self.get_logger()

        # 1. Position in BASE frame from FK
        try:
            fid = self._model.getFrameId(frame_name)
            p_base = self._data.oMf[fid].translation.copy()
        except Exception as e:
            log.warn(f"  [{label}] cannot get frame '{frame_name}': {e}")
            return

        # 2. Transform to CAMERA frame:  p_cam = R_cb @ p_base + t_cb
        p_cam = R_cb @ p_base + t_cb
        z_cam = float(p_cam[2])

        # 3. Log coordinates
        log.info(
            f"  [{label}]  base=({p_base[0]:+.4f}, {p_base[1]:+.4f}, {p_base[2]:+.4f})"
            f"  →  cam=({p_cam[0]:+.4f}, {p_cam[1]:+.4f}, {p_cam[2]:+.4f})"
        )

        # 4. Warnings on z_cam
        if z_cam < 0.0:
            log.warn(
                f"  ⚠ [{label}] z_cam={z_cam:+.4f} < 0  → POINT IS BEHIND CAMERA"
            )
        elif z_cam < 0.05:
            log.warn(
                f"  ⚠ [{label}] z_cam={z_cam:+.4f} < 0.05  → NEAR-PLANE CLIPPING"
            )

        # 5. Pinhole projection
        uv = self._project(p_cam, K)
        if uv[0] is None:
            log.warn(
                f"  ⚠ [{label}] projection FAILED (z ≤ 0.01)"
            )
            return

        u, v = uv
        inside = (0.0 <= u < float(img_w)) and (0.0 <= v < float(img_h))
        log.info(
            f"  [{label}]  pixel=({u:.1f}, {v:.1f})  "
            f"inside={'YES' if inside else 'NO'}  "
            f"img=({img_w}×{img_h})"
        )

        # 6. Warning if pixel is far outside
        if not inside:
            overshoot_u = max(0.0, -u, u - float(img_w))
            overshoot_v = max(0.0, -v, v - float(img_h))
            log.warn(
                f"  ⚠ [{label}] pixel OUTSIDE image  "
                f"overshoot=({overshoot_u:.0f}px, {overshoot_v:.0f}px)  "
                f"→ possible extrinsic mismatch"
            )

    # ── Capsule geometry helpers (mirror make_capsule_markers) ─────────

    @staticmethod
    def _capsule_frame(
        p0: np.ndarray, p1: np.ndarray,
    ) -> Optional[tuple]:
        """Build an orthonormal frame (e_ax, e_u, e_v) for a capsule axis.

        Returns None if the capsule is degenerate (zero-length).
        """
        axis = p1 - p0
        axis_len = float(np.linalg.norm(axis))
        if axis_len < 1e-6:
            return None
        e_ax = axis / axis_len
        ref = np.array([1.0, 0.0, 0.0])
        if abs(float(np.dot(e_ax, ref))) > 0.9:
            ref = np.array([0.0, 1.0, 0.0])
        e_u = np.cross(e_ax, ref)
        e_u = e_u / (float(np.linalg.norm(e_u)) + 1e-12)
        e_v = np.cross(e_ax, e_u)
        return e_ax, e_u, e_v

    def _project_circle_pts(
        self,
        center: np.ndarray,
        radius: float,
        e_u: np.ndarray,
        e_v: np.ndarray,
        R_cb: np.ndarray,
        t_cb: np.ndarray,
        K: np.ndarray,
    ) -> List[tuple]:
        """Project a 3-D circle (centre + radius in plane e_u/e_v) to 2-D."""
        angles = np.linspace(0.0, 2.0 * np.pi, _N_CIRCLE_PTS, endpoint=False)
        pts: List[tuple] = []
        for a in angles:
            pt = center + radius * (float(np.cos(a)) * e_u
                                    + float(np.sin(a)) * e_v)
            pc = R_cb @ pt + t_cb
            if pc[2] <= 0.05:
                continue
            uv = self._project(pc, K)
            if uv[0] is not None:
                pts.append((int(uv[0]), int(uv[1])))
        return pts

    def _project_cylinder(
        self,
        p0: np.ndarray,
        p1: np.ndarray,
        radius: float,
        e_u: np.ndarray,
        e_v: np.ndarray,
        R_cb: np.ndarray,
        t_cb: np.ndarray,
        K: np.ndarray,
    ) -> Optional[List[tuple]]:
        """Project the CYLINDER body (p0→p1, constant radius) to 2-D.

        Mirrors the RViz CYLINDER marker in ``make_capsule_markers``.
        Samples circles at both endpoints + 2 intermediate stations to
        handle perspective correctly, then returns all 2-D points for the
        caller to compute a convex hull.
        """
        axis = p1 - p0
        all_pts: List[tuple] = []
        for t in (0.0, 0.33, 0.67, 1.0):
            c = p0 + float(t) * axis
            all_pts.extend(
                self._project_circle_pts(
                    c, radius, e_u, e_v, R_cb, t_cb, K,
                )
            )
        return all_pts if len(all_pts) >= 3 else None

    def _project_hemisphere(
        self,
        center: np.ndarray,
        outward: np.ndarray,
        radius: float,
        e_u: np.ndarray,
        e_v: np.ndarray,
        R_cb: np.ndarray,
        t_cb: np.ndarray,
        K: np.ndarray,
    ) -> Optional[List[tuple]]:
        """Project a hemisphere cap (SPHERE in ``make_capsule_markers``) to 2-D.

        The hemisphere extends from the equator circle at *center* outward
        along *outward* (unit vector pointing away from the capsule body).
        We sample latitude rings from equator (φ=0) to pole (φ=π/2).
        The equator circle itself is included so that the cap seamlessly
        covers the cylinder end.
        """
        all_pts: List[tuple] = []
        # Equator (φ = 0) — coincides with the cylinder end
        all_pts.extend(
            self._project_circle_pts(
                center, radius, e_u, e_v, R_cb, t_cb, K,
            )
        )
        # Latitude rings from equator to pole
        for i in range(1, _N_HEMI_RINGS + 1):
            phi = (float(i) / float(_N_HEMI_RINGS)) * (np.pi / 2.0)
            ring_r = radius * float(np.cos(phi))
            offset = radius * float(np.sin(phi))
            ring_center = center + offset * outward
            all_pts.extend(
                self._project_circle_pts(
                    ring_center, ring_r, e_u, e_v, R_cb, t_cb, K,
                )
            )
        # Pole point
        pole = center + radius * outward
        pc = R_cb @ pole + t_cb
        if pc[2] > 0.05:
            uv = self._project(pc, K)
            if uv[0] is not None:
                all_pts.append((int(uv[0]), int(uv[1])))
        return all_pts if len(all_pts) >= 3 else None


def main(args=None):
    rclpy.init(args=args)
    node = CapsuleOverlayNode()
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
