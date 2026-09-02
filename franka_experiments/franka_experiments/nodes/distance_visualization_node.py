#!/usr/bin/env python3
"""DistanceVisualizationNode — standalone rendering node for human-robot distance.

Completely independent from the compute node (real_time_distance).  A
freeze or restart of either node does not affect the other.

Subscriptions
-------------
  <depth_image topic>            Image            — raw depth frames
  <depth_camera_info topic>      CameraInfo       — camera intrinsics (once)
  /cbf/per_link_distances        MultiLinkDistance — one entry per CONTROL
                                                     POINT (robot_link_name
                                                     repeats) + 3-D closest
                                                     points from compute

The distance subscription uses BEST_EFFORT QoS matching the compute publisher.
Distance data is decoupled from the depth frame rate: the most-recent message
is cached under a lock and consumed by the render thread on each depth frame,
so a stale distance overlay is shown rather than blocking.

TF usage
--------
The node runs its own TFManager in best-effort mode (Level 1 + 2, never
blocks on missing or slightly lagged transforms).  This is intentional:
small temporal mismatches are visually acceptable.  The MaskBuilder also
carries its own per-link pose cache, providing additional resilience.

Published
---------
  /real_time_distance/overlay_image   Image   (only when publish_overlay_image=true
                                               or at least one subscriber is active)
"""
from __future__ import annotations

import os
import queue
import threading
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy
import trimesh
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from franka_msgs.msg import MultiLinkDistance
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from franka_experiments.utils.distance_utils import (
    define_robot_segments,
    load_extrinsics,
    load_robot_config,
)
from franka_experiments.utils.mask_builder import MaskBuilder
from franka_experiments.utils.tf_manager import TFManager
from franka_experiments.utils.visualization import VisFrame, draw_overlay


@dataclass
class _VizCP:
    """Minimal adapter matching the ControlPointResult interface used by draw_overlay."""
    point:         np.ndarray
    closest_pixel: Optional[Tuple[int, int]]
    distance:      float


class DistanceVisualization(Node):

    def __init__(self):
        super().__init__('distance_visualization')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('robot_config_path', '')
        self.declare_parameter('camera_extrinsics_path', '')
        self.declare_parameter('publish_overlay_image', False)

        robot_config_path      = self.get_parameter('robot_config_path').value
        camera_extrinsics_path = self.get_parameter('camera_extrinsics_path').value

        if not robot_config_path:
            raise RuntimeError('Parameter robot_config_path must be set.')
        if not camera_extrinsics_path:
            raise RuntimeError('Parameter camera_extrinsics_path must be set.')

        # ── Load configs ─────────────────────────────────────────────────
        self.config       = load_robot_config(robot_config_path)
        self.robot_cfg    = self.config['robot']
        self.distance_cfg = self.config['distance']
        self.mask_cfg     = self.config['mask']
        self.mesh_cfg     = self.config['meshes']
        self.zones        = self.config.get('zones', {})

        self.ee_link = self.robot_cfg.get('ee_link', 'fr3_link8')

        self.R_base, self.t_base = load_extrinsics(camera_extrinsics_path)

        # ── Visualization flags ──────────────────────────────────────────
        booleans = self.config.get('booleans', {})
        self.enable_visualization        = booleans.get('visualize', True)
        self.visual_robot_exclusion_mask = booleans.get('exclusion_mask', True)
        self.visualize_only_raw_video    = booleans.get('raw_video', False)
        self.use_segment_distance        = booleans.get('use_segment_distance', False)
        self.visual_ROI                  = booleans.get('visual_ROI', False)

        self.publish_overlay_image = (
            self.get_parameter('publish_overlay_image').value
            or booleans.get('publish_overlay_image', False)
        )

        # ── Camera intrinsics (set once by camera_info_callback) ─────────
        self.bridge                   = CvBridge()
        self.K: Optional[np.ndarray]  = None

        # ── Depth frame queue (drop-oldest, size 1) ───────────────────────
        self._frame_queue: queue.Queue = queue.Queue(maxsize=1)

        # ── Latest distance message (written by ROS thread, read by vis thread) ─
        self._latest_dist_msg: Optional[MultiLinkDistance] = None
        self._dist_lock = threading.Lock()

        # ── TF — relaxed: viz tolerates temporal lag, never blocks ───────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_mgr = TFManager(
            tf_buffer=self.tf_buffer,
            base_frame=self.robot_cfg['base_frame'],
            critical_links=[],    # no critical links — partial skeleton is fine
            throttle_s=5.0,       # reduce log spam for a pure-vis node
            cache_max_age_s=2.0,  # accept poses up to 2 s old for overlay rendering
            logger=self.get_logger(),
        )

        # ── Mesh loading ─────────────────────────────────────────────────
        mesh_pkg      = self.mesh_cfg.get('package', 'franka_description')
        mesh_base_dir = get_package_share_directory(mesh_pkg)
        sample_pts    = int(self.mesh_cfg.get('sample_points_per_link', 300))

        link_mesh_samples: Dict[str, np.ndarray] = {}
        for link_name, rel_path in self.mesh_cfg['files'].items():
            full_path = os.path.join(mesh_base_dir, rel_path)
            link_mesh_samples[link_name] = trimesh.load(
                full_path, force='mesh').sample(sample_pts)

        # ── MaskBuilder — separate instance, never shared with compute node ─
        self.mask_builder = MaskBuilder(
            link_mesh_samples=link_mesh_samples,
            R_base=self.R_base,
            t_base=self.t_base,
            ee_link=self.ee_link,
            mask_cfg=self.mask_cfg,
            logger=self.get_logger(),
        )

        # ── Subscriptions ────────────────────────────────────────────────
        topics_cfg  = self.config.get('topics', {})
        depth_topic = topics_cfg.get('depth_image',
                                     '/camera/camera/depth/image_rect_raw')
        info_topic  = topics_cfg.get('depth_camera_info',
                                     '/camera/camera/depth/camera_info')
        _be_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Image,      depth_topic, self.depth_callback,       10)
        self.create_subscription(CameraInfo, info_topic,  self.camera_info_callback, 10)
        self.create_subscription(
            MultiLinkDistance, '/cbf/per_link_distances',
            self.distance_callback, _be_qos)

        # ── Overlay publisher (created only when needed) ──────────────────
        self.overlay_pub = None
        if self.enable_visualization or self.publish_overlay_image:
            overlay_topic = topics_cfg.get(
                'overlay_image', '/real_time_distance/overlay_image')
            self.overlay_pub = self.create_publisher(Image, overlay_topic, 10)

        self.get_logger().info(
            f'DistanceVisualization ready — '
            f'viz={self.enable_visualization}  '
            f'publish_overlay={self.publish_overlay_image}  '
            f'ee_link={self.ee_link}')

        # ── Visualization thread ──────────────────────────────────────────
        self._vis_thread = threading.Thread(
            target=self._vis_loop, name='viz_render', daemon=True)
        self._vis_thread.start()

    # ── Camera callbacks ──────────────────────────────────────────────────────

    def depth_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
            frame = (cv_image, msg.header.stamp)
            try:
                self._frame_queue.put_nowait(frame)
            except queue.Full:
                try:
                    self._frame_queue.get_nowait()
                except queue.Empty:
                    pass
                self._frame_queue.put_nowait(frame)
        except Exception as exc:
            self.get_logger().warn(f'depth_callback error: {exc}')

    def camera_info_callback(self, msg):
        if self.K is not None:
            return
        if msg.k[0] == 0.0:
            self.get_logger().warn('camera_info: degenerate K (fx=0), skipping')
            return
        K = np.array(msg.k, dtype=float).reshape(3, 3)
        self.K = K
        self.mask_builder.set_intrinsics(K)
        self.get_logger().info(
            f'Camera intrinsics set: fx={msg.k[0]:.1f}  cx={msg.k[2]:.1f}')

    def distance_callback(self, msg: MultiLinkDistance):
        with self._dist_lock:
            self._latest_dist_msg = msg

    # ── Visualization loop ────────────────────────────────────────────────────

    def _vis_loop(self):
        """Dedicated render thread: pulls depth frames, renders overlay, never blocks compute."""
        while rclpy.ok():
            try:
                depth, stamp = self._frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if self.K is None:
                continue

            need_render = (
                self.enable_visualization
                or (self.publish_overlay_image
                    and self.overlay_pub is not None
                    and self.overlay_pub.get_subscription_count() > 0)
            )
            if not need_render:
                continue

            try:
                self._render_frame(depth, stamp)
            except Exception as exc:
                self.get_logger().warn(
                    f'vis error: {exc}', throttle_duration_sec=2.0)

    def _render_frame(self, depth: np.ndarray, stamp):
        H, W = depth.shape

        # TF — best-effort: non-blocking, tolerates missing or lagged transforms.
        transforms = self.tf_mgr.lookup_best_effort(
            self.robot_cfg['segment_links'], stamp)

        # Rebuild visual mask (uses internal per-link cache for missing links).
        if transforms:
            self.mask_builder.rebuild(transforms, depth.shape)

        # Full skeleton (no distance-based link filtering for visual clarity).
        robot_segments = None
        if transforms:
            robot_segments = define_robot_segments(
                transforms, self.robot_cfg, self.distance_cfg)

        # Snapshot distance data — read under lock, then work on local reference.
        with self._dist_lock:
            dist_msg = self._latest_dist_msg

        cp_results, closest_robot_pt, closest_uv_obs, min_dist = (
            self._build_vis_data(dist_msg, H, W))

        margin = int(self.distance_cfg['image_margin_px'])
        vis_frame = VisFrame(
            depth=depth,
            robot_mask=self.mask_builder.robot_mask,
            contours=self.mask_builder.contours,
            robot_segments=robot_segments,
            cp_results=cp_results,
            closest_robot_pt=closest_robot_pt,
            closest_uv_obs=closest_uv_obs,
            min_dist=min_dist,
            roi_bounds=(margin, margin, W - margin, H - margin),
            use_segment_mode=self.use_segment_distance,
            stamp=stamp,
            visual_ROI=self.visual_ROI,
            visual_exclusion_mask=self.visual_robot_exclusion_mask,
            visualize_only_raw_video=self.visualize_only_raw_video,
        )

        depth_vis = draw_overlay(
            frame=vis_frame,
            zones=self.zones,
            K=self.K,
            R_base=self.R_base,
            t_base=self.t_base,
            depth_shape=depth.shape,
        )

        if self.enable_visualization:
            cv2.imshow('Robot + closest distance', depth_vis)
            cv2.waitKey(1)

        if self.publish_overlay_image and self.overlay_pub is not None:
            self._publish_overlay(depth_vis, stamp)

    # ── Distance data → VisFrame adapter ─────────────────────────────────────

    def _build_vis_data(
        self,
        dist_msg: Optional[MultiLinkDistance],
        H: int,
        W: int,
    ) -> Tuple[Optional[list], Optional[np.ndarray], Optional[Tuple[int, int]], float]:
        """Convert a MultiLinkDistance message into VisFrame-compatible fields.

        closest_point_human (base frame) is projected to pixel coordinates so
        draw_overlay can draw the distance line from robot CP to obstacle pixel.
        Returns (cp_results, closest_robot_pt, closest_uv_obs, min_dist).
        """
        if dist_msg is None or self.K is None:
            return None, None, None, float('inf')

        cp_list: list = []
        min_dist         = float('inf')
        closest_robot_pt = None
        closest_uv_obs   = None

        for ld in dist_msg.links:
            if not ld.valid or not np.isfinite(ld.distance) or ld.distance <= 0.0:
                continue

            pt_robot = np.array(
                [ld.closest_point_robot.x,
                 ld.closest_point_robot.y,
                 ld.closest_point_robot.z],
                dtype=np.float64)
            pt_human = np.array(
                [ld.closest_point_human.x,
                 ld.closest_point_human.y,
                 ld.closest_point_human.z],
                dtype=np.float64)

            # Skip links where closest_point_human was never set (stays at origin).
            human_set = not (
                ld.closest_point_human.x == 0.0
                and ld.closest_point_human.y == 0.0
                and ld.closest_point_human.z == 0.0)
            pixel = self._project_to_pixel(pt_human, H, W) if human_set else None

            cp_list.append(_VizCP(
                point=pt_robot,
                closest_pixel=pixel,
                distance=float(ld.distance),
            ))

            if ld.distance < min_dist:
                min_dist         = ld.distance
                closest_robot_pt = pt_robot
                closest_uv_obs   = pixel

        return cp_list if cp_list else None, closest_robot_pt, closest_uv_obs, min_dist

    # ── Geometry helpers ──────────────────────────────────────────────────────

    def _project_to_pixel(
        self, p_base: np.ndarray, H: int, W: int
    ) -> Optional[Tuple[int, int]]:
        """Project a base-frame 3-D point into the depth image plane."""
        p_cam = self.R_base.T @ (p_base - self.t_base)
        if p_cam[2] <= 0.0:
            return None
        uv = self.K @ p_cam
        u = int(uv[0] / uv[2])
        v = int(uv[1] / uv[2])
        return (u, v) if 0 <= u < W and 0 <= v < H else None

    def _publish_overlay(self, img: np.ndarray, stamp):
        try:
            msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            msg.header.stamp    = stamp
            msg.header.frame_id = 'camera_depth_optical_frame'
            self.overlay_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'overlay publish error: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = DistanceVisualization()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
