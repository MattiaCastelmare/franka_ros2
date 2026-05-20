#!/usr/bin/env python3
"""RealTimeDistanceNode — pure compute node for human-robot distance estimation.

All safety-critical logic lives here: TF lookup, robot mask construction, and
distance calculation.  No rendering, no OpenCV windows, no overlay images.

Published topics
----------------
  /human_robot/multi_distance   MultiDistance       — per-segment CP distances
  /human_robot/distance         HumanRobotDistance  — fallback / aggregate
  /cbf/per_link_distances       MultiLinkDistance   — per-link for CBF + viz

The standalone distance_visualization_node subscribes to
/cbf/per_link_distances and the depth/camera-info topics to draw the overlay
in a completely separate process — no shared locks, no shared queues.
"""
from __future__ import annotations

import os
import queue
import threading
import time
from typing import Optional

import numpy as np
import rclpy
import trimesh
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from franka_msgs.msg import HumanRobotDistance, MultiDistance, MultiLinkDistance
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from franka_experiments.utils.distance_engine import DistanceEngine
from franka_experiments.utils.distance_utils import (
    base_to_cam_z,
    compute_roi,
    define_control_points,
    load_extrinsics,
    load_robot_config,
)
from franka_experiments.utils.mask_builder import MaskBuilder
from franka_experiments.utils.node_utils import (
    PerfTimer,
    build_cp_messages,
    no_obs_warn,
)
from franka_experiments.utils.tf_manager import TFManager


class RealTimeDistance(Node):

    def __init__(self):
        super().__init__('real_time_distance')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('robot_config_path', '')
        self.declare_parameter('camera_extrinsics_path', '')
        self.declare_parameter('process_rate_hz', 20.0)
        self.declare_parameter('compute_rate_hz', 20.0)

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
        self.R_base_f32 = self.R_base.astype(np.float32)
        self.t_base_f32 = self.t_base.astype(np.float32)

        # ── Camera intrinsics (populated by camera_info_callback) ────────
        self.bridge    = CvBridge()
        self.K         = None
        self.K_inv     = None
        self.fx = self.fy = None
        self.cx = self.cy = None
        self.cx_f32 = self.cy_f32 = None
        self.fx_inv_f32 = self.fy_inv_f32 = None

        # ── Frame queue ───────────────────────────────────────────────────
        self._frame_queue      = queue.Queue(maxsize=1)
        self._last_depth_shape: Optional[tuple] = None
        self._last_transforms: Optional[dict]   = None

        # ── TF ───────────────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        critical_links = self.robot_cfg.get('critical_links', [self.ee_link])
        self.tf_mgr = TFManager(
            tf_buffer=self.tf_buffer,
            base_frame=self.robot_cfg['base_frame'],
            critical_links=critical_links,
            throttle_s=2.0,
            cache_max_age_s=float(self.distance_cfg.get('tf_cache_max_age_s', 0.5)),
            logger=self.get_logger(),
        )

        # ── Mesh loading ─────────────────────────────────────────────────
        mesh_pkg      = self.mesh_cfg.get('package', 'franka_description')
        mesh_base_dir = get_package_share_directory(mesh_pkg)
        sample_pts    = int(self.mesh_cfg.get('sample_points_per_link', 300))

        link_mesh_samples: dict[str, np.ndarray] = {}
        for link_name, rel_path in self.mesh_cfg['files'].items():
            full_path = os.path.join(mesh_base_dir, rel_path)
            link_mesh_samples[link_name] = trimesh.load(
                full_path, force='mesh').sample(sample_pts)

        # ── Sub-systems ──────────────────────────────────────────────────
        self.mask_builder = MaskBuilder(
            link_mesh_samples=link_mesh_samples,
            R_base=self.R_base,
            t_base=self.t_base,
            ee_link=self.ee_link,
            mask_cfg=self.mask_cfg,
            logger=self.get_logger(),
        )
        self.distance_engine = DistanceEngine(
            distance_cfg=self.distance_cfg,
            logger=self.get_logger(),
        )

        # ── Throttle / profiling ─────────────────────────────────────────
        self._THROTTLE_S         = 2.0
        self._last_no_obs_warn_t = 0.0
        self._last_dist_log_t    = 0.0
        self._process_skip_count = 0
        self._perf               = PerfTimer()

        # ── Subscriptions ────────────────────────────────────────────────
        topics_cfg  = self.config.get('topics', {})
        depth_topic = topics_cfg.get('depth_image',
                                     '/camera/camera/depth/image_rect_raw')
        info_topic  = topics_cfg.get('depth_camera_info',
                                     '/camera/camera/depth/camera_info')
        self.create_subscription(Image,      depth_topic, self.depth_callback,       10)
        self.create_subscription(CameraInfo, info_topic,  self.camera_info_callback, 10)

        # ── Publishers ───────────────────────────────────────────────────
        _be_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.multi_dist_pub = self.create_publisher(
            MultiDistance,
            topics_cfg.get('multi_distance', '/human_robot/multi_distance'), 10)
        self.dist_pub = self.create_publisher(
            HumanRobotDistance,
            topics_cfg.get('distance', '/human_robot/distance'), 10)
        self.per_link_dist_pub = self.create_publisher(
            MultiLinkDistance, '/cbf/per_link_distances', _be_qos)

        self.get_logger().info(
            f'RealTimeDistance (compute) ready — '
            f'mode=control_point  '
            f'ee_link={self.ee_link}')

        self._compute_thread = threading.Thread(
            target=self._compute_loop, name='rtd_compute', daemon=True)
        self._compute_thread.start()

    # ── Camera callbacks ──────────────────────────────────────────────────────

    def depth_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
            frame = (cv_image, msg)
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
        if self.fx is not None:
            return
        if msg.k[0] == 0.0:
            self.get_logger().warn(
                'camera_info_callback: degenerate K (fx=0), skipping')
            return
        try:
            K     = np.array(msg.k, dtype=float).reshape(3, 3)
            K_inv = np.linalg.inv(K)
        except Exception as exc:
            self.get_logger().warn(
                f'camera_info_callback: K inversion failed ({exc}), skipping')
            return
        self.K     = K
        self.K_inv = K_inv
        self.fx = msg.k[0];  self.fy = msg.k[4]
        self.cx = msg.k[2];  self.cy = msg.k[5]
        self.fx_inv_f32 = np.float32(1.0 / self.fx)
        self.fy_inv_f32 = np.float32(1.0 / self.fy)
        self.cx_f32     = np.float32(self.cx)
        self.cy_f32     = np.float32(self.cy)
        self.mask_builder.set_intrinsics(K)
        self.get_logger().info(
            f'Camera intrinsics set: fx={self.fx:.1f}  fy={self.fy:.1f}  '
            f'cx={self.cx:.1f}  cy={self.cy:.1f}')

    # ── Compute loop ──────────────────────────────────────────────────────────

    def _compute_loop(self):
        """Background daemon thread: blocks on queue for freshest frame."""
        while rclpy.ok():
            try:
                frame = self._frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                if self.fx is not None:
                    self._process_depth_impl(frame)
            except Exception as exc:
                self._process_skip_count += 1
                self.get_logger().error(
                    f'compute error (skip #{self._process_skip_count}): {exc}',
                    throttle_duration_sec=2.0)

    def _process_depth_impl(self, frame: tuple):
        depth, depth_msg = frame
        stamp = depth_msg.header.stamp

        # ── Depth resolution guard ────────────────────────────────────────
        if self._last_depth_shape != depth.shape:
            self._last_depth_shape = depth.shape
            self.mask_builder.invalidate()
            self.distance_engine.invalidate_grid_cache()
            self.distance_engine.reset_lpf()

        H, W   = depth.shape
        step   = int(self.distance_cfg['pixel_step'])
        margin = int(self.distance_cfg['image_margin_px'])

        # ── TF (best-effort for mask, strict for distance) ────────────────
        with self._perf('tf'):
            mask_transforms = self.tf_mgr.lookup_best_effort(
                self.robot_cfg['segment_links'], stamp)
            transforms = self.tf_mgr.lookup_all(
                self.robot_cfg['segment_links'], stamp)

        # ── Mask — always rebuilt ─────────────────────────────────────────
        mask_tf = transforms or mask_transforms or self._last_transforms
        if mask_tf:
            with self._perf('mask'):
                self.mask_builder.rebuild(mask_tf, depth.shape)

        if transforms is not None:
            self._last_transforms = transforms

        # ── Distance pipeline (strict TF required) ────────────────────────
        if transforms is None:
            if self._last_transforms is not None:
                self._publish_fallback(
                    float(self.distance_cfg.get('fallback_distance', 2.0)), stamp)
            return

        control_points = define_control_points(
            transforms, self.robot_cfg, self.distance_cfg)
        if not control_points:
            return

        roi_bounds = compute_roi(
            self.mask_builder.search_exclusion_mask, H, W, margin,
            int(self.distance_cfg['roi_pad_px']))
        x0, y0, x1, y1 = roi_bounds
        x, y = np.array([x0, x1]), np.array([y0, y1])

        thresholds        = self.distance_cfg['thresholds']
        fallback_distance = float(self.distance_cfg.get('fallback_distance', 2.0))

        with self._perf('dist'):
            cp_results, n_pts = self.distance_engine.compute(
                depth=depth,
                cx_f32=self.cx_f32,
                cy_f32=self.cy_f32,
                fx_inv_f32=self.fx_inv_f32,
                fy_inv_f32=self.fy_inv_f32,
                R_base_f32=self.R_base_f32,
                t_base_f32=self.t_base_f32,
                control_points=control_points,
                x=x, y=y, step=step,
                search_exclusion_mask=self.mask_builder.search_exclusion_mask,
                transforms=transforms,
            )
        if cp_results is None:
            return

        valid = [
            r for r in cp_results
            if np.isfinite(r.distance)
            and thresholds['min_thresh'] <= r.distance <= thresholds['max_thresh']
        ]
        if not valid:
            self._last_no_obs_warn_t = no_obs_warn(
                self.get_logger(), self._last_no_obs_warn_t,
                self._THROTTLE_S, fallback_distance, 'CP')
            self._publish_fallback(fallback_distance, stamp)
            return

        best_cp        = min(valid, key=lambda r: r.distance)
        min_dist       = best_cp.distance
        closest_uv_obs = best_cp.closest_pixel

        closest_Z = base_to_cam_z(
            best_cp.closest_obstacle_point, self.R_base, self.t_base)

        # ── Throttled log ─────────────────────────────────────────────────
        now = time.monotonic()
        if now - self._last_dist_log_t >= self._THROTTLE_S:
            self._last_dist_log_t = now
            self.get_logger().info(
                f'dist={min_dist:.3f} m  Z={closest_Z:.3f} m  '
                f'pix={closest_uv_obs}  | {self._perf.summary()}')

        # ── Publish ───────────────────────────────────────────────────────
        multi_msg, mld_msg = build_cp_messages(
            cp_results=cp_results,
            n_pts=n_pts,
            stamp=stamp,
            frame_id=self.robot_cfg['base_frame'],
            segment_links=self.robot_cfg.get('segment_links', []),
            thresholds=thresholds,
            fallback=fallback_distance,
            zones=self.zones,
        )
        self.multi_dist_pub.publish(multi_msg)
        self.per_link_dist_pub.publish(mld_msg)

    # ── Fallback publisher ────────────────────────────────────────────────────

    def _publish_fallback(self, distance: float, stamp):
        msg = HumanRobotDistance()
        msg.header.stamp = stamp
        msg.distance     = float(distance)
        self.dist_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RealTimeDistance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
