#!/usr/bin/env python3
"""RealTimeDistance — real-time human-robot distance estimation (ROS2 Humble).

Architecture
------------
  TFManager      — 3-level TF fallback + critical-link validation
  MaskBuilder    — always-rebuilt robot mask + exclusion mask + contours
  DistanceEngine — depth-space per-CP distances (Flacco) with conservative LPF
  VisFrame       — immutable snapshot for lock-free compute/visualize handoff
  draw_overlay() — pure rendering function (no shared state)

Thread safety
-------------
depth_callback puts frames into a Queue(maxsize=1); old frames are dropped when
the compute thread is busy. _compute_loop blocks on queue.get() and always
processes the freshest frame. A 50ms ROS watchdog timer (_watchdog_check) runs
on the spin thread and publishes safety distances if mask computation stalls.
_vis_frame is swapped via GIL-atomic reference assignment; visualize() reads the
reference under _vis_lock then operates exclusively on its local snapshot.

Visualization note
------------------
cv2.imshow() is intentionally called inside the visualize timer callback.
In headless deployments set booleans.visualize: false in the YAML config.
For a fully decoupled GUI, consume the /real_time_distance/overlay_image topic
from a standalone process.

Behavior changes vs. previous version
--------------------------------------
- Overlay image header stamp now uses the depth-frame timestamp (previously
  used get_clock().now()). This is more semantically correct.
- ee_link is now read from robot.ee_link in the YAML (default 'fr3_link8').
- Critical TF links are validated before each frame (missing EE → skip frame).
- cp_results items are ControlPointResult dataclasses, not dicts.
"""
from __future__ import annotations

import math
import os
import queue
import threading
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
import trimesh
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from franka_msgs.msg import (
    HumanRobotDistance, LinkDistance, MultiDistance, MultiLinkDistance)
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from franka_experiments.utils.distance_engine import ControlPointResult, DistanceEngine
from franka_experiments.utils.distance_utils import (
    compute_closest_distance_from_segments,
    define_robot_segments,
    find_pt_confidence,
    load_extrinsics,
    load_robot_config,
)
from franka_experiments.utils.mask_builder import MaskBuilder
from franka_experiments.utils.tf_manager import TFManager
from franka_experiments.utils.visualization import VisFrame, draw_overlay


# ── Lightweight profiling ─────────────────────────────────────────────────────

class _PerfTimer:
    """Accumulates wall-clock timings for a set of named stages."""

    def __init__(self):
        self._ms: dict[str, float] = {}

    def __call__(self, key: str) -> '_TimerCtx':
        return _TimerCtx(self._ms, key)

    def summary(self) -> str:
        return '  '.join(f'{k}={v:.1f}ms' for k, v in self._ms.items())


class _TimerCtx:
    def __init__(self, store: dict, key: str):
        self._s = store
        self._k = key

    def __enter__(self):
        self._t0 = time.perf_counter()
        return self

    def __exit__(self, *_):
        self._s[self._k] = (time.perf_counter() - self._t0) * 1000.0


# ── Node ──────────────────────────────────────────────────────────────────────

class RealTimeDistance(Node):

    def __init__(self):
        super().__init__('real_time_distance')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('robot_config_path', '')
        self.declare_parameter('camera_extrinsics_path', '')
        self.declare_parameter('publish_overlay_image', False)
        self.declare_parameter('process_rate_hz', 20.0)  # kept for launch-file compat
        self.declare_parameter('compute_rate_hz', 20.0)  # kept for launch-file compat

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

        # Configurable EE link — defaults to 'fr3_link8' when not in YAML
        self.ee_link = self.robot_cfg.get('ee_link', 'fr3_link8')

        self.R_base, self.t_base = load_extrinsics(camera_extrinsics_path)
        self.R_base_f32 = self.R_base.astype(np.float32)
        self.t_base_f32 = self.t_base.astype(np.float32)

        # ── Flags ────────────────────────────────────────────────────────
        booleans = self.config.get('booleans', {})
        self.enable_visualization        = booleans.get('visualize', False)
        self.visual_robot_exclusion_mask = booleans.get('exclusion_mask', True)
        self.visualize_only_raw_video    = booleans.get('raw_video', False)
        self.use_segment_distance        = booleans.get('use_segment_distance', False)
        self.visual_ROI                  = booleans.get('visual_ROI', False)

        self.publish_overlay_image = (
            self.get_parameter('publish_overlay_image').value
            or booleans.get('publish_overlay_image', False)
        )

        self.min_seg_idx_for_distance = 3

        # ── Camera intrinsics (set by camera_info_callback) ──────────────
        self.bridge    = CvBridge()
        self.K         = None
        self.K_inv     = None
        self.fx = self.fy = None
        self.cx = self.cy = None
        self.cx_f32 = self.cy_f32 = None
        self.fx_inv_f32 = self.fy_inv_f32 = None

        # ── Depth frame queue ─────────────────────────────────────────────
        # depth_callback drops the stale frame and puts the new one (maxsize=1).
        # _compute_loop blocks on queue.get() — always processes freshest frame.
        self._frame_queue      = queue.Queue(maxsize=1)
        self._last_depth_shape: Optional[tuple] = None
        self._last_mask_t      = 0.0   # updated after each successful mask rebuild

        # ── Visualization snapshot ────────────────────────────────────────
        # process_depth() builds a VisFrame and assigns it under _vis_lock.
        # visualize() acquires _vis_lock briefly to read the reference,
        # then releases and works exclusively on its local copy.
        self._vis_frame: Optional[VisFrame] = None
        self._vis_lock  = threading.Lock()

        # ── TF ───────────────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        critical_links = self.robot_cfg.get('critical_links', [self.ee_link])
        self.tf_mgr = TFManager(
            tf_buffer=self.tf_buffer,
            base_frame=self.robot_cfg['base_frame'],
            critical_links=critical_links,
            throttle_s=2.0,
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

        # ROI bounds (updated when mask rebuilds; lives here so process_depth
        # can reset it on camera resolution change)
        self.roi_bounds: Optional[tuple] = None

        # ── Throttle / profiling ─────────────────────────────────────────
        self._THROTTLE_S         = 2.0
        self._last_no_obs_warn_t = 0.0
        self._last_dist_log_t    = 0.0
        self._process_skip_count = 0
        self._vis_skip_count     = 0
        self._perf               = _PerfTimer()

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
            f'RealTimeDistance ready — '
            f'mode={"segment" if self.use_segment_distance else "control_point"}  '
            f'viz={self.enable_visualization}  ee_link={self.ee_link}')

        self.create_timer(0.05, self._watchdog_check)
        self._compute_thread = threading.Thread(
            target=self._compute_loop, name='rtd_compute', daemon=True)
        self._compute_thread.start()
        if self.enable_visualization or self.publish_overlay_image:
            overlay_topic = topics_cfg.get(
                'overlay_image', '/real_time_distance/overlay_image')
            self.overlay_pub = self.create_publisher(Image, overlay_topic, 10)
            self.create_timer(0.1, self.visualize)

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
                    self._frame_queue.get_nowait()   # drop stale frame
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
        # All fields set atomically — no partial-initialisation window
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

    # ── Main processing loop ──────────────────────────────────────────────────

    def _compute_loop(self):
        """Background daemon thread: blocks on queue for freshest frame, then processes."""
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
                self._publish_safety_distance()

    def _watchdog_check(self):
        """ROS timer (50 ms): publish safety distance if mask computation has stalled."""
        if self._last_mask_t <= 0.0:
            return
        dt = time.monotonic() - self._last_mask_t
        if dt > 0.05:
            self._publish_safety_distance()
            self.get_logger().warn(
                f'Mask stale ({dt*1000:.0f} ms) — publishing safety distance',
                throttle_duration_sec=1.0)

    def _publish_safety_distance(self):
        """Publish distance=0 on all output topics to halt the robot via CBF."""
        stamp    = self.get_clock().now().to_msg()
        frame_id = self.robot_cfg['base_frame']

        d_msg          = HumanRobotDistance()
        d_msg.header.stamp = stamp
        d_msg.distance = 0.0
        self.dist_pub.publish(d_msg)

        link_entries = []
        for lk in self.robot_cfg.get('segment_links', []):
            ld                 = LinkDistance()
            ld.robot_link_name = lk
            ld.distance        = 0.0
            ld.valid           = True
            ld.confidence      = 1.0
            ld.zone            = 'critical'
            link_entries.append(ld)

        mld_msg                 = MultiLinkDistance()
        mld_msg.header.stamp    = stamp
        mld_msg.header.frame_id = frame_id
        mld_msg.links           = link_entries
        self.per_link_dist_pub.publish(mld_msg)

    def _process_depth_impl(self, frame: tuple):
        depth, depth_msg = frame
        stamp = depth_msg.header.stamp

        # ── Depth resolution guard ────────────────────────────────────────
        if self._last_depth_shape != depth.shape:
            self._last_depth_shape = depth.shape
            self.mask_builder.invalidate()
            self.distance_engine.invalidate_grid_cache()
            self.distance_engine.reset_lpf()
            self.roi_bounds = None

        H, W   = depth.shape
        step   = int(self.distance_cfg['pixel_step'])
        margin = int(self.distance_cfg['image_margin_px'])

        # ── TF ────────────────────────────────────────────────────────────
        with self._perf('tf'):
            transforms = self.tf_mgr.lookup_all(
                self.robot_cfg['segment_links'], stamp)
        if transforms is None:
            return

        # ── Control points + capsule segments ─────────────────────────────
        control_points = self._define_control_points(transforms)
        robot_segments = define_robot_segments(
            transforms, self.robot_cfg, self.distance_cfg)
        if robot_segments is None:
            return
        robot_segments = [
            s for s in robot_segments
            if s['seg_idx'] >= self.min_seg_idx_for_distance
        ]
        if not robot_segments:
            return

        # ── Mask + ROI (rebuilt every frame) ──────────────────────────────
        with self._perf('mask'):
            self.mask_builder.rebuild(transforms, depth.shape)
            self.roi_bounds = self._compute_roi(
                self.mask_builder.search_exclusion_mask, H, W, margin)
            self._last_mask_t = time.monotonic()

        if self.roi_bounds is None:
            self.roi_bounds = (margin, margin, W - margin, H - margin)
        x0, y0, x1, y1 = self.roi_bounds
        x, y = np.array([x0, x1]), np.array([y0, y1])

        thresholds        = self.distance_cfg['thresholds']
        fallback_distance = float(self.distance_cfg.get('fallback_distance', 2.0))

        # ── Distance computation ──────────────────────────────────────────
        if self.use_segment_distance:
            with self._perf('dist'):
                best_seg, n_pts = compute_closest_distance_from_segments(
                    last_depth=depth,
                    K_inv=self.K_inv,
                    transform_camera_to_base_fn=self._cam_to_base,
                    robot_segments=robot_segments,
                    x=x, y=y, step=step,
                    search_exclusion_mask=self.mask_builder.search_exclusion_mask,
                    distance_cfg=self.distance_cfg,
                )
            if not self._in_range(best_seg, thresholds, key='distance'):
                self._no_obs_warn(fallback_distance, 'segment')
                self._publish_fallback(fallback_distance, stamp)
                return
            min_dist         = float(best_seg['distance'])
            closest_obs_pt   = best_seg['closest_obstacle_point']
            closest_robot_pt = best_seg['point']
            closest_uv_obs   = best_seg['closest_pixel']
            cp_results       = None

        else:
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
                self._no_obs_warn(fallback_distance, 'CP')
                self._publish_fallback(fallback_distance, stamp)
                return

            best_cp          = min(valid, key=lambda r: r.distance)
            min_dist         = best_cp.distance
            closest_obs_pt   = best_cp.closest_obstacle_point
            closest_robot_pt = best_cp.point
            closest_uv_obs   = best_cp.closest_pixel

        # ── Guard: obstacle point may be None in edge cases ───────────────
        closest_Z = self._base_to_cam_z(closest_obs_pt)

        # ── Throttled log ─────────────────────────────────────────────────
        now = time.monotonic()
        if now - self._last_dist_log_t >= self._THROTTLE_S:
            self._last_dist_log_t = now
            self.get_logger().info(
                f'dist={min_dist:.3f} m  Z={closest_Z:.3f} m  '
                f'pix={closest_uv_obs}  | {self._perf.summary()}')

        # ── Publish ───────────────────────────────────────────────────────
        if cp_results is not None:
            self._publish_cp_results(
                cp_results, n_pts, stamp, thresholds, fallback_distance)

        # ── Visualisation snapshot (single reference swap) ─────────────────
        new_frame = VisFrame(
            depth=depth,
            robot_mask=self.mask_builder.robot_mask,
            contours=self.mask_builder.contours,
            robot_segments=robot_segments,
            cp_results=cp_results,
            closest_robot_pt=closest_robot_pt,
            closest_uv_obs=closest_uv_obs,
            min_dist=min_dist,
            roi_bounds=self.roi_bounds,
            use_segment_mode=self.use_segment_distance,
            stamp=stamp,
            visual_ROI=self.visual_ROI,
            visual_exclusion_mask=self.visual_robot_exclusion_mask,
            visualize_only_raw_video=self.visualize_only_raw_video,
        )
        with self._vis_lock:
            self._vis_frame = new_frame

    # ── Visualisation ─────────────────────────────────────────────────────────

    def visualize(self):
        try:
            self._visualize_impl()
        except Exception as exc:
            self._vis_skip_count += 1
            self.get_logger().warn(
                f'visualize error (skip #{self._vis_skip_count}): {exc}',
                throttle_duration_sec=2.0)

    def _visualize_impl(self):
        with self._vis_lock:
            frame = self._vis_frame          # read reference; release lock immediately
        if frame is None or self.K is None:
            return

        with self._perf('vis'):
            depth_vis = draw_overlay(
                frame=frame,
                zones=self.zones,
                K=self.K,
                R_base=self.R_base,
                t_base=self.t_base,
                depth_shape=frame.depth.shape,
            )

        self._publish_overlay(depth_vis, frame.stamp)

        if self.enable_visualization:
            cv2.imshow('Robot + closest distance', depth_vis)
            cv2.waitKey(1)

    def _publish_overlay(self, img: np.ndarray, stamp):
        if not self.publish_overlay_image:
            return
        try:
            msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            msg.header.stamp    = stamp        # depth frame timestamp (more correct than now())
            msg.header.frame_id = 'camera_depth_optical_frame'
            self.overlay_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'overlay publish error: {exc}')

    # ── Control points ────────────────────────────────────────────────────────

    def _define_control_points(self, transforms: dict) -> list:
        ee_tip_axis   = self.distance_cfg['ee_tip_axis']
        ee_tip_offset = self.distance_cfg['ee_tip_offset']
        control_points = []

        for seg in self.robot_cfg['segments']:
            seg_idx    = int(seg['seg_idx'])
            start_link = seg['start_link']
            end_link   = seg['end_link']
            n_cp       = int(seg.get('control_points', 0))
            radius     = float(seg.get('radius', 0.0))

            if n_cp <= 0:
                continue
            if start_link not in transforms or end_link not in transforms:
                continue

            _, p0 = transforms[start_link]
            _, p1 = transforms[end_link]

            ts = [(k + 1) / (n_cp + 1) for k in range(n_cp)]
            if end_link == self.ee_link:
                ts = [1.0] if n_cp == 1 else [(k + 1) / n_cp for k in range(n_cp)]

            for k, t in enumerate(ts):
                p = p0 + t * (p1 - p0)
                if end_link == self.ee_link and np.isclose(t, 1.0):
                    R_ee, p_ee = transforms[self.ee_link]
                    p = p_ee + ee_tip_offset * R_ee[:, ee_tip_axis]

                control_points.append({
                    'point':      p,
                    'seg_idx':    seg_idx,
                    'cp_idx':     k,
                    'radius':     radius,
                    'start_link': start_link,
                    'end_link':   end_link,
                })

        return control_points

    # ── Publishing helpers ────────────────────────────────────────────────────

    def classify_zone(self, distance: float) -> str:
        if not self.zones:
            return 'unknown'
        if distance <= self.zones.get('critical', 0.1):
            return 'critical'
        if distance <= self.zones.get('danger', 0.2):
            return 'danger'
        if distance <= self.zones.get('warning', 0.3):
            return 'warning'
        return 'safe'

    def _publish_fallback(self, distance: float, stamp):
        msg = HumanRobotDistance()
        msg.header.stamp = stamp
        msg.distance     = float(distance)
        self.dist_pub.publish(msg)

    def _publish_cp_results(
        self,
        cp_results: list,
        n_pts: int,
        stamp,
        thresholds: dict,
        fallback: float,
    ):
        frame_id      = self.robot_cfg['base_frame']
        min_thresh    = thresholds['min_thresh']
        max_thresh    = thresholds['max_thresh']
        segment_links = self.robot_cfg.get('segment_links', [])

        # Zone thresholds pre-bound — avoids repeated dict.get() inside loops
        zones_exist = bool(self.zones)
        z_critical  = self.zones.get('critical', 0.1)
        z_danger    = self.zones.get('danger',   0.2)
        z_warning   = self.zones.get('warning',  0.3)

        def _zone(d: float) -> str:
            if not zones_exist:
                return 'unknown'
            if d <= z_critical:
                return 'critical'
            if d <= z_danger:
                return 'danger'
            if d <= z_warning:
                return 'warning'
            return 'safe'

        # Single pass: build seg_best and link_best simultaneously
        seg_best:  dict[int, ControlPointResult] = {}
        link_best: dict[str, ControlPointResult] = {}
        for r in cp_results:
            s = r.seg_idx
            if s not in seg_best or r.distance < seg_best[s].distance:
                seg_best[s] = r
            lk = r.end_link
            if lk not in link_best or r.distance < link_best[lk].distance:
                link_best[lk] = r

        # ── MultiDistance: one entry per segment ──────────────────────────
        # seg_best keys are in insertion order, which matches ascending seg_idx
        # because cp_results is produced in config segment order by
        # _define_control_points — no sorted() allocation needed.
        entries = []
        for r in seg_best.values():
            msg = HumanRobotDistance()
            msg.header.stamp    = stamp
            msg.header.frame_id = frame_id
            msg.robot_link_name = r.end_link
            d  = r.distance
            di = r.direction
            if math.isfinite(d) and min_thresh <= d <= max_thresh and di is not None:
                pt = r.point
                msg.valid    = True
                msg.distance = d
                msg.closest_point_robot = Point(
                    x=float(pt[0]), y=float(pt[1]), z=float(pt[2]))
                msg.direction = Vector3(
                    x=float(di[0]), y=float(di[1]), z=float(di[2]))
                msg.zone       = _zone(d)
                msg.confidence = float(find_pt_confidence(d, n_pts))
            else:
                msg.valid    = False
                msg.distance = fallback
            entries.append(msg)

        multi_msg = MultiDistance()
        multi_msg.header.stamp    = stamp
        multi_msg.header.frame_id = frame_id
        multi_msg.distances       = entries
        self.multi_dist_pub.publish(multi_msg)

        # ── MultiLinkDistance: one entry per link ─────────────────────────
        link_entries = []
        for lk in segment_links:
            if lk not in link_best:
                continue
            r   = link_best[lk]
            d   = r.distance
            di  = r.direction
            pt  = r.point
            obs = r.closest_obstacle_point
            ld  = LinkDistance()
            ld.robot_link_name = lk
            if pt is not None:
                ld.closest_point_robot = Point(
                    x=float(pt[0]), y=float(pt[1]), z=float(pt[2]))
            if obs is not None:
                ld.closest_point_human = Point(
                    x=float(obs[0]), y=float(obs[1]), z=float(obs[2]))
            if di is not None:
                ld.direction = Vector3(
                    x=float(di[0]), y=float(di[1]), z=float(di[2]))
            ld.distance   = d
            ld.valid      = math.isfinite(d) and d > 0.0 and di is not None
            ld.confidence = 1.0
            ld.zone       = _zone(d)
            link_entries.append(ld)

        mld_msg = MultiLinkDistance()
        mld_msg.header.stamp    = stamp
        mld_msg.header.frame_id = frame_id
        mld_msg.links           = link_entries
        self.per_link_dist_pub.publish(mld_msg)

    # ── Geometry helpers ──────────────────────────────────────────────────────

    def _cam_to_base(self, p_cam) -> np.ndarray:
        return self.R_base @ np.asarray(p_cam) + self.t_base

    def _base_to_cam_z(self, p_base) -> float:
        if p_base is None:
            return 0.0
        p_cam = self.R_base.T @ (np.asarray(p_base) - self.t_base)
        return float(p_cam[2])

    def _compute_roi(
        self,
        exclusion_mask,
        H: int,
        W: int,
        margin: int,
    ) -> tuple:
        roi_pad = int(self.distance_cfg['roi_pad_px'])
        if exclusion_mask is None:
            return (margin, margin, W - margin, H - margin)
        ys, xs = np.where(exclusion_mask)
        if xs.size == 0 or ys.size == 0:
            return (margin, margin, W - margin, H - margin)
        return (
            max(margin,          int(xs.min()) - roi_pad),
            max(margin,          int(ys.min()) - roi_pad),
            min(W - margin,      int(xs.max()) + roi_pad),
            min(H - margin,      int(ys.max()) + roi_pad),
        )

    # ── Misc ─────────────────────────────────────────────────────────────────

    def _in_range(self, result, thresholds: dict, key: str = 'distance') -> bool:
        if result is None:
            return False
        d = result[key] if isinstance(result, dict) else getattr(result, key)
        return thresholds['min_thresh'] <= float(d) <= thresholds['max_thresh']

    def _no_obs_warn(self, fallback: float, mode: str):
        now = time.monotonic()
        if now - self._last_no_obs_warn_t >= self._THROTTLE_S:
            self._last_no_obs_warn_t = now
            self.get_logger().debug(
                f'No near obstacle ({mode} mode). Fallback={fallback} m')


def main(args=None):
    rclpy.init(args=args)
    node = RealTimeDistance()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
