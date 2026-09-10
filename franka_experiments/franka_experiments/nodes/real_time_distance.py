#!/usr/bin/env python3
"""RealTimeDistance — real-time human-robot distance estimation (ROS2 Humble).

Architecture
------------
  TFManager      — 3-level TF fallback + critical-link validation
  MaskBuilder    — always-rebuilt robot mask + exclusion mask + contours
  DistanceEngine — depth-space per-CP distances (Flacco) with conservative LPF
  ObstacleTrackPipeline — cluster → track → 3D obstacle velocity (OPTIONAL)
  VisFrame       — immutable snapshot for lock-free compute/visualize handoff
  draw_overlay() — pure rendering function (no shared state)

Obstacle tracking (``tracking.enabled``, OFF by default)
--------------------------------------------------------
When on, the obstacle pixels this node has ALREADY selected are additionally
clustered and tracked, and each published ``LinkDistance`` carries the 3D
velocity (with covariance) of the object its nearest obstacle point belongs to.

It lives HERE, rather than in a node of its own, for one reason that is not
convenience: the clusters must be built from EXACTLY the pixels the distance
pass selected — same exclusion mask, same depth-range filter, same ROI stride —
or the tracker would report velocities for obstacles the barrier never saw. A
separate node has to either re-derive that selection (and drift from it the
first time either side is touched) or re-run this node's whole pipeline to
recover it. Here the point cloud is already in hand.

The fields are APPENDED to the existing message on the existing topic. With
tracking off they are the documented all-zero "no track" state, which every
consumer already treats as "contribute nothing", so the published topic is
unchanged for anyone who does not ask for it.

Thread safety
-------------
depth_callback puts frames into a Queue(maxsize=1); old frames are dropped when
the compute thread is busy.  _compute_loop blocks on queue.get() and always
processes the freshest frame.  _vis_frame is swapped via GIL-atomic reference
assignment; visualize() reads the reference under _vis_lock then operates
exclusively on its local snapshot.

All topic names, distance thresholds, mesh paths, and timing parameters are
read from the YAML config file passed via the ``robot_config_path`` ROS
parameter.  No value is hardcoded in this module.
"""
from __future__ import annotations

import os
import queue
import threading
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
import trimesh
import yaml
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
from franka_experiments.utils.logging_utils import ThrottledLogger
from franka_experiments.utils.mask_builder import MaskBuilder
from franka_experiments.utils.params import declare_bool, declare_str
from franka_experiments.utils.logging_utils import PerfTimer
from franka_experiments.utils.obstacle_sim import InjectedSphere
from franka_experiments.utils.calibration_check import calibration_residual
from franka_experiments.utils.self_detection import SelfDetectionMonitor
from franka_experiments.utils.obstacle_track_pipeline import ObstacleTrackPipeline
from franka_experiments.utils.perception_msgs import (
    annotate_track_fields, build_cp_messages)
from franka_experiments.utils.tf_manager import TFManager
from franka_experiments.utils.visualization import VisFrame, draw_overlay


class RealTimeDistance(Node):

    def __init__(self):
        super().__init__('real_time_distance')

        # ── Parameters ──────────────────────────────────────────────────
        # declare_str(allow_empty=False) logs at ERROR naming the parameter and
        # the received value, then raises — replacing the two bare RuntimeErrors
        # that used to be raised below without any log line.
        robot_config_path      = declare_str(self, 'robot_config_path', '')
        camera_extrinsics_path = declare_str(self, 'camera_extrinsics_path', '')
        _publish_overlay_param = declare_bool(self, 'publish_overlay_image', False)
        # publish_empty_per_link (liveness heartbeat) is declared further down,
        # once distance_cfg is loaded, so its default can come from the YAML.

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

        # ── Flags ────────────────────────────────────────────────────────
        booleans = self.config.get('booleans', {})
        self.enable_visualization        = booleans.get('visualize', False)
        self.visual_robot_exclusion_mask = booleans.get('exclusion_mask', True)
        self.visualize_only_raw_video    = booleans.get('raw_video', False)
        self.visual_ROI                  = booleans.get('visual_ROI', False)
        self.publish_overlay_image       = (
            _publish_overlay_param
            or booleans.get('publish_overlay_image', False)
        )
        self._publish_empty_per_link     = declare_bool(
            self, 'publish_empty_per_link',
            self.distance_cfg.get('publish_empty_per_link', True))

        # ── Camera intrinsics (populated by camera_info_callback) ────────
        self.bridge    = CvBridge()
        self.K         = None
        self.K_inv     = None
        self.fx = self.fy = None
        self.cx = self.cy = None
        self.cx_f32 = self.cy_f32 = None
        self.fx_inv_f32 = self.fy_inv_f32 = None

        # ── Frame queue + visualisation snapshot ─────────────────────────
        self._frame_queue      = queue.Queue(maxsize=1)
        self._last_depth_shape: Optional[tuple] = None
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
        # ── Obstacle tracking (optional) ──────────────────────────────────
        # Config lives in the `tracking:` block of the robot config, so every
        # perception knob is in one file. The export flag is set on the COPY
        # handed to the engine and only when tracking is on, so with tracking
        # off the engine stores nothing and costs nothing.
        trk_cfg = self.config.get('tracking', {}) or {}
        self.tracking_enabled = bool(trk_cfg.get('enabled', False))
        self.distance_cfg = dict(self.distance_cfg)
        self.distance_cfg['export_obstacle_cloud'] = self.tracking_enabled

        # ── Multi-obstacle rows (perception.multi_obstacle_k) ─────────────
        # The flag and the cbf_obstacle_horizon it prunes against live in the
        # CONTROL config (fr3_control.yaml), not in robot_config_path, so that
        # file is read here. Unreadable -> k = 1, i.e. today's output.
        ctrl_path = (self.declare_parameter('control_config_path', '').value
                     or os.path.join(get_package_share_directory('franka_experiments'),
                                     'config', 'fr3_control.yaml'))
        try:
            with open(ctrl_path) as f:
                ctrl_cfg = yaml.safe_load(f) or {}
        except Exception as exc:
            self.get_logger().warn(
                f'control config {ctrl_path!r} not read ({exc}); multi_obstacle_k=1')
            ctrl_cfg = {}
        perc_cfg = ctrl_cfg.get('perception', {}) or {}
        # ROS parameter override (launch arg multi_obstacle_k); 0 = use the YAML.
        k_param = int(self.declare_parameter('multi_obstacle_k', 0).value)
        self.multi_k = max(1, k_param if k_param > 0
                           else int(perc_cfg.get('multi_obstacle_k', 1)))
        self.distance_cfg['multi_obstacle_k'] = self.multi_k
        self.distance_cfg['multi_obstacle_max_rows'] = int(
            perc_cfg.get('multi_obstacle_max_rows', 24))
        self.distance_cfg['multi_obstacle_horizon'] = float(
            (ctrl_cfg.get('params', {}) or {}).get('cbf_obstacle_horizon', float('inf')))
        # Same depth_jump as the tracker: with k > 1 the tracker clusters on
        # the engine's labels, so there is one labelling, not two.
        self.distance_cfg['multi_obstacle_depth_jump_m'] = float(
            trk_cfg.get('cluster_depth_jump_m', 0.10))

        self.distance_engine = DistanceEngine(
            distance_cfg=self.distance_cfg,
            logger=self.get_logger(),
        )

        # Self-detection guard. Runs whenever tracking does, because it is the
        # tracker that makes self-detection dangerous rather than merely
        # annoying: the arm's own body becomes a cluster with a real velocity,
        # that velocity is fed back as v_obs, the barrier tightens because the
        # arm is moving, the arm brakes, v_obs drops, and the filter chases
        # itself. See utils/self_detection.py.
        self.self_detect = None
        self.track_pipeline = None
        if self.tracking_enabled:
            sd = trk_cfg.get('self_detection', {}) or {}
            if sd.get('enabled', True):
                self.self_detect = SelfDetectionMonitor(
                    window=int(sd.get('window', 20)),
                    motion_min_m=float(sd.get('motion_min_m', 0.08)),
                    offset_tol_m=float(sd.get('offset_tol_m', 0.02)),
                    confirm=int(sd.get('confirm', 5)),
                    release=int(sd.get('release', 15)))
            self.track_pipeline = ObstacleTrackPipeline(
                voxel_m=float(trk_cfg.get('cluster_voxel_m', 0.02)),
                min_cluster_points=int(trk_cfg.get('cluster_min_points', 10)),
                max_clusters=int(trk_cfg.get('max_clusters', 16)),
                max_cluster_radius=(float(trk_cfg['cluster_max_radius_m'])
                                    if trk_cfg.get('cluster_max_radius_m')
                                    else None),
                depth_jump=float(trk_cfg.get('cluster_depth_jump_m', 0.10)),
                contains_tol=float(trk_cfg.get('cluster_contains_tol_m', 0.05)),
                q_jerk=float(trk_cfg.get('q_jerk', 2.0)),
                sigma_meas=float(trk_cfg.get('sigma_meas_m', 0.01)),
                gate_mahalanobis=float(trk_cfg.get('gate_mahalanobis', 3.0)),
                gate_max_m=float(trk_cfg.get('gate_max_m', 0.5)),
                confirm_hits=int(trk_cfg.get('confirm_hits', 3)),
                confirm_window=int(trk_cfg.get('confirm_window', 5)),
                max_missed=int(trk_cfg.get('max_missed', 5)),
                max_tracks=int(trk_cfg.get('max_tracks', 12)),
            )

        # ── Calibration drift check (utils/calibration_check) ─────────────
        # Runs at most once every `period_s`, so it is free. It compares the
        # projected robot model against the measured depth — the one comparison
        # this pipeline never makes, and the one whose failure turns the arm
        # into its own obstacle with no error anywhere.
        #
        # It reports a NUMBER, and only judges it when a baseline is configured:
        # the absolute value carries several centimetres of mesh-sampling bias
        # (measured, see the module docstring), so judging it against zero would
        # fire on a good calibration.
        cal_cfg = (self.config.get('tracking', {}) or {}).get(
            'calibration_check', {}) or {}
        self._cal_enabled = bool(cal_cfg.get('enabled', True))
        self._cal_period = float(cal_cfg.get('period_s', 10.0))
        self._cal_baseline = cal_cfg.get('baseline_m')
        self._cal_tol = float(cal_cfg.get('tolerance_m', 0.03))
        self._cal_next = 0.0
        self._link_samples = link_mesh_samples

        # ── Simulated obstacle (utils/obstacle_sim) ───────────────────────
        # Renders a sphere with a known trajectory INTO the depth frame before
        # anything else sees it, so the full avoidance chain can be exercised
        # end to end without a person in front of the arm.
        #
        # A fake obstacle inside a safety pipeline is exactly the thing that
        # must never be on by accident — the arm WILL move away from something
        # that is not there. Hence: off by default, and a loud warning on every
        # startup where it is on.
        sim_cfg = self.config.get('sim_obstacle', {}) or {}
        self.sim_sphere = None
        if bool(sim_cfg.get('enabled', False)):
            self.sim_sphere = InjectedSphere(
                c0=sim_cfg.get('start_cam', [0.0, 0.0, 1.6]),
                vel=sim_cfg.get('velocity_cam', [0.0, 0.0, -0.5]),
                radius=float(sim_cfg.get('radius_m', 0.15)),
                period=float(sim_cfg.get('period_s', 2.0)),
                amplitude=float(sim_cfg.get('amplitude_m', 0.5)),
            )
            self.get_logger().warn(
                'SIMULATED OBSTACLE ENABLED — a synthetic sphere is being '
                'rendered into the depth stream. The arm will react to an '
                'object that is not there. Set sim_obstacle.enabled=false '
                'before running with a person nearby.')

        self.roi_bounds: Optional[tuple] = None

        # ── Throttled logging (period from config) ────────────────────────
        _throttle_s = float(self.distance_cfg.get('log_throttle_s', 2.0))
        self._tlog_dist   = ThrottledLogger(self.get_logger(), period_s=_throttle_s)
        self._tlog_no_obs = ThrottledLogger(self.get_logger(), period_s=_throttle_s)

        self._process_skip_count = 0
        self._vis_skip_count     = 0
        self._perf               = PerfTimer()
        # True while the per-link topic carries empty heartbeats; cleared on every
        # real publish so the DEBUG line fires per transition, never per frame.
        self._hb_active = False

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
            MultiLinkDistance,
            topics_cfg.get('per_link_distances', '/cbf/per_link_distances'), _be_qos)

        # ── Overlay frame id (for published overlay image header) ─────────
        self._overlay_frame_id = topics_cfg.get(
            'depth_optical_frame', 'camera_depth_optical_frame')

        self.get_logger().info(
            f'RealTimeDistance ready — '
            f'mode=control_point  '
            f'viz={self.enable_visualization}  ee_link={self.ee_link}  '
            f'tracking={self.tracking_enabled}  '
            f'sim_obstacle={self.sim_sphere is not None}')

        self._compute_thread = threading.Thread(
            target=self._compute_loop, name='rtd_compute', daemon=True)
        self._compute_thread.start()

        if self.enable_visualization or self.publish_overlay_image:
            overlay_topic = topics_cfg.get(
                'overlay_image', '/real_time_distance/overlay_image')
            self.overlay_pub = self.create_publisher(Image, overlay_topic, 10)
            _vis_period = 1.0 / float(self.distance_cfg.get('vis_rate_hz', 10.0))
            self.create_timer(_vis_period, self.visualize)

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

        # Simulated obstacle FIRST, so every stage below — the exclusion mask,
        # the depth-range filter, the ROI stride, the distances, the clusters —
        # sees it exactly as it would see a real one. Copy rather than mutate:
        # the buffer belongs to cv_bridge and the visualiser reads it later.
        if self.sim_sphere is not None and self.K is not None:
            depth = depth.copy()
            self.sim_sphere.render(
                depth, stamp.sec + stamp.nanosec * 1e-9, self.K)

        # ── Depth resolution guard ────────────────────────────────────────
        if self._last_depth_shape != depth.shape:
            self._last_depth_shape = depth.shape
            self.mask_builder.invalidate()
            self.distance_engine.invalidate_grid_cache()
            self.distance_engine.reset_lpf()
            self.roi_bounds = None
            if self.self_detect is not None:
                self.self_detect.reset()
            if self.track_pipeline is not None:
                # Every track's position is in metres, but its ASSOCIATION was
                # built from a pixel grid that just changed shape. Keeping them
                # would carry one geometry's identities into another's.
                self.track_pipeline.reset()

        H, W   = depth.shape
        step   = int(self.distance_cfg['pixel_step'])
        margin = int(self.distance_cfg['image_margin_px'])

        # ── TF ────────────────────────────────────────────────────────────
        with self._perf('tf'):
            transforms = self.tf_mgr.lookup_all(
                self.robot_cfg['segment_links'], stamp)

        # ── Control points ────────────────────────────────────────────────
        control_points = define_control_points(
            transforms, self.robot_cfg, self.distance_cfg)
        if not control_points:
            self._publish_per_link_heartbeat(stamp)
            return

        # ── Mask + ROI ────────────────────────────────────────────────────
        with self._perf('mask'):
            self.mask_builder.rebuild(transforms, depth.shape)
            self.roi_bounds = compute_roi(
                self.mask_builder.search_exclusion_mask, H, W, margin,
                int(self.distance_cfg['roi_pad_px']))

        if self.roi_bounds is None:
            self.roi_bounds = (margin, margin, W - margin, H - margin)
        x0, y0, x1, y1 = self.roi_bounds
        x, y = np.array([x0, x1]), np.array([y0, y1])

        thresholds        = self.distance_cfg['thresholds']
        fallback_distance = float(self.distance_cfg.get('fallback_distance', 2.0))

        # ── Distance — Control Points pipeline ───────────────────────────
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
                ee_source_mask=self.mask_builder.ee_source_mask,
                dilation_margins_px=self.mask_builder.dilation_margins_px,
                frame_stamp=stamp.sec + stamp.nanosec * 1e-9,  # REAL dt for approach rate-limit
            )
        if cp_results is None:
            self._publish_per_link_heartbeat(stamp)
            return

        self._check_calibration(depth, transforms)

        # ── Cluster + track the SAME obstacle pixels the distances used ────
        # Fails soft on purpose: the distances are the primary safety signal and
        # must go out even if tracking blows up. A consumer that receives
        # distances with no track falls back to the residual estimator; one that
        # receives nothing brakes.
        if self.track_pipeline is not None:
            try:
                with self._perf('track'):
                    self.track_pipeline.update(
                        self.distance_engine.last_obstacle_cloud,
                        self.R_base, self.t_base,
                        stamp=stamp.sec + stamp.nanosec * 1e-9)
            except Exception as exc:
                self.get_logger().error(
                    f'obstacle tracking skipped this frame: {exc}',
                    throttle_duration_sec=2.0)

        valid = [
            r for r in cp_results
            if np.isfinite(r.distance)
            and thresholds['min_thresh'] <= r.distance <= thresholds['max_thresh']
        ]
        now = time.monotonic()
        if not valid:
            if self._tlog_no_obs.due(now):
                self._tlog_no_obs.debug(
                    f'No near obstacle (CP mode). Fallback={fallback_distance} m')
            self._publish_fallback(fallback_distance, stamp)
            self._publish_per_link_heartbeat(stamp)
            return

        best_cp          = min(valid, key=lambda r: r.distance)
        min_dist         = best_cp.distance
        closest_obs_pt   = best_cp.closest_obstacle_point
        closest_robot_pt = best_cp.point
        closest_uv_obs   = best_cp.closest_pixel

        closest_Z = base_to_cam_z(closest_obs_pt, self.R_base, self.t_base)

        # ── Throttled log ─────────────────────────────────────────────────
        if self._tlog_dist.due(now):
            trk = (f'  | {self.track_pipeline.describe()}'
                   if self.track_pipeline is not None else '')
            if self.self_detect is not None and self.self_detect.flagged:
                trk += f' SELF={len(self.self_detect.flagged)}'
            self._tlog_dist.info(
                f'dist={min_dist:.3f} m  Z={closest_Z:.3f} m  '
                f'pix={closest_uv_obs}  | {self._perf.summary()}{trk}')

        # ── Publish ───────────────────────────────────────────────────────
        msgs = build_cp_messages(
            cp_results=cp_results,
            n_pts=n_pts,
            stamp=stamp,
            frame_id=self.robot_cfg['base_frame'],
            segment_links=self.robot_cfg.get('segment_links', []),
            thresholds=thresholds,
            fallback=fallback_distance,
            zones=self.zones,
            return_cluster_ids=self.multi_k > 1,
        )
        multi_msg, mld_msg = msgs[0], msgs[1]
        cluster_ids = msgs[2] if self.multi_k > 1 else None
        # Track fields onto the message that is about to go out. Same topic,
        # same entries, same order — only the four appended fields are written,
        # and only for control points whose nearest obstacle point falls inside
        # a CONFIRMED track. Everything else keeps the all-zero "no track"
        # defaults the consumer already treats as "contribute nothing".
        if self.track_pipeline is not None:
            try:
                annotate_track_fields(mld_msg, self.track_pipeline,
                                      skip_keys=self._self_detected(cp_results),
                                      cluster_ids=cluster_ids)
            except Exception as exc:
                self.get_logger().error(
                    f'track annotation skipped this frame: {exc}',
                    throttle_duration_sec=2.0)

        self.multi_dist_pub.publish(multi_msg)
        self.per_link_dist_pub.publish(mld_msg)
        self._hb_active = False   # re-arm the heartbeat transition log

        # ── Visualisation snapshot ────────────────────────────────────────
        with self._vis_lock:
            self._vis_frame = VisFrame(
                depth=depth,
                robot_mask=self.mask_builder.robot_mask,
                contours=self.mask_builder.contours,
                robot_segments=[],
                cp_results=cp_results,
                closest_robot_pt=closest_robot_pt,
                closest_uv_obs=closest_uv_obs,
                min_dist=min_dist,
                roi_bounds=self.roi_bounds,
                use_segment_mode=False,
                stamp=stamp,
                visual_ROI=self.visual_ROI,
                visual_exclusion_mask=self.visual_robot_exclusion_mask,
                visualize_only_raw_video=self.visualize_only_raw_video,
            )

    def _check_calibration(self, depth, transforms) -> None:
        """Throttled model-vs-measurement comparison. Never raises, never blocks.

        The mesh samples are transformed into base frame here rather than being
        cached, because they move with the arm — but only ``period_s`` apart, so
        the cost is a few thousand rotations every ten seconds.
        """
        if not self._cal_enabled or self.K is None:
            return
        now = time.monotonic()
        if now < self._cal_next:
            return
        self._cal_next = now + max(1.0, self._cal_period)
        try:
            base_pts = {n: (transforms[n][0] @ p.T).T + transforms[n][1]
                        for n, p in self._link_samples.items() if n in transforms}
            if not base_pts:
                return
            res = calibration_residual(
                base_pts, self.R_base, self.t_base, self.K,
                depth.astype(np.float64) * 0.001,
                min_depth=self.distance_engine.min_depth,
                max_depth=self.distance_engine.max_depth)
            txt = res.describe(self._cal_baseline, self._cal_tol)
            if res.is_suspicious(self._cal_baseline, self._cal_tol):
                self.get_logger().warn(txt)
            else:
                self.get_logger().info(txt)
        except Exception as exc:
            self.get_logger().error(f'calibration check skipped: {exc}',
                                    throttle_duration_sec=30.0)

    def _self_detected(self, cp_results) -> set:
        """Control-point labels whose "obstacle" is moving with the arm.

        Keyed ``link#k`` with k counting occurrences of that link in ARRIVAL
        order — the same convention ``build_cp_messages`` publishes in and
        ``cbf_safety_filter`` labels rows with, so one key names one control
        point everywhere.

        Fails soft: on any error it returns the empty set, i.e. no suppression,
        which is the pre-existing behaviour.
        """
        if self.self_detect is None:
            return set()
        try:
            seen: dict = {}
            for r in cp_results:
                if r.closest_obstacle_point is None or r.point is None:
                    continue
                k = seen.get(r.end_link, 0)
                seen[r.end_link] = k + 1
                self.self_detect.update(f'{r.end_link}#{k}', r.point,
                                        r.closest_obstacle_point)
            msg = self.self_detect.report()
            if msg:
                # Throttled but NOT rate-limited to invisibility: this is a
                # configuration fault, and the run it is silently corrupting can
                # be hours long.
                self.get_logger().warn(msg, throttle_duration_sec=10.0)
            return self.self_detect.flagged
        except Exception as exc:
            self.get_logger().error(f'self-detection check skipped: {exc}',
                                    throttle_duration_sec=5.0)
            return set()

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
            frame = self._vis_frame
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
            msg.header.stamp    = stamp
            msg.header.frame_id = self._overlay_frame_id
            self.overlay_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'overlay publish error: {exc}')

    # ── Fallback publisher ────────────────────────────────────────────────────

    def _publish_fallback(self, distance: float, stamp):
        msg = HumanRobotDistance()
        msg.header.stamp = stamp
        msg.distance     = float(distance)
        self.dist_pub.publish(msg)

    def _publish_per_link_heartbeat(self, stamp) -> None:
        """Publish an empty MultiLinkDistance so downstream nodes can tell
        'no obstacle in range' apart from 'perception is dead'."""
        if not self._publish_empty_per_link:
            return
        if not self._hb_active:
            self._hb_active = True
            self.get_logger().debug(
                'no CP in range - publishing empty per-link heartbeat',
                throttle_duration_sec=5.0)
        msg = MultiLinkDistance()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.robot_cfg['base_frame']
        msg.links           = []
        self.per_link_dist_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RealTimeDistance()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
