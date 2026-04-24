#!/usr/bin/env python3

import os
import time
import cv2
import numpy as np
import rclpy
import trimesh
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from franka_msgs.msg import HumanRobotDistance, LinkDistance, MultiDistance, MultiLinkDistance
from geometry_msgs.msg import Point, Vector3
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from franka_experiments.utils.distance_utils import (
    compute_closest_distance_from_segments,
    define_robot_segments,
    find_pt_confidence,
    get_rotation_from_quaternion,
    load_extrinsics,
    load_robot_config,
)


class RealTimeDistance(Node):
    def __init__(self):
        super().__init__('real_time_distance')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('robot_config_path', '')
        self.declare_parameter('camera_extrinsics_path', '')

        robot_config_path = self.get_parameter('robot_config_path').value
        camera_extrinsics_path = self.get_parameter('camera_extrinsics_path').value

        if not robot_config_path:
            raise RuntimeError('Parameter robot_config_path must be set.')
        if not camera_extrinsics_path:
            raise RuntimeError('Parameter camera_extrinsics_path must be set.')

        # ── Load YAML configs ────────────────────────────────────────────
        self.config = load_robot_config(robot_config_path)
        self.robot_cfg = self.config['robot']
        self.distance_cfg = self.config['distance']
        self.mask_cfg = self.config['mask']
        self.mesh_cfg = self.config['meshes']
        self.zones = self.config.get('zones', {})

        self.R_base, self.t_base = load_extrinsics(camera_extrinsics_path)

        # ── Runtime flags (booleans section of fr3_complete.yaml) ────────
        booleans = self.config.get('booleans', {})
        self.enable_visualization       = booleans.get('visualize', False)
        self.visual_robot_exclusion_mask = booleans.get('exclusion_mask', True)
        self.visualize_only_raw_video   = booleans.get('raw_video', False)
        self.use_segment_distance       = booleans.get('use_segment_distance', False)
        self.visual_ROI                 = booleans.get('visual_ROI', False)

        # Exclude base segments (0-2) from distance computation —
        # they are nearly always occluded by the robot itself.
        self.min_seg_idx_for_distance = 3

        # ── Internal state ───────────────────────────────────────────────
        self.bridge = CvBridge()
        self.fx = self.fy = None
        self.cx = self.cy = None
        self.last_depth = None
        self.last_depth_msg = None

        # Shared state: written by process_depth, read by visualize timer
        self.robot_mask = None
        self.search_exclusion_mask = None
        self.control_points = None
        self.robot_segments = []
        self.cp_results = None
        self.closest_robot_point = None
        self.closest_uv_obs = None
        self.min_dist = np.inf
        self.roi_bounds = None

        # ── TF ───────────────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Robustness counters / throttle timestamps ─────────────────────
        self._process_skip_count = 0
        self._vis_skip_count     = 0
        self._last_tf_warn_t     = 0.0   # wall clock [s]
        self._last_no_obs_warn_t = 0.0
        self._last_dist_log_t    = 0.0
        self._THROTTLE_S         = 2.0   # min seconds between repeated log lines

        # ── Subscriptions ────────────────────────────────────────────────
        topics_cfg = self.config.get('topics', {})
        depth_topic = topics_cfg.get('depth_image', '/camera/camera/depth/image_rect_raw')
        info_topic  = topics_cfg.get('depth_camera_info', '/camera/camera/depth/camera_info')

        self.create_subscription(Image,      depth_topic, self.depth_callback,       10)
        self.create_subscription(CameraInfo, info_topic,  self.camera_info_callback, 10)

        # ── Mesh loading ─────────────────────────────────────────────────
        mesh_pkg      = self.mesh_cfg.get('package', 'franka_description')
        mesh_base_dir = get_package_share_directory(mesh_pkg)
        sample_pts    = int(self.mesh_cfg.get('sample_points_per_link', 300))

        self.link_meshes = {}
        self.link_mesh_samples = {}
        for link_name, rel_path in self.mesh_cfg['files'].items():
            full_path = os.path.join(mesh_base_dir, rel_path)
            mesh = trimesh.load(full_path, force='mesh')
            self.link_meshes[link_name] = mesh
            self.link_mesh_samples[link_name] = mesh.sample(sample_pts)

        # ── Publisher ────────────────────────────────────────────────────
        multi_distance_topic = topics_cfg.get(
            'multi_distance', '/human_robot/multi_distance')
        self.multi_dist_pub = self.create_publisher(
            MultiDistance, multi_distance_topic, 10)

        distance_topic = topics_cfg.get('distance', '/human_robot/distance')
        self.dist_pub = self.create_publisher(
            HumanRobotDistance, distance_topic, 10)

        _be_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.per_link_dist_pub = self.create_publisher(
            MultiLinkDistance, '/cbf/per_link_distances', _be_qos)

        self.get_logger().info(
            f'RealTimeDistance ready. '
            f'mode={"segment" if self.use_segment_distance else "control_point"}  '
            f'viz={self.enable_visualization}')

        self.create_timer(0.1, self.process_depth)
        if self.enable_visualization:
            self.create_timer(0.1, self.visualize)


    def classify_zone(self, distance: float) -> str:
        """Map distance [m] to safety zone string."""
        if not self.zones:
            return 'unknown'
        if distance <= self.zones.get('critical', 0.1):
            return 'critical'
        if distance <= self.zones.get('danger', 0.2):
            return 'danger'
        if distance <= self.zones.get('warning', 0.3):
            return 'warning'
        return 'safe'

    # === Camera Callbacks ===
    def depth_callback(self, msg):
        try:
            self.last_depth_msg = msg
            self.last_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:
            self.get_logger().warn(f'depth_callback error (skipping frame): {exc}')

    def camera_info_callback(self, msg):
        if self.fx is not None:
            return
        try:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.K     = np.array(msg.k, dtype=float).reshape(3, 3)
            self.K_inv = np.linalg.inv(self.K)
        except Exception as exc:
            self.get_logger().warn(f'camera_info_callback error: {exc}')


    # === Transform functions ===
    def get_link_rotation_translation(self, link_name):
        try:
            # Use RclpyTime() (latest available) — non-blocking, no timestamp sync needed.
            tf = self.tf_buffer.lookup_transform(
                self.robot_cfg['base_frame'],
                link_name,
                RclpyTime(),
            )
            t     = tf.transform.translation
            q     = tf.transform.rotation
            R     = get_rotation_from_quaternion(q)
            t_vec = np.array([t.x, t.y, t.z], dtype=float)
            return R, t_vec
        except Exception as exc:
            now = time.monotonic()
            if now - self._last_tf_warn_t >= self._THROTTLE_S:
                self._last_tf_warn_t = now
                self.get_logger().warn(f'TF lookup failed for {link_name}: {exc}')
            return None, None

    def get_all_link_transforms(self):
        link_names = self.robot_cfg['segment_links']
        transforms = {}
        for name in link_names:
            R, t = self.get_link_rotation_translation(name)
            if R is None or t is None:
                return None
            transforms[name] = (R, t)
        return transforms

    def get_link_mesh_points_in_base_from_transforms(self, link_name, transforms):
        if link_name not in transforms:
            return None
        R, t = transforms[link_name]
        points_local = self.link_mesh_samples[link_name]
        points_base  = (R @ points_local.T).T + t
        return points_base

    def transform_camera_to_base(self, p_cam):
        return self.R_base @ np.array(p_cam) + self.t_base

    def transform_base_to_camera(self, p_base):
        return self.R_base.T @ (np.array(p_base) - self.t_base)

    def project_point_to_image(self, p_base):
        p_cam = self.transform_base_to_camera(p_base)
        if p_cam is None or p_cam[2] <= 0:
            return None
        uv = self.K @ p_cam
        u  = int(uv[0] / uv[2])
        v  = int(uv[1] / uv[2])
        if 0 <= u < self.last_depth.shape[1] and 0 <= v < self.last_depth.shape[0]:
            return (u, v)
        return None


    # === Robot Mask ===
    def build_robot_mask_from_transforms(self, transforms, dilate_px, ee_dilate_px):
        if self.last_depth is None:
            return None

        H, W = self.last_depth.shape
        # Separate accumulation masks: one for normal links, one for the EE link.
        # This avoids any per-link dilation accumulation.
        mask_normal = np.zeros((H, W), dtype=np.uint8)
        mask_ee     = np.zeros((H, W), dtype=np.uint8)

        for link_name in self.link_mesh_samples.keys():
            points_base = self.get_link_mesh_points_in_base_from_transforms(
                link_name, transforms)
            if points_base is None:
                continue

            # Batch project: base -> camera -> image
            p_cam = (self.R_base.T @ (points_base - self.t_base).T).T  # (N, 3)
            in_front = p_cam[:, 2] > 0
            p_cam = p_cam[in_front]
            if p_cam.shape[0] == 0:
                continue

            uv = (self.K @ p_cam.T).T                                   # (N, 3)
            us = (uv[:, 0] / uv[:, 2]).astype(int)
            vs = (uv[:, 1] / uv[:, 2]).astype(int)

            in_bounds = (us >= 0) & (us < W) & (vs >= 0) & (vs < H)
            us, vs = us[in_bounds], vs[in_bounds]

            if link_name == 'fr3_link8':
                mask_ee[vs, us] = 255
            else:
                mask_normal[vs, us] = 255

        # Single dilation per mask, then combine
        def _dilate(m, r):
            if r <= 0 or not np.any(m):
                return m
            k = 2 * r + 1
            return cv2.dilate(m, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))

        mask_normal = _dilate(mask_normal, dilate_px)
        mask_ee     = _dilate(mask_ee,     ee_dilate_px)

        return (mask_normal | mask_ee) > 0

    def build_search_exclusion_mask(self, robot_mask, extra_px=12):
        if robot_mask is None:
            return None
        excl = (robot_mask.astype(np.uint8) * 255)
        if extra_px > 0:
            k      = 2 * extra_px + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            excl   = cv2.dilate(excl, kernel)
        return excl > 0


    # === Control Points ===
    def define_control_points(self, transforms):
        control_points = []
        ee_tip_axis   = self.distance_cfg['ee_tip_axis']
        ee_tip_offset = self.distance_cfg['ee_tip_offset']

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

            # Default: interior control points
            ts = [(k + 1) / (n_cp + 1) for k in range(n_cp)]

            # Last segment ending at EE → force last point on the physical tip
            if end_link == 'fr3_link8':
                ts = [1.0] if n_cp == 1 else [(k + 1) / n_cp for k in range(n_cp)]

            for k, t in enumerate(ts):
                p = p0 + t * (p1 - p0)

                if end_link == 'fr3_link8' and np.isclose(t, 1.0):
                    R_ee, p1 = transforms['fr3_link8']
                    direction = R_ee[:, ee_tip_axis]
                    p = p1 + ee_tip_offset * direction

                control_points.append({
                    'point':      p,
                    'seg_idx':    seg_idx,
                    'cp_idx':     k,
                    'radius':     radius,
                    'start_link': start_link,
                    'end_link':   end_link,
                })

        return control_points if control_points else None

    def draw_control_points(self, image, control_points, color=(0, 165, 255)):
        if control_points is None:
            return
        for cp in control_points:
            uv = self.project_point_to_image(cp['point'])
            if uv is not None:
                cv2.circle(image, uv, 3, color, -1)

    def compute_closest_distance(self, control_points, x, y, step,
                                 search_exclusion_mask, transforms=None):
        if self.last_depth is None or control_points is None:
            return None, 0

        n_cp         = len(control_points)
        cp_positions = np.array([cp['point']  for cp in control_points], dtype=float)
        radii        = np.array([cp['radius'] for cp in control_points], dtype=float)

        min_depth = float(self.distance_cfg['min_depth_m'])
        max_depth = float(self.distance_cfg['max_depth_m'])

        # Build strided pixel grids over the ROI
        us = np.arange(x[0], x[1], step)
        vs = np.arange(y[0], y[1], step)
        ug, vg = np.meshgrid(us, vs)          # (Nv, Nu)
        ug = ug.ravel()
        vg = vg.ravel()

        # Apply search exclusion mask
        if search_exclusion_mask is not None:
            keep = ~search_exclusion_mask[vg, ug]
            ug, vg = ug[keep], vg[keep]

        # Depth filter
        Z = self.last_depth[vg, ug].astype(float) / 1000.0
        valid = (Z >= min_depth) & (Z <= max_depth)
        ug, vg, Z = ug[valid], vg[valid], Z[valid]

        valid_point_count = int(ug.size)

        if valid_point_count == 0:
            results = [{
                'point':                  cp['point'],
                'seg_idx':                cp['seg_idx'],
                'cp_idx':                 cp['cp_idx'],
                'radius':                 cp['radius'],
                'start_link':             cp['start_link'],
                'end_link':               cp['end_link'],
                'distance':               np.inf,
                'direction':              None,
                'closest_obstacle_point': None,
                'closest_pixel':          None,
            } for cp in control_points]
            return results, 0

        # Unproject all valid pixels at once: p_cam = Z * K_inv @ [u, v, 1]^T
        ones     = np.ones(valid_point_count, dtype=float)
        uv_hom   = np.stack([ug.astype(float), vg.astype(float), ones], axis=1)  # (N, 3)
        p_cam    = Z[:, None] * (self.K_inv @ uv_hom.T).T                        # (N, 3)

        # Transform all points to base frame in batch: p_obs = R_base @ p_cam.T + t_base
        p_obs = (self.R_base @ p_cam.T).T + self.t_base                          # (N, 3)

        # 3D self-filter: remove obstacle candidates that are geometrically on the robot.
        # Guard set = CP surfaces + link origins (covers links with no CPs assigned).
        eps         = float(self.distance_cfg.get('self_collision_eps', 0.05))
        body_radius = float(self.distance_cfg.get('robot_body_radius', 0.10))

        all_guard_pts    = list(cp_positions)
        all_guard_radii  = list(radii)
        if transforms is not None:
            for _, (_, t_link) in transforms.items():
                all_guard_pts.append(t_link)
                all_guard_radii.append(body_radius)
        all_guard_pts   = np.array(all_guard_pts,   dtype=float)
        all_guard_radii = np.array(all_guard_radii, dtype=float)

        robot_self = np.full(p_obs.shape[0], np.inf, dtype=float)
        for i in range(len(all_guard_pts)):
            d = np.maximum(
                np.linalg.norm(p_obs - all_guard_pts[i], axis=1) - all_guard_radii[i],
                0.0)
            np.minimum(robot_self, d, out=robot_self)

        is_robot   = robot_self < eps
        n_filtered = int(is_robot.sum())
        if n_filtered > 0:
            self.get_logger().debug(
                f'3D self-filter: removed {n_filtered}/{p_obs.shape[0]} robot-self pixels '
                f'(eps={eps:.3f} m)')

        not_robot = ~is_robot
        p_obs = p_obs[not_robot]
        ug    = ug[not_robot]
        vg    = vg[not_robot]

        valid_point_count = int(p_obs.shape[0])
        if valid_point_count == 0:
            self.get_logger().debug('3D self-filter: all points removed — no obstacle detected')
            results = [{
                'point':                  cp['point'],
                'seg_idx':                cp['seg_idx'],
                'cp_idx':                 cp['cp_idx'],
                'radius':                 cp['radius'],
                'start_link':             cp['start_link'],
                'end_link':               cp['end_link'],
                'distance':               np.inf,
                'direction':              None,
                'closest_obstacle_point': None,
                'closest_pixel':          None,
            } for cp in control_points]
            return results, 0

        # Per-CP distances: loop over n_cp (~10-20) to avoid N x n_cp x 3 allocation
        best_idx  = np.empty(n_cp, dtype=int)
        min_dists = np.full(n_cp, np.inf, dtype=float)
        for i in range(n_cp):
            diff        = p_obs - cp_positions[i]               # (N, 3)
            raw_dists_i = np.linalg.norm(diff, axis=1)          # (N,)
            dists_i     = np.maximum(raw_dists_i - radii[i], 0.0)
            bi          = int(np.argmin(dists_i))
            best_idx[i] = bi
            min_dists[i] = dists_i[bi]

        results = []
        for i, cp in enumerate(control_points):
            bi = best_idx[i]
            if np.isfinite(min_dists[i]):
                obs_pt  = p_obs[bi]
                obs_pix = (int(ug[bi]), int(vg[bi]))
                vec     = cp_positions[i] - obs_pt
                norm    = np.linalg.norm(vec)
                direction = vec / norm if norm > 1e-9 else np.zeros(3)
            else:
                obs_pt    = None
                obs_pix   = None
                direction = None
            results.append({
                'point':                  cp['point'],
                'seg_idx':                cp['seg_idx'],
                'cp_idx':                 cp['cp_idx'],
                'radius':                 cp['radius'],
                'start_link':             cp['start_link'],
                'end_link':               cp['end_link'],
                'distance':               min_dists[i],
                'direction':              direction,
                'closest_obstacle_point': obs_pt,
                'closest_pixel':          obs_pix,
            })

        return results, valid_point_count


    def publish_fallback_distance(self, distance, stamp):
        msg = HumanRobotDistance()
        msg.header.stamp = stamp
        msg.distance = float(distance)
        self.dist_pub.publish(msg)

    # === Main Loop ===
    def process_depth(self):
        try:
            self._process_depth_impl()
        except Exception as exc:
            self._process_skip_count += 1
            self.get_logger().error(
                f'process_depth unhandled error '
                f'(skip #{self._process_skip_count}): {exc}')

    def _process_depth_impl(self):
        if self.last_depth is None or self.last_depth_msg is None or self.fx is None:
            return

        H, W   = self.last_depth.shape
        step   = int(self.distance_cfg['pixel_step'])
        margin = int(self.distance_cfg['image_margin_px'])

        # Get all link transforms
        transforms = self.get_all_link_transforms()
        if transforms is None:
            return

        # Define control points (needed for CP mode and for CP-mode visualization)
        self.control_points = self.define_control_points(transforms)

        # Define capsule segments (needed for segment mode AND for skeleton overlay)
        self.robot_segments = define_robot_segments(
            transforms, self.robot_cfg, self.distance_cfg)
        if self.robot_segments is None:
            return

        # Filter out base segments for distance computation
        self.robot_segments = [
            s for s in self.robot_segments
            if s['seg_idx'] >= self.min_seg_idx_for_distance
        ]
        if not self.robot_segments:
            return

        # Build robot mask and search exclusion mask
        self.robot_mask = self.build_robot_mask_from_transforms(
            transforms,
            dilate_px=int(self.mask_cfg['robot_mask_dilate_px']),
            ee_dilate_px=int(self.mask_cfg['ee_mask_dilate_px']),
        )
        self.search_exclusion_mask = self.build_search_exclusion_mask(
            self.robot_mask,
            extra_px=int(self.mask_cfg['search_exclusion_extra_px']),
        )

        # ROI around the robot
        roi_pad = int(self.distance_cfg['roi_pad_px'])
        if self.search_exclusion_mask is not None:
            ys, xs = np.where(self.search_exclusion_mask)
            if len(xs) > 0 and len(ys) > 0:
                x0 = max(margin, int(xs.min()) - roi_pad)
                x1 = min(W - margin, int(xs.max()) + roi_pad)
                y0 = max(margin, int(ys.min()) - roi_pad)
                y1 = min(H - margin, int(ys.max()) + roi_pad)
            else:
                x0, x1 = margin, W - margin
                y0, y1 = margin, H - margin
        else:
            x0, x1 = margin, W - margin
            y0, y1 = margin, H - margin

        self.roi_bounds = (x0, y0, x1, y1)
        x, y = np.array([x0, x1]), np.array([y0, y1])

        # Reset per-cycle result state
        self.cp_results          = None
        self.min_dist            = np.inf
        self.closest_robot_point = None
        self.closest_uv_obs      = None

        thresholds        = self.distance_cfg['thresholds']
        fallback_distance = float(self.distance_cfg.get('fallback_distance', 2.0))

        # === Distance computation (segment or control-point mode) =========
        if self.use_segment_distance:
            best_result, n_pts = compute_closest_distance_from_segments(
                last_depth=self.last_depth,
                K_inv=self.K_inv,
                transform_camera_to_base_fn=self.transform_camera_to_base,
                robot_segments=self.robot_segments,
                x=x, y=y, step=step,
                search_exclusion_mask=self.search_exclusion_mask,
                distance_cfg=self.distance_cfg,
            )

            if best_result is None or not (
                thresholds['min_thresh'] <= best_result['distance'] <= thresholds['max_thresh']
            ):
                now = time.monotonic()
                if now - self._last_no_obs_warn_t >= self._THROTTLE_S:
                    self._last_no_obs_warn_t = now
                    self.get_logger().debug(
                        f'No near obstacle (segment mode). Fallback={fallback_distance} m')
                self.publish_fallback_distance(
                    fallback_distance, self.last_depth_msg.header.stamp)
                return

        else:
            self.cp_results, n_pts = self.compute_closest_distance(
                control_points=self.control_points,
                x=x, y=y, step=step,
                search_exclusion_mask=self.search_exclusion_mask,
                transforms=transforms,
            )
            if self.cp_results is None:
                return

            valid_results = [
                r for r in self.cp_results
                if r['distance'] < np.inf
                and thresholds['min_thresh'] <= r['distance'] <= thresholds['max_thresh']
            ]

            if not valid_results:
                now = time.monotonic()
                if now - self._last_no_obs_warn_t >= self._THROTTLE_S:
                    self._last_no_obs_warn_t = now
                    self.get_logger().debug(
                        f'No near obstacle (CP mode). Fallback={fallback_distance} m')
                self.publish_fallback_distance(
                    fallback_distance, self.last_depth_msg.header.stamp)
                return

            best_result = min(valid_results, key=lambda r: r['distance'])

        # === Extract data from best result ================================
        self.min_dist            = best_result['distance']
        closest_point            = best_result['closest_obstacle_point']
        self.closest_robot_point = best_result['point']
        self.closest_uv_obs      = best_result['closest_pixel']
        p_cam_closest            = self.transform_base_to_camera(closest_point)
        closest_Z                = p_cam_closest[2]

        # === Log (throttled — avoids 10 Hz spam) ==========================
        now = time.monotonic()
        if now - self._last_dist_log_t >= self._THROTTLE_S:
            self._last_dist_log_t = now
            self.get_logger().info(
                f'Min distance: {self.min_dist:.3f} m  '
                f'Closest depth Z: {closest_Z:.3f} m  '
                f'Closest pixel: {self.closest_uv_obs}')

        # === Publish MultiDistance (closest CP per active segment) ========
        if self.cp_results is not None:
            # For each active seg_idx keep only the CP with minimum distance.
            seg_best: dict[int, dict] = {}
            for r in self.cp_results:
                idx = r['seg_idx']
                if idx not in seg_best or r['distance'] < seg_best[idx]['distance']:
                    seg_best[idx] = r

            # Build one HumanRobotDistance per segment, ordered by seg_idx.
            stamp     = self.last_depth_msg.header.stamp
            frame_id  = self.robot_cfg['base_frame']
            fallback  = float(self.distance_cfg.get('fallback_distance', 2.0))
            thresholds = self.distance_cfg['thresholds']
            entries: list[HumanRobotDistance] = []
            for idx in sorted(seg_best):
                r   = seg_best[idx]
                msg = HumanRobotDistance()
                msg.header.stamp    = stamp
                msg.header.frame_id = frame_id
                msg.robot_link_name = r['end_link']

                valid_dist = (
                    np.isfinite(r['distance'])
                    and thresholds['min_thresh'] <= r['distance'] <= thresholds['max_thresh']
                    and r['direction'] is not None
                )
                if valid_dist:
                    msg.valid    = True
                    msg.distance = float(r['distance'])
                    msg.closest_point_robot = Point(
                        x=round(float(r['point'][0]), 4),
                        y=round(float(r['point'][1]), 4),
                        z=round(float(r['point'][2]), 4),
                    )
                    msg.direction = Vector3(
                        x=round(float(r['direction'][0]), 4),
                        y=round(float(r['direction'][1]), 4),
                        z=round(float(r['direction'][2]), 4),
                    )
                    msg.zone       = self.classify_zone(r['distance'])
                    msg.confidence = float(find_pt_confidence(r['distance'], n_pts))
                else:
                    msg.valid    = False
                    msg.distance = fallback
                entries.append(msg)

            multi_msg = MultiDistance()
            multi_msg.header.stamp    = stamp
            multi_msg.header.frame_id = frame_id
            multi_msg.distances       = entries
            self.multi_dist_pub.publish(multi_msg)

            # Publish per-link distances (one LinkDistance per segment_link with CPs)
            link_best: dict[str, dict] = {}
            for r in self.cp_results:
                lk = r['end_link']
                if lk not in link_best or r['distance'] < link_best[lk]['distance']:
                    link_best[lk] = r

            link_entries: list[LinkDistance] = []
            for lk in self.robot_cfg.get('segment_links', []):
                if lk not in link_best:
                    continue
                r = link_best[lk]
                ld = LinkDistance()
                ld.robot_link_name = lk
                if r['point'] is not None:
                    ld.closest_point_robot = Point(
                        x=float(r['point'][0]),
                        y=float(r['point'][1]),
                        z=float(r['point'][2]),
                    )
                obs = r['closest_obstacle_point']
                if obs is not None:
                    ld.closest_point_human = Point(
                        x=float(obs[0]),
                        y=float(obs[1]),
                        z=float(obs[2]),
                    )
                if r['direction'] is not None:
                    d = r['direction']
                    ld.direction = Vector3(
                        x=float(d[0]), y=float(d[1]), z=float(d[2]))
                ld.distance   = float(r['distance'])
                ld.valid      = (
                    np.isfinite(r['distance'])
                    and r['distance'] > 0.0
                    and r['direction'] is not None
                )
                ld.confidence = 1.0
                ld.zone       = self.classify_zone(r['distance'])
                link_entries.append(ld)

            mld_msg = MultiLinkDistance()
            mld_msg.header.stamp    = stamp
            mld_msg.header.frame_id = frame_id
            mld_msg.links           = link_entries
            self.per_link_dist_pub.publish(mld_msg)


    # === Visualization ====================================================
    def visualize(self):
        """Draw depth image with robot overlay and distance annotation."""
        try:
            self._visualize_impl()
        except Exception as exc:
            self._vis_skip_count += 1
            self.get_logger().warn(
                f'visualize error (skip #{self._vis_skip_count}): {exc}')

    def _visualize_impl(self):
        if self.last_depth is None or self.last_depth_msg is None or self.fx is None:
            return

        depth_vis = cv2.normalize(self.last_depth, None, 0, 255, cv2.NORM_MINMAX)
        depth_vis = depth_vis.astype(np.uint8)
        depth_vis = cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)

        if self.visual_ROI and self.roi_bounds is not None:
            x0, y0, x1, y1 = self.roi_bounds
            cv2.rectangle(depth_vis, (x0, y0), (x1, y1), (255, 0, 255), 2)

        if self.visualize_only_raw_video:
            cv2.imshow('Robot + closest distance', depth_vis)
            cv2.waitKey(1)
            return

        # Robot mask overlay
        if self.robot_mask is not None and self.visual_robot_exclusion_mask:
            depth_vis[self.robot_mask] = (0, 0, 0)
            contours, _ = cv2.findContours(
                (self.robot_mask.astype(np.uint8) * 255),
                cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(depth_vis, contours, -1, (0, 255, 0), 3)

        # Robot segment skeleton (green)
        if self.robot_segments is not None:
            for seg in self.robot_segments:
                uv_a = self.project_point_to_image(seg['p0'])
                uv_b = self.project_point_to_image(seg['p1'])
                if uv_a is not None and uv_b is not None:
                    cv2.line(depth_vis, uv_a, uv_b, (0, 255, 0), 1)
                    cv2.circle(depth_vis, uv_a, 2, (0, 255, 0), -1)
                    cv2.circle(depth_vis, uv_b, 2, (0, 255, 0), -1)

        # Per-control-point distance lines (thin white) — CP mode only
        if not self.use_segment_distance and self.cp_results is not None:
            for r in self.cp_results:
                if r['distance'] == np.inf:
                    continue
                uv_cp  = self.project_point_to_image(r['point'])
                uv_obs = r['closest_pixel']
                if uv_cp is not None and uv_obs is not None:
                    cv2.line(depth_vis, uv_cp, uv_obs, (255, 255, 255), 1)

        # Closest obstacle pixel (red dot)
        u, v = 20, 20
        if self.closest_uv_obs is not None:
            u, v = self.closest_uv_obs
            cv2.circle(depth_vis, (u, v), 5, (0, 0, 255), -1)

        # Closest point on robot (cyan) + distance segment (white)
        if self.closest_robot_point is not None and self.closest_uv_obs is not None:
            uv_robot = self.project_point_to_image(self.closest_robot_point)
            if uv_robot is not None:
                cv2.circle(depth_vis, uv_robot, 5, (255, 255, 0), -1)
                cv2.line(depth_vis, self.closest_uv_obs, uv_robot, (255, 255, 255), 2)

        # Control-point markers (orange) — CP mode only
        if not self.use_segment_distance and self.control_points is not None:
            self.draw_control_points(depth_vis, self.control_points, color=(0, 165, 255))

        # Distance text
        if np.isfinite(self.min_dist):
            cv2.putText(
                depth_vis, f'{self.min_dist:.3f} m',
                (u + 10, v), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        cv2.imshow('Robot + closest distance', depth_vis)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    distance_calculator = RealTimeDistance()
    rclpy.spin(distance_calculator)
    distance_calculator.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()