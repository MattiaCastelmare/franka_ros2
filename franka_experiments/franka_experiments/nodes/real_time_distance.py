#!/usr/bin/env python3
import os

import cv2
import numpy as np
import rclpy
import trimesh
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from franka_msgs.msg import HumanRobotDistance
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from franka_experiments.utils.distance_utils import (
    compute_closest_distance_from_segments,
    compute_direction_vector,
    define_robot_segments,
    find_pt_confidence,
    get_robot_segments_from_transforms,
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
        closest_distance_topic = topics_cfg.get(
            'closest_distance', '/human_robot/closest_distance')
        self.dist_pub = self.create_publisher(
            HumanRobotDistance, closest_distance_topic, 10)

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
        self.last_depth_msg = msg
        self.last_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def camera_info_callback(self, msg):
        if self.fx is not None:
            return
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.K     = np.array(msg.k, dtype=float).reshape(3, 3)
        self.K_inv = np.linalg.inv(self.K)


    # === Transform functions ===
    def get_link_rotation_translation(self, link_name):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.robot_cfg['base_frame'],
                link_name,
                self.last_depth_msg.header.stamp,
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            R     = get_rotation_from_quaternion(q)
            t_vec = np.array([t.x, t.y, t.z], dtype=float)
            return R, t_vec
        except TransformException:
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
        mask = np.zeros((H, W), dtype=np.uint8)

        for link_name in self.link_mesh_samples.keys():
            points_base = self.get_link_mesh_points_in_base_from_transforms(
                link_name, transforms)
            if points_base is None:
                continue
            r = ee_dilate_px if link_name == 'fr3_link8' else dilate_px
            for p in points_base:
                uv = self.project_point_to_image(p)
                if uv is not None:
                    cv2.circle(mask, uv, r, 255, -1)

        return mask > 0

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
                                 search_exclusion_mask):
        if self.last_depth is None or control_points is None:
            return None, 0

        n_cp        = len(control_points)
        cp_positions = np.array([cp['point']  for cp in control_points], dtype=float)
        radii        = np.array([cp['radius'] for cp in control_points], dtype=float)

        min_dists         = np.full(n_cp, np.inf, dtype=float)
        closest_obs_points  = [None] * n_cp
        closest_obs_pixels  = [None] * n_cp
        closest_directions  = [None] * n_cp
        valid_point_count = 0

        min_depth = float(self.distance_cfg['min_depth_m'])
        max_depth = float(self.distance_cfg['max_depth_m'])

        for v in range(y[0], y[1], step):
            for u in range(x[0], x[1], step):
                if search_exclusion_mask is not None and search_exclusion_mask[v, u]:
                    continue

                Z = float(self.last_depth[v, u]) / 1000.0
                if Z < min_depth or Z > max_depth:
                    continue
                valid_point_count += 1

                uv1   = np.array([u, v, 1.0], dtype=float)
                p_cam = Z * (self.K_inv @ uv1)
                p_obs = self.transform_camera_to_base(p_cam)
                if p_obs is None:
                    continue

                raw_dists = np.linalg.norm(cp_positions - p_obs, axis=1)
                dists     = np.maximum(raw_dists - radii, 0.0)

                better = dists < min_dists
                for i in np.where(better)[0]:
                    min_dists[i]          = dists[i]
                    closest_obs_points[i]  = p_obs
                    closest_obs_pixels[i]  = (u, v)
                    closest_directions[i]  = compute_direction_vector(
                        p_obs, cp_positions, i)

        results = []
        for i in range(n_cp):
            cp = control_points[i]
            results.append({
                'point':                 cp['point'],
                'seg_idx':               cp['seg_idx'],
                'cp_idx':                cp['cp_idx'],
                'radius':                cp['radius'],
                'start_link':            cp['start_link'],
                'end_link':              cp['end_link'],
                'distance':              min_dists[i],
                'direction':             closest_directions[i],
                'closest_obstacle_point': closest_obs_points[i],
                'closest_pixel':         closest_obs_pixels[i],
            })

        return results, valid_point_count


    # === Main Loop ===
    def process_depth(self):
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
                self.get_logger().info(
                    f'No near obstacle (segment mode). '
                    f'Fallback to {fallback_distance} m')
                msg = HumanRobotDistance()
                msg.header.stamp = self.last_depth_msg.header.stamp
                msg.distance = fallback_distance
                self.dist_pub.publish(msg)
                return

        else:
            self.cp_results, n_pts = self.compute_closest_distance(
                control_points=self.control_points,
                x=x, y=y, step=step,
                search_exclusion_mask=self.search_exclusion_mask,
            )
            if self.cp_results is None:
                return

            valid_results = [
                r for r in self.cp_results
                if r['distance'] < np.inf
                and thresholds['min_thresh'] <= r['distance'] <= thresholds['max_thresh']
            ]

            if not valid_results:
                self.get_logger().info(
                    f'No near obstacle (control-point mode). '
                    f'Fallback to {fallback_distance} m')
                msg = HumanRobotDistance()
                msg.header.stamp = self.last_depth_msg.header.stamp
                msg.distance = fallback_distance
                self.dist_pub.publish(msg)
                return

            best_result = min(valid_results, key=lambda r: r['distance'])

        # === Extract data from best result ================================
        self.min_dist            = best_result['distance']
        closest_point            = best_result['closest_obstacle_point']
        self.closest_robot_point = best_result['point']
        self.closest_uv_obs      = best_result['closest_pixel']
        direction                = best_result['direction']
        confidence               = find_pt_confidence(self.min_dist, n_pts)
        p_cam_closest            = self.transform_base_to_camera(closest_point)
        closest_Z                = p_cam_closest[2]

        # === Log ==========================================================
        self.get_logger().info(
            f'Min distance: {self.min_dist:.3f} m  '
            f'Closest depth Z: {closest_Z:.3f} m  '
            f'Closest pixel: {self.closest_uv_obs}')

        # === Publish ======================================================
        if direction is not None:
            dist_msg = HumanRobotDistance()
            dist_msg.header.stamp    = self.last_depth_msg.header.stamp
            dist_msg.header.frame_id = self.robot_cfg['base_frame']
            dist_msg.valid           = True
            dist_msg.distance        = float(self.min_dist)
            dist_msg.robot_link_name = best_result['end_link']
            dist_msg.closest_point_robot = Point(
                x=float(f'{self.closest_robot_point[0]:.4f}'),
                y=float(f'{self.closest_robot_point[1]:.4f}'),
                z=float(f'{self.closest_robot_point[2]:.4f}'),
            )
            dist_msg.direction = Vector3(
                x=float(f'{direction[0]:.4f}'),
                y=float(f'{direction[1]:.4f}'),
                z=float(f'{direction[2]:.4f}'),
            )
            dist_msg.confidence = float(confidence)
            dist_msg.zone       = self.classify_zone(self.min_dist)
            self.dist_pub.publish(dist_msg)


    # === Visualization ====================================================
    def visualize(self):
        """Draw depth image with robot overlay and distance annotation.

        Called by a dedicated 10 Hz timer (created only when
        booleans.visualize is true).  Reads shared state written by
        process_depth — no distance computation here.
        """
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
