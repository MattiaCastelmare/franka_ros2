#!/usr/bin/env python3
"""Real-time minimum distance calculator between robot and obstacles.

Subscribes to a depth image and robot TF tree, projects robot mesh
point clouds into the depth frame, and computes the closest obstacle
distance for each control point on the robot body.

ROS2 Parameters
---------------
robot_config_path : str
    Absolute path to the robot config YAML (fr3_complete-style).
camera_extrinsics_path : str
    Absolute path to the camera extrinsics YAML
    (parent_frame / child_frame / translation / rotation).
"""

import os

import cv2
import numpy as np
import rclpy
import trimesh
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from franka_experiments.utils.distance_utils import (
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

        self.R_base, self.t_base = load_extrinsics(camera_extrinsics_path)

        # ── Internal state ───────────────────────────────────────────────
        self.bridge = CvBridge()
        self.fx = self.fy = None
        self.cx = self.cy = None
        self.last_depth = None
        self.last_depth_msg = None

        # ── TF ───────────────────────────────────────────────────────────
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Subscriptions ────────────────────────────────────────────────
        topics_cfg = self.config.get('topics', {})
        depth_topic = topics_cfg.get('depth_image', '/camera/camera/depth/image_rect_raw')
        info_topic = topics_cfg.get('depth_camera_info', '/camera/camera/depth/camera_info')

        self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, 10)

        # ── Mesh loading ─────────────────────────────────────────────────
        mesh_pkg = self.mesh_cfg.get('package', 'franka_description')
        mesh_base_dir = get_package_share_directory(mesh_pkg)
        sample_pts = int(self.mesh_cfg.get('sample_points_per_link', 300))

        self.link_meshes = {}
        self.link_mesh_samples = {}
        for link_name, rel_path in self.mesh_cfg['files'].items():
            full_path = os.path.join(mesh_base_dir, rel_path)
            mesh = trimesh.load(full_path, force='mesh')
            self.link_meshes[link_name] = mesh
            self.link_mesh_samples[link_name] = mesh.sample(sample_pts)

        self.get_logger().info('RealTimeDistance node ready.')
        self.create_timer(0.1, self.process_depth)

    # ── Camera callbacks ────────────────────────────────────────────────
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
        self.K = np.array(msg.k, dtype=float).reshape(3, 3)
        self.K_inv = np.linalg.inv(self.K)

    # ── Transform helpers ───────────────────────────────────────────────
    def get_link_rotation_translation(self, link_name):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.robot_cfg['base_frame'],
                link_name,
                self.last_depth_msg.header.stamp,
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            R = get_rotation_from_quaternion(q)
            return R, np.array([t.x, t.y, t.z], dtype=float)
        except TransformException:
            return None, None

    def get_all_link_transforms(self):
        transforms = {}
        for name in self.robot_cfg['segment_links']:
            R, t = self.get_link_rotation_translation(name)
            if R is None:
                return None
            transforms[name] = (R, t)
        return transforms

    def get_link_mesh_points_in_base_from_transforms(self, link_name, transforms):
        if link_name not in transforms:
            return None
        R, t = transforms[link_name]
        pts = self.link_mesh_samples[link_name]
        return (R @ pts.T).T + t

    def transform_camera_to_base(self, p_cam):
        return self.R_base @ np.array(p_cam) + self.t_base

    def transform_base_to_camera(self, p_base):
        return self.R_base.T @ (np.array(p_base) - self.t_base)

    def project_point_to_image(self, p_base):
        p_cam = self.transform_base_to_camera(p_base)
        if p_cam[2] <= 0:
            return None
        uv = self.K @ p_cam
        u = int(uv[0] / uv[2])
        v = int(uv[1] / uv[2])
        if 0 <= u < self.last_depth.shape[1] and 0 <= v < self.last_depth.shape[0]:
            return (u, v)
        return None

    # ── Robot mask ──────────────────────────────────────────────────────
    def build_robot_mask_from_transforms(self, transforms, dilate_px=10, ee_dilate_px=12):
        if self.last_depth is None:
            return None
        H, W = self.last_depth.shape
        mask = np.zeros((H, W), dtype=np.uint8)
        for link_name in self.link_mesh_samples:
            pts = self.get_link_mesh_points_in_base_from_transforms(link_name, transforms)
            if pts is None:
                continue
            r = ee_dilate_px if link_name == 'fr3_link8' else dilate_px
            for p in pts:
                uv = self.project_point_to_image(p)
                if uv is not None:
                    cv2.circle(mask, uv, r, 255, -1)
        return mask > 0

    def build_search_exclusion_mask(self, robot_mask, extra_px=12):
        if robot_mask is None:
            return None
        excl = robot_mask.astype(np.uint8) * 255
        if extra_px > 0:
            k = 2 * extra_px + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            excl = cv2.dilate(excl, kernel)
        return excl > 0

    # ── Control points ──────────────────────────────────────────────────
    def define_control_points(self, transforms):
        control_points = []
        for seg in self.robot_cfg['segments']:
            n_cp = int(seg.get('control_points', 0))
            if n_cp <= 0:
                continue
            start_link = seg['start_link']
            end_link = seg['end_link']
            if start_link not in transforms or end_link not in transforms:
                continue
            _, p0 = transforms[start_link]
            _, p1 = transforms[end_link]
            radius = float(seg.get('radius', 0.0))
            for k in range(n_cp):
                t = (k + 1) / (n_cp + 1)
                control_points.append({
                    'point': p0 + t * (p1 - p0),
                    'seg_idx': int(seg['seg_idx']),
                    'cp_idx': k,
                    'radius': radius,
                    'start_link': start_link,
                    'end_link': end_link,
                })
        return control_points if control_points else None

    def draw_control_points(self, image, control_points, color=(0, 165, 255)):
        if control_points is None:
            return
        for cp in control_points:
            uv = self.project_point_to_image(cp['point'])
            if uv is not None:
                cv2.circle(image, uv, 3, color, -1)

    def compute_closest_distance(self, control_points, x, y, step, search_exclusion_mask):
        if self.last_depth is None or control_points is None:
            return None
        n_cp = len(control_points)
        cp_positions = np.array([cp['point'] for cp in control_points], dtype=float)
        radii = np.array([cp['radius'] for cp in control_points], dtype=float)
        min_dists = np.full(n_cp, np.inf, dtype=float)
        closest_obs_points = [None] * n_cp
        closest_obs_pixels = [None] * n_cp
        min_depth = float(self.distance_cfg['min_depth_m'])
        max_depth = float(self.distance_cfg['max_depth_m'])

        for v in range(y[0], y[1], step):
            for u in range(x[0], x[1], step):
                if search_exclusion_mask is not None and search_exclusion_mask[v, u]:
                    continue
                Z = float(self.last_depth[v, u]) / 1000.0
                if Z < min_depth or Z > max_depth:
                    continue
                p_cam = Z * (self.K_inv @ np.array([u, v, 1.0], dtype=float))
                p_obs = self.transform_camera_to_base(p_cam)
                raw_dists = np.linalg.norm(cp_positions - p_obs, axis=1)
                dists = np.maximum(raw_dists - radii, 0.0)
                for i in np.where(dists < min_dists)[0]:
                    min_dists[i] = dists[i]
                    closest_obs_points[i] = p_obs
                    closest_obs_pixels[i] = (u, v)

        return [
            {
                'point': control_points[i]['point'],
                'seg_idx': control_points[i]['seg_idx'],
                'cp_idx': control_points[i]['cp_idx'],
                'radius': control_points[i]['radius'],
                'start_link': control_points[i]['start_link'],
                'end_link': control_points[i]['end_link'],
                'distance': min_dists[i],
                'closest_obstacle_point': closest_obs_points[i],
                'closest_pixel': closest_obs_pixels[i],
            }
            for i in range(n_cp)
        ]

    # ── Main loop ───────────────────────────────────────────────────────
    def process_depth(self):
        if self.last_depth is None or self.last_depth_msg is None or self.fx is None:
            return

        H, W = self.last_depth.shape
        step = int(self.distance_cfg['pixel_step'])
        margin = int(self.distance_cfg['image_margin_px'])

        transforms = self.get_all_link_transforms()
        if transforms is None:
            return
        control_points = self.define_control_points(transforms)
        if control_points is None:
            return
        segments = get_robot_segments_from_transforms(
            transforms, self.robot_cfg['segment_links'])
        if segments is None:
            return

        robot_mask = self.build_robot_mask_from_transforms(
            transforms, dilate_px=int(self.mask_cfg['robot_mask_dilate_px']))
        search_exclusion_mask = self.build_search_exclusion_mask(
            robot_mask, extra_px=int(self.mask_cfg['search_exclusion_extra_px']))

        roi_pad = int(self.distance_cfg['roi_pad_px'])
        if search_exclusion_mask is not None:
            ys, xs = np.where(search_exclusion_mask)
            if len(xs) > 0:
                x0 = max(margin, int(xs.min()) - roi_pad)
                x1 = min(W - margin, int(xs.max()) + roi_pad)
                y0 = max(margin, int(ys.min()) - roi_pad)
                y1 = min(H - margin, int(ys.max()) + roi_pad)
            else:
                x0, x1, y0, y1 = margin, W - margin, margin, H - margin
        else:
            x0, x1, y0, y1 = margin, W - margin, margin, H - margin

        cp_results = self.compute_closest_distance(
            control_points=control_points,
            x=np.array([x0, x1]),
            y=np.array([y0, y1]),
            step=step,
            search_exclusion_mask=search_exclusion_mask,
        )
        if cp_results is None:
            return

        thresholds = self.distance_cfg['thresholds']
        valid_results = [
            r for r in cp_results
            if r['distance'] < np.inf
            and thresholds['min_thresh'] <= r['distance'] <= thresholds['max_thresh']
        ]
        if not valid_results:
            return

        best = min(valid_results, key=lambda r: r['distance'])
        min_dist = best['distance']
        closest_point = best['closest_obstacle_point']
        closest_robot_point = best['point']
        closest_uv_obs = best['closest_pixel']
        closest_Z = self.transform_base_to_camera(closest_point)[2]

        self.get_logger().info(
            f'Min dist: {min_dist:.3f} m  Z: {closest_Z:.3f} m  obs@{closest_uv_obs}')

        # ── Visualization ────────────────────────────────────────────────
        depth_vis = cv2.normalize(self.last_depth, None, 0, 255, cv2.NORM_MINMAX)
        depth_vis = cv2.cvtColor(depth_vis.astype(np.uint8), cv2.COLOR_GRAY2BGR)

        if robot_mask is not None:
            depth_vis[robot_mask] = (0, 0, 0)
            contours, _ = cv2.findContours(
                robot_mask.astype(np.uint8) * 255,
                cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(depth_vis, contours, -1, (0, 255, 0), 3)

        for a, b in segments:
            uv_a = self.project_point_to_image(a)
            uv_b = self.project_point_to_image(b)
            if uv_a and uv_b:
                cv2.line(depth_vis, uv_a, uv_b, (0, 255, 0), 1)
                cv2.circle(depth_vis, uv_a, 2, (0, 255, 0), -1)
                cv2.circle(depth_vis, uv_b, 2, (0, 255, 0), -1)

        for r in cp_results:
            if r['distance'] == np.inf:
                continue
            uv_cp = self.project_point_to_image(r['point'])
            uv_obs = r['closest_pixel']
            if uv_cp and uv_obs:
                cv2.line(depth_vis, uv_cp, uv_obs, (255, 255, 255), 1)

        u, v = closest_uv_obs if closest_uv_obs else (20, 20)
        cv2.circle(depth_vis, (u, v), 5, (0, 0, 255), -1)

        uv_robot = self.project_point_to_image(closest_robot_point)
        if uv_robot and closest_uv_obs:
            cv2.circle(depth_vis, uv_robot, 5, (255, 255, 0), -1)
            cv2.line(depth_vis, closest_uv_obs, uv_robot, (255, 255, 255), 2)

        self.draw_control_points(depth_vis, control_points, color=(0, 165, 255))
        cv2.putText(depth_vis, f'{min_dist:.3f} m',
                    (u + 10, v), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        cv2.imshow('Robot + closest distance', depth_vis)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RealTimeDistance()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
