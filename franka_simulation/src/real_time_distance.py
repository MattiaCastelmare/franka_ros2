from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
from cv_bridge import CvBridge
import cv2
import yaml
from tf2_ros import Buffer, TransformListener, TransformException
import trimesh
from utils import load_extrinsics, load_robot_config, get_robot_segments_from_transforms, get_rotation_from_quaternion


class RealTimeDistance(Node):
    def __init__(self):
        super().__init__('real_time_distance')
        self.bridge = CvBridge()
        self.fx = self.fy = None
        self.cx = self.cy = None
        self.last_depth = None
        self.last_depth_msg = None
        self.R_base, self.t_base = load_extrinsics()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10)
        
        self.create_subscription(
            CameraInfo,
            '/camera/camera/depth/camera_info',
            self.camera_info_callback,
            10)
        
        self.config = load_robot_config()
        self.robot_cfg = self.config['robot']
        self.distance_cfg = self.config['distance']
        self.mask_cfg = self.config['mask']
        self.mesh_cfg = self.config['meshes']
        self.topics_cfg = self.config['topics']
        
        # Load mesh files for each link and sample points on them
        self.link_mesh_files = self.mesh_cfg['files']
        self.link_meshes = {}
        self.link_mesh_samples = {}
        for link_name, mesh_path in self.link_mesh_files.items():
            mesh = trimesh.load(mesh_path, force='mesh')
            self.link_meshes[link_name] = mesh
            self.link_mesh_samples[link_name] = mesh.sample(300)

        self.create_timer(0.1, self.process_depth)


    # === Camera Callbacks ===
    def depth_callback(self, msg):
        self.last_depth_msg = msg
        # convert ROS Image to OpenCV image
        self.last_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def camera_info_callback(self, msg):
        if self.fx is not None:
            return
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        self.K = np.array(msg.k, dtype=float).reshape(3, 3)
        self.K_inv = np.linalg.inv(self.K)


    # === Transform functions ===
    def get_link_position(self, link_name):
        R, t = self.get_link_rotation_translation(link_name)
        if t is None:
            return None
        return t
        
    def get_link_rotation_translation(self, link_name):
        try:
            # lookup transform from base frame to link frame at the self.last_depth_msg = msg
            tf = self.tf_buffer.lookup_transform(
                'fr3_link0',
                link_name,
                self.last_depth_msg.header.stamp,
            )

            t = tf.transform.translation
            q = tf.transform.rotation
            R = get_rotation_from_quaternion(q)

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
        points_base = (R @ points_local.T).T + t
        return points_base
        
    def transform_camera_to_base(self, p_cam):
        return self.R_base @ np.array(p_cam) + self.t_base    

    def transform_base_to_camera(self, p_base):
        return self.R_base.T @ (np.array(p_base) - self.t_base)

    def project_point_to_image(self, p_base):
        # project a 3D point in the base frame to pixel coordinates in the depth image
        p_cam = self.transform_base_to_camera(p_base)
        if p_cam is None:
            return None

        if p_cam[2] <= 0:
            return None

        uv = self.K @ p_cam
        u = int(uv[0] / uv[2])
        v = int(uv[1] / uv[2])

        if 0 <= u < self.last_depth.shape[1] and 0 <= v < self.last_depth.shape[0]:
            return (u, v)

        return None


    # === Robot Mask ===
    def build_robot_mask_from_transforms(self, transforms, dilate_px=10):
        if self.last_depth is None:
            return None

        H, W = self.last_depth.shape
        mask = np.zeros((H, W), dtype=np.uint8)

        for link_name in self.link_mesh_samples.keys():
            points_base = self.get_link_mesh_points_in_base_from_transforms(link_name, transforms)
            if points_base is None:
                continue

            for p in points_base:
                uv = self.project_point_to_image(p)
                if uv is not None:
                    cv2.circle(mask, uv, 2, 255, -1)

        if dilate_px > 0:
            k = 2 * dilate_px + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            mask = cv2.dilate(mask, kernel)

        return mask > 0

    def build_search_exclusion_mask(self, robot_mask, extra_px=12):
        if robot_mask is None:
            return None

        excl = (robot_mask.astype(np.uint8) * 255)
        if extra_px > 0:
            k = 2 * extra_px + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            excl = cv2.dilate(excl, kernel)

        return excl > 0
        

    # === Control Points ===
    def define_control_points(self, transforms):
        control_points = []

        for seg in self.robot_cfg['segments']:
            seg_idx = int(seg['seg_idx'])
            start_link = seg['start_link']
            end_link = seg['end_link']
            n_cp = int(seg.get('control_points', 0))
            radius = float(seg.get('radius', 0.0))

            if n_cp <= 0:
                continue

            if start_link not in transforms or end_link not in transforms:
                continue

            _, p0 = transforms[start_link]
            _, p1 = transforms[end_link]

            for k in range(n_cp):
                t = (k + 1) / (n_cp + 1)
                p = p0 + t * (p1 - p0)

                control_points.append({
                    'point': p,
                    'seg_idx': seg_idx,
                    'cp_idx': k,
                    'radius': radius,
                    'start_link': start_link,
                    'end_link': end_link
                })

        if len(control_points) == 0:
            return None

        return control_points
    
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

                uv1 = np.array([u, v, 1.0], dtype=float)
                p_cam = Z * (self.K_inv @ uv1)
                p_obs = self.transform_camera_to_base(p_cam)

                if p_obs is None:
                    continue

                raw_dists = np.linalg.norm(cp_positions - p_obs, axis=1)
                dists = np.maximum(raw_dists - radii, 0.0)

                better = dists < min_dists
                for i in np.where(better)[0]:
                    min_dists[i] = dists[i]
                    closest_obs_points[i] = p_obs
                    closest_obs_pixels[i] = (u, v)

        results = []
        for i in range(n_cp):
            cp = control_points[i]
            results.append({
                'point': cp['point'],
                'seg_idx': cp['seg_idx'],
                'cp_idx': cp['cp_idx'],
                'radius': cp['radius'],
                'start_link': cp['start_link'],
                'end_link': cp['end_link'],
                'distance': min_dists[i],
                'closest_obstacle_point': closest_obs_points[i],
                'closest_pixel': closest_obs_pixels[i]
            })

        return results


    # === Main Loop ===
    def process_depth(self):
        if self.last_depth is None or self.last_depth_msg is None or self.fx is None:
            return

        H, W = self.last_depth.shape
        step = int(self.distance_cfg['pixel_step'])
        margin = int(self.distance_cfg['image_margin_px'])
        
        # Get all link transforms
        transforms = self.get_all_link_transforms()
        if transforms is None:
            return
        
        # Define control points on the robot based on the current transforms
        control_points = self.define_control_points(transforms)
        if control_points is None:
            return

        # Get robot segments from link transforms
        segments = get_robot_segments_from_transforms(transforms)
        if segments is None:
            return

        # Build robot mask and search exclusion mask
        robot_mask = self.build_robot_mask_from_transforms(
            transforms,
            dilate_px=int(self.mask_cfg['robot_mask_dilate_px'])
        )
        search_exclusion_mask = self.build_search_exclusion_mask(
            robot_mask,
            extra_px=int(self.mask_cfg['search_exclusion_extra_px'])
        )

        # Search only in a ROI around the robot, instead of the whole image
        roi_pad = int(self.distance_cfg['roi_pad_px'])  # pixels

        if search_exclusion_mask is not None:
            # Find bounding box of the robot in the image
            ys, xs = np.where(search_exclusion_mask)

            # Define ROI around the robot with some padding, but keep it within image bounds
            if len(xs) > 0 and len(ys) > 0:
                x0 = max(margin, int(xs.min()) - roi_pad)
                x1 = min(W - margin, int(xs.max()) + roi_pad)
                y0 = max(margin, int(ys.min()) - roi_pad)
                y1 = min(H - margin, int(ys.max()) + roi_pad)
            else:
                # If robot is not found, just search the whole image with margins
                x0, x1 = margin, W - margin
                y0, y1 = margin, H - margin
        else:
            # If we don't have a robot mask, search the whole image with margins
            x0, x1 = margin, W - margin
            y0, y1 = margin, H - margin

        x, y = np.array([x0, x1]), np.array([y0, y1])

        # Compute closest distance from control points to obstacles in the depth image
        cp_results = self.compute_closest_distance(
            control_points=control_points,
            x=x,
            y=y,
            step=step,
            search_exclusion_mask=search_exclusion_mask
        )

        if cp_results is None:
            return

        min_dist = np.inf
        closest_Z = None
        closest_point = None
        closest_robot_point = None
        closest_uv_obs = None

        # Filter out invalid results where distance is still inf
        valid_results = [r for r in cp_results if r["distance"] < np.inf]
        if len(valid_results) == 0:
            return

        # Find the control point with the minimum distance to an obstacle
        best_result = min(valid_results, key=lambda r: r["distance"])

        # Extract info from best result
        min_dist = best_result["distance"]
        closest_point = best_result["closest_obstacle_point"]
        closest_robot_point = best_result["point"]
        closest_uv_obs = best_result["closest_pixel"]
        p_cam_closest = self.transform_base_to_camera(closest_point)
        closest_Z = p_cam_closest[2]

        # Print results and visualize
        if min_dist < np.inf:
            print(f"Min distance: {min_dist:.3f} m")
            print(f"Closest depth Z: {closest_Z:.3f} m")
            print(f"Closest point in base: {closest_point}")
            print(f"Closest point on robot: {closest_robot_point}")
            print(f"Closest pixel: {closest_uv_obs}")

            # Visualize distance on depth image
            depth_vis = cv2.normalize(self.last_depth, None, 0, 255, cv2.NORM_MINMAX)
            depth_vis = depth_vis.astype(np.uint8)
            depth_vis = cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)

            if robot_mask is not None:
                depth_vis[robot_mask] = (0, 0, 0)
            if robot_mask is not None:
                contours, _ = cv2.findContours(
                    (robot_mask.astype(np.uint8) * 255),
                    cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE
                )
                cv2.drawContours(depth_vis, contours, -1, (0, 255, 0), 3)

            # Draw robot segments in green
            for a, b in segments:
                uv_a = self.project_point_to_image(a)
                uv_b = self.project_point_to_image(b)

                if uv_a is not None and uv_b is not None:
                    cv2.line(depth_vis, uv_a, uv_b, (0, 255, 0), 1)
                    cv2.circle(depth_vis, uv_a, 2, (0, 255, 0), -1)
                    cv2.circle(depth_vis, uv_b, 2, (0, 255, 0), -1)

            # Draw ALL control point distances (thin white lines)
            for r in cp_results:
                if r["distance"] == np.inf:
                    continue

                uv_cp = self.project_point_to_image(r["point"])
                uv_obs = r["closest_pixel"]

                if uv_cp is not None and uv_obs is not None:
                    cv2.line(depth_vis, uv_cp, uv_obs, (255, 255, 255), 1)

            # Closest obstacle pixel in red
            if closest_uv_obs is not None:
                u, v = closest_uv_obs
                cv2.circle(depth_vis, (u, v), 5, (0, 0, 255), -1)
            else:
                u, v = 20, 20
                cv2.circle(depth_vis, (u, v), 5, (0, 0, 255), -1)

            # Closest point on robot in cyan
            uv_robot = self.project_point_to_image(closest_robot_point)
            if uv_robot is not None and closest_uv_obs is not None:
                cv2.circle(depth_vis, uv_robot, 5, (255, 255, 0), -1)

                # Distance segment in white
                cv2.line(depth_vis, closest_uv_obs, uv_robot, (255, 255, 255), 2)

            # Draw control points in orange
            self.draw_control_points(depth_vis, control_points, color=(0, 165, 255))

            # Distance text
            cv2.putText(
                depth_vis,
                f"{min_dist:.3f} m",
                (u + 10, v),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                1
            )

            cv2.imshow("Robot + closest distance", depth_vis)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    distance_calculator = RealTimeDistance()
    rclpy.spin(distance_calculator)
    distance_calculator.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()