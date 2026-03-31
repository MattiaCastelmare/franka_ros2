from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
from cv_bridge import CvBridge
import cv2
import yaml
from tf2_ros import Buffer, TransformListener, TransformException
import trimesh


class RealTimeDistance(Node):
    def __init__(self):
        super().__init__('real_time_distance')
        self.bridge = CvBridge()
        self.fx = self.fy = None
        self.cx = self.cy = None
        self.last_depth = None
        self.last_depth_msg = None
        self.R_base, self.t_base = self.load_extrinsics()
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
        
        self.link_mesh_files = {
            'fr3_link0': '/ros2_ws/src/franka_description/meshes/robot_arms/fr3/collision/link0.stl',
            'fr3_link1': '/ros2_ws/src/franka_description/meshes/robot_arms/fr3/collision/link1.stl',
            'fr3_link2': '/ros2_ws/src/franka_description/meshes/robot_arms/fr3/collision/link2.stl',
            'fr3_link3': '/ros2_ws/src/franka_description/meshes/robot_arms/fr3/collision/link3.stl',
            'fr3_link4': '/ros2_ws/src/franka_description/meshes/robot_arms/fr3/collision/link4.stl',
            'fr3_link5': '/ros2_ws/src/franka_description/meshes/robot_arms/fr3/collision/link5.stl',
            'fr3_link6': '/ros2_ws/src/franka_description/meshes/robot_arms/fr3/collision/link6.stl',
            'fr3_link7': '/ros2_ws/src/franka_description/meshes/robot_arms/fr3/collision/link7.stl',
        }
        self.link_meshes = {}
        self.link_mesh_samples = {}
        for link_name, mesh_path in self.link_mesh_files.items():
            mesh = trimesh.load(mesh_path, force='mesh')
            self.link_meshes[link_name] = mesh
            self.link_mesh_samples[link_name] = mesh.sample(300)

        self.create_timer(0.1, self.process_depth)

    def load_extrinsics(self):
        filename = '/ros2_ws/src/franka_experiments/config/camera_extrinsics.yaml'
        with open(filename, 'r') as f:
            data = yaml.safe_load(f)

        tx = data['translation']['x']
        ty = data['translation']['y']
        tz = data['translation']['z']

        qx = data['rotation']['x']
        qy = data['rotation']['y']
        qz = data['rotation']['z']
        qw = data['rotation']['w']

        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
        ], dtype=float)

        t = np.array([tx, ty, tz], dtype=float)

        return R, t

    def depth_callback(self, msg):
        self.last_depth_msg = msg
        # convert ROS Image to OpenCV image
        self.last_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        
    def process_depth(self):
        if self.last_depth is None or self.last_depth_msg is None or self.fx is None:
            return

        H, W = self.last_depth.shape
        step = 8
        margin = 10

        transforms = self.get_all_link_transforms()
        if transforms is None:
            return

        segments = self.get_robot_segments_from_transforms(transforms)
        if segments is None:
            return

        robot_mask = self.build_robot_mask_from_transforms(transforms, dilate_px=8)
        search_exclusion_mask = self.build_search_exclusion_mask(robot_mask, extra_px=10)

        # Search only in a ROI around the robot, instead of the whole image
        roi_pad = 80  # pixels around the robot area

        if search_exclusion_mask is not None:
            ys, xs = np.where(search_exclusion_mask)

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

        min_dist = np.inf
        closest_Z = None
        closest_point = None
        closest_robot_point = None
        closest_uv_obs = None

        for v in range(y0, y1, step):
            for u in range(x0, x1, step):
                if search_exclusion_mask is not None and search_exclusion_mask[v, u]:
                    continue
                # convert depth from mm to meters
                Z = float(self.last_depth[v, u]) / 1000.0
                if Z < 0.15 or Z > 3.0:
                    continue

                uv1 = np.array([u, v, 1.0], dtype=float) # pixel homogeneous coordinates
                p_cam = Z * (self.K_inv @ uv1) # back-project pixel to camera coordinates
                p = self.transform_camera_to_base(p_cam) # transform point from camera to base coordinates
                if p is None:
                    continue 

                for a, b in segments:
                    # compute distance from point p to segment ab and its projection
                    d, proj = self.point_to_segment_distance_with_projection(p, a, b) 

                    if d < min_dist:
                        min_dist = d
                        closest_Z = Z
                        closest_point = p
                        closest_robot_point = proj
                        closest_uv_obs = (u, v)

        if min_dist < np.inf:
            print(f"Min distance: {min_dist:.3f} m")
            print(f"Closest depth Z: {closest_Z:.3f} m")
            print(f"Closest point in base: {closest_point}")
            print(f"Closest point on robot: {closest_robot_point}")
            print(f"Closest pixel: {closest_uv_obs}")

            # visualize distance on depth image
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
                cv2.drawContours(depth_vis, contours, -1, (0, 255, 0), 1)

            # draw robot segments in green
            for a, b in segments:
                uv_a = self.project_point_to_image(a)
                uv_b = self.project_point_to_image(b)

                if uv_a is not None and uv_b is not None:
                    cv2.line(depth_vis, uv_a, uv_b, (0, 255, 0), 1)
                    cv2.circle(depth_vis, uv_a, 2, (0, 255, 0), -1)
                    cv2.circle(depth_vis, uv_b, 2, (0, 255, 0), -1)

            for link_name in self.link_mesh_samples.keys():
                points_base = self.get_link_mesh_points_in_base_from_transforms(link_name, transforms)
                if points_base is None:
                    continue

                for p in points_base:
                    uv = self.project_point_to_image(p)
                    if uv is not None:
                        cv2.circle(depth_vis, uv, 1, (0, 255, 0), -1)

            # closest obstacle pixel in red
            if closest_uv_obs is not None:
                u, v = closest_uv_obs
                cv2.circle(depth_vis, (u, v), 5, (0, 0, 255), -1)
            else:
                u, v = 20, 20
                cv2.circle(depth_vis, (u, v), 5, (0, 0, 255), -1)

            # closest point on robot in cyan
            uv_robot = self.project_point_to_image(closest_robot_point)
            if uv_robot is not None and closest_uv_obs is not None:
                cv2.circle(depth_vis, uv_robot, 5, (255, 255, 0), -1)

                # distance segment in white
                cv2.line(depth_vis, closest_uv_obs, uv_robot, (255, 255, 255), 2)

            # distance text
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

    def point_to_segment_distance_with_projection(self, p, a, b):
        ab = b - a # segment vector
        denom = np.dot(ab, ab) # norm of segment

        if denom < 1e-12: # if segment is a point, then distance is just distance to that point
            return np.linalg.norm(p - a), a

        t = np.dot(p - a, ab) / denom # point p projected onto line defined by a and b, expressed as t in [0, 1]
        t = np.clip(t, 0.0, 1.0) # clamp to segment

        proj = a + t * ab # projection of p onto segment
        d = np.linalg.norm(p - proj) # distance from p to its projection on the segment
        return d, proj

    def camera_info_callback(self, msg):
        if self.fx is not None:
            return
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        self.K = np.array(msg.k, dtype=float).reshape(3, 3)
        self.K_inv = np.linalg.inv(self.K)

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
            x, y, z, w = q.x, q.y, q.z, q.w

            R = np.array([
                [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
                [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]
            ], dtype=float)

            t_vec = np.array([t.x, t.y, t.z], dtype=float)

            return R, t_vec

        except TransformException:
            return None, None
        
    def get_all_link_transforms(self):
        link_names = [
            'fr3_link0',
            'fr3_link1',
            'fr3_link2',
            'fr3_link3',
            'fr3_link4',
            'fr3_link5',
            'fr3_link6',
            'fr3_link7',
            'fr3_link8'
        ]

        transforms = {}
        for name in link_names:
            R, t = self.get_link_rotation_translation(name)
            if R is None or t is None:
                return None
            transforms[name] = (R, t)

        return transforms
        
    def get_robot_segments_from_transforms(self, transforms):
        link_names = [
            'fr3_link1',
            'fr3_link2',
            'fr3_link3',
            'fr3_link4',
            'fr3_link5',
            'fr3_link6',
            'fr3_link7',
            'fr3_link8'
        ]

        points = []
        for name in link_names:
            if name not in transforms:
                return None
            _, t = transforms[name]
            points.append(t)

        segments = []
        for i in range(len(points) - 1):
            segments.append((points[i], points[i + 1]))

        return segments
    
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


def main(args=None):
    rclpy.init(args=args)
    distance_calculator = RealTimeDistance()
    rclpy.spin(distance_calculator)
    distance_calculator.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()