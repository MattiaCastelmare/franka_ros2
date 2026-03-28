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

        segments = self.get_robot_segments()
        if segments is None:
            return

        min_dist = np.inf
        closest_Z = None
        closest_point = None
        closest_robot_point = None

        for v in range(margin, H - margin, step):
            for u in range(margin, W - margin, step):
                
                # convert depth from mm to meters
                Z = float(self.last_depth[v, u]) / 1000.0
                if Z <= 0:
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

        if min_dist < np.inf:
            print(f"Min distance: {min_dist:.3f} m")
            print(f"Closest depth Z: {closest_Z:.3f} m")
            print(f"Closest point in base: {closest_point}")
            print(f"Closest point on robot: {closest_robot_point}")

            # visualize distance on depth image
            depth_vis = cv2.normalize(self.last_depth, None, 0, 255, cv2.NORM_MINMAX)
            depth_vis = depth_vis.astype(np.uint8)
            depth_vis = cv2.cvtColor(depth_vis, cv2.COLOR_GRAY2BGR)

            # draw robot semgents in green
            for a, b in segments:
                uv_a = self.project_point_to_image(a)
                uv_b = self.project_point_to_image(b)

                if uv_a is not None and uv_b is not None:
                    cv2.line(depth_vis, uv_a, uv_b, (0, 255, 0), 2)
                    cv2.circle(depth_vis, uv_a, 3, (0, 255, 0), -1)
                    cv2.circle(depth_vis, uv_b, 3, (0, 255, 0), -1)

            for link_name in self.link_mesh_samples.keys():
                points_base = self.get_link_mesh_points_in_base(link_name)
                if points_base is None:
                    continue

                for p in points_base:
                    uv = self.project_point_to_image(p)
                    if uv is not None:
                        cv2.circle(depth_vis, uv, 1, (0, 255, 0), -1)

            # closest point on obstacle in red
            uv_obs = self.project_point_to_image(closest_point)
            if uv_obs is not None:
                u, v = uv_obs
                cv2.circle(depth_vis, (u, v), 5, (0, 0, 255), -1)
            else:
                u, v = 20, 20
                cv2.circle(depth_vis, (u, v), 5, (0, 0, 255), -1)

            # closest point on robot in blue
            uv_robot = self.project_point_to_image(closest_robot_point)
            if uv_robot is not None:
                cv2.circle(depth_vis, uv_robot, 5, (0, 0, 255), -1)

                # euclidean distance line in yellow
                cv2.line(depth_vis, (u, v), uv_robot, (0, 0, 255), 2)
                
            # plot distance text
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
        
    def get_robot_segments(self):
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
            p = self.get_link_position(name) # get position of each link in the base frame
            if p is None:
                return None
            points.append(p)

        segments = []
        for i in range(len(points) - 1):
            segments.append((points[i], points[i + 1])) # create segments between consecutive links
        return segments
    
    def get_link_mesh_points_in_base(self, link_name):
        R, t = self.get_link_rotation_translation(link_name)
        if R is None:
            return None

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
    
def main(args=None):
    rclpy.init(args=args)
    distance_calculator = RealTimeDistance()
    rclpy.spin(distance_calculator)
    distance_calculator.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
