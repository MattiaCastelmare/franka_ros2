"""Lightweight ROS 2 visualizer for human-arm landmarks.

The node renders the newest camera frame only. Landmark detections are held for
short dropouts and smoothly interpolated between updates, so the overlay does
not disappear whenever MediaPipe misses a single frame.
"""

import os
import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image, PointCloud
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import CameraInfo
from franka_msgs.msg import MultiLinkDistance
from geometry_msgs.msg import Point

from franka_experiments.utils.distance_utils import load_robot_config
from franka_experiments.utils.human_utils import (
    draw_landmarks, landmarks_are_recent, stamp_to_ns,
    update_display_points, quaternion_to_rotation,
)


class HumanArmVisualizer(Node):
    LANDMARK_NAMES = ('shoulder', 'elbow', 'wrist', 'index')

    def __init__(self) -> None:
        config_path = os.path.join(
            get_package_share_directory('franka_experiments'),
            'config',
            'human_params.yaml',
        )
        full_config = load_robot_config(config_path)
        config = full_config['human_visualizer']
        use_sim_time = bool(
            full_config.get('common', {}).get('use_sim_time', True)
        )

        super().__init__(
            'human_arm_visualizer',
            parameter_overrides=[
                Parameter(
                    'use_sim_time',
                    Parameter.Type.BOOL,
                    use_sim_time,
                )
            ],
            automatically_declare_parameters_from_overrides=True,
        )

        color_topic = str(config['color_topic'])
        landmarks_topic = str(config['landmarks_topic'])
        overlay_topic = str(config['overlay_topic'])

        self.visibility_threshold = float(
            config['visibility_threshold']
        )
        self.max_hz = max(1.0, float(config['max_hz']))
        self.scale = float(config['scale'])
        self.scale = min(max(self.scale, 0.1), 1.0)
        self.landmark_hold_s = max(
            0.0,
            float(config['landmark_hold_s']),
        )
        self.smoothing_tau_s = max(
            0.0,
            float(config['smoothing_tau_s']),
        )
        self.draw_labels = bool(config['draw_labels'])

        self.bridge = CvBridge()
        self.latest_image_msg: Image | None = None
        self.last_rendered_stamp_ns: int | None = None

        # Latest valid MediaPipe result. Empty landmark messages do not erase it;
        # it expires naturally after landmark_hold_s
        self.target_points: np.ndarray | None = None
        self.display_points: np.ndarray | None = None
        self.visibilities = np.zeros(len(self.LANDMARK_NAMES), dtype=np.float32)
        self.last_valid_landmark_stamp_ns: int | None = None
        self.last_render_monotonic_ns: int | None = None

        # Camera intrinsics
        self.fx = self.fy = self.cx = self.cy = None
        self.camera_frame = None
        self.latest_distances = None
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # --- Subscribers ---
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_cb, 10)
            
        self.dist_sub = self.create_subscription(
            MultiLinkDistance, '/cbf/per_link_distances', self.dist_cb, 10)

        # Both subscriptions keep only one sample. A debug visualizer should
        # always prefer the newest data instead of processing an old queue
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_sub = self.create_subscription(
            Image,
            color_topic,
            self.image_cb,
            sensor_qos,
        )
        self.landmarks_sub = self.create_subscription(
            PointCloud,
            landmarks_topic,
            self.landmarks_cb,
            sensor_qos,
        )

        overlay_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.overlay_pub = self.create_publisher(
            Image,
            overlay_topic,
            overlay_qos,
        )

        self.render_timer = self.create_timer(
            1.0 / self.max_hz,
            self.render_latest,
        )

        self.get_logger().info(
            f'HumanArmVisualizer ready: image={color_topic}, '
            f'landmarks={landmarks_topic}, overlay={overlay_topic}, '
            f'max_hz={self.max_hz:.1f}, scale={self.scale:.2f}, '
            f'hold={self.landmark_hold_s:.2f}s'
        )

    def camera_info_cb(self, msg: CameraInfo):
        """Save camera intrinsics once."""
        if self.fx is None:
            self.fx, self.fy = msg.k[0], msg.k[4]
            self.cx, self.cy = msg.k[2], msg.k[5]
            self.camera_frame = msg.header.frame_id

    def dist_cb(self, msg: MultiLinkDistance):
        """Save the latest calculated distances."""
        self.latest_distances = msg

    def image_cb(self, msg: Image) -> None:
        """Store only the newest camera image."""
        self.latest_image_msg = msg

    def landmarks_cb(self, msg: PointCloud) -> None:
        """Accept complete detections. Per landmark, only move the target if
        MediaPipe currently trusts that point; a low-visibility landmark keeps
        its last confident position instead of jumping to a noisy/occluded
        guess."""
        if len(msg.points) < len(self.LANDMARK_NAMES):
            return

        points = np.asarray(
            [[point.x, point.y] for point in msg.points[:4]],
            dtype=np.float32,
        )
        if not np.all(np.isfinite(points)):
            return

        visibilities = np.zeros(len(self.LANDMARK_NAMES), dtype=np.float32)
        for channel in msg.channels:
            if channel.name == 'visibility':
                count = min(len(channel.values), len(self.LANDMARK_NAMES))
                if count > 0:
                    visibilities[:count] = np.asarray(
                        channel.values[:count],
                        dtype=np.float32,
                    )
                break

        if self.target_points is None:
            self.target_points = points.copy()
        else:
            trusted = visibilities >= self.visibility_threshold
            self.target_points[trusted] = points[trusted]

        self.visibilities = visibilities
        self.last_valid_landmark_stamp_ns = stamp_to_ns(msg)

        if self.display_points is None:
            self.display_points = self.target_points.copy()

    def render_latest(self) -> None:
        """Render the newest image and the most recent valid pose."""
        if self.overlay_pub.get_subscription_count() == 0:
            return

        image_msg = self.latest_image_msg
        if image_msg is None:
            return

        image_stamp_ns = stamp_to_ns(image_msg)
        if image_stamp_ns == self.last_rendered_stamp_ns:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding='bgr8',
            ).copy()
        except Exception as exc:
            self.get_logger().warn(
                f'Image conversion failed: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        if self.scale < 1.0:
            image = cv2.resize(
                image,
                dsize=None,
                fx=self.scale,
                fy=self.scale,
                interpolation=cv2.INTER_AREA,
            )

        if landmarks_are_recent(
            image_stamp_ns,
            self.last_valid_landmark_stamp_ns,
            self.landmark_hold_s,
        ):
            (
                self.display_points,
                self.last_render_monotonic_ns,
            ) = update_display_points(
                self.target_points,
                self.display_points,
                self.smoothing_tau_s,
                self.max_hz,
                self.last_render_monotonic_ns,
            )
            if self.display_points is not None:
                draw_landmarks(
                    image,
                    self.display_points,
                    self.visibilities,
                    self.LANDMARK_NAMES,
                    self.visibility_threshold,
                    self.scale,
                    self.draw_labels,
                )

        # --- DRAW SHORTEST DISTANCE LINE ---
        if self.latest_distances and self.fx is not None and self.camera_frame:
            try:
                # Find the link with the absolute minimum distance
                if self.latest_distances.links:
                    min_link = min(self.latest_distances.links, key=lambda l: l.distance)
                    
                    # Get TF from base (fr3_link0) to camera optical frame
                    tf_msg = self.tf_buffer.lookup_transform(
                        self.camera_frame, 
                        'fr3_link0', # Robot base frame
                        rclpy.time.Time()
                    )
                    
                    # Extract rotation and translation from TF
                    q = tf_msg.transform.rotation
                    R = quaternion_to_rotation(q.x, q.y, q.z, q.w)
                    t = np.array([tf_msg.transform.translation.x, 
                                  tf_msg.transform.translation.y, 
                                  tf_msg.transform.translation.z])
                    
                    def project_3d_to_2d(point_msg):
                        """Transforms 3D base point to camera 2D pixel."""
                        p_base = np.array([point_msg.x, point_msg.y, point_msg.z])
                        p_cam = R @ p_base + t
                        
                        # Only project if the point is in front of the camera (Z > 0)
                        if p_cam[2] > 0.01:
                            u = int((p_cam[0] / p_cam[2]) * self.fx + self.cx)
                            v = int((p_cam[1] / p_cam[2]) * self.fy + self.cy)
                            return (u, v)
                        return None

                    # Project both robot and human points
                    uv_robot = project_3d_to_2d(min_link.closest_point_robot)
                    uv_human = project_3d_to_2d(min_link.closest_point_human)
                    
                    if uv_robot and uv_human:
                        # Scale the coordinates if the image is resized
                        if self.scale < 1.0:
                            uv_robot = (int(uv_robot[0] * self.scale), int(uv_robot[1] * self.scale))
                            uv_human = (int(uv_human[0] * self.scale), int(uv_human[1] * self.scale))
                        
                        # Draw the white line and points
                        cv2.line(image, uv_robot, uv_human, (255, 255, 255), 1)
                        cv2.circle(image, uv_robot, 3, (0, 255, 255), -1) # Yellow dot on robot
                        cv2.circle(image, uv_human, 3, (0, 0, 255), -1)   # Red dot on human
            
            except Exception as e:
                self.get_logger().warn(f"Could not draw distance line: {e}", throttle_duration_sec=2.0)

        overlay_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        overlay_msg.header = image_msg.header
        self.overlay_pub.publish(overlay_msg)
        self.last_rendered_stamp_ns = image_stamp_ns


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HumanArmVisualizer()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()