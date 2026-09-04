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
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image, PointCloud, CameraInfo
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from franka_msgs.msg import HumanArmState, MultiLinkDistance

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
        use_sim_time = bool(full_config.get('common', {}).get('use_sim_time', True))

        super().__init__(
            'human_arm_visualizer',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)],
            automatically_declare_parameters_from_overrides=True,
        )

        color_topic = str(config['color_topic'])
        landmarks_topic = str(config['landmarks_topic'])
        overlay_topic = str(config['overlay_topic'])
        # Read camera info from config, default to color camera info if missing
        camera_info_topic = str(config.get('camera_info_topic', '/camera/camera/color/camera_info'))

        self.visibility_threshold = float(config['visibility_threshold'])
        self.max_hz = max(1.0, float(config['max_hz']))
        self.scale = float(config['scale'])
        self.scale = min(max(self.scale, 0.1), 1.0)
        self.landmark_hold_s = max(0.0, float(config['landmark_hold_s']))
        self.smoothing_tau_s = max(0.0, float(config['smoothing_tau_s']))
        self.draw_labels = bool(config['draw_labels'])

        self.bridge = CvBridge()
        self.latest_image_msg: Image | None = None
        self.last_rendered_stamp_ns: int | None = None

        self.target_points: np.ndarray | None = None
        self.display_points: np.ndarray | None = None
        self.visibilities = np.zeros(len(self.LANDMARK_NAMES), dtype=np.float32)
        self.last_valid_landmark_stamp_ns: int | None = None
        self.last_render_monotonic_ns: int | None = None

        # Camera intrinsics
        self.fx = self.fy = self.cx = self.cy = None
        self.camera_frame = None

        # --- 3D Visualization State ---
        self.latest_arm_state = None
        self.latest_distances = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- 3D Subscribers ---
        self.arm_state_sub = self.create_subscription(
            HumanArmState, '/human/arm_state', self.arm_state_cb, 10
        )
        self.dist_sub = self.create_subscription(
            MultiLinkDistance, '/cbf/per_link_distances', self.dist_cb, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_cb, 10
        )

        # --- 3D Publisher ---
        self.marker_pub = self.create_publisher(MarkerArray, '/human_robot/markers', 10)
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_sub = self.create_subscription(
            Image, color_topic, self.image_cb, sensor_qos
        )
        self.landmarks_sub = self.create_subscription(
            PointCloud, landmarks_topic, self.landmarks_cb, sensor_qos
        )

        overlay_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.overlay_pub = self.create_publisher(Image, overlay_topic, overlay_qos)

        self.render_timer = self.create_timer(1.0 / self.max_hz, self.render_latest)

        self.get_logger().info(
            f'HumanArmVisualizer ready: image={color_topic}, max_hz={self.max_hz:.1f}'
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

    def arm_state_cb(self, msg: HumanArmState) -> None:
        """Stores the latest 3D human arm state for RViz."""
        self.latest_arm_state = msg

    def landmarks_cb(self, msg: PointCloud) -> None:
        """Accept complete detections."""
        if len(msg.points) < len(self.LANDMARK_NAMES):
            return

        points = np.asarray([[p.x, p.y] for p in msg.points[:4]], dtype=np.float32)
        if not np.all(np.isfinite(points)):
            return

        visibilities = np.zeros(len(self.LANDMARK_NAMES), dtype=np.float32)
        for channel in msg.channels:
            if channel.name == 'visibility':
                count = min(len(channel.values), len(self.LANDMARK_NAMES))
                if count > 0:
                    visibilities[:count] = np.asarray(channel.values[:count], dtype=np.float32)
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

    def publish_3d_markers(self) -> None:
        """Generates and publishes 3D markers for RViz visualization."""
        if self.latest_arm_state is None:
            return

        marker_array = MarkerArray()
        base_frame = self.latest_arm_state.header.frame_id
        timestamp = self.get_clock().now().to_msg()

        # 1. --- HUMAN ARM MARKER ---
        arm_marker = Marker()
        arm_marker.header.frame_id = base_frame
        arm_marker.header.stamp = timestamp
        arm_marker.ns = "human_arm"
        arm_marker.id = 0
        arm_marker.type = Marker.LINE_STRIP
        arm_marker.action = Marker.ADD
        arm_marker.scale.x = 0.12 
        arm_marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.5)

        state = self.latest_arm_state
        pts_valid = state.keypoint_valid
        keypoints = [state.shoulder, state.elbow, state.wrist, state.hand]
        
        for i, pt in enumerate(keypoints):
            if pts_valid[i]:
                arm_marker.points.append(pt)
                
        marker_array.markers.append(arm_marker)

        # 2. --- ROBOT CONTROL POINTS & DISTANCE ARROWS ---
        if self.latest_distances is not None and self.latest_distances.links:
            
            # Draw a sphere for each control point on the robot
            for i, link in enumerate(self.latest_distances.links):
                sphere = Marker()
                sphere.header.frame_id = base_frame
                sphere.header.stamp = timestamp
                sphere.ns = "robot_points"
                sphere.id = i
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                sphere.pose.position = link.closest_point_robot
                sphere.scale.x = 0.06
                sphere.scale.y = 0.06
                sphere.scale.z = 0.06
                sphere.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.8) # Yellow
                marker_array.markers.append(sphere)

            # Find the absolute minimum distance link to highlight it
            min_link = min(self.latest_distances.links, key=lambda l: l.distance)
            
            # Draw an arrow for EACH link
            for i, link in enumerate(self.latest_distances.links):
                dist_marker = Marker()
                dist_marker.header.frame_id = base_frame
                dist_marker.header.stamp = timestamp
                dist_marker.ns = "distances"
                dist_marker.id = i + 100 # Offset to avoid ID conflicts
                dist_marker.type = Marker.ARROW
                dist_marker.action = Marker.ADD
                
                # Arrow points: from Human to Robot
                dist_marker.points.append(link.closest_point_human)
                dist_marker.points.append(link.closest_point_robot)
                
                is_min = (link == min_link)
                
                if is_min:
                    # HIGHLIGHTED: Thick arrow for the absolute minimum distance
                    dist_marker.scale.x = 0.02  # Shaft
                    dist_marker.scale.y = 0.04  # Head
                    dist_marker.scale.z = 0.04
                    if link.zone == 'critical':
                        dist_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
                    elif link.zone == 'danger':
                        dist_marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
                    else:
                        dist_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
                else:
                    # SECONDARY: Thin, semi-transparent grey line for other links
                    dist_marker.scale.x = 0.005 # Thin shaft
                    dist_marker.scale.y = 0.010 # Thin head
                    dist_marker.scale.z = 0.010
                    dist_marker.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.4)
                    
                marker_array.markers.append(dist_marker)

        self.marker_pub.publish(marker_array)

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
            image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8').copy()
        except Exception as exc:
            self.get_logger().warn(f'Image conversion failed: {exc}', throttle_duration_sec=2.0)
            return

        if self.scale < 1.0:
            image = cv2.resize(
                image, dsize=None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA
            )

        if landmarks_are_recent(image_stamp_ns, self.last_valid_landmark_stamp_ns, self.landmark_hold_s):
            self.display_points, self.last_render_monotonic_ns = update_display_points(
                self.target_points, self.display_points, self.smoothing_tau_s, self.max_hz, self.last_render_monotonic_ns
            )
            if self.display_points is not None:
                draw_landmarks(
                    image, self.display_points, self.visibilities, self.LANDMARK_NAMES, 
                    self.visibility_threshold, self.scale, self.draw_labels
                )

        # --- DRAW SHORTEST DISTANCE LINE ---
        if self.latest_distances and self.fx is not None and self.camera_frame:
            try:
                if self.latest_distances.links:
                    min_link = min(self.latest_distances.links, key=lambda l: l.distance)
                    
                    # Use the exact timestamp of the image to avoid TF errors with the bag!
                    tf_msg = self.tf_buffer.lookup_transform(
                        self.camera_frame, 
                        'fr3_link0', 
                        image_msg.header.stamp,
                        timeout=Duration(seconds=0.05)
                    )
                    
                    q = tf_msg.transform.rotation
                    R = quaternion_to_rotation(q.x, q.y, q.z, q.w)
                    t = np.array([tf_msg.transform.translation.x, 
                                  tf_msg.transform.translation.y, 
                                  tf_msg.transform.translation.z])
                    
                    def project_3d_to_2d(point_msg):
                        p_base = np.array([point_msg.x, point_msg.y, point_msg.z])
                        p_cam = R @ p_base + t
                        if p_cam[2] > 0.01:
                            u = int((p_cam[0] / p_cam[2]) * self.fx + self.cx)
                            v = int((p_cam[1] / p_cam[2]) * self.fy + self.cy)
                            return (u, v)
                        return None

                    uv_robot = project_3d_to_2d(min_link.closest_point_robot)
                    uv_human = project_3d_to_2d(min_link.closest_point_human)
                    
                    if uv_robot and uv_human:
                        if self.scale < 1.0:
                            uv_robot = (int(uv_robot[0] * self.scale), int(uv_robot[1] * self.scale))
                            uv_human = (int(uv_human[0] * self.scale), int(uv_human[1] * self.scale))
                        
                        cv2.line(image, uv_robot, uv_human, (255, 255, 255), 1)
                        cv2.circle(image, uv_robot, 3, (0, 255, 255), -1) 
                        cv2.circle(image, uv_human, 3, (0, 0, 255), -1)   
            
            except Exception as e:
                # This log will tell you if the line jumps due to TF or other issues
                self.get_logger().warn(f"Could not draw 2D distance line: {e}", throttle_duration_sec=2.0)

        overlay_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        overlay_msg.header = image_msg.header
        self.overlay_pub.publish(overlay_msg)
        self.last_rendered_stamp_ns = image_stamp_ns

        self.publish_3d_markers()


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