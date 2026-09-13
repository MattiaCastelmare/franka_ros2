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
from franka_msgs.msg import HumanArmState, MultiLinkDistance, HumanArmPrediction

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

        # --- Camera Intrinsics ---
        self.fx = self.fy = self.cx = self.cy = None
        self.camera_frame = None

        # --- 3D Visualization State ---
        self.latest_arm_state = None
        self.latest_distances = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Subscriptions ---
        self.arm_state_sub = self.create_subscription(
            HumanArmState, '/human/arm_state', self.arm_state_cb, 10
        )
        self.dist_sub = self.create_subscription(
            MultiLinkDistance, '/cbf/per_link_distances', self.dist_cb, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_cb, 10
        )
        self.pred_sub = self.create_subscription(
            HumanArmPrediction, '/human/arm_prediction', self.pred_cb, 10
        )

        # --- Publishers ---
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
        self.latest_distances = msg

    def image_cb(self, msg: Image) -> None:
        self.latest_image_msg = msg

    def arm_state_cb(self, msg: HumanArmState) -> None:
        self.latest_arm_state = msg

    def pred_cb(self, msg: HumanArmPrediction) -> None:
        self.latest_arm_prediction = msg

    def landmarks_cb(self, msg: PointCloud) -> None:
        """Accept complete MediaPipe detections."""
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

    # -------------------------------------------------------------------------
    # 3D Marker Generation
    # -------------------------------------------------------------------------
    def publish_3d_markers(self) -> None:
        """Generates and publishes human arm and distance arrows for RViz."""
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

        # 2. --- HUMAN ARM PREDICTION (FADING GHOST ARMS WITH JOINTS) ---
        if self.latest_arm_prediction is not None:
            pred = self.latest_arm_prediction
            pts_valid = pred.keypoint_valid
            
            # Limit the trail to the first 4 future steps to avoid visual clutter
            max_display_steps = 10
            display_steps = min(max_display_steps, pred.num_steps)
            
            # Iterate through each future step to create a "ghost arm"
            for step in range(display_steps):
                # Calculate fading alpha (transparency) based on the step index
                alpha = max(0.05, 0.4 - (0.15 * step))
                
                # Use Red color for prediction to indicate future danger zones
                pred_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=alpha)
                
                # Extract the 3D pose of the arm at this specific future instant
                keypoints_future = [
                    pred.shoulder[step], 
                    pred.elbow[step], 
                    pred.wrist[step], 
                    pred.hand[step]
                ]
                
                # Gather only the valid keypoints
                valid_points = []
                for i, pt in enumerate(keypoints_future):
                    if pts_valid[i]:
                        valid_points.append(pt)
                
                if len(valid_points) > 0:
                    # --- A. Ghost Arm Segments (LINE_STRIP) ---
                    lines_marker = Marker()
                    lines_marker.header.frame_id = base_frame
                    lines_marker.header.stamp = timestamp
                    lines_marker.ns = "human_prediction_lines"
                    lines_marker.id = step + 10  # Unique ID per step
                    lines_marker.type = Marker.LINE_STRIP
                    lines_marker.action = Marker.ADD
                    
                    # Line thickness (thinner than the actual blue arm)
                    lines_marker.scale.x = 0.04  
                    lines_marker.color = pred_color
                    lines_marker.points = valid_points
                    
                    marker_array.markers.append(lines_marker)
                    
                    # --- B. Ghost Arm Joints (SPHERE_LIST) ---
                    joints_marker = Marker()
                    joints_marker.header.frame_id = base_frame
                    joints_marker.header.stamp = timestamp
                    joints_marker.ns = "human_prediction_joints"
                    joints_marker.id = step + 50  # Unique ID per step (offset to avoid conflicts)
                    joints_marker.type = Marker.SPHERE_LIST
                    joints_marker.action = Marker.ADD
                    
                    # Sphere size (slightly larger than the line thickness to pop out)
                    joints_marker.scale.x = 0.06
                    joints_marker.scale.y = 0.06
                    joints_marker.scale.z = 0.06
                    joints_marker.color = pred_color
                    joints_marker.points = valid_points
                    
                    marker_array.markers.append(joints_marker)

        # 3. --- ROBOT CPs & DISTANCE ARROWS ---
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
            
            # Draw an arrow for each link
            for i, link in enumerate(self.latest_distances.links):
                dist_marker = Marker()
                dist_marker.header.frame_id = base_frame
                dist_marker.header.stamp = timestamp
                dist_marker.ns = "distances"
                dist_marker.id = i
                dist_marker.type = Marker.ARROW
                dist_marker.action = Marker.ADD
                
                # Arrow points: from Human to Robot
                dist_marker.points.append(link.closest_point_human)
                dist_marker.points.append(link.closest_point_robot)
                
                is_min = (link == min_link)
                
                if is_min:
                    # Highlight absolute minimum distance
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
                    # Secondary distances
                    dist_marker.scale.x = 0.005
                    dist_marker.scale.y = 0.010
                    dist_marker.scale.z = 0.010
                    dist_marker.color = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.4)
                    
                marker_array.markers.append(dist_marker)

        self.marker_pub.publish(marker_array)

    # -------------------------------------------------------------------------
    # 2D Projection Helpers
    # -------------------------------------------------------------------------
    def _project_point(self, point_msg, R: np.ndarray, t: np.ndarray):
        """Transforms a 3D base point into a 2D camera pixel."""
        p_base = np.array([point_msg.x, point_msg.y, point_msg.z])
        p_cam = R @ p_base + t
        
        # Check if the point is physically in front of the camera (Z > 0)
        if p_cam[2] > 0.01:
            u = int((p_cam[0] / p_cam[2]) * self.fx + self.cx)
            v = int((p_cam[1] / p_cam[2]) * self.fy + self.cy)
            return (u, v)
        return None

    def _draw_distance_line(self, image: np.ndarray, stamp) -> None:
        """Projects and draws the shortest geometric distance on the 2D overlay."""
        if not self.latest_distances or self.fx is None or not self.camera_frame:
            return
            
        if not self.latest_distances.links:
            return

        try:
            min_link = min(self.latest_distances.links, key=lambda l: l.distance)
            
            # Lookup TF from robot base to camera optical frame
            tf_msg = self.tf_buffer.lookup_transform(
                self.camera_frame, 
                'fr3_link0', 
                stamp,
                timeout=Duration(seconds=0.05)
            )
            
            q = tf_msg.transform.rotation
            R = quaternion_to_rotation(q.x, q.y, q.z, q.w)
            t = np.array([
                tf_msg.transform.translation.x, 
                tf_msg.transform.translation.y, 
                tf_msg.transform.translation.z
            ])
            
            # Project exact geometric centers to 2D
            uv_robot = self._project_point(min_link.closest_point_robot, R, t)
            uv_human = self._project_point(min_link.closest_point_human, R, t)
            
            if uv_robot and uv_human:
                if self.scale < 1.0:
                    uv_robot = (int(uv_robot[0] * self.scale), int(uv_robot[1] * self.scale))
                    uv_human = (int(uv_human[0] * self.scale), int(uv_human[1] * self.scale))
                
                # Draw the shortest distance line and the exact collision points
                cv2.line(image, uv_robot, uv_human, (255, 255, 255), 2)
                cv2.circle(image, uv_robot, 6, (0, 255, 255), -1)  # Yellow for robot
                cv2.circle(image, uv_human, 6, (0, 0, 255), -1)    # Red for human 

                # Add the Control Point name next to the robot point
                cp_name = min_link.robot_link_name
                text_pos = (uv_robot[0] + 8, uv_robot[1] - 8)
                cv2.putText(
                    image, cp_name, text_pos, 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA
                )

        except Exception:
            pass

    # -------------------------------------------------------------------------
    # Main Render Loop
    # -------------------------------------------------------------------------
    def render_latest(self) -> None:
        """Main rendering pipeline executed at a fixed frequency."""
        if self.overlay_pub.get_subscription_count() == 0:
            return

        image_msg = self.latest_image_msg
        if image_msg is None:
            return

        image_stamp_ns = stamp_to_ns(image_msg)
        if image_stamp_ns == self.last_rendered_stamp_ns:
            return

        # Convert Image
        try:
            image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8').copy()
        except Exception as exc:
            self.get_logger().warn(f'Image conversion failed: {exc}', throttle_duration_sec=2.0)
            return

        # Resize
        if self.scale < 1.0:
            image = cv2.resize(
                image, dsize=None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_AREA
            )

        # Draw Human Landmarks
        if landmarks_are_recent(image_stamp_ns, self.last_valid_landmark_stamp_ns, self.landmark_hold_s):
            self.display_points, self.last_render_monotonic_ns = update_display_points(
                self.target_points, self.display_points, self.smoothing_tau_s, self.max_hz, self.last_render_monotonic_ns
            )
            if self.display_points is not None:
                draw_landmarks(
                    image, self.display_points, self.visibilities, self.LANDMARK_NAMES, 
                    self.visibility_threshold, self.scale, self.draw_labels
                )

        # Draw Shortest Distance Line
        self._draw_distance_line(image, image_msg.header.stamp)

        # HUD: INFO PANEL (Horizontal Top Bar)
        img_h, img_w = image.shape[:2]
        overlay_box = image.copy()
        x, y, w, h = 0, 0, img_w, 45 
        cv2.rectangle(overlay_box, (x, y), (x + w, y + h), (0, 0, 0), -1)
        
        # Apply a semi-transparent overlay to the top bar
        cv2.addWeighted(overlay_box, 0.6, image, 0.4, 0, image)

        # Prepare the string for the Time
        timestamp_sec = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
        time_str = f"Time: {timestamp_sec:.2f} s"
        
        # Prepare the string for minimum distance
        if self.latest_distances and self.latest_distances.links:
            min_link = min(self.latest_distances.links, key=lambda l: l.distance)
            dist_str = f"Min Dist: {min_link.distance:.3f} m"
        else:
            dist_str = "Min Dist: --"
            
        # Prepare the string for joint speeds
        speeds = [0.0, 0.0, 0.0, 0.0]
        if self.latest_arm_state:
            state = self.latest_arm_state
            if hasattr(state, 'velocities') and len(state.velocities) >= 4:
                speeds = [np.linalg.norm([v.x, v.y, v.z]) for v in state.velocities]
            else:
                vel_fields = [
                    getattr(state, 'shoulder_vel', getattr(state, 'shoulder_velocity', None)),
                    getattr(state, 'elbow_vel', getattr(state, 'elbow_velocity', None)),
                    getattr(state, 'wrist_vel', getattr(state, 'wrist_velocity', None)),
                    getattr(state, 'hand_vel', getattr(state, 'hand_velocity', None))
                ]
                speeds = [np.linalg.norm([v.x, v.y, v.z]) if v else 0.0 for v in vel_fields]

        speeds_str = f"Speeds [m/s]: v_sh {speeds[0]:.2f} | v_el: {speeds[1]:.2f} | v_wr: {speeds[2]:.2f} | v_ha: {speeds[3]:.2f}"

        # Draw the text on the overlay
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        color = (255, 255, 255)
        thickness = 1
        
        # Time and distance
        cv2.putText(image, f"{time_str}   |   {dist_str}", (10, 18), 
                    font, font_scale, color, thickness, cv2.LINE_AA)
        # Speeds
        cv2.putText(image, speeds_str, (10, 38), 
                    font, font_scale, color, thickness, cv2.LINE_AA)
        
        # Publish 2D Overlay
        overlay_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        overlay_msg.header = image_msg.header
        self.overlay_pub.publish(overlay_msg)
        self.last_rendered_stamp_ns = image_stamp_ns

        # Generate and Publish 3D Markers
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