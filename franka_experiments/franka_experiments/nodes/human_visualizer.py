#!/usr/bin/env python3
"""Lightweight ROS 2 visualizer for human-arm landmarks.

With sync_to_landmarks the overlay is drawn on the newest camera frame already
processed by the tracker (constant delay, landmarks aligned with the image).
Otherwise it renders the newest frame, with landmarks held for short dropouts
and smoothly interpolated between updates.
Supports both single arm tracking and dual arm ('both') tracking dynamically.
"""

import os
import cv2
import numpy as np
import rclpy
from functools import partial
from collections import deque
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image, PointCloud, CameraInfo
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from franka_msgs.msg import HumanArmState, MultiLinkDistance, HumanArmPrediction

from franka_experiments.utils.distance_utils import load_robot_config
from franka_experiments.utils.human_utils import (
    draw_landmarks, landmarks_are_recent, stamp_to_ns,
    update_display_points, quaternion_to_rotation, format_topic
)


class HumanArmVisualizer(Node):
    LANDMARK_NAMES = ('shoulder', 'elbow', 'wrist', 'index')

    def __init__(self) -> None:
        super().__init__('human_arm_visualizer')

        # --- Load Config ---
        config_path = os.path.join(
            get_package_share_directory('franka_experiments'),
            'config',
            'human_params.yaml',
        )
        full_config = load_robot_config(config_path)
        config = full_config['human_visualizer']

        color_topic = str(config['color_topic'])
        overlay_topic = str(config['overlay_topic'])
        camera_info_topic = str(config.get('camera_info_topic', '/camera/camera/color/camera_info'))

        # Fetch pose_side from tracker config to know if we are in single or dual mode
        tracker_config = full_config.get('human_tracker', {})
        self.pose_side = str(tracker_config.get('pose_side', 'right')).lower()
        self.active_sides = ["left", "right"] if self.pose_side == "both" else [self.pose_side]

        self.visibility_threshold = float(config['visibility_threshold'])
        self.max_hz = max(1.0, float(config['max_hz']))
        self.scale = float(config['scale'])
        self.scale = min(max(self.scale, 0.1), 1.0)
        self.landmark_hold_s = max(0.0, float(config['landmark_hold_s']))
        self.smoothing_tau_s = max(0.0, float(config['smoothing_tau_s']))
        self.draw_labels = bool(config['draw_labels'])
        # Draw on the frame the landmarks were detected on, not the newest one
        self.sync_to_landmarks = bool(config.get('sync_to_landmarks', True))

        # --- OpenCV Bridge ---
        self.bridge = CvBridge()
        self.image_buffer: deque[Image] = deque(maxlen=30)
        self.last_rendered_stamp_ns: int | None = None

        # --- Dictionaries for 2D states ---
        self.target_points = {side: None for side in self.active_sides}
        self.display_points = {side: None for side in self.active_sides}
        self.visibilities = {side: np.zeros(len(self.LANDMARK_NAMES), dtype=np.float32) for side in self.active_sides}
        self.last_valid_landmark_stamp_ns = {side: None for side in self.active_sides}
        self.last_render_monotonic_ns = {side: None for side in self.active_sides}
        # Stamp of the latest frame processed by the tracker, per side (empty detections included)
        self.processed_stamp_ns = {side: None for side in self.active_sides}

        # --- Camera Intrinsics ---
        self.fx = self.fy = self.cx = self.cy = None
        self.camera_frame = None
        self.base_to_camera = None  # cached static TF (R, t)

        # --- 3D Visualization State ---
        self.latest_arm_states = {side: None for side in self.active_sides}
        self.latest_arm_predictions = {side: None for side in self.active_sides}
        self.latest_distances = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Subscriptions ---
        latest_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.dist_sub = self.create_subscription(
            MultiLinkDistance, '/human/per_link_distances', self.dist_cb, latest_qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_cb, 10
        )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_sub = self.create_subscription(
            Image, color_topic, self.image_cb, sensor_qos
        )

        # --- Dynamic Subscriptions for Arms ---
        self.arm_state_subs = {}
        self.pred_subs = {}
        self.landmarks_subs = {}

        for side in self.active_sides:
            prefix = f"{side}_" if self.pose_side == "both" else ""

            s_topic = format_topic('/human/arm_state', prefix)
            p_topic = format_topic('/human/arm_prediction', prefix)
            l2d_topic = format_topic(str(config['landmarks_topic']), prefix)

            self.arm_state_subs[side] = self.create_subscription(
                HumanArmState, s_topic, partial(self.arm_state_cb, side=side), 10
            )
            self.pred_subs[side] = self.create_subscription(
                HumanArmPrediction, p_topic, partial(self.pred_cb, side=side), 10
            )
            self.landmarks_subs[side] = self.create_subscription(
                PointCloud, l2d_topic, partial(self.landmarks_cb, side=side), sensor_qos
            )

        # --- Publishers ---    
        overlay_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.overlay_pub = self.create_publisher(Image, overlay_topic, overlay_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/human_robot/markers', 10)
        # Overlay is event-driven (image / landmarks callbacks), markers run on a timer
        self.marker_timer = self.create_timer(1.0 / self.max_hz, self.publish_markers_cb)

        self.get_logger().info(
            f'HumanArmVisualizer ready: image={color_topic}, max_hz={self.max_hz:.1f}, mode={self.pose_side}'
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
        # Time jumped back (e.g. rosbag loop): restart from scratch
        if self.image_buffer and stamp_to_ns(msg) < stamp_to_ns(self.image_buffer[-1]):
            self.image_buffer.clear()
            self.last_rendered_stamp_ns = None
            self.processed_stamp_ns = {side: None for side in self.active_sides}
        self.image_buffer.append(msg)
        self.render_latest()

    def select_image(self) -> Image | None:
        """Newest frame already processed by the tracker, or the newest one if it is silent."""
        if not self.image_buffer:
            return None
        newest = self.image_buffer[-1]
        if not self.sync_to_landmarks:
            return newest

        # A frame is ready when every side has been published for it
        stamps = list(self.processed_stamp_ns.values())
        if any(s is None for s in stamps):
            return newest
        ready_ns = min(stamps)
        # Tracker not publishing anymore: fall back to the live image
        if not landmarks_are_recent(stamp_to_ns(newest), ready_ns, self.landmark_hold_s):
            return newest
        for msg in reversed(self.image_buffer):
            if stamp_to_ns(msg) <= ready_ns:
                return msg
        return None

    def distances_are_recent(self, image_stamp_ns: int) -> bool:
        """True if the latest distances refer to a frame close to the rendered one."""
        if self.latest_distances is None:
            return False
        return landmarks_are_recent(
            image_stamp_ns, stamp_to_ns(self.latest_distances), self.landmark_hold_s
        )

    def lookup_base_to_camera(self):
        """Static base -> camera transform, cached after the first lookup."""
        if self.base_to_camera is None:
            tf_msg = self.tf_buffer.lookup_transform(
                self.camera_frame, 'fr3_link0', rclpy.time.Time(),
                timeout=Duration(seconds=0.0)
            )
            q = tf_msg.transform.rotation
            tr = tf_msg.transform.translation
            self.base_to_camera = (
                quaternion_to_rotation(q.x, q.y, q.z, q.w),
                np.array([tr.x, tr.y, tr.z]),
            )
        return self.base_to_camera

    def arm_state_cb(self, msg: HumanArmState, side: str) -> None:
        self.latest_arm_states[side] = msg

    def pred_cb(self, msg: HumanArmPrediction, side: str) -> None:
        self.latest_arm_predictions[side] = msg

    def landmarks_cb(self, msg: PointCloud, side: str) -> None:
        """Mark the frame as processed and accept complete MediaPipe detections."""
        self.processed_stamp_ns[side] = stamp_to_ns(msg)
        self.update_landmarks(msg, side)
        self.render_latest()

    def update_landmarks(self, msg: PointCloud, side: str) -> None:
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

        if self.target_points[side] is None:
            self.target_points[side] = points.copy()
        else:
            trusted = visibilities >= self.visibility_threshold
            self.target_points[side][trusted] = points[trusted]

        self.visibilities[side] = visibilities
        self.last_valid_landmark_stamp_ns[side] = stamp_to_ns(msg)

        if self.display_points[side] is None:
            self.display_points[side] = self.target_points[side].copy()

    # -------------------------------------------------------------------------
    # 3D Marker Generation
    # -------------------------------------------------------------------------
    def publish_markers_cb(self) -> None:
        if self.marker_pub.get_subscription_count() > 0:
            self.publish_3d_markers()

    def publish_3d_markers(self) -> None:
        """Generates and publishes human arm and distance arrows for RViz."""
        
        base_frame = None
        for side in self.active_sides:
            if self.latest_arm_states[side] is not None:
                base_frame = self.latest_arm_states[side].header.frame_id
                break
        
        if base_frame is None:
            base_frame = "fr3_link0"

        marker_array = MarkerArray()
        timestamp = self.get_clock().now().to_msg()

        # Iterate over all tracked arms to draw segments, velocity vectors and ghosts
        for side in self.active_sides:
            state = self.latest_arm_states[side]
            if state is None:
                continue
                
            pts_valid = state.keypoint_valid

            # --- Delete Markers if arm is lost ---
            if not any(pts_valid):
                for ns in [f"human_arm_{side}", f"human_prediction_lines_{side}", f"human_prediction_joints_{side}", f"velocities_{side}"]:
                    del_marker = Marker()
                    del_marker.header.frame_id = base_frame
                    del_marker.header.stamp = timestamp
                    del_marker.ns = ns
                    del_marker.action = Marker.DELETEALL
                    marker_array.markers.append(del_marker)
                continue
            
            keypoints = [state.shoulder, state.elbow, state.wrist, state.hand]

            # 1. --- HUMAN ARM MARKER ---
            arm_marker = Marker()
            arm_marker.header.frame_id = base_frame
            arm_marker.header.stamp = timestamp
            arm_marker.ns = f"human_arm_{side}"
            arm_marker.id = 0
            arm_marker.type = Marker.LINE_STRIP
            arm_marker.action = Marker.ADD
            arm_marker.scale.x = 0.12 
            arm_marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.5)
            
            for i, pt in enumerate(keypoints):
                if pts_valid[i]:
                    arm_marker.points.append(pt)
                    
            marker_array.markers.append(arm_marker)

            # 2. --- HUMAN ARM PREDICTION (FADING GHOST ARMS) ---
            pred = self.latest_arm_predictions[side]
            if pred is not None:
                pred_pts_valid = pred.keypoint_valid
                display_steps = min(10, pred.num_steps)
                
                for step in range(display_steps):
                    alpha = max(0.05, 0.4 - (0.15 * step))
                    pred_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=alpha)
                    
                    keypoints_future = [
                        pred.shoulder[step], pred.elbow[step], 
                        pred.wrist[step], pred.hand[step]
                    ]
                    valid_points = [pt for i, pt in enumerate(keypoints_future) if pred_pts_valid[i]]
                    
                    if len(valid_points) > 0:
                        lines_marker = Marker()
                        lines_marker.header.frame_id = base_frame
                        lines_marker.header.stamp = timestamp
                        lines_marker.ns = f"human_prediction_lines_{side}"
                        lines_marker.id = step + 10
                        lines_marker.type = Marker.LINE_STRIP
                        lines_marker.action = Marker.ADD
                        lines_marker.scale.x = 0.04  
                        lines_marker.color = pred_color
                        lines_marker.points = valid_points
                        marker_array.markers.append(lines_marker)
                        
                        joints_marker = Marker()
                        joints_marker.header.frame_id = base_frame
                        joints_marker.header.stamp = timestamp
                        joints_marker.ns = f"human_prediction_joints_{side}"
                        joints_marker.id = step + 50
                        joints_marker.type = Marker.SPHERE_LIST
                        joints_marker.action = Marker.ADD
                        joints_marker.scale.x = 0.06
                        joints_marker.scale.y = 0.06
                        joints_marker.scale.z = 0.06
                        joints_marker.color = pred_color
                        joints_marker.points = valid_points
                        marker_array.markers.append(joints_marker)

            # 3. --- VELOCITY VECTORS (ARROWS AT KEYPOINTS) ---
            vels = []
            if hasattr(state, 'velocities') and len(state.velocities) >= 4:
                vels = state.velocities
            else:
                vels = [
                    getattr(state, 'shoulder_vel', getattr(state, 'shoulder_velocity', None)),
                    getattr(state, 'elbow_vel', getattr(state, 'elbow_velocity', None)),
                    getattr(state, 'wrist_vel', getattr(state, 'wrist_velocity', None)),
                    getattr(state, 'hand_vel', getattr(state, 'hand_velocity', None))
                ]
            
            for i, (pt, v) in enumerate(zip(keypoints, vels)):
                vel_marker = Marker()
                vel_marker.header.frame_id = base_frame
                vel_marker.header.stamp = timestamp
                vel_marker.ns = f"velocities_{side}"
                vel_marker.id = i + 200
                
                if pts_valid[i] and v is not None:
                    vel_marker.type = Marker.ARROW
                    vel_marker.action = Marker.ADD
                    
                    vel_marker.points.append(pt)
                    
                    end_pt = Point()
                    vel_scale = 0.3  
                    end_pt.x = pt.x + v.x * vel_scale
                    end_pt.y = pt.y + v.y * vel_scale
                    end_pt.z = pt.z + v.z * vel_scale
                    vel_marker.points.append(end_pt)
                    
                    vel_marker.scale.x = 0.015
                    vel_marker.scale.y = 0.030
                    vel_marker.scale.z = 0.030
                    vel_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
                else:
                    # Removes the velocity marker if the keypoint is lost or velocity is None
                    vel_marker.action = Marker.DELETE
                    
                marker_array.markers.append(vel_marker)

        # 4. --- ROBOT CPs & DISTANCE ARROWS ---
        if self.latest_distances is not None:
            # Distances stopped arriving (e.g. human disengaged): treat them as empty
            if self.image_buffer and not self.distances_are_recent(stamp_to_ns(self.image_buffer[-1])):
                self.latest_distances.links = []

            # If there are no valid links (empty message), clean the distance markers
            if not self.latest_distances.links:
                for ns in ["robot_points", "distances"]:
                    del_marker = Marker()
                    del_marker.header.frame_id = base_frame
                    del_marker.header.stamp = timestamp
                    del_marker.ns = ns
                    del_marker.action = Marker.DELETEALL
                    marker_array.markers.append(del_marker)
            else:
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
                    sphere.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.8)
                    marker_array.markers.append(sphere)

                min_link = min(self.latest_distances.links, key=lambda l: l.distance)
                
                for i, link in enumerate(self.latest_distances.links):
                    dist_marker = Marker()
                    dist_marker.header.frame_id = base_frame
                    dist_marker.header.stamp = timestamp
                    dist_marker.ns = "distances"
                    dist_marker.id = i
                    dist_marker.type = Marker.ARROW
                    dist_marker.action = Marker.ADD
                    
                    dist_marker.points.append(link.closest_point_human)
                    dist_marker.points.append(link.closest_point_robot)
                    
                    is_min = (link == min_link)
                    
                    if is_min:
                        dist_marker.scale.x = 0.02
                        dist_marker.scale.y = 0.04
                        dist_marker.scale.z = 0.04
                        if link.zone == 'critical':
                            dist_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
                        elif link.zone == 'danger':
                            dist_marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)
                        else:
                            dist_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
                    else:
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
        
        if p_cam[2] > 0.01:
            u = int((p_cam[0] / p_cam[2]) * self.fx + self.cx)
            v = int((p_cam[1] / p_cam[2]) * self.fy + self.cy)
            return (u, v)
        return None

    def _draw_distance_line(self, image: np.ndarray, image_stamp_ns: int) -> None:
        """Projects and draws the shortest geometric distance on the 2D overlay."""
        if self.fx is None or not self.camera_frame:
            return

        if not self.distances_are_recent(image_stamp_ns) or not self.latest_distances.links:
            return

        try:
            min_link = min(self.latest_distances.links, key=lambda l: l.distance)
            R, t = self.lookup_base_to_camera()

            uv_robot = self._project_point(min_link.closest_point_robot, R, t)
            uv_human = self._project_point(min_link.closest_point_human, R, t)
            
            if uv_robot and uv_human:
                if self.scale < 1.0:
                    uv_robot = (int(uv_robot[0] * self.scale), int(uv_robot[1] * self.scale))
                    uv_human = (int(uv_human[0] * self.scale), int(uv_human[1] * self.scale))
                
                cv2.line(image, uv_robot, uv_human, (255, 255, 255), 2)
                cv2.circle(image, uv_robot, 6, (0, 255, 255), -1)
                cv2.circle(image, uv_human, 6, (0, 0, 255), -1)

                cp_name = min_link.robot_link_name
                text_pos = (uv_robot[0] + 8, uv_robot[1] - 8)
                cv2.putText(
                    image, cp_name, text_pos, 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA
                )
        except Exception as exc:
            self.get_logger().warn(f'Distance overlay skipped: {exc}', throttle_duration_sec=2.0)

    # -------------------------------------------------------------------------
    # Main Render Loop
    # -------------------------------------------------------------------------
    def render_latest(self) -> None:
        """Render the selected frame once; rendered stamps only move forward."""
        if self.overlay_pub.get_subscription_count() == 0:
            return

        image_msg = self.select_image()
        if image_msg is None:
            return

        image_stamp_ns = stamp_to_ns(image_msg)
        if self.last_rendered_stamp_ns is not None and image_stamp_ns <= self.last_rendered_stamp_ns:
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

        # Draw Human Landmarks for each active arm
        for side in self.active_sides:
            # Check if the state is valid for this arm before rendering it
            arm_is_valid = False
            if self.latest_arm_states[side] is not None:
                arm_is_valid = any(self.latest_arm_states[side].keypoint_valid)

            if arm_is_valid and landmarks_are_recent(image_stamp_ns, self.last_valid_landmark_stamp_ns[side], self.landmark_hold_s):
                # No interpolation needed when the image matches the detection
                tau = 0.0 if self.sync_to_landmarks else self.smoothing_tau_s
                self.display_points[side], self.last_render_monotonic_ns[side] = update_display_points(
                    self.target_points[side], self.display_points[side], tau, self.max_hz, self.last_render_monotonic_ns[side]
                )
                if self.display_points[side] is not None:
                    draw_landmarks(
                        image, self.display_points[side], self.visibilities[side], self.LANDMARK_NAMES, 
                        self.visibility_threshold, self.scale, self.draw_labels
                    )
            else:
                # If the arm is lost, clean old 2D pixels to avoid freezing
                self.display_points[side] = None
                self.target_points[side] = None

        # If both arms are active, draw a line between the shoulders if they are visible
        if "left" in self.active_sides and "right" in self.active_sides:
            left_pts = self.display_points["left"]
            right_pts = self.display_points["right"]
            
            if left_pts is not None and right_pts is not None:
                vis_left_shoulder = self.visibilities["left"][0]
                vis_right_shoulder = self.visibilities["right"][0]
                
                # Check if both shoulders are visible enough to draw the line
                if (vis_left_shoulder >= self.visibility_threshold and 
                    vis_right_shoulder >= self.visibility_threshold):
                    
                    # Convert to integer pixel coordinates for drawing
                    pt1 = (int(left_pts[0][0]), int(left_pts[0][1]))
                    pt2 = (int(right_pts[0][0]), int(right_pts[0][1]))
                    cv2.line(image, pt1, pt2, (0, 255, 255), 2)

        self._draw_distance_line(image, image_stamp_ns)

        # HUD: INFO PANEL (Dynamic Height), darken only the top bar
        h_bar = 45 + 20 * (len(self.active_sides) - 1)
        bar = image[:h_bar]
        bar[:] = (bar * 0.4).astype(image.dtype)

        # Draw Time and Min Distance
        timestamp_sec = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
        time_str = f"Time: {timestamp_sec:.2f} s"
        if self.distances_are_recent(image_stamp_ns) and self.latest_distances.links:
            min_link = min(self.latest_distances.links, key=lambda l: l.distance)
            dist_str = f"Min Dist: {min_link.distance:.3f} m"
        else:
            dist_str = "Min Dist: --"
            
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(image, f"{time_str}   |   {dist_str}", (10, 18), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        
        # Draw Speeds for each tracked arm
        y_offset = 38
        for side in self.active_sides:
            speeds = [0.0, 0.0, 0.0, 0.0]
            if self.latest_arm_states[side]:
                state = self.latest_arm_states[side]
                if any(state.keypoint_valid):
                    if hasattr(state, 'velocities') and len(state.velocities) >= 4:
                        vels = state.velocities
                    else:
                        vels = [
                            getattr(state, 'shoulder_vel', getattr(state, 'shoulder_velocity', None)),
                            getattr(state, 'elbow_vel', getattr(state, 'elbow_velocity', None)),
                            getattr(state, 'wrist_vel', getattr(state, 'wrist_velocity', None)),
                            getattr(state, 'hand_vel', getattr(state, 'hand_velocity', None))
                        ]
                    # If the keypoint is lost, publish velocity at 0.0
                    speeds = [np.linalg.norm([v.x, v.y, v.z]) if (state.keypoint_valid[i] and v is not None) else 0.0 for i, v in enumerate(vels)]

            prefix_lbl = f"{side.upper()[:1]}: " if len(self.active_sides) > 1 else ""
            speeds_str = f"Speeds [{prefix_lbl}m/s]: sh {speeds[0]:.2f} | el {speeds[1]:.2f} | wr {speeds[2]:.2f} | ha {speeds[3]:.2f}"
            
            cv2.putText(image, speeds_str, (10, y_offset), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            y_offset += 20
        
        # Publish 2D Overlay
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