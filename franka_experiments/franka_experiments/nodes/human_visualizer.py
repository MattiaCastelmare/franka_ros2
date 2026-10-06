#!/usr/bin/env python3
"""Lightweight ROS 2 visualizer for human-arm landmarks.

The 2D overlay shows the arm as the tracker uses it, like the 3D markers: only the
keypoints valid in the arm state, filled where measured in that frame (MediaPipe's
pixel), hollow where only predicted by the Kalman filter (its estimate projected),
under the fading red ghosts of the constant-velocity prediction (draw_prediction).
With sync_to_landmarks the overlay is drawn on the newest camera frame the tracker
has published a state for (constant delay, landmarks aligned with the image).
Otherwise it renders the newest frame, with landmarks held for short dropouts
and smoothly interpolated between updates.
Supports both single arm tracking and dual arm ('both') tracking dynamically.
"""

import os
import cv2
import rclpy
from functools import partial
from collections import deque
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
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
from visualization_msgs.msg import MarkerArray
from franka_msgs.msg import HumanArmState, MultiLinkDistance, HumanArmPrediction

from franka_experiments.utils.distance_utils import load_robot_config
from franka_experiments.utils.human_markers import arm_markers, distance_markers
from franka_experiments.utils.human_overlay import (
    CameraProjector, arm_speeds, draw_info_bar, draw_distance_line, draw_predicted_arm,
    draw_tracked_arm, landmarks_are_recent, parse_landmarks_msg, predicted_pixels,
    prediction_pixels, tracked_arm_pixels, update_display_points,
)
from franka_experiments.utils.human_utils import format_topic, stamp_to_ns


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
        # Red ghosts of the tracker's constant-velocity prediction, as the 3D markers
        self.draw_prediction = bool(config['draw_prediction'])
        self.prediction_dt = float(tracker_config['prediction_dt'])
        # Only the nearest ghosts on the 2D overlay (the tracker still predicts all its steps)
        self.prediction_steps = min(int(tracker_config['prediction_steps']),
                                    int(config['prediction_steps_2d']))
        # Draw on the frame the landmarks were detected on, not the newest one
        self.sync_to_landmarks = bool(config.get('sync_to_landmarks', True))

        # --- OpenCV Bridge ---
        self.bridge = CvBridge()
        self.image_buffer: deque[Image] = deque(maxlen=30)
        self.last_rendered_stamp_ns: int | None = None

        # --- Dictionaries for 2D states ---
        self.target_points = {side: None for side in self.active_sides}
        self.display_points = {side: None for side in self.active_sides}
        self.last_valid_landmark_stamp_ns = {side: None for side in self.active_sides}
        self.last_render_monotonic_ns = {side: None for side in self.active_sides}
        # Stamp of the latest frame the tracker published an arm state for, per side
        # (disengaged heartbeats included); its 2D landmarks are published just before
        self.processed_stamp_ns = {side: None for side in self.active_sides}

        # --- 3D Visualization State ---
        self.latest_arm_states = {side: None for side in self.active_sides}
        self.latest_arm_predictions = {side: None for side in self.active_sides}
        self.latest_distances = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Base-frame points to overlay pixels (intrinsics from camera_info_cb)
        self.projector = CameraProjector(self.tf_buffer)

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

        # Full-resolution images need RELIABLE
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.image_sub = self.create_subscription(
            Image, color_topic, self.image_cb, image_qos
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
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.overlay_pub = self.create_publisher(Image, overlay_topic, overlay_qos)
        self.marker_pub = self.create_publisher(MarkerArray, '/human_robot/markers', 10)
        # Overlay is event-driven (image / arm state callbacks), markers run on a timer
        self.marker_timer = self.create_timer(1.0 / self.max_hz, self.publish_markers_cb)

        self.get_logger().info(
            f'HumanArmVisualizer ready: image={color_topic}, max_hz={self.max_hz:.1f}, mode={self.pose_side}'
        )

    def camera_info_cb(self, msg: CameraInfo):
        self.projector.set_camera_info(msg)

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

    def arm_state_cb(self, msg: HumanArmState, side: str) -> None:
        """Mark the frame as processed: its landmarks and state are both in."""
        self.latest_arm_states[side] = msg
        self.processed_stamp_ns[side] = stamp_to_ns(msg)
        self.render_latest()

    def pred_cb(self, msg: HumanArmPrediction, side: str) -> None:
        self.latest_arm_predictions[side] = msg

    def landmarks_cb(self, msg: PointCloud, side: str) -> None:
        """Accept complete MediaPipe detections; the frame is drawn once its state arrives."""
        self.update_landmarks(msg, side)

    def update_landmarks(self, msg: PointCloud, side: str) -> None:
        parsed = parse_landmarks_msg(msg, len(self.LANDMARK_NAMES))
        if parsed is None:
            return
        points, visibilities = parsed

        if self.target_points[side] is None:
            self.target_points[side] = points.copy()
        else:
            trusted = visibilities >= self.visibility_threshold
            self.target_points[side][trusted] = points[trusted]

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
        """Arm, prediction and velocity markers of each arm, then the distance markers."""
        base_frame = next(
            (self.latest_arm_states[side].header.frame_id for side in self.active_sides
             if self.latest_arm_states[side] is not None), "fr3_link0")
        timestamp = self.get_clock().now().to_msg()

        marker_array = MarkerArray()
        for side in self.active_sides:
            state = self.latest_arm_states[side]
            if state is not None:
                marker_array.markers += arm_markers(
                    state, self.latest_arm_predictions[side], side, base_frame, timestamp)

        if self.latest_distances is not None:
            # Distances stopped arriving (e.g. human disengaged): treat them as empty
            if self.image_buffer and not self.distances_are_recent(stamp_to_ns(self.image_buffer[-1])):
                self.latest_distances.links = []
            marker_array.markers += distance_markers(
                self.latest_distances.links, base_frame, timestamp)

        self.marker_pub.publish(marker_array)

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

        # Draw each arm as the tracker uses it: only its valid keypoints (see tracked_arm_pixels)
        drawn = {}
        for side in self.active_sides:
            state = self.latest_arm_states[side]
            if state is None or not any(state.keypoint_valid):
                # Arm lost: clean old 2D pixels to avoid freezing
                self.display_points[side] = None
                self.target_points[side] = None
                drawn[side] = None
                continue

            landmark_px = None
            if landmarks_are_recent(image_stamp_ns, self.last_valid_landmark_stamp_ns[side], self.landmark_hold_s):
                # No interpolation needed when the image matches the detection
                tau = 0.0 if self.sync_to_landmarks else self.smoothing_tau_s
                self.display_points[side], self.last_render_monotonic_ns[side] = update_display_points(
                    self.target_points[side], self.display_points[side], tau, self.max_hz, self.last_render_monotonic_ns[side]
                )
                landmark_px = self.display_points[side]
            if self.draw_prediction:
                draw_predicted_arm(image, prediction_pixels(
                    state, self.projector.project, self.prediction_dt, self.prediction_steps),
                    self.scale)
            drawn[side] = tracked_arm_pixels(
                landmark_px, state.measured, state.keypoint_valid,
                predicted_pixels(state, self.projector.project))
            draw_tracked_arm(image, drawn[side], self.LANDMARK_NAMES, self.scale, self.draw_labels)

        # If both arms are active, join the shoulders when both are drawn
        if "left" in self.active_sides and "right" in self.active_sides:
            shoulders = [drawn[side][0] if drawn[side] else None for side in ("left", "right")]
            if all(shoulders):
                pt1, pt2 = [(int(u * self.scale), int(v * self.scale)) for (u, v), _ in shoulders]
                cv2.line(image, pt1, pt2, (0, 255, 255), 2)

        # Shortest robot-human distance of this frame, and the info bar
        min_link = None
        if self.distances_are_recent(image_stamp_ns) and self.latest_distances.links:
            min_link = min(self.latest_distances.links, key=lambda l: l.distance)
            draw_distance_line(image, min_link, self.projector.project, self.scale)
        draw_info_bar(
            image, image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9,
            None if min_link is None else min_link.distance,
            {side: arm_speeds(self.latest_arm_states[side]) for side in self.active_sides})

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