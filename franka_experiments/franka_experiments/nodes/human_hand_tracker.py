#!/usr/bin/env python3

import time

import cv2
import mediapipe as mp
import numpy as np
import rclpy
import yaml

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from franka_msgs.msg import HandTrackingRaw
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

from franka_experiments.utils.camera_yaml import load_camera_info_yaml
from franka_experiments.utils.hand_visualization import (
    draw_selected_landmarks,
    draw_status,
)


class HumanHandTracker(Node):
    """RGB-D tracker for hand landmarks 0, 5, 9 and 17."""

    LANDMARK_IDS = (0, 5, 9, 17)

    def __init__(self):
        super().__init__('human_hand_tracker')

        config_dir = (
            get_package_share_directory('franka_experiments') + '/config/'
        )

        intrinsics = load_camera_info_yaml(
            config_dir + 'camera_intrinsics.yaml'
        )
        if intrinsics is None:
            raise RuntimeError('camera_intrinsics.yaml not valid')

        k = intrinsics['k']
        self.fx = float(k[0])
        self.fy = float(k[4])
        self.cx = float(k[2])
        self.cy = float(k[5])

        with open(
            config_dir + 'camera_extrinsics.yaml',
            'r',
            encoding='utf-8',
        ) as file:
            extrinsics = yaml.safe_load(file)

        self.target_frame = extrinsics['parent_frame']
        self.camera_frame = extrinsics['child_frame']

        t = extrinsics['translation']
        q = extrinsics['rotation']

        self.camera_translation = np.array(
            [t['x'], t['y'], t['z']],
            dtype=float,
        )
        self.camera_rotation = Rotation.from_quat([
            q['x'],
            q['y'],
            q['z'],
            q['w'],
        ]).as_matrix()

        self.declare_parameter('show_selected_landmarks', False)
        self.show_selected_landmarks = bool(
            self.get_parameter('show_selected_landmarks').value
        )

        self.declare_parameter('publish_debug_image', True)
        self.publish_debug = bool(
            self.get_parameter('publish_debug_image').value
        )

        self.bridge = CvBridge()

        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        self.drawing_utils = mp.solutions.drawing_utils
        self.hand_connections = mp.solutions.hands.HAND_CONNECTIONS

        self.wrist_publisher = self.create_publisher(
            PoseStamped,
            '/handover/hand_state_raw',
            10,
        )
        self.landmarks_publisher = self.create_publisher(
            PoseArray,
            '/handover/hand_landmarks_raw',
            10,
        )
        self.tracking_publisher = self.create_publisher(
            HandTrackingRaw,
            '/handover/hand_tracking_raw',
            10,
        )
        self.debug_image_publisher = self.create_publisher(
            Image,
            '/handover/hand_debug_image',
            10,
        )

        image_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.rgb_sub = Subscriber(
            self,
            Image,
            '/camera/camera/color/image_raw',
            qos_profile=image_qos,
        )

        self.depth_sub = Subscriber(
            self,
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            qos_profile=image_qos,
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.color_info_callback,
            qos_profile_sensor_data,
        )

        self.synchronizer = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=4,
            slop=0.05,
        )
        self.synchronizer.registerCallback(self.image_callback)

        self.published_frames = 0

        self.get_logger().info(
            'Human hand tracker avviato: '
            f'{self.camera_frame} -> {self.target_frame}; '
            f'show_selected_landmarks={self.show_selected_landmarks}'
        )

    def color_info_callback(self, msg):
        if msg.k[0] <= 0.0 or msg.k[4] <= 0.0:
            return

        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def image_callback(self, rgb_msg, depth_msg):
        start_time = time.perf_counter()

        try:
            bgr_image = self.bridge.imgmsg_to_cv2(
                rgb_msg,
                desired_encoding='bgr8',
            )
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding='passthrough',
            )
        except Exception as error:
            self.get_logger().warn(
                f'Error in cv_bridge: {error}',
                throttle_duration_sec=2.0,
            )
            return

        depth_encoding = depth_msg.encoding

        debug_image = (
            bgr_image.copy()
            if self.show_selected_landmarks
            else None
        )

        rgb_image = cv2.cvtColor(
            bgr_image,
            cv2.COLOR_BGR2RGB,
        )
        result = self.hands.process(rgb_image)

        if not result.multi_hand_landmarks:
            self.publish_tracking(
                rgb_msg.header.stamp,
                HandTrackingRaw.NO_HAND,
                [None] * 4,
                [HandTrackingRaw.INVALID] * 4,
                start_time,
            )

            if debug_image is not None:
                draw_status(debug_image, 'NO HAND')

            self.publish_debug_image(debug_image, rgb_msg)

            self.get_logger().info(
                'No hand detected by MediaPipe',
                throttle_duration_sec=2.0,
            )
            return

        hand_landmarks = result.multi_hand_landmarks[0]
        hand = hand_landmarks.landmark

        if self.show_selected_landmarks:
            self.drawing_utils.draw_landmarks(
                debug_image,
                hand_landmarks,
                self.hand_connections,
            )
            draw_selected_landmarks(
                debug_image,
                hand,
                self.LANDMARK_IDS,
            )

        points_camera = {}

        for landmark_id in self.LANDMARK_IDS:
            points_camera[landmark_id] = self.landmark_to_3d(
                hand[landmark_id],
                bgr_image.shape,
                depth_image,
                depth_encoding,
            )

        direct_valid = {
            landmark_id: points_camera[landmark_id] is not None
            for landmark_id in self.LANDMARK_IDS
        }

        valid_depths = [
            point[2]
            for point in points_camera.values()
            if point is not None
        ]

        estimated = False

        if len(valid_depths) >= 2:
            reference_depth = float(np.median(valid_depths))

            consistent_depths = [
                depth
                for depth in valid_depths
                if abs(depth - reference_depth) <= 0.15
            ]

            if len(consistent_depths) >= 2:
                reference_depth = float(
                    np.median(consistent_depths)
                )

                for landmark_id in self.LANDMARK_IDS:
                    if points_camera[landmark_id] is None:
                        points_camera[landmark_id] = self.landmark_to_3d(
                            hand[landmark_id],
                            bgr_image.shape,
                            depth_image,
                            depth_encoding,
                            fallback_depth=reference_depth,
                        )
                        estimated = True

        points_base = [
            self.apply_transform(points_camera[landmark_id])
            if points_camera[landmark_id] is not None
            else None
            for landmark_id in self.LANDMARK_IDS
        ]

        measurement_types = [
            HandTrackingRaw.DIRECT
            if direct_valid[landmark_id]
            else (
                HandTrackingRaw.ESTIMATED
                if points_camera[landmark_id] is not None
                else HandTrackingRaw.INVALID
            )
            for landmark_id in self.LANDMARK_IDS
        ]

        valid_count = sum(
            point is not None
            for point in points_base
        )

        if valid_count == 0:
            tracking_state = HandTrackingRaw.INVALID_DEPTH
        elif valid_count < 4:
            tracking_state = HandTrackingRaw.TRACKING_PARTIAL
        elif HandTrackingRaw.ESTIMATED in measurement_types:
            tracking_state = HandTrackingRaw.TRACKING_ESTIMATED
        else:
            tracking_state = HandTrackingRaw.TRACKING_FULL

        self.publish_tracking(
            rgb_msg.header.stamp,
            tracking_state,
            points_base,
            measurement_types,
            start_time,
        )

        wrist = points_base[0]

        if wrist is None:
            if debug_image is not None:
                draw_status(
                    debug_image,
                    'INVALID WRIST DEPTH',
                )

            self.publish_debug_image(
                debug_image,
                rgb_msg,
            )

            self.get_logger().warn(
                'Hand detected, but wrist depth is invalid',
                throttle_duration_sec=2.0,
            )
            return

        self.publish_wrist(
            wrist,
            rgb_msg.header.stamp,
        )

        if all(point is not None for point in points_base):
            self.publish_landmarks(
                points_base,
                rgb_msg.header.stamp,
            )
            status = (
                'TRACKING ESTIMATED'
                if estimated
                else 'TRACKING FULL'
            )
        else:
            status = 'TRACKING WRIST'

        if debug_image is not None:
            draw_status(
                debug_image,
                status,
                wrist,
            )

        self.publish_debug_image(
            debug_image,
            rgb_msg,
        )

        self.published_frames += 1

        if self.published_frames % 30 == 0:
            self.get_logger().info(
                f'Wrist [cm] in {self.target_frame}: '
                f'x={100.0 * wrist[0]:.1f}, '
                f'y={100.0 * wrist[1]:.1f}, '
                f'z={100.0 * wrist[2]:.1f}'
            )

    def landmark_to_3d(
        self,
        landmark,
        rgb_shape,
        depth_image,
        depth_encoding,
        fallback_depth=None,
    ):
        rgb_height, rgb_width = rgb_shape[:2]

        u_rgb = int(np.clip(
            round(landmark.x * (rgb_width - 1)),
            0,
            rgb_width - 1,
        ))
        v_rgb = int(np.clip(
            round(landmark.y * (rgb_height - 1)),
            0,
            rgb_height - 1,
        ))

        depth_m = self.median_depth(
            depth_image,
            depth_encoding,
            u_rgb,
            v_rgb,
        )

        if depth_m is None:
            depth_m = fallback_depth

        if depth_m is None:
            return None

        x = (u_rgb - self.cx) * depth_m / self.fx
        y = (v_rgb - self.cy) * depth_m / self.fy

        return np.array(
            [x, y, depth_m],
            dtype=float,
        )

    @staticmethod
    def median_depth(depth_image, encoding, u, v):
        radius = 2
        height, width = depth_image.shape[:2]

        patch = depth_image[
            max(0, v - radius):min(height, v + radius + 1),
            max(0, u - radius):min(width, u + radius + 1),
        ]

        valid = patch[
            np.isfinite(patch) & (patch > 0)
        ]

        if valid.size == 0:
            return None

        depth = float(np.median(valid))

        if (
            encoding in ('16UC1', 'mono16')
            or depth_image.dtype == np.uint16
        ):
            depth *= 0.001

        if depth < 0.10 or depth > 3.00:
            return None

        return depth

    def apply_transform(self, point):
        return (
            self.camera_rotation @ point
            + self.camera_translation
        )

    def publish_tracking(
        self,
        stamp,
        tracking_state,
        points,
        measurement_types,
        start_time,
    ):
        msg = HandTrackingRaw()
        msg.header.stamp = stamp
        msg.header.frame_id = self.target_frame

        msg.tracking_state = int(tracking_state)
        msg.landmark_ids = list(self.LANDMARK_IDS)
        msg.valid = [
            point is not None
            for point in points
        ]
        msg.measurement_type = [
            int(value)
            for value in measurement_types
        ]
        msg.processing_latency_ms = float(
            1000.0 * (
                time.perf_counter() - start_time
            )
        )

        ros_points = []

        for point in points:
            ros_point = Point()

            if point is not None:
                ros_point.x = float(point[0])
                ros_point.y = float(point[1])
                ros_point.z = float(point[2])

            ros_points.append(ros_point)

        msg.positions = ros_points
        self.tracking_publisher.publish(msg)

    def publish_wrist(self, wrist, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.target_frame

        msg.pose.position.x = float(wrist[0])
        msg.pose.position.y = float(wrist[1])
        msg.pose.position.z = float(wrist[2])
        msg.pose.orientation.w = 1.0

        self.wrist_publisher.publish(msg)

    def publish_landmarks(self, points, stamp):
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self.target_frame

        for point in points:
            pose = Pose()
            pose.position.x = float(point[0])
            pose.position.y = float(point[1])
            pose.position.z = float(point[2])
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.landmarks_publisher.publish(msg)

    def publish_debug_image(self, image, original_msg):
        if not self.publish_debug:
            return

        if image is None:
            self.debug_image_publisher.publish(original_msg)
            return

        msg = self.bridge.cv2_to_imgmsg(
            image,
            encoding='bgr8',
        )
        msg.header = original_msg.header
        self.debug_image_publisher.publish(msg)

    def destroy_node(self):
        self.hands.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HumanHandTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()