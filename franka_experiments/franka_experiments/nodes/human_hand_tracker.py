#!/usr/bin/env python3

import cv2
import subprocess
import mediapipe as mp
import numpy as np
import rclpy
import yaml

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from franka_experiments.utils.camera_yaml import load_camera_info_yaml
from franka_experiments.utils.hand_visualization import (
    draw_selected_landmarks,
    draw_status,
)

from scipy.spatial.transform import Rotation


class HumanHandTracker(Node):
    """
    Simple Tracker RGB-D for human hand.

    Publish:
      /handover/hand_state_raw -> wrist 3D
      /handover/hand_landmarks_raw -> landmark 0, 5, 9, 17 in 3D
      /handover/hand_debug_image -> image RGB for RViz
    """

    LANDMARK_IDS = (0, 5, 9, 17)

    def __init__(self):
        super().__init__('human_hand_tracker')

        # -------------------------------------------------------------
        # Camera Calibration
        # -------------------------------------------------------------
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

        # Transformation camera -> base, directly applied.
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

        # Parameters.
        self.declare_parameter('play_bag', True)
        self.declare_parameter(
            'bag_path',
            '/ros2_ws/rosbags/handratacker_objecy',
        )
        self.declare_parameter('show_selected_landmarks', False)

        self.bag_process = None
        self.show_selected_landmarks = bool(
            self.get_parameter('show_selected_landmarks').value
        )

        # -------------------------------------------------------------
        # OpenCV and MediaPipe
        # -------------------------------------------------------------
        self.bridge = CvBridge()

        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        self.drawing_utils = mp.solutions.drawing_utils
        self.hand_connections = mp.solutions.hands.HAND_CONNECTIONS

        # -------------------------------------------------------------
        # Publisher
        # -------------------------------------------------------------
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

        self.debug_image_publisher = self.create_publisher(
            Image,
            '/handover/hand_debug_image',
            10,
        )

        # -------------------------------------------------------------
        # RGB and depth synchronized
        # -------------------------------------------------------------
        self.rgb_subscriber = Subscriber(
            self,
            Image,
            '/camera/camera/color/image_raw',
            qos_profile=qos_profile_sensor_data,
        )

        self.depth_subscriber = Subscriber(
            self,
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            qos_profile=qos_profile_sensor_data,
        )

        self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.color_info_callback,
            10,
        )

        self.synchronizer = ApproximateTimeSynchronizer(
            [self.rgb_subscriber, self.depth_subscriber],
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

        if self.get_parameter('play_bag').value:
            bag_path = self.get_parameter('bag_path').value
            self.bag_process = subprocess.Popen(
                ['ros2', 'bag', 'play', bag_path, '--clock']
            )

    def color_info_callback(self, msg):
        """Update camera intrinsics with CameraInfo of the depth image."""
        if msg.k[0] <= 0.0 or msg.k[4] <= 0.0:
            return

        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def image_callback(self, rgb_msg, depth_msg):
        """Elaborate a synchronized RGB-depth pair."""

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
            self.get_logger().warn(f'Error in cv_bridge: {error}')
            return

        # The depth image is already registered by the camera driver, so we can directly use the pixel coordinates of the RGB image to access the depth values.
        depth_encoding = depth_msg.encoding
        debug_image = bgr_image.copy()

        # MediaPipe works in RGB.
        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb_image)

        if not result.multi_hand_landmarks:
            draw_status(debug_image, 'NO HAND')
            self.publish_debug_image(debug_image, rgb_msg)
            self.get_logger().info(
                'Nessuna mano rilevata da MediaPipe',
                throttle_duration_sec=2.0,
            )
            return

        hand_landmarks = min(
            result.multi_hand_landmarks,
            key=lambda detected_hand: detected_hand.landmark[0].y,
        )
        hand = hand_landmarks.landmark

        # Visualization
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

        # -------------------------------------------------------------
        # 3D Reconstruction of landmarks 0, 5, 9 and 17
        # -------------------------------------------------------------
        points_camera = {}

        for landmark_id in self.LANDMARK_IDS:
            points_camera[landmark_id] = self.landmark_to_3d(
                hand[landmark_id],
                bgr_image.shape,
                depth_image,
                depth_encoding,
            )

        # Median depth of valid landmarks, used as fallback for missing points.
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
                reference_depth = float(np.median(consistent_depths))

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

        wrist_camera = points_camera[0]

        if wrist_camera is None:
            draw_status(debug_image, 'INVALID WRIST DEPTH')
            self.publish_debug_image(debug_image, rgb_msg)
            self.get_logger().warn(
                'Hand detected, but wrist depth is invalid',
                throttle_duration_sec=2.0,
            )
            return

        wrist = self.apply_transform(wrist_camera)
        self.publish_wrist(wrist, rgb_msg.header.stamp)

        all_landmarks_valid = all(
            points_camera[landmark_id] is not None
            for landmark_id in self.LANDMARK_IDS
        )

        if all_landmarks_valid:
            points_3d = [
                self.apply_transform(points_camera[landmark_id])
                for landmark_id in self.LANDMARK_IDS
            ]

            self.publish_landmarks(
                points_3d,
                rgb_msg.header.stamp,
            )

            status = (
                'TRACKING ESTIMATED'
                if estimated
                else 'TRACKING FULL'
            )
        else:
            status = 'TRACKING WRIST'

        draw_status(debug_image, status, wrist)
        self.publish_debug_image(debug_image, rgb_msg)

        self.published_frames += 1
        if self.published_frames % 30 == 0:
            self.get_logger().info(
                f'Polso [cm] in {self.target_frame}: '
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
        """Landmark MediaPipe -> pixel -> depth -> 3D point in camera frame."""

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
        z = depth_m

        return np.array([x, y, z], dtype=float)

    @staticmethod
    def median_depth(depth_image, encoding, u, v):
        """Median of valid values in a 5x5 depth patch."""

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

        if encoding in ('16UC1', 'mono16') or depth_image.dtype == np.uint16:
            depth *= 0.001

        if depth < 0.10 or depth > 3.00:
            return None

        return depth

    def apply_transform(self, point):
        """Apply the fixed camera calibration -> base transform."""
        return self.camera_rotation @ point + self.camera_translation

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

        # Fixed order: 0, 5, 9, 17.
        for point in points:
            pose = Pose()
            pose.position.x = float(point[0])
            pose.position.y = float(point[1])
            pose.position.z = float(point[2])
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.landmarks_publisher.publish(msg)

    def publish_debug_image(self, image, original_msg):
        msg = self.bridge.cv2_to_imgmsg(
            image,
            encoding='bgr8',
        )
        msg.header = original_msg.header
        self.debug_image_publisher.publish(msg)

    def destroy_node(self):
        if self.bag_process is not None:
            self.bag_process.terminate()
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