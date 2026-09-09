#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
import yaml

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from franka_msgs.msg import (
    HandState,
    HandTrackingFiltered,
    HandoverDistance,
)
from message_filters import Subscriber, TimeSynchronizer
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

from franka_experiments.utils.camera_yaml import load_camera_info_yaml


class HandCompareVisualizer(Node):

    def __init__(self):
        super().__init__('hand_compare_visualizer')

        config_dir = (
            get_package_share_directory('franka_experiments')
            + '/config/'
        )

        intrinsics = load_camera_info_yaml(
            config_dir + 'camera_intrinsics.yaml'
        )

        if intrinsics is None:
            raise RuntimeError(
                'camera_intrinsics.yaml not valid'
            )

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

        t = extrinsics['translation']
        q = extrinsics['rotation']

        self.t_camera_base = np.array(
            [t['x'], t['y'], t['z']],
            dtype=float,
        )

        self.r_camera_base = Rotation.from_quat(
            [q['x'], q['y'], q['z'], q['w']]
        ).as_matrix()

        self.r_base_camera = self.r_camera_base.T

        self.bridge = CvBridge()

        self.create_subscription(
            CameraInfo,
            (
                '/camera/camera/'
                'aligned_depth_to_color/'
                'camera_info'
            ),
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.publisher = self.create_publisher(
            Image,
            '/handover/hand_state_debug_image',
            2,
        )

        qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.image_sub = Subscriber(
            self,
            Image,
            '/handover/hand_debug_image',
            qos_profile=qos,
        )

        self.filtered_sub = Subscriber(
            self,
            HandTrackingFiltered,
            '/handover/hand_tracking_filtered',
            qos_profile=qos,
        )

        self.state_sub = Subscriber(
            self,
            HandState,
            '/handover/hand_state',
            qos_profile=qos,
        )

        self.distance_sub = Subscriber(
            self,
            HandoverDistance,
            '/handover/distance',
            qos_profile=qos,
        )

        self.sync = TimeSynchronizer(
            [
                self.image_sub,
                self.filtered_sub,
                self.state_sub,
                self.distance_sub,
            ],
            queue_size=3,
        )

        self.sync.registerCallback(self.callback)

        self.get_logger().info(
            'Hand compare visualizer node started'
        )

    def camera_info_callback(self, msg):
        if msg.k[0] <= 0.0 or msg.k[4] <= 0.0:
            return

        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def project(self, point, width, height):
        p_base = np.array(
            [point.x, point.y, point.z],
            dtype=float,
        )

        if not np.all(np.isfinite(p_base)):
            return None

        p_camera = (
            self.r_base_camera
            @ (p_base - self.t_camera_base)
        )

        x, y, z = p_camera

        if z <= 1e-6:
            return None

        u = int(round(self.fx * x / z + self.cx))
        v = int(round(self.fy * y / z + self.cy))

        if (
            u < 0
            or u >= width
            or v < 0
            or v >= height
        ):
            return None

        return u, v

    def callback(
        self,
        image_msg,
        filtered_msg,
        state_msg,
        distance_msg,
    ):
        image = self.bridge.imgmsg_to_cv2(
            image_msg,
            desired_encoding='bgr8',
        )

        height, width = image.shape[:2]

        # -----------------------------------------
        # Filtered hand polygon
        # -----------------------------------------

        usable = all(
            state in (
                HandTrackingFiltered.TRACKING,
                HandTrackingFiltered.PREDICT_ONLY,
            )
            for state in filtered_msg.landmark_state
        )

        if usable:
            pixels = [
                self.project(point, width, height)
                for point in filtered_msg.positions
            ]

            if all(pixel is not None for pixel in pixels):
                polygon = np.array(
                    pixels,
                    dtype=np.int32,
                ).reshape((-1, 1, 2))

                cv2.polylines(
                    image,
                    [polygon],
                    True,
                    (255, 0, 255),
                    2,
                    cv2.LINE_8,
                )

        # -----------------------------------------
        # Palm center
        # -----------------------------------------

        palm_pixel = None

        if state_msg.valid:
            palm_pixel = self.project(
                state_msg.palm_position,
                width,
                height,
            )

            if palm_pixel is not None:
                cv2.drawMarker(
                    image,
                    palm_pixel,
                    (0, 255, 255),
                    cv2.MARKER_CROSS,
                    12,
                    2,
                    cv2.LINE_8,
                )

        if distance_msg.valid:
            ee_pixel = self.project(
                distance_msg.ee_control_point,
                width,
                height,
            )

            distance_palm_pixel = self.project(
                distance_msg.palm_position,
                width,
                height,
            )

            if (
                ee_pixel is not None
                and distance_palm_pixel is not None
            ):
                cv2.line(
                    image,
                    ee_pixel,
                    distance_palm_pixel,
                    (255, 255, 0),
                    2,
                    cv2.LINE_AA,
                )

                cv2.circle(
                    image,
                    ee_pixel,
                    5,
                    (255, 255, 0),
                    -1,
                )

        # -----------------------------------------
        # v + d overlay
        # -----------------------------------------

        timestamp_s = (
            float(image_msg.header.stamp.sec)
            + 1e-9
            * float(image_msg.header.stamp.nanosec)
        )

        velocity_text = (
            f'v = {state_msg.palm_speed:.2f} m/s'
            if state_msg.valid
            else 'v = --'
        )

        distance_text = (
            f'd = {distance_msg.distance:.2f} m'
            if distance_msg.valid
            else 'd = --'
        )

        timestamp_text = (
            f'timestamp = {timestamp_s:.3f}'
        )

        text = (
            f'{velocity_text}   '
            f'{distance_text}   '
            f'{timestamp_text}'
        )

        text_color = (
            (0, 255, 0)
            if (
                state_msg.valid
                and distance_msg.valid
            )
            else (0, 0, 255)
        )

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.40
        thickness = 1

        (text_width, text_height), baseline = (
            cv2.getTextSize(
                text,
                font,
                font_scale,
                thickness,
            )
        )

        box_right = min(
            width - 8,
            18 + text_width,
        )

        cv2.rectangle(
            image,
            (8, 8),
            (box_right, 38),
            (0, 0, 0),
            -1,
        )

        cv2.putText(
            image,
            text,
            (14, 29),
            font,
            font_scale,
            text_color,
            thickness,
            cv2.LINE_8,
        )

        output = self.bridge.cv2_to_imgmsg(
            image,
            encoding='bgr8',
        )

        output.header = image_msg.header

        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)

    node = HandCompareVisualizer()

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