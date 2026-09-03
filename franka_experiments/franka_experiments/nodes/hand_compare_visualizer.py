#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
import yaml

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from franka_msgs.msg import HandState, HandTrackingFiltered
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
    """
    Visualize the hand tracking output and the hand state on top of the RGB image.
    """

    def __init__(self):
        super().__init__(
            'hand_compare_visualizer'
        )

        config_dir = (
            get_package_share_directory(
                'franka_experiments'
            )
            + '/config/'
        )

        intrinsics = load_camera_info_yaml(
            config_dir
            + 'camera_intrinsics.yaml'
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
            config_dir
            + 'camera_extrinsics.yaml',
            'r',
            encoding='utf-8',
        ) as file:
            extrinsics = yaml.safe_load(
                file
            )

        translation = (
            extrinsics['translation']
        )

        rotation = (
            extrinsics['rotation']
        )

        self.t_camera_base = np.array(
            [
                translation['x'],
                translation['y'],
                translation['z'],
            ],
            dtype=float,
        )

        self.r_camera_base = (
            Rotation.from_quat(
                [
                    rotation['x'],
                    rotation['y'],
                    rotation['z'],
                    rotation['w'],
                ]
            ).as_matrix()
        )
        self.r_base_camera = (
            self.r_camera_base.T
        )

        self.bridge = CvBridge()

        self.camera_info_subscription = (
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
        )

        # -------------------------------------------------
        # Output RViz
        # -------------------------------------------------

        self.publisher = (
            self.create_publisher(
                Image,
                (
                    '/handover/'
                    'hand_state_debug_image'
                ),
                2,
            )
        )

        qos = QoSProfile(
            depth=2,
            reliability=(
                ReliabilityPolicy.RELIABLE
            ),
        )

        # -------------------------------------------------
        # Synchronized input: RGB + filtered + hand state
        # -------------------------------------------------

        self.image_sub = Subscriber(
            self,
            Image,
            '/handover/hand_debug_image',
            qos_profile=qos,
        )

        self.filtered_sub = Subscriber(
            self,
            HandTrackingFiltered,
            (
                '/handover/'
                'hand_tracking_filtered'
            ),
            qos_profile=qos,
        )

        self.state_sub = Subscriber(
            self,
            HandState,
            '/handover/hand_state',
            qos_profile=qos,
        )

        self.sync = TimeSynchronizer(
            [
                self.image_sub,
                self.filtered_sub,
                self.state_sub,
            ],
            queue_size=2,
        )

        self.sync.registerCallback(
            self.callback
        )

        self.get_logger().info(
            'Hand compare visualizer node started'
        )

    # -----------------------------------------------------
    # Camera intrinsics
    # -----------------------------------------------------

    def camera_info_callback(
        self,
        msg,
    ):
        """
        Use the camera info message to update the intrinsics.
        """

        if (
            msg.k[0] <= 0.0
            or
            msg.k[4] <= 0.0
        ):
            return

        self.fx = float(
            msg.k[0]
        )

        self.fy = float(
            msg.k[4]
        )

        self.cx = float(
            msg.k[2]
        )

        self.cy = float(
            msg.k[5]
        )

    def project(
        self,
        point,
        width,
        height,
    ):
        p_base = np.array(
            [
                point.x,
                point.y,
                point.z,
            ],
            dtype=float,
        )

        if not np.all(
            np.isfinite(
                p_base
            )
        ):
            return None

        p_camera = (
            self.r_base_camera
            @ (
                p_base
                - self.t_camera_base
            )
        )

        x = float(
            p_camera[0]
        )

        y = float(
            p_camera[1]
        )

        z = float(
            p_camera[2]
        )

        if z <= 1e-6:
            return None

        u = int(
            round(
                self.fx
                * x
                / z
                + self.cx
            )
        )

        v = int(
            round(
                self.fy
                * y
                / z
                + self.cy
            )
        )

        if (
            u < 0
            or
            u >= width
            or
            v < 0
            or
            v >= height
        ):
            return None

        return (
            u,
            v,
        )

    # -----------------------------------------------------
    # Main synchronized callback
    # -----------------------------------------------------

    def callback(
        self,
        image_msg,
        filtered_msg,
        state_msg,
    ):
        image = (
            self.bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding='bgr8',
            )
        )

        height, width = (
            image.shape[:2]
        )

        usable = all(
            state in (
                HandTrackingFiltered.TRACKING,
                HandTrackingFiltered.PREDICT_ONLY,
            )
            for state
            in filtered_msg.landmark_state
        )

        if usable:
            pixels = []

            for point in (
                filtered_msg.positions
            ):
                pixel = self.project(
                    point,
                    width,
                    height,
                )

                pixels.append(
                    pixel
                )

            if all(
                pixel is not None
                for pixel in pixels
            ):

                polygon = np.array(
                    pixels,
                    dtype=np.int32,
                ).reshape(
                    (-1, 1, 2)
                )

                cv2.polylines(
                    image,
                    [polygon],
                    True,
                    (255, 0, 255),
                    2,
                    cv2.LINE_8,
                )

        if state_msg.valid:
            center = self.project(
                state_msg.palm_position,
                width,
                height,
            )

            if center is not None:
                cv2.drawMarker(
                    image,
                    center,
                    (0, 255, 255),
                    cv2.MARKER_CROSS,
                    12,
                    2,
                    cv2.LINE_8,
                )

        if state_msg.valid:
            text = (
                f'v = '
                f'{state_msg.palm_speed:.2f} '
                f'm/s'
            )

            text_color = (
                0,
                255,
                0,
            )

        else:
            text = 'v = --'

            text_color = (
                0,
                0,
                255,
            )

        cv2.rectangle(
            image,
            (8, 8),
            (185, 38),
            (0, 0, 0),
            -1,
        )

        cv2.putText(
            image,
            text,
            (14, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            text_color,
            1,
            cv2.LINE_8,
        )

        output = (
            self.bridge.cv2_to_imgmsg(
                image,
                encoding='bgr8',
            )
        )

        output.header = (
            image_msg.header
        )

        self.publisher.publish(
            output
        )


def main(args=None):
    rclpy.init(
        args=args
    )

    node = (
        HandCompareVisualizer()
    )

    try:
        rclpy.spin(
            node
        )

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()