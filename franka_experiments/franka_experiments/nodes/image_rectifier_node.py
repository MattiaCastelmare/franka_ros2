#!/usr/bin/env python3
"""Rectify a colour stream so apriltag_ros sees an undistorted image.

apriltag_node estimates the tag pose from ``image_rect`` with a pinhole model:
it ignores ``camera_info.d``. The D405 colour stream is NOT rectified — the
driver publishes ``color/image_raw`` with a non-zero plumb_bob distortion
(k1 ≈ −0.054 on serial 126122270738) — and image_proc is not installed in the
container, so this node does the remap with OpenCV.

Subscribes ``image_raw`` + ``camera_info``; publishes ``image_rect`` and a
matching ``camera_info`` (zero distortion, K = P) in the output namespace,
with the input header (stamp and frame) unchanged so TF lookups stay exact.
Remap the four topics from the launch file.
"""

from __future__ import annotations

import copy

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

# One thread: remapping 848x480 at 10 Hz needs no pool, and idle pool threads
# are just more contenders next to the ros2_control FIFO thread.
cv2.setNumThreads(1)


class ImageRectifier(Node):

    def __init__(self) -> None:
        super().__init__('image_rectifier')
        self._bridge = CvBridge()
        self._maps = None
        self._map_key = None
        self._info_rect: CameraInfo | None = None

        self.create_subscription(CameraInfo, 'camera_info', self._info_cb,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, 'image_raw', self._image_cb,
                                 qos_profile_sensor_data)
        self._pub_img = self.create_publisher(Image, 'image_rect', 5)
        self._pub_info = self.create_publisher(CameraInfo, 'camera_info_rect', 5)

    def _info_cb(self, msg: CameraInfo) -> None:
        key = (msg.width, msg.height, tuple(msg.k), tuple(msg.d), tuple(msg.p))
        if key == self._map_key:
            return
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64)
        R = np.array(msg.r, dtype=np.float64).reshape(3, 3)
        P = np.array(msg.p, dtype=np.float64).reshape(3, 4)[:, :3]
        if not np.any(P):          # some drivers leave P empty
            P = K
        self._maps = cv2.initUndistortRectifyMap(
            K, D, R, P, (msg.width, msg.height), cv2.CV_16SC2)
        self._map_key = key

        info = copy.deepcopy(msg)
        info.d = [0.0] * len(msg.d)
        info.k = P.ravel().tolist()
        info.r = np.eye(3).ravel().tolist()
        self._info_rect = info
        self.get_logger().info(
            f'Rectification maps built: {msg.width}x{msg.height} '
            f'model={msg.distortion_model} d={np.round(D, 4).tolist()} '
            f'frame={msg.header.frame_id}')

    def _image_cb(self, msg: Image) -> None:
        if self._maps is None:
            self.get_logger().warn('No camera_info yet — dropping image.',
                                   throttle_duration_sec=2.0)
            return
        img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        rect = cv2.remap(img, self._maps[0], self._maps[1], cv2.INTER_LINEAR)
        out = self._bridge.cv2_to_imgmsg(rect, encoding=msg.encoding)
        out.header = msg.header
        info = copy.copy(self._info_rect)
        info.header = msg.header
        self._pub_img.publish(out)
        self._pub_info.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = ImageRectifier()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C in a terminal reaches the node twice (the terminal's SIGINT and
        # ros2 launch forwarding it); the second must not abort the cleanup.
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
