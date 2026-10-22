#!/usr/bin/env python3
"""depth_sim_adapter: Gazebo's depth camera, re-encoded as the D455 stream (plan H2, H11).

SIM only. Subscribes to the bridged Gazebo depth image (32FC1 metres) and its
camera_info, and publishes what ``real_time_distance`` reads from the
RealSense driver: ``/camera/camera/depth/image_rect_raw`` (16UC1 millimetres,
0 = no return) and ``/camera/camera/depth/camera_info``, both with the
capture stamp, ``frame_id`` set to the D455's depth optical frame.

The encoding is always applied. Delay, dropout and range noise are off by
default, each with its own parameter (``config/sim/depth_adapter.yaml``); the
logic and its rationale live in ``utils/depth_conversion.py``.
"""
import array

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from utils.depth_conversion import FidelityStages


class DepthSimAdapter(Node):
    def __init__(self):
        super().__init__('depth_sim_adapter')
        dp = self.declare_parameter
        self.in_image = dp('in_image', '/sim/d455/depth/image').value
        self.in_info = dp('in_info', '/sim/d455/depth/camera_info').value
        self.out_image = dp('out_image', '/camera/camera/depth/image_rect_raw').value
        self.out_info = dp('out_info', '/camera/camera/depth/camera_info').value
        self.frame_id = dp('frame_id', 'camera_depth_optical_frame').value
        self.delay_s = float(dp('delay_s', 0.0).value)
        self.dropout_p = float(dp('dropout_p', 0.0).value)
        self.range_noise = bool(dp('range_noise', False).value)
        self.baseline_m = float(dp('baseline_m', 0.095).value)
        self.sigma_d_px = float(dp('sigma_d_px', 0.20).value)
        self.seed = int(dp('seed', 0).value)
        # Hold frames until the arm's TF resolves (base -> last link). On the
        # rig the robot state exists before the camera; in Gazebo the camera
        # renders ~1 s before the robot is spawned and its TF published, and
        # real_time_distance then masks a robot it cannot place: measured, a
        # phantom 0.146 m from fr3_link5 that the filter reacted to before the
        # episode started. Empty = publish at once.
        self.wait_tf = [f for f in dp('wait_for_tf', ['base', 'fr3_link8']).value if f]
        self.tf_ready = not self.wait_tf
        self.n_held = 0
        self.stages = None
        self.info = None
        qos_out = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_img = self.create_publisher(Image, self.out_image, qos_out)
        self.pub_info = self.create_publisher(CameraInfo, self.out_info, qos_out)
        self.create_subscription(CameraInfo, self.in_info, self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.in_image, self._on_image, qos_profile_sensor_data)
        if self.delay_s > 0.0:
            self.create_timer(0.001, self._flush)
        if not self.tf_ready:
            import tf2_ros
            self._tf_buf = tf2_ros.Buffer()
            self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)
            self._tf_timer = self.create_timer(0.1, self._check_tf)
        self.get_logger().info(
            f'SIM depth adapter {self.in_image} (32FC1 m) -> {self.out_image} (16UC1 mm); '
            f'delay_s={self.delay_s} dropout_p={self.dropout_p} range_noise={self.range_noise} '
            f'(baseline {self.baseline_m} m, sigma_d {self.sigma_d_px} px) seed={self.seed}')

    def _check_tf(self):
        from rclpy.time import Time
        try:
            self._tf_buf.lookup_transform(self.wait_tf[0], self.wait_tf[1], Time())
        except Exception:                                   # noqa: BLE001
            return
        self.tf_ready = True
        self._tf_timer.cancel()
        self.get_logger().info(f'TF {self.wait_tf[0]} -> {self.wait_tf[1]} available: '
                               f'publishing depth ({self.n_held} frames held before)')

    def _on_info(self, msg: CameraInfo):
        if self.info is None:
            self.get_logger().info(f'camera_info: {msg.width}x{msg.height} fx={msg.k[0]:.3f} '
                                   f'fy={msg.k[4]:.3f} cx={msg.k[2]:.3f} cy={msg.k[5]:.3f}')
        self.info = msg
        if self.stages is None:
            self.stages = FidelityStages(delay_s=self.delay_s, dropout_p=self.dropout_p,
                                         range_noise=self.range_noise, f_px=float(msg.k[0]),
                                         baseline_m=self.baseline_m, sigma_d_px=self.sigma_d_px,
                                         seed=self.seed)

    def _on_image(self, msg: Image):
        if self.stages is None:
            return
        if not self.tf_ready:
            self.n_held += 1
            return
        if msg.encoding != '32FC1':
            self.get_logger().error(f'expected 32FC1 from Gazebo, got {msg.encoding}',
                                    throttle_duration_sec=5.0)
            return
        row = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)
        depth = row[:, :msg.width]
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.stages.push(stamp, depth, msg.header.stamp)
        if self.delay_s <= 0.0:
            self._flush(stamp)

    def _flush(self, now_s=None):
        if self.stages is None:
            return
        if now_s is None:
            now_s = self.get_clock().now().nanoseconds * 1e-9
        for _, stamp_msg, img in self.stages.pop_due(now_s):
            self._publish(stamp_msg, img)

    def _publish(self, stamp_msg, img: np.ndarray):
        out = Image()
        out.header.stamp = stamp_msg
        out.header.frame_id = self.frame_id
        out.height, out.width = img.shape
        out.encoding = '16UC1'
        out.is_bigendian = 0
        out.step = img.shape[1] * 2
        # array('B', ...) is a memcpy; assigning bytes would make rosidl's
        # setter type-check all 814 080 elements in Python (measured: the
        # adapter then managed 31 of 86 frames/s).
        out.data = array.array('B', img.tobytes())
        info = self.info
        info.header.stamp = out.header.stamp
        info.header.frame_id = self.frame_id
        self.pub_info.publish(info)
        self.pub_img.publish(out)


def main():
    rclpy.init()
    node = DepthSimAdapter()
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
