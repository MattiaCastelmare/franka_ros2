#!/usr/bin/env python3

import os
import numpy as np
import rclpy

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from franka_msgs.msg import HandState, HandoverDistance

from franka_experiments.utils.distance_utils import (
    define_control_points,
    load_robot_config,
)
from franka_experiments.utils.tf_manager import TFManager


class HandoverDistanceEstimator(Node):

    def __init__(self):
        super().__init__('distance_handover_estimator')

        default_config = os.path.join(
            get_package_share_directory('franka_experiments'),
            'config',
            'fr3_complete.yaml',
        )

        self.declare_parameter('robot_config_path', default_config)

        config = load_robot_config(
            self.get_parameter('robot_config_path').value
        )

        self.robot_cfg = config['robot']
        self.distance_cfg = config['distance']

        self.base_frame = self.robot_cfg['base_frame']
        self.ee_link = self.robot_cfg.get('ee_link', 'fr3_link8')

        ee_segments = [
            seg
            for seg in self.robot_cfg['segments']
            if seg['end_link'] == self.ee_link
        ]

        if not ee_segments:
            raise RuntimeError(
                f'No robot segment ending at {self.ee_link}'
            )

        ee_segment = ee_segments[-1]

        self.ee_segment_links = [
            ee_segment['start_link'],
            ee_segment['end_link'],
        ]

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.tf_manager = TFManager(
            tf_buffer=self.tf_buffer,
            base_frame=self.base_frame,
            critical_links=[self.ee_link],
            cache_max_age_s=float(
                self.distance_cfg.get(
                    'tf_cache_max_age_s',
                    0.5,
                )
            ),
            logger=self.get_logger(),
        )

        self.publisher = self.create_publisher(
            HandoverDistance,
            '/handover/distance',
            10,
        )

        self.subscription = self.create_subscription(
            HandState,
            '/handover/hand_state',
            self.callback,
            10,
        )

        self.get_logger().info(
            f'Handover distance estimator started '
            f'({self.ee_link} -> palm, frame={self.base_frame})'
        )

    @staticmethod
    def point_array(p):
        return np.array(
            [p.x, p.y, p.z],
            dtype=np.float64,
        )

    @staticmethod
    def quaternion_matrix(q):
        x, y, z, w = q.x, q.y, q.z, q.w

        return np.array([
            [
                1 - 2 * (y*y + z*z),
                2 * (x*y - z*w),
                2 * (x*z + y*w),
            ],
            [
                2 * (x*y + z*w),
                1 - 2 * (x*x + z*z),
                2 * (y*z - x*w),
            ],
            [
                2 * (x*z - y*w),
                2 * (y*z + x*w),
                1 - 2 * (x*x + y*y),
            ],
        ])

    def transform_hand(self, msg):
        p = self.point_array(msg.palm_position)

        covariance = np.diag([
            max(0.0, msg.palm_position_variance.x),
            max(0.0, msg.palm_position_variance.y),
            max(0.0, msg.palm_position_variance.z),
        ])

        if msg.header.frame_id == self.base_frame:
            return p, covariance

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
            )
        except Exception as exc:
            self.get_logger().warn(
                f'Cannot transform hand '
                f'{msg.header.frame_id} -> {self.base_frame}: {exc}',
                throttle_duration_sec=2.0,
            )
            return None, None

        t = tf.transform.translation
        R = self.quaternion_matrix(
            tf.transform.rotation
        )

        translation = np.array(
            [t.x, t.y, t.z],
            dtype=np.float64,
        )

        return (
            R @ p + translation,
            R @ covariance @ R.T,
        )

    def callback(self, msg: HandState):
        out = HandoverDistance()

        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.base_frame

        if not msg.valid:
            self.publisher.publish(out)
            return

        # Hand position -> robot base frame.
        p_hand, covariance = self.transform_hand(msg)

        if p_hand is None:
            self.publisher.publish(out)
            return

        # Robot TF at the same timestamp as HandState.
        transforms = self.tf_manager.lookup_all(
            self.ee_segment_links,
            msg.header.stamp,
        )

        if not transforms:
            self.publisher.publish(out)
            return

        # Reuse existing robot control-point geometry.
        control_points = define_control_points(
            transforms,
            self.robot_cfg,
            self.distance_cfg,
        )

        ee_points = [
            cp
            for cp in control_points
            if cp['end_link'] == self.ee_link
        ]

        if not ee_points:
            self.publisher.publish(out)
            return

        # Final EE CP = physical fingertip point.
        ee_cp = max(
            ee_points,
            key=lambda cp: cp['cp_idx'],
        )

        p_ee = np.asarray(
            ee_cp['point'],
            dtype=np.float64,
        )

        delta = p_hand - p_ee
        distance = float(np.linalg.norm(delta))

        if not np.isfinite(distance):
            self.publisher.publish(out)
            return

        direction = (
            delta / distance
            if distance > 1e-9
            else np.zeros(3)
        )

        distance_variance = float(
            direction.T
            @ covariance
            @ direction
        )

        out.valid = True

        out.palm_position = Point(
            x=float(p_hand[0]),
            y=float(p_hand[1]),
            z=float(p_hand[2]),
        )

        out.ee_control_point = Point(
            x=float(p_ee[0]),
            y=float(p_ee[1]),
            z=float(p_ee[2]),
        )

        out.ee_to_palm = Vector3(
            x=float(delta[0]),
            y=float(delta[1]),
            z=float(delta[2]),
        )

        out.distance = distance

        out.distance_sigma = float(
            np.sqrt(
                max(distance_variance, 0.0)
            )
        )

        out.tracking_confidence = float(
            msg.tracking_confidence
        )

        out.motion_stability = float(
            msg.motion_stability
        )

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)

    node = HandoverDistanceEstimator()

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