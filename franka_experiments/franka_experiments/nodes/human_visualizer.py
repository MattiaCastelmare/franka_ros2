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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image, PointCloud

from franka_experiments.utils.distance_utils import load_robot_config
from franka_experiments.utils.human_utils import (
    draw_landmarks,
    landmarks_are_recent,
    stamp_to_ns,
    update_display_points,
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
        use_sim_time = bool(
            full_config.get('common', {}).get('use_sim_time', True)
        )

        super().__init__(
            'human_arm_visualizer',
            parameter_overrides=[
                Parameter(
                    'use_sim_time',
                    Parameter.Type.BOOL,
                    use_sim_time,
                )
            ],
            automatically_declare_parameters_from_overrides=True,
        )

        color_topic = str(config['color_topic'])
        landmarks_topic = str(config['landmarks_topic'])
        overlay_topic = str(config['overlay_topic'])

        self.visibility_threshold = float(
            config['visibility_threshold']
        )
        self.max_hz = max(1.0, float(config['max_hz']))
        self.scale = float(config['scale'])
        self.scale = min(max(self.scale, 0.1), 1.0)
        self.landmark_hold_s = max(
            0.0,
            float(config['landmark_hold_s']),
        )
        self.smoothing_tau_s = max(
            0.0,
            float(config['smoothing_tau_s']),
        )
        self.draw_labels = bool(config['draw_labels'])

        self.bridge = CvBridge()
        self.latest_image_msg: Image | None = None
        self.last_rendered_stamp_ns: int | None = None

        # Latest valid MediaPipe result. Empty landmark messages do not erase it;
        # it expires naturally after landmark_hold_s
        self.target_points: np.ndarray | None = None
        self.display_points: np.ndarray | None = None
        self.visibilities = np.zeros(len(self.LANDMARK_NAMES), dtype=np.float32)
        self.last_valid_landmark_stamp_ns: int | None = None
        self.last_render_monotonic_ns: int | None = None

        # Both subscriptions keep only one sample. A debug visualizer should
        # always prefer the newest data instead of processing an old queue
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.image_sub = self.create_subscription(
            Image,
            color_topic,
            self.image_cb,
            sensor_qos,
        )
        self.landmarks_sub = self.create_subscription(
            PointCloud,
            landmarks_topic,
            self.landmarks_cb,
            sensor_qos,
        )

        overlay_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.overlay_pub = self.create_publisher(
            Image,
            overlay_topic,
            overlay_qos,
        )

        self.render_timer = self.create_timer(
            1.0 / self.max_hz,
            self.render_latest,
        )

        self.get_logger().info(
            f'HumanArmVisualizer ready: image={color_topic}, '
            f'landmarks={landmarks_topic}, overlay={overlay_topic}, '
            f'max_hz={self.max_hz:.1f}, scale={self.scale:.2f}, '
            f'hold={self.landmark_hold_s:.2f}s'
        )

    def image_cb(self, msg: Image) -> None:
        """Store only the newest camera image."""
        self.latest_image_msg = msg

    def landmarks_cb(self, msg: PointCloud) -> None:
        """Accept complete detections. Per landmark, only move the target if
        MediaPipe currently trusts that point; a low-visibility landmark keeps
        its last confident position instead of jumping to a noisy/occluded
        guess."""
        if len(msg.points) < len(self.LANDMARK_NAMES):
            return

        points = np.asarray(
            [[point.x, point.y] for point in msg.points[:4]],
            dtype=np.float32,
        )
        if not np.all(np.isfinite(points)):
            return

        visibilities = np.zeros(len(self.LANDMARK_NAMES), dtype=np.float32)
        for channel in msg.channels:
            if channel.name == 'visibility':
                count = min(len(channel.values), len(self.LANDMARK_NAMES))
                if count > 0:
                    visibilities[:count] = np.asarray(
                        channel.values[:count],
                        dtype=np.float32,
                    )
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

    def render_latest(self) -> None:
        """Render the newest image and the most recent valid pose."""
        if self.overlay_pub.get_subscription_count() == 0:
            return

        image_msg = self.latest_image_msg
        if image_msg is None:
            return

        image_stamp_ns = stamp_to_ns(image_msg)
        if image_stamp_ns == self.last_rendered_stamp_ns:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(
                image_msg,
                desired_encoding='bgr8',
            ).copy()
        except Exception as exc:
            self.get_logger().warn(
                f'Image conversion failed: {exc}',
                throttle_duration_sec=2.0,
            )
            return

        if self.scale < 1.0:
            image = cv2.resize(
                image,
                dsize=None,
                fx=self.scale,
                fy=self.scale,
                interpolation=cv2.INTER_AREA,
            )

        if landmarks_are_recent(
            image_stamp_ns,
            self.last_valid_landmark_stamp_ns,
            self.landmark_hold_s,
        ):
            (
                self.display_points,
                self.last_render_monotonic_ns,
            ) = update_display_points(
                self.target_points,
                self.display_points,
                self.smoothing_tau_s,
                self.max_hz,
                self.last_render_monotonic_ns,
            )
            if self.display_points is not None:
                draw_landmarks(
                    image,
                    self.display_points,
                    self.visibilities,
                    self.LANDMARK_NAMES,
                    self.visibility_threshold,
                    self.scale,
                    self.draw_labels,
                )

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