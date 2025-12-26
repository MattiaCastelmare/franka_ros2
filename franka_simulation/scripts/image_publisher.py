#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class ImageRepublisher(Node):

    def __init__(self):
        super().__init__("image_republisher")

        self.sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.callback,
            qos_profile_sensor_data
        )

        self.pub = self.create_publisher(
            Image,
            "/my_camera/image",
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
        )

        self.counter = 0
        self.get_logger().info("📸 Image republisher started")

    def callback(self, msg):
        self.pub.publish(msg)
        self.counter += 1
        if self.counter % 30 == 0:
            self.get_logger().info(f"Republished {self.counter} frames")


def main():
    rclpy.init()
    node = ImageRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
