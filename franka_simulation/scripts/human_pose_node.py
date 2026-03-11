#!/usr/bin/env python3
"""Human Pose Node – raw 2-D MediaPipe landmark publisher.

Subscribes:
    /my_camera/image – RGB image (sensor_msgs/Image)

Publishes:
    /human_pose/image     – annotated RGB image            (sensor_msgs/Image)
    /human_pose/landmarks – raw 2-D pixel-coordinate poses (franka_simulation/HumanPose2D)

All landmarks are in *pixel* coordinates of the source RGB image.
No depth or camera-intrinsic processing is performed – students are
expected to do geometric back-projection offline from recorded rosbags.
"""

import cv2
import mediapipe as mp

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from franka_simulation.msg import HumanPose2D


class HumanPoseNode(Node):
    """Detect 2-D pose with MediaPipe and publish raw pixel landmarks."""

    def __init__(self):
        super().__init__("human_pose_node")

        self.bridge = CvBridge()

        # ----- Subscriber -----
        self.create_subscription(
            Image, "/my_camera/image", self._image_cb, 10
        )

        # ----- Publishers -----
        self.image_pub = self.create_publisher(
            Image, "/human_pose/image", 10
        )
        self.landmarks_pub = self.create_publisher(
            HumanPose2D, "/human_pose/landmarks", 10
        )

        # ----- MediaPipe -----
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.drawer = mp.solutions.drawing_utils
        self.style = mp.solutions.drawing_styles

        self.get_logger().info(
            "🧍 Human Pose Node started "
            "(MediaPipe + annotated image + 2-D landmarks)"
        )

    # ==================================================================
    # Callback
    # ==================================================================
    def _image_cb(self, msg: Image):
        """Run MediaPipe pose, publish annotated image and raw 2-D landmarks."""
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        result = self.pose.process(rgb)

        if result.pose_landmarks:
            # Draw annotated stick-figure on the image
            self.drawer.draw_landmarks(
                frame,
                result.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=self.style.get_default_pose_landmarks_style(),
            )

            # Publish raw 2-D landmarks
            h, w = frame.shape[:2]
            self._publish_landmarks(result.pose_landmarks, msg.header, w, h)

        # Publish annotated image
        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out_msg.header = msg.header
        self.image_pub.publish(out_msg)

    # ==================================================================
    # 2-D landmark publisher
    # ==================================================================
    @staticmethod
    def _to_pixel(landmark, img_w: int, img_h: int) -> tuple:
        """Convert normalised MediaPipe landmark to clamped pixel coords."""
        u = min(max(landmark.x * img_w, 0.0), float(img_w - 1))
        v = min(max(landmark.y * img_h, 0.0), float(img_h - 1))
        return u, v

    def _publish_landmarks(self, pose_landmarks, header, img_w: int, img_h: int):
        """Build and publish a HumanPose2D message with all 33 landmarks."""
        pose_msg = HumanPose2D()
        pose_msg.header = header
        pose_msg.image_width = img_w
        pose_msg.image_height = img_h

        ids = []
        us = []
        vs = []
        visibilities = []

        for idx, lm in enumerate(pose_landmarks.landmark):
            u, v = self._to_pixel(lm, img_w, img_h)
            ids.append(idx)
            us.append(float(u))
            vs.append(float(v))
            visibilities.append(float(lm.visibility))

        pose_msg.ids = ids
        pose_msg.u = us
        pose_msg.v = vs
        pose_msg.visibility = visibilities

        self.landmarks_pub.publish(pose_msg)


def main():
    rclpy.init()
    node = HumanPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
