#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

class HumanPoseNode(Node):

    def __init__(self):
        super().__init__("human_pose_node")

        self.bridge = CvBridge()

        # Subscribe camera image
        self.sub = self.create_subscription(
            Image,
            "/my_camera/image",
            self.image_cb,
            10
        )

        # Publish annotated image
        self.pub = self.create_publisher(
            Image,
            "/human_pose/image",
            10
        )

        # MediaPipe
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            enable_segmentation=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.drawer = mp.solutions.drawing_utils
        self.style = mp.solutions.drawing_styles

        self.get_logger().info("🧍 Human Pose Node started (MediaPipe + RViz overlay)")

    def image_cb(self, msg: Image):
        # ROS → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Pose estimation
        result = self.pose.process(rgb)

        if result.pose_landmarks:
            # Disegna stickman sull'immagine
            self.drawer.draw_landmarks(
                frame,
                result.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=self.style.get_default_pose_landmarks_style()
            )

        # OpenCV → ROS
        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        out_msg.header = msg.header

        # Publish annotated image
        self.pub.publish(out_msg)


def main():
    rclpy.init()
    node = HumanPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
