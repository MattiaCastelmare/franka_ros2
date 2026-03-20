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

Performance notes
-----------------
* The image callback only stores the latest frame and returns immediately;
  MediaPipe inference is decoupled and runs in a separate ROS timer at a
  controllable rate (``pose_rate``).  This avoids lag accumulation when the
  camera publishes faster than MediaPipe can process.
* An optional ``inference_downscale`` factor resizes the frame before
  inference and maps landmarks back to the original resolution.  Because
  MediaPipe outputs normalised [0,1] coordinates, the rescaling is exact:
  ``u_orig = lm.x * w_orig``  regardless of inference resolution.
* The annotated image is published at a separate, lower rate
  (``annotated_image_rate``) to avoid wasting bandwidth on visualisation.
* Skeleton drawing is gated by ``draw_landmarks``; disabling it skips the
  cv2/MediaPipe draw call entirely.
"""

import time

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

        # ── Parameters ────────────────────────────────────────────────
        self.declare_parameter("pose_rate", 10.0)
        self.declare_parameter("inference_downscale", 1.0)
        self.declare_parameter("annotated_image_rate", 5.0)
        self.declare_parameter("draw_landmarks", True)

        self._pose_rate = float(self.get_parameter("pose_rate").value)
        self._downscale = float(
            self.get_parameter("inference_downscale").value)
        self._annotated_rate = float(
            self.get_parameter("annotated_image_rate").value)
        self._draw_landmarks = bool(
            self.get_parameter("draw_landmarks").value)

        # ── Latest-frame state (written by image callback, read by timer) ─
        self._latest_frame = None   # BGR numpy array, most recent decoded frame
        self._latest_header = None  # corresponding ROS header

        # Wall-clock timestamp of the last annotated image publish
        self._last_annotated_pub: float = 0.0

        # ── Subscriber ────────────────────────────────────────────────
        self.create_subscription(
            Image, "/my_camera/image", self._image_cb, 10
        )

        # ── Publishers ────────────────────────────────────────────────
        self.image_pub = self.create_publisher(
            Image, "/human_pose/image", 10
        )
        self.landmarks_pub = self.create_publisher(
            HumanPose2D, "/human_pose/landmarks", 10
        )

        # ── MediaPipe ─────────────────────────────────────────────────
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

        # ── Inference timer ───────────────────────────────────────────
        self.create_timer(1.0 / self._pose_rate, self._pose_timer_cb)

        self.get_logger().info(
            f"🧍 Human Pose Node started  "
            f"pose_rate={self._pose_rate} Hz  "
            f"downscale={self._downscale}  "
            f"annotated_rate={self._annotated_rate} Hz  "
            f"draw_landmarks={self._draw_landmarks}"
        )

    # ================================================================
    # Image callback — store only, no inference
    # ================================================================

    def _image_cb(self, msg: Image) -> None:
        """Store the latest frame; inference runs in the timer."""
        # Decode once; CvBridge returns a new array each call — safe to store.
        self._latest_frame = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="bgr8")
        self._latest_header = msg.header

    # ================================================================
    # Pose timer — decoupled inference pipeline
    # ================================================================

    def _pose_timer_cb(self) -> None:
        """Run MediaPipe on the latest stored frame.

        Pipeline:
        1. Skip if no frame has arrived yet.
        2. Optionally downscale for inference (saves ~(1-s²)×100 % compute).
        3. Convert BGR → RGB and run pose.process().
        4. Publish landmarks (always, at pose_rate).
        5. Publish annotated image (throttled to annotated_image_rate).
        """
        # ── 1. Guard: no frame yet ────────────────────────────────────
        if self._latest_frame is None:
            return

        frame = self._latest_frame       # original-resolution BGR
        header = self._latest_header
        h_orig, w_orig = frame.shape[:2]

        # ── 2. Optional downscale for inference ───────────────────────
        # Normalised MediaPipe coords [0,1] are resolution-independent:
        # u_orig = lm.x * w_orig  is exact regardless of inference size.
        scale = self._downscale
        if scale < 1.0 and scale > 0.0:
            w_inf = max(1, int(w_orig * scale))
            h_inf = max(1, int(h_orig * scale))
            frame_inf = cv2.resize(
                frame, (w_inf, h_inf), interpolation=cv2.INTER_LINEAR)
        else:
            frame_inf = frame   # no copy — read-only in pose.process()

        # ── 3. Colour conversion + MediaPipe inference ────────────────
        rgb = cv2.cvtColor(frame_inf, cv2.COLOR_BGR2RGB)
        result = self.pose.process(rgb)

        # ── 4. Publish landmarks (at full pose_rate) ──────────────────
        if result.pose_landmarks:
            # Always report in original resolution so downstream consumers
            # (e.g. human_distance_estimator) see consistent pixel coords.
            self._publish_landmarks(
                result.pose_landmarks, header, w_orig, h_orig)

        # ── 5. Publish annotated image (throttled) ────────────────────
        t_now = time.monotonic()
        annotated_interval = (
            1.0 / self._annotated_rate if self._annotated_rate > 0.0
            else float("inf")
        )
        should_pub_image = (
            (t_now - self._last_annotated_pub) >= annotated_interval
            and self.image_pub.get_subscription_count() > 0
        )

        if should_pub_image:
            self._publish_annotated_image(frame, result, header)
            self._last_annotated_pub = t_now

    # ================================================================
    # Annotated image publishing
    # ================================================================

    def _publish_annotated_image(self, frame, result, header) -> None:
        """Draw skeleton overlay (if enabled) and publish the image.

        A copy of *frame* is made only when drawing is active, to avoid
        mutating the stored latest frame.
        """
        if self._draw_landmarks and result.pose_landmarks:
            # Deferred copy: only when we will actually paint
            annotated = frame.copy()
            self.drawer.draw_landmarks(
                annotated,
                result.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=self.style.get_default_pose_landmarks_style(),
            )
        else:
            # No drawing → publish the raw frame without copying
            annotated = frame

        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        out_msg.header = header
        self.image_pub.publish(out_msg)

    # ================================================================
    # 2-D landmark publisher
    # ================================================================

    @staticmethod
    def _to_pixel(landmark, img_w: int, img_h: int) -> tuple:
        """Convert normalised MediaPipe landmark to clamped pixel coords."""
        u = min(max(landmark.x * img_w, 0.0), float(img_w - 1))
        v = min(max(landmark.y * img_h, 0.0), float(img_h - 1))
        return u, v

    def _publish_landmarks(
        self, pose_landmarks, header, img_w: int, img_h: int
    ) -> None:
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
