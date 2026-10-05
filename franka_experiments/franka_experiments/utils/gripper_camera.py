#!/usr/bin/env python3
"""Gripper camera (D405, eye-in-hand) as a second view of the hand.

Passive unless its images and the robot TF arrive: MediaPipe Hands runs in a
worker thread and the hands are mapped to the base frame with
TF(base -> fr3_link8) and the fixed transform of config/d405_extrinsics.yaml.

Offline (handover_20260929_161100, hand near the gripper): it re-finds 84% of
the hands the main camera loses inside its view, and its palm normal is right
on 20/22 labelled hands (main camera 16/22).
"""

import threading

import cv2
import mediapipe as mp
import numpy as np
import yaml
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

from franka_experiments.utils.palm import signed_palm_normal


class GripperCamera:

    PALM_IDS = (0, 5, 9, 17)  # same landmarks as HandTrackingRaw

    def __init__(self, node, extrinsics_path, tf_lookup, namespace='/d405/d405', debug=False):
        with open(extrinsics_path, 'r', encoding='utf-8') as file:
            e = yaml.safe_load(file)
        q, t = e['rotation'], e['translation']
        self.link = e['parent_frame']
        self.R_link_cam = Rotation.from_quat([q['x'], q['y'], q['z'], q['w']]).as_matrix()
        self.t_link_cam = np.array([t['x'], t['y'], t['z']], dtype=float)
        self.tf_lookup = tf_lookup
        self.logger, self.state = node.get_logger(), None
        self.bridge = CvBridge()
        self.K, self.frame, self.result = None, None, None
        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False, max_num_hands=2, model_complexity=1,
            min_detection_confidence=0.5, min_tracking_confidence=0.5)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.sync = ApproximateTimeSynchronizer(
            [Subscriber(node, Image, f'{namespace}/color/image_raw', qos_profile=qos),
             Subscriber(node, Image, f'{namespace}/aligned_depth_to_color/image_raw', qos_profile=qos)],
            queue_size=2, slop=0.05)
        self.sync.registerCallback(self._images)
        node.create_subscription(CameraInfo, f'{namespace}/aligned_depth_to_color/camera_info',
                                 self._info, qos_profile_sensor_data)
        # landmarks drawn on the D405 image; hand_compare_visualizer adds the HandState overlay
        self.debug = node.create_publisher(Image, '/handover/gripper_debug_image_raw', 2) if debug else None
        self.stopping, self.wake = threading.Event(), threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _info(self, msg):
        if msg.k[0] > 0.0:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _images(self, rgb_msg, depth_msg):
        self.frame = (rgb_msg, depth_msg)  # newest wins
        self.wake.set()

    def stop(self):
        self.stopping.set()
        self.wake.set()
        self.thread.join(timeout=2.0)
        self.hands.close()

    def _run(self):
        while not self.stopping.is_set():
            self.wake.wait(0.5)
            self.wake.clear()
            frame, self.frame = self.frame, None
            if frame is None or self.K is None:
                continue
            rgb_msg, depth_msg = frame
            pose = self.tf_lookup(self.link, rgb_msg.header.stamp)
            self._log_state(pose is not None)
            if pose is None:            # no robot TF: the camera cannot be placed
                self.result = None
                continue
            R_base_link, t_base_link = pose
            R = R_base_link @ self.R_link_cam
            t = R_base_link @ self.t_link_cam + t_base_link
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='rgb8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            h, w = rgb.shape[:2]
            hands = []
            found = self.hands.process(rgb).multi_hand_landmarks or []
            for lms in found:
                V = np.array([[p.x * w, p.y * h, p.z * w] for p in lms.landmark])
                pts = [self._point(depth, *V[i, :2]) for i in self.PALM_IDS]
                hands.append({'pts': [None if p is None else R @ p + t for p in pts], 'V': V, 'R': R})
            self.result = (rgb_msg.header.stamp.sec + 1e-9 * rgb_msg.header.stamp.nanosec, hands)
            if self.debug is not None:
                image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                for lms in found:
                    mp.solutions.drawing_utils.draw_landmarks(image, lms, mp.solutions.hands.HAND_CONNECTIONS)
                msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
                msg.header = rgb_msg.header
                self.debug.publish(msg)

    def _log_state(self, placed):
        # no TF for ~1 s of images (not just the start-up race): say it once
        self.missing = 0 if placed else getattr(self, 'missing', 0) + 1
        state = True if placed else (False if self.missing >= 30 else self.state)
        if state != self.state:
            self.state = state
            if state:
                self.logger.info(f'Gripper camera active (TF base -> {self.link})')
            else:
                self.logger.warn(f'Gripper camera images but no TF base -> {self.link}: not used')

    def _point(self, depth, u, v):
        h, w = depth.shape[:2]
        u, v = int(np.clip(round(u), 0, w - 1)), int(np.clip(round(v), 0, h - 1))
        patch = depth[max(0, v - 2):v + 3, max(0, u - 2):u + 3]
        patch = patch[patch > 0]
        if patch.size == 0:
            return None
        z = float(np.median(patch)) * (1e-3 if depth.dtype == np.uint16 else 1.0)
        if not 0.07 <= z <= 1.5:
            return None
        fx, fy, cx, cy = self.K
        return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])

    def hand_near(self, point, t, radius, max_age=0.1):
        """Hand whose palm (base frame) is within radius of point, from a frame <= max_age old."""
        result = self.result
        if result is None or abs(t - result[0]) > max_age:
            return None
        best = None
        for hand in result[1]:
            valid = [p for p in hand['pts'] if p is not None]
            if len(valid) < 3:
                continue
            d = float(np.linalg.norm(np.mean(valid, 0) - point))
            if d < radius and (best is None or d < best[0]):
                best = (d, hand)
        return None if best is None else best[1]

    @staticmethod
    def palm_normal(hand, right):
        """Out-of-palm normal in the base frame (MediaPipe 3D landmarks, side from the main camera)."""
        n = hand['R'] @ signed_palm_normal(hand['V'], right)
        return n / np.linalg.norm(n)
