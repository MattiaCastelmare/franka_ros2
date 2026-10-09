#!/usr/bin/env python3
"""Hand tracker: RGB-D image -> landmarks 0, 5, 9, 17 of the ACTIVE hand -> HandTrackingRaw.

Stage 1 (camera callback): hand detector, RTMW (GPU) or MediaPipe Holistic (fallback).
Stage 2 (worker thread, overlapped with the next stage 1): ACTIVE hand (HandSelector),
3D landmarks from the aligned depth, palm plane, publishing.
When the ACTIVE hand is not seen: gripper camera (D405), then the arm bridge (pose
wrist + elbow, up to arm_bridge_s), then NO_HAND.
"""

import queue
import threading
import time

import cv2
import mediapipe as mp
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from franka_msgs.msg import HandTrackingRaw
from geometry_msgs.msg import Point, Vector3
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

from franka_experiments.utils.active_hand_selector import HandSelector
from franka_experiments.utils.camera_yaml import load_camera_info_yaml
from franka_experiments.utils.hand_detectors import GripperCamera, MediapipeHandDetector, signed_palm_normal

RAW = HandTrackingRaw


def unit(vector):
    vector = np.asarray(vector, dtype=float)
    if not np.isfinite(vector).all():
        return None
    norm = float(np.linalg.norm(vector))
    return None if norm <= 1e-12 else vector / norm


def draw_status(image, text):
    if image is not None:
        cv2.putText(image, text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 1)


class HumanHandTracker(Node):

    LANDMARK_IDS = (0, 5, 9, 17)
    GEOMETRY_LANDMARK_IDS = (0, 5, 9, 13, 17)  # palm plane only, not in HandTrackingRaw
    POSE_ARM = {RAW.HAND_LEFT: (15, 13), RAW.HAND_RIGHT: (16, 14)}  # pose wrist, elbow

    def __init__(self):
        super().__init__('human_hand_tracker')
        self.param = lambda name: self.declare_parameter(  # config/hand_tracking.yaml, passed by the launch
            name, descriptor=ParameterDescriptor(dynamic_typing=True)).value
        config_dir = get_package_share_directory('franka_experiments') + '/config/'

        # ---- camera model
        intrinsics = load_camera_info_yaml(config_dir + 'camera_intrinsics.yaml')
        if intrinsics is None:
            raise RuntimeError('camera_intrinsics.yaml not valid')
        k = intrinsics['k']
        self.fx, self.fy, self.cx, self.cy = float(k[0]), float(k[4]), float(k[2]), float(k[5])

        with open(config_dir + 'camera_extrinsics.yaml', 'r', encoding='utf-8') as file:
            extrinsics = yaml.safe_load(file)
        self.target_frame, self.camera_frame = extrinsics['parent_frame'], extrinsics['child_frame']
        t, q = extrinsics['translation'], extrinsics['rotation']
        self.camera_translation = np.array([t['x'], t['y'], t['z']], dtype=float)
        self.camera_rotation = Rotation.from_quat([q['x'], q['y'], q['z'], q['w']]).as_matrix()

        # ---- hand detector: RTMW, MediaPipe Holistic if RTMW is not available
        self.publish_debug = bool(self.param('publish_debug_image'))
        self.show_selected_landmarks = bool(self.param('show_selected_landmarks'))
        self.detector_name = str(self.param('hand_detector'))
        self.detector = None
        if self.detector_name == 'rtmw':
            mode = str(self.param('rtmw_mode'))
            if mode not in ('lightweight', 'balanced'):
                self.get_logger().warn(f'rtmw_mode {mode} not supported (lightweight | balanced): lightweight')
                mode = 'lightweight'
            try:
                from franka_experiments.utils.hand_detectors import RtmwHandDetector
                self.detector = RtmwHandDetector(
                    mode=mode, tensorrt=bool(self.param('rtmw_tensorrt')),
                    to_base=lambda u, v, z: self.apply_transform(
                        np.array([(u - self.cx) * z / self.fx, (v - self.cy) * z / self.fy, z])))
                self.get_logger().info(f'RTMW {mode}: {self.detector.engine}')
            except Exception as error:  # no GPU / rtmlib: keep the tracker running
                self.get_logger().warn(f'RTMW unavailable ({error}): hand_detector=mediapipe')
                self.detector_name = 'mediapipe'
        if self.detector is None:
            complexity = int(self.param('model_complexity'))
            if complexity not in (0, 1):
                raise ValueError('model_complexity must be 0 or 1')
            self.detector = MediapipeHandDetector(
                bool(self.param('static_image_mode')), complexity,
                float(self.param('min_detection_confidence')), float(self.param('min_tracking_confidence')))

        self.palm_normal_method = str(self.param('palm_normal_method'))
        if self.detector_name == 'rtmw' and self.palm_normal_method == 'mediapipe':
            self.palm_normal_method = 'depth'  # RTMW hands have no z
        self.arm_bridge_s = float(self.param('arm_bridge_s'))
        self.get_logger().info(
            f'hand_detector={self.detector_name}, palm_normal_method={self.palm_normal_method}')

        # ---- ACTIVE hand selector and gripper camera (both need the robot TF)
        self.selector = HandSelector(self, config_dir + 'fr3_complete.yaml', self.target_frame, self.param)
        self.gripper = None
        self._last_active = None  # (t, side, palm in base) of the last measured ACTIVE hand
        self._arm_anchor = None
        if bool(self.param('gripper_camera')):
            self.gripper = GripperCamera(
                self, config_dir + 'd405_extrinsics.yaml',
                lambda link, stamp: self.selector.tf.lookup_best_effort([link], stamp).get(link),
                str(self.param('gripper_camera_namespace')),
                debug=self.publish_debug,
                target=lambda: (None if self._last_active is None
                                else (self._last_active[0], self._last_active[2])))

        # ---- ROS I/O: newest frame only (no backlog latency), two overlapped stages
        self.bridge = CvBridge()
        self.tracking_publisher = self.create_publisher(HandTrackingRaw, '/handover/hand_tracking_raw', 10)
        self.debug_image_publisher = self.create_publisher(Image, '/handover/hand_debug_image_raw', 10)
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.synchronizer = ApproximateTimeSynchronizer(
            [Subscriber(self, Image, '/camera/camera/color/image_raw', qos_profile=image_qos),
             Subscriber(self, Image, '/camera/camera/aligned_depth_to_color/image_raw', qos_profile=image_qos)],
            queue_size=2, slop=0.05)
        self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info',
                                 self.color_info_callback, qos_profile_sensor_data)

        self._frames = queue.Queue(maxsize=1)  # stage 2 one frame behind at most
        self._timing = {'stage1': [], 'stage2': [], 'stamps': []}
        self._stopping = threading.Event()
        self._worker = threading.Thread(target=self._frame_worker, daemon=True)
        self._worker.start()
        self.synchronizer.registerCallback(self.image_callback)

        self.get_logger().info(f'Human hand tracker: {self.camera_frame} -> {self.target_frame}')

    def color_info_callback(self, msg):
        if msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.fx, self.fy = float(msg.k[0]), float(msg.k[4])
            self.cx, self.cy = float(msg.k[2]), float(msg.k[5])

    # ------------------------------------------------------------ RGB-D geometry
    def landmark_to_3d(self, landmark, rgb_shape, depth_image, depth_encoding, fallback_depth=None):
        """Normalised landmark -> camera point, depth = median of a 5x5 patch (or fallback_depth)."""
        h, w = rgb_shape[:2]
        u = int(np.clip(round(landmark.x * (w - 1)), 0, w - 1))
        v = int(np.clip(round(landmark.y * (h - 1)), 0, h - 1))
        depth = self.median_depth(depth_image, depth_encoding, u, v)
        depth = fallback_depth if depth is None else depth
        if depth is None:
            return None

        return np.array([(u - self.cx) * depth / self.fx, (v - self.cy) * depth / self.fy, depth], dtype=float)

    @staticmethod
    def median_depth(depth_image, encoding, u, v, radius=2):
        h, w = depth_image.shape[:2]
        patch = depth_image[max(0, v - radius):min(h, v + radius + 1),
                            max(0, u - radius):min(w, u + radius + 1)]
        valid = patch[np.isfinite(patch) & (patch > 0)]
        if valid.size == 0:
            return None

        depth = float(np.median(valid))
        if encoding in ('16UC1', 'mono16') or depth_image.dtype == np.uint16:
            depth *= 0.001
        return depth if 0.10 <= depth <= 3.00 else None

    def apply_transform(self, point):
        return self.camera_rotation @ point + self.camera_translation

    def landmarks_3d(self, hand, ids, shape, depth, encoding):
        """Camera points of `ids`; a landmark without depth takes the median depth of the others
        (if at least two agree within 15 cm). Returns (points, directly measured, any filled)."""
        points = {i: self.landmark_to_3d(hand[i], shape, depth, encoding) for i in ids}
        direct = {i: points[i] is not None for i in ids}
        depths = [p[2] for p in points.values() if p is not None]
        filled = False

        if len(depths) >= 2:
            reference = float(np.median(depths))
            consistent = [d for d in depths if abs(d - reference) <= 0.15]
            if len(consistent) >= 2:
                reference = float(np.median(consistent))
                for i in ids:
                    if points[i] is None:
                        points[i] = self.landmark_to_3d(hand[i], shape, depth, encoding,
                                                        fallback_depth=reference)
                        filled = True

        return points, direct, filled

    def candidate_palm(self, hand_landmarks, shape, depth, encoding):
        """Palm of a candidate hand for the selector: mean of MCP 5, 9, 17 in the target frame."""
        points, _, _ = self.landmarks_3d(hand_landmarks.landmark, (5, 9, 17), shape, depth, encoding)
        if any(p is None for p in points.values()):
            return None

        palm = np.array([self.apply_transform(p) for p in points.values()], dtype=float)
        return palm.mean(axis=0) if np.isfinite(palm).all() else None

    # ------------------------------------------------------------ palm plane
    def palm_plane(self, hand, shape, depth, encoding, right):
        """(normal, anchor) of the palm in the target frame (palm_normal_method):
          depth     : PCA plane of the RGB-D points 0/5/9/13/17 (>= 4, with 0, 5, 17); the normal keeps the
                      PCA sign, anchor = (index - wrist) x (pinky - wrist) resolves it downstream
          mediapipe : MediaPipe 3D landmarks, signed out of the palm per frame; anchor = +-normal (RIGHT +)"""
        if self.palm_normal_method != 'depth' and right is not None:
            h, w = shape[:2]
            V = np.array([[p.x * w, p.y * h, p.z * w] for p in hand])  # weak perspective
            n = unit(self.camera_rotation @ signed_palm_normal(V, right))
            return (None, None) if n is None else (n, n if right else -n)

        points = {}
        for i in self.GEOMETRY_LANDMARK_IDS:
            p = self.landmark_to_3d(hand[i], shape, depth, encoding)
            if p is not None and np.isfinite(p := self.apply_transform(p)).all():
                points[i] = np.asarray(p, dtype=float)
        if len(points) < 4 or not all(i in points for i in (0, 5, 17)):
            return None, None

        P = np.stack(list(points.values()))
        centred = P - np.mean(P, axis=0)
        try:
            values, vectors = np.linalg.eigh(centred.T @ centred / len(P))
        except np.linalg.LinAlgError:
            return None, None

        normal = unit(vectors[:, np.argmin(values)])
        anchor = unit(np.cross(points[5] - points[0], points[17] - points[0]))
        return (None, None) if normal is None or anchor is None else (normal, anchor)

    # ------------------------------------------------------------ lost ACTIVE hand
    def _pose_point(self, pose, index, shape, depth, encoding):
        if pose is None or pose.landmark[index].visibility < 0.5:
            return None
        return self.landmark_to_3d(pose.landmark[index], shape, depth, encoding)

    def _store_arm_anchor(self, pose, side, points_camera, shape, depth, encoding, t):
        """Palm landmarks relative to the pose wrist, while the hand is observed."""
        if side not in self.POSE_ARM or any(points_camera.get(i) is None for i in self.LANDMARK_IDS):
            return

        wrist = self._pose_point(pose, self.POSE_ARM[side][0], shape, depth, encoding)
        points = np.array([points_camera[i] for i in self.LANDMARK_IDS])
        if wrist is None or np.linalg.norm(points.mean(0) - wrist) > 0.25:  # wrist depth on background
            return
        elbow = self._pose_point(pose, self.POSE_ARM[side][1], shape, depth, encoding)
        self._arm_anchor = (t, side, points - wrist, wrist, elbow)

    def _arm_prediction(self, pose, shape, depth, encoding, t):
        """Lost ACTIVE hand: pose wrist (RGB-D) + last wrist->palm offsets, rotated with the
        forearm. Offline (6 bags): 1.4 cm median at 0.2 s, 2.0 cm at 1 s, against 2.1 / 10 cm
        of a constant-velocity prediction. Only extends the hand being tracked."""
        anchor = self._arm_anchor
        if anchor is None or not 0.0 < t - anchor[0] <= self.arm_bridge_s:
            return None

        t0, side, offsets, wrist0, elbow0 = anchor
        if side != self.selector.active:
            return None
        wrist = self._pose_point(pose, self.POSE_ARM[side][0], shape, depth, encoding)
        if wrist is None or np.linalg.norm(wrist - wrist0) > 0.1 + 2.0 * (t - t0):  # <= 2 m/s
            return None

        # rotate the offsets with the forearm (elbow -> wrist), when both elbows are seen
        elbow = self._pose_point(pose, self.POSE_ARM[side][1], shape, depth, encoding)
        R = np.eye(3)
        if elbow is not None and elbow0 is not None:
            a, b = wrist0 - elbow0, wrist - elbow
            if min(np.linalg.norm(a), np.linalg.norm(b)) > 0.05:
                R = Rotation.align_vectors([b], [a])[0].as_matrix()

        return side, list(wrist + offsets @ R.T)

    def _publish_gripper_hand(self, stamp, t, start_time):
        """ACTIVE hand lost by the main camera but seen by the gripper camera near its
        last position (<= 1 s, <= 1 m/s): publish that measurement."""
        last = self._last_active
        if self.gripper is None or last is None or not 0.0 < t - last[0] <= 1.0:
            return False
        side = last[1]
        if side != self.selector.active:
            return False
        hand = self.gripper.hand_near(last[2], t, 0.15 + 1.0 * (t - last[0]))
        if hand is None:
            return False

        right = side == RAW.HAND_RIGHT
        n = self.gripper.palm_normal(hand, right)
        points = hand['pts']
        valid = [p for p in points if p is not None]
        self.publish_tracking(
            stamp, RAW.TRACKING_FULL if len(valid) == 4 else RAW.TRACKING_PARTIAL, points,
            [RAW.DIRECT if p is not None else RAW.INVALID for p in points], start_time,
            handedness=side,
            palm_plane_normal=n,
            palm_anchor_cross=None if n is None else (n if right else -n))
        self._last_active = (t, side, np.mean(valid, axis=0))
        return True

    # ------------------------------------------------------------ stage 1: front-end
    def image_callback(self, rgb_msg, depth_msg):
        start_time = time.perf_counter()
        try:
            image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as error:
            self.get_logger().warn(f'Error in cv_bridge: {error}', throttle_duration_sec=2.0)
            return

        front = self.detector.process(
            image, depth_image, 1e-3 if depth_msg.encoding in ('16UC1', 'mono16') else 1.0, self.fx,
            rgb_msg.header.stamp.sec + 1e-9 * rgb_msg.header.stamp.nanosec)
        self._timing['stage1'].append(1e3 * (time.perf_counter() - start_time))

        try:
            self._frames.put_nowait((rgb_msg, depth_msg, image, depth_image, front, start_time))
        except queue.Full:
            self.get_logger().warn('Tracker stage 2 behind: frame dropped', throttle_duration_sec=2.0)

    def _frame_worker(self):
        while not self._stopping.is_set():
            try:
                job = self._frames.get(timeout=0.2)
            except queue.Empty:
                continue

            t0 = time.perf_counter()
            try:
                self._finish_frame(*job)
            except Exception as error:  # keep the worker alive
                self.get_logger().error(f'Tracker stage 2: {error}', throttle_duration_sec=2.0)
            self._log_timing(job[0].header.stamp, 1e3 * (time.perf_counter() - t0))

    def _log_timing(self, stamp, stage2_ms):
        """Every 150 frames: rate, stage times, camera frames skipped (gaps in the stamps)."""
        tm = self._timing
        tm['stage2'].append(stage2_ms)
        tm['stamps'].append(stamp.sec + 1e-9 * stamp.nanosec)
        if len(tm['stamps']) < 150:
            return

        st = np.array(tm['stamps'])
        gaps = np.diff(st)
        period = max(float(gaps.min()), 1e-3)
        skipped = int(np.sum(np.maximum(np.round(gaps / period) - 1, 0)))
        self.get_logger().info(
            f'Tracker {len(gaps) / (st[-1] - st[0]):.1f} Hz | stage1 {np.median(tm["stage1"]):.0f}/'
            f'{np.percentile(tm["stage1"], 95):.0f} ms | stage2 {np.median(tm["stage2"]):.0f}/'
            f'{np.percentile(tm["stage2"], 95):.0f} ms (median/p95) | '
            f'camera frames skipped {skipped}/{len(gaps) + skipped}')
        for v in tm.values():
            v.clear()

    # ------------------------------------------------------------ stage 2: ACTIVE hand -> HandTrackingRaw
    def _finish_frame(self, rgb_msg, depth_msg, image, depth_image, front, start_time):
        stamp, encoding, shape = rgb_msg.header.stamp, depth_msg.encoding, image.shape
        stamp_s = stamp.sec + 1e-9 * stamp.nanosec
        result = self.detector.complete(
            front, image, depth_image, 1e-3 if encoding in ('16UC1', 'mono16') else 1.0, stamp_s)
        debug_image = image.copy() if self.publish_debug else None

        active_side, hand_landmarks, _, standby_landmarks = self.selector.select(
            result, lambda lms: self.candidate_palm(lms, shape, depth_image, encoding), stamp)
        if debug_image is not None and standby_landmarks is not None:  # standby: drawn only, never tracked
            overlay = debug_image.copy()
            mp.solutions.drawing_utils.draw_landmarks(
                overlay, standby_landmarks, mp.solutions.hands.HAND_CONNECTIONS,
                mp.solutions.drawing_utils.DrawingSpec(color=(185, 185, 185), thickness=1, circle_radius=1),
                mp.solutions.drawing_utils.DrawingSpec(color=(150, 150, 150), thickness=1, circle_radius=1))
            debug_image[:] = cv2.addWeighted(overlay, 0.38, debug_image, 0.62, 0.0)

        if hand_landmarks is None:
            if self._publish_gripper_hand(stamp, stamp_s, start_time):
                self.get_logger().info('ACTIVE hand from the gripper camera', throttle_duration_sec=2.0)
                draw_status(debug_image, 'HAND FROM GRIPPER CAMERA')
            elif (predicted := self._arm_prediction(
                    result.pose_landmarks, shape, depth_image, encoding, stamp_s)):
                side, points_camera = predicted
                self.publish_tracking(stamp, RAW.TRACKING_ESTIMATED,
                                      [self.apply_transform(p) for p in points_camera],
                                      [RAW.ESTIMATED] * 4, start_time, handedness=side)
                draw_status(debug_image, 'HAND FROM ARM (pose)')
            else:
                self.publish_tracking(stamp, RAW.NO_HAND, [None] * 4, [RAW.INVALID] * 4, start_time)
                draw_status(debug_image, 'ACTIVE HAND NOT OBSERVED')
            self.publish_debug_image(debug_image, rgb_msg)
            return

        hand, right = hand_landmarks.landmark, active_side == RAW.HAND_RIGHT
        plane, anchor = self.palm_plane(hand, shape, depth_image, encoding, right)
        if self.show_selected_landmarks and debug_image is not None:
            h, w = shape[:2]
            for i in self.GEOMETRY_LANDMARK_IDS:
                cv2.circle(debug_image, (int(np.clip(round(hand[i].x * (w - 1)), 0, w - 1)),
                                         int(np.clip(round(hand[i].y * (h - 1)), 0, h - 1))), 4, (0, 0, 255), 1)

        points_camera, direct, estimated = self.landmarks_3d(
            hand, self.LANDMARK_IDS, shape, depth_image, encoding)
        points_base = [None if points_camera[i] is None else self.apply_transform(points_camera[i])
                       for i in self.LANDMARK_IDS]
        types = [RAW.DIRECT if direct[i] else (RAW.ESTIMATED if points_camera[i] is not None else RAW.INVALID)
                 for i in self.LANDMARK_IDS]
        valid_count = sum(p is not None for p in points_base)
        state = (RAW.INVALID_DEPTH if valid_count == 0
                 else RAW.TRACKING_PARTIAL if valid_count < 4
                 else RAW.TRACKING_ESTIMATED if RAW.ESTIMATED in types
                 else RAW.TRACKING_FULL)

        if valid_count:
            palm = np.mean([p for p in points_base if p is not None], axis=0)
            self._last_active = (stamp_s, int(active_side), palm)
            # the gripper camera sees the palm from close by: its normal wins
            seen = None if self.gripper is None else self.gripper.hand_near(palm, stamp_s, 0.06)
            n = None if seen is None else self.gripper.palm_normal(seen, right)
            if n is not None:
                plane, anchor = n, (n if right else -n)

        self.publish_tracking(stamp, state, points_base, types, start_time, handedness=int(active_side),
                              palm_plane_normal=plane, palm_anchor_cross=anchor)
        self._store_arm_anchor(result.pose_landmarks, int(active_side), points_camera, shape, depth_image,
                               encoding, stamp_s)

        if points_base[0] is None:
            draw_status(debug_image, 'INVALID WRIST DEPTH')
            self.get_logger().warn('Hand detected, but wrist depth is invalid', throttle_duration_sec=2.0)
        elif all(p is not None for p in points_base):
            draw_status(debug_image, 'TRACKING ESTIMATED' if estimated else 'TRACKING FULL')
        else:
            draw_status(debug_image, 'TRACKING WRIST')
        self.publish_debug_image(debug_image, rgb_msg)

    # ------------------------------------------------------------ publishing
    def publish_tracking(self, stamp, tracking_state, points, measurement_types, start_time,
                         handedness=None, palm_plane_normal=None, palm_anchor_cross=None):
        msg = HandTrackingRaw()
        msg.header.stamp, msg.header.frame_id = stamp, self.target_frame
        msg.tracking_state = int(tracking_state)
        msg.landmark_ids = list(self.LANDMARK_IDS)
        msg.valid = [p is not None for p in points]
        msg.measurement_type = [int(m) for m in measurement_types]
        msg.processing_latency_ms = float(1000.0 * (time.perf_counter() - start_time))
        msg.handedness = int(RAW.HAND_UNKNOWN if handedness is None else handedness)
        msg.handedness_score = 0.0 if handedness is None else 1.0
        msg.palm_plane_valid = palm_plane_normal is not None and palm_anchor_cross is not None
        if msg.palm_plane_valid:
            msg.palm_plane_normal = Vector3(x=float(palm_plane_normal[0]), y=float(palm_plane_normal[1]),
                                            z=float(palm_plane_normal[2]))
            msg.palm_anchor_cross = Vector3(x=float(palm_anchor_cross[0]), y=float(palm_anchor_cross[1]),
                                            z=float(palm_anchor_cross[2]))
        msg.positions = [Point() if p is None else Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))
                         for p in points]
        self.tracking_publisher.publish(msg)

    def publish_debug_image(self, image, original_msg):
        if not self.publish_debug:
            return
        if image is None:
            self.debug_image_publisher.publish(original_msg)
            return
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header = original_msg.header
        self.debug_image_publisher.publish(msg)

    def destroy_node(self):
        self._stopping.set()
        self._worker.join(timeout=1.0)
        self.detector.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HumanHandTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C / launch shutdown
        pass
    except Exception:
        if rclpy.ok():  # otherwise a shutdown race (context already gone)
            raise
    finally:
        if node.gripper is not None:
            node.gripper.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
