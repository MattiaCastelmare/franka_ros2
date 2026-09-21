#!/usr/bin/env python3

import time
from collections import deque

import cv2
import mediapipe as mp
import numpy as np
import rclpy
import yaml

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from franka_msgs.msg import HandTrackingRaw
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped, Vector3
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

from franka_experiments.utils.camera_yaml import load_camera_info_yaml
from franka_experiments.utils.hand_visualization import (
    draw_selected_landmarks,
    draw_status,
)

from franka_experiments.utils.distance_utils import load_robot_config
from franka_experiments.utils.tf_manager import TFManager
from franka_experiments.utils.active_hand_selector import ActiveHandSelectorMixin
from franka_experiments.utils.palm import PalmGeometryMixin
from pathlib import Path



def _load_hand_tracking_defaults():
    candidates = []
    try:
        candidates.append(
            Path(
                get_package_share_directory(
                    "franka_experiments"
                )
            )
            / "config"
            / "hand_tracking.yaml"
        )
    except Exception:
        pass
    candidates.append(
        Path(__file__).resolve().parents[2]
        / "config"
        / "hand_tracking.yaml"
    )
    path = next(
        (
            candidate
            for candidate in candidates
            if candidate.exists()
        ),
        None,
    )
    if path is None:
        raise FileNotFoundError(
            "config/hand_tracking.yaml not found"
        )
    with path.open(
        "r",
        encoding="utf-8",
    ) as stream:
        data = yaml.safe_load(stream) or {}
    params = (
        data
        .get(
            "human_hand_tracker",
            {},
        )
        .get(
            "ros__parameters",
            {},
        )
    )
    if not isinstance(
        params,
        dict,
    ):
        raise RuntimeError(
            "Invalid hand_tracking.yaml"
        )
    return params


_HAND_TRACKING_DEFAULTS = (
    _load_hand_tracking_defaults()
)

class HandRgbdMixin:
    """RGB-D operations used by HumanHandTracker."""

    def landmark_to_3d(
        self,
        landmark,
        rgb_shape,
        depth_image,
        depth_encoding,
        fallback_depth=None,
    ):
        rgb_height, rgb_width = rgb_shape[:2]

        u_rgb = int(np.clip(
            round(landmark.x * (rgb_width - 1)),
            0,
            rgb_width - 1,
        ))
        v_rgb = int(np.clip(
            round(landmark.y * (rgb_height - 1)),
            0,
            rgb_height - 1,
        ))

        depth_m = self.median_depth(
            depth_image,
            depth_encoding,
            u_rgb,
            v_rgb,
        )

        if depth_m is None:
            depth_m = fallback_depth

        if depth_m is None:
            return None

        x = (u_rgb - self.cx) * depth_m / self.fx
        y = (v_rgb - self.cy) * depth_m / self.fy

        return np.array(
            [x, y, depth_m],
            dtype=float,
        )

    @staticmethod
    def median_depth(depth_image, encoding, u, v):
        radius = 2
        height, width = depth_image.shape[:2]

        patch = depth_image[
            max(0, v - radius):min(height, v + radius + 1),
            max(0, u - radius):min(width, u + radius + 1),
        ]

        valid = patch[
            np.isfinite(patch) & (patch > 0)
        ]

        if valid.size == 0:
            return None

        depth = float(np.median(valid))

        if (
            encoding in ('16UC1', 'mono16')
            or depth_image.dtype == np.uint16
        ):
            depth *= 0.001

        if depth < 0.10 or depth > 3.00:
            return None

        return depth

    def apply_transform(self, point):
        return (
            self.camera_rotation @ point
            + self.camera_translation
        )


class HumanHandTracker(ActiveHandSelectorMixin, HandRgbdMixin, PalmGeometryMixin, Node):
    """RGB-D tracker for hand landmarks 0, 5, 9 and 17."""

    LANDMARK_IDS = (0, 5, 9, 17)

    # Additional palm-only geometry landmark.
    #
    # This DOES NOT change the Kalman/W75 landmark set.
    GEOMETRY_LANDMARK_IDS = (
        0,
        5,
        9,
        13,
        17,
    )

    def _tracking_param(self, name):
        if name not in _HAND_TRACKING_DEFAULTS:
            raise KeyError(
                f"Missing YAML parameter: {name}"
            )
        self.declare_parameter(
            name,
            _HAND_TRACKING_DEFAULTS[name],
        )
        return self.get_parameter(name)

    def __init__(self):
        super().__init__('human_hand_tracker')
        config_dir = (
            get_package_share_directory('franka_experiments') + '/config/'
        )
        intrinsics = load_camera_info_yaml(
            config_dir + 'camera_intrinsics.yaml'
        )
        if intrinsics is None:
            raise RuntimeError('camera_intrinsics.yaml not valid')
        k = intrinsics['k']
        self.fx = float(k[0])
        self.fy = float(k[4])
        self.cx = float(k[2])
        self.cy = float(k[5])
        with open(
            config_dir + 'camera_extrinsics.yaml',
            'r',
            encoding='utf-8',
        ) as file:
            extrinsics = yaml.safe_load(file)
        self.target_frame = extrinsics['parent_frame']
        self.camera_frame = extrinsics['child_frame']
        t = extrinsics['translation']
        q = extrinsics['rotation']
        self.camera_translation = np.array(
            [t['x'], t['y'], t['z']],
            dtype=float,
        )
        self.camera_rotation = Rotation.from_quat([
            q['x'],
            q['y'],
            q['z'],
            q['w'],
        ]).as_matrix()
        self.show_selected_landmarks = bool(
            self._tracking_param("show_selected_landmarks").value
        )
        self.publish_debug = bool(
            self._tracking_param("publish_debug_image").value
        )
        self.model_complexity = int(
            self._tracking_param("model_complexity").value
        )
        if self.model_complexity not in (0, 1):
            raise ValueError(
                'model_complexity must be 0 or 1'
            )
        self.static_image_mode = bool(
            self._tracking_param("static_image_mode").value
        )
        self.min_tracking_confidence = float(
            self._tracking_param("min_tracking_confidence").value
        )
        self.min_detection_confidence = float(
            self._tracking_param("min_detection_confidence").value
        )
        self.max_num_hands = int(
            self._tracking_param("max_num_hands").value
        )
        if self.max_num_hands not in (1, 2):
            raise ValueError(
                'max_num_hands must be 1 or 2'
            )
        self.bridge = CvBridge()
        # HOLISTIC_ACTIVE_STANDBY_V1
        #
        # One body-aware frontend:
        #   - anatomical LEFT / RIGHT
        #   - pose wrists / shoulders
        #   - hand landmarks
        #
        # No global Hands+Holistic double inference.
        self.hands = mp.solutions.holistic.Holistic(
            static_image_mode=self.static_image_mode,
            model_complexity=self.model_complexity,
            smooth_landmarks=True,
            enable_segmentation=False,
            refine_face_landmarks=False,
            min_detection_confidence=self.min_detection_confidence,
            min_tracking_confidence=self.min_tracking_confidence,
        )
        self.drawing_utils = mp.solutions.drawing_utils
        self.hand_connections = mp.solutions.hands.HAND_CONNECTIONS
        # ACTIVE / STANDBY role state.
        #
        # Identity and interaction role are separate:
        # LEFT/RIGHT is anatomical.
        # active_hand_side is the hand currently feeding
        # Kalman -> W75 -> HandState.
        # INTERACTION_SELECTOR_V2_ROBOT_CENTRIC
        #
        # LEFT / RIGHT = anatomical identity from Holistic.
        #
        # ACTIVE / STANDBY = interaction role.
        #
        # Only lightweight direct RGB-D palm candidates are
        # evaluated here.  Kalman/W75/geometry still run ONLY
        # on the selected ACTIVE physical hand.
        self.active_hand_side = (
            HandTrackingRaw.HAND_UNKNOWN
        )
        self._pending_active_side = (
            HandTrackingRaw.HAND_UNKNOWN
        )
        self._pending_active_count = 0
        self._visible_hand_sides = set()
        self.interaction_switch_confirm_frames = int(
            self._tracking_param("interaction_switch_confirm_frames").value
        )
        # Challenger must improve the robot-centric metric
        # by this amount before challenging the ACTIVE hand.
        self.interaction_switch_margin_m = float(
            self._tracking_param("interaction_switch_margin_m").value
        )
        # Short constant-rate look-ahead used ONLY by the
        # selector metric:
        #
        # metric = distance - horizon * closing_velocity
        self.interaction_selector_horizon_s = float(
            self._tracking_param("interaction_selector_horizon_s").value
        )
        # If the ACTIVE observation disappears, do not jump
        # immediately to a pre-existing standby hand.
        self.interaction_missing_grace_frames = int(
            self._tracking_param("interaction_missing_grace_frames").value
        )
        # Before grace expires, a missing-active switch may
        # start only if the challenger is actually approaching
        # the robot.
        self.interaction_min_closing_m_s = float(
            self._tracking_param("interaction_min_closing_m_s").value
        )
        # INTERACTION_SELECTOR_V3_ONSET_GATE
        #
        # Body-relative arm extension:
        #
        #   |wrist - shoulder| / shoulder_width
        #
        # Its derivative is largely insensitive to whole-body
        # translation and provides evidence that a hand is
        # actively being extended for a new interaction.
        self.interaction_min_arm_extension_rate_s = float(
            self._tracking_param("interaction_min_arm_extension_rate_s").value
        )
        # INTERACTION_SELECTOR_V31_CONSENSUS_ONSET
        #
        # Arm extension alone is NOT sufficient anymore.
        #
        # If body-relative extension is weak/unavailable,
        # allow a switch only for a clearly stronger
        # robot-directed approach.
        self.interaction_strong_closing_m_s = float(
            self._tracking_param("interaction_strong_closing_m_s").value
        )
        self.interaction_closing_advantage_m_s = float(
            self._tracking_param("interaction_closing_advantage_m_s").value
        )
        # INTERACTION_SELECTOR_V34_ENGAGED_LOCK
        # INTERACTION_SELECTOR_V341_ENGAGED_GUARDS
        #
        # These are interaction-state parameters, not
        # dataset-specific LEFT/RIGHT rules.
        # Preserve an ENGAGED physical-hand role through a
        # short occlusion.
        self.interaction_engaged_memory_s = float(
            self._tracking_param("interaction_engaged_memory_s").value
        )
        # Static interaction advantage needed when the
        # challenger also shows a genuine interaction onset.
        self.interaction_engaged_onset_advantage = float(
            self._tracking_param("interaction_engaged_onset_advantage").value
        )
        # Strong proximity + body-reach evidence can correct
        # an already-wrong lock even after the real hand has
        # become stationary.
        self.interaction_engaged_recovery_advantage = float(
            self._tracking_param("interaction_engaged_recovery_advantage").value
        )
        # Explicit robot-directed retraction of current ACTIVE.
        self.interaction_engaged_retract_m_s = float(
            self._tracking_param("interaction_engaged_retract_m_s").value
        )
        if self.interaction_switch_confirm_frames < 1:
            raise ValueError(
                'interaction_switch_confirm_frames '
                'must be >= 1'
            )
        # Robot control-point model.
        #
        # Same implementation/configuration already used by
        # distance_handover_estimator.py.
        selector_config = load_robot_config(
            config_dir + 'fr3_complete.yaml'
        )
        self.selector_robot_cfg = (
            selector_config['robot']
        )
        self.selector_distance_cfg = (
            selector_config['distance']
        )
        self.selector_base_frame = (
            self.selector_robot_cfg[
                'base_frame'
            ]
        )
        self.selector_ee_link = (
            self.selector_robot_cfg.get(
                'ee_link',
                'fr3_link8',
            )
        )
        selector_segments = [
            segment
            for segment
            in self.selector_robot_cfg[
                'segments'
            ]
            if segment[
                'end_link'
            ]
            ==
            self.selector_ee_link
        ]
        if not selector_segments:
            raise RuntimeError(
                'Interaction selector: '
                'no EE segment found'
            )
        selector_ee_segment = (
            selector_segments[-1]
        )
        self.selector_ee_segment_links = [
            selector_ee_segment[
                'start_link'
            ],
            selector_ee_segment[
                'end_link'
            ],
        ]
        self.selector_tf_buffer = Buffer()
        self.selector_tf_listener = (
            TransformListener(
                self.selector_tf_buffer,
                self,
            )
        )
        self.selector_tf_manager = TFManager(
            tf_buffer=self.selector_tf_buffer,
            base_frame=self.selector_base_frame,
            critical_links=[
                self.selector_ee_link
            ],
            cache_max_age_s=float(
                self.selector_distance_cfg.get(
                    'tf_cache_max_age_s',
                    0.5,
                )
            ),
            logger=self.get_logger(),
        )
        # Per-physical-hand lightweight distance histories.
        self._interaction_distance_history = {
            HandTrackingRaw.HAND_LEFT:
                deque(maxlen=4),
            HandTrackingRaw.HAND_RIGHT:
                deque(maxlen=4),
        }
        # V3:
        # short body-relative arm-motion history.
        self._interaction_arm_history = {
            HandTrackingRaw.HAND_LEFT:
                deque(maxlen=5),
            HandTrackingRaw.HAND_RIGHT:
                deque(maxlen=5),
        }
        self.wrist_publisher = self.create_publisher(
            PoseStamped,
            '/handover/hand_state_raw',
            10,
        )
        self.landmarks_publisher = self.create_publisher(
            PoseArray,
            '/handover/hand_landmarks_raw',
            10,
        )
        # Diagnostic only:
        # MediaPipe 21 world landmarks.
        #
        # These landmarks are NOT used by Kalman, W75,
        # HandState or control.
        self.tracking_publisher = self.create_publisher(
            HandTrackingRaw,
            '/handover/hand_tracking_raw',
            10,
        )
        self.debug_image_publisher = self.create_publisher(
            Image,
            '/handover/hand_debug_image_raw',
            10,
        )
        image_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.rgb_sub = Subscriber(
            self,
            Image,
            '/camera/camera/color/image_raw',
            qos_profile=image_qos,
        )
        self.depth_sub = Subscriber(
            self,
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            qos_profile=image_qos,
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.color_info_callback,
            qos_profile_sensor_data,
        )
        self.synchronizer = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=4,
            slop=0.05,
        )
        self.synchronizer.registerCallback(self.image_callback)
        self.published_frames = 0
        self.get_logger().info(
            'Human hand tracker avviato: '
            f'{self.camera_frame} -> {self.target_frame}; '
            f'show_selected_landmarks={self.show_selected_landmarks}'
        )

    def color_info_callback(self, msg):
        if msg.k[0] <= 0.0 or msg.k[4] <= 0.0:
            return
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])








    # SELECTOR_DEBUG_V1_DIAGNOSTIC_ONLY
    #
    # IMPORTANT:
    # This code does NOT influence ACTIVE/STANDBY selection.
    # It only records what LEFT and RIGHT looked like before
    # the selector decision.












    #
    # DIAGNOSTIC ONLY.
    #
    # Measures whether each Holistic hand is geometrically
    # owned by the corresponding body:
    #
    # shoulder -> elbow -> pose wrist -> hand wrist
    #
    # Absolutely NO effect on selection/tracking.


    #
    # DIAGNOSTIC ONLY.
    #
    # Compares Pose:
    #   wrist / thumb / index / pinky
    #
    # against Hand:
    #   0 / 4 / 8 / 20
    #
    # Also measures temporal continuity of Pose wrist.
    #
    # NO selector decision is changed.



    def _draw_soft_standby(
        self,
        image,
        hand_landmarks,
        side,
    ):
        if (
            image is None
            or hand_landmarks is None
        ):
            return
        overlay = image.copy()
        self.drawing_utils.draw_landmarks(
            overlay,
            hand_landmarks,
            self.hand_connections,
            self.drawing_utils.DrawingSpec(
                color=(185, 185, 185),
                thickness=1,
                circle_radius=1,
            ),
            self.drawing_utils.DrawingSpec(
                color=(150, 150, 150),
                thickness=1,
                circle_radius=1,
            ),
        )
        # Soft / translucent standby only.
        image[:] = cv2.addWeighted(
            overlay,
            0.38,
            image,
            0.62,
            0.0,
        )
        wrist = hand_landmarks.landmark[0]
        height, width = image.shape[:2]
        u = int(
            np.clip(
                round(
                    wrist.x
                    * (width - 1)
                ),
                0,
                width - 1,
            )
        )
        v = int(
            np.clip(
                round(
                    wrist.y
                    * (height - 1)
                ),
                0,
                height - 1,
            )
        )



    def image_callback(self, rgb_msg, depth_msg):
        start_time = time.perf_counter()
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(
                rgb_msg,
                desired_encoding='bgr8',
            )
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding='passthrough',
            )
        except Exception as error:
            self.get_logger().warn(
                f'Error in cv_bridge: {error}',
                throttle_duration_sec=2.0,
            )
            return
        depth_encoding = depth_msg.encoding
        debug_image = (
            bgr_image.copy()
            if self.publish_debug
            else None
        )
        rgb_image = cv2.cvtColor(
            bgr_image,
            cv2.COLOR_BGR2RGB,
        )
        result = self.hands.process(rgb_image)
        (
            active_side,
            hand_landmarks,
            standby_side,
            standby_landmarks,
        ) = self._select_active_holistic_hand(
            result,
            bgr_image.shape,
            depth_image,
            depth_encoding,
            rgb_msg.header.stamp,
        )
        # Standby is visualization / identity awareness only.
        # It does NOT enter Kalman, W75 or HandState.
        self._draw_soft_standby(
            debug_image,
            standby_landmarks,
            standby_side,
        )
        if hand_landmarks is None:
            self.publish_tracking(
                rgb_msg.header.stamp,
                HandTrackingRaw.NO_HAND, [None] * 4, [HandTrackingRaw.INVALID] * 4, start_time,)
            if debug_image is not None:
                draw_status(debug_image, 'ACTIVE HAND NOT OBSERVED')
            self.publish_debug_image(debug_image, rgb_msg,)
            return
        hand = hand_landmarks.landmark
        # This is no longer the legacy MediaPipe Hands
        # handedness classifier.
        #
        # It is the anatomical physical side already resolved
        # by the Holistic body-aware frontend.
        raw_handedness = int(active_side)
        raw_handedness_score = 1.0
        (geometry_plane, geometry_anchor,) = self.compute_palm_geometry_candidate(
            hand, bgr_image.shape, depth_image, depth_encoding,)
        if debug_image is not None:
            wrist_2d = hand[0]
            height, width = debug_image.shape[:2]
            u = int(np.clip(round(wrist_2d.x * (width - 1)), 0, width - 1,))
            v = int(np.clip(round(wrist_2d.y * (height - 1)), 0, height - 1,))
        if self.show_selected_landmarks:
            draw_selected_landmarks(debug_image, hand, self.GEOMETRY_LANDMARK_IDS,)
        points_camera = {}
        for landmark_id in self.LANDMARK_IDS:
            points_camera[landmark_id] = self.landmark_to_3d(
                hand[landmark_id], bgr_image.shape, depth_image, depth_encoding,)
        direct_valid = {landmark_id: points_camera[landmark_id] is not None
            for landmark_id in self.LANDMARK_IDS}
        valid_depths = [point[2] for point in points_camera.values() if point is not None]
        estimated = False
        if len(valid_depths) >= 2:
            reference_depth = float(np.median(valid_depths))
            consistent_depths = [
                depth for depth in valid_depths if abs(depth - reference_depth) <= 0.15]
            if len(consistent_depths) >= 2:
                reference_depth = float(np.median(consistent_depths))
                for landmark_id in self.LANDMARK_IDS:
                    if points_camera[landmark_id] is None:
                        points_camera[landmark_id] = self.landmark_to_3d(
                            hand[landmark_id], bgr_image.shape,
                            depth_image, depth_encoding, fallback_depth=reference_depth,)
                        estimated = True
        points_base = [self.apply_transform(points_camera[landmark_id])
            if points_camera[landmark_id] is not None
            else None for landmark_id in self.LANDMARK_IDS]
        measurement_types = [HandTrackingRaw.DIRECT if direct_valid[landmark_id]
            else (HandTrackingRaw.ESTIMATED if points_camera[landmark_id] is not None
                else HandTrackingRaw.INVALID) for landmark_id in self.LANDMARK_IDS]
        valid_count = sum(point is not None for point in points_base)
        if valid_count == 0:
            tracking_state = HandTrackingRaw.INVALID_DEPTH
        elif valid_count < 4:
            tracking_state = HandTrackingRaw.TRACKING_PARTIAL
        elif HandTrackingRaw.ESTIMATED in measurement_types:
            tracking_state = HandTrackingRaw.TRACKING_ESTIMATED
        else:
            tracking_state = HandTrackingRaw.TRACKING_FULL
        self.publish_tracking(rgb_msg.header.stamp, tracking_state, points_base, measurement_types,
            start_time, handedness=raw_handedness, handedness_score=raw_handedness_score,
            palm_plane_normal=geometry_plane, palm_anchor_cross=geometry_anchor,)
        wrist = points_base[0]
        if wrist is None:
            if debug_image is not None:
                draw_status(debug_image, 'INVALID WRIST DEPTH',)
            self.publish_debug_image(debug_image, rgb_msg,)
            self.get_logger().warn(
                'Hand detected, but wrist depth is invalid', throttle_duration_sec=2.0,)
            return
        self.publish_wrist(wrist, rgb_msg.header.stamp,)
        if all(point is not None for point in points_base):
            self.publish_landmarks(points_base, rgb_msg.header.stamp,)
            status = ('TRACKING ESTIMATED' if estimated else 'TRACKING FULL')
        else:
            status = 'TRACKING WRIST'
        if debug_image is not None:
            draw_status(debug_image, status, wrist,)
        self.publish_debug_image(debug_image, rgb_msg,)
        self.published_frames += 1
        if self.published_frames % 30 == 0:
            self.get_logger().info(
                f'Wrist [cm] in {self.target_frame}: ' f'x={100.0 * wrist[0]:.1f}, '
                f'y={100.0 * wrist[1]:.1f}, ' f'z={100.0 * wrist[2]:.1f}')




    def publish_tracking(self, stamp, tracking_state, points, measurement_types, start_time,
        handedness=None, handedness_score=0.0, palm_plane_normal=None, palm_anchor_cross=None,):
        msg = HandTrackingRaw()
        msg.header.stamp = stamp
        msg.header.frame_id = self.target_frame
        msg.tracking_state = int(tracking_state)
        msg.landmark_ids = list(self.LANDMARK_IDS)
        msg.valid = [point is not None for point in points]
        msg.measurement_type = [int(value) for value in measurement_types]
        msg.processing_latency_ms = float(1000.0 * (time.perf_counter() - start_time))
        msg.handedness = int((HandTrackingRaw.HAND_UNKNOWN if handedness is None else handedness))
        msg.handedness_score = float(handedness_score)
        geometry_valid = bool(palm_plane_normal is not None and palm_anchor_cross is not None)
        msg.palm_plane_valid = (geometry_valid)
        if geometry_valid:
            msg.palm_plane_normal = Vector3(x=float(palm_plane_normal[0]
                ), y=float(palm_plane_normal[1]), z=float(palm_plane_normal[2]),)
            msg.palm_anchor_cross = Vector3(x=float(palm_anchor_cross[0]
                ), y=float(palm_anchor_cross[1]), z=float(palm_anchor_cross[2]),)
        ros_points = []
        for point in points:
            ros_point = Point()
            if point is not None:
                ros_point.x = float(point[0])
                ros_point.y = float(point[1])
                ros_point.z = float(point[2])
            ros_points.append(ros_point)
        msg.positions = ros_points
        self.tracking_publisher.publish(msg)

    def publish_wrist(self, wrist, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.target_frame
        msg.pose.position.x = float(wrist[0])
        msg.pose.position.y = float(wrist[1])
        msg.pose.position.z = float(wrist[2])
        msg.pose.orientation.w = 1.0
        self.wrist_publisher.publish(msg)

    def publish_landmarks(self, points, stamp):
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self.target_frame
        for point in points:
            pose = Pose()
            pose.position.x = float(point[0])
            pose.position.y = float(point[1])
            pose.position.z = float(point[2])
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.landmarks_publisher.publish(msg)


    def publish_debug_image(self, image, original_msg):
        if not self.publish_debug:
            return
        if image is None:
            self.debug_image_publisher.publish(original_msg)
            return
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8',)
        msg.header = original_msg.header
        self.debug_image_publisher.publish(msg)

    def destroy_node(self):
        self.hands.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HumanHandTracker()
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
