#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
import yaml

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from franka_msgs.msg import (
    HandState,
    HandTrackingFiltered,
    HandTrackingRaw,
    HandoverDistance,
)
from message_filters import Subscriber, TimeSynchronizer
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

from franka_experiments.utils.camera_yaml import load_camera_info_yaml

class HandCompareVisualizer(Node):

    def __init__(self):
        super().__init__('hand_compare_visualizer')
        config_dir = (
            get_package_share_directory('franka_experiments')
            + '/config/'
        )
        intrinsics = load_camera_info_yaml(
            config_dir + 'camera_intrinsics.yaml'
        )
        if intrinsics is None:
            raise RuntimeError(
                'camera_intrinsics.yaml not valid'
            )
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
        t = extrinsics['translation']
        q = extrinsics['rotation']
        self.t_camera_base = np.array(
            [t['x'], t['y'], t['z']],
            dtype=float,
        )
        self.r_camera_base = Rotation.from_quat(
            [q['x'], q['y'], q['z'], q['w']]
        ).as_matrix()
        self.r_base_camera = self.r_camera_base.T
        self.bridge = CvBridge()
        self.create_subscription(
            CameraInfo,
            (
                '/camera/camera/'
                'aligned_depth_to_color/'
                'camera_info'
            ),
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.publisher = self.create_publisher(
            Image,
            '/handover/hand_debug_image',
            2,
        )
        qos = QoSProfile(
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.image_sub = Subscriber(
            self,
            Image,
            '/handover/hand_debug_image_raw',
            qos_profile=qos,
        )
        # Raw tracking state is cached only for
        # visualization semantics:
        # FULL / ESTIMATED / NO_HAND.
        self._raw_state_by_stamp = {}
        self.raw_state_sub = self.create_subscription(
            HandTrackingRaw,
            '/handover/hand_tracking_raw',
            self._raw_state_callback,
            qos,
        )
        self.filtered_sub = Subscriber(
            self,
            HandTrackingFiltered,
            '/handover/hand_tracking_filtered',
            qos_profile=qos,
        )
        self.state_sub = Subscriber(
            self,
            HandState,
            '/handover/hand_state',
            qos_profile=qos,
        )
        self.distance_sub = Subscriber(
            self,
            HandoverDistance,
            '/handover/distance',
            qos_profile=qos,
        )
        self.sync = TimeSynchronizer(
            [
                self.image_sub,
                self.filtered_sub,
                self.state_sub,
                self.distance_sub,
            ],
            queue_size=3,
        )
        self.sync.registerCallback(self.callback)
        # Visual-only FUTURE W75 forecast.
        #
        # The current dropout bridge is now owned by the
        # real runtime HandState / distance pipeline.
        #
        # This visual state is therefore used only for the
        # +200 ms future ghost shown from a fresh state.
        self._prediction_anchor_stamp_s = None
        self._prediction_anchor_palm = None
        self._prediction_anchor_landmarks = None
        self._prediction_anchor_velocity = None
        self._prediction_horizons_s = (0.20,)
        # DISPLAY-ONLY palm UP/DOWN hysteresis.
        #
        # This NEVER modifies HandState.palm_normal.
        #
        # Since palm_normal is a unit vector, |nz| = 0.15
        # corresponds to about 8.6 degrees from the
        # horizontal plane.
        self._palm_ud_state = None
        # Physical hand used only to reset DISPLAY hysteresis
        # when the active interaction hand changes.
        self._display_hand_side = None
        # UP / SIDE / DOWN display only.
        #
        # Numerical HandState.palm_normal is untouched.
        #
        # Schmitt-style thresholds avoid flicker around
        # the SIDE boundaries.
        # DISPLAY ONLY.
        #
        # Nominal SIDE band ~= +/-20 deg from horizontal.
        #
        # Schmitt hysteresis:
        #   UP/DOWN -> SIDE at 15 deg
        #   SIDE -> UP/DOWN at 25 deg
        #
        # Since palm_normal is unit length:
        #   sin(15 deg) = 0.258819...
        #   sin(25 deg) = 0.422618...
        #
        # This does NOT modify HandState.palm_normal.
        self._palm_side_enter_threshold = 0.2588190451
        self._palm_side_exit_threshold = 0.4226182617
        self.get_logger().info(
            'Hand compare visualizer node started'
        )

    def camera_info_callback(self, msg):
        if msg.k[0] <= 0.0 or msg.k[4] <= 0.0:
            return
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def project(self, point, width, height):
        p_base = np.array(
            [point.x, point.y, point.z],
            dtype=float,
        )
        if not np.all(np.isfinite(p_base)):
            return None
        p_camera = (
            self.r_base_camera
            @ (p_base - self.t_camera_base)
        )
        x, y, z = p_camera
        if z <= 1e-6:
            return None
        u = int(round(self.fx * x / z + self.cx))
        v = int(round(self.fy * y / z + self.cy))
        if (
            u < 0
            or u >= width
            or v < 0
            or v >= height
        ):
            return None
        return u, v

    # W75 VISUAL PREDICTION

    @staticmethod
    def _prediction_stamp_s(stamp):
        return (
            float(stamp.sec)
            + 1e-9 * float(stamp.nanosec)
        )

    @staticmethod
    def _prediction_np_point(point):
        return np.array(
            [
                float(point.x),
                float(point.y),
                float(point.z),
            ],
            dtype=float,
        )

    @staticmethod
    def _prediction_ros_point(array):
        return Point(
            x=float(array[0]),
            y=float(array[1]),
            z=float(array[2]),
        )

    def _prediction_source_good(
        self,
        filtered_msg,
        state_msg,
    ):
        """
        Same conservative semantics used by the offline
        prediction validation.

        Prediction can START only from a fresh/current
        W75 state. It never starts from HOLD/PREDICT_ONLY.
        """
        position_valid = bool(
            getattr(
                state_msg,
                'position_valid',
                state_msg.valid,
            )
        )
        position_fresh = bool(
            getattr(
                state_msg,
                'position_fresh',
                position_valid,
            )
        )
        velocity_valid = bool(
            getattr(
                state_msg,
                'velocity_valid',
                state_msg.valid,
            )
        )
        velocity_age_s = float(
            getattr(
                state_msg,
                'velocity_age_s',
                0.0,
            )
        )
        updated_source = int(
            getattr(
                HandState,
                'VELOCITY_SOURCE_UPDATED',
                1,
            )
        )
        velocity_source = int(
            getattr(
                state_msg,
                'velocity_source',
                updated_source,
            )
        )
        filter_tracking = (
            int(state_msg.filter_state)
            == int(
                HandTrackingFiltered.TRACKING
            )
        )
        landmarks_tracking = all(
            int(landmark_state)
            == int(
                HandTrackingFiltered.TRACKING
            )
            for landmark_state
            in filtered_msg.landmark_state
        )
        palm = self._prediction_np_point(
            state_msg.palm_position
        )
        velocity = np.array(
            [
                float(state_msg.palm_velocity.x),
                float(state_msg.palm_velocity.y),
                float(state_msg.palm_velocity.z),
            ],
            dtype=float,
        )
        landmarks_finite = all(
            np.all(
                np.isfinite(
                    self._prediction_np_point(point)
                )
            )
            for point
            in filtered_msg.positions
        )
        return (
            position_valid
            and position_fresh
            and velocity_valid
            and velocity_source
                == updated_source
            and velocity_age_s
                <= 0.10 + 1e-9
            and filter_tracking
            and landmarks_tracking
            and np.all(np.isfinite(palm))
            and np.all(np.isfinite(velocity))
            and landmarks_finite
        )

    def _update_prediction_anchor(
        self,
        filtered_msg,
        state_msg,
    ):
        self._prediction_anchor_stamp_s = (
            self._prediction_stamp_s(
                state_msg.header.stamp
            )
        )
        self._prediction_anchor_palm = (
            self._prediction_np_point(
                state_msg.palm_position
            )
        )
        self._prediction_anchor_velocity = (
            np.array(
                [
                    float(
                        state_msg.palm_velocity.x
                    ),
                    float(
                        state_msg.palm_velocity.y
                    ),
                    float(
                        state_msg.palm_velocity.z
                    ),
                ],
                dtype=float,
            )
        )
        self._prediction_anchor_landmarks = (
            np.array(
                [
                    self._prediction_np_point(
                        point
                    )
                    for point
                    in filtered_msg.positions
                ],
                dtype=float,
            )
        )

    def _draw_prediction_polygon(
        self,
        image,
        points_xyz,
        color,
        thickness,
    ):
        height, width = image.shape[:2]
        pixels = []
        for xyz in points_xyz:
            pixel = self.project(
                self._prediction_ros_point(
                    xyz
                ),
                width,
                height,
            )
            if pixel is None:
                return None
            pixels.append(pixel)
        polygon = np.array(
            pixels,
            dtype=np.int32,
        ).reshape(
            (-1, 1, 2)
        )
        cv2.polylines(
            image,
            [polygon],
            True,
            color,
            thickness,
            cv2.LINE_AA,
        )
        for pixel in pixels:
            cv2.circle(
                image,
                pixel,
                3,
                color,
                -1,
                cv2.LINE_AA,
            )
        return pixels

    def _draw_prediction_palm(
        self,
        image,
        palm_xyz,
        color,
        label=None,
    ):
        height, width = image.shape[:2]
        pixel = self.project(
            self._prediction_ros_point(
                palm_xyz
            ),
            width,
            height,
        )
        if pixel is None:
            return None
        cv2.drawMarker(
            image,
            pixel,
            color,
            cv2.MARKER_CROSS,
            11,
            2,
            cv2.LINE_AA,
        )
        if label:
            cv2.putText(
                image,
                label,
                (
                    pixel[0] + 7,
                    pixel[1] - 5,
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                color,
                1,
                cv2.LINE_AA,
            )
        return pixel

    def _draw_prediction_overlay(
        self,
        image,
        filtered_msg,
        state_msg,
        distance_msg,
    ):
        """
        Pure visualization.

        Fresh current state:
            current drawing remains unchanged
            + red ghosts at +100 and +200 ms

        Short dropout:
            visual bridge from the last fresh W75 anchor
            for <=100 ms.

        Longer dropout:
            nothing is hidden; prediction disappears.
        """
        now_s = self._prediction_stamp_s(
            state_msg.header.stamp
        )
        # Keep only the last REAL robot control point.
        source_good = (
            self._prediction_source_good(
                filtered_msg,
                state_msg,
            )
        )
        if source_good:
            self._update_prediction_anchor(
                filtered_msg,
                state_msg,
            )
        if (
            self._prediction_anchor_stamp_s
            is None
        ):
            return
        age_s = (
            now_s
            - self._prediction_anchor_stamp_s
        )
        # Timestamp discontinuity / bag restart.
        if age_s < -1e-6:
            self._prediction_anchor_stamp_s = None
            return
        # Fresh state -> future ghosts.
        if source_good:
            # Single validated visual forecast.
            # Only +200 ms is shown to keep the overlay clean.
            # Light red rather than saturated red.
            colors = (
                (80, 80, 255),
            )
            labels = (
                None,
            )
            for (
                horizon_s,
                color,
                label,
            ) in zip(
                self._prediction_horizons_s,
                colors,
                labels,
            ):
                predicted_landmarks = (
                    self._prediction_anchor_landmarks
                    + self._prediction_anchor_velocity[
                        None,
                        :
                    ]
                    * horizon_s
                )
                predicted_palm = (
                    self._prediction_anchor_palm
                    + self._prediction_anchor_velocity
                    * horizon_s
                )
                self._draw_prediction_polygon(
                    image,
                    predicted_landmarks,
                    color,
                    2,
                )
                self._draw_prediction_palm(
                    image,
                    predicted_palm,
                    color,
                    label,
                )
            return
        # No fresh source.
        #
        # The REAL runtime prediction bridge is already
        # contained in HandState and HandoverDistance.
        #
        # Do not draw an additional red/dashed visual-only
        # bridge on top of it.
        return

    def _raw_state_callback(self, msg):
        key = (
            int(msg.header.stamp.sec),
            int(msg.header.stamp.nanosec),
        )
        self._raw_state_by_stamp[key] = int(
            msg.tracking_state
        )
        # Keep only a tiny recent cache.
        while len(self._raw_state_by_stamp) > 64:
            first_key = next(
                iter(self._raw_state_by_stamp)
            )
            self._raw_state_by_stamp.pop(
                first_key,
                None,
            )

    def _raw_state_for_image(self, image_msg):
        key = (
            int(image_msg.header.stamp.sec),
            int(image_msg.header.stamp.nanosec),
        )
        return self._raw_state_by_stamp.get(
            key,
            None,
        )

    def _palm_direction_text(
        self,
        state_msg,
    ):
        """
        DISPLAY ONLY.

        Three states:
            UP
            SIDE
            DOWN

        Uses only normal.z in BASE frame.

        Hysteresis is graphical only and never changes the
        continuous 3-D normal published in HandState.
        """
        normal = np.array(
            [
                float(
                    state_msg.palm_normal.x
                ),
                float(
                    state_msg.palm_normal.y
                ),
                float(
                    state_msg.palm_normal.z
                ),], dtype=float,)
        if not np.isfinite(normal).all():
            return 'palm=--'
        norm = float(np.linalg.norm(normal))
        if norm <= 1e-6:
            return 'palm=--'
        normal = (normal / norm)
        nz = float(normal[2])
        enter = float(self._palm_side_enter_threshold)
        exit_side = float(self._palm_side_exit_threshold)
        state = (self._palm_ud_state)
        # Initial classification.
        if state is None:
            if nz >= exit_side:
                state = 'UP'
            elif nz <= -exit_side:
                state = 'DOWN'
            else:
                state = 'SIDE'
        elif state == 'UP':
            # A very strong direct flip can go directly DOWN.
            if nz <= -exit_side:
                state = 'DOWN'
            elif nz <= enter:
                state = 'SIDE'
        elif state == 'DOWN':
            if nz >= exit_side:
                state = 'UP'
            elif nz >= -enter:
                state = 'SIDE'
        else:
            # SIDE only exits when z is clearly vertical.
            if nz >= exit_side:
                state = 'UP'
            elif nz <= -exit_side:
                state = 'DOWN'
        self._palm_ud_state = (state)
        return (f'palm={state}')

    # VISUAL_TRUST_GATE_V1
    #
    # DISPLAY ONLY.
    #
    # A raw 3-D position is not enough to visually promote a
    # detection to a fully confirmed ACTIVE hand.
    #
    # The detection must also have short-term temporal support
    # through a valid semantic velocity.
    #
    # This suppresses isolated false positives while keeping
    # the runtime pipeline completely unchanged.

    def _visual_hand_trusted(self, filtered_msg, state_msg,):
        position_valid = bool(getattr(state_msg, 'position_valid', False,))
        velocity_valid = bool(getattr(state_msg, 'velocity_valid', False,))
        velocity_age_s = float(getattr(state_msg, 'velocity_age_s', float('inf'),))
        filter_state = int(getattr(state_msg, 'filter_state', -1,))
        tracking_states = (
            int(HandTrackingFiltered.TRACKING), int(HandTrackingFiltered.PREDICT_ONLY),)
        return bool(position_valid and velocity_valid and np.isfinite(velocity_age_s
            ) and velocity_age_s <= 0.10 + 1e-9 and filter_state in tracking_states)

    def callback(self, image_msg, filtered_msg, state_msg, distance_msg,):
        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8',)
        height, width = image.shape[:2]
        # DISPLAY ONLY:
        # distinguish a metric candidate from a temporally
        # confirmed ACTIVE hand.
        visual_trusted = self._visual_hand_trusted(filtered_msg, state_msg,)
        # Filtered hand polygon
        usable = bool(visual_trusted) and all(state in (HandTrackingFiltered.TRACKING,
                HandTrackingFiltered.PREDICT_ONLY,) for state in filtered_msg.landmark_state)
        if usable:
            pixels = [self.project(point, width, height) for point in filtered_msg.positions]
            if all(pixel is not None for pixel in pixels):
                polygon = np.array(pixels, dtype=np.int32,).reshape((-1, 1, 2))
                cv2.polylines(image, [polygon], True, (255, 0, 255), 2, cv2.LINE_8,)
        # Physical-hand display episode
        current_side = int(filtered_msg.handedness)
        valid_sides = (HandTrackingFiltered.HAND_LEFT, HandTrackingFiltered.HAND_RIGHT,)
        if current_side in valid_sides:
            if (self._display_hand_side in valid_sides and current_side != self._display_hand_side):
                # DISPLAY ONLY:
                # do not carry UP/SIDE/DOWN hysteresis from
                # the previous physical hand.
                self._palm_ud_state = None
            self._display_hand_side = (current_side)
        # Palm center
        palm_pixel = None
        if visual_trusted:
            palm_pixel = self.project(state_msg.palm_position, width, height,)
            if palm_pixel is not None:
                cv2.drawMarker(
                    image, palm_pixel, (0, 255, 255), cv2.MARKER_CROSS, 12, 2, cv2.LINE_8,)
        if (visual_trusted and distance_msg.valid):
            ee_pixel = self.project(distance_msg.ee_control_point, width, height,)
            distance_palm_pixel = self.project(distance_msg.palm_position, width, height,)
            if (ee_pixel is not None and distance_palm_pixel is not None):
                cv2.line(image, ee_pixel, distance_palm_pixel, (255, 255, 0), 2, cv2.LINE_AA,)
                cv2.circle(image, ee_pixel, 5, (255, 255, 0), -1,)
        # v + d overlay
        timestamp_s = (
            float(image_msg.header.stamp.sec) + 1e-9 * float(image_msg.header.stamp.nanosec))
        raw_tracking_state = (self._raw_state_for_image(image_msg))
        raw_no_hand_value = int(getattr(HandTrackingRaw, 'NO_HAND', 0,))
        raw_full_value = int(
            getattr(HandTrackingRaw, 'FULL', getattr(HandTrackingRaw, 'TRACKING_FULL', 2,),))
        raw_reports_no_hand = bool(
            raw_tracking_state is not None and raw_tracking_state == raw_no_hand_value)
        runtime_available = bool(visual_trusted)
        # LOST is now a runtime concept, not simply a
        # MediaPipe raw NO_HAND frame.
        visual_no_hand = bool(not runtime_available)
        raw_degraded = bool(raw_tracking_state is not None
            and not raw_reports_no_hand and raw_tracking_state != raw_full_value)
        updated_source = int(getattr(HandState, 'VELOCITY_SOURCE_UPDATED', 1,))
        visual_estimated = bool(runtime_available and (raw_reports_no_hand
                or raw_degraded or (state_msg.position_valid and not state_msg.position_fresh
                ) or (state_msg.position_valid and not bool(state_msg.geometry_ok
                    )) or (int(state_msg.filter_state) != int(HandTrackingFiltered.TRACKING)
                ) or (state_msg.velocity_valid and int(state_msg.velocity_source) != updated_source
                ) or (distance_msg.valid and bool(distance_msg.rate_degraded))))
        self._visual_raw_no_hand = (visual_no_hand)
        self._visual_estimated = (visual_estimated)
        # A complete runtime loss starts a new display episode.
        # Do NOT reset this during a valid predicted bridge.
        if self._visual_raw_no_hand:
            self._palm_ud_state = None
        if not self._visual_raw_no_hand:
            velocity_text = (
                f'v = {state_msg.palm_speed:.2f} m/s' if state_msg.velocity_valid else 'v = --')
            distance_text = (
                f'd = {distance_msg.distance:.2f} m' if distance_msg.valid else 'd = --')
            direction_text = (self._palm_direction_text(state_msg))
            timestamp_text = (f'timestamp = {timestamp_s:.3f}')
            text = (f'{velocity_text}   '
                f'{distance_text}   ' f'{direction_text}   ' f'{timestamp_text}')
            if self._visual_estimated:
                text += '   *'
            # Current / estimated / predicted are all shown
            # with normal tracking semantics.
            text_color = (0, 255, 0,)
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.40
            thickness = 1
            (text_width,
                text_height,), baseline = (cv2.getTextSize(text, font, font_scale, thickness,))
            box_right = min(width - 8, 18 + text_width,)
            cv2.rectangle(image, (8, 8), (box_right, 38), (0, 0, 0), -1,)
            cv2.putText(image, text, (14, 29), font, font_scale, text_color, thickness, cv2.LINE_8,)
        # Visual-only W75 prediction overlay.
        self._draw_prediction_overlay(image, filtered_msg, state_msg, distance_msg,)
        output = self.bridge.cv2_to_imgmsg(image, encoding='bgr8',)
        output.header = image_msg.header
        self.publisher.publish(output)

def main(args=None):
    rclpy.init(args=args)
    node = HandCompareVisualizer()
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
