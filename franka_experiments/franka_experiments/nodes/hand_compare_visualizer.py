#!/usr/bin/env python3

from collections import deque

import cv2
import numpy as np
import rclpy
import yaml

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from franka_msgs.msg import (
    HandObjectState,
    HandState,
    HandTrackingFiltered,
    HandTrackingRaw,
    HandoverDistance,
)
from message_filters import Subscriber, TimeSynchronizer
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

from franka_experiments.nodes.grasp import CROP_M
from franka_experiments.utils.camera_yaml import load_camera_info_yaml

class HandCompareVisualizer(Node):

    def __init__(self):
        super().__init__('hand_compare_visualizer')
        self.declare_parameter('standoff_m', 0.20)
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
        self.base_frame = extrinsics['parent_frame']
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

        # Recent object states: the debug image can lag behind them (RViz load),
        # so the overlay uses the one closest in time to the image.
        self._object_states = deque(maxlen=60)
        self.create_subscription(
            HandObjectState, '/handover/hand_object', self._object_states.append, 10)
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
        # Gripper camera (D405) view: same overlay on its image, which the tracker
        # publishes only while the camera is present and placed with the robot TF.
        with open(config_dir + 'd405_extrinsics.yaml', 'r', encoding='utf-8') as file:
            gripper = yaml.safe_load(file)
        q, t = gripper['rotation'], gripper['translation']
        self.gripper_link = gripper['parent_frame']
        self.r_link_gripper = Rotation.from_quat([q['x'], q['y'], q['z'], q['w']]).as_matrix()
        self.t_link_gripper = np.array([t['x'], t['y'], t['z']], dtype=float)
        self.gripper_k, self._latest = None, None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            CameraInfo, '/d405/d405/aligned_depth_to_color/camera_info',
            self.gripper_info_callback, qos_profile_sensor_data)
        self.create_subscription(Image, '/handover/gripper_debug_image_raw', self.gripper_callback, qos)
        self.gripper_publisher = self.create_publisher(Image, '/handover/gripper_debug_image', 2)
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

    def _draw_object_overlay(self, image, image_msg, state_msg, pixels=True):
        t_image = self._prediction_stamp_s(image_msg.header.stamp)
        msg = min(self._object_states, default=None,
                  key=lambda m: abs(t_image - self._prediction_stamp_s(m.header.stamp)))
        color = (160, 160, 160)
        text = 'Oggetto: dati assenti/scaduti'
        if msg is not None:
            age = t_image - self._prediction_stamp_s(msg.header.stamp)
            current = (abs(age) <= .2 and msg.physical_hand == state_msg.physical_hand
                       and msg.header.frame_id == state_msg.header.frame_id)
            if current:
                text = 'Oggetto: non osservabile'
                if msg.valid:
                    text, color = 'Oggetto: non confermato', (0, 210, 255)
                    if msg.object_present:
                        d = msg.dimensions
                        text = (f'Oggetto: CONFERMATO  c={msg.object_confidence:.2f}  eta={msg.object_age:.2f}s'
                                f'  dim={100*d.x:.0f}x{100*d.y:.0f}x{100*d.z:.0f} cm')
                        color = (60, 255, 60)
                    if msg.object_present and pixels:  # contour / box are main-camera pixels
                        contour = np.array(msg.contour_px, np.int32).reshape(-1, 1, 2)
                        if len(contour) >= 3:
                            fill = image.copy()
                            cv2.fillPoly(fill, [contour], color)
                            cv2.addWeighted(fill, .35, image, .65, 0, image)
                            cv2.polylines(image, [contour], True, color, 2, cv2.LINE_AA)
                        u0, v0, u1, v1 = map(int, msg.bbox_px)
                        cv2.rectangle(image, (u0, v0), (u1, v1), color, 1, cv2.LINE_AA)
                        cv2.putText(image, f'OGGETTO {100*d.x:.0f}x{100*d.y:.0f} cm', (u0, max(12, v0 - 6)),
                                    cv2.FONT_HERSHEY_SIMPLEX, .4, color, 1, cv2.LINE_AA)
        width = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, .4, 1)[0][0]
        cv2.rectangle(image, (8, 62), (min(image.shape[1]-8, width+20), 84), (0, 0, 0), -1)
        cv2.putText(image, text, (14, 78), cv2.FONT_HERSHEY_SIMPLEX, .4, color, 1, cv2.LINE_AA)

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
        Prediction is drawn from the current HandState when the robot
        would use it: position and velocity valid, velocity fresh.
        """
        position_valid = bool(
            getattr(
                state_msg,
                'position_valid',
                state_msg.valid,
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
        # Same validity the robot uses (HandoverDistance: position_valid and
        # velocity_valid). Requiring every landmark in TRACKING hid the
        # forecast whenever the Kalman gate skipped one landmark for a frame
        # (~80 times/min); the filtered landmarks are still predicted then.
        landmarks_initialized = all(
            int(landmark_state)
            in (
                int(HandTrackingFiltered.TRACKING),
                int(HandTrackingFiltered.PREDICT_ONLY),
            )
            for landmark_state
            in filtered_msg.landmark_state
        )
        return (
            position_valid
            and velocity_valid
            and velocity_age_s
                <= 0.10 + 1e-9
            and landmarks_initialized
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
        self._latest = (filtered_msg, state_msg, distance_msg)
        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8',)
        self._draw(image, image_msg, filtered_msg, state_msg, distance_msg)
        output = self.bridge.cv2_to_imgmsg(image, encoding='bgr8',)
        output.header = image_msg.header
        self.publisher.publish(output)

    def gripper_info_callback(self, msg):
        if msg.k[0] > 0.0:
            self.gripper_k = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]))

    def gripper_callback(self, image_msg):
        """Overlay of the latest HandState on the D405 image, projected with the
        D405 pose (TF base -> fr3_link8 + d405_extrinsics.yaml) and intrinsics."""
        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8',)
        latest, k = self._latest, self.gripper_k
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.gripper_link, Time())
        except Exception:
            tf = None
        if (latest is not None and k is not None and tf is not None and abs(
                self._prediction_stamp_s(image_msg.header.stamp)
                - self._prediction_stamp_s(latest[1].header.stamp)) <= 0.2):
            q, t = tf.transform.rotation, tf.transform.translation
            r_link = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            camera = (self.fx, self.fy, self.cx, self.cy, self.r_base_camera, self.t_camera_base)
            display = {key: value for key, value in vars(self).items() if key.startswith(
                ('_palm_ud', '_display_hand', '_prediction_anchor', '_visual_'))}
            self.fx, self.fy, self.cx, self.cy = k
            self.r_base_camera = (r_link @ self.r_link_gripper).T
            self.t_camera_base = r_link @ self.t_link_gripper + np.array([t.x, t.y, t.z])
            try:
                self._draw(image, image_msg, *latest, main=False)
            finally:  # main-camera projection and display state untouched
                self.fx, self.fy, self.cx, self.cy, self.r_base_camera, self.t_camera_base = camera
                vars(self).update(display)
        output = self.bridge.cv2_to_imgmsg(image, encoding='bgr8',)
        output.header = image_msg.header
        self.gripper_publisher.publish(output)

    def _draw(self, image, image_msg, filtered_msg, state_msg, distance_msg, main=True):
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


        # Hands23 crop of grasp.py: +-CROP_M around the active palm.
        if state_msg.position_valid:
            palm = np.array([state_msg.palm_position.x, state_msg.palm_position.y,
                             state_msg.palm_position.z])
            z = (self.r_base_camera @ (palm - self.t_camera_base))[2]
            centre = self.project(state_msg.palm_position, width, height)
            if centre is not None and z > 0.1:
                r = int(CROP_M * self.fx / z)
                x0, y0 = max(0, centre[0] - r), max(0, centre[1] - r)
                cv2.rectangle(image, (x0, y0), (min(width - 1, centre[0] + r),
                              min(height - 1, centre[1] + r)), (0, 200, 255), 1, cv2.LINE_AA)
                cv2.putText(image, 'ROI oggetto (Hands23)', (x0 + 4, max(14, y0 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 200, 255), 1, cv2.LINE_AA)
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
        if state_msg.position_valid:
            palm_pixel = self.project(state_msg.palm_position, width, height,)
            if palm_pixel is not None:
                cv2.drawMarker(
                    image, palm_pixel, (0, 255, 255), cv2.MARKER_CROSS, 12, 2, cv2.LINE_8,)
        if (state_msg.position_valid and distance_msg.valid):
            ee_pixel = self.project(distance_msg.ee_control_point, width, height,)
            distance_palm_pixel = self.project(distance_msg.palm_position, width, height,)
            if (ee_pixel is not None and distance_palm_pixel is not None):
                cv2.line(image, ee_pixel, distance_palm_pixel, (255, 255, 0), 2, cv2.LINE_AA,)
                cv2.circle(image, ee_pixel, 5, (255, 255, 0), -1,)
            # Geometric standoff, before the commander's reference smoothing.
            palm = self._prediction_np_point(distance_msg.palm_position)
            ee = self._prediction_np_point(distance_msg.ee_control_point)
            delta = ee - palm
            standoff = float(self.get_parameter('standoff_m').value)
            if np.isfinite(palm).all() and np.isfinite(standoff) and standoff >= 0.0:
                target = palm.copy()
                target[2] += standoff
                target_pixel = self.project(self._prediction_ros_point(target), width, height)
                if target_pixel is not None:
                    color = (0, 140, 255)
                    if distance_palm_pixel is not None:
                        cv2.line(image, distance_palm_pixel, target_pixel, color, 2, cv2.LINE_AA)
                    cv2.drawMarker(image, target_pixel, color, cv2.MARKER_DIAMOND, 18, 2, cv2.LINE_AA)
                    cv2.putText(image, 'STANDOFF +Z',
                        (max(0, min(target_pixel[0] + 12, width - 125)),
                         max(16, target_pixel[1] - 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
                cv2.putText(image,
                    f'Standoff +Z: {standoff * 100:.0f} cm | EE-palmo Z: {delta[2] * 100:.1f} cm',
                    (14, height - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 140, 255), 1, cv2.LINE_AA)
        # v + d overlay
        timestamp_s = (
            float(image_msg.header.stamp.sec) + 1e-9 * float(image_msg.header.stamp.nanosec))
        raw_tracking_state = (self._raw_state_for_image(state_msg))  # same stamp as the main image
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
        # Position remains useful while the velocity estimator is warming up.
        coordinates = 'Palmo [base] m: X=--  Y=--  Z=--'
        if distance_msg.valid:
            palm_base = distance_msg.palm_position
        elif state_msg.position_valid and state_msg.header.frame_id in ('base', 'fr3_link0'):
            palm_base = state_msg.palm_position
        else:
            palm_base = None
        if palm_base is not None and np.isfinite(self._prediction_np_point(palm_base)).all():
            coordinates = (f'Palmo [base] m: X={palm_base.x:+.3f}  '
                           f'Y={palm_base.y:+.3f}  Z={palm_base.z:+.3f}')
        text_width = cv2.getTextSize(coordinates, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)[0][0]
        cv2.rectangle(image, (8, 39), (min(width - 8, 18 + text_width), 60), (0, 0, 0), -1)
        cv2.putText(image, coordinates, (14, 54), cv2.FONT_HERSHEY_SIMPLEX,
                    0.38, (0, 255, 255), 1, cv2.LINE_AA)
        # Visual-only W75 prediction overlay.
        self._draw_prediction_overlay(image, filtered_msg, state_msg, distance_msg,)
        self._draw_object_overlay(image, image_msg, state_msg, pixels=main)



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
