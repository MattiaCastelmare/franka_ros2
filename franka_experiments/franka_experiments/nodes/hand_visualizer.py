#!/usr/bin/env python3
"""Debug images: pipeline state drawn on the main camera image and on the gripper camera (D405).

  hand        filtered landmarks (magenta, only when the hand is trusted: position + fresh velocity),
              palm (yellow cross), Hands23 crop around the palm (orange box)
  robot       gripper tip - palm line (cyan) and the commander target, palm + standoff_m in z (orange)
  status      speed, distance, palm UP / SIDE / DOWN (hysteresis on normal.z), timestamp, '*' = estimated;
              palm coordinates in the base frame
  prediction  hand + velocity x 0.2 s (red), when the velocity is fresh
  object      Hands23 object (HandObjectState) and the grasp pose (handover_grasp.py pose), if published
The D405 view reuses the same drawing with the D405 pose (TF base -> fr3_link8 + d405_extrinsics.yaml).
"""

from collections import deque

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from franka_msgs.msg import (
    HandObjectState, HandoverDistance, HandState, HandTrackingFiltered, HandTrackingRaw)
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber, TimeSynchronizer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from franka_experiments.nodes.grasp import CROP_M
from franka_experiments.utils.camera_yaml import load_camera_info_yaml

F = HandTrackingFiltered
# |normal.z| = sin 15 deg / sin 25 deg (UP/SIDE/DOWN hysteresis)
SIDE_ENTER, SIDE_EXIT = 0.2588190451, 0.4226182617
PREDICTION_S = 0.20


def stamp_s(stamp):
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def xyz(p):
    return (np.array([float(p.x), float(p.y), float(p.z)], dtype=float) if hasattr(p, 'x')
            else np.asarray(p, dtype=float))


def rotation_translation(q, t):
    return (Rotation.from_quat([q['x'], q['y'], q['z'], q['w']]).as_matrix(),
            np.array([t['x'], t['y'], t['z']], dtype=float))


def text_box(image, text, origin, box, scale, color, line=cv2.LINE_AA):
    """Text on a black box from (box[0], box[1]) to the text width + 10 px, at most the image width - 8."""
    width = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)[0][0]
    cv2.rectangle(image, (box[0], box[1]), (min(image.shape[1] - 8, origin[0] + 4 + width), box[2]),
                  (0, 0, 0), -1)
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, line)


class HandVisualizer(Node):

    def __init__(self):
        super().__init__('hand_visualizer')
        self.declare_parameter('standoff_m', 0.20)
        self.declare_parameter('grasp_width_m', 0.06)

        # main camera: intrinsics and extrinsics from config
        config_dir = get_package_share_directory('franka_experiments') + '/config/'
        intrinsics = load_camera_info_yaml(config_dir + 'camera_intrinsics.yaml')
        if intrinsics is None:
            raise RuntimeError('camera_intrinsics.yaml not valid')
        k = intrinsics['k']

        with open(config_dir + 'camera_extrinsics.yaml', 'r', encoding='utf-8') as file:
            extrinsics = yaml.safe_load(file)
        self.base_frame = extrinsics['parent_frame']
        r_camera_base, t_camera_base = rotation_translation(extrinsics['rotation'], extrinsics['translation'])
        # camera used by project(): fx, fy, cx, cy, R base -> camera, camera position in base
        self.camera = [float(k[0]), float(k[4]), float(k[2]), float(k[5]), r_camera_base.T, t_camera_base]

        # gripper camera (D405) mounted on the flange
        with open(config_dir + 'd405_extrinsics.yaml', 'r', encoding='utf-8') as file:
            gripper = yaml.safe_load(file)
        self.gripper_link = gripper['parent_frame']
        self.r_link_gripper, self.t_link_gripper = rotation_translation(
            gripper['rotation'], gripper['translation'])

        self.gripper_k, self._latest = None, None
        self.palm_ud, self.display_side = None, None  # UP/SIDE/DOWN display state, reset with the physical hand
        self._raw_state_by_stamp = {}                 # raw tracking state of recent frames (the '*' marker)
        self._object_states, self._grasp_poses = deque(maxlen=60), deque(maxlen=30)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Image, '/handover/hand_debug_image', 2)
        self.gripper_publisher = self.create_publisher(Image, '/handover/gripper_debug_image', 2)
        self.create_subscription(HandObjectState, '/handover/hand_object', self._object_states.append, 10)
        self.create_subscription(PoseStamped, '/handover/grasp_pose', self._grasp_poses.append, 10)
        self.create_subscription(CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info',
                                 self.camera_info_callback, qos_profile_sensor_data)
        self.create_subscription(HandTrackingRaw, '/handover/hand_tracking_raw', self._raw_state_callback, qos)
        self.create_subscription(CameraInfo, '/d405/d405/aligned_depth_to_color/camera_info',
                                 self.gripper_info_callback, qos_profile_sensor_data)
        self.create_subscription(Image, '/handover/gripper_debug_image_raw', self.gripper_callback, qos)
        self.sync = TimeSynchronizer(
            [Subscriber(self, Image, '/handover/hand_debug_image_raw', qos_profile=qos),
             Subscriber(self, HandTrackingFiltered, '/handover/hand_tracking_filtered', qos_profile=qos),
             Subscriber(self, HandState, '/handover/hand_state', qos_profile=qos),
             Subscriber(self, HandoverDistance, '/handover/distance', qos_profile=qos)], queue_size=3)
        self.sync.registerCallback(self.callback)

        self.get_logger().info('Hand visualizer node started')

    # ------------------------------------------------------------ inputs
    def camera_info_callback(self, msg):
        if msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.camera[:4] = [float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5])]

    def gripper_info_callback(self, msg):
        if msg.k[0] > 0.0:
            self.gripper_k = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]))

    def _raw_state_callback(self, msg):
        self._raw_state_by_stamp[(int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))] = \
            int(msg.tracking_state)
        while len(self._raw_state_by_stamp) > 64:
            self._raw_state_by_stamp.pop(next(iter(self._raw_state_by_stamp)), None)

    def project(self, point, width, height):
        """Base-frame point -> pixel of the current camera, None if behind it or outside the image."""
        p = xyz(point)
        if not np.all(np.isfinite(p)):
            return None

        fx, fy, cx, cy, r_base_camera, t_camera_base = self.camera
        x, y, z = r_base_camera @ (p - t_camera_base)
        if z <= 1e-6:
            return None
        u, v = int(round(fx * x / z + cx)), int(round(fy * y / z + cy))
        return (u, v) if 0 <= u < width and 0 <= v < height else None

    # ------------------------------------------------------------ images
    def callback(self, image_msg, filtered_msg, state_msg, distance_msg):
        self._latest = (filtered_msg, state_msg, distance_msg)
        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        self._draw(image, image_msg, filtered_msg, state_msg, distance_msg)
        output = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        output.header = image_msg.header
        self.publisher.publish(output)

    def gripper_callback(self, image_msg):
        """The latest state drawn on the D405 image, with the D405 pose and intrinsics; the main-camera
        model and the display state are left untouched."""
        image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        latest, k = self._latest, self.gripper_k
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.gripper_link, Time())
        except Exception:
            tf = None

        if (latest is not None and k is not None and tf is not None
                and abs(stamp_s(image_msg.header.stamp) - stamp_s(latest[1].header.stamp)) <= 0.2):
            q, t = tf.transform.rotation, tf.transform.translation
            r_link = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            main, display = self.camera, (self.palm_ud, self.display_side)
            self.camera = [*k, (r_link @ self.r_link_gripper).T,
                           r_link @ self.t_link_gripper + np.array([t.x, t.y, t.z])]
            try:
                self._draw(image, image_msg, *latest, main=False)
            finally:
                self.camera, (self.palm_ud, self.display_side) = main, display

        output = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        output.header = image_msg.header
        self.gripper_publisher.publish(output)

    # ------------------------------------------------------------ drawing
    @staticmethod
    def _hand_trusted(state):
        """Position and a fresh velocity: an isolated detection is not drawn as the ACTIVE hand."""
        return bool(state.position_valid and state.velocity_valid
                    and np.isfinite(state.velocity_age_s)
                    and state.velocity_age_s <= 0.10 + 1e-9
                    and int(state.filter_state) in (int(F.TRACKING), int(F.PREDICT_ONLY)))

    def _palm_direction(self, state):
        """UP / SIDE / DOWN from normal.z (base frame), Schmitt hysteresis: display only."""
        normal = xyz(state.palm_normal)
        if not np.isfinite(normal).all() or (norm := float(np.linalg.norm(normal))) <= 1e-6:
            return 'palm=--'

        nz = float(normal[2] / norm)
        ud = self.palm_ud
        if ud is None:
            ud = 'UP' if nz >= SIDE_EXIT else 'DOWN' if nz <= -SIDE_EXIT else 'SIDE'
        elif ud == 'UP':
            ud = 'DOWN' if nz <= -SIDE_EXIT else 'SIDE' if nz <= SIDE_ENTER else ud
        elif ud == 'DOWN':
            ud = 'UP' if nz >= SIDE_EXIT else 'SIDE' if nz >= -SIDE_ENTER else ud
        else:
            ud = 'UP' if nz >= SIDE_EXIT else 'DOWN' if nz <= -SIDE_EXIT else ud
        self.palm_ud = ud
        return f'palm={ud}'

    def _draw_prediction(self, image, filtered, state):
        """Hand + velocity x PREDICTION_S when the robot would use it: position and fresh velocity valid,
        every landmark initialised, all finite."""
        palm, velocity = xyz(state.palm_position), xyz(state.palm_velocity)
        landmarks = np.array([xyz(p) for p in filtered.positions], dtype=float)
        if not (state.position_valid and state.velocity_valid
                and float(state.velocity_age_s) <= 0.10 + 1e-9
                and all(int(s) in (int(F.TRACKING), int(F.PREDICT_ONLY)) for s in filtered.landmark_state)
                and np.all(np.isfinite(palm))
                and np.all(np.isfinite(velocity))
                and np.all(np.isfinite(landmarks))):
            return

        h, w = image.shape[:2]
        color = (80, 80, 255)
        pixels = [self.project(p, w, h) for p in landmarks + velocity[None, :] * PREDICTION_S]
        if all(p is not None for p in pixels):
            cv2.polylines(image, [np.array(pixels, dtype=np.int32).reshape((-1, 1, 2))], True, color, 2,
                          cv2.LINE_AA)
            for p in pixels:
                cv2.circle(image, p, 3, color, -1, cv2.LINE_AA)

        if (p := self.project(palm + velocity * PREDICTION_S, w, h)) is not None:
            cv2.drawMarker(image, p, color, cv2.MARKER_CROSS, 11, 2, cv2.LINE_AA)

    def _draw_object_overlay(self, image, image_msg, state_msg, pixels=True):
        t_image = stamp_s(image_msg.header.stamp)
        msg = min(self._object_states, default=None,
                  key=lambda m: abs(t_image - stamp_s(m.header.stamp)))
        color = (160, 160, 160)
        text = 'Oggetto: dati assenti/scaduti'

        if msg is not None:
            age = t_image - stamp_s(msg.header.stamp)
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

    # Franka hand in the TCP frame [m]: fingers 5.8 cm, hand body behind them.
    _FINGER_M = 0.058
    _BODY_M = 0.035
    _APPROACH_M = 0.12

    def _draw_grasp_overlay(self, image, image_msg):
        """Grasp pose as a translucent Franka gripper with its approach path."""
        t_image = stamp_s(image_msg.header.stamp)
        msg = min(self._grasp_poses, default=None,
                  key=lambda m: abs(t_image - stamp_s(m.header.stamp)))
        if msg is None or abs(t_image - stamp_s(msg.header.stamp)) > 0.3:
            return

        q, o = msg.pose.orientation, msg.pose.position
        R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        origin = np.array([o.x, o.y, o.z])
        if not np.isfinite(origin).all():
            return

        h, w = image.shape[:2]
        half = 0.5 * float(self.get_parameter('grasp_width_m').value)
        f, b = self._FINGER_M, self._BODY_M

        def px(y, z):
            p = origin + R @ np.array([0.0, y, z])
            return self.project(p, w, h)

        # polygons in the gripper (y, z) plane: two fingers, the hand body behind them, the approach axis
        fingers = [[px(s * half, 0.0), px(s * (half + 0.012), 0.0), px(s * (half + 0.012), -f),
                    px(s * half, -f)] for s in (-1, 1)]
        body = [px(-half - 0.03, -f), px(half + 0.03, -f), px(half + 0.03, -f - b), px(-half - 0.03, -f - b)]
        approach = [px(0.0, -f - b), px(0.0, -f - b - self._APPROACH_M)]
        if any(p is None for p in sum(fingers, []) + body + approach):
            return

        color, dark = (255, 190, 40), (60, 30, 0)
        shapes = [np.array(s, np.int32) for s in fingers + [body]]
        layer = image.copy()
        for poly in shapes:
            cv2.fillPoly(layer, [poly], color, cv2.LINE_AA)
        cv2.addWeighted(layer, 0.45, image, 0.55, 0, image)
        for poly in shapes:  # dark halo, then the outline: readable on any background
            cv2.polylines(image, [poly], True, dark, 4, cv2.LINE_AA)
            cv2.polylines(image, [poly], True, color, 2, cv2.LINE_AA)

        a, e = np.array(approach[1], float), np.array(approach[0], float)
        for k in range(0, 10, 2):  # dashed approach path into the hand body
            p0, p1 = a + (e - a) * k / 10, a + (e - a) * (k + 1) / 10
            cv2.line(image, tuple(map(int, p0)), tuple(map(int, p1)), color, 2, cv2.LINE_AA)

        for tip in (fingers[0][0], fingers[1][0]):
            cv2.circle(image, tip, 4, dark, -1, cv2.LINE_AA)
            cv2.circle(image, tip, 3, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.putText(image, 'PRESA', (approach[1][0] + 8, approach[1][1] + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, dark, 3, cv2.LINE_AA)
        cv2.putText(image, 'PRESA', (approach[1][0] + 8, approach[1][1] + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

    def _draw(self, image, image_msg, filtered, state, distance, main=True):
        h, w = image.shape[:2]
        trusted = self._hand_trusted(state)
        if trusted and all(s in (F.TRACKING, F.PREDICT_ONLY) for s in filtered.landmark_state):
            pixels = [self.project(p, w, h) for p in filtered.positions]
            if all(p is not None for p in pixels):
                cv2.polylines(image, [np.array(pixels, dtype=np.int32).reshape((-1, 1, 2))], True,
                              (255, 0, 255), 2, cv2.LINE_8)

        fx, r_base_camera, t_camera_base = self.camera[0], self.camera[4], self.camera[5]
        if state.position_valid:  # Hands23 crop of grasp.py: +-CROP_M around the palm
            z = (r_base_camera @ (xyz(state.palm_position) - t_camera_base))[2]
            centre = self.project(state.palm_position, w, h)
            if centre is not None and z > 0.1:
                r = int(CROP_M * fx / z)
                x0, y0 = max(0, centre[0] - r), max(0, centre[1] - r)
                cv2.rectangle(image, (x0, y0), (min(w - 1, centre[0] + r), min(h - 1, centre[1] + r)),
                              (0, 200, 255), 1, cv2.LINE_AA)
                cv2.putText(image, 'ROI oggetto (Hands23)', (x0 + 4, max(14, y0 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 200, 255), 1, cv2.LINE_AA)

        side = int(filtered.handedness)
        if side in (F.HAND_LEFT, F.HAND_RIGHT):
            if self.display_side in (F.HAND_LEFT, F.HAND_RIGHT) and side != self.display_side:
                self.palm_ud = None  # UP/SIDE/DOWN hysteresis does not carry over to the other hand
            self.display_side = side

        if state.position_valid and (palm_px := self.project(state.palm_position, w, h)) is not None:
            cv2.drawMarker(image, palm_px, (0, 255, 255), cv2.MARKER_CROSS, 12, 2, cv2.LINE_8)

        # gripper tip - palm, and the commander target (palm + standoff in z, before its smoothing)
        if state.position_valid and distance.valid:
            ee_px = self.project(distance.ee_control_point, w, h)
            palm_px = self.project(distance.palm_position, w, h)
            if ee_px is not None and palm_px is not None:
                cv2.line(image, ee_px, palm_px, (255, 255, 0), 2, cv2.LINE_AA)
                cv2.circle(image, ee_px, 5, (255, 255, 0), -1)

            palm, ee = xyz(distance.palm_position), xyz(distance.ee_control_point)
            standoff = float(self.get_parameter('standoff_m').value)
            if np.isfinite(palm).all() and np.isfinite(standoff) and standoff >= 0.0:
                color = (0, 140, 255)
                if (target_px := self.project(palm + np.array([0.0, 0.0, standoff]), w, h)) is not None:
                    if palm_px is not None:
                        cv2.line(image, palm_px, target_px, color, 2, cv2.LINE_AA)
                    cv2.drawMarker(image, target_px, color, cv2.MARKER_DIAMOND, 18, 2, cv2.LINE_AA)
                    cv2.putText(image, 'STANDOFF +Z',
                                (max(0, min(target_px[0] + 12, w - 125)), max(16, target_px[1] - 12)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
                cv2.putText(image,
                            f'Standoff +Z: {standoff * 100:.0f} cm | EE-palmo Z: {(ee - palm)[2] * 100:.1f} cm',
                            (14, h - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

        # status line: '*' = trusted but estimated (raw not full, degraded / held / predicted state)
        raw = self._raw_state_by_stamp.get((int(state.header.stamp.sec), int(state.header.stamp.nanosec)))
        raw_no_hand = raw is not None and raw == int(HandTrackingRaw.NO_HAND)
        raw_degraded = raw is not None and not raw_no_hand and raw != int(HandTrackingRaw.TRACKING_FULL)
        estimated = trusted and (
            raw_no_hand or raw_degraded
            or (state.position_valid and not state.position_fresh)
            or (state.position_valid and not bool(state.geometry_ok))
            or int(state.filter_state) != int(F.TRACKING)
            or (state.velocity_valid and int(state.velocity_source) != int(HandState.VELOCITY_SOURCE_UPDATED))
            or (distance.valid and bool(distance.rate_degraded)))

        if not trusted:
            self.palm_ud = None  # a complete loss starts a new display episode
        else:
            text = (f'{f"v = {state.palm_speed:.2f} m/s" if state.velocity_valid else "v = --"}   '
                    f'{f"d = {distance.distance:.2f} m" if distance.valid else "d = --"}   '
                    f'{self._palm_direction(state)}   timestamp = {stamp_s(image_msg.header.stamp):.3f}')
            text_box(image, text + ('   *' if estimated else ''), (14, 29), (8, 8, 38), 0.40, (0, 255, 0),
                     cv2.LINE_8)

        palm_base = (distance.palm_position if distance.valid
                     else state.palm_position
                     if state.position_valid and state.header.frame_id in ('base', 'fr3_link0') else None)
        coordinates = ('Palmo [base] m: X=--  Y=--  Z=--'
                       if palm_base is None or not np.isfinite(xyz(palm_base)).all()
                       else f'Palmo [base] m: X={palm_base.x:+.3f}  Y={palm_base.y:+.3f}  Z={palm_base.z:+.3f}')
        text_box(image, coordinates, (14, 54), (8, 39, 60), 0.38, (0, 255, 255))

        self._draw_prediction(image, filtered, state)
        self._draw_object_overlay(image, image_msg, state, pixels=main)
        if main:  # grasp drawn with the main-camera intrinsics
            self._draw_grasp_overlay(image, image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = HandVisualizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):  # Ctrl+C / launch shutdown
        pass
    except Exception:
        if rclpy.ok():  # otherwise a shutdown race (context already gone)
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
