#!/usr/bin/env python3
"""TrajectoryOverlayNode — the two EE traces drawn on the camera image.

Opens the scene camera's RGB stream, the one that looks at the robot, and draws
on it:

* **red**  — ``P(s)``, the Cartesian point ``pentagon_qddot_commander`` is asking
  for at this instant, with a short fading trail behind it;
* **blue** — where the end effector actually is, same trail;
* an amber tie between the two heads, and the distance in millimetres, whenever
  the gap is worth naming.

Both trails die of old age (``trail_seconds``), so what you watch is two short
comets chasing each other over the live picture of the arm — not two curves that
grow until they cover the robot they are supposed to be measured against.

WHY THE POINTS COME FROM THE COMMANDER
--------------------------------------
Both topics are published by the commander, from the same tick and the same
forward-kinematics evaluation, in ``fr3_link0``. Deriving either one here — the
actual from TF, the desired from ``q_des_state`` — would have been less code and
a worse answer; ``nodes/trajectory_visualization_node`` documents why at length,
and the reasons are unchanged by drawing on pixels instead of in RViz.

WHAT ELSE THE OVERLAY CHECKS, FOR FREE
--------------------------------------
Projection uses ``camera_extrinsics.yaml``, the SAME calibration
``real_time_distance`` subtracts the robot mask with. So a blue dot that does
not sit on the gripper in the image is not a cosmetic complaint: it is the
safety pipeline's calibration drifting, seen before it shows up as the arm
reading its own links as an obstacle.

Subscriptions
-------------
  <image_topic>        Image       — colour frames from the scene camera
  <camera_info_topic>  CameraInfo  — its K and distortion
  <ee_desired_topic>   PointStamped — P(s), the commanded Cartesian point
  <ee_actual_topic>    PointStamped — the measured EE position

Published
---------
  <output_topic>       Image (bgr8) — the annotated frame

``show_window`` also opens an OpenCV window, which is the point of this node:
the overlay is meant to be watched while the arm runs, without RViz and without
a second tool to start. It is a GUI process like any other, so clear it for runs
whose timing you intend to trust — the published topic stays either way.
"""

from __future__ import annotations

import os
import threading
from typing import Optional, Tuple

import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from franka_experiments.utils.cbf_utils import load_robot_config
from franka_experiments.utils.config import load_extrinsics
from franka_experiments.utils.node_runtime import run_node_main
from franka_experiments.utils.params import (
    declare_bool,
    declare_float,
    declare_int,
    declare_str,
)
from franka_experiments.utils.trajectory_overlay import (
    PixelBounds,
    composite,
    draw_fading_polyline,
    project_base_to_pixels,
    trail_alphas,
)
from franka_experiments.utils.trajectory_trace import TraceBuffer

# One thread. This node runs beside a 1 kHz RT loop on a box whose cores 2-3 are
# isolated for it; OpenCV's default pool would happily spin on them.
cv2.setNumThreads(1)

# BGR — the frame is converted to bgr8 on arrival so that cv2.imshow, the
# drawing calls and the published encoding all agree without a swap anywhere.
_RED = (40, 40, 235)          # desired P(s)
_BLUE = (235, 120, 30)        # measured EE
_AMBER = (0, 190, 255)        # the tie between the two heads
_BLACK = (0, 0, 0)
_WHITE = (245, 245, 245)


def _text(img, lines, origin=(10, 24), scale=0.6) -> None:
    """Text on a dark plate: the lab bench reads white under this camera."""
    x, y = origin
    for line, color in lines:
        (w, h), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, scale, 2)
        cv2.rectangle(img, (x - 4, y - h - 6), (x + w + 4, y + 6), _BLACK, -1)
        cv2.putText(img, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale,
                    color, 2, cv2.LINE_AA)
        y += h + 12


class TrajectoryOverlayNode(Node):
    """Accumulate the two EE traces and draw them on the camera frames."""

    def __init__(self):
        super().__init__('trajectory_overlay')
        self.done = False

        cfg = load_robot_config('control')
        topics = cfg['topics']

        des_topic = declare_str(
            self, 'ee_desired_topic', topics.get('ee_desired', '/NS_1/ee_desired'))
        act_topic = declare_str(
            self, 'ee_actual_topic', topics.get('ee_actual', '/NS_1/ee_actual'))
        img_topic = declare_str(
            self, 'image_topic',
            topics.get('color_image', '/camera/camera/color/image_raw'))
        info_topic = declare_str(
            self, 'camera_info_topic',
            topics.get('color_camera_info', '/camera/camera/color/camera_info'))
        out_topic = declare_str(
            self, 'output_topic',
            topics.get('trajectory_overlay', '/NS_1/trajectory_overlay'))
        ext_path = declare_str(self, 'camera_extrinsics_path', '',
                               allow_empty=True)

        # ── Trail shape ──────────────────────────────────────────────────
        self._ttl = declare_float(self, 'trail_seconds', 2.0,
                                  minimum=0.0, maximum=120.0)
        self._hold = declare_float(self, 'trail_head_hold', 0.40,
                                   minimum=0.0, maximum=0.99)
        self._levels = declare_int(self, 'fade_levels', 8, minimum=1, maximum=64)
        # Sized to be read at a glance on the live picture rather than
        # measured: at 640x480 a 2 px trail and a 6 px head disappear against a
        # busy bench. Both are parameters, so a run that wants the thinnest
        # honest line back can ask for it without touching this file.
        self._w_line = declare_int(self, 'line_width_px', 4,
                                   minimum=1, maximum=20)
        self._r_tip = declare_int(self, 'tip_radius_px', 11,
                                  minimum=1, maximum=50)
        self._tie = declare_bool(self, 'draw_deviation', True)
        self._tie_min = declare_float(self, 'label_min_error_m', 0.002,
                                      minimum=0.0, maximum=1.0)
        self._hud = declare_bool(self, 'draw_hud', True)
        max_points = declare_int(self, 'max_points', 2000,
                                 minimum=2, maximum=200000)
        spacing = declare_float(self, 'min_point_spacing_m', 0.002,
                                minimum=0.0, maximum=1.0)
        jump = declare_float(self, 'jump_reset_m', 0.10,
                             minimum=0.0, maximum=10.0)

        self._show = declare_bool(self, 'show_window', True)
        self._win = declare_str(self, 'window_name', 'EE trajectory overlay')
        # 0 = leave the window at the color frame's native resolution. The
        # launch file fills these in from camera_depth_profile so this window
        # lands at the SAME on-screen size as real_time_distance's — that one
        # is auto-sized to the depth stream, which is usually a different
        # resolution than this node's color stream.
        self._win_w = declare_int(self, 'window_width', 0, minimum=0, maximum=8000)
        self._win_h = declare_int(self, 'window_height', 0, minimum=0, maximum=8000)
        publish = declare_bool(self, 'publish_overlay', True)

        # Same settings for both: the two curves are meant to be compared, and
        # decimating them differently would put the red and blue vertices at
        # different arc lengths — a sawtooth between two identical curves.
        self._des = TraceBuffer(max_points=max_points, min_spacing_m=spacing,
                                jump_reset_m=jump)
        self._act = TraceBuffer(max_points=max_points, min_spacing_m=spacing,
                                jump_reset_m=jump)
        self._lock = threading.Lock()

        # ── Calibration ──────────────────────────────────────────────────
        self._R = self._t = None
        self._ext_path = ext_path or os.path.join(
            _share_dir(), 'config', 'camera_extrinsics.yaml')
        try:
            self._R, self._t = load_extrinsics(self._ext_path)
        except Exception as exc:                                  # noqa: BLE001
            # Not fatal: the window still shows the camera, and the banner on it
            # says why nothing is drawn. A viewer that exits on a missing file
            # teaches you nothing while you are looking for the file.
            self.get_logger().error(
                f'camera extrinsics unreadable ({self._ext_path}): {exc}. '
                f'The frames will be shown WITHOUT the traces.')

        self._K: Optional[np.ndarray] = None
        self._D: Optional[np.ndarray] = None
        self._bridge = CvBridge()
        self._layer: Optional[np.ndarray] = None
        self._alpha: Optional[np.ndarray] = None
        self._dirty: Optional[Tuple[int, int, int, int]] = None
        self._win_open = False
        self._frames = 0

        self._img_topic = img_topic
        self._pub = (self.create_publisher(Image, out_topic, 1)
                     if publish else None)
        self.create_subscription(CameraInfo, info_topic, self._on_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, img_topic, self._on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(PointStamped, des_topic, self._on_desired, 10)
        self.create_subscription(PointStamped, act_topic, self._on_actual, 10)
        # A viewer with no frames publishes nothing and opens no window, which
        # looks exactly like a crashed node. Say which topic is silent instead.
        self.create_timer(5.0, self._watchdog)

        self.get_logger().info(
            f'trajectory_overlay ready\n'
            f'  image          ← {img_topic}\n'
            f'  camera_info    ← {info_topic}\n'
            f'  desired (red)  ← {des_topic}\n'
            f'  actual  (blue) ← {act_topic}\n'
            f'  overlay        → {out_topic if publish else "(not published)"}\n'
            f'  window         {"ON" if self._show else "OFF"}, '
            f'trail {self._ttl:.1f} s, spacing {spacing * 1000:.0f} mm, '
            f'extrinsics {os.path.basename(self._ext_path)}')

    # ── Subscriptions ────────────────────────────────────────────────────

    def _on_info(self, msg: CameraInfo) -> None:
        K = np.asarray(msg.k, dtype=float).reshape(3, 3)
        if K[0, 0] <= 0.0 or K[1, 1] <= 0.0:
            return
        D = np.asarray(msg.d, dtype=float) if len(msg.d) else None
        with self._lock:
            first = self._K is None
            self._K, self._D = K, (D if D is not None and np.any(D) else None)
        if first:
            self.get_logger().info(
                f'camera_info: fx={K[0, 0]:.1f} fy={K[1, 1]:.1f} '
                f'c=({K[0, 2]:.1f}, {K[1, 2]:.1f}), '
                f'distortion {"applied" if self._D is not None else "zero"}')

    def _stamp_s(self, msg) -> float:
        """Seconds for the trail clock, from the message when it carries one.

        The commander stamps from the same ROS clock this node reads, so using
        its stamp keeps a vertex's age honest even when the overlay is a frame
        behind. An unstamped message falls back to now — better a trail that is
        a millisecond optimistic than one whose whole history is at t=0 and
        expires the instant it arrives.
        """
        t = Time.from_msg(msg.header.stamp).nanoseconds
        return (t if t > 0 else self.get_clock().now().nanoseconds) * 1e-9

    def _on_desired(self, msg: PointStamped) -> None:
        with self._lock:
            self._des.add((msg.point.x, msg.point.y, msg.point.z),
                          self._stamp_s(msg))

    def _on_actual(self, msg: PointStamped) -> None:
        with self._lock:
            self._act.add((msg.point.x, msg.point.y, msg.point.z),
                          self._stamp_s(msg))

    # ── Render ───────────────────────────────────────────────────────────

    def _on_image(self, msg: Image) -> None:
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:                                  # noqa: BLE001
            self.get_logger().warn(f'frame dropped, cannot convert: {exc}',
                                   throttle_duration_sec=5.0)
            return
        img = img.copy()          # cv_bridge may hand back a view of the message
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._frames == 0:
            self.get_logger().info(
                f'first frame: {img.shape[1]}x{img.shape[0]} from '
                f'{self._img_topic}')

        with self._lock:
            K, D, R, t = self._K, self._D, self._R, self._t
            self._des.expire(now, self._ttl)
            self._act.expire(now, self._ttl)
            des = np.asarray(self._des.points, dtype=float).reshape(-1, 3)
            act = np.asarray(self._act.points, dtype=float).reshape(-1, 3)
            t_des = list(self._des.times)
            t_act = list(self._act.times)
            tip_d, age_d = self._des.tip, now - self._des.tip_time
            tip_a, age_a = self._act.tip, now - self._act.tip_time

        ready = K is not None and R is not None
        if ready:
            self._draw_traces(img, des, t_des, act, t_act, now, K, D, R, t)
            self._draw_heads(img, tip_d, age_d, tip_a, age_a, K, D, R, t)
        if self._hud:
            self._draw_hud(img, K, tip_d, age_d, tip_a, age_a)

        self._frames += 1
        if self._pub is not None:
            out = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
            out.header = msg.header
            self._pub.publish(out)
        if self._show:
            self._imshow(img)

    def _draw_traces(self, img, des, t_des, act, t_act, now, K, D, R, t) -> None:
        """Project both trails and alpha-blend them onto the frame."""
        h, w = img.shape[:2]
        if self._layer is None or self._layer.shape[:2] != (h, w):
            self._layer = np.zeros((h, w, 3), dtype=np.uint8)
            self._alpha = np.zeros((h, w), dtype=np.uint8)
            self._dirty = None
        elif self._dirty is not None:
            # Clear only what the last frame dirtied. The scratch layers are
            # allocated once; zeroing 720p twice a frame would cost more than
            # everything else this node does.
            x0, y0, x1, y1 = self._dirty
            self._layer[y0:y1 + 1, x0:x1 + 1] = 0
            self._alpha[y0:y1 + 1, x0:x1 + 1] = 0

        bounds = PixelBounds()
        for pts, times, color in ((des, t_des, _RED), (act, t_act, _BLUE)):
            if len(pts) < 2:
                continue
            uv, valid = project_base_to_pixels(pts, R, t, K, D)
            draw_fading_polyline(
                self._layer, self._alpha, uv, valid,
                trail_alphas(times, now, self._ttl, head_hold=self._hold),
                color, self._w_line, bounds, levels=self._levels)

        roi = bounds.roi(self._w_line + 2, w, h)
        composite(img, self._layer, self._alpha, roi)
        self._dirty = roi

    def _draw_heads(self, img, tip_d, age_d, tip_a, age_a, K, D, R, t) -> None:
        """The two chasing dots, and the tie between them.

        Drawn straight onto the frame at full opacity, after the blend: the
        heads are the answer to "where is it NOW" and must not fade with the
        trail they lead. Stale ones are not drawn at all — a commander that has
        stopped publishing should leave an empty image, not a dot parked
        wherever it died.
        """
        uv, valid = project_base_to_pixels(
            [p for p in (tip_d, tip_a) if p is not None], R, t, K, D)
        pix = {}
        i = 0
        for name, tip, age in (('d', tip_d, age_d), ('a', tip_a, age_a)):
            if tip is None:
                continue
            if valid[i] and age <= self._fresh_s():
                pix[name] = (int(round(uv[i, 0])), int(round(uv[i, 1])))
            i += 1

        if self._tie and 'd' in pix and 'a' in pix:
            err = float(np.linalg.norm(np.asarray(tip_d) - np.asarray(tip_a)))
            if err >= self._tie_min:
                cv2.line(img, pix['d'], pix['a'], _AMBER, 2, cv2.LINE_AA)
        for name, color in (('d', _RED), ('a', _BLUE)):
            if name in pix:
                cv2.circle(img, pix[name], self._r_tip, color, -1, cv2.LINE_AA)
                # The dark rim scales with the dot: a 1 px outline around an
                # 11 px disc stops separating it from a bright bench.
                cv2.circle(img, pix[name], self._r_tip, _BLACK,
                           max(2, self._r_tip // 5), cv2.LINE_AA)

    def _fresh_s(self) -> float:
        """How old a head may be and still be drawn.

        Tied to the trail length, with a floor: whatever the trail is set to,
        a head older than a few frames is not "where the arm is", and one
        younger than that has simply not been replaced yet.
        """
        return max(0.2, self._ttl)

    def _draw_hud(self, img, K, tip_d, age_d, tip_a, age_a) -> None:
        lines = []
        if self._R is None:
            lines.append(('NO EXTRINSICS: ' + os.path.basename(self._ext_path),
                          _AMBER))
        elif K is None:
            lines.append(('waiting for camera_info', _AMBER))
        fresh = self._fresh_s()
        if tip_d is None or tip_a is None or min(age_d, age_a) > fresh:
            waited = min(age_d, age_a) if tip_d is not None and tip_a is not None \
                else float('inf')
            lines.append(('no commander data'
                          + (f' ({waited:.1f} s)' if np.isfinite(waited) else ''),
                          _AMBER))
        else:
            err = float(np.linalg.norm(np.asarray(tip_d) - np.asarray(tip_a)))
            lines.append((f'tracking error {err * 1000:.0f} mm', _WHITE))
        lines.append(('red = commanded P(s)', _RED))
        lines.append(('blue = measured EE', _BLUE))
        _text(img, lines)

    def _imshow(self, img) -> None:
        try:
            if not self._win_open:
                cv2.namedWindow(self._win, cv2.WINDOW_NORMAL)
                if self._win_w > 0 and self._win_h > 0:
                    cv2.resizeWindow(self._win, self._win_w, self._win_h)
                self._win_open = True
            cv2.imshow(self._win, img)
            cv2.waitKey(1)
        except cv2.error as exc:
            # No display (headless container, no X forwarding). Say so once and
            # keep publishing: the topic is the part that still works.
            self._show = False
            self.get_logger().warn(
                f'cannot open a window ({exc}). show_window disabled; the '
                f'overlay is still published.')

    def _watchdog(self) -> None:
        if self._frames == 0:
            self.get_logger().warn(
                f'no frames on {self._img_topic} — the camera is not running, '
                f'or image_topic points at the wrong stream')

    # ── Lifecycle ────────────────────────────────────────────────────────

    def request_stop(self) -> None:
        self.done = True
        self._close_window()

    def destroy_node(self) -> bool:
        self._close_window()
        return super().destroy_node()

    def _close_window(self) -> None:
        if self._win_open:
            self._win_open = False
            try:
                cv2.destroyWindow(self._win)
                cv2.waitKey(1)
            except cv2.error:
                pass


def _share_dir() -> str:
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory('franka_experiments')


def main(args=None):
    run_node_main(TrajectoryOverlayNode, args=args)


if __name__ == '__main__':
    main()
