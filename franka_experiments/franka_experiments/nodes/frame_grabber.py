#!/usr/bin/env python3
"""Save camera frames to PNG for poster / figure capture.

Subscribes to a colour image topic (RealSense by default) and writes PNG frames
to a timestamped folder at a configurable rate.  Optionally saves *only* while
the CBF reports active constraints (``/NS_1/cbf_status`` data[0] > 0), so you
capture exactly the moments the robot deviates its trajectory.

Decoupled design: the image callback just caches the latest frame; a timer at
``save_rate_hz`` writes the most recent one, so the save rate is independent of
the camera frame rate.

Run (after sourcing the workspace, with the stack + camera running):

    ros2 run franka_experiments frame_grabber

    # only while the CBF is deviating the robot, 5 frames/s:
    ros2 run franka_experiments frame_grabber --ros-args \\
        -p only_when_cbf_active:=true -p save_rate_hz:=5.0

Filenames encode index, wall-time since start and the CBF flag, e.g.
``frame_00042_t0013.40_cbf1.png`` — clean PNGs (no overlay) unless
``annotate:=true``.
"""
from __future__ import annotations

import time
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge

from franka_experiments.utils.ros import run_node_main


class FrameGrabber(Node):

    def __init__(self):
        super().__init__('frame_grabber')

        self.declare_parameter('image_topic',      '/camera/camera/color/image_raw')
        self.declare_parameter('cbf_status_topic',  '/NS_1/cbf_status')
        self.declare_parameter('output_dir',
                               '/ros2_ws/src/franka_experiments/franka_logs/poster_frames')
        self.declare_parameter('save_rate_hz',       5.0)
        self.declare_parameter('only_when_cbf_active', False)
        self.declare_parameter('annotate',           False)

        image_topic    = str(self.get_parameter('image_topic').value)
        cbf_topic      = str(self.get_parameter('cbf_status_topic').value)
        out_base       = str(self.get_parameter('output_dir').value)
        self._rate     = float(self.get_parameter('save_rate_hz').value)
        self._only_cbf = bool(self.get_parameter('only_when_cbf_active').value)
        self._annotate = bool(self.get_parameter('annotate').value)

        self.done           = False
        self._bridge        = CvBridge()
        self._latest        = None    # latest cv2 BGR frame
        self._cbf_active    = False
        self._n_constraints = 0.0
        self._saved         = 0
        self._t0            = time.monotonic()

        # Timestamped subfolder so successive runs never overwrite each other.
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._dir = Path(out_base) / f'run_{stamp}'
        try:
            self._dir.mkdir(parents=True, exist_ok=True)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Cannot create output dir {self._dir}: {exc}')
            raise SystemExit(1) from exc

        self.create_subscription(Image, image_topic, self._img_cb, 10)
        self.create_subscription(Float64MultiArray, cbf_topic, self._cbf_cb, 10)

        period = 1.0 / max(0.1, self._rate)
        self._timer = self.create_timer(period, self._save_tick)

        self.get_logger().info(
            f'frame_grabber\n'
            f'  image_topic : {image_topic}\n'
            f'  cbf_status  : {cbf_topic}\n'
            f'  output_dir  : {self._dir}\n'
            f'  save_rate   : {self._rate} Hz\n'
            f'  only_cbf    : {self._only_cbf}\n'
            f'  annotate    : {self._annotate}')

    # ── Subscriptions ──────────────────────────────────────────────────────

    def _img_cb(self, msg: Image) -> None:
        try:
            self._latest = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'cv_bridge conversion failed: {exc}')

    def _cbf_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 1:
            self._n_constraints = float(msg.data[0])
            self._cbf_active = self._n_constraints > 0.0

    # ── Timer: save the latest frame ───────────────────────────────────────

    def _save_tick(self) -> None:
        frame = self._latest
        if frame is None:
            return
        if self._only_cbf and not self._cbf_active:
            return

        t = time.monotonic() - self._t0
        cbf_flag = 1 if self._cbf_active else 0
        name = f'frame_{self._saved:05d}_t{t:07.2f}_cbf{cbf_flag}.png'

        img = frame
        if self._annotate:
            img = frame.copy()
            label = (f't={t:6.2f}s  '
                     f'CBF={"ON" if self._cbf_active else "off"} '
                     f'(n={self._n_constraints:.0f})')
            # Black outline + green text for legibility on any background.
            cv2.putText(img, label, (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(img, label, (12, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 255, 0), 2, cv2.LINE_AA)

        try:
            cv2.imwrite(str(self._dir / name), img)
            self._saved += 1
            if self._saved % 10 == 0:
                self.get_logger().info(f'Saved {self._saved} frames → {self._dir}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'imwrite failed: {exc}')

    # ── Lifecycle ──────────────────────────────────────────────────────────

    def request_stop(self) -> None:
        self.get_logger().info(
            f'Stopping — {self._saved} frames saved in {self._dir}')
        self.done = True


def main(args=None):
    run_node_main(FrameGrabber, args=args)


if __name__ == '__main__':
    main()
