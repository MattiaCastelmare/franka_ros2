#!/usr/bin/env python3
"""Camera view for placing the calibration tag (shown by RViz as an Image).

Draws on the rectified D405 image the same start check that
``handeye_eye_in_hand_node`` applies before it moves the arm:

* a cross at the principal point and a circle at ``max_offaxis_deg``;
* the detected tag outline — GREEN when it is inside the circle with a good
  decision margin (the calibration would start), ORANGE with an arrow toward
  the cross when it has to be moved, and a red banner when no tag is seen;
* ID, decision margin, off-axis angle and a distance estimated from the tag's
  apparent size (``fx · size / side_px``, exact for a fronto-parallel tag).

Subscribes ``image`` + ``camera_info`` (the rectified pair) and
``/detections``; publishes ``image_overlay``.
"""

from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

cv2.setNumThreads(1)

# RGB (the D405 publishes rgb8)
_GREEN, _ORANGE, _RED, _BLACK = (0, 210, 0), (255, 150, 0), (230, 30, 30), (0, 0, 0)
_TARGET = (0, 220, 255)   # cyan: the start circle and cross


def _text(img, lines, color, origin=(10, 24)):
    x, y = origin
    for line in lines:
        (w, h), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(img, (x - 4, y - h - 6), (x + w + 4, y + 6), _BLACK, -1)
        cv2.putText(img, line, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA)
        y += h + 14


class TagOverlay(Node):

    def __init__(self) -> None:
        super().__init__('tag_overlay')
        self.declare_parameter('tag_id', 0)
        self.declare_parameter('tag_size', 0.10)
        self.declare_parameter('max_offaxis_deg', 12.0)
        self.declare_parameter('min_decision_margin', 20.0)
        self.declare_parameter('distance_min', 0.20)
        self.declare_parameter('distance_max', 0.40)
        self._p = {n: self.get_parameter(n).value for n in (
            'tag_id', 'tag_size', 'max_offaxis_deg', 'min_decision_margin',
            'distance_min', 'distance_max')}

        self._bridge = CvBridge()
        self._K = None
        self._dets = None
        self.create_subscription(CameraInfo, 'camera_info', self._info_cb, qos_profile_sensor_data)
        self.create_subscription(AprilTagDetectionArray, '/detections', self._det_cb, 10)
        self.create_subscription(Image, 'image', self._image_cb, qos_profile_sensor_data)
        self._pub = self.create_publisher(Image, 'image_overlay', 5)

    def _info_cb(self, msg: CameraInfo) -> None:
        self._K = np.array(msg.k, dtype=float).reshape(3, 3)

    def _det_cb(self, msg) -> None:
        self._dets = msg

    def _detection_for(self, stamp):
        if self._dets is None:
            return None
        age = abs((Time.from_msg(stamp) - Time.from_msg(self._dets.header.stamp)).nanoseconds)
        if age > 0.3e9:
            return None
        for det in self._dets.detections:
            if det.id == self._p['tag_id']:
                return det
        return None

    def _image_cb(self, msg: Image) -> None:
        if self._K is None:
            return
        p = self._p
        img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8').copy()
        fx, fy, cx, cy = self._K[0, 0], self._K[1, 1], self._K[0, 2], self._K[1, 2]
        centre = (int(round(cx)), int(round(cy)))
        radius = int(fx * math.tan(math.radians(p['max_offaxis_deg'])))
        # Dark underlay + bright line: the desk under the D405 is often
        # overexposed white, where a plain white marker disappears.
        for color, width in ((_BLACK, 5), (_TARGET, 2)):
            cv2.drawMarker(img, centre, color, cv2.MARKER_CROSS, 40, width)
            cv2.circle(img, centre, radius, color, width, cv2.LINE_AA)

        # Measured on the raw frame, before anything is drawn on it. With the
        # D405 auto exposure over the white desk half the image clipped and the
        # tag's black read 77/255: zero detections, fixed by manual exposure.
        saturated = float((cv2.cvtColor(img, cv2.COLOR_RGB2GRAY) >= 250).mean())
        exposure_hint = ([f'OVEREXPOSED: {100 * saturated:.0f}% of pixels clipped - lower exposure']
                         if saturated > 0.20 else [])

        det = self._detection_for(msg.header.stamp)
        if det is None:
            _text(img, ['TAG NOT DETECTED', f'looking for tag36h11 id {p["tag_id"]}']
                  + exposure_hint, _RED)
        else:
            corners = np.array([[c.x, c.y] for c in det.corners], dtype=float)
            u, v = det.centre.x, det.centre.y
            off = math.degrees(math.atan(math.hypot((u - cx) / fx, (v - cy) / fy)))
            side_px = float(np.mean(np.linalg.norm(corners - np.roll(corners, 1, axis=0), axis=1)))
            dist = fx * p['tag_size'] / max(side_px, 1.0)
            centred = off <= p['max_offaxis_deg']
            good_margin = det.decision_margin >= p['min_decision_margin']
            color = _GREEN if centred and good_margin else _ORANGE
            cv2.polylines(img, [corners.astype(np.int32)], True, color, 3, cv2.LINE_AA)
            if not centred:
                cv2.arrowedLine(img, (int(u), int(v)), centre, _ORANGE, 2, cv2.LINE_AA, tipLength=0.05)
            in_range = p['distance_min'] <= dist <= p['distance_max']
            lines = [
                f'id {det.id}  margin {det.decision_margin:.0f}'
                + ('' if good_margin else f' (< {p["min_decision_margin"]:.0f})'),
                f'off-axis {off:.1f} deg (max {p["max_offaxis_deg"]:.0f})',
                f'distance ~{dist:.2f} m (best {p["distance_min"]:.2f}-{p["distance_max"]:.2f})'
                + ('' if in_range else ' !'),
                'OK: ready to calibrate' if centred and good_margin
                else 'move the TAG toward the cross',
            ]
            _text(img, lines, color)

        out = self._bridge.cv2_to_imgmsg(img, encoding='rgb8')
        out.header = msg.header
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TagOverlay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C in a terminal reaches the node twice (the terminal's SIGINT and
        # ros2 launch forwarding it); the second must not abort the cleanup.
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
