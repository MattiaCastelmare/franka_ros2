"""2D overlay of the tracked arm on the camera image, for human_visualizer.

The arm as the tracker uses it (only valid keypoints: filled where measured, hollow where
only predicted), the red ghosts of its constant-velocity prediction, the minimum
robot-human distance and the info bar. Base-frame points reach the image through
CameraProjector.
"""

import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration

from franka_experiments.utils.human_utils import (
    extract_human_keypoints, predict_future_positions, quaternion_to_rotation,
)


class CameraProjector:
    """Pixels of base-frame points: intrinsics from CameraInfo, static base -> camera TF cached."""

    def __init__(self, tf_buffer, base_frame='fr3_link0'):
        self.tf_buffer = tf_buffer
        self.base_frame = base_frame
        self.intrinsics = None   # (fx, fy, cx, cy)
        self.camera_frame = None
        self.base_to_camera = None

    def set_camera_info(self, msg):
        """Keep the first intrinsics received."""
        if self.intrinsics is None:
            self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
            self.camera_frame = msg.header.frame_id

    def lookup_base_to_camera(self):
        """Static base -> camera transform (R, t), cached after the first lookup."""
        if self.base_to_camera is None:
            tf_msg = self.tf_buffer.lookup_transform(
                self.camera_frame, self.base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.0)
            )
            q = tf_msg.transform.rotation
            tr = tf_msg.transform.translation
            self.base_to_camera = (
                quaternion_to_rotation(q.x, q.y, q.z, q.w),
                np.array([tr.x, tr.y, tr.z]),
            )
        return self.base_to_camera

    def project(self, point_base):
        """Pixel (u, v) of a base-frame point, or None (no intrinsics/TF, behind the camera)."""
        if self.intrinsics is None or not self.camera_frame:
            return None
        try:
            R, t = self.lookup_base_to_camera()
        except Exception:
            return None
        return project_to_pixel(point_base, R, t, *self.intrinsics)


def parse_landmarks_msg(msg, count):
    """MediaPipe pixels (count, 2) and visibilities (count,) of a 2D landmarks message.

    None if the message has fewer points or any is not finite.
    """
    if len(msg.points) < count:
        return None
    points = np.asarray([[p.x, p.y] for p in msg.points[:count]], dtype=np.float32)
    if not np.all(np.isfinite(points)):
        return None
    visibilities = np.zeros(count, dtype=np.float32)
    for channel in msg.channels:
        if channel.name == 'visibility':
            n = min(len(channel.values), count)
            if n > 0:
                visibilities[:n] = np.asarray(channel.values[:n], dtype=np.float32)
            break
    return points, visibilities


def predicted_pixels(state, project):
    """Projection of each valid keypoint's Kalman estimate, None for the others."""
    positions, _, valid = extract_human_keypoints(state)
    return [project(p) if valid[i] else None for i, p in enumerate(positions)]


def prediction_pixels(state, project, step_dt, num_steps):
    """Per step of the prediction, the projection of each valid keypoint (None if not).

    Computed from the arm state as the tracker does (constant velocity): the ghosts belong
    to the drawn frame, while the prediction topic may still hold the previous one.
    """
    positions, velocities, valid = extract_human_keypoints(state)
    future = [
        predict_future_positions(positions[i], velocities[i], step_dt, num_steps)
        if valid[i] else None
        for i in range(len(valid))
    ]
    return [[None if f is None else project(f[step]) for f in future]
            for step in range(num_steps)]


def arm_speeds(state):
    """Speed of each keypoint [m/s], 0 where not valid (no state: all 0)."""
    if state is None:
        return [0.0] * 4
    _, velocities, valid = extract_human_keypoints(state)
    return [float(np.linalg.norm(v)) if valid[i] else 0.0 for i, v in enumerate(velocities)]


def landmarks_are_recent(
    image_stamp_ns,
    last_valid_landmark_stamp_ns,
    landmark_hold_s,
):
    """Check whether the last pose can still be drawn on the current image."""
    if last_valid_landmark_stamp_ns is None:
        return False

    age_s = max(
        0.0,
        (image_stamp_ns - last_valid_landmark_stamp_ns) * 1e-9,
    )
    return age_s <= landmark_hold_s


def update_display_points(
    target_points,
    display_points,
    smoothing_tau_s,
    max_hz,
    last_render_monotonic_ns,
):
    """Apply the same exponential interpolation used by the visualizer."""
    if target_points is None:
        return display_points, last_render_monotonic_ns

    if display_points is None or smoothing_tau_s <= 0.0:
        return target_points.copy(), time.monotonic_ns()

    now_ns = time.monotonic_ns()
    if last_render_monotonic_ns is None:
        dt = 1.0 / max_hz
    else:
        dt = max(
            1e-4,
            (now_ns - last_render_monotonic_ns) * 1e-9,
        )

    alpha = 1.0 - math.exp(-dt / smoothing_tau_s)
    display_points += alpha * (target_points - display_points)
    return display_points, now_ns


def project_to_pixel(point_base, rotation, translation, fx, fy, cx, cy):
    """Pixel (u, v) of a base-frame point; None if not finite or behind the camera.

    rotation, translation: base -> camera optical frame.
    """
    point_camera = rotation @ np.asarray(point_base, dtype=float) + translation
    if not np.all(np.isfinite(point_camera)) or point_camera[2] <= 0.01:
        return None
    return (
        float(point_camera[0] / point_camera[2] * fx + cx),
        float(point_camera[1] / point_camera[2] * fy + cy),
    )


def tracked_arm_pixels(landmark_px, measured, valid, predicted_px):
    """Where to draw each keypoint of an arm as the tracker uses it, None where not drawn.

    Only the keypoints valid in the arm state (what reaches the CBF) are drawn: one measured
    in this frame at its MediaPipe pixel, one only predicted by the Kalman filter at the
    projection of its estimate (MediaPipe's pixel for it is a guess). Returns, per keypoint,
    None or ((u, v), measured).

    landmark_px: (4, 2) MediaPipe pixels or None; measured, valid: from HumanArmState;
    predicted_px: per keypoint the projected estimate or None.
    """
    keypoints = []
    for i, is_valid in enumerate(valid):
        pixel = None
        if is_valid and measured[i] and landmark_px is not None and np.all(np.isfinite(landmark_px[i])):
            pixel = (float(landmark_px[i][0]), float(landmark_px[i][1]))
        elif is_valid:
            pixel = predicted_px[i]
        keypoints.append(None if pixel is None else (pixel, bool(measured[i])))
    return keypoints


def draw_tracked_arm(image, keypoints, landmark_names, scale, draw_labels):
    """Draw the keypoints of tracked_arm_pixels: filled = measured, hollow = predicted.

    Segments join consecutive keypoints only when both are drawn.
    """
    radius = max(2, int(round(6 * scale)))
    thickness = max(1, int(round(2 * scale)))
    points = []
    for index, keypoint in enumerate(keypoints):
        if keypoint is None:
            points.append(None)
            continue
        (u, v), measured = keypoint
        point = (int(round(u * scale)), int(round(v * scale)))
        points.append(point)
        cv2.circle(image, point, radius, (0, 255, 0), -1 if measured else thickness)
        if draw_labels:
            cv2.putText(
                image,
                landmark_names[index],
                (point[0] + 6, point[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                max(0.3, 0.45 * scale),
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

    for first, second in zip(points[:-1], points[1:]):
        if first is not None and second is not None:
            cv2.line(image, first, second, (0, 255, 255), thickness)


def draw_predicted_arm(image, steps, scale, color=(0, 0, 255)):
    """Ghost arms of the constant-velocity prediction, fading with the horizon as in RViz.

    steps: per prediction step, the pixel (u, v) of each keypoint or None. Each ghost is
    alpha-blended over its bounding box only, so the cost does not grow with the frame size.
    """
    radius = max(2, int(round(5 * scale)))
    thickness = max(1, int(round(3 * scale)))
    margin = radius + thickness
    height, width = image.shape[:2]
    for step in reversed(range(len(steps))):   # nearest ghost drawn last, on top
        alpha = max(0.05, 0.4 - 0.15 * step)
        points = [None if p is None else (int(round(p[0] * scale)), int(round(p[1] * scale)))
                  for p in steps[step]]
        drawn = [p for p in points if p is not None]
        if not drawn:
            continue
        xs, ys = zip(*drawn)
        x0, y0 = max(0, min(xs) - margin), max(0, min(ys) - margin)
        x1, y1 = min(width, max(xs) + margin + 1), min(height, max(ys) + margin + 1)
        if x0 >= x1 or y0 >= y1:
            continue
        roi = image[y0:y1, x0:x1]
        layer = roi.copy()
        local = [None if p is None else (p[0] - x0, p[1] - y0) for p in points]
        for first, second in zip(local[:-1], local[1:]):
            if first is not None and second is not None:
                cv2.line(layer, first, second, color, thickness, cv2.LINE_AA)
        for point in local:
            if point is not None:
                cv2.circle(layer, point, radius, color, -1, cv2.LINE_AA)
        cv2.addWeighted(layer, alpha, roi, 1.0 - alpha, 0.0, dst=roi)

def draw_distance_line(image, link, project, scale):
    """Shortest robot-human distance: white line, robot point (link name) and human point."""
    uv_robot = project((link.closest_point_robot.x, link.closest_point_robot.y,
                        link.closest_point_robot.z))
    uv_human = project((link.closest_point_human.x, link.closest_point_human.y,
                        link.closest_point_human.z))
    if not (uv_robot and uv_human):
        return
    uv_robot = (int(uv_robot[0] * scale), int(uv_robot[1] * scale))
    uv_human = (int(uv_human[0] * scale), int(uv_human[1] * scale))
    cv2.line(image, uv_robot, uv_human, (255, 255, 255), 2)
    cv2.circle(image, uv_robot, 6, (0, 255, 255), -1)
    cv2.circle(image, uv_human, 6, (0, 0, 255), -1)
    cv2.putText(image, link.robot_link_name, (uv_robot[0] + 8, uv_robot[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)


def draw_info_bar(image, time_s, min_distance, speeds_by_side):
    """Top bar: time, minimum distance (None = --) and the keypoint speeds of each arm.

    speeds_by_side: {side: [sh, el, wr, ha] m/s}; the side is labelled only with two arms.
    """
    bar = image[:45 + 20 * (len(speeds_by_side) - 1)]
    bar[:] = (bar * 0.4).astype(image.dtype)
    font = cv2.FONT_HERSHEY_SIMPLEX
    dist_str = "--" if min_distance is None else f"{min_distance:.3f} m"
    cv2.putText(image, f"Time: {time_s:.2f} s   |   Min Dist: {dist_str}", (10, 18),
                font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    for row, (side, speeds) in enumerate(speeds_by_side.items()):
        label = f"{side.upper()[:1]}: " if len(speeds_by_side) > 1 else ""
        text = (f"Speeds [{label}m/s]: sh {speeds[0]:.2f} | el {speeds[1]:.2f} | "
                f"wr {speeds[2]:.2f} | ha {speeds[3]:.2f}")
        cv2.putText(image, text, (10, 38 + 20 * row), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
