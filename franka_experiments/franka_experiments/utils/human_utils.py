"""Shared helper functions for human tracking and visualization.

The functions in this module contain the same calculations previously embedded
in human_tracker.py, human_visualizer.py and human.launch.py. Moving them here
changes only code organization, not runtime behavior.
"""

import math
import time
import cv2
import numpy as np
import mediapipe as mp
from geometry_msgs.msg import Point, Vector3


def extract_arm_landmarks(
    pose_landmarks,
    image_shape,
    pose_side,
    keypoint_names,
):
    """Extract shoulder, elbow, wrist and index pixel coordinates."""
    if pose_landmarks is None:
        return None

    lm = mp.solutions.pose.PoseLandmark
    if pose_side == "right":
        indices = (
            lm.RIGHT_SHOULDER,
            lm.RIGHT_ELBOW,
            lm.RIGHT_WRIST,
            lm.RIGHT_INDEX,
        )
    else:
        indices = (
            lm.LEFT_SHOULDER,
            lm.LEFT_ELBOW,
            lm.LEFT_WRIST,
            lm.LEFT_INDEX,
        )

    height, width = image_shape[:2]
    landmarks = {}
    for name, index in zip(keypoint_names, indices):
        point = pose_landmarks.landmark[index]
        landmarks[name] = {
            "x_px": float(np.clip(point.x * width, 0, width - 1)),
            "y_px": float(np.clip(point.y * height, 0, height - 1)),
            "visibility": float(getattr(point, "visibility", 0.0)),
        }
    return landmarks


def depth_patch_median(
    depth_image,
    u,
    v,
    radius,
    min_depth_m,
    max_depth_m,
):
    """Return the median valid depth around one image pixel."""
    height, width = depth_image.shape[:2]
    u0, u1 = max(0, u - radius), min(width, u + radius + 1)
    v0, v1 = max(0, v - radius), min(height, v + radius + 1)

    values = depth_image[v0:v1, u0:u1].astype(np.float32).ravel()
    values = values[np.isfinite(values) & (values > 0.0)]
    if values.size == 0:
        return None

    depth_m = float(np.median(values))
    if depth_image.dtype == np.uint16 or depth_m > 100.0:
        depth_m /= 1000.0

    if not min_depth_m <= depth_m <= max_depth_m:
        return None
    return depth_m


def deproject(u, v, depth_m, fx, fy, cx, cy):
    """Deproject one RGB-D pixel with the pinhole-camera model."""
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    return np.array([x, y, depth_m], dtype=float)


def quaternion_to_rotation(qx, qy, qz, qw):
    """Convert a quaternion into the same 3x3 rotation matrix used before."""
    return np.array(
        [
            [
                1 - 2 * (qy*qy + qz*qz),
                2 * (qx*qy - qz*qw),
                2 * (qx*qz + qy*qw),
            ],
            [
                2 * (qx*qy + qz*qw),
                1 - 2 * (qx*qx + qz*qz),
                2 * (qy*qz - qx*qw),
            ],
            [
                2 * (qx*qz - qy*qw),
                2 * (qy*qz + qx*qw),
                1 - 2 * (qx*qx + qy*qy),
            ],
        ],
        dtype=float,
    )


def measurement_age(last_valid_time, current_time):
    """Compute the age of the latest valid measurement for each keypoint."""
    age = np.full(4, -1.0, dtype=float)
    known = np.isfinite(last_valid_time)
    age[known] = current_time - last_valid_time[known]
    return age


def to_point(values):
    """Convert a 3-vector into geometry_msgs/Point."""
    msg = Point()
    msg.x, msg.y, msg.z = map(float, values)
    return msg


def to_vector(values):
    """Convert a 3-vector into geometry_msgs/Vector3."""
    msg = Vector3()
    msg.x, msg.y, msg.z = map(float, values)
    return msg


def stamp_to_ns(msg):
    """Convert a ROS message header timestamp to integer nanoseconds."""
    stamp = msg.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


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


def draw_landmarks(
    image,
    points_px,
    visibilities,
    landmark_names,
    visibility_threshold,
    scale,
    draw_labels,
):
    """Draw the arm points, labels and connecting segments."""

    points = []
    radius = max(2, int(round(6 * scale)))
    thickness = max(1, int(round(2 * scale)))

    for index, point_px in enumerate(points_px):
        point = (
            int(round(float(point_px[0]) * scale)),
            int(round(float(point_px[1]) * scale)),
        )
        points.append(point)

        visibility = float(visibilities[index])
        color = (
            (0, 255, 0)
            if visibility >= visibility_threshold
            else (0, 0, 255)
        )

        cv2.circle(image, point, radius, color, -1)

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
        cv2.line(image, first, second, (0, 255, 255), thickness)