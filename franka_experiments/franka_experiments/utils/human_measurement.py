"""From a camera frame to the 3D keypoints of an arm, for human_tracker.

MediaPipe Pose (creation, re-detection) and its landmarks in pixels, then the aligned
depth: own depth of each keypoint, borrowed depth when it has none, deprojection into the
robot base frame. The FR3 link polyline from TF is here too, as the robot geometry the
validator measures the keypoints against.
"""

import mediapipe as mp
import numpy as np
import rclpy

from franka_experiments.utils.human_utils import quaternion_to_rotation
from franka_experiments.utils.logging_utils import quiet_stderr

TORSO_NAMES = ("left_shoulder", "right_shoulder", "left_hip", "right_hip")


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

    return _pixel_landmarks(pose_landmarks, image_shape, zip(keypoint_names, indices))


def extract_torso_landmarks(pose_landmarks, image_shape):
    """Extract both shoulders and both hips (for the torso check of human_validation)."""
    if pose_landmarks is None:
        return None
    lm = mp.solutions.pose.PoseLandmark
    indices = (lm.LEFT_SHOULDER, lm.RIGHT_SHOULDER, lm.LEFT_HIP, lm.RIGHT_HIP)
    return _pixel_landmarks(pose_landmarks, image_shape, zip(TORSO_NAMES, indices))


def _pixel_landmarks(pose_landmarks, image_shape, named_indices):
    height, width = image_shape[:2]
    landmarks = {}
    for name, index in named_indices:
        point = pose_landmarks.landmark[index]
        # MediaPipe extrapolates out-of-frame landmarks: clipped to the border
        # they would read the depth of an unrelated pixel, so they are not visible
        in_frame = 0.0 <= point.x <= 1.0 and 0.0 <= point.y <= 1.0
        landmarks[name] = {
            "x_px": float(np.clip(point.x * width, 0, width - 1)),
            "y_px": float(np.clip(point.y * height, 0, height - 1)),
            "visibility": float(getattr(point, "visibility", 0.0)) if in_frame else 0.0,
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


def ray_covariance(ray_direction, sigma_perp, sigma_ray):
    """3x3 covariance with std sigma_ray along the camera ray and sigma_perp across it.

    A depth error moves a deprojected point only along its viewing ray, while
    the pixel coordinates still fix it across the ray.
    """
    r = np.asarray(ray_direction, dtype=float)
    r = r / np.linalg.norm(r)
    return sigma_perp**2 * np.eye(3) + (sigma_ray**2 - sigma_perp**2) * np.outer(r, r)


def robot_polyline(tf_buffer, base_frame, links, tip_offset_m, logger):
    """FR3 link origins in base_frame (latest TF) plus the hand tip, (K+1, 3); None if unknown."""
    nodes = []
    for link in links:
        try:
            tf_msg = tf_buffer.lookup_transform(base_frame, link, rclpy.time.Time())
        except Exception as exc:
            logger.warn(f"Robot check skipped, no TF for {link}: {exc}", throttle_duration_sec=5.0)
            return None
        t = tf_msg.transform.translation
        nodes.append(np.array([t.x, t.y, t.z], dtype=float))
    q = tf_msg.transform.rotation
    z_axis = quaternion_to_rotation(q.x, q.y, q.z, q.w)[:, 2]
    nodes.append(nodes[-1] + tip_offset_m * z_axis)
    return np.asarray(nodes)


def create_pose(model_complexity, min_detection_confidence, min_tracking_confidence):
    """MediaPipe Pose in tracking mode, its graph already started.

    The graph starts on an empty frame, with the warnings TFLite prints meanwhile silenced
    (see PoseRedetector).
    """
    pose = mp.solutions.pose.Pose(
        static_image_mode=False,
        model_complexity=int(np.clip(model_complexity, 0, 2)),
        smooth_landmarks=True,
        enable_segmentation=False,
        min_detection_confidence=float(min_detection_confidence),
        min_tracking_confidence=float(min_tracking_confidence),
    )
    with quiet_stderr():
        pose.process(np.zeros((64, 64, 3), dtype=np.uint8))
    return pose


class PoseRedetector:
    """Make MediaPipe look for a person again when the pose it tracks is not one.

    MediaPipe Pose follows one pose and runs its detector again only once that pose is
    lost; a skeleton it has latched onto a robot can hold for seconds while a real
    person walks in, unseen. Its detector alone does not fire on the robots in these
    scenes: resetting the graph (~45 ms, so at most once per cooldown_s) drops the ghost
    and finds the person as soon as one is in view.
    """

    def __init__(self, pose, after_s, cooldown_s):
        self.pose = pose
        self.after_s = float(after_s)
        self.cooldown_s = float(cooldown_s)
        self.not_a_person_since = None
        self.last_redetect = None

    def update(self, not_a_person, t, image):
        """True when the graph was reset now, after not_a_person held for after_s.

        Time going back (a bag loop) restarts both timers.
        """
        if not not_a_person:
            self.not_a_person_since = None
            return False
        if self.not_a_person_since is None or t < self.not_a_person_since:
            self.not_a_person_since = t
        cooldown_over = (self.last_redetect is None or t < self.last_redetect
                         or t - self.last_redetect >= self.cooldown_s)
        if t - self.not_a_person_since < self.after_s or not cooldown_over:
            return False
        # TFLite prints a warning each time the graph starts, from its own threads: the
        # first frame waits for the start, so both run with stderr silenced. That frame
        # (the current one) is also where the detector looks for a person again
        with quiet_stderr():
            self.pose.reset()
            self.pose.process(image)
        self.last_redetect = t
        self.not_a_person_since = None
        return True


class ArmMeasurer:
    """3D keypoints from MediaPipe landmarks and the depth frame aligned to the colour one.

    A visible keypoint without its own depth borrows the arm's, and reaches the filter
    with a ray-shaped covariance (std fallback_depth_std_m along the ray, <= 0: no
    borrowing). Intrinsics come from set_intrinsics; until then nothing is measured.
    """

    def __init__(self, keypoint_names, visibility_threshold, patch_radius, min_depth_m,
                 max_depth_m, measurement_std, fallback_depth_std_m):
        self.keypoint_names = tuple(keypoint_names)
        self.visibility_threshold = float(visibility_threshold)
        self.patch_radius = int(patch_radius)
        self.min_depth_m = float(min_depth_m)
        self.max_depth_m = float(max_depth_m)
        self.measurement_std = float(measurement_std)
        self.fallback_depth_std_m = float(fallback_depth_std_m)
        self.intrinsics = None   # (fx, fy, cx, cy)

    def set_intrinsics(self, fx, fy, cx, cy):
        self.intrinsics = (float(fx), float(fy), float(cx), float(cy))

    def depth_at(self, depth, landmark):
        """Pixel (u, v) of a landmark and the median depth around it [m] (None if invalid)."""
        u, v = int(round(landmark["x_px"])), int(round(landmark["y_px"]))
        return (u, v), depth_patch_median(
            depth, u, v, self.patch_radius, self.min_depth_m, self.max_depth_m)

    def keypoint_depths(self, landmarks, visibilities, depth):
        """Pixel and own depth of each visible keypoint of one arm.

        Returns the pixels {index: (u, v)} and depths (4,) [m], 0 where the keypoint is not
        visible enough or has no valid depth around its pixel.
        """
        depths = np.zeros(len(self.keypoint_names), dtype=float)
        pixels = {}
        if landmarks is None:
            return pixels, depths
        for i, name in enumerate(self.keypoint_names):
            if visibilities[i] < self.visibility_threshold:
                continue
            pixels[i], depth_m = self.depth_at(depth, landmarks[name])
            if depth_m is not None:
                depths[i] = depth_m
        return pixels, depths

    def deproject_arm(self, pixels, own_depths, camera_tf):
        """3D keypoints of one arm in the base frame.

        Returns positions (4, 3) (NaN = not measured), the depths used (4,) and the
        measurement covariances (4, 3, 3) (NaN = default isotropic R, i.e. own depth).
        own_depths is not modified.
        """
        positions = np.full((len(own_depths), 3), np.nan, dtype=float)
        depths = own_depths.copy()
        measurement_covs = np.full((len(own_depths), 3, 3), np.nan, dtype=float)
        if camera_tf is None or self.intrinsics is None:
            return positions, depths, measurement_covs

        rotation, translation = camera_tf
        valid_depths = own_depths[own_depths > 0.0]
        fallback_depth = None
        if self.fallback_depth_std_m > 0.0 and len(valid_depths) >= 2:
            ref_median = float(np.median(valid_depths))
            consistent = valid_depths[np.abs(valid_depths - ref_median) <= 0.20]
            if len(consistent) >= 2:
                fallback_depth = float(np.median(consistent))

        for i, (u, v) in pixels.items():
            borrowed = own_depths[i] <= 0.0
            d = fallback_depth if borrowed else own_depths[i]
            if d is None:
                continue

            # Deproject the 2D pixel to 3D in the camera frame and transform to the robot base frame
            point_camera = deproject(u, v, d, *self.intrinsics)
            point_base = rotation @ point_camera + translation
            if not np.all(np.isfinite(point_base)):
                continue
            positions[i] = point_base
            if borrowed:
                depths[i] = d
                measurement_covs[i] = ray_covariance(
                    rotation @ point_camera, self.measurement_std, self.fallback_depth_std_m
                )
        return positions, depths, measurement_covs

    def torso_points(self, pose_landmarks, depth, image_shape, camera_tf):
        """Shoulders (2, 3) and hips (2, 3) in the base frame, NaN where not measured."""
        points = np.full((len(TORSO_NAMES), 3), np.nan)
        landmarks = extract_torso_landmarks(pose_landmarks, image_shape)
        if landmarks is not None and camera_tf is not None and self.intrinsics is not None:
            rotation, translation = camera_tf
            for i, name in enumerate(TORSO_NAMES):
                if landmarks[name]["visibility"] < self.visibility_threshold:
                    continue
                (u, v), depth_m = self.depth_at(depth, landmarks[name])
                if depth_m is not None:
                    points[i] = rotation @ deproject(u, v, depth_m, *self.intrinsics) + translation
        return points[:2], points[2:]