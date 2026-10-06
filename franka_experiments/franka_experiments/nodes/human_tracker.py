#!/usr/bin/env python3
"""Human-arm tracking node.

RGB-D -> MediaPipe -> 3D keypoints in fr3_link0 -> Kalman filter
-> current arm state and constant-velocity prediction.

Distances, controllers and visualization are intentionally kept outside this
node. The only robot geometry here is the FR3 link polyline from TF, used by the
validator to reject MediaPipe "people" detected on the robot itself.
"""

import os
import threading
import traceback
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Vector3

from franka_msgs.msg import HumanArmPrediction, HumanArmState, KalmanDiagnostics
from franka_experiments.utils.arm_kf import ArmKalmanFilter
from franka_experiments.utils.distance_utils import load_robot_config
from franka_experiments.utils.human_measurement import (
    ArmMeasurer, PoseRedetector, create_pose, extract_arm_landmarks, robot_polyline,
)
from franka_experiments.utils.human_utils import (
    measurement_age, quaternion_to_rotation, build_arm_state_msg, build_disengaged_state_msg,
    build_prediction_msg, build_2d_landmarks_msg, format_topic,
    check_engagement_start, check_engagement_loss, stamp_to_ns,
)
from franka_experiments.utils.human_validation import (
    DepthBackground, HumanValidator, landmark_regions,
)


class HumanTracker(Node):
    KEYPOINT_NAMES = ("shoulder", "elbow", "wrist", "index")

    def __init__(self):
        super().__init__("human_tracker")

        # Config
        config_path = os.path.join(
            get_package_share_directory("franka_experiments"),
            "config",
            "human_params.yaml",
        )
        config = load_robot_config(config_path)["human_tracker"]
        self.base_frame = str(config["base_frame"])
        self.pose_side = str(config["pose_side"]).lower()
        if self.pose_side not in ("left", "right", "both"):
            raise ValueError("pose_side must be 'left' or 'right' or 'both'.")
        self.active_sides = ["left", "right"] if self.pose_side == "both" else [self.pose_side]

        # Inference and filtering parameters
        self.is_engaged = False
        self.first_visible_time = None
        self.first_lost_time = None
        self.engage_stability_s = float(config["engage_stability_s"])
        self.loss_stability_s = float(config["loss_stability_s"])
        self.inference_hz = max(1.0, float(config["inference_hz"]))
        self.visibility_threshold = float(config["visibility_threshold"])
        self.max_state_age_s = float(config["max_state_age_s"])
        self.reset_after_s = max(
            self.max_state_age_s,
            float(config["reset_after_s"]),
        )
        self.max_speed_m_s = float(config["max_speed_m_s"])
        self.validator = HumanValidator(config["validation"], self.visibility_threshold)
        # A keypoint the filter only predicts stays within the segment band max of a measured neighbour
        self.segment_max_m = (
            [hi for _, hi in self.validator.bands] if bool(config["kf_limit_segments"]) else None)
        self.background = DepthBackground(
            config["validation"]["background"], float(config["min_depth_m"]))
        self.protect_px = float(config["validation"]["background"]["protect_px"])
        # FR3 links whose origins make the polyline of the robot check (fr3_link0 ... fr3_link8)
        robot_cfg = load_robot_config(os.path.join(
            get_package_share_directory("franka_experiments"), "config", "fr3_complete.yaml"))
        self.robot_links = list(robot_cfg["robot"]["segment_links"])
        self.robot_tip_offset_m = float(config["validation"]["robot_tip_offset_m"])
        self.measurement_std = float(config["kf_measurement_std"])
        # RGB-D measurement of the keypoints (intrinsics from camera_info_cb)
        self.measurer = ArmMeasurer(
            self.KEYPOINT_NAMES, self.visibility_threshold,
            patch_radius=int(config["depth_patch_radius"]),
            min_depth_m=float(config["min_depth_m"]),
            max_depth_m=float(config["max_depth_m"]),
            measurement_std=self.measurement_std,
            fallback_depth_std_m=float(config["fallback_depth_std_m"]),
        )

        self.publish_prediction_enabled = bool(
            config["publish_prediction"]
        )
        self.prediction_dt = float(config["prediction_dt"])
        self.prediction_steps = int(config["prediction_steps"])

        # Camera state
        self.bridge = CvBridge()
        self.camera_frame = None
        self.last_image = None
        self.last_depth = None
        self.image_header = None
        self.current_image_time = None
        self.last_update_time = None

        # MediaPipe runs in a separate worker so old camera frames never accumulate
        self.frame_lock = threading.Lock()
        self.pending_rgbd = None
        self.frame_ready = threading.Event()
        self.stop_event = threading.Event()

        # Static camera -> robot-base transform, cached after the first lookup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.R_camera_to_base = None
        self.t_camera_to_base = None

        # MediaPipe pose estimation, made to detect again when it tracks something not a person
        model_complexity = int(np.clip(config["model_complexity"], 0, 2))
        self.pose = create_pose(model_complexity, config["min_detection_confidence"],
                                config["min_tracking_confidence"])
        self.redetector = PoseRedetector(self.pose, config["validation"]["redetect_after_s"],
                                         config["validation"]["redetect_cooldown_s"])

        # Kalman filter for 3D keypoints
        self.kfs = {}
        self.last_valid_time = {}
        
        for side in self.active_sides:
            self.kfs[side] = ArmKalmanFilter(
                dt=float(config["kf_nominal_dt"]),
                process_accel_std=float(config["kf_process_accel_std"]),
                measurement_std=self.measurement_std,
                initial_position_std=float(config["kf_initial_position_std"]),
                initial_velocity_std=float(config["kf_initial_velocity_std"]),
                visibility_threshold=self.visibility_threshold,
                mahalanobis_threshold=float(config["kf_mahalanobis_threshold"]),
            )
            self.last_valid_time[side] = np.full(4, np.nan, dtype=float)

        color_topic = str(config["color_topic"])
        depth_topic = str(config["depth_topic"])
        camera_info_topic = str(config["camera_info_topic"])

        # Subscribers and Synchronizer
        rgbd_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.color_sub = Subscriber(
            self, Image, color_topic, qos_profile=rgbd_qos
        )
        self.depth_sub = Subscriber(
            self, Image, depth_topic, qos_profile=rgbd_qos
        )
        self.rgbd_sync = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=max(1, int(config["sync_queue_size"])),
            slop=max(0.0, float(config["sync_slop_s"])),
        )
        self.rgbd_sync.registerCallback(self.rgbd_cb)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_cb,
            qos_profile_sensor_data,
        )

        # Dynamic Publishers
        latest_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        
        self.state_pubs = {}
        self.raw_state_pubs = {}
        self.prediction_pubs = {}
        self.landmarks_2d_pubs = {}
        self.kf_diag_pubs = {}

        for side in self.active_sides:
            prefix = f"{side}_" if self.pose_side == "both" else ""

            s_topic = format_topic(str(config["state_topic"]), prefix)
            rs_topic = format_topic(str(config["raw_state_topic"]), prefix)
            p_topic = format_topic(str(config["prediction_topic"]), prefix)
            l2d_topic = format_topic(str(config["landmarks_2d_topic"]), prefix)
            diag_topic = f"/human/{prefix}kf_diagnostics"

            self.state_pubs[side] = self.create_publisher(HumanArmState, s_topic, latest_qos)
            self.raw_state_pubs[side] = self.create_publisher(HumanArmState, rs_topic, latest_qos)
            self.prediction_pubs[side] = self.create_publisher(HumanArmPrediction, p_topic, latest_qos)
            self.landmarks_2d_pubs[side] = self.create_publisher(PointCloud, l2d_topic, qos_profile_sensor_data)
            self.kf_diag_pubs[side] = self.create_publisher(KalmanDiagnostics, diag_topic, latest_qos)

        self.worker_thread = threading.Thread(
            target=self.processing_loop,
            name="human_tracker_worker",
            daemon=True,
        )
        self.worker_thread.start()

        self.get_logger().info(
            f"HumanTracker ready: side={self.pose_side}, "
            f"base_frame={self.base_frame}, model_complexity={model_complexity}, "
            f"inference_hz={self.inference_hz:.1f}"
        )

    # ------------------------------------------------------------------
    # Camera input
    # ------------------------------------------------------------------
    def camera_info_cb(self, msg):
        if self.measurer.intrinsics is not None:
            return
        if msg.k[0] <= 0.0 or msg.k[4] <= 0.0:
            self.get_logger().warn("Invalid camera intrinsics, skipping.")
            return

        self.measurer.set_intrinsics(msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.camera_frame = msg.header.frame_id

        self.get_logger().info(
            "Camera intrinsics: fx={:.1f}, fy={:.1f}, cx={:.1f}, cy={:.1f}".format(
                *self.measurer.intrinsics)
        )

    def rgbd_cb(self, color_msg, depth_msg):
        """Store only the newest synchronized RGB-D pair and wake the worker."""
        with self.frame_lock:
            self.pending_rgbd = (color_msg, depth_msg)
        self.frame_ready.set()

    def processing_loop(self):
        """Process each new pair as soon as it arrives, at most inference_hz.

        The cap works on the image stamps: a frame closer than ~one period to
        the last processed one is skipped, never delayed, so it adds no latency.
        """
        min_spacing_ns = int(0.9e9 / self.inference_hz)  # 10% tolerance on stamp jitter
        last_stamp_ns = None

        while not self.stop_event.is_set():
            # The timeout only lets a shutdown be noticed while no frame arrives
            if not self.frame_ready.wait(timeout=0.1):
                continue

            with self.frame_lock:
                rgbd = self.pending_rgbd
                self.pending_rgbd = None
                self.frame_ready.clear()
            if rgbd is None:
                continue

            stamp_ns = stamp_to_ns(rgbd[0])
            if last_stamp_ns is not None and 0 <= stamp_ns - last_stamp_ns < min_spacing_ns:
                continue
            last_stamp_ns = stamp_ns

            try:
                self.process_rgbd(*rgbd)
            except Exception:
                if not rclpy.ok():
                    break
                # Keep the worker alive: a dead thread would silently stop every publication
                self.get_logger().error(
                    f"Frame processing failed:\n{traceback.format_exc()}",
                    throttle_duration_sec=2.0,
                )

    def process_rgbd(self, color_msg, depth_msg):
        """Convert and process one synchronized RGB-D pair."""
        try:
            # MediaPipe wants RGB, which is also the RealSense encoding: no conversion
            self.last_image = self.bridge.imgmsg_to_cv2(
                color_msg, desired_encoding="rgb8")
            self.last_depth = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough")
        except Exception as exc:
            self.get_logger().warn(
                f"Image conversion failed: {exc}", throttle_duration_sec=2.0)
            return

        # Aligned depth must have exactly the same image size as RGB
        if self.last_image.shape[:2] != self.last_depth.shape[:2]:
            self.get_logger().error(
                f"RGB and aligned depth have different resolutions: "
                f"RGB={self.last_image.shape[1]}x{self.last_image.shape[0]}, "
                f"depth={self.last_depth.shape[1]}x{self.last_depth.shape[0]}",
                throttle_duration_sec=2.0,
            )
            return

        self.image_header = color_msg.header
        self.current_image_time = rclpy.time.Time.from_msg(
            color_msg.header.stamp
        )
        self.update_state()

    # ------------------------------------------------------------------
    # Static TF
    # ------------------------------------------------------------------
    def get_camera_to_base_transform(self):
        if self.R_camera_to_base is not None:
            return self.R_camera_to_base, self.t_camera_to_base
        if self.camera_frame is None:
            return None

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Camera-to-base TF unavailable: {exc}",
                throttle_duration_sec=2.0,
            )
            return None

        q = tf_msg.transform.rotation
        self.R_camera_to_base = quaternion_to_rotation(
            q.x, q.y, q.z, q.w
        )
        self.t_camera_to_base = np.array(
            [
                tf_msg.transform.translation.x,
                tf_msg.transform.translation.y,
                tf_msg.transform.translation.z,
            ],
            dtype=float,
        )
        self.get_logger().info(
            f"Cached TF: {self.camera_frame} -> {self.base_frame}"
        )
        return self.R_camera_to_base, self.t_camera_to_base

    def compute_dt(self, current_time):
        if self.last_update_time is None:
            return list(self.kfs.values())[0].dt

        dt = current_time - self.last_update_time
        if dt <= 0.0:
            for side in self.active_sides:
                self.kfs[side].reset()
                self.last_valid_time[side][:] = np.nan
            return list(self.kfs.values())[0].dt
        return float(np.clip(dt, 1e-3, 0.2))


    # ------------------------------------------------------------------
    # Kalman update and publications
    # ------------------------------------------------------------------
    def update_state(self):
        if self.measurer.intrinsics is None:
            return

        # Run MediaPipe pose estimation on the latest RGB image and extract 2D keypoints
        result = self.pose.process(self.last_image)

        # Compute current time for dt and engage logic
        current_time = self.current_image_time.nanoseconds * 1e-9

        # Landmarks and Visibility
        current_visibilities = {}
        extracted_landmarks = {}

        for side in self.active_sides:
            landmarks = extract_arm_landmarks(
                result.pose_landmarks, self.last_image.shape, side, self.KEYPOINT_NAMES
            )
            extracted_landmarks[side] = landmarks
            
            vis = np.zeros(4, dtype=float)
            if landmarks is not None:
                for i, name in enumerate(self.KEYPOINT_NAMES):
                    vis[i] = landmarks[name]["visibility"]
            current_visibilities[side] = vis

            # Published for every processed frame (also when not engaged)
            self.landmarks_2d_pubs[side].publish(
                build_2d_landmarks_msg(landmarks, self.KEYPOINT_NAMES, self.image_header)
            )

        # 3D measurement and validation run on every frame: tracking starts only on an arm
        # that is already measured in 3D and passes the identity checks (human_validation)
        camera_tf = self.get_camera_to_base_transform()
        if self.validator.check_background:
            # The tracked person is never absorbed into the static scene
            protect = (landmark_regions(extracted_landmarks.values(), self.protect_px)
                       if self.is_engaged else ())
            self.background.update(self.last_depth, current_time, protect)
        robot_nodes = (
            robot_polyline(self.tf_buffer, self.base_frame, self.robot_links,
                           self.robot_tip_offset_m, self.get_logger())
            if self.validator.check_robot else None)
        measurements = {}
        for side in self.active_sides:
            pixels, own_depths = self.measurer.keypoint_depths(
                extracted_landmarks[side], current_visibilities[side], self.last_depth)
            positions, depths, measurement_covs = self.measurer.deproject_arm(
                pixels, own_depths, camera_tf)
            # Own depth: NaN covariance = default isotropic R (a borrowed depth has a ray one)
            direct = np.isnan(measurement_covs[:, 0, 0])
            foreground = [
                self.background.foreground(*pixels[i], depths[i])
                if direct[i] and i in pixels and depths[i] > 0.0 else None
                for i in range(4)
            ]
            # A keypoint reading the robot or the scene in front of the arm is placed again,
            # with a depth borrowed from the rest of the arm (rare: only then deproject twice)
            occluded = self.validator.occluded(positions, depths, direct, foreground, robot_nodes)
            if occluded:
                own_depths[occluded] = 0.0
                positions, depths, measurement_covs = self.measurer.deproject_arm(
                    pixels, own_depths, camera_tf)
                direct = np.isnan(measurement_covs[:, 0, 0])
                for i in occluded:
                    foreground[i] = None
            # Rejected keypoints become NaN: the filter only predicts them this frame
            accepted = self.validator.filter_arm(
                side, positions, direct, foreground, robot_nodes, learn=self.is_engaged,
                t=current_time)
            measurements[side] = (positions, accepted, depths, measurement_covs, direct)
        rejects = self.validator.report(current_time)
        if rejects:
            self.get_logger().info(
                f"Validation rejects in the last 5 s: {rejects} "
                f"(since start: {self.validator.summary()})")
        reason = self.validator.not_a_person(self.active_sides)
        if self.redetector.update(reason is not None, current_time, self.last_image):
            self.get_logger().info(
                f"MediaPipe re-detection: the tracked pose is not a person ({reason})",
                throttle_duration_sec=5.0)

        # Engage Logic: on one arm, higher visibility, shoulder and elbow measured in 3D and
        # accepted by the validator, and a plausible torso; all of it for engage_stability_s
        if not self.is_engaged:
            candidate = any(
                check_engagement_start([side], self.validator.engage_visibility_threshold,
                                       current_visibilities)
                and self.validator.engageable(measurements[side][1], measurements[side][4])
                for side in self.active_sides
            )
            # The torso is measured only when its check is on
            if candidate and (not self.validator.check_torso or self.validator.torso_ok(
                    *self.measurer.torso_points(result.pose_landmarks, self.last_depth,
                                                self.last_image.shape, camera_tf))):
                if self.first_visible_time is None:
                    self.first_visible_time = current_time
                elif (current_time - self.first_visible_time) >= self.engage_stability_s:
                    self.is_engaged = True
                    self.first_visible_time = None
                    self.get_logger().info(
                        "Human ENGAGED: at least one arm is stably measured. Start tracking.",
                        throttle_duration_sec=1.0
                    )
            else:
                self.first_visible_time = None

            if not self.is_engaged:
                for side in self.active_sides:
                    self.state_pubs[side].publish(build_disengaged_state_msg(
                        current_visibilities[side], self.image_header, self.base_frame))
                return

        # Compute the time delta since the last update
        dt = self.compute_dt(current_time)

        speed_logs = []
        current_validities = {}

        # Iterate on each active side (left/right) and process the keypoints
        for side in self.active_sides:
            visibilities = current_visibilities[side]
            positions, accepted, depths, measurement_covs, _ = measurements[side]
            log_prefix = f"[{side.upper()}] " if len(self.active_sides) > 1 else ""

            # Publish raw data for KF post-comparison
            raw_msg = build_arm_state_msg(
                positions=positions, velocities=np.zeros((4, 3)), visibilities=visibilities, 
                measured=np.ones(4, dtype=bool), keypoint_valid=np.ones(4, dtype=bool), 
                age=np.zeros(4), header=self.image_header, base_frame=self.base_frame
            )
            self.raw_state_pubs[side].publish(raw_msg)

            # --- Kalman Filter Update ---
            filtered_pos, filtered_vel, measured = self.kfs[side].step(
                positions=accepted, visibilities=visibilities, depths=depths, dt=dt,
                measurement_covariances=measurement_covs,
            )
            if self.segment_max_m is not None:
                filtered_pos, filtered_vel = self.kfs[side].limit_segments(
                    measured, self.segment_max_m)

            # Update the last valid time for each keypoint
            self.last_valid_time[side][measured] = current_time
            age = measurement_age(self.last_valid_time[side], current_time)

            # Reset any keypoint whose latest valid measurement is too old
            for i in range(4):
                if self.kfs[side].initialized[i] and age[i] > self.reset_after_s:
                    self.kfs[side].reset(i)
                    self.last_valid_time[side][i] = np.nan
                    filtered_pos[i] = np.nan
                    filtered_vel[i] = np.nan
                    age[i] = -1.0

            # Determine which keypoints are valid for publication
            keypoint_valid = (
                self.kfs[side].initialized & np.all(np.isfinite(filtered_pos), axis=1)
                & (age >= 0.0) & (age <= self.max_state_age_s)
            )
            
            # Save validity for the disengage logic
            current_validities[side] = keypoint_valid

            # Publish KF Diagnostics
            innovations, p_traces = self.kfs[side].get_diagnostics()
            nis, gated = self.kfs[side].get_consistency()
            diag_msg = KalmanDiagnostics()
            diag_msg.header = self.image_header
            for i in range(4):
                vec = Vector3()
                vec.x, vec.y, vec.z = float(innovations[i, 0]), float(innovations[i, 1]), float(innovations[i, 2])
                diag_msg.innovations.append(vec)
                diag_msg.p_traces.append(float(p_traces[i]))
                diag_msg.nis.append(float(nis[i]))
                diag_msg.gated.append(bool(gated[i]))
            self.kf_diag_pubs[side].publish(diag_msg)

            # Sanity Check: discard any keypoint whose speed exceeds a reasonable threshold
            speed = np.linalg.norm(filtered_vel, axis=1)
            for i in range(4):
                if keypoint_valid[i] and speed[i] > self.max_speed_m_s:
                    self.get_logger().warn(
                        f"{log_prefix}Anomalous velocity for {self.KEYPOINT_NAMES[i]}: {speed[i]:.2f} m/s. Discard data.",
                        throttle_duration_sec=1.0,
                    )
                    keypoint_valid[i] = False
                    self.kfs[side].reset(i)

            # Publish Filtered State
            state_msg = build_arm_state_msg(
                positions=filtered_pos, velocities=filtered_vel, visibilities=visibilities, measured=measured,
                keypoint_valid=keypoint_valid, age=age, header=self.image_header, base_frame=self.base_frame,
                covariances=self.kfs[side].get_covariances(), frames_seen=self.kfs[side].update_count,
            )
            self.state_pubs[side].publish(state_msg)

            # Publish Constant-Velocity Prediction if enabled
            if self.publish_prediction_enabled:
                pred_msg = build_prediction_msg(
                    filtered_pos, filtered_vel, keypoint_valid, age, self.prediction_dt, 
                    self.prediction_steps, self.image_header, self.base_frame
                )
                self.prediction_pubs[side].publish(pred_msg)

            # Log the KF speed for each keypoint
            values = [speed[i] if keypoint_valid[i] else np.nan for i in range(4)]
            speed_logs.append(
                f"{log_prefix}SH={values[0]:.3f}, EL={values[1]:.3f}, "
                f"WR={values[2]:.3f}, HA={values[3]:.3f}"
            )

        self.last_update_time = current_time

        if speed_logs:
            self.get_logger().info(
                " | ".join(speed_logs),
                throttle_duration_sec=1.0,
            )

        # Disengage Logic
        if self.is_engaged:
            if check_engagement_loss(self.active_sides, current_validities):
                if self.first_lost_time is None:
                    self.first_lost_time = current_time
                elif (current_time - self.first_lost_time) >= self.loss_stability_s:
                    self.is_engaged = False
                    self.first_lost_time = None
                    self.get_logger().warn(
                        "Human LOST: all keypoints are occluded or expired. DISENGAGE.", 
                        throttle_duration_sec=1.0
                    )
                    
                    # Total reset of filters
                    for side in self.active_sides:
                        self.kfs[side].reset()
                        self.last_valid_time[side][:] = np.nan
                    self.validator.reset()
            else:
                self.first_lost_time = None

    def stop_worker(self):
        self.stop_event.set()
        self.worker_thread.join()


def main(args=None):
    rclpy.init(args=args)
    node = HumanTracker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_worker()
        node.pose.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()