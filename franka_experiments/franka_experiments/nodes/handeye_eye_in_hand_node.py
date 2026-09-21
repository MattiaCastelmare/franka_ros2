#!/usr/bin/env python3
"""Eye-in-hand hand-eye calibration: D405 on the flange, AprilTag on the desk.

Solves ``T_BE(i) · X · T_CT(i) = Y`` (maths in ``utils/handeye_solver.py``)
for X = ``fr3_link8 → d405_color_optical_frame``; Y = ``fr3_link0 → tag`` comes
out alongside it. This is the dual of ``handeye_calibration_node`` (camera
fixed in the scene, tag on the flange), which stays untouched.

Procedure
---------
Place the arm so the D405 sees the tag from ~0.2–0.4 m and start the launch
(``handeye_eye_in_hand_calibration.launch.py``; ``preview:=true`` shows the
camera in RViz without starting the robot). Then:

0. **Start gate** — the arm does not move until the tag has been detected
   within ``max_start_offaxis_deg`` of the optical axis for
   ``start_centred_s``. Until then the log says which way to move the TAG (the
   robot holds still), and RViz shows the same check on ``image_overlay``.
1. **Bootstrap** — one sample at the start pose, then ±10° about each flange
   axis and ±3 cm along base x/y. Needs no prior on the camera mount. A first
   solve gives a rough X and the tag position.
2. **Orbit** — look-at camera poses on a spherical cap above the tag, aimed
   with the bootstrap X; each candidate is IK-checked (joint-limit margin,
   singularity) a few per control tick, so the 200 Hz command never stalls.
3. **Solve** — outlier rejection, train/test split, verdict. The file is
   written only if the held-out errors pass ``accept_*``.

Every pose is logged as ``<phase> k/N · overall i/M``.

Motion is resolved-rate Cartesian control (Pinocchio + DLS + null-space pull to
the start posture) published as joint velocities on ``tracking_qdot`` for
``rt_velocity_executor_controller``. The command is C²-continuous: a rate
limiter at ``qdot_acc_max`` FOLLOWED by a first-order low-pass, so acceleration
is continuous and jerk bounded (~2·a_max·(1−α)/dt). The controller's own
limiter bounds acceleration but not jerk, and a ``joint_motion_generator_
acceleration_discontinuity`` reflex ended the first hardware run.

Outputs (config dir of the source tree and of the install)
----------------------------------------------------------
* ``camera_EE_extrinsic.yaml`` — ``fr3_link8 → d405_color_optical_frame``
  (top level, same keys as ``camera_extrinsics.yaml``), ``camera_link``:
  ``fr3_link8 → d405_link`` through the driver's own static TF, and
  ``calibration``: residuals, sample count, tag pose in base.
* ``dataset_handeye_eye_in_hand.yaml`` — raw samples, ALWAYS saved, so a run
  can be re-solved without moving the robot::

      ros2 run franka_experiments handeye_eye_in_hand_node --ros-args -p solve_only:=true
"""

from __future__ import annotations

import datetime
import math
import os
import time as _time
from typing import List, Optional

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray

from franka_experiments.utils import handeye_solver as hes
from franka_experiments.utils.constants import AUTO_SENTINEL, NUM_JOINTS
from franka_experiments.utils.logging_utils import ThrottledLogger
from franka_experiments.utils.math_utils import (
    cosine_ramp, lpf, min_jerk, rate_limit)
from franka_experiments.utils.node_runtime import (
    resolve_tracking_topic, run_node_main)

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
except ImportError:  # checked in __init__ when motion is requested
    AprilTagDetectionArray = None

# FR3 shoulder (joint 2 axis) in fr3_link0: reach is measured from here.
_SHOULDER = np.array([0.0, 0.0, 0.333])
_WS_CONFIG_DIR = '/ros2_ws/src/franka_experiments/config'
# Seconds of IK work allowed per 5 ms control tick while planning the orbit.
_PLAN_BUDGET_S = 0.0015


def _quat_dict(R: np.ndarray) -> dict:
    x, y, z, w = Rotation.from_matrix(R).as_quat()
    return {'x': round(float(x), 6), 'y': round(float(y), 6),
            'z': round(float(z), 6), 'w': round(float(w), 6)}


def _transform_dict(T: np.ndarray, parent: str, child: str) -> dict:
    return {
        'parent_frame': parent,
        'child_frame': child,
        'translation': {k: round(float(v), 6) for k, v in zip('xyz', T[:3, 3])},
        'rotation': _quat_dict(T[:3, :3]),
    }


def _tf_to_matrix(tf_msg) -> np.ndarray:
    t, q = tf_msg.transform.translation, tf_msg.transform.rotation
    return hes.make_T(Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix(),
                      [t.x, t.y, t.z])


def _offaxis(T_CT: np.ndarray):
    """Angle [deg] of the tag from the optical axis and where it sits in the image."""
    x, y, z = T_CT[:3, 3]
    angle = math.degrees(math.atan2(math.hypot(x, y), z))
    parts = []
    if abs(y) > 0.05 * z:
        parts.append('bottom' if y > 0 else 'top')
    if abs(x) > 0.05 * z:
        parts.append('right' if x > 0 else 'left')
    return angle, '-'.join(parts) or 'centre'


class HandEyeEyeInHandNode(Node):

    def __init__(self) -> None:
        super().__init__('handeye_eye_in_hand_node')
        P = self.declare_parameter

        # Frames
        P('base_frame', 'fr3_link0')
        P('tool_frame', 'fr3_link8')
        P('camera_frame', 'd405_color_optical_frame')
        P('camera_link_frame', 'd405_link')
        P('tag_frame', 'tag36h11:0')
        P('tag_id', 0)
        P('overlay_topic', '/d405/d405/color_rect/image_overlay')
        # Files
        P('output_file', 'camera_EE_extrinsic.yaml')
        P('dataset_file', 'dataset_handeye_eye_in_hand.yaml')
        P('output_dir', '')
        P('solve_only', False)
        # Control
        P('tracking_topic', '')
        P('joint_state_topic', AUTO_SENTINEL)
        P('rate_hz', 200.0)
        P('kp_pos', 2.0)
        P('kp_rot', 1.5)
        P('damping', 0.02)
        P('qdot_max', 0.2)              # rad/s — far inside the FR3 envelope
        P('qdot_acc_max', 0.5)          # rad/s² — = the controller's max_accel in the launch
        P('lpf_alpha', 0.8)             # after the rate limiter: continuous acceleration
        P('lin_speed', 0.05)            # m/s   — segment duration from distance
        P('ang_speed', 0.25)            # rad/s — segment duration from rotation
        P('min_segment_s', 2.0)
        P('warmup_s', 2.0)
        P('ramp_s', 1.0)
        P('joint_limit_margin_rad', 0.15)
        P('max_tracking_error_m', 0.03)
        P('max_tracking_error_deg', 6.0)
        P('null_space_gain', 0.5)       # 1/s — posture pull toward the start configuration
        P('feasibility_margin_rad', 0.30)
        P('min_jacobian_sigma', 0.05)
        # Settling and capture
        P('pos_tol_m', 0.004)
        P('rot_tol_deg', 1.0)
        P('still_speed_m_s', 0.003)
        P('settle_s', 1.0)
        P('settle_timeout_s', 8.0)
        P('capture_detections', 5)
        P('capture_max_spread_m', 0.002)
        P('capture_max_spread_deg', 0.7)
        P('detection_timeout_s', 4.0)
        P('min_decision_margin', 20.0)
        # Start gate
        P('start_tag_timeout_s', 300.0)
        P('max_start_offaxis_deg', 12.0)
        P('start_centred_s', 2.0)
        # Bootstrap
        P('bootstrap_rot_deg', 10.0)
        P('bootstrap_trans_m', 0.03)
        P('min_bootstrap_samples', 6)
        P('bootstrap_max_error_mm', 15.0)
        # Orbit
        P('orbit_samples', 25)
        P('orbit_candidates', 300)
        P('orbit_radius_min', 0.20)
        P('orbit_radius_max', 0.40)
        P('orbit_max_tilt_deg', 35.0)
        P('orbit_azimuth_half_range_deg', 60.0)
        P('orbit_max_roll_deg', 30.0)
        # Workspace safety (fr3_link0 frame, applied to fr3_link8)
        P('min_ee_z_m', 0.20)
        P('reach_min_m', 0.25)
        P('reach_max_m', 0.80)
        P('max_flange_tilt_deg', 75.0)
        # Solve and verdict
        P('validation_ratio', 0.2)
        P('accept_translation_mm', 5.0)
        P('accept_rotation_deg', 2.0)

        self._p = {name: self.get_parameter(name).value for name in self._parameters}
        p = self._p

        self.done = False
        self._samples_BE: List[np.ndarray] = []
        self._samples_CT: List[np.ndarray] = []
        self._sample_phase: List[str] = []
        self._T_link_optical: Optional[np.ndarray] = None

        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info('=' * 64)
        self.get_logger().info('Eye-in-hand calibration: camera ON the flange, tag FIXED')
        self.get_logger().info(f'  X = {p["tool_frame"]} → {p["camera_frame"]}  (result)')
        self.get_logger().info(f'  Y = {p["base_frame"]} → {p["tag_frame"]}')
        self.get_logger().info(f'  output : {p["output_file"]}   dataset: {p["dataset_file"]}')
        self.get_logger().info('=' * 64)

        if p['solve_only']:
            self._run_solve_only()
            self.done = True
            return

        if AprilTagDetectionArray is None:
            self.get_logger().error('apriltag_msgs not importable — cannot run.')
            self.done = True
            return
        self._init_motion()

    # ══════════════════════════════════════════════════════════════════
    # Setup
    # ══════════════════════════════════════════════════════════════════

    def _init_motion(self) -> None:
        import pinocchio as pin
        from franka_experiments.utils.kinematics import (
            JointStateManager, compute_arm_jacobian, compute_ee_fk, dls_solve,
            generate_urdf_from_xacro, load_pinocchio_model, resolve_arm_joint_ids,
            resolve_frame_id)
        p = self._p
        self.get_logger().info('Building Pinocchio model …')
        self._model, self._data = load_pinocchio_model(generate_urdf_from_xacro())
        self._ee_id = resolve_frame_id(self._model, p['tool_frame'])
        self._joint_ids = resolve_arm_joint_ids(self._model)
        self._idx_q = [self._model.joints[j].idx_q for j in self._joint_ids]
        self._q_lo = np.array(self._model.lowerPositionLimit)[self._idx_q]
        self._q_hi = np.array(self._model.upperPositionLimit)[self._idx_q]
        self._q_neutral = np.array(pin.neutral(self._model))
        self._fk, self._jac, self._dls = compute_ee_fk, compute_arm_jacobian, dls_solve
        self._js = JointStateManager(self, self._model, self._joint_ids,
                                     topic_param=p['joint_state_topic'])

        topic = p['tracking_topic'] or resolve_tracking_topic()
        self._pub = self.create_publisher(Float64MultiArray, topic, 10)
        self.create_subscription(AprilTagDetectionArray, '/detections',
                                 self._detections_cb, 10)
        self._last_det_margin = 0.0
        self._last_det_wall = 0.0

        # State machine
        self._state = 'WARMUP'
        self._phase = 'bootstrap'
        self._queue: List[np.ndarray] = []
        self._goal: Optional[np.ndarray] = None
        self._seg_T0: Optional[np.ndarray] = None
        self._seg_elapsed = 0.0
        self._seg_dur = 1.0
        self._time_scale = 1.0
        self._state_t0 = 0.0
        self._still_since: Optional[float] = None
        self._centred_since: Optional[float] = None
        self._capture_buf: List[np.ndarray] = []
        self._capture_last_stamp: Optional[Time] = None
        self._capture_p0: Optional[np.ndarray] = None
        self._T_BE0: Optional[np.ndarray] = None
        self._q_start: Optional[np.ndarray] = None
        self._plan: Optional[dict] = None
        self._n_skipped = 0
        self._stop_solve = False
        self._p_prev: Optional[np.ndarray] = None
        self._ee_speed = 0.0

        # Progress: poses started overall and within the current phase.
        self._n_boot_total = 0
        self._n_orbit_total: Optional[int] = None
        self._pose_no = 0
        self._phase_pose_no = 0

        self._qdot_rl = np.zeros(NUM_JOINTS)    # rate-limited command
        self._qdot_out = np.zeros(NUM_JOINTS)   # published (rate limit → LPF)
        self._dt = 1.0 / p['rate_hz']
        self._t0 = _time.monotonic()
        self._tlog = ThrottledLogger(self.get_logger(), period_s=1.0)
        self._timer = self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'Motion ready: topic={topic} rate={p["rate_hz"]} Hz '
            f'qdot_max={p["qdot_max"]} rad/s acc_max={p["qdot_acc_max"]} rad/s² '
            f'planned poses: 11 bootstrap + {p["orbit_samples"]} orbit')

    def _detections_cb(self, msg) -> None:
        for det in msg.detections:
            if det.id == self._p['tag_id']:
                self._last_det_margin = float(det.decision_margin)
                self._last_det_wall = _time.monotonic()
                return

    # ══════════════════════════════════════════════════════════════════
    # Perception helpers
    # ══════════════════════════════════════════════════════════════════

    def _lookup_cam_tag(self):
        """Latest camera → tag transform as (matrix, stamp) or None."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._p['camera_frame'], self._p['tag_frame'], Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        return _tf_to_matrix(tf), Time.from_msg(tf.header.stamp)

    def _tag_seen_recently(self) -> bool:
        return (_time.monotonic() - self._last_det_wall < 1.0
                and self._last_det_margin >= self._p['min_decision_margin'])

    def _try_link_optical(self) -> None:
        if self._T_link_optical is not None:
            return
        try:
            tf = self._tf_buffer.lookup_transform(
                self._p['camera_link_frame'], self._p['camera_frame'], Time())
            self._T_link_optical = _tf_to_matrix(tf)
            self.get_logger().info(
                f'Driver TF {self._p["camera_link_frame"]} → '
                f'{self._p["camera_frame"]} acquired.')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass

    # ══════════════════════════════════════════════════════════════════
    # Workspace and kinematic feasibility
    # ══════════════════════════════════════════════════════════════════

    def _ee_pose_cheap_ok(self, T_BE: np.ndarray) -> bool:
        p = self._p
        pos = T_BE[:3, 3]
        if pos[2] < p['min_ee_z_m']:
            return False
        if np.hypot(pos[0], pos[1]) < p['reach_min_m']:
            return False
        if np.linalg.norm(pos - _SHOULDER) > p['reach_max_m']:
            return False
        tilt = math.degrees(math.acos(float(np.clip(-T_BE[2, 2], -1.0, 1.0))))
        return tilt <= p['max_flange_tilt_deg']

    def _ee_pose_ok(self, T_BE: np.ndarray) -> bool:
        return self._ee_pose_cheap_ok(T_BE) and self._ik_feasible(T_BE)

    def _q_full(self, q: np.ndarray) -> np.ndarray:
        q_full = self._q_neutral.copy()
        q_full[self._idx_q] = q
        return q_full

    def _ik_feasible(self, T_goal: np.ndarray) -> bool:
        """Can the arm hold ``T_goal`` comfortably?

        Damped IK from the start posture with the same null-space pull the
        controller applies. Rejects solutions within ``feasibility_margin_rad``
        of a joint limit — the velocity fade would stall the arm short of the
        target there (in simulation joint 4 stopped on its straight-elbow
        limit) — or near a singularity.
        """
        p = self._p
        q = self._q_start.copy()
        for _ in range(100):
            q_full = self._q_full(q)
            o = self._fk(self._model, self._data, q_full, self._ee_id)
            e = np.concatenate([T_goal[:3, 3] - np.array(o.translation),
                                hes.rotvec(T_goal[:3, :3] @ np.array(o.rotation).T)])
            J = self._jac(self._model, self._data, q_full, self._ee_id, self._joint_ids)
            if np.linalg.norm(e[:3]) < 5e-4 and np.linalg.norm(e[3:]) < math.radians(0.3):
                break
            J_pinv = J.T @ np.linalg.inv(J @ J.T + 0.05 ** 2 * np.eye(6))
            dq = (J_pinv @ np.clip(e, -0.1, 0.1)
                  + (np.eye(NUM_JOINTS) - J_pinv @ J) @ (0.2 * (self._q_start - q)))
            q = np.clip(q + dq, self._q_lo, self._q_hi)
        else:
            return False
        margin = float(np.min(np.minimum(q - self._q_lo, self._q_hi - q)))
        sigma = float(np.linalg.svd(J, compute_uv=False)[-1])
        return margin >= p['feasibility_margin_rad'] and sigma >= p['min_jacobian_sigma']

    def _limit_report(self) -> str:
        q = self._js.q
        if q is None:
            return ''
        margin = np.minimum(q - self._q_lo, self._q_hi - q)
        j = int(np.argmin(margin))
        return f' [closest joint limit: joint{j + 1} at {margin[j]:.3f} rad]'

    # ══════════════════════════════════════════════════════════════════
    # Command output
    # ══════════════════════════════════════════════════════════════════

    def _publish(self, qdot: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in qdot]
        self._pub.publish(msg)

    def _send(self, qdot_target: np.ndarray) -> None:
        """Rate limit, THEN low-pass: the published velocity has continuous
        acceleration (|q̈| ≤ qdot_acc_max) and bounded jerk. The reverse order
        (the old calibration's) bounds acceleration but lets it flip sign
        between two messages."""
        p = self._p
        self._qdot_rl = rate_limit(self._qdot_rl, qdot_target, p['qdot_acc_max'] * self._dt)
        self._qdot_out = lpf(self._qdot_out, self._qdot_rl, p['lpf_alpha'])
        self._publish(self._qdot_out)

    # ══════════════════════════════════════════════════════════════════
    # Progress
    # ══════════════════════════════════════════════════════════════════

    def _progress(self) -> str:
        if self._phase == 'bootstrap':
            phase_total = self._n_boot_total
            orbit = (self._n_orbit_total if self._n_orbit_total is not None
                     else self._p['orbit_samples'])
            approx = '~' if self._n_orbit_total is None else ''
        else:
            phase_total = self._n_orbit_total or 0
            orbit, approx = phase_total, ''
        total = self._n_boot_total + orbit
        return (f'{self._phase} {self._phase_pose_no}/{phase_total} · '
                f'overall {self._pose_no}/{approx}{total}')

    # ══════════════════════════════════════════════════════════════════
    # State machine
    # ══════════════════════════════════════════════════════════════════

    def _set_state(self, state: str, t: float) -> None:
        self._state = state
        self._state_t0 = t
        self._still_since = None

    def _start_segment(self, T_goal: np.ndarray, T_now: np.ndarray, t: float) -> None:
        p = self._p
        dist = float(np.linalg.norm(T_goal[:3, 3] - T_now[:3, 3]))
        ang = math.radians(hes.rotation_distance_deg(T_now[:3, :3], T_goal[:3, :3]))
        self._goal = T_goal
        self._seg_T0 = T_now.copy()
        self._seg_elapsed = 0.0
        self._seg_dur = max(p['min_segment_s'], dist / p['lin_speed'], ang / p['ang_speed'])
        self._set_state('MOVE', t)

    def _next_goal(self, T_now: np.ndarray, t: float) -> None:
        if self._queue:
            self._pose_no += 1
            self._phase_pose_no += 1
            self._start_segment(self._queue.pop(0), T_now, t)
            self.get_logger().info(
                f'→ pose {self._progress()}  (samples so far: {len(self._samples_BE)}, '
                f'move {self._seg_dur:.1f} s)')
            return
        if self._phase == 'bootstrap':
            self._finish_bootstrap(T_now, t)
        else:
            self.get_logger().info('Orbit complete.')
            self._request_stop(solve=True)

    def _tick(self) -> None:  # noqa: C901
        p = self._p
        t = _time.monotonic() - self._t0

        if self._state == 'STOPPING':
            self._send(np.zeros(NUM_JOINTS))
            if np.max(np.abs(self._qdot_out)) < 1e-4 and not np.any(self._qdot_rl):
                self._timer.cancel()
                self._publish(np.zeros(NUM_JOINTS))
                self._finalise()
            return

        self._try_link_optical()
        js = self._js
        if js.q is None or (self.get_clock().now() - js.stamp).nanoseconds * 1e-9 > 0.1:
            self._send(np.zeros(NUM_JOINTS))
            self._p_prev = None
            if self._tlog.due(t):
                self._tlog.info('Waiting for fresh joint states …')
            return

        q_full = js.q_full.copy()
        oMee = self._fk(self._model, self._data, q_full, self._ee_id)
        T_now = hes.make_T(np.array(oMee.rotation), np.array(oMee.translation))
        J = self._jac(self._model, self._data, q_full, self._ee_id, self._joint_ids)
        if self._p_prev is not None:
            self._ee_speed = lpf(self._ee_speed,
                                 float(np.linalg.norm(T_now[:3, 3] - self._p_prev)) / self._dt,
                                 0.9)
        self._p_prev = T_now[:3, 3].copy()

        if self._state == 'WARMUP':
            self._send(np.zeros(NUM_JOINTS))
            if t >= p['warmup_s']:
                self._start_gate(T_now, t)
            return

        # ── Desired pose ──────────────────────────────────────────────
        T_goal = self._goal
        if self._state == 'MOVE':
            # The reference clock runs at _time_scale (cut by the joint-speed cap
            # below), so the arm is never asked to outrun qdot_max and does not
            # fall behind the reference.
            self._seg_elapsed += self._dt * self._time_scale
            tau = min(self._seg_elapsed / self._seg_dur, 1.0)
            s, sdot = min_jerk(tau)
            w_goal = hes.rotvec(self._seg_T0[:3, :3].T @ T_goal[:3, :3])
            p_d = self._seg_T0[:3, 3] + s * (T_goal[:3, 3] - self._seg_T0[:3, 3])
            R_d = self._seg_T0[:3, :3] @ hes.from_rotvec(s * w_goal)
            k = self._time_scale * sdot / self._seg_dur
            v_ff = k * (T_goal[:3, 3] - self._seg_T0[:3, 3])
            w_ff = k * (self._seg_T0[:3, :3] @ w_goal)
            if tau >= 1.0:
                self._set_state('SETTLE', t)
        else:                                   # SETTLE, CAPTURE, PLAN: hold the goal
            p_d, R_d = T_goal[:3, 3], T_goal[:3, :3]
            v_ff = w_ff = np.zeros(3)
        p_d = np.array([p_d[0], p_d[1], max(p_d[2], p['min_ee_z_m'])])

        pos_err = float(np.linalg.norm(p_d - T_now[:3, 3]))
        rot_err = hes.rotation_distance_deg(T_now[:3, :3], R_d)

        # Watchdog: the reference moves slowly and continuously, so a large
        # gap means the arm is not following — controller inactive, commands
        # going nowhere, or the arm blocked. Stop instead of skipping poses.
        if pos_err > p['max_tracking_error_m'] or rot_err > p['max_tracking_error_deg']:
            self.get_logger().error(
                f'Arm is not following the reference (pos {pos_err*1000:.1f} mm, '
                f'rot {rot_err:.2f}°): is the controller active on the tracking topic? '
                f'Stopping.' + self._limit_report())
            self._request_stop(solve=len(self._samples_BE) >= 8)
            return

        # ── SETTLE: at rest on the target ─────────────────────────────
        if self._state == 'SETTLE':
            at_rest = (pos_err < p['pos_tol_m'] and rot_err < p['rot_tol_deg']
                       and self._ee_speed < p['still_speed_m_s'])
            if at_rest:
                self._still_since = self._still_since if self._still_since is not None else t
                if t - self._still_since >= p['settle_s']:
                    self._set_state('CAPTURE', t)
                    self._capture_buf = []
                    self._capture_last_stamp = self.get_clock().now()
                    self._capture_p0 = T_now[:3, 3].copy()
            else:
                self._still_since = None
            if self._state == 'SETTLE' and t - self._state_t0 > p['settle_timeout_s']:
                self._skip(f'did not settle (pos {pos_err*1000:.1f} mm, '
                           f'rot {rot_err:.2f}°, speed {self._ee_speed*1000:.1f} mm/s)'
                           + self._limit_report(), T_now, t)
                return

        # ── CAPTURE: N fresh, consistent detections ───────────────────
        elif self._state == 'CAPTURE':
            got = self._lookup_cam_tag()
            if got is not None and got[1] > self._capture_last_stamp and self._tag_seen_recently():
                self._capture_last_stamp = got[1]
                self._capture_buf.append(got[0])
            if len(self._capture_buf) >= p['capture_detections']:
                self._finish_capture(T_now, t)
                return
            if t - self._state_t0 > p['detection_timeout_s']:
                self._skip(f'only {len(self._capture_buf)}/{p["capture_detections"]} '
                           f'detections (tag out of view or blurred?)', T_now, t)
                return

        # ── PLAN: IK-check orbit candidates a few per tick ────────────
        elif self._state == 'PLAN':
            if self._plan_step(T_now, t):
                return

        # ── Resolved-rate law ─────────────────────────────────────────
        v = v_ff + p['kp_pos'] * (p_d - T_now[:3, 3])
        w = w_ff + p['kp_rot'] * hes.rotvec(R_d @ T_now[:3, :3].T)
        qdot = self._dls(J, np.concatenate([v, w]), p['damping'])
        if qdot is None:
            qdot = np.zeros(NUM_JOINTS)
        # Posture: pull toward the start configuration in the task null space,
        # so the redundant arm does not wander between poses and keeps the
        # elbow configuration the feasibility check assumed.
        J_pinv = np.linalg.pinv(J)
        qdot = (np.asarray(qdot).reshape(NUM_JOINTS)
                + (np.eye(NUM_JOINTS) - J_pinv @ J)
                @ (p['null_space_gain'] * (self._q_start - js.q)))
        qdot = cosine_ramp(t - p['warmup_s'], p['ramp_s']) * qdot

        # Fade any joint velocity that pushes further into a position limit:
        # libfranka's velocity envelope collapses there and aborts the motion.
        q = js.q
        margin = p['joint_limit_margin_rad']
        up = np.clip((self._q_hi - q) / margin, 0.0, 1.0)
        dn = np.clip((q - self._q_lo) / margin, 0.0, 1.0)
        qdot = np.where(qdot > 0, qdot * up, qdot * dn)

        # Uniform scaling keeps the Cartesian direction (a per-joint clamp bends
        # the path); the same ratio slows the reference clock, recovering at
        # 1 %/tick once the cap is no longer hit.
        peak = float(np.max(np.abs(qdot)))
        if peak > p['qdot_max']:
            qdot = qdot * (p['qdot_max'] / peak)
            self._time_scale = max(0.05, self._time_scale * p['qdot_max'] / peak)
        else:
            self._time_scale = min(1.0, self._time_scale * 1.01)
        self._send(qdot)

        if self._tlog.due(t):
            self._tlog.info(
                f'[{self._progress()} · {self._state}] samples={len(self._samples_BE)} '
                f'pos_err={pos_err*1000:.1f} mm rot_err={rot_err:.2f}° '
                f'tag={"yes" if self._tag_seen_recently() else "no"}')

    def _start_gate(self, T_now: np.ndarray, t: float) -> None:
        """Hold the arm until the tag is seen, centred and steady."""
        p = self._p
        got = self._lookup_cam_tag()
        if got is None or not self._tag_seen_recently():
            self._centred_since = None
            if t - p['warmup_s'] > p['start_tag_timeout_s']:
                self.get_logger().error(
                    f'Tag {p["tag_frame"]} not usable within {p["start_tag_timeout_s"]:.0f} s. '
                    f'Stopping without moving.')
                self._request_stop(solve=False)
            elif self._tlog.due(t):
                self._tlog.info(f'Waiting for tag {p["tag_frame"]} (margin ≥ '
                                f'{p["min_decision_margin"]}) — check RViz {p["overlay_topic"]}')
            return

        T_CT = got[0]
        angle, where = _offaxis(T_CT)
        dist = float(np.linalg.norm(T_CT[:3, 3]))
        if angle > p['max_start_offaxis_deg']:
            self._centred_since = None
            if self._tlog.due(t):
                self.get_logger().warn(
                    f'Tag is {angle:.1f}° off the optical axis (limit '
                    f'{p["max_start_offaxis_deg"]:.0f}°), in the {where} of the image: move the '
                    f'TAG toward the cross in RViz ({p["overlay_topic"]}). The arm holds still.')
            return
        if self._centred_since is None:
            self._centred_since = t
            self.get_logger().info(f'Tag centred ({angle:.1f}° off-axis, {dist:.3f} m): '
                                   f'starting in {p["start_centred_s"]:.0f} s — hands off the tag.')
        if t - self._centred_since < p['start_centred_s']:
            return

        self._T_BE0 = T_now.copy()
        self._q_start = self._js.q.copy()
        self.get_logger().info(
            f'Tag at {dist:.3f} m, {angle:.1f}° off-axis (margin {self._last_det_margin:.0f}). '
            f'Start flange at {np.round(T_now[:3, 3], 3).tolist()}')
        if not (p['orbit_radius_min'] <= dist <= p['orbit_radius_max']):
            self.get_logger().warn(
                f'Start distance {dist:.3f} m is outside the orbit range '
                f'[{p["orbit_radius_min"]}, {p["orbit_radius_max"]}] m — the first orbit '
                f'move will be long.')
        boot = [T for T in hes.bootstrap_ee_poses(
            T_now, rot_deg=p['bootstrap_rot_deg'], trans_m=p['bootstrap_trans_m'])
            if self._ee_pose_ok(T)]
        self._queue = [T_now.copy()] + boot
        self._n_boot_total = len(self._queue)
        self.get_logger().info(
            f'Phase 1/2 BOOTSTRAP: {self._n_boot_total} poses around the start '
            f'(then ~{p["orbit_samples"]} orbit poses, ~{self._n_boot_total + p["orbit_samples"]} total).')
        self._next_goal(T_now, t)

    def _skip(self, why: str, T_now: np.ndarray, t: float) -> None:
        self._n_skipped += 1
        self.get_logger().warn(f'  ✗ pose {self._progress()} skipped: {why}')
        self._next_goal(T_now, t)

    def _finish_capture(self, T_now: np.ndarray, t: float) -> None:
        p = self._p
        buf = np.array(self._capture_buf)
        pos = buf[:, :3, 3]
        spread_m = float(np.max(np.linalg.norm(pos - pos.mean(axis=0), axis=1)))
        R_mean = Rotation.from_matrix(buf[:, :3, :3]).mean().as_matrix()
        spread_deg = max(hes.rotation_distance_deg(R_mean, T[:3, :3]) for T in buf)
        drift = float(np.linalg.norm(T_now[:3, 3] - self._capture_p0))
        if spread_m > p['capture_max_spread_m'] or spread_deg > p['capture_max_spread_deg'] \
                or drift > 0.001:
            self._skip(f'unstable detections (spread {spread_m*1000:.1f} mm / '
                       f'{spread_deg:.2f}°, flange drift {drift*1000:.1f} mm)', T_now, t)
            return
        self._samples_BE.append(T_now.copy())
        self._samples_CT.append(hes.make_T(R_mean, pos.mean(axis=0)))
        self._sample_phase.append(self._phase)
        angle, _ = _offaxis(self._samples_CT[-1])
        self.get_logger().info(
            f'  ✓ sample {len(self._samples_BE)} at pose {self._progress()}: tag '
            f'{np.linalg.norm(pos.mean(axis=0)):.3f} m, {angle:.1f}° off-axis, spread '
            f'{spread_m*1000:.2f} mm / {spread_deg:.2f}°, margin {self._last_det_margin:.0f}')
        self._next_goal(T_now, t)

    def _finish_bootstrap(self, T_now: np.ndarray, t: float) -> None:
        p = self._p
        n = len(self._samples_BE)
        if n < p['min_bootstrap_samples']:
            self.get_logger().error(
                f'Bootstrap got {n} samples (< {p["min_bootstrap_samples"]}). '
                f'Is the tag well lit, centred and fully in view?')
            self._request_stop(solve=False)
            return
        try:
            res = hes.solve(self._samples_BE, self._samples_CT, validation_ratio=0.0)
        except ValueError as exc:
            self.get_logger().error(f'Bootstrap solve failed: {exc}')
            self._request_stop(solve=False)
            return
        self.get_logger().info(
            f'Bootstrap solve: {res.train_t_mm:.2f} mm / {res.train_r_deg:.2f}° over '
            f'{int(res.inliers.sum())}/{n} samples; X translation '
            f'{np.round(res.X[:3, 3], 3).tolist()} m')
        if res.train_t_mm > p['bootstrap_max_error_mm']:
            self.get_logger().error(
                f'Bootstrap error {res.train_t_mm:.1f} mm > {p["bootstrap_max_error_mm"]} mm: '
                f'not trusting it to aim the orbit.')
            self._request_stop(solve=False)
            return

        # The IK checks run a few per tick in PLAN, so the 200 Hz command
        # keeps flowing while the arm holds the last bootstrap pose.
        self._plan = {
            'X_inv': hes.inv_T(res.X),
            'T_BC0': self._T_BE0 @ res.X,
            'p_tag': res.Y[:3, 3],
            'candidates': hes.sample_orbit_candidates(
                res.Y[:3, 3], self._T_BE0 @ res.X, p['orbit_candidates'],
                radius_range=(p['orbit_radius_min'], p['orbit_radius_max']),
                max_tilt_deg=p['orbit_max_tilt_deg'],
                azimuth_half_range_deg=p['orbit_azimuth_half_range_deg'],
                max_roll_deg=p['orbit_max_roll_deg']),
            'feasible': [],
            'i': 0,
            't0': _time.monotonic(),
        }
        self._set_state('PLAN', t)
        self.get_logger().info(
            f'Planning the orbit: IK-checking {p["orbit_candidates"]} candidate views '
            f'(arm holds still) …')

    def _plan_step(self, T_now: np.ndarray, t: float) -> bool:
        """Check candidates for one tick's budget. True once the orbit starts."""
        p, pl = self._p, self._plan
        deadline = _time.monotonic() + _PLAN_BUDGET_S
        cands = pl['candidates']
        while pl['i'] < len(cands):
            T_BC = cands[pl['i']]
            pl['i'] += 1
            if self._ee_pose_ok(T_BC @ pl['X_inv']):
                pl['feasible'].append(T_BC)
            if _time.monotonic() > deadline:
                break
        if pl['i'] < len(cands):
            return False

        poses = hes.select_diverse_poses(pl['feasible'], pl['T_BC0'], p['orbit_samples'])
        self.get_logger().info(
            f'Orbit planned in {_time.monotonic() - pl["t0"]:.1f} s: '
            f'{len(pl["feasible"])}/{len(cands)} candidates feasible, {len(poses)} selected.')
        if not poses:
            self.get_logger().error('No orbit pose passes the workspace/IK checks — '
                                    'solving with the bootstrap samples only.')
            self._request_stop(solve=True)
            return True
        self._phase = 'orbit'
        self._phase_pose_no = 0
        self._n_orbit_total = len(poses)
        self._queue = [T_BC @ pl['X_inv'] for T_BC in poses]
        self.get_logger().info(
            f'Phase 2/2 ORBIT: {len(self._queue)} look-at poses around the tag at '
            f'{np.round(pl["p_tag"], 3).tolist()} m '
            f'({self._n_boot_total + len(self._queue)} poses total).')
        self._plan = None
        self._next_goal(T_now, t)
        return True

    def _request_stop(self, solve: bool) -> None:
        if getattr(self, '_state', None) == 'STOPPING':
            return
        self._stop_solve = solve
        self._state = 'STOPPING'

    def request_stop(self) -> None:
        """Ctrl-C (run_node_main): stop the arm, then solve what was collected."""
        if not hasattr(self, '_timer'):
            self.done = True
            return
        self.get_logger().info('Interrupted — stopping the arm, then solving.')
        self._request_stop(solve=len(self._samples_BE) >= 8)

    def _finalise(self) -> None:
        self.get_logger().info(
            f'Motion stopped: {len(self._samples_BE)} samples from {self._pose_no} poses, '
            f'{self._n_skipped} skipped.')
        self._save_dataset()
        if self._stop_solve:
            self._solve_and_write()
        self.done = True

    # ══════════════════════════════════════════════════════════════════
    # Files
    # ══════════════════════════════════════════════════════════════════

    def _config_dirs(self) -> List[str]:
        """Where to write: explicit output_dir, else source tree + install."""
        if self._p['output_dir']:
            return [self._p['output_dir']]
        dirs = []
        candidates = [_WS_CONFIG_DIR,
                      os.path.join(os.path.dirname(__file__), '..', '..', 'config')]
        try:
            from ament_index_python.packages import get_package_share_directory
            candidates.append(os.path.join(
                get_package_share_directory('franka_experiments'), 'config'))
        except Exception:
            pass
        for d in candidates:
            rd = os.path.realpath(d)
            if os.path.isdir(rd) and rd not in dirs:
                dirs.append(rd)
        return dirs

    def _dataset_path_for_read(self) -> Optional[str]:
        f = self._p['dataset_file']
        if os.path.isabs(f):
            return f if os.path.isfile(f) else None
        for d in self._config_dirs():
            if os.path.isfile(os.path.join(d, f)):
                return os.path.join(d, f)
        return None

    def _save_dataset(self) -> None:
        if not self._samples_BE:
            return
        p = self._p
        data = {
            'created': datetime.datetime.now().isoformat(timespec='seconds'),
            'setup': 'eye_in_hand',
            'base_frame': p['base_frame'], 'tool_frame': p['tool_frame'],
            'camera_frame': p['camera_frame'], 'camera_link_frame': p['camera_link_frame'],
            'tag_frame': p['tag_frame'],
            'T_link_optical': (None if self._T_link_optical is None
                               else self._T_link_optical.tolist()),
            'samples': [{'phase': ph, 'T_base_tool': A.tolist(), 'T_cam_tag': B.tolist()}
                        for ph, A, B in zip(self._sample_phase, self._samples_BE,
                                            self._samples_CT)],
        }
        f = p['dataset_file']
        paths = [f] if os.path.isabs(f) else [os.path.join(d, f) for d in self._config_dirs()[:1]]
        for path in paths:
            try:
                with open(path, 'w') as fh:
                    yaml.safe_dump(data, fh, default_flow_style=None, sort_keys=False)
                self.get_logger().info(f'Dataset saved → {path} ({len(self._samples_BE)} samples)')
            except OSError as exc:
                self.get_logger().error(f'Could not save dataset {path}: {exc}')

    def _run_solve_only(self) -> None:
        path = self._dataset_path_for_read()
        if path is None:
            self.get_logger().error(f'Dataset {self._p["dataset_file"]} not found in '
                                    f'{self._config_dirs()}')
            return
        with open(path) as fh:
            data = yaml.safe_load(fh)
        for s in data.get('samples', []):
            self._samples_BE.append(np.array(s['T_base_tool'], dtype=float))
            self._samples_CT.append(np.array(s['T_cam_tag'], dtype=float))
            self._sample_phase.append(s.get('phase', ''))
        if data.get('T_link_optical') is not None:
            self._T_link_optical = np.array(data['T_link_optical'], dtype=float)
        self.get_logger().info(f'Loaded {len(self._samples_BE)} samples from {path}')
        self._solve_and_write()

    def _solve_and_write(self) -> None:
        p = self._p
        log = self.get_logger()
        n = len(self._samples_BE)
        try:
            res = hes.solve(self._samples_BE, self._samples_CT,
                            validation_ratio=p['validation_ratio'])
        except ValueError as exc:
            log.error(f'Solve failed with {n} samples: {exc}. Nothing written.')
            return

        t_mm, r_deg, source = res.verdict_errors()
        ok = t_mm < p['accept_translation_mm'] and r_deg < p['accept_rotation_deg']
        X, Y = res.X, res.Y
        (tx, ty, tz), q = X[:3, 3], _quat_dict(X[:3, :3])

        log.info('')
        log.info('=' * 64)
        log.info('EYE-IN-HAND CALIBRATION RESULT')
        log.info('=' * 64)
        log.info(f'Samples: {n} collected, {int(res.inliers.sum())} inliers, '
                 f'{len(res.train)} train / {len(res.test)} test')
        log.info(f'Rotation-axis diversity σ = {np.round(res.axis_sigmas, 3).tolist()}'
                 + ('  ⚠ poorly conditioned' if res.axis_sigmas.min() < 0.15 else ''))
        log.info(f'Train error: {res.train_t_mm:.2f} mm / {res.train_r_deg:.3f}°')
        if res.test_t_mm is not None:
            log.info(f'Test error : {res.test_t_mm:.2f} mm / {res.test_r_deg:.3f}°')
        log.info(f'X = {p["tool_frame"]} → {p["camera_frame"]}')
        log.info(f'  translation [m] : [{tx:.6f}, {ty:.6f}, {tz:.6f}]')
        log.info(f'  quaternion xyzw : [{q["x"]}, {q["y"]}, {q["z"]}, {q["w"]}]')
        log.info(f'Y = {p["base_frame"]} → {p["tag_frame"]}: '
                 f'{np.round(Y[:3, 3], 4).tolist()} m')
        log.info(f'Verdict ({source} set, limits {p["accept_translation_mm"]} mm / '
                 f'{p["accept_rotation_deg"]}°): {"ACCEPTABLE" if ok else "NOT ACCEPTABLE"}')

        if not ok:
            log.warn(f'{p["output_file"]} NOT written; the previous file (if any) is kept. '
                     f'Re-solve the saved dataset with solve_only:=true (and looser '
                     f'accept_* if you judge the numbers good enough), or repeat the run.')
            return

        out = _transform_dict(X, p['tool_frame'], p['camera_frame'])
        if self._T_link_optical is not None:
            out['camera_link'] = _transform_dict(
                X @ hes.inv_T(self._T_link_optical), p['tool_frame'], p['camera_link_frame'])
        else:
            log.warn(f'Driver TF {p["camera_link_frame"]} → {p["camera_frame"]} never seen: '
                     f'camera_link section omitted.')
        out['calibration'] = {
            'stamp': datetime.datetime.now().isoformat(timespec='seconds'),
            'method': 'eye-in-hand, T_BE·X·T_CT = Y (handeye_eye_in_hand_node)',
            'samples_collected': n,
            'samples_used': int(res.inliers.sum()),
            'error_source': source,
            'translation_error_mm': round(float(t_mm), 3),
            'rotation_error_deg': round(float(r_deg), 4),
            'train_translation_error_mm': round(res.train_t_mm, 3),
            'train_rotation_error_deg': round(res.train_r_deg, 4),
            'rotation_axis_sigmas': [round(float(s), 3) for s in res.axis_sigmas],
            'tag_in_base': _transform_dict(Y, p['base_frame'], p['tag_frame']),
        }
        for d in self._config_dirs():
            path = os.path.join(d, p['output_file'])
            try:
                with open(path, 'w') as fh:
                    fh.write('# Eye-in-hand extrinsic of the D405: flange (fr3_link8) → camera.\n'
                             '# Written by handeye_eye_in_hand_node — do not edit by hand.\n')
                    yaml.safe_dump(out, fh, default_flow_style=False, sort_keys=False)
                log.info(f'YAML written → {path}')
            except OSError as exc:
                log.error(f'Could not write {path}: {exc}')


def main(args=None):
    run_node_main(HandEyeEyeInHandNode, args=args)


if __name__ == '__main__':
    main()
