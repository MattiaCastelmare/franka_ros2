#!/usr/bin/env python3
"""Pentagon trajectory for cbf_torque_controller — no CBF filter.

Unified phase-variable design
─────────────────────────────
There is no APPROACH/TRACK state machine.  A single dimensionless phase
``s`` drives one continuous geometric path:

    s, s_dot, s_ddot = timing.step(dt)      # how fast       (TimingLaw)
    p_d = path.position(s)                   # where          (GeometricPath)
    v_d = path.velocity(s, s_dot)            # = P'(s)·s_dot
    a_d = path.acceleration(s, s_dot, s_ddot)# = P''(s)·s_dot² + P'(s)·s_ddot

The path is a :class:`CompositePath`:

  * phase ``s ∈ [0, S_a)`` — straight-line approach lead-in whose ``P(0)`` is the
    *current EE position* captured at start-up (no position jump, no impulsive
    command), running into
  * phase ``s ≥ S_a``       — the cyclic :class:`PentagonPath` (C¹ Bézier corner
    blending), one loop per ``Δs = 1``.

Initial smoothing lives in the timing law (a C² soft-start ramps ``s_dot`` from
0), not in a separate state.  A constant phase rate ``1/cycle_time`` after the
ramp reproduces the original constant-speed pentagon exactly.

The feedback controller is task-space PD + adaptive damped-LS pseudo-inverse
with a continuous regularisation floor, augmented by a projected null-space
posture task (``N = I − J⁺J``) that damps redundant internal motion.

The ``qddot → dq_d → q_d`` reference is produced by a *bounded* integrator: the
double integration is coupled to the measured joint state (``k_sync_pos`` /
``k_sync_vel``) and hard-limited (``q_des_max_error`` / ``dq_des_max``) so the
reference can never drift away from the robot, plus a two-level anti-windup
(soft blend / hard snap) on the Cartesian error.

Safety: qddot output is clamped to ±qddot_max per joint at every tick; per-joint
saturation ratios are logged for offline actuator-limit analysis.
"""

from __future__ import annotations

import csv
import math
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import numpy as np
import pinocchio as pin
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from franka_experiments.utils.constants import FR3_JOINT_NAMES, NUM_JOINTS, AUTO_SENTINEL
from franka_experiments.utils.ros import get_namespace_from_config, run_node_main
from franka_experiments.utils.cbf_utils import load_robot_config
from franka_experiments.utils.kinematics import (
    generate_urdf_from_xacro,
    load_pinocchio_model,
    resolve_frame_id,
    resolve_arm_joint_ids,
    transform_ee_to_frame,
    so3_log,
)
from sensor_msgs.msg import JointState as SensorJointState
from franka_experiments.utils.trajectory import (
    PentagonTrajectory, PentagonPath, CircularPath, LissajousPath,
    LinearSegmentPath, CompositePath,
)
from franka_experiments.utils.timing_law import (
    LinearTimingLaw, ExponentialTimingLaw, TrapezoidalTimingLaw,
)
from franka_experiments.utils.logging_utils import ThrottledLogger, vec_to_str


class PentagonQddotCommander(Node):

    def __init__(self):
        super().__init__('pentagon_qddot_commander')

        self.done       = False
        self._stopping  = False
        self._stop_end  = 0.0
        self._running   = False    # False = warm-up (zeros); True = phase-driven

        # ── Load robot config (topics + per-joint limits) ────────────────
        _cfg    = load_robot_config('control')
        _topics = _cfg['topics']
        _limits = _cfg['joint_limits']
        _jnames = [f'joint{i}' for i in range(1, 8)]

        # ── Parameters ───────────────────────────────────────────────────
        # qddot_safe_topic: default reads qddot_nom from fr3_control.yaml so
        # the node connects directly to qddot_to_torque without a launch override.
        # When cbf_safety_filter is inserted, point this to topics['qddot_nom']
        # and let the CBF filter publish from qddot_nom to qddot_safe.
        self.declare_parameter('qddot_safe_topic',  _topics.get('qddot_nom', '/NS_1/qddot_nom'))
        self.declare_parameter('q_des_topic',       '/NS_1/q_des_state')
        self.declare_parameter('reset_thr_m',       0.10)
        self.declare_parameter('joint_state_topic', AUTO_SENTINEL)
        self.declare_parameter('ee_frame',          'fr3_hand_tcp')
        self.declare_parameter('rate_hz',           100.0)
        self.declare_parameter('warmup_s',            3.0)
        # Unified phase timing.  approach_time = wall-clock duration of the
        # straight-line lead-in (current EE → pentagon start), realised as the
        # soft-start window of the timing law.  timing_law selects how the phase
        # advances: 'linear' (constant rate + C² soft start, recommended for the
        # cyclic pentagon), 'trapezoidal' (constant-accel ramp then cruise), or
        # 'exponential' (one-shot ease — does not cycle).
        self.declare_parameter('approach_time',       2.5)
        self.declare_parameter('timing_law',          'linear')
        self.declare_parameter('center_xyz',          [0.4, 0.0, 0.4])
        self.declare_parameter('radius',              0.30)
        # Path shape:  'circle' (default, C∞ analytic — smooth, no accel jumps)
        # | 'lissajous' (C∞ analytic) | 'pentagon' (C¹ corner-blended: feedforward
        # acceleration STEPS at every line↔blend junction → jerky; legacy only).
        self.declare_parameter('path_type',           'cricle')
        self.declare_parameter('plane',               'xy')
        self.declare_parameter('plane_frame',         'fr3_link0')
        self.declare_parameter('cycle_time',          10.0)
        self.declare_parameter('smoothness',          0.20)
        # Lissajous amplitudes [m] (used only when path_type='lissajous')
        self.declare_parameter('lissajous_a',         0.12)
        self.declare_parameter('lissajous_b',         0.08)
        # kp/kd: task-space Cartesian gains [N/m, N·s/m].
        # Conservative for torque control — overshoot propagates via M·q̈.
        # kd ≈ 2√kp for near-critical damping.
        self.declare_parameter('kp_cart',             20.0)
        self.declare_parameter('kd_cart',              9.0)
        # kp_rot temporaneamente ridotto 10.0 → 5.0 per il PRIMO test con
        # l'anello di orientamento ora effettivamente regolante: prima la
        # formula di e_rot era in frame/segno errati → loop di fatto
        # divergente, quindi nessun comportamento osservato finora è
        # indicativo del loop chiuso correttamente. Da rivalutare (probabile
        # ritorno verso ~10) dopo aver osservato il transitorio corretto su
        # hardware. kd_rot invariato: con kp_rot=5 dà ζ=kd/(2√kp)≈1.34.
        self.declare_parameter('kp_rot',               5.0)
        self.declare_parameter('kd_rot',               6.0)
        # High-rate CSV logging: output directory + measured-effort source topic.
        self.declare_parameter('log_dir',
                               '/ros2_ws/src/franka_experiments/franka_logs')
        self.declare_parameter('joint_effort_topic', '/NS_1/joint_states')

        # ── Robustness / stability parameters (control-theoretic review) ──
        # Bounded reference integrator: couple q_d/dq_d to the measured state
        # so the double integration cannot drift without bound.
        self.declare_parameter('k_sync_pos',       2.0)
        self.declare_parameter('k_sync_vel',       5.0)
        # Two-level anti-windup on Cartesian error: soft → blend, hard → snap.
        self.declare_parameter('soft_reset_thr',   0.02)
        self.declare_parameter('hard_reset_thr',   0.05)
        self.declare_parameter('soft_reset_alpha', 0.95)
        # Null-space secondary task: posture regulation toward the start pose.
        self.declare_parameter('k_null',           5.0)
        self.declare_parameter('d_null',           2.0)
        # Damped least-squares regularisation (continuous, with a real floor).
        self.declare_parameter('lambda_sq_min',    1e-4)
        self.declare_parameter('lambda_sq_max',    5e-2)
        self.declare_parameter('manip_thr',        0.05)
        # Reference sanity limits: keep integrated references physical.
        self.declare_parameter('q_des_max_error',  0.5)   # rad, |q_d − q|
        self.declare_parameter('dq_des_max',       2.0)   # rad/s
        # Low-pass on the *measured* joint velocity, logging only (real signal,
        # lightly smoothed):  dq_filt += alpha·(dq_meas − dq_filt).
        self.declare_parameter('dq_filter_alpha',  0.2)

        # ── Sequential Joint Isolation Test (TEMPORARY diagnostic) ────────
        # When isolation_test=True the Cartesian pentagon/circle trajectory is
        # bypassed: each arm joint in turn performs a single-joint sinusoid for
        # isolation_seg_s seconds while every OTHER joint is held at its home
        # value with strictly zero velocity/acceleration.  The sequence runs
        # joint1→joint7 (indices 0→6), then the node stops.  Purpose: confirm
        # from the CSV that a commanded joint index actually moves the matching
        # physical joint (diagnoses cross-mapping / unresponsive joints 6 & 7).
        #
        # Profile = raised-cosine bump (C¹, zero velocity at every window edge):
        #   q   = q_home + 0.5·A·(1 − cos(ω·τ))
        #   q̇  =          0.5·A·ω·sin(ω·τ)
        #   q̈  =          0.5·A·ω²·cos(ω·τ)
        # with ω = 2π·isolation_freq_hz.  At 1 Hz the active joint runs exactly
        # 2 smooth out-and-back cycles per 2 s window.  A vigorous A=0.4 rad
        # (~23°) makes the motion physically unmistakable.  An isolation-only
        # joint-space PD (isolation_kp / isolation_kd) is added on top of the
        # feedforward q̈ to overcome FR3 wrist static friction.
        self.declare_parameter('isolation_test',    False)
        self.declare_parameter('isolation_amp',     0.4)   # rad, bump amplitude
        self.declare_parameter('isolation_seg_s',   2.0)   # s dedicated to each joint
        self.declare_parameter('isolation_freq_hz', 1.0)   # Hz (1.0 → 2 cycles/window)
        self.declare_parameter('isolation_kp',      40.0)  # joint-space P gain
        self.declare_parameter('isolation_kd',      12.0)  # joint-space D gain

        qddot_topic    = self.get_parameter('qddot_safe_topic').value
        q_des_topic    = self.get_parameter('q_des_topic').value
        self.reset_thr = float(self.get_parameter('reset_thr_m').value)
        js_topic_param = self.get_parameter('joint_state_topic').value
        ee_frame_name  = self.get_parameter('ee_frame').value
        self.rate_hz   = float(self.get_parameter('rate_hz').value)
        self.warmup_s  = float(self.get_parameter('warmup_s').value)
        self.approach_time = float(self.get_parameter('approach_time').value)
        self.timing_law_name = str(self.get_parameter('timing_law').value).lower()
        center_xyz     = list(self.get_parameter('center_xyz').value)
        radius         = float(self.get_parameter('radius').value)
        path_type      = str(self.get_parameter('path_type').value).lower()
        plane          = self.get_parameter('plane').value
        plane_frame    = self.get_parameter('plane_frame').value
        self.cycle_time = float(self.get_parameter('cycle_time').value)
        smoothness     = float(self.get_parameter('smoothness').value)
        lissajous_a    = float(self.get_parameter('lissajous_a').value)
        lissajous_b    = float(self.get_parameter('lissajous_b').value)
        self.kp        = float(self.get_parameter('kp_cart').value)
        self.kd        = float(self.get_parameter('kd_cart').value)
        self.kp_rot    = float(self.get_parameter('kp_rot').value)
        self.kd_rot    = float(self.get_parameter('kd_rot').value)
        self._log_dir  = str(self.get_parameter('log_dir').value)
        effort_topic   = str(self.get_parameter('joint_effort_topic').value)
        self.k_sync_pos       = float(self.get_parameter('k_sync_pos').value)
        self.k_sync_vel       = float(self.get_parameter('k_sync_vel').value)
        self.soft_reset_thr   = float(self.get_parameter('soft_reset_thr').value)
        self.hard_reset_thr   = float(self.get_parameter('hard_reset_thr').value)
        self.soft_reset_alpha = float(self.get_parameter('soft_reset_alpha').value)
        self.k_null           = float(self.get_parameter('k_null').value)
        self.d_null           = float(self.get_parameter('d_null').value)
        self._lambda_sq_min   = float(self.get_parameter('lambda_sq_min').value)
        self._lambda_sq_max   = float(self.get_parameter('lambda_sq_max').value)
        self._manip_thr       = float(self.get_parameter('manip_thr').value)
        self.q_des_max_error  = float(self.get_parameter('q_des_max_error').value)
        self.dq_des_max       = float(self.get_parameter('dq_des_max').value)
        self._dq_filter_alpha = float(self.get_parameter('dq_filter_alpha').value)
        self.isolation_test   = bool(self.get_parameter('isolation_test').value)
        self.iso_amp          = float(self.get_parameter('isolation_amp').value)
        self.iso_seg_s        = float(self.get_parameter('isolation_seg_s').value)
        self.iso_freq_hz      = float(self.get_parameter('isolation_freq_hz').value)
        self.iso_kp           = float(self.get_parameter('isolation_kp').value)
        self.iso_kd           = float(self.get_parameter('isolation_kd').value)
        self._dt       = 1.0 / self.rate_hz

        # Per-joint q̈ clamps from fr3_control.yaml joint_limits column [3].
        # Matches franka_description deceleration_limit: [6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0]
        self.qddot_max = np.array([_limits[j][3] for j in _jnames], dtype=np.float64)

        # ── Pinocchio ────────────────────────────────────────────────────
        self.get_logger().info('Generating URDF …')
        try:
            urdf_xml = generate_urdf_from_xacro()
        except Exception as exc:
            self.get_logger().error(f'URDF generation failed: {exc}')
            raise SystemExit(1) from exc

        self.pin_model, self.pin_data = load_pinocchio_model(urdf_xml)
        # Separate Data for the independent open-loop nominal channel so its
        # kinematics never clobber the measured-state pin_data (also used by
        # _compute_tau_des for M and nle).
        self.pin_data_nom = self.pin_model.createData()

        try:
            self.ee_frame_id = resolve_frame_id(self.pin_model, ee_frame_name)
        except RuntimeError as exc:
            self.get_logger().error(str(exc)); raise SystemExit(1) from exc

        self._use_plane_frame = (plane == 'front')
        self._plane_frame_id  = -1
        if self._use_plane_frame:
            try:
                self._plane_frame_id = resolve_frame_id(self.pin_model, plane_frame)
            except RuntimeError as exc:
                self.get_logger().error(str(exc)); raise SystemExit(1) from exc

        try:
            self._pin_joint_ids = resolve_arm_joint_ids(self.pin_model)
        except RuntimeError as exc:
            self.get_logger().error(str(exc)); raise SystemExit(1) from exc

        self._arm_v_ids = [self.pin_model.joints[p].idx_v for p in self._pin_joint_ids]

        # ── Geometric path (pure geometry, immutable) ─────────────────────
        # Phase-parametrised cyclic path selected by path_type.  'circle' and
        # 'lissajous' compute their derivatives analytically (C∞, smooth/constant
        # curvature → the feedforward acceleration a_d never jumps).  'pentagon'
        # is corner-blended C¹ only: a_d steps at the 10 line↔blend junctions per
        # loop, which makes the robot jerk — kept for legacy, not recommended.
        # The runtime approach lead-in + composite path are built in
        # _start_trajectory() once the current EE position is known.
        center_arr = np.array(center_xyz)
        self.traj = None
        if path_type == 'pentagon':
            self.traj = PentagonTrajectory(
                center=center_arr, radius=radius,
                plane=plane, cycle_time=self.cycle_time, smoothness=smoothness,
            )
            self._cycle_path = PentagonPath(self.traj)
            self.get_logger().warn(
                "path_type='pentagon' is only C¹ (corner-blended): the "
                'feedforward acceleration steps at each corner → jerky motion. '
                "Use 'circle' or 'lissajous' for smooth tracking.")
        elif path_type == 'lissajous':
            self._cycle_path = LissajousPath(
                center=center_arr, A=lissajous_a, B=lissajous_b, plane=plane,
            )
        else:
            if path_type != 'circle':
                self.get_logger().warn(
                    f'Unknown path_type "{path_type}" → using circle')
            self._cycle_path = CircularPath(
                center=center_arr, radius=radius, plane=plane,
            )
        # Start point = curve entry P(0) (continuous approach→curve seam).
        self._start_xyz = self._cycle_path.position(0.0).copy()

        # Phase variable + path/timing, assembled in _start_trajectory()
        self._path: Optional[CompositePath] = None
        self._timing = None
        self._approach_span = 0.0

        # ── Pre-allocated buffers ─────────────────────────────────────────
        nv = self.pin_model.nv
        self._q_neutral      = pin.neutral(self.pin_model)
        self._q_full         = pin.neutral(self.pin_model)
        self._qdot_full      = np.zeros(nv)
        self._J6n            = np.zeros((6, nv))
        self._dJ6n           = np.zeros((6, nv))
        self._J_arm          = np.zeros((6, NUM_JOINTS))
        self._dJ_arm         = np.zeros((6, NUM_JOINTS))
        self._J_rot_tmp      = np.zeros((3, NUM_JOINTS))
        self._p_ee           = np.zeros(3)
        self._R_des          = np.eye(3)
        self._R_err          = np.zeros((3, 3))
        self._e_rot          = np.zeros(3)
        self._e6             = np.zeros(6)
        self._edot6          = np.zeros(6)
        self._xddot6         = np.zeros(6)
        self._tmp3           = np.zeros(3)
        self._tmp6           = np.zeros(6)
        self._JJT            = np.zeros((6, 6))
        self._JJT_reg        = np.zeros((6, 6))
        self._J_pinv         = np.zeros((NUM_JOINTS, 6))
        self._q_ddot         = np.zeros(NUM_JOINTS)
        # Adaptive damped-LS state (limits come from the ROS params above).
        self._lambda_sq      = self._lambda_sq_min
        self._orient_ok      = False

        # ── Null-space posture control (Change 3) ─────────────────────────
        self._I7             = np.eye(NUM_JOINTS)
        self._N              = np.zeros((NUM_JOINTS, NUM_JOINTS))   # I − J⁺J
        self._JpinvJ         = np.zeros((NUM_JOINTS, NUM_JOINTS))
        self._qddot_task     = np.zeros(NUM_JOINTS)   # task-space solution
        self._qddot_null     = np.zeros(NUM_JOINTS)   # raw secondary objective
        self._null_proj      = np.zeros(NUM_JOINTS)   # N·qddot_null (added term)
        self._q_home         = np.zeros(NUM_JOINTS)   # captured at start-up
        self._tmp7           = np.zeros(NUM_JOINTS)

        # ── Bounded-integrator / reference-limit scratch (Changes 1 & 6) ──
        self._sync_vel       = np.zeros(NUM_JOINTS)
        self._sync_pos       = np.zeros(NUM_JOINTS)
        self._q_lo           = np.zeros(NUM_JOINTS)
        self._q_hi           = np.zeros(NUM_JOINTS)

        # ── Saturation monitor + extended diagnostics (Changes 5 & 8) ─────
        self._sat_ratio            = np.zeros(NUM_JOINTS)
        self._diag_max_sat         = 0.0
        self._diag_num_sat         = 0
        self._diag_qddot_norm      = 0.0
        self._diag_qddot_task_norm = 0.0
        self._diag_qddot_null_norm = 0.0
        self._diag_sync_pos_norm   = 0.0
        self._diag_sync_vel_norm   = 0.0

        # Actual-dt tracking — compensates Python timer jitter in integration
        self._prev_tick_time = None

        # Isolation-test run clock (set on the first running tick) + done latch
        self._iso_t0   = None
        self._iso_done = False
        # Isolation scratch: PD-augmented command + joint-space tracking errors
        self._qddot_cmd = np.zeros(NUM_JOINTS)
        self._iso_e_q   = np.zeros(NUM_JOINTS)
        self._iso_e_dq  = np.zeros(NUM_JOINTS)

        # Joint-space integration buffers for q_d, dq_d
        self._q_d            = np.zeros(NUM_JOINTS)
        self._dq_d           = np.zeros(NUM_JOINTS)

        # ── Independent open-loop NOMINAL channel (ideal trajectory) ──────
        # Mirrors the task + null-space accel law but on its OWN state and a
        # separate pin_data; pure ∫∫ (no sync / clamp-to-measured / reset), so
        # it never reads the robot → the trajectory the arm WOULD follow if the
        # CBF never deviated it.  Seeded at the measured start in _start_trajectory.
        self._q_nom          = np.zeros(NUM_JOINTS)
        self._dq_nom         = np.zeros(NUM_JOINTS)
        self._qddot_nom      = np.zeros(NUM_JOINTS)
        self._q_nom_full     = pin.neutral(self.pin_model)
        self._qdot_nom_full  = np.zeros(nv)
        self._J6n_nom        = np.zeros((6, nv))
        self._dJ6n_nom       = np.zeros((6, nv))
        self._Jn_arm         = np.zeros((6, NUM_JOINTS))
        self._dJn_arm        = np.zeros((6, NUM_JOINTS))
        self._Jn_rot_tmp     = np.zeros((3, NUM_JOINTS))
        self._p_ee_nom       = np.zeros(3)
        self._R_des_nom      = np.eye(3)
        self._R_err_nom      = np.zeros((3, 3))
        self._e_rot_nom      = np.zeros(3)
        self._en6            = np.zeros(6)
        self._edotn6         = np.zeros(6)
        self._xddotn6        = np.zeros(6)
        self._tmp3n          = np.zeros(3)
        self._tmp6n          = np.zeros(6)
        self._tmp7n          = np.zeros(NUM_JOINTS)
        self._JJTn           = np.zeros((6, 6))
        self._JJTn_reg       = np.zeros((6, 6))
        self._Jn_pinv        = np.zeros((NUM_JOINTS, 6))
        self._qddot_task_nom = np.zeros(NUM_JOINTS)
        self._qddot_null_nom = np.zeros(NUM_JOINTS)
        self._null_proj_nom  = np.zeros(NUM_JOINTS)
        self._Nn             = np.zeros((NUM_JOINTS, NUM_JOINTS))
        self._JpinvJn        = np.zeros((NUM_JOINTS, NUM_JOINTS))
        self._lambda_sq_nom  = self._lambda_sq_min
        self._orient_nom_ok  = False

        # Low-pass-filtered measured velocity (real signal, logging only)
        self._dq_filt        = np.zeros(NUM_JOINTS)

        # ── Logging buffers (pre-allocated → no per-tick allocation) ──────
        # tau_des = M(q)·q̈_des + nle(q, q̇).  computeAllTerms() already fills
        # pin_data.M (upper triangle only) and pin_data.nle each tick, so we
        # only need scratch space for the matrix-vector product.
        self._qddot_full_des = np.zeros(nv)          # full-model q̈ (arm rows set)
        self._tau_full       = np.zeros(nv)          # full-model torque scratch
        self._tau_des        = np.zeros(NUM_JOINTS)  # desired torque (arm joints)
        self._tau_meas       = np.zeros(NUM_JOINTS)  # measured torque (from effort)
        # CRBA stores only the upper triangle of M; cache the lower-triangle
        # indices once so we can mirror it to a full symmetric matrix in-place.
        self._M_tril         = np.tril_indices(nv, -1)
        self._eff_imap: Optional[List[int]] = None   # effort name→index map

        # Pre-built JointState setpoint message (avoids allocation per tick)
        self._sp_msg         = SensorJointState()
        self._sp_msg.name    = list(FR3_JOINT_NAMES)
        self._sp_msg.position = [0.0] * NUM_JOINTS
        self._sp_msg.velocity = [0.0] * NUM_JOINTS
        self._sp_msg.effort   = [0.0] * NUM_JOINTS

        # Output
        self._out_msg        = Float64MultiArray()
        self._out_msg.data   = [0.0] * NUM_JOINTS
        self._zero_msg       = Float64MultiArray()
        self._zero_msg.data  = [0.0] * NUM_JOINTS

        # ── Joint state double buffer ─────────────────────────────────────
        self._js_lock  = threading.Lock()
        self._js_a     = {'q': np.zeros(NUM_JOINTS), 'qdot': np.zeros(NUM_JOINTS),
                          'q_full': pin.neutral(self.pin_model), 'valid': False}
        self._js_b     = {'q': np.zeros(NUM_JOINTS), 'qdot': np.zeros(NUM_JOINTS),
                          'q_full': pin.neutral(self.pin_model), 'valid': False}
        self._js_write = self._js_a
        self._js_read  = self._js_b
        self._js_stamp = self.get_clock().now()
        self._js_imap: Optional[List[int]] = None

        js_topic = js_topic_param
        if js_topic == AUTO_SENTINEL:
            ns = get_namespace_from_config()
            js_topic = f'/{ns}/joint_states' if ns else '/joint_states'

        self._js_sub = self.create_subscription(JointState, js_topic, self._js_cb, 10)

        # Measured-effort subscription (configurable; used only for logging)
        self._eff_sub = self.create_subscription(
            JointState, effort_topic, self._effort_cb, 10)

        # ── Publisher / timer ─────────────────────────────────────────────
        self.pub      = self.create_publisher(Float64MultiArray, qddot_topic, 10)
        self._sp_pub  = self.create_publisher(SensorJointState,  q_des_topic,  10)
        self.timer    = self.create_timer(self._dt, self._tick)
        self.t0    = self.get_clock().now()
        self._tlog = ThrottledLogger(self.get_logger())

        # High-rate CSV logger (opened once; closed on node destruction)
        self._init_logger(effort_topic)

        self.get_logger().info(
            f'pentagon_qddot_commander\n'
            f'  topic    : {qddot_topic}\n'
            f'  js_topic : {js_topic}\n'
            f'  center   : {center_xyz}  r={radius} m  plane={plane}\n'
            f'  start_xyz: {self._start_xyz.tolist()}\n'
            f'  cycle_t  : {self.cycle_time} s  Kp={self.kp} Kd={self.kd}\n'
            f'  timing   : {self.timing_law_name}  approach_time={self.approach_time} s\n'
            f'  qddot_max: {self.qddot_max.tolist()} rad/s²  (per-joint)')

    # ── Joint state callback ──────────────────────────────────────────────

    def _js_cb(self, msg: JointState) -> None:
        if self._js_imap is None:
            try:
                self._js_imap = [msg.name.index(jn) for jn in FR3_JOINT_NAMES]
            except ValueError:
                return
        if len(msg.position) <= max(self._js_imap):
            return
        buf = self._js_write
        for k, i in enumerate(self._js_imap):
            buf['q'][k]    = msg.position[i]
            buf['qdot'][k] = msg.velocity[i] if len(msg.velocity) > i else 0.0
        q_full = buf['q_full']
        np.copyto(q_full, self._q_neutral)
        for k, pid in enumerate(self._pin_joint_ids):
            q_full[self.pin_model.joints[pid].idx_q] = buf['q'][k]
        buf['valid'] = True
        with self._js_lock:
            self._js_write, self._js_read = self._js_read, self._js_write
        self._js_stamp = self.get_clock().now()

    # ── Measured effort callback (logging only) ───────────────────────────

    def _effort_cb(self, msg: JointState) -> None:
        """Cache measured joint torques (msg.effort) for the next log row."""
        if self._eff_imap is None:
            try:
                self._eff_imap = [msg.name.index(jn) for jn in FR3_JOINT_NAMES]
            except ValueError:
                return
        if len(msg.effort) <= max(self._eff_imap):
            return
        for k, i in enumerate(self._eff_imap):
            self._tau_meas[k] = msg.effort[i]

    # ── Stop ─────────────────────────────────────────────────────────────

    def request_stop(self, dur: float = 0.5):
        if self._stopping:
            return
        self._stopping = True
        self._stop_end = time.monotonic() + dur
        self.get_logger().info(f'Stopping: zero qddot for {dur} s')

    # ── Main tick ────────────────────────────────────────────────────────

    def _tick(self):
        if self._stopping:
            self.pub.publish(self._zero_msg)
            if time.monotonic() >= self._stop_end:
                self.timer.cancel()
                self.done = True
            return

        now_stamp = self.get_clock().now()
        t = (now_stamp - self.t0).nanoseconds * 1e-9

        # Measure actual elapsed time to compensate Python timer jitter.
        # Clamped to [0.5×dt, 2×dt] to avoid integration explosions on startup
        # or after a long pause.
        if self._prev_tick_time is not None:
            actual_dt = float((now_stamp - self._prev_tick_time).nanoseconds * 1e-9)
            actual_dt = float(np.clip(actual_dt, self._dt * 0.5, self._dt * 2.0))
        else:
            actual_dt = self._dt
        self._prev_tick_time = now_stamp

        # Warm-up: publish zeros until a valid joint state arrives, then build
        # the phase-driven path/timing rooted at the current EE position.
        if not self._running:
            self.pub.publish(self._zero_msg)
            if t >= self.warmup_s:
                with self._js_lock:
                    js = self._js_read
                if js['valid']:
                    self._start_trajectory(js)
            if self._tlog.due(t):
                self._tlog.info(f'[WARMUP {t:.1f}/{self.warmup_s}s]')
            return

        # Read joint state
        with self._js_lock:
            js = self._js_read
        if not js['valid']:
            self.pub.publish(self._zero_msg)
            return
        age = (self.get_clock().now() - self._js_stamp).nanoseconds * 1e-9
        if age > 0.1:
            self.pub.publish(self._zero_msg)
            if self._tlog.due(t):
                self.get_logger().warn(f'JS stale {age:.3f}s')
            return

        # Pinocchio kinematics
        np.copyto(self._q_full, js['q_full'])
        self._qdot_full[:] = 0.0
        for k, vid in enumerate(self._arm_v_ids):
            self._qdot_full[vid] = js['qdot'][k]
        qdot = js['qdot']

        # First-order low-pass of the measured velocity (real signal, logged
        # alongside the raw dq).  alpha∈(0,1]; smaller = smoother / more lag.
        self._dq_filt += self._dq_filter_alpha * (qdot - self._dq_filt)

        pin.computeAllTerms(self.pin_model, self.pin_data,
                            self._q_full, self._qdot_full)
        pin.updateFramePlacements(self.pin_model, self.pin_data)

        self._J6n[:] = pin.getFrameJacobian(
            self.pin_model, self.pin_data, self.ee_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        self._dJ6n[:] = pin.getFrameJacobianTimeVariation(
            self.pin_model, self.pin_data, self.ee_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        for i, vid in enumerate(self._arm_v_ids):
            self._J_arm[:, i]  = self._J6n[:, vid]
            self._dJ_arm[:, i] = self._dJ6n[:, vid]

        oMee  = self.pin_data.oMf[self.ee_frame_id]
        R_cur = np.asarray(oMee.rotation)

        if self._use_plane_frame:
            p_raw, _ = transform_ee_to_frame(
                self.pin_model, self.pin_data, self._plane_frame_id, oMee,
                self._J6n[:, self._arm_v_ids])
            np.copyto(self._p_ee, p_raw)
            R_ref = np.asarray(self.pin_data.oMf[self._plane_frame_id].rotation)
            np.dot(R_ref.T, self._J_arm[:3, :], out=self._J_rot_tmp)
            self._J_arm[:3, :] = self._J_rot_tmp
            np.dot(R_ref.T, self._dJ_arm[:3, :], out=self._J_rot_tmp)
            self._dJ_arm[:3, :] = self._J_rot_tmp
        else:
            np.copyto(self._p_ee, oMee.translation)

        # Orientation: capture once, hold throughout
        if not self._orient_ok:
            np.copyto(self._R_des, R_cur)
            self._orient_ok = True
        # Errore di orientamento = so3_log(R_des @ R_cur.T), espresso in WORLD
        # (coerente con J_arm[3:] in LOCAL_WORLD_ALIGNED) e con segno "des − cur"
        # (coerente con l'errore di posizione p_d − p_ee). Calcolato come
        # −R_des @ so3_log(R_des.T @ R_cur), riusando il buffer R_err già qui.
        # so3_log NON degenera per disallineamenti grandi (vedi kinematics.py):
        # la vecchia ½·vee(R_err−R_errᵀ) era in frame body, segno opposto, e con
        # norma sin(angle) non monotona → invertiva il gradiente del PD >90°.
        np.dot(self._R_des.T, R_cur, out=self._R_err)
        np.dot(self._R_des, so3_log(self._R_err), out=self._e_rot)
        np.negative(self._e_rot, out=self._e_rot)

        # ── TEMPORARY: Sequential Joint Isolation Test ────────────────────────
        # Bypass the whole Cartesian task-space trajectory + nominal integrator
        # and instead drive one joint at a time with a joint-space sinusoid.
        # _p_ee / dq_filt / computeAllTerms are already done above, so the CSV
        # (incl. the genuine measured q/dq and tau_des) stays fully populated.
        if self.isolation_test:
            self._tick_isolation(t, js, qdot)
            return

        # Unified timing law + geometric path — no APPROACH/TRACK branching.
        #   s, s_dot, s_ddot = timing.step(dt)
        #   p_d = P(s) ; v_d = P'(s)·s_dot ; a_d = P''(s)·s_dot² + P'(s)·s_ddot
        s, s_dot, s_ddot = self._timing.step(actual_dt)
        p_d = self._path.position(s)
        v_d = self._path.velocity(s, s_dot)
        a_d = self._path.acceleration(s, s_dot, s_ddot)

        # 6D error
        np.subtract(p_d, self._p_ee, out=self._tmp3)
        self._e6[:3] = self._tmp3
        self._e6[3:] = self._e_rot

        # 6D velocity error: [v_d - J_lin·qdot ; -J_ang·qdot]
        np.dot(self._J_arm, qdot, out=self._edot6)
        np.subtract(v_d, self._edot6[:3], out=self._edot6[:3])
        np.negative(self._edot6[3:], out=self._edot6[3:])

        # Task-space acceleration: xddot6 = a_d + Kp*e + Kd*edot
        self._xddot6[:3] = a_d
        np.multiply(self.kp,     self._e6[:3],    out=self._tmp3)
        self._xddot6[:3] += self._tmp3
        np.multiply(self.kd,     self._edot6[:3], out=self._tmp3)
        self._xddot6[:3] += self._tmp3
        np.multiply(self.kp_rot, self._e6[3:],    out=self._xddot6[3:])
        np.multiply(self.kd_rot, self._edot6[3:], out=self._tmp3)
        self._xddot6[3:] += self._tmp3

        # Adaptive damped LS with a continuous, non-zero floor (Change 4).
        #   λ² = λ_min + (λ_max − λ_min)·f(w),  f(w)=(1−w/w₀)² for w<w₀ else 0
        # Continuous at w=w₀ (f→0) and never below λ_min, so the pseudo-inverse
        # is always regularised — no near-undamped 1e-6 regime.
        np.dot(self._J_arm, self._J_arm.T, out=self._JJT)
        w = math.sqrt(max(0.0, float(np.linalg.det(self._JJT))))
        if w < self._manip_thr:
            f_w = (1.0 - w / self._manip_thr) ** 2
        else:
            f_w = 0.0
        self._lambda_sq = (self._lambda_sq_min
                           + (self._lambda_sq_max - self._lambda_sq_min) * f_w)
        np.copyto(self._JJT_reg, self._JJT)
        for i in range(6):
            self._JJT_reg[i, i] += self._lambda_sq

        # J⁺ = Jᵀ(JJᵀ+λ²I)⁻¹.  Avoid an explicit inverse (Change 7): A is SPD,
        # so solve A·X = J for X = A⁻¹J, then J⁺ = (A⁻¹J)ᵀ (A⁻¹ symmetric).
        self._J_pinv[:] = np.linalg.solve(self._JJT_reg, self._J_arm).T

        # Task-space solution: qddot_task = J⁺(ẍ_d − J̇·q̇)   (7,)
        np.dot(self._dJ_arm, qdot, out=self._tmp6)
        np.subtract(self._xddot6, self._tmp6, out=self._xddot6)
        np.dot(self._J_pinv, self._xddot6, out=self._qddot_task)

        # Null-space posture control (Change 3): N = I − J⁺J  (7×7).
        #   qddot_null = −k_null·(q − q_home) − d_null·q̇
        # Projected by N so it leaves the task acceleration untouched while
        # damping redundant internal motion (reduces wrist loading / drift).
        np.dot(self._J_pinv, self._J_arm, out=self._JpinvJ)     # (7×6)(6×7)=7×7
        np.subtract(self._I7, self._JpinvJ, out=self._N)
        np.subtract(js['q'], self._q_home, out=self._qddot_null)
        self._qddot_null *= -self.k_null
        np.multiply(self.d_null, qdot, out=self._tmp7)
        self._qddot_null -= self._tmp7
        np.dot(self._N, self._qddot_null, out=self._null_proj)

        # Total command: task + projected null-space term
        np.add(self._qddot_task, self._null_proj, out=self._q_ddot)
        self._diag_qddot_task_norm = float(np.linalg.norm(self._qddot_task))
        self._diag_qddot_null_norm = float(np.linalg.norm(self._null_proj))

        # Saturation monitor (Change 5): per-joint ratio BEFORE clipping.
        np.abs(self._q_ddot, out=self._sat_ratio)
        self._sat_ratio /= self.qddot_max
        self._diag_max_sat = float(self._sat_ratio.max())
        self._diag_num_sat = int(np.count_nonzero(self._sat_ratio > 0.95))

        # Safety clamp
        np.clip(self._q_ddot, -self.qddot_max, self.qddot_max, out=self._q_ddot)
        self._diag_qddot_norm = float(np.linalg.norm(self._q_ddot))

        # ── Bounded reference integration (Change 1) ─────────────────────────
        # Pure ∫∫qddot drifts indefinitely; couple the reference to the
        # measured state with first-order sync terms so it stays physical:
        #   dq_d += qddot·dt + k_sync_vel·(q̇ − dq_d)·dt
        #   q_d  += dq_d·dt  + k_sync_pos·(q  − q_d )·dt
        self._dq_d += self._q_ddot * actual_dt
        np.subtract(qdot, self._dq_d, out=self._sync_vel)
        self._sync_vel *= (self.k_sync_vel * actual_dt)
        self._dq_d += self._sync_vel
        # Reference velocity clamp (Change 6)
        np.clip(self._dq_d, -self.dq_des_max, self.dq_des_max, out=self._dq_d)

        self._q_d += self._dq_d * actual_dt
        np.subtract(js['q'], self._q_d, out=self._sync_pos)
        self._sync_pos *= (self.k_sync_pos * actual_dt)
        self._q_d += self._sync_pos
        # Reference position clamp around the measured config (Change 6)
        np.subtract(js['q'], self.q_des_max_error, out=self._q_lo)
        np.add(js['q'],      self.q_des_max_error, out=self._q_hi)
        np.clip(self._q_d, self._q_lo, self._q_hi, out=self._q_d)

        self._diag_sync_vel_norm = float(np.linalg.norm(self._sync_vel))
        self._diag_sync_pos_norm = float(np.linalg.norm(self._sync_pos))

        # ── Two-level anti-windup on Cartesian error (Change 2) ──────────────
        # hard_reset_thr > soft_reset_thr: snap on large error, blend on small.
        ee_err = float(np.linalg.norm(self._e6[:3]))
        if ee_err > self.hard_reset_thr:
            np.copyto(self._q_d,  js['q'])          # full synchronisation
            np.copyto(self._dq_d, qdot)
            if self._tlog.due(t):
                self.get_logger().warn(
                    f'HARD reset: EE error {ee_err:.3f} m > {self.hard_reset_thr} m')
        elif ee_err > self.soft_reset_thr:
            a = self.soft_reset_alpha               # blend toward measured state
            self._q_d  *= a; self._q_d  += (1.0 - a) * js['q']
            self._dq_d *= a; self._dq_d += (1.0 - a) * qdot
            if self._tlog.due(t):
                self.get_logger().info(
                    f'soft reset: EE error {ee_err:.3f} m > {self.soft_reset_thr} m '
                    f'(blend α={a})')

        # ── Publish Float64MultiArray (legacy / CBF filter path) ─────────────
        for i in range(NUM_JOINTS):
            self._out_msg.data[i] = float(self._q_ddot[i])
        self.pub.publish(self._out_msg)

        # ── Publish JointState setpoint (new C++ interface) ──────────────────
        self._sp_msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(NUM_JOINTS):
            self._sp_msg.position[i] = float(self._q_d[i])
            self._sp_msg.velocity[i] = float(self._dq_d[i])
            self._sp_msg.effort[i]   = float(self._q_ddot[i])
        self._sp_pub.publish(self._sp_msg)

        # ── Independent open-loop nominal integration (ideal; ignores robot) ──
        self._step_nominal(p_d, v_d, a_d, actual_dt)

        # ── High-rate CSV log (one row per tick; no-op if logger disabled) ───
        if self._csv_writer is not None:
            self._compute_tau_des()                 # fills self._tau_des
            self._log_data(t, js['q'], qdot, p_d, v_d, a_d,
                           s, s_dot, s_ddot, w, ee_err)

        if self._tlog.due(t):
            en = float(np.linalg.norm(self._e6[:3]))
            qn = float(np.linalg.norm(self._q_ddot))
            seg = 'APPROACH' if s < self._approach_span else 'TRACK'
            self._tlog.info(
                f'[{seg} s={s:.3f} t={t:.1f}s] '
                f'p=[{vec_to_str(self._p_ee)}] p_d=[{vec_to_str(p_d)}] '
                f'|e|={en:.4f} m  |qddot|={qn:.3f} rad/s²\n'
                f'        |null|={self._diag_qddot_null_norm:.3f} '
                f'max_sat={self._diag_max_sat:.2f} nsat={self._diag_num_sat} '
                f'sync_p={self._diag_sync_pos_norm:.4f} '
                f'sync_v={self._diag_sync_vel_norm:.4f} λ²={self._lambda_sq:.2e}')

    # ── Sequential Joint Isolation Test (TEMPORARY diagnostic) ──────────────

    def _tick_isolation(self, t: float, js: dict, qdot: np.ndarray) -> None:
        """Drive one joint at a time with a raised-cosine bump.

        Segment ``seg = ⌊(t − t0) / iso_seg_s⌋`` selects the active joint
        (0→6 ⇒ joint1→joint7).  The active joint follows the C¹ bump
            q   = q_home + 0.5·A·(1 − cos(ω·τ))
            q̇  =          0.5·A·ω·sin(ω·τ)
            q̈  =          0.5·A·ω²·cos(ω·τ)
        with ω = 2π·iso_freq_hz and τ the in-segment time.  q̇ = 0 at every
        window edge (τ = k/iso_freq_hz) ⇒ smooth handoffs between joints; at
        1 Hz the active joint runs exactly 2 out-and-back cycles per 2 s window.
        Every other joint is held strictly at its q_home with 0 velocity / 0
        acceleration.

        The desired reference (q_d, dq_d) and the nominal channel
        (q_nom, dq_nom, q̈_nom) carry this profile EXACTLY (feedforward only).
        The published command q̈ adds an isolation-only joint-space PD on top of
        the feedforward q̈ to overcome FR3 wrist static friction:
            q̈_cmd = q̈_ff + Kp·(q_d − q_meas) + Kd·(dq_d − dq_meas).

        The logged *real* q/dq stay genuine encoder feedback (js['q'],
        js['qdot'] from /joint_states) — never a software integral — so a CSV
        plot directly compares commanded-vs-measured per joint.
        """
        if self._iso_t0 is None:
            self._iso_t0 = t
        tau = t - self._iso_t0
        seg = int(tau // self.iso_seg_s)

        # Default for ALL joints: hold the captured home posture, zero motion.
        np.copyto(self._q_d, self._q_home)
        self._dq_d[:]      = 0.0
        self._qddot_nom[:] = 0.0            # feedforward q̈ profile (= q̈_des)

        if 0 <= seg < NUM_JOINTS:
            local_t = tau - seg * self.iso_seg_s
            w = 2.0 * math.pi * self.iso_freq_hz
            A = self.iso_amp
            c = math.cos(w * local_t)
            s = math.sin(w * local_t)
            # Only the active joint index `seg` is perturbed (raised-cosine bump).
            self._q_d[seg]      = self._q_home[seg] + 0.5 * A * (1.0 - c)
            self._dq_d[seg]     = 0.5 * A * w * s
            self._qddot_nom[seg] = 0.5 * A * w * w * c
            active = seg + 1                       # human-readable joint number
        else:
            active, local_t = -1, 0.0              # sequence finished
            if not self._iso_done:
                self._iso_done = True
                self.get_logger().info(
                    'Isolation sequence complete (joints 1-7 probed) — stopping.')
                self.request_stop()

        # Nominal (ideal) channel mirrors the profile EXACTLY — pure feedforward,
        # no PD term, no measured-state coupling (q̈_nom already set above).
        np.copyto(self._q_nom,  self._q_d)
        np.copyto(self._dq_nom, self._dq_d)

        # Published command = feedforward q̈ + joint-space PD (beats wrist
        # friction):  q̈_cmd = q̈_ff + Kp·(q_d − q_meas) + Kd·(dq_d − dq_meas).
        np.subtract(self._q_d,  js['q'], out=self._iso_e_q)
        np.subtract(self._dq_d, qdot,    out=self._iso_e_dq)
        np.copyto(self._qddot_cmd, self._qddot_nom)
        self._qddot_cmd += self.iso_kp * self._iso_e_q
        self._qddot_cmd += self.iso_kd * self._iso_e_dq
        # Same per-joint q̈ safety clamp as the normal command path.
        np.clip(self._qddot_cmd, -self.qddot_max, self.qddot_max,
                out=self._qddot_cmd)
        # q_ddot is the buffer the publish/log/tau_des path reads.
        np.copyto(self._q_ddot, self._qddot_cmd)

        # ── Publish: Float64MultiArray (qddot) + JointState setpoint ─────────
        for i in range(NUM_JOINTS):
            self._out_msg.data[i] = float(self._q_ddot[i])
        self.pub.publish(self._out_msg)

        self._sp_msg.header.stamp = self.get_clock().now().to_msg()
        for i in range(NUM_JOINTS):
            self._sp_msg.position[i] = float(self._q_d[i])
            self._sp_msg.velocity[i] = float(self._dq_d[i])
            self._sp_msg.effort[i]   = float(self._q_ddot[i])
        self._sp_pub.publish(self._sp_msg)

        # ── CSV log: real q/dq are genuine encoder feedback (js), not ∫ ──────
        # Cartesian columns are not meaningful here; report desired EE = measured
        # EE so the error columns read ~0 and never mask the joint comparison.
        self._e6[:] = 0.0
        zero3 = (0.0, 0.0, 0.0)
        if self._csv_writer is not None:
            self._compute_tau_des()                # fills self._tau_des
            self._log_data(t, js['q'], qdot, self._p_ee, zero3, zero3,
                           tau, 0.0, 0.0, 0.0, 0.0)

        if self._tlog.due(t):
            if active > 0:
                j = active - 1
                self._tlog.info(
                    f'[ISOLATION joint{active} t={t:.1f}s τ={local_t:.2f}s] '
                    f'q_des={self._q_d[j]:+.4f} q_meas={js["q"][j]:+.4f} '
                    f'dq_des={self._dq_d[j]:+.4f} dq_meas={qdot[j]:+.4f}')
            else:
                self._tlog.info(f'[ISOLATION complete t={t:.1f}s — holding home]')

    # ── Trajectory start-up (replaces the APPROACH/TRACK state machine) ─────

    def _current_ee_position(self, js: dict) -> np.ndarray:
        """FK of the measured joint state → EE position in the control frame."""
        np.copyto(self._q_full, js['q_full'])
        pin.forwardKinematics(self.pin_model, self.pin_data, self._q_full)
        pin.updateFramePlacements(self.pin_model, self.pin_data)
        oMee = self.pin_data.oMf[self.ee_frame_id]
        if self._use_plane_frame:
            p_raw, _ = transform_ee_to_frame(
                self.pin_model, self.pin_data, self._plane_frame_id, oMee,
                np.zeros((6, NUM_JOINTS)))
            return np.asarray(p_raw).copy()
        return np.asarray(oMee.translation).copy()

    def _build_timing_law(self):
        """Construct the selected timing law.  Steady phase rate = 1/cycle_time
        so a completed soft-start reproduces the original constant-speed pentagon."""
        rate = 1.0 / self.cycle_time
        name = self.timing_law_name
        if name == 'exponential':
            self.get_logger().warn(
                'timing_law=exponential is a one-shot ease (settles at the '
                'pentagon start and does NOT cycle continuously). '
                'Use linear/trapezoidal for cyclic tracking.')
            k = 1.0 / max(1e-3, self.approach_time)
            return ExponentialTimingLaw(k=k, s_target=self._approach_span)
        if name == 'trapezoidal':
            accel = rate / max(1e-3, self.approach_time)   # reach cruise in ~approach_time
            return TrapezoidalTimingLaw(rate=rate, accel=accel)
        if name != 'linear':
            self.get_logger().warn(f'Unknown timing_law "{name}" → using linear')
        return LinearTimingLaw(rate=rate, soft_start_s=self.approach_time)

    def _step_nominal(self, p_d, v_d, a_d, dt) -> None:
        """Integrate an independent open-loop *nominal* joint trajectory.

        Faithfully mirrors the measured task + null-space acceleration law
        (see _tick) but on its OWN state (q_nom, dq_nom) and a separate
        pin_data, so it never reads the robot.  The result is integrated as a
        pure double integral — no measured-state sync, no clamp-to-measured,
        no anti-windup reset — hence it stays the trajectory the arm WOULD
        follow with no CBF deviation.  The only physical limit kept is the
        per-joint q̈ saturation, identical to the commanded channel.
        """
        model, data = self.pin_model, self.pin_data_nom

        # Full-model configuration / velocity from the nominal arm state.
        np.copyto(self._q_nom_full, self._q_neutral)
        self._qdot_nom_full[:] = 0.0
        for k, pid in enumerate(self._pin_joint_ids):
            self._q_nom_full[model.joints[pid].idx_q] = self._q_nom[k]
        for k, vid in enumerate(self._arm_v_ids):
            self._qdot_nom_full[vid] = self._dq_nom[k]

        pin.computeAllTerms(model, data, self._q_nom_full, self._qdot_nom_full)
        pin.updateFramePlacements(model, data)

        self._J6n_nom[:] = pin.getFrameJacobian(
            model, data, self.ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        self._dJ6n_nom[:] = pin.getFrameJacobianTimeVariation(
            model, data, self.ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        for i, vid in enumerate(self._arm_v_ids):
            self._Jn_arm[:, i]  = self._J6n_nom[:, vid]
            self._dJn_arm[:, i] = self._dJ6n_nom[:, vid]

        oMee  = data.oMf[self.ee_frame_id]
        R_cur = np.asarray(oMee.rotation)

        if self._use_plane_frame:
            p_raw, _ = transform_ee_to_frame(
                model, data, self._plane_frame_id, oMee,
                self._J6n_nom[:, self._arm_v_ids])
            np.copyto(self._p_ee_nom, p_raw)
            R_ref = np.asarray(data.oMf[self._plane_frame_id].rotation)
            np.dot(R_ref.T, self._Jn_arm[:3, :], out=self._Jn_rot_tmp)
            self._Jn_arm[:3, :] = self._Jn_rot_tmp
            np.dot(R_ref.T, self._dJn_arm[:3, :], out=self._Jn_rot_tmp)
            self._dJn_arm[:3, :] = self._Jn_rot_tmp
        else:
            np.copyto(self._p_ee_nom, oMee.translation)

        # Orientation: capture the nominal hold target once, then hold.
        if not self._orient_nom_ok:
            np.copyto(self._R_des_nom, R_cur)
            self._orient_nom_ok = True
        # Stesso errore di orientamento corretto del canale misurato
        # (so3_log(R_des @ R_cur.T), world, segno des−cur). Vedi _tick.
        np.dot(self._R_des_nom.T, R_cur, out=self._R_err_nom)
        np.dot(self._R_des_nom, so3_log(self._R_err_nom), out=self._e_rot_nom)
        np.negative(self._e_rot_nom, out=self._e_rot_nom)

        # 6D pose error (against the SAME desired path as the measured channel)
        np.subtract(p_d, self._p_ee_nom, out=self._tmp3n)
        self._en6[:3] = self._tmp3n
        self._en6[3:] = self._e_rot_nom

        # 6D velocity error: [v_d − J_lin·dq_nom ; −J_ang·dq_nom]
        np.dot(self._Jn_arm, self._dq_nom, out=self._edotn6)
        np.subtract(v_d, self._edotn6[:3], out=self._edotn6[:3])
        np.negative(self._edotn6[3:], out=self._edotn6[3:])

        # Task-space acceleration: ẍ = a_d + Kp·e + Kd·edot
        self._xddotn6[:3] = a_d
        np.multiply(self.kp,     self._en6[:3],    out=self._tmp3n)
        self._xddotn6[:3] += self._tmp3n
        np.multiply(self.kd,     self._edotn6[:3], out=self._tmp3n)
        self._xddotn6[:3] += self._tmp3n
        np.multiply(self.kp_rot, self._en6[3:],    out=self._xddotn6[3:])
        np.multiply(self.kd_rot, self._edotn6[3:], out=self._tmp3n)
        self._xddotn6[3:] += self._tmp3n

        # Adaptive damped least-squares pseudo-inverse (same law as measured).
        np.dot(self._Jn_arm, self._Jn_arm.T, out=self._JJTn)
        w = math.sqrt(max(0.0, float(np.linalg.det(self._JJTn))))
        if w < self._manip_thr:
            f_w = (1.0 - w / self._manip_thr) ** 2
        else:
            f_w = 0.0
        self._lambda_sq_nom = (self._lambda_sq_min
                               + (self._lambda_sq_max - self._lambda_sq_min) * f_w)
        np.copyto(self._JJTn_reg, self._JJTn)
        for i in range(6):
            self._JJTn_reg[i, i] += self._lambda_sq_nom
        self._Jn_pinv[:] = np.linalg.solve(self._JJTn_reg, self._Jn_arm).T

        # qddot_task = J⁺(ẍ_d − J̇·q̇)
        np.dot(self._dJn_arm, self._dq_nom, out=self._tmp6n)
        np.subtract(self._xddotn6, self._tmp6n, out=self._xddotn6)
        np.dot(self._Jn_pinv, self._xddotn6, out=self._qddot_task_nom)

        # Null-space posture control toward the same home posture.
        np.dot(self._Jn_pinv, self._Jn_arm, out=self._JpinvJn)
        np.subtract(self._I7, self._JpinvJn, out=self._Nn)
        np.subtract(self._q_nom, self._q_home, out=self._qddot_null_nom)
        self._qddot_null_nom *= -self.k_null
        np.multiply(self.d_null, self._dq_nom, out=self._tmp7n)
        self._qddot_null_nom -= self._tmp7n
        np.dot(self._Nn, self._qddot_null_nom, out=self._null_proj_nom)

        np.add(self._qddot_task_nom, self._null_proj_nom, out=self._qddot_nom)
        np.clip(self._qddot_nom, -self.qddot_max, self.qddot_max, out=self._qddot_nom)

        # Pure open-loop double integration (NO measured-state coupling).
        self._dq_nom += self._qddot_nom * dt
        self._q_nom  += self._dq_nom * dt

    def _start_trajectory(self, js: dict) -> None:
        """Assemble the phase-driven path + timing law rooted at the current EE.

        The composite path begins exactly at the current EE position (no jump),
        runs a straight-line lead-in to the curve start over phase span
        ``S_a = approach_time / cycle_time``, then tracks the cyclic curve.
        """
        p0 = self._current_ee_position(js)

        # Target the actual curve entry P(0) so the approach→curve seam is
        # continuous in position.
        entry = self._cycle_path.position(0.0).copy()

        # Approach span in phase units (constant phase rate ⇒ S_a·cycle_time wall-s)
        self._approach_span = max(0.0, self.approach_time) / self.cycle_time

        if self._approach_span > 0.0:
            approach = LinearSegmentPath(p0, entry, span=self._approach_span)
            self._path = CompositePath([
                (approach, self._approach_span),
                (self._cycle_path, math.inf),
            ])
        else:
            self._path = CompositePath([(self._cycle_path, math.inf)])

        self._timing = self._build_timing_law()
        self._timing.reset(0.0)

        # Seed joint-space integration from the current measured state
        np.copyto(self._q_d,  js['q'])
        np.copyto(self._dq_d, js['qdot'])

        # Seed the independent open-loop nominal channel from the SAME start
        # state so the ideal and real trajectories coincide at t0 and diverge
        # only once the CBF deviates the real robot.
        np.copyto(self._q_nom,  js['q'])
        np.copyto(self._dq_nom, js['qdot'])
        self._orient_nom_ok = False

        # Seed the velocity low-pass filter at the measured velocity
        np.copyto(self._dq_filt, js['qdot'])

        # Capture the start configuration as the null-space home posture
        np.copyto(self._q_home, js['q'])

        self._running = True
        self.get_logger().info(
            f'Trajectory start: approach {vec_to_str(p0)} → '
            f'{vec_to_str(entry)}  (S_a={self._approach_span:.3f}, '
            f'timing={self.timing_law_name})')

    # ── High-rate CSV logging ──────────────────────────────────────────────

    def _init_logger(self, effort_topic: str) -> None:
        """Open a timestamped CSV file once and write the header row.

        On any failure the node keeps running with logging disabled
        (``self._csv_writer is None``) after emitting a warning.
        """
        # Disabled by default until the file is successfully opened.
        self._log_file = None
        self._csv_writer = None

        # Column layout — keep in sync with _log_data().
        header = ['time']
        header += [f'q{i}'          for i in range(1, 8)]   # real positions
        header += [f'dq{i}'         for i in range(1, 8)]   # real velocities
        header += [f'qddot_des_{i}' for i in range(1, 8)]   # commanded accel
        header += [f'q_des_{i}'     for i in range(1, 8)]   # integrated q_d
        header += [f'dq_des_{i}'    for i in range(1, 8)]   # integrated dq_d
        header += ['ee_x', 'ee_y', 'ee_z']                  # real EE position
        header += ['ee_x_des', 'ee_y_des', 'ee_z_des']      # desired EE position
        header += ['vx_des', 'vy_des', 'vz_des']            # desired Cart. velocity
        header += ['ax_des', 'ay_des', 'az_des']            # desired Cart. accel
        header += ['ex', 'ey', 'ez']                        # Cartesian error
        header += ['ee_error_norm']                         # ‖error‖
        header += ['s', 's_dot', 's_ddot']                  # phase variable
        header += ['manipulability', 'lambda_sq']           # conditioning
        header += [f'tau_des_{i}'   for i in range(1, 8)]   # desired torque
        header += [f'tau_meas_{i}'  for i in range(1, 8)]   # measured torque
        # Extended diagnostics (Change 8)
        header += ['qddot_norm', 'qddot_task_norm', 'qddot_null_norm']
        header += [f'sat_joint_{i}' for i in range(1, 8)]   # |qddot|/qddot_max
        header += ['max_sat_ratio', 'num_saturated_joints']
        header += ['sync_pos_norm', 'sync_vel_norm']
        # Independent open-loop NOMINAL (ideal) channel + filtered real velocity.
        # q_nom/dq_nom/qddot_nom ignore the robot (no CBF deviation); dq_filt is
        # the measured velocity after the first-order low-pass.
        header += [f'q_nom_{i}'     for i in range(1, 8)]   # ideal positions
        header += [f'dq_nom_{i}'    for i in range(1, 8)]   # ideal velocities
        header += [f'qddot_nom_{i}' for i in range(1, 8)]   # ideal accelerations
        header += [f'dq_filt_{i}'   for i in range(1, 8)]   # real velocity, low-pass

        # Pre-allocated row buffer (filled in place every tick → no allocation).
        self._log_row = [0.0] * len(header)

        try:
            log_dir = Path(self._log_dir)
            log_dir.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            path = log_dir / f'pentagon_run_{stamp}.csv'
            # 1 MiB buffer keeps disk I/O off the control path; newline=''
            # is required for the csv module on all platforms.
            self._log_file = open(path, 'w', newline='', buffering=1 << 20)
            self._csv_writer = csv.writer(self._log_file)
            self._csv_writer.writerow(header)
            self.get_logger().info(
                f'Logging to {path}  (effort topic: {effort_topic})')
        except Exception as exc:  # noqa: BLE001
            self._log_file = None
            self._csv_writer = None
            self.get_logger().warn(
                f'Could not open log file in {self._log_dir}: {exc} — '
                f'continuing without logging.')

    def _compute_tau_des(self) -> None:
        """tau_des = M(q)·q̈_des + nle(q, q̇), written into self._tau_des.

        Reuses M and nle already computed by computeAllTerms() this tick.
        CRBA only fills M's upper triangle, so mirror it to the lower triangle
        before the matrix-vector product.
        """
        M = self.pin_data.M
        M[self._M_tril] = M.T[self._M_tril]            # symmetrise in place
        self._qddot_full_des[:] = 0.0
        for k, vid in enumerate(self._arm_v_ids):
            self._qddot_full_des[vid] = self._q_ddot[k]
        np.dot(M, self._qddot_full_des, out=self._tau_full)
        self._tau_full += self.pin_data.nle
        for k, vid in enumerate(self._arm_v_ids):
            self._tau_des[k] = self._tau_full[vid]

    def _log_data(self, t, q, dq, p_d, v_d, a_d,
                  s, s_dot, s_ddot, w, ee_err) -> None:
        """Fill the pre-allocated row buffer and write it to the CSV file."""
        row = self._log_row
        o = 0
        row[o] = float(t);                                       o += 1
        for i in range(NUM_JOINTS): row[o + i] = float(q[i]);            # real q
        o += NUM_JOINTS
        for i in range(NUM_JOINTS): row[o + i] = float(dq[i]);           # real dq
        o += NUM_JOINTS
        for i in range(NUM_JOINTS): row[o + i] = float(self._q_ddot[i])  # q̈_des
        o += NUM_JOINTS
        for i in range(NUM_JOINTS): row[o + i] = float(self._q_d[i])     # q_des
        o += NUM_JOINTS
        for i in range(NUM_JOINTS): row[o + i] = float(self._dq_d[i])    # dq_des
        o += NUM_JOINTS
        row[o] = float(self._p_ee[0]); row[o+1] = float(self._p_ee[1]); row[o+2] = float(self._p_ee[2]); o += 3
        row[o] = float(p_d[0]);        row[o+1] = float(p_d[1]);        row[o+2] = float(p_d[2]);        o += 3
        row[o] = float(v_d[0]);        row[o+1] = float(v_d[1]);        row[o+2] = float(v_d[2]);        o += 3
        row[o] = float(a_d[0]);        row[o+1] = float(a_d[1]);        row[o+2] = float(a_d[2]);        o += 3
        row[o] = float(self._e6[0]);   row[o+1] = float(self._e6[1]);   row[o+2] = float(self._e6[2]);   o += 3
        row[o] = float(ee_err);                                  o += 1
        row[o] = float(s); row[o+1] = float(s_dot); row[o+2] = float(s_ddot); o += 3
        row[o] = float(w); row[o+1] = float(self._lambda_sq);    o += 2
        for i in range(NUM_JOINTS): row[o + i] = float(self._tau_des[i])  # tau_des
        o += NUM_JOINTS
        for i in range(NUM_JOINTS): row[o + i] = float(self._tau_meas[i]) # tau_meas
        o += NUM_JOINTS
        # Extended diagnostics (Change 8)
        row[o] = self._diag_qddot_norm;      o += 1
        row[o] = self._diag_qddot_task_norm; o += 1
        row[o] = self._diag_qddot_null_norm; o += 1
        for i in range(NUM_JOINTS): row[o + i] = float(self._sat_ratio[i])  # sat ratio
        o += NUM_JOINTS
        row[o] = self._diag_max_sat;          o += 1
        row[o] = float(self._diag_num_sat);   o += 1
        row[o] = self._diag_sync_pos_norm;    o += 1
        row[o] = self._diag_sync_vel_norm;    o += 1
        # Independent nominal (ideal) channel + filtered real velocity
        for i in range(NUM_JOINTS): row[o + i] = float(self._q_nom[i])     # q_nom
        o += NUM_JOINTS
        for i in range(NUM_JOINTS): row[o + i] = float(self._dq_nom[i])    # dq_nom
        o += NUM_JOINTS
        for i in range(NUM_JOINTS): row[o + i] = float(self._qddot_nom[i]) # qddot_nom
        o += NUM_JOINTS
        for i in range(NUM_JOINTS): row[o + i] = float(self._dq_filt[i])   # dq_filt
        o += NUM_JOINTS

        self._csv_writer.writerow(row)

    def _close_logger(self) -> None:
        """Flush and close the CSV file (idempotent)."""
        if self._log_file is not None:
            try:
                self._log_file.flush()
                self._log_file.close()
                self.get_logger().info('Log file closed.')
            except Exception:  # noqa: BLE001
                pass
            self._log_file = None
            self._csv_writer = None

    def destroy_node(self):
        """Ensure the log file is closed on node destruction."""
        self._close_logger()
        return super().destroy_node()


def main(args=None):
    run_node_main(PentagonQddotCommander, args=args)


if __name__ == '__main__':
    main()