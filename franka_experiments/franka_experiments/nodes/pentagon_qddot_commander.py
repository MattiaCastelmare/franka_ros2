#!/usr/bin/env python3
"""Pentagon reference generator built on MoveIt (franka_fr3_moveit_config).

This node is a *trajectory generation + reference provider* — not a control
loop.  Kinematics are delegated to MoveIt; the closed pentagon "race track" is
built as a periodic C² curve and replayed forever with the timeline taken
modulo one cycle, so the end-effector overflies the five vertices without ever
stopping and without any seam between laps.

A) MoveIt initialization
   ``pymoveit2.MoveIt2`` (the ROS 2 ``moveit_commander`` equivalent) bound to
   the conventions of ``franka_fr3_moveit_config``: planning group
   ``fr3_arm``, kinematic chain ``fr3_link0`` → ``fr3_hand_tcp``.

B) Closed C² track generation (done ONCE)
   The five pentagon vertices are used as the **control points** of a closed,
   uniform **cubic B-spline** (C² by construction).  The curve does *not* pass
   exactly through the vertices — it overflies (rounds) them — which is exactly
   what removes the corner stops.  The closed curve is resampled at
   ``curve_samples`` stations with **uniform arc-length spacing** and sent —
   once — to move_group's ``compute_cartesian_path`` service to obtain the
   joint-space path (IK along the curve, with config-flip guarding).  Only the
   joint *positions* are kept; MoveIt's stop-to-stop time parameterization
   (TOTG) is discarded — it would force zero velocity at the seam.

   Timing is re-derived from Cartesian **arc length** so the EE travels at
   (near-)constant speed: ``t[i] = arclen[i] / total * cycle_time``.  A
   **periodic cubic spline** ``q(t)`` is then fit per joint with periodic
   boundary conditions, which makes q, q̇ and q̈ continuous *including across
   the lap seam* (C²).  The Cartesian reference ``x(t)`` gets the same
   treatment for the tracking term.

C) Playback / reference output  (100 Hz, forever — no state machine)
   Each tick samples the periodic splines at ``tau = (t − t0) mod cycle_time``
   and publishes
     * ``qddot_nom``   Float64MultiArray(7) — cbf_safety_filter input
     * ``q_des_state`` JointState (position=q_d, velocity=q̇_d, effort=q̈)
     * ``~/planned_trajectory`` JointTrajectory — debugging / RViz.

   The published acceleration is feedforward + a 6D Cartesian-space tracking term:

       a_cart = [Kp_pos·clip(x_d − x, ±clamp) + Kd_pos·(ẋ_d − ẋ),
                 Kp_ori·log3(R_des·R_curr^T) + Kd_ori·(0 − ω)]
       q̈ = q̈_ff + J6†(q) · a_cart

   where J6†(q) is the Moore-Penrose pseudoinverse of the full 6×7 Jacobian,
   log3(·) is the SO(3) logarithm giving the axis-angle orientation error, and
   ω = J_ang·q̇ is the current angular velocity.  R_des is captured once at
   start-up and held constant (fixed EE orientation throughout the track).
   This term also pulls the EE *onto* the track at start-up, so no APPROACH/
   TRACK state machine is needed.  It acts BEFORE the CBF filter, preserving the
   CBF safety guarantees.  If the joint state goes stale the term is dropped
   (feedforward only).

   The track is planned exactly once.  There is no per-lap replanning and no
   reset of position/velocity/acceleration between laps — the node behaves like
   a robot endlessly driving a closed circuit.

Requires a running ``move_group`` node.  Until move_group and a valid joint
state are available the node publishes zero accelerations.

Safety: q̈ output is clamped to ±qddot_max per joint at every tick.
"""

from __future__ import annotations

import glob
import math
import os
import subprocess
import time
from typing import Dict, List, Optional

import numpy as np
import pinocchio as pin
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath
from pymoveit2 import MoveIt2
from rclpy.node import Node
from scipy.interpolate import CubicSpline
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from franka_experiments.utils.constants import FR3_JOINT_NAMES, NUM_JOINTS, AUTO_SENTINEL
from franka_experiments.utils.cbf_utils import load_robot_config
from franka_experiments.utils.logging_utils import ThrottledLogger
from franka_experiments.utils.ros import get_namespace_from_config, run_node_main


def _build_urdf_with_hand_cached() -> str:
    """Build (once) and cache the FR3 URDF with Franka hand for Pinocchio.

    The hand is required so that the fr3_hand_tcp frame is present in the
    model and FK/Jacobian can be evaluated at the actual EE TCP.
    """
    cache_path = '/tmp/franka_pent_fr3_hand.urdf'
    if os.path.exists(cache_path):
        return cache_path
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory('franka_description')
    xs = glob.glob(os.path.join(share, '**', 'fr3.urdf.xacro'), recursive=True)
    if not xs:
        raise RuntimeError('fr3.urdf.xacro not found under franka_description share')
    r = subprocess.run(
        ['xacro', xs[0], 'hand:=true', 'ee_id:=franka_hand', '-o', cache_path],
        capture_output=True, text=True,
    )
    if r.returncode != 0:
        raise RuntimeError(f'xacro failed:\n{r.stderr}')
    return cache_path


# Uniform cubic B-spline basis (1/6 · M), columns weight [P_{i-1}, P_i, P_{i+1}, P_{i+2}].
_BSPLINE_M = np.array(
    [[-1.0, 3.0, -3.0, 1.0],
     [3.0, -6.0, 3.0, 0.0],
     [-3.0, 0.0, 3.0, 0.0],
     [1.0, 4.0, 1.0, 0.0]], dtype=float) / 6.0


def closed_cubic_bspline(control_pts: np.ndarray, samples_per_segment: int = 60) -> np.ndarray:
    """Closed (periodic) uniform cubic B-spline through *control_pts*.

    The control points are NOT interpolated — the curve overflies them, which
    is exactly what rounds the pentagon corners.  A uniform cubic B-spline is
    C² everywhere, including across the closing seam (periodic indexing).

    Returns an (n·samples_per_segment, dim) array of curve points; the curve is
    closed (its last sample connects back to its first).
    """
    P = np.asarray(control_pts, dtype=float)
    n = len(P)
    if n < 3:
        raise ValueError('closed_cubic_bspline needs at least 3 control points')
    ts = np.linspace(0.0, 1.0, samples_per_segment, endpoint=False)
    powers = np.stack([ts ** 3, ts ** 2, ts, np.ones_like(ts)], axis=1)  # (s, 4)
    segments = []
    for i in range(n):
        G = np.stack([P[(i - 1) % n], P[i % n], P[(i + 1) % n], P[(i + 2) % n]], axis=0)
        segments.append(powers @ (_BSPLINE_M @ G))  # (s, dim)
    return np.vstack(segments)


def resample_uniform_arclength(curve: np.ndarray, n_out: int) -> np.ndarray:
    """Resample a closed *curve* at *n_out* stations of equal arc length.

    The closing segment (last point → first point) is included in the length
    budget so the spacing stays uniform across the seam.  The returned set has
    ``n_out`` points and does NOT repeat the start (the caller closes the loop).
    """
    closed = np.vstack([curve, curve[:1]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s[-1])
    if total < 1e-9:
        raise ValueError('degenerate curve (zero arc length)')
    su = np.linspace(0.0, total, n_out, endpoint=False)
    return np.stack([np.interp(su, s, closed[:, d]) for d in range(curve.shape[1])], axis=1)


def pentagon_vertices(center: np.ndarray, radius: float, plane: str) -> List[np.ndarray]:
    """Five pentagon vertices around *center* in the given *plane*.

    Same geometry as the legacy PentagonTrajectory: vertex 0 at the top,
    counter-clockwise.  ``front`` is an alias for the yz-plane of the base
    frame (vertical plane in front of the robot).
    """
    plane = plane.lower()
    vertices: List[np.ndarray] = []
    for k in range(5):
        angle = 2.0 * math.pi * k / 5.0 + math.pi / 2.0
        u = radius * math.cos(angle)
        v = radius * math.sin(angle)
        pt = np.asarray(center, dtype=float).copy()
        if plane == 'xy':
            pt[0] += u; pt[1] += v
        elif plane == 'xz':
            pt[0] += u; pt[2] += v
        elif plane in ('yz', 'front'):
            pt[1] += u; pt[2] += v
        else:
            raise ValueError(f'Unknown plane "{plane}", use xy/xz/yz/front')
        vertices.append(pt)
    return vertices


class PentagonQddotCommander(Node):

    def __init__(self):
        super().__init__('pentagon_qddot_commander')

        self.done      = False
        self._stopping = False
        self._stop_end = 0.0

        # ── Load robot config (topics + per-joint limits) ────────────────
        _cfg    = load_robot_config('control')
        _topics = _cfg['topics']
        _limits = _cfg['joint_limits']
        _jnames = [f'joint{i}' for i in range(1, 8)]

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('qddot_safe_topic',  _topics.get('qddot_nom', '/NS_1/qddot_nom'))
        self.declare_parameter('q_des_topic',       '/NS_1/q_des_state')
        self.declare_parameter('joint_state_topic', AUTO_SENTINEL)
        self.declare_parameter('base_frame',        'fr3_link0')
        self.declare_parameter('ee_frame',          'fr3_hand_tcp')
        self.declare_parameter('group_name',        'fr3_arm')
        self.declare_parameter('rate_hz',           100.0)
        self.declare_parameter('center_xyz',        [0.4, 0.0, 0.4])
        self.declare_parameter('radius',            0.20)
        self.declare_parameter('plane',             'front')
        self.declare_parameter('cycle_time',        8.0)
        # Closed-track geometry
        self.declare_parameter('curve_samples',     400)    # uniform arc-length stations
        self.declare_parameter('bspline_seg_samples', 80)   # B-spline samples / edge
        self.declare_parameter('control_scale',     1.0)    # scale vertices about center
        # MoveIt Cartesian path settings
        self.declare_parameter('max_step',          0.005)  # EE resolution [m]
        self.declare_parameter('jump_threshold',    5.0)    # config-flip guard
        self.declare_parameter('min_fraction',      0.80)   # reject partial paths
        self.declare_parameter('avoid_collisions',  True)
        self.declare_parameter('plan_retry_s',      1.0)    # back-off after failure
        # Cartesian-space tracking PD (acceleration space, applied before the CBF
        # filter).  kd_cart ≈ 2√kp_cart for near-critical damping.
        # Units: kp_cart [1/s²], kd_cart [1/s], cart_err_clamp [m].
        self.declare_parameter('kp_cart',           25.0)   # position gain [1/s²]
        self.declare_parameter('kd_cart',           10.0)   # velocity gain [1/s]
        self.declare_parameter('cart_err_clamp',    0.05)   # per-axis |x_d−x| cap [m]
        self.declare_parameter('js_timeout_s',      0.20)   # stale JS → drop correction
        self.declare_parameter('kp_ori',            25.0)   # orientation gain [rad/s²]
        self.declare_parameter('kd_ori',            10.0)   # orientation damping [1/s]
        self.declare_parameter('ori_err_clamp',      0.20)  # max |e_ori| fed to PD [rad]
        self.declare_parameter('j6_damp',            0.05)  # DLS damping λ for J6† [m or rad]
        self.declare_parameter('ramp_time',          1.0)   # seconds to ramp correction from 0→full

        qddot_topic       = str(self.get_parameter('qddot_safe_topic').value)
        q_des_topic       = str(self.get_parameter('q_des_topic').value)
        js_topic          = str(self.get_parameter('joint_state_topic').value)
        self._base_frame  = str(self.get_parameter('base_frame').value)
        self._ee_frame    = str(self.get_parameter('ee_frame').value)
        self._group_name  = str(self.get_parameter('group_name').value)
        self.rate_hz      = float(self.get_parameter('rate_hz').value)
        center_xyz        = list(self.get_parameter('center_xyz').value)
        radius            = float(self.get_parameter('radius').value)
        plane             = str(self.get_parameter('plane').value)
        self.cycle_time   = float(self.get_parameter('cycle_time').value)
        self._curve_samples = int(self.get_parameter('curve_samples').value)
        self._seg_samples = int(self.get_parameter('bspline_seg_samples').value)
        self._control_scale = float(self.get_parameter('control_scale').value)
        self._max_step    = float(self.get_parameter('max_step').value)
        self._jump_thr    = float(self.get_parameter('jump_threshold').value)
        self._min_fraction = float(self.get_parameter('min_fraction').value)
        self._avoid_collisions = bool(self.get_parameter('avoid_collisions').value)
        self._plan_retry  = float(self.get_parameter('plan_retry_s').value)
        self.kp_cart      = float(self.get_parameter('kp_cart').value)
        self.kd_cart      = float(self.get_parameter('kd_cart').value)
        self._cart_clamp  = float(self.get_parameter('cart_err_clamp').value)
        self._js_timeout  = float(self.get_parameter('js_timeout_s').value)
        self.kp_ori       = float(self.get_parameter('kp_ori').value)
        self.kd_ori       = float(self.get_parameter('kd_ori').value)
        self._ori_clamp   = float(self.get_parameter('ori_err_clamp').value)
        self._j6_damp     = float(self.get_parameter('j6_damp').value)
        self._ramp_time   = float(self.get_parameter('ramp_time').value)

        # Per-joint q̈ clamps from fr3_control.yaml joint_limits column [3].
        self.qddot_max = np.array([_limits[j][3] for j in _jnames], dtype=np.float64)

        # ── Pinocchio model (FR3 + Franka hand, finger joints locked) ────
        # The hand URDF defines fr3_hand_tcp as a fixed frame attached to
        # fr3_hand at the tool-centre-point.  FK and the geometric Jacobian
        # evaluated at this frame are used for the Cartesian tracking term and
        # for arc-length re-timing of the planned joint path.
        urdf_path = _build_urdf_with_hand_cached()
        _model_full = pin.buildModelFromUrdf(urdf_path)
        _lock = [_model_full.getJointId(n)
                 for n in _model_full.names if 'finger' in str(n)]
        _pin_model = pin.buildReducedModel(_model_full, _lock, pin.neutral(_model_full))
        self._pin_model = _pin_model
        self._pin_data  = _pin_model.createData()
        self._ee_fid: int = -1
        for frame in _pin_model.frames:
            if 'hand_tcp' in frame.name:
                self._ee_fid = _pin_model.getFrameId(frame.name)
                break
        if self._ee_fid < 0:
            raise RuntimeError(
                f'Frame containing "hand_tcp" not found in Pinocchio model '
                f'(URDF: {urdf_path}). '
                f'Available frames: {[f.name for f in _pin_model.frames]}')
        _tcp_name = _pin_model.frames[self._ee_fid].name

        # ── A) MoveIt initialization ──────────────────────────────────────
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=list(FR3_JOINT_NAMES),
            base_link_name=self._base_frame,
            end_effector_name=self._ee_frame,
            group_name=self._group_name,
        )
        self._cartesian_client = self.create_client(
            GetCartesianPath, 'compute_cartesian_path')

        # ── B) Pentagon control points (Cartesian, base_frame) ────────────
        self._center = np.asarray(center_xyz, dtype=float)
        verts = pentagon_vertices(self._center, radius, plane)
        # Vertices are the B-spline CONTROL POINTS (optionally scaled about the
        # centre to compensate for the curve being inscribed ~0.77·radius).
        self._control_pts = [
            self._center + self._control_scale * (v - self._center) for v in verts]
        self._ee_quat: Optional[np.ndarray] = None  # captured once via MoveIt FK (xyzw)
        self._R_des:   Optional[np.ndarray] = None  # R_des cached from _ee_quat (3×3)

        # ── C) Minimal playback state ─────────────────────────────────────
        # _traj holds the periodic splines: {'cs_q', 'cs_x', 'T'}.
        self._traj: Optional[Dict[str, object]] = None
        self._traj_t0 = None
        self._planned = False           # the closed track is planned exactly once
        self._plan_future = None
        self._retry_at = 0.0

        # ── Joint state buffer ────────────────────────────────────────────
        self._q = np.zeros(NUM_JOINTS)
        self._qdot = np.zeros(NUM_JOINTS)
        self._js_valid = False
        self._js_stamp: Optional[object] = None
        self._js_imap: Optional[List[int]] = None

        if js_topic == AUTO_SENTINEL:
            ns = get_namespace_from_config()
            js_topic = f'/{ns}/joint_states' if ns else '/joint_states'
        self.create_subscription(JointState, js_topic, self._js_cb, 10)

        # ── Publishers / timer ────────────────────────────────────────────
        self.pub       = self.create_publisher(Float64MultiArray, qddot_topic, 10)
        self._sp_pub   = self.create_publisher(JointState, q_des_topic, 10)
        self._traj_pub = self.create_publisher(JointTrajectory, '~/planned_trajectory', 1)

        self._out_msg       = Float64MultiArray()
        self._out_msg.data  = [0.0] * NUM_JOINTS
        self._zero_msg      = Float64MultiArray()
        self._zero_msg.data = [0.0] * NUM_JOINTS
        self._sp_msg          = JointState()
        self._sp_msg.name     = list(FR3_JOINT_NAMES)
        self._sp_msg.position = [0.0] * NUM_JOINTS
        self._sp_msg.velocity = [0.0] * NUM_JOINTS
        self._sp_msg.effort   = [0.0] * NUM_JOINTS

        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self.t0    = self.get_clock().now()
        self._tlog = ThrottledLogger(self.get_logger())

        self.get_logger().info(
            f'pentagon_qddot_commander (closed periodic C² track)\n'
            f'  qddot    : {qddot_topic}\n'
            f'  q_des    : {q_des_topic}\n'
            f'  js_topic : {js_topic}\n'
            f'  group    : {self._group_name}  ({self._base_frame} → {self._ee_frame})\n'
            f'  center   : {center_xyz}  r={radius} m  plane={plane}  '
            f'control_scale={self._control_scale}\n'
            f'  cycle_t  : {self.cycle_time} s  curve_samples={self._curve_samples}  '
            f'max_step={self._max_step} m\n'
            f'  track    : closed cubic B-spline (vertices = control points, overflown)\n'
            f'  cart PD  : kp_pos={self.kp_cart} kd_pos={self.kd_cart} '
            f'kp_ori={self.kp_ori} kd_ori={self.kd_ori} '
            f'err_clamp={self._cart_clamp} m  (6D Cartesian, pre-CBF)\n'
            f'  pin frame: {_tcp_name} (id={self._ee_fid})\n'
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
        for k, i in enumerate(self._js_imap):
            self._q[k]    = msg.position[i]
            self._qdot[k] = msg.velocity[i] if len(msg.velocity) > i else 0.0
        self._js_valid = True
        self._js_stamp = self.get_clock().now()

    # ── Stop ─────────────────────────────────────────────────────────────

    def request_stop(self, dur: float = 0.5):
        if self._stopping:
            return
        self._stopping = True
        self._stop_end = time.monotonic() + dur
        self.get_logger().info(f'Stopping: zero qddot for {dur} s')

    # ── Main tick: sample periodic track, publish references ──────────────

    def _tick(self):
        if self._stopping:
            self.pub.publish(self._zero_msg)
            if time.monotonic() >= self._stop_end:
                self.timer.cancel()
                self.done = True
            return

        now    = self.get_clock().now()
        t_node = (now - self.t0).nanoseconds * 1e-9

        if self._traj is None:
            self.pub.publish(self._zero_msg)
            self._maybe_plan()
            if self._tlog.due(t_node):
                self._tlog.info(
                    '[INIT] waiting for MoveIt track  '
                    f'(move_group: {self._cartesian_client.service_is_ready()}, '
                    f'joint_state: {self._js_valid})')
            return

        # Infinite closed circuit: replay the periodic splines modulo one cycle.
        # No reset of q/q̇/q̈ between laps — the periodic BCs make the seam C².
        cs_q = self._traj['cs_q']
        cs_x = self._traj['cs_x']
        T    = float(self._traj['T'])
        elapsed = (now - self._traj_t0).nanoseconds * 1e-9
        tau  = elapsed % T

        q_d   = np.asarray(cs_q(tau))
        dq_d  = np.asarray(cs_q(tau, 1))
        qdd_d = np.asarray(cs_q(tau, 2))
        x_d   = np.asarray(cs_x(tau))
        xd_d  = np.asarray(cs_x(tau, 1))

        # 6D Cartesian-space tracking correction (pre-CBF).
        # Uses Pinocchio FK + full 6×7 Jacobian at fr3_hand_tcp.
        # Position PD on linear part; orientation PD via SO(3) log error on angular part.
        # Dropped when the joint state is stale — feedforward keeps streaming.
        js_age = ((now - self._js_stamp).nanoseconds * 1e-9
                  if self._js_stamp is not None else float('inf'))
        # Linear ramp from 0 to 1 over ramp_time seconds to avoid the initial
        # step-change from zero (during planning) to full correction.  Prevents
        # torque-rate spikes that trigger Franka protective stops.
        ramp = min(1.0, elapsed / self._ramp_time) if self._ramp_time > 0.0 else 1.0

        do_log     = self._tlog.due(t_node)   # evaluated ONCE; used by all branches below
        e_ori_norm = 0.0
        omega_norm = 0.0
        cond_J6    = 0.0
        corr_norm  = 0.0
        if self._js_valid and js_age <= self._js_timeout and self._R_des is not None:
            pin.computeJointJacobians(self._pin_model, self._pin_data, self._q)
            pin.updateFramePlacements(self._pin_model, self._pin_data)
            J6 = pin.getFrameJacobian(
                self._pin_model, self._pin_data, self._ee_fid,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            J_lin      = J6[:3, :]                                 # (3, 7) linear velocity
            J_ang      = J6[3:, :]                                 # (3, 7) angular velocity
            x_curr     = self._pin_data.oMf[self._ee_fid].translation.copy()
            xd_curr    = J_lin @ self._qdot
            omega_curr = J_ang @ self._qdot

            e_p     = np.clip(x_d - x_curr, -self._cart_clamp, self._cart_clamp)
            e_v     = xd_d - xd_curr

            R_curr  = self._pin_data.oMf[self._ee_fid].rotation
            R_err   = self._R_des @ R_curr.T
            e_ori   = np.clip(pin.log3(R_err), -self._ori_clamp, self._ori_clamp)
            e_omega = -omega_curr   # omega_des = 0 (fixed orientation)

            a_cart = np.concatenate([
                self.kp_cart * e_p + self.kd_cart * e_v,
                self.kp_ori  * e_ori + self.kd_ori  * e_omega,
            ])

            # Damped least-squares (Levenberg-Marquardt) pseudoinverse.
            # Limits amplification near singular configs: ||J6_dls|| ≤ 1/(2λ).
            λ = self._j6_damp
            J6J6T = J6 @ J6.T
            J6_dls = J6.T @ np.linalg.solve(J6J6T + λ * λ * np.eye(6), np.eye(6))
            correction = ramp * (J6_dls @ a_cart)
            qdd_d = qdd_d + correction

            e_ori_norm = float(np.linalg.norm(e_ori))
            omega_norm = float(np.linalg.norm(omega_curr))
            corr_norm  = float(np.linalg.norm(correction))
            if do_log:
                sv = np.linalg.svd(J6, compute_uv=False)
                cond_J6 = float(sv[0] / sv[-1]) if sv[-1] > 1e-10 else float('inf')
        elif do_log:
            self.get_logger().warn(
                f'joint state stale ({js_age:.2f}s) — Cartesian correction dropped')

        # Safety clamp
        np.clip(qdd_d, -self.qddot_max, self.qddot_max, out=qdd_d)

        for i in range(NUM_JOINTS):
            self._out_msg.data[i] = float(qdd_d[i])
        self.pub.publish(self._out_msg)

        self._sp_msg.header.stamp = now.to_msg()
        for i in range(NUM_JOINTS):
            self._sp_msg.position[i] = float(q_d[i])
            self._sp_msg.velocity[i] = float(dq_d[i])
            self._sp_msg.effort[i]   = float(qdd_d[i])
        self._sp_pub.publish(self._sp_msg)

        if do_log:
            qn   = float(np.linalg.norm(qdd_d))
            qsat = float(np.sum(np.abs(qdd_d) >= 0.99 * self.qddot_max))
            lap  = int(elapsed // T) + 1
            self._tlog.info(
                f'[TRACK lap={lap} tau={tau:.2f}/{T:.2f}s ramp={ramp:.2f}] '
                f'|qddot|={qn:.3f} rad/s²  sat_joints={qsat:.0f}  '
                f'|corr|={corr_norm:.3f}  cond(J6)={cond_J6:.1f}  '
                f'|ori_err|={e_ori_norm:.4f} rad  |omega|={omega_norm:.4f} rad/s')

    # ── Planning pipeline (asynchronous, runs exactly once) ───────────────

    def _maybe_plan(self) -> None:
        if self._stopping or self._planned or self._plan_future is not None:
            return
        if time.monotonic() < self._retry_at:
            return
        if not self._cartesian_client.service_is_ready():
            return
        if self._ee_quat is None:
            if self._js_valid:
                self._request_fk()
        else:
            self._request_cartesian_path()

    def _request_fk(self) -> None:
        # Capture the current EE orientation once via MoveIt FK; held for all
        # track waypoints (same orientation throughout the lap).
        js = JointState()
        js.name = list(FR3_JOINT_NAMES)
        js.position = [float(v) for v in self._q]
        future = self.moveit2.compute_fk_async(joint_state=js)
        if future is None:
            self._retry_at = time.monotonic() + self._plan_retry
            return
        self._plan_future = future
        future.add_done_callback(self._on_fk_done)

    def _on_fk_done(self, future) -> None:
        self._plan_future = None
        pose = self.moveit2.get_compute_fk_result(future)
        if pose is None:
            self.get_logger().warn('MoveIt FK failed — retrying')
            self._retry_at = time.monotonic() + self._plan_retry
            return
        o = pose.pose.orientation
        p = pose.pose.position
        self._ee_quat = np.array([o.x, o.y, o.z, o.w])
        q_pin = pin.Quaternion(float(o.w), float(o.x), float(o.y), float(o.z))
        self._R_des = q_pin.toRotationMatrix().copy()
        self.get_logger().info(
            f'EE pose captured in {self._base_frame}: '
            f'p=[{p.x:.3f}, {p.y:.3f}, {p.z:.3f}] — planning closed track')
        self._request_cartesian_path()

    def _request_cartesian_path(self) -> None:
        req = GetCartesianPath.Request()
        req.header.frame_id   = self._base_frame
        req.header.stamp      = self.get_clock().now().to_msg()
        req.group_name        = self._group_name
        req.link_name         = self._ee_frame
        req.max_step          = self._max_step
        req.jump_threshold    = self._jump_thr
        req.avoid_collisions  = self._avoid_collisions

        # Start from the measured config; the planner builds a short lead-in to
        # the track which is trimmed off in _ingest so the stored loop is purely
        # periodic.  The Cartesian PD term covers the residual at start-up.
        start = JointState()
        start.name = list(FR3_JOINT_NAMES)
        start.position = [float(v) for v in self._q]

        # Closed C² track: vertices (control points) → cubic B-spline → uniform
        # arc-length resampling.  The waypoint list is closed (first appended at
        # the end) so MoveIt returns to the start and the joint path closes.
        curve = closed_cubic_bspline(self._control_pts, self._seg_samples)
        stations = resample_uniform_arclength(curve, self._curve_samples)
        waypoints = list(stations) + [stations[0]]

        req.start_state.joint_state = start
        req.waypoints = [self._pose_from_xyz(v) for v in waypoints]

        self._plan_future = self._cartesian_client.call_async(req)
        self._plan_future.add_done_callback(self._on_plan_done)

    def _pose_from_xyz(self, xyz: np.ndarray) -> Pose:
        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])
        pose.orientation.x = float(self._ee_quat[0])
        pose.orientation.y = float(self._ee_quat[1])
        pose.orientation.z = float(self._ee_quat[2])
        pose.orientation.w = float(self._ee_quat[3])
        return pose

    def _on_plan_done(self, future) -> None:
        self._plan_future = None
        try:
            res = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'compute_cartesian_path failed: {exc}')
            self._retry_at = time.monotonic() + self._plan_retry
            return
        if (res.error_code.val != MoveItErrorCodes.SUCCESS
                or res.fraction < self._min_fraction):
            self.get_logger().warn(
                f'Cartesian path rejected: error_code={res.error_code.val} '
                f'fraction={res.fraction:.3f} (min {self._min_fraction})')
            self._retry_at = time.monotonic() + self._plan_retry
            return

        traj = self._ingest(res.solution.joint_trajectory)
        if traj is None:
            self._retry_at = time.monotonic() + self._plan_retry
            return

        self._traj    = traj
        self._traj_t0 = self.get_clock().now()
        self._planned = True
        self.get_logger().info(
            f"Closed track armed: period T={traj['T']:.2f} s "
            f"(fraction {res.fraction:.3f}) — replaying forever")
        self._publish_debug(traj)

    def _ingest(self, jt: JointTrajectory) -> Optional[Dict[str, object]]:
        """Turn a MoveIt joint path into a *periodic C² track*.

        Steps: take joint positions only (TOTG timing discarded) → FK to
        Cartesian → trim the start-up lead-in → close the loop exactly →
        re-time by Cartesian arc length (constant speed) → fit periodic cubic
        splines q(t) and x(t).  Logs closure/speed/jerk diagnostics.
        """
        if len(jt.points) < 4:
            self.get_logger().warn('MoveIt returned a degenerate trajectory')
            return None
        try:
            order = [jt.joint_names.index(jn) for jn in FR3_JOINT_NAMES]
        except ValueError:
            self.get_logger().error(
                f'Unexpected joint names from MoveIt: {jt.joint_names}')
            return None

        q_raw = np.array([[p.positions[j] for j in order] for p in jt.points])

        # FK every waypoint (offline, once) for arc-length timing and tracking.
        model, data, fid = self._pin_model, self._pin_data, self._ee_fid
        n_raw = len(q_raw)
        x_raw = np.empty((n_raw, 3))
        for i in range(n_raw):
            pin.framesForwardKinematics(model, data, q_raw[i])
            x_raw[i] = data.oMf[fid].translation.copy()

        # Trim the lead-in: the loop begins where the path first reaches the
        # start station (which equals the final point of the closed waypoints).
        d_to_end = np.linalg.norm(x_raw - x_raw[-1], axis=1)
        tol = max(3.0 * self._max_step, 1e-3)
        cand = np.where(d_to_end < tol)[0]
        loop_start = int(cand[0]) if len(cand) and cand[0] < n_raw - 4 else 0
        q_loop = q_raw[loop_start:].copy()
        x_loop = x_raw[loop_start:].copy()

        # Drop consecutive (near-)duplicate samples so arc length is monotone.
        keep = np.concatenate(
            ([True], np.linalg.norm(np.diff(x_loop, axis=0), axis=1) > 1e-6))
        q_loop, x_loop = q_loop[keep], x_loop[keep]
        if len(q_loop) < 4:
            self.get_logger().warn('track too short after trimming')
            return None

        # Closure gap left by MoveIt (the discontinuity the periodic spline fixes).
        closure_pos  = float(np.linalg.norm(q_loop[-1] - q_loop[0]))
        closure_cart = float(np.linalg.norm(x_loop[-1] - x_loop[0]))

        # Null-space drift redistribution.
        #
        # MoveIt solves IK greedily along the path.  With a 7-DOF robot there is
        # one redundant DOF; the IK drifts in the null space over the full loop,
        # producing a deterministic joint-space closure gap that CANNOT be
        # eliminated by retrying with the same seed.
        #
        # Fix: subtract a linearly-ramped fraction of the gap from every knot.
        # For index k of N:  q_loop[k] -= (k/(N-1)) * q_gap
        # This distributes the accumulated drift uniformly, so q_loop[-1] = q_loop[0]
        # exactly by construction.  The Cartesian position change is negligible
        # because the gap lives almost entirely in the null space (wrist rotation).
        if closure_pos > 1e-4:
            q_gap  = q_loop[-1] - q_loop[0]                  # (7,) accumulated drift
            N      = len(q_loop)
            alpha  = np.linspace(0.0, 1.0, N)[:, None]       # (N,1) ramp
            q_loop -= alpha * q_gap                           # redistribute in-place
            # Recompute Cartesian positions for the modified joint angles
            # (change is sub-mm for wrist-joint drift; needed for correct arc-length timing).
            for i in range(N):
                pin.framesForwardKinematics(model, data, q_loop[i])
                x_loop[i] = data.oMf[fid].translation.copy()
            closure_pos_new  = float(np.linalg.norm(q_loop[-1] - q_loop[0]))
            closure_cart_new = float(np.linalg.norm(x_loop[-1] - x_loop[0]))
            self.get_logger().info(
                f'Null-space redistribution: '
                f'|Δq| {closure_pos:.3f} → {closure_pos_new:.2e} rad  '
                f'|Δx| {closure_cart:.4f} → {closure_cart_new:.4f} m')
            closure_pos  = closure_pos_new
            closure_cart = closure_cart_new

        # Force exact periodicity (q_loop[-1] is already ≈ q_loop[0] after redistribution;
        # this assignment removes the residual floating-point noise).
        q_loop[-1] = q_loop[0]
        x_loop[-1] = x_loop[0]

        # Re-time by Cartesian arc length → (near-)constant EE speed, period =
        # cycle_time.  q̇ at the vertices is therefore non-zero by construction.
        seg = np.linalg.norm(np.diff(x_loop, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(seg)])
        total_len = float(s[-1])
        if total_len < 1e-6 or self.cycle_time <= 0.0:
            self.get_logger().warn('cannot re-time track (zero length / cycle_time)')
            return None
        t = s / total_len * self.cycle_time
        # Enforce strict monotonicity for the spline knots.
        dt = np.diff(t)
        if np.any(dt <= 0.0):
            keep2 = np.concatenate(([True], dt > 1e-9))
            keep2[-1] = True  # always keep the closing knot
            t, q_loop, x_loop = t[keep2], q_loop[keep2], x_loop[keep2]

        # Periodic cubic splines → C² q(t) and x(t), continuous across the seam.
        cs_q = CubicSpline(t, q_loop, bc_type='periodic', axis=0)
        cs_x = CubicSpline(t, x_loop, bc_type='periodic', axis=0)
        T = float(t[-1])

        self._log_track_diagnostics(cs_q, cs_x, T, total_len, closure_pos, closure_cart)
        return {'cs_q': cs_q, 'cs_x': cs_x, 'T': T}

    def _log_track_diagnostics(self, cs_q, cs_x, T, total_len,
                               closure_pos, closure_cart) -> None:
        """Continuity at the seam, minimum speed, and peak numerical jerk."""
        grid = np.linspace(0.0, T, 2000, endpoint=False)
        # Seam continuity from the periodic spline (≈0 by construction).
        seam_q   = float(np.max(np.abs(cs_q(0.0)        - cs_q(T))))
        seam_qd  = float(np.max(np.abs(cs_q(0.0, 1)     - cs_q(T, 1))))
        seam_qdd = float(np.max(np.abs(cs_q(0.0, 2)     - cs_q(T, 2))))
        # Speeds along the track (must stay > 0 — no stops at the vertices).
        cart_speed  = np.linalg.norm(cs_x(grid, 1), axis=1)
        joint_speed = np.linalg.norm(cs_q(grid, 1), axis=1)
        # Peak jerk via finite differences of q̈ (per the spec).
        qdd = cs_q(grid, 2)
        jerk = np.gradient(qdd, grid, axis=0)
        max_jerk = float(np.max(np.abs(jerk)))
        max_qdd  = float(np.max(np.abs(qdd)))

        self.get_logger().info(
            'Track diagnostics:\n'
            f'  seam continuity   : |Δq|={seam_q:.2e}  |Δq̇|={seam_qd:.2e}  '
            f'|Δq̈|={seam_qdd:.2e}  (periodic C²; MoveIt closure gap was '
            f'|Δq|={closure_pos:.2e} rad, |Δx|={closure_cart:.2e} m)\n'
            f'  cart speed        : min={cart_speed.min():.4f}  '
            f'max={cart_speed.max():.4f} m/s  (perimeter {total_len:.3f} m)\n'
            f'  joint speed       : min={joint_speed.min():.4f}  '
            f'max={joint_speed.max():.4f} rad/s\n'
            f'  max |q̈| (spline)  : {max_qdd:.3f} rad/s²  '
            f'(qddot_max_min={self.qddot_max.min():.1f} — ratio={max_qdd/self.qddot_max.min():.1f}×)\n'
            f'  max |jerk| (q⃛)    : {max_jerk:.3f} rad/s³  (finite-diff of q̈)')
        if cart_speed.min() <= 1e-6:
            self.get_logger().warn(
                'track has a zero-speed point — vertices may not be overflown')
        if max_qdd > self.qddot_max.min():
            self.get_logger().warn(
                f'spline peak q̈ {max_qdd:.1f} rad/s² EXCEEDS minimum joint limit '
                f'{self.qddot_max.min():.1f} rad/s² — feedforward will saturate')

    def _publish_debug(self, tr: Dict[str, object]) -> None:
        cs_q = tr['cs_q']
        T    = float(tr['T'])
        ts   = np.linspace(0.0, T, 200)
        Q, QD, QDD = cs_q(ts), cs_q(ts, 1), cs_q(ts, 2)
        msg = JointTrajectory()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._base_frame
        msg.joint_names     = list(FR3_JOINT_NAMES)
        for ti, qi, qdi, qddi in zip(ts, Q, QD, QDD):
            pt = JointTrajectoryPoint()
            pt.positions     = [float(v) for v in qi]
            pt.velocities    = [float(v) for v in qdi]
            pt.accelerations = [float(v) for v in qddi]
            pt.time_from_start.sec     = int(ti)
            pt.time_from_start.nanosec = int((ti - int(ti)) * 1e9)
            msg.points.append(pt)
        self._traj_pub.publish(msg)


def main(args=None):
    run_node_main(PentagonQddotCommander, args=args)


if __name__ == '__main__':
    main()
