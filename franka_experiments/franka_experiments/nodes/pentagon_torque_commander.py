#!/usr/bin/env python3
"""End-effector pentagon trajectory tracker via joint-torque commands.

Publishes ``std_msgs/Float64MultiArray`` (7 joints) to the ``torque_cmd``
topic (consumed by ``rt_torque_controller``) using Pinocchio for Jacobian-
based 6D Cartesian PD control (position + orientation) mapped to joint torques.

Control law (6D, acceleration-space feedback linearization):
    e_pos   = p_d - p_ee                       # 3-D position error
    e_rot   = 0.5 · vee(R_d.T @ R - R.T @ R_d)  # SO(3) orientation error
    e6      = [e_pos; e_rot]
    e6_dot  = [v_d - J_lin @ qdot; -J_ang @ qdot]
    xddot6  = [Kp_pos·e_pos + Kd_pos·e_dot_pos;
               Kp_rot·e_rot + Kd_rot·e_dot_rot]
    J_pinv  = J.T @ inv(J @ J.T + λ²·I₆)      # damped least-squares 6×6
    q_ddot  = J_pinv @ (xddot6 - dJ @ qdot)
    tau     = M_arm @ q_ddot + C(q,qdot)·qdot

Orientation reference R_d is captured at the first valid joint-state tick and
held constant — this eliminates elbow drift while tracking the pentagon.

Gravity compensation is intentionally omitted — the rt_torque_controller
adds gravity torques on the hardware side.

Shutdown: on SIGINT the node publishes zero torques for ~0.5 s then exits.
"""

from __future__ import annotations

import threading
import time
from typing import List, Optional

import numpy as np

import pinocchio as pin
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from franka_experiments.utils.constants import FR3_JOINT_NAMES, NUM_JOINTS, AUTO_SENTINEL
from franka_experiments.utils.ros import get_namespace_from_config, run_node_main
from franka_experiments.utils.kinematics import (
    generate_urdf_from_xacro,
    load_pinocchio_model,
    resolve_frame_id,
    resolve_arm_joint_ids,
    transform_ee_to_frame,
)
from franka_experiments.utils.trajectory import PentagonTrajectory
from franka_experiments.utils.math_utils import cosine_ramp
from franka_experiments.utils.logging_utils import ThrottledLogger, vec_to_str


class PentagonTorqueCommander(Node):
    """6D Cartesian-PD pentagon tracker publishing joint torques."""

    def __init__(self):
        super().__init__('pentagon_torque_commander')

        self.done = False
        self._stopping = False
        self._stop_end_time = 0.0

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('cbf_mode', False)
        self.declare_parameter('qddot_nom_topic', '/NS_1/qddot_nom')
        self.declare_parameter('torque_topic', '/NS_1/torque_cmd')
        self.declare_parameter('joint_state_topic', AUTO_SENTINEL)
        self.declare_parameter('ee_frame', 'fr3_hand_tcp')
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('warmup_s', 3.0)
        self.declare_parameter('ramp_s', 3.0)
        self.declare_parameter('center_xyz', [0.4, 0.0, 0.4])
        self.declare_parameter('radius', 0.4)
        self.declare_parameter('plane', 'front')
        self.declare_parameter('plane_frame', 'fr3_link0')
        self.declare_parameter('cycle_time', 10.0)
        self.declare_parameter('kp_cart', 80.0)
        self.declare_parameter('kd_cart', 15.0)
        self.declare_parameter('kp_rot', 20.0)
        self.declare_parameter('kd_rot', 5.0)
        self.declare_parameter('smoothness', 0.15)

        self.cbf_mode    = self.get_parameter('cbf_mode').value
        qddot_nom_topic  = self.get_parameter('qddot_nom_topic').value
        torque_topic     = self.get_parameter('torque_topic').value
        js_topic_param   = self.get_parameter('joint_state_topic').value
        ee_frame_name  = self.get_parameter('ee_frame').value
        self.rate_hz   = self.get_parameter('rate_hz').value
        self.warmup_s  = self.get_parameter('warmup_s').value
        self.ramp_s    = self.get_parameter('ramp_s').value
        center_xyz     = list(self.get_parameter('center_xyz').value)
        radius         = self.get_parameter('radius').value
        plane          = self.get_parameter('plane').value
        plane_frame    = self.get_parameter('plane_frame').value
        cycle_time     = self.get_parameter('cycle_time').value
        self.kp        = self.get_parameter('kp_cart').value
        self.kd        = self.get_parameter('kd_cart').value
        self.kp_rot    = self.get_parameter('kp_rot').value
        self.kd_rot    = self.get_parameter('kd_rot').value
        smoothness     = self.get_parameter('smoothness').value

        if len(center_xyz) != 3:
            self.get_logger().error('center_xyz must have 3 elements')
            raise SystemExit(1)

        # ── Pinocchio ────────────────────────────────────────────────────
        self.get_logger().info('Generating URDF via xacro …')
        try:
            urdf_xml = generate_urdf_from_xacro()
        except Exception as exc:
            self.get_logger().error(f'Failed to generate URDF: {exc}')
            raise SystemExit(1) from exc

        self.pin_model, self.pin_data = load_pinocchio_model(urdf_xml)

        try:
            self.ee_frame_id = resolve_frame_id(self.pin_model, ee_frame_name)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            raise SystemExit(1) from exc

        self._use_plane_frame = (plane == 'front')
        self._plane_frame_id = -1
        if self._use_plane_frame:
            try:
                self._plane_frame_id = resolve_frame_id(self.pin_model, plane_frame)
            except RuntimeError as exc:
                self.get_logger().error(str(exc))
                raise SystemExit(1) from exc

        try:
            self._pin_joint_ids = resolve_arm_joint_ids(self.pin_model)
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            raise SystemExit(1) from exc

        self._arm_v_ids = [
            self.pin_model.joints[pid].idx_v for pid in self._pin_joint_ids
        ]
        self._arm_v_ix  = np.ix_(self._arm_v_ids, self._arm_v_ids)

        # ── Pre-allocated buffers ─────────────────────────────────────────
        nv = self.pin_model.nv
        self._q_neutral        = pin.neutral(self.pin_model)       # read-only
        self._q_full_buf       = pin.neutral(self.pin_model)
        self._qdot_full_buf    = np.zeros(nv)
        self._Cqdot_full_buf   = np.zeros(nv)
        self._J6n_buf          = np.zeros((6, nv))
        self._dJ6n_buf         = np.zeros((6, nv))
        self._J_arm_buf        = np.zeros((6, NUM_JOINTS))          # 6×7
        self._dJ_arm_buf       = np.zeros((6, NUM_JOINTS))          # 6×7
        self._J_rot_scratch    = np.zeros((3, NUM_JOINTS))          # temp for plane-frame rotation
        self._M_arm_buf        = np.zeros((NUM_JOINTS, NUM_JOINTS))
        self._C_qdot_buf       = np.zeros(NUM_JOINTS)
        self._tau_buf          = np.zeros(NUM_JOINTS)
        self._q_ddot_buf       = np.zeros(NUM_JOINTS)
        self._p_ee_buf         = np.zeros(3)
        # 6D task-space buffers
        self._e6_buf           = np.zeros(6)                        # [e_pos; e_rot]
        self._e6_dot_buf       = np.zeros(6)                        # [e_dot_pos; e_dot_rot]
        self._xddot6_buf       = np.zeros(6)                        # desired 6D acceleration
        self._scratch6_buf     = np.zeros(6)                        # scratch
        self._scratch3_buf     = np.zeros(3)                        # scratch 3-vec
        # Orientation error (SO3)
        self._R_des            = np.eye(3)                          # desired rotation (world frame)
        self._R_err_buf        = np.zeros((3, 3))
        self._e_rot_buf        = np.zeros(3)
        self._orientation_initialized = False
        # Damped LS (6×6)
        self._JJT_buf          = np.zeros((6, 6))
        self._JJT_reg_buf      = np.zeros((6, 6))
        self._J_pinv_buf       = np.zeros((NUM_JOINTS, 6))          # 7×6
        self._lambda_sq        = 1e-6                               # damping factor²

        self._tau_msg          = Float64MultiArray()
        self._tau_msg.data     = [0.0] * NUM_JOINTS

        # ── Double-buffered joint state (GIL-safe swap) ───────────────────
        self._js_lock   = threading.Lock()
        self._js_buf_a  = {
            'q':      np.zeros(NUM_JOINTS),
            'qdot':   np.zeros(NUM_JOINTS),
            'q_full': pin.neutral(self.pin_model),
            'valid':  False,
        }
        self._js_buf_b  = {
            'q':      np.zeros(NUM_JOINTS),
            'qdot':   np.zeros(NUM_JOINTS),
            'q_full': pin.neutral(self.pin_model),
            'valid':  False,
        }
        self._js_write_buf = self._js_buf_a
        self._js_read_buf  = self._js_buf_b

        self._js_stamp      = self.get_clock().now()
        self._js_index_map: Optional[List[int]] = None

        js_topic = js_topic_param
        if js_topic == AUTO_SENTINEL:
            ns = get_namespace_from_config()
            js_topic = f'/{ns}/joint_states' if ns else '/joint_states'

        self._js_sub = self.create_subscription(
            JointState, js_topic, self._joint_state_cb, 10)
        self.get_logger().info(f'Joint-state topic: {js_topic}')

        # ── Trajectory ───────────────────────────────────────────────────
        self.traj = PentagonTrajectory(
            center=np.array(center_xyz),
            radius=radius,
            plane=plane,
            cycle_time=cycle_time,
            smoothness=smoothness,
        )

        # ── Publisher + timer ────────────────────────────────────────────
        if self.cbf_mode:
            pub_topic = qddot_nom_topic
        else:
            pub_topic = torque_topic
        self.pub = self.create_publisher(Float64MultiArray, pub_topic, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self._timer_cb)
        self.t0 = self.get_clock().now()

        self._zero_msg = Float64MultiArray()
        self._zero_msg.data = [0.0] * NUM_JOINTS
        self._tlog = ThrottledLogger(self.get_logger())

        mode_str = f'CBF (qddot_nom → {qddot_nom_topic})' if self.cbf_mode else f'torque → {torque_topic}'
        self.get_logger().info(
            f'pentagon_torque_commander started\n'
            f'  mode         : {mode_str}\n'
            f'  ee_frame     : {ee_frame_name}\n'
            f'  rate         : {self.rate_hz} Hz\n'
            f'  warmup/ramp  : {self.warmup_s} / {self.ramp_s} s\n'
            f'  center       : {center_xyz}\n'
            f'  radius       : {radius} m\n'
            f'  plane        : {plane}\n'
            f'  Kp/Kd pos    : {self.kp} / {self.kd}\n'
            f'  Kp/Kd rot    : {self.kp_rot} / {self.kd_rot}')

    # ── Joint-state callback ──────────────────────────────────────────────

    def _joint_state_cb(self, msg: JointState) -> None:
        if self._js_index_map is None:
            try:
                self._js_index_map = [msg.name.index(jn) for jn in FR3_JOINT_NAMES]
            except ValueError:
                return

        max_idx = max(self._js_index_map)
        if len(msg.position) < max_idx + 1:
            return

        buf = self._js_write_buf
        for k, i in enumerate(self._js_index_map):
            buf['q'][k]    = msg.position[i]
            buf['qdot'][k] = msg.velocity[i] if len(msg.velocity) > i else 0.0

        q_full = buf['q_full']
        np.copyto(q_full, self._q_neutral)  # reset to neutral in-place
        for k, pid in enumerate(self._pin_joint_ids):
            q_full[self.pin_model.joints[pid].idx_q] = buf['q'][k]
        buf['valid'] = True

        # Atomic swap — only reference assignment, GIL-safe
        with self._js_lock:
            self._js_write_buf, self._js_read_buf = \
                self._js_read_buf, self._js_write_buf

        self._js_stamp = self.get_clock().now()

    # ── Stopping ─────────────────────────────────────────────────────────

    def request_stop(self, stop_duration_s: float = 0.5):
        if self._stopping:
            return
        self._stopping = True
        self._stop_end_time = time.monotonic() + stop_duration_s
        self.get_logger().info(
            f'Stopping: publishing zero torques for {stop_duration_s} s')

    # ── Timer callback ────────────────────────────────────────────────────

    def _timer_cb(self):
        if self._stopping:
            try:
                self.pub.publish(self._zero_msg)
            except Exception:
                pass
            if time.monotonic() >= self._stop_end_time:
                self.get_logger().info('Stop complete')
                self.timer.cancel()
                self.done = True
            return

        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9

        if t < self.warmup_s:
            self.pub.publish(self._zero_msg)
            if self._tlog.due(t):
                self._tlog.info(f'[t={t:.1f}s warmup]')
            return

        tr = t - self.warmup_s
        envelope = cosine_ramp(tr, self.ramp_s)

        with self._js_lock:
            js = self._js_read_buf

        if not js['valid']:
            self.pub.publish(self._zero_msg)
            if self._tlog.due(t):
                self.get_logger().warn('No joint state — publishing zero torques')
            return

        js_age = (self.get_clock().now() - self._js_stamp).nanoseconds * 1e-9
        if js_age > 0.1:
            self.pub.publish(self._zero_msg)
            if self._tlog.due(t):
                self.get_logger().warn(
                    f'Joint-state stale ({js_age:.3f} s) — publishing zeros')
            return

        # ── Copy joint state into Pinocchio work buffers ──────────────────
        np.copyto(self._q_full_buf, js['q_full'])
        self._qdot_full_buf[:] = 0.0
        for k, vid in enumerate(self._arm_v_ids):
            self._qdot_full_buf[vid] = js['qdot'][k]
        qdot = js['qdot']  # 7-element arm qdot — no copy

        # ── Kinematics + dynamics (single Pinocchio pass) ─────────────────
        pin.computeAllTerms(self.pin_model, self.pin_data,
                            self._q_full_buf, self._qdot_full_buf)
        pin.updateFramePlacements(self.pin_model, self.pin_data)

        # ── Full 6×nv Jacobians → copy into pre-allocated buffers ─────────
        self._J6n_buf[:] = pin.getFrameJacobian(
            self.pin_model, self.pin_data, self.ee_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        self._dJ6n_buf[:] = pin.getFrameJacobianTimeVariation(
            self.pin_model, self.pin_data, self.ee_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

        # Extract 6×7 arm Jacobians element-wise (avoids fancy-index intermediate)
        for i, vid in enumerate(self._arm_v_ids):
            self._J_arm_buf[:, i]  = self._J6n_buf[:, vid]
            self._dJ_arm_buf[:, i] = self._dJ6n_buf[:, vid]

        # Mass matrix 7×7 arm sub-block
        M_full = np.asarray(self.pin_data.M)  # view, no copy
        np.copyto(self._M_arm_buf, M_full[self._arm_v_ix])

        # Coriolis × qdot → 7-element arm vector
        np.dot(np.asarray(self.pin_data.C), self._qdot_full_buf,
               out=self._Cqdot_full_buf)
        for i, vid in enumerate(self._arm_v_ids):
            self._C_qdot_buf[i] = self._Cqdot_full_buf[vid]

        # ── EE pose ───────────────────────────────────────────────────────
        oMee = self.pin_data.oMf[self.ee_frame_id]

        if self._use_plane_frame:
            # p_ee in plane frame — transform_ee_to_frame may allocate internally
            p_ee_raw, _ = transform_ee_to_frame(
                self.pin_model, self.pin_data,
                self._plane_frame_id, oMee,
                self._J6n_buf[:, self._arm_v_ids])
            np.copyto(self._p_ee_buf, p_ee_raw)
            # Rotate positional rows (0:3) of J and dJ into plane frame
            R_ref = np.asarray(self.pin_data.oMf[self._plane_frame_id].rotation)
            np.dot(R_ref.T, self._J_arm_buf[:3, :], out=self._J_rot_scratch)
            self._J_arm_buf[:3, :] = self._J_rot_scratch
            np.dot(R_ref.T, self._dJ_arm_buf[:3, :], out=self._J_rot_scratch)
            self._dJ_arm_buf[:3, :] = self._J_rot_scratch
        else:
            np.copyto(self._p_ee_buf, oMee.translation)

        p_ee = self._p_ee_buf

        # ── Orientation reference — captured once at first valid tick ──────
        # oMee.rotation is always in world frame (LOCAL_WORLD_ALIGNED)
        R_cur = np.asarray(oMee.rotation)  # view, no copy
        if not self._orientation_initialized:
            np.copyto(self._R_des, R_cur)
            self._orientation_initialized = True
            self.get_logger().info('Orientation reference captured')

        # ── Orientation error: e_rot = 0.5 · vee(R_d.T @ R - R.T @ R_d) ──
        # Equivalent to 0.5 · vee(R_err - R_err.T) where R_err = R_d.T @ R
        np.dot(self._R_des.T, R_cur, out=self._R_err_buf)
        self._e_rot_buf[0] = 0.5 * (self._R_err_buf[2, 1] - self._R_err_buf[1, 2])
        self._e_rot_buf[1] = 0.5 * (self._R_err_buf[0, 2] - self._R_err_buf[2, 0])
        self._e_rot_buf[2] = 0.5 * (self._R_err_buf[1, 0] - self._R_err_buf[0, 1])

        # ── Trajectory reference ──────────────────────────────────────────
        p_d, v_d = self.traj.evaluate(tr)

        # ── 6D error ─────────────────────────────────────────────────────
        # Position error: e_pos = p_d - p_ee
        np.subtract(p_d, p_ee, out=self._scratch3_buf)
        self._e6_buf[:3] = self._scratch3_buf
        # Orientation error (already in _e_rot_buf)
        self._e6_buf[3:] = self._e_rot_buf

        # ── 6D velocity error ─────────────────────────────────────────────
        # Current EE velocity: v6 = J_arm @ qdot
        np.dot(self._J_arm_buf, qdot, out=self._e6_dot_buf)
        # e6_dot_pos = v_d - J_lin @ qdot
        np.subtract(v_d, self._e6_dot_buf[:3], out=self._e6_dot_buf[:3])
        # e6_dot_rot = 0 - J_ang @ qdot  (desired angular velocity = 0)
        np.negative(self._e6_dot_buf[3:], out=self._e6_dot_buf[3:])

        # ── 6D desired acceleration ───────────────────────────────────────
        # xddot6[:3] = Kp_pos·e_pos + Kd_pos·e_dot_pos
        np.multiply(self.kp, self._e6_buf[:3], out=self._xddot6_buf[:3])
        np.multiply(self.kd, self._e6_dot_buf[:3], out=self._scratch3_buf)
        np.add(self._xddot6_buf[:3], self._scratch3_buf, out=self._xddot6_buf[:3])
        # xddot6[3:] = Kp_rot·e_rot + Kd_rot·e_dot_rot
        np.multiply(self.kp_rot, self._e6_buf[3:], out=self._xddot6_buf[3:])
        np.multiply(self.kd_rot, self._e6_dot_buf[3:], out=self._scratch3_buf)
        np.add(self._xddot6_buf[3:], self._scratch3_buf, out=self._xddot6_buf[3:])

        # ── Damped least-squares pseudoinverse (6×6) ──────────────────────
        # JJT = J @ J.T  (6×6)
        np.dot(self._J_arm_buf, self._J_arm_buf.T, out=self._JJT_buf)
        # JJT_reg = JJT + λ²·I₆  (allocation-free diagonal add)
        np.copyto(self._JJT_reg_buf, self._JJT_buf)
        for i in range(6):
            self._JJT_reg_buf[i, i] += self._lambda_sq
        # J_pinv = J.T @ inv(JJT_reg)  (7×6)
        JJT_inv = np.linalg.inv(self._JJT_reg_buf)  # 6×6 — unavoidable small alloc
        np.dot(self._J_arm_buf.T, JJT_inv, out=self._J_pinv_buf)

        # ── q_ddot_des = J_pinv @ (xddot6 - dJ @ qdot) ───────────────────
        np.dot(self._dJ_arm_buf, qdot, out=self._scratch6_buf)
        np.subtract(self._xddot6_buf, self._scratch6_buf, out=self._xddot6_buf)
        np.dot(self._J_pinv_buf, self._xddot6_buf, out=self._q_ddot_buf)

        if self.cbf_mode:
            # CBF mode: publish q_ddot scaled by envelope as qddot_nom
            for i in range(NUM_JOINTS):
                self._tau_msg.data[i] = float(self._q_ddot_buf[i] * envelope)
            self.pub.publish(self._tau_msg)

            if self._tlog.due(t):
                phase  = 'ramp' if envelope < 1.0 else 'active'
                p_ee_s = vec_to_str(p_ee)
                p_d_s  = vec_to_str(p_d)
                epos_n = float(np.linalg.norm(self._e6_buf[:3]))
                erot_n = float(np.linalg.norm(self._e6_buf[3:]))
                qdd_n  = float(np.linalg.norm(self._q_ddot_buf))
                self._tlog.info(
                    f'[CBF t={t:.1f}s {phase} env={envelope:.3f}] '
                    f'p=[{p_ee_s}] p_d=[{p_d_s}] '
                    f'|e_pos|={epos_n:.4f} |e_rot|={erot_n:.4f} '
                    f'|qddot_nom|={qdd_n:.4f} rad/s²')
        else:
            # Standard mode: tau = M @ q_ddot + C_qdot
            np.dot(self._M_arm_buf, self._q_ddot_buf, out=self._tau_buf)
            np.add(self._tau_buf, self._C_qdot_buf, out=self._tau_buf)
            self._tau_buf *= envelope

            for i in range(NUM_JOINTS):
                self._tau_msg.data[i] = float(self._tau_buf[i])
            self.pub.publish(self._tau_msg)

            if self._tlog.due(t):
                phase    = 'ramp' if envelope < 1.0 else 'active'
                p_ee_s   = vec_to_str(p_ee)
                p_d_s    = vec_to_str(p_d)
                epos_n   = float(np.linalg.norm(self._e6_buf[:3]))
                erot_n   = float(np.linalg.norm(self._e6_buf[3:]))
                tau_norm = float(np.linalg.norm(self._tau_buf))
                self._tlog.info(
                    f'[t={t:.1f}s {phase} env={envelope:.3f}] '
                    f'p=[{p_ee_s}] p_d=[{p_d_s}] '
                    f'|e_pos|={epos_n:.4f} |e_rot|={erot_n:.4f} '
                    f'|tau|={tau_norm:.2f} N·m')


def main(args=None):
    run_node_main(PentagonTorqueCommander, args=args)


if __name__ == '__main__':
    main()
