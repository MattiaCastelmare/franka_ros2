#!/usr/bin/env python3
"""Acceleration-to-torque dynamics converter — placeholder for CBF safety filter.

Converts joint acceleration commands (q̈_nom) to joint torques (τ) by evaluating
the robot's equations of motion with Pinocchio:

    τ = M(q) · q̈_nom + C(q, q̇) · q̇

Gravity is intentionally excluded — ``rt_torque_controller`` adds g(q) internally.

This node acts as a pure dynamics passthrough in the acceleration-space pipeline:

    pentagon_qddot_commander → /NS_1/qddot_nom
    qddot_to_torque          → /NS_1/torque_cmd
    rt_torque_controller     → hardware  (adds g(q))

It is meant to replace ``cbf_safety_filter`` while the CBF formulation is being
developed. When the CBF filter is ready it can be swapped back in without any
other changes to the pipeline.

Topics (loaded from fr3_control.yaml):
  Subscribes:
    - topics['joint_states_topic']  JointState (q, q̇)
    - topics['qddot_nom']           Float64MultiArray (7) — q̈ from commander
  Publishes:
    - torque_out_topic              Float64MultiArray (7) — τ to rt_torque_controller

Parameters:
  torque_out_topic (str, default '/NS_1/torque_cmd')
    Topic on which the computed torque is published.
"""

from __future__ import annotations

import threading

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from franka_experiments.utils.constants import FR3_JOINT_NAMES, NUM_JOINTS
from franka_experiments.utils.cbf_utils import load_robot_config
from franka_experiments.utils.kinematics import (
    generate_urdf_from_xacro,
    load_pinocchio_model,
    resolve_arm_joint_ids,
)
from franka_experiments.utils.ros import run_node_main


class QddotToTorqueNode(Node):
    """Converts q̈_nom → τ = M·q̈ + C·q̇ (gravity excluded)."""

    def __init__(self):
        super().__init__('qddot_to_torque')
        self.done = False

        # ── Configuration from fr3_control.yaml ───────────────────────────
        cfg    = load_robot_config('control')
        topics = cfg['topics']

        self.declare_parameter('torque_out_topic', '/NS_1/torque_cmd')
        torque_out_topic = self.get_parameter('torque_out_topic').value

        # ── Pinocchio dynamics model (hand:=true — matches rt_torque_controller) ─
        self.get_logger().info('Building dynamics model via xacro …')
        urdf_xml = generate_urdf_from_xacro()
        self._model, self._data = load_pinocchio_model(urdf_xml)

        # Arm joint index maps: Python list [idx_q] and [idx_v] for fr3_joint1..7
        _pin_jids       = resolve_arm_joint_ids(self._model)
        self._arm_v_ids = [self._model.joints[j].idx_v for j in _pin_jids]
        self._arm_q_ids = [self._model.joints[j].idx_q for j in _pin_jids]
        self._arm_v_ix  = np.ix_(self._arm_v_ids, self._arm_v_ids)

        _nv = self._model.nv
        self._q_neutral = pin.neutral(self._model)
        self._q_full    = pin.neutral(self._model)
        self._qdot_full = np.zeros(_nv)

        # Pre-allocate output buffers
        self._M_arm      = np.zeros((NUM_JOINTS, NUM_JOINTS))
        self._C_qdot_arm = np.zeros(NUM_JOINTS)

        # Warm-up to avoid first-call overhead
        pin.computeAllTerms(self._model, self._data,
                            self._q_full, self._qdot_full)

        # ── Joint-name → JointState index map ────────────────────────────
        # Filled on first JointState message; used to reorder joint data.
        self._js_idx: dict[str, int] | None = None

        # ── Shared state (written by JS callback, read by qddot callback) ─
        self._q    = np.zeros(NUM_JOINTS)
        self._qdot = np.zeros(NUM_JOINTS)
        self._has_js = False
        self._lock = threading.Lock()

        # ── Publisher / subscribers ───────────────────────────────────────
        self._pub = self.create_publisher(Float64MultiArray, torque_out_topic, 10)

        self.create_subscription(
            JointState,
            topics['joint_states_topic'],
            self._on_joint_state,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            topics['qddot_nom'],
            self._on_qddot_nom,
            10,
        )

        self.get_logger().info(
            f'qddot_to_torque ready\n'
            f'  qddot_nom ← {topics["qddot_nom"]}\n'
            f'  torque    → {torque_out_topic}'
        )

    # ── Joint state callback ──────────────────────────────────────────────────

    def _on_joint_state(self, msg: JointState) -> None:
        # Build name→index map on first message
        if self._js_idx is None:
            self._js_idx = {name: i for i, name in enumerate(msg.name)}

        q    = np.zeros(NUM_JOINTS)
        qdot = np.zeros(NUM_JOINTS)
        for k, name in enumerate(FR3_JOINT_NAMES):
            idx = self._js_idx.get(name)
            if idx is None:
                return
            q[k]    = msg.position[idx]
            qdot[k] = msg.velocity[idx]

        with self._lock:
            self._q[:]    = q
            self._qdot[:] = qdot
            self._has_js  = True

    # ── Acceleration callback → compute and publish torque ────────────────────

    def _on_qddot_nom(self, msg: Float64MultiArray) -> None:
        with self._lock:
            if not self._has_js:
                self.get_logger().warn(
                    'qddot_nom received but no joint state yet',
                    throttle_duration_sec=5.0,
                )
                return
            q    = self._q.copy()
            qdot = self._qdot.copy()

        qddot_nom = np.asarray(msg.data, dtype=np.float64)
        if qddot_nom.shape[0] != NUM_JOINTS:
            self.get_logger().error(
                f'qddot_nom has {qddot_nom.shape[0]} elements, expected {NUM_JOINTS}')
            return

        tau = self._compute_tau(q, qdot, qddot_nom)

        out = Float64MultiArray()
        out.data = tau.tolist()
        self._pub.publish(out)

    # ── Dynamics ──────────────────────────────────────────────────────────────

    def _compute_tau(self, q: np.ndarray, qdot: np.ndarray,
                     qddot: np.ndarray) -> np.ndarray:
        """τ = M(q)·q̈ + C(q,q̇)·q̇   (gravity excluded)."""
        np.copyto(self._q_full, self._q_neutral)
        self._qdot_full[:] = 0.0
        for k, (iq, iv) in enumerate(zip(self._arm_q_ids, self._arm_v_ids)):
            self._q_full[iq]    = q[k]
            self._qdot_full[iv] = qdot[k]

        pin.computeAllTerms(self._model, self._data,
                            self._q_full, self._qdot_full)

        # M_arm: 7×7 mass matrix restricted to arm joints
        np.copyto(self._M_arm,
                  np.asarray(self._data.M)[self._arm_v_ix])

        # C·qdot restricted to arm joints
        Cqdot_full = np.asarray(self._data.C) @ self._qdot_full
        for k, iv in enumerate(self._arm_v_ids):
            self._C_qdot_arm[k] = Cqdot_full[iv]

        return self._M_arm @ qddot + self._C_qdot_arm

    def request_stop(self) -> None:
        self.done = True


def main(args=None):
    run_node_main(QddotToTorqueNode, args=args)


if __name__ == '__main__':
    main()
