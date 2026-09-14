#!/usr/bin/env python3
"""Acceleration-to-torque dynamics converter — placeholder for CBF safety filter.

Converts joint acceleration commands (q̈_nom) to joint torques (τ) by evaluating
the robot's equations of motion with Pinocchio:

    τ = M(q) · q̈_nom + C(q, q̇) · q̇

Gravity is intentionally excluded — ``rt_torque_controller`` adds g(q) internally.

This node acts as a pure dynamics passthrough in the acceleration-space pipeline:

    pentagon_qddot_commander → /NS_1/qddot_nom
    cbf_safety_filter        → /NS_1/qddot_safe
    qddot_to_torque          → /NS_1/torque_cmd
    rt_torque_controller     → hardware  (adds g(q))

It is meant to replace ``cbf_safety_filter`` while the CBF formulation is being
developed. When the CBF filter is ready it can be swapped back in without any
other changes to the pipeline.

Topics (loaded from fr3_control.yaml):
  Subscribes:
    - topics['joint_states_topic']  JointState (q, q̇)
    - topics['qddot_safe']          Float64MultiArray (7) — q̈ SAFE from the CBF filter
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
from franka_experiments.utils.config import load_franka_joint_limits
from franka_experiments.utils.kinematics import (
    generate_urdf_from_xacro,
    load_pinocchio_model,
    resolve_arm_joint_ids,
)
from franka_experiments.utils.params import declare_bool, declare_float, declare_str
from franka_experiments.utils.node_runtime import run_node_main


class QddotToTorqueNode(Node):
    """Converts q̈_nom → τ = M·q̈ + C·q̇ (gravity excluded)."""

    def __init__(self):
        super().__init__('qddot_to_torque')
        self.done = False

        # ── Configuration from fr3_control.yaml ───────────────────────────
        cfg    = load_robot_config('control')
        topics = cfg['topics']

        torque_out_topic = declare_str(
            self, 'torque_out_topic',
            topics.get('torque_cmd', '/NS_1/torque_cmd'))

        # ── ISO layer: command feasibility (roadmap Step 8) ───────────────
        # S_p assumes a_s is actually DELIVERED. This chain is open-loop —
        # tau = M q̈ + C q̇, no PD, no friction model — so the first thing to
        # establish is that the torque it asks for is a torque the robot can
        # produce. effort_max comes from franka_description's own
        # joint_limits.yaml (the file the URDF is generated from), not from a
        # copy that can drift.
        #
        # rt_torque_controller ALREADY clips. What is new here is that the
        # clipping becomes OBSERVABLE: /NS_1/torque_saturation carries one flag
        # per joint, published on EVERY command whatever the flags say, because
        # observability changes no number and a stopping-distance claim made
        # against a silently saturating actuator is worth nothing.
        #
        # The clip and the rate limit are APPLIED only under iso_enabled, so
        # with the flag off the torque on the wire is bit-identical to what it
        # was before this block existed.
        params = cfg.get('params', {}) or {}
        self._iso_enabled = declare_bool(
            self, 'iso_enabled', bool(params.get('iso_enabled', False)))
        self._tau_rate_max = declare_float(
            self, 'iso_tau_rate_max',
            float(params.get('iso_tau_rate_max', 40.0)),
            positive=True, maximum=1000.0)
        jl = load_franka_joint_limits(
            [f'joint{i}' for i in range(1, NUM_JOINTS + 1)])
        self._effort_max = jl['effort_max']
        self._tau_prev = None            # None until the first command goes out
        self._sat_pub = self.create_publisher(
            Float64MultiArray,
            topics.get('torque_saturation', '/NS_1/torque_saturation'), 10)
        self._sat_msg = Float64MultiArray()
        self._sat_msg.data = [0.0] * NUM_JOINTS

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

        # ── Shared state (written by JS callback, read by qddot callback) ─
        self._q    = np.zeros(NUM_JOINTS)
        self._qdot = np.zeros(NUM_JOINTS)
        self._has_js = False
        self._lock = threading.Lock()

        # ── Publisher / subscribers ───────────────────────────────────────
        self._pub = self.create_publisher(Float64MultiArray, torque_out_topic, 10)

        # joint_states_fast (the joint_state_broadcaster's own 1 kHz output),
        # not the 30 Hz Python republisher on joint_states: M(q) and C(q,qdot)
        # are evaluated at this state, so a 33 ms lag here biases every torque
        # the CBF asks for. See the topics block in fr3_control.yaml.
        # depth=1: this callback only CACHES the state (the torque is computed
        # in the qddot_safe callback), so with a 1 kHz publisher a deeper queue
        # would only build a backlog of states that are stale by the time they
        # are read. Always consume the latest.
        self.create_subscription(
            JointState,
            topics.get('joint_states_fast', topics['joint_states_topic']),
            self._on_joint_state,
            1,
        )
        self.create_subscription(
            Float64MultiArray,
            topics['qddot_safe'],
            self._on_qddot_nom,
            10,
        )

        self.get_logger().info(
            f'qddot_to_torque ready\n'
            f'  qddot_safe ← {topics["qddot_safe"]}\n'
            f'  torque     → {torque_out_topic}'
        )

    # ── Joint state callback ──────────────────────────────────────────────────

    def _on_joint_state(self, msg: JointState) -> None:
        # Rebuild the index map from each message: different publishers
        # (joint_state_broadcaster, joint_state_publisher, finger_state_publisher)
        # can interleave on the same topic with different joint subsets and
        # orderings, so a cached map from the first message would cause
        # IndexError when a later message has fewer positions.
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        n_pos = len(msg.position)
        n_vel = len(msg.velocity)

        q    = np.zeros(NUM_JOINTS)
        qdot = np.zeros(NUM_JOINTS)
        for k, name in enumerate(FR3_JOINT_NAMES):
            idx = name_to_idx.get(name)
            if idx is None or idx >= n_pos or idx >= n_vel:
                return  # message doesn't contain all 7 arm joints — skip
            q[k]    = msg.position[idx]
            qdot[k] = msg.velocity[idx]

        with self._lock:
            self._q[:]    = q
            self._qdot[:] = qdot
            self._has_js  = True

    # ── Acceleration callback → compute and publish torque ────────────────────

    # TODO[LEGACY]: name is now a misnomer — this callback carries qddot_SAFE (the CBF-filtered acceleration), not qddot_nom. Not renamed: ground rule 3 forbids renaming | confidence: high | superseded-by: none | flagged: 2026-09-01
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
        tau = self._limit_and_report(tau)

        out = Float64MultiArray()
        out.data = tau.tolist()
        self._pub.publish(out)

    def _limit_and_report(self, tau: np.ndarray) -> np.ndarray:
        """Effort clip + torque-rate limit, and the saturation flags. [E]

        Two bounds, in this order:

        * ``|tau| <= effort_max`` — the manufacturer's per-joint effort limit.
        * ``|tau - tau_prev| <= iso_tau_rate_max`` per tick. A torque STEP is
          not a torque the arm delivers: the drive current ramps, and the
          difference between commanded and realized shows up as exactly the
          ``qdd_cmd_rad`` / ``qdd_real_rad`` gap the CBFDIAG line reports. The
          rate limit makes the command something the actuator can follow, so
          that ``a_s`` means what ``S_p`` assumes it means.

        The flags are computed and published UNCONDITIONALLY. The limits are
        applied only with ``iso_enabled``; otherwise the returned torque is the
        input, unchanged, and the topic is pure diagnosis.

        Not a rated function. The rated analogues are *stopping time limiting*
        and *stopping distance limiting* (ISO 10218-1:2025, 5.5.6 / 5.5.7), and
        this is neither.
        """
        hit_effort = np.abs(tau) > self._effort_max
        tau_clipped = np.clip(tau, -self._effort_max, self._effort_max)

        if self._tau_prev is None:
            hit_rate = np.zeros(NUM_JOINTS, dtype=bool)
            tau_limited = tau_clipped
        else:
            step = tau_clipped - self._tau_prev
            hit_rate = np.abs(step) > self._tau_rate_max
            tau_limited = self._tau_prev + np.clip(
                step, -self._tau_rate_max, self._tau_rate_max)

        sat = np.logical_or(hit_effort, hit_rate)
        self._sat_msg.data = [1.0 if b else 0.0 for b in sat]
        self._sat_pub.publish(self._sat_msg)
        if sat.any():
            self.get_logger().warn(
                'torque saturation on joint(s) '
                + ', '.join(str(i + 1) for i in np.flatnonzero(sat))
                + f' — effort={np.round(np.abs(tau), 1).tolist()} vs '
                f'{self._effort_max.tolist()} N*m'
                + (f', rate limit {self._tau_rate_max:.0f} N*m/tick'
                   if hit_rate.any() else '')
                + ('' if self._iso_enabled
                   else ' (DIAGNOSTIC ONLY: iso_enabled is false, the command '
                        'goes out unclipped and rt_torque_controller clips it)'),
                throttle_duration_sec=1.0)

        out = tau_limited if self._iso_enabled else tau
        # tau_prev tracks what ACTUALLY went out, so the next tick's rate limit
        # is centred on the real command and not on one that was never sent.
        self._tau_prev = np.array(out, copy=True)
        return out

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
