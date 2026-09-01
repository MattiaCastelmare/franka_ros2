#!/usr/bin/env python3
"""RL policy commander — ONNX Safe-RL policy → /NS_1/qddot_nom (Sim-to-Real).

Step 3 of ``franka_sim_to_real_roadmap.md``: the deployment counterpart of
``franka_sim``.  A SAC policy trained in MuJoCo *against the same
acceleration-level CBF shield that runs on this robot* is exported to ONNX by
``franka_sim/export_onnx.py``; this node replays it on hardware.

    /joint_states ────┐                     ┌─► q̈_nom = a·q̈_max
    /cbf/per_link_… ──┼─► observation(24) ──┤   (Float64MultiArray, 7)
    target (param /   │   (identical layout  └─► /NS_1/qddot_nom
     topic) ──────────┘    to FrankaCBFEnv)            │
                                                       ▼
                                          cbf_safety_filter  →  qddot_safe
                                          → qddot_to_torque  →  torque_cmd

Division of labour (the paper's argument): the POLICY owns the task and the
macroscopic avoidance behaviour it learned under the shield; the CBF filter
downstream owns the safety certificate and absorbs the sim-to-real dynamics
gap.  This node deliberately adds NO avoidance logic of its own — that would
be a second, untrained motion planner competing with the policy.

Real-time discipline (mirrors cbf_safety_filter / pentagon_qddot_commander)
--------------------------------------------------------------------------
* every per-tick buffer is preallocated; the tick allocates only the ONNX
  output tensor (onnxruntime owns it) and never builds strings unless a
  throttled log actually fires;
* ONNX Runtime is pinned to a single intra-op thread — a thread pool fighting
  the executor for the GIL is the classic source of 100 Hz jitter;
* all lazy costs (xacro, Pinocchio FK, the first inference) are paid in
  ``__init__`` while the robot is stationary;
* joint-state snapshots use the lock-free double-buffer + atomic swap pattern
  used by the rest of the stack.

Output-gating policy — the node publishes ZEROS (never a stale/again-guessed
command) whenever it cannot produce a trustworthy action:

    warm-up window                  → zeros (robot settles, topics connect)
    joint state stale/missing       → zeros
    perception seen, then stale     → zeros  (a FAULT: the policy's obstacle
                                      slot would be a lie; the CBF filter is
                                      independently braking on the same event)
    perception never seen           → run with a parked synthetic obstacle
                                      (legitimate no-camera configuration)
    target reached & stop_on_success→ zeros

``qddot_nom`` is clamped to ±q̈_max per joint here; joint velocity/position
limits, acceleration continuity and the workspace box are enforced downstream
by ``cbf_safety_filter`` (hard box + hard rows), exactly as for the pentagon
commander.
"""

from __future__ import annotations

import csv
import math
import os
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import numpy as np
import pinocchio as pin
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from franka_msgs.msg import MultiLinkDistance

from franka_experiments.utils.cbf_utils import load_robot_config
from franka_experiments.utils.constants import (
    AUTO_SENTINEL, FR3_JOINT_NAMES, NUM_JOINTS,
)
from franka_experiments.utils.kinematics import (
    compute_ee_fk,
    generate_urdf_from_xacro,
    load_pinocchio_model,
    resolve_arm_joint_ids,
    resolve_frame_id,
)
from franka_experiments.utils.logging_utils import ThrottledLogger, vec_to_str
from franka_experiments.utils.ros import get_namespace_from_config, run_node_main
from franka_experiments.utils.rl_policy import (
    ACT_DIM,
    OBS_DIM,
    action_to_qddot,
    build_observation,
    find_sim_root,
    joint_limits_mismatch,
    load_yaml,
    nearest_obstacle,
    qddot_max_from_limits,
    resolve_model_path,
    resolve_sim_config_path,
    synthetic_obstacle,
)


class RLPolicyCommander(Node):

    def __init__(self):
        super().__init__('rl_policy_commander')

        self.done      = False
        self._stopping = False
        self._stop_end = 0.0

        # ── Robot-side config (topics + limits actually enforced on hardware) ─
        _cfg    = load_robot_config('control')
        _topics = _cfg['topics']
        _params = _cfg.get('params', {})
        _limits = _cfg['joint_limits']

        # ── Parameters ───────────────────────────────────────────────────────
        # Policy / config discovery.  franka_sim is a standalone (non-ROS)
        # module, so paths are parameters — never hard-coded — with a source-tree
        # fallback that works under colcon --symlink-install.
        self.declare_parameter('onnx_model',   '')
        self.declare_parameter('sim_root',     '')
        self.declare_parameter('sim_config',   '')
        # Topics.
        self.declare_parameter('qddot_nom_topic',
                               _topics.get('qddot_nom', '/NS_1/qddot_nom'))
        self.declare_parameter('joint_state_topic', AUTO_SENTINEL)
        self.declare_parameter('per_link_distances_topic',
                               _topics.get('per_link_distances',
                                           '/cbf/per_link_distances'))
        self.declare_parameter('cbf_status_topic',
                               _topics.get('cbf_status', '/NS_1/cbf_status'))
        self.declare_parameter('status_topic', '/NS_1/rl_status')
        self.declare_parameter('target_topic', '')
        # Kinematics.  The sim observation's EE slot is MuJoCo's
        # `attachment_site` = link7 + 0.107 m in z = the FR3 flange = fr3_link8.
        self.declare_parameter('ee_frame', 'fr3_link8')
        # Rates / timeouts (defaults follow fr3_control.yaml).
        self.declare_parameter('rate_hz', 0.0)          # 0 = take sim control rate
        self.declare_parameter('warmup_s', 3.0)
        self.declare_parameter('joint_state_timeout',
                               float(_params.get('joint_state_timeout', 0.1)))
        self.declare_parameter('distance_timeout',
                               float(_params.get('distance_timeout', 0.5)))
        # Task.
        self.declare_parameter('target_xyz', [0.45, 0.0, 0.45])
        self.declare_parameter('target_sequence', [])   # flat [x,y,z, x,y,z, …]
        self.declare_parameter('target_tol', 0.0)       # 0 = take sim task tol
        self.declare_parameter('dwell_s', 1.0)
        self.declare_parameter('stop_on_success', False)
        # Obstacle slot reconstruction (see utils/rl_policy).
        self.declare_parameter('obstacle_radius', 0.0)  # 0 = take sim radius
        self.declare_parameter('no_obstacle_xyz', [1.5, 0.0, 0.5])
        self.declare_parameter('distance_links', [])    # [] = all reported links
        # Deployment derate: q̈_nom = a·q̈_max·action_scale.  ≤ 1 only.
        self.declare_parameter('action_scale', 1.0)
        # Logging.
        self.declare_parameter('log_csv', True)
        self.declare_parameter('log_dir', '')

        gp = self.get_parameter
        qddot_topic  = str(gp('qddot_nom_topic').value)
        js_topic     = str(gp('joint_state_topic').value)
        dist_topic   = str(gp('per_link_distances_topic').value)
        status_topic = str(gp('status_topic').value)
        target_topic = str(gp('target_topic').value)
        ee_frame     = str(gp('ee_frame').value)

        self._warmup_s   = float(gp('warmup_s').value)
        self._js_timeout = float(gp('joint_state_timeout').value)
        self._d_timeout  = float(gp('distance_timeout').value)
        self._dwell_s    = float(gp('dwell_s').value)
        self._stop_ok    = bool(gp('stop_on_success').value)
        self._no_obs_xyz = np.array(gp('no_obstacle_xyz').value, dtype=np.float64)
        self._dist_links = [str(s) for s in (gp('distance_links').value or [])]

        scale = float(gp('action_scale').value)
        if not (0.0 < scale <= 1.0):
            self.get_logger().warn(
                f'action_scale={scale} out of (0, 1] — clamped. It is a '
                'deployment DERATE, it must never widen the trained envelope.')
            scale = min(max(scale, 0.0), 1.0)
        self._action_scale = scale

        # ── Policy + training config ─────────────────────────────────────────
        sim_root = str(gp('sim_root').value) or find_sim_root(__file__)
        model_param = str(gp('onnx_model').value)
        if not model_param:
            raise RuntimeError(
                'Parameter "onnx_model" is empty — this node has nothing to '
                'run. Export a policy first:\n'
                '  python3 -m franka_sim.export_onnx --model '
                '<franka_sim/models/<exp>/best_model.zip>')
        self._model_path = resolve_model_path(model_param, sim_root)

        cfg_path = resolve_sim_config_path(str(gp('sim_config').value),
                                           self._model_path, sim_root)
        sim_cfg = load_yaml(cfg_path) if cfg_path else {}
        if not sim_cfg:
            self.get_logger().warn(
                'No franka_sim config found (sim_config / model dir / '
                f'sim_root="{sim_root}") — falling back to fr3_control.yaml '
                'limits and parameter defaults. Verify that the policy was '
                'trained with these values.')

        # q̈_max: the policy emits a FRACTION of it, so a sim/robot divergence
        # silently rescales every command. Take the ROBOT values (what hardware
        # actually enforces) and report any drift against the training config.
        self._qddot_max = qddot_max_from_limits(_limits)
        sim_limits = sim_cfg.get('joint_limits')
        if sim_limits:
            for line in joint_limits_mismatch(sim_limits, _limits):
                self.get_logger().warn(f'SIM-TO-REAL joint_limits drift — {line}')

        sim_env  = sim_cfg.get('env', {})
        sim_task = sim_cfg.get('task', {})
        sim_obst = sim_cfg.get('obstacle', {})

        rate = float(gp('rate_hz').value) or float(
            sim_env.get('control_rate_hz', 100.0))
        self._dt = 1.0 / rate
        self._target_tol = float(gp('target_tol').value) or float(
            sim_task.get('target_tol', 0.05))
        self._r_obs = float(gp('obstacle_radius').value) or float(
            sim_obst.get('radius', 0.08))

        # ── Targets ──────────────────────────────────────────────────────────
        seq = [float(v) for v in (gp('target_sequence').value or [])]
        if seq and len(seq) % 3 != 0:
            raise RuntimeError(
                f'target_sequence has {len(seq)} values — must be a multiple of 3')
        if seq:
            self._targets = [np.array(seq[i:i + 3]) for i in range(0, len(seq), 3)]
        else:
            self._targets = [np.array(gp('target_xyz').value, dtype=np.float64)]
        self._tgt_idx     = 0
        self._dwell_until = 0.0

        # ── Kinematics (Pinocchio FK for the observation's EE slot) ──────────
        self.pin_model, self.pin_data = load_pinocchio_model(
            generate_urdf_from_xacro())
        self._ee_fid        = resolve_frame_id(self.pin_model, ee_frame)
        self._pin_joint_ids = resolve_arm_joint_ids(self.pin_model)
        self._q_neutral     = pin.neutral(self.pin_model)

        # ── ONNX Runtime session ─────────────────────────────────────────────
        # Single-threaded on purpose: a 100 Hz 2×256 MLP is microseconds of
        # work, and an ORT thread pool only adds scheduling jitter and GIL
        # contention against the executor thread.
        import onnxruntime as ort
        so = ort.SessionOptions()
        so.intra_op_num_threads = 1
        so.inter_op_num_threads = 1
        so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self._sess = ort.InferenceSession(
            self._model_path, sess_options=so, providers=['CPUExecutionProvider'])
        self._in_name = self._sess.get_inputs()[0].name
        in_shape = self._sess.get_inputs()[0].shape
        obs_dim = in_shape[-1] if isinstance(in_shape[-1], int) else OBS_DIM
        if obs_dim != OBS_DIM:
            raise RuntimeError(
                f'ONNX policy expects observation width {obs_dim}, this node '
                f'builds {OBS_DIM}. Model and franka_sim env are out of sync.')
        out_shape = self._sess.get_outputs()[0].shape
        act_dim = out_shape[-1] if isinstance(out_shape[-1], int) else ACT_DIM
        if act_dim != ACT_DIM:
            raise RuntimeError(
                f'ONNX policy emits {act_dim} actions, the FR3 needs {ACT_DIM}. '
                'Wrong model file?')

        # ── Preallocated per-tick state ──────────────────────────────────────
        self._obs_buf   = np.zeros((1, OBS_DIM), dtype=np.float32)
        self._feed      = {self._in_name: self._obs_buf}
        self._qddot_nom = np.zeros(ACT_DIM)
        self._action    = np.zeros(ACT_DIM)
        self._ee_pos    = np.zeros(3)
        self._obs_xyz   = np.zeros(3)
        self._d_min     = 99.0
        self._infer_ms  = 0.0
        self._tick_ms   = 0.0
        self._last_tick = None

        self._out_msg       = Float64MultiArray()
        self._out_msg.data  = [0.0] * ACT_DIM
        self._zero_msg      = Float64MultiArray()
        self._zero_msg.data = [0.0] * ACT_DIM
        self._status_msg      = Float64MultiArray()
        # [infer_ms, tick_ms, d_min, dist_to_target, target_idx, gated]
        self._status_msg.data = [0.0] * 6

        # ── Joint-state double buffer (lock-free swap, see pentagon node) ────
        self._js_lock = threading.Lock()
        self._js_a = {'q': np.zeros(NUM_JOINTS), 'qdot': np.zeros(NUM_JOINTS),
                      'q_full': pin.neutral(self.pin_model), 'valid': False}
        self._js_b = {'q': np.zeros(NUM_JOINTS), 'qdot': np.zeros(NUM_JOINTS),
                      'q_full': pin.neutral(self.pin_model), 'valid': False}
        self._js_write = self._js_a
        self._js_read  = self._js_b
        self._js_imap: Optional[List[int]] = None
        self._js_stamp = 0.0

        # ── Perception / CBF snapshots (immutable, atomic attribute swap) ────
        self._obs_snap: Optional[tuple] = None   # (entries, stamp)
        self._cbf_snap: Optional[tuple] = None   # (n_c, slack, fault, stamp)

        if js_topic == AUTO_SENTINEL:
            ns = get_namespace_from_config()
            js_topic = f'/{ns}/joint_states' if ns else '/joint_states'

        self.create_subscription(JointState, js_topic, self._js_cb,
                                 QoSProfile(depth=1))
        self.create_subscription(
            MultiLinkDistance, dist_topic, self._obs_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Float64MultiArray, self.get_parameter(
            'cbf_status_topic').value, self._cbf_status_cb, QoSProfile(depth=1))
        if target_topic:
            self.create_subscription(PointStamped, target_topic,
                                     self._target_cb, QoSProfile(depth=1))

        self.pub        = self.create_publisher(Float64MultiArray, qddot_topic, 10)
        self._status_pub = self.create_publisher(Float64MultiArray, status_topic, 10)

        # Pay the lazy costs (FK structures, first inference) before the timer
        # starts, not on the first live control tick.
        self._warmup()

        self._t0 = time.monotonic()
        self.timer = self.create_timer(self._dt, self._tick)
        self._tlog = ThrottledLogger(self.get_logger())

        self._init_logger()

        self.get_logger().info(
            f'rl_policy_commander\n'
            f'  policy    : {self._model_path}\n'
            f'  sim config: {cfg_path or "<none>"}\n'
            f'  out topic : {qddot_topic}\n'
            f'  js topic  : {js_topic}\n'
            f'  distances : {dist_topic}\n'
            f'  ee_frame  : {ee_frame}   rate: {rate:.0f} Hz\n'
            f'  targets   : {[t.tolist() for t in self._targets]} '
            f'(tol={self._target_tol} m, dwell={self._dwell_s} s)\n'
            f'  r_obs     : {self._r_obs} m   action_scale: {self._action_scale}\n'
            f'  qddot_max : {self._qddot_max.tolist()} rad/s²')

    # ── Warm-up ──────────────────────────────────────────────────────────────

    def _warmup(self) -> None:
        """Run every per-tick code path once with dummy data, results discarded.

        Same rationale as ``cbf_safety_filter._warmup``: the first Pinocchio FK
        on a never-resolved frame and the first ONNX inference each pay one-shot
        lazy costs (frame placement allocation, kernel selection, memory-arena
        growth) that would otherwise land on the first live 100 Hz tick.
        Failures only warn — warm-up must never block node startup.
        """
        t0 = time.perf_counter()
        try:
            compute_ee_fk(self.pin_model, self.pin_data, self._q_neutral,
                          self._ee_fid)
        except Exception as exc:                                  # noqa: BLE001
            self.get_logger().warn(f'warmup: FK failed: {exc}')
        try:
            self._obs_buf[:] = 0.0
            self._sess.run(None, self._feed)
        except Exception as exc:                                  # noqa: BLE001
            self.get_logger().warn(f'warmup: ONNX inference failed: {exc}')
        self.get_logger().info(
            f'warmup done in {(time.perf_counter() - t0) * 1e3:.1f} ms')

    # ── Callbacks ────────────────────────────────────────────────────────────

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
        self._js_stamp = time.monotonic()

    def _obs_cb(self, msg: MultiLinkDistance) -> None:
        """Per-link FILTERED distances → immutable snapshot for ``_tick``.

        Same decoding convention as ``cbf_safety_filter`` / the pentagon
        commander: ``d`` and ``n̂`` are taken verbatim from the message (the
        engine's LPF'd surface distance and EMA-smoothed normal); entries with a
        degenerate direction are dropped.
        """
        entries = []
        for ld in msg.links:
            if not ld.valid or not math.isfinite(ld.distance):
                continue
            n = np.array([ld.direction.x, ld.direction.y, ld.direction.z])
            n_norm = float(np.linalg.norm(n))
            if n_norm < 0.5:
                continue
            entries.append((
                ld.robot_link_name,
                float(ld.distance),
                n / n_norm,
                np.array([ld.closest_point_human.x,
                          ld.closest_point_human.y,
                          ld.closest_point_human.z]),
            ))
        self._obs_snap = (tuple(entries), time.monotonic())

    def _cbf_status_cb(self, msg: Float64MultiArray) -> None:
        """cbf_status → (n_c, slack, fault). Diagnostics/CSV only.

        The policy was trained WITH the shield in the loop, so it must keep
        seeing the raw task observation; feeding the filter's reaction back into
        the command would be an untrained outer loop. The pentagon commander's
        feasibility governor deliberately has no counterpart here.
        """
        if len(msg.data) >= 3:
            self._cbf_snap = (float(msg.data[0]), float(msg.data[1]),
                              float(msg.data[2]), time.monotonic())

    def _target_cb(self, msg: PointStamped) -> None:
        """External task target — replaces the parameter target/sequence."""
        self._targets = [np.array([msg.point.x, msg.point.y, msg.point.z])]
        self._tgt_idx = 0
        self._dwell_until = 0.0

    # ── Stop ─────────────────────────────────────────────────────────────────

    def request_stop(self, dur: float = 0.5):
        if self._stopping:
            return
        self._stopping = True
        self._stop_end = time.monotonic() + dur
        self.get_logger().info(f'Stopping: zero qddot for {dur} s')

    # ── Main tick ────────────────────────────────────────────────────────────

    def _tick(self) -> None:
        now = time.monotonic()
        if self._last_tick is not None:
            self._tick_ms = (now - self._last_tick) * 1e3
        self._last_tick = now

        if self._stopping:
            self.pub.publish(self._zero_msg)
            if now >= self._stop_end:
                self.timer.cancel()
                self.done = True
            return

        t = now - self._t0

        # ── Gate 1: warm-up window ───────────────────────────────────────────
        if t < self._warmup_s:
            self._publish_zero(gate=1.0)
            return

        # ── Gate 2: joint state present and fresh ────────────────────────────
        with self._js_lock:
            js = self._js_read
        if not js['valid'] or (now - self._js_stamp) > self._js_timeout:
            if self._tlog.due(t):
                self._tlog.warn(
                    'joint state missing or stale '
                    f'({now - self._js_stamp:.3f} s > {self._js_timeout} s) '
                    '— publishing zeros')
            self._publish_zero(gate=2.0)
            return
        q, qdot, q_full = js['q'], js['qdot'], js['q_full']

        # ── Observation: EE position (FK) ────────────────────────────────────
        np.copyto(self._ee_pos,
                  compute_ee_fk(self.pin_model, self.pin_data, q_full,
                                self._ee_fid).translation)

        # ── Observation: obstacle slot ───────────────────────────────────────
        snap = self._obs_snap
        if snap is None:
            # Perception never started (legitimate: enable_camera:=false).
            c, d = synthetic_obstacle(self._ee_pos, self._no_obs_xyz, self._r_obs)
            np.copyto(self._obs_xyz, c)
            self._d_min = d
        elif (now - snap[1]) > self._d_timeout:
            # Perception WAS running and went stale → safety-chain fault. The
            # obstacle slot would be a lie, and cbf_safety_filter is already
            # braking on the same event; stop commanding.
            if self._tlog.due(t):
                self._tlog.warn(
                    f'per-link distances stale ({now - snap[1]:.2f} s > '
                    f'{self._d_timeout} s) — publishing zeros')
            self._publish_zero(gate=3.0)
            return
        else:
            found = nearest_obstacle(snap[0], self._r_obs, self._dist_links)
            if found is None:
                c, d = synthetic_obstacle(self._ee_pos, self._no_obs_xyz,
                                          self._r_obs)
                np.copyto(self._obs_xyz, c)
                self._d_min = d
            else:
                np.copyto(self._obs_xyz, found[0])
                self._d_min = found[1]

        # ── Task: target advance / dwell ─────────────────────────────────────
        # Read the list ONCE: _target_cb may replace it (with a shorter one)
        # between statements if a future refactor puts it in another callback
        # group, and an index into a stale length would raise inside the loop.
        targets = self._targets
        self._tgt_idx = idx = self._tgt_idx % len(targets)
        target = targets[idx]
        dist   = float(np.linalg.norm(self._ee_pos - target))
        if dist < self._target_tol:
            if len(targets) > 1:
                if self._dwell_until == 0.0:
                    self._dwell_until = now + self._dwell_s
                elif now >= self._dwell_until:
                    self._tgt_idx = (idx + 1) % len(targets)
                    self._dwell_until = 0.0
                    self.get_logger().info(
                        f'target reached — advancing to #{self._tgt_idx} '
                        f'{targets[self._tgt_idx].tolist()}')
            elif self._stop_ok:
                self._publish_zero(gate=4.0)
                self._log_row_write(t, q, qdot, target, dist)
                return
        else:
            self._dwell_until = 0.0

        # ── Inference ────────────────────────────────────────────────────────
        build_observation(q, qdot, self._ee_pos, target, self._obs_xyz,
                          self._d_min, out=self._obs_buf)
        t_inf = time.perf_counter()
        action = self._sess.run(None, self._feed)[0][0]
        self._infer_ms = (time.perf_counter() - t_inf) * 1e3
        np.copyto(self._action, np.clip(action, -1.0, 1.0))

        action_to_qddot(self._action, self._qddot_max, self._action_scale,
                        out=self._qddot_nom)

        out = self._out_msg.data
        for i in range(ACT_DIM):
            out[i] = float(self._qddot_nom[i])
        self.pub.publish(self._out_msg)
        self._publish_status(dist, gate=0.0)
        self._log_row_write(t, q, qdot, target, dist)

        if self._tlog.due(t):
            self._tlog.info(
                f'[t={t:5.1f}s] dist={dist:.3f} m  d_min={self._d_min:.3f} m  '
                f'infer={self._infer_ms:.2f} ms  tick={self._tick_ms:.2f} ms  '
                f'qddot_nom=[{vec_to_str(self._qddot_nom, ".2f")}]')

    # ── Output helpers ───────────────────────────────────────────────────────

    def _publish_zero(self, gate: float) -> None:
        self._qddot_nom[:] = 0.0
        self._action[:] = 0.0
        self.pub.publish(self._zero_msg)
        self._publish_status(float('nan'), gate=gate)

    def _publish_status(self, dist: float, gate: float) -> None:
        d = self._status_msg.data
        d[0] = self._infer_ms
        d[1] = self._tick_ms
        d[2] = self._d_min
        d[3] = dist
        d[4] = float(self._tgt_idx)
        d[5] = gate
        self._status_pub.publish(self._status_msg)

    # ── CSV logging (jitter / sim-to-real evidence for the paper) ────────────

    def _init_logger(self) -> None:
        """Open a timestamped CSV once and write the header row.

        Columns give the paper's timing figure (``tick_ms``/``infer_ms``
        jitter) plus the full observation actually fed to the network, so a run
        can be replayed offline against the sim policy.  Any failure disables
        logging and the node keeps running.
        """
        self._log_file = None
        self._csv_writer = None
        if not bool(self.get_parameter('log_csv').value):
            return

        header  = ['time', 'tick_ms', 'infer_ms']
        header += [f'q{i}' for i in range(1, 8)]
        header += [f'dq{i}' for i in range(1, 8)]
        header += ['ee_x', 'ee_y', 'ee_z']
        header += ['target_x', 'target_y', 'target_z', 'target_idx', 'dist']
        header += ['obs_x', 'obs_y', 'obs_z', 'd_min']
        header += [f'action_{i}' for i in range(1, 8)]
        header += [f'qddot_nom_{i}' for i in range(1, 8)]
        header += ['cbf_n_c', 'cbf_slack', 'cbf_fault']
        self._log_row = [0.0] * len(header)

        log_dir = str(self.get_parameter('log_dir').value)
        if not log_dir:
            # Repo convention: <franka_experiments>/franka_logs, found relative
            # to the source file (realpath survives --symlink-install) so no
            # container-absolute path is baked in. ~/franka_logs is the fallback
            # when the package is installed outside a source checkout.
            src_logs = Path(os.path.realpath(__file__)).parents[2] / 'franka_logs'
            log_dir = str(src_logs if src_logs.parent.is_dir()
                          else Path.home() / 'franka_logs')
        try:
            Path(log_dir).mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            path = Path(log_dir) / f'rl_policy_run_{stamp}.csv'
            self._log_file = open(path, 'w', newline='', buffering=1 << 20)
            self._csv_writer = csv.writer(self._log_file)
            self._csv_writer.writerow(header)
            self.get_logger().info(f'Logging to {path}')
        except Exception as exc:                                  # noqa: BLE001
            self._log_file = None
            self._csv_writer = None
            self.get_logger().warn(
                f'Could not open log file in {log_dir}: {exc} — '
                'continuing without logging.')

    def _log_row_write(self, t, q, qdot, target, dist) -> None:
        if self._csv_writer is None:
            return
        row = self._log_row
        o = 0
        row[o] = float(t);              o += 1
        row[o] = float(self._tick_ms);  o += 1
        row[o] = float(self._infer_ms); o += 1
        for i in range(NUM_JOINTS):
            row[o + i] = float(q[i])
        o += NUM_JOINTS
        for i in range(NUM_JOINTS):
            row[o + i] = float(qdot[i])
        o += NUM_JOINTS
        for i in range(3):
            row[o + i] = float(self._ee_pos[i])
        o += 3
        for i in range(3):
            row[o + i] = float(target[i])
        o += 3
        row[o] = float(self._tgt_idx);  o += 1
        row[o] = float(dist);           o += 1
        for i in range(3):
            row[o + i] = float(self._obs_xyz[i])
        o += 3
        row[o] = float(self._d_min);    o += 1
        for i in range(ACT_DIM):
            row[o + i] = float(self._action[i])
        o += ACT_DIM
        for i in range(ACT_DIM):
            row[o + i] = float(self._qddot_nom[i])
        o += ACT_DIM
        cbf = self._cbf_snap
        row[o] = float(cbf[0]) if cbf else 0.0;     o += 1
        row[o] = float(cbf[1]) if cbf else 0.0;     o += 1
        row[o] = float(cbf[2]) if cbf else 0.0
        self._csv_writer.writerow(row)

    def destroy_node(self):
        if getattr(self, '_log_file', None) is not None:
            try:
                self._log_file.close()
            except Exception:                                     # noqa: BLE001
                pass
            self._log_file = None
        return super().destroy_node()


def main(args=None):
    run_node_main(RLPolicyCommander, args=args)


if __name__ == '__main__':
    main()
