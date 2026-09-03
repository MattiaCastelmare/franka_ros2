#!/usr/bin/env python3
"""CBF safety filter — acceleration-level, three rates, one process.

    I/O callbacks (event-driven)     perception group        control group
    ────────────────────────────     ────────────────        ─────────────
    /joint_states  ─┐                50 Hz: Pinocchio        100 Hz: OSQP
    /qddot_nom     ─┼► snapshots ──► FK + point Jacobians ─► solve + publish
    /per_link_dist ─┘  (atomic       ConstraintBuilder       /qddot_safe
                        swap)        (atomic swap)

Per QP tick:

    min ½‖q̈ − q̈_nom‖² + ½ Σ_g ρ_g s_g²
    s.t.  q̈_min ≤ q̈ ≤ q̈_max                     (state box, hard)
          aᵢᵀq̈ + mᵢ·s_g ≥ bᵢ   ∀ row i           (CBF rows, relaxable)

This module is the ORCHESTRATOR and nothing else. It owns the ROS wiring, the
three clocks, the staleness policy and the OSQP instance. It owns no formula:

* what a row IS            → utils.cbf_state_rows (all six families + builder)
* the rows' right-hand side→ utils.cbf_qp_assembly.build_row_rhs
* the hard state box       → utils.cbf_hard_limits
* the singularity barrier  → utils.cbf_singularity
* every parameter          → config/fr3_control.yaml, via utils.config
* the CBFDIAG line         → utils.logging_utils.format_cbf_diag

Staleness policy, all of it in one place (``_qp_tick``):

    joint state older than joint_state_timeout   → brake on last known q̇
    joint state FROZEN (identical, re-stamped)   → brake, CBF rows dropped
    distances older than distance_timeout        → brake, CBF rows dropped
    q̈_nom older than nom_timeout                 → brake
    QP not solved                                → brake, reset the warm start

Every one of those degrades toward braking, never toward passthrough, and
raises ``fault_braking`` on /cbf_status.
"""

import gc
import os
import threading
import time

import numpy as np
import osqp
import pinocchio as pin
import rclpy
import scipy.sparse as sparse
from franka_msgs.msg import MultiLinkDistance
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from franka_experiments.utils.cbf_hard_limits import (
    apply_slew_limit,
    position_velocity_accel_box,
)
from franka_experiments.utils.cbf_qp_assembly import (
    build_osqp_A,
    build_osqp_bounds,
    build_row_rhs,
)
from franka_experiments.utils.cbf_state_rows import (
    FR3_JOINT_KEYS,
    FR3_JOINTS,
    G_CAP,
    G_OBS,
    G_QLIM,
    G_SC,
    G_SING,
    G_SPD,
    NV,
    NX,
    N_SLACK,
    ConstraintBuilder,
    JointSnap,
    NomSnap,
    Obstacle,
    ObstacleSnap,
    build_optional_row_builders,
)
from franka_experiments.utils.config import (
    load_cbf_config,
    load_franka_joint_limits,
)
from franka_experiments.utils.kinematics import (
    CBFKinematics,
    build_urdf_no_hand,
)
from franka_experiments.utils.logging_utils import (
    format_cbf_diag,
    format_velocity_summary,
)


class CBFSafetyFilter(Node):

    # ═════════════════════════════════════════════════════════════════════
    #  Construction
    # ═════════════════════════════════════════════════════════════════════

    def __init__(self):
        super().__init__('cbf_safety_filter')

        # ── 1. Configuration ────────────────────────────────────────────
        # Every knob comes from config/fr3_control.yaml and from nowhere else;
        # utils.config.CBF_PARAM_SPEC carries only the type and the validation
        # range. A missing key is a startup failure naming the key.
        topics, P = load_cbf_config(self)
        self.P = P

        # ── 2. State limits, straight from franka_description ───────────
        # NOT from the joint_limits: block at the bottom of fr3_control.yaml —
        # that one is read by four other nodes and the two can drift.
        jl = load_franka_joint_limits(FR3_JOINT_KEYS)
        self._lb, self._ub = -jl['decel_max'], jl['decel_max']
        self._qdot_max = jl['qdot_max']
        self._q_min, self._q_max = jl['q_min'], jl['q_max']

        # ── 3. Kinematics and the row builders ──────────────────────────
        kin = CBFKinematics(pin.buildModelFromUrdf(build_urdf_no_hand()))
        opt = build_optional_row_builders(P, kin, self.get_logger())

        # The one object that turns a (joint state, obstacle) pair into rows.
        # Stateful — barrier smoothing, per-track velocity filters, frame
        # counters — and driven at the constraint rate, never on the QP tick.
        self._rows = ConstraintBuilder(
            P, kin, q_min=self._q_min, q_max=self._q_max,
            acc_lb=self._lb, acc_ub=self._ub, logger=self.get_logger(), **opt)

        # ── 4. QP, preallocated once ────────────────────────────────────
        # Slack penalty is QUADRATIC (½ρs²) and one slack per FAMILY. Quadratic
        # is C¹ at s=0 and prices small violations softly; per-family because a
        # single shared slack let a joint-limit row in RADIANS relax every
        # self-collision row in METRES by the same amount, until the firmware
        # fired its own reflex.
        P_mat = np.eye(NX)
        for g, rho in ((G_OBS,  P.rho_slack),
                       (G_SC,   P.rho_slack_self_collision),
                       (G_QLIM, P.rho_slack_joint_limit),
                       (G_SING, P.rho_slack_singularity),
                       (G_CAP,  P.rho_slack_retreat),
                       (G_SPD,  P.rho_slack_link_speed)):
            P_mat[NV + g, NV + g] = rho
        self._P_csc = sparse.csc_matrix(P_mat)
        self._qvec = np.zeros(NX)
        self._box_lb = np.concatenate([self._lb, np.zeros(N_SLACK)])
        self._box_ub = np.concatenate([self._ub, np.full(N_SLACK, 1e6)])
        self._osqp_prob = None
        self._prev_nc = -1
        self._qp_fail_count = 0
        self._dt_qp = 1.0 / P.qp_rate_hz

        # ── 5. Shared state (lock-free: one immutable snapshot per producer,
        #       published by a single atomic attribute assignment) ────────
        self._js = self._nom = self._obs = self._con = None
        self._js_frozen_since = None
        self._qdot_cbf = np.zeros(NV)
        self._qddot_prev = np.zeros(NV)

        # ── 6. Diagnostics ──────────────────────────────────────────────
        self._diag_slack = np.zeros(N_SLACK)
        self._diag_vel_ratio = np.zeros(NV)
        self._diag_vel_bite = np.zeros(NV, dtype=bool)
        self._diag_slew_step = np.zeros(NV)
        self._diag_slew_bite = np.zeros(NV, dtype=bool)
        self._diag_qddot_real = np.zeros(NV)
        self._diag_qdot_prev = self._diag_t_prev = None
        self._diag_cap_age = 0.0
        self._diag_caps = (0.0, 0.0, 0.0, 0.0)
        self._cap_warned = False
        self._last_diag_t = 0.0
        self._last_tick_t = None
        self._tick_count = 0
        self._priority_set = False
        self._gap_warn_thr_ms = P.tick_gap_warn_factor * 1000.0 * self._dt_qp

        # ── 7. ROS wiring ───────────────────────────────────────────────
        # ── Callback groups: QP loop must never wait on perception ──────────
        grp_io   = MutuallyExclusiveCallbackGroup()
        grp_perc = MutuallyExclusiveCallbackGroup()
        grp_ctrl = MutuallyExclusiveCallbackGroup()

        # depth=1: always consume the latest sample, never drain a backlog
        # joint_states_fast, NOT joint_states: the latter comes from a 30 Hz
        # Python republisher that re-stamps its CACHED values, so a stall in it
        # is invisible to every staleness check here (see the topics block in
        # fr3_control.yaml). Falls back to the old key if the config predates it.
        js_topic = topics.get('joint_states_fast',
                              topics['joint_states_topic'])
        self.create_subscription(
            JointState, js_topic, self._on_joint_state,
            QoSProfile(depth=1), callback_group=grp_io)
        self.create_subscription(
            Float64MultiArray, topics['qddot_nom'], self._on_qddot_nom,
            QoSProfile(depth=1), callback_group=grp_io)
        self.create_subscription(
            MultiLinkDistance, topics['per_link_distances'], self._on_distances,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=grp_io)

        self._pub = self.create_publisher(
            Float64MultiArray, topics['qddot_safe'], 10)

        # CBF activity status for downstream consumers.
        # data = [n_active_constraints, slack, fault_braking, n_active_cps,
        #         d_obstacle_min].
        # data[4] is the closest OBSTACLE surface gap [m], +inf when nothing is
        # in range. It is NOT min(h_bar) + d_safe: h_bar mixes obstacle rows
        # with joint-limit rows measured in radians. pentagon_qddot_commander
        # reads it to scale its phase rate — slow the trajectory down only when
        # something is genuinely close, never on tracking error (an error-driven
        # governor deadlocks: CBF blocks -> error grows -> phase freezes ->
        # reference parked on the blocked pose -> CBF keeps blocking).
        # data[3] (n_active_cps) counts the rows whose barrier is actually
        # VIOLATED (h̄ < 0, i.e. the CP is inside d_safe), which is the honest
        # "how many control points triggered this cycle" figure. data[0] is the
        # larger count of rows PRESENT in the QP — every CP inside
        # cbf_obstacle_horizon, most of them non-binding. Both consumers
        # (frame_grabber, rl_policy_commander) index positionally behind a
        # len() guard, so appending a 4th element is backward compatible.
        # Today the only subscriber is frame_grabber.py, which uses it to gate
        # frame saving (save while CBF active or fault-braking). NOTE: no
        # consumer currently freezes virtual time on this signal — a "freeze
        # virtual time while CBF active" motion generator is planned (roadmap)
        # but not yet implemented; do not read this comment as if it happens.
        # fault_braking=1 marks a SAFETY-CHAIN fault that forced braking with no
        # geometric CBF rows (distance stale, or QP failure) — distinct from
        # n_active_constraints=0 during normal "no obstacle nearby" operation.
        self._status_pub = self.create_publisher(
            Float64MultiArray, topics.get('cbf_status', '/NS_1/cbf_status'), 10)
        self._status_msg = Float64MultiArray()
        self._status_msg.data = [0.0, 0.0, 0.0, 0.0, float('inf')]

        # ── 8. Start ────────────────────────────────────────────────────
        # Warm-up FIRST: the one-shot lazy costs (Pinocchio's first FK, OSQP's
        # first factorization, numpy's first BLAS call) stalled the first real
        # tick by 400-800 ms. Pay them here, with the robot stationary.
        self._warmup()
        if P.diag_disable_gc:
            # Diagnostic only — used to test whether _qp_tick gaps coincide with
            # GC pauses. Memory can grow unbounded if cyclic garbage collects.
            gc.disable()
            self.get_logger().warn('diag_disable_gc=TRUE → gc.disable(). '
                                   'TEMPORARY diagnostic mode, not for normal use.')
        self.create_timer(1.0 / P.cbf_update_rate_hz, self._update_constraints,
                          callback_group=grp_perc)
        self.create_timer(1.0 / P.qp_rate_hz, self._qp_tick,
                          callback_group=grp_ctrl)
        self.get_logger().info(
            f'CBF filter  QP={P.qp_rate_hz:.0f} Hz  rows={P.cbf_update_rate_hz:.0f} Hz  '
            f'solver={P.qp_solver}\n'
            f'  config: {P.config_path}\n'
            f'  d_safe={P.d_safe} m  horizon={P.cbf_obstacle_horizon} m  '
            f'k0={P.k0_cbf} k1={P.k1_cbf} rho={P.rho_slack}\n'
            f'  limits from franka_description: q margin={P.position_margin_rad} rad, '
            f'brake_eta={P.position_brake_eta}, qdot at {P.velocity_box_margin:.0%}, '
            f'qddot +-{np.round(self._ub, 2).tolist()} rad/s^2')

    def _warmup(self) -> None:
        """Run every per-tick code path once with dummy data, results discarded.

        Measured in the logs: the first real _qp_tick / _update_constraints after
        startup stalled ~400-800 ms (the tick=442 solve=391 ms event, then an
        830 ms gap), all during pentagon's WARMUP phase with the robot stationary.
        Cause is one-shot lazy cost paid on first use: first Pinocchio FK/Jacobian
        on a never-resolved frame, first _frame_id() cache miss, first OSQP setup
        + lazy library init, cold memory pages. Doing it here moves that cost into
        node construction — where the robot is not moving and a delay has no
        safety/motion consequence — instead of onto the first live control tick.

        Nothing is published (no _pub / _status_pub); the node's real-tick OSQP
        state (_osqp_prob / _prev_nc) is left untouched by using throwaway OSQP
        instances. Any sub-step that fails only logs a warning and is skipped, so
        warm-up can never block node startup.
        """
        t0      = time.perf_counter()
        q0      = np.zeros(NV)
        qdot0   = np.zeros(NV)
        p_dummy = np.array([0.3, 0.0, 0.5])      # plausible workspace point

        # (a) Pinocchio FK + Jacobian internal structures. with_jdot=True so the
        #     Jacobian-time-variation pass (used per real tick now) is also warmed.
        try:
            self._rows._kin.update(q0, qdot0, with_jdot=True)
        except Exception as exc:
            self.get_logger().warn(f'warmup: kin.update failed: {exc}')

        # (c) Populate _fid_cache for every link that can appear as
        #     robot_link_name in MultiLinkDistance (fr3_link0..link8), and
        # (b) warm the point-Jacobian path on the first link that resolves.
        first_fid = None
        for link in (f'fr3_link{i}' for i in range(9)):
            try:
                fid = self._rows._frame_id(link)
                if fid is not None and first_fid is None:
                    first_fid = fid
            except Exception as exc:
                self.get_logger().warn(f"warmup: _frame_id('{link}') failed: {exc}")
        if first_fid is not None:
            try:
                self._rows._kin.point_jacobian(first_fid, p_dummy)   # warms J and J̇ paths
            except Exception as exc:
                self.get_logger().warn(f'warmup: point_jacobian failed: {exc}')
        else:
            self.get_logger().warn('warmup: no robot link resolved — Jacobian path not warmed')

        # (d) Exercise the native-OSQP paths used in _qp_tick for both n_c=0 and
        #     n_c=1 (setup + update(Ax) + solve), on throwaway problems. Also pays
        #     osqp's lazy library-init cost here rather than on the first tick.
        try:
            l0, u0 = build_osqp_bounds(None, None, self._box_lb, self._box_ub)
            prob0 = osqp.OSQP()
            prob0.setup(P=self._P_csc, q=self._qvec, A=build_osqp_A(None, NV, N_SLACK),
                        l=l0, u=u0, warm_start=True,
                        max_iter=self.P.osqp_max_iter, verbose=False)
            prob0.solve()

            G1 = np.zeros((1, NX)); G1[0, NV + G_OBS] = -1.0   # dummy row
            h1 = np.array([1.0])
            l1, u1 = build_osqp_bounds(G1, h1, self._box_lb, self._box_ub)
            prob1 = osqp.OSQP()
            prob1.setup(P=self._P_csc, q=self._qvec, A=build_osqp_A(G1, NV, N_SLACK),
                        l=l1, u=u1, warm_start=True,
                        max_iter=self.P.osqp_max_iter, verbose=False)
            prob1.update(q=self._qvec, l=l1, u=u1, Ax=build_osqp_A(G1, NV, N_SLACK).data)
            prob1.solve()
        except Exception as exc:
            self.get_logger().warn(f'warmup: OSQP path failed: {exc}')

        dt_ms = (time.perf_counter() - t0) * 1e3
        self.get_logger().info(
            f'warmup complete in {dt_ms:.1f} ms — one-shot lazy costs paid in '
            f'__init__ (robot stationary), not on the first control tick')

    def _on_joint_state(self, msg: JointState) -> None:
        n2p = dict(zip(msg.name, msg.position))
        n2v = dict(zip(msg.name, msg.velocity))
        try:
            q    = np.array([n2p[n] for n in FR3_JOINTS])
            qdot = np.array([n2v[n] for n in FR3_JOINTS])
        except KeyError:
            return
        prev = self._js
        if (prev is not None
                and np.array_equal(q, prev.q)
                and np.array_equal(qdot, prev.qdot)):
            # Identical to the previous message. Remember when the run STARTED
            # (the previous message's stamp), so the QP thread can age it.
            if self._js_frozen_since is None:
                self._js_frozen_since = prev.stamp
        else:
            self._js_frozen_since = None
        self._js = JointSnap(q, qdot, self._now())

        # ── DIAGNOSTIC ONLY — realized q̈ estimate (see __init__; NOT control) ─
        # Finite difference of MEASURED q̇ → q̈_real, EMA-smoothed. Δt uses the
        # ROS header stamp (sensor/controller time), NOT receipt wall-time:
        # callback-scheduling jitter would corrupt the derivative (a late
        # callback inflates Δt and deflates the estimate). Guarded to a sane
        # joint_states interval so a dropped/duplicated stamp can't blow it up.
        t_hdr = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._diag_qdot_prev is not None and self._diag_t_prev is not None:
            dt = t_hdr - self._diag_t_prev
            if 1e-4 < dt < 0.1:
                raw = (qdot - self._diag_qdot_prev) / dt
                a   = self.P.diag_qddot_alpha
                self._diag_qddot_real = a * self._diag_qddot_real + (1.0 - a) * raw
        self._diag_qdot_prev = qdot
        self._diag_t_prev    = t_hdr

    def _on_qddot_nom(self, msg: Float64MultiArray) -> None:
        data = np.asarray(msg.data, dtype=np.float64)
        if data.shape == (NV,):
            self._nom = NomSnap(data, self._now())

    def _on_distances(self, msg: MultiLinkDistance) -> None:
        P = self.P
        items = tuple(
            Obstacle(
                link=ld.robot_link_name,
                d=float(ld.distance),
                pr=np.array([ld.closest_point_robot.x,
                             ld.closest_point_robot.y,
                             ld.closest_point_robot.z]),
                ph=np.array([ld.closest_point_human.x,
                             ld.closest_point_human.y,
                             ld.closest_point_human.z]),
                conf=float(ld.confidence),
            )
            for ld in msg.links if ld.valid
        )
        now = self._now()
        # Capture time, with a plausibility guard: an unset header stamp reads
        # as 0.0 and would make Δt ≈ 1.8e9 s, silently zeroing every velocity
        # estimate; a clock skew the other way would make Δt negative. Accept it
        # only when it sits in a sane window behind `now`, else fall back to the
        # receipt time (which reproduces the pre-fix behaviour exactly).
        t_cap = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = now - t_cap
        if not (-P.distance_capture_skew_tol < age < P.distance_capture_age_max):
            if t_cap != 0.0 or self._cap_warned is False:
                self.get_logger().warn(
                    f'per_link_distances header stamp implausible '
                    f'(age={age:.3f} s) → falling back to receipt time; '
                    f'v_obs keeps the pre-fix jitter',
                    throttle_duration_sec=5.0)
                self._cap_warned = True
            t_cap = now
        self._diag_cap_age = age
        self._obs = ObstacleSnap(items, now, t_cap)

    # ═════════════════════════════════════════════════════════════════════
    #  Perception rate (50 Hz) — geometry only; Pinocchio lives here
    # ═════════════════════════════════════════════════════════════════════

    def _update_constraints(self) -> None:
        js, obs = self._js, self._obs
        if js is not None and obs is not None:
            self._con = self._rows.build(js, obs, self._now())

    def _elevate_thread_priority(self) -> None:
        """Put *only* the calling (QP) thread on SCHED_FIFO at moderate priority.

        On Linux, scheduling policy/priority are per-thread and sched_setscheduler
        targets a kernel TID; passing threading.get_native_id() (this thread's
        TID) elevates ONLY this thread — the grp_io / grp_perc executor threads
        keep default scheduling. Must be called from inside the QP callback, not
        __init__: the executor spawns the per-group thread only after spin().

        Priority 50 (mid SCHED_FIFO range) deliberately leaves headroom above it
        for the real 1 kHz robot controller, which must outrank this Python node.
        Degrades gracefully: if the process may not raise its RT priority the
        node keeps running at default scheduling (logs a warning, no crash).

        Privilege comes from an RTPRIO rlimit, NOT a capability: this is the same
        real-time setup libfranka already requires. With the user in a `realtime`
        group and /etc/security/limits.conf granting
            @realtime  -  rtprio  99
            @realtime  -  memlock unlimited
        an unprivileged process may call sched_setscheduler(SCHED_FIFO, prio) for
        any prio ≤ the soft RTPRIO limit — no CAP_SYS_NICE, no root. Prefer this
        over `setcap cap_sys_nice+ep` on the interpreter: file caps mark python3
        as AT_SECURE, which strips LD_LIBRARY_PATH/PYTHONPATH and breaks ROS 2's
        env-based library/package discovery. Verify the limit with `ulimit -r`
        (already confirmed 99 on this machine). Do NOT request root here.
        """
        try:
            tid = threading.get_native_id()          # this QP thread's kernel TID
            os.sched_setscheduler(tid, os.SCHED_FIFO, os.sched_param(50))
            self.get_logger().info(
                f'_qp_tick thread (tid={tid}) elevated to SCHED_FIFO prio 50')
        except PermissionError:
            self.get_logger().warn(
                'Cannot set SCHED_FIFO (RTPRIO rlimit too low) — _qp_tick runs '
                'at default scheduling. Add the user to the `realtime` group and '
                'set `rtprio 99` in /etc/security/limits.conf (same setup as '
                'libfranka); check with `ulimit -r`.')
        except Exception as exc:
            self.get_logger().warn(f'_elevate_thread_priority failed: {exc}')

    # ═════════════════════════════════════════════════════════════════════
    #  Control rate (100 Hz) — read snapshots, solve, publish
    # ═════════════════════════════════════════════════════════════════════

    def _qp_tick(self) -> None:
        P = self.P
        if not self._priority_set:                      # one-shot, needs the
            self._priority_set = True                   # executor's own thread
            self._elevate_thread_priority()
        self._watch_tick_gap()

        js = self._js
        if js is None:
            return
        now = self._now()

        # ── STEP 1: is the state usable at all? ─────────────────────────
        # Both failures degrade to braking on the last known q̇. Zeros would
        # NOT: q̈ = 0 reads as "hold this velocity" all the way down the chain,
        # so the arm coasts through the outage — measured, twice, ending in a
        # joint_velocity_violation reflex.
        if now - js.stamp > P.joint_state_timeout:
            self.get_logger().error('joint state stale → braking on last known q̇',
                                    throttle_duration_sec=0.5)
            self._publish(np.clip(-P.k_brake * js.qdot, self._lb, self._ub))
            self._publish_status(0, 0.0, fault=1.0, n_act=0)
            return
        qdot = js.qdot
        js_frozen = self._joint_state_frozen(now, qdot)

        # ── STEP 2: the nominal command we are filtering ────────────────
        nom = self._nom
        fresh_nom = nom is not None and now - nom.stamp < P.nom_timeout
        if not fresh_nom:
            self.get_logger().warn('qddot_nom stale → braking',
                                   throttle_duration_sec=2.0)
        qddot_nom = nom.qddot if fresh_nom else -P.k_brake * qdot
        if js_frozen:
            qddot_nom = -P.k_brake * qdot           # overrides even a fresh one

        # ── STEP 3: the CBF rows, and their right-hand side ─────────────
        # The rows themselves were built at 50 Hz; only their RHS is refreshed
        # here, with this tick's q̇.
        con, obs = self._con, self._obs
        G = h_qp = None
        n_c = n_active = 0
        fault = 0.0
        if js_frozen:
            fault = 1.0                             # blind: no rows, brake
        elif con is not None and now - con.t_dist < P.distance_timeout:
            n_c, G = con.A.shape[0], con.G
            n_active = int(np.count_nonzero(con.h_bar < 0.0))
            self._smooth_qdot(qdot)
            h_qp, self._diag_caps = build_row_rhs(
                con, qdot, self._qdot_cbf, k0=P.k0_cbf, k1=P.k1_cbf,
                retreat_horizon=P.retreat_cap_horizon_s,
                speed_horizon=P.link_speed_horizon_s)
        elif obs is not None and now - obs.stamp > P.distance_timeout:
            # Perception was received once and has since gone stale — a failure
            # of the channel that feeds the barrier must degrade toward MORE
            # conservative, never toward silent passthrough. (Obstacles merely
            # out of range keep obs fresh, so they do not land here.)
            qddot_nom = -P.k_brake * qdot
            fault = 1.0
            self.get_logger().warn('distance stale → braking fallback (CBF inactive)',
                                   throttle_duration_sec=2.0)

        # ── STEP 4: the hard state box ──────────────────────────────────
        # Underneath every row, and NOT relaxable: one integration step must not
        # push |q̇| past the limit, and the position braking curve must keep the
        # joint able to stop. Then the slew box, so the arm can track what comes
        # out. Both mutate _box_lb/_box_ub in place, before the bounds are read.
        self._qvec[:NV] = -qddot_nom
        self._diag_vel_ratio, self._diag_vel_bite = position_velocity_accel_box(
            js.q, qdot, acc_lb=self._lb, acc_ub=self._ub,
            qdot_max=self._qdot_max, v_margin=P.velocity_box_margin,
            q_min=self._q_min, q_max=self._q_max,
            q_margin=P.position_margin_rad, brake_eta=P.position_brake_eta,
            dt=self._dt_qp, relax_dt=P.state_box_relax_s,
            out_lb=self._box_lb[:NV], out_ub=self._box_ub[:NV])
        if P.slew_box_enabled:
            self._box_lb[:NV], self._box_ub[:NV] = apply_slew_limit(
                self._box_lb[:NV], self._box_ub[:NV],
                self._qddot_prev, P.max_qddot_delta)
        if float(np.max(self._diag_vel_ratio)) > P.diag_vel_ratio_thr:
            self.get_logger().info(
                f'VELHI t={now:.3f} n_c={n_c} '
                + format_velocity_summary(qdot, self._diag_vel_ratio,
                                          self._diag_vel_bite))

        # ── STEP 5: solve ───────────────────────────────────────────────
        qddot_safe, slack, solve_ms, res = self._solve(G, h_qp, n_c)
        if qddot_safe is None:
            qddot_safe = np.clip(-P.k_brake * qdot, self._lb, self._ub)
            fault = 1.0

        # ── STEP 6: publish, then report ────────────────────────────────
        if P.slew_box_enabled:
            np.subtract(qddot_safe, self._qddot_prev, out=self._diag_slew_step)
            np.greater(np.abs(self._diag_slew_step), P.max_qddot_delta - 1e-6,
                       out=self._diag_slew_bite)
        self._publish(qddot_safe)
        self._publish_status(n_c, slack, fault, n_active,
                             con.d_obs_min if con is not None else float('inf'))
        self._report(now, con, h_qp, n_c, n_active, qdot, qddot_safe,
                     qddot_nom, slack, solve_ms, res)

    # ── QP tick helpers ──────────────────────────────────────────────────

    def _watch_tick_gap(self) -> None:
        """Warn when the executor did not come back on time."""
        t = time.perf_counter()
        if self._last_tick_t is not None:
            gap_ms = (t - self._last_tick_t) * 1e3
            if gap_ms > self._gap_warn_thr_ms:
                self.get_logger().warn(f'_qp_tick gap: {gap_ms:.1f} ms')
        self._last_tick_t = t

    def _joint_state_frozen(self, now: float, qdot) -> bool:
        """A re-stamped but UNCHANGED state, which the timeout cannot see.

        Strictly worse than a dead topic: the velocity box reads a q̇ that never
        grows, so it never tightens, while the real joint accelerates. This is
        what a 30 Hz republisher does when it stalls — it keeps emitting its
        cached values with fresh header stamps.
        """
        P = self.P
        if (self._js_frozen_since is None
                or (now - self._js_frozen_since) <= P.joint_state_freeze_timeout
                or float(np.max(np.abs(qdot))) <= P.joint_state_freeze_min_speed):
            return False
        self.get_logger().error(
            f'joint state FROZEN for {now - self._js_frozen_since:.3f} s '
            f'(identical q/q̇, fresh stamps) while |q̇|max='
            f'{float(np.max(np.abs(qdot))):.2f} rad/s → braking, CBF rows '
            f'dropped. The publisher is re-stamping cached values.',
            throttle_duration_sec=0.5)
        return True

    def _smooth_qdot(self, qdot) -> None:
        """EMA on the q̇ used by the k1 anticipation term, and by nothing else.

        ḣ is a DERIVATIVE of a measured signal and k1 multiplies it straight
        into the bound: unfiltered it was 159 % of h_qp's whole swing on
        hardware and the command flipped sign nine times in 28 intervals. The
        barrier value, the row direction, the accel box, the braking fallback
        and every diagnostic keep the raw q̇.
        """
        a = self.P.cbf_hdot_filter_alpha
        if a > 0.0:
            self._qdot_cbf *= a
            self._qdot_cbf += (1.0 - a) * qdot
        else:
            np.copyto(self._qdot_cbf, qdot)

    def _solve(self, G, h_qp, n_c):
        """Push the moving parts into OSQP and solve. Returns (q̈, slack, ms, res).

        ``setup()`` only when n_c changes the sparsity pattern; otherwise
        ``update()`` the vectors that move. q̈ is ``None`` when the solve failed,
        which the caller turns into braking — and the warm-start iterate is
        discarded so a bad solve cannot seed the next one.
        """
        l, u = build_osqp_bounds(G, h_qp, self._box_lb, self._box_ub)
        if n_c != self._prev_nc or self._osqp_prob is None:
            self._prev_nc = n_c
            self._osqp_prob = osqp.OSQP()
            self._osqp_prob.setup(
                P=self._P_csc, q=self._qvec, A=build_osqp_A(G, NV, N_SLACK),
                l=l, u=u, warm_start=True,
                max_iter=self.P.osqp_max_iter, verbose=False)
        elif n_c > 0:
            self._osqp_prob.update(q=self._qvec, l=l, u=u,
                                   Ax=build_osqp_A(G, NV, N_SLACK).data)
        else:
            self._osqp_prob.update(q=self._qvec, l=l, u=u)

        t0 = time.perf_counter()
        res = self._osqp_prob.solve()
        solve_ms = (time.perf_counter() - t0) * 1e3

        self._diag_slack[:] = 0.0
        x = res.x
        if (res.info.status_val != osqp.constant('OSQP_SOLVED')
                or x is None or not np.all(np.isfinite(x))):
            self._qp_fail_count += 1
            self.get_logger().error(
                f'QP not solved ({res.info.status}) → braking output '
                f'[qp_fail_count={self._qp_fail_count}]',
                throttle_duration_sec=0.5)
            self._osqp_prob, self._prev_nc = None, -1
            return None, 0.0, solve_ms, res
        if n_c > 0:
            np.copyto(self._diag_slack, x[NV:])
        return x[:NV], float(self._diag_slack.max()), solve_ms, res

    def _report(self, now, con, h_qp, n_c, n_active, qdot, qddot_safe,
                qddot_nom, slack, solve_ms, res) -> None:
        """Solve-time line every 100 ticks, CBFDIAG line on its own throttle."""
        self._tick_count += 1
        if self._tick_count % 100 == 0 or solve_ms > 5.0:
            tail = (f'iter={res.info.iter} status={res.info.status} '
                    f'h_norm={float(np.linalg.norm(h_qp)):.2f}'
                    if n_c > 0 else 'iter=- status=- h_norm=-')
            self.get_logger().info(
                f'tick={self._tick_count} n_c={n_c} solve={solve_ms:.2f}ms '
                f'qddot_nom_norm={float(np.linalg.norm(qddot_nom)):.2f} '
                f'qp_fails={self._qp_fail_count} ' + tail)
        if n_c > 0 and (now - self._last_diag_t) >= self.P.diag_period_s:
            self._last_diag_t = now
            self.get_logger().info(format_cbf_diag(
                now=now, con=con, rows=self._rows, caps=self._diag_caps,
                h_qp=h_qp, qdot=qdot, qdot_cbf=self._qdot_cbf,
                qddot_safe=qddot_safe, qddot_nom=qddot_nom,
                qddot_real=self._diag_qddot_real, slack=self._diag_slack,
                n_active_cps=n_active, vel_ratio=self._diag_vel_ratio,
                vel_bite=self._diag_vel_bite, slew_bite=self._diag_slew_bite,
                cap_age=self._diag_cap_age))

    # ═════════════════════════════════════════════════════════════════════
    #  Output
    # ═════════════════════════════════════════════════════════════════════

    def _publish(self, qddot_safe) -> None:
        """Send q̈_safe, and remember it: the NEXT tick's slew box is centred on
        what actually went out, not on whatever the QP happened to compute."""
        self._qddot_prev[:] = qddot_safe
        msg = Float64MultiArray()
        msg.data = qddot_safe.tolist()
        self._pub.publish(msg)

    def _publish_status(self, n_c, slack, fault, n_act, d_obs=float('inf')) -> None:
        """/cbf_status = [n_rows, max slack, fault_braking, n_violated, d_min].

        ``fault_braking`` marks a SAFETY-CHAIN fault (state stale or frozen,
        perception stale, QP failed) — distinct from n_c = 0 during normal
        "nothing nearby" operation. ``d_obs`` is the closest obstacle gap, which
        the commander's phase governor reads: slowing on genuine proximity is
        safe, slowing on tracking error deadlocks (error grows → phase freezes →
        reference parked on the blocked pose → the CBF keeps blocking).
        """
        self._status_msg.data = [float(n_c), slack, fault, float(n_act), d_obs]
        self._status_pub.publish(self._status_msg)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = CBFSafetyFilter()
    # One thread per callback group: I/O, constraint builder, QP loop. The QP
    # thread must never wait on perception.
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
