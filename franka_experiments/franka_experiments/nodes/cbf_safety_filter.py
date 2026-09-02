#!/usr/bin/env python3
"""CBF Safety Filter — acceleration-level, real-time decoupled architecture.

Three rates, one process (MultiThreadedExecutor + per-group callback threads):

    I/O callbacks (event-driven)     perception group         control group
    ────────────────────────────     ─────────────────        ──────────────
    /joint_states  ─┐                constraint timer         QP timer
    /qddot_nom     ─┼► snapshots ──► ~50 Hz: Pinocchio  ────► ~200 Hz: OSQP
    /per_link_dist ─┘  (atomic       FK + point Jacobians     only — no
                        swap)        builds A, h̄, G           Pinocchio, no
                                     (atomic swap)            allocations →
                                                              /qddot_safe

QP solved per control tick:

    min  ½ ‖qddot − qddot_nom‖²  +  ½ ρ s²
    s.t. qddot_min ≤ qddot ≤ qddot_max
         aᵢᵀ qddot + s ≥ bᵢ   ∀ control point attivo   (s ≥ 0, slack)

One row per CONTROL POINT (perception publishes one LinkDistance per CP, up to
11), all active simultaneously — never argmin over them.  A single slack s is
shared by every row, so one deeply-violated CP relaxes all the others by the
same amount; watch data[1] of cbf_status when many rows are active.

Per active control point i (h̄ has relative degree 2 → HOCBF, d̈ depends on q̈):
    h̄ᵢ  = dᵢ − d_safe                 ┐ geometry — rebuilt at ~50 Hz
          dᵢ = LinkDistance.distance,   │ the SURFACE gap (capsule radius and
          NOT ‖pr − ph‖                 │ pixel-dilation margin already removed
                                        │ upstream by DistanceEngine)
    aᵢ  = n̂ᵢᵀ Jᵢ                      │ (perception group); the centripetal/
    ċᵢ  = n̂ᵢᵀ (J̇ᵢ q̇)                 ┘ Coriolis term ċᵢ is frozen at the
                                        snapshot q̇ (J̇ needs Pinocchio)
    bᵢ  = −k1·(aᵢᵀ q̇) − k0·h̄ᵢ − ċᵢ    aᵢᵀq̇ recomputed EVERY QP tick with the
                                        latest q̇; ċᵢ carried from the snapshot

Shared-state rule (lock-free): each producer publishes one *immutable*
NamedTuple by assigning a single attribute — reference assignment is atomic
under the GIL. Each consumer reads the attribute once into a local variable
and works on that consistent snapshot. No field is ever mutated in place.

Staleness policy (checked in the QP loop with carried timestamps):
    distances older than distance_timeout → CBF rows dropped (passthrough)
    qddot_nom older than nom_timeout      → braking fallback  −k_brake·q̇
    joint state older than js_timeout     → publish zeros, log error
    QP failure                            → braking fallback, reset warm start

Pubblica /NS_1/qddot_safe (Float64MultiArray, 7-dim); la conversione
qddot_safe → torque resta delegata a qddot_to_torque.py.
"""

import gc
import os
import threading
import time

from typing import NamedTuple

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

from franka_experiments.utils.cbf_hard_limits import velocity_accel_box
from franka_experiments.utils.cbf_qp_assembly import (
    build_osqp_A,
    build_osqp_bounds,
)
from franka_experiments.utils.config import load_package_yaml
from franka_experiments.utils.kinematics import CBFKinematics, build_urdf_no_hand
from franka_experiments.utils.logging_utils import format_velocity_summary
from franka_experiments.utils.params import (
    declare_bool,
    declare_float,
    declare_int,
    declare_str,
)

# ─────────────────────────────────────────────────────────────────────────────

FR3_JOINTS = [f'fr3_joint{i}' for i in range(1, 8)]
NV         = 7


# ── Immutable snapshots (atomic-swap shared state) ───────────────────────────

class _JointSnap(NamedTuple):
    q:     np.ndarray   # (NV,)
    qdot:  np.ndarray   # (NV,)
    stamp: float


class _NomSnap(NamedTuple):
    qddot: np.ndarray   # (NV,)
    stamp: float


class _Obstacle(NamedTuple):
    link: str
    d:    float
    pr:   np.ndarray    # closest point on robot, world frame
    ph:   np.ndarray    # closest point on human, world frame
    conf: float


class _ObstacleSnap(NamedTuple):
    items: tuple        # tuple[_Obstacle, ...]
    stamp: float


class _ConstraintSnap(NamedTuple):
    A:         np.ndarray  # (n_c, NV)     aᵢ = n̂ᵢᵀ Jᵢ rows
    h_bar:     np.ndarray  # (n_c,)        barrier values h̄ᵢ
    jdot_qdot: np.ndarray  # (n_c,)        ċᵢ = n̂ᵢᵀ(J̇ᵢ q̇) at the snapshot q̇
    G:         np.ndarray  # (n_c, NV+1)   prebuilt [−A | −1] for the QP
    t_dist:    float       # stamp of the distance data the geometry is based on
    links:     tuple       # (n_c,) link names, DIAGNOSTIC only — not used by QP


# ── Helpers ───────────────────────────────────────────────────────────────────

# ── Node ─────────────────────────────────────────────────────────────────────

class CBFSafetyFilter(Node):

    def __init__(self):
        super().__init__('cbf_safety_filter')

        cfg    = load_package_yaml('franka_experiments', 'config/fr3_control.yaml')
        topics = cfg['topics']
        p      = cfg['params']
        lims   = cfg['joint_limits']
        keys   = [f'joint{i}' for i in range(1, 8)]

        self._lb = np.array([-lims[k][3] for k in keys])   # −qddot_max per joint
        self._ub = np.array([ lims[k][3] for k in keys])   #  qddot_max per joint
        # Official Franka per-joint velocity limit (joint_limits.yaml limit.velocity,
        # = fr3_control.yaml col[2]). Until now UNUSED by the QP — the box bounded
        # only q̈, so a sustained q̈ could integrate q̇ past this limit and trip the
        # firmware `joint_velocity_violation` reflex. Now used to tighten the accel
        # box per tick (see utils.cbf_hard_limits.velocity_accel_box).
        self._qdot_max = np.array([lims[k][2] for k in keys])   # q̇_max per joint [rad/s]

        # Every knob below is declared as a real ROS parameter (visible in
        # `ros2 param list`, overridable from a launch file) with the value from
        # config/fr3_control.yaml as its default, and range-checked on startup:
        # a bad gain kills the node at construction instead of at 100 Hz.
        qp_rate          = declare_float(self, 'qp_rate_hz',
                                         p.get('qp_rate_hz', 200.0),
                                         positive=True, maximum=1000.0)
        cbf_rate         = declare_float(self, 'cbf_update_rate_hz',
                                         p.get('cbf_update_rate_hz', 50.0),
                                         positive=True, maximum=1000.0)
        self._d_safe     = declare_float(self, 'd_safe',
                                         p.get('d_safe', 0.20),
                                         minimum=0.0, maximum=2.0)
        # Pure computational horizon (NOT a safety activation gate): obstacles
        # beyond this are skipped only to cap/stabilize n_c. The linear HOCBF
        # self-deactivates at large d (k0·h̄ term), so activation is continuous.
        self._obstacle_horizon = declare_float(self, 'cbf_obstacle_horizon',
                                               p.get('cbf_obstacle_horizon', 1.2),
                                               positive=True, maximum=5.0)
        self._k0         = declare_float(self, 'k0_cbf',
                                         p.get('k0_cbf', 25.0),
                                         minimum=0.0, maximum=1.0e4)
        self._k1         = declare_float(self, 'k1_cbf',
                                         p.get('k1_cbf', 10.5),
                                         minimum=0.0, maximum=1.0e4)
        self._rho        = declare_float(self, 'rho_slack',
                                         p.get('rho_slack', 1000.0),
                                         positive=True)
        self._solver     = declare_str(self, 'qp_solver',
                                       p.get('qp_solver', 'osqp'),
                                       choices=('osqp',))
        self._dist_to    = declare_float(self, 'distance_timeout',
                                         p.get('distance_timeout', 0.5),
                                         positive=True, maximum=10.0)
        self._nom_to     = declare_float(self, 'nom_timeout',
                                         p.get('nom_timeout', 0.5),
                                         positive=True, maximum=10.0)
        self._js_to      = declare_float(self, 'joint_state_timeout',
                                         p.get('joint_state_timeout', 0.1),
                                         positive=True, maximum=10.0)
        self._k_brake    = declare_float(self, 'k_brake',
                                         p.get('k_brake', 3.0),
                                         minimum=0.0, maximum=100.0)
        self._conf_min   = declare_float(self, 'min_confidence',
                                         p.get('min_confidence', 0.2),
                                         minimum=0.0, maximum=1.0)
        # Min CBF leverage ‖a‖=‖n̂ᵀJp‖ [m/rad] for a constraint to be kept.
        # Replaces the old cond(Jp)>1e5 test (see _update_constraints).
        self._a_min      = declare_float(self, 'cbf_min_leverage',
                                         p.get('cbf_min_leverage', 0.05),
                                         minimum=0.0, maximum=10.0)

        self._kin = CBFKinematics(pin.buildModelFromUrdf(build_urdf_no_hand()))
        self._fid_cache: dict[str, int | None] = {}

        # ── Preallocated QP buffers (fixed-shape; G/h vary with n_c) ─────────
        # Slack penalty is QUADRATIC (½ρs²: ρ sits on the s² diagonal of P,
        # with NO linear slack term in q — _qvec[NV] stays 0). This DIVERGES
        # from OSCBF Eq.6 (Morton & Pavone), which uses a LINEAR slack penalty
        # ρᵀt. Deliberate choice: ½ρs² is C¹ at s=0 (no kink ⇒ smoother for the
        # QP solver) and prices small violations softly (marginal cost ρs→0 as
        # s→0) while punishing large ones harder. Consequence: ρ is NOT directly
        # comparable to an OSCBF linear ρ — the same ρ=1000 gives a
        # violation-dependent price (the two penalties cross only at s=2, ρ
        # cancelling). See CBF review notes for the full comparison; do not
        # "match OSCBF" by retuning ρ here.
        self._P = np.eye(NV + 1)
        self._P[-1, -1] = self._rho
        # Cost matrix is constant → convert to CSC once. Native OSQP requires
        # sparse matrices; reusing this instance avoids per-tick conversion.
        self._P_csc  = sparse.csc_matrix(self._P)
        self._qvec   = np.zeros(NV + 1)
        # Box bounds: indices 0..NV-1 are the per-joint q̈ bounds (STATIC accel
        # part from decel_limit, dynamically TIGHTENED by the velocity bound each
        # tick via velocity_accel_box); index NV is the slack slot s∈[0, 1e6]
        # (NEVER touched by the velocity update).
        self._box_lb = np.append(self._lb, 0.0)
        self._box_ub = np.append(self._ub, 1e6)
        # Velocity-aware box: one integration step (Δt = nominal QP period) must
        # not push |q̇| past v_margin·q̇_max. Nominal Δt (not measured) chosen on
        # purpose — see velocity_accel_box: error only shifts conservativeness,
        # absorbed by the 0.9 margin, and a fixed Δt keeps the bound deterministic
        # (independent of scheduling jitter).
        self._v_margin = declare_float(self, 'velocity_box_margin',
                                       p.get('velocity_box_margin', 0.9),
                                       positive=True, maximum=1.0)
        self._dt_qp    = 1.0 / qp_rate
        # Per-joint velocity diagnostics, refilled each tick by
        # velocity_accel_box; read by the CBFDIAG line and the high-res VELHI
        # log. ratio = |q̇|/q̇_max; bite = this joint's q̈ box was tightened by the
        # velocity bound (vs the static decel box).
        self._diag_vel_ratio = np.zeros(NV)
        self._diag_vel_bite  = np.zeros(NV, dtype=bool)
        # VELHI gate: emit the per-tick (10 ms) per-joint line only when the worst
        # joint exceeds this q̇/q̇_max ratio. Default 0.85 → silent in normal
        # operation, full resolution in the pre-violation window. >1.0 disables.
        self._diag_vel_ratio_thr = declare_float(
            self, 'diag_vel_ratio_thr',
            p.get('diag_vel_ratio_thr', 0.85), minimum=0.0)
        # Cumulative QP-failure counter (surfaced in the QP-fail error + the
        # periodic tick log) to correlate failures with the critical window.
        self._qp_fail_count = 0
        self._prev_nc = -1   # forces (re)setup the first time n_c is seen
        # Persistent native-OSQP problem. solve_qp() rebuilt the OSQP problem
        # (alloc + scaling + factorization) on every call — 5-24 ms even for the
        # empty n_c=0 problem. We instead keep one OSQP instance and .update()
        # the vectors (and A values when CBF is active) between ticks, paying
        # setup() only when the constraint count n_c changes the sparsity pattern.
        self._osqp_prob = None
        # OSQP's default max_iter (4000) can be hit on ill-scaled instances;
        # at this problem size 20k iterations still complete in < 1.5 ms.
        self._osqp_max_iter = declare_int(self, 'osqp_max_iter',
                                          p.get('osqp_max_iter', 20000),
                                          positive=True)

        # ── Shared snapshots (written/read by attribute assignment only) ─────
        self._js:  _JointSnap      | None = None
        self._nom: _NomSnap        | None = None
        self._obs: _ObstacleSnap   | None = None
        self._con: _ConstraintSnap | None = None

        # ── Structured CBF-episode diagnostic (manual time throttle) ─────────
        # Emitted only when n_c>0. Manual gate (not get_logger throttle) so the
        # projection math + f-string are computed ONLY when actually emitted,
        # keeping per-tick overhead ~0 between emissions. ~50 ms → good temporal
        # resolution for a few-second approach episode without log spam.
        self._diag_period = declare_float(self, 'diag_period_s',
                                          p.get('diag_period_s', 0.05),
                                          minimum=0.0)
        self._last_diag_t = 0.0

        # ── DIAGNOSTIC ONLY — realized joint acceleration estimate ───────────
        # q̈_real ≈ Δq̇/Δt from consecutive MEASURED q̇ (finite difference at the
        # native joint_states rate), lightly EMA-smoothed. This is a NOISY
        # numerical-derivative estimate kept SOLELY to compare commanded q̈_safe
        # against what the robot actually does (Phase-1 actuation debug). It is
        # NEVER read by the QP, the constraint builder, or ANY control/safety
        # decision — only by the CBFDIAG log block below. Do not wire it into
        # control. EMA α=0.7: measured-velocity differencing is noisy at the
        # high joint_states rate; 0.7 trims high-freq derivative noise while
        # still tracking the ~50 ms-scale trend a sub-second approach needs —
        # light enough not to mask a genuine commanded-vs-realized deficit.
        self._diag_qddot_real  = np.zeros(NV)               # latest est [rad/s²]
        self._diag_qdot_prev: np.ndarray | None = None
        self._diag_t_prev:    float | None = None
        self._diag_qddot_alpha = declare_float(self, 'diag_qddot_alpha',
                                               p.get('diag_qddot_alpha', 0.7),
                                               minimum=0.0, maximum=1.0)

        # ── Diagnostics: detect QP-tick scheduling gaps (see _qp_tick) ───────
        self._last_tick_t = None
        # Gap-warning threshold = 3x the nominal QP period; derived from qp_rate
        # so the "gap = 3x period" criterion stays correct if qp_rate_hz changes.
        # (qp_rate=100 Hz → 3 * 10 ms = 30 ms; qp_rate=200 Hz → 15 ms.)
        self._gap_warn_thr_ms = declare_float(
            self, 'tick_gap_warn_factor',
            p.get('tick_gap_warn_factor', 3.0), positive=True) * (1000.0 / qp_rate)
        # Set once, on the first _qp_tick, from the executor's QP thread (the
        # thread is only created after executor.spin(), so it cannot be done
        # here in __init__ which runs on the main thread). See _qp_tick.
        self._priority_set = False

        # ── Callback groups: QP loop must never wait on perception ──────────
        grp_io   = MutuallyExclusiveCallbackGroup()
        grp_perc = MutuallyExclusiveCallbackGroup()
        grp_ctrl = MutuallyExclusiveCallbackGroup()

        # depth=1: always consume the latest sample, never drain a backlog
        self.create_subscription(
            JointState, topics['joint_states_topic'], self._on_joint_state,
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
        # data = [n_active_constraints, slack, fault_braking, n_active_cps].
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
        self._status_msg.data = [0.0, 0.0, 0.0, 0.0]

        # Pay one-shot lazy costs now (robot stationary) instead of on the first
        # real tick — must run before the timers start firing callbacks.
        self._warmup()

        self.create_timer(1.0 / cbf_rate, self._update_constraints,
                          callback_group=grp_perc)
        self.create_timer(1.0 / qp_rate, self._qp_tick,
                          callback_group=grp_ctrl)

        self.get_logger().info(
            f'CBF filter  QP={qp_rate:.0f} Hz  constraints={cbf_rate:.0f} Hz  '
            f'solver={self._solver}\n'
            f'  d_safe={self._d_safe} m   obstacle_horizon={self._obstacle_horizon} m\n'
            f'  k0={self._k0}   k1={self._k1}   rho={self._rho}')

        # ── Diagnostic: optionally disable the garbage collector ─────────────
        # Used to test whether _qp_tick gaps coincide with GC pauses. This is a
        # temporary diagnostic mode only — NOT for normal operation, since it
        # can grow memory unbounded if cyclic garbage accumulates.
        if declare_bool(self, 'diag_disable_gc', False):
            gc.disable()
            self.get_logger().warn(
                'diag_disable_gc=TRUE → gc.disable() called. This is a TEMPORARY '
                'DIAGNOSTIC mode, not for normal use; memory may grow unbounded.')

    # ── Warm-up: pay one-shot lazy costs in __init__, not on the first tick ──

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
            self._kin.update(q0, qdot0, with_jdot=True)
        except Exception as exc:
            self.get_logger().warn(f'warmup: kin.update failed: {exc}')

        # (c) Populate _fid_cache for every link that can appear as
        #     robot_link_name in MultiLinkDistance (fr3_link0..link8), and
        # (b) warm the point-Jacobian path on the first link that resolves.
        first_fid = None
        for link in (f'fr3_link{i}' for i in range(9)):
            try:
                fid = self._frame_id(link)
                if fid is not None and first_fid is None:
                    first_fid = fid
            except Exception as exc:
                self.get_logger().warn(f"warmup: _frame_id('{link}') failed: {exc}")
        if first_fid is not None:
            try:
                self._kin.point_jacobian(first_fid, p_dummy)   # warms J and J̇ paths
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
            prob0.setup(P=self._P_csc, q=self._qvec, A=build_osqp_A(None, NV),
                        l=l0, u=u0, warm_start=True,
                        max_iter=self._osqp_max_iter, verbose=False)
            prob0.solve()

            G1 = np.zeros((1, NV + 1)); G1[0, -1] = -1.0   # dummy [−a | −1] row
            h1 = np.array([1.0])
            l1, u1 = build_osqp_bounds(G1, h1, self._box_lb, self._box_ub)
            prob1 = osqp.OSQP()
            prob1.setup(P=self._P_csc, q=self._qvec, A=build_osqp_A(G1, NV),
                        l=l1, u=u1, warm_start=True,
                        max_iter=self._osqp_max_iter, verbose=False)
            prob1.update(q=self._qvec, l=l1, u=u1, Ax=build_osqp_A(G1, NV).data)
            prob1.solve()
        except Exception as exc:
            self.get_logger().warn(f'warmup: OSQP path failed: {exc}')

        dt_ms = (time.perf_counter() - t0) * 1e3
        self.get_logger().info(
            f'warmup complete in {dt_ms:.1f} ms — one-shot lazy costs paid in '
            f'__init__ (robot stationary), not on the first control tick')

    # ── I/O callbacks: parse + atomic swap, nothing else ────────────────────

    def _on_joint_state(self, msg: JointState) -> None:
        n2p = dict(zip(msg.name, msg.position))
        n2v = dict(zip(msg.name, msg.velocity))
        try:
            q    = np.array([n2p[n] for n in FR3_JOINTS])
            qdot = np.array([n2v[n] for n in FR3_JOINTS])
        except KeyError:
            return
        self._js = _JointSnap(q, qdot, self._now())

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
                a   = self._diag_qddot_alpha
                self._diag_qddot_real = a * self._diag_qddot_real + (1.0 - a) * raw
        self._diag_qdot_prev = qdot
        self._diag_t_prev    = t_hdr

    def _on_qddot_nom(self, msg: Float64MultiArray) -> None:
        data = np.asarray(msg.data, dtype=np.float64)
        if data.shape == (NV,):
            self._nom = _NomSnap(data, self._now())

    def _on_distances(self, msg: MultiLinkDistance) -> None:
        items = tuple(
            _Obstacle(
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
        self._obs = _ObstacleSnap(items, self._now())

    # ── Perception-rate loop: geometry only (Pinocchio lives here) ──────────

    def _update_constraints(self) -> None:
        js, obs = self._js, self._obs
        if js is None or obs is None:
            return
        if self._now() - obs.stamp > self._dist_to:
            self._con = None
            return

        # with_jdot=True: also run computeJointJacobiansTimeVariation so that
        # point_jacobian() can return J̇p — needed for the J̇q̇ term of d̈
        # (h̄ has relative degree 2). One extra O(nv) backward pass at 50 Hz.
        self._kin.update(js.q, js.qdot, with_jdot=True)
        rows_a, rows_h, rows_jdq = [], [], []
        # Label per kept row (DIAGNOSTIC only, never read by the QP). Perception
        # now sends one entry per CONTROL POINT, so several rows share a
        # robot_link_name; the #k suffix counts occurrences in arrival order so
        # CBFDIAG can name which CP on a link is driving the constraint.
        rows_link     = []
        link_seen: dict[str, int] = {}
        n_weak        = 0           # obstacles dropped this tick for low leverage
        min_a_dropped = np.inf      # smallest ‖a‖ among the dropped ones (debug)

        for ob in obs.items:
            # obstacle_horizon is a COMPUTATIONAL cutoff, NOT a safety gate: the
            # linear HOCBF row self-deactivates at large d (its lower bound
            # −k0·h̄ becomes very negative), and the k1·ḣ̄ + jdq terms let a fast
            # approach engage the row gradually from afar — so activation is now
            # continuous. We skip only obstacles so far that engaging them would
            # need a physically unrealistic approach speed (at 1.2 m with the
            # current k0/k1 that is ≈ −2.4 m/s, beyond human motion), purely to
            # bound/stabilize n_c and avoid OSQP setup() churn. Replaces the old
            # d_safe+cbf_activation_margin step gate.
            if ob.d > self._obstacle_horizon or ob.conf < self._conf_min:
                continue

            # delta/‖delta‖ gives the unit normal n̂ (obstacle → control point).
            # ‖delta‖ itself is the CENTRE-to-obstacle distance and is NOT the
            # barrier argument — see the h assignment below.
            delta   = ob.pr - ob.ph
            d_ctr   = float(np.linalg.norm(delta))
            if d_ctr < 1e-8:
                continue
            n_w = delta / d_ctr

            fid = self._frame_id(ob.link)
            if fid is None:
                continue

            # point_jacobian returns (Jp, J̇p): Jp is the 3×7 position Jacobian
            # (same matrix as the old point_jacobian_pos), J̇p feeds jdq below.
            Jp, Jpd = self._kin.point_jacobian(fid, ob.pr)
            a      = (n_w @ Jp).astype(np.float64)      # (NV,)  aᵢ = n̂ᵀ Jp
            a_norm = float(np.linalg.norm(a))
            # Drop constraints with too little leverage on q̈ along THIS
            # obstacle's normal. Replaces the old cond(Jp)>1e5 test: cond(Jp)
            # measured the conditioning of the whole 3×7 map, not the leverage
            # of the specific row aᵢ the QP actually uses — a high cond only
            # weakens ‖aᵢ‖ when n̂ aligns with the ill-conditioned singular
            # direction, so it dropped direction-OK constraints and was an
            # indirect proxy for the weak case (see CBF review notes).
            if a_norm < self._a_min:
                n_weak       += 1
                min_a_dropped = min(min_a_dropped, a_norm)
                continue

            # Barrier argument is the SURFACE gap published by perception
            # (ob.d = LinkDistance.distance), not ‖pr − ph‖.  pr is the control
            # point on the segment AXIS, so ‖pr − ph‖ omits both the capsule
            # radius (0.05 m) and the pixel-dilation margin DistanceEngine
            # already subtracted (0.014 m at Z = 0.5 m, ~0.11 m at Z = 2 m).
            # Using it made the filter believe it was 6–16 cm farther from the
            # obstacle than perception reported — an optimistic bias in exactly
            # the unsafe direction, and a direct cause of contact while the CBF
            # still read a positive h̄.  ob.d is finite by construction: the
            # publisher sets valid=False otherwise and _on_distances drops it.
            #
            # a and jdq are UNCHANGED and remain exact: d_surface = d_ctr − r −
            # margin with r constant, so ḋ_surface = ḋ_ctr = n̂ᵀJp q̇.  Only the
            # zeroth-order term of the HOCBF moves.
            h = ob.d - self._d_safe                     # barrier value h̄
            # ċᵢ = n̂ᵀ(J̇p q̇): centripetal/Coriolis part of d̈ that does NOT
            # depend on q̈ (the relative-degree-2 term previously omitted).
            # Frozen at this snapshot's q̇ (js.qdot); the QP refreshes only aᵀq̇.
            jdq = float(n_w @ (Jpd @ js.qdot))          # scalar ċᵢ

            if np.all(np.isfinite(a)) and np.isfinite(h) and np.isfinite(jdq):
                rows_a.append(a)
                rows_h.append(h)
                rows_jdq.append(jdq)
                k = link_seen.get(ob.link, 0)
                link_seen[ob.link] = k + 1
                rows_link.append(f'{ob.link}#{k}')

        if n_weak > 0:
            # Previously this drop was silent (no log/counter). Throttled so a
            # persistently weak-leverage obstacle can't spam the log.
            self.get_logger().warn(
                f'{n_weak} obstacle(s) dropped: CBF leverage ‖a‖ < cbf_min_leverage='
                f'{self._a_min:.3g} (min ‖a‖={min_a_dropped:.3g})',
                throttle_duration_sec=2.0)

        if not rows_a:
            self._con = None
            return

        A     = np.vstack(rows_a)
        h_bar = np.array(rows_h,   dtype=np.float64)
        jdq_v = np.array(rows_jdq, dtype=np.float64)    # (n_c,) ċᵢ
        n_c   = A.shape[0]
        G     = np.empty((n_c, NV + 1))                # [−A | −1]: A q̈ + s ≥ b
        G[:, :NV] = -A
        G[:, -1]  = -1.0
        self._con = _ConstraintSnap(A, h_bar, jdq_v, G, obs.stamp, tuple(rows_link))

    def _frame_id(self, link: str) -> int | None:
        if link not in self._fid_cache:
            self._fid_cache[link] = self._kin.resolve_frame_id(link)
        return self._fid_cache[link]

    # ── Control-rate loop: read snapshots, solve QP, publish ────────────────

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

    def _qp_tick(self) -> None:
        # One-shot: elevate this (QP) thread's OS scheduling priority on the
        # first tick. Guarded so a permission failure isn't retried every tick.
        if not self._priority_set:
            self._priority_set = True
            self._elevate_thread_priority()

        # ── Diagnostic: measure wall-clock gap between consecutive ticks ─────
        now_tick = time.perf_counter()
        if self._last_tick_t is not None:
            dt_tick_ms = (now_tick - self._last_tick_t) * 1e3
            if dt_tick_ms > self._gap_warn_thr_ms:  # 3x nominal period (see __init__)
                self.get_logger().warn(f'_qp_tick gap: {dt_tick_ms:.1f} ms')
        self._last_tick_t = now_tick

        js = self._js
        if js is None:
            return
        now = self._now()

        if now - js.stamp > self._js_to:
            self.get_logger().error('joint state stale → zero output',
                                    throttle_duration_sec=0.5)
            self._publish(np.zeros(NV))
            return
        qdot = js.qdot

        nom = self._nom
        if nom is not None and now - nom.stamp < self._nom_to:
            qddot_nom = nom.qddot
        else:
            qddot_nom = -self._k_brake * qdot          # safe deceleration
            self.get_logger().warn('qddot_nom stale → braking',
                                   throttle_duration_sec=2.0)

        con  = self._con
        obs  = self._obs
        G    = h_qp = None
        n_c  = 0
        # Safety-chain fault flag (status data[2]); set by the distance-stale
        # and QP-failure paths below. Stays 0 for normal operation and for the
        # qddot_nom-stale fallback (that is a nominal-command loss, not a
        # safety-chain fault).
        fault_braking = 0.0
        # Control points whose barrier is actually violated (h̄ < 0 ⇒ inside
        # d_safe). Published as status data[3] and logged by CBFDIAG. Since
        # perception moved to one entry per control point this can exceed 1 —
        # that is the multi-CP activation working, not a fault. One vectorised
        # pass over an ≤11-element array per tick.
        n_active_cps = 0
        if con is not None and now - con.t_dist < self._dist_to:
            n_c  = con.A.shape[0]
            G    = con.G
            n_active_cps = int(np.count_nonzero(con.h_bar < 0.0))
            # HOCBF, relative degree 2 (per obstacle i), with linear class-K:
            #   d̈ + k1·ḋ + k0·h̄ ≥ 0 ,  ḋ = aᵀq̇ ,  d̈ = aᵀq̈ + n̂ᵀ(J̇q̇)
            #   ⇒  aᵀq̈ + n̂ᵀ(J̇q̇) ≥ −k1·(aᵀq̇) − k0·h̄              (with slack s≥0
            #   ⇒  aᵀq̈ + s ≥ −k1·(aᵀq̇) − k0·h̄ − n̂ᵀ(J̇q̇)          relaxes it)
            # QP rows: G = [−A | −1], assembled as G x ≤ u (u ≡ h_qp), so
            #   −Aq̈ − s ≤ h_qp  ⇔  aᵀq̈ + s ≥ −h_qp .  Matching the two:
            #   −h_qp = −k1·(aᵀq̇) − k0·h̄ − n̂ᵀ(J̇q̇)
            #   ⇒  h_qp =  k1·(aᵀq̇) + k0·h̄ + n̂ᵀ(J̇q̇)   (J̇q̇ term ADDED, sign +)
            # aᵀq̇ uses the fresh QP-tick q̇; n̂ᵀ(J̇q̇)=con.jdot_qdot is carried
            # from the 50 Hz snapshot (J̇ needs Pinocchio, absent in this loop).
            h_qp = (self._k1 * (con.A @ qdot)
                    + self._k0 * con.h_bar
                    + con.jdot_qdot)
        elif obs is not None and (now - obs.stamp) > self._dist_to:
            # Genuine perception staleness: distance data WAS received but the
            # latest sample is older than distance_timeout (camera frozen,
            # distance node crashed, link down). A failure in the channel that
            # feeds the safety barrier must degrade toward a MORE conservative
            # behaviour, not toward zero constraints (silent passthrough). Mirror
            # the stale-qddot_nom fallback above: replace the nominal with
            # braking, so the QP then runs with n_c=0 (no CBF rows) but on an
            # already-decelerating nominal. The expression is intentionally
            # duplicated from the qddot_nom-stale branch rather than factored,
            # to keep that branch byte-for-byte unchanged.
            # This branch is reached ONLY on real staleness; when obstacles are
            # simply out of range obs is fresh (now-obs.stamp ≤ dist_to) so we
            # fall through here and keep normal passthrough — no spurious braking.
            qddot_nom = -self._k_brake * qdot          # safe deceleration
            fault_braking = 1.0                         # safety-feed fault (status data[2])
            self.get_logger().warn('distance stale → braking fallback (CBF inactive)',
                                   throttle_duration_sec=2.0)

        self._qvec[:NV] = -qddot_nom

        # Velocity-aware box: tighten the per-joint q̈ bounds from the fresh q̇ so
        # the integrated velocity can't trip the firmware joint_velocity_violation
        # reflex. Mutates self._box_lb/_box_ub (accel rows only) in place → must
        # run BEFORE build_osqp_bounds reads them. Also refills _diag_vel_ratio/_bite.
        self._diag_vel_ratio, self._diag_vel_bite = velocity_accel_box(
            qdot, acc_lb=self._lb, acc_ub=self._ub, qdot_max=self._qdot_max,
            v_margin=self._v_margin, dt=self._dt_qp,
            out_lb=self._box_lb[:NV], out_ub=self._box_ub[:NV])

        # ── High-resolution velocity telemetry (VELHI) ───────────────────────
        # Per-TICK (10 ms, NO throttle) per-joint line, emitted ONLY when the
        # worst joint exceeds diag_vel_ratio_thr (0.85) — silent in normal
        # operation, full resolution exactly in the pre-violation window. Catches
        # transients faster than the 50 ms CBFDIAG throttle and is independent of
        # n_c, so it shows a velocity saturation even when no CBF row is active
        # (the suspected "biting joint ≠ CBF-projected joint" case). Disable with
        # diag_vel_ratio_thr > 1.0.
        if float(np.max(self._diag_vel_ratio)) > self._diag_vel_ratio_thr:
            self.get_logger().info(
                f'VELHI t={now:.3f} n_c={n_c} {format_velocity_summary(qdot, self._diag_vel_ratio, self._diag_vel_bite)}')

        # ── Native-OSQP solve ────────────────────────────────────────────────
        # Reuse one OSQP instance: setup() (alloc + scaling + symbolic
        # factorization) only when n_c changes the sparsity pattern; otherwise
        # update() the vectors that move each tick and reuse the factorization.
        #   n_c == 0 : A is the constant identity block, but the box bounds l/u
        #              now MOVE every tick (velocity box) → push q AND l/u.
        #   n_c  > 0 : the CBF normals in G move every tick (geometry recomputed
        #              at cbf_rate) and h_qp moves with q̇/h̄ → push Ax + u too.
        l, u = build_osqp_bounds(G, h_qp, self._box_lb, self._box_ub)
        if n_c != self._prev_nc or self._osqp_prob is None:
            self._prev_nc   = n_c
            self._osqp_prob = osqp.OSQP()
            self._osqp_prob.setup(
                P=self._P_csc, q=self._qvec, A=build_osqp_A(G, NV), l=l, u=u,
                warm_start=True, max_iter=self._osqp_max_iter, verbose=False)
        elif n_c > 0:
            self._osqp_prob.update(q=self._qvec, l=l, u=u,
                                   Ax=build_osqp_A(G, NV).data)
        else:
            # Box-only problem, but the box is now dynamic → push l/u, not just q.
            self._osqp_prob.update(q=self._qvec, l=l, u=u)

        # solve_ms measures .solve() only (not setup/update), so it stays
        # directly comparable to the pre-migration diagnostic.
        t0 = time.perf_counter()
        res = self._osqp_prob.solve()
        solve_ms = (time.perf_counter() - t0) * 1e3
        x = res.x

        # ── Diagnostic: log solve time on every tick (counter-throttled), so
        # anomalous samples are never dropped by a time-based throttle window.
        self._tick_count = getattr(self, '_tick_count', 0) + 1
        if self._tick_count % 100 == 0 or solve_ms > 5.0:
            qddot_nom_norm = float(np.linalg.norm(qddot_nom))
            self.get_logger().info(
                f'tick={self._tick_count} n_c={n_c} solve={solve_ms:.2f}ms '
                f'qddot_nom_norm={qddot_nom_norm:.2f} qp_fails={self._qp_fail_count} '
                + (f'iter={res.info.iter} status={res.info.status} h_norm={float(np.linalg.norm(h_qp)):.2f}'
                   if n_c > 0 else 'iter=- status=- h_norm=-')
            )

        slack = 0.0
        solved = res.info.status_val == osqp.constant('OSQP_SOLVED')
        if not solved or x is None or not np.all(np.isfinite(x)):
            self._qp_fail_count += 1
            self.get_logger().error(
                f'QP not solved ({res.info.status}) → braking output '
                f'[qp_fail_count={self._qp_fail_count}]',
                throttle_duration_sec=0.5)
            # Discard the (possibly poisoned) internal warm-start iterate: force
            # a clean setup() next tick so a bad solve can't seed the next one.
            self._osqp_prob = None
            self._prev_nc   = -1
            qddot_safe = np.clip(-self._k_brake * qdot, self._lb, self._ub)
            fault_braking = 1.0                         # safety-QP fault (status data[2])
        else:
            qddot_safe = x[:NV]
            if n_c > 0:
                slack = float(x[-1])

        self._publish(qddot_safe)
        self._status_msg.data[0] = float(n_c)
        self._status_msg.data[1] = slack
        self._status_msg.data[2] = fault_braking
        self._status_msg.data[3] = float(n_active_cps)
        self._status_pub.publish(self._status_msg)

        # ── Structured CBF-episode diagnostic ────────────────────────────────
        # One compact, CSV-like line per ~50 ms while any CBF row is active.
        # Whole block (projections + f-string) runs only when the manual throttle
        # is due → negligible average per-tick cost. DIAGNOSTIC ONLY: reads
        # already-computed values, changes nothing in the control/safety path.
        # Field guide (units): n_c rows in the QP (every CP inside the horizon);
        #   n_act rows actually VIOLATED (h̄ < 0, CP inside d_safe) — n_act > 1
        #   is simultaneous multi-CP activation, expected on an angled approach;
        #   d_min [m] closest active obstacle (= min h̄ + d_safe), now the SURFACE
        #   gap, so it should agree with real_time_distance's `dist=` line;
        #   link closest link; hdot=aᵀq̇ [m/s] approach rate (velocity anticipation);
        #   h_qp [m/s²] RHS of that row's bound (more positive ⇒ looser/inactive);
        #   dnorm=‖q̈_safe−q̈_nom‖ [rad/s²] how hard the QP bends the nominal;
        #   s slack (>0 ⇒ QP relaxing the constraint, local conflict/infeasibility);
        #   dq_rad/dq_ort split of (q̈_safe−q̈_nom) along/⊥ the constrained joint
        #   direction â (large dq_ort ⇒ motion leaking into UNconstrained joints —
        #   the suspected "throws itself backward"); cart_rad=aᵀΔq̈ [m/s²] Cartesian
        #   accel change along n̂ at the control point.
        #   Phase-1 actuation debug (commanded vs realized, DIAGNOSTIC q̈_real):
        #   qdd_cmd_rad=aᵀq̈_safe [m/s²] COMMANDED Cartesian accel along n̂;
        #   qdd_real_rad=aᵀq̈_real [m/s²] REALIZED (from measured q̇ finite diff);
        #   trk_err=‖q̈_safe−q̈_real‖ [rad/s²] joint-space tracking error.
        #   qdd_real_rad ≪ qdd_cmd_rad while pushing away ⇒ command not executed.
        if n_c > 0 and (now - self._last_diag_t) >= self._diag_period:
            self._last_diag_t = now
            dq    = qddot_safe - qddot_nom
            # Closest row, for LABELLING this log line only — every row is in
            # the QP regardless. Not an activation gate; do not read it as one.
            i     = int(np.argmin(con.h_bar))
            a_i   = con.A[i]
            a_n   = float(np.linalg.norm(a_i))
            a_hat = a_i / a_n if a_n > 1e-12 else a_i
            dq_rad = float(a_hat @ dq)
            dq_ort = float(np.linalg.norm(dq - dq_rad * a_hat))
            link_i = con.links[i] if i < len(con.links) else '?'
            # Phase-1: commanded vs realized accel along â (diagnostic q̈_real).
            qddot_real   = self._diag_qddot_real
            qdd_cmd_rad  = float(a_i @ qddot_safe)
            qdd_real_rad = float(a_i @ qddot_real)
            trk_err      = float(np.linalg.norm(qddot_safe - qddot_real))
            self.get_logger().info(
                f'CBFDIAG t={now:.3f} n_c={n_c} n_act={n_active_cps} '
                f'd_min={float(con.h_bar[i]) + self._d_safe:.3f} '
                f'link={link_i} hdot={float(a_i @ qdot):+.3f} h_qp={float(h_qp[i]):+.3f} '
                f'dnorm={float(np.linalg.norm(dq)):.3f} s={slack:.4f} '
                f'dq_rad={dq_rad:+.3f} dq_ort={dq_ort:.3f} cart_rad={float(a_i @ dq):+.3f} '
                f'qdd_cmd_rad={qdd_cmd_rad:+.3f} qdd_real_rad={qdd_real_rad:+.3f} trk_err={trk_err:.3f} '
                f'| {format_velocity_summary(qdot, self._diag_vel_ratio, self._diag_vel_bite)}')

    def _publish(self, qddot_safe: np.ndarray) -> None:
        msg      = Float64MultiArray()
        msg.data = qddot_safe.tolist()
        self._pub.publish(msg)

    # ── Utility ──────────────────────────────────────────────────────────────

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CBFSafetyFilter()
    # one thread per callback group: I/O, constraint builder, QP loop
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
