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
         aᵢᵀ qddot + s ≥ bᵢ   ∀ ostacolo attivo   (s ≥ 0, slack)

Per active obstacle i:
    h̄ᵢ = dᵢ − d_safe                  ┐ geometry — rebuilt at ~50 Hz
    aᵢ  = nᵢᵀ Jᵢ                       ┘ (perception group)
    bᵢ  = −k1·(aᵢᵀ q̇) − k0·h̄ᵢ          recomputed EVERY QP tick with the
                                        latest q̇ (one (n_c×7)@(7,) matvec)

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
import glob
import os
import subprocess
import tempfile
import threading
import time

from typing import NamedTuple

import numpy as np
import osqp
import pinocchio as pin
import rclpy
import scipy.sparse as sparse
import yaml
from ament_index_python.packages import get_package_share_directory
from franka_msgs.msg import MultiLinkDistance
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from franka_experiments.utils.cbf_kinematics import CBFKinematics

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
    A:      np.ndarray  # (n_c, NV)     aᵢ rows
    h_bar:  np.ndarray  # (n_c,)        barrier values h̄ᵢ
    G:      np.ndarray  # (n_c, NV+1)   prebuilt [−A | −1] for the QP
    t_dist: float       # stamp of the distance data the geometry is based on


# ── Helpers ───────────────────────────────────────────────────────────────────

def _load_yaml(pkg: str, rel: str) -> dict:
    with open(os.path.join(get_package_share_directory(pkg), rel)) as f:
        return yaml.safe_load(f)


def _build_urdf_no_hand() -> str:
    share = get_package_share_directory('franka_description')
    xs = glob.glob(os.path.join(share, '**', 'fr3.urdf.xacro'), recursive=True)
    if not xs:
        raise RuntimeError('fr3.urdf.xacro not found under franka_description share')
    tmp = tempfile.NamedTemporaryFile(suffix='.urdf', delete=False, prefix='fr3_cbf_')
    tmp.close()
    r = subprocess.run(['xacro', xs[0], 'hand:=false', '-o', tmp.name],
                       capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(f'xacro failed:\n{r.stderr}')
    return tmp.name


# ── Node ─────────────────────────────────────────────────────────────────────

class CBFSafetyFilter(Node):

    def __init__(self):
        super().__init__('cbf_safety_filter')

        cfg    = _load_yaml('franka_experiments', 'config/fr3_control.yaml')
        topics = cfg['topics']
        p      = cfg['params']
        lims   = cfg['joint_limits']
        keys   = [f'joint{i}' for i in range(1, 8)]

        self._lb = np.array([-lims[k][3] for k in keys])   # −qddot_max per joint
        self._ub = np.array([ lims[k][3] for k in keys])   #  qddot_max per joint

        qp_rate          = float(p.get('qp_rate_hz',           200.0))
        cbf_rate         = float(p.get('cbf_update_rate_hz',    50.0))
        self._d_safe     = float(p.get('d_safe',                 0.20))
        self._margin     = float(p.get('cbf_activation_margin',  0.10))
        self._k0         = float(p.get('k0_cbf',                25.0))
        self._k1         = float(p.get('k1_cbf',                10.0))
        self._rho        = float(p.get('rho_slack',           1000.0))
        self._solver     = str(  p.get('qp_solver',            'osqp'))
        self._dist_to    = float(p.get('distance_timeout',       0.5))
        self._nom_to     = float(p.get('nom_timeout',            0.5))
        self._js_to      = float(p.get('joint_state_timeout',    0.1))
        self._k_brake    = float(p.get('k_brake',                3.0))
        self._conf_min   = float(p.get('min_confidence',         0.2))

        self._kin = CBFKinematics(pin.buildModelFromUrdf(_build_urdf_no_hand()))
        self._fid_cache: dict[str, int | None] = {}

        # ── Preallocated QP buffers (fixed-shape; G/h vary with n_c) ─────────
        self._P = np.eye(NV + 1)
        self._P[-1, -1] = self._rho
        # Cost matrix is constant → convert to CSC once. Native OSQP requires
        # sparse matrices; reusing this instance avoids per-tick conversion.
        self._P_csc  = sparse.csc_matrix(self._P)
        self._qvec   = np.zeros(NV + 1)
        self._box_lb = np.append(self._lb, 0.0)
        self._box_ub = np.append(self._ub, 1e6)
        self._prev_nc = -1   # forces (re)setup the first time n_c is seen
        # Persistent native-OSQP problem. solve_qp() rebuilt the OSQP problem
        # (alloc + scaling + factorization) on every call — 5-24 ms even for the
        # empty n_c=0 problem. We instead keep one OSQP instance and .update()
        # the vectors (and A values when CBF is active) between ticks, paying
        # setup() only when the constraint count n_c changes the sparsity pattern.
        self._osqp_prob = None
        # OSQP's default max_iter (4000) can be hit on ill-scaled instances;
        # at this problem size 20k iterations still complete in < 1.5 ms.
        self._osqp_max_iter = 20000

        # ── Shared snapshots (written/read by attribute assignment only) ─────
        self._js:  _JointSnap      | None = None
        self._nom: _NomSnap        | None = None
        self._obs: _ObstacleSnap   | None = None
        self._con: _ConstraintSnap | None = None

        # ── Diagnostics: detect QP-tick scheduling gaps (see _qp_tick) ───────
        self._last_tick_t = None
        # Gap-warning threshold = 3x the nominal QP period; derived from qp_rate
        # so the "gap = 3x period" criterion stays correct if qp_rate_hz changes.
        # (qp_rate=100 Hz → 3 * 10 ms = 30 ms; qp_rate=200 Hz → 15 ms.)
        self._gap_warn_thr_ms = 3.0 * (1000.0 / qp_rate)
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

        # CBF activity status for downstream consumers (e.g. the motion
        # generator freezes its virtual time while CBF is active).
        # data = [n_active_constraints, slack].
        self._status_pub = self.create_publisher(
            Float64MultiArray, topics.get('cbf_status', '/NS_1/cbf_status'), 10)
        self._status_msg = Float64MultiArray()
        self._status_msg.data = [0.0, 0.0]

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
            f'  d_safe={self._d_safe} m   margin={self._margin} m\n'
            f'  k0={self._k0}   k1={self._k1}   rho={self._rho}')

        # ── Diagnostic: optionally disable the garbage collector ─────────────
        # Used to test whether _qp_tick gaps coincide with GC pauses. This is a
        # temporary diagnostic mode only — NOT for normal operation, since it
        # can grow memory unbounded if cyclic garbage accumulates.
        self.declare_parameter('diag_disable_gc', False)
        if self.get_parameter('diag_disable_gc').value:
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

        # (a) Pinocchio FK + Jacobian internal structures.
        try:
            self._kin.update(q0, qdot0, with_jdot=False)
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
                self._kin.point_jacobian_pos(first_fid, p_dummy)
            except Exception as exc:
                self.get_logger().warn(f'warmup: point_jacobian_pos failed: {exc}')
        else:
            self.get_logger().warn('warmup: no robot link resolved — Jacobian path not warmed')

        # (d) Exercise the native-OSQP paths used in _qp_tick for both n_c=0 and
        #     n_c=1 (setup + update(Ax) + solve), on throwaway problems. Also pays
        #     osqp's lazy library-init cost here rather than on the first tick.
        try:
            l0, u0 = self._osqp_lu(None, None)
            prob0 = osqp.OSQP()
            prob0.setup(P=self._P_csc, q=self._qvec, A=self._osqp_A(None),
                        l=l0, u=u0, warm_start=True,
                        max_iter=self._osqp_max_iter, verbose=False)
            prob0.solve()

            G1 = np.zeros((1, NV + 1)); G1[0, -1] = -1.0   # dummy [−a | −1] row
            h1 = np.array([1.0])
            l1, u1 = self._osqp_lu(G1, h1)
            prob1 = osqp.OSQP()
            prob1.setup(P=self._P_csc, q=self._qvec, A=self._osqp_A(G1),
                        l=l1, u=u1, warm_start=True,
                        max_iter=self._osqp_max_iter, verbose=False)
            prob1.update(q=self._qvec, l=l1, u=u1, Ax=self._osqp_A(G1).data)
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

        d_threshold = self._d_safe + self._margin
        self._kin.update(js.q, js.qdot, with_jdot=False)
        rows_a, rows_h = [], []

        for ob in obs.items:
            if ob.d > d_threshold or ob.conf < self._conf_min:
                continue

            delta = ob.pr - ob.ph
            d     = float(np.linalg.norm(delta))
            if d < 1e-8:
                continue
            n_w = delta / d

            fid = self._frame_id(ob.link)
            if fid is None:
                continue

            Jp = self._kin.point_jacobian_pos(fid, ob.pr)
            if float(np.linalg.cond(Jp)) > 1e5:
                continue

            a = (n_w @ Jp).astype(np.float64)          # (NV,)
            h = d - self._d_safe                        # barrier value

            if np.all(np.isfinite(a)) and np.isfinite(h):
                rows_a.append(a)
                rows_h.append(h)

        if not rows_a:
            self._con = None
            return

        A     = np.vstack(rows_a)
        h_bar = np.array(rows_h, dtype=np.float64)
        n_c   = A.shape[0]
        G     = np.empty((n_c, NV + 1))                # [−A | −1]: A q̈ + s ≥ b
        G[:, :NV] = -A
        G[:, -1]  = -1.0
        self._con = _ConstraintSnap(A, h_bar, G, obs.stamp)

    def _frame_id(self, link: str) -> int | None:
        if link not in self._fid_cache:
            self._fid_cache[link] = self._kin.resolve_frame_id(link)
        return self._fid_cache[link]

    # ── Control-rate loop: read snapshots, solve QP, publish ────────────────

    @staticmethod
    def _osqp_A(G: np.ndarray | None) -> sparse.csc_matrix:
        """Build the native-OSQP constraint matrix  A = [ G ; I ].

        Rows 0..n_c-1 are the CBF constraints ([−A | −1]); the trailing NV+1
        rows are the identity block that carries the box bounds (joint qddot
        limits + slack ≥ 0). The CBF block is stored with a FULL (dense)
        sparsity pattern — every entry is an explicit structural nonzero,
        including zeros — so the pattern is invariant for a given n_c. That lets
        prob.update(Ax=...) stay valid across ticks even when a Jacobian entry
        passes through zero (a plain csc_matrix(G) would drop those zeros and
        change the pattern). l <= A x <= u is set by _osqp_lu().
        """
        box = sparse.identity(NV + 1, format='csc')
        if G is None:                       # n_c == 0 → box bounds only
            return box
        n_c  = G.shape[0]
        rows = np.repeat(np.arange(n_c), NV + 1)
        cols = np.tile(np.arange(NV + 1), n_c)
        cbf  = sparse.csc_matrix((G.ravel(), (rows, cols)), shape=(n_c, NV + 1))
        return sparse.vstack([cbf, box], format='csc')

    def _osqp_lu(self, G: np.ndarray | None, h_qp: np.ndarray | None):
        """Bounds for A x: CBF rows  −inf ≤ Gx ≤ h_qp,  box rows  lb ≤ x ≤ ub.

        Same row order as _osqp_A (CBF rows first, then the identity block).
        """
        if G is None:
            return self._box_lb, self._box_ub
        n_c = G.shape[0]
        l = np.concatenate([np.full(n_c, -np.inf), self._box_lb])
        u = np.concatenate([h_qp,                  self._box_ub])
        return l, u

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
        if con is not None and now - con.t_dist < self._dist_to:
            n_c  = con.A.shape[0]
            G    = con.G
            # b = −k1·(A q̇) − k0·h̄  with fresh q̇;  G x ≤ −b
            h_qp = self._k1 * (con.A @ qdot) + self._k0 * con.h_bar
        elif obs is not None and (now - obs.stamp) > self._dist_to:
            # warn only for genuine staleness; silence when obstacles are simply out of range
            self.get_logger().warn('distance stale — CBF inactive',
                                   throttle_duration_sec=2.0)

        self._qvec[:NV] = -qddot_nom

        # ── Native-OSQP solve ────────────────────────────────────────────────
        # Reuse one OSQP instance: setup() (alloc + scaling + symbolic
        # factorization) only when n_c changes the sparsity pattern; otherwise
        # update() the vectors that move each tick and reuse the factorization.
        #   n_c == 0 : A is the constant identity block → only q changes.
        #   n_c  > 0 : the CBF normals in G move every tick (geometry recomputed
        #              at cbf_rate) and h_qp moves with q̇/h̄ → push Ax + u too.
        l, u = self._osqp_lu(G, h_qp)
        if n_c != self._prev_nc or self._osqp_prob is None:
            self._prev_nc   = n_c
            self._osqp_prob = osqp.OSQP()
            self._osqp_prob.setup(
                P=self._P_csc, q=self._qvec, A=self._osqp_A(G), l=l, u=u,
                warm_start=True, max_iter=self._osqp_max_iter, verbose=False)
        elif n_c > 0:
            self._osqp_prob.update(q=self._qvec, l=l, u=u,
                                   Ax=self._osqp_A(G).data)
        else:
            self._osqp_prob.update(q=self._qvec)

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
                f'qddot_nom_norm={qddot_nom_norm:.2f} '
                + (f'iter={res.info.iter} status={res.info.status} h_norm={float(np.linalg.norm(h_qp)):.2f}'
                   if n_c > 0 else 'iter=- status=- h_norm=-')
            )

        slack = 0.0
        solved = res.info.status_val == osqp.constant('OSQP_SOLVED')
        if not solved or x is None or not np.all(np.isfinite(x)):
            self.get_logger().error(
                f'QP not solved ({res.info.status}) → braking output',
                throttle_duration_sec=0.5)
            # Discard the (possibly poisoned) internal warm-start iterate: force
            # a clean setup() next tick so a bad solve can't seed the next one.
            self._osqp_prob = None
            self._prev_nc   = -1
            qddot_safe = np.clip(-self._k_brake * qdot, self._lb, self._ub)
        else:
            qddot_safe = x[:NV]
            if n_c > 0:
                slack = float(x[-1])

        self._publish(qddot_safe)
        self._status_msg.data[0] = float(n_c)
        self._status_msg.data[1] = slack
        self._status_pub.publish(self._status_msg)

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
