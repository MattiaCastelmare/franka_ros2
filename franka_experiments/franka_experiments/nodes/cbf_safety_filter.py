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

import glob
import os
import subprocess
import tempfile
import time

from typing import NamedTuple

import numpy as np
import pinocchio as pin
import qpsolvers
import rclpy
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
        self._qvec   = np.zeros(NV + 1)
        self._box_lb = np.append(self._lb, 0.0)
        self._box_ub = np.append(self._ub, 1e6)
        self._warm: np.ndarray | None = None
        self._prev_nc = -1   # warm start invalid when constraint count changes
        # OSQP's default max_iter (4000) can be hit on ill-scaled instances;
        # at this problem size 20k iterations still complete in < 1.5 ms.
        self._solver_kwargs = {'max_iter': 20000} if self._solver == 'osqp' else {}

        # ── Shared snapshots (written/read by attribute assignment only) ─────
        self._js:  _JointSnap      | None = None
        self._nom: _NomSnap        | None = None
        self._obs: _ObstacleSnap   | None = None
        self._con: _ConstraintSnap | None = None

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

        self.create_timer(1.0 / cbf_rate, self._update_constraints,
                          callback_group=grp_perc)
        self.create_timer(1.0 / qp_rate, self._qp_tick,
                          callback_group=grp_ctrl)

        self.get_logger().info(
            f'CBF filter  QP={qp_rate:.0f} Hz  constraints={cbf_rate:.0f} Hz  '
            f'solver={self._solver}\n'
            f'  d_safe={self._d_safe} m   margin={self._margin} m\n'
            f'  k0={self._k0}   k1={self._k1}   rho={self._rho}')

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

    def _qp_tick(self) -> None:
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

        if n_c != self._prev_nc:
            self._warm   = None
            self._prev_nc = n_c

        self._qvec[:NV] = -qddot_nom
        t0 = time.perf_counter()
        x = qpsolvers.solve_qp(
            P=self._P, q=self._qvec,
            G=G, h=h_qp,
            lb=self._box_lb, ub=self._box_ub,
            solver=self._solver,
            initvals=self._warm,
            verbose=False,
            **self._solver_kwargs,
        )
        solve_ms = (time.perf_counter() - t0) * 1e3

        if x is None or not np.all(np.isfinite(x)):
            self.get_logger().error('QP failed → braking output',
                                    throttle_duration_sec=0.5)
            self._warm = None
            qddot_safe = np.clip(-self._k_brake * qdot, self._lb, self._ub)
        else:
            self._warm = x
            qddot_safe = x[:NV]
            if n_c > 0:
                self.get_logger().info(
                    f'CBF ON  n_c={n_c}  slack={float(x[-1]):.2e}  '
                    f'solve={solve_ms:.2f} ms',
                    throttle_duration_sec=0.5)

        self._publish(qddot_safe)

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
