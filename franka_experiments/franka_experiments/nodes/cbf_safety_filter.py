#!/usr/bin/env python3
"""CBF Safety Filter — acceleration-level baseline (minimal).

QP per ogni tick:

    min  ½ ‖qddot − qddot_nom‖²  +  ½ ρ s²
    s.t. qddot_min ≤ qddot ≤ qddot_max
         aᵢᵀ qddot + s ≥ bᵢ   ∀ ostacolo attivo   (s ≥ 0, slack)

dove:
    hᵢ  = dᵢ − d_safe
    aᵢ  = nᵢᵀ Jᵢ        (normale mondo × Jacobiano punto)
    bᵢ  = −k1·(aᵢᵀ q̇) − k0·hᵢ

Pubblica:
    /NS_1/qddot_safe  (Float64MultiArray, 7-dim)

La conversione qddot_safe → torque è delegata a qddot_to_torque.py.
"""

import glob
import os
import subprocess
import tempfile
import yaml

import numpy as np
import pinocchio as pin
import qpsolvers
import rclpy
from ament_index_python.packages import get_package_share_directory
from franka_msgs.msg import MultiLinkDistance
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from franka_experiments.utils.cbf_kinematics import CBFKinematics

# ─────────────────────────────────────────────────────────────────────────────

FR3_JOINTS = [f'fr3_joint{i}' for i in range(1, 8)]
NV         = 7


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


# ── QP ────────────────────────────────────────────────────────────────────────

def _solve(
    qddot_nom : np.ndarray,          # (NV,)
    lb        : np.ndarray,          # (NV,) box lower
    ub        : np.ndarray,          # (NV,) box upper
    A_cbf     : np.ndarray | None,   # (n_c, NV) or None
    b_cbf     : np.ndarray | None,   # (n_c,)    or None
    rho       : float,
    solver    : str,
    warm      : np.ndarray | None,   # (NV+1,) primal warm-start or None
) -> tuple[np.ndarray | None, np.ndarray | None]:
    """Return (qddot_safe, x_full) or (None, None) on failure."""
    n = NV + 1   # qddot + slack

    P          = np.eye(n); P[-1, -1] = rho
    q_vec      = np.zeros(n); q_vec[:NV] = -qddot_nom
    box_lb     = np.append(lb, 0.0)
    box_ub     = np.append(ub, 1e6)

    G, h = None, None
    if A_cbf is not None and A_cbf.shape[0] > 0:
        n_c     = A_cbf.shape[0]
        G       = np.zeros((n_c, n))
        G[:, :NV] = -A_cbf
        G[:, -1]  = -1.0          # slack column
        h       = -b_cbf.astype(np.float64)

    x = qpsolvers.solve_qp(
        P=P, q=q_vec,
        G=G, h=h,
        lb=box_lb, ub=box_ub,
        solver=solver,
        initvals=warm,
        verbose=False,
    )

    if x is None or not np.all(np.isfinite(x)):
        return None, None
    return x[:NV].copy(), x.copy()


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

        self._dt         = 1.0 / float(p['control_rate_hz'])
        self._d_safe     = float(p.get('d_safe',                 0.20))
        self._margin     = float(p.get('cbf_activation_margin',  0.10))
        self._k0         = float(p.get('k0_cbf',                25.0))
        self._k1         = float(p.get('k1_cbf',                10.0))
        self._rho        = float(p.get('rho_slack',           1000.0))
        self._solver     = str(  p.get('qp_solver',            'osqp'))
        self._dist_to    = float(p.get('distance_timeout',       0.5))
        self._nom_to     = 0.5   # [s] stale threshold for qddot_nom

        self._kin  = CBFKinematics(pin.buildModelFromUrdf(_build_urdf_no_hand()))
        self._warm : np.ndarray | None = None

        # ── State ────────────────────────────────────────────────────────────
        self._q         : np.ndarray | None = None
        self._qdot      : np.ndarray | None = None
        self._t_js      : float | None      = None
        self._qddot_nom : np.ndarray        = np.zeros(NV)
        self._t_nom     : float | None      = None
        self._dists     : list              = []
        self._t_dist    : float | None      = None

        # ── ROS I/O ──────────────────────────────────────────────────────────
        self.create_subscription(
            JointState, topics['joint_states_topic'], self._on_joint_state, 10)
        self.create_subscription(
            Float64MultiArray, topics['qddot_nom'], self._on_qddot_nom, 10)
        self.create_subscription(
            MultiLinkDistance, '/cbf/per_link_distances', self._on_distances,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        self._pub = self.create_publisher(
            Float64MultiArray, topics['qddot_safe'], 10)
        self.create_timer(self._dt, self._tick)

        self.get_logger().info(
            f'CBF baseline  {1/self._dt:.0f} Hz  solver={self._solver}\n'
            f'  d_safe={self._d_safe} m   margin={self._margin} m\n'
            f'  k0={self._k0}   k1={self._k1}   rho={self._rho}')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_joint_state(self, msg: JointState) -> None:
        n2p = dict(zip(msg.name, msg.position))
        n2v = dict(zip(msg.name, msg.velocity))
        try:
            self._q    = np.array([n2p[n] for n in FR3_JOINTS])
            self._qdot = np.array([n2v[n] for n in FR3_JOINTS])
            self._t_js = self._now()
        except KeyError:
            pass

    def _on_qddot_nom(self, msg: Float64MultiArray) -> None:
        data = np.asarray(msg.data, dtype=np.float64)
        if data.shape == (NV,):
            self._qddot_nom = data
            self._t_nom     = self._now()

    def _on_distances(self, msg: MultiLinkDistance) -> None:
        self._dists = [
            {
                'link': ld.robot_link_name,
                'd':    float(ld.distance),
                'pr':   np.array([ld.closest_point_robot.x,
                                  ld.closest_point_robot.y,
                                  ld.closest_point_robot.z]),
                'ph':   np.array([ld.closest_point_human.x,
                                  ld.closest_point_human.y,
                                  ld.closest_point_human.z]),
                'conf': float(ld.confidence),
            }
            for ld in msg.links if ld.valid
        ]
        self._t_dist = self._now()

    # ── Control loop ─────────────────────────────────────────────────────────

    def _tick(self) -> None:
        if self._q is None:
            return

        now  = self._now()
        q    = self._q.copy()
        qdot = self._qdot.copy()

        nom_ok  = self._t_nom  is not None and (now - self._t_nom)  < self._nom_to
        dist_ok = self._t_dist is not None and (now - self._t_dist) < self._dist_to

        qddot_nom      = self._qddot_nom.copy() if nom_ok else np.zeros(NV)
        A_cbf, b_cbf   = self._build_cbf(q, qdot) if dist_ok else (None, None)

        qddot_safe, x_full = _solve(
            qddot_nom, self._lb, self._ub,
            A_cbf, b_cbf, self._rho, self._solver, self._warm)

        if qddot_safe is None:
            self.get_logger().error('QP failed → zero output',
                                    throttle_duration_sec=0.5)
            qddot_safe = np.zeros(NV)
            self._warm = None
        else:
            self._warm = x_full

        if not dist_ok:
            self.get_logger().warn('distance stale — CBF inactive',
                                   throttle_duration_sec=2.0)

        if A_cbf is not None and A_cbf.shape[0] > 0:
            slack = float(x_full[-1]) if x_full is not None else float('nan')
            self.get_logger().info(
                f'CBF ON  n_c={A_cbf.shape[0]}  slack={slack:.2e}',
                throttle_duration_sec=0.5)

        msg      = Float64MultiArray()
        msg.data = qddot_safe.tolist()
        self._pub.publish(msg)

    def _build_cbf(
        self, q: np.ndarray, qdot: np.ndarray
    ) -> tuple[np.ndarray | None, np.ndarray | None]:
        """Build (A_cbf, b_cbf) for A @ qddot >= b from active obstacles."""
        d_threshold = self._d_safe + self._margin
        self._kin.update(q, qdot)
        rows_A, rows_b = [], []

        for item in self._dists:
            if item['d'] > d_threshold or item['conf'] < 0.2:
                continue

            delta = item['pr'] - item['ph']
            d     = float(np.linalg.norm(delta))
            if d < 1e-8:
                continue
            n_w = delta / d

            fid = self._kin.resolve_frame_id(item['link'])
            if fid is None:
                continue

            Jp, _ = self._kin.point_jacobian(fid, item['pr'])
            if float(np.linalg.cond(Jp)) > 1e5:
                continue

            a = (n_w @ Jp).astype(np.float64)          # (NV,)
            h = d - self._d_safe                        # barrier value
            b = float(-self._k1 * (a @ qdot) - self._k0 * h)

            if np.all(np.isfinite(a)) and np.isfinite(b):
                rows_A.append(a)
                rows_b.append(b)

        if not rows_A:
            return None, None
        return np.vstack(rows_A), np.array(rows_b, dtype=np.float64)

    # ── Utility ──────────────────────────────────────────────────────────────

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CBFSafetyFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
