#!/usr/bin/env python3
"""Velocity-space CBF safety filter — Zeroing CBF (ZCBF), relative degree 1.

Implements a kinematic-level Control Barrier Function filter (Ferraguti et al.,
RAM 2022, eq. 12).  The barrier h = d - d_safe has relative degree 1 w.r.t. the
velocity input q̇: ḣ = n^T·J_p·q̇.  The QP solves directly for q̇_safe:

    min  ½‖q̇ - q̇_nom‖²  +  ½ρ·s²
    s.t. (n_i^T·J_p,i)·q̇ + s  ≥  -γ_i·h_i    ∀ active link i   (s ≥ 0)
         q̇_min ≤ q̇ ≤ q̇_max

The QP output is published directly — no post-processing is applied.
Any smoothing after the QP would invalidate the formal CBF safety guarantee.

Pipeline position:
  ee_pentagon_velocity_commander  →  /NS_1/tracking_qdot  (qdot_nom)
  cbf_velocity_filter             →  /NS_1/qdot_cmd       (qdot_safe)
  rt_velocity_executor_controller →  hardware

Topics (loaded from fr3_control.yaml and fr3_distance.yaml):
  Subscribes:
    - topics_ctr['joint_states_topic']   Joint states for kinematics
    - topics_vis['multi_distance']       MultiDistance from real_time_distance
    - topics_ctr['qdot_nom']             Nominal velocity from commander
  Publishes:
    - topics_ctr['velocity_topic']       Safe velocity command to controller

Parameters:
  bypass_cbf (bool, default False)
    When true, qdot_nom is passed through to qdot_cmd without solving the QP.
    Use for Phase 1 testing (no camera, no distances required).
"""

import time

import numpy as np
import qpsolvers as qp
import pinocchio as pin

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from franka_msgs.msg import MultiLinkDistance
from rclpy.qos import QoSProfile, ReliabilityPolicy

from franka_experiments.utils.ros_setup import init_pinocchio_only, make_joint_state_callback
from franka_experiments.utils.cbf_utils import load_robot_config, skew, select_gamma


class CbfVelocityFilter(Node):
    def __init__(self):
        super().__init__('cbf_velocity_filter')
        self.get_logger().info('CBF velocity filter initializing...')

        # ── Load configuration ─────────────────────────────────────────────
        self.vision_config  = load_robot_config('distance')
        self.robot_cfg      = self.vision_config['robot']
        self.topics_vis     = self.vision_config['topics']

        self.control_config = load_robot_config('control')
        self.topics_ctr     = self.control_config['topics']
        self.params         = self.control_config['params']
        self.limits         = self.control_config['joint_limits']

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('bypass_cbf', False)
        self._bypass_cbf = bool(self.get_parameter('bypass_cbf').value)

        # ── Pinocchio model ───────────────────────────────────────────────
        self.pin_ok, self.model, self.data = init_pinocchio_only(self)
        if not self.pin_ok:
            self.get_logger().error('Pinocchio initialization failed')
            return

        # Build frame-name lookup
        self._frame_id_cache: dict[str, int] = {}
        for link in self.robot_cfg.get('segment_links', []):
            fid = self.model.getFrameId(link)
            if fid < len(self.model.frames):
                self._frame_id_cache[link] = fid
            else:
                resolved = self._try_resolve_frame(link)
                if resolved is not None:
                    self._frame_id_cache[link] = resolved
                    actual = self.model.frames[resolved].name
                    self.get_logger().warn(
                        f"frame '{link}' not found, using fallback '{actual}'")
                else:
                    self.get_logger().error(f"frame '{link}' not found and no fallback")

        # ── Joint limits ──────────────────────────────────────────────────
        _joint_order = [f'joint{i}' for i in range(1, 8)]
        self.q_min     = np.array([self.limits[j][0] for j in _joint_order], dtype=np.float64)
        self.q_max     = np.array([self.limits[j][1] for j in _joint_order], dtype=np.float64)
        self.qdot_max  = np.array([self.limits[j][2] for j in _joint_order], dtype=np.float64)
        self.qdot_min  = -self.qdot_max


        self._dist_timeout = float(self.params.get('distance_timeout', 0.2))
        _rate_hz           = float(self.params.get('control_rate_hz', 200.0))

        # ── State ─────────────────────────────────────────────────────────
        self.q                       = None
        self.qdot                    = None
        self.qdot_nom                = np.zeros(self.model.nv, dtype=np.float64)
        self.multi_distances         = []
        self._last_distance_stamp    : float = 0.0   # time.monotonic()

        # ── Publisher ─────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(
            Float64MultiArray,
            self.topics_ctr['velocity_topic'],
            10,
        )

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            JointState,
            self.topics_ctr['joint_states_topic'],
            make_joint_state_callback(
                controller=self,
                joint_names=self.robot_cfg['joint_names'],
            ),
            10,
        )

        _be_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            MultiLinkDistance,
            self.topics_ctr['per_link_distances'],
            self._multi_distance_callback,
            _be_qos,
        )

        self.create_subscription(
            Float64MultiArray,
            self.topics_ctr['qdot_nom'],
            self._nominal_velocity_callback,
            10,
        )

        # ── Control timer ─────────────────────────────────────────────────
        self.create_timer(1.0 / _rate_hz, self._control_loop)

        mode_str = 'BYPASS (passthrough)' if self._bypass_cbf else 'CBF QP active'
        self.get_logger().info(
            f'CBF velocity filter ready — mode: {mode_str}\n'
            f'  qdot_nom topic  : {self.topics_ctr["qdot_nom"]}\n'
            f'  qdot_cmd topic  : {self.topics_ctr["velocity_topic"]}\n'
            f'  distance topic  : {self.topics_ctr["per_link_distances"]}'
        )

    # ── Frame resolution ──────────────────────────────────────────────────────

    def _try_resolve_frame(self, link_name: str):
        aliases = {'fr3_link8': ['fr3_hand', 'panda_link8', 'panda_hand']}
        for candidate in aliases.get(link_name, []):
            fid = self.model.getFrameId(candidate)
            if fid < len(self.model.frames):
                return fid
        for i, f in enumerate(self.model.frames):
            if link_name in f.name or f.name in link_name:
                return i
        return None

    def _resolve_frame_id(self, link_name: str):
        if link_name in self._frame_id_cache:
            return self._frame_id_cache[link_name]
        fid = self.model.getFrameId(link_name)
        if fid < len(self.model.frames):
            self._frame_id_cache[link_name] = fid
            return fid
        resolved = self._try_resolve_frame(link_name)
        if resolved is not None:
            self._frame_id_cache[link_name] = resolved
            return resolved
        return None

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _multi_distance_callback(self, msg: MultiLinkDistance) -> None:
        """Store per-link distances. One entry per link, closest CP per link."""
        self.multi_distances = [
            {
                'link_name':           ld.robot_link_name,
                'distance':            float(ld.distance),
                'closest_point_robot': np.array([
                    ld.closest_point_robot.x,
                    ld.closest_point_robot.y,
                    ld.closest_point_robot.z,
                ], dtype=np.float64),
                'direction': np.array([
                    ld.direction.x,
                    ld.direction.y,
                    ld.direction.z,
                ], dtype=np.float64),
                'zone':       ld.zone,
                'confidence': float(ld.confidence),
                'valid':      bool(ld.valid),
            }
            for ld in msg.links
            if ld.valid
        ]
        self._last_distance_stamp = time.monotonic()

    def _nominal_velocity_callback(self, msg):
        self.qdot_nom = np.array(msg.data, dtype=np.float64)

    # ── CBF QP ────────────────────────────────────────────────────────────────

    def _compute_point_jacobian(self, q, link_name, p_world):
        frame_id = self._resolve_frame_id(link_name)
        if frame_id is None:
            self.get_logger().warn(f'Cannot resolve frame: {link_name}')
            return None

        oMf = self.data.oMf[frame_id]
        r_world = p_world - oMf.translation

        J6 = pin.computeFrameJacobian(
            self.model, self.data, q, frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        Jv = J6[:3, :]
        Jw = J6[3:, :]
        Jp = Jv - skew(r_world) @ Jw

        if not np.all(np.isfinite(Jp)):
            self.get_logger().error('Jacobian contains non-finite values')
            return None
        return Jp

    def _build_cbf_constraints(self, q, qdot, multi_distances):
        """Build ZCBF constraints in velocity space (Ferraguti RAM 2022, eq. 12).

        For each active link i:
            h_i   = d_i - d_safe
            a_vel = n_i^T · J_p,i      (nv,)  — Lie derivative of h w.r.t. q̇
            b_vel = -gamma_i · h_i     scalar — pure ZCBF bound

        Constraint form: a_vel · q̇_cmd + s >= b_vel  (enforced via QP with slack s).

        NOTE: b_vel = -gamma*h (no hdot term).  Including hdot = a·q̇_meas in
        b_vel creates a velocity-feedback loop that at 200 Hz generates chattering
        at the boundary.  The pure -γh term is sufficient: it enforces
        h(t+dt) >= h(t)*(1 - gamma*dt) = h(t)*0.96 at each 5 ms tick, which is
        the standard discrete ZCBF safety condition.
        qdot is accepted as argument for future use but is not read here.
        """
        constraints = []
        if not multi_distances:
            return constraints

        act_margin = float(self.params.get('cbf_activation_margin', 0.10))

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        for item in multi_distances:
            link_name = item['link_name']
            d         = item['distance']
            p_world   = item['closest_point_robot']
            n_world   = item['direction'].copy()

            if not np.isfinite(d) or p_world is None or n_world is None:
                continue
            norm_n = np.linalg.norm(n_world)
            if norm_n < 1e-8:
                continue
            n_world /= norm_n

            h = float(d) - float(self.params['d_safe'])

            # ── Activation gate — skip links outside the activation zone ────
            if h > act_margin:
                continue

            if h < 0.0:
                self.get_logger().warn(
                    f'[CBF-VEL] h < 0 on {link_name}: h={h:.4f} m — '
                    'robot inside safe set, CBF cannot guarantee recovery',
                    throttle_duration_sec=1.0,
                )

            gamma = float(select_gamma(
                item['zone'], item['confidence'],
                d=float(d), d_safe=float(self.params['d_safe'])))

            Jp = self._compute_point_jacobian(q, link_name, p_world)
            if Jp is None:
                continue

            # ZCBF: a·q̇_cmd ≥ -γ·h  (standard discrete ZCBF, no velocity feedback)
            a_vel = (n_world @ Jp).astype(np.float64)   # (nv,)
            b_vel = float(-gamma * h)                    # scalar

            if not np.all(np.isfinite(a_vel)) or not np.isfinite(b_vel):
                continue
            constraints.append((a_vel, b_vel, link_name, d))

        return constraints

    def _solve_cbf_qp(self, qdot_nom, constraints):
        """Solve the ZCBF velocity QP.

        Decision variable: x = [q̇_1, ..., q̇_nv, s]  (nv + 1)

        min  ½‖q̇ - q̇_nom‖²  +  ½ρ·s²
        s.t. -a_vel·q̇ - s  ≤  -b_vel    ∀ constraint (CBF)
             q̇_min ≤ q̇ ≤ q̇_max          (box via lb/ub)
             s ≥ 0                        (box via lb)

        Returns q̇_safe (nv,).  On solver failure returns a soft fallback:
        half the clipped nominal velocity.  No warm-starting — OSQP handles
        that internally and external initvals can cause inconsistencies.
        """
        nv  = self.model.nv
        rho = float(self.params['rho_slack'])

        P = np.eye(nv + 1, dtype=np.float64)
        P[-1, -1] = rho

        q_vec = np.zeros(nv + 1, dtype=np.float64)
        q_vec[:nv] = -np.asarray(qdot_nom, dtype=np.float64)

        lb = np.concatenate([self.qdot_min, [0.0]])
        ub = np.concatenate([self.qdot_max, [1e6]])

        G_rows, h_rows = [], []
        for a_vel, b_vel, _, _ in constraints:
            # a_vel·q̇ + s >= b_vel  →  -a_vel·q̇ - s <= -b_vel
            row = np.zeros(nv + 1, dtype=np.float64)
            row[:nv] = -a_vel
            row[-1]  = -1.0
            G_rows.append(row)
            h_rows.append(-b_vel)

        G = np.vstack(G_rows).astype(np.float64) if G_rows else None
        h = np.array(h_rows, dtype=np.float64)   if h_rows else None

        def _soft_fallback(reason: str) -> np.ndarray:
            min_d = min((d for _, _, _, d in constraints), default=float('nan'))
            self.get_logger().error(
                f'[CBF-VEL] QP {reason}  n_c={len(constraints)}'
                f'  min_d={min_d:.3f} m — soft fallback (0.5 × qdot_nom)',
                throttle_duration_sec=0.5,
            )
            return np.clip(qdot_nom, self.qdot_min, self.qdot_max) * 0.5

        try:
            x = qp.solve_qp(
                P=P, q=q_vec, G=G, h=h, A=None, b=None,
                lb=lb, ub=ub,
                solver=str(self.params['qp_solver']),
                verbose=False,
            )
        except Exception as e:
            self.get_logger().error(f'QP solver exception: {e}')
            return _soft_fallback('exception')

        if x is None or not np.all(np.isfinite(x)):
            return _soft_fallback('infeasible/non-finite')

        x      = np.asarray(x, dtype=np.float64).reshape(-1)
        slack  = float(x[-1])
        if slack > 0.1:
            min_d = min((d for _, _, _, d in constraints), default=float('nan'))
            self.get_logger().error(
                f'[CBF-VEL] large slack s={slack:.4f} m/s  n_c={len(constraints)}'
                f'  min_d={min_d:.3f} m — CBF constraint violated',
                throttle_duration_sec=0.5,
            )
        return x[:nv]

    # ── Main control step (called at 200 Hz by ROS2 timer) ───────────────────

    def _control_loop(self):
        if self.q is None or self.qdot is None:
            return

        if self._bypass_cbf:
            msg = Float64MultiArray()
            msg.data = self.qdot_nom.tolist()
            self.cmd_pub.publish(msg)
            return

        # Distance staleness guard — passthrough if data is too old
        age = time.monotonic() - self._last_distance_stamp
        if age > self._dist_timeout:
            self.get_logger().warn(
                f'[CBF-VEL] distance stale ({age:.3f} s > {self._dist_timeout:.3f} s)'
                ' — passthrough',
                throttle_duration_sec=2.0,
            )
            msg = Float64MultiArray()
            msg.data = np.clip(self.qdot_nom, self.qdot_min, self.qdot_max).tolist()
            self.cmd_pub.publish(msg)
            return

        q    = self.q.copy()
        qdot = self.qdot.copy()

        constraints = self._build_cbf_constraints(q, qdot, list(self.multi_distances))

        if constraints:
            links_active = [lname for _, _, lname, _ in constraints]
            min_d = min(d for _, _, _, d in constraints)
            self.get_logger().info(
                f'[CBF-VEL] active_links={links_active}  '
                f'n_constraints={len(constraints)}  min_d={min_d:.3f} m',
                throttle_duration_sec=1.0,
            )
        else:
            self.get_logger().info(
                '[CBF-VEL] no valid distances — nominal passthrough',
                throttle_duration_sec=2.0,
            )

        qdot_cmd = self._solve_cbf_qp(self.qdot_nom.copy(), constraints)
        qdot_cmd = np.clip(qdot_cmd, self.qdot_min, self.qdot_max)

        if constraints:
            min_margin = min(float(a_vel @ qdot_cmd) - b_vel
                            for a_vel, b_vel, _, _ in constraints)
            self.get_logger().info(
                f'[CBF-VEL] min_constraint_margin={min_margin:.4f} m/s'
                f'  (≥0 → all constraints satisfied)',
                throttle_duration_sec=1.0,
            )

        msg = Float64MultiArray()
        msg.data = [float(v) for v in qdot_cmd]
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CbfVelocityFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
