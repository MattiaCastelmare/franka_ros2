#!/usr/bin/env python3
"""Velocity-space CBF safety filter.

Subscribes to a nominal joint velocity command (tracking_qdot), applies a
Control Barrier Function QP to enforce obstacle-avoidance constraints derived
from real-time distance estimates, and publishes the safe velocity command
(qdot_cmd) to the robot velocity controller.

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
import threading

import numpy as np
import qpsolvers as qp
import pinocchio as pin

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from franka_msgs.msg import MultiDistance

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
        self.qddot_max = np.array([self.limits[j][3] for j in _joint_order], dtype=np.float64)

        # ── State ─────────────────────────────────────────────────────────
        self.q              = None
        self.qdot           = None
        self.qdot_nom       = np.zeros(self.model.nv, dtype=np.float64)
        self.multi_distances = []
        self._last_solve_time = None
        self.last_qp_slack   = 0.0
        self.qdot_cmd_prev   = np.zeros(self.model.nv, dtype=np.float64)
        self._has_prev_cmd   = False
        self.qp_initvals     = None

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

        self.create_subscription(
            MultiDistance,
            self.topics_vis['multi_distance'],
            self._multi_distance_callback,
            10,
        )

        self.create_subscription(
            Float64MultiArray,
            self.topics_ctr['qdot_nom'],
            self._nominal_velocity_callback,
            10,
        )

        # ── Solver thread (skips main work in bypass mode) ────────────────
        self._new_distance = threading.Event()
        self._solver_thread = threading.Thread(
            target=self._solver_loop, daemon=True)
        self._solver_thread.start()

        mode_str = 'BYPASS (passthrough)' if self._bypass_cbf else 'CBF QP active'
        self.get_logger().info(
            f'CBF velocity filter ready — mode: {mode_str}\n'
            f'  qdot_nom topic  : {self.topics_ctr["qdot_nom"]}\n'
            f'  qdot_cmd topic  : {self.topics_ctr["velocity_topic"]}\n'
            f'  distance topic  : {self.topics_vis["multi_distance"]}'
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

    def _multi_distance_callback(self, msg):
        distances = []
        for item in msg.distances:
            distances.append({
                'link_name':            item.robot_link_name,
                'distance':             float(item.distance),
                'closest_point_robot':  np.array([
                    item.closest_point_robot.x,
                    item.closest_point_robot.y,
                    item.closest_point_robot.z,
                ], dtype=np.float64),
                'direction': np.array([
                    item.direction.x,
                    item.direction.y,
                    item.direction.z,
                ], dtype=np.float64),
                'zone':       item.zone,
                'confidence': float(item.confidence),
                'valid':      bool(item.valid),
            })
        self.multi_distances = distances
        self._new_distance.set()

    def _nominal_velocity_callback(self, msg):
        self.qdot_nom = np.array(msg.data, dtype=np.float64)
        if self._bypass_cbf:
            # Passthrough: publish nominal directly as safe command.
            self.cmd_pub.publish(msg)

    # ── Solver loop ───────────────────────────────────────────────────────────

    def _solver_loop(self):
        while rclpy.ok():
            if self._bypass_cbf:
                # Nothing to do — _nominal_velocity_callback handles publishing.
                time.sleep(1.0)
                continue

            triggered = self._new_distance.wait(timeout=0.5)
            self._new_distance.clear()
            if not triggered:
                self.get_logger().warn(
                    'No MultiDistance message in 0.5 s — is real_time_distance running?',
                    throttle_duration_sec=5.0,
                )
                continue
            if self.q is None or self.qdot is None:
                self.get_logger().warn(
                    'Distance received but joint state not ready — '
                    'is the robot/joint_states topic publishing?',
                    throttle_duration_sec=5.0,
                )
                continue
            self._control_loop()

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

    def _build_cbf_constraints(self, q, qdot, dt, multi_distances):
        constraints = []
        if not multi_distances:
            return constraints

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        for item in multi_distances:
            if not item['valid']:
                continue
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

            h     = float(d) - float(self.params['d_safe'])
            gamma = float(select_gamma(item['zone'], item['confidence']))

            Jp = self._compute_point_jacobian(q, link_name, p_world)
            if Jp is None:
                continue

            a    = (n_world @ Jp).astype(np.float64)
            hdot = float(a @ qdot)

            # CBF constraint at acceleration level:
            #   a·(qdot + qddot·dt) ≥ -γ·h  →  (a·dt)·qddot ≥ -γ·h - a·qdot
            a_acc = (a * dt).astype(np.float64)
            b_acc = float(-gamma * h - hdot)

            if not np.all(np.isfinite(a_acc)) or not np.isfinite(b_acc):
                continue
            constraints.append((a_acc, b_acc, link_name, d))

        return constraints

    def _solve_cbf_qp(self, qddot_nom, constraints, qdot, dt):
        n       = self.model.nv
        rho     = float(self.params['rho_slack'])

        P = np.eye(n + 1, dtype=np.float64)
        P[-1, -1] = rho

        q_vec = np.zeros(n + 1, dtype=np.float64)
        q_vec[:n] = -np.asarray(qddot_nom, dtype=np.float64)

        lb = np.concatenate([-self.qddot_max, [0.0]])
        ub = np.concatenate([ self.qddot_max, [np.inf]])

        G_rows, h_rows = [], []

        # Velocity-next box constraints: qdot_min ≤ qdot + qddot·dt ≤ qdot_max
        for i in range(n):
            row = np.zeros(n + 1, dtype=np.float64)
            row[i] = dt
            G_rows.append(row);  h_rows.append(self.qdot_max[i] - qdot[i])

            row2 = np.zeros(n + 1, dtype=np.float64)
            row2[i] = -dt
            G_rows.append(row2); h_rows.append(qdot[i] - self.qdot_min[i])

        # CBF constraints
        for a, b, _, _ in constraints:
            row = np.zeros(n + 1, dtype=np.float64)
            row[:n] = -a
            row[-1] = -1.0
            G_rows.append(row); h_rows.append(-b)

        G = np.vstack(G_rows).astype(np.float64) if G_rows else None
        h = np.array(h_rows, dtype=np.float64) if h_rows else None

        try:
            x = qp.solve_qp(
                P=P, q=q_vec, G=G, h=h, A=None, b=None,
                lb=lb, ub=ub,
                solver=str(self.params['qp_solver']),
                qp_initvals=self.qp_initvals,
                verbose=False,
            )
        except Exception as e:
            self.get_logger().error(f'QP solver exception: {e}')
            self.qp_initvals = None
            return np.zeros(n, dtype=np.float64)

        if x is None or not np.all(np.isfinite(x)):
            self.get_logger().warn('QP failed, returning zero acceleration')
            self.qp_initvals = None
            return np.zeros(n, dtype=np.float64)

        x = np.asarray(x, dtype=np.float64).reshape(-1)
        self.qp_initvals  = x.copy()
        self.last_qp_slack = float(x[-1])
        return x[:n]

    # ── Main control step ─────────────────────────────────────────────────────

    def _control_loop(self):
        if self.q is None or self.qdot is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_solve_time is None:
            dt = 0.01
        else:
            dt = float(np.clip(now - self._last_solve_time, 0.001, 0.05))
        self._last_solve_time = now

        q    = self.q.copy()
        qdot = self.qdot.copy()

        qddot_nom = (self.qdot_nom - qdot) / dt
        qddot_nom = np.clip(qddot_nom, -self.qddot_max, self.qddot_max)

        constraints = self._build_cbf_constraints(q, qdot, dt, list(self.multi_distances))

        if constraints:
            min_d = min(d for _, _, _, d in constraints)
            self.get_logger().info(
                f'[CBF-VEL] constraints={len(constraints)}  min_d={min_d:.3f} m  dt={dt:.4f}',
                throttle_duration_sec=1.0,
            )
        else:
            self.get_logger().info(
                '[CBF-VEL] no valid distances — nominal passthrough',
                throttle_duration_sec=2.0,
            )

        qddot_cmd  = self._solve_cbf_qp(qddot_nom, constraints, qdot, dt)
        qdot_cmd   = np.clip(qdot + qddot_cmd * dt, self.qdot_min, self.qdot_max)

        alpha = float(self.params['ema_alpha'])
        if not self._has_prev_cmd:
            self._has_prev_cmd = True
        else:
            qdot_cmd = alpha * qdot_cmd + (1.0 - alpha) * self.qdot_cmd_prev
        self.qdot_cmd_prev = qdot_cmd.copy()

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
