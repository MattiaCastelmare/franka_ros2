#!/usr/bin/env python3
"""CBF Safety Filter — acceleration-level QP node.

Subscribes:
  /NS_1/joint_states       (sensor_msgs/JointState)
  /NS_1/qddot_nom          (std_msgs/Float64MultiArray, 7-dim)
  /cbf/per_link_distances  (franka_msgs/MultiLinkDistance)  [Phase 2 only]

Publishes:
  /NS_1/qddot_safe         (std_msgs/Float64MultiArray, 7-dim)  — monitoring
  /NS_1/torque_cmd         (std_msgs/Float64MultiArray, 7-dim)  — to rt_torque_controller

Runs at fixed 200 Hz.

Parameters
----------
bypass_cbf : bool (default False)
    When True skips the QP and passes qddot_nom straight through to dynamics.
    Use for Phase-1 testing without obstacle distances.
torque_out_topic : str (default '/NS_1/torque_cmd')
    Topic on which τ = M(q)·qddot_safe + C(q,q̇)·q̇ is published.
    rt_torque_controller adds gravity on top.

Models
------
CBF kinematics model: loaded without hand (nv=7) — used for constraint Jacobians.
Dynamics model: loaded with hand:=true (same URDF as rt_torque_controller) —
  used for accurate RNEA including hand mass.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from franka_msgs.msg import MultiLinkDistance

import pinocchio as pin
import yaml
import os
import glob
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory

from franka_experiments.utils.kinematics import (
    generate_urdf_from_xacro,
    load_pinocchio_model,
    resolve_arm_joint_ids,
)
from franka_experiments.utils.cbf_kinematics  import CBFKinematics
from franka_experiments.utils.cbf_constraints import build_hocbf_constraints, CBFParams
from franka_experiments.utils.cbf_qp          import CBFQP


def _load_yaml(pkg: str, rel: str) -> dict:
    path = os.path.join(get_package_share_directory(pkg), rel)
    with open(path) as f:
        return yaml.safe_load(f)


FR3_JOINT_NAMES = [f'fr3_joint{i}' for i in range(1, 8)]
_NUM_ARM = 7


class CBFSafetyFilter(Node):
    def __init__(self):
        super().__init__('cbf_safety_filter')

        # ── Config from YAML ────────────────────────────────────────────────
        cfg    = _load_yaml('franka_experiments', 'config/fr3_control.yaml')
        topics = cfg['topics']
        p      = cfg['params']
        lims   = cfg['joint_limits']

        keys = [f'joint{i}' for i in range(1, 8)]
        self.q_min     = np.array([lims[k][0] for k in keys])
        self.q_max     = np.array([lims[k][1] for k in keys])
        self.qdot_max  = np.array([lims[k][2] for k in keys])
        self.qdot_min  = -self.qdot_max
        self.qddot_max = np.array([lims[k][3] for k in keys])
        self.qddot_min = -self.qddot_max

        self.dt           = 1.0 / float(p['control_rate_hz'])
        self.dist_timeout = float(p['distance_timeout'])
        self.nom_timeout  = 0.5

        # ── ROS parameters ──────────────────────────────────────────────────
        self.declare_parameter('bypass_cbf',       False)
        self.declare_parameter('torque_out_topic', '/NS_1/torque_cmd')
        self.bypass_cbf    = self.get_parameter('bypass_cbf').value
        torque_out_topic   = self.get_parameter('torque_out_topic').value

        # ── CBF kinematics model (no hand, nv=7) — for constraint Jacobians ─
        cbf_urdf = self._find_urdf_no_hand()
        cbf_model = pin.buildModelFromUrdf(cbf_urdf)
        self.kin = CBFKinematics(cbf_model)

        self.cbf_params = CBFParams(
            k0=float(p['k0']),
            k1=float(p['k1']),
            d_safe_default=float(p['d_safe_default']),
        )
        self.qp = CBFQP(
            nv=cbf_model.nv,
            rho_slack=float(p['rho_slack']),
            solver=str(p['qp_solver']),
        )

        # ── Dynamics model (hand:=true) — for τ = M·q̈ + C·q̇ ──────────────
        self.get_logger().info('Building dynamics model (hand:=true) …')
        urdf_xml = generate_urdf_from_xacro()
        self._dyn_model, self._dyn_data = load_pinocchio_model(urdf_xml)

        _pin_jids       = resolve_arm_joint_ids(self._dyn_model)
        self._arm_v_ids = [self._dyn_model.joints[j].idx_v for j in _pin_jids]
        self._arm_q_ids = [self._dyn_model.joints[j].idx_q for j in _pin_jids]
        self._arm_v_ix  = np.ix_(self._arm_v_ids, self._arm_v_ids)

        _nv = self._dyn_model.nv
        self._q_neutral_dyn  = pin.neutral(self._dyn_model)
        self._q_full_dyn     = pin.neutral(self._dyn_model)
        self._qdot_full_dyn  = np.zeros(_nv)
        self._Cqdot_full_dyn = np.zeros(_nv)
        self._M_arm          = np.zeros((_NUM_ARM, _NUM_ARM))
        self._C_qdot_arm     = np.zeros(_NUM_ARM)
        self._tau_arm        = np.zeros(_NUM_ARM)
        self._tau_msg        = Float64MultiArray()
        self._tau_msg.data   = [0.0] * _NUM_ARM

        # Warm-up Pinocchio to avoid first-call overhead in the RT loop
        pin.computeAllTerms(self._dyn_model, self._dyn_data,
                            self._q_full_dyn, self._qdot_full_dyn)

        # ── Node state ──────────────────────────────────────────────────────
        self.q            = None
        self.qdot         = None
        self.t_js         = None
        self.qddot_nom    = np.zeros(cbf_model.nv)
        self.t_nom        = None
        self.link_distances = []
        self.t_dist       = None

        # ── Subscriptions ───────────────────────────────────────────────────
        self.create_subscription(
            JointState, topics['joint_states_topic'],
            self._on_joint_state, 10)
        self.create_subscription(
            Float64MultiArray, topics['qddot_nom'],
            self._on_qddot_nom, 10)
        if not self.bypass_cbf:
            self.create_subscription(
                MultiLinkDistance, '/cbf/per_link_distances',
                self._on_distances, 10)

        # ── Publishers ──────────────────────────────────────────────────────
        self.pub     = self.create_publisher(
            Float64MultiArray, topics['qddot_safe'], 10)
        self.tau_pub = self.create_publisher(
            Float64MultiArray, torque_out_topic, 10)

        # ── Control loop ────────────────────────────────────────────────────
        self.create_timer(self.dt, self._tick)

        mode_str = 'BYPASS (no QP)' if self.bypass_cbf else 'CBF QP active'
        self.get_logger().info(
            f'CBF safety filter  [{mode_str}]  {1.0/self.dt:.0f} Hz\n'
            f'  qddot_nom  ← {topics["qddot_nom"]}\n'
            f'  qddot_safe → {topics["qddot_safe"]}  (monitoring)\n'
            f'  torque     → {torque_out_topic}  (to rt_torque_controller)')

    # ── Helpers ─────────────────────────────────────────────────────────────

    def _find_urdf_no_hand(self) -> str:
        """Generate FR3 URDF without hand, return path to temp file."""
        share = get_package_share_directory('franka_description')
        candidates = glob.glob(
            os.path.join(share, '**', 'fr3.urdf.xacro'), recursive=True)
        if not candidates:
            raise RuntimeError(f'fr3.urdf.xacro not found under {share}')
        tmp = tempfile.NamedTemporaryFile(
            suffix='.urdf', delete=False, prefix='fr3_cbf_nohand_')
        tmp.close()
        result = subprocess.run(
            ['xacro', candidates[0], 'hand:=false', '-o', tmp.name],
            capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(f'xacro failed:\n{result.stderr}')
        return tmp.name

    def _compute_dynamics(self, q_arm: np.ndarray, qdot_arm: np.ndarray) -> None:
        """Fill _M_arm (7×7) and _C_qdot_arm (7) from robot state.

        τ = M_arm · q̈_arm + C_qdot_arm   (gravity excluded — added by rt_torque_controller)
        """
        np.copyto(self._q_full_dyn, self._q_neutral_dyn)
        self._qdot_full_dyn[:] = 0.0
        for k, (iq, iv) in enumerate(zip(self._arm_q_ids, self._arm_v_ids)):
            self._q_full_dyn[iq]    = q_arm[k]
            self._qdot_full_dyn[iv] = qdot_arm[k]
        pin.computeAllTerms(self._dyn_model, self._dyn_data,
                            self._q_full_dyn, self._qdot_full_dyn)
        M_full = np.asarray(self._dyn_data.M)
        np.copyto(self._M_arm, M_full[self._arm_v_ix])
        np.dot(np.asarray(self._dyn_data.C), self._qdot_full_dyn,
               out=self._Cqdot_full_dyn)
        for k, iv in enumerate(self._arm_v_ids):
            self._C_qdot_arm[k] = self._Cqdot_full_dyn[iv]

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _on_joint_state(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        name_to_vel = dict(zip(msg.name, msg.velocity))
        try:
            q    = np.array([name_to_pos[n] for n in FR3_JOINT_NAMES])
            qdot = np.array([name_to_vel[n] for n in FR3_JOINT_NAMES])
        except KeyError:
            return
        self.q, self.qdot = q, qdot
        self.t_js = self.get_clock().now().nanoseconds * 1e-9

    def _on_qddot_nom(self, msg: Float64MultiArray):
        data = np.asarray(msg.data, dtype=np.float64)
        if len(data) == self.qp.nv:
            self.qddot_nom = data
            self.t_nom = self.get_clock().now().nanoseconds * 1e-9

    def _on_distances(self, msg: MultiLinkDistance):
        out = []
        for ld in msg.links:
            out.append({
                'link_name':           ld.robot_link_name,
                'distance':            float(ld.distance),
                'closest_point_robot': np.array([ld.closest_point_robot.x,
                                                  ld.closest_point_robot.y,
                                                  ld.closest_point_robot.z]),
                'closest_point_human': np.array([ld.closest_point_human.x,
                                                  ld.closest_point_human.y,
                                                  ld.closest_point_human.z]),
                'valid':      bool(ld.valid),
                'confidence': float(ld.confidence),
                'zone':       ld.zone,
            })
        self.link_distances = out
        self.t_dist = self.get_clock().now().nanoseconds * 1e-9

    # ── Control tick (200 Hz) ────────────────────────────────────────────────

    def _tick(self):
        if self.q is None:
            return

        now       = self.get_clock().now().nanoseconds * 1e-9
        nom_stale = (self.t_nom is None) or (now - self.t_nom > self.nom_timeout)

        if self.bypass_cbf:
            # ── Phase 1: bypass QP, pass qddot_nom straight through ─────────
            qddot_arm = np.zeros(_NUM_ARM) if nom_stale else self.qddot_nom.copy()

        else:
            # ── Phase 2: full CBF QP ─────────────────────────────────────────
            dist_stale = (self.t_dist is None) or (now - self.t_dist > self.dist_timeout)
            if dist_stale:
                self.get_logger().warn('distance stale → braking',
                                       throttle_duration_sec=1.0)
                self._publish_braking()
                return

            qddot_nom = np.zeros(self.qp.nv) if nom_stale else self.qddot_nom.copy()

            try:
                A, b, meta = build_hocbf_constraints(
                    self.kin, self.q, self.qdot,
                    self.link_distances,
                    p_h_dot_est=None,
                    p_h_ddot_est=None,
                    cbf_params=self.cbf_params,
                )
            except Exception as exc:
                self.get_logger().error(f'CBF build error: {exc} → braking',
                                        throttle_duration_sec=1.0)
                self._publish_braking()
                return

            qddot_safe, slack = self.qp.solve(
                qddot_nom=qddot_nom,
                qdot=self.qdot, q=self.q, dt=self.dt,
                A_cbf=A, b_cbf=b,
                qddot_min=self.qddot_min, qddot_max=self.qddot_max,
                qdot_min=self.qdot_min,   qdot_max=self.qdot_max,
                q_min=self.q_min,         q_max=self.q_max,
            )

            if qddot_safe is None:
                self.get_logger().error('QP failed → braking',
                                        throttle_duration_sec=1.0)
                self._publish_braking()
                return

            qddot_arm = qddot_safe.copy()

            if slack > 1e-6:
                self.get_logger().warn(f'slack={slack:.3e}  active_links={len(meta)}',
                                       throttle_duration_sec=0.5)

        # ── τ = M(q)·q̈ + C(q,q̇)·q̇  (gravity excluded — rt_torque_controller adds it) ──
        self._compute_dynamics(self.q, self.qdot)
        np.dot(self._M_arm, qddot_arm, out=self._tau_arm)
        self._tau_arm += self._C_qdot_arm

        # Publish qddot for monitoring / downstream CBF debugging
        qddot_msg      = Float64MultiArray()
        qddot_msg.data = qddot_arm.tolist()
        self.pub.publish(qddot_msg)

        # Publish torques to rt_torque_controller
        for i in range(_NUM_ARM):
            self._tau_msg.data[i] = float(self._tau_arm[i])
        self.tau_pub.publish(self._tau_msg)

    def _publish_braking(self):
        """Decelerate each joint toward zero at max rate and publish braking torques."""
        if self.qdot is None:
            return
        qddot_need  = -self.qdot / max(self.dt, 1e-3)
        qddot_brake = np.clip(qddot_need, self.qddot_min, self.qddot_max)

        qddot_msg      = Float64MultiArray()
        qddot_msg.data = qddot_brake.tolist()
        self.pub.publish(qddot_msg)

        if self.q is None:
            return
        self._compute_dynamics(self.q, self.qdot)
        np.dot(self._M_arm, qddot_brake, out=self._tau_arm)
        self._tau_arm += self._C_qdot_arm
        for i in range(_NUM_ARM):
            self._tau_msg.data[i] = float(self._tau_arm[i])
        self.tau_pub.publish(self._tau_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CBFSafetyFilter()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
