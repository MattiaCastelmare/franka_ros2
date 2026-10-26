#!/usr/bin/env python3
"""MPC commander: tracks a task preview with the condensed QP of utils/mpc_problem.py.

Every dt (config/mpc.yaml) control_loop:
  1. reads q, q̇ of the 7 arm joints (1 kHz joint states) and the q̈ in force on them;
  2. samples the task preview (/task/trajectory) at t0 + k·dt, k = 1..N, where t0 = now +
     input_delay_s is when u_0 reaches the controller (this loop's own time);
  3. predicts the state at t0 and linearises around the shifted previous plan q̄_k;
  4. adds the task terms and 5. the human rows to the QP;
  6. solves; on failure runs the previous plan for max_failures cycles, then brakes;
  7. publishes u_0 on /NS_1/qddot_nom (or the brake), 8. the plan and its next state;
  and reports one MpcDiagnostics per solve, one summary line per log_period_s.

The node does not know the task: pick_place_qddot_commander with reference_only: true
publishes the preview (easy_torque.launch.py commander:=mpc). With control_mode:=cbf the
CBF filters /NS_1/qddot_nom downstream (shielded MPC), and its output /NS_1/qddot_safe is
read back as the q̈ in force.

q̈ = 0 is NOT a stop: rt_torque_controller integrates q̈, so every stop path brakes.
"""
import os
import dataclasses
from functools import partial

import numpy as np
import pinocchio as pin
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from scipy.spatial.transform import Rotation, Slerp
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import MultiDOFJointTrajectory

from franka_msgs.msg import HumanArmState, MpcDiagnostics
from franka_experiments.utils.capsule_geometry import HumanArmGeometry, point_to_capsule_distance
from franka_experiments.utils.constants import AUTO_SENTINEL, FR3_JOINT_NAMES, NUM_JOINTS
from franka_experiments.utils.distance_utils import load_robot_config
from franka_experiments.utils.human_utils import (
    define_control_points, ee_trajectory_arrays, extract_human_covariances, extract_human_keypoints,
    fill_ee_trajectory_msg, format_topic, make_ee_trajectory_msg, mpc_diagnostics_msg, braking_qddot
)
from franka_experiments.utils.kinematics import (
    CBFKinematics, generate_urdf_from_xacro, load_pinocchio_model, resolve_arm_joint_ids,
    resolve_frame_id, so3_log,
)
from franka_experiments.utils.logging_utils import ThrottledLogger, format_mpc_summary
from franka_experiments.utils.mpc_problem import MPCProblem, MPCSettings
from franka_experiments.utils.node_runtime import get_namespace_from_config, run_node_main

MAX_IMAGE_LATENCY_S = 0.5   # an arm state stamped further back is a replayed bag: use its receipt time


class MPCCommander(Node):
    def __init__(self, **node_kwargs):
        # node_kwargs: e.g. parameter_overrides, for scripts/mpc_human_replay.py
        super().__init__('mpc_commander', **node_kwargs)
        self.done = False
        config_dir = os.path.join(get_package_share_directory('franka_experiments'), 'config')

        # ── Parameters: every key of config/mpc.yaml ─────────────────────────
        for section in load_robot_config(os.path.join(config_dir, 'mpc.yaml')).values():
            for name, value in section.items():
                self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value
        self.settings = MPCSettings(**{field.name: param(field.name)
                                       for field in dataclasses.fields(MPCSettings)})
        self.dt = self.settings.dt
        self.input_delay_s = float(param('input_delay_s'))
        if not 0.0 <= self.input_delay_s < self.dt:
            raise ValueError('input_delay_s must be in [0, dt)')
        self.brake_gain = float(param('brake_gain'))
        self.joint_state_timeout_s = float(param('joint_state_timeout_s'))
        self.task_timeout_s = float(param('task_timeout_s'))
        self.max_failures = int(param('max_failures'))
        self.ee_frame = str(param('ee_frame'))
        self.q_home = np.asarray(param('q_home'), dtype=float)
        self.human_enabled = bool(param('human_enabled'))
        self.human_mode = str(param('human_mode'))
        if self.human_mode not in ('prediction', 'static'):
            raise ValueError("human_mode must be 'prediction' or 'static'")
        self.prediction_cap_s = float(param('prediction_cap_s'))
        self.velocity_deadband = float(param('velocity_deadband'))
        self.t_blind = float(param('t_blind'))
        self.k_sigma = float(param('k_sigma'))
        self.margin_max = float(param('margin_max'))
        self.max_state_age_s = float(param('max_state_age_s'))
        self.d_activate = float(param('d_activate'))

        # ── Joint limits (fr3_control.yaml, as the commander) and the QP ─────
        control_cfg = load_robot_config(os.path.join(config_dir, 'fr3_control.yaml'))
        control_topics = control_cfg['topics']
        limits = np.array([control_cfg['joint_limits'][f'joint{i}'] for i in range(1, NUM_JOINTS + 1)])
        self.qddot_max = np.minimum(limits[:, 3], float(param('qddot_cap')))
        self.problem = MPCProblem(self.settings, q_min=limits[:, 0], q_max=limits[:, 1],
                                  qdot_max=limits[:, 2], qddot_max=self.qddot_max)

        # ── Robot model: TCP, arm joints, control points (as human_distance and the CBF) ──
        self.model, _ = load_pinocchio_model(generate_urdf_from_xacro())
        self.kin = CBFKinematics(self.model)
        self.ee_frame_id = resolve_frame_id(self.model, self.ee_frame)
        arm_joints = [self.model.joints[j] for j in resolve_arm_joint_ids(self.model)]
        self.arm_q_idx = [joint.idx_q for joint in arm_joints]
        self.arm_v_idx = [joint.idx_v for joint in arm_joints]
        self.q_full = pin.neutral(self.model)   # fingers stay at their neutral position
        robot_cfg = load_robot_config(os.path.join(config_dir, 'fr3_complete.yaml'))
        self.robot_cfg, self.distance_cfg = robot_cfg['robot'], robot_cfg['distance']
        self.link_frame_ids = {frame.name: self.model.getFrameId(frame.name)
                               for frame in self.model.frames if frame.name.startswith('fr3_link')}
        # Same arm capsules as human_distance
        self.human_geometry = HumanArmGeometry(upper_arm_radius=0.075, forearm_radius=0.065, hand_radius=0.075)

        # ── State ────────────────────────────────────────────────────────────
        self.latest_joint_state = None          # (msg, receipt time)
        self.latest_task_trajectory = None      # (msg, receipt time)
        self.latest_qddot_safe = None           # (msg, receipt time): the shield's output, if any
        self.last_qdot = np.zeros(NUM_JOINTS)   # for braking without a fresh joint state
        self.U = np.zeros(self.problem.nu)      # current plan
        self.u_prev = np.zeros(NUM_JOINTS)      # last published q̈: the q̈ in force without a shield
        self.failures = 0
        self.stop_end = None                    # request_stop: brake until this time
        self.diag_window, self.fallbacks = [], 0
        self.summary_log = ThrottledLogger(self.get_logger(), period_s=float(param('log_period_s')))

        # ── Subscriptions ────────────────────────────────────────────────────
        human_cfg = load_robot_config(os.path.join(config_dir, 'human_params.yaml'))['human_tracker']
        pose_side = str(human_cfg['pose_side']).lower()
        self.active_sides = ['left', 'right'] if pose_side == 'both' else [pose_side]

        js_topic = param('joint_state_topic')
        if js_topic == AUTO_SENTINEL:
            # joint_states_fast = the joint_state_broadcaster's own 1 kHz output
            js_topic = control_topics.get('joint_states_fast')
            if not js_topic:
                ns = get_namespace_from_config()
                js_topic = f'/{ns}/joint_states' if ns else '/joint_states'
        js_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        self.joint_states_sub = self.create_subscription(JointState, js_topic, self.joint_state_cb, js_qos)

        latest_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.latest_arm_states = {side: None for side in self.active_sides}   # (msg, receipt time)
        self.arm_state_subs = {}
        for side in self.active_sides:
            prefix = f'{side}_' if pose_side == 'both' else ''
            self.arm_state_subs[side] = self.create_subscription(
                HumanArmState, format_topic(str(human_cfg['state_topic']), prefix),
                partial(self.arm_state_cb, side=side), latest_qos)
        # Task reference over the horizon: the MPC does not know which task it tracks
        self.task_trajectory_sub = self.create_subscription(
            MultiDOFJointTrajectory, param('task_trajectory_topic'), self.task_trajectory_cb, latest_qos)
        # With a CBF downstream its output, not our u, is the q̈ the arm gets ('' = always ours)
        self.qddot_safe_topic = param('qddot_safe_topic')
        if self.qddot_safe_topic == AUTO_SENTINEL:
            self.qddot_safe_topic = control_topics.get('qddot_safe', '')
        if self.qddot_safe_topic:
            self.qddot_safe_sub = self.create_subscription(
                Float64MultiArray, self.qddot_safe_topic, self.qddot_safe_cb, js_qos)

        # ── Publishers ───────────────────────────────────────────────────────
        qddot_topic = param('qddot_nom_topic')
        if qddot_topic == AUTO_SENTINEL:
            qddot_topic = control_topics.get('qddot_nom', '/NS_1/qddot_nom')
        self.pub = self.create_publisher(Float64MultiArray, qddot_topic, 10)
        self.sp_pub = self.create_publisher(JointState, param('q_des_topic'), 10)
        self.plan_pub = self.create_publisher(MultiDOFJointTrajectory, param('plan_topic'), 1)
        self.diag_pub = self.create_publisher(MpcDiagnostics, param('diagnostics_topic'), 10)
        self.qddot_msg = Float64MultiArray(data=[0.0] * NUM_JOINTS)
        self.sp_msg = JointState(name=list(FR3_JOINT_NAMES))
        self.plan_msg = make_ee_trajectory_msg(self.ee_frame, self.problem.n_steps, self.dt)

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(
            f'MPC ready: N={self.problem.n_steps} x dt={self.dt:.3f} s, solver {self.settings.solver}, '
            f'human {self.human_mode if self.human_enabled else "off"}, joint states {js_topic}, '
            f'q̈ -> {qddot_topic}')


    # ── Callbacks ──
    def joint_state_cb(self, msg: JointState):
        self.latest_joint_state = (msg, self.get_clock().now())

    def arm_state_cb(self, msg: HumanArmState, side: str):
        self.latest_arm_states[side] = (msg, self.get_clock().now())

    def task_trajectory_cb(self, msg: MultiDOFJointTrajectory):
        self.latest_task_trajectory = (msg, self.get_clock().now())

    def qddot_safe_cb(self, msg: Float64MultiArray):
        self.latest_qddot_safe = (msg, self.get_clock().now())

    def request_stop(self):
        """Ctrl-C: brake for 0.5 s, then let run_node_main exit."""
        if self.stop_end is None:
            self.stop_end = self.get_clock().now() + Duration(seconds=0.5)


    # ── Helpers of the control loop ──
    def kinematics(self, q):
        """At the arm configuration q: (p_tcp, R_tcp, J_tcp 6x7, control points).

        J_tcp is LOCAL_WORLD_ALIGNED (rows v then ω, in base axes), as the commander's.
        Each control point has 'position', 'radius', 'name' and its 3x7 'jacobian'.
        """
        data = self.kin.data
        self.q_full[self.arm_q_idx] = q
        self.kin.update(self.q_full, np.zeros(self.model.nv), with_jdot=False)
        tcp = data.oMf[self.ee_frame_id]
        J_tcp = pin.getFrameJacobian(self.model, data, self.ee_frame_id,
                                     pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:, self.arm_v_idx]
        transforms = {name: (data.oMf[fid].rotation.copy(), data.oMf[fid].translation.copy())
                      for name, fid in self.link_frame_ids.items()}
        control_points = define_control_points(transforms, self.robot_cfg, self.distance_cfg)
        for cp in control_points:
            frame_id = self.link_frame_ids[cp['source_capsule']]
            cp['jacobian'] = self.kin.point_jacobian_pos(frame_id, cp['position'])[:, self.arm_v_idx]
        return tcp.translation.copy(), tcp.rotation.copy(), J_tcp, control_points

    def tcp_position(self, q):
        """TCP position only (forward kinematics, no Jacobians)."""
        self.q_full[self.arm_q_idx] = q
        pin.framesForwardKinematics(self.model, self.kin.data, self.q_full)
        return self.kin.data.oMf[self.ee_frame_id].translation.copy()

    def sample_task_reference(self, msg, t_query):
        """The task preview at the absolute times t_query: (p (n, 3), R (n, 3, 3)).

        Linear interpolation of the positions, slerp of the orientations; before the
        first and after the last preview point the ends are held.
        """
        times, positions, quats = ee_trajectory_arrays(msg)
        times = times + Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        t = np.clip(t_query, times[0], times[-1])
        p = np.column_stack([np.interp(t, times, positions[:, axis]) for axis in range(3)])
        if len(times) < 2:
            return p, np.repeat(Rotation.from_quat(quats[:1]).as_matrix(), len(t), axis=0)
        return p, Slerp(times, Rotation.from_quat(quats))(t).as_matrix()

    def human_arms(self, now):
        """The tracked arms usable this cycle, each a dict of numpy arrays:

        p, v (4, 3) keypoints and velocities (deadband applied), valid (4,),
        P_pp, P_pv, P_vv (4, 3, 3) Kalman covariance blocks, and latency [s]: how old
        the image is, where the forecast starts. A side not refreshed for
        max_state_age_s, or without valid keypoints, is left out.
        """
        arms = []
        for side in self.active_sides:
            if self.latest_arm_states[side] is None:
                continue
            msg, received = self.latest_arm_states[side]
            if (now - received).nanoseconds * 1e-9 > self.max_state_age_s or not any(msg.keypoint_valid):
                continue
            p, v, valid = extract_human_keypoints(msg)
            P_pp, P_vv, P_pv, _ = extract_human_covariances(msg)
            # Below the deadband the arm is still: Kalman noise must not push the forecast
            speed = np.linalg.norm(v, axis=1, keepdims=True)
            v = v * np.clip(1.0 - self.velocity_deadband / np.maximum(speed, 1e-9), 0.0, None)
            latency = (now - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
            if not 0.0 <= latency <= MAX_IMAGE_LATENCY_S:
                latency = (now - received).nanoseconds * 1e-9 + self.t_blind
            arms.append({'p': p, 'v': v, 'valid': valid, 'P_pp': P_pp, 'P_pv': P_pv, 'P_vv': P_vv,
                         'latency': latency})
        return arms


    # ── Control loop ──
    def control_loop(self):
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9
        problem, cfg, N = self.problem, self.settings, self.problem.n_steps
        u = None          # q̈ to publish this cycle; None = brake
        planned = False   # True when u comes from a plan (then the plan is published too)

        # ── 1. Arm state: q, q̇ of the 7 arm joints by name (the message has the fingers too),
        # how old it is, and the q̈ in force on them: the shield's last output while fresh (a CBF
        # downstream), else our last u
        q = qd = None
        state_age = 0.0
        if self.latest_joint_state is not None:
            msg, received = self.latest_joint_state
            index = {name: i for i, name in enumerate(msg.name)}
            state_age = (now - received).nanoseconds * 1e-9
            if (state_age <= self.joint_state_timeout_s
                    and all(name in index for name in FR3_JOINT_NAMES)
                    and len(msg.velocity) == len(msg.position)):
                idx = [index[name] for name in FR3_JOINT_NAMES]
                q, qd = np.array(msg.position)[idx], np.array(msg.velocity)[idx]
                self.last_qdot = qd
        u_in_force = self.u_prev
        if self.latest_qddot_safe is not None:
            msg, received = self.latest_qddot_safe
            if (now - received).nanoseconds * 1e-9 <= self.dt and len(msg.data) == NUM_JOINTS:
                u_in_force = np.array(msg.data)

        # ── 2. Task reference at t0 + k·dt, k = 1..N (none yet, stale, or another frame: None).
        # t0 = now + input_delay_s: when u_0 reaches the controller, so where the plan starts
        t0 = now_s + self.input_delay_s
        reference = None
        if self.latest_task_trajectory is not None:
            msg, received = self.latest_task_trajectory
            if (now - received).nanoseconds * 1e-9 <= self.task_timeout_s:
                if msg.joint_names[:1] != [self.ee_frame]:
                    self.get_logger().warn(f'The task preview moves {msg.joint_names}, not {self.ee_frame}',
                                           throttle_duration_sec=5.0)
                else:
                    reference = self.sample_task_reference(msg, t0 + self.dt * np.arange(1, N + 1))

        if self.stop_end is not None:
            self.done = now >= self.stop_end
        elif q is None:
            self.get_logger().warn('Joint state missing or stale: braking', throttle_duration_sec=2.0)
        elif reference is None:
            self.get_logger().info('Waiting for the task preview: holding (braking)', throttle_duration_sec=5.0)
        else:
            # ── 3. The state at t0, τ after the measured one, with the q̈ in force until then:
            #   q0 = q + τ·q̇ + ½τ²·u_in_force,   q̇0 = q̇ + τ·u_in_force
            # Real-time iteration: linearise around the previous plan, one step on.
            # lin[k-1] = (p_tcp, R_tcp, J_tcp 6×7, control points) at q̄_k
            tau = state_age + self.input_delay_s
            q0, qd0 = q + tau * qd + 0.5 * tau ** 2 * u_in_force, qd + tau * u_in_force
            U_bar = problem.shift_plan(self.U)
            q_bar, _ = problem.rollout(q0, qd0, U_bar)
            lin = [self.kinematics(q_k) for q_k in q_bar]
            problem.begin_cycle(q0, qd0, u_in_force)

            # ── 4. Task terms ‖C q_k − r‖²_W, Gauss-Newton around q̄_k (add_tracking_term):
            #   TCP position     C = J̄_v,  r = p_ref − p̄ + J̄_v q̄_k
            #   TCP orientation  e_R = log(R_ref R̄ᵀ)^∨ = −R_ref so3_log(R_refᵀ R̄),  C = −J̄_ω,  r = −e_R − J̄_ω q̄_k
            #   posture          C = I, r = q_home (the null space)
            p_ref, R_ref = reference
            W_task = np.diag([cfg.w_position] * 3 + [cfg.w_orientation] * 3)
            W_posture = cfg.w_posture * np.eye(NUM_JOINTS)
            for k in range(1, N + 1):
                p, R, J, _ = lin[k - 1]
                q_lin = q_bar[k - 1]
                e_R = -R_ref[k - 1] @ so3_log(R_ref[k - 1].T @ R)
                C = np.vstack((J[:3], -J[3:]))
                r = np.hstack((p_ref[k - 1] - p + J[:3] @ q_lin, -e_R - J[3:] @ q_lin))
                problem.add_tracking_term(k, C, r, W_task)
                problem.add_tracking_term(k, np.eye(NUM_JOINTS), self.q_home, W_posture)

            # ── 5. Human rows: at each step the arm where the forecast puts it, and the K
            # control-point / capsule pairs closest to it (set_human_rows):
            #   d̄ + a·(q_k − q̄_k) ≥ d_safe + δ + m,   a = n̂ᵀ J_c(q̄_k)
            # τ counts from the image: its latency, input_delay_s and k·dt. 'prediction' moves the keypoints
            # along their velocity up to prediction_cap_s, 'static' leaves them where they are;
            # both widen the margin m with the forecast uncertainty at τ:
            #   m = min(k_sigma·σ, margin_max),  σ² = n̂ᵀ(Σ_pp + τ(Σ_pv + Σ_pvᵀ) + τ²Σ_vv)n̂
            # with Σ = (1−α)²Σ_a + α²Σ_b between the capsule's two keypoints (as human_distance).
            arms = self.human_arms(now) if self.human_enabled else []
            for k in range(1, N + 1):
                capsules = []   # (capsule, arm, τ) of this step
                for arm in arms:
                    tau = arm['latency'] + self.input_delay_s + k * self.dt
                    shift = min(tau, self.prediction_cap_s) if self.human_mode == 'prediction' else 0.0
                    for capsule in self.human_geometry.build_capsules(arm['p'] + shift * arm['v'], arm['valid']):
                        capsules.append((capsule, arm, tau))
                pairs = []      # (d̄, a, m) per control point, against its closest capsule
                for cp in lin[k - 1][3]:
                    closest = None
                    for capsule, arm, tau in capsules:
                        d, axis_point, alpha = point_to_capsule_distance(cp['position'], capsule, cp['radius'])
                        if closest is None or d < closest[0]:
                            closest = (d, axis_point, alpha, capsule, arm, tau)
                    if closest is None or closest[0] >= self.d_activate:
                        continue
                    d, axis_point, alpha, capsule, arm, tau = closest
                    gap = cp['position'] - axis_point
                    if np.linalg.norm(gap) < 1e-9:
                        continue   # on the capsule axis: no normal (the CBF below still acts)
                    n_hat = gap / np.linalg.norm(gap)
                    i, j = capsule['indices']
                    blend = lambda P: (1.0 - alpha) ** 2 * P[i] + alpha ** 2 * P[j]
                    P_pv = blend(arm['P_pv'])
                    sigma2 = n_hat @ (blend(arm['P_pp']) + tau * (P_pv + P_pv.T) + tau ** 2 * blend(arm['P_vv'])) @ n_hat
                    margin = min(self.k_sigma * np.sqrt(max(sigma2, 0.0)), self.margin_max)
                    pairs.append((d, n_hat @ cp['jacobian'], margin))
                if pairs:
                    pairs = sorted(pairs, key=lambda pair: pair[0])[:problem.n_pairs]
                    problem.set_human_rows(k, [pair[1] for pair in pairs], [pair[0] for pair in pairs],
                                           [pair[2] for pair in pairs], q_bar[k - 1])

            # ── 6. Solve. A failed solve keeps running the previous plan, then brakes
            U = problem.solve(U_bar)
            fallback = U is None
            self.failures = 0 if U is not None else self.failures + 1
            self.U = U if U is not None else U_bar
            self.diag_pub.publish(mpc_diagnostics_msg(problem.diag, now.to_msg(), fallback, self.input_delay_s))
            self.diag_window.append(problem.diag)
            self.fallbacks += int(fallback)
            if self.summary_log.due(now_s):
                line = format_mpc_summary(self.diag_window, self.fallbacks)
                (self.summary_log.warn if self.fallbacks else self.summary_log.info)(line)
                self.diag_window, self.fallbacks = [], 0
            if self.failures <= self.max_failures:
                u, planned = problem.first_input(self.U), True
            else:
                self.get_logger().warn(f'{self.failures} failed solves in a row: braking',
                                       throttle_duration_sec=2.0)

        # ── 7. Command: the plan's first input, or the brake (q̈ = −k_b·q̇ on the last q̇)
        if u is None:
            u = braking_qddot(self.last_qdot, self.brake_gain, self.qddot_max)
            self.U[:] = 0.0   # the next plan starts from scratch
        self.qddot_msg.data = u.tolist()
        self.pub.publish(self.qddot_msg)
        self.u_prev = u

        # ── 8. The plan from t0: its TCP path (RViz, logs) and its next state on q_des_state
        if planned:
            q_plan, qd_plan = problem.rollout(q0, qd0, self.U)
            stamp = now.to_msg()
            fill_ee_trajectory_msg(self.plan_msg, stamp, np.array([self.tcp_position(q_k) for q_k in q_plan]))
            self.plan_pub.publish(self.plan_msg)
            self.sp_msg.header.stamp = stamp
            self.sp_msg.position, self.sp_msg.velocity = q_plan[0].tolist(), qd_plan[0].tolist()
            self.sp_msg.effort = u.tolist()
            self.sp_pub.publish(self.sp_msg)


def main(args=None):
    run_node_main(MPCCommander, args)

if __name__ == '__main__':
    main()