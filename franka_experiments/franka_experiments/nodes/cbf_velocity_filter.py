#!/usr/bin/env python3
"""Velocity-space CBF safety filter — Zeroing CBF (ZCBF), relative degree 1.

Implements a kinematic-level Control Barrier Function filter (Ferraguti et al.,
RAM 2022, eq. 12).  The barrier h = d - d_safe has relative degree 1 w.r.t. the
velocity input q̇: ḣ = n^T·J_p·q̇.  The QP solves directly for q̇_safe:

    min  ½‖q̇ - q̇_nom‖²  +  ½ρ·s²
    s.t. (n_i^T·J_p,i)·q̇ + s  ≥  -γ_i·h_i    ∀ active link i   (s ≥ 0)
         q̇_min ≤ q̇ ≤ q̇_max

Hardware continuity
-------------------
The QP is solved at every timer tick (timer-driven, same pattern as
cbf_safety_filter).  After the QP, a per-joint acceleration clamp limits how
much the published velocity can change between consecutive 100 Hz ticks:

    |q̇_cmd[k] - q̇_cmd[k-1]| / dt  ≤  q̈_max_j  (per joint)

This keeps the velocity profile smooth enough that the Franka joint motion
generator never sees an "acceleration discontinuity" reflex.

Hysteresis on activation
------------------------
A per-link active/inactive flag with a hysteresis band prevents rapid
constraint toggling when the measured distance oscillates near the threshold:
  - Link activates   when  d < d_safe + margin
  - Link deactivates when  d > d_safe + margin + hysteresis

Pipeline position:
  ee_pentagon_velocity_commander  →  /NS_1/tracking_qdot  (qdot_nom)
  cbf_velocity_filter             →  /NS_1/qdot_cmd       (qdot_safe)
  rt_velocity_executor_controller →  hardware

Parameters:
  bypass_cbf (bool, default False)
    When true, qdot_nom is passed through to qdot_cmd without solving the QP.
    Use for Phase 1 testing (no camera, no distances required).
"""

# TODO[LEGACY]: velocity-space control mode; the stack is torque/acceleration-based | confidence: high | superseded-by: nodes/cbf_safety_filter.py | flagged: 2026-09-01

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
        self.q_min    = np.array([self.limits[j][0] for j in _joint_order], dtype=np.float64)
        self.q_max    = np.array([self.limits[j][1] for j in _joint_order], dtype=np.float64)
        self.qdot_max = np.array([self.limits[j][2] for j in _joint_order], dtype=np.float64)
        self.qdot_min = -self.qdot_max

        # ── Per-joint acceleration clamp for velocity-command continuity ──
        #
        # The Franka joint motion generator checks JERK (= Δacceleration / dt_hw)
        # in addition to acceleration.  With first-order C++ interpolation, when
        # the commanded velocity changes direction (e.g. CBF activates then
        # deactivates), the interpolation slope reverses and the instantaneous
        # jerk seen by the motion generator is:
        #
        #   jerk_worst_case = 2 × qddot_cmd / dt_hw
        #
        # To keep this below the FR3 hardware jerk limit (from libfranka):
        #
        #   qddot_cmd ≤ jerk_limit_j × dt_hw / 2 × JERK_SAFETY
        #
        # FR3 jerk limits (rad/s³): [7500, 3750, 5000, 6250, 7500, 10000, 10000]
        # (joint1 … joint7, from libfranka Robot::Limits)
        _FR3_JERK_LIMITS = np.array(
            [7500., 3750., 5000., 6250., 7500., 10000., 10000.], dtype=np.float64)
        _DT_HW      = 0.001    # 1 kHz Franka real-time loop
        _JERK_SAFETY = 0.80   # 80 % margin below the hardware jerk limit

        # qddot_from_jerk = [3.0, 1.5, 2.0, 2.5, 3.0, 4.0, 4.0] rad/s²
        _qddot_from_jerk = _FR3_JERK_LIMITS * _DT_HW * _JERK_SAFETY / 2.0
        _qddot_from_cfg  = np.array(
            [self.limits[j][3] for j in _joint_order], dtype=np.float64)

        # Final limit: tightest of config value and jerk-derived bound
        self.qddot_max = np.minimum(_qddot_from_cfg, _qddot_from_jerk)

        self._dist_timeout = float(self.params.get('distance_timeout', 0.5))
        _rate_hz           = float(self.params.get('control_rate_hz', 100.0))
        self._dt           = 1.0 / _rate_hz

        # ── State ─────────────────────────────────────────────────────────
        self.q                    = None
        self.qdot                 = None
        self.qdot_nom             = np.zeros(self.model.nv, dtype=np.float64)
        # Last published velocity — used by the per-joint acceleration clamp.
        self._qdot_prev           = np.zeros(self.model.nv, dtype=np.float64)
        # Per-link hysteresis state: links currently in "constraint active" state.
        self._active_links        : set[str] = set()
        self.multi_distances      = []
        self._last_distance_stamp : float    = 0.0   # time.monotonic()

        # ── Diagnostic state ──────────────────────────────────────────────
        # Used to compute per-tick acceleration and hardware-level jerk estimates.
        self._qdot_initialized    : bool               = False  # one-time init flag
        self._diag_last_tick_t    : float              = 0.0
        self._diag_accel_prev     : np.ndarray         = np.zeros(self.model.nv)
        self._diag_first_tick     : bool               = True   # skip jerk check tick 0
        self._diag_prev_constrained : bool             = False
        # FR3 jerk limits [rad/s³] — for warn-threshold logging only
        self._FR3_JERK_LIMITS     = np.array(
            [7500., 3750., 5000., 6250., 7500., 10000., 10000.], dtype=np.float64)
        self._JERK_WARN_FRAC      = 0.75   # warn when jerk > 75 % of hw limit

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
        self.create_timer(self._dt, self._control_loop)

        mode_str = 'BYPASS (passthrough)' if self._bypass_cbf else 'CBF QP active'
        self.get_logger().info(
            f'CBF velocity filter ready — mode: {mode_str}\n'
            f'  rate            : {_rate_hz:.0f} Hz\n'
            f'  qdot_nom topic  : {self.topics_ctr["qdot_nom"]}\n'
            f'  qdot_cmd topic  : {self.topics_ctr["velocity_topic"]}\n'
            f'  distance topic  : {self.topics_ctr["per_link_distances"]}\n'
            f'  qddot_max (85%) : {np.round(self.qddot_max, 3).tolist()}'
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
        """Store per-link distances (QP is solved in the timer callback)."""
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
        """Build ZCBF constraints with hysteresis on the activation threshold.

        Hysteresis prevents rapid constraint toggling when the measured distance
        oscillates near the activation boundary (d_safe + margin), which would
        otherwise cause the QP target to oscillate and produce jerky motion.

        Activation   : d < d_safe + act_margin
        Deactivation : d > d_safe + act_margin + hysteresis

        For each active link i:
            h_i   = d_i - d_safe
            a_vel = n_i^T · J_p,i      (nv,)
            b_vel = -gamma_i · h_i
        """
        constraints = []
        if not multi_distances:
            return constraints

        act_margin = float(self.params.get('cbf_activation_margin', 0.10))
        hysteresis = float(self.params.get('cbf_hysteresis',         0.05))
        d_safe     = float(self.params['d_safe'])

        act_thresh   = d_safe + act_margin               # activate   below this
        deact_thresh = d_safe + act_margin + hysteresis  # deactivate above this

        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        # Build set of links present in this message to clean up stale active links
        measured_links = {item['link_name'] for item in multi_distances}
        # Links no longer measured are removed from active set (camera lost sight)
        self._active_links &= measured_links

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

            # ── Hysteresis gate ───────────────────────────────────────────
            if link_name in self._active_links:
                # Already active: deactivate only when well past the threshold
                if d > deact_thresh:
                    self._active_links.discard(link_name)
                    self.get_logger().warn(
                        f'[CBF-VEL] DEACTIVATE {link_name}  '
                        f'd={d:.4f} m > deact_thresh={deact_thresh:.4f} m  '
                        f'h={d - d_safe:.4f} m'
                    )
                    continue
            else:
                # Not active: activate when inside the activation zone
                if d >= act_thresh:
                    continue
                self._active_links.add(link_name)
                self.get_logger().warn(
                    f'[CBF-VEL] ACTIVATE {link_name}  '
                    f'd={d:.4f} m < act_thresh={act_thresh:.4f} m  '
                    f'h={d - d_safe:.4f} m'
                )

            h = d - d_safe

            if h < 0.0:
                self.get_logger().warn(
                    f'[CBF-VEL] h < 0 on {link_name}: h={h:.4f} m — '
                    'robot inside safe set, CBF cannot guarantee recovery',
                    throttle_duration_sec=1.0,
                )

            gamma = float(select_gamma(
                item['zone'], item['confidence'],
                d=float(d), d_safe=float(d_safe)))

            Jp = self._compute_point_jacobian(q, link_name, p_world)
            if Jp is None:
                continue

            # ZCBF: a·q̇_cmd ≥ -γ·h
            a_vel = (n_world @ Jp).astype(np.float64)
            b_vel = float(-gamma * h)

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

        Returns q̇_safe (nv,).  On solver failure returns a soft fallback.
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

        x     = np.asarray(x, dtype=np.float64).reshape(-1)
        slack = float(x[-1])
        if slack > 0.1:
            min_d = min((d for _, _, _, d in constraints), default=float('nan'))
            self.get_logger().error(
                f'[CBF-VEL] large slack s={slack:.4f} m/s  n_c={len(constraints)}'
                f'  min_d={min_d:.3f} m — CBF constraint violated',
                throttle_duration_sec=0.5,
            )
        return x[:nv]

    # ── Main control step (timer-driven at control_rate_hz) ───────────────────

    def _control_loop(self):
        """Solve CBF QP and publish safe velocity at every timer tick."""
        if self.q is None or self.qdot is None:
            return

        now = time.monotonic()

        # ── Timer jitter diagnostic ────────────────────────────────────────
        if self._diag_last_tick_t > 0.0:
            dt_actual = now - self._diag_last_tick_t
            dt_err    = dt_actual - self._dt
            if abs(dt_err) > 0.003:   # warn if > 3 ms late/early
                self.get_logger().warn(
                    f'[CBF-DIAG] timer jitter: dt_actual={dt_actual*1e3:.1f} ms '
                    f'(nominal={self._dt*1e3:.1f} ms  err={dt_err*1e3:+.1f} ms)',
                    throttle_duration_sec=0.5,
                )
        self._diag_last_tick_t = now

        # Seed _qdot_prev from actual joint velocity on the very first tick to
        # avoid a step from zero on start-up or after a mode switch.
        # NOTE: we use a dedicated flag, NOT `not np.any(self._qdot_prev)`,
        # because the latter re-triggers every time qdot_cmd reaches exactly zero
        # (e.g. at a motion endpoint), causing a sudden reset of the continuity
        # reference and an artificial jerk spike.
        if not self._qdot_initialized:
            self._qdot_prev = self.qdot.copy()
            self._qdot_initialized = True
            self.get_logger().info(
                f'[CBF-DIAG] _qdot_prev initialized from qdot: '
                f'{np.round(self.qdot, 4).tolist()}'
            )

        # ── Compute target velocity ────────────────────────────────────────
        is_constrained = False

        if self._bypass_cbf:
            target = np.clip(self.qdot_nom, self.qdot_min, self.qdot_max)

        else:
            age = time.monotonic() - self._last_distance_stamp
            if age > self._dist_timeout:
                self.get_logger().warn(
                    f'[CBF-VEL] distance stale ({age:.3f} s > {self._dist_timeout:.3f} s)'
                    ' — passthrough',
                    throttle_duration_sec=2.0,
                )
                target = np.clip(self.qdot_nom, self.qdot_min, self.qdot_max)
            else:
                q    = self.q.copy()
                qdot = self.qdot.copy()

                constraints = self._build_cbf_constraints(
                    q, qdot, list(self.multi_distances))

                is_constrained = len(constraints) > 0

                # Log constraint ON/OFF transitions immediately (no throttle)
                if is_constrained and not self._diag_prev_constrained:
                    min_d = min(d for _, _, _, d in constraints)
                    self.get_logger().warn(
                        f'[CBF-DIAG] >>> CONSTRAINT ON  '
                        f'links={[n for _,_,n,_ in constraints]}  '
                        f'min_d={min_d:.4f} m  '
                        f'qdot_nom={np.round(self.qdot_nom,3).tolist()}  '
                        f'qdot_prev={np.round(self._qdot_prev,3).tolist()}'
                    )
                elif not is_constrained and self._diag_prev_constrained:
                    self.get_logger().warn(
                        f'[CBF-DIAG] <<< CONSTRAINT OFF  '
                        f'qdot_prev={np.round(self._qdot_prev,3).tolist()}  '
                        f'qdot_nom={np.round(self.qdot_nom,3).tolist()}'
                    )
                self._diag_prev_constrained = is_constrained

                if is_constrained:
                    min_d = min(d for _, _, _, d in constraints)
                    self.get_logger().info(
                        f'[CBF-VEL] active n={len(constraints)}  min_d={min_d:.3f} m',
                        throttle_duration_sec=1.0,
                    )
                else:
                    self.get_logger().info(
                        '[CBF-VEL] no constraints — nominal',
                        throttle_duration_sec=2.0,
                    )

                qdot_qp = self._solve_cbf_qp(self.qdot_nom.copy(), constraints)
                target  = np.clip(qdot_qp, self.qdot_min, self.qdot_max)

                if is_constrained:
                    # Log how much the QP modified the nominal velocity
                    delta_qp = qdot_qp - self.qdot_nom
                    worst_j  = int(np.argmax(np.abs(delta_qp)))
                    self.get_logger().info(
                        f'[CBF-DIAG] QP  nom={np.round(self.qdot_nom,3).tolist()}  '
                        f'safe={np.round(qdot_qp,3).tolist()}  '
                        f'max_mod=j{worst_j+1}:{delta_qp[worst_j]:.3f} rad/s',
                        throttle_duration_sec=0.5,
                    )

        # ── Per-joint acceleration clamp ───────────────────────────────────
        delta_max = self.qddot_max * self._dt
        raw_delta = target - self._qdot_prev
        clipped   = np.clip(raw_delta, -delta_max, delta_max)
        qdot_cmd  = self._qdot_prev + clipped

        # Log when the clamp actually saturates (target unreachable in one tick)
        clamp_active = np.any(np.abs(raw_delta) > delta_max + 1e-6)
        if clamp_active:
            saturated = np.where(np.abs(raw_delta) > delta_max + 1e-6)[0]
            msgs = [
                f'j{j+1}: want={raw_delta[j]:.4f} capped={clipped[j]:.4f} rad/s'
                for j in saturated
            ]
            self.get_logger().warn(
                f'[CBF-DIAG] CLAMP saturated — {", ".join(msgs)}',
                throttle_duration_sec=0.2,
            )

        # ── Hardware-level jerk estimate ───────────────────────────────────
        # accel_cmd ≈ velocity ramp slope seen by C++ interpolation [rad/s²]
        # jerk_hw   ≈ change in that slope at this sample boundary [rad/s³]
        #             = Δaccel / dt_hw  (worst-case, at direction change)
        # Compare against FR3 jerk limits to predict reflex triggers.
        accel_cmd = clipped / self._dt                     # (nv,) [rad/s²]
        if self._diag_first_tick:
            # Skip jerk check on tick 0: _diag_accel_prev is 0, so jerk would
            # be artificially high regardless of what the robot is actually doing.
            self._diag_first_tick = False
        else:
            jerk_hw   = np.abs(accel_cmd - self._diag_accel_prev) / 0.001  # [rad/s³]
            jerk_warn = self._FR3_JERK_LIMITS * self._JERK_WARN_FRAC
            over_jerk = np.where(jerk_hw > jerk_warn)[0]
            if over_jerk.size > 0:
                msgs = [
                    f'j{j+1}: jerk={jerk_hw[j]:.0f} > warn={jerk_warn[j]:.0f} '
                    f'(limit={self._FR3_JERK_LIMITS[j]:.0f}) rad/s³'
                    for j in over_jerk
                ]
                self.get_logger().error(
                    f'[CBF-DIAG] HIGH JERK (hw estimate) — {", ".join(msgs)}  '
                    f'accel_curr={np.round(accel_cmd,2).tolist()}  '
                    f'accel_prev={np.round(self._diag_accel_prev,2).tolist()}',
                    throttle_duration_sec=0.1,
                )
        self._diag_accel_prev = accel_cmd.copy()

        self._qdot_prev = qdot_cmd.copy()

        msg      = Float64MultiArray()
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
