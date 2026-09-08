#!/usr/bin/env python3

from __future__ import annotations

import time
from typing import List, Optional

import numpy as np

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool

from franka_experiments.utils.constants import NUM_JOINTS, AUTO_SENTINEL
from franka_experiments.utils.cbf_utils import load_robot_config
from franka_experiments.utils.ros import run_node_main
from franka_experiments.utils.math_utils import (
    min_jerk, cosine_ramp, clamp_joints, lpf,
)
from franka_experiments.utils.kinematics import (
    generate_urdf_from_xacro,
    load_pinocchio_model,
    resolve_frame_id,
    resolve_arm_joint_ids,
    compute_ee_fk,
    compute_arm_jacobian,
    dls_solve,
    JointStateManager,
)
from franka_experiments.utils.logging_utils import ThrottledLogger, vec_to_str

# ── HARDCODED WAYPOINTS — edit these ────────────────────────────────────────
# Offsets [dx, dy, dz] in metres from the EE position at startup, base frame.
# Defaults: the six faces of a 10 cm cube around home, the last one shallower
# to stay away from the table.
WAYPOINT_OFFSETS_M: List[List[float]] = [
    [ 0.10,  0.00,  0.00],
    [ 0.00,  0.10,  0.00],
    [-0.10,  0.00,  0.00],
    [ 0.00, -0.10,  0.00],
    [ 0.00,  0.00,  0.10],
    [ 0.00,  0.00, -0.08],
]
MIN_RADIUS_M = 0.05
MAX_RADIUS_M = 0.15

# ── Tuning that does not change per run ─────────────────────────────────────
EE_FRAME = 'fr3_hand_tcp'
RATE_HZ = 200.0
WARMUP_S = 2.0          # zero output before moving
RAMP_S = 2.0            # cosine ramp-up of the output envelope
SETTLE_S = 0.5          # hold on arrival before touching the gripper
KP_CART = 2.0           # Cartesian position gain [1/s]
DAMPING = 0.02          # damped-least-squares lambda
LPF_ALPHA = 0.8         # low-pass on the velocity reference
GRIPPER_SERVICE = 'gripper_controller/set_gripper'
GRIPPER_WAIT_S = 5.0    # wait for the service at startup
GRIPPER_TIMEOUT_S = 10.0

# Same single source of truth qddot_to_torque and rt_torque_controller read.
try:
    QDDOT_TOPIC = load_robot_config('control')['topics']['qddot_safe']
except Exception:                       # config not installed yet
    QDDOT_TOPIC = '/NS_1/qddot_safe'


class WorkspaceGripperCycle(Node):
    """Waypoint cycle with a grasp/release at every waypoint."""

    def __init__(self):
        super().__init__('workspace_gripper_cycle')
        self.done = False
        self._stopping = False
        self._stop_end = 0.0

        self.declare_parameter('segment_duration_s', 3.0)
        self.declare_parameter('grip_hold_s', 2.0)
        self.declare_parameter('loop', False)
        self.declare_parameter('use_gripper', True)
        self.declare_parameter('qdot_max', 0.3)
        self.declare_parameter('qddot_max', 2.0)

        self.seg_s: float = self.get_parameter('segment_duration_s').value
        self.grip_s: float = self.get_parameter('grip_hold_s').value
        self.loop: bool = self.get_parameter('loop').value
        self.use_gripper: bool = self.get_parameter('use_gripper').value
        self.qdot_max: float = self.get_parameter('qdot_max').value
        self.qddot_max: float = self.get_parameter('qddot_max').value

        # ── Validate the hardcoded waypoints ───────────────────────────────
        self._offsets: List[np.ndarray] = []
        for i, off in enumerate(WAYPOINT_OFFSETS_M):
            vec = np.asarray(off, dtype=float)
            radius = float(np.linalg.norm(vec))
            if vec.size != 3 or not (MIN_RADIUS_M <= radius <= MAX_RADIUS_M):
                self.get_logger().error(
                    f'WAYPOINT_OFFSETS_M[{i}] = {off} is {radius:.3f} m from '
                    f'home — must be 3 values inside '
                    f'[{MIN_RADIUS_M}, {MAX_RADIUS_M}] m')
                raise SystemExit(1)
            self._offsets.append(vec)
        if not self._offsets:
            self.get_logger().error('WAYPOINT_OFFSETS_M is empty')
            raise SystemExit(1)

        # ── Kinematics ─────────────────────────────────────────────────────
        try:
            self.model, self.data = load_pinocchio_model(
                generate_urdf_from_xacro())
            self.ee_id = resolve_frame_id(self.model, EE_FRAME)
            self._joint_ids = resolve_arm_joint_ids(self.model)
        except Exception as exc:
            self.get_logger().error(f'Kinematics setup failed: {exc}')
            raise SystemExit(1) from exc
        self._js = JointStateManager(self, self.model, self._joint_ids,
                                     topic_param=AUTO_SENTINEL)

        # ── Gripper client ─────────────────────────────────────────────────
        self._grip_cli = None
        if self.use_gripper:
            cli = self.create_client(SetBool, GRIPPER_SERVICE)
            if cli.wait_for_service(timeout_sec=GRIPPER_WAIT_S):
                self._grip_cli = cli
            else:
                self.get_logger().warn(
                    f'{cli.srv_name} not available after {GRIPPER_WAIT_S} s — '
                    f'running the motion cycle WITHOUT the gripper. '
                    f'Is gripper_controller active?')
                self.use_gripper = False

        # ── Cycle state (waypoints resolved on the first active tick) ──────
        self._waypoints: Optional[List[np.ndarray]] = None
        self._idx = 0
        self._phase = 'move'
        self._phase_t0 = 0.0
        self._seg_from: Optional[np.ndarray] = None
        self._started = False
        self._grip_future = None
        self._grip_deadline = 0.0

        self._dt = 1.0 / RATE_HZ
        self._qdot_lpf = np.zeros(NUM_JOINTS)   # LPF state
        self._qdot_ref = np.zeros(NUM_JOINTS)   # integrator the controller mirrors
        self._zero = np.zeros(NUM_JOINTS)
        self._tlog = ThrottledLogger(self.get_logger())

        self.pub = self.create_publisher(Float64MultiArray, QDDOT_TOPIC, 10)
        self.timer = self.create_timer(self._dt, self._tick)
        self.t0 = self.get_clock().now()

        self.get_logger().info(
            f'workspace_gripper_cycle: qddot → {QDDOT_TOPIC}  '
            f'{len(self._offsets)} waypoints in '
            f'[{MIN_RADIUS_M}, {MAX_RADIUS_M}] m  '
            f'segment={self.seg_s}s  grip_hold={self.grip_s}s  '
            f'loop={self.loop}  '
            f'gripper={"on" if self.use_gripper else "off"}  '
            f'qdot_max={self.qdot_max} rad/s  qddot_max={self.qddot_max} rad/s^2')

    # ── Shutdown ───────────────────────────────────────────────────────────
    def request_stop(self, stop_duration_s: float = 0.5):
        """Brake to zero for *stop_duration_s*, then let main() exit."""
        if not self._stopping:
            self._stopping = True
            self._stop_end = time.monotonic() + stop_duration_s
            self.get_logger().info('Stopping: braking to zero')

    # ── Output ─────────────────────────────────────────────────────────────
    def _publish_qddot(self, qdot_ref: np.ndarray) -> None:
        """Publish the acceleration that moves the reference to *qdot_ref*."""
        qddot = clamp_joints((qdot_ref - self._qdot_ref) / self._dt,
                             self.qddot_max)
        self._qdot_ref = self._qdot_ref + qddot * self._dt   # rewind on clamp
        msg = Float64MultiArray()
        msg.data = qddot.tolist()
        self.pub.publish(msg)

    def _track(self, p_d, v_d, p_ee, J, envelope) -> np.ndarray:
        """Cartesian PD → joint velocity → published acceleration."""
        v_cmd = v_d + KP_CART * (p_d - p_ee)
        qdot = dls_solve(J, v_cmd, DAMPING)
        if qdot is None:
            qdot = dls_solve(J, 0.5 * v_cmd, DAMPING, damping_boost=0.1)
        if qdot is None:
            self.get_logger().warn(
                f'Singularity near waypoint [{self._idx}] — braking')
            self._qdot_lpf = np.zeros(NUM_JOINTS)
            self._publish_qddot(self._zero)
            return self._zero
        qdot = lpf(self._qdot_lpf, clamp_joints(envelope * qdot, self.qdot_max),
                   LPF_ALPHA)
        self._qdot_lpf = qdot.copy()
        self._publish_qddot(qdot)
        return qdot

    # ── Gripper ────────────────────────────────────────────────────────────
    def _send_gripper(self, close: bool) -> None:
        if self._grip_cli is None or not self._grip_cli.service_is_ready():
            self.get_logger().warn('Gripper service not ready — skipping')
            return
        request = SetBool.Request()
        request.data = close
        self._grip_future = self._grip_cli.call_async(request)
        self._grip_deadline = time.monotonic() + GRIPPER_TIMEOUT_S
        self.get_logger().info(
            f'Gripper: {"CLOSE" if close else "OPEN"} at waypoint [{self._idx}]')

    def _gripper_settled(self) -> bool:
        """True once the pending request answered, timed out, or never went."""
        if self._grip_future is None:
            return True
        if self._grip_future.done():
            try:
                response = self._grip_future.result()
                if response is not None and not response.success:
                    self.get_logger().warn(f'Gripper refused: {response.message}')
            except Exception as exc:
                self.get_logger().warn(f'Gripper call failed: {exc}')
        elif time.monotonic() < self._grip_deadline:
            return False
        else:
            self.get_logger().warn('Gripper did not answer — continuing')
        self._grip_future = None
        return True

    # ── Control loop ───────────────────────────────────────────────────────
    def _tick(self):
        if self._stopping:
            self._publish_qddot(self._zero)
            if time.monotonic() >= self._stop_end:
                self.timer.cancel()
                self._js.cancel_discovery()
                self.done = True
            return

        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        if t < WARMUP_S:
            self._publish_qddot(self._zero)
            return

        tr = t - WARMUP_S
        envelope = cosine_ramp(tr, RAMP_S)

        if self._js.q_full is None:
            self._publish_qddot(self._zero)
            if self._tlog.due(t):
                self.get_logger().warn('No joint state yet — braking')
            return
        age = (self.get_clock().now() - self._js.stamp).nanoseconds * 1e-9
        if age > 0.1:
            self._publish_qddot(self._zero)
            if self._tlog.due(t):
                self.get_logger().warn(f'Joint state stale ({age:.3f} s) — braking')
            return

        q_full = self._js.q_full.copy()
        p_ee = compute_ee_fk(self.model, self.data, q_full, self.ee_id).translation
        J = compute_arm_jacobian(self.model, self.data, q_full,
                                 self.ee_id, self._joint_ids)[:3, :]

        # First active tick: home is here, resolve the absolute waypoints.
        if not self._started:
            self._started = True
            self._waypoints = [p_ee + off for off in self._offsets]
            self._phase_t0 = tr
            self._seg_from = p_ee.copy()
            self.get_logger().info(
                f'Home EE [{vec_to_str(p_ee)}] → waypoint [0/'
                f'{len(self._waypoints) - 1}] '
                f'[{vec_to_str(self._waypoints[0])}]')

        target = self._waypoints[min(self._idx, len(self._waypoints) - 1)]
        elapsed = tr - self._phase_t0

        # move: min-jerk from the segment start to the target
        if self._phase == 'move':
            tau = min(elapsed / self.seg_s, 1.0) if self.seg_s > 0 else 1.0
            s, sdot = min_jerk(tau)
            p_d = self._seg_from + s * (target - self._seg_from)
            v_d = (sdot / self.seg_s) * (target - self._seg_from)
            qdot = self._track(p_d, v_d, p_ee, J, envelope)
            self._log(t, p_ee, p_d, qdot)
            if tau >= 1.0:
                self._phase, self._phase_t0 = 'settle', tr
            return

        # every phase below holds the arm on the waypoint
        qdot = self._track(target, np.zeros(3), p_ee, J, envelope)
        self._log(t, p_ee, target, qdot)

        if self._phase == 'done':
            return

        if self._phase == 'settle' and elapsed >= SETTLE_S:
            if self.use_gripper:
                self._send_gripper(close=True)
                self._phase, self._phase_t0 = 'close', tr
            else:
                self._next_waypoint(tr, p_ee)

        elif self._phase == 'close':
            if self._gripper_settled() and elapsed >= self.grip_s:
                self._send_gripper(close=False)
                self._phase, self._phase_t0 = 'open', tr

        elif self._phase == 'open':
            if self._gripper_settled() and elapsed >= self.grip_s:
                self._next_waypoint(tr, p_ee)

    def _next_waypoint(self, tr: float, p_ee: np.ndarray):
        self._idx += 1
        if self._idx >= len(self._waypoints):
            if not self.loop:
                self.get_logger().info('All waypoints visited — cycle complete')
                self._idx = len(self._waypoints) - 1
                self._phase = 'done'
                return
            self._idx = 0
        self._phase, self._phase_t0 = 'move', tr
        self._seg_from = p_ee.copy()
        self.get_logger().info(
            f'Moving to waypoint [{self._idx}/{len(self._waypoints) - 1}] '
            f'[{vec_to_str(self._waypoints[self._idx])}]')

    def _log(self, t, p_ee, p_d, qdot):
        if self._tlog.due(t):
            self._tlog.info(
                f'[t={t:.1f}s {self._phase} wp={self._idx}] '
                f'p=[{vec_to_str(p_ee)}] p_d=[{vec_to_str(p_d)}] '
                f'|e|={np.linalg.norm(p_d - p_ee):.4f} '
                f'|qdot|={np.linalg.norm(qdot):.4f}')


def main(args=None):
    run_node_main(WorkspaceGripperCycle, args=args)


if __name__ == '__main__':
    main()
