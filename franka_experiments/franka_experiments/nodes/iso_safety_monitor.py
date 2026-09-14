#!/usr/bin/env python3
"""Independent SSM monitor and non-safety-rated stop (roadmap Step 6).

WHY A SECOND CHANNEL
--------------------
``cbf_safety_filter`` SHAPES the command: its speed rows are slack-relaxable, so
under enough barrier pressure the QP can and will buy its way past a cap. That
is the right behaviour for a filter — an infeasible QP is a worse answer than a
relaxed row — but it means the filter cannot also be the thing that ENFORCES the
separation-distance bound. ISO 10218-2:2025 (5.14.5) is explicit that below
``S_p`` the robot system must **stop** [R]; a speed reduction alone is not
enough.

So this node watches the same geometry from outside the QP, with its own
kinematics, its own clock and its own copy of the ISO constants, and latches a
stop when the bound is crossed. It publishes no command: the stop is carried out
by the filter (which brakes through ``box_only_solve``) and by the commander
(which zeroes ``q̈_nom`` and holds the phase), both of which subscribe to
``/NS_1/iso_safety``.

TERMINOLOGY — READ THIS BEFORE WRITING A LOG LINE
-------------------------------------------------
What this implements is a **non-safety-rated stop**. It is NOT a *protective
stop* in the sense of ISO 10218: that term is reserved for a rated function
(PL d / SIL 2 for a Class II robot's safety functions, ISO 13849-1:2023 /
IEC 62061:2021), and nothing in this chain reaches it — single-channel Python,
best-effort DDS, no diagnostic coverage, no proof test. The term does not appear
in this node's logs, in its topic names, or in its documentation, and it should
not be added.

LATCH vs AUTOMATIC RESUMPTION — [E]
-----------------------------------
SSM **permits** motion to resume automatically once ``S >= S_p`` again. The
latch (``iso_stop_requires_reset``) is a LOCAL CHOICE for supervised
experiments, consistent with the start/restart interlock and reset function of
ISO 10218-1:2025 (5.5.2) but not required by it. Setting
``iso_stop_requires_reset: false`` is equally standard-conformant and resumes on
its own the moment every trip condition clears.

TRIP CONDITIONS — all [E] in their thresholds, [R] in their intent
------------------------------------------------------------------
Each must hold for ``iso_monitor_ticks`` consecutive ticks, so one noisy depth
frame cannot stop the cell:

1. ``v_closing > ssm_speed_cap(d, v_app) + iso_speed_tol`` on any control point
   — the robot is closing faster than it could stop in the room it has.
2. ``d < C + Z_d + Z_r`` — inside the irreducible part of ``S_p``, where no
   positive approach speed is admissible at all.
3. ``cbf_status[2] == 1.0`` — the safety chain itself is faulted (perception
   stale, joint state frozen, QP fell back). A blind filter is not a filter.

Topics
------
Subscribes
    ``joint_states_fast``       JointState, RELIABLE depth 1
    ``per_link_distances``      MultiLinkDistance, BEST_EFFORT depth 1
    ``cbf_status``              Float64MultiArray, depth 1
Publishes
    ``iso_safety``              Float64MultiArray, ALWAYS, at ``qp_rate_hz``::

        [0] stop_latched    1.0 while the stop is active
        [1] trip_reason     0 none, 1 speed cap, 2 inside C+Z, 3 chain fault,
                            4 input stale (no distances / no joint state)
        [2] S_p             [m] separation distance demanded by the control
                            point with the SMALLEST MARGIN d - S_p. (The
                            roadmap calls this field S_p_min; it is the most
                            BINDING S_p in the scene, not the numerically
                            smallest one, which would be the least interesting
                            number in the frame.)
        [3] v_cap_min       [m/s] tightest SSM cap over the CPs
        [4] v_closing_max   [m/s] fastest closing speed over the CPs

    A silent monitor is a dead monitor: the message goes out every tick whether
    or not anything is wrong, so a consumer can tell "nothing to report" from
    "not running" by the message's own age.

Services
    ``safety_reset``            std_srvs/Trigger — clears the latch, and
                                REFUSES while any trip condition is still true.
"""

from __future__ import annotations

import numpy as np
import pinocchio as pin
import rclpy
from franka_msgs.msg import MultiLinkDistance
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from franka_experiments.utils.cbf_state_rows import FR3_JOINTS, NV
from franka_experiments.utils.config import load_cbf_config
from franka_experiments.utils.iso_ssm import protective_separation, ssm_speed_cap
from franka_experiments.utils.kinematics import CBFKinematics, build_urdf_no_hand
from franka_experiments.utils.perception_msgs import labelled_links

#: trip_reason values published on data[1].
REASON_NONE, REASON_SPEED, REASON_INSIDE, REASON_FAULT, REASON_STALE = 0, 1, 2, 3, 4
REASON_NAMES = {
    REASON_NONE: 'none',
    REASON_SPEED: 'closing faster than the SSM cap',
    REASON_INSIDE: 'inside C + Z_d + Z_r',
    REASON_FAULT: 'safety-chain fault on cbf_status',
    REASON_STALE: 'monitor inputs stale',
}


class ISOSafetyMonitor(Node):

    def __init__(self):
        super().__init__('iso_safety_monitor')

        # Same file, same spec, same failure mode as the filter: a key missing
        # from either side is a startup failure naming the key. The monitor
        # deliberately re-reads the constants rather than being handed them —
        # a second channel that takes its thresholds from the first is one
        # channel wearing two hats.
        topics, P = load_cbf_config(self)
        self.P = P
        self._floor = P.iso_c_intrusion + P.iso_z_depth + P.iso_z_robot
        self._reduced = str(P.iso_mode) == 'reduced'
        self._tcp_link = self.declare_parameter(
            'iso_tcp_link', 'fr3_link8').value

        self._kin = CBFKinematics(pin.buildModelFromUrdf(build_urdf_no_hand()))
        self._fid: dict = {}

        # ── State ───────────────────────────────────────────────────────
        self._q = self._qdot = None
        self._js_stamp = 0.0
        self._dist = None
        self._dist_stamp = 0.0
        self._fault = 0.0
        self._latched = False
        self._reason = REASON_NONE
        self._streak = {REASON_SPEED: 0, REASON_INSIDE: 0,
                        REASON_FAULT: 0, REASON_STALE: 0}
        self._live = REASON_NONE       # trip condition true RIGHT NOW, for reset
        self._diag = (0.0, float('inf'), 0.0)   # S_p, v_cap, v_closing

        grp_io = MutuallyExclusiveCallbackGroup()
        grp_tick = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            JointState,
            topics.get('joint_states_fast', topics['joint_states_topic']),
            self._on_js, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=grp_io)
        self.create_subscription(
            MultiLinkDistance, topics['per_link_distances'], self._on_dist,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=grp_io)
        self.create_subscription(
            Float64MultiArray, topics.get('cbf_status', '/NS_1/cbf_status'),
            self._on_status, QoSProfile(depth=1), callback_group=grp_io)

        self._pub = self.create_publisher(
            Float64MultiArray, topics.get('iso_safety', '/NS_1/iso_safety'), 10)
        self._msg = Float64MultiArray()
        self._msg.data = [0.0, 0.0, 0.0, float('inf'), 0.0]
        self.create_service(
            Trigger, topics.get('safety_reset', '/NS_1/safety_reset'),
            self._on_reset, callback_group=grp_io)

        self.create_timer(1.0 / P.qp_rate_hz, self._tick, callback_group=grp_tick)
        self.get_logger().info(
            f'iso_safety_monitor at {P.qp_rate_hz:.0f} Hz  mode={P.iso_mode}\n'
            f'  config: {P.config_path}\n'
            f'  C+Z_d+Z_r = {self._floor:.3f} m   T_r = {P.iso_t_reaction:.3f} s   '
            f'a_s = {P.iso_a_stop:.2f} m/s^2   v_h = {P.iso_v_human:.2f} m/s\n'
            f'  trip after {P.iso_monitor_ticks} ticks, tol '
            f'{P.iso_speed_tol:.3f} m/s, latch='
            f'{"ON (manual reset)" if P.iso_stop_requires_reset else "off (auto-resume)"}\n'
            f'  NON-SAFETY-RATED. Not a protective stop, not PL d, not certified '
            f'— see franka_experiments/SAFETY.md')

    # ── Inputs ───────────────────────────────────────────────────────────

    def _on_js(self, msg: JointState) -> None:
        n2p = dict(zip(msg.name, msg.position))
        n2v = dict(zip(msg.name, msg.velocity))
        try:
            self._q = np.array([n2p[n] for n in FR3_JOINTS])
            self._qdot = np.array([n2v[n] for n in FR3_JOINTS])
        except KeyError:
            return
        self._js_stamp = self._now()

    def _on_dist(self, msg: MultiLinkDistance) -> None:
        self._dist = msg
        self._dist_stamp = self._now()

    def _on_status(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 3:
            self._fault = float(msg.data[2])

    # ── The tick ─────────────────────────────────────────────────────────

    def _tick(self) -> None:
        P = self.P
        now = self._now()
        live = REASON_NONE
        s_p, v_cap_min, v_cls_max = 0.0, float('inf'), 0.0

        stale = (self._q is None or self._dist is None
                 or (now - self._js_stamp) > P.joint_state_timeout
                 or (now - self._dist_stamp) > P.distance_timeout)

        if stale:
            live = REASON_STALE
        else:
            s_p, v_cap_min, v_cls_max, live = self._evaluate()

        # The safety chain's own fault flag is a trip in its own right, and it
        # is checked even when this node's inputs are fine: the filter can be
        # blind while perception is still reaching the monitor.
        chain_fault = self._fault >= 1.0

        # ── Streaks: a trip needs iso_monitor_ticks consecutive ticks ───
        # One noisy depth frame must not stop the cell; a real incursion holds
        # for many frames. Each condition counts separately, so a scene that
        # alternates between two of them does not accumulate a trip out of two
        # single-tick events.
        for reason in (REASON_SPEED, REASON_INSIDE, REASON_STALE):
            self._streak[reason] = self._streak[reason] + 1 if live == reason else 0
        self._streak[REASON_FAULT] = self._streak[REASON_FAULT] + 1 if chain_fault else 0
        self._live = live if live != REASON_NONE else (
            REASON_FAULT if chain_fault else REASON_NONE)

        tripped = REASON_NONE
        for reason in (REASON_INSIDE, REASON_SPEED, REASON_FAULT, REASON_STALE):
            if self._streak[reason] >= P.iso_monitor_ticks:
                tripped = reason
                break

        if tripped != REASON_NONE and not self._latched:
            self._latched = True
            self._reason = tripped
            self.get_logger().error(
                f'ISO SSM STOP (non-safety-rated) — {REASON_NAMES[tripped]}. '
                f'S_p={s_p:.3f} m  v_cap_min={v_cap_min:.3f} m/s  '
                f'v_closing_max={v_cls_max:.3f} m/s  fault={self._fault:.0f}. '
                f'{"Call /NS_1/safety_reset to clear." if P.iso_stop_requires_reset else "Clears on its own once every condition is false."}')
        elif (self._latched and not P.iso_stop_requires_reset
                and self._live == REASON_NONE and tripped == REASON_NONE):
            # Automatic resumption: what SSM actually permits. Only reachable
            # with iso_stop_requires_reset false.
            self._latched = False
            self._reason = REASON_NONE
            self.get_logger().warn('ISO SSM stop cleared automatically '
                                   '(S >= S_p again, no latch configured)')

        self._diag = (s_p, v_cap_min, v_cls_max)
        self._msg.data = [1.0 if self._latched else 0.0,
                          float(self._reason if self._latched else tripped),
                          s_p, v_cap_min, v_cls_max]
        self._pub.publish(self._msg)

    def _evaluate(self):
        """Per control point: closing speed against its own SSM cap.

        Returns ``(S_p, v_cap_min, v_closing_max, live_reason)``.

        The published ``S_p`` is the separation distance demanded by the
        control point with the SMALLEST MARGIN ``d − S_p`` — the most binding
        one in the scene, not the numerically smallest ``S_p``. Read against
        that point's own ``d`` it says how close the cell is to the bound; read
        alone it says how much room the worst point wants. ``v_cap_min`` is the
        tightest cap and ``v_closing_max`` the fastest closing speed, over all
        control points.
        """
        P = self.P
        q, qdot = self._q, self._qdot
        self._kin.update(q, qdot, with_jdot=False)

        margin_min, s_p_worst = float('inf'), 0.0
        v_cap_min, v_cls_max = float('inf'), 0.0
        live = REASON_NONE
        for label, ld in labelled_links(self._dist):
            if not ld.valid:
                continue
            d = float(ld.distance)
            if not np.isfinite(d):
                continue
            if d < self._floor:
                # Inside the irreducible part of S_p: no positive approach speed
                # is admissible, so the cap is zero and the margin is negative
                # whatever the arm is doing. No Jacobian needed to know that.
                live = REASON_INSIDE
                v_cap_min = 0.0
                if d - self._floor < margin_min:
                    margin_min, s_p_worst = d - self._floor, self._floor
                continue

            fid = self._frame_id(ld.robot_link_name)
            if fid is None:
                continue
            n = np.array([ld.direction.x, ld.direction.y, ld.direction.z])
            n_norm = float(np.linalg.norm(n))
            if n_norm < 1e-9:
                continue
            n /= n_norm                       # n̂: obstacle -> control point
            p_r = np.array([ld.closest_point_robot.x, ld.closest_point_robot.y,
                            ld.closest_point_robot.z])
            Jp = self._kin.point_jacobian_pos(fid, p_r)
            v_cp = Jp @ qdot
            # +n̂ is AWAY from the obstacle, so the robot's closing speed is the
            # negative projection. Clamped at 0: a receding point is not closing.
            v_closing = max(-float(n @ v_cp), 0.0)
            # The obstacle's own closing speed, along the same normal and with
            # the same sign convention the barrier uses (v_obs = n̂ᵀ ṗ_obs).
            v_app = max(float(n @ np.array([ld.obstacle_velocity.x,
                                            ld.obstacle_velocity.y,
                                            ld.obstacle_velocity.z])), 0.0)

            cap = ssm_speed_cap(
                d, v_app, t_r=P.iso_t_reaction, a_s=P.iso_a_stop,
                c=P.iso_c_intrusion, z_d=P.iso_z_depth, z_r=P.iso_z_robot,
                v_max=P.link_speed_max)
            # Reduced speed (ISO 10218-1:2025 5.5.3 / -2:2025 5.5.6, 250 mm/s):
            # the SAME cap the filter's extra TCP row carries, applied here so
            # the two channels agree on what 'reduced' means. [S] value,
            # [E] cell-wide application.
            if self._reduced and ld.robot_link_name == self._tcp_link:
                cap = min(cap, P.iso_tcp_reduced_speed)

            s_p = protective_separation(
                v_closing, v_app, t_r=P.iso_t_reaction, a_s=P.iso_a_stop,
                c=P.iso_c_intrusion, z_d=P.iso_z_depth, z_r=P.iso_z_robot)
            if d - s_p < margin_min:
                margin_min, s_p_worst = d - s_p, s_p
            v_cap_min = min(v_cap_min, cap)
            v_cls_max = max(v_cls_max, v_closing)
            if v_closing > cap + P.iso_speed_tol and live == REASON_NONE:
                live = REASON_SPEED
                self.get_logger().warn(
                    f'SSM cap exceeded at {label}: closing {v_closing:.3f} m/s '
                    f'> cap {cap:.3f} + tol {P.iso_speed_tol:.3f} m/s '
                    f'(d={d:.3f} m, v_app={v_app:.3f} m/s)',
                    throttle_duration_sec=1.0)
        return s_p_worst, v_cap_min, v_cls_max, live

    def _frame_id(self, link: str):
        if link not in self._fid:
            self._fid[link] = self._kin.resolve_frame_id(link)
        return self._fid[link]

    # ── Reset ────────────────────────────────────────────────────────────

    def _on_reset(self, request, response):
        """Clear the latch — but only when nothing is currently tripping.

        A reset that clears a latch while the condition is still true is not a
        reset, it is a bypass with a service call in front of it. ISO
        10218-1:2025 (5.5.2) requires the reset to be a deliberate, separate
        action AND requires the hazardous condition to be gone first; only the
        second half is enforceable here, and it is.
        """
        if not self._latched:
            response.success = True
            response.message = 'no stop latched'
            return response
        if self._live != REASON_NONE:
            response.success = False
            response.message = (
                f'REFUSED: {REASON_NAMES[self._live]} is still true '
                f'(S_p={self._diag[0]:.3f} m, v_cap_min={self._diag[1]:.3f} '
                f'm/s, v_closing_max={self._diag[2]:.3f} m/s). Clear the '
                f'condition first — a reset is not a bypass.')
            self.get_logger().warn(response.message)
            return response
        self._latched = False
        self._reason = REASON_NONE
        for k in self._streak:
            self._streak[k] = 0
        response.success = True
        response.message = 'ISO SSM stop cleared'
        self.get_logger().warn('ISO SSM stop cleared by /safety_reset')
        return response

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = ISOSafetyMonitor()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy installs its own SIGINT handler and may already have shut the
        # context down by the time we get here; calling it twice raises.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
