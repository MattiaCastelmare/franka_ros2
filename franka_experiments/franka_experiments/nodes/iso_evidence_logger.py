#!/usr/bin/env python3
"""Record the evidence an ISO assessment needs — and nothing it cannot support.

WHAT THIS IS FOR
----------------
"Does the cell respect the ISO limits?" is not a question a log file can answer.
Conformity needs a risk assessment, rated (PL d / SIL 2) safety functions and
validation by a competent person. What a log file CAN answer is narrower and
still worth having:

* did any part of the robot ever move faster than the limits the configuration
  claims to respect (``iso_v_pfl``, ``iso_tcp_reduced_speed``, the joint box)?
* did any control point ever get closer than the separation distance its own
  speed demanded (``d < S_p``), or cross the irreducible ``C + Z_d + Z_r``?
* when a stop was commanded, how long did it take and how far did the arm go —
  i.e. is ``iso_a_stop`` the honest number or an optimistic one?
* was the data good enough for any of the above to mean anything?

This node writes that evidence. ``scripts/iso_evidence_report.py`` reads it back
and produces a verdict per check, with the ones that are structurally
unanswerable from logs marked as such rather than quietly omitted.

**IT EVALUATES THE ISO CRITERIA WHETHER OR NOT THE ISO LAYER IS ON.** That is
the point: with every ``iso_*`` flag false you still get to see which limits the
current system would and would not have satisfied. It is a passive observer — it
subscribes, computes and writes, and publishes nothing.

WHAT IT WRITES
--------------
One directory per run::

    <output_dir>/<stamp>_<run_name>/
        iso_manifest.json   the ENTIRE iso_* block, the limits, the joint
                            limits, topic names, git SHA, sampling rate.
                            Without it the CSV is uninterpretable: a number
                            means nothing without the threshold it was judged
                            against, and thresholds move between runs.
        iso_evidence.csv    one row per sample (default 100 Hz)
        iso_events.csv      sparse: stops, resets, faults, saturation onsets,
                            perception dropouts. A rate-sampled CSV misses a
                            30 ms event; this does not.

Send the whole directory. Any one of the three alone is not enough.

SAMPLING, STATED HONESTLY
-------------------------
Forward kinematics runs on the sample timer (``sample_rate_hz``, default 100 Hz,
matching the QP). Joint speed and Cartesian speed are additionally tracked as a
RUNNING MAXIMUM in the 1 kHz joint-state callback, so a short excursion between
two samples still reaches the CSV. The Cartesian part of that uses the Jacobian
cached at the last FK, which is exact in q̇ and one sample stale in q — the
approximation is in the geometry, not in the speed, and over 10 ms of arm motion
it is small. Everything a report says about speed carries that caveat, and
``iso_evidence_report.py`` prints it.

Anything shorter than the 1 kHz joint-state period is invisible to this node and
to every other Python node in the stack.
"""

from __future__ import annotations

import csv
import json
import os
import subprocess
import time
from datetime import datetime
from typing import Optional

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

from franka_experiments.utils.cbf_state_rows import (
    FR3_JOINT_KEYS, FR3_JOINTS, FR3_TCP_LINK, NV)
from franka_experiments.utils.config import (
    load_cbf_config, load_franka_joint_limits, resolve_log_dir)
from franka_experiments.utils.iso_ssm import protective_separation, ssm_speed_cap
from franka_experiments.utils.kinematics import CBFKinematics, build_urdf_no_hand
from franka_experiments.utils.params import declare_float, declare_str
from franka_experiments.utils.perception_msgs import labelled_links

#: Columns of ``iso_evidence.csv``. Declared once, here, so the writer and the
#: reader cannot drift: ``iso_evidence_report.py`` validates the header against
#: its own copy of the names it needs and says so when one is missing.
COLUMNS = [
    't',                    # [s] since node start
    't_wall',               # [s] unix, so a run lines up with a bag or a video
    # ── Cartesian speed: the quantity ISO actually limits ────────────────
    'tcp_x', 'tcp_y', 'tcp_z',
    'tcp_speed',            # [m/s] ‖J_v q̇‖ at the TCP, this sample
    'tcp_speed_max',        # [m/s] running max since the previous sample
    'cp_speed_max',         # [m/s] fastest control point reported by perception
    'cp_speed_max_label',
    # ── Joint space ──────────────────────────────────────────────────────
    'qdot_ratio_max',       # worst |q̇|/q̇_max this sample
    'qdot_ratio_max_run',   # running max since the previous sample
    'qdot_ratio_joint',     # which joint
    # ── Separation ───────────────────────────────────────────────────────
    'n_cp_valid', 'n_cp_total',
    'd_min', 'd_min_label',         # [m] closest surface gap
    'margin_min',                   # [m] min over CPs of (d − S_p); < 0 = SSM breached
    'margin_min_label',
    'S_p_at_margin_min',            # [m] the demand behind that margin
    'v_cap_min',                    # [m/s] tightest SSM cap
    'v_closing_max',                # [m/s] fastest closing speed
    'cap_excess_max',               # [m/s] max(v_closing − cap); > 0 = cap exceeded
    'n_inside_floor',               # CPs with d < C + Z_d + Z_r
    'perception_age',               # [s] now − header stamp of the last frame
    # ── What the stack itself said ───────────────────────────────────────
    'cbf_n_rows', 'cbf_slack', 'cbf_fault', 'cbf_n_violated', 'cbf_d_min',
    'iso_latched', 'iso_reason',
    # ── Command feasibility ──────────────────────────────────────────────
    'qddot_cmd_norm', 'qddot_real_norm', 'brake_frac',
    'tau_sat_count', 'tau_max_abs',
]

#: Kinds written to ``iso_events.csv``. Sparse by construction.
EV_STOP_ON, EV_STOP_OFF = 'iso_stop_latched', 'iso_stop_cleared'
EV_FAULT_ON, EV_FAULT_OFF = 'cbf_fault_on', 'cbf_fault_off'
EV_SAT_ON, EV_SAT_OFF = 'torque_saturation_on', 'torque_saturation_off'
EV_PERC_LOST, EV_PERC_BACK = 'perception_lost', 'perception_back'
EV_MARGIN, EV_CAP, EV_FLOOR = 'ssm_margin_negative', 'ssm_cap_exceeded', 'inside_floor'
EV_SPEED_PFL, EV_SPEED_RED = 'tcp_over_v_pfl', 'tcp_over_reduced_speed'
EV_NOTE = 'note'


class ISOEvidenceLogger(Node):

    def __init__(self):
        super().__init__('iso_evidence_logger')

        topics, P = load_cbf_config(self)
        self.P = P
        self._floor = P.iso_c_intrusion + P.iso_z_depth + P.iso_z_robot

        # Same default as experiment_logger, and for the same reason: $HOME
        # is not mounted out of the container, franka_logs/ is.
        out_root = declare_str(self, 'output_dir', resolve_log_dir())
        run_name = declare_str(self, 'run_name', 'run', allow_empty=True)
        self._rate = declare_float(self, 'sample_rate_hz', 100.0,
                                   positive=True, maximum=1000.0)
        self._tcp_link = declare_str(self, 'tcp_link', FR3_TCP_LINK)

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._dir = os.path.join(os.path.expanduser(out_root),
                                 f'{stamp}_{run_name}' if run_name else stamp)
        os.makedirs(self._dir, exist_ok=True)

        jl = load_franka_joint_limits(FR3_JOINT_KEYS)
        self._qdot_max = jl['qdot_max']
        self._effort_max = jl['effort_max']

        self._kin = CBFKinematics(pin.buildModelFromUrdf(build_urdf_no_hand()))
        self._fid: dict = {}
        self._tcp_fid = self._kin.resolve_frame_id(self._tcp_link)
        if self._tcp_fid is None:
            raise ValueError(f'tcp_link {self._tcp_link!r} is not in the model')

        # ── State ───────────────────────────────────────────────────────
        self._q = self._qdot = None
        self._js_stamp = 0.0
        self._dist = None
        self._dist_stamp = 0.0
        self._dist_age = float('nan')
        self._cbf = None
        self._iso = None
        self._tau_sat = np.zeros(NV)
        self._tau = np.zeros(NV)
        self._qddot_cmd = np.zeros(NV)
        self._qddot_real = np.zeros(NV)
        # "Received at least once" per optional channel. Without these, a topic
        # that never publishes still writes a column of zeros — and a column of
        # zeros reads as "measured, and fine" to the report, which then returns
        # PASS on a channel that was not running. Measured: a smoke run with no
        # qddot_to_torque produced `torque-saturation PASS 100%`. NaN until the
        # first message is what makes absent distinguishable from zero.
        self._have = {'sat': False, 'qddot': False, 'tau': False, 'real': False}
        self._qdot_prev = self._t_prev = None
        # Running extrema between samples, reset on every write.
        self._run_tcp_speed = 0.0
        self._run_ratio = 0.0
        self._run_ratio_joint = -1
        self._J_tcp: Optional[np.ndarray] = None   # cached at the last FK
        # Edge detectors for the event log.
        self._ev_state = {}
        self._t0 = time.time()
        self._t0_ros = self._now()
        self._n_rows = 0

        self._write_manifest(topics, jl)
        self._csv_f = open(os.path.join(self._dir, 'iso_evidence.csv'), 'w',
                           newline='')
        self._csv = csv.DictWriter(self._csv_f, fieldnames=COLUMNS)
        self._csv.writeheader()
        self._ev_f = open(os.path.join(self._dir, 'iso_events.csv'), 'w',
                          newline='')
        self._ev = csv.DictWriter(self._ev_f,
                                  fieldnames=['t', 't_wall', 'kind', 'detail'])
        self._ev.writeheader()

        # ── Wiring. Everything optional: a topic that never publishes leaves
        #    its columns NaN, which is how the CSV says "that channel was not
        #    running" as distinct from "it ran and read zero". ──────────────
        grp_io = MutuallyExclusiveCallbackGroup()
        grp_tick = MutuallyExclusiveCallbackGroup()
        best = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            JointState, topics.get('joint_states_fast',
                                   topics['joint_states_topic']),
            self._on_js, QoSProfile(depth=1), callback_group=grp_io)
        self.create_subscription(
            MultiLinkDistance, topics['per_link_distances'], self._on_dist,
            best, callback_group=grp_io)
        for key, default, cb in (
                ('cbf_status', '/NS_1/cbf_status', self._on_cbf),
                ('iso_safety', '/NS_1/iso_safety', self._on_iso),
                ('torque_saturation', '/NS_1/torque_saturation', self._on_sat),
                ('qddot_safe', '/NS_1/qddot_safe', self._on_qddot),
                ('torque_cmd', '/NS_1/torque_cmd', self._on_tau)):
            self.create_subscription(
                Float64MultiArray, topics.get(key, default), cb,
                QoSProfile(depth=1), callback_group=grp_io)

        self.create_timer(1.0 / self._rate, self._tick, callback_group=grp_tick)
        self.get_logger().warn(
            f'iso_evidence_logger → {self._dir}\n'
            f'  sampling {self._rate:.0f} Hz, TCP = {self._tcp_link}\n'
            f'  limits judged against: v_PFL={P.iso_v_pfl:.3f} m/s, reduced='
            f'{P.iso_tcp_reduced_speed:.3f} m/s, C+Z_d+Z_r={self._floor:.3f} m, '
            f'd_safe={P.d_safe:.3f} m\n'
            f'  ISO layer is {"ON" if P.iso_enabled else "OFF"} — the criteria '
            f'are evaluated either way.\n'
            f'  THIS IS EVIDENCE, NOT A CONFORMITY STATEMENT. See SAFETY.md.')

    # ── Manifest ─────────────────────────────────────────────────────────

    def _write_manifest(self, topics, jl) -> None:
        """The thresholds, WITH the data. A CSV whose limits live somewhere else
        is not evidence — the limits move between runs, and six months later
        nobody can say which ones a given run was judged against."""
        P = self.P
        man = {
            'schema': 'franka_experiments/iso_evidence/1',
            'created': datetime.now().isoformat(timespec='seconds'),
            'created_unix': self._t0,
            'run_dir': self._dir,
            'sample_rate_hz': self._rate,
            'tcp_link': self._tcp_link,
            'config_path': P.config_path,
            'git': _git_describe(),
            'disclaimer': (
                'Evidence about implemented measures. NOT a conformity or '
                'certification statement. The CBF chain is single-channel '
                'Python over best-effort DDS and cannot reach PL d / SIL 2. '
                'See franka_experiments/SAFETY.md.'),
            'iso_params': {k: getattr(P, k) for k in sorted(vars(P))
                           if k.startswith('iso_')},
            'limits': {
                'd_safe': P.d_safe,
                'floor_c_zd_zr': self._floor,
                'link_speed_max': P.link_speed_max,
                'retreat_cap_max_speed': P.retreat_cap_max_speed,
                'velocity_box_margin': P.velocity_box_margin,
                'min_confidence': P.min_confidence,
                'qp_rate_hz': P.qp_rate_hz,
                'cbf_update_rate_hz': P.cbf_update_rate_hz,
                'distance_timeout': P.distance_timeout,
                'zone_r_notice': P.zone_r_notice,
                'zone_r_active': P.zone_r_active,
                'zone_r_priority': P.zone_r_priority,
                'zone_r_hold': P.zone_r_hold,
                # FR3 datasheet maximum reach. Recorded because it is what
                # decides whether a separation distance is ACHIEVABLE at all:
                # a floor larger than the reach cannot be satisfied by any
                # motion, so the report must say "structural" rather than
                # printing a per-run failure the operator could try to tune away.
                'robot_reach_m': 0.855,
            },
            'joint_limits': {k: np.asarray(v).tolist()
                             for k, v in jl.items() if k != 'joints'},
            'topics': dict(topics),
            'iso_layer_active': {
                'iso_enabled': P.iso_enabled,
                'iso_mode': P.iso_mode,
                'iso_ssm_speed_rows': P.iso_ssm_speed_rows,
                'iso_monitor_enabled': P.iso_monitor_enabled,
            },
        }
        with open(os.path.join(self._dir, 'iso_manifest.json'), 'w') as fh:
            json.dump(man, fh, indent=2, sort_keys=True, default=str)

    # ── Inputs ───────────────────────────────────────────────────────────

    def _on_js(self, msg: JointState) -> None:
        n2p = dict(zip(msg.name, msg.position))
        n2v = dict(zip(msg.name, msg.velocity))
        try:
            q = np.array([n2p[n] for n in FR3_JOINTS])
            qdot = np.array([n2v[n] for n in FR3_JOINTS])
        except KeyError:
            return
        self._q, self._qdot = q, qdot
        self._js_stamp = self._now()

        # Running extrema at the FULL joint-state rate, so an excursion shorter
        # than the sample period still reaches the CSV.
        ratio = np.abs(qdot) / self._qdot_max
        j = int(np.argmax(ratio))
        if ratio[j] > self._run_ratio:
            self._run_ratio, self._run_ratio_joint = float(ratio[j]), j
        if self._J_tcp is not None:
            # Jacobian cached at the last FK: exact in q̇, one sample stale in q.
            # The approximation is in the geometry, not the speed.
            v = float(np.linalg.norm(self._J_tcp @ qdot))
            if v > self._run_tcp_speed:
                self._run_tcp_speed = v

        # Realized q̈, finite-differenced on the HEADER stamp — callback jitter
        # would corrupt the derivative.
        t_h = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._qdot_prev is not None and self._t_prev is not None:
            dt = t_h - self._t_prev
            if 1e-4 < dt < 0.1:
                self._qddot_real = (qdot - self._qdot_prev) / dt
                self._have['real'] = True
        self._qdot_prev, self._t_prev = qdot, t_h

    def _on_dist(self, msg: MultiLinkDistance) -> None:
        self._dist = msg
        now = self._now()
        self._dist_stamp = now
        t_cap = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = now - t_cap
        self._dist_age = age if -1.0 < age < 10.0 else float('nan')

    def _on_cbf(self, msg):
        self._cbf = [float(v) for v in msg.data]

    def _on_iso(self, msg):
        if len(msg.data) >= 2:
            self._iso = [float(v) for v in msg.data]

    def _on_sat(self, msg):
        d = np.asarray(msg.data, dtype=float)
        if d.size == NV:
            self._tau_sat = d
            self._have['sat'] = True

    def _on_qddot(self, msg):
        d = np.asarray(msg.data, dtype=float)
        if d.size == NV:
            self._qddot_cmd = d
            self._have['qddot'] = True

    def _on_tau(self, msg):
        d = np.asarray(msg.data, dtype=float)
        if d.size == NV:
            self._tau = d
            self._have['tau'] = True

    # ── Sample ───────────────────────────────────────────────────────────

    def _tick(self) -> None:
        P = self.P
        now = self._now()
        t = now - self._t0_ros
        row = {k: float('nan') for k in COLUMNS}
        row['t'] = round(t, 6)
        row['t_wall'] = round(time.time(), 6)

        if self._q is None:
            self._write(row)
            return

        q, qdot = self._q, self._qdot
        self._kin.update(q, qdot, with_jdot=False)
        p_tcp = np.array(self._kin.data.oMf[self._tcp_fid].translation)
        self._J_tcp = self._kin.point_jacobian_pos(self._tcp_fid, p_tcp)
        v_tcp = float(np.linalg.norm(self._J_tcp @ qdot))

        row['tcp_x'], row['tcp_y'], row['tcp_z'] = (round(float(x), 6) for x in p_tcp)
        row['tcp_speed'] = round(v_tcp, 6)
        row['tcp_speed_max'] = round(max(self._run_tcp_speed, v_tcp), 6)
        ratio = np.abs(qdot) / self._qdot_max
        j = int(np.argmax(ratio))
        row['qdot_ratio_max'] = round(float(ratio[j]), 6)
        row['qdot_ratio_max_run'] = round(max(self._run_ratio, float(ratio[j])), 6)
        row['qdot_ratio_joint'] = (self._run_ratio_joint
                                   if self._run_ratio >= ratio[j] else j) + 1

        # ── Separation, evaluated with the CONFIG's own ISO constants,
        #    whether or not the ISO layer is switched on. ─────────────────
        fresh = (self._dist is not None
                 and (now - self._dist_stamp) < P.distance_timeout)
        row['perception_age'] = (round(self._dist_age, 6)
                                 if np.isfinite(self._dist_age) else float('nan'))
        if fresh:
            ev = self._evaluate(qdot)
            row.update(ev)
        self._event_edge(EV_PERC_LOST, EV_PERC_BACK, not fresh,
                         f'no fresh {self.P.distance_timeout:.2f} s distance frame', t)

        if self._cbf is not None:
            c = self._cbf
            row['cbf_n_rows'] = c[0] if len(c) > 0 else float('nan')
            row['cbf_slack'] = c[1] if len(c) > 1 else float('nan')
            row['cbf_fault'] = c[2] if len(c) > 2 else float('nan')
            row['cbf_n_violated'] = c[3] if len(c) > 3 else float('nan')
            row['cbf_d_min'] = c[4] if len(c) > 4 else float('nan')
            self._event_edge(EV_FAULT_ON, EV_FAULT_OFF,
                             len(c) > 2 and c[2] >= 1.0, 'cbf_status[2]=1', t)
        if self._iso is not None:
            row['iso_latched'] = self._iso[0]
            row['iso_reason'] = self._iso[1]
            self._event_edge(EV_STOP_ON, EV_STOP_OFF, self._iso[0] >= 1.0,
                             f'trip_reason={self._iso[1]:.0f}', t)

        nan = float('nan')
        row['qddot_cmd_norm'] = (round(float(np.linalg.norm(self._qddot_cmd)), 6)
                                 if self._have['qddot'] else nan)
        row['qddot_real_norm'] = (round(float(np.linalg.norm(self._qddot_real)), 6)
                                  if self._have['real'] else nan)
        n2 = float(self._qddot_cmd @ self._qddot_cmd)
        row['brake_frac'] = (
            round(float(self._qddot_real @ self._qddot_cmd) / n2, 6)
            if (self._have['qddot'] and self._have['real'] and n2 > 0.25) else nan)
        n_sat = int(np.count_nonzero(self._tau_sat))
        row['tau_sat_count'] = n_sat if self._have['sat'] else nan
        row['tau_max_abs'] = (round(float(np.max(np.abs(self._tau))), 6)
                              if self._have['tau'] else nan)
        self._event_edge(EV_SAT_ON, EV_SAT_OFF, self._have['sat'] and n_sat > 0,
                         f'{n_sat} joint(s) on a torque bound', t)

        # Threshold crossings, as events, so a short one is not lost to sampling.
        self._event_edge(EV_SPEED_PFL, None, v_tcp > P.iso_v_pfl,
                         f'tcp {v_tcp:.3f} > v_PFL {P.iso_v_pfl:.3f} m/s', t)
        if str(P.iso_mode) == 'reduced':
            self._event_edge(EV_SPEED_RED, None,
                             v_tcp > P.iso_tcp_reduced_speed,
                             f'tcp {v_tcp:.3f} > reduced '
                             f'{P.iso_tcp_reduced_speed:.3f} m/s', t)

        self._run_tcp_speed = 0.0
        self._run_ratio, self._run_ratio_joint = 0.0, -1
        self._write(row)

    def _evaluate(self, qdot) -> dict:
        """Per control point: its speed, its separation demand, its cap.

        Deliberately a SECOND implementation of the same arithmetic
        ``iso_safety_monitor`` runs, fed from the same published geometry. Two
        independent readings of one scene is what makes the evidence worth
        anything: if this and ``cbf_status[5..8]`` disagree in a run, that
        disagreement is the finding.
        """
        P = self.P
        out = {}
        n_valid = n_total = 0
        d_min, d_lbl = float('inf'), ''
        margin_min, margin_lbl, s_p_at = float('inf'), '', float('nan')
        v_cap_min, v_cls_max, excess_max = float('inf'), 0.0, -float('inf')
        cp_speed_max, cp_lbl = 0.0, ''
        n_inside = 0

        for label, ld in labelled_links(self._dist):
            n_total += 1
            if not ld.valid:
                continue
            d = float(ld.distance)
            if not np.isfinite(d):
                continue
            n_valid += 1
            if d < d_min:
                d_min, d_lbl = d, label
            if d < self._floor:
                n_inside += 1

            fid = self._frame_id(ld.robot_link_name)
            if fid is None:
                continue
            n = np.array([ld.direction.x, ld.direction.y, ld.direction.z])
            nn = float(np.linalg.norm(n))
            if nn < 1e-9:
                continue
            n /= nn                                  # obstacle → control point
            p_r = np.array([ld.closest_point_robot.x, ld.closest_point_robot.y,
                            ld.closest_point_robot.z])
            v_cp = self._kin.point_jacobian_pos(fid, p_r) @ qdot
            speed = float(np.linalg.norm(v_cp))
            if speed > cp_speed_max:
                cp_speed_max, cp_lbl = speed, label
            v_closing = max(-float(n @ v_cp), 0.0)
            v_app = max(float(n @ np.array([ld.obstacle_velocity.x,
                                            ld.obstacle_velocity.y,
                                            ld.obstacle_velocity.z])), 0.0)
            cap = ssm_speed_cap(d, v_app, t_r=P.iso_t_reaction, a_s=P.iso_a_stop,
                                c=P.iso_c_intrusion, z_d=P.iso_z_depth,
                                z_r=P.iso_z_robot, v_max=P.link_speed_max)
            if str(P.iso_mode) == 'reduced' and ld.robot_link_name == self._tcp_link:
                cap = min(cap, P.iso_tcp_reduced_speed)
            s_p = protective_separation(v_closing, v_app, t_r=P.iso_t_reaction,
                                        a_s=P.iso_a_stop, c=P.iso_c_intrusion,
                                        z_d=P.iso_z_depth, z_r=P.iso_z_robot)
            if d - s_p < margin_min:
                margin_min, margin_lbl, s_p_at = d - s_p, label, s_p
            v_cap_min = min(v_cap_min, cap)
            v_cls_max = max(v_cls_max, v_closing)
            excess_max = max(excess_max, v_closing - cap)

        t = self._now() - self._t0_ros
        self._event_edge(EV_MARGIN, None, np.isfinite(margin_min) and margin_min < 0.0,
                         f'{margin_lbl}: d − S_p = {margin_min:.4f} m', t)
        self._event_edge(EV_CAP, None,
                         np.isfinite(excess_max) and excess_max > P.iso_speed_tol,
                         f'v_closing exceeds cap by {excess_max:.4f} m/s', t)
        self._event_edge(EV_FLOOR, None, n_inside > 0,
                         f'{n_inside} CP inside C+Z_d+Z_r ({self._floor:.3f} m), '
                         f'closest {d_min:.4f} m at {d_lbl}', t)

        out['n_cp_valid'], out['n_cp_total'] = n_valid, n_total
        out['d_min'] = round(d_min, 6) if np.isfinite(d_min) else float('nan')
        out['d_min_label'] = d_lbl
        out['margin_min'] = round(margin_min, 6) if np.isfinite(margin_min) else float('nan')
        out['margin_min_label'] = margin_lbl
        out['S_p_at_margin_min'] = round(s_p_at, 6) if np.isfinite(s_p_at) else float('nan')
        out['v_cap_min'] = round(v_cap_min, 6) if np.isfinite(v_cap_min) else float('nan')
        out['v_closing_max'] = round(v_cls_max, 6)
        out['cap_excess_max'] = round(excess_max, 6) if np.isfinite(excess_max) else float('nan')
        out['n_inside_floor'] = n_inside
        out['cp_speed_max'] = round(cp_speed_max, 6)
        out['cp_speed_max_label'] = cp_lbl
        return out

    # ── Plumbing ─────────────────────────────────────────────────────────

    def _frame_id(self, link):
        if link not in self._fid:
            self._fid[link] = self._kin.resolve_frame_id(link)
        return self._fid[link]

    def _event_edge(self, on_kind, off_kind, active: bool, detail: str, t: float):
        """Write an event only on the RISING (and optionally falling) edge.

        A condition true for ten seconds is one event with a duration, not a
        thousand rows. ``iso_evidence_report.py`` pairs them up.
        """
        was = self._ev_state.get(on_kind, False)
        if active and not was:
            self._event(on_kind, detail, t)
        elif was and not active and off_kind is not None:
            self._event(off_kind, detail, t)
        self._ev_state[on_kind] = active

    def _event(self, kind: str, detail: str, t: float) -> None:
        self._ev.writerow({'t': round(t, 6), 't_wall': round(time.time(), 6),
                           'kind': kind, 'detail': detail})
        self._ev_f.flush()

    def _write(self, row) -> None:
        self._csv.writerow(row)
        self._n_rows += 1
        if self._n_rows % 500 == 0:
            self._csv_f.flush()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def close(self) -> None:
        try:
            self._event(EV_NOTE, f'run ended after {self._n_rows} samples',
                        self._now() - self._t0_ros)
            self._csv_f.flush(); os.fsync(self._csv_f.fileno()); self._csv_f.close()
            self._ev_f.flush(); os.fsync(self._ev_f.fileno()); self._ev_f.close()
        except Exception:
            pass
        self.get_logger().warn(
            f'iso_evidence_logger: {self._n_rows} samples → {self._dir}\n'
            f'  send the WHOLE directory (manifest + evidence + events), then\n'
            f'  python3 scripts/iso_evidence_report.py {self._dir}')


def _git_describe() -> dict:
    here = os.path.dirname(os.path.abspath(__file__))
    def _run(*a):
        try:
            return subprocess.run(a, cwd=here, capture_output=True, text=True,
                                  timeout=5).stdout.strip() or None
        except Exception:
            return None
    return {'sha': _run('git', 'rev-parse', 'HEAD'),
            'branch': _run('git', 'rev-parse', '--abbrev-ref', 'HEAD'),
            'dirty': bool(_run('git', 'status', '--porcelain'))}


def main(args=None):
    rclpy.init(args=args)
    node = ISOEvidenceLogger()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
