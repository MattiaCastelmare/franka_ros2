#!/usr/bin/env python3
"""episode_supervisor: start the episode when the stack is up, end the run, label it SIM.

SIM only. One per run:

1. waits until the closed loop is actually running: q̈_nom (commander or scenario
   commander), q̈_safe (filter) and tau_cmd (qddot_to_torque) all flowing, the
   RT controller ACTIVE in the controller manager, plus ``warmup_s`` of sim
   time. With perception on it also waits for /cbf/per_link_distances. Until
   then /sim/episode reads WAITING. (The controller check is not implied by
   tau_cmd: qddot_to_torque publishes before the stack's spawner has fired,
   and a scenario episode used to start, and end, with the arm floating on the
   plugin's gravity term.);
2. publishes /sim/episode = [state, t_start, t_now] (state 0 waiting, 1 running,
   2 done) at 50 Hz. The scenario commander and the obstacle system start their
   laws on the WAITING -> RUNNING edge, so the episode clock is one clock;
3. after ``duration_s`` of sim time (0 = until stopped) it finalises and exits.
   The launch ends on that exit.

``start_mode`` (H2, opt-in; default ``legacy`` = the behaviour above, unchanged):

* ``legacy``: t_start = (ready + warmup_s) + start_lead_s + start_offset_s.
* ``phase``:  on top of every legacy condition, the obstacle is born only after the arm has
  tracked ``start_settle_cycles`` full cycles of the commander's curve with the controller
  active, every |ee_desired - ee_actual| <= ``start_track_err_max``, no stale or off-curve
  sample, and the phase advancing at its nominal rate; t_start is then the predicted sim time
  at which the curve phase reaches ``start_phase`` (announced 0.25-0.5 s ahead). The phase is
  recovered from /NS_1/ee_desired against the commander's own curve, rebuilt from the
  commander's own parameters (simexp/phase_start.py explains why the time of the first
  ee_desired message cannot be used). If that does not happen within ``start_timeout_s`` of
  sim time, measured from the supervisor's FIRST sim tick (PREREGISTRATION.md §1; one deadline
  shared by every stage: commander parameters, controller status, tracking qualification), the
  obstacle is NEVER born: the episode is finalised as ``start_failed`` and the node exits with
  code 3 (an explicit failure, never an uncontrolled start). The state at the
  birth (phase, tracking, q, q-dot, EE, controller, HARD-reset ticks, perception readiness) is
  written to run_manifest_sim.json under ``episode.start``. start_offset_s is NOT applied in
  this mode (it is a time offset from an uncontrolled instant); it is recorded.

  Controller status in phase mode (F-8-23): at most ONE list_controllers request is in flight;
  a request without a response after ``start_ctrl_timeout_s`` of sim time is cancelled
  (rclpy then drops its late response: the client no longer holds the sequence number) and a
  new one is sent; the controller counts as active only while the last answered request said so
  and was SENT at most ``start_ctrl_max_age_s`` of sim time ago (an answer cannot describe a
  state older than its request, so a reply delayed in transport arrives already aged). A missing, late or stale
  answer is NOT active: the window restarts and the start waits (and fails at the deadline).

  Sim-clock liveness (phase mode, before the birth only): every deadline above is sim time and
  the supervisor's tick is a sim-time timer, so a sim clock that stops would freeze them all. A
  separate STEADY-clock timer fails the start explicitly if sim time has not advanced for
  ``start_clock_stall_wall_s`` of monotonic wall time. Wall time is used for nothing else.

Finalising (ground rule 5):
* run_manifest_sim.json gets the episode times, the reflex verdict and RTF;
* experiment_logger's run_manifest.json gets ``"plant": "gazebo"``, ``"SIM": true``,
  the RTF and the git hash (an adapter on the sim side, the logger is unchanged);
* franka_experiments/franka_logs is checked: anything that appeared there during
  the run is a SIM output in a hardware directory. It is moved into
  ``<run_dir>/stray_logs`` and the run is marked as such.
"""
import glob
import json
import math
import os
import shutil
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from simexp.sim_config import git_provenance  # noqa: E402

WAITING, RUNNING, DONE = 0.0, 1.0, 2.0


def hardware_log_dir():
    for cand in ('/ros2_ws/src/franka_experiments/franka_logs',
                 os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..',
                              'franka_experiments', 'franka_logs')):
        if os.path.isdir(cand):
            return os.path.normpath(cand)
    return None


class EpisodeSupervisor(Node):
    def __init__(self):
        super().__init__('episode_supervisor')
        dp = self.declare_parameter
        self.run_dir = dp('run_dir', '').value
        self.duration_s = float(dp('duration_s', 0.0).value)
        self.warmup_s = float(dp('warmup_s', 2.0).value)
        self.need_perception = bool(dp('perception', True).value)
        # The start is announced AHEAD (start_lead_s) so the gz obstacle system,
        # the scenario commander and this node all switch at the same sim
        # instant. start_offset_s is the seeded phase within a camera frame.
        self.start_lead_s = float(dp('start_lead_s', 0.25).value)
        self.start_offset_s = float(dp('start_offset_s', 0.0).value)
        self.controller = dp('controller_name', 'rt_torque_controller').value
        self.controller_active = not self.controller
        from controller_manager_msgs.srv import ListControllers
        self._list_req = ListControllers.Request()
        self._list_cli = self.create_client(
            ListControllers, dp('controller_manager', '/NS_1/controller_manager').value
            + '/list_controllers')
        self._list_fut = None
        self.state = WAITING
        self.t_start = None
        self.seen = {'qddot_nom': None, 'qddot_safe': None, 'torque_cmd': None,
                     'per_link_distances': None}
        self.ready_since = None
        self.t_first_tick = None
        self.hw_dir = hardware_log_dir()
        self.hw_before = set(os.listdir(self.hw_dir)) if self.hw_dir else set()
        self.t_wall0 = time.time()
        self.pub = self.create_publisher(Float64MultiArray, '/sim/episode',
                                         QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        from std_msgs.msg import Float64
        self._Float64 = Float64
        self.pub_start = self.create_publisher(Float64, '/sim/episode_start', 10)
        for key, topic in (('qddot_nom', '/NS_1/qddot_nom'), ('qddot_safe', '/NS_1/qddot_safe'),
                           ('torque_cmd', '/NS_1/torque_cmd')):
            self.create_subscription(Float64MultiArray, topic,
                                     lambda m, k=key: self._seen(k), 10)
        if self.need_perception:
            from franka_msgs.msg import MultiLinkDistance
            self.create_subscription(
                MultiLinkDistance, '/cbf/per_link_distances',
                lambda m: self._seen('per_link_distances'),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(0.02, self._tick)
        self.finished = False
        self._n_start = 0
        self.exit_code = 0
        self.start_mode = str(dp('start_mode', 'legacy').value).strip().lower()
        if self.start_mode not in ('legacy', 'phase'):
            raise ValueError(f'start_mode={self.start_mode!r}: expected legacy or phase')
        if self.start_mode == 'phase':
            self._init_phase_start(dp)

    # ------------------------------------------------------------------ H2: phase-aligned start
    def _init_phase_start(self, dp):
        from geometry_msgs.msg import PointStamped
        from rcl_interfaces.msg import Log
        from rcl_interfaces.srv import GetParameters
        from sensor_msgs.msg import JointState
        from simexp import phase_start as ps
        self._ps = ps
        self.start_cfg = ps.StartConfig(
            phase0=float(dp('start_phase', 0.0).value) % 1.0,
            settle_cycles=float(dp('start_settle_cycles', 1.0).value),
            track_err_max=float(dp('start_track_err_max', 0.02).value),
            timeout_s=float(dp('start_timeout_s', 90.0).value),
            lead_min_s=float(dp('start_lead_min_s', 0.25).value),
            lead_announce_s=float(dp('start_lead_announce_s', 0.5).value))
        # F-8-23: controller-status polling with a timeout and a freshness bound (sim time)
        self.ctrl_poll_s = float(dp('start_ctrl_poll_s', 0.2).value)
        self.ctrl_timeout_s = float(dp('start_ctrl_timeout_s', 1.0).value)
        self.ctrl_max_age_s = float(dp('start_ctrl_max_age_s', 0.5).value)
        self._list_sent_t = None                  # sim time the in-flight request was sent
        self._ctrl_resp_t = None                  # sim SEND time of the last answered request
        self._ctrl_resp_active = False            # what that answer said
        self.ctrl_stats = dict(sent=0, answered=0, timeouts=0, stale=0)
        # Sim-clock liveness (monotonic wall; see the module docstring)
        from rclpy.clock import Clock, ClockType
        self.clock_stall_wall_s = float(dp('start_clock_stall_wall_s', 60.0).value)
        self._mono = time.monotonic
        self._clock_last_sim = None
        self._clock_last_adv_wall = self._mono()
        self._watchdog = self.create_timer(min(1.0, self.clock_stall_wall_s / 4.0),
                                           self._clock_watchdog,
                                           clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.commander = dp('commander_node', '/NS_1/pentagon_qddot_commander').value
        self._par_cli = self.create_client(GetParameters, self.commander + '/get_parameters')
        self._par_req = GetParameters.Request(names=list(ps.COMMANDER_PARAMS))
        self._par_fut = None
        self.commander_params = None
        self.qual = None                          # StartQualifier, once the commander answered
        self.start_failed = None
        self._ctrl_hist = []                      # (sim t, active) on every change
        self._ee = {'d': {}, 'a': {}}
        self._js = deque(maxlen=400)              # (stamp, {name: (q, qd)})
        self._pld = deque(maxlen=400)             # (receipt sim t, capture stamp, n valid)
        self._hard_log = []                       # (sim t, text) of the commander's HARD resets
        self.placement = None
        self.create_subscription(PointStamped, dp('ee_desired_topic', '/NS_1/ee_desired').value,
                                 lambda m: self._ee_cb('d', m), 200)
        self.create_subscription(PointStamped, dp('ee_actual_topic', '/NS_1/ee_actual').value,
                                 lambda m: self._ee_cb('a', m), 200)
        self.create_subscription(JointState, dp('joint_states_topic', '/NS_1/franka/joint_states').value,
                                 self._js_cb, 200)
        self.create_subscription(Log, '/rosout', self._rosout_cb, 200)
        if self.need_perception:
            from franka_msgs.msg import MultiLinkDistance
            self.create_subscription(
                MultiLinkDistance, '/cbf/per_link_distances', self._pld_cb,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.get_logger().info(f'start_mode=phase: {self.start_cfg} (start_offset_s='
                               f'{self.start_offset_s} recorded, not applied)')

    @staticmethod
    def _stamp(m):
        return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9

    def _ee_cb(self, side, m):
        t = self._stamp(m)
        p = (m.point.x, m.point.y, m.point.z)
        other = self._ee['a' if side == 'd' else 'd']
        if t in other:                            # the commander stamps both with ONE clock read
            q = other.pop(t)
            pd, pa = (p, q) if side == 'd' else (q, p)
            for old in [k for k in other if k < t]:
                del other[old]                    # its partner will never come
            if self.qual is not None:
                self.qual.feed(t, pd, pa, self.controller_active)
        else:
            self._ee[side][t] = p
            if len(self._ee[side]) > 400:
                del self._ee[side][min(self._ee[side])]

    def _js_cb(self, m):
        n = len(m.velocity)
        self._js.append((self._stamp(m), {nm: (m.position[i], m.velocity[i] if i < n else math.nan)
                                          for i, nm in enumerate(m.name)}))

    def _pld_cb(self, m):
        self._seen('per_link_distances')
        nv = sum(1 for lk in m.links if lk.valid and math.isfinite(lk.distance))
        self._pld.append((self._now(), self._stamp(m), nv))

    def _rosout_cb(self, m):
        if m.name.endswith('pentagon_qddot_commander') and m.msg.startswith('HARD reset'):
            self._hard_log.append((m.stamp.sec + m.stamp.nanosec * 1e-9, m.msg[:120]))

    def _poll_controller_continuous(self, now):
        """Phase mode only (F-8-23). Never blocks; never more than one request in flight.

        Keeps asking, so a controller that drops out is seen (legacy stops at activation). A
        request unanswered after ctrl_timeout_s (sim) is cancelled: Future.cancel() runs the
        client's remove_pending_request, so a late response finds no sequence number and rclpy
        drops it, and it can never complete a newer request. controller_active is recomputed
        on every tick from the last answer and its age, so an old 'active' never stands in for
        a current one.
        """
        if not self.controller:                   # no controller to gate on (legacy meaning)
            self.controller_active = True
            return
        fut = self._list_fut
        if fut is not None:
            if fut.done():
                self._list_fut = None
                res = fut.result()
                self.ctrl_stats['answered'] += 1
                # the server's snapshot is no older than the request: age it from the send
                # time, so a reply delayed in transport is not taken for a current state
                self._ctrl_resp_t = self._list_sent_t
                self._ctrl_resp_active = res is not None and any(
                    c.name == self.controller and c.state == 'active' for c in res.controller)
            elif not (0.0 <= now - self._list_sent_t <= self.ctrl_timeout_s):
                fut.cancel()
                self._list_fut = None
                self.ctrl_stats['timeouts'] += 1
                if self.ctrl_stats['timeouts'] <= 5:
                    self.get_logger().warn(
                        f'list_controllers: no response {now - self._list_sent_t:.3f} s (sim) after '
                        f'the request -> cancelled, asking again (timeouts so far '
                        f'{self.ctrl_stats["timeouts"]})')
        if (self._list_fut is None and self._list_cli.service_is_ready()
                and (self._list_sent_t is None or not (0.0 <= now - self._list_sent_t < self.ctrl_poll_s))):
            self._list_sent_t = now
            self._list_fut = self._list_cli.call_async(self._list_req)
            self.ctrl_stats['sent'] += 1
        fresh = self._ctrl_resp_t is not None and 0.0 <= now - self._ctrl_resp_t <= self.ctrl_max_age_s
        act = bool(fresh and self._ctrl_resp_active)
        if act != self.controller_active or not self._ctrl_hist:
            why = ('answer' if fresh else 'no answer yet' if self._ctrl_resp_t is None else
                   f'stale answer ({now - self._ctrl_resp_t:.3f} s old)')
            if self.controller_active and not fresh:
                self.ctrl_stats['stale'] += 1
            self._ctrl_hist.append((round(now, 4), act, why))
            self.get_logger().info(f'{self.controller} {"is active" if act else "is NOT active"}'
                                   f' ({why})')
        self.controller_active = act

    def _clock_watchdog(self):
        """STEADY-clock timer, phase mode, before the birth only: fail if sim time stops.

        Sim time decides every start condition; this only catches a sim clock that does not
        advance (no /clock, or a stalled simulator), which would otherwise freeze the sim-time
        tick and every sim-time deadline and leave the run waiting until an outside kill.
        """
        if self.state != WAITING or self.finished:
            return
        now, w = self._now(), self._mono()
        if self._clock_last_sim is None or now != self._clock_last_sim:
            self._clock_last_sim, self._clock_last_adv_wall = now, w
            return
        if w - self._clock_last_adv_wall <= self.clock_stall_wall_s:
            return
        self.start_failed = (f'sim clock stalled: sim time {now:.3f} s has not advanced for '
                             f'{w - self._clock_last_adv_wall:.1f} s of monotonic wall time '
                             f'(limit {self.clock_stall_wall_s} s); first sim tick '
                             f'{self.t_first_tick}')
        self.get_logger().error(f'SIM episode START FAILED: {self.start_failed} '
                                f'-> no obstacle, run aborted')
        self.state = DONE
        self.exit_code = 3
        self.pub.publish(Float64MultiArray(data=[self.state, -1.0, now]))
        self.finalise()
        self.finished = True
        raise SystemExit(self.exit_code)

    def _poll_commander_params(self):
        if self.qual is not None:
            return
        if self._par_fut is not None:
            if not self._par_fut.done():
                return
            res, self._par_fut = self._par_fut.result(), None
            if res is None or len(res.values) != len(self._par_req.names):
                return
            from rcl_interfaces.msg import ParameterType as PT
            vals = {}
            for name, v in zip(self._par_req.names, res.values):
                vals[name] = {PT.PARAMETER_DOUBLE: v.double_value, PT.PARAMETER_INTEGER: v.integer_value,
                              PT.PARAMETER_STRING: v.string_value, PT.PARAMETER_BOOL: v.bool_value,
                              PT.PARAMETER_DOUBLE_ARRAY: list(v.double_array_value),
                              PT.PARAMETER_INTEGER_ARRAY: list(v.integer_array_value)}.get(v.type)
            missing = [k for k in ('path_type', 'center_xyz', 'radius', 'plane', 'cycle_time')
                       if vals.get(k) in (None, [], '')]
            if missing:
                self.get_logger().error(f'commander parameters missing {missing}: {vals}')
                return
            self.commander_params = vals
            self.start_cfg.cycle_time = float(vals['cycle_time'])
            if vals.get('hard_reset_thr') is not None:
                self.start_cfg.hard_reset_thr = float(vals['hard_reset_thr'])
            ps = self._ps
            self.qual = ps.StartQualifier(cfg=self.start_cfg,
                                          est=ps.PhaseEstimator(ps.build_cycle_path(vals)))
            # F-8-24: ONE deadline, from the supervisor's first sim tick (PREREGISTRATION.md §1),
            # not a fresh timeout_s from the moment the commander's parameters arrived.
            self.qual.t_first_poll = self.t_first_tick
            est = self.qual.est
            p0 = est.tab[int(round(self.start_cfg.phase0 * est.n)) % est.n].round(4).tolist()
            self.get_logger().info(f'phase start: commander curve {vals["path_type"]} '
                                   f'centre {vals["center_xyz"]} r={vals["radius"]} plane={vals["plane"]} '
                                   f'cycle {vals["cycle_time"]} s; P(phase0)={p0}')
            return
        if self._par_cli.service_is_ready():
            self._par_fut = self._par_cli.call_async(self._par_req)

    def _record_placement(self):
        """Once the birth has passed: the state at t_start, from data stamped at or before it."""
        q = self.qual
        if self.placement is not None or q is None or q.placement is None:
            return
        ts = self.t_start
        after = [x for x in self._js if x[0] >= ts]
        before = [x for x in self._js if x[0] <= ts]
        if not after:
            return                                # wait for one joint state past the birth
        jn = [f'fr3_joint{i}' for i in range(1, 8)]
        b = before[-1] if before else None
        js = None if b is None else dict(
            stamp=b[0], q=[b[1].get(n, (math.nan, math.nan))[0] for n in jn],
            qdot=[b[1].get(n, (math.nan, math.nan))[1] for n in jn])
        pld = [x for x in self._pld if x[0] <= ts]
        last1 = [x for x in pld if x[0] > ts - 1.0]
        self.placement = dict(
            q.placement, joint_state_before=js,
            controller_active_at_start=bool(self.controller_active), controller_history=self._ctrl_hist,
            perception=dict(msgs_last_1s=len(last1),
                            last_capture_age_s=(ts - pld[-1][1]) if pld else None,
                            nonempty_last_1s=sum(1 for x in last1 if x[2] > 0)),
            hard_reset_log_lines_before_start=[x for x in self._hard_log if x[0] <= ts])

    def _phase_tick(self, now, ready):
        """WAITING in phase mode. Returns True when the episode was announced."""
        self._poll_commander_params()
        if self.qual is None:
            if self.t_first_tick is not None and now - self.t_first_tick > self.start_cfg.timeout_s:
                self.start_failed = (f'commander parameters not received from {self.commander} '
                                     f'within {self.start_cfg.timeout_s} s sim')
            return False
        warm = ready and self.ready_since is not None and now - self.ready_since >= self.warmup_s
        dec = self.qual.poll(now, allow=warm)
        if self.qual.state == 'failed':
            self.start_failed = (f'{self.qual.fail_reason}; deadline from the first sim tick '
                                 f'{self.t_first_tick}; controller status {self.ctrl_stats}, '
                                 f'fresh-active now {self.controller_active}')
            return False
        if dec is None:
            return False
        self.t_start = dec['t_start']
        self.get_logger().info(
            f'SIM episode START at t={self.t_start:.4f} s (sim), announced at {now:.3f} '
            f'[phase mode: phase0={self.start_cfg.phase0} lead {dec["lead_s"]:.3f} s, '
            f'{dec["window_cycles"]:.3f} clean cycles since {dec["window_t0"]:.3f}, '
            f'max |e| {dec["window_max_err_m"] * 1e3:.1f} mm, rate x{dec["rate_over_nominal"]:.4f}]')
        return True

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _seen(self, key):
        self.seen[key] = self._now()

    def _poll_controller(self):
        if self.controller_active:
            return
        if self._list_fut is not None:
            if not self._list_fut.done():
                return
            res = self._list_fut.result()
            self._list_fut = None
            if res is not None and any(c.name == self.controller and c.state == 'active'
                                       for c in res.controller):
                self.controller_active = True
                self.get_logger().info(f'{self.controller} is active')
                return
        if self._list_cli.service_is_ready():
            self._list_fut = self._list_cli.call_async(self._list_req)

    def _ready(self, now):
        keys = ['qddot_nom', 'qddot_safe', 'torque_cmd'] + (
            ['per_link_distances'] if self.need_perception else [])
        if self.start_mode != 'phase':            # phase mode polls continuously in _tick
            self._poll_controller()
        return self.controller_active and all(
            self.seen[k] is not None and now - self.seen[k] < 0.5 for k in keys)

    def _tick(self):
        now = self._now()
        if self.start_mode == 'phase':
            if now > 0.0 and self.t_first_tick is None:
                self.t_first_tick = now
            self._poll_controller_continuous(now)
            if self.state == WAITING:
                ready = self._ready(now)
                if ready:
                    self.ready_since = self.ready_since if self.ready_since is not None else now
                else:
                    self.ready_since = None
                if self._phase_tick(now, ready):
                    self.state = RUNNING
                elif self.start_failed is not None:
                    self.get_logger().error(f'SIM episode START FAILED: {self.start_failed} '
                                            f'-> no obstacle, run aborted')
                    self.state = DONE
                    self.exit_code = 3
            elif self.state == RUNNING:
                self._record_placement()
        elif self.state == WAITING:
            if self._ready(now):
                self.ready_since = self.ready_since if self.ready_since is not None else now
                if now - self.ready_since >= self.warmup_s:
                    self.state = RUNNING
                    self.t_start = now + self.start_lead_s + self.start_offset_s
                    self.get_logger().info(f'SIM episode START at t={self.t_start:.4f} s (sim), '
                                           f'announced at {now:.3f}')
            else:
                self.ready_since = None
        if self.state == RUNNING and self._n_start < 50:
            self.pub_start.publish(self._Float64(data=float(self.t_start)))
            self._n_start += 1
        if self.state == RUNNING and self.duration_s > 0 and now - self.t_start >= self.duration_s:
            self.state = DONE
            self.get_logger().info(f'SIM episode DONE at t={now:.3f} s (sim)')
        self.pub.publish(Float64MultiArray(data=[self.state, self.t_start or -1.0, now]))
        if self.state == DONE and not self.finished:
            self.finalise()
            self.finished = True
            raise SystemExit(self.exit_code)

    def finalise(self):
        rd = self.run_dir
        reflex = {}
        try:
            with open(os.path.join(rd, 'reflex_summary.json')) as f:
                reflex = json.load(f)
        except (OSError, ValueError):
            pass
        stray = []
        if self.hw_dir:
            new = sorted(set(os.listdir(self.hw_dir)) - self.hw_before)
            for name in new:
                src = os.path.join(self.hw_dir, name)
                if os.path.getmtime(src) >= self.t_wall0 - 1.0:
                    dst_dir = os.path.join(rd, 'stray_logs')
                    os.makedirs(dst_dir, exist_ok=True)
                    shutil.move(src, os.path.join(dst_dir, name))
                    stray.append(name)
            if stray:
                self.get_logger().error(f'SIM output appeared in {self.hw_dir}: {stray} '
                                        f'-> moved to {rd}/stray_logs')
        episode = {'t_start_sim': self.t_start, 't_end_sim': self._now(),
                   'controller_active_at_start': bool(self.controller_active),
                   'duration_s': self.duration_s, 'warmup_s': self.warmup_s,
                   'wall_s': time.time() - self.t_wall0}
        if self.start_mode == 'phase':
            if self.state == RUNNING or self.t_start is not None:
                self._record_placement()
            q = self.qual.summary() if self.qual is not None else None
            episode['start'] = dict(
                mode='phase', status=('start_failed' if self.start_failed is not None else
                                      'started' if self.t_start is not None else 'not_started'),
                fail_reason=self.start_failed, start_offset_s_not_applied=self.start_offset_s,
                commander_node=self.commander, commander_params=self.commander_params,
                qualifier=q, placement=self.placement, controller_history=self._ctrl_hist,
                hard_reset_log_lines=self._hard_log[:200],
                first_sim_tick=self.t_first_tick, deadline_s=self.start_cfg.timeout_s,
                controller_status=dict(self.ctrl_stats, poll_s=self.ctrl_poll_s,
                                       timeout_s=self.ctrl_timeout_s,
                                       max_age_s=self.ctrl_max_age_s),
                clock_stall_wall_s=self.clock_stall_wall_s)
        sim_label = {'plant': 'gazebo', 'SIM': True, 'git_sim': git_provenance(),
                     'rtf': reflex.get('rtf'), 'reflex_verdict': reflex.get('verdict'),
                     'reflex_counts': reflex.get('counts'), 'episode': episode,
                     'stray_hardware_logs': stray}
        p = os.path.join(rd, 'run_manifest_sim.json')
        try:
            with open(p) as f:
                man = json.load(f)
        except (OSError, ValueError):
            man = {}
        man.update(sim_label)
        with open(p, 'w') as f:
            json.dump(man, f, indent=2, sort_keys=True, default=str)
        for lm in glob.glob(os.path.join(rd, 'logger', '*', 'run_manifest.json')):
            try:
                with open(lm) as f:
                    m = json.load(f)
                m.update({'plant': 'gazebo', 'SIM': True, 'rtf': reflex.get('rtf'),
                          'git_sim': sim_label['git_sim'],
                          'sim_run_manifest': p})
                with open(lm, 'w') as f:
                    json.dump(m, f, indent=2, sort_keys=True, default=str)
            except (OSError, ValueError) as exc:
                self.get_logger().warn(f'could not label {lm}: {exc}')


def main():
    rclpy.init()
    node = EpisodeSupervisor()
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as e:
        code = e.code or 0
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        if not node.finished:
            node.finalise()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if code:
        sys.exit(code)


if __name__ == '__main__':
    main()
