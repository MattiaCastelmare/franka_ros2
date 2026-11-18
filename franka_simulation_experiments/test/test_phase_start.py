"""H2 phase-aligned episode start (simexp/phase_start.py, episode_supervisor start_mode:=phase).

No Gazebo, no launch. The commander is emulated with ITS OWN path and timing classes
(franka_experiments.utils.trajectory / timing_law, imported read-only) and its own phase
integration rule (actual_dt clipped to [dt/2, 2 dt], scaled by the governor), so the phase the
qualifier recovers can be compared with the commander's true s.
"""
import json
import math

import numpy as np
import pytest

from simexp import phase_start as ps

pytest.importorskip('franka_experiments.utils.trajectory')
from franka_experiments.utils.timing_law import LinearTimingLaw  # noqa: E402
from franka_experiments.utils.trajectory import CompositePath, LinearSegmentPath  # noqa: E402

# E3's commander, as its start-up banner reports it (path_type circle in fr3_control.yaml)
E3_PARAMS = dict(path_type='circle', center_xyz=[0.4, 0.0, 0.45], radius=0.25, plane='yz',
                 cycle_time=6.0, smoothness=0.2, lissajous_a=0.12, lissajous_b=0.08,
                 approach_time=2.5, timing_law='linear', hard_reset_thr=0.05, soft_reset_thr=0.02,
                 governor_enabled=True)
P_HOME = np.array([0.3222, 0.0097, 0.4608])         # EE at q0 (E3-03 banner, first APPROACH line)
DT = 0.01                                            # commander rate_hz 100


@pytest.fixture(scope='module')
def est():
    return ps.PhaseEstimator(ps.build_cycle_path(E3_PARAMS))


class Commander:
    """pentagon_qddot_commander's phase law: CompositePath + LinearTimingLaw, dt clamped."""

    def __init__(self, t_start, params=E3_PARAMS):
        T, ta = params['cycle_time'], params['approach_time']
        self.cyc = ps.build_cycle_path(params)
        self.Sa = ta / T
        self.path = CompositePath([(LinearSegmentPath(P_HOME, self.cyc.position(0.0).copy(),
                                                      span=self.Sa), self.Sa), (self.cyc, math.inf)])
        self.timing = LinearTimingLaw(rate=1.0 / T, soft_start_s=ta)
        self.timing.reset(0.0)
        self.t_prev = None
        self.t_start = t_start
        self.s = 0.0

    def tick(self, t, sigma=1.0):
        """Returns p_d (None before the trajectory starts)."""
        if t < self.t_start:
            return None
        dt = DT if self.t_prev is None else float(np.clip(t - self.t_prev, DT * 0.5, DT * 2.0))
        self.t_prev = t
        self.s, _, _ = self.timing.step(dt * sigma)
        return np.asarray(self.path.position(self.s), float).copy()

    def phi(self):
        return None if self.s < self.Sa else (self.s - self.Sa) % 1.0


def run(est, *, t_traj=8.4, t_end=80.0, ctrl_on=lambda t: True, arm=None, sample_ok=None,
        tick_gap=lambda t, rng: DT, poll_gap=lambda t, rng: 0.02, cfg=None, sigma=lambda t: 1.0,
        seed=0):
    """Emulated run. ``arm(t, p_d, state)`` -> measured EE; default = 3 mm lag along the path.

    Returns (qualifier, log of (t, s, phi_true)). Polls at the supervisor's own (jittered) sim
    ticks; ee pairs arrive at the commander's (jittered) ticks.
    """
    rng = np.random.default_rng(seed)
    cfg = cfg or ps.StartConfig()
    cfg.cycle_time = E3_PARAMS['cycle_time']
    q = ps.StartQualifier(cfg=cfg, est=est)
    cmd = Commander(t_traj)
    st = {'p': P_HOME.copy()}
    t, next_poll, log = 3.0, 3.0, []
    while t < t_end:
        pd = cmd.tick(t, sigma(t))
        if pd is not None:
            if arm is None:
                pa = pd + np.array([0.0, 0.0, 0.003])
            else:
                pa = arm(t, pd, st)
            if sample_ok is None or sample_ok(t):
                q.feed(t, pd, pa, ctrl_on(t))
            log.append((t, cmd.s, cmd.phi()))
        while next_poll <= t:
            q.poll(next_poll)
            next_poll += poll_gap(next_poll, rng)
        if q.placement is not None or q.state == 'failed':
            break
        t = round(t + tick_gap(t, rng), 9)
    return q, log


def true_phi_at(log, t):
    """Commander's true curve phase at sim time t (linear between its ticks)."""
    ts = np.array([x[0] for x in log])
    ss = np.array([x[1] for x in log])
    s = float(np.interp(t, ts, ss))
    return (s - 2.5 / 6.0) % 1.0


# ---------------------------------------------------------------------------- the estimator
def test_estimator_recovers_the_commander_phase_exactly(est):
    cmd = Commander(0.0)
    worst, lead_in_on_path = 0.0, 0
    for k in range(int(20.0 / DT)):
        pd = cmd.tick(k * DT)
        phi, d = est.locate(pd)
        if cmd.phi() is None:
            lead_in_on_path += d <= ps.StartConfig().on_path_tol
            continue
        worst = max(worst, abs(ps.wrap(phi - cmd.phi())))
        assert d < 1e-6          # chord sagitta of the 10000-point table: r (pi/n)^2 / 2 = 1.2e-8 m
    assert worst < 1e-7
    # the straight lead-in is NOT the curve: only its last few samples come within 1 mm of P(0)
    assert lead_in_on_path <= 5


def test_phase0_is_the_curve_entry_and_farthest_from_the_e3_ball(est):
    """phase 0 = P(0) = (0.4, 0.25, 0.45), the commander's curve entry; the E3 ball sits beside
    phase 0.5 (0.4, -0.25, 0.45), so phase 0 is the curve point farthest from it."""
    assert np.allclose(est.tab[0], [0.4, 0.25, 0.45], atol=1e-12)
    ball = np.array([0.55, -0.25, 0.449])
    d = np.linalg.norm(est.tab - ball, axis=1)
    assert abs(int(np.argmax(d)) / est.n) < 0.01 or abs(int(np.argmax(d)) / est.n - 1) < 0.01
    assert abs(int(np.argmin(d)) / est.n - 0.5) < 0.01


# ---------------------------------------------------------------------------- start sequence
def test_normal_startup(est):
    q, log = run(est)
    d = q.decision
    assert q.state == 'announced' and d is not None
    # never before one full clean cycle on the curve: lead-in 2.5 s + one 6 s cycle
    assert d['t_announce'] >= 8.4 + 2.5 + 6.0
    assert 0.25 - 1e-9 <= d['lead_s'] <= 0.5 + 0.02
    assert d['window_cycles'] >= 1.0 and d['window_max_err_m'] <= 0.02
    assert abs(d['rate_over_nominal'] - 1.0) < 1e-3
    assert abs(ps.wrap(true_phi_at(log, d['t_start']) - 0.0)) < 1e-3
    assert abs(q.placement['phase_error_cycles']) < 1e-3
    assert q.hard_ticks == 0 and q.placement['lead_violations'] == {}


@pytest.mark.parametrize('delay', [0.0, 3.0, 6.1, 10.0, 17.3])
def test_delayed_controller_activation_does_not_move_the_phase(est, delay):
    """Spawner retry (E3-04): the path advances while the arm cannot move; then it catches up.
    The birth phase must stay phase0 and must come one clean cycle after the catch-up."""
    t_on = 8.4 + delay

    def arm(t, pd, st):
        if t < t_on:
            return st['p'].copy()                                 # unpowered: frozen
        st['p'] += np.clip(pd - st['p'], -0.0025, 0.0025)        # catch-up, then tracks
        return st['p'].copy()
    q, log = run(est, ctrl_on=lambda t: t >= t_on, arm=arm)
    assert q.state == 'announced'
    assert abs(ps.wrap(true_phi_at(log, q.decision['t_start']))) < 1e-3
    assert q.decision['window_t0'] >= t_on
    if delay >= 6.0:
        assert q.hard_ticks > 0                                   # E3-04's HARD-reset condition
        assert q.resets.get('tracking_error', 0) > 0
        assert q.decision['hard_reset_ticks_before_announce'] == q.hard_ticks
        assert q.decision['hard_reset_last_t'] < q.decision['window_t0']    # resolved, then a cycle


def test_spawner_never_activates_the_controller_fails_explicitly(est):
    cfg = ps.StartConfig(timeout_s=40.0)
    q, _ = run(est, ctrl_on=lambda t: False, arm=lambda t, pd, st: st['p'].copy(), cfg=cfg)
    assert q.state == 'failed' and q.decision is None and q.placement is None
    assert 'within 40.0 s' in q.fail_reason and 'controller_inactive' in q.fail_reason


def test_controller_drops_out_mid_window_restarts_it(est):
    q, log = run(est, ctrl_on=lambda t: not (15.0 <= t < 15.3))
    assert q.resets.get('controller_inactive', 0) > 0
    assert q.decision['window_t0'] >= 15.3
    assert abs(ps.wrap(true_phi_at(log, q.decision['t_start']))) < 1e-3


def test_no_trajectory_at_all_fails_explicitly(est):
    cfg = ps.StartConfig(timeout_s=30.0)
    q, _ = run(est, sample_ok=lambda t: False, cfg=cfg, t_end=60.0)
    assert q.state == 'failed' and q.n_pairs == 0
    assert 'no ee_desired/ee_actual pair' in q.fail_reason


def test_stale_trajectory_restarts_the_window(est):
    q, log = run(est, sample_ok=lambda t: not (16.0 <= t < 16.25))
    assert q.resets.get('stale_trajectory', 0) >= 1
    assert q.decision['window_t0'] >= 16.25
    assert abs(ps.wrap(true_phi_at(log, q.decision['t_start']))) < 1e-3


def test_stale_at_poll_time_is_detected_without_new_samples(est):
    """The poll itself notices a silent commander (no sample arrives to trigger the check)."""
    cfg = ps.StartConfig()
    cfg.cycle_time = 6.0
    q = ps.StartQualifier(cfg=cfg, est=est)
    cmd = Commander(0.0)
    for k in range(int(12.0 / DT)):
        t = 5.0 + k * DT
        pd = cmd.tick(t)
        q.feed(t, pd, pd, True)
    q.poll(t + 0.5)
    assert q.win is None and q.resets.get('stale_trajectory') == 1


def test_off_curve_samples_are_rejected(est):
    """A desired point off the commander's curve (wrong geometry, lead-in) never qualifies."""
    shifted = dict(E3_PARAMS, center_xyz=[0.4, 0.0, 0.46])     # supervisor rebuilt the WRONG curve
    wrong = ps.PhaseEstimator(ps.build_cycle_path(shifted))
    q, _ = run(wrong, cfg=ps.StartConfig(timeout_s=40.0))
    assert q.state == 'failed' and q.resets.get('not_on_cycle', 0) > 0


@pytest.mark.parametrize('err,expect', [(0.019, 'announced'), (0.021, 'failed')])
def test_tracking_error_threshold(est, err, expect):
    q, _ = run(est, arm=lambda t, pd, st: pd + np.array([0.0, err, 0.0]),
               cfg=ps.StartConfig(timeout_s=40.0))
    assert q.state == expect
    if expect == 'failed':
        assert q.resets.get('tracking_error', 0) > 0


def test_tracking_error_spike_restarts_the_window(est):
    q, log = run(est, arm=lambda t, pd, st: pd + (np.array([0, 0.03, 0]) if 17.0 <= t < 17.05
                                                  else np.zeros(3)))
    assert q.resets.get('tracking_error') == 5
    assert q.decision['window_t0'] >= 17.05
    assert abs(ps.wrap(true_phi_at(log, q.decision['t_start']))) < 1e-3


def test_hard_reset_during_startup_is_counted_and_must_be_followed_by_a_clean_cycle(est):
    q, log = run(est, arm=lambda t, pd, st: pd + (np.array([0, 0.08, 0]) if 13.0 <= t < 13.2
                                                  else np.zeros(3)))
    assert q.hard_ticks == 20 and q.hard_last_t < 13.2
    assert q.decision['window_t0'] >= 13.2
    assert q.decision['window_t0'] + 6.0 <= q.decision['t_announce'] + 1e-9
    assert q.placement['hard_reset_ticks_total'] == 20


@pytest.mark.parametrize('phase0', [0.0, 0.25, 0.5, 0.731, 0.999])
def test_phase_alignment(est, phase0):
    q, log = run(est, cfg=ps.StartConfig(phase0=phase0))
    assert abs(ps.wrap(true_phi_at(log, q.decision['t_start']) - phase0)) < 1e-3
    assert abs(ps.wrap(q.placement['phase_at_start'] - phase0)) < 1e-3


@pytest.mark.parametrize('name,tick_gap,poll_gap', [
    ('rtf~1, regular', lambda t, r: DT, lambda t, r: 0.02),
    ('low rtf: jittered commander and supervisor ticks',
     lambda t, r: DT * r.uniform(0.6, 1.9), lambda t, r: 0.02 * r.uniform(1.0, 3.0)),
    ('starved supervisor: polls every 40-90 ms sim',
     lambda t, r: DT, lambda t, r: r.uniform(0.04, 0.09)),
])
def test_real_time_factor_cannot_change_the_birth_phase(est, name, tick_gap, poll_gap):
    """Only sim stamps enter the decision. A lower RTF shows up as jitter of the sim spacing of
    both nodes' ticks; the birth phase must not move."""
    q, log = run(est, tick_gap=tick_gap, poll_gap=poll_gap, seed=3)
    assert q.state == 'announced', name
    assert abs(ps.wrap(true_phi_at(log, q.decision['t_start']))) < 2e-3, name
    assert q.decision['lead_s'] >= 0.25 - 1e-9, name


def test_commander_losing_phase_to_late_ticks_is_not_announced(est):
    """Ticks 25 ms apart in sim time exceed the commander's 2*dt clamp: its phase then runs at
    80 % of nominal. The prediction would be wrong, so nothing is announced and the run fails."""
    q, _ = run(est, tick_gap=lambda t, r: 0.025, cfg=ps.StartConfig(timeout_s=40.0))
    assert q.state == 'failed' and q.resets.get('phase_rate_off_nominal', 0) > 0


def test_governor_slowing_blocks_the_announcement(est):
    q, _ = run(est, sigma=lambda t: 0.5, cfg=ps.StartConfig(timeout_s=40.0))
    assert q.state == 'failed' and q.resets.get('phase_rate_off_nominal', 0) > 0


def test_timeout_is_measured_from_the_first_poll_in_sim_time(est):
    cfg = ps.StartConfig(timeout_s=10.0)
    cfg.cycle_time = 6.0
    q = ps.StartQualifier(cfg=cfg, est=est)
    assert q.poll(0.0) is None and q.t_first_poll is None       # no /clock yet: not counted
    q.poll(4.0)
    q.poll(14.0)
    assert q.state == 'qualifying'
    q.poll(14.0001)
    assert q.state == 'failed'


def test_readiness_holds_the_announcement_but_not_the_timeout(est):
    cfg = ps.StartConfig(timeout_s=25.0)
    cfg.cycle_time = 6.0
    q = ps.StartQualifier(cfg=cfg, est=est)
    cmd = Commander(0.0)
    for k in range(int(30.0 / DT)):
        t = 3.0 + k * DT
        pd = cmd.tick(t)
        q.feed(t, pd, pd, True)
        q.poll(t, allow=False)
    assert q.decision is None and q.state == 'failed'


def test_lead_violation_is_recorded_not_hidden(est):
    q, _ = run(est, arm=lambda t, pd, st: pd)
    # rerun with a violation exactly inside the lead of that announcement
    t_ann, t_start = q.decision['t_announce'], q.decision['t_start']
    q2, _ = run(est, arm=lambda t, pd, st: pd + (np.array([0, 0.03, 0]) if t_ann + 0.005 < t < t_start
                                                 else np.zeros(3)))
    assert q2.decision['t_start'] == pytest.approx(t_start, abs=1e-9)
    assert q2.placement['lead_violations'].get('tracking_error', 0) > 0


def test_summary_is_json_serialisable(est):
    q, _ = run(est)
    json.dumps(q.summary())


# ---------------------------------------------------------------------------- supervisor glue
@pytest.fixture
def make_supervisor(tmp_path, monkeypatch):
    """EpisodeSupervisor instances with parameter overrides; never spun, no other node needed."""
    rclpy = pytest.importorskip('rclpy')
    import rclpy.node
    from rclpy.parameter import Parameter
    import episode_supervisor as es
    monkeypatch.setattr(es, 'hardware_log_dir', lambda: None)
    rclpy.init()
    nodes = []
    real_init = rclpy.node.Node.__init__

    def make(**params):
        params.setdefault('run_dir', str(tmp_path))

        def init_with_params(self, name, **kw):
            kw['parameter_overrides'] = [Parameter(k, value=v) for k, v in params.items()]
            real_init(self, name, **kw)
        monkeypatch.setattr(rclpy.node.Node, '__init__', init_with_params)
        try:
            n = es.EpisodeSupervisor()
        finally:
            monkeypatch.setattr(rclpy.node.Node, '__init__', real_init)
        nodes.append(n)
        return n
    yield make, es
    for n in nodes:
        n.destroy_node()
    rclpy.shutdown()


def test_legacy_mode_is_the_default_and_adds_nothing(make_supervisor):
    make, es = make_supervisor
    n = make(perception=True)
    assert n.start_mode == 'legacy'
    assert not hasattr(n, 'qual')
    subs = sorted(s.topic_name for s in n.subscriptions)
    assert subs == ['/NS_1/qddot_nom', '/NS_1/qddot_safe', '/NS_1/torque_cmd',
                    '/cbf/per_link_distances']
    assert sorted(p.topic_name for p in n.publishers if not p.topic_name.startswith('/parameter')
                  and p.topic_name != '/rosout') == ['/sim/episode', '/sim/episode_start']


def test_legacy_start_time_is_unchanged(make_supervisor):
    """legacy: t_start = (ready + warmup_s) + start_lead_s + start_offset_s, as at HEAD."""
    make, es = make_supervisor
    n = make(perception=False, warmup_s=2.0, start_offset_s=0.004, duration_s=0.0)
    clock = {'t': 10.0}
    n._now = lambda: clock['t']
    n.controller_active = True
    published = []
    n.pub_start.publish = lambda m: published.append(m.data)
    n.pub.publish = lambda m: None
    for k in range(200):
        clock['t'] = 10.0 + 0.02 * k
        for key in ('qddot_nom', 'qddot_safe', 'torque_cmd'):
            n.seen[key] = clock['t']
        n._tick()
    assert n.state == es.RUNNING
    assert n.t_start == pytest.approx(10.0 + 2.0 + 0.25 + 0.004, abs=1e-9)
    assert published and published[0] == n.t_start


def test_phase_mode_subscribes_and_fails_explicitly_with_exit_code_3(make_supervisor, tmp_path):
    make, es = make_supervisor
    with open(tmp_path / 'run_manifest_sim.json', 'w') as f:
        json.dump({'SIM': True}, f)
    n = make(perception=True, start_mode='phase', start_timeout_s=5.0, duration_s=30.0)
    subs = sorted(s.topic_name for s in n.subscriptions)
    for t in ('/NS_1/ee_desired', '/NS_1/ee_actual', '/NS_1/franka/joint_states', '/rosout'):
        assert t in subs
    clock = {'t': 4.0}
    n._now = lambda: clock['t']
    published = []
    n.pub_start.publish = lambda m: published.append(m.data)
    n.pub.publish = lambda m: None
    with pytest.raises(SystemExit) as ex:
        for k in range(1000):
            clock['t'] = 4.0 + 0.02 * k
            n._tick()
    assert ex.value.code == 3
    assert published == []                                      # the obstacle is never born
    man = json.load(open(tmp_path / 'run_manifest_sim.json'))
    st = man['episode']['start']
    assert st['status'] == 'start_failed' and 'commander parameters not received' in st['fail_reason']
    assert man['episode']['t_start_sim'] is None


def test_phase_mode_end_to_end_with_fake_commander(make_supervisor, tmp_path, est):
    """Supervisor glue: parameter parsing, ee pairing by stamp, announcement, placement record."""
    from geometry_msgs.msg import PointStamped
    from rcl_interfaces.msg import ParameterType as PT, ParameterValue
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import JointState
    make, es = make_supervisor
    with open(tmp_path / 'run_manifest_sim.json', 'w') as f:
        json.dump({'SIM': True}, f)
    n = make(perception=False, start_mode='phase', duration_s=1.0, warmup_s=2.0)
    clock = {'t': 3.0}
    n._now = lambda: clock['t']
    n.pub.publish = lambda m: None
    published = []
    n.pub_start.publish = lambda m: published.append(m.data)

    class Fut:
        def __init__(self, r):
            self.r = r

        def done(self):
            return True

        def result(self):
            return self.r
    res = GetParameters.Response()
    for name in n._par_req.names:
        v = E3_PARAMS[name]
        pv = ParameterValue()
        if isinstance(v, bool):
            pv.type, pv.bool_value = PT.PARAMETER_BOOL, v
        elif isinstance(v, str):
            pv.type, pv.string_value = PT.PARAMETER_STRING, v
        elif isinstance(v, list):
            pv.type, pv.double_array_value = PT.PARAMETER_DOUBLE_ARRAY, v
        else:
            pv.type, pv.double_value = PT.PARAMETER_DOUBLE, float(v)
        res.values.append(pv)
    n._par_fut = Fut(res)
    n._poll_controller_continuous = lambda now: setattr(n, 'controller_active', True)
    n.controller_active = True
    cmd = Commander(8.4)

    def msg(t, p):
        m = PointStamped()
        m.header.stamp.sec, m.header.stamp.nanosec = int(t), int(round((t - int(t)) * 1e9))
        m.point.x, m.point.y, m.point.z = (float(x) for x in p)
        return m
    log = []
    with pytest.raises(SystemExit) as ex:
        for k in range(int(40.0 / DT)):
            t = round(3.0 + k * DT, 6)
            clock['t'] = t
            for key in ('qddot_nom', 'qddot_safe', 'torque_cmd'):
                n.seen[key] = t
            pd = cmd.tick(t)
            if pd is not None:
                m_d, m_a = msg(t, pd), msg(t, pd + [0, 0, 0.003])
                t_exact = m_d.header.stamp.sec + m_d.header.stamp.nanosec * 1e-9
                if k % 2:                                        # either arrival order
                    n._ee_cb('a', m_a)
                    n._ee_cb('d', m_d)
                else:
                    n._ee_cb('d', m_d)
                    n._ee_cb('a', m_a)
                log.append((t_exact, cmd.s, cmd.phi()))
                js = JointState()
                js.header.stamp = m_d.header.stamp
                js.name = [f'fr3_joint{i}' for i in (1, 3, 6, 7, 2, 4, 5)]   # scrambled, as recorded
                js.position = [float(i) for i in (1, 3, 6, 7, 2, 4, 5)]
                js.velocity = [0.1 * i for i in (1, 3, 6, 7, 2, 4, 5)]
                n._js_cb(js)
            if k % 2 == 0:
                n._tick()
    assert ex.value.code == 0
    assert published and published[0] == pytest.approx(n.t_start)
    assert abs(ps.wrap(true_phi_at(log, n.t_start))) < 1e-3
    man = json.load(open(tmp_path / 'run_manifest_sim.json'))
    st = man['episode']['start']
    assert st['status'] == 'started' and st['commander_params']['path_type'] == 'circle'
    pl = st['placement']
    assert pl['joint_state_before']['q'] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]  # mapped by name
    assert pl['joint_state_before']['qdot'] == pytest.approx([0.1 * i for i in range(1, 8)])
    assert abs(pl['phase_error_cycles']) < 1e-3
    assert man['episode']['t_start_sim'] == pytest.approx(n.t_start)


# ---------------------------------------------------------------------------- batch / CLI plumbing
def test_batch_forwards_episode_start_only_when_set(tmp_path, monkeypatch):
    from simexp import batch
    seen = []
    monkeypatch.setattr(batch.runner, 'run_cell', lambda rd, **kw: seen.append(kw) or dict(
        wall_s=1.0, timed_out=False, leftovers=[], returncode=0))
    monkeypatch.setattr(batch, 'score_cell', lambda rd, c: {'closed_loop': {}})
    base = dict(scenario='pentagon_static_baseline_bare', condition='full', seed=0, perception='true',
                sets={'executor_mode': 'multi'})
    batch.run_one(str(tmp_path / 'a'), dict(base))
    batch.run_one(str(tmp_path / 'b'), dict(base, episode_start='phase', start_phase=0.0,
                                            start_timeout_s=90.0))
    assert seen[0]['extra_args'] is None                          # legacy: exactly as before
    assert seen[1]['extra_args'] == {'episode_start': 'phase', 'start_phase': '0',
                                     'start_timeout_s': '90'}
    assert batch.cell_name(base) == batch.cell_name(dict(base, episode_start='phase'))


# ---------------------------------------------------------------------------- F-8-23 / F-8-24
# Controller-status polling with a timeout and a freshness bound; one startup deadline from the
# supervisor's first sim tick; sim-clock liveness. Every test that can end without a birth
# checks that /sim/episode_start was never published (the obstacle stays parked).

def _lc_response(active, name='rt_torque_controller'):
    from controller_manager_msgs.msg import ControllerState
    from controller_manager_msgs.srv import ListControllers
    r = ListControllers.Response()
    c = ControllerState()
    c.name, c.state = name, ('active' if active else 'inactive')
    r.controller.append(c)
    return r


class FakeListClient:
    """list_controllers client. ``answer(t_sent)`` -> None (never answered) or (delay_s, active).

    Futures are real rclpy Futures. A cancelled future is never delivered, as in rclpy (the
    client removes it from its pending requests); ``late()`` forces a late result anyway.
    """

    def __init__(self, answer=lambda t: (0.0, True), ready=True):
        self.answer, self.ready = answer, ready
        self.pending, self.futs, self.now = [], [], 0.0

    def service_is_ready(self):
        return self.ready

    def call_async(self, req):
        from rclpy.task import Future
        f = Future()
        self.futs.append(f)
        a = self.answer(self.now)
        if a is not None:
            self.pending.append((self.now + a[0], f, a[1]))
        return f

    def deliver(self, now):
        self.now = now
        for item in list(self.pending):
            due, f, act = item
            if due <= now + 1e-12:
                self.pending.remove(item)
                if not f.cancelled():
                    f.set_result(_lc_response(act))

    def in_flight(self):
        return sum(1 for f in self.futs if not f.done() and not f.cancelled())


def _phase(make, cli=None, **kw):
    kw.setdefault('perception', False)
    n = make(start_mode='phase', **kw)
    n._list_cli = cli or FakeListClient()
    return n, n._list_cli


def _step(n, cli, t):
    cli.deliver(t)
    n._poll_controller_continuous(t)
    assert cli.in_flight() <= 1                     # never more than one request in flight


def test_ctrl_normal_response_is_active_while_answers_are_fresh(make_supervisor):
    make, es = make_supervisor
    n, cli = _phase(make)
    assert n.ctrl_timeout_s == 1.0 and n.ctrl_max_age_s == 0.5 and n.ctrl_poll_s == 0.2
    for k in range(200):
        _step(n, cli, round(10.0 + 0.02 * k, 6))
        if k >= 1:
            assert n.controller_active
    assert n.ctrl_stats['timeouts'] == 0 and n.ctrl_stats['stale'] == 0
    assert n.ctrl_stats['sent'] == pytest.approx(4.0 / 0.2, abs=2)      # one request per 0.2 s


def test_ctrl_request_never_answered_is_cancelled_and_asked_again(make_supervisor):
    make, es = make_supervisor
    n, cli = _phase(make, FakeListClient(answer=lambda t: None))
    for k in range(250):                                                # 5 s of sim time
        _step(n, cli, round(10.0 + 0.02 * k, 6))
        assert not n.controller_active
    st = n.ctrl_stats
    assert st['answered'] == 0 and st['timeouts'] >= 4
    assert st['sent'] == st['timeouts'] + 1                             # the last one still in flight
    assert all(f.cancelled() for f in cli.futs[:-1]) and not cli.futs[-1].cancelled()


def test_ctrl_delayed_response_within_the_timeout_is_accepted(make_supervisor):
    make, es = make_supervisor
    n, cli = _phase(make, FakeListClient(answer=lambda t: (0.3, True)))
    _step(n, cli, 10.0)
    for t in (10.1, 10.2, 10.28):
        _step(n, cli, t)
        assert not n.controller_active                                 # nothing known yet
    _step(n, cli, 10.32)
    assert n.controller_active and n.ctrl_stats['timeouts'] == 0
    assert n._ctrl_resp_t == 10.0                                      # aged from the request


def test_ctrl_response_older_than_the_freshness_bound_is_answered_but_not_active(make_supervisor):
    """A reply 0.8 s after its request (< 1 s timeout) describes a state >= 0.8 s old > 0.5 s."""
    make, es = make_supervisor
    n, cli = _phase(make, FakeListClient(answer=lambda t: (0.8, True)))
    for k in range(150):
        _step(n, cli, round(10.0 + 0.02 * k, 6))
        assert not n.controller_active
    assert n.ctrl_stats['answered'] >= 2 and n.ctrl_stats['timeouts'] == 0


def test_ctrl_late_response_after_the_timeout_is_ignored(make_supervisor):
    make, es = make_supervisor
    n, cli = _phase(make, FakeListClient(answer=lambda t: None))
    for k in range(53):
        _step(n, cli, round(10.0 + 0.02 * k, 6))                        # to 11.04: first request cancelled
    first = cli.futs[0]
    assert first.cancelled() and n.ctrl_stats['timeouts'] == 1 and n._list_fut is cli.futs[1]
    first.set_result(_lc_response(True))                                # worst case: it arrives anyway
    _step(n, cli, 11.06)
    assert not n.controller_active and n.ctrl_stats['answered'] == 0


def test_cancel_removes_the_request_from_the_real_rclpy_client(make_supervisor):
    """Late responses are dropped by rclpy itself: after cancel() the sequence number is gone,
    so Executor._execute_client takes its KeyError branch ('The request was cancelled')."""
    make, es = make_supervisor
    n = make(perception=False, start_mode='phase')
    cli = n._list_cli                                                   # the real rclpy client
    f = cli.call_async(n._list_req)
    seqs = list(cli._pending_requests)
    assert len(seqs) == 1
    n._list_fut, n._list_sent_t = f, 10.0
    n._poll_controller_continuous(11.5)                                 # 1.5 s > timeout
    assert f.cancelled() and n._list_fut is None and n.ctrl_stats['timeouts'] == 1
    assert cli._pending_requests == {}
    with pytest.raises(KeyError):
        cli.get_pending_request(seqs[0])


def test_ctrl_becoming_inactive_after_active_is_seen_at_the_next_answer(make_supervisor):
    make, es = make_supervisor
    n, cli = _phase(make, FakeListClient(answer=lambda t: (0.0, t < 12.0)))
    seen = []
    for k in range(200):
        t = round(10.0 + 0.02 * k, 6)
        _step(n, cli, t)
        seen.append((t, n.controller_active))
    t_off = min(t for t, a in seen if t > 10.1 and not a)
    assert 12.0 < t_off <= 12.0 + 0.22 + 0.02 + 1e-9                    # within one poll spacing + one tick
    assert [h[1] for h in n._ctrl_hist] == [False, True, False]


def test_ctrl_stale_active_answer_is_not_active(make_supervisor):
    """The last answer said 'active', then the controller manager goes silent."""
    make, es = make_supervisor
    n, cli = _phase(make, FakeListClient(answer=lambda t: (0.0, True) if t < 10.1 else None))
    for k in range(60):
        t = round(10.0 + 0.02 * k, 6)
        _step(n, cli, t)
        if n._ctrl_resp_t is None:
            assert not n.controller_active
            continue
        assert n.controller_active == (t - n._ctrl_resp_t <= 0.5)
    assert not n.controller_active and n._ctrl_resp_active              # stale, although it said active
    assert n.ctrl_stats['stale'] == 1 and 'stale answer' in n._ctrl_hist[-1][2]


def test_ctrl_repeated_polling_never_accumulates_requests(make_supervisor):
    make, es = make_supervisor
    rng = np.random.default_rng(3)

    def answer(t):
        u = rng.uniform()
        return None if u < 0.3 else (float(rng.uniform(0.0, 1.6)), bool(u < 0.9))
    n, cli = _phase(make, FakeListClient(answer=answer))
    for k in range(3000):                                               # 60 s of sim time
        _step(n, cli, round(10.0 + 0.02 * k, 6))
        st = n.ctrl_stats
        assert st['sent'] - st['answered'] - st['timeouts'] in (0, 1)
        if n.controller_active:
            assert n._ctrl_resp_active
            assert 0.0 <= round(10.0 + 0.02 * k, 6) - n._ctrl_resp_t <= 0.5 + 1e-9
    assert n.ctrl_stats['timeouts'] > 10 and n.ctrl_stats['answered'] > 10    # both paths exercised


# ---------------------------------------------------------------------------- supervisor-level runs
def _e3_param_future(names):
    from rcl_interfaces.msg import ParameterType as PT, ParameterValue
    from rcl_interfaces.srv import GetParameters
    from rclpy.task import Future
    res = GetParameters.Response()
    for name in names:
        v = E3_PARAMS[name]
        pv = ParameterValue()
        if isinstance(v, bool):
            pv.type, pv.bool_value = PT.PARAMETER_BOOL, v
        elif isinstance(v, str):
            pv.type, pv.string_value = PT.PARAMETER_STRING, v
        elif isinstance(v, list):
            pv.type, pv.double_array_value = PT.PARAMETER_DOUBLE_ARRAY, v
        else:
            pv.type, pv.double_value = PT.PARAMETER_DOUBLE, float(v)
        res.values.append(pv)
    f = Future()
    f.set_result(res)
    return f


class FakeParClient:
    def __init__(self, names, ready_at, clock):
        self.names, self.ready_at, self.clock = names, ready_at, clock

    def service_is_ready(self):
        return self.clock['t'] >= self.ready_at

    def call_async(self, req):
        return _e3_param_future(self.names)


def _pt(t, p):
    from geometry_msgs.msg import PointStamped
    m = PointStamped()
    m.header.stamp.sec, m.header.stamp.nanosec = int(t), int(round((t - int(t)) * 1e9))
    m.point.x, m.point.y, m.point.z = (float(x) for x in p)
    return m


def _drive(n, cli, tmp_path, *, t0=4.0, t_end=200.0, params_at=4.0, cmd_start=8.4,
           err=lambda t: 0.003, feed=True):
    """Supervisor ticks at 50 Hz sim, commander at 100 Hz; returns (exit code, t, published)."""
    from sensor_msgs.msg import JointState
    with open(tmp_path / 'run_manifest_sim.json', 'w') as f:
        json.dump({'SIM': True}, f)
    clock = {'t': t0}
    n._now = lambda: clock['t']
    n.pub.publish = lambda m: None
    published = []
    n.pub_start.publish = lambda m: published.append(m.data)
    n._par_cli = FakeParClient(n._par_req.names, params_at, clock)
    cmd = Commander(cmd_start)
    k, code = 0, None
    try:
        while True:
            t = round(t0 + k * DT, 6)
            if t > t_end:
                break
            clock['t'] = t
            cli.deliver(t)
            for key in ('qddot_nom', 'qddot_safe', 'torque_cmd'):
                n.seen[key] = t
            pd = cmd.tick(t) if feed else None
            if pd is not None:
                n._ee_cb('d', _pt(t, pd))
                n._ee_cb('a', _pt(t, pd + np.array([0.0, 0.0, err(t)])))
                js = JointState()
                js.header.stamp = _pt(t, pd).header.stamp
                js.name = [f'fr3_joint{i}' for i in range(1, 8)]
                js.position, js.velocity = [0.0] * 7, [0.0] * 7
                n._js_cb(js)
            if k % 2 == 0:
                n._tick()
                assert cli.in_flight() <= 1
            k += 1
    except SystemExit as e:
        code = e.code
    return code, clock['t'], published


def _manifest_start(tmp_path):
    return json.load(open(tmp_path / 'run_manifest_sim.json'))['episode']['start']


def test_deadline_counts_from_the_first_tick_when_the_commander_answers_late(make_supervisor, tmp_path):
    """F-8-24: parameters at first tick + 20 s, no ee data -> fail at first tick + 30 s, not + 50 s."""
    make, es = make_supervisor
    n, cli = _phase(make, start_timeout_s=30.0, duration_s=1.0)
    code, t, published = _drive(n, cli, tmp_path, params_at=24.0, feed=False)
    assert code == 3 and published == []
    assert n.t_first_tick == 4.0 and n.qual is not None and n.qual.t_first_poll == 4.0
    assert 34.0 < t <= 34.05
    st = _manifest_start(tmp_path)
    assert st['status'] == 'start_failed' and st['first_sim_tick'] == 4.0 and st['deadline_s'] == 30.0
    assert 'deadline from the first sim tick 4.0' in st['fail_reason']


def test_deadline_without_commander_parameters_is_the_same_deadline(make_supervisor, tmp_path):
    make, es = make_supervisor
    n, cli = _phase(make, start_timeout_s=30.0, duration_s=1.0)
    code, t, published = _drive(n, cli, tmp_path, params_at=1e9, feed=False)
    assert code == 3 and published == [] and 34.0 < t <= 34.05
    assert 'commander parameters not received' in _manifest_start(tmp_path)['fail_reason']


def test_controller_activated_after_the_deadline_fails_with_the_obstacle_parked(make_supervisor, tmp_path):
    make, es = make_supervisor
    n, cli = _phase(make, FakeListClient(answer=lambda t: (0.0, t >= 40.0)),
                    start_timeout_s=30.0, duration_s=1.0)
    code, t, published = _drive(n, cli, tmp_path)
    assert code == 3 and published == [] and 34.0 < t <= 34.05
    st = _manifest_start(tmp_path)
    assert st['status'] == 'start_failed' and st['controller_status']['answered'] > 100
    assert st['qualifier']['window_restarts'].get('controller_inactive', 0) > 0


def test_delayed_controller_activation_within_the_deadline_starts_after_a_clean_cycle(make_supervisor, tmp_path):
    make, es = make_supervisor
    n, cli = _phase(make, FakeListClient(answer=lambda t: (0.0, t >= 20.0)),
                    start_timeout_s=60.0, duration_s=1.0)
    code, t, published = _drive(n, cli, tmp_path)
    assert code == 0 and published and published[0] == pytest.approx(n.t_start)
    assert n.t_start - 20.0 >= E3_PARAMS['cycle_time']                  # a full cycle after activation
    assert abs(n.qual.placement['phase_error_cycles']) < 1e-3
    st = _manifest_start(tmp_path)
    assert st['status'] == 'started' and st['placement']['controller_active_at_start'] is True


def test_controller_status_going_stale_blocks_the_start(make_supervisor, tmp_path):
    """Answers stop at t = 12 s (lost responses): the last 'active' must not carry the start."""
    make, es = make_supervisor
    n, cli = _phase(make, FakeListClient(answer=lambda t: (0.0, True) if t < 12.0 else None),
                    start_timeout_s=40.0, duration_s=1.0)
    code, t, published = _drive(n, cli, tmp_path)
    assert code == 3 and published == [] and 44.0 < t <= 44.05
    st = _manifest_start(tmp_path)
    assert st['controller_status']['timeouts'] > 20 and st['controller_status']['stale'] == 1


def test_tracking_never_established_keeps_the_obstacle_parked(make_supervisor, tmp_path):
    make, es = make_supervisor
    n, cli = _phase(make, start_timeout_s=40.0, duration_s=1.0)
    code, t, published = _drive(n, cli, tmp_path, err=lambda t: 0.03)
    assert code == 3 and published == []
    st = _manifest_start(tmp_path)
    assert st['qualifier']['window_restarts'].get('tracking_error', 0) > 0


# ---------------------------------------------------------------------------- sim-clock liveness
def _watch(n, tmp_path, sim, wall):
    with open(tmp_path / 'run_manifest_sim.json', 'w') as f:
        json.dump({'SIM': True}, f)
    n._now = lambda: sim['t']
    n._mono = lambda: wall['t']
    n.pub.publish = lambda m: None
    published = []
    n.pub_start.publish = lambda m: published.append(m.data)
    n._clock_last_adv_wall = wall['t']
    return published


def test_sim_clock_that_stops_fails_the_start_explicitly(make_supervisor, tmp_path):
    make, es = make_supervisor
    n, cli = _phase(make, duration_s=1.0)
    sim, wall = {'t': 7.5}, {'t': 100.0}
    published = _watch(n, tmp_path, sim, wall)
    n._clock_watchdog()
    for w in (110.0, 130.0, 160.0):                                     # 60 s limit, not yet
        wall['t'] = w
        n._clock_watchdog()
    wall['t'] = 160.5
    with pytest.raises(SystemExit) as ex:
        n._clock_watchdog()
    assert ex.value.code == 3 and published == [] and n.state == es.DONE
    st = _manifest_start(tmp_path)
    assert st['status'] == 'start_failed' and 'sim clock stalled' in st['fail_reason']


def test_sim_clock_that_never_starts_fails_the_start_explicitly(make_supervisor, tmp_path):
    make, es = make_supervisor
    n, cli = _phase(make, duration_s=1.0)
    sim, wall = {'t': 0.0}, {'t': 5.0}
    published = _watch(n, tmp_path, sim, wall)
    n._clock_watchdog()
    wall['t'] = 66.0
    with pytest.raises(SystemExit) as ex:
        n._clock_watchdog()
    assert ex.value.code == 3 and published == []
    assert 'sim time 0.000 s has not advanced' in _manifest_start(tmp_path)['fail_reason']


def test_slow_but_advancing_sim_clock_never_trips_the_watchdog(make_supervisor, tmp_path):
    """RTF 0.0076 (the worst start-up window recorded, F-8-10) still advances."""
    make, es = make_supervisor
    n, cli = _phase(make, duration_s=1.0)
    sim, wall = {'t': 0.0}, {'t': 0.0}
    _watch(n, tmp_path, sim, wall)
    for k in range(400):
        wall['t'] = float(k)
        sim['t'] = 0.0076 * k
        n._clock_watchdog()
    assert n.state == es.WAITING and n.start_failed is None


def test_watchdog_is_inert_after_the_birth(make_supervisor, tmp_path):
    make, es = make_supervisor
    n, cli = _phase(make, duration_s=1.0)
    sim, wall = {'t': 20.0}, {'t': 0.0}
    _watch(n, tmp_path, sim, wall)
    n.state = es.RUNNING
    for w in (0.0, 100.0, 1000.0):
        wall['t'] = w
        n._clock_watchdog()
    assert n.state == es.RUNNING and n.start_failed is None


def test_real_spin_with_a_frozen_sim_clock_ends_with_exit_code_3(make_supervisor, tmp_path):
    """A real executor, use_sim_time and no /clock: the sim-time tick never advances, the
    STEADY watchdog still fires and ends the run explicitly."""
    import time as _time
    import rclpy
    from rclpy.clock import ClockType
    make, es = make_supervisor
    with open(tmp_path / 'run_manifest_sim.json', 'w') as f:
        json.dump({'SIM': True}, f)
    n = make(perception=False, start_mode='phase', use_sim_time=True,
             start_clock_stall_wall_s=0.4, duration_s=1.0)
    assert n._watchdog.clock.clock_type == ClockType.STEADY_TIME
    published = []
    n.pub_start.publish = lambda m: published.append(m.data)
    t_end = _time.monotonic() + 10.0
    with pytest.raises(SystemExit) as ex:
        while _time.monotonic() < t_end:
            rclpy.spin_once(n, timeout_sec=0.05)
    assert ex.value.code == 3 and published == []
    assert 'sim clock stalled' in _manifest_start(tmp_path)['fail_reason']


# ---------------------------------------------------------------------------- legacy unchanged
def test_legacy_mode_has_no_watchdog_no_new_parameters_and_polls_as_before(make_supervisor):
    make, es = make_supervisor
    n = make(perception=False)
    assert n.start_mode == 'legacy' and len(list(n.timers)) == 1               # only the sim-time tick
    assert not hasattr(n, '_watchdog') and not hasattr(n, 'ctrl_stats')
    for p in ('start_ctrl_poll_s', 'start_ctrl_timeout_s', 'start_ctrl_max_age_s',
              'start_clock_stall_wall_s', 'start_timeout_s'):
        assert not n.has_parameter(p)
    cli = FakeListClient()
    n._list_cli = cli
    n._poll_controller()                                                 # sends one request
    cli.deliver(0.0)
    n._poll_controller()                                                 # takes the answer
    assert n.controller_active and len(cli.futs) == 1
    for _ in range(10):
        n._poll_controller()                                             # legacy stops at activation
    assert len(cli.futs) == 1
