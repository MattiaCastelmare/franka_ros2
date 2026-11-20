"""Deterministic, executor-free characterisation harness for the CBF safety filter.

Purpose: pin the CURRENT input/output behaviour of ``nodes/cbf_safety_filter.py`` and
``utils/cbf_state_rows.py::ConstraintBuilder`` before they are split into smaller modules
(docs/framework_study/refactoring/CBF_MONOLITHS_ANALYSIS.md). It is a test helper only; it is not
collected (no ``test_`` prefix) and it modifies no production file.

What is real and what is driven:

* REAL: the shipped node ``CBFSafetyFilter`` built from the shipped ``fr3_control.yaml`` (through
  ``load_cbf_config``, i.e. the INSTALLED share copy, whose SHA-256 is recorded and checked), its
  ``ConstraintBuilder``, Pinocchio kinematics, self-collision / singularity row builders, the
  governor, go-around, state box, slew limit, OSQP and the fallback ladder.
* DRIVEN: the callbacks are called directly on one thread in a fixed order (no executor, no spin;
  the sequential order that ``executor_mode: single`` produces). Time is a fake clock: the node's
  ``_now`` is replaced on the INSTANCE. The robot is a double integrator fed with the published
  q̈_safe. Perception is synthesised from the harness's OWN ``CBFKinematics`` (same URDF and control
  points as the node), so the filter's kinematics state is never touched by the harness.

Test-only deviations, each justified:

* OSQP ``adaptive_rho_interval`` is fixed (``ADAPTIVE_RHO_INTERVAL``) through a module shim that
  wraps ``osqp.OSQP`` for ``cbf_safety_filter`` and ``cbf_qp_assembly.box_only_solve``. Production leaves it at 0,
  where OSQP chooses the ρ-update interval from the measured SETUP TIME: the iterates then depend on
  wall-clock timing and replay is not bit-reproducible (F-8-56: 1 of 12 replays differed by up to
  0.0376 rad/s²). Every OTHER setting and every matrix and vector given to OSQP is the production one
  and is recorded, so a change to them is still detected.
* ``_priority_set`` is set to True so the first tick does not try SCHED_FIFO on the test thread
  (``qp_thread_fifo_prio`` is 0 in the shipped config, so this is a no-op for the outputs).
* The two publishers are replaced by recording sinks on the instance; ``_publish`` and
  ``_publish_status`` themselves run unchanged.
* Wall-clock diagnostics are excluded from comparison: ``solve_wall_ms`` and
  ``constraint_build_wall_ms`` in /cbf_status, and ``build_wall_ms`` in the snapshot.

Goldens: ``data/cbf_characterisation/<scenario>.npz`` (numeric arrays, exact float64) and the same
name ``.json`` (non-numeric fields and the environment fingerprint). Regenerate ONLY at a commit whose
behaviour is the intended reference, with ``gen_cbf_characterisation_goldens.py``.
"""
import hashlib
import json
import os
import platform
import sys
import types

import numpy as np

import rclpy
from franka_msgs.msg import LinkDistance, MultiLinkDistance
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

import franka_experiments.nodes.cbf_safety_filter as csf
from franka_experiments.nodes.cbf_safety_filter import CBFSafetyFilter, FR3_JOINTS
from franka_experiments.utils.cbf_diagnostics import STATUS_FIELDS

HERE = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(HERE, 'data', 'cbf_characterisation')
SOURCE_CONFIG = os.path.join(HERE, '..', 'config', 'fr3_control.yaml')
FORMAT_VERSION = 1
NV = 7
ADAPTIVE_RHO_INTERVAL = 25          # F-8-56; any fixed value works, 25 is OSQP's documented example
STATUS_KEEP = [i for i, f in enumerate(STATUS_FIELDS) if 'wall' not in f]
SNAP_SKIP = {'build_wall_ms'}

Q0 = np.array([0.0, -0.4, 0.0, -1.8, 0.0, 1.6, 0.8])     # the smoke test's pose (sigma_min 0.143)
T0 = 1000.0                                             # fake-clock origin [s]


def sha256(path):
    with open(path, 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def installed_vs_source():
    """Files of the imported ``franka_experiments`` package that differ from the source tree.

    The node and its utils are imported from wherever Python finds them (normally the colcon
    INSTALL tree, which is a copy). A characterisation run against a stale install compares the
    wrong code, so the tests require this list to be empty: rebuild first.
    """
    import franka_experiments
    imp = os.path.dirname(os.path.abspath(franka_experiments.__file__))
    src = os.path.abspath(os.path.join(HERE, '..', 'franka_experiments'))
    if imp == src:
        return []
    bad = []
    for root, _, files in os.walk(src):
        for f in files:
            if f.endswith('.py'):
                rel = os.path.relpath(os.path.join(root, f), src)
                other = os.path.join(imp, rel)
                if not os.path.exists(other) or sha256(other) != sha256(os.path.join(root, f)):
                    bad.append(rel)
    return sorted(bad)


def fingerprint():
    """What a bit-exact golden depends on besides the code under test."""
    import osqp
    import pinocchio
    import scipy
    return dict(python=platform.python_version(), machine=platform.machine(),
                numpy=np.__version__, scipy=scipy.__version__, osqp=osqp.__version__,
                pinocchio=pinocchio.__version__)


# ── OSQP shim ─────────────────────────────────────────────────────────────────

class QPLog:
    """Records every OSQP call of one tick; can inject a failure into ``solve``."""

    def __init__(self):
        self.calls = []
        self.fail_next = 0          # number of upcoming solve() calls that raise

    def take(self):
        c, self.calls = self.calls, []
        return c


def _arr(x):
    if x is None:
        return None
    if hasattr(x, 'toarray'):                       # scipy sparse
        x = x.tocsc()
        return dict(data=np.array(x.data, float), indices=np.array(x.indices, np.int64),
                    indptr=np.array(x.indptr, np.int64), shape=np.array(x.shape, np.int64))
    return np.array(x, dtype=float, copy=True)


class _DetOSQP:
    """Role 'main' = the warm-started filter QP (``_solve``); 'box' = a fresh
    ``box_only_solve`` problem (warm_start=False). Failures are injected into 'main' only, so the
    fallback rungs themselves still run."""

    def __init__(self, real, log):
        self._p, self._log, self._role = real, log, 'main'

    def setup(self, **kw):
        self._role = 'box' if kw.get('warm_start') is False else 'main'
        kw.setdefault('adaptive_rho_interval', ADAPTIVE_RHO_INTERVAL)
        rec = {k: _arr(v) if k in ('P', 'q', 'A', 'l', 'u') else v for k, v in kw.items()}
        self._log.calls.append((self._role + '_setup', rec))
        return self._p.setup(**kw)

    def update(self, **kw):
        self._log.calls.append((self._role + '_update', {k: _arr(v) for k, v in kw.items()}))
        return self._p.update(**kw)

    def solve(self):
        if self._role == 'main' and self._log.fail_next > 0:
            self._log.fail_next -= 1
            self._log.calls.append(('main_solve', dict(injected_failure=True)))
            raise RuntimeError('injected OSQP failure (characterisation test)')
        res = self._p.solve()
        self._log.calls.append((self._role + '_solve', dict(
            x=None if res.x is None else np.array(res.x, float),
            status_val=int(res.info.status_val), iter=int(res.info.iter))))
        return res


class _OsqpShim(types.ModuleType):
    def __init__(self, real, log):
        super().__init__('osqp_characterisation_shim')
        self._real, self._log = real, log

    def __getattr__(self, name):
        return getattr(self._real, name)

    def OSQP(self):                                         # noqa: N802 (osqp's own name)
        return _DetOSQP(self._real.OSQP(), self._log)


class deterministic_osqp:
    """Context manager: route the production ``osqp`` uses through the shim."""

    def __init__(self, log):
        self.log = log

    def __enter__(self):
        # cbf_safety_filter holds a module reference; cbf_qp_assembly.box_only_solve runs
        # ``import osqp`` at call time, which reads sys.modules. Both are routed while active.
        self._saved = (csf.osqp, sys.modules['osqp'])
        shim = _OsqpShim(self._saved[0], self.log)
        csf.osqp = shim
        sys.modules['osqp'] = shim
        return self.log

    def __exit__(self, *exc):
        csf.osqp, sys.modules['osqp'] = self._saved


# ── node lifecycle ───────────────────────────────────────────────────────────

def _ros_value(v):
    if isinstance(v, bool):
        return 'true' if v else 'false'
    return str(v)


class _Sink:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(list(msg.data))

    def get_subscription_count(self):
        return 0


class Clock:
    def __init__(self, t=T0):
        self.t = float(t)


def make_node(overrides=None):
    """rclpy.init + the shipped node (+ ROS parameter overrides). Pair with ``close_node``."""
    args = ['cbf_characterisation', '--ros-args', '-r', '__node:=cbf_safety_filter',
            '--log-level', 'cbf_safety_filter:=fatal']
    for k, v in sorted((overrides or {}).items()):
        args += ['-p', f'{k}:={_ros_value(v)}']
    rclpy.init(args=args)
    try:
        node = CBFSafetyFilter()
    except BaseException:
        rclpy.shutdown()            # a refused configuration must not leak an initialised context
        raise
    clock = Clock()
    node._now = lambda: clock.t                         # instance attribute shadows the method
    node._priority_set = True
    node._pub, node._status_pub = _Sink(), _Sink()
    return node, clock


def close_node(node):
    node.destroy_node()
    rclpy.shutdown()


def harness_kinematics(node):
    """An independent CBFKinematics with the node's own URDF and control points."""
    from franka_experiments.utils.config import load_config_file
    kin = csf.CBFKinematics(csf.pin.buildModelFromUrdf(csf.build_urdf_no_hand()))
    # the node already declared robot_config_path; read it instead of declaring it again
    cfg = load_config_file(node.get_parameter('robot_config_path').value)
    kin.configure_control_points(cfg['robot'], cfg['distance'])
    assert list(kin._control_points) == list(node._rows._kin._control_points)
    return kin


# ── synthetic inputs ─────────────────────────────────────────────────────────

def joint_state_msg(q, qdot, t):
    m = JointState()
    _stamp(m, t)                    # the node differentiates q̇ over header stamps (_diag_qddot_real)
    m.name = list(FR3_JOINTS)
    m.position = [float(x) for x in q]
    m.velocity = [float(x) for x in qdot]
    return m


def nominal_msg(qddot):
    m = Float64MultiArray()
    m.data = [float(x) for x in qddot]
    return m


def _stamp(m, t):
    m.header.stamp.sec = int(np.floor(t))
    m.header.stamp.nanosec = int(round((t - np.floor(t)) * 1e9))
    if m.header.stamp.nanosec >= 1_000_000_000:
        m.header.stamp.sec += 1
        m.header.stamp.nanosec -= 1_000_000_000


def distance_msg(kin, q, qdot, t_capture, ball=None, *, frames_seen=0, cp_radius=0.06,
                 track_id=7, confidence=0.9, nan_label=None, invalid_labels=(),
                 low_conf_labels=()):
    """One MultiLinkDistance over EVERY configured control point, in label order.

    ``ball`` = (centre, velocity, radius) or None (all entries invalid: nothing in view).
    The robot point is on a sphere of radius ``cp_radius`` around the control point, the obstacle
    point on the ball; distance = centre gap − both radii. ``nan_label`` gets a NaN distance,
    ``invalid_labels`` valid=False, ``low_conf_labels`` confidence 0.1.
    """
    kin.update(np.asarray(q, float), np.asarray(qdot, float), with_jdot=False)
    msg = MultiLinkDistance()
    _stamp(msg, t_capture)
    for label in kin._control_points:
        ld = LinkDistance()
        ld.robot_link_name = label.partition('#')[0]
        p = np.asarray(kin.control_point_position(label), float)
        if ball is None:
            ld.valid = False
            ld.distance = 10.0
            msg.links.append(ld)
            continue
        c, v, r = ball
        g = c - p
        n = float(np.linalg.norm(g))
        u = g / n
        pr, ph = p + cp_radius * u, c - r * u
        ld.closest_point_robot = Point(x=float(pr[0]), y=float(pr[1]), z=float(pr[2]))
        ld.closest_point_human = Point(x=float(ph[0]), y=float(ph[1]), z=float(ph[2]))
        ld.direction = Vector3(x=float(-u[0]), y=float(-u[1]), z=float(-u[2]))
        ld.distance = float(n - cp_radius - r)
        ld.valid = label not in invalid_labels
        ld.confidence = 0.1 if label in low_conf_labels else confidence
        ld.track_id = track_id
        ld.frames_seen = int(frames_seen)
        ld.obstacle_velocity = Vector3(x=float(v[0]), y=float(v[1]), z=float(v[2]))
        ld.velocity_covariance = [1e-4, 0.0, 0.0, 0.0, 1e-4, 0.0, 0.0, 0.0, 1e-4]
        if label == nan_label:
            ld.distance = float('nan')
        msg.links.append(ld)
    return msg


# ── scenarios ────────────────────────────────────────────────────────────────

class Scenario:
    """One closed-loop run. Every field is plain data; ``run`` is deterministic.

    goal: q the joint-space PD nominal drives to. ball(t) -> (centre, velocity, radius) or None.
    faults: dict of name -> (t_from, t_to) windows relative to the start:
      'no_js' (no joint state), 'no_nom' (no nominal), 'no_dist' (no distance message),
      'freeze_js' (joint state re-sent unchanged, fresh stamps), 'osqp_fail' (every main QP solve
      raises), 'vanish' (the ball leaves the frames: every entry invalid),
      'nan_entry', 'bad_stamp' (capture stamp 5 s old), 'invalid_some', 'low_conf'.
    """

    def __init__(self, name, *, duration_s=1.0, q0=Q0, qdot0=None, goal=None, ball=None,
                 faults=None, overrides=None, kp=25.0, kd=10.0, qddot_nom_max=8.0,
                 capture_latency_s=0.02, note=''):
        self.name, self.duration_s, self.note = name, duration_s, note
        self.q0 = np.array(q0, float)
        self.qdot0 = np.zeros(NV) if qdot0 is None else np.array(qdot0, float)
        self.goal = self.q0.copy() if goal is None else np.array(goal, float)
        self.ball = ball
        self.faults = dict(faults or {})
        self.overrides = dict(overrides or {})
        self.kp, self.kd, self.qddot_nom_max = kp, kd, qddot_nom_max
        self.capture_latency_s = capture_latency_s

    def active(self, fault, t_rel):
        w = self.faults.get(fault)
        return w is not None and w[0] <= t_rel < w[1]


def _linear_ball(p0, v, r=0.05):
    p0, v = np.array(p0, float), np.array(v, float)
    return lambda t: (p0 + v * t, v, r)


def _ee_point(kin, q):
    kin.update(np.asarray(q, float), np.zeros(NV), with_jdot=False)
    return np.asarray(kin.control_point_position('fr3_link8#0'), float)


#: Every default-OFF family that lives in ConstraintBuilder.build or _qp_tick and needs no extra
#: input stream (SSB needs shadow fields, ISO stop needs /iso_safety: not exercised here).
OPTIONAL_FAMILIES = {
    'enable_zone_ladder': True, 'enable_obstacle_memory': True, 'enable_weighted_slack': True,
    'enable_livelock_escape': True, 'enable_outrun_evasion': True, 'enable_lateral_evasion': True,
    'enable_curvature_term': True, 'enable_vobs_in_hdot': True, 'enable_uncertainty_margin': True,
    'enable_velocity_feedforward': True, 'enable_velocity_standoff': True,
    'enable_evidence_margin': True,
}


def scenarios(kin):
    """The pinned scenario set. ``kin``: a harness kinematics (for poses relative to the arm)."""
    ee0 = _ee_point(kin, Q0)
    # joints 1 and 7 (no self-collision on the way, unlike a folded joint 4): start inside the
    # position-brake region of q_max (2.75 / 3.02 effective), nominal pushes past it
    q_jl = Q0.copy()
    q_jl[0], q_jl[6] = 2.55, 2.85
    goal_jl = Q0.copy()
    goal_jl[0], goal_jl[6] = 3.10, 3.40
    goal_free = Q0 + np.array([0.3, 0.2, 0.0, 0.25, 0.0, -0.2, 0.0])
    goal_sing = Q0.copy()
    goal_sing[3] = -0.30                                # stretch the elbow toward the singularity
    q_sc = np.array([0.0, 0.6, 0.0, -2.5, 0.0, 1.0, 0.8])   # folded: 7 sc rows, gap 11.8 mm
    goal_sc = np.array([0.0, 0.6, 0.0, -2.8, 0.0, 1.3, 0.8])  # folds further (gap < 0 there)
    toward = np.array([-1.0, 0.0, 0.0])
    return [
        Scenario('free_space', goal=goal_free, ball=None,
                 note='nothing in view; joint/self/singularity families only; nominal passes'),
        Scenario('obstacle_approach', duration_s=1.5, goal=goal_free,
                 ball=_linear_ball(ee0 + np.array([0.55, 0.05, 0.0]), 0.45 * toward),
                 note='ball closes on the end effector at 0.45 m/s; obstacle rows bind'),
        Scenario('obstacle_static_close', duration_s=0.8, goal=goal_free,
                 ball=_linear_ball(ee0 + np.array([0.0, 0.13, 0.0]), np.zeros(3)),
                 note='static ball 13 cm (centre) from the EE; nominal drives along it'),
        Scenario('joint_limit', q0=q_jl, goal=goal_jl, ball=None, duration_s=1.0,
                 note='joints 1 and 7 driven past q_max; joint-limit rows and the position brake'),
        Scenario('singularity', goal=goal_sing, ball=None, duration_s=1.5,
                 note='elbow stretched; singularity row and the state governor'),
        Scenario('self_collision', q0=q_sc, goal=goal_sc, ball=None, duration_s=1.0,
                 note='folded pose driven further in; self-collision rows'),
        # windows exceed the shipped timeouts: nom 0.5 s, joint state 0.1 s, freeze 0.1 s,
        # distance 0.5 s (fr3_control.yaml), so each fault is reached and then recovers
        Scenario('faults', duration_s=2.4, goal=goal_free,
                 ball=_linear_ball(ee0 + np.array([0.35, 0.0, 0.0]), 0.05 * toward),
                 faults={'no_nom': (0.10, 0.70), 'no_js': (0.80, 0.95),
                         'freeze_js': (1.05, 1.30), 'no_dist': (1.40, 2.00),
                         'osqp_fail': (2.10, 2.15)},
                 note='stale nominal, stale joint state, frozen joint state, stale distance, '
                      'forced OSQP failure (brake rung), recovery between them'),
        Scenario('faults_box_only', duration_s=0.5, goal=goal_free,
                 ball=_linear_ball(ee0 + np.array([0.35, 0.0, 0.0]), 0.05 * toward),
                 faults={'osqp_fail': (0.20, 0.25)}, overrides={'qp_failure_policy': 'box_only'},
                 note='qp_failure_policy box_only: the level-1 rung'),
        Scenario('optional_families', duration_s=1.5, goal=goal_free,
                 ball=_linear_ball(ee0 + np.array([0.40, 0.03, 0.0]), 0.45 * toward),
                 faults={'vanish': (0.75, 0.95)}, overrides=OPTIONAL_FAMILIES,
                 note='the non-default builder/tick families switched on; the ball comes close, '
                      'vanishes for 0.2 s (obstacle memory) and returns'),
        Scenario('iso_speed_rows', duration_s=1.0, goal=goal_free,
                 ball=_linear_ball(ee0 + np.array([0.45, 0.05, 0.0]), 0.45 * toward),
                 # C = 8*(16-14) mm, the node's admissible resolution (a); at the shipped C = 0.85 m
                 # the node refuses to start with iso_enabled (pinned in test_cbf_characterisation)
                 # and the Annex M ceiling the node demands: link_speed_max <= iso_v_pfl
                 overrides={'iso_enabled': True, 'iso_ssm_speed_rows': True,
                            'iso_c_intrusion': 0.016, 'link_speed_max': 0.68,
                            'retreat_cap_max_speed': 0.61},
                 note='ISO SSM speed rows (rho_spd raised) on an approaching ball'),
        Scenario('input_edge_cases', duration_s=0.8, goal=goal_free,
                 ball=_linear_ball(ee0 + np.array([0.25, 0.0, 0.0]), 0.1 * toward),
                 faults={'nan_entry': (0.10, 0.30), 'bad_stamp': (0.30, 0.45),
                         'invalid_some': (0.45, 0.60), 'low_conf': (0.60, 0.80)},
                 note='NaN distance, implausible capture stamp, invalid and low-confidence entries'),
    ]


def run(node, clock, kin, sc, qp_log):
    """Drive one scenario; return (tick events, rebuild events)."""
    P = node.P
    dt = 1.0 / P.qp_rate_hz
    k_nom = max(1, int(round(P.qp_rate_hz / 100.0)))            # commander ~100 Hz
    k_dist = max(1, int(round(P.qp_rate_hz / 30.0)))            # perception ~30 Hz
    k_reb = max(1, int(round(P.qp_rate_hz / P.cbf_update_rate_hz)))
    n = int(round(sc.duration_s / dt))
    q, qdot = sc.q0.copy(), sc.qdot0.copy()
    frozen = None
    frames = 0
    ticks, rebuilds = [], []
    nan_label = list(kin._control_points)[-1]
    some_invalid = tuple(list(kin._control_points)[-3:])
    for k in range(n):
        t_rel = k * dt
        clock.t = T0 + t_rel
        # joint state
        if not sc.active('no_js', t_rel):
            if sc.active('freeze_js', t_rel):
                if frozen is None:
                    frozen = (q.copy(), qdot.copy())
                node._on_joint_state(joint_state_msg(*frozen, clock.t))
            else:
                frozen = None
                node._on_joint_state(joint_state_msg(q, qdot, clock.t))
        # nominal
        qddot_nom = np.clip(sc.kp * (sc.goal - q) - sc.kd * qdot,
                            -sc.qddot_nom_max, sc.qddot_nom_max)
        if k % k_nom == 0 and not sc.active('no_nom', t_rel):
            node._on_qddot_nom(nominal_msg(qddot_nom))
        # perception
        if k % k_dist == 0 and not sc.active('no_dist', t_rel):
            b = (sc.ball(t_rel) if sc.ball is not None and not sc.active('vanish', t_rel)
                 else None)
            frames = frames + 1 if b is not None else 0
            t_cap = clock.t - sc.capture_latency_s
            if sc.active('bad_stamp', t_rel):
                t_cap = clock.t - 5.0
            msg = distance_msg(
                kin, q, qdot, t_cap, b, frames_seen=frames,
                nan_label=nan_label if sc.active('nan_entry', t_rel) else None,
                invalid_labels=some_invalid if sc.active('invalid_some', t_rel) else (),
                low_conf_labels=some_invalid if sc.active('low_conf', t_rel) else ())
            node._on_distances(msg)
        # 50 Hz rebuild, then the tick (the order a single-threaded executor gives at a tie)
        if k % k_reb == 0:
            node._update_constraints()
            rebuilds.append(snapshot_event(k, node))
        qp_log.take()
        qp_log.fail_next = 10 ** 6 if sc.active('osqp_fail', t_rel) else 0
        n_cmd, n_st = len(node._pub.msgs), len(node._status_pub.msgs)
        node._qp_tick()
        calls = qp_log.take()
        cmd = (np.array(node._pub.msgs[-1], float) if len(node._pub.msgs) > n_cmd
               else np.full(NV, np.nan))
        st = (np.array(node._status_pub.msgs[-1], float)[STATUS_KEEP]
              if len(node._status_pub.msgs) > n_st else np.zeros(0))
        ticks.append(tick_event(k, node, q, qdot, qddot_nom, cmd, st, calls))
        # plant: exact double integrator on what went out (nothing published -> hold q̈ = 0)
        a = np.where(np.isfinite(cmd), cmd, 0.0)
        q = q + qdot * dt + 0.5 * a * dt * dt
        qdot = qdot + a * dt
    return ticks, rebuilds


def tick_event(k, node, q, qdot, qddot_nom, cmd, st, calls):
    gov = node._diag_gov
    ev = dict(k=k, q=q.copy(), qdot=qdot.copy(), qddot_nom_sent=np.array(qddot_nom, float),
              qddot_safe=cmd, status=st, level=int(node._last_level),
              w_task=float(node._diag_w_task),
              gov_w=float('nan') if gov is None else float(gov.w),
              gov_binding='' if gov is None else str(gov.binding),
              box_lb=np.array(node._box_lb, float), box_ub=np.array(node._box_ub, float),
              tan_bias=np.array(node._tan_bias, float),
              fallback_count=np.array(node._fallback_count, np.int64))
    for j, (kind, rec) in enumerate(calls):
        for key, val in rec.items():
            ev[f'osqp{j}_{kind}_{key}'] = val
    return ev


def snapshot_event(k, node):
    con = node._con
    ev = dict(k=k, has_snapshot=con is not None)
    if con is not None:
        for f in con._fields:
            if f not in SNAP_SKIP:
                ev['snap_' + f] = getattr(con, f)
    b = node._rows
    for name in sorted(vars(b)):
        if name.startswith('diag_'):
            ev['rows_' + name] = getattr(b, name)
    return ev


# ── packing and comparison ───────────────────────────────────────────────────

def _flat(prefix, v, num, obj):
    if v is None:
        obj[prefix] = None
    elif isinstance(v, dict):
        for kk in sorted(v):
            _flat(f'{prefix}.{kk}', v[kk], num, obj)
    elif isinstance(v, np.ndarray) and v.dtype.kind in 'fiub':
        num[prefix] = v
    elif isinstance(v, (bool, np.bool_, int, np.integer, float, np.floating)):
        num[prefix] = np.asarray(v)
    elif isinstance(v, str):
        obj[prefix] = v
    elif isinstance(v, (tuple, list)):
        if len(v) and all(isinstance(x, (int, float, np.integer, np.floating, bool, np.bool_))
                          for x in v):
            num[prefix] = np.asarray(v)
        else:
            obj[prefix + '#len'] = len(v)
            for i, x in enumerate(v):
                _flat(f'{prefix}[{i}]', x, num, obj)
    elif isinstance(v, np.ndarray):
        obj[prefix] = [str(x) for x in v.ravel().tolist()]
    else:
        obj[prefix] = repr(v)


def pack(events, tag):
    """events -> (numeric dict for npz, json-able object list). Exact; no rounding."""
    flat = []
    for ev in events:
        num, obj = {}, {}
        for key, v in ev.items():
            _flat(key, v, num, obj)
        flat.append((num, obj))
    keys = sorted({k for num, _ in flat for k in num})
    out = {}
    for key in keys:
        vals = [num.get(key) for num, _ in flat]
        present = np.array([v is not None for v in vals])
        shapes = [np.asarray(v).shape for v in vals if v is not None]
        dtype = np.result_type(*[np.asarray(v).dtype for v in vals if v is not None])
        if present.all() and len(set(shapes)) == 1:
            out[f'{tag}/{key}'] = np.stack([np.asarray(v, dtype) for v in vals])
        else:
            out[f'{tag}/{key}::flat'] = (np.concatenate([np.asarray(v, dtype).ravel()
                                                         for v in vals if v is not None])
                                         if shapes else np.zeros(0, dtype))
            nd = max((len(s) for s in shapes), default=0)
            sh = np.full((len(vals), nd + 1), -1, np.int64)
            for i, v in enumerate(vals):
                if v is not None:
                    s = np.asarray(v).shape
                    sh[i, 0] = len(s)
                    sh[i, 1:1 + len(s)] = s
            out[f'{tag}/{key}::shape'] = sh
    objs = [obj for _, obj in flat]
    return out, objs


def record(sc, overrides_applied=None):
    """Run one scenario in a fresh node; return (npz dict, json dict)."""
    node, clock = make_node(sc.overrides)
    try:
        kin = harness_kinematics(node)
        log = QPLog()
        with deterministic_osqp(log):
            ticks, rebuilds = run(node, clock, kin, sc, log)
        cfg = node.P.config_path
        num_t, obj_t = pack(ticks, 'tick')
        num_r, obj_r = pack(rebuilds, 'rebuild')
        num = {**num_t, **num_r}
        meta = dict(format=FORMAT_VERSION, scenario=sc.name, note=sc.note,
                    duration_s=sc.duration_s, overrides=sc.overrides,
                    faults={k: list(v) for k, v in sc.faults.items()},
                    adaptive_rho_interval=ADAPTIVE_RHO_INTERVAL,
                    config_sha256=sha256(cfg), status_fields=[STATUS_FIELDS[i] for i in STATUS_KEEP],
                    environment=fingerprint(), n_ticks=len(ticks), n_rebuilds=len(rebuilds),
                    tick_objects=obj_t, rebuild_objects=obj_r)
        return num, json.loads(json.dumps(meta, allow_nan=True))
    finally:
        close_node(node)


def golden_paths(name):
    return (os.path.join(DATA_DIR, name + '.npz'), os.path.join(DATA_DIR, name + '.json'))


def save(name, num, meta):
    os.makedirs(DATA_DIR, exist_ok=True)
    npz, js = golden_paths(name)
    np.savez_compressed(npz, **num)
    with open(js, 'w') as f:
        json.dump(meta, f, indent=0, sort_keys=True, allow_nan=True)
        f.write('\n')


def load(name):
    npz, js = golden_paths(name)
    with np.load(npz, allow_pickle=False) as z:
        num = {k: z[k] for k in z.files}
    with open(js) as f:
        meta = json.load(f)
    return num, meta


def _first_bad_event(a, b):
    if a.ndim == 0 or a.shape[:1] != b.shape[:1]:
        return None
    for i in range(a.shape[0]):
        if not np.array_equal(a[i], b[i], equal_nan=a.dtype.kind == 'f'):
            return i
    return None


def compare(num_g, meta_g, num_n, meta_n):
    """Exact comparison. Returns a list of human-readable differences (empty == identical)."""
    diffs = []
    for key in sorted(set(num_g) | set(num_n)):
        if key not in num_g or key not in num_n:
            diffs.append(f'{key}: present only in {"golden" if key in num_g else "new run"}')
            continue
        a, b = num_g[key], num_n[key]
        if a.shape != b.shape or a.dtype != b.dtype:
            diffs.append(f'{key}: shape/dtype {a.shape}/{a.dtype} != {b.shape}/{b.dtype}')
        elif not np.array_equal(a, b, equal_nan=a.dtype.kind == 'f'):
            i = _first_bad_event(a, b)
            with np.errstate(invalid='ignore'):
                mx = (float(np.nanmax(np.abs(a.astype(float) - b.astype(float))))
                      if a.dtype.kind in 'fiu' else float('nan'))
            diffs.append(f'{key}: differs (first event index {i}, max |Δ| {mx:.3e})')
    for part in ('tick_objects', 'rebuild_objects'):
        ga, na = meta_g.get(part, []), meta_n.get(part, [])
        if len(ga) != len(na):
            diffs.append(f'{part}: {len(ga)} != {len(na)} events')
            continue
        for i, (x, y) in enumerate(zip(ga, na)):
            if x != y:
                ks = sorted(k for k in set(x) | set(y) if x.get(k) != y.get(k))
                diffs.append(f'{part}[{i}]: {ks[:6]}')
                break
    return diffs
