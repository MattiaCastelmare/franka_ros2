#!/usr/bin/env python3
"""Observability: what the loggers and recorders actually capture (chapter 13).

Production code: `ExperimentLogger._vector_from_joint_state`, `joint_state_cb` and
`multi_distance_cb` (called unbound on a stub `self`, with real `sensor_msgs`/`franka_msgs`
messages). Source facts by text match on the logger, the evidence logger, the torque-stack launch,
`ball_throw_eval.read_bag` and `utils/run_metadata.py`. Recorded CSVs under
`franka_simulation_experiments/sim_runs/` are read (read-only) when present.

  1. F-5-21: a finger-only JointState (the launch's 10 Hz `ros2 topic pub` on /NS_1/joint_states)
     turns the logger's q into [0, 0, NaN×5] (positional fallback); an arm message in scrambled
     order is mapped by name. The launch publisher is unconditional, on the logger's default
     topic, and the logger gets no parameters. Recorded CSVs show the pattern            (NUM/FACT,
                                                                             Runtime (recorded))
  2. the logger holds every last value with no age check (a channel that dies mid-run keeps its
     last value); the legacy `min_h`/`cbf{k}_h` columns are MultiDistance.distance − d_safe,
     not the filter's h̄                                                                    (FACT/NUM)
  3. F-5-22: the rosbag topic list of torque_control_stack omits /rosout, /NS_1/iso_safety,
     /NS_1/torque_saturation, /NS_1/q_des_state; the recorder compresses with zstd (file mode)
     while `read_bag` reads only uncompressed .db3                                           (FACT)
  4. the evidence logger's tau_max_abs comes from /NS_1/torque_cmd, which excludes gravity (09);
     `run_metadata.write_runtime_metadata` has no caller outside tests                         (FACT)

Checks 5–11 were written by a parallel session (originally numbered 1–7, host-only text checks)
and merged here unchanged apart from their numbers (reconciliation, C-035):

  5. /cbf_status has 41 fields (STATUS_FIELDS); experiment_logger reads only data[0..10]      (FACT)
  6. the logger's `min_h` / `cbf{k}_h` are its own d − d_safe from the legacy MultiDistance
     topic (one entry per segment), not the filter's h̄                                        (FACT)
  7. the logger samples the 30 Hz /NS_1/joint_states; experiment_name defaults to
     'bag_02_dynamic_test', and the torque launch passes it no parameters                      (FACT)
  8. utils/run_metadata.write_runtime_metadata has no production caller                         (FACT)
  9. the torque launch's bag records /NS_1/cbf_status but not /NS_1/iso_safety,
     /NS_1/torque_saturation, /rosout or /NS_1/q_des_state                                       (FACT)
  10. stale tool defaults: watch_obstacle_tracks topic, throw_clearance rosbag/ path,
     pipeline_rate_monitor nominal rates, iso_constants_measure camera topics                   (FACT)
  11. the commander's publish parameter is named `qddot_safe_topic` but defaults to /NS_1/qddot_nom (FACT)

Needs: the ROS 2 container (rclpy, sensor_msgs, franka_msgs, matplotlib). Read-only.
"""
import csv
import glob
import math
import os
import re
import sys
from types import SimpleNamespace

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _common5 as C5  # noqa: E402

C, C3 = C5.C, C5.C3
import numpy as np  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from franka_msgs.msg import MultiDistance  # noqa: E402
from franka_experiments.nodes import experiment_logger as ELM  # noqa: E402

EL = ELM.ExperimentLogger
R = C.Report('check_observability (chapter 13)')


def _logger_stub():
    s = C5.stub(joint_names=list(ELM.DEFAULT_JOINT_NAMES), accel_lpf_alpha=0.35,
                _prev_qdot=None, _prev_js_time=None, _prev_qddot_filt=None,
                last_qddot=np.full(7, np.nan), d_safe=0.10, max_cbf_entries=12)
    s._t = [100.0]
    s.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(s._t[0] * 1e9)))
    s._vector_from_joint_state = lambda m, f: EL._vector_from_joint_state(s, m, f)
    return s


def _js(names, pos, vel, eff, t):
    m = JointState()
    m.name = list(names)
    m.position, m.velocity, m.effort = list(pos), list(vel), list(eff)
    m.header.stamp.sec, m.header.stamp.nanosec = int(t), int((t % 1) * 1e9)
    return m


# 1) F-5-21 with the production callback
arm_order = [1, 3, 6, 7, 2, 4, 5]                    # the driver's scrambled order (04 §2.2)
q_true = np.array([0.1, -0.2, 0.3, -1.9, 0.05, 1.6, 0.7])
names = [f'fr3_joint{j}' for j in arm_order]
s = _logger_stub()
EL.joint_state_cb(s, _js(names, q_true[np.array(arm_order) - 1], np.zeros(7), np.zeros(7), 10.00))
R.close('1. arm message in scrambled order: last_q mapped by name', s.last_q, q_true, 0.0, kind='NUM')
s._t[0] += 0.01
EL.joint_state_cb(s, _js(['fr3_finger_joint1', 'fr3_finger_joint2'], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], 0.0))
R.fact(f'1. finger-only message: last_q = {np.round(s.last_q, 3).tolist()} (positional fallback: q1 = q2 = 0, '
       'q3..q7 NaN); last_qdot and last_tau_effort likewise',
       s.last_q[0] == 0.0 and s.last_q[1] == 0.0 and np.all(np.isnan(s.last_q[2:]))
       and s.last_qdot[0] == 0.0 and np.all(np.isnan(s.last_tau_effort[2:])))
L = C5.src('franka_experiments/launch/torque_control_stack.launch.py', 'py')
fp = L[L.index('finger_state_publisher = ExecuteProcess('):L.index("actions.append(TimerAction(period=float(p['torque_finger_pub_delay_s'])")]
lines = L.splitlines()
fp_line = next(i for i, x in enumerate(lines) if 'finger_state_publisher = ExecuteProcess(' in x)
R.fact('1. launch: finger publisher on f"/{namespace}/joint_states" at top level of the function (unconditional)',
       "'{name: [fr3_finger_joint1, fr3_finger_joint2]," in fp and
       lines[fp_line].startswith('    finger_state_publisher') and
       "js_topic = f'/{p[\"namespace\"]}/joint_states'" in L)
el_node = L[L.index("executable='experiment_logger'"):L.index("actions.append(TimerAction(\n            period=float(p['experiment_logger_delay_s'])")]
R.fact("1. launch: experiment_logger is started with no parameters (default joint_state_topic = '/NS_1/joint_states')",
       'parameters' not in el_node and ELM.DEFAULT_JOINT_STATE_TOPIC == '/NS_1/joint_states')

runs = sorted(glob.glob(os.path.join(C.REPO, 'franka_simulation_experiments', 'sim_runs', '*', 'logger', '*',
                                     'experiment_log.csv')), key=os.path.getmtime)[-4:]
if not runs:
    R.info('1. no recorded experiment_log.csv under sim_runs/ (Runtime check skipped)')
for f in runs:
    def fl(x):
        try:
            return float(x)
        except (TypeError, ValueError):
            return float('nan')
    rows = list(csv.DictReader(open(f)))
    pat = sum(1 for r in rows if fl(r['q_1']) == 0.0 and fl(r['q_2']) == 0.0 and math.isnan(fl(r['q_3'])))
    tcpn = sum(1 for r in rows if math.isnan(fl(r.get('tcp_speed'))))
    R.info(f'1. Runtime (recorded) {os.path.relpath(f, C.REPO)[:75]}: {pat}/{len(rows)} rows '
           f'({100.0 * pat / max(len(rows), 1):.1f} %) with q1 = q2 = 0 and q3 NaN; tcp_speed NaN in {tcpn}')

# 2) no age check; legacy h
LG = C5.src('franka_experiments/franka_experiments/nodes/experiment_logger.py', 'py')
_i = LG.index('    def sample(self):')
smp = LG[_i:LG.index('\n    def ', _i + 10)]
R.fact('2. sample() compares no message age: every last_* value is held until the next message',
       not re.search(r'stale|_age\b|age\s*[<>]|timeout', smp))
s = _logger_stub()
md = MultiDistance()
from franka_msgs.msg import HumanRobotDistance  # noqa: E402
ld = HumanRobotDistance()
ld.distance, ld.valid, ld.robot_link_name, ld.confidence = 0.25, True, 'fr3_link5', 0.9
md.distances = [ld]
EL.multi_distance_cb(s, md)
R.close('2. legacy min_h = MultiDistance.distance − d_safe (the logger\'s own d_safe, no margins)',
        s.cbf_entries[0]['h'], 0.25 - 0.10, 1e-12, kind='NUM')

# 3) F-5-22 the bag
rec = L[L.index("if _as_bool(p['start_rosbag']):"):L.index('recorder = ExecuteProcess(')]
topics = re.findall(r"'(/[^']+)'", rec)
missing = [t for t in ('/rosout', '/NS_1/iso_safety', '/NS_1/torque_saturation', '/NS_1/q_des_state')
           if t not in topics]
R.fact(f'3. start_rosbag records {len(topics)} topics; not recorded: {missing}', len(missing) == 4)
R.fact("3. the recorder compresses per file with zstd", "'--compression-mode', 'file', '--compression-format', 'zstd'" in L)
B = C5.src('franka_experiments/scripts/ball_throw_eval.py', 'py')
R.fact('3. ball_throw_eval.read_bag (also used by throw_*, pass_plot, smoothness_report) opens only *.db3 '
       'through sqlite3 and asks for a manual `zstd -d` otherwise',
       "glob.glob(os.path.join(bag_dir, '*.db3'))" in B and 'zstd -d' in B and 'sqlite3.connect' in B)

# 4) evidence logger torque, run_metadata
EV = C5.src('franka_experiments/franka_experiments/nodes/iso_evidence_logger.py', 'py')
R.fact("4. evidence logger tau_max_abs = max|τ| of /NS_1/torque_cmd (gravity excluded, 09 §2)",
       "('torque_cmd', '/NS_1/torque_cmd', self._on_tau)" in EV and "np.max(np.abs(self._tau))" in EV)
users = [p for p in glob.glob(os.path.join(C.REPO, 'franka_*', '**', '*.py'), recursive=True)
         if 'write_runtime_metadata' in open(p, errors='ignore').read()
         and not os.path.basename(p).startswith('test_') and not p.endswith('run_metadata.py')
         and not re.search(r'/(sim_runs|experiment_logs|franka_logs|build|install|log)/', p)]
R.fact('4. run_metadata.write_runtime_metadata has no caller outside tests (its docstring says "enabled by the '
       'experiment runner"); data directories excluded', users == [], str([os.path.relpath(u, C.REPO) for u in users]))

# 5)–11) checks from the parallel session (their 1–7)
PKG = 'franka_experiments/franka_experiments'

from franka_experiments.utils.cbf_diagnostics import STATUS_FIELDS as fields  # noqa: E402  (pure module)
EL = C5.src(f'{PKG}/nodes/experiment_logger.py')  # unstripped text (comments kept)
i0 = EL.rindex('\n    def ', 0, EL.index("The barrier's status: the core data[0..4]"))
cb = EL[i0:EL.index('\n    def ', i0 + 5)]
idx = sorted({int(m) for m in re.findall(r'\bd\[(\d+)\]', cb)})
R.fact(f'5. STATUS_FIELDS has {len(fields)} fields; the logger reads indices {idx}', len(fields) == 41 and max(idx) == 10)
R.info('5. never logged: ' + ', '.join(fields[11:]))
R.fact('6. the logger computes h = d − self.d_safe itself', 'h = d - self.d_safe if valid else np.nan' in EL)
R.fact('6. …from the legacy MultiDistance topic (DEFAULT_MULTI_DISTANCE_TOPIC)',
       re.search(r'DEFAULT_MULTI_DISTANCE_TOPIC\s*=\s*"/human_robot/multi_distance"', EL) is not None)
R.fact('7. joint state default is the 30 Hz /NS_1/joint_states', 'DEFAULT_JOINT_STATE_TOPIC = "/NS_1/joint_states"' in EL)
R.fact("7. experiment_name default 'bag_02_dynamic_test'",
       re.search(r'DEFAULT_EXPERIMENT_NAME\s*=\s*"bag_02_dynamic_test"', EL) is not None)
TCS = C5.src('franka_experiments/launch/torque_control_stack.launch.py')
blk = TCS[TCS.index("executable='experiment_logger'"):]
blk = blk[:blk.index(')')]
R.fact('7. the torque launch starts experiment_logger with no parameters', 'parameters' not in blk)
callers, historical = [], 0
for root in ('franka_experiments', 'franka_simulation_experiments'):
    for dp, _, fs in os.walk(os.path.join(C.REPO, root)):
        if 'sim_runs' in dp.split(os.sep):        # data directory (excluded from scope): count only
            historical += sum(1 for f in fs if f.endswith('.py') and 'write_runtime_metadata'
                              in open(os.path.join(dp, f), errors='replace').read())
            continue
        for f in fs:
            if f.endswith('.py') and 'test' not in dp.split(os.sep) and f != 'run_metadata.py':
                if 'write_runtime_metadata' in open(os.path.join(dp, f), errors='replace').read():
                    callers.append(os.path.relpath(os.path.join(dp, f), C.REPO))
R.fact('8. write_runtime_metadata: no caller in the current source outside tests', not callers, str(callers))
R.info(f'8. {historical} archived node copies under sim_runs/*/provenance (data, out of scope) DID call it: '
       'the current nodes dropped the call')
i0 = TCS.index("if _as_bool(p['start_rosbag']):")
bag = TCS[TCS.index('topics = [', i0):]
bag = bag[:bag.index(']') + 1]
R.fact('9. bag topic list includes /NS_1/cbf_status and excludes iso_safety, torque_saturation, rosout, q_des_state',
       '/NS_1/cbf_status' in bag and not any(t in bag for t in ('iso_safety', 'torque_saturation', 'rosout', 'q_des_state')))
W = C5.src('franka_experiments/scripts/watch_obstacle_tracks.py')
pubs = [f for dp, _, fs in os.walk(os.path.join(C.REPO, PKG)) for f in fs
        if f.endswith('.py') and 'per_link_distances_tracked' in open(os.path.join(dp, f), errors='replace').read()]
R.fact("10. watch_obstacle_tracks defaults to '/cbf/per_link_distances_tracked', which no package module mentions",
       "'/cbf/per_link_distances_tracked'" in W and not pubs)
T = C5.src('franka_experiments/scripts/throw_clearance.py')
R.fact('10. throw_clearance loads <pkg>/rosbag/<BAG>_truth.npz; no rosbag/ directory exists',
       "'rosbag'" in T and not os.path.isdir(os.path.join(C.PKG_SRC, 'rosbag')))
PR = C5.src('franka_experiments/scripts/pipeline_rate_monitor.py')
y = C5.C3.yaml_params()
R.fact(f"10. pipeline_rate_monitor expects /NS_1/qddot_safe at 100 Hz; qp_rate_hz is {y['qp_rate_hz']:g}",
       "('cbf',        '/NS_1/qddot_safe',                   100.0)," in PR and y['qp_rate_hz'] != 100.0)
IM = C5.src('franka_experiments/scripts/iso_constants_measure.py')
FC = C5.src('franka_experiments/config/fr3_complete.yaml')
R.fact("10. iso_constants_measure 'uncertainty' defaults to /cams/d455/...; the shipped camera topics are /camera/camera/...",
       "/cams/d455/aligned_depth_to_color/image_raw" in IM and '/camera/camera/' in FC and '/cams/d455' not in FC)
CM = C5.src(f'{PKG}/nodes/pentagon_qddot_commander.py')
R.fact("11. commander: declare_parameter('qddot_safe_topic', … '/NS_1/qddot_nom')",
       re.search(r"declare_parameter\('qddot_safe_topic',\s*_topics\.get\('qddot_nom', '/NS_1/qddot_nom'\)\)", CM) is not None)
sys.exit(R.finish())
