"""Phase 4: ScenarioObstacles holds the law at the physics rate; ground truth reaches the log.

SLOW: one Gazebo run of tests.p4_laws (four obstacles, one per law, away from
the arm), ~25 s wall. Data: the 1 kHz truth poses the system publishes
(/sim/truth/obstacle_poses[_vel], in the run's bag), truth.npz, and the
experiment_logger CSV.

* static / linear / ballistic: every pose and velocity after the start equals
  the analytic law to 1e-6 m (the system evaluates it per step, no integrator);
* pursuit: equals explicit Euler on the TRUE flange (truth.npz) to 2 mm.
  The residual comes from sampling the flange at joint-state stamps instead of
  inside the physics step;
* before the start every parked obstacle is out of view (z = -20 m); the first
  pose after the start is p0 + v·(<= 2 ms): a birth, not a teleport;
* experiment_logger carries finite truth_* columns (truth_topic is set by the
  sim launch; default-off in the logger).
"""
import csv
import glob
import json
import os
import sys

import numpy as np
import pytest

pytestmark = pytest.mark.slow


def _simexp():
    from ament_index_python.packages import get_package_prefix
    sys.path.insert(0, os.path.join(get_package_prefix('franka_simulation_experiments'), 'lib',
                                    'franka_simulation_experiments'))
    from simexp import analysis, obstacle_laws, runner, sim_config
    return analysis, obstacle_laws, runner, sim_config


@pytest.fixture(scope='module')
def run():
    A, L, runner, sc = _simexp()
    rd = sc.make_run_dir(name='phase4_p4_laws')
    res = runner.run_cell(rd, condition='kf', scenario='p4_laws', timeout_s=300)
    assert not res['timed_out'] and not res['leftovers'], res
    from geometry_msgs.msg import PoseArray
    from std_msgs.msg import Float64
    b = A.read_bag_topics(os.path.join(rd, 'bag'), {
        '/sim/truth/obstacle_poses': PoseArray, '/sim/truth/obstacle_poses_vel': PoseArray,
        '/sim/episode_start': Float64})
    man = A.load_json(os.path.join(rd, 'run_manifest_sim.json'))

    def arr(msgs):
        t = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in msgs])
        P = np.array([[[p.position.x, p.position.y, p.position.z] for p in m.poses] for _, m in msgs])
        return t, P
    tp, P = arr(b['/sim/truth/obstacle_poses'])
    tv, V = arr(b['/sim/truth/obstacle_poses_vel'])
    t0 = b['/sim/episode_start'][0][1].data
    return dict(rd=rd, man=man, tp=tp, P=P, tv=tv, V=V, t0=t0, A=A, L=L,
                specs=man['scenario_resolved']['obstacles'], truth=A.load_truth(rd))


def _record(r, key, val):
    p = os.path.join(r['rd'], 'phase4_results.json')
    d = json.load(open(p)) if os.path.exists(p) else {}
    d[key] = val
    json.dump(d, open(p, 'w'), indent=1, default=float)


def test_the_start_time_is_the_supervisors(run):
    assert run['t0'] == pytest.approx(run['man']['episode']['t_start_sim'], abs=1e-9)


@pytest.mark.parametrize('k', [0, 1, 2])
def test_analytic_laws_hold_exactly(run, k):
    L = run['L']
    spec = run['specs'][k]
    after = run['tp'] >= run['t0'] + 0.002
    err = np.array([np.linalg.norm(run['P'][i, k] - L.position(spec, t - run['t0']))
                    for i, t in zip(np.flatnonzero(after), run['tp'][after])])
    verr = np.array([np.linalg.norm(run['V'][i, k] - L.velocity(spec, t - run['t0']))
                     for i, t in zip(np.flatnonzero(run['tv'] >= run['t0'] + 0.002),
                                     run['tv'][run['tv'] >= run['t0'] + 0.002])])
    _record(run, f'law_{spec["law"]}', dict(n=int(err.size), pos_err_max=float(err.max()),
                                            vel_err_max=float(verr.max())))
    assert err.size > 1000
    assert err.max() < 1e-6, err.max()
    assert verr.max() < 1e-6, verr.max()


def test_pursuit_is_euler_on_the_true_flange(run):
    L = run['L']
    spec = run['specs'][3]
    tr = run['truth']
    sel = run['tp'] >= run['t0']
    tps = run['tp'][sel]
    targets = np.stack([np.interp(tps, tr['t'], tr['flange'][:, j]) for j in range(3)], axis=1)
    h = np.diff(np.r_[run['t0'], tps])
    pred = L.pursue(spec['p0'], spec['speed'], targets, h)
    err = np.linalg.norm(run['P'][sel, 3] - pred, axis=1)
    _record(run, 'law_pursuit', dict(n=int(err.size), pos_err_max=float(err.max()),
                                     travelled=float(np.linalg.norm(pred[-1] - pred[0]))))
    assert err.max() < 0.002, err.max()


def test_parked_before_and_born_at_p0_without_a_teleport(run):
    before = run['tp'] < run['t0'] - 0.001
    assert before.any()
    np.testing.assert_allclose(run['P'][before][:, :, 2], -20.0, atol=1e-9)
    i = int(np.flatnonzero(run['tp'] >= run['t0'])[0])
    jumps = []
    for k, spec in enumerate(run['specs']):
        v = np.linalg.norm(spec.get('v0', [0, 0, 0])) + spec.get('speed', 0.0)
        if spec['law'] == 'ballistic':
            v += 9.81 * 0.002
        jump = float(np.linalg.norm(run['P'][i, k] - np.asarray(spec['p0'])))
        jumps.append(jump)
        assert jump <= v * 0.002 + 1e-9, (k, jump)
    _record(run, 'first_step_jump_m', jumps)


def test_the_logger_carries_the_truth_columns(run):
    f = glob.glob(os.path.join(run['rd'], 'logger', '*', 'experiment_log.csv'))[0]
    with open(f) as fh:
        rows = list(csv.DictReader(fh))
    assert 'truth_min_gap' in rows[0] and 'truth_gap_10' in rows[0]
    vals = [float(r['truth_min_gap']) for r in rows if r['truth_min_gap'] not in ('', 'nan')]
    assert len(vals) > 100 and all(np.isfinite(vals))
    _record(run, 'logger_truth_rows', len(vals))
