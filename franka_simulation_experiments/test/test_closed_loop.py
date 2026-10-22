"""simexp.closed_loop on a synthetic run directory (no bag): truth, log window, counters."""
import json
import os

import numpy as np
import pytest

from simexp import closed_loop as CL


W0 = 1792592000.0          # log stamps are epoch seconds


def _line(w, node, msg):
    return f'[{node}-9] [INFO] [{W0 + w:.9f}] [{node}]: {msg}\n'


@pytest.fixture
def run(tmp_path):
    t = np.arange(0.0, 10.0, 0.001)
    gap = np.full((t.size, 3, 1), 0.5)
    gap[(t > 6.0) & (t < 6.5), 1, 0] = 0.10          # h = -0.05 for 0.5 s
    gap[(t > 7.0) & (t < 7.01), 2, 0] = -0.01        # contact for 10 ms
    gap[t < 5.0, 0, 0] = -1.0                        # before the episode: ignored
    q = np.zeros((t.size, 7))
    q[:, 0] = 0.8 * np.clip((t - 5.0) / 3.0, 0, 1)
    np.savez(tmp_path / 'truth.npz', t=t, gap=gap, q=q, d_safe=0.15,
             labels=np.array(['a#0', 'b#0', 'c#0']))
    man = {'episode': {'t_start_sim': 5.0, 't_end_sim': 9.0, 'controller_active_at_start': True},
           'rtf': {'rtf_run': 0.9, 'rtf_p10_window': 0.85}, 'reflex_verdict': 'PASS',
           'reflex_counts': {'velocity': 0, 'position': 0, 'torque': 1, 'torque_rate_clipped': 4},
           'scenario_resolved': {'nominal': {'law': 'joint_pd',
                                             'goal_q': [0.8, 0, 0, 0, 0, 0, 0]}}}
    (tmp_path / 'run_manifest_sim.json').write_text(json.dumps(man))
    f = 'cbf_safety_filter'
    lines = [_line(100.0, f, 'tick=100 n_c=3 solve=0.1ms qddot_nom_norm=0.1 qp_fails=0 fb=[90, 10, 0] lock=0'),
             _line(101.0, 'episode_supervisor', 'SIM episode START at t=5.0000 s (sim), announced at 4.75'),
             _line(101.5, f, 'LIVELOCK: QP bending the nominal for 1.0 s with the arm still -> tangential escape #1'),
             _line(102.0, f, 'CBFDIAG t=6.100 n_c=3 s[obs/sc/qlim/sing/cap/spd]=0.200/0.000/0.000/0.000/0.050/0.000'),
             _line(102.1, f, 'CBFDIAG t=4.000 n_c=3 s[obs/sc/qlim/sing/cap/spd]=9.000/0.000/0.000/0.000/0.000/0.000'),
             _line(103.0, f, 'tick=500 n_c=3 solve=0.1ms qddot_nom_norm=0.1 qp_fails=0 fb=[460, 30, 10] lock=0'),
             _line(105.0, 'episode_supervisor', 'SIM episode DONE at t=9.000 s (sim)'),
             _line(106.0, f, 'tick=900 n_c=3 solve=0.1ms qddot_nom_norm=0.1 qp_fails=0 fb=[800, 90, 10] lock=0'),
             _line(106.5, f, 'LIVELOCK: QP bending the nominal for 1.0 s with the arm still -> tangential escape #2')]
    (tmp_path / 'launch.log').write_text(''.join(lines))
    lg = tmp_path / 'logger'
    lg.mkdir()
    with open(lg / 'pentagon_run_x.csv', 'w') as f:
        f.write('time,ee_x,ee_y,ee_z,ee_x_des,ee_y_des,ee_z_des\n')
        for k in range(300, 900):
            tt = k / 100.0
            off = 0.2 if k < 500 else 0.001        # the approach before the episode is not tracking
            f.write(f'{tt:.2f},0.4,0,{0.45 + off},0.4,0,0.45\n')
    return str(tmp_path)


def test_barrier_metrics_come_from_the_truth_inside_the_episode(run):
    r = CL.summarize_run(run)
    assert r['min_h'] == pytest.approx(-0.16)                 # the contact sample
    assert r['min_gap'] == pytest.approx(-0.01)
    assert r['contact'] is True
    assert r['neg_duration_s'] == pytest.approx(0.5 + 0.01, abs=0.003)
    assert r['min_h_cp'] == 'c#0'


def test_task_and_reflex(run):
    r = CL.summarize_run(run)
    assert r['task_done'] is True and r['task_final_err'] < 1e-9
    assert r['reflex_trips'] == 1 and r['rate_clips'] == 4 and r['rtf'] == 0.9


def test_log_counters_are_bracketed_by_the_episode(run):
    r = CL.summarize_run(run)
    assert r['fallback_levels'] == {'full': 370, 'box-only': 20, 'brake': 10}
    assert r['livelock_escapes'] == 1
    assert r['slack_max']['obs'] == 0.2 and r['slack_max']['cap'] == 0.05
    assert r['slack_max']['n_lines'] == 1


def test_names_are_the_filters():
    from franka_experiments.utils.cbf_qp_assembly import LEVEL_NAMES
    from franka_experiments.utils.cbf_state_rows import GROUP_NAMES
    assert CL.LEVELS == tuple(LEVEL_NAMES)
    assert CL.FAMILIES == tuple(GROUP_NAMES)


def test_pentagon_tracking_is_scored_over_the_episode(run):
    p = CL.summarize_run(run)['pentagon']
    assert p['max_m'] == pytest.approx(0.001, abs=1e-9) and p['n'] == 400


def test_reflex_events_are_counted_inside_the_episode_from_the_trip_file(run):
    with open(os.path.join(run, 'reflex_trips.csv'), 'w') as f:
        f.write('t_sim,kind,joint,value,limit\n'
                '4.0,torque_rate_clipped,fr3_joint2,1100,999.999\n'     # warm-up: outside
                '6.0,torque_rate_clipped,fr3_joint2,1100,999.999\n'
                '7.0,torque_rate_clipped,fr3_joint1,1200,999.999\n'
                '8.0,velocity,fr3_joint7,2.7,2.6\n'
                '9.5,torque_rate_clipped,fr3_joint2,1100,999.999\n')    # teardown: outside
    with open(os.path.join(run, 'reflex_summary.json'), 'w') as f:
        json.dump({'verdict': 'FAIL', 'rtf': {'rtf_run': 0.8}, 'counts': {}}, f)
    r = CL.summarize_run(run)
    assert r['rate_clips'] == 2 and r['reflex_trips'] == 1 and r['reflex_trips_whole_run'] == 1
    assert r['reflex_verdict'] == 'FAIL' and r['rtf'] == 0.8     # the final summary, not the copy
