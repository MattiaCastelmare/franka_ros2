"""simexp.offline (column 1) and simexp.attribution (the causes) without Gazebo."""
import json
import os
import re

import numpy as np
import pytest

from simexp import attribution as AT
from simexp import conditions, offline
from simexp.emulation import _cbf_scenarios

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..'))


@pytest.mark.parametrize('cond', ['residual', 'kf', 'imm', 'imm+pred'])
def test_every_cbf_flag_reaches_the_offline_filter(cond):
    """No flag of the Gazebo condition is silently left at fr3_control.yaml's value."""
    C = _cbf_scenarios()
    P0 = C.shipped_params()
    r = conditions.resolve(cond)
    over = offline.overrides(r, P0)
    for k in conditions.CBF_ARGS:
        v = r['toggles'][k]
        if k.startswith('iso_') or k in offline.NOT_OFFLINE:
            continue
        if k in offline.GUARD_THRESHOLDS:
            name, val = offline.GUARD_THRESHOLDS[k]
            assert over[name] == (val if v else 0.0)
            continue
        name = f'enable_{k}' if hasattr(P0, f'enable_{k}') else k
        assert hasattr(P0, name), k
        assert over[name] == v, k
    assert over['obstacle_velocity_source'] == r['toggles']['obstacle_velocity_source']
    C.shipped_params(**over)                      # every name exists


def test_guard_thresholds_are_the_stacks():
    src = open(os.path.join(REPO, 'franka_experiments', 'launch',
                            'torque_control_stack.launch.py')).read()
    for flag, (name, val) in offline.GUARD_THRESHOLDS.items():
        m = re.search(rf"'{name}':\s*\(([\d.]+) if _as_bool\(p\['{flag}'\]\) else 0\.0\)", src)
        assert m and float(m.group(1)) == val, flag


def test_offline_runs_one_scenario_with_the_flags():
    r = offline.run(['slow'], 'kf', fps=90.0)['slow']
    for k in ('min_h', 'neg_duration_s', 'contact', 'slack_max', 'fallback_levels',
              'livelock_escapes', 'task_done', 'jerk_p99'):
        assert k in r
    assert r['fps'] == 90.0 and r['flags']['enable_vobs_in_hdot'] is True


SCORE = """== bag: 10 distance msgs, 3 ball passes (moving >= 1.5 m/s, within 0.6 m of a CP), trust gate 24 frames
 pass  t_close  v[m/s] min_gap |  row_lead  on_ball  tracked  max_fs  max|v| | trust_lead  fast_lead | cbf_lead  max_dev
    1     1.20    3.0    0.30 |     500ms      40      38      40   2.90 |      300ms      nanms |    480ms    12.0
    2     2.20    3.0    0.30 |     500ms      40      38      40   2.90 |      300ms      nanms |    -40ms    12.0
    3     3.20    3.0    0.30 |     nanms       0       0       0   0.00 |      nanms      nanms |    nanms     0.0
"""


def test_a_negative_lead_is_classified_not_dropped():
    import types
    AT.fe_scripts()
    import approach_probe
    import contextlib
    import io
    import tempfile
    with tempfile.NamedTemporaryFile('w', suffix='.txt', delete=False) as f:
        f.write(SCORE)
    with contextlib.redirect_stdout(io.StringIO()):
        raw = approach_probe.cmd_causes(types.SimpleNamespace(file=f.name, need_ms=450.0))
    os.unlink(f.name)
    assert len(raw) == 2                          # the shipped regex drops pass 2
    out = AT.causes(SCORE, 450.0)
    assert out['n_passes_scored'] == 3 and out['n_parsed'] == 3 and out['n_negative_leads'] == 1
    assert [p['cause'] for p in out['passes']] == ['- ok', '4 too-late', '1 not-in-rows']


def test_truth_files_mark_parked_samples_not_ok(tmp_path):
    t = np.arange(0, 1, 0.001)
    p = np.zeros((t.size, 2, 3))
    p[:, :, 2] = np.where(t < 0.5, -20.0, 0.5)[:, None]
    np.savez(tmp_path / 'truth.npz', t=t, p=p, gap=np.zeros((t.size, 1, 2)))
    man = {'scenario_resolved': {'obstacles': [
        {'name': 'still', 'law': 'static', 'radius': 0.05},
        {'name': 'ball', 'law': 'linear', 'radius': 0.036}]}}
    (tmp_path / 'run_manifest_sim.json').write_text(json.dumps(man))
    files = AT.truth_files(str(tmp_path), stride=1)
    assert [f[0] for f in files] == ['ball']
    z = np.load(files[0][2])
    assert set(z.files) == {'t', 'ok', 'p'}
    assert not z['ok'][t < 0.5].any() and z['ok'][t >= 0.5].all()


def test_residual_runs_are_classified_without_the_tracker_causes():
    """Tracking off: approach_probe calls every pass '2 untracked'; rows and lead still decide."""
    out = AT.causes(SCORE.replace('     40      38      40   2.90', '     40       0       0   2.90'),
                    450.0)
    assert [p['cause'] for p in out['passes']][:2] == ['2 untracked', '2 untracked']
    assert AT.residual_causes(out['passes'], 450.0) == {'- ok': 1, '4 too-late': 1,
                                                        '1 not-in-rows': 1}
