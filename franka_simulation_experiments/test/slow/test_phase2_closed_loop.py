"""Phase 2: pentagon -> cbf_safety_filter -> qddot_to_torque -> RtTorqueController, no obstacles.

SLOW: one 60 s (sim) Gazebo run, ~1.5 min wall.

The full chain runs, perception included (D455 -> depth_sim_adapter ->
real_time_distance), in the obstacle-free ``bare`` scene. Perception must be on:
cbf_safety_filter builds its constraint rows (joint limits, self-collision,
singularity, link speed) only once a perception message has arrived
(``_update_constraints``, cbf_safety_filter.py:842-844). Without it the filter
would run box-only, which no hardware run ever does.

Acceptance (spec Phase 2):
* experiment_summary.py and smoothness_report.py run UNCHANGED (exit 0);
* pentagon tracking error is reported. As a regression guard, p95 < 2 cm
  (measured 4.3 mm); before armature was added (plan H19) it was 46 cm;
* zero reflex trips (velocity envelope / position / torque). Torque-rate
  exceedances are libfranka-limiter clips on this robot and are reported, not
  failed (plan §2 H5);
* the qdd_cmd vs qdd_real figures are reported (CBFDIAG + joint-space projection).
"""
import json
import os
import sys

import pytest

pytestmark = pytest.mark.slow


def _simexp():
    from ament_index_python.packages import get_package_prefix
    sys.path.insert(0, os.path.join(get_package_prefix('franka_simulation_experiments'), 'lib',
                                    'franka_simulation_experiments'))
    from simexp import runner, sim_config
    return runner, sim_config


@pytest.fixture(scope='module')
def cell():
    runner, sc = _simexp()
    rd = sc.make_run_dir(name='phase2_pentagon_kf_bare')
    res = runner.run_cell(rd, condition='kf', motion='pentagon', duration_s=60.0,
                          perception=True, scene='bare', timeout_s=420)
    dig = runner.digest_cell(rd)
    with open(os.path.join(rd, 'phase2_result.json'), 'w') as f:
        json.dump({'run': res, 'digest': dig}, f, indent=1, default=float)
    return res, dig


def test_the_run_finished_cleanly(cell):
    res, dig = cell
    assert not res['timed_out'] and not res['leftovers'], res
    assert dig['manifest']['plant'] == 'gazebo' and dig['manifest']['SIM'] is True
    assert dig['manifest']['episode']['t_start_sim'] is not None
    assert not dig['manifest'].get('stray_hardware_logs')


def test_the_shipped_digests_run_unchanged(cell):
    _, dig = cell
    assert dig['experiment_summary_rc'] == 0
    assert dig['smoothness_report_rc'] == 0


def test_no_reflex_trip(cell):
    _, dig = cell
    assert dig['reflex']['verdict'] == 'PASS', dig['reflex']


def test_pentagon_tracking_error_is_reported_and_small(cell):
    _, dig = cell
    tr = dig['pentagon_tracking']
    assert tr['n'] > 4000
    assert tr['p95_m'] < 0.02, tr


def test_command_vs_realised_is_reported(cell):
    _, dig = cell
    pr = dig['qdd_projection']
    assert pr['n'] > 1000
    assert 0.8 < pr['frac_median'] < 1.2, pr
    assert dig['cbfdiag']['qdd_ratio_0.1']['n_lines'] > 0
