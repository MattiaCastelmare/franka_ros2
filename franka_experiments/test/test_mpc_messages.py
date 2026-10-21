"""What the MPC and the task publish and log, around utils/mpc_problem.py.

- human_utils: the TCP path messages (task preview, MPC plan) and MpcDiagnostics;
- human_markers.ee_path_markers: how human_visualizer draws the paths;
- logging_utils.format_mpc_summary: the once-a-second terminal line.

ROS messages only, no graph.
"""

import numpy as np
import pytest
from visualization_msgs.msg import Marker

from franka_experiments.utils.human_markers import EE_PLAN_COLOR, ee_path_markers
from franka_experiments.utils.human_utils import (
    ee_trajectory_arrays, fill_ee_trajectory_msg, make_ee_trajectory_msg, mpc_diagnostics_msg,
)
from franka_experiments.utils.logging_utils import format_mpc_summary
from franka_experiments.utils.mpc_problem import MPCDiagnostics


def _stamp():
    return make_ee_trajectory_msg('x', 1, 0.1).header.stamp


def _solved(**over):
    fields = dict(status='solved', solved=True, iterations=40, solve_time_ms=0.3, cost=1.0)
    fields.update(over)
    return MPCDiagnostics(**fields)


# ── TCP path messages ───────────────────────────────────────────────────────

def test_trajectory_message_round_trip():
    msg = make_ee_trajectory_msg('fr3_hand_tcp', 20, 0.05, orientation=(1.0, 0.0, 0.0, 0.0))
    rng = np.random.default_rng(0)
    p, v, a = rng.normal(size=(20, 3)), rng.normal(size=(20, 3)), rng.normal(size=(20, 3))
    fill_ee_trajectory_msg(msg, msg.header.stamp, p, v, a)

    times, positions, orientations = ee_trajectory_arrays(msg)
    assert msg.joint_names == ['fr3_hand_tcp'] and msg.header.frame_id == 'fr3_link0'
    assert np.allclose(times, 0.05 * np.arange(20))
    assert np.allclose(positions, p)
    assert np.allclose(orientations, [1.0, 0.0, 0.0, 0.0])
    assert np.allclose([pt.velocities[0].linear.x for pt in msg.points], v[:, 0])
    assert np.allclose([pt.accelerations[0].linear.z for pt in msg.points], a[:, 2])


def test_a_refill_overwrites_and_can_set_orientations():
    msg = make_ee_trajectory_msg('fr3_hand_tcp', 5, 0.1)
    fill_ee_trajectory_msg(msg, msg.header.stamp, np.ones((5, 3)))
    quats = np.tile([0.0, 0.0, 0.7071068, 0.7071068], (5, 1))
    fill_ee_trajectory_msg(msg, msg.header.stamp, np.zeros((5, 3)), orientations=quats)
    _, positions, orientations = ee_trajectory_arrays(msg)
    assert np.allclose(positions, 0.0) and np.allclose(orientations, quats)


# ── Markers ─────────────────────────────────────────────────────────────────

def test_path_markers_are_a_line_and_dots():
    path = np.linspace([0.3, -0.2, 0.4], [0.3, 0.2, 0.4], 10)
    line, dots = ee_path_markers(path, 'ee_plan', EE_PLAN_COLOR, 'fr3_link0', _stamp())
    assert (line.type, dots.type) == (Marker.LINE_STRIP, Marker.SPHERE_LIST)
    assert len(line.points) == len(dots.points) == 10 and line.color == EE_PLAN_COLOR
    assert line.ns == dots.ns == 'ee_plan' and line.id != dots.id

    (only_line,) = ee_path_markers(path, 'ee_trail', EE_PLAN_COLOR, 'fr3_link0', _stamp(), dot_size=0.0)
    assert only_line.type == Marker.LINE_STRIP


@pytest.mark.parametrize('n_points', [0, 1])
def test_empty_or_single_point_paths_delete(n_points):
    markers = ee_path_markers(np.zeros((n_points, 3)), 'ee_reference', EE_PLAN_COLOR, 'fr3_link0', _stamp())
    assert [(m.action, m.ns) for m in markers] == [(Marker.DELETEALL, 'ee_reference')]


# ── Diagnostics ─────────────────────────────────────────────────────────────

def test_diagnostics_message_copies_every_field():
    diag = _solved(human_rows=6, human_binding=2, human_clearance=-0.004, max_slack=0.004,
                   accel_saturation=0.25)
    msg = mpc_diagnostics_msg(diag, _stamp(), fallback=True)
    assert (msg.status, msg.solved, msg.fallback, msg.iterations) == ('solved', True, True, 40)
    assert (msg.human_rows, msg.human_binding) == (6, 2)
    assert msg.human_clearance == pytest.approx(-0.004) and msg.accel_saturation == pytest.approx(0.25)
    # A failed solve keeps its NaNs (cost, clearance): the CSV shows them as nan
    failed = mpc_diagnostics_msg(MPCDiagnostics(status='primal infeasible'), _stamp())
    assert not failed.solved and np.isnan(failed.cost) and np.isnan(failed.human_clearance)


def test_summary_of_a_quiet_second():
    line = format_mpc_summary([_solved(solve_time_ms=t) for t in (0.2, 0.3, 0.9)])
    assert line == ('MPC 3 solves, 0 failed, 0 fallback | solve 0.30 ms median, 0.90 max, 40 it max'
                    ' | human: none | q̈ at box 0%')


def test_summary_with_the_human_and_a_failure():
    diags = [_solved(human_rows=4, human_binding=1, human_clearance=0.05, max_slack=0.0),
             _solved(human_rows=6, human_binding=2, human_clearance=-0.01, max_slack=0.01,
                     accel_saturation=0.5),
             MPCDiagnostics(status='maximum iterations reached', iterations=2000, solve_time_ms=15.0)]
    line = format_mpc_summary(diags, fallbacks=1)
    parts = line.split(' | ')
    assert parts[0] == 'MPC 3 solves, 1 failed, 1 fallback'
    assert parts[1] == 'solve 0.30 ms median, 15.00 max, 2000 it max'
    assert parts[2] == 'human: 6 rows, 2 binding, clearance -0.010 m, slack 0.010 m'
    assert parts[3] == 'q̈ at box 25%'
    assert parts[4] == 'last failure: maximum iterations reached'


def test_summary_without_solves():
    assert format_mpc_summary([]) == 'MPC no solves'
