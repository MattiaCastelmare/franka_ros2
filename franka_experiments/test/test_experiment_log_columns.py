"""The experiment log's column contract, and the summary that reads it back.

``experiment_logger`` is the node that runs on EVERY experiment and writes the
one file anybody looks at afterwards. Two ways to break it silently:

* header and row disagree — ``csv.DictWriter`` then raises mid-run, on the
  robot, after the interesting part has already happened;
* a channel that was not publishing writes zeros instead of NaN, so "that
  channel was down" becomes "it was up and read zero" — which is what makes a
  reader conclude the arm was stationary when in fact nothing was recording it.

The second one has bitten this package once already (see
``test_iso_evidence_report.py::test_an_absent_channel_is_inconclusive_not_a_pass``).
"""

import importlib.util
import math
import os
import types

import numpy as np
import pytest

from franka_experiments.nodes.experiment_logger import (
    DEFAULT_TCP_LINK, ExperimentLogger)

NUM_JOINTS = 7
_SPEC = importlib.util.spec_from_file_location(
    'experiment_summary',
    os.path.join(os.path.dirname(os.path.realpath(__file__)), '..',
                 'scripts', 'experiment_summary.py'))
S = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(S)


def header(max_cbf=3):
    return ExperimentLogger._make_header(
        types.SimpleNamespace(max_cbf_entries=max_cbf))


# ── the header ───────────────────────────────────────────────────────────────

def test_the_header_has_no_duplicates():
    h = header()
    assert len(h) == len(set(h))


@pytest.mark.parametrize('prefix', [
    'q', 'qdot', 'qddot', 'tau_effort', 'tau_cmd',
    'qdot_nom', 'qdot_cmd', 'qddot_nom', 'qddot_safe'])
def test_every_per_joint_block_has_seven_columns(prefix):
    h = header()
    assert [c for c in h if c.startswith(prefix + '_')
            and c[len(prefix) + 1:].isdigit()] == \
           [f'{prefix}_{i}' for i in range(1, NUM_JOINTS + 1)]


def test_the_acceleration_pipeline_is_logged():
    """The torque stack commands q̈, not q̇: without these the file records the
    task and the outcome but not the command in between."""
    h = header()
    for c in ('qddot_nom_1', 'qddot_safe_7', 'qddot_nom_norm',
              'qddot_safe_norm', 'qddot_delta_norm'):
        assert c in h, c


def test_the_cartesian_block_is_logged():
    """TCP speed is the quantity ISO limits. Joint speed is not a substitute:
    a folded or near-singular arm decouples the two in both directions."""
    h = header()
    for c in ('tcp_x', 'tcp_y', 'tcp_z', 'tcp_qx', 'tcp_qy', 'tcp_qz', 'tcp_qw',
              'tcp_speed', 'tcp_omega'):
        assert c in h, c


def test_the_per_control_point_avoidance_block_is_logged():
    h = header()
    for c in ('cp_n_valid', 'cp_n_total', 'cp_min_distance', 'cp_min_link',
              'cp_v_obs_max', 'cp_n_tracked'):
        assert c in h, c


def test_the_core_barrier_status_is_logged_not_just_the_iso_tail():
    """Found live: a run recorded that the filter was bending the command
    (qddot_delta_norm = 5.9) without recording what it was bending it around.

    cbf_status_cb gated EVERYTHING on len(d) >= 9, so a filter publishing the
    pre-ISO 5-element message logged nothing at all, and even with 9 elements
    the core fields were never stored. The summary asked for cbf_slack and got
    "not recorded" on a run where the barrier was demonstrably active.
    """
    h = header()
    for c in ('cbf_n_rows', 'cbf_slack', 'cbf_fault', 'cbf_n_violated',
              'cbf_d_min'):
        assert c in h, c


def test_a_five_element_cbf_status_still_fills_the_core_fields():
    n = types.SimpleNamespace(
        last_cbf_n_rows=float('nan'), last_cbf_slack=float('nan'),
        last_cbf_fault=float('nan'), last_cbf_n_viol=float('nan'),
        last_cbf_d_min=float('nan'), last_cbf_sp=float('nan'),
        last_cbf_vcap=float('nan'), last_cbf_vcls=float('nan'),
        last_cbf_isostop=float('nan'))
    ExperimentLogger.cbf_status_cb(n, types.SimpleNamespace(
        data=[8.0, 0.25, 0.0, 2.0, 0.31]))
    assert n.last_cbf_n_rows == 8.0
    assert n.last_cbf_slack == pytest.approx(0.25)
    assert n.last_cbf_n_viol == 2.0
    assert n.last_cbf_d_min == pytest.approx(0.31)
    # the ISO tail was not there, so it must stay NaN rather than become 0
    assert math.isnan(n.last_cbf_sp)


def test_a_nine_element_cbf_status_fills_both_halves():
    n = types.SimpleNamespace(**{k: float('nan') for k in (
        'last_cbf_n_rows', 'last_cbf_slack', 'last_cbf_fault',
        'last_cbf_n_viol', 'last_cbf_d_min', 'last_cbf_sp', 'last_cbf_vcap',
        'last_cbf_vcls', 'last_cbf_isostop')})
    ExperimentLogger.cbf_status_cb(n, types.SimpleNamespace(
        data=[8.0, 0.25, 0.0, 2.0, 0.31, 0.90, 0.05, 1.20, 1.0]))
    assert n.last_cbf_slack == pytest.approx(0.25)
    assert n.last_cbf_sp == pytest.approx(0.90)
    assert n.last_cbf_isostop == 1.0


def test_the_summary_only_asks_for_columns_the_logger_writes():
    """The two drifted once: experiment_summary reported "cbf_status not
    recorded" on runs where it was publishing, because it read a column name
    experiment_logger never emitted."""
    h = set(header())
    for c in ('tcp_speed', 'qddot_nom_norm', 'qddot_safe_norm',
              'qddot_delta_norm', 'cp_min_distance', 'cbf_slack',
              'cbf_n_violated', 'cbf_fault', 'iso_stop_latched',
              'tau_sat_1', 'comm_success_min', 'min_distance'):
        assert c in h, f'experiment_summary reads {c!r} and the header lacks it'


def test_the_legacy_and_iso_columns_all_survived():
    h = header()
    for c in ('t', 'q_1', 'min_distance', 'min_h', 'valid_cbf_count',
              'comm_success_rate', 'robot_mode', 'cbf1_link',
              'cbf_S_p', 'iso_stop_latched', 'tau_sat_1'):
        assert c in h, c


# ── absent ≠ zero ────────────────────────────────────────────────────────────

def test_an_unpublished_acceleration_topic_reads_nan_not_zero():
    n = types.SimpleNamespace(
        last_qddot_nom=np.full(NUM_JOINTS, np.nan),
        last_qddot_safe=np.full(NUM_JOINTS, np.nan))
    assert all(math.isnan(v) for v in n.last_qddot_nom)
    # and the norm of an all-NaN vector must not come out as 0.0
    assert math.isnan(ExperimentLogger._norm(n.last_qddot_nom))


def test_the_norm_of_a_partly_invalid_vector_is_nan():
    v = np.zeros(NUM_JOINTS)
    v[3] = np.nan
    assert math.isnan(ExperimentLogger._norm(v))


def test_per_link_stats_count_invalid_entries_separately():
    """n_total − n_valid is the difference between "nothing was near" and
    "perception produced entries the barrier had to throw away"."""
    def pt(x=0.0, y=0.0, z=0.0):
        return types.SimpleNamespace(x=x, y=y, z=z)

    def ld(valid, d, v=(0.0, 0.0, 0.0), track=0, ph=0.0):
        # labelled_links reads closest_point_robot / closest_point_human to
        # work out the control-point label and the multi-obstacle rank, so a
        # stub has to carry them.
        return types.SimpleNamespace(
            valid=valid, distance=d, track_id=track,
            robot_link_name='fr3_link5',
            closest_point_robot=pt(0.5, 0.0, 0.5),
            closest_point_human=pt(0.5, ph - 0.25, 0.5),
            obstacle_velocity=pt(*v))

    node = types.SimpleNamespace(cp_stats={})
    msg = types.SimpleNamespace(links=[
        ld(True, 0.30, (0.0, 0.5, 0.0), track=3, ph=0.0),
        ld(True, 0.12, ph=0.1),
        ld(False, float('inf'), ph=0.2),
    ])
    ExperimentLogger.per_link_cb(node, msg)
    st = node.cp_stats
    assert st['cp_n_total'] == 3 and st['cp_n_valid'] == 2
    assert st['cp_min_distance'] == pytest.approx(0.12)
    assert st['cp_v_obs_max'] == pytest.approx(0.5)
    assert st['cp_n_tracked'] == 1


def test_an_empty_frame_leaves_the_distance_nan_not_zero():
    node = types.SimpleNamespace(cp_stats={})
    ExperimentLogger.per_link_cb(node, types.SimpleNamespace(links=[]))
    assert math.isnan(node.cp_stats['cp_min_distance'])
    assert node.cp_stats['cp_n_total'] == 0


# ── the summary ──────────────────────────────────────────────────────────────

def _rows(n=500, **cols):
    out = []
    for i in range(n):
        r = {'t': str(i / 100.0)}
        for k, v in cols.items():
            r[k] = str(v[i] if isinstance(v, list) else v)
        out.append(r)
    return out


def test_the_summary_reports_a_peak_and_when_it_happened():
    v = [0.1] * 500
    v[300] = 0.42
    val, t = S.peak(_rows(tcp_speed=v), 'tcp_speed')
    assert val == pytest.approx(0.42) and t == pytest.approx(3.0)


def test_the_summary_finds_the_worst_joint_of_a_block():
    rows = _rows(**{f'qdot_{j}': (0.1 * j) for j in range(1, 8)})
    val, _, j = S.vec_peak(rows, 'qdot')
    assert j == 7 and val == pytest.approx(0.7)


def test_a_missing_value_gets_no_timestamp_and_no_annotation():
    """"not recorded @ t = 15.42 s joint 7" reads as a measurement and is not
    one — it was printing the position of a NaN."""
    out = S.line('peak |tau|', float('nan'), 15.42, ' N·m', extra='   joint 7')
    assert 'not recorded' in out
    assert 't =' not in out and 'joint' not in out


def test_a_present_value_keeps_its_timestamp():
    out = S.line('peak |tau|', 12.5, 3.0, ' N·m', nd=2, extra='   joint 4')
    assert '12.50 N·m' in out and 't =   3.00 s' in out and 'joint 4' in out


def test_coverage_distinguishes_absent_from_zero():
    assert S.covered(_rows(tcp_speed=0.0), 'tcp_speed') == 1.0
    assert S.covered(_rows(tcp_speed='nan'), 'tcp_speed') == 0.0


def test_the_summary_survives_a_run_with_only_timestamps():
    """An early-aborted run must produce a report, not a traceback."""
    val, t = S.peak(_rows(), 'tcp_speed')
    assert math.isnan(val)
    assert math.isnan(S.vec_peak(_rows(), 'qdot')[0])


def test_the_summary_derives_the_saturation_count_from_the_per_joint_flags():
    """No redundant tau_sat_count column: the per-joint flags carry strictly
    more, and a duplicated total is one more thing that can disagree."""
    rows = [{f'tau_sat_{j}': ('1.0' if j in (2, 5) else '0.0')
             for j in range(1, 8)}]
    assert S.sat_count(rows) == [2.0]


def test_absent_saturation_flags_read_nan_not_zero():
    assert math.isnan(S.sat_count([{'t': '0.0'}])[0])
    assert math.isnan(S.sat_count([{f'tau_sat_{j}': 'nan'
                                    for j in range(1, 8)}])[0])
