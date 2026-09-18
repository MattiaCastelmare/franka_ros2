"""The ISO status contract, the CBFDIAG fields and the CSV columns (Step 10).

An ISO quantity that exists only inside a node is not observable, and a claim
about separation distance that cannot be read back off a bag is not a claim.
Three surfaces carry them, and all three have to agree:

* ``/NS_1/cbf_status`` data[5..8], APPENDED so the two existing positional
  consumers keep working;
* the CBFDIAG line's ``sp= vcap= vcls= isostop=``;
* ``experiment_logger``'s CSV columns.

The last of those is the easiest to break silently: a header and a row that
disagree make ``csv.DictWriter`` raise mid-run, on the robot, after the
interesting part of the experiment has already happened.
"""

import types

import numpy as np
import pytest

from franka_experiments.nodes.cbf_safety_filter import CBFSafetyFilter
from franka_experiments.nodes.experiment_logger import ExperimentLogger
from franka_experiments.utils.cbf_state_rows import G_OBS, N_SLACK
from franka_experiments.utils.logging_utils import format_cbf_diag

from _cbf_builder_harness import NV, make_builder, make_obstacle, ObstacleSnap, JointSnap


# ── the status message ───────────────────────────────────────────────────────

class _StatusPub:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(list(msg.data))


class _StatusStub:
    def __init__(self, iso=None, iso_age=0.0, rows=None):
        self.P = types.SimpleNamespace(distance_timeout=0.5)
        self._iso = iso
        self._iso_stamp = 100.0 - iso_age
        self._rows = rows if rows is not None else types.SimpleNamespace(
            diag_ssm_sp=0.0, diag_ssm_cap=float('inf'))
        self._status_msg = types.SimpleNamespace(data=[0.0] * 11)
        self._status_pub = _StatusPub()
        self._now = lambda: 100.0


def _status(stub, **kw):
    CBFSafetyFilter._publish_status(stub, 3, 0.1, 0.0, 1, 0.25, **kw)
    return stub._status_pub.sent[-1]


def test_the_first_five_fields_are_unchanged():
    row = _status(_StatusStub())
    assert row[:5] == [3.0, 0.1, 0.0, 1.0, 0.25]


def test_the_message_carries_eleven_fields():
    """Append-only: data[9..10] carry the CONDITIONED v_obs the barrier was
    fed, beside the raw tracker output that is already on the wire."""
    assert len(_status(_StatusStub())) == 11


def test_the_conditioned_v_obs_comes_from_the_row_builder():
    rows = types.SimpleNamespace(diag_ssm_sp=0.0, diag_ssm_cap=float('inf'),
                                 diag_v_obs=0.42, diag_vobs_hdot=-0.13)
    row = _status(_StatusStub(rows=rows))
    assert row[9] == pytest.approx(0.42)
    assert row[10] == pytest.approx(-0.13)


def test_a_builder_without_those_diagnostics_reports_zero_not_a_crash():
    row = _status(_StatusStub(rows=types.SimpleNamespace(
        diag_ssm_sp=0.0, diag_ssm_cap=float('inf'))))
    assert row[9] == 0.0 and row[10] == 0.0


def test_with_no_monitor_the_tail_is_the_builders_own_view():
    rows = types.SimpleNamespace(diag_ssm_sp=0.42, diag_ssm_cap=0.31)
    row = _status(_StatusStub(rows=rows))
    assert row[5] == pytest.approx(0.42)
    assert row[6] == pytest.approx(0.31)
    assert row[8] == 0.0, 'no monitor, nothing latched'


def test_a_fresh_monitor_message_wins():
    """The monitor is the channel that decides; publishing the filter's second
    opinion next to it would make the log unreadable exactly when it matters."""
    rows = types.SimpleNamespace(diag_ssm_sp=0.42, diag_ssm_cap=0.31)
    row = _status(_StatusStub(iso=[1.0, 1.0, 0.90, 0.05, 1.20], rows=rows))
    assert row[5:9] == pytest.approx([0.90, 0.05, 1.20, 1.0])


def test_a_stale_monitor_message_does_not():
    rows = types.SimpleNamespace(diag_ssm_sp=0.42, diag_ssm_cap=0.31)
    row = _status(_StatusStub(iso=[1.0, 1.0, 0.90, 0.05, 1.20],
                              iso_age=5.0, rows=rows))
    assert row[5] == pytest.approx(0.42)
    assert row[8] == 0.0


def test_v_closing_max_is_measured_from_the_obstacle_rows():
    b = make_builder(obstacle_velocity_enabled=False)
    qdot = np.array([0.6, -0.4, 0.5, 0.3, -0.5, 0.4, 0.2])
    con = b.build(JointSnap(np.zeros(NV), qdot, 0.0),
                  ObstacleSnap((make_obstacle(d=0.25, cp_label='fr3_link5#0'),),
                               0.0, 0.0), 0.0)
    row = _status(_StatusStub(), qdot=qdot, con=con)
    sep = con.A @ qdot
    expect = max(float(-np.min(sep[con.group == G_OBS])), 0.0)
    assert row[7] == pytest.approx(expect)


def test_v_closing_max_is_never_negative():
    """A purely separating scene reports 0, not a negative "closing" speed."""
    row = _status(_StatusStub(), qdot=np.zeros(NV), con=None)
    assert row[7] == 0.0


# ── the CBFDIAG line ─────────────────────────────────────────────────────────

class _Rows:
    diag_h_hold = diag_v_obs = diag_vapp = diag_hbrake = 0.0
    diag_hunc = diag_hlat = diag_esc_w = diag_outrun_r = diag_outrun_w = 0.0
    diag_sigma = float('nan')
    diag_w = diag_wq = None
    diag_ssm_sp = 0.0
    diag_ssm_cap = float('inf')


def _line(rows=None, **kw):
    b = make_builder()
    qdot = np.zeros(NV)
    con = b.build(JointSnap(np.zeros(NV), qdot, 0.0),
                  ObstacleSnap((make_obstacle(d=0.25),), 0.0, 0.0), 0.0)
    n = con.A.shape[0]
    return format_cbf_diag(
        now=1.0, con=con, rows=rows or _Rows(), caps=(0.0, 0.0, 0.0, 0.0),
        h_qp=np.zeros(n), qdot=qdot, qdot_cbf=qdot,
        qddot_safe=np.zeros(NV), qddot_nom=np.zeros(NV), qddot_real=np.zeros(NV),
        slack=np.zeros(N_SLACK), n_active_cps=0, vel_ratio=np.zeros(NV),
        vel_bite=np.zeros(NV, bool), slew_bite=np.zeros(NV, bool), cap_age=0.0,
        **kw)


def test_the_diag_line_carries_the_four_iso_fields():
    line = _line()
    for field in ('sp=', 'vcap=', 'vcls=', 'isostop='):
        assert field in line, field


def test_the_iso_fields_read_their_off_state_with_the_flags_off():
    line = _line()
    assert 'sp=0.000' in line
    assert 'vcap=inf' in line, 'no SSM cap in force must not print inf.000'
    assert 'vcls=0.000' in line
    assert 'isostop=0' in line


def test_the_iso_fields_carry_real_values_when_there_are_some():
    class R(_Rows):
        diag_ssm_sp = 0.912
        diag_ssm_cap = 0.047
    line = _line(rows=R(), iso_v_closing=1.234, iso_stop=1.0)
    assert 'sp=0.912' in line and 'vcap=0.047' in line
    assert 'vcls=1.234' in line and 'isostop=1' in line


def test_the_line_is_still_one_line():
    assert '\n' not in _line()


# ── the CSV ──────────────────────────────────────────────────────────────────

def test_every_column_the_sample_writes_is_in_the_header():
    """A header and a row that disagree make csv.DictWriter raise mid-run, on
    the robot, after the interesting part of the experiment has happened."""
    stub = types.SimpleNamespace(max_cbf_entries=3)
    header = ExperimentLogger._make_header(stub)
    for col in ('cbf_S_p', 'cbf_v_cap_min', 'cbf_v_closing_max', 'cbf_iso_stop',
                'iso_stop_latched', 'iso_trip_reason', 'iso_S_p',
                'iso_v_cap_min', 'iso_v_closing_max', 'iso_stop_count',
                'tau_sat_1', 'tau_sat_7'):
        assert col in header, col
    assert len(header) == len(set(header)), 'duplicate column'


def test_the_legacy_columns_are_all_still_there():
    stub = types.SimpleNamespace(max_cbf_entries=3)
    header = ExperimentLogger._make_header(stub)
    for col in ('t', 'q_1', 'min_distance', 'min_h', 'valid_cbf_count',
                'comm_success_rate', 'robot_mode', 'cbf1_link'):
        assert col in header, col


def test_the_stop_counter_counts_edges_not_ticks():
    n = types.SimpleNamespace(last_iso_latched=float('nan'), iso_stop_count=0)
    msg = types.SimpleNamespace(data=[1.0, 1.0, 0.9, 0.0, 1.2])
    for _ in range(10):
        ExperimentLogger.iso_safety_cb(n, msg)
    assert n.iso_stop_count == 1
    ExperimentLogger.iso_safety_cb(n, types.SimpleNamespace(
        data=[0.0, 0.0, 0.0, 1.0, 0.0]))
    ExperimentLogger.iso_safety_cb(n, msg)
    assert n.iso_stop_count == 2


def test_a_short_cbf_status_leaves_the_iso_columns_alone():
    """An older filter publishes five elements. That must read as 'no data',
    not as zeros."""
    n = types.SimpleNamespace(last_cbf_sp=float('nan'), last_cbf_vcap=float('nan'),
                              last_cbf_vcls=float('nan'),
                              last_cbf_isostop=float('nan'))
    ExperimentLogger.cbf_status_cb(n, types.SimpleNamespace(
        data=[1.0, 0.0, 0.0, 0.0, 0.3]))
    assert np.isnan(n.last_cbf_sp)
