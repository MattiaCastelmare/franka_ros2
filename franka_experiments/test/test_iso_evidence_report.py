"""Verdict logic of scripts/iso_evidence_report.py.

This script is the thing that will be pointed at a real run and asked "did the
cell respect the limits?". Getting a verdict wrong in either direction is bad in
a specific way:

* a false PASS tells someone a limit held when it did not;
* a false FAIL on a foregone conclusion (deviation D1, where the configured
  intrusion distance exceeds the arm's reach) buries the findings that ARE about
  the run, and sends the operator tuning something no tuning can fix.

So the tests below pin the four-way verdict — PASS / FAIL / INCONCLUSIVE /
STRUCTURAL — on synthetic rows, plus the coverage floor that stops a check from
claiming PASS on data nobody recorded.
"""

import importlib.util
import math
import os

import pytest

_SPEC = importlib.util.spec_from_file_location(
    'iso_evidence_report',
    os.path.join(os.path.dirname(os.path.realpath(__file__)), '..',
                 'scripts', 'iso_evidence_report.py'))
R = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(R)


def manifest(**over):
    man = {
        'sample_rate_hz': 100.0,
        'iso_params': {'iso_v_pfl': 0.68, 'iso_tcp_reduced_speed': 0.25,
                       'iso_a_stop': 1.0, 'iso_speed_tol': 0.05,
                       'iso_brake_frac_min': 0.5, 'iso_c_intrusion': 0.10,
                       'iso_z_depth': 0.06, 'iso_z_robot': 0.01},
        'limits': {'d_safe': 0.15, 'floor_c_zd_zr': 0.17,
                   'velocity_box_margin': 0.6, 'robot_reach_m': 0.855},
        'iso_layer_active': {'iso_enabled': True, 'iso_mode': 'automatic'},
    }
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(man.get(k), dict):
            man[k] = dict(man[k], **v)
        else:
            man[k] = v
    return man


def rows(n=1000, **cols):
    """n samples at 100 Hz; every column constant unless a list is given."""
    out = []
    for i in range(n):
        r = {'t': i / 100.0}
        for k, v in cols.items():
            r[k] = v[i] if isinstance(v, list) else v
        out.append(r)
    return out


# ── workspace feasibility: the D1 gate ───────────────────────────────────────

def test_a_floor_inside_the_reach_passes():
    res = R.check_workspace_feasibility(manifest(), rows(), [])
    assert res.verdict == R.PASS


def test_a_floor_beyond_the_reach_is_structural_not_a_failure():
    man = manifest(limits={'floor_c_zd_zr': 0.92},
                   iso_params={'iso_c_intrusion': 0.85})
    res = R.check_workspace_feasibility(man, rows(), [])
    assert res.verdict == R.STRUCT
    assert 'UNACHIEVABLE' in res.headline
    body = ' '.join(res.detail)
    assert 'D1' in body and 'tuning' in body
    assert 'detection capability' in body      # names the only way out


# ── separation: structural vs actionable ─────────────────────────────────────

def test_a_positive_margin_passes():
    res = R.check_separation(manifest(), rows(margin_min=0.05,
                                             margin_min_label='cp'), [])
    assert res.verdict == R.PASS


def test_a_negative_margin_fails_when_the_floor_is_reachable():
    res = R.check_separation(manifest(), rows(margin_min=-0.02,
                                             margin_min_label='cp'), [])
    assert res.verdict == R.FAIL


def test_under_d1_a_margin_negative_only_because_of_c_is_structural():
    """The motion terms held; C alone put the margin under zero."""
    man = manifest(limits={'floor_c_zd_zr': 0.92},
                   iso_params={'iso_c_intrusion': 0.85})
    # margin = -0.80, so margin + C = +0.05: positive once C is taken out.
    res = R.check_separation(man, rows(margin_min=-0.80, margin_min_label='cp'), [])
    assert res.verdict == R.STRUCT
    assert 'the motion terms held' in res.headline
    assert any('EXCLUDING C' in d for d in res.detail)


def test_under_d1_a_margin_negative_even_without_c_still_fails():
    """This one is a real finding and must not be excused as D1."""
    man = manifest(limits={'floor_c_zd_zr': 0.92},
                   iso_params={'iso_c_intrusion': 0.85})
    res = R.check_separation(man, rows(margin_min=-0.90, margin_min_label='cp'), [])
    assert res.verdict == R.FAIL
    assert 'not D1' in res.headline


def test_separation_is_inconclusive_without_data():
    res = R.check_separation(manifest(), rows(margin_min=float('nan')), [])
    assert res.verdict == R.INCONC


# ── speed ceilings ───────────────────────────────────────────────────────────

def test_tcp_under_the_pfl_ceiling_passes():
    assert R.check_tcp_speed(manifest(), rows(tcp_speed_max=0.5), []).verdict == R.PASS


def test_tcp_over_the_pfl_ceiling_fails_and_reports_the_peak():
    v = [0.5] * 1000
    v[400:410] = [0.95] * 10                 # 100 ms excursion
    res = R.check_tcp_speed(manifest(), rows(tcp_speed_max=v), [])
    assert res.verdict == R.FAIL
    assert '0.950' in res.headline
    assert any('0.10 s' in d for d in res.detail)


def test_reduced_speed_is_not_checked_when_it_was_not_claimed():
    res = R.check_reduced_speed(manifest(), rows(tcp_speed_max=0.9), [])
    assert res.verdict == R.NOTLOG


def test_reduced_speed_is_checked_when_the_run_claimed_it():
    man = manifest(iso_layer_active={'iso_mode': 'reduced'})
    assert R.check_reduced_speed(man, rows(tcp_speed_max=0.9), []).verdict == R.FAIL
    assert R.check_reduced_speed(man, rows(tcp_speed_max=0.2), []).verdict == R.PASS


def test_the_joint_box_uses_the_running_maximum_not_the_sample():
    """A 10 ms excursion between samples must still be caught."""
    r = [0.3] * 1000
    r[500] = 0.75
    assert R.check_joint_speed(manifest(), rows(qdot_ratio_max_run=r,
                                               qdot_ratio_joint=4),
                               []).verdict == R.FAIL


# ── coverage floor ───────────────────────────────────────────────────────────

def test_a_check_will_not_claim_pass_on_data_nobody_recorded():
    """A limit nobody measured is not a limit that held."""
    v = [float('nan')] * 900 + [0.1] * 100        # 10 % coverage
    assert R.check_tcp_speed(manifest(), rows(tcp_speed_max=v), []).verdict == R.INCONC


def test_a_short_run_is_inconclusive_whatever_it_contains():
    res = R.check_integrity(manifest(), rows(200, tcp_speed=0.1, d_min=0.3), [])
    assert res.verdict == R.INCONC
    assert 'too short' in res.headline


def test_a_run_with_dead_perception_is_inconclusive():
    res = R.check_integrity(manifest(),
                            rows(3000, tcp_speed=0.1, d_min=float('nan')), [])
    assert res.verdict == R.INCONC
    assert 'perception' in res.headline


# ── stops ────────────────────────────────────────────────────────────────────

def test_no_stop_in_the_run_is_inconclusive_not_a_pass():
    res = R.check_stops(manifest(), rows(tcp_speed=0.3), [])
    assert res.verdict == R.INCONC
    assert 'Provoke one' in res.headline


def test_a_stop_slower_than_iso_a_stop_fails():
    # 0.5 m/s bled off over 2 s => 0.25 m/s², well under the assumed 1.0.
    v = [0.5] * 1000
    for i in range(500, 700):
        v[i] = max(0.0, 0.5 - (i - 500) / 100.0 * 0.25)
    ev = [{'t': '5.0', 'kind': 'iso_stop_latched', 'detail': 'trip_reason=1'},
          {'t': '9.0', 'kind': 'iso_stop_cleared', 'detail': ''}]
    res = R.check_stops(manifest(), rows(tcp_speed=v), ev)
    assert res.verdict == R.FAIL
    assert 'optimistic' in res.headline


# ── events ───────────────────────────────────────────────────────────────────

def test_episodes_pair_rising_and_falling_edges():
    ev = [{'t': '1.0', 'kind': 'on', 'detail': 'a'},
          {'t': '2.0', 'kind': 'off', 'detail': ''},
          {'t': '5.0', 'kind': 'on', 'detail': 'b'}]
    eps = R.ev_episodes(ev, 'on', 'off')
    assert eps == [(1.0, 2.0, 'a'), (5.0, None, 'b')]


def test_an_unclosed_episode_is_kept_with_no_end():
    eps = R.ev_episodes([{'t': '1.0', 'kind': 'on', 'detail': ''}], 'on', 'off')
    assert len(eps) == 1 and eps[0][1] is None


# ── the report never claims conformity ───────────────────────────────────────

def test_the_unanswerable_list_names_the_things_logs_cannot_settle():
    joined = ' '.join(n + c + w for n, c, w in R.UNANSWERABLE)
    for must in ('PL d', 'PFMD', 'Detection capability', 'Risk assessment',
                 '13849', '6.3.3', '13855'):
        assert must in joined, must


def test_no_verdict_word_means_compliant():
    assert R.PASS == 'PASS' and R.STRUCT == 'STRUCTURAL'
    assert 'COMPLIANT' not in (R.PASS + R.FAIL + R.INCONC + R.NOTLOG + R.STRUCT)


# ── the false-PASS regression this suite exists to stop ──────────────────────

def test_an_absent_channel_is_inconclusive_not_a_pass():
    """Found live: /torque_saturation was not publishing and the report said
    "PASS, inside the joint limits in 100.0% of samples".

    The cause was in the logger, not here: tau_sat_count was initialised to
    zero, so a topic that never published still wrote a column of zeros, and a
    column of zeros reads as "measured, and fine". The logger now writes NaN
    until the first message; this pins the report's half of the contract.
    """
    nan = float('nan')
    assert R.check_saturation(manifest(), rows(tau_sat_count=nan),
                              []).verdict == R.INCONC
    assert R.check_saturation(manifest(), rows(tau_sat_count=0.0),
                              []).verdict == R.PASS


def test_the_same_holds_for_the_safety_chain_and_the_cross_check():
    nan = float('nan')
    assert R.check_chain_faults(manifest(), rows(cbf_fault=nan),
                                []).verdict == R.INCONC
    assert R.check_two_channels(manifest(), rows(d_min=nan, cbf_d_min=nan),
                                []).verdict == R.INCONC
