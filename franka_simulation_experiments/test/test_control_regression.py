import json
from pathlib import Path

import numpy as np

from simexp.control_regression import (
    SCENARIOS, cells, summarize_diagnostics, tracking_from_stamped_points)
from simexp.scenario_catalog import load, resolve


def test_protocol_has_four_separate_scenarios_and_three_reproducible_seeds():
    assert len(cells()) == 12
    cfg = load(str(Path(__file__).parents[1] / 'config/control_regression_scenarios.yaml'))
    for s in SCENARIOS:
        a, b = resolve(s, seed=2, cfg=cfg), resolve(s, seed=2, cfg=cfg)
        assert a == b and a['motion'] == 'pentagon'
    assert not resolve('control_clear', cfg=cfg)['obstacles']
    stick = resolve('control_stick', cfg=cfg)['obstacles']
    assert len(stick) == 7 and all(o['law'] == 'oscillate' for o in stick)
    assert len({tuple(o['p0']) for o in stick}) == 7


def test_diagnostic_scoring_preserves_missing_values_and_counts_every_fallback():
    fields = ['fallback_level', 'slew_conflict_joints', 'slack_obs', 'distance_capture_age_s']
    got = summarize_diagnostics([[0, 0, 1, np.nan], [2, 3, 4, .1], [0, 1, 2, .2]], fields)
    assert got['fallback_ticks'] == {'0': 2, '1': 0, '2': 1}
    assert got['slew_conflict_ticks'] == 2 and got['slew_conflict_joint_ticks'] == 4
    assert got['slack_obs']['max'] == 4.
    assert got['distance_capture_age_s']['max'] == .2
    old = summarize_diagnostics([[0], [2]], fields[:1])
    assert 'slack_obs' not in old
    assert 'distance_capture_age_s' not in old


def test_tracking_matches_header_stamps_despite_recorder_and_node_start_offsets():
    from types import SimpleNamespace as NS
    def msg(sec, x):
        return NS(header=NS(stamp=NS(sec=sec, nanosec=0)), point=NS(x=x, y=0., z=0.))
    actual = [(90., msg(12, .13)), (91., msg(13, .24))]
    desired = [(88., msg(12, .1)), (89., msg(13, .2))]
    result = tracking_from_stamped_points(actual, desired, 12.5, 13.5)
    assert result['n'] == 1
    assert abs(result['max_m']-.04) < 1e-12
