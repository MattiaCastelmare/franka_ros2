"""experiment_logger's truth_topic ships OFF, and with it off nothing moved.

The Gazebo replica (franka_simulation_experiments) logs true per-control-point
gaps through ``truth_topic``. With the parameter empty, the default, the header
must be the one the logger wrote at 94cd7b0 (golden, data/), and no row may
carry a truth_* field. With it set, the truth columns come last and only the
truth columns are added.
"""
import json
import os
import types

import numpy as np
import pytest

from franka_experiments.nodes.experiment_logger import ExperimentLogger

GOLDEN = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data',
                      'experiment_log_header_94cd7b0.json')


@pytest.fixture(scope='module')
def golden():
    with open(GOLDEN) as f:
        return json.load(f)['headers']


@pytest.mark.parametrize('k', [3, 12])
def test_off_header_is_the_one_at_94cd7b0(golden, k):
    # the shape every existing caller uses (no truth attribute at all) ...
    assert ExperimentLogger._make_header(types.SimpleNamespace(max_cbf_entries=k)) == golden[str(k)]
    # ... and the shape a node with the parameter empty has (truth_n_cp = 0)
    assert ExperimentLogger._make_header(
        types.SimpleNamespace(max_cbf_entries=k, truth_n_cp=0)) == golden[str(k)]


def test_on_header_appends_only_truth_columns(golden):
    h = ExperimentLogger._make_header(types.SimpleNamespace(max_cbf_entries=12, truth_n_cp=11))
    assert h[:len(golden['12'])] == golden['12']
    extra = h[len(golden['12']):]
    assert extra == ['truth_t', 'truth_min_gap', 'truth_h_min'] + [f'truth_gap_{i}' for i in range(11)]
    assert len(h) == len(set(h))


def test_off_rows_carry_no_truth_field():
    node = types.SimpleNamespace(truth_n_cp=0, last_truth=np.full(3, np.nan))
    assert ExperimentLogger._truth_fields(node) == {}
    assert ExperimentLogger._truth_fields(types.SimpleNamespace()) == {}


def test_truth_message_fills_the_columns_and_absent_reads_nan():
    node = types.SimpleNamespace(truth_n_cp=3, last_truth=np.full(6, np.nan))
    f = ExperimentLogger._truth_fields(node)
    assert set(f) == {'truth_t', 'truth_min_gap', 'truth_h_min', 'truth_gap_0',
                      'truth_gap_1', 'truth_gap_2'}
    assert all(np.isnan(v) for v in f.values())        # absent != measured-as-zero
    ExperimentLogger.truth_cb(node, types.SimpleNamespace(
        data=[12.5, 3.0, 0.40, 0.31, 0.52, 0.31, 0.16]))
    f = ExperimentLogger._truth_fields(node)
    assert f['truth_t'] == 12.5 and f['truth_min_gap'] == 0.31 and f['truth_h_min'] == 0.16
    assert [f[f'truth_gap_{i}'] for i in range(3)] == [0.40, 0.31, 0.52]
    # a malformed message changes nothing
    ExperimentLogger.truth_cb(node, types.SimpleNamespace(data=[1.0, 5.0, 0.1]))
    assert ExperimentLogger._truth_fields(node)['truth_t'] == 12.5
