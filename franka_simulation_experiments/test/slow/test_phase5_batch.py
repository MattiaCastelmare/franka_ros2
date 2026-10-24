"""Phase 5/6: one cell through the batch runner; emulated perception is exact; the report builds.

SLOW: one Gazebo run (``static`` × kf × perception:=emulated, ~20 s wall) into a
temporary batch directory, then:

* the emulated rows arrive at the frame rate (90 Hz ± 5 %), stamped with the
  capture time, and each rank-0 gap equals the TRUE gap of its control point at
  that stamp (truth.npz, 1 kHz) to 1 mm, clamped at 0;
* the RT controller was active when the episode started (the supervisor waits);
* the arm completed the task, as the offline double integrator does;
* cells.csv has the row, tagged SIM, with RTF and QP rate; the report builds,
  with the offline column, and says SIM.
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
    from simexp import analysis, batch, report, sim_config
    return analysis, batch, report, sim_config


@pytest.fixture(scope='module')
def cell():
    A, B, R, sc = _simexp()
    bd = sc.make_run_dir(name='phase5_batch_test')
    c = dict(scenario='static', condition='kf', seed=0, perception='emulated')
    csv_path = B.run_grid(bd, [c], log=lambda s: None)
    rd = os.path.join(bd, 'cells', B.cell_name(c))
    return dict(bd=bd, rd=rd, csv=csv_path, A=A, R=R,
                res=json.load(open(os.path.join(rd, 'cell_result.json'))))


def test_the_cell_ran_and_was_scored(cell):
    r = cell['res']
    assert r['ok'] and not r['timed_out'] and not r['leftovers'], r.get('error')
    assert r['closed_loop']['controller_active_at_start'] is True
    assert r['closed_loop']['task_done'] is True
    assert r['closed_loop']['reflex_verdict'] == 'PASS'


def test_emulated_rows_are_the_truth_at_the_capture_stamp(cell):
    from franka_experiments.utils.perception_msgs import labelled_links
    from franka_msgs.msg import MultiLinkDistance
    A = cell['A']
    rd = cell['rd']
    T = A.load_truth(rd)
    ep = A.load_json(os.path.join(rd, 'run_manifest_sim.json'))['episode']
    msgs = [m for _, m in A.read_bag_topics(os.path.join(rd, 'bag'), {
        '/cbf/per_link_distances': MultiLinkDistance})['/cbf/per_link_distances']]
    st = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in msgs])
    w = (st > ep['t_start_sim'] + 0.1) & (st < ep['t_end_sim'])
    rate = (w.sum() - 1) / (st[w][-1] - st[w][0])
    assert rate == pytest.approx(90.0, rel=0.05)
    labels = [str(x) for x in T['labels']]
    errs = []
    for m, t in zip(np.array(msgs, dtype=object)[w], st[w]):
        i = int(np.argmin(np.abs(T['t'] - t)))
        assert abs(T['t'][i] - t) < 0.0015
        for lbl, ld in labelled_links(m):
            if '.' in lbl or lbl not in labels:
                continue
            true_gap = max(float(T['gap'][i, labels.index(lbl)].min()), 0.0)
            errs.append(abs(ld.distance - true_gap))
    errs = np.array(errs)
    assert errs.size > 1000
    assert np.percentile(errs, 99) < 0.001, np.percentile(errs, [50, 99, 100])


def test_the_csv_row_and_the_report(cell, tmp_path):
    with open(cell['csv']) as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 1 and rows[0]['SIM'] == 'True' and rows[0]['perception'] == 'emulated'
    assert float(rows[0]['rtf']) > 0.3 and float(rows[0]['qp_rate_hz']) > 100
    path = cell['R'].write(cell['bd'], str(tmp_path / 'report.md'))
    txt = open(path).read()
    assert 'Every number below is SIM' in txt and '### D. Three columns' in txt
    assert os.path.isfile(os.path.join(cell['bd'], 'offline.json'))
    assert glob.glob(os.path.join(cell['rd'], 'compare_vobs_sim.npz'))
