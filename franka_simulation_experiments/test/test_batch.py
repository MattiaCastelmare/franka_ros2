"""simexp.batch: the Phase 5 grid and the flat row."""
import collections

from simexp import batch, scenario_catalog


def test_the_phase5_grid():
    g = batch.phase5_grid()
    names = [batch.cell_name(c) for c in g]
    assert len(names) == len(set(names))
    cfg = scenario_catalog.load()
    cat = set(cfg['catalog'])
    full = [c for c in g if c['perception'] == 'true' and c['scenario'] in cat]
    assert collections.Counter(c['seed'] for c in full) == {0: 44, 1: 44, 2: 44}   # 11 x 4 x 3
    em = [c for c in g if c['perception'] == 'emulated']
    assert {c['scenario'] for c in em} == cat and {c['condition'] for c in em} == {'residual', 'kf'}
    first_non0 = next(i for i, c in enumerate(g) if c['seed'] != 0)
    assert all(c['seed'] == 1 for c in g[first_non0:first_non0 + 5])
    assert all(c['seed'] == 0 for c in g[:first_non0])


def test_flag_and_rate_variants_get_their_own_cells():
    base = dict(scenario='slow', condition='kf', seed=0, perception='emulated')
    names = {batch.cell_name(base), batch.cell_name(dict(base, emulated_fps=30)),
             batch.cell_name(dict(base, perception='true', sets={'vobs_in_hdot': False}))}
    assert len(names) == 3
    assert {c['sets'].popitem()[0] for c in batch.flags_grid()} == set(batch.TRACK_CONSUMERS)


def test_cell_name_is_a_path_component():
    n = batch.cell_name(dict(scenario='throw_a_v1.5', condition='imm+pred', seed=2,
                             perception='emulated'))
    assert n == 'throw_a_v1.5__immPpred__emul__s2' and '/' not in n


def test_flat_row_picks_the_estimator_the_cbf_used():
    r = {'ok': True, 'closed_loop': {'min_h': -0.1, 'rtf': 0.9},
         'estimator': {'used': 'v_res', 'residual': {'rms': 0.2, 'lag_ms': -40.0},
                       'tracker': {'rms': 0.05}, 'tracks': {}},
         'attribution': {'o': {'counts': {'2 untracked': 3}, 'n_parsed': 3, 'need_ms': 450.0,
                               'passes': [{'row': 500.0, 'cbf': 480.0}, {'row': 500.0, 'cbf': None},
                                          {'row': None, 'cbf': None}]}}}
    row = batch._flat(dict(scenario='slow', condition='residual', seed=0, perception='true'), r)
    assert row['SIM'] is True and row['est'] == 'v_res' and row['est_rms'] == 0.2
    assert row['trk_rms'] is None                  # tracking is off: nothing to score
    # tracking off: classified on rows and CBF lead, not '2 untracked' by construction
    assert row['passes'] == 3 and row['causes'] == '- ok:1;1 not-in-rows:1;4 too-late:1'
    r['estimator']['used'] = 'v_trk'
    row = batch._flat(dict(scenario='slow', condition='kf', seed=0, perception='true',
                           sets={'vobs_in_hdot': False}), r)
    assert row['est_rms'] == 0.05 and row['trk_rms'] == 0.05 and row['res_rms'] == 0.2
    assert row['sets'] == 'vobs_in_hdot=False' and 'sets' not in row['cell']
    assert row['causes'] == '2 untracked:3'           # a tracker condition keeps approach_probe's
