"""config/scenarios.yaml reproduces the Python definitions it ports (spec Phase 4).

The definitions are IMPORTED, not retyped: cbf_scenarios.scenarios() and
approach_envelope.cell() run with their module-level ``run_scenario`` replaced by
a recorder, so the test sees exactly the spheres, durations, goals and
perception arguments the offline experiments use.
"""
import importlib
import os
import sys

import numpy as np
import pytest
import yaml

from simexp import scenario_catalog as SC

REPO = os.path.normpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..'))
SCRIPTS = os.path.join(REPO, 'franka_experiments', 'scripts')


def _import(name):
    if SCRIPTS not in sys.path:
        sys.path.insert(0, SCRIPTS)
    return importlib.import_module(name)


@pytest.fixture(scope='module')
def cfg():
    return SC.load()


@pytest.fixture(scope='module')
def captured():
    C = _import('cbf_scenarios')
    cap = {}

    def rec(name, P, spheres, **kw):
        cap[name] = dict(spheres=[(s.p.copy(), s.v.copy(), s.r, s.pursue, s.speed)
                                  for s in spheres], **kw)
        return {}
    orig = C.run_scenario
    C.run_scenario = rec
    try:
        P = C.shipped_params()
        C.scenarios(P)
        flange = C.Filter(P).control_points(C.Q0, np.zeros(C.NV))[-1][1]
    finally:
        C.run_scenario = orig
    return dict(scen=cap, flange=flange, Q0=C.Q0.copy(), R_CAP=C.R_CAP)


def test_the_port_has_exactly_the_python_scenarios_in_order(cfg, captured):
    assert list(cfg['catalog']) == list(captured['scen'])
    assert set(cfg['catalog']) == {'static', 'around', 'static_noisy', 'approach_hold', 'slow',
                                   'slow_noisy', 'fast', 'fast_seen_early', 'ball', 'ball_slow',
                                   'wedge'}


def test_q0_and_flange(cfg, captured):
    np.testing.assert_allclose(cfg['q0'], captured['Q0'], atol=0)
    np.testing.assert_allclose(cfg['flange_q0'], captured['flange'], atol=1e-6)


@pytest.mark.parametrize('name', ['static', 'around', 'static_noisy', 'approach_hold', 'slow',
                                  'slow_noisy', 'fast', 'fast_seen_early', 'ball', 'ball_slow',
                                  'wedge'])
def test_each_scenario_matches_cbf_scenarios(cfg, captured, name):
    py = captured['scen'][name]
    r = SC.resolve(name, seed=0, cfg=cfg)
    assert r['T'] == py['T']
    goal = py.get('goal_q')
    if r['nominal']['law'] == 'cartesian':
        np.testing.assert_allclose(r['nominal']['cart_ref'], py['cart_ref'], atol=0)
        assert goal is None
    else:
        np.testing.assert_allclose(r['nominal']['goal_q'], captured['Q0'] if goal is None else goal,
                                   atol=1e-12)
        assert py.get('cart_ref') is None
    for k in ('fps', 'cam_noise', 'max_thresh'):
        if k in py:
            assert r['emulation'].get(k) == py[k], k
    for k in r['emulation']:
        assert k in py, f'{k} is not an argument cbf_scenarios passes for {name}'
    assert len(r['obstacles']) == len(py['spheres'])
    for o, (p, v, rad, pursue, speed) in zip(r['obstacles'], py['spheres']):
        np.testing.assert_allclose(o['p0'], p, atol=1e-5)
        assert o['radius'] == rad
        if pursue:
            assert o['law'] == 'pursuit' and o['speed'] == speed
        elif np.linalg.norm(v) > 0:
            assert o['law'] == 'linear'
            np.testing.assert_allclose(o['v0'], v, atol=0)
        else:
            assert o['law'] == 'static'


def test_emulation_flags_have_a_gazebo_counterpart(cfg):
    for name, sc in cfg['catalog'].items():
        em = sc.get('emulation', {})
        if em.get('cam_noise'):
            assert sc['gazebo']['depth_adapter']['range_noise'] is True, name
        if 'max_thresh' in em:
            assert sc['gazebo']['perception']['distance']['thresholds']['max_thresh'] == \
                em['max_thresh'], name


def test_throw_sweep_matches_approach_envelope_cells(cfg):
    E = _import('approach_envelope')
    C = _import('cbf_scenarios')
    assert cfg['throw_sweep']['radius'] == E.R_BALL
    assert cfg['throw_sweep']['t_sep'] == E.BASE['t_sep']
    assert cfg['throw_sweep']['gate'] == E.BASE['max_thresh']
    assert cfg['throw_sweep']['r_cap'] == C.R_CAP
    rec = {}

    def run(name, P, spheres, **kw):
        rec['s'] = [(s.p.copy(), s.v.copy(), s.r) for s in spheres]
        rec['T'] = kw['T']
        return dict(min_h=0.0, v_obs_max=0.0)
    orig = E.run_scenario
    E.run_scenario = run
    try:
        cases = [('a', 0)] + [('b', 0)] + [('c', L) for L in cfg['throw_sweep']['kinds']['c']['L']]
        for v in cfg['throw_sweep']['speeds']:
            for kind, L in cases:
                E.cell(kind, v, L, {})
                name = f'throw_{kind if kind != "c" else "c" + format(L, "g")}_v{v:g}'
                r = SC.resolve(name, cfg=cfg)
                (p, vel, rad), = rec['s']
                assert r['T'] == pytest.approx(rec['T'], abs=1e-12), name
                np.testing.assert_allclose(r['obstacles'][0]['p0'], p, atol=1e-5)
                np.testing.assert_allclose(r['obstacles'][0]['v0'], vel, atol=1e-12)
                assert r['obstacles'][0]['radius'] == rad
    finally:
        E.run_scenario = orig


def test_ballistic_throws_reach_the_flange(cfg):
    for name in SC.throw_names(cfg):
        if not name.startswith('throw_ballistic'):
            continue
        r = SC.resolve(name, cfg=cfg)
        o = r['obstacles'][0]
        v = float(name.split('_v')[-1])
        Tf = cfg['ballistic_throws']['release_distance'] / v
        p = np.asarray(o['p0']) + np.asarray(o['v0']) * Tf + 0.5 * np.asarray(o['gravity']) * Tf ** 2
        np.testing.assert_allclose(p, cfg['flange_q0'], atol=1e-9)


def test_pentagon_path_is_the_commanders(cfg):
    with open(os.path.join(REPO, 'franka_experiments', 'config', 'fr3_control.yaml')) as f:
        P = yaml.safe_load(f)['params']
    path = cfg['pentagon']['path']
    assert P['path_type'] == 'circle'
    np.testing.assert_allclose(path['center'], P['path_center_xyz'])
    assert path['radius'] == P['path_radius'] and path['cycle_time'] == P['path_cycle_time']


def test_every_scenario_resolves_and_none_starts_inside_the_barrier(cfg):
    """franka_rl's rule: forward invariance is a statement about trajectories
    that START in the safe set. Checked on TRUE gaps at Q0."""
    from simexp.truth_geometry import TruthGeometry, load_perception_config
    sys.path.insert(0, os.path.join(REPO, 'franka_simulation', 'scripts'))
    from utils import gz_robot_description as G
    _, rsp = G.build_descriptions(G.process_xacro(), initial_q=cfg['q0'])
    pc = load_perception_config(os.path.join(REPO, 'franka_experiments', 'config',
                                             'fr3_complete.yaml'))
    cps = TruthGeometry(rsp, pc).control_points(cfg['q0'])
    with open(os.path.join(REPO, 'franka_experiments', 'config', 'fr3_control.yaml')) as f:
        d_safe = yaml.safe_load(f)['params']['d_safe']
    shifted = {}
    for name in SC.all_names(cfg):
        r = SC.enforce_start_clearance(SC.resolve(name, cfg=cfg), cps, d_safe)
        chk = SC.check_start(r, cps, d_safe)
        assert chk['ok'], (name, chk)
        if r['start_shift']:
            shifted[name] = [s['shift_m'] for s in r['start_shift']]
    # The only scenario the real control-point set puts inside the barrier is
    # wedge: 9.4 mm on fr3_link8#0, so its shift is 9.4 + 1 (margin) = 10.4 mm.
    # Anything else moving, or more than 12 mm, means the catalog or the CP
    # geometry changed.
    assert set(shifted) == {'wedge'}, shifted
    assert max(shifted['wedge']) < 0.012, shifted


def test_seeds_replay_and_seed0_is_the_port(cfg):
    a, b = SC.resolve('slow', seed=3, cfg=cfg), SC.resolve('slow', seed=3, cfg=cfg)
    assert a == b
    assert SC.resolve('slow', seed=0, cfg=cfg)['start_offset_s'] == 0.0
    assert 0.0 <= a['start_offset_s'] < 1.0 / 90.0
