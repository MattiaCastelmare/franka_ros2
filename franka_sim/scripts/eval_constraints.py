"""Task + constraint evaluation of one controller on a seed range.

    python3 -m franka_sim.scripts.eval_constraints --controller onnx:franka_sim/models/sac_c2_ens_off/ens_off_big11.onnx \\
        --config franka_sim/models/sac_c2_ens_off/config.yaml --mode alone --scenario static --seeds 3000:100 --out x.json

--controller   onnx:PATH | zip:PATH | baseline:cartesian_pd | zero
--mode         alone        env.shield false: nothing between the action and the torque chain
               obstacle_off the old "shield OFF": QP on (joint box, slew, workspace, floor,
                            base keep-out), obstacle rows off
               full         the deployed shield: QP with the obstacle rows on

Same episode settings as runs/eval_all/test_traces.py (blocking 0.6, 5 s, no
termination on success, termination on obstacle collision, one obstacle mode
per run), so held/coll match the existing traces. Constraint violations are
measured every tick (envs/constraints.py) and never end the episode here.
"""
from __future__ import annotations

import argparse
import json
import os

import numpy as np
import yaml

from franka_sim.envs.constraints import FAMILIES
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv


def build_env(cfg_path, mode, scenario, overrides=()):
    c = yaml.safe_load(open(cfg_path))
    c.setdefault('task', {})['blocking_fraction'] = 0.6
    c.setdefault('reward', {})['terminate_on_success'] = False
    c['reward']['terminate_on_collision'] = True
    c.setdefault('obstacle', {})['mode'] = 'static' if scenario == 'static' else 'sinusoidal'
    c['obstacle']['static_fraction'] = 0
    e = c.setdefault('env', {})
    e['cbf_obstacle_on_prob'] = None
    e['shield'] = mode != 'alone'
    e['cbf_obstacle_enabled'] = mode == 'full'
    c['constraints'] = {**(c.get('constraints') or {}), 'enabled': True, 'terminate': False}
    for kv in overrides:
        k, v = kv.split('=', 1); *path, key = k.split('.'); d = c
        for p in path:
            d = d.setdefault(p, {})
        d[key] = yaml.safe_load(v)
    return FrankaCBFEnv(config=c)


def make_controller(spec, env):
    kind, _, arg = spec.partition(':')
    if kind == 'zero':
        return lambda obs: np.zeros(7, np.float32), None
    if kind == 'baseline':
        from franka_sim.baselines import CartesianPDBaseline
        b = CartesianPDBaseline(env)
        return b, b.reset
    from franka_sim.scripts.evaluate_policy import _load_policy
    return _load_policy(arg)[0], None


def run_episode(env, ctrl, reset_hook, seed):
    obs, _ = env.reset(seed=seed)
    if reset_hook:
        reset_hook()
    n = 0; ever = False
    cnt = {k: 0 for k in FAMILIES}; mx = {k: 0.0 for k in FAMILIES}
    sig_min, gap_min, gap_pair, dmin = 9.0, 9.0, '', 9.0
    while True:
        obs, _, te, tr, info = env.step(ctrl(obs)); n += 1
        ever |= info['success']
        dmin = min(dmin, float(info['d_min']))
        for k, v in info['constraint_excess'].items():
            if v > 0:
                cnt[k] += 1; mx[k] = max(mx[k], v)
        sig_min = min(sig_min, info['sigma_min'])
        if info['self_gap'] < gap_min:
            gap_min, gap_pair = info['self_gap'], info['self_pair']
        if te or tr:
            break
    held = (not info['collision']) and info['dist'] < env.target_tol
    return dict(seed=seed, steps=n, reached=bool(ever), held=bool(held), coll=bool(info['collision']),
                err_f=round(float(info['dist']), 4), dmin=round(dmin, 4),
                viol_steps=cnt, viol_max={k: round(v, 4) for k, v in mx.items()},
                sigma_min=round(sig_min, 4), self_gap=round(gap_min, 4), self_pair=gap_pair)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--controller', required=True)
    ap.add_argument('--config', required=True)
    ap.add_argument('--mode', choices=('alone', 'obstacle_off', 'full'), required=True)
    ap.add_argument('--scenario', choices=('static', 'dynamic'), required=True)
    ap.add_argument('--seeds', default='3000:100')
    ap.add_argument('--set', action='append', default=[])
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    env = build_env(a.config, a.mode, a.scenario, a.set)
    ctrl, hook = make_controller(a.controller, env)
    s0, ns = map(int, a.seeds.split(':'))
    eps = [run_episode(env, ctrl, hook, s) for s in range(s0, s0 + ns)]
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    json.dump(dict(controller=a.controller, config=a.config, mode=a.mode, scenario=a.scenario,
                   overrides=a.set, eps=eps), open(a.out, 'w'), separators=(',', ':'))
    print(a.out, 'held', sum(e['held'] for e in eps), 'coll', sum(e['coll'] for e in eps),
          'any-viol', sum(any(e['viol_steps'][k] for k in FAMILIES) for e in eps), flush=True)


if __name__ == '__main__':
    main()
