"""Per-step traces of the 40-seed hold test (same episodes and settings as hold.py), for plotting.

    python3 franka_sim/runs/eval_all/test_traces.py RUN MODEL CONFIG OUT.json [section.key=value ...]
SEEDS=start:count env var overrides the default 1000:40.
Every 5th step: t, EE-target error, d_min, shield intervention, EE xyz, obstacle xyz.
"""
import os, sys, json, yaml, numpy as np
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from franka_sim.scripts.evaluate_policy import _load_policy
run, m, cfgp, out = sys.argv[1:5]
c = yaml.safe_load(open(cfgp)); c.setdefault('task', {})['blocking_fraction'] = 0.6
for kv in sys.argv[5:]:   # section.key=value, any depth (randomization.latency.enabled=true)
    k, v = kv.split('='); *path, key = k.split('.'); d = c
    for p in path: d = d.setdefault(p, {})
    d[key] = yaml.safe_load(v)
c.setdefault('reward', {})['terminate_on_success'] = False; c['reward']['terminate_on_collision'] = True
env = FrankaCBFEnv(config=c)
if run == 'ens':   # action ensemble: m = 'runA/ckptA,runB/ckptB,...' (paths under models/, no .zip)
    _preds = [_load_policy(f'franka_sim/models/{p}.zip')[0] for p in m.split(',')]
    pred = lambda o: np.mean([f(o) for f in _preds], axis=0).astype(np.float32)
else:
    pred, _ = _load_policy(run if run in ('zero', 'random') else
                           f'franka_sim/models/{run}/{m}' + ('' if m.endswith('.onnx') else '.zip'))
r3 = lambda a: [round(float(x), 3) for x in a]
eps = []
s0, ns = map(int, os.environ.get('SEEDS', '1000:40').split(':'))  # SEEDS=start:count
for s in range(s0, s0 + ns):
    o, _ = env.reset(seed=s); tr = dict(t=[], err=[], d=[], iv=[], ee=[], ob=[])
    ever = False; n_in = 0; n = 0; fl = False; dmin = 9.0
    def rec(info):
        tr['t'].append(round(n * env.dt, 2)); tr['err'].append(round(float(info['dist']), 4))
        tr['d'].append(round(float(info['d_min']), 4)); tr['iv'].append(round(float(info['cbf_intervention']), 2))
        tr['ee'].append(r3(env._ee_pos())); tr['ob'].append(r3(env.data.mocap_pos[env._obs_mocap]))
    while True:
        o, _, te, tr_, info = env.step(pred(o)); n += 1
        ever |= info['success']; n_in += info['success']; fl |= info.get('floor_contact', False)
        dmin = min(dmin, float(info['d_min']))
        if n % 5 == 0 or te or tr_: rec(info)
        if te or tr_: break
    held = (not info['collision']) and info['dist'] < env.target_tol
    eps.append(dict(seed=s, reached=bool(ever), held=bool(held), coll=bool(info['collision']), floor=bool(fl),
                    tot=round(n_in / n, 3), err_f=round(float(info['dist']), 4), dmin=round(dmin, 4),
                    target=r3(env._target), **tr))
json.dump(dict(run=run, model=m, tol=float(env.target_tol), d_safe=float(env.cfg['cbf']['d_safe']), eps=eps),
          open(out, 'w'), separators=(',', ':'))
print(run, m, out, 'held', sum(e['held'] for e in eps), 'reached', sum(e['reached'] for e in eps), flush=True)
