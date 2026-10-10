"""Outcome stats behind the slide video: one model on a list of blocking seeds, shield OFF, 10 s (same env as
b3_slide_videos.sh / record_video --held).   python3 b3_slide_eval.py MODEL CONFIG MODE SEED,SEED,... OUT.json [on]   (on = obstacle shield ON)"""
import copy, json, sys, yaml, numpy as np
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from franka_sim.scripts.evaluate_policy import _load_policy
m, cfgp, mode, seeds, out = sys.argv[1:6]
c = copy.deepcopy(yaml.safe_load(open(cfgp)))
c['env']['max_episode_steps'] = int(round(10.0 * float(c['env'].get('control_rate_hz', 100.0))))
c['env']['cbf_obstacle_enabled'] = len(sys.argv) > 6 and sys.argv[6] == 'on'
c['reward']['terminate_on_success'] = False; c['reward']['terminate_on_collision'] = True
c['task']['blocking_fraction'] = 1.0; c['obstacle']['mode'] = mode; c['obstacle']['static_fraction'] = 0
env = FrankaCBFEnv(config=c); pred, _ = _load_policy(m)
rows = []
for s in map(int, seeds.split(',')):
    o, _ = env.reset(seed=s); n = 0; dmin = 9.0; reached = False; held5 = None
    while True:
        o, _, te, tr, info = env.step(pred(o)); n += 1; dmin = min(dmin, float(info['d_min'])); reached |= info['success']
        if n == 500: held5 = (not info['collision']) and info['dist'] < env.target_tol
        if te or tr: break
    rows.append(dict(seed=s, coll=bool(info['collision']), t_end=round(n * env.dt, 2), held=(not info['collision']) and info['dist'] < env.target_tol,
                     held5=bool(held5) if held5 is not None else False, reached=bool(reached), err=round(float(info['dist']), 3), dmin=round(dmin, 3)))
json.dump(rows, open(out, 'w'))
print(out, len(rows), 'held', sum(r['held'] for r in rows), 'coll', sum(r['coll'] for r in rows), flush=True)
