"""40 seeds, hard task, success does NOT end the episode: reached-ever vs held-at-end."""
import sys, copy, yaml, numpy as np
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from franka_sim.scripts.evaluate_policy import _load_policy
run, m, cfgp, tag = sys.argv[1:5]
c = yaml.safe_load(open(cfgp)); c.setdefault('task', {})['blocking_fraction'] = 0.6
for kv in sys.argv[5:]:              # section.key=value overrides
    k, v = kv.split('='); sec, key = k.split('.')
    c.setdefault(sec, {})[key] = yaml.safe_load(v)
c.setdefault('reward', {})['terminate_on_success'] = False; c['reward']['terminate_on_collision'] = True
env = FrankaCBFEnv(config=c); pred, _ = _load_policy(f'franka_sim/models/{run}/{m}.zip')
reach = hold = coll = floor_ep = 0; err = []; t_in = []
for s in range(1000, 1040):
    o, _ = env.reset(seed=s); ever = False; n_in = 0; n = 0; fl = False
    while True:
        o, _, te, tr, info = env.step(pred(o)); n += 1
        ever |= info['success']; n_in += info['success']; fl |= info.get('floor_contact', False)
        if te or tr: break
    reach += ever; floor_ep += fl; coll += info['collision']; err.append(info['dist'])
    hold += (not info['collision']) and info['dist'] < env.target_tol; t_in.append(n_in / n)
print(f'{tag:14s} {run}/{m:11s} reached-ever {reach}/40  HELD-at-5s {hold}/40  collisions {coll}/40  floor {floor_ep}/40  '
      f'final err median {1000*np.median(err):.0f} mm  time-on-target {100*np.mean(t_in):.0f}%  (blocking misses {env.blocking_misses}, fallbacks {env.reset_fallbacks})', flush=True)
