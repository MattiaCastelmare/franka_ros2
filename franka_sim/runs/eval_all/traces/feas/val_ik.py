import sys, json, time, yaml, glob, numpy as np
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
L = [l.split() for l in open('franka_sim/runs/eval_all/traces/feas/list.txt')]
F = {}
for f in glob.glob('franka_sim/runs/eval_all/traces/feas/*.json'):
    x = json.load(open(f)); F[x['seed']] = x
def mk(on, extra={}):
    c = yaml.safe_load(open('franka_sim/models/sac_v10c/config.yaml')); c['task']['blocking_fraction'] = 0.6
    c['obstacle']['mode'] = 'static'; c['obstacle']['static_fraction'] = 0
    c['task']['target_ik_check'] = on; c['task'].update(extra)
    return FrankaCBFEnv(config=c)
off, on = mk(False), mk(True, json.loads(sys.argv[1]) if len(sys.argv) > 1 else {})
bad = []; changed = 0; t = []
for s, grp in L:
    s = int(s)
    off.reset(seed=s); t0 = time.time(); on.reset(seed=s); t.append(time.time() - t0)
    ch = not np.allclose(off._target, on._target)
    changed += ch
    if ch == F[s]['feasible']: bad.append((s, grp, F[s]['feasible'], F[s]['min_err']))
print('changed', changed, 'ik_rejects', on.ik_rejects, 'mismatch vs 41-start reference', bad)
print('reset time on: median %.3f s, max %.3f s, total %.1f s' % (np.median(t), max(t), sum(t)))
# cost on ordinary training resets (dynamic + static mix, as in training)
c = yaml.safe_load(open('franka_sim/models/sac_v10c/config.yaml')); c['task']['target_ik_check'] = True
e = FrankaCBFEnv(config=c); t0 = time.time()
for s in range(200): e.reset(seed=100000 + s)
print('training-mix resets: %.1f ms/reset, ik_rejects %d / 200, fallbacks %d' % (1000 * (time.time() - t0) / 200, e.ik_rejects, e.reset_fallbacks))
c['task']['target_ik_check'] = False; e = FrankaCBFEnv(config=c); t0 = time.time()
for s in range(200): e.reset(seed=100000 + s)
print('same, check off: %.1f ms/reset' % (1000 * (time.time() - t0) / 200))
