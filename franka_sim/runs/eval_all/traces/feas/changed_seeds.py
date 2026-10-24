"""Which of the 400 held-out seeds draw a different episode with task.target_ik_check on? (same env settings as avg_eval.sh)"""
import yaml, numpy as np
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
seeds = list(range(3000, 3100)) + list(range(4000, 4300))
for mode in ('static', 'sinusoidal'):
    envs = []
    for on in (False, True):
        c = yaml.safe_load(open('franka_sim/models/sac_v10c/config.yaml')); c['task']['blocking_fraction'] = 0.6
        c['obstacle']['mode'] = mode; c['obstacle']['static_fraction'] = 0; c['task']['target_ik_check'] = on
        envs.append(FrankaCBFEnv(config=c))
    ch = []
    for s in seeds:
        for e in envs: e.reset(seed=s)
        if not (np.allclose(envs[0]._target, envs[1]._target) and np.allclose(envs[0]._obs_base, envs[1]._obs_base)
                and np.allclose(envs[0]._obs_dir, envs[1]._obs_dir)):
            ch.append(s)
    print(mode, 'changed', len(ch), ','.join(map(str, ch)), 'ik_rejects', envs[1].ik_rejects, 'fallbacks', envs[1].reset_fallbacks, flush=True)
