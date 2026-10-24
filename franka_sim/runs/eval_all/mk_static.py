import yaml, numpy as np
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
c = yaml.safe_load(open('franka_sim/runs/eval_all/sac_v4_hard.yaml'))
c['obstacle']['mode'] = 'static'
c['task'].update(blocking_fraction=0.6, target_clearance=0.26, target_free_always=True, reset_max_tries=3000)
yaml.safe_dump(c, open('franka_sim/runs/eval_all/sac_v4_static.yaml', 'w'))
env = FrankaCBFEnv(config=c)
for s in range(12345, 12355):
    env.reset(seed=s)
    pc = env._path_clearance(env._target, env._obs_base, env._obs_dir)
    print(s, 'path clearance %.2f' % pc, 'ON path' if pc <= env.blocking_margin else 'off path',
          ' obs-target %.2f' % np.linalg.norm(env._obs_base - env._target))
print('blocking misses', env.blocking_misses, 'fallbacks', env.reset_fallbacks)
