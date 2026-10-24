import numpy as np, yaml
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
for name, cfg in [('v4_hard', 'franka_sim/runs/eval_all/sac_v4_hard.yaml'),
                  ('v9', 'franka_sim/models/sac_v9/config.yaml')]:
    env = FrankaCBFEnv(config=yaml.safe_load(open(cfg)))
    d = []
    for s in range(1000, 1040):
        env.reset(seed=s); d.append(np.linalg.norm(env._ee_pos() - env._target))
    d = np.array(d)
    print(name, 'obs', env.observation_space.shape, 'd0 mean %.3f median %.3f min %.3f  <0.10m: %d/40' % (d.mean(), np.median(d), d.min(), (d < 0.10).sum()),
          'ee_site', env.cfg['env'].get('ee_site'), 'tol', env.target_tol)
    print('  task cfg:', {k: v for k, v in env.cfg.get('task', {}).items()})
