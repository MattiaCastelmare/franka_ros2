"""c1 probe: why do single SAC policies reach the target and then drift off?
Runs ft_prec 3.75M on its reached-not-held seeds with counterfactual variants.
    python3 franka_sim/runs/eval_all/c1/probe_hold.py SC SEED[,SEED...]
"""
import sys, yaml, numpy as np, torch
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from stable_baselines3 import SAC
torch.set_num_threads(1)
sc, seeds = sys.argv[1], [int(s) for s in sys.argv[2].split(',')]
M = 'franka_sim/models/'
c = yaml.safe_load(open(M + 'sac_v11_ft_prec/config.yaml'))
c['task']['blocking_fraction'] = 0.6
c['reward']['terminate_on_success'] = False; c['reward']['terminate_on_collision'] = True
c['obstacle']['mode'] = 'static' if sc == 'static' else 'sinusoidal'; c['obstacle']['static_fraction'] = 0
env = FrankaCBFEnv(config=c)
single = SAC.load(M + 'sac_v11_ft_prec/checkpoints/sac_3750000_steps.zip', device='cpu')
ens = [SAC.load(M + f'{r}/checkpoints/sac_{s}_steps.zip', device='cpu')
       for r in ('sac_v11_ft_prec', 'sac_v11_ft_lr1e4', 'sac_v11_ft_lr1e4_s2') for s in (3500000, 3750000, 4000000)]
def det(m, o): return m.predict(o, deterministic=True)[0]
def std(m, o):
    with torch.no_grad():
        mu, ls, _ = m.actor.get_action_dist_params(torch.as_tensor(o[None]))
    return float(torch.exp(ls).mean())
variants = {
  'single': lambda o, k: det(single, o),
  'ens9': lambda o, k: np.mean([det(m, o) for m in ens], 0),
  # freeze the obstacle slots (centre + d_min) at their value at first reach when the obstacle is far
  'single_frozen_obs': None,
}
for seed in seeds:
    for name in ('single', 'ens9', 'single_frozen_obs'):
        o, _ = env.reset(seed=seed); reached_at = None; frozen = None; stds = []; amags = []; errs = []
        for k in range(500):
            oo = o.copy()
            if name == 'single_frozen_obs' and frozen is not None: oo[20:24] = frozen
            a = variants['ens9'](oo, k) if name == 'ens9' else det(single, oo)
            if reached_at is not None: stds.append(std(single, oo)); amags.append(np.abs(a).mean())
            o, r, te, tr, info = env.step(a); errs.append(info['dist'])
            if reached_at is None and info['dist'] < 0.05:
                reached_at = k
                if info['d_min'] > 0.2: frozen = o[20:24].copy()
            if te or tr: break
        e = np.array(errs)
        print(f'{sc} seed {seed} {name:18s} reach@{reached_at} final err {e[-1]:.3f} '
              f'err@[1,2,3,4,5]s {np.round(e[[99,199,299,399,len(e)-1]],3)} '
              f'policy std after reach {np.mean(stds) if stds else 0:.3f} |a| {np.mean(amags) if amags else 0:.3f} '
              f'qdot_end {np.linalg.norm(env._qdot):.3f}', flush=True)
