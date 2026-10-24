"""c1 probe: policy std (pre-tanh) by phase — transit (err>10cm) vs hold (err<5cm). ft_prec 3.75M, 20 seeds per scenario."""
import yaml, numpy as np, torch
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from stable_baselines3 import SAC
torch.set_num_threads(1)
M = 'franka_sim/models/'
m = SAC.load(M + 'sac_v11_ft_prec/checkpoints/sac_3750000_steps.zip', device='cpu')
print('target_entropy', m.target_entropy, 'ent_coef', float(torch.exp(m.log_ent_coef)))
for sc in ('dynamic', 'static'):
    c = yaml.safe_load(open(M + 'sac_v11_ft_prec/config.yaml'))
    c['obstacle']['mode'] = 'static' if sc == 'static' else 'sinusoidal'; c['obstacle']['static_fraction'] = 0
    env = FrankaCBFEnv(config=c); S = {'transit': [], 'hold': []}; A = {'transit': [], 'hold': []}; LP = {'transit': [], 'hold': []}
    for seed in range(3000, 3020):
        o, _ = env.reset(seed=seed)
        for k in range(500):
            with torch.no_grad():
                obs_t = torch.as_tensor(o[None])
                mu, ls, _ = m.actor.get_action_dist_params(obs_t)
                a_s, lp = m.actor.action_log_prob(obs_t)
            ph = 'hold' if np.linalg.norm(o[14:17] - o[17:20]) < 0.05 else ('transit' if np.linalg.norm(o[14:17] - o[17:20]) > 0.10 else None)
            if ph: S[ph].append(torch.exp(ls)[0].numpy()); A[ph].append(np.abs(np.tanh(mu[0].numpy()))); LP[ph].append(float(lp))
            o, r, te, tr, info = env.step(np.tanh(mu[0].numpy()))
            if te or tr: break
    for ph in S:
        s = np.array(S[ph]); print(f'{sc:7s} {ph:7s} n={len(s)} std per joint {np.round(s.mean(0),3)} mean {s.mean():.3f} | |mean action| {np.array(A[ph]).mean():.3f} | log_prob(squashed) mean {np.mean(LP[ph]):.2f}')
