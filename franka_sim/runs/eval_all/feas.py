"""Per seed: fraction of the obstacle cycle in which the target is reachable, vs v4's outcome."""
import yaml, numpy as np
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from franka_sim.scripts.evaluate_policy import _load_policy
c = yaml.safe_load(open('franka_sim/runs/eval_all/sac_v4_hard.yaml'))
c['reward']['terminate_on_success'] = False
env = FrankaCBFEnv(config=c); pred, _ = _load_policy('franka_sim/models/sac_v4/final_model.zip')
r_hand = env._cp_radius[-1]; d_safe = env.cbf.d_safe
T_opt = env.r_obs + d_safe + r_hand - 0.10   # hand 10 cm behind the TCP, best approach
T_pes = env.r_obs + d_safe + r_hand          # hand on top of the target
print(f'r_obs {env.r_obs:.3f} r_hand {r_hand:.2f} d_safe {d_safe:.2f}  filter thr {env.r_obs+d_safe:.2f}  T_opt {T_opt:.2f}  T_pes {T_pes:.2f}')
rows = []
for s in range(1000, 1040):
    env.reset(seed=s)
    pts = np.array([env._obstacle_at(p) for p in env._phases])
    dist = np.linalg.norm(pts - env._target, axis=1)
    free_opt, free_pes = (dist >= T_opt).mean(), (dist >= T_pes).mean()
    o, _ = env.reset(seed=s); ever = False
    while True:
        o, _, te, tr, info = env.step(pred(o)); ever |= info['success']
        if te or tr: break
    held = (not info['collision']) and info['dist'] < env.target_tol
    rows.append((s, dist.min(), free_opt, free_pes, ever, held))
    print(f'seed {s}  obs-target dist min {dist.min():.2f} max {dist.max():.2f}  free(opt) {100*free_opt:3.0f}%  free(pes) {100*free_pes:3.0f}%  reached {int(ever)} held {int(held)}', flush=True)
r = np.array(rows, dtype=float)
for name, m in [('never free (opt)', r[:,2] == 0), ('partly free (opt)', (r[:,2] > 0) & (r[:,2] < 1)), ('always free (opt)', r[:,2] == 1),
                ('always free (pes)', r[:,3] == 1)]:
    print(f'{name:20s} n={int(m.sum()):2d}  reached {int(r[m,4].sum())}  held {int(r[m,5].sum())}')
