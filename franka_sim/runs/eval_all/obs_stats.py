"""Per-block scale of the 51-D observation over 5 episodes of sac_v10_obs51/best (dynamic scenario)."""
import yaml, numpy as np
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from franka_sim.scripts.evaluate_policy import _load_policy
c = yaml.safe_load(open('franka_sim/runs/eval_all/sac_v10_obs51_dynamic.yaml'))
env = FrankaCBFEnv(config=c); pred, _ = _load_policy('franka_sim/models/sac_v10_obs51/best_model.zip')
O = []
for s in range(1000, 1005):
    o, _ = env.reset(seed=s); O.append(o)
    for _ in range(500):
        o, _, te, tr, _ = env.step(pred(o)); O.append(o)
        if te or tr: break
O = np.array(O)
blocks = [('q', 0, 7), ('qdot', 7, 14), ('ee', 14, 17), ('target', 17, 20), ('obstacle', 20, 23), ('d_min', 23, 24), ('v_obs', 24, 27)]
n_cp = (O.shape[1] - 27) // 4
blocks += [('cp_d', None, None), ('cp_n', None, None)]
cpd = O[:, 27::4][:, :n_cp]; cpn = np.concatenate([O[:, 28 + 4 * k:31 + 4 * k] for k in range(n_cp)], 1)
print('dim', O.shape[1], 'n_cp', n_cp)
for name, a, b in blocks:
    X = cpd if name == 'cp_d' else cpn if name == 'cp_n' else O[:, a:b]
    print(f'{name:9s} min {X.min():7.3f} max {X.max():7.3f} std {X.std():6.3f} |mean| {np.abs(X).mean():6.3f}  frac==0 {np.mean(X == 0):.2f}')
print('v_obs first-step |v| per episode:', np.round(np.linalg.norm(O[1::501, 24:27], axis=1), 2))
