"""relabel_buffer must reproduce the new env's reward on the same trajectory."""
import copy, yaml, numpy as np
from stable_baselines3.common.buffers import ReplayBuffer
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from franka_sim.train import relabel_buffer
old = yaml.safe_load(open('franka_sim/config_v10.yaml')); old['reward']['terminate_on_collision'] = True
new = copy.deepcopy(old); new['reward'].update(w_prec=2.0, prec_scale=0.05, w_obs_margin=2.0,
                                                obs_soft_margin=0.30, obs_shaping='potential')
rng = np.random.default_rng(0); acts = rng.uniform(-1, 1, (1500, 7)).astype(np.float32)
# a policy that heads for the target so dist gets small: bias actions with a crude P-term is overkill; mix in zeros
acts[::3] = 0
def roll(cfg):
    env = FrankaCBFEnv(config=cfg); buf = ReplayBuffer(5000, env.observation_space, env.action_space, device='cpu',
                                                        handle_timeout_termination=True)
    o, _ = env.reset(seed=7); rs = []
    for a in acts:
        o2, r, te, tr, info = env.step(a); rs.append(r)
        buf.add(o[None], o2[None], a[None], np.array([r]), np.array([te or tr]), [{'TimeLimit.truncated': tr and not te}])
        o = o2
        if te or tr: o, _ = env.reset()
    return buf, np.array(rs)
b_old, r_old = roll(old); _, r_new = roll(new)
relabel_buffer(b_old, new)
err = np.abs(b_old.rewards[:len(r_new), 0] - r_new)
print('max |relabel - env| =', err.max(), ' mean added', (r_new - r_old).mean(), ' min d_min obs', b_old.observations[:1500, 0, 23].min())
assert err.max() < 1e-4
