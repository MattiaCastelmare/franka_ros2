"""b4 / C3 (2026-10-06): where do the OBSTACLE rows of the shield actually bend the action?

The v10c replay buffer was collected with the obstacle CBF rows ON. In a shield-off fine-tune, a transition whose
next state was produced by an obstacle-row correction comes from a different MDP. This rolls out the v10c policy
(stochastic, as when the buffer was filled) with the shield on and, every tick, runs a SHADOW filter on the same
state with NO obstacle rows (same slew anchor, everything else identical). effect = |q̈_safe − q̈_shadow| is the
part of the correction owed to the obstacle alone; its support in d_min (the obs slot the buffer stores) sets the
threshold for train.py --filter-buffer-dmin.

    python3 franka_sim/runs/eval_all/b4_shield_effect.py MODEL.zip CONFIG.yaml N_EPISODES SEED0 OUT.npz
"""
import copy, sys
import numpy as np
import yaml
from stable_baselines3 import SAC
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from franka_sim.envs.obs_layout import spec_from_config

model_p, cfg_p, n_ep, seed0, out = sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]), sys.argv[5]
cfg = yaml.safe_load(open(cfg_p))
cfg['env']['cbf_obstacle_enabled'] = True
env = FrankaCBFEnv(config=cfg)
model = SAC.load(model_p, device='cpu')
d_slot = {k: s for k, s, _ in spec_from_config(cfg).slots}['d_min']

main_filter = env.cbf.filter
shadow = copy.copy(env.cbf)
rec = []          # (d_min obs, effect, total intervention, static?)
cur = {}

def wrapped(q, qdot, qddot_nom, obstacles, **kw):
    prev = env.cbf._qddot_prev.copy()
    out_main = main_filter(q, qdot, qddot_nom, obstacles, **kw)
    shadow._qddot_prev = prev
    shadow._probs = {}
    qs, _ = type(env.cbf).filter(shadow, q, qdot, qddot_nom, [], **kw)
    cur['effect'] = float(np.linalg.norm(out_main[0] - qs))
    cur['interv'] = out_main[1].intervention
    return out_main

env.cbf.filter = wrapped
for ep in range(n_ep):
    obs, _ = env.reset(seed=seed0 + ep)
    static = env.obs_mode == 'static'
    done = False
    while not done:
        a, _ = model.predict(obs, deterministic=False)
        d_obs = float(obs[d_slot])
        obs, r, term, trunc, info = env.step(a)
        rec.append((d_obs, cur['effect'], cur['interv'], static))
        done = term or trunc
R = np.array(rec)
np.savez(out, d=R[:, 0], effect=R[:, 1], interv=R[:, 2], static=R[:, 3])
print(f'{len(R)} steps; effect>0.1 in {np.mean(R[:, 1] > 0.1):.3f} of steps')
