"""Validate scripts/shield_buffer_mask.py on live rollouts (b4 / C3, 2026-10-06).

Offline obstacle-row effect |q̈_on − q̈_off| must match the live one (shadow filter, as in b4_shield_effect.py) in
shield-ON rollouts; err_exec = |q̈_off − q̈_exec| must be ≈ 0 in shield-OFF rollouts (reconstruction is exact).
    python3 franka_sim/runs/eval_all/b4_validate_mask.py MODEL.zip CONFIG.yaml N_EP
"""
import copy, sys
import numpy as np
import yaml
from stable_baselines3 import SAC
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from franka_sim.scripts.shield_buffer_mask import replay_errors

model = SAC.load(sys.argv[1], device='cpu')
cfg0 = yaml.safe_load(open(sys.argv[2]))
n_ep = int(sys.argv[3])

for shield in (False, True):
    cfg = copy.deepcopy(cfg0)
    cfg['env']['cbf_obstacle_enabled'] = shield
    env = FrankaCBFEnv(config=cfg)
    main_filter = env.cbf.filter
    shadow = copy.copy(env.cbf)
    cur = {}

    def wrapped(q, qdot, qddot_nom, obstacles, **kw):
        prev = env.cbf._qddot_prev.copy()
        out = main_filter(q, qdot, qddot_nom, obstacles, **kw)
        shadow._qddot_prev = prev
        shadow._probs = {}
        qs, _ = type(env.cbf).filter(shadow, q, qdot, qddot_nom, [], **kw)
        cur['effect'] = float(np.linalg.norm(out[0] - qs))
        return out
    env.cbf.filter = wrapped
    O, X, A, D, E = [], [], [], [], []
    for ep in range(n_ep):
        obs, _ = env.reset(seed=9000 + ep)
        done = False
        while not done:
            a, _ = model.predict(obs, deterministic=False)
            nxt, r, term, trunc, info = env.step(a)
            done = term or trunc
            O.append(obs); X.append(nxt); A.append(a); D.append(done); E.append(cur['effect'])
            obs = nxt
    O, X, A, D, E = map(np.asarray, (O, X, A, D, E))
    eff, err = replay_errors(cfg, O, X, A, D, np.arange(len(O)), workers=1)
    eff, err, E = eff[1:], err[1:], E[1:]
    msg = f'shield {"ON " if shield else "OFF"}: n={len(err)}  err_exec p50 {np.median(err):.2e} p99 {np.percentile(err, 99):.2e} max {err.max():.2e}'
    if shield:
        msg += (f' | offline effect>0.5 {np.mean(eff > 0.5):.3f}  live {np.mean(E > 0.5):.3f}  '
                f'agree {np.mean((eff > 0.5) == (E > 0.5)):.4f}  max|diff| {np.abs(eff - E).max():.2e}')
    print(msg)
