import sys; sys.path.insert(0, '/ros2_ws/src/franka_sim/tests')
import numpy as np
from test_b4_finetune import _cfg, _rollout, _buf, _fill
from franka_sim.scripts.shield_buffer_mask import buffer_mask
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
cfg = _cfg(cbf_obstacle_enabled=False)
rows = _rollout(cfg, 400, seed=7)
buf = _buf(cfg, 1000); _fill(buf, rows)
valid, err = buffer_mask(cfg, buf, tol=0.5, workers=1)
bad = np.where(~valid)[0]
print('bad', bad, np.round(err[bad], 2))
# replay live to look at floor contact / limits at those steps
env = FrankaCBFEnv(config=cfg); rng = np.random.default_rng(7); env.reset(seed=7)
for t in range(max(bad) + 1):
    a = rng.uniform(-1, 1, 7).astype(np.float32)
    q = env._q; qd = env._qdot
    _, _, term, trunc, info = env.step(a)
    if t in bad:
        print(t, 'floor', info['floor_contact'], 'ncon', env.data.ncon, 'q-qmin', np.round(q - env.q_min, 3).min(), 'qmax-q', np.round(env.q_max - q, 3).min())
