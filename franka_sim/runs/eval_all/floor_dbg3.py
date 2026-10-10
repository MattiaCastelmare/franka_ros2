import sys, yaml, numpy as np, mujoco
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from franka_sim.scripts.evaluate_policy import _load_policy
c = yaml.safe_load(open(sys.argv[1])); c['reward']['terminate_on_success'] = False; c['env']['max_episode_steps'] = 1000
env = FrankaCBFEnv(config=c); pred, _ = _load_policy('franka_sim/models/sac_v4/final_model.zip'); m = env.model
orig = env.cbf.filter; last = {}
def spy(*a, **k):
    r = orig(*a, **k); last['i'] = r[1]; return r
env.cbf.filter = spy
o, _ = env.reset(seed=int(sys.argv[2])); n = 0
while True:
    o, _, te, tr, info = env.step(pred(o)); n += 1
    fl = [round(z - r, 3) for z, r, *_ in env._floor_pts]
    I = last['i']
    if info['floor_contact'] or not I.solved or min(fl) < 0.01:
        cons = [(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g1), mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g2), round(d, 4))
                for g1, g2, d in [(env.data.contact[i].geom1, env.data.contact[i].geom2, env.data.contact[i].dist) for i in range(env.data.ncon)]]
        print(n, 'fc', int(info['floor_contact']), 'min(z-r)', min(fl), 'solved', I.solved, 'brake', I.braking, 'slack %.2f' % I.slack, cons)
    if te or tr: break
