import yaml, mujoco
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
env = FrankaCBFEnv(config=yaml.safe_load(open('franka_sim/runs/eval_all/sac_v4_static.yaml'))); m = env.model
for g in range(m.ngeom):
    b = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
    if b in ('world', 'fr3_hand', 'fr3_link7', 'fr3_link5', 'fr3_link4', 'fr3_link6', 'fr3_leftfinger', 'fr3_rightfinger', 'left_finger', 'right_finger') or g < 2:
        print(g, b, mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g), 'type', m.geom_type[g], 'contype', m.geom_contype[g], 'conaff', m.geom_conaffinity[g], 'group', m.geom_group[g])
