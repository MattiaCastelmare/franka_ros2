"""Several obstacle spheres per episode (``obstacle.count_range``).

Guards:
1. a config WITHOUT extras (or with count_range [1, 1]) is bit-identical to the
   old single-sphere environment — every existing policy and seed replays;
2. the extra spheres are real hazards (geometry rows, d_min, nearest-sphere
   observation slot) but never make the episode unsolvable (start outside
   d_safe, sweep clear of the target);
3. the arm model compiled through MjSpec is the arm model of the MJCF.
"""

import os

import mujoco
import numpy as np
import pytest
import yaml

from franka_rl.envs.franka_cbf_env import (
    _DEFAULT_SCENE, _model_with_extra_obstacles, FrankaCBFEnv,
)

_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config.yaml')
SEEDS = range(777, 777 + 10)


def _cfg(**obstacle):
    with open(_CONFIG) as fh:
        c = yaml.safe_load(fh)
    c.setdefault('obstacle', {}).update(obstacle)
    c['obs'] = {'obstacle_velocity': True, 'control_point_geometry': True}
    return c


def _per_cp_distance(env, centre):
    return np.array([np.linalg.norm(env.data.xpos[b] - centre) - env.r_obs - r
                     for b, r in zip(env._cp_body, env._cp_radius)])


def test_count_range_one_replays_the_single_sphere_env_bit_identically():
    a = FrankaCBFEnv(_cfg())
    b = FrankaCBFEnv(_cfg(count_range=[1, 1]))
    acts = np.random.default_rng(0).uniform(-1, 1, (40, 7)).astype(np.float32)
    for seed in (11, 12):
        oa, _ = a.reset(seed=seed)
        ob, _ = b.reset(seed=seed)
        assert np.array_equal(oa, ob)
        for act in acts:
            ra, rb = a.step(act), b.step(act)
            assert np.array_equal(ra[0], rb[0]) and ra[1] == rb[1] and ra[2] == rb[2]
            if ra[2] or ra[3]:
                break
    a.close(); b.close()


def test_extra_spheres_are_sampled_and_feasible():
    env = FrankaCBFEnv(_cfg(count_range=[3, 3], static_fraction=0.5))
    assert len(env._extra_mocap) == 2
    n_full = 0
    for seed in SEEDS:
        obs, _ = env.reset(seed=seed)
        n_full += len(env._extra) == 2
        centres = [env.data.mocap_pos[env._obs_mocap]] + [env.data.mocap_pos[e['mocap']] for e in env._extra]
        # Every sphere starts outside the safe set and clear of the others.
        for k, c in enumerate(centres):
            assert _per_cp_distance(env, c).min() >= env.cbf.d_safe - 1e-9
            for q in centres[:k]:
                assert np.linalg.norm(c - q) >= 2 * env.r_obs
        # Extras never come within target_clearance + 0.06 of the target over their sweep.
        for e in env._extra:
            sweep = env._sweep_points(e['base'], e['dir'], e['mode'])
            assert np.linalg.norm(sweep - env._target, axis=1).min() >= env.target_clearance + 0.06 - 1e-9
        # Geometry: one row per (control point, sphere); cp block = nearest sphere; obs slot = nearest sphere.
        rows, *_, d_min, cp_geom = env._build_obstacles(env._qdot)
        assert len(rows) == env._n_cp * len(centres)
        per = np.stack([_per_cp_distance(env, c) for c in centres])        # (n_sphere, n_cp)
        assert np.allclose(cp_geom[:, 0], per.min(0))
        assert np.isclose(d_min, per.min())
        near = centres[int(np.argmin(per.min(1)))]
        assert np.allclose(env._p_obs_near, near)
        assert np.allclose(obs[20:23], near, atol=1e-6)
        assert np.isclose(obs[23], min(d_min, env.obs_spec.clip_distance), atol=1e-6)
    assert n_full >= 7, f'only {n_full}/10 episodes placed both extras'
    env.close()


def test_extras_move_and_stay_in_their_box():
    env = FrankaCBFEnv(_cfg(count_range=[2, 2], static_fraction=0.0, mode='sinusoidal'))
    moved = 0
    for seed in SEEDS:
        env.reset(seed=seed)
        if not env._extra:
            continue
        m = env._extra[0]['mocap']
        p0 = env.data.mocap_pos[m].copy()
        for _ in range(50):
            _, _, te, tr, info = env.step(np.zeros(7, np.float32))
            p = env.data.mocap_pos[m]
            assert np.all(p >= env.obs_box_min - 1e-9) and np.all(p <= env.obs_box_max + 1e-9)
            assert info['n_obstacles'] == 2
            if te or tr:
                break
        moved += np.linalg.norm(env.data.mocap_pos[m] - p0) > 1e-3
    assert moved >= 7
    env.close()


def test_mjspec_model_keeps_the_arm_identical():
    base = mujoco.MjModel.from_xml_path(_DEFAULT_SCENE)
    ext = _model_with_extra_obstacles(_DEFAULT_SCENE, 2)
    assert (ext.nq, ext.nv, ext.nu, ext.nkey) == (base.nq, base.nv, base.nu, base.nkey)
    assert ext.nmocap == base.nmocap + 2
    assert np.allclose(ext.body_mass[:base.nbody], base.body_mass)
    assert np.allclose(ext.dof_damping, base.dof_damping)
    assert np.allclose(ext.key_qpos[0], base.key_qpos[0])
    assert ext.opt.timestep == base.opt.timestep
    for k in (1, 2):
        g = mujoco.mj_name2id(ext, mujoco.mjtObj.mjOBJ_GEOM, f'obstacle_geom_{k}')
        og = mujoco.mj_name2id(base, mujoco.mjtObj.mjOBJ_GEOM, 'obstacle_geom')
        assert ext.geom_size[g, 0] == base.geom_size[og, 0]
        assert ext.geom_contype[g] == 0 and ext.geom_conaffinity[g] == 0


def test_eval_override_reaches_the_env():
    from franka_rl.scripts.eval_constraints import build_env
    env = build_env(_CONFIG, 'full', 'dynamic', ['obstacle.count_range=[2,3]'])
    assert env.obs_count_range == (2, 3) and len(env._extra_mocap) == 2
    env.close()


def test_bad_count_range_is_rejected():
    with pytest.raises(ValueError):
        FrankaCBFEnv(_cfg(count_range=[0, 2]))
    with pytest.raises(ValueError):
        FrankaCBFEnv(_cfg(count_range=[3, 2]))
