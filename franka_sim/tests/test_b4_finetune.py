"""Guards for the b4 fine-tune machinery (2026-10-06): shield mixing, the shield-validity buffer mask, the
24→27-D observation widening of a trained SAC and its replay buffer, and the differential reward relabel.

    cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src python3 -m pytest franka_sim/tests/test_b4_finetune.py -q
"""
import copy
import os

import numpy as np
import pytest
import torch as th
import yaml
from stable_baselines3 import SAC
from stable_baselines3.common.buffers import ReplayBuffer
from stable_baselines3.common.logger import configure

from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from franka_sim.scripts.shield_buffer_mask import buffer_mask, chronological
from franka_sim.train import mask_buffer, relabel_buffer, widen_buffer, widen_model

_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config.yaml')


def _cfg(**env):
    with open(_CONFIG) as fh:
        c = yaml.safe_load(fh)
    c['obs'] = dict(c.get('obs') or {}, obstacle_velocity=False, control_point_geometry=False)
    c['env'].update(env)
    c['obstacle']['mode'] = 'sinusoidal'
    c['obstacle']['static_fraction'] = 0.0
    c['rl']['device'] = 'cpu'
    return c


def _wide(c):
    c = copy.deepcopy(c)
    c['obs']['obstacle_velocity'] = True
    return c


def _rollout(cfg, n_steps, seed=0, policy=None):
    """[(obs, next_obs, action, reward, done, timeout)] with SB3's terminal-obs convention."""
    env = FrankaCBFEnv(config=cfg)
    rng = np.random.default_rng(seed)
    obs, _ = env.reset(seed=seed)
    out = []
    ep = 0
    while len(out) < n_steps:
        a = rng.uniform(-1, 1, 7).astype(np.float32) if policy is None else policy(obs)
        nxt, r, term, trunc, _ = env.step(a)
        out.append((obs, nxt, a, r, term or trunc, trunc and not term))
        obs = nxt
        if term or trunc:
            ep += 1
            obs, _ = env.reset(seed=seed + ep)
    return out


def _fill(buf, rows, obs_dim=None):
    for o, x, a, r, d, t in rows:
        o, x = (o, x) if obs_dim is None else (o[:obs_dim], x[:obs_dim])
        buf.add(o[None], x[None], a[None], np.array([r]), np.array([d]), [{'TimeLimit.truncated': t}])


def _buf(cfg, size):
    env = FrankaCBFEnv(config=cfg)
    return ReplayBuffer(size, env.observation_space, env.action_space, device='cpu', n_envs=1)


# ── C2: shield mixing ────────────────────────────────────────────────────────

def test_shield_mixing_draws_per_episode_and_overrides_flag():
    env = FrankaCBFEnv(config=_cfg(cbf_obstacle_enabled=False, cbf_obstacle_on_prob=0.5))
    seen = set()
    for s in range(20):
        env.reset(seed=s)
        seen.add(env.cbf_obstacle_enabled)
    assert seen == {True, False}
    env = FrankaCBFEnv(config=_cfg(cbf_obstacle_enabled=False, cbf_obstacle_on_prob=1.0))
    env.reset(seed=1)
    assert env.cbf_obstacle_enabled


def test_shield_mixing_unset_is_bit_identical():
    a = FrankaCBFEnv(config=_cfg(cbf_obstacle_enabled=False))
    b = FrankaCBFEnv(config=_cfg(cbf_obstacle_enabled=False, cbf_obstacle_on_prob=None))
    for s in (3, 4):
        oa, _ = a.reset(seed=s)
        ob, _ = b.reset(seed=s)
        np.testing.assert_array_equal(oa, ob)
        assert not b.cbf_obstacle_enabled


# ── C3: shield-validity mask ─────────────────────────────────────────────────

def test_buffer_mask_effect_matches_live_shadow_filter():
    """Shield-ON rollout: the offline obstacle-row effect equals the live one, measured by running a shadow
    filter with NO obstacle rows on the same state and slew anchor inside env.step."""
    cfg = _cfg(cbf_obstacle_enabled=True)
    cfg['obstacle']['speed'] = 0.6           # make the rows bind often
    env = FrankaCBFEnv(config=cfg)
    main_filter, shadow, live = env.cbf.filter, copy.copy(env.cbf), []

    def wrapped(q, qdot, qddot_nom, obstacles, **kw):
        prev = env.cbf._qddot_prev.copy()
        out = main_filter(q, qdot, qddot_nom, obstacles, **kw)
        shadow._qddot_prev, shadow._probs = prev, {}
        qs, _ = type(env.cbf).filter(shadow, q, qdot, qddot_nom, [], **kw)
        live.append(float(np.linalg.norm(out[0] - qs)))
        return out
    env.cbf.filter = wrapped
    rng = np.random.default_rng(3)
    buf = _buf(cfg, 2000)
    obs, _ = env.reset(seed=3)
    for t in range(1000):
        a = rng.uniform(-1, 1, 7).astype(np.float32)
        nxt, r, term, trunc, _ = env.step(a)
        buf.add(obs[None], nxt[None], a[None], np.array([r]), np.array([term or trunc]), [{'TimeLimit.truncated': trunc}])
        obs = nxt if not (term or trunc) else env.reset(seed=4 + t)[0]
    valid, effect, _ = buffer_mask(cfg, buf, tol=0.5, workers=1)
    live = np.asarray(live)
    assert np.mean(live > 0.5) > 0.02                      # the test exercises the rows
    np.testing.assert_allclose(effect[1:], live[1:], atol=0.05)
    np.testing.assert_array_equal(valid[1:], live[1:] < 0.5)
    assert not valid[0]


def test_mask_buffer_compacts_in_chronological_order():
    cfg = _cfg(cbf_obstacle_enabled=False)
    rows = _rollout(cfg, 130, seed=2)
    buf = _buf(cfg, 100)                      # wraps: full, pos = 30
    _fill(buf, rows)
    assert buf.full and buf.pos == 30
    order = chronological(100, buf.pos, buf.full)
    valid = np.ones(100, bool)
    valid[order[::3]] = False
    expect = buf.observations[order[valid[order]]].copy()
    mask_buffer(buf, valid)
    assert not buf.full and buf.pos == len(expect)
    np.testing.assert_array_equal(buf.observations[:buf.pos], expect)


# ── C4: widening ─────────────────────────────────────────────────────────────

def test_widen_buffer_reproduces_env_velocity():
    cfg_w = _wide(_cfg(cbf_obstacle_enabled=False))
    rows = _rollout(cfg_w, 1150, seed=5)     # > 2 episodes, and wraps the 1000 buffer
    truth = _buf(cfg_w, 1000)
    _fill(truth, rows)
    narrow = _buf(_cfg(cbf_obstacle_enabled=False), 1000)
    _fill(narrow, rows, obs_dim=24)
    new = _buf(cfg_w, 1000)
    widen_buffer(narrow, new, cfg_w)
    order = chronological(1000, truth.pos, truth.full)[1:]    # oldest: predecessor gone
    np.testing.assert_allclose(new.observations[order], truth.observations[order], atol=2e-4)
    np.testing.assert_allclose(new.next_observations[order], truth.next_observations[order], atol=2e-4)
    assert np.abs(truth.observations[order, 0, 24:27]).max() > 0.05     # the obstacle really moves


def test_widen_model_is_the_same_function():
    c24 = _cfg(cbf_obstacle_enabled=False)
    c27 = _wide(c24)
    e24, e27 = FrankaCBFEnv(config=c24), FrankaCBFEnv(config=c27)
    old = SAC('MlpPolicy', e24, device='cpu', seed=0, learning_starts=0, batch_size=8)
    old.set_logger(configure(None, []))
    rng = np.random.default_rng(0)
    for _ in range(16):                      # move every parameter and Adam moment off its init
        old.replay_buffer.add(rng.normal(size=(1, 24)), rng.normal(size=(1, 24)), rng.uniform(-1, 1, (1, 7)),
                              rng.normal(size=1), np.array([False]), [{}])
    old.train(gradient_steps=5, batch_size=8)
    new = SAC('MlpPolicy', e27, device='cpu', seed=1)
    widen_model(old, new)
    o24 = np.random.randn(16, 24).astype(np.float32)
    o27 = np.concatenate([o24, np.random.randn(16, 3).astype(np.float32)], 1)
    a_old, _ = old.predict(o24, deterministic=True)
    a_new, _ = new.predict(o27, deterministic=True)
    np.testing.assert_allclose(a_new, a_old, atol=1e-6)
    act = th.as_tensor(np.random.uniform(-1, 1, (16, 7)), dtype=th.float32)
    for net in ('critic', 'critic_target'):
        q_old = getattr(old, net)(th.as_tensor(o24), act)
        q_new = getattr(new, net)(th.as_tensor(o27), act)
        for x, y in zip(q_old, q_new):
            np.testing.assert_allclose(y.detach().numpy(), x.detach().numpy(), atol=1e-5)
    assert float(new.log_ent_coef) == pytest.approx(float(old.log_ent_coef))
    st = new.critic.optimizer.state_dict()['state']
    assert st[0]['exp_avg'].shape == (256, 27 + 7) and st[0]['exp_avg'][:, 24:27].abs().max() == 0
    assert st[0]['exp_avg'][:, :24].abs().max() > 0


# ── differential relabel ─────────────────────────────────────────────────────

def test_relabel_from_same_config_is_a_no_op():
    cfg = _cfg(cbf_obstacle_enabled=False)
    cfg['reward'].update(w_prec=2.0, prec_scale=0.05, w_obs_margin=1.0, obs_soft_margin=0.2, obs_shaping='penalty')
    buf = _buf(cfg, 300)
    _fill(buf, _rollout(cfg, 200, seed=4))
    before = buf.rewards.copy()
    relabel_buffer(buf, cfg, copy.deepcopy(cfg))
    np.testing.assert_allclose(buf.rewards, before, atol=1e-5)


def test_relabel_matches_env_reward_change():
    """A buffer collected under config A, relabelled A→B, carries B's reward (B differs only in obs-derivable terms)."""
    a = _cfg(cbf_obstacle_enabled=False)
    a['reward'].update(w_prec=2.0, prec_scale=0.05, w_obs_margin=2.0, obs_soft_margin=0.10, obs_shaping='potential',
                       collision_step_cost=0.0)
    b = copy.deepcopy(a)
    b['reward'].update(w_obs_margin=1.0, obs_soft_margin=0.20, obs_shaping='penalty', w_prec=0.0)
    ra, rb = _rollout(a, 300, seed=11), _rollout(b, 300, seed=11)
    for x, y in zip(ra, rb):                  # same actions → same trajectory
        np.testing.assert_allclose(x[1], y[1], atol=1e-6)
    buf = _buf(a, 400)
    _fill(buf, ra)
    relabel_buffer(buf, b, a)
    np.testing.assert_allclose(buf.rewards[:300, 0], [y[3] for y in rb], atol=1e-4)
