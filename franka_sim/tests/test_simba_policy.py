"""SimBa actor/critic for SB3 SAC (franka_sim/policies/simba.py).

What has to hold for a SimBa run to be deployable through the existing chain:
the running statistics are right and shared with the target critic, a saved
model reloads as the same function, and export_onnx.py's validation passes
(the RSNorm is inside the actor graph, so the robot needs nothing else).
"""

import os

import numpy as np
import pytest
import torch as th
import yaml
from stable_baselines3 import SAC

from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from franka_sim.policies.simba import (
    RSNorm, RSNormUpdateCallback, SimbaActor, SimbaCritic, SimbaSACPolicy,
)

_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config.yaml')


def _cfg():
    with open(_CONFIG) as fh:
        c = yaml.safe_load(fh)
    c['obs'] = {'obstacle_velocity': True, 'control_point_geometry': True}
    c['obstacle']['count_range'] = [1, 2]
    return c


def test_rsnorm_matches_numpy_statistics():
    rng = np.random.default_rng(0)
    norm = RSNorm(5)
    xs = [rng.normal(3.0, 2.0, (n, 5)) * np.arange(1, 6) for n in (7, 1, 64, 13)]
    for x in xs:
        norm.update(th.as_tensor(x, dtype=th.float32))
    allx = np.concatenate(xs).astype(np.float32).astype(np.float64)
    assert np.allclose(norm.running_mean.numpy(), allx.mean(0), rtol=1e-4, atol=1e-4)
    assert np.allclose(norm.running_var.numpy(), allx.var(0), rtol=1e-3, atol=1e-3)
    y = norm(th.as_tensor(allx, dtype=th.float32)).numpy()
    assert np.allclose(y.mean(0), 0.0, atol=1e-3) and np.allclose(y.std(0), 1.0, atol=1e-2)
    # Critic input: obs columns normalised, action columns untouched.
    z = norm(th.cat([th.as_tensor(allx[:3], dtype=th.float32), th.ones(3, 2)], 1))
    assert z.shape == (3, 7) and th.allclose(z[:, 5:], th.ones(3, 2))


@pytest.fixture(scope='module')
def model(tmp_path_factory):
    env = FrankaCBFEnv(_cfg())
    m = SAC(SimbaSACPolicy, env, learning_starts=40, batch_size=32, buffer_size=2000,
            train_freq=1, gradient_steps=1, device='cpu', seed=3,
            policy_kwargs=dict(actor_hidden=32, actor_blocks=1, critic_hidden=48,
                               critic_blocks=2, weight_decay=1e-2))
    m.learn(160, callback=RSNormUpdateCallback())
    yield m
    env.close()


def test_architecture_and_optimizer(model):
    assert isinstance(model.actor, SimbaActor) and isinstance(model.critic, SimbaCritic)
    assert isinstance(model.actor.optimizer, th.optim.AdamW)
    assert model.actor.optimizer.param_groups[0]['weight_decay'] == 1e-2
    assert model.actor.mu.in_features == 32 and model.critic.q_networks[0][-1].in_features == 48


def test_running_stats_follow_the_rollout_and_reach_the_target_critic(model):
    n = float(model.actor.obs_norm.running_count)
    assert 150 <= n <= 170, n
    crit = [m for m in model.critic.modules() if isinstance(m, RSNorm)]
    targ = [m for m in model.critic_target.modules() if isinstance(m, RSNorm)]
    assert len(crit) == 2 == len(targ)
    for c, t in zip(crit, targ):
        assert th.equal(c.running_mean, model.actor.obs_norm.running_mean)
        assert th.equal(c.running_mean, t.running_mean) and th.equal(c.running_var, t.running_var)
    # Joint angles sit around the home pose: the mean is not the zero it started from.
    assert float(model.actor.obs_norm.running_mean[:7].abs().max()) > 0.1


def test_save_load_roundtrip_and_onnx_export(model, tmp_path):
    p = tmp_path / 'simba.zip'
    model.save(p)
    m2 = SAC.load(p, device='cpu')
    assert isinstance(m2.policy, SimbaSACPolicy) and isinstance(m2.actor, SimbaActor)
    assert m2.policy.simba['critic_hidden'] == 48
    obs = model.observation_space.sample().astype(np.float32)
    a1, _ = model.predict(obs, deterministic=True)
    a2, _ = m2.predict(obs, deterministic=True)
    assert np.allclose(a1, a2, atol=1e-6)
    assert th.equal(m2.actor.obs_norm.running_mean, model.actor.obs_norm.running_mean)

    from franka_sim.export_onnx import export
    out = export(str(p), str(tmp_path / 'simba.onnx'), verbose=False)   # asserts |onnx − sb3| < 1e-4
    assert os.path.exists(out)
    import onnxruntime as ort
    sess = ort.InferenceSession(out, providers=['CPUExecutionProvider'])
    a3 = sess.run(None, {'observation': obs[None]})[0][0]
    assert np.allclose(a3, a1, atol=1e-4)
