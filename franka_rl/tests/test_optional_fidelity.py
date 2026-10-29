"""Guards for the two OPTIONAL fidelity layers (backlog P1.3 / P1.4 / P1.5).

Both ship OFF. The single most important property in this file is that with
them off the environment is **bit-identical** to the one `sac_v4` trained on —
if that ever breaks, every number in
`franka_sim_to_real_implementation_status.md` silently stops describing the
shipped env, which is exactly the class of drift §5.3 was about.

The rest check that each mechanism, when switched on, does the thing it claims
and not something adjacent to it.
"""

import copy

import numpy as np
import pytest
import yaml

from franka_rl.envs.franka_cbf_env import FrankaCBFEnv
from franka_rl.envs.cbf_filter import NV
from franka_rl.envs.randomization import Randomizer

_CFG = __file__.rsplit('/', 2)[0] + '/config.yaml'


@pytest.fixture(scope='module')
def base():
    with open(_CFG) as fh:
        return yaml.safe_load(fh)


def _rollout(cfg, seed=5, n=40, action=None):
    env = FrankaCBFEnv(config=cfg)
    obs, _ = env.reset(seed=seed)
    obss, qs = [obs.copy()], []
    rng = np.random.default_rng(0)
    for _ in range(n):
        a = action if action is not None else rng.uniform(-1, 1, NV).astype(np.float32)
        obs, _, term, trunc, _ = env.step(a)
        obss.append(obs.copy())
        qs.append(env._q.copy())
        if term or trunc:
            break
    env.close()
    return np.array(obss), np.array(qs)


def _with(base, key, block):
    cfg = copy.deepcopy(base)
    cfg[key] = block
    return cfg


def _without(base, *keys):
    cfg = copy.deepcopy(base)
    for k in keys:
        cfg.pop(k, None)
    return cfg


# ── The property that protects every published number ────────────────────────

def test_both_layers_off_is_bit_identical(base):
    """`enabled: false` == the key being absent entirely.

    This is what makes sac_v4's results survive the addition of these layers.
    """
    ref_obs, ref_q = _rollout(_without(base, 'actuation', 'randomization'))

    off = copy.deepcopy(base)
    off['actuation'] = {'enabled': False}
    off['randomization'] = {'enabled': False}
    obs, q = _rollout(off)

    assert np.array_equal(obs, ref_obs), 'observation stream changed with both layers off'
    assert np.array_equal(q, ref_q), 'joint trajectory changed with both layers off'


def test_shipped_config_has_both_layers_off(base):
    """The repo default must stay the measured configuration."""
    assert base['actuation']['enabled'] is False
    assert base['randomization']['enabled'] is False


def test_randomization_master_switch_gates_every_block(base):
    """Master off must neutralise blocks that are individually on."""
    cfg = _with(base, 'randomization', {
        'enabled': False,
        'latency': {'enabled': True, 'mean_s': 0.05, 'jitter_s': 0.0},
        'obs_noise': {'enabled': True, 'obstacle_pos_std': 0.1},
        'joint_noise': {'enabled': True, 'q_std': 0.1},
        'dynamics': {'enabled': True},
    })
    obs, q = _rollout(cfg)
    ref_obs, ref_q = _rollout(_without(base, 'randomization'))
    assert np.array_equal(obs, ref_obs)
    assert np.array_equal(q, ref_q)


# ── Actuation law (P1.3) ─────────────────────────────────────────────────────

def test_actuation_on_keeps_control_authority(base):
    """The feedback law must not cost the action its authority."""
    cfg = _with(base, 'actuation', {'enabled': True})
    _, q_idle = _rollout(cfg, n=50, action=np.zeros(NV, np.float32))
    _, q_unit = _rollout(cfg, n=50, action=np.eye(NV, dtype=np.float32)[0])
    idle = np.linalg.norm(q_idle[-1] - q_idle[0])
    unit = np.linalg.norm(q_unit[-1] - q_unit[0])
    assert unit > 0.1, f'unit action moved only {unit:.4f} rad with feedback on'
    assert unit > 5.0 * max(idle, 1e-9)


def test_actuation_on_does_not_fade_gravity(base):
    """A zero command must HOLD, not sag.

    `ffScale` may only scale the term that accelerates the joint. If gravity
    were inside the faded feedforward, the arm would drop exactly when the gate
    engaged — a torque gate that can drop the load is not a safety feature.
    """
    cfg = _with(base, 'actuation', {'enabled': True})
    _, q = _rollout(cfg, n=50, action=np.zeros(NV, np.float32))
    drift = np.linalg.norm(q[-1] - q[0])
    assert drift < 1e-3, f'arm sagged {drift:.6f} rad under a zero command'


def test_actuation_rejects_malformed_gains(base):
    with pytest.raises(ValueError, match='d_gains'):
        FrankaCBFEnv(config=_with(base, 'actuation',
                                  {'enabled': True, 'd_gains': [1.0, 2.0]}))


def test_ff_scale_is_directional(base):
    """A braking torque always passes; only one pushing further in is faded."""
    env = FrankaCBFEnv(config=_with(base, 'actuation', {'enabled': True}))
    q = np.zeros(NV)
    q[3], q[5] = -1.5, 4.45            # joint6 near its upper reference
    qdot = np.zeros(NV)
    qdot[5] = 0.40                     # moving toward it, little margin left
    push = np.ones(NV)                 # positive tau -> further into the wall
    brake = -np.ones(NV)               # negative tau -> away from it
    s_push = env._ff_scale(push, q, qdot)
    s_brake = env._ff_scale(brake, q, qdot)
    env.close()
    assert s_push[5] < 1.0, 'feedforward pushing into the envelope was not faded'
    assert s_brake[5] == pytest.approx(1.0), 'a braking feedforward must not fade'
    assert np.all((s_push >= 0.0) & (s_push <= 1.0))


# ── Randomisation (P1.4 / P1.5) ──────────────────────────────────────────────

def test_latency_delays_the_obstacle_slot():
    """The delayed slot must equal the value from `n` ticks earlier."""
    r = Randomizer({'enabled': True,
                    'latency': {'enabled': True, 'mean_s': 0.03, 'jitter_s': 0.0}},
                   dt=0.01, rng=np.random.default_rng(0))

    class _M:
        body_mass = np.ones(3); dof_damping = np.ones(3); dof_frictionloss = np.ones(3)
    r.reset(_M(), np.random.default_rng(0))
    assert r._n_delay == 3

    seen = []
    for i in range(8):
        p, d = r.obstacle(np.array([float(i), 0.0, 0.0]), float(i))
        seen.append(d)
    # First ticks hold the oldest sample, then the stream trails by 3.
    assert seen[3] == pytest.approx(0.0)
    assert seen[7] == pytest.approx(4.0)


def test_obs_noise_never_reports_penetration():
    """`clamp_d_min_at_zero` mirrors distance_engine's np.maximum(·, 0)."""
    r = Randomizer({'enabled': True,
                    'obs_noise': {'enabled': True, 'd_min_std': 0.2,
                                  'lpf_alpha': 1.0, 'obstacle_pos_std': 0.0,
                                  'clamp_d_min_at_zero': True}},
                   dt=0.01, rng=np.random.default_rng(0))

    class _M:
        body_mass = np.ones(3); dof_damping = np.ones(3); dof_frictionloss = np.ones(3)
    r.reset(_M(), np.random.default_rng(0))
    worst = min(r.obstacle(np.zeros(3), 0.01)[1] for _ in range(300))
    assert worst >= 0.0, f'reported a negative d_min ({worst}) with the clamp on'


def test_dropout_holds_the_last_good_frame():
    """A dropped frame must HOLD, never teleport the obstacle to the origin."""
    r = Randomizer({'enabled': True,
                    'obs_noise': {'enabled': True, 'dropout_prob': 1.0,
                                  'obstacle_pos_std': 0.0, 'd_min_std': 0.0,
                                  'lpf_alpha': 1.0}},
                   dt=0.01, rng=np.random.default_rng(0))

    class _M:
        body_mass = np.ones(3); dof_damping = np.ones(3); dof_frictionloss = np.ones(3)
    r.reset(_M(), np.random.default_rng(0))
    first = r.obstacle(np.array([0.5, 0.0, 0.5]), 0.3)     # no history -> passes
    held = r.obstacle(np.array([9.9, 9.9, 9.9]), 99.0)     # dropped -> holds
    assert np.allclose(held[0], first[0])
    assert held[1] == pytest.approx(first[1])


def test_noise_never_touches_the_cbf_rows(base):
    """Observation noise must not move the arm: the shield sees true geometry."""
    cfg = _with(base, 'randomization', {
        'enabled': True,
        'obs_noise': {'enabled': True, 'obstacle_pos_std': 0.05,
                      'd_min_std': 0.02, 'lpf_alpha': 0.5},
        'joint_noise': {'enabled': True, 'q_std': 0.01, 'qdot_std': 0.05},
    })
    obs, q = _rollout(cfg)
    ref_obs, ref_q = _rollout(_without(base, 'randomization'))
    assert np.array_equal(q, ref_q), (
        'observation noise changed the trajectory — it leaked into the CBF rows')
    assert not np.array_equal(obs, ref_obs), 'noise had no effect on the observation'


def test_dynamics_randomisation_restores_nominal_when_disabled(base):
    """MjModel is mutated in place; a later episode must not inherit it."""
    env = FrankaCBFEnv(config=_with(base, 'randomization',
                                    {'enabled': True, 'dynamics': {'enabled': True}}))
    env.reset(seed=1)
    perturbed = env.model.body_mass.copy()
    # Same env, randomisation switched off at runtime -> nominal must return.
    env.randomizer.enabled = False
    env.reset(seed=1)
    restored = env.model.body_mass.copy()
    nominal = env.randomizer._nominal[0]
    env.close()
    assert not np.allclose(perturbed, nominal), 'dynamics randomisation had no effect'
    assert np.allclose(restored, nominal), 'nominal masses were not restored'


# ── Control-point block perceived like d_min (randomization.cp_geometry, e2) ──

def _cp_randomizer(**obs_noise):
    r = Randomizer({'enabled': True, 'cp_geometry': True,
                    'latency': {'enabled': True, 'mean_s': 0.03, 'jitter_s': 0.0},
                    'obs_noise': {'enabled': bool(obs_noise), **obs_noise}},
                   dt=0.01, rng=np.random.default_rng(0))

    class _M:
        body_mass = np.ones(3); dof_damping = np.ones(3); dof_frictionloss = np.ones(3)
    r.reset(_M(), np.random.default_rng(0))
    return r


def test_cp_block_is_delayed_with_the_obstacle_slot():
    """Same message on the robot → same delay as d_min, tick for tick."""
    r = _cp_randomizer()
    n_hat = np.array([0.0, 0.0, 1.0])
    for i in range(8):
        _, d = r.obstacle(np.zeros(3), float(i))
        g = r.cp_geometry(np.tile(np.r_[float(i), n_hat], (6, 1)))
        assert np.allclose(g[:, 0], d)
        assert np.allclose(g[:, 1:], n_hat)


def test_cp_block_noise_keeps_unit_normals_and_never_penetrates():
    r = _cp_randomizer(d_min_std=0.2, lpf_alpha=0.5, obstacle_pos_std=0.0, cp_normal_std=0.3)
    raw = np.tile([0.01, 1.0, 0.0, 0.0], (6, 1))
    raw[5] = [-0.05, 0.0, 0.0, 0.0]                  # degenerate row: no direction
    for _ in range(100):
        r.obstacle(np.zeros(3), 0.01)
        g = r.cp_geometry(raw)
        assert (g[:, 0] >= 0.0).all()
        np.testing.assert_allclose(np.linalg.norm(g[:5, 1:], axis=1), 1.0, atol=1e-9)
        assert not g[5, 1:].any(), 'a zero normal must stay the "no direction" token'


def test_cp_perception_off_is_bit_identical_and_on_never_moves_the_arm(base):
    """cp_geometry false/absent replays every 51-D run; true changes only what the policy reads."""
    b = copy.deepcopy(base)
    b['obs'] = {'obstacle_velocity': True, 'control_point_geometry': True}
    rnd = {'enabled': True, 'latency': {'enabled': True, 'mean_s': 0.05, 'jitter_s': 0.0},
           'obs_noise': {'enabled': True}, 'joint_noise': {'enabled': True}}
    ref_obs, ref_q = _rollout(_with(b, 'randomization', rnd))
    off_obs, off_q = _rollout(_with(b, 'randomization', {**rnd, 'cp_geometry': False}))
    on_obs, on_q = _rollout(_with(b, 'randomization', {**rnd, 'cp_geometry': True}))
    np.testing.assert_array_equal(off_obs, ref_obs)
    np.testing.assert_array_equal(on_q, ref_q)
    assert not np.array_equal(on_obs[:, 27:], ref_obs[:, 27:]), 'cp perception had no effect'
