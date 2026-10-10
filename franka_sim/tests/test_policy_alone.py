"""Guards for the policy-alone mode (env.shield: false) and the constraint monitor.

The defaults must leave the environment bit-identical (shield on, monitor off),
the monitor must only read, the shield switch must really remove the QP, and
each constraint family must fire on a state that violates it.
"""

import copy

import mujoco
import numpy as np
import pytest
import yaml

from franka_sim.baselines import CartesianPDBaseline
from franka_sim.envs.cbf_filter import NV
from franka_sim.envs.constraints import capsule_pairs, segment_distance, segment_distances
from franka_sim.envs.franka_cbf_env import FrankaCBFEnv

_CFG = __file__.rsplit('/', 2)[0] + '/config.yaml'


@pytest.fixture(scope='module')
def base():
    with open(_CFG) as fh:
        return yaml.safe_load(fh)


def _cfg(base, shield=True, monitor=False):
    c = copy.deepcopy(base)
    c['env']['shield'] = shield
    c['constraints'] = {'enabled': monitor}
    return c


def _rollout(cfg, seed=5, n=40):
    env = FrankaCBFEnv(config=cfg)
    env.reset(seed=seed)
    rng = np.random.default_rng(0)
    qs, infos = [], []
    for _ in range(n):
        _, _, te, tr, info = env.step(rng.uniform(-1, 1, NV).astype(np.float32))
        qs.append(env._q.copy()); infos.append(info)
        if te or tr:
            break
    return np.array(qs), infos


def test_monitor_only_reads(base):
    """Same seed, same actions: the monitor on or off must not move the arm."""
    q_off, _ = _rollout(_cfg(base, monitor=False))
    q_on, infos = _rollout(_cfg(base, monitor=True))
    assert np.array_equal(q_off, q_on)
    assert 'constraint_excess' in infos[0]


def test_shield_false_never_calls_the_filter(base):
    env = FrankaCBFEnv(config=_cfg(base, shield=False))
    env.reset(seed=1)

    def boom(*a, **k):
        raise AssertionError('cbf.filter called with env.shield false')
    env.cbf.filter = boom
    env.step(np.zeros(NV, np.float32))


def test_shield_false_changes_the_command(base):
    """A full-scale action the QP would clip reaches the plant unclipped."""
    a = np.zeros(NV, np.float32); a[4] = 1.0     # joint5: action scale 17 > box 10
    out = {}
    for shield in (True, False):
        env = FrankaCBFEnv(config=_cfg(base, shield=shield, monitor=True))
        env.reset(seed=1)
        _, _, _, _, info = env.step(a)
        out[shield] = info['constraint_excess']['acc']
    assert out[True] == 0.0 and out[False] > 0.0


def test_velocity_limit_is_detected(base):
    c = _cfg(base, shield=False, monitor=True)
    c['reward']['terminate_on_collision'] = False   # the swing crosses the obstacle
    env = FrankaCBFEnv(config=c)
    env.reset(seed=2)
    a = np.zeros(NV, np.float32); a[0] = 1.0
    seen = set()
    for _ in range(200):
        _, _, te, tr, info = env.step(a)
        seen |= {k for k, v in info['constraint_excess'].items() if v > 0}
        if te or tr:
            break
    assert 'vel' in seen or 'pos' in seen


def test_self_collision_capsules(base):
    env = FrankaCBFEnv(config=_cfg(base, monitor=True))
    env.reset(seed=0)
    mon = env.constraints
    assert len(mon.pairs) == 24                 # the robot's count (fr3_control.yaml)
    assert mon.self_gap()[0] > 0.05             # home pose is clear
    # A folded pose found by sampling: link1 and link6 overlap.
    env.data.qpos[env._qadr] = [-1.866, 1.684, 0.093, -2.712, 0.694, 3.632, 0.682]
    mujoco.mj_kinematics(env.model, env.data)
    gap, pair = mon.self_gap()
    assert gap < -0.05 and pair == 'fr3_link1|fr3_link6'


def test_capsule_pair_rules():
    bodies = ['fr3_link0', 'fr3_link1', 'fr3_link5', 'fr3_link6', 'fr3_link7', 'fr3_hand']
    got = {(bodies[i], bodies[j]) for i, j in capsule_pairs(bodies)}
    assert ('fr3_link1', 'fr3_link6') in got
    assert ('fr3_link6', 'fr3_hand') not in got      # hand is adjacent to link6
    assert ('fr3_link5', 'fr3_link7') not in got     # SRDF "Never"


def test_segment_distance_matches_sampling():
    rng = np.random.default_rng(3)
    for _ in range(50):
        p1, q1, p2, q2 = rng.normal(size=(4, 3))
        s = np.linspace(0, 1, 201)
        A = p1 + s[:, None] * (q1 - p1); B = p2 + s[:, None] * (q2 - p2)
        brute = np.linalg.norm(A[:, None] - B[None], axis=2).min()
        assert segment_distance(p1, q1, p2, q2) <= brute + 1e-9
        assert segment_distance(p1, q1, p2, q2) >= brute - 0.02


def test_baseline_reaches_with_the_full_shield(base):
    c = _cfg(base, monitor=True)
    c['obstacle']['mode'] = 'static'; c['obstacle']['static_fraction'] = 0
    c['reward']['terminate_on_success'] = False
    env = FrankaCBFEnv(config=c)
    env.reset(seed=3000)
    ctrl = CartesianPDBaseline(env)
    for _ in range(500):
        _, _, te, tr, info = env.step(ctrl())
        if te or tr:
            break
    assert info['dist'] < env.target_tol


def test_vectorised_segment_distance_matches_scalar():
    rng = np.random.default_rng(4)
    P = rng.normal(size=(500, 4, 3))
    P[:20, 1] = P[:20, 0]            # some point-like segments
    P[20:40, 3] = P[20:40, 2]
    P[40:60, 3] = P[40:60, 2] + (P[40:60, 1] - P[40:60, 0])   # parallel
    v = segment_distances(P[:, 0], P[:, 1], P[:, 2], P[:, 3])
    for k in range(500):
        assert abs(v[k] - segment_distance(*P[k])) < 1e-9
