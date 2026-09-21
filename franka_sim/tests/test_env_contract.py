"""Guards for the training environment (franka_sim/envs/franka_cbf_env.py).

These are the checks that would have caught the three defects this project has
actually hit, in the order they were found:

1. the ACTUATION defect — the action never reached the plant, so a trained, a
   random and a motionless policy produced one trajectory;
2. the RESET defect — 9 of 50 episodes began already penetrating, so the
   collision rate measured the reset distribution rather than the controller;
3. the obstacle TELEPORT — reset parked the sphere at `_obs_base` while the
   first step evaluated `base + amp·sin(phase)`, displacing it by up to the
   full amplitude (0.200 m) in one 10 ms tick.

ROS-free, plain pytest (see §8 gotcha 7 — franka_sim has no package.xml):

    cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl \
        python3 -m pytest franka_sim/tests -q
"""

import os

import numpy as np
import pytest
import yaml

from franka_sim.envs.franka_cbf_env import FrankaCBFEnv
from franka_sim.envs.cbf_filter import NV

_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       '..', 'config.yaml')


def _shipped_config() -> dict:
    with open(_CONFIG) as fh:
        return yaml.safe_load(fh)

SEEDS = range(12345, 12345 + 12)     # kept small: each reset builds a scene


@pytest.fixture(scope='module')
def env():
    e = FrankaCBFEnv()
    yield e
    e.close()


# ── Observation / action contract ────────────────────────────────────────────

def test_observation_layout(env):
    """Slots land where obs_layout.ObsSpec says they do.

    utils/rl_policy.build_observation rebuilds this on the robot; a change here
    without a change there silently offsets every slot the network reads.
    """
    obs, _ = env.reset(seed=0)
    spec = env.obs_spec
    assert obs.shape == (spec.dim,) and obs.dtype == np.float32
    assert obs.shape == env.observation_space.shape
    assert np.allclose(obs[0:7], env._q)
    assert np.allclose(obs[7:14], env._qdot)
    assert np.allclose(obs[14:17], env._ee_pos())
    assert np.allclose(obs[17:20], env._target)
    assert np.allclose(obs[20:23], env.data.mocap_pos[env._obs_mocap])


def test_legacy_config_still_builds_the_24_dim_vector():
    """A config with no `obs:` block must reproduce the sac_v4 observation.

    The optional blocks are prefix extensions precisely so that a model frozen
    before they existed keeps deploying; if this breaks, every archived policy
    silently reads shifted slots.
    """
    import copy
    cfg = copy.deepcopy(_shipped_config())
    cfg.pop('obs', None)
    legacy = FrankaCBFEnv(config=cfg)
    try:
        assert legacy.obs_spec.dim == 24
        assert not legacy.obs_spec.obstacle_velocity
        assert legacy.obs_spec.n_cp == 0
        obs, _ = legacy.reset(seed=0)
        assert obs.shape == (24,)
    finally:
        legacy.close()


def test_control_point_geometry_is_what_the_cbf_rows_are_built_from(env):
    """obs[cp block] must BE the (dᵢ, n̂ᵢ) of the barrier rows, not a copy.

    The whole point of the block is that the policy and the filter correcting
    it reason over one geometric picture. Two independently computed pictures
    would be the old problem wearing a new shape.
    """
    env.reset(seed=3)
    obs, *_ = env.step(np.zeros(NV))
    spec = env.obs_spec
    start = next(i for k, i, _ in spec.slots if k.startswith('cp:'))
    geom = obs[start:start + 4 * spec.n_cp].reshape(spec.n_cp, 4)

    rows, *_, cp_geom = env._build_obstacles(env._qdot)
    assert np.allclose(geom, cp_geom, atol=1e-6)

    by_name = {o.name: o for o in rows}
    for i, body in enumerate(env._cp_name):
        if body in by_name:               # degenerate rows are dropped, geom is not
            assert geom[i, 0] == pytest.approx(by_name[body].d, abs=1e-6)
    # d_min is the minimum of exactly these distances.
    assert float(obs[23]) == pytest.approx(float(geom[:, 0].min()), abs=1e-6)


def test_control_point_normals_point_from_obstacle_to_robot(env):
    """n̂ᵢ is a unit vector along (p_cp − p_obs) — sign included.

    A flipped normal is the worst kind of silent bug here: it is still unit
    length, still finite, and it teaches the policy to dodge the wrong way.
    """
    env.reset(seed=5)
    obs, *_ = env.step(np.zeros(NV))
    spec = env.obs_spec
    start = next(i for k, i, _ in spec.slots if k.startswith('cp:'))
    geom = obs[start:start + 4 * spec.n_cp].reshape(spec.n_cp, 4)
    p_obs = env.data.mocap_pos[env._obs_mocap]
    for i, bid in enumerate(env._cp_body):
        diff = env.data.xpos[bid] - p_obs
        n = geom[i, 1:]
        assert np.linalg.norm(n) == pytest.approx(1.0, abs=1e-5)
        assert np.allclose(n, diff / np.linalg.norm(diff), atol=1e-5)


def test_obstacle_velocity_is_zero_on_reset_and_tracks_the_sweep(env):
    """v_obs must be a finite difference of the observed centre, not a guess.

    Zero on the first tick of an episode: the previous episode left the sphere
    somewhere unrelated, and differencing across a reset would report a
    teleport the policy would learn to duck.
    """
    spec = env.obs_spec
    vi = next(i for k, i, _ in spec.slots if k == 'v_obs')
    obs, _ = env.reset(seed=7)
    assert np.allclose(obs[vi:vi + 3], 0.0)

    peak = 2.0 * np.pi * env.obs_speed * env.obs_amp
    prev = env.data.mocap_pos[env._obs_mocap].copy()
    for _ in range(40):
        obs, *_ = env.step(np.zeros(NV))
        now = env.data.mocap_pos[env._obs_mocap].copy()
        assert np.allclose(obs[vi:vi + 3], (now - prev) / env.dt, atol=1e-4)
        # Never faster than the configured sweep — the teleport guard, on the
        # slot the policy actually reads.
        assert np.linalg.norm(obs[vi:vi + 3]) <= peak * 1.05 + 1e-6
        prev = now


def test_action_is_a_fraction_of_qddot_max(env):
    """q̈_nom = a·q̈_max, with q̈_max the UNCAPPED limit (see cbf_filter)."""
    assert env.action_space.shape == (NV,)
    assert np.allclose(env.action_space.high, 1.0)
    assert np.allclose(env.action_space.low, -1.0)
    # The cap belongs to the box, never to the action scale.
    assert np.all(env.qddot_max >= env.cbf.qddot_box - 1e-12)


# ── Actuation authority (defect 1) ───────────────────────────────────────────

def test_action_actually_moves_the_arm(env):
    """A unit action must dominate the do-nothing baseline.

    The condensed form of scripts/validate_actuation: if this ever ties again,
    suspect the plant before the policy.
    """
    def run(a):
        env.reset(seed=3)
        q0 = env._q.copy()
        for _ in range(30):
            env.step(a)
        return np.linalg.norm(env._q - q0)

    idle = run(np.zeros(NV, np.float32))
    driven = run(np.eye(NV, dtype=np.float32)[0])
    assert driven > 5.0 * max(idle, 1e-6), (
        f'no control authority: {driven:.4f} rad vs {idle:.4f} rad of drift')


def test_opposite_actions_move_opposite_ways(env):
    def qdot1(a):
        env.reset(seed=3)
        for _ in range(30):
            env.step(a)
        return env._qdot[0]

    pos = qdot1(np.eye(NV, dtype=np.float32)[0])
    neg = qdot1(-np.eye(NV, dtype=np.float32)[0])
    assert pos > 0.1 > -0.1 > neg


# ── Reset feasibility (defect 2) ─────────────────────────────────────────────

def test_every_episode_starts_inside_the_safe_set(env):
    """h = d − d_safe ≥ 0 at t = 0, for every seed.

    The HOCBF's forward-invariance guarantee is a statement about trajectories
    that START in the safe set. Before the rejection sampler, 9 of 50 episodes
    began with d_min < 0, where the barrier certifies nothing at all.
    """
    for seed in SEEDS:
        obs, _ = env.reset(seed=seed)
        d_min = float(obs[23])
        assert d_min >= env.reset_min_clearance - 1e-9, (
            f'seed {seed} starts at d_min={d_min:.4f} m, inside the '
            f'{env.reset_min_clearance:.3f} m clearance')
    assert env.reset_fallbacks == 0, (
        f'{env.reset_fallbacks} resets exhausted rejection sampling — the '
        'target/obstacle boxes are over-constrained')


def test_zero_action_never_collides(env):
    """A motionless arm must not collide; if it does, the benchmark is broken.

    This is the baseline every safety number is quoted against, so it has to
    measure the obstacle's behaviour and not the reset's.
    """
    for seed in SEEDS:
        env.reset(seed=seed)
        for _ in range(60):
            _, _, term, trunc, info = env.step(np.zeros(NV, np.float32))
            assert not info['collision'], (
                f'seed {seed}: motionless arm collided at d_min='
                f'{info["d_min"]:.4f} m')
            if term or trunc:
                break


def test_target_is_reachable_around_the_obstacle(env):
    """No episode may have a target blocked at EVERY phase of the sweep.

    Transiently blocked targets are kept on purpose — waiting one out is the
    behaviour we want. Always-blocked ones cannot be reached without driving
    h < 0, so they are a hard ceiling on the success rate.
    """
    for seed in SEEDS:
        env.reset(seed=seed)
        assert not env._target_always_blocked(
            env._target, env._obs_base, env._obs_dir), \
            f'seed {seed}: target is inside r_obs + d_safe at every phase'


# ── Obstacle motion (defect 3) ───────────────────────────────────────────────

def test_obstacle_does_not_teleport_on_the_first_tick(env):
    """One tick of travel, not one amplitude.

    Measured before the fix: mean 0.113 m, max 0.200 m per 10 ms tick — 11 to
    20 m/s, against a configured peak of 2π·speed·amplitude = 0.251 m/s. No
    barrier can bound a 20 m/s obstacle that lands inside d_safe, and it is why
    lowering `obstacle.speed` never fixed the collision rate: the jump was set
    by `amplitude`.
    """
    peak = 2.0 * np.pi * env.obs_speed * env.obs_amp
    budget = 1.5 * peak * env.dt          # 50 % headroom over the true peak
    for seed in SEEDS:
        env.reset(seed=seed)
        p0 = env.data.mocap_pos[env._obs_mocap].copy()
        env.step(np.zeros(NV, np.float32))
        p1 = env.data.mocap_pos[env._obs_mocap].copy()
        jump = float(np.linalg.norm(p1 - p0))
        assert jump <= budget, (
            f'seed {seed}: obstacle moved {jump:.4f} m in one {env.dt * 1e3:.0f} '
            f'ms tick ({jump / env.dt:.2f} m/s) against a {peak:.3f} m/s peak')


def test_obstacle_speed_stays_within_its_configured_peak(env):
    """Over a whole episode, not just the first tick."""
    peak = 2.0 * np.pi * env.obs_speed * env.obs_amp
    env.reset(seed=7)
    prev = env.data.mocap_pos[env._obs_mocap].copy()
    worst = 0.0
    for _ in range(200):
        env.step(np.zeros(NV, np.float32))
        now = env.data.mocap_pos[env._obs_mocap].copy()
        worst = max(worst, float(np.linalg.norm(now - prev)) / env.dt)
        prev = now
    assert worst <= 1.5 * peak, f'obstacle reached {worst:.3f} m/s vs peak {peak:.3f}'


def test_reset_and_step_agree_on_the_obstacle_position(env):
    """_obstacle_at is the single phase → position map both paths must use."""
    env.reset(seed=11)
    assert np.allclose(env.data.mocap_pos[env._obs_mocap],
                       env._obstacle_at(env._obs_phase))


# ── Control points ───────────────────────────────────────────────────────────

def test_control_points_cover_the_links_the_robot_reports(env):
    """fr3_link3 mirrors the robot's segment 3 (fr3_link3 → fr3_link4).

    Without it the sim was blind to a forearm approach the real perception
    pipeline reports — i.e. OPTIMISTIC relative to hardware, the one direction
    a sim-to-real gap must never point.
    """
    assert 'fr3_link3' in env._cp_name
    assert 'fr3_hand' in env._cp_name, 'the gripper must be a control point'
    assert len(env._cp_name) == len(set(env._cp_name))
    assert np.all(env._cp_radius > 0.0)


# ── Obstacle on the path (difficulty) ────────────────────────────────────────

def test_blocking_fraction_zero_is_the_default_distribution(env):
    """The shipped default must not have changed the sampled distribution.

    Under uniform sampling the obstacle happens to obstruct the direct path in
    roughly half the episodes; that is the distribution sac_v4 trained on and
    every number in the docs was measured with.
    """
    assert env.blocking_fraction == 0.0
    blocked = 0
    for seed in SEEDS:
        env.reset(seed=seed)
        if env._path_clearance(env._target, env._obs_base,
                               env._obs_dir) <= env.blocking_margin:
            blocked += 1
    assert 0 < blocked < len(SEEDS), (
        f'{blocked}/{len(SEEDS)} blocked — uniform sampling should give a mix')


def test_blocking_fraction_one_forces_every_obstacle_onto_the_path(env):
    """With the knob at 1.0 no episode may be solvable by ignoring the obstacle.

    Half the 50-episode benchmark was, under uniform sampling — which made a
    success rate mostly a reaching score. The feasibility guarantees must
    survive the extra constraint: the start still clears d_safe.
    """
    env.blocking_fraction = 1.0
    env.blocking_misses = 0
    try:
        for seed in SEEDS:
            obs, _ = env.reset(seed=seed)
            pc = env._path_clearance(env._target, env._obs_base, env._obs_dir)
            assert pc <= env.blocking_margin, (
                f'seed {seed}: obstacle {pc:.3f} m off the path, above the '
                f'{env.blocking_margin:.3f} m blocking margin')
            assert float(obs[23]) >= env.reset_min_clearance - 1e-9, (
                f'seed {seed}: blocking sampling broke the start clearance')
        assert env.blocking_misses == 0, (
            f'{env.blocking_misses} episodes could not find a blocking draw')
    finally:
        env.blocking_fraction = 0.0


def test_path_clearance_is_a_distance_to_the_segment(env):
    """Sanity: on the path -> ~0; off to the side -> clearly more.

    Note the sweep is CLIPPED into `obstacle.box_min/max`, so "far away" cannot
    be tested with a point outside that box: it gets clipped back onto the box
    face. The comparison is therefore between two in-box positions.
    """
    env.reset(seed=SEEDS[0])
    midpoint = 0.5 * (env._ee_pos() + env._target)
    aside = midpoint.copy()
    aside[1] = env.obs_box_max[1]          # same height/depth, pushed to the edge

    on_path = env._path_clearance(env._target, midpoint, np.zeros(3))
    off_path = env._path_clearance(env._target, aside, np.zeros(3))
    assert on_path < 0.05, f'midpoint of the path scored {on_path:.3f} m'
    assert off_path > on_path + 0.10, (
        f'a sideways obstacle ({off_path:.3f} m) must score well above one on '
        f'the path ({on_path:.3f} m)')
