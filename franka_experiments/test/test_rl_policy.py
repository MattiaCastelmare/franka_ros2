"""Unit tests for utils/rl_policy.py — the sim↔real contract of the Safe-RL
deployment node (pure numpy/YAML, no ROS environment needed).

The critical property under test is that ``rl_policy_commander`` rebuilds on the
robot exactly the observation/action pair ``franka_sim/envs/franka_cbf_env.py``
produced during training: same layout, same obstacle geometry, same q̈ scaling.
A silent divergence here is invisible until the arm moves.

Run with pytest, or directly:  python3 test_rl_policy.py
"""

import numpy as np
import pytest

from franka_experiments.utils.rl_policy import (
    ACT_DIM,
    OBS_DIM,
    action_to_qddot,
    build_observation,
    joint_limits_mismatch,
    nearest_obstacle,
    obstacle_centre,
    qddot_max_from_limits,
    resolve_model_path,
    resolve_sim_config_path,
    synthetic_obstacle,
)


# ── Observation layout ───────────────────────────────────────────────────────

def _obs_args():
    return (np.arange(7.0),              # q
            np.arange(7.0) * 0.1,        # qdot
            np.array([0.4, 0.1, 0.5]),   # ee
            np.array([0.6, -0.2, 0.3]),  # target
            np.array([0.5, 0.3, 0.4]),   # obstacle
            0.23)                        # d_min


def test_observation_width_and_layout():
    obs = build_observation(*_obs_args())
    assert obs.shape == (OBS_DIM,) and obs.dtype == np.float32
    assert OBS_DIM == 24
    assert np.allclose(obs[0:7], np.arange(7.0))
    assert np.allclose(obs[7:14], np.arange(7.0) * 0.1)
    assert np.allclose(obs[14:17], [0.4, 0.1, 0.5])
    assert np.allclose(obs[17:20], [0.6, -0.2, 0.3])
    assert np.allclose(obs[20:23], [0.5, 0.3, 0.4])
    assert obs[23] == pytest.approx(0.23)


def test_observation_matches_env_concatenation():
    """Byte-for-byte the same vector FrankaCBFEnv._get_obs builds."""
    q, qdot, ee, tgt, obst, d = _obs_args()
    env_style = np.concatenate([q, qdot, ee, tgt, obst, [d]]).astype(np.float32)
    assert np.array_equal(build_observation(q, qdot, ee, tgt, obst, d), env_style)


def test_observation_fills_preallocated_2d_buffer():
    buf = np.zeros((1, OBS_DIM), dtype=np.float32)
    out = build_observation(*_obs_args(), out=buf)
    assert out is buf                       # no allocation on the control path
    assert buf[0, 23] == pytest.approx(0.23)


def test_observation_sanitizes_nan_and_inf():
    q = np.full(7, np.nan)
    obs = build_observation(q, np.zeros(7), np.full(3, np.inf),
                            np.zeros(3), np.zeros(3), float('nan'))
    assert np.all(np.isfinite(obs))


def test_observation_rejects_wrong_buffer():
    with pytest.raises(ValueError):
        build_observation(*_obs_args(), out=np.zeros(10, dtype=np.float32))


# ── Action scaling ───────────────────────────────────────────────────────────

_QDDOT_MAX = np.array([6.0, 2.585, 3.5, 4.0, 17.0, 5.5, 17.0])


def test_action_scaling_matches_env():
    a = np.array([1.0, -1.0, 0.5, 0.0, -0.25, 0.1, -0.9])
    assert np.allclose(action_to_qddot(a, _QDDOT_MAX), a * _QDDOT_MAX)


def test_action_is_clipped_to_unit_box():
    a = np.array([5.0, -5.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    out = action_to_qddot(a, _QDDOT_MAX)
    assert out[0] == pytest.approx(_QDDOT_MAX[0])
    assert out[1] == pytest.approx(-_QDDOT_MAX[1])
    assert np.all(np.abs(out) <= _QDDOT_MAX + 1e-12)


def test_action_derate_never_widens_envelope():
    a = np.ones(ACT_DIM)
    out = action_to_qddot(a, _QDDOT_MAX, scale=0.3)
    assert np.allclose(out, 0.3 * _QDDOT_MAX)
    assert np.all(np.abs(out) < _QDDOT_MAX)


def test_action_non_finite_collapses_to_zero():
    a = np.array([np.nan, np.inf, -np.inf, 0.0, 0.0, 0.0, 0.0])
    out = action_to_qddot(a, _QDDOT_MAX)
    assert np.allclose(out[:3], 0.0)


def test_action_fills_preallocated_buffer():
    buf = np.zeros(ACT_DIM)
    out = action_to_qddot(np.ones(ACT_DIM), _QDDOT_MAX, out=buf)
    assert out is buf


# ── Obstacle slot reconstruction ─────────────────────────────────────────────

def test_obstacle_centre_restores_sim_surface_distance():
    """‖p_cp − centre‖ − r_obs − r_cp must equal the reported surface distance."""
    r_obs, r_cp, d = 0.08, 0.09, 0.25
    p_cp = np.array([0.4, 0.0, 0.5])
    n_hat = np.array([0.0, 1.0, 0.0])            # obstacle → robot
    p_human = p_cp - n_hat * (d + r_cp)          # engine convention
    c = obstacle_centre(p_human, n_hat, r_obs)
    assert np.linalg.norm(p_cp - c) - r_obs - r_cp == pytest.approx(d)


def test_nearest_obstacle_picks_minimum_distance():
    n = np.array([0.0, 1.0, 0.0])
    entries = [
        ('fr3_link4', 0.40, n, np.array([0.4, -0.4, 0.5])),
        ('fr3_link7', 0.12, n, np.array([0.4, -0.1, 0.5])),
        ('fr3_link5', 0.30, n, np.array([0.4, -0.3, 0.5])),
    ]
    centre, d_min = nearest_obstacle(entries, 0.08)
    assert d_min == pytest.approx(0.12)
    assert np.allclose(centre, np.array([0.4, -0.1, 0.5]) - 0.08 * n)


def test_nearest_obstacle_link_filter():
    n = np.array([1.0, 0.0, 0.0])
    entries = [
        ('fr3_link2', 0.05, n, np.zeros(3)),      # closest but not a trained CP
        ('fr3_link7', 0.20, n, np.ones(3)),
    ]
    _, d_min = nearest_obstacle(entries, 0.08, links=['fr3_link7'])
    assert d_min == pytest.approx(0.20)


def test_nearest_obstacle_empty_and_nan():
    n = np.array([1.0, 0.0, 0.0])
    assert nearest_obstacle([], 0.08) is None
    assert nearest_obstacle([('l', float('nan'), n, np.zeros(3))], 0.08) is None


def test_synthetic_obstacle_is_geometrically_consistent():
    ee = np.array([0.4, 0.0, 0.5])
    centre, d = synthetic_obstacle(ee, np.array([1.5, 0.0, 0.5]), 0.08)
    assert d == pytest.approx(np.linalg.norm(ee - centre) - 0.08)
    assert d > 1.0                                  # far ⇒ CBF rows inactive


# ── Config consistency (the sim-to-real guard) ───────────────────────────────

_LIMITS = {
    'joint1': [-2.9007, 2.9007, 2.62, 6.0, 500.0],
    'joint2': [-1.8361, 1.8361, 2.62, 2.585, 500.0],
    'joint3': [-2.9007, 2.9007, 2.62, 3.5, 500.0],
    'joint4': [-3.0770, -0.1169, 2.62, 4.0, 500.0],
    'joint5': [-2.8763, 2.8763, 5.26, 17.0, 500.0],
    'joint6': [0.4398, 4.6216, 4.18, 5.5, 500.0],
    'joint7': [-3.0508, 3.0508, 5.26, 17.0, 500.0],
}


def test_qddot_max_ordering():
    assert np.allclose(qddot_max_from_limits(_LIMITS), _QDDOT_MAX)


def test_limits_mismatch_detects_drift():
    drifted = {k: list(v) for k, v in _LIMITS.items()}
    drifted['joint5'][3] = 10.0
    msgs = joint_limits_mismatch(_LIMITS, drifted)
    assert len(msgs) == 1 and 'joint5.qddot_max' in msgs[0]


def test_limits_mismatch_silent_when_identical():
    assert joint_limits_mismatch(_LIMITS, _LIMITS) == []


def test_real_configs_are_in_sync():
    """franka_sim/config.yaml and config/fr3_control.yaml must stay mirrors.

    Skipped when the standalone franka_sim checkout is not next to the package
    (e.g. an installed-only deployment).
    """
    import os
    import yaml
    from franka_experiments.utils.rl_policy import find_sim_root

    sim_root = find_sim_root(__file__)
    ctrl = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                        '..', 'config', 'fr3_control.yaml')
    if not sim_root or not os.path.isfile(ctrl):
        pytest.skip('franka_sim / fr3_control.yaml not available in this layout')
    with open(os.path.join(sim_root, 'config.yaml')) as fh:
        sim = yaml.safe_load(fh)
    with open(ctrl) as fh:
        robot = yaml.safe_load(fh)
    assert joint_limits_mismatch(sim['joint_limits'],
                                 robot['joint_limits']) == []
    # The CBF gains the policy trained against must be the ones on the robot.
    for key in ('d_safe', 'k0_cbf', 'k1_cbf', 'rho_slack', 'k_brake',
                'cbf_obstacle_horizon', 'cbf_min_leverage', 'max_qddot_delta',
                'hard_v_margin', 'hard_q_margin', 'hard_brake_eta',
                'ws_margin', 'ws_horizon'):
        assert sim['cbf'][key] == robot['params'][key], f'cbf.{key} drift'
    assert sim['cbf']['ws_min'] == robot['params']['ws_min']
    assert sim['cbf']['ws_max'] == robot['params']['ws_max']


# ── Path resolution ──────────────────────────────────────────────────────────

def test_resolve_model_path(tmp_path):
    root = tmp_path / 'franka_sim'
    (root / 'models' / 'exp').mkdir(parents=True)
    onnx = root / 'models' / 'exp' / 'best_model.onnx'
    onnx.write_bytes(b'\x00')
    assert resolve_model_path(str(onnx)) == str(onnx)
    assert resolve_model_path('models/exp/best_model.onnx', str(root)) == str(onnx)
    with pytest.raises(FileNotFoundError):
        resolve_model_path('nope.onnx', str(root))


def test_resolve_sim_config_prefers_frozen_config(tmp_path):
    model_dir = tmp_path / 'models' / 'exp'
    model_dir.mkdir(parents=True)
    model = model_dir / 'best_model.onnx'
    model.write_bytes(b'\x00')
    frozen = model_dir / 'config.yaml'
    frozen.write_text('rl: {}\n')
    root = tmp_path / 'franka_sim'
    root.mkdir()
    (root / 'config.yaml').write_text('rl: {}\n')

    assert resolve_sim_config_path('', str(model), str(root)) == str(frozen)
    frozen.unlink()
    assert resolve_sim_config_path('', str(model), str(root)) == \
        str(root / 'config.yaml')
    assert resolve_sim_config_path('/explicit.yaml', str(model), str(root)) == \
        '/explicit.yaml'


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
