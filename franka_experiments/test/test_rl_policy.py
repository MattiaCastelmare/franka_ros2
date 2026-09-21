"""Unit tests for utils/rl_policy.py — the sim↔real contract of the Safe-RL
deployment node (pure numpy/YAML, no ROS environment needed).

The critical property under test is that ``rl_policy_commander`` rebuilds on the
robot exactly the observation/action pair ``franka_sim/envs/franka_cbf_env.py``
produced during training: same layout, same obstacle geometry, same q̈ scaling.
A silent divergence here is invisible until the arm moves.

Run with pytest, or directly:  python3 test_rl_policy.py
"""

import dataclasses
import os

import numpy as np
import pytest

from franka_experiments.utils.rl_policy import (
    ACT_DIM,
    CP_WIDTH,
    DEFAULT_CONTROL_POINTS,
    LEGACY_OBS_SPEC,
    OBS_DIM,
    ObsSpec,
    action_to_qddot,
    build_observation,
    control_point_geometry,
    obs_spec_from_config,
    obstacle_velocity,
    find_latest_model,
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


# ── Extended observation layout (obstacle velocity + control-point geometry) ─

_EXT_SPEC = ObsSpec(obstacle_velocity=True,
                    control_points=('fr3_link3', 'fr3_link7', 'fr3_link8'),
                    clip_distance=1.2)


def _sim_obs_layout():
    """Load franka_sim/envs/obs_layout.py off disk, or skip.

    Imported by PATH, not by package: franka_sim is deliberately not a ROS
    package and obs_layout is written to need nothing but numpy, precisely so
    this cross-check can run wherever the source tree is.
    """
    import importlib.util
    import os
    import sys
    from franka_experiments.utils.rl_policy import find_sim_root

    sim_root = find_sim_root(__file__)
    path = os.path.join(sim_root, 'envs', 'obs_layout.py') if sim_root else ''
    if not path or not os.path.isfile(path):
        pytest.skip('franka_sim checkout not available in this layout')
    spec = importlib.util.spec_from_file_location('_sim_obs_layout', path)
    mod = importlib.util.module_from_spec(spec)
    # Register BEFORE exec: @dataclass resolves annotations through
    # sys.modules[cls.__module__], which is None for an unregistered module.
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def test_observation_layout_mirrors_franka_sim():
    """This module and franka_sim/envs/obs_layout.py must agree, slot for slot.

    The layout is written out twice on purpose (the node cannot import a
    non-ROS package at runtime), so this test is the ONLY thing standing
    between a one-sided edit and a policy reading shifted slots on hardware.
    """
    sim = _sim_obs_layout()
    assert CP_WIDTH == sim.CP_WIDTH
    assert OBS_DIM == 2 * sim.NUM_JOINTS + 10
    assert list(DEFAULT_CONTROL_POINTS) == list(sim.DEFAULT_CONTROL_POINTS)

    for kwargs in ({},
                   {'obstacle_velocity': True},
                   {'control_points': ('fr3_link3', 'fr3_link8')},
                   {'obstacle_velocity': True,
                    'control_points': ('a', 'b', 'c'), 'clip_distance': 0.8}):
        here, there = ObsSpec(**kwargs), sim.ObsSpec(**kwargs)
        assert here.dim == there.dim
        assert here.slots == there.slots
        assert here.describe() == there.describe()


def test_extended_observation_matches_the_sim_assembly():
    """Byte-for-byte the vector franka_sim builds, optional blocks included."""
    sim = _sim_obs_layout()
    q, qdot, ee, tgt, obst, d = _obs_args()
    v = np.array([0.1, -0.2, 0.05])
    g = np.array([[0.30, 1.0, 0.0, 0.0],
                  [0.21, 0.0, 1.0, 0.0],
                  [0.17, 0.0, 0.0, 1.0]])
    theirs = sim.assemble(sim.ObsSpec(**dataclasses.asdict(_EXT_SPEC)),
                          q, qdot, ee, tgt,
                          obst, d, v_obs=v, cp_geometry=g)
    ours = build_observation(q, qdot, ee, tgt, obst, d, v_obs=v,
                             cp_geometry=g, spec=_EXT_SPEC)
    assert np.array_equal(ours, theirs)
    assert ours.shape == (_EXT_SPEC.dim,) == (24 + 3 + 3 * CP_WIDTH,)


def test_extended_observation_is_a_prefix_of_the_legacy_one():
    """Slots 0..23 keep their meaning, so sac_v4 keeps deploying unchanged."""
    args = _obs_args()
    legacy = build_observation(*args)
    extended = build_observation(*args, v_obs=np.zeros(3),
                                 cp_geometry=np.zeros((3, CP_WIDTH)),
                                 spec=_EXT_SPEC)
    assert np.array_equal(extended[:OBS_DIM], legacy)


def test_spec_from_config_defaults_to_the_legacy_layout():
    """A config frozen before `obs:` existed must resolve to 24 dims."""
    assert obs_spec_from_config({}) == LEGACY_OBS_SPEC
    assert obs_spec_from_config(None).dim == OBS_DIM
    assert obs_spec_from_config({'cbf': {'control_points': [{'body': 'x'}]}}).dim \
        == OBS_DIM


def test_spec_from_config_reads_the_robot_link_names():
    """The observation slot follows `robot_link`, not the MuJoCo body name.

    fr3_hand and fr3_link8 are the same frame, but MultiLinkDistance reports
    the URDF spelling — look the wrong one up and the gripper's slot is
    permanently "nothing near me".
    """
    cfg = {'obs': {'control_point_geometry': True},
           'cbf': {'cbf_obstacle_horizon': 0.9,
                   'control_points': [{'body': 'fr3_link7', 'radius': 0.07},
                                      {'body': 'fr3_hand', 'radius': 0.13,
                                       'robot_link': 'fr3_link8'}]}}
    spec = obs_spec_from_config(cfg)
    assert spec.control_points == ('fr3_link7', 'fr3_link8')
    assert spec.clip_distance == 0.9        # null → cbf_obstacle_horizon
    assert spec.dim == OBS_DIM + 2 * CP_WIDTH


def test_shipped_sim_config_is_the_extended_layout():
    """Guard the config the next run will actually train against."""
    import os
    import yaml
    from franka_experiments.utils.rl_policy import find_sim_root

    sim_root = find_sim_root(__file__)
    if not sim_root:
        pytest.skip('franka_sim checkout not available in this layout')
    with open(os.path.join(sim_root, 'config.yaml')) as fh:
        spec = obs_spec_from_config(yaml.safe_load(fh))
    assert spec.obstacle_velocity
    assert spec.control_points == ('fr3_link3', 'fr3_link4', 'fr3_link5',
                                   'fr3_link6', 'fr3_link7', 'fr3_link8')
    assert spec.dim == 51


# ── Control-point geometry from MultiLinkDistance ────────────────────────────

def _entry(name, d, n):
    return (name, d, np.asarray(n, float), np.zeros(3))


def test_control_point_geometry_takes_the_nearest_entry_per_link():
    """The robot reports several control points per link; the sim has one.

    fr3_complete.yaml samples 11 points along the segment axes, all carrying
    the same robot_link_name, so the nearest is the one that maps onto the
    sim's single sphere at the body origin.
    """
    g = control_point_geometry([
        _entry('fr3_link3', 0.40, [1, 0, 0]),
        _entry('fr3_link3', 0.12, [0, 1, 0]),   # nearest → wins
        _entry('fr3_link3', 0.25, [0, 0, 1]),
        _entry('fr3_link7', 0.31, [0, 0, 1]),
        _entry('fr3_link8', 0.19, [1, 0, 0]),
    ], _EXT_SPEC)
    assert g.shape == (3, CP_WIDTH)
    assert g[0, 0] == pytest.approx(0.12)
    assert np.allclose(g[0, 1:], [0, 1, 0])
    assert g[1, 0] == pytest.approx(0.31)
    assert g[2, 0] == pytest.approx(0.19)


def test_control_point_geometry_ignores_unknown_links_and_nans():
    g = control_point_geometry([
        _entry('fr3_link5', 0.05, [1, 0, 0]),       # not in the spec
        _entry('fr3_link3', float('nan'), [1, 0, 0]),
    ], _EXT_SPEC)
    assert np.allclose(g[:, 0], _EXT_SPEC.clip_distance)
    assert np.allclose(g[:, 1:], 0.0)


def test_control_point_geometry_marks_missing_links_with_a_zero_normal():
    """"Nothing near this link" = (clip, 0,0,0) — a token, not a direction.

    The sim emits the same thing for a degenerate row, so the two sides agree
    on the ABSENCE of information as well as on its presence.
    """
    g = control_point_geometry([_entry('fr3_link7', 0.22, [0, 0, 1])], _EXT_SPEC)
    assert g[0, 0] == pytest.approx(_EXT_SPEC.clip_distance)
    assert np.allclose(g[0, 1:], 0.0)
    assert g[1, 0] == pytest.approx(0.22)


def test_control_point_geometry_fills_a_preallocated_buffer():
    buf = np.full((3, CP_WIDTH), 7.0)
    out = control_point_geometry([_entry('fr3_link8', 0.1, [1, 0, 0])],
                                 _EXT_SPEC, out=buf)
    assert out is buf                       # no allocation on the control path
    assert buf[0, 0] == pytest.approx(_EXT_SPEC.clip_distance)


def test_far_distances_are_clipped_into_the_observation():
    """An unbounded 'far' feature would dominate the network's input scaling."""
    g = np.array([[9.0, 1.0, 0.0, 0.0]] * 3)
    obs = build_observation(*_obs_args()[:5], 9.0, v_obs=np.zeros(3),
                            cp_geometry=g, spec=_EXT_SPEC)
    assert obs[23] == pytest.approx(_EXT_SPEC.clip_distance)   # d_min too
    assert np.allclose(obs[27::CP_WIDTH][:3], _EXT_SPEC.clip_distance)


def test_build_observation_refuses_a_spec_it_cannot_fill():
    """Silently zero-filling a required block would train-test skew in silence."""
    with pytest.raises(ValueError):
        build_observation(*_obs_args(), spec=_EXT_SPEC)
    with pytest.raises(ValueError):
        build_observation(*_obs_args(), v_obs=np.zeros(3), spec=_EXT_SPEC)
    with pytest.raises(ValueError):
        build_observation(*_obs_args(), v_obs=np.zeros(3),
                          cp_geometry=np.zeros((2, CP_WIDTH)), spec=_EXT_SPEC)


# ── Obstacle velocity ────────────────────────────────────────────────────────

def test_obstacle_velocity_is_a_plain_finite_difference():
    v = obstacle_velocity(np.array([0.5, 0.2, 0.4]),
                          np.array([0.5, 0.1, 0.4]), 0.01)
    assert np.allclose(v, [0.0, 10.0, 0.0])


def test_obstacle_velocity_is_zero_without_a_previous_sample():
    """First tick, and after a perception outage — never a phantom sweep."""
    assert np.allclose(obstacle_velocity(np.ones(3), None, 0.01), 0.0)


def test_obstacle_velocity_rejects_an_unusable_timestep():
    """A zero/negative/NaN dt must not become an unbounded velocity."""
    p, q = np.array([0.5, 0.2, 0.4]), np.array([0.5, 0.1, 0.4])
    for dt in (0.0, -0.01, float('nan'), float('inf')):
        assert np.allclose(obstacle_velocity(p, q, dt), 0.0)


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
    # Same spelling on both sides — a key missing from either file is a failure,
    # not a skip: the robot renamed three of these once already (hard_v_margin →
    # velocity_box_margin, …) and a .get()-with-default here would have hidden it.
    for key in ('d_safe', 'k0_cbf', 'k1_cbf', 'rho_slack', 'k_brake',
                'cbf_obstacle_horizon', 'cbf_min_leverage', 'max_qddot_delta',
                'velocity_box_margin', 'position_margin_rad',
                'position_brake_eta',
                # Box SHAPE, added on the robot in 4606e39 / f5a59f8. Missing
                # from this list, `qddot_max_abs` silently cost the policy 41 %
                # of its trained action range on joints 5 and 7: the sim box
                # allowed 17 rad/s² while the robot clipped to 10.
                'qddot_max_abs', 'state_box_relax_s',
                'accel_box_clip_to_limits'):
        assert key in sim['cbf'], f'cbf.{key} missing from franka_sim/config.yaml'
        assert key in robot['params'], f'params.{key} missing from fr3_control.yaml'
        assert sim['cbf'][key] == robot['params'][key], f'cbf.{key} drift'


def test_accel_cap_applies_to_the_box_and_not_to_the_action_scale():
    """`qddot_max_abs` must cap the QP box WITHOUT rescaling the action.

    This is the robot's own arrangement and both halves matter:
    `rl_policy_commander` scales the action by fr3_control.yaml's
    `joint_limits` (17 rad/s² on joints 5 and 7) and `cbf_safety_filter`'s box
    then clips to `qddot_max_abs` = 10.

    * capping only the box (what this asserts) reproduces hardware;
    * capping the action TOO would change what `a = 1` means, so the ONNX
      actor would mean something different in sim than on the robot;
    * capping NEITHER is the bug this test was written for — the sim executed
      17 rad/s² on joints 5 and 7 while the robot clipped to 10, quietly
      removing 41 % of the trained action range at deployment.
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

    cap = float(sim['cbf']['qddot_max_abs'])
    sim_qdd = qddot_max_from_limits(sim['joint_limits'])
    robot_qdd = qddot_max_from_limits(robot['joint_limits'])

    # The action scale is the UNCAPPED limit, identical on both sides.
    assert np.allclose(sim_qdd, robot_qdd)
    assert sim_qdd.max() > cap, (
        'joint_limits qddot_max has been capped at qddot_max_abs — the action '
        'scale must stay uncapped so that a=1 means the same acceleration in '
        'sim as it does on the robot')
    # ...and the cap must actually bite somewhere, or it is not mirroring.
    assert np.any(sim_qdd > cap)


def test_workspace_box_is_sim_only():
    """The EE workspace box exists in sim and has NO robot counterpart.

    ``workspace_face_rows`` lost its last live importer in commit 4d4d450, so
    the robot enforces no Cartesian box.  Training inside one is deliberate and
    conservative (the policy learns a region smaller than the robot allows), but
    it must stay a documented asymmetry rather than a silent one: if workspace
    rows ever come back on the robot, this test fails and the two must be
    re-synchronised like every other key above.
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
    for key in ('ws_enable', 'ws_min', 'ws_max', 'ws_margin', 'ws_horizon'):
        assert key in sim['cbf'], f'cbf.{key} missing from franka_sim/config.yaml'
        assert key not in robot['params'], (
            f'params.{key} appeared in fr3_control.yaml — the workspace box is '
            f'no longer sim-only; add it to test_real_configs_are_in_sync')


#: Robot feature flags whose EFFECTIVE value comes from a launch argument
#: rather than from fr3_control.yaml. Mirrors the `parameters=[{...}]` block
#: that torque_control_stack.launch.py passes to cbf_safety_filter: those
#: overrides are ordinary ROS parameters, so the YAML value is only a default
#: and reading the YAML alone tells you the wrong thing (LEGACY.md §4).
_LAUNCH_OVERRIDES = {
    'enable_lateral_evasion':      'lateral_evasion',
    'enable_outrun_evasion':       'outrun_evasion',
    'enable_livelock_escape':      'livelock_escape',
    'enable_latency_compensation': 'latency_compensation',
    'enable_uncertainty_margin':   'uncertainty_margin',
    'enable_zone_ladder':          'zone_ladder',
    'enable_vobs_in_hdot':         'vobs_in_hdot',
    'enable_velocity_standoff':    'velocity_standoff',
    'iso_enabled':                 'iso_enabled',
    'iso_monitor_enabled':         'iso_monitor_enabled',
}


def test_shield_families_are_declared():
    """Every robot shield flag must be classified in `shield_parity`.

    The sim shield is a SUBSET of the deployed one and is allowed to be — but
    the gap has to be written down, because the difference between "trained
    under the deployed shield" and "trained under a subset of it" is the
    difference between a safety claim that holds and one that does not.

    Three things fail here, all of them things that have actually happened to
    this repo in one form or another:

    * a NEW row family appears on the robot and nobody re-reads franka_sim;
    * a family the register calls active is quietly switched off;
    * a family the register calls inactive (ISO, latency compensation) is
      switched on, silently widening the gap the policy never trained against.

    Effective values are resolved the way a RUN resolves them: launch argument
    first, YAML default only when the launch does not override it.
    """
    import os
    import re
    import yaml
    from franka_experiments.utils.rl_policy import find_sim_root

    here = os.path.dirname(os.path.realpath(__file__))
    sim_root = find_sim_root(__file__)
    ctrl = os.path.join(here, '..', 'config', 'fr3_control.yaml')
    defaults = os.path.join(here, '..', 'config', 'launch_defaults.yaml')
    if not sim_root or not os.path.isfile(ctrl) or not os.path.isfile(defaults):
        pytest.skip('franka_sim / robot configs not available in this layout')

    with open(os.path.join(sim_root, 'config.yaml')) as fh:
        sim = yaml.safe_load(fh)
    with open(ctrl) as fh:
        robot = yaml.safe_load(fh)['params']
    with open(defaults) as fh:
        launch = yaml.safe_load(fh)

    reg = sim.get('shield_parity')
    assert reg, 'franka_sim/config.yaml has no shield_parity block'

    declared = (list(reg.get('mirrored', []))
                + list(reg.get('robot_only_active', []))
                + list(reg.get('robot_only_inactive', [])))
    dupes = {k for k in declared if declared.count(k) > 1}
    assert not dupes, f'shield_parity lists these more than once: {sorted(dupes)}'

    # Discovery rule: the robot's own naming convention for a feature switch.
    # (`*_rows_enabled` is subsumed by `*_enabled`. The ISO row switches carry
    # no such suffix but are all gated by `iso_enabled`, which IS discovered.)
    flag = re.compile(r'(^enable_|_enabled$)')
    found = {k for k in robot if flag.search(k)}

    missing = found - set(declared)
    assert not missing, (
        'these robot shield flags are not classified in franka_sim/config.yaml '
        f'shield_parity: {sorted(missing)} — add each to mirrored, '
        'robot_only_active or robot_only_inactive')
    stale = set(declared) - found
    assert not stale, (
        f'shield_parity names flags that fr3_control.yaml no longer has: '
        f'{sorted(stale)}')

    def effective(key):
        """What a real `torque_control_stack` run would use for *key*."""
        arg = _LAUNCH_OVERRIDES.get(key)
        return bool(launch[arg]) if arg in launch else bool(robot[key])

    for key in reg.get('mirrored', []) + reg.get('robot_only_active', []):
        assert effective(key), (
            f'{key} is declared active but resolves to False — either the '
            'robot turned it off (move it to robot_only_inactive) or a launch '
            'default changed')
    for key in reg.get('robot_only_inactive', []):
        assert not effective(key), (
            f'{key} is declared INACTIVE but now resolves to True. It is a '
            'shield mechanism the policy has never trained against: move it '
            'to robot_only_active and state the consequence for the results')


def test_shield_parity_sim_only_entries_exist_in_sim():
    """`sim_only` must name real sim-side switches, not aspirational ones."""
    import os
    import yaml
    from franka_experiments.utils.rl_policy import find_sim_root

    sim_root = find_sim_root(__file__)
    if not sim_root:
        pytest.skip('franka_sim not available in this layout')
    with open(os.path.join(sim_root, 'config.yaml')) as fh:
        sim = yaml.safe_load(fh)
    for key in sim.get('shield_parity', {}).get('sim_only', []):
        assert key in sim['cbf'], f'shield_parity.sim_only names {key}, absent from cbf:'


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


def test_find_latest_model_picks_newest_by_mtime(tmp_path):
    """Newest WRITTEN export wins, not the highest-numbered filename.

    ``best_model.onnx`` is rewritten every time eval improves, so the two
    orderings genuinely disagree and only mtime answers "the one I just
    trained".
    """
    root = tmp_path / 'franka_sim'
    for name in ('sac_v9', 'sac_v1'):
        (root / 'models' / name).mkdir(parents=True)
        (root / 'models' / name / 'best_model.onnx').write_bytes(b'\x00')
    older = root / 'models' / 'sac_v9' / 'best_model.onnx'   # higher name...
    newer = root / 'models' / 'sac_v1' / 'best_model.onnx'   # ...older mtime
    os.utime(older, (1_000_000, 1_000_000))
    os.utime(newer, (2_000_000, 2_000_000))
    assert find_latest_model(str(root)) == str(newer)


def test_find_latest_model_searches_nested_checkpoints(tmp_path):
    """Episode checkpoints live in models/<exp>/checkpoints/ — reachable too."""
    root = tmp_path / 'franka_sim'
    ckpt = root / 'models' / 'exp' / 'checkpoints'
    ckpt.mkdir(parents=True)
    onnx = ckpt / 'sac_ep000400.onnx'
    onnx.write_bytes(b'\x00')
    assert find_latest_model(str(root)) == str(onnx)


def test_find_latest_model_ignores_zip_and_missing_roots(tmp_path):
    """A .zip must never be auto-selected: onnxruntime cannot load it.

    Returning '' lets the node raise its explicit "export a policy first"
    error instead of failing later inside InferenceSession.
    """
    root = tmp_path / 'franka_sim'
    (root / 'models' / 'exp').mkdir(parents=True)
    (root / 'models' / 'exp' / 'best_model.zip').write_bytes(b'\x00')
    assert find_latest_model(str(root)) == ''
    assert find_latest_model('') == ''                       # no sim checkout
    assert find_latest_model(str(tmp_path / 'nope')) == ''    # no models/ dir


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


def test_ee_frame_matches_sim_ee_site():
    """The node's EE frame must be the same physical point as the sim's site.

    The observation's `ee_pos` slot is whatever this point is; if MuJoCo tracks
    the hand TCP and Pinocchio the bare flange, every observation is offset by
    0.1034 m and the policy silently drives to the wrong place.  Parsed from the
    node's source rather than imported: this module is pure numpy/YAML on
    purpose and must stay importable with no ROS installed.
    """
    import os
    import re
    import yaml
    from franka_experiments.utils.rl_policy import find_sim_root

    # MuJoCo site  ->  the URDF frame naming the same point.
    EQUIVALENT = {
        'hand_tcp_site': 'fr3_hand_tcp',    # Franka Hand grasp centre
        'attachment_site': 'fr3_link8',     # bare flange (hand-less robot)
    }

    sim_root = find_sim_root(__file__)
    node = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..',
                        'franka_experiments', 'nodes', 'rl_policy_commander.py')
    if not sim_root or not os.path.isfile(node):
        pytest.skip('franka_sim / rl_policy_commander not available in this layout')

    with open(os.path.join(sim_root, 'config.yaml')) as fh:
        ee_site = yaml.safe_load(fh)['env']['ee_site']
    m = re.search(r"declare_parameter\(\s*'ee_frame'\s*,\s*'([^']+)'", open(node).read())
    assert m, "could not find the ee_frame declare_parameter default"
    ee_frame = m.group(1)

    assert ee_site in EQUIVALENT, (
        f'unknown ee_site {ee_site!r} — add its URDF frame to EQUIVALENT')
    assert ee_frame == EQUIVALENT[ee_site], (
        f'sim observes {ee_site!r} but the node defaults to ee_frame='
        f'{ee_frame!r}; expected {EQUIVALENT[ee_site]!r}')


if __name__ == '__main__':
    raise SystemExit(pytest.main([__file__, '-v']))
