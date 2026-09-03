"""Reading ``franka_experiments`` configuration off disk.

OWNS
----
Every path-resolution and YAML-parsing helper the package uses to turn a file
into a plain Python ``dict`` / array:

* :func:`load_package_config`  — ``config/fr3_<name>.yaml`` from the package share
* :func:`load_config_file`     — any YAML file, by absolute path
* :func:`load_launch_defaults` — ``config/launch_defaults.yaml``
* :func:`load_franka_config_defaults` — ``franka_bringup/config/franka.config.yaml``
* :func:`load_extrinsics`      — camera extrinsic calibration → (R, t)
* :func:`load_camera_info_yaml` — ``sensor_msgs/CameraInfo`` dumps (multi-document)

DOES NOT OWN
------------
* Turning a config value into a validated ROS parameter — that is ``utils.params``.
* Declaring launch arguments or generating controller YAML — that is ``utils.ros``.
* Any interpretation of what the loaded values *mean*; callers own that.

Naming note: the package historically had **two** different functions called
``load_robot_config`` — one taking a config *name* and resolving it against the
package share (``utils.cbf_utils``), one taking an absolute *path*
(``utils.distance_utils``).  Same name, different contract.  They are
disambiguated here as :func:`load_package_config` and :func:`load_config_file`.
Both original names remain importable from their original modules, unchanged, so
no existing call site is affected.

Hot-path note: nothing here is called from a timer or subscription callback.
Every function does blocking file I/O and belongs in ``__init__`` only.
"""

from __future__ import annotations

import os
from typing import Any, Dict, Optional, Tuple

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory


def load_package_config(name: str) -> dict:
    """Load ``config/fr3_<name>.yaml`` from the ``franka_experiments`` share dir.

    Args:
        name: Config stem, e.g. ``'control'`` for ``config/fr3_control.yaml``.

    Returns:
        The parsed YAML document.

    Raises:
        FileNotFoundError: If the resolved file does not exist.
    """
    pkg_share = get_package_share_directory('franka_experiments')
    filename = os.path.join(pkg_share, 'config', f'fr3_{name}.yaml')
    with open(filename, 'r') as f:
        data = yaml.safe_load(f)
    return data


def load_config_file(path: str) -> dict:
    """Load an arbitrary YAML config file by absolute path.

    Args:
        path: Absolute path to the YAML file.

    Returns:
        The parsed YAML document.

    Raises:
        FileNotFoundError: If *path* does not exist.
    """
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def load_extrinsics(path: str) -> Tuple[np.ndarray, np.ndarray]:
    """Load camera extrinsic calibration from a YAML file.

    Args:
        path: Absolute path to a YAML file with ``translation`` (x, y, z) and
            ``rotation`` (x, y, z, w quaternion) sections.

    Returns:
        ``(R, t)`` where ``R`` is the (3, 3) rotation matrix (camera → base) and
        ``t`` the (3,) translation vector (camera → base).
    """
    with open(path, 'r') as f:
        data = yaml.safe_load(f)

    tx = data['translation']['x']
    ty = data['translation']['y']
    tz = data['translation']['z']

    qx = data['rotation']['x']
    qy = data['rotation']['y']
    qz = data['rotation']['z']
    qw = data['rotation']['w']

    R = np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ], dtype=float)

    return R, np.array([tx, ty, tz], dtype=float)


def load_camera_info_yaml(
    path: str,
    required_keys: Tuple[str, ...] = ("k",),
) -> Optional[Dict[str, Any]]:
    """Load a CameraInfo-style YAML file, handling multi-document streams.

    ``rgb_intrinsics.yaml`` / ``depth_intrinsics.yaml`` are raw dumps of
    ``sensor_msgs/CameraInfo`` and may contain ``---`` separators, which make a
    plain ``yaml.safe_load()`` fail with *"expected a single document"*.

    Args:
        path: Absolute path to the YAML file.
        required_keys: Keys that must be present for a document to be accepted.

    Returns:
        The first YAML document that is a non-empty ``dict`` containing all
        *required_keys*, or ``None`` if no such document is found.
    """
    with open(path, "r") as fh:
        for doc in yaml.safe_load_all(fh):
            if not isinstance(doc, dict):
                continue
            if all(key in doc for key in required_keys):
                return doc
    return None


def load_franka_config_defaults(robot_key: str = 'ROBOT1') -> Tuple[dict, str]:
    """Read ``franka_bringup/config/franka.config.yaml`` and return defaults.

    Args:
        robot_key: Top-level key naming the robot entry, e.g. ``'ROBOT1'``.

    Returns:
        ``(defaults, config_path)`` where *defaults* maps arm_id, robot_ip,
        use_fake_hardware, namespace, fake_sensor_commands and load_gripper to
        **string** values suitable for ``DeclareLaunchArgument``, and
        *config_path* is the file that was read (for logging).

    Note:
        Never raises: on any failure it warns on stdout and returns the
        hard-coded defaults, so launch composition cannot be broken by a missing
        or malformed bringup config.
    """
    defaults = {
        'arm_id': 'fr3',
        'robot_ip': '192.168.2.10',
        'use_fake_hardware': 'false',
        'namespace': '',
        'fake_sensor_commands': 'false',
        'load_gripper': 'false',
    }
    config_path = '<not found>'

    try:
        bringup_share = get_package_share_directory('franka_bringup')
        config_path = os.path.join(
            bringup_share, 'config', 'franka.config.yaml')
        with open(config_path, 'r') as fh:
            config = yaml.safe_load(fh)

        if config and robot_key in config:
            robot = config[robot_key]
            for key in defaults:
                if key in robot:
                    val = robot[key]
                    if isinstance(val, bool):
                        defaults[key] = 'true' if val else 'false'
                    else:
                        defaults[key] = str(val)
    except Exception as exc:                       # noqa: BLE001
        print(f'[load_franka_config_defaults] WARN: could not read '
              f'{config_path}: {exc} — using hard-coded defaults.')

    return defaults, config_path


def load_launch_defaults() -> Tuple[dict, str]:
    """Load ``franka_experiments/config/launch_defaults.yaml``.

    Returns:
        ``(defaults, defaults_path)`` where every key of *defaults* is a
        launch-argument name and every value is already a **string** suitable
        for ``DeclareLaunchArgument(default_value=...)``, and *defaults_path* is
        the file that was loaded (for logging).

    Note:
        Never raises: on any failure it warns on stdout and returns an empty
        dict, so callers fall back to their own hard-coded defaults.
    """
    defaults = {}  # empty → callers fall back to their own hardcoded values
    defaults_path = '<not found>'

    try:
        pkg_share = get_package_share_directory('franka_experiments')
        defaults_path = os.path.join(pkg_share, 'config', 'launch_defaults.yaml')
        with open(defaults_path, 'r') as fh:
            raw = yaml.safe_load(fh) or {}
        # Normalise: bools → 'true'/'false', everything else → str
        for k, v in raw.items():
            if isinstance(v, bool):
                defaults[k] = 'true' if v else 'false'
            else:
                defaults[k] = str(v)
    except Exception as exc:  # noqa: BLE001
        print(f'[load_launch_defaults] WARN: could not read '
              f'{defaults_path}: {exc} — helpers will use hard-coded defaults.')

    return defaults, defaults_path


# TODO[LEGACY]: duplicates load_package_config for the fr3_*.yaml files (same resolved path) | confidence: low | superseded-by: load_package_config (same module) | flagged: 2026-09-01
def load_package_yaml(pkg: str, rel: str) -> dict:
    """Load a YAML file addressed as (package, path relative to its share dir).

    MOVED here from ``nodes/cbf_safety_filter.py`` in Phase 3.

    Args:
        pkg: ROS package name, e.g. ``'franka_experiments'``.
        rel: Path relative to that package's share directory, e.g.
            ``'config/fr3_control.yaml'``.

    Returns:
        The parsed YAML document.

    Note:
        Overlaps :func:`load_package_config` for the ``fr3_*.yaml`` files —
        ``load_package_yaml('franka_experiments', 'config/fr3_control.yaml')``
        and ``load_package_config('control')`` resolve to the same file. Kept
        distinct because the call sites differ; see LEGACY.md.
    """
    with open(os.path.join(get_package_share_directory(pkg), rel)) as f:
        return yaml.safe_load(f)


def load_franka_joint_limits(
    joint_keys=None,
    robot: str = 'fr3',
    package: str = 'franka_description',
) -> dict:
    """Official per-joint limits, straight from ``franka_description``.

    Reads ``robots/<robot>/joint_limits.yaml`` — the file the URDF itself is
    generated from — so the numbers a safety filter enforces are the
    manufacturer's, not a copy that can drift. ``fr3_control.yaml`` still
    carries a ``joint_limits`` block for the other nodes; this bypasses it.

    Returns arrays in ``joint_keys`` order:

    ==============  ===========================================================
    ``q_min``       ``limit.lower``  [rad]
    ``q_max``       ``limit.upper``  [rad]
    ``qdot_max``    ``limit.velocity``  [rad/s]
    ``decel_max``   ``position_based_velocity_limits.deceleration_limit``
                    [rad/s²] — the deceleration authority Franka assumes when
                    it builds its position-based velocity envelope
    ``effort_max``  ``limit.effort``  [N·m]
    ``v_offset``    ``position_based_velocity_limits.velocity_offset`` [rad/s]
    ==============  ===========================================================

    NOTE — there is no acceleration limit to read. ``joint_limits.yaml`` has no
    ``acceleration`` field for the FR3 (nor does franka_fr3_moveit_config), so
    ``decel_max`` is the only official q̈-scale number the robot ships. Used
    symmetrically it is exact in the braking direction (that is literally what
    it means) and conservative in the accelerating one.

    ``v_offset`` is returned for completeness but is NOT used by
    :func:`~franka_experiments.utils.cbf_hard_limits.hard_accel_box`, whose
    braking curve is the pure ``sqrt(2·a·h)`` form without Franka's offset.

    Raises:
        KeyError / FileNotFoundError if the package or a joint is missing —
        deliberately fatal: a safety filter must not silently fall back to
        guessed limits.
    """
    import numpy as _np
    keys = list(joint_keys) if joint_keys is not None else [
        f'joint{i}' for i in range(1, 8)]
    doc = load_package_yaml(package, os.path.join('robots', robot,
                                                  'joint_limits.yaml'))
    missing = [k for k in keys if k not in doc]
    if missing:
        raise KeyError(
            f'{package}/robots/{robot}/joint_limits.yaml has no entry for '
            f'{missing} (found: {sorted(doc)})')

    def _col(path):
        out = []
        for k in keys:
            node = doc[k]
            for part in path:
                node = node[part]
            out.append(float(node))
        return _np.array(out)

    return {
        'joints':     keys,
        'q_min':      _col(('limit', 'lower')),
        'q_max':      _col(('limit', 'upper')),
        'qdot_max':   _col(('limit', 'velocity')),
        'effort_max': _col(('limit', 'effort')),
        'decel_max':  _col(('position_based_velocity_limits',
                            'deceleration_limit')),
        'v_offset':   _col(('position_based_velocity_limits',
                            'velocity_offset')),
    }


# ── CBF safety filter: the whole parameter block, as data ────────────────────
#
# Every knob of ``nodes/cbf_safety_filter`` lives here as ``name: (kind,
# bounds)`` and nowhere else. The VALUES live in
# ``config/fr3_control.yaml`` under ``params:`` and nowhere else either — this
# table carries only the type and the validation range, which are not
# configuration: a launch file must be able to retune a gain, not to widen the
# range a gain is allowed to take.
#
# The pattern this replaces had the default written twice, once in the YAML and
# once in the node, and they had drifted: ``qp_rate_hz`` read 200.0 in the node
# against 100.0 in the YAML, ``d_safe`` 0.2 against 0.15,
# ``joint_limit_row_horizon`` 0.6 against 0.30. The YAML won every time — the
# key exists, so ``p.get`` never reached its default — which made the node-side
# numbers dead code that nonetheless read like the configuration. A missing key
# is now a startup failure naming the key, not a silent fallback.

CBF_PARAM_SPEC = {
    'qp_rate_hz': ('float', dict(positive=True, maximum=1000.0)),
    'cbf_update_rate_hz': ('float', dict(positive=True, maximum=1000.0)),
    'd_safe': ('float', dict(minimum=0.0, maximum=2.0)),
    'cbf_obstacle_horizon': ('float', dict(positive=True, maximum=5.0)),
    'k0_cbf': ('float', dict(minimum=0.0, maximum=10000.0)),
    'k1_cbf': ('float', dict(minimum=0.0, maximum=10000.0)),
    'rho_slack': ('float', dict(positive=True)),
    'rho_slack_self_collision': ('float', dict(positive=True)),
    'rho_slack_joint_limit': ('float', dict(positive=True)),
    'rho_slack_singularity': ('float', dict(positive=True)),
    'rho_slack_retreat': ('float', dict(positive=True)),
    'rho_slack_link_speed': ('float', dict(positive=True)),
    'qp_solver': ('str', dict(choices=('osqp',))),
    'distance_timeout': ('float', dict(positive=True, maximum=10.0)),
    'nom_timeout': ('float', dict(positive=True, maximum=10.0)),
    'joint_state_timeout': ('float', dict(positive=True, maximum=10.0)),
    'joint_state_freeze_timeout': ('float', dict(positive=True, maximum=10.0)),
    'joint_state_freeze_min_speed': ('float', dict(positive=True, maximum=5.0)),
    'k_brake': ('float', dict(minimum=0.0, maximum=100.0)),
    'min_confidence': ('float', dict(minimum=0.0, maximum=1.0)),
    'cbf_min_leverage': ('float', dict(minimum=0.0, maximum=10.0)),
    'joint_limit_rows_enabled': ('bool', dict()),
    'joint_limit_row_margin': ('float', dict(minimum=0.0, maximum=1.0)),
    'joint_limit_row_horizon': ('float', dict(positive=True, maximum=3.0)),
    'joint_limit_row_velocity_horizon': ('bool', dict()),
    'joint_limit_row_horizon_lead': ('float', dict(minimum=1.0, maximum=10.0)),
    'self_collision_rows_enabled': ('bool', dict()),
    'self_collision_row_margin': ('float', dict(minimum=0.0, maximum=0.5)),
    'self_collision_row_horizon': ('float', dict(positive=True, maximum=1.0)),
    'self_collision_max_rows': ('int', dict(minimum=1, maximum=20)),
    'singularity_rows_enabled': ('bool', dict()),
    'singularity_sigma_floor': ('float', dict(positive=True, maximum=1.0)),
    'singularity_row_horizon': ('float', dict(positive=True, maximum=1.0)),
    'singularity_grad_eps': ('float', dict(positive=True, maximum=0.01)),
    'singularity_rot_scale': ('float', dict(positive=True, maximum=5.0)),
    'singularity_min_leverage': ('float', dict(minimum=0.0, maximum=1.0)),
    'singularity_frame': ('str', dict()),
    'velocity_box_margin': ('float', dict(positive=True, maximum=1.0)),
    'position_margin_rad': ('float', dict(minimum=0.0, maximum=0.5)),
    'position_brake_eta': ('float', dict(positive=True, maximum=1.0)),
    'obstacle_velocity_enabled': ('bool', dict()),
    'obstacle_velocity_alpha': ('float', dict(minimum=0.0, maximum=0.95)),
    'obstacle_velocity_max': ('float', dict(positive=True, maximum=10.0)),
    'distance_capture_age_max': ('float', dict(positive=True, maximum=10.0)),
    'distance_capture_skew_tol': ('float', dict(positive=True, maximum=1.0)),
    'enable_velocity_feedforward': ('bool', dict()),
    'obstacle_decel_assumed': ('float', dict(positive=True, maximum=50.0)),
    'velocity_feedforward_gain': ('float', dict(minimum=0.0, maximum=50.0)),
    'velocity_braking_margin_max': ('float', dict(positive=True, maximum=1.0)),
    'velocity_feedforward_min_frames': ('int', dict(minimum=1, maximum=100)),
    'enable_weighted_slack': ('bool', dict()),
    'slack_weight_max': ('float', dict(minimum=1.0, maximum=100.0)),
    'slack_weight_rho': ('float', dict(positive=True, maximum=3.0)),
    'slack_weight_rho_qlim': ('float', dict(minimum=0.0, maximum=6.0)),
    'slack_weight_alpha': ('float', dict(positive=True, maximum=50.0)),
    'retreat_cap_enabled': ('bool', dict()),
    'retreat_cap_base_speed': ('float', dict(minimum=0.0, maximum=5.0)),
    'retreat_cap_obstacle_gain': ('float', dict(minimum=0.0, maximum=5.0)),
    'retreat_cap_depth_gain': ('float', dict(minimum=0.0, maximum=50.0)),
    'retreat_cap_depth_speed_ref': ('float', dict(minimum=0.0, maximum=5.0)),
    'retreat_cap_engage_gap': ('float', dict(positive=True, maximum=2.0)),
    'retreat_cap_max_speed': ('float', dict(positive=True, maximum=5.0)),
    'retreat_cap_horizon_s': ('float', dict(positive=True, maximum=2.0)),
    'link_speed_rows_enabled': ('bool', dict()),
    'link_speed_max': ('float', dict(positive=True, maximum=5.0)),
    'link_speed_reaction_s': ('float', dict(positive=True, maximum=2.0)),
    'link_speed_horizon_s': ('float', dict(positive=True, maximum=2.0)),
    'link_speed_activate_frac': ('float', dict(minimum=0.0, maximum=1.0)),
    'cbf_h_recovery_alpha': ('float', dict(minimum=0.0, maximum=0.95)),
    'cbf_hdot_filter_alpha': ('float', dict(minimum=0.0, maximum=0.95)),
    'state_box_relax_s': ('float', dict(positive=True, maximum=1.0)),
    'slew_box_enabled': ('bool', dict()),
    'max_qddot_delta': ('float', dict(positive=True, maximum=100.0)),
    'diag_vel_ratio_thr': ('float', dict(minimum=0.0)),
    'osqp_max_iter': ('int', dict(positive=True)),
    'diag_period_s': ('float', dict(minimum=0.0)),
    'diag_disable_gc': ('bool', dict()),
    'diag_qddot_alpha': ('float', dict(minimum=0.0, maximum=1.0)),
    'tick_gap_warn_factor': ('float', dict(positive=True)),
}

#: Read straight from the YAML onto the namespace, without declaring them as
#: ROS parameters — their types (a string list, an optional float) are outside
#: what :mod:`utils.params` validates, and both are consumed once at
#: construction rather than tuned live.
CBF_RAW_KEYS = ('self_collision_safety_distance', 'self_collision_exclude_pairs')


def load_cbf_config(node, pkg: str = 'franka_experiments',
                    rel: str = 'config/fr3_control.yaml'):
    """Load the CBF filter's configuration and declare every parameter.

    Args:
        node: The node to declare the parameters on.
        pkg / rel: Where the configuration lives. Defaults to the file the
            torque-control stack actually launches with.

    Returns:
        ``(topics, P)`` — the raw ``topics:`` mapping, and a namespace with one
        attribute per entry of :data:`CBF_PARAM_SPEC`, named exactly as the YAML
        key so a call site greps straight to the configuration and to
        ``ros2 param list``. ``P.config_path`` carries the file it actually
        read: that is the INSTALLED copy, not the one being edited, and the
        difference between the two is a whole class of startup failure.

    Raises:
        KeyError: a spec entry has no value in the YAML.
        ValueError: a value is outside its declared bounds.
    """
    from franka_experiments.utils.params import declare_from_spec
    path = os.path.join(get_package_share_directory(pkg), rel)
    cfg = load_package_yaml(pkg, rel)
    P = declare_from_spec(node, cfg['params'], CBF_PARAM_SPEC, source=path)
    P.config_path = path
    # Two values the declare_* helpers cannot express, attached to the same
    # namespace so call sites do not need a second source: a LIST of strings,
    # and an OPTIONAL float whose None means "leave the xacro default alone".
    # Not ROS parameters, therefore not retunable at runtime — which is correct,
    # both are consumed once at construction to build the capsule model.
    for k in CBF_RAW_KEYS:
        setattr(P, k, cfg['params'].get(k))
    return cfg['topics'], P
