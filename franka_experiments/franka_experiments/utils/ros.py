"""ROS helpers: namespace resolution, topic building, common main patterns."""

from __future__ import annotations

import os
from typing import Optional

import yaml

import rclpy
import rclpy.executors
from rclpy.node import Node

from .constants import TRACKING_TOPIC_SUFFIX, CONTROLLER_NAME


# ---------------------------------------------------------------------------
# Namespace / topic resolution
# ---------------------------------------------------------------------------

def get_namespace_from_config(robot_key: str = 'ROBOT1') -> str:
    """Read namespace from ``franka_bringup/config/franka.config.yaml``.

    Returns the namespace string, or ``''`` if not found / any error.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        bringup_share = get_package_share_directory('franka_bringup')
        config_path = os.path.join(
            bringup_share, 'config', 'franka.config.yaml')
        with open(config_path, 'r') as fh:
            config = yaml.safe_load(fh)
        if config and robot_key in config:
            return str(config[robot_key].get('namespace', '')).strip()
    except Exception:  # noqa: BLE001
        pass
    return ''


def build_namespaced_topic(suffix: str, robot_key: str = 'ROBOT1') -> str:
    """Prefix *suffix* with ``/<ns>/`` if a namespace is configured."""
    ns = get_namespace_from_config(robot_key)
    if ns:
        return f'/{ns}/{suffix}'
    return f'/{suffix}'


def resolve_tracking_topic(robot_key: str = 'ROBOT1') -> str:
    """Auto-detect namespace → build ``tracking_qdot`` topic."""
    return build_namespaced_topic(TRACKING_TOPIC_SUFFIX, robot_key)


def resolve_controller_topic(robot_key: str = 'ROBOT1') -> str:
    """Auto-detect namespace → build ``<controller>/commands`` topic."""
    return build_namespaced_topic(f'{CONTROLLER_NAME}/commands', robot_key)


def resolve_topic_with_deprecated_alias(
    node: Node,
    new_param: str,
    deprecated_param: str,
    default_value: str,
) -> str:
    """Resolve a topic parameter honouring a deprecated alias.

    If *deprecated_param* is set and *new_param* still equals *default_value*,
    the deprecated value wins (with a deprecation warning).
    """
    new_val: str = node.get_parameter(new_param).value
    old_val: str = node.get_parameter(deprecated_param).value
    if old_val:
        node.get_logger().warn(
            f"Parameter '{deprecated_param}' is DEPRECATED — "
            f"use '{new_param}' instead.")
        if new_val == default_value:
            return old_val
    return new_val


# ---------------------------------------------------------------------------
# Common main() patterns
# ---------------------------------------------------------------------------

def run_node_main(node_factory, args=None) -> None:
    """Executor-based main loop for nodes with ``done`` flag + ``request_stop``.

    The node returned by *node_factory* must expose:

    * ``done: bool`` — set to ``True`` when the node is finished.
    * ``request_stop()`` — called on first ``KeyboardInterrupt``.
    """
    rclpy.init(args=args, signal_handler_options=rclpy.SignalHandlerOptions.NO)
    node = node_factory()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok() and not node.done:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        node.request_stop()
        # Keep spinning so the timer can publish zeros and set node.done
        try:
            while rclpy.ok() and not node.done:
                executor.spin_once(timeout_sec=0.1)
        except KeyboardInterrupt:
            pass  # 2nd Ctrl-C → exit immediately
    finally:
        try:
            executor.remove_node(node)
        except Exception:
            pass
        teardown(node)


def teardown(node: Node) -> None:
    """Destroy *node* and shutdown rclpy (best-effort, no exceptions)."""
    try:
        node.destroy_node()
    except Exception:
        pass
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


# ---------------------------------------------------------------------------
# Franka bringup configuration
# ---------------------------------------------------------------------------

def load_franka_config_defaults(robot_key='ROBOT1'):
    """Read ``franka_bringup/config/franka.config.yaml`` and return defaults.

    Returns
    -------
    defaults : dict
        Keys: arm_id, robot_ip, use_fake_hardware, namespace,
              fake_sensor_commands, load_gripper.
    config_path : str
        Absolute path to the YAML file that was read (for logging).
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
        from ament_index_python.packages import get_package_share_directory
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


def load_launch_defaults():
    """Load ``franka_experiments/config/launch_defaults.yaml``.

    Returns
    -------
    defaults : dict
        Every key is a launch-argument name; every value is already a *string*
        suitable for ``DeclareLaunchArgument(default_value=…)``.
    defaults_path : str
        Absolute path to the YAML file that was loaded (for logging).
    """
    defaults = {}  # empty → callers fall back to their own hardcoded values
    defaults_path = '<not found>'

    try:
        from ament_index_python.packages import get_package_share_directory
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


# ---------------------------------------------------------------------------
# Controller manager
# ---------------------------------------------------------------------------

def resolve_controller_manager_name(namespace: str) -> str:
    """Build the absolute ``controller_manager`` node path from *namespace*."""
    if namespace:
        return '/' + namespace + '/controller_manager'
    return '/controller_manager'


# ---------------------------------------------------------------------------
# Controllers YAML generation & selection
# ---------------------------------------------------------------------------

def generate_rt_controllers_yaml(
    is_real, arm_id, qdot_max, alpha,
    trk_topic, avd_topic, alpha_topic,
    max_accel, timeout_threshold_s, timeout_ramp_s,
    gazebo, enable_interpolation,
):
    """Build a complete, self-contained controllers YAML for RT blender mode.

    Returns the YAML content as a string.  All parameters are fully resolved
    — no ``REPLACE_ME`` placeholders.
    """
    lines = [
        '# Auto-generated at launch time by wrapper_forward_velocity.launch.py',
        '# Mode: RT velocity blender  —  DO NOT EDIT (regenerated every launch)',
        '',
        '/**:',
        '  controller_manager:',
        '    ros__parameters:',
        f'      update_rate: {1000 if is_real else 100}',
    ]
    if is_real:
        lines.append('      thread_priority: 98')
    lines += [
        '',
        '      joint_state_broadcaster:',
        '        type: joint_state_broadcaster/JointStateBroadcaster',
    ]
    if is_real:
        lines += [
            '',
            '      franka_robot_state_broadcaster:',
            '        type: franka_robot_state_broadcaster/FrankaRobotStateBroadcaster',
        ]
    lines += [
        '',
        '      rt_velocity_blender_controller:',
        '        type: franka_rt_controllers/RtVelocityBlenderController',
    ]
    if is_real:
        lines += [
            '',
            '/**:',
            '  franka_robot_state_broadcaster:',
            '    ros__parameters:',
            '      lock_try_count: 200',
            '      lock_sleep_interval: 50',
            '      lock_log_error: false',
            '      lock_update_success: true',
        ]
    lines += [
        '',
        '/**:',
        '  joint_state_broadcaster:',
        '    ros__parameters:',
        f'      arm_id: "{arm_id}"',
        '',
        '/**:',
        '  rt_velocity_blender_controller:',
        '    ros__parameters:',
        f'      arm_id: {arm_id}',
        '      joints:',
    ]
    for i in range(1, 8):
        lines.append(f'        - {arm_id}_joint{i}')
    lines += [
        f'      tracking_topic: {trk_topic}',
        f'      avoidance_topic: {avd_topic}',
        f'      alpha_topic: {alpha_topic}',
        f'      alpha: {alpha}',
        f'      qdot_max: {qdot_max}',
        f'      max_accel: {max_accel}',
        f'      timeout_threshold_s: {timeout_threshold_s}',
        f'      timeout_ramp_s: {timeout_ramp_s}',
        f'      gazebo: {gazebo}',
        f'      enable_interpolation: {enable_interpolation}',
        '',
    ]
    return '\n'.join(lines)


def write_temp_yaml(content, prefix='franka_rt_blender_'):
    """Write *content* to a temporary YAML file and return its path."""
    import tempfile
    fd, path = tempfile.mkstemp(prefix=prefix, suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        f.write(content)
    return path


def resolve_legacy_controllers_yaml(use_fake):
    """Return absolute path to the legacy static controllers YAML.

    Parameters
    ----------
    use_fake : bool
        If True, returns the fake-hardware YAML; otherwise the real one.
    """
    from ament_index_python.packages import get_package_share_directory
    pkg_share = get_package_share_directory('franka_experiments')
    yaml_file = ('controllers_velocity_forward.yaml' if use_fake
                 else 'controllers_velocity_forward_real.yaml')
    return os.path.join(pkg_share, 'config', yaml_file)


def pick_controllers_yaml(explicit, use_fake, use_rt, rt_params=None):
    """Select (or generate) the correct controllers YAML path.

    Parameters
    ----------
    explicit : str
        CLI override value (``'__auto__'`` means auto-select).
    use_fake : bool
        Whether fake hardware is being used.
    use_rt : bool
        Whether RT blender mode is active.
    rt_params : dict or None
        Keyword arguments for :func:`generate_rt_controllers_yaml`
        (required when *use_rt* is True).
    """
    if explicit != '__auto__':
        return explicit
    if use_rt:
        content = generate_rt_controllers_yaml(**rt_params)
        return write_temp_yaml(content)
    return resolve_legacy_controllers_yaml(use_fake)


# ---------------------------------------------------------------------------
# Launch argument declarations
# ---------------------------------------------------------------------------

def declare_robot_args(defaults=None):
    """Return common robot/hardware ``DeclareLaunchArgument`` list.

    These arguments are shared across multiple launch files in
    ``franka_experiments``.

    Parameters
    ----------
    defaults : dict or None
        Override default values.  Typically the dict returned by
        :func:`load_launch_defaults` (or :func:`load_franka_config_defaults`).
    """
    from launch.actions import DeclareLaunchArgument
    d = defaults or {}
    return [
        DeclareLaunchArgument(
            'arm_id', default_value=d.get('arm_id', 'fr3'),
            description='Robot arm model identifier'),
        DeclareLaunchArgument(
            'robot_ip', default_value=d.get('robot_ip', '192.168.2.10'),
            description='IP address of the robot'),
        DeclareLaunchArgument(
            'namespace', default_value=d.get('namespace', ''),
            description='Namespace for the robot (empty = no namespace)'),
        DeclareLaunchArgument(
            'use_fake_hardware', default_value=d.get('use_fake_hardware', 'false'),
            description='Use fake (mock) hardware'),
        DeclareLaunchArgument(
            'fake_sensor_commands', default_value=d.get('fake_sensor_commands', 'false'),
            description='Fake sensor commands'),
        DeclareLaunchArgument(
            'load_gripper', default_value=d.get('load_gripper', 'false'),
            description='Load Franka Gripper'),
        DeclareLaunchArgument(
            'controllers_yaml', default_value=d.get('controllers_yaml', '__auto__'),
            description='Path to controllers YAML (__auto__ = auto-select)'),
    ]


def declare_rt_blender_args(defaults=None):
    """Return RT-mode ``DeclareLaunchArgument`` list.

    Parameters
    ----------
    defaults : dict or None
        Override default values (from :func:`load_launch_defaults`).
    """
    from launch.actions import DeclareLaunchArgument
    d = defaults or {}
    return [
        DeclareLaunchArgument(
            'use_rt_blender', default_value=d.get('use_rt_blender', 'true'),
            description='true = RT C++ blending controller; '
                        'false = legacy ForwardCommandController + Python blender.'),
        DeclareLaunchArgument(
            'qdot_max', default_value=d.get('qdot_max', '0.2'),
            description='[RT] Per-joint velocity clamp [rad/s]. 0.0 = disabled.'),
        DeclareLaunchArgument(
            'alpha', default_value=d.get('alpha', '1.0'),
            description='[RT] Blend weight: qdot = alpha*tracking + (1-alpha)*avoidance.'),
        DeclareLaunchArgument(
            'max_accel', default_value=d.get('max_accel', '0.0'),
            description='[RT] Max joint acceleration [rad/s²]. 0.0 = disabled.'),
        DeclareLaunchArgument(
            'timeout_threshold_s', default_value=d.get('timeout_threshold_s', '0.0'),
            description='[RT] Seconds before smooth timeout ramp. 0.0 = disabled.'),
        DeclareLaunchArgument(
            'timeout_ramp_s', default_value=d.get('timeout_ramp_s', '0.0'),
            description='[RT] Linear ramp to zero duration [s]. 0.0 = disabled.'),
        DeclareLaunchArgument(
            'gazebo', default_value=d.get('gazebo', 'false'),
            description='[RT] Set true when running in Gazebo.'),
        DeclareLaunchArgument(
            'enable_interpolation', default_value=d.get('enable_interpolation', 'true'),
            description='[RT] Linear interpolation between low-rate samples.'),
        DeclareLaunchArgument(
            'alpha_topic', default_value=d.get('alpha_topic', 'blend_alpha'),
            description='[RT] Topic for runtime alpha updates (std_msgs/Float64).'),
        DeclareLaunchArgument(
            'tracking_topic', default_value=d.get('tracking_topic', '__auto__'),
            description='Tracking qdot topic (__auto__ = tracking_qdot).'),
        DeclareLaunchArgument(
            'avoidance_topic', default_value=d.get('avoidance_topic', '__auto__'),
            description='Avoidance qdot topic (__auto__ = avoidance_qdot).'),
    ]


def declare_legacy_blender_args(defaults=None):
    """Return legacy-blender ``DeclareLaunchArgument`` list.

    Parameters
    ----------
    defaults : dict or None
        Override default values (from :func:`load_launch_defaults`).
    """
    from launch.actions import DeclareLaunchArgument
    d = defaults or {}
    return [
        DeclareLaunchArgument(
            'start_blender', default_value=d.get('start_blender', 'true'),
            description='[Legacy] Start Python velocity_blender node.'),
        DeclareLaunchArgument(
            'output_command_topic', default_value=d.get('output_command_topic', '__auto__'),
            description='[Legacy] Blender output topic.'),
        DeclareLaunchArgument(
            'blender_rate_hz', default_value=d.get('blender_rate_hz', '200.0'),
            description='[Legacy] Python blender publish rate [Hz].'),
        DeclareLaunchArgument(
            'blender_qdot_max', default_value=d.get('blender_qdot_max', '0.2'),
            description='[Legacy] Python blender per-joint velocity clamp [rad/s].'),
        DeclareLaunchArgument(
            'blender_watchdog_s', default_value=d.get('blender_watchdog_s', '0.2'),
            description='[Legacy] Python blender per-channel watchdog timeout [s].'),
    ]


# ---------------------------------------------------------------------------
# Launch logging helpers
# ---------------------------------------------------------------------------

def build_wrapper_log_actions(
    params, use_rt, config_path, controllers_yaml, controller_name, cm_name,
):
    """Build ``LogInfo`` actions for ``wrapper_forward_velocity`` launch.

    Parameters
    ----------
    params : dict
        All resolved launch parameters (string values).
    use_rt : bool
        Whether RT blender mode is active.
    config_path : str
        Path to the ``franka.config.yaml`` that was loaded.
    controllers_yaml : str
        Resolved controllers YAML path.
    controller_name : str
        Name of the controller being spawned.
    cm_name : str
        Absolute controller_manager node path.
    """
    from launch.actions import LogInfo

    mode_str = ('RT (rt_velocity_blender_controller)' if use_rt
                else 'LEGACY (fr3_forward_velocity_controller)')
    ns_display = params['namespace'] or '<none>'

    actions = [
        LogInfo(msg=['╔══ wrapper_forward_velocity ══════════════════════════╗']),
        LogInfo(msg=['[wrapper] mode               : ', mode_str]),
        LogInfo(msg=['[wrapper] franka.config.yaml : ', config_path]),
        LogInfo(msg=['[wrapper] arm_id             : ', params['arm_id']]),
        LogInfo(msg=['[wrapper] robot_ip           : ', params['robot_ip']]),
        LogInfo(msg=['[wrapper] namespace          : ', ns_display]),
        LogInfo(msg=['[wrapper] use_fake_hardware  : ',
                     params['use_fake_hardware']]),
        LogInfo(msg=['[wrapper] controllers_yaml   : ', controllers_yaml]),
        LogInfo(msg=['[wrapper] controller_to_spawn: ', controller_name]),
        LogInfo(msg=['[wrapper] controller-manager : ', cm_name]),
    ]

    if use_rt:
        p = params
        trk_raw = p['tracking_topic']
        avd_raw = p['avoidance_topic']
        trk = 'tracking_qdot' if trk_raw == '__auto__' else trk_raw
        avd = 'avoidance_qdot' if avd_raw == '__auto__' else avd_raw

        actions += [
            LogInfo(msg=['[wrapper] ── RT controller parameters ──']),
            LogInfo(msg=['[wrapper]   tracking_topic   : ', trk,
                         '  (relative to controller NS)']),
            LogInfo(msg=['[wrapper]   avoidance_topic  : ', avd,
                         '  (relative to controller NS)']),
            LogInfo(msg=['[wrapper]   alpha_topic      : ',
                         p['alpha_topic']]),
            LogInfo(msg=['[wrapper]   alpha            : ', p['alpha']]),
            LogInfo(msg=['[wrapper]   qdot_max         : ', p['qdot_max'],
                         ' rad/s (scalar, same for all 7 joints)']),
            LogInfo(msg=['[wrapper]   max_accel        : ', p['max_accel'],
                         ' rad/s²',
                         '  [ACTIVE]' if float(p['max_accel']) > 0
                         else '  [DISABLED]']),
            LogInfo(msg=['[wrapper]   timeout_threshold: ',
                         p['timeout_threshold_s'], ' s',
                         '  [ACTIVE]' if (float(p['timeout_threshold_s']) > 0
                                          and float(p['timeout_ramp_s']) > 0)
                         else '  [DISABLED]']),
            LogInfo(msg=['[wrapper]   timeout_ramp     : ',
                         p['timeout_ramp_s'], ' s']),
            LogInfo(msg=['[wrapper]   gazebo           : ', p['gazebo']]),
            LogInfo(msg=['[wrapper]   interpolation    : ',
                         p['enable_interpolation'],
                         '  [ACTIVE]' if p['enable_interpolation'] == 'true'
                         else '  [DISABLED]']),
        ]

        if p.get('start_blender', 'false').lower() == 'true':
            actions.append(
                LogInfo(msg=['[wrapper] ⚠  start_blender=true IGNORED '
                             '— RT controller handles blending internally']))

        actions.append(
            LogInfo(msg=['[wrapper] Python velocity_blender: DISABLED '
                         '(RT mode — blending in C++ update())']))

    actions.append(
        LogInfo(msg=['╚═════════════════════════════════════════════════════╝']))

    return actions


# ---------------------------------------------------------------------------
# Self-check (moved from selfcheck.py)
# ---------------------------------------------------------------------------

def selfcheck_run():
    """Import all utils modules and run basic sanity checks.

    Returns 0 on success, 1 on failure.

    Usage::

        python3 -c "from franka_experiments.utils.ros import selfcheck_run; selfcheck_run()"
    """
    import pathlib
    import py_compile
    import sys

    errors = []

    # ---- constants ----
    try:
        from franka_experiments.utils.constants import (
            NUM_JOINTS, FR3_JOINT_NAMES, TRACKING_TOPIC_SUFFIX,
            CONTROLLER_NAME, AUTO_SENTINEL,
        )
        assert NUM_JOINTS == 7
        assert len(FR3_JOINT_NAMES) == 7
        assert FR3_JOINT_NAMES[0] == 'fr3_joint1'
        assert isinstance(TRACKING_TOPIC_SUFFIX, str)
        assert isinstance(CONTROLLER_NAME, str)
        assert isinstance(AUTO_SENTINEL, str)
        print('[OK] constants')
    except Exception as e:
        errors.append(f'constants: {e}')
        print(f'[FAIL] constants: {e}')

    # ---- math_utils ----
    try:
        from franka_experiments.utils.math_utils import (
            min_jerk, cosine_ramp, clamp_joints, lpf,
        )
        import numpy as np

        s, sd = min_jerk(0.5)
        assert 0.4 < s < 0.6, f's={s}'
        assert sd > 0.0, f'sdot={sd}'

        s0, sd0 = min_jerk(0.0)
        assert s0 == 0.0 and sd0 == 0.0
        s1, sd1 = min_jerk(1.0)
        assert abs(s1 - 1.0) < 1e-12 and abs(sd1) < 1e-12

        assert cosine_ramp(0.0, 1.0) == 0.0
        assert cosine_ramp(1.0, 1.0) == 1.0
        assert cosine_ramp(0.5, 0.0) == 1.0
        assert 0.0 < cosine_ramp(0.5, 1.0) < 1.0

        v = clamp_joints(np.array([1.0, -1.0, 0.1]), 0.5)
        assert np.allclose(v, [0.5, -0.5, 0.1])

        f = lpf(np.array([1.0]), np.array([0.0]), 0.5)
        assert np.allclose(f, [0.5])

        print('[OK] math_utils')
    except Exception as e:
        errors.append(f'math_utils: {e}')
        print(f'[FAIL] math_utils: {e}')

    # ---- trajectory ----
    try:
        from franka_experiments.utils.trajectory import (
            PentagonTrajectory, sample_waypoints, sample_single_waypoint,
        )
        import numpy as np

        traj = PentagonTrajectory(
            center=np.array([0.4, 0.0, 0.4]),
            radius=0.03, plane='front', cycle_time=15.0,
        )
        p, v = traj.evaluate(0.0)
        assert p.shape == (3,)
        assert v.shape == (3,)

        rng = np.random.default_rng(42)
        pts = sample_waypoints(
            5, [0.2, 0.5, -0.2, 0.2, 0.1, 0.5], 0.01, rng)
        assert len(pts) == 5

        wp = sample_single_waypoint(
            [0.2, 0.5, -0.2, 0.2, 0.1, 0.5], 0.01, rng)
        assert wp.shape == (3,)

        print('[OK] trajectory')
    except Exception as e:
        errors.append(f'trajectory: {e}')
        print(f'[FAIL] trajectory: {e}')

    # ---- logging_utils ----
    try:
        from franka_experiments.utils.logging_utils import (
            ThrottledLogger, vec_to_str,
        )
        import numpy as np

        assert vec_to_str(np.array([1.0, 2.0])) == '1.0000, 2.0000'
        assert vec_to_str(None) == '?'
        print('[OK] logging_utils')
    except Exception as e:
        errors.append(f'logging_utils: {e}')
        print(f'[FAIL] logging_utils: {e}')

    # ---- ros (import only — no rclpy.init needed) ----
    try:
        from franka_experiments.utils import ros  # noqa: F401
        print('[OK] ros (import)')
    except Exception as e:
        errors.append(f'ros: {e}')
        print(f'[FAIL] ros: {e}')

    # ---- kinematics (import only — pinocchio may not be installed) ----
    try:
        from franka_experiments.utils import kinematics  # noqa: F401
        # JointStateManager is now in kinematics (merged from joint_state)
        from franka_experiments.utils.kinematics import JointStateManager  # noqa: F401
        print('[OK] kinematics (import, includes JointStateManager)')
    except ImportError:
        print('[SKIP] kinematics (pinocchio not installed)')
    except Exception as e:
        errors.append(f'kinematics: {e}')
        print(f'[FAIL] kinematics: {e}')

    # ---- py_compile all .py files ----
    pkg_root = pathlib.Path(__file__).resolve().parent.parent
    py_files = sorted(pkg_root.rglob('*.py'))
    compile_ok = 0
    compile_fail = 0
    for pf in py_files:
        if '__pycache__' in str(pf):
            continue
        try:
            py_compile.compile(str(pf), doraise=True)
            compile_ok += 1
        except py_compile.PyCompileError as e:
            compile_fail += 1
            errors.append(f'py_compile {pf.name}: {e}')
            print(f'[FAIL] py_compile {pf.relative_to(pkg_root)}: {e}')
    print(f'[{"OK" if compile_fail == 0 else "FAIL"}] '
          f'py_compile: {compile_ok} ok, {compile_fail} failed '
          f'(out of {compile_ok + compile_fail} files)')

    print()
    if errors:
        print(f'FAILED ({len(errors)} error(s)):')
        for e in errors:
            print(f'  - {e}')
        return 1
    print('All checks passed.')
    return 0
