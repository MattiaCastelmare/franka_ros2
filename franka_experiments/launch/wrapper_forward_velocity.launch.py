"""Wrapper: bringup + velocity controller spawner (RT or legacy).

Supports two modes controlled by ``use_rt_blender`` (default **true**):

**RT mode** (``use_rt_blender:=true``, DEFAULT):
    Spawns ``rt_velocity_blender_controller`` from ``franka_rt_controllers``.
    Blending (tracking + avoidance), rate-limiting, and velocity clamping all
    happen inside the 1 kHz C++ ``update()`` loop — NO Python blender is
    started, eliminating sample-and-hold jitter entirely.
    A runtime YAML is generated with resolved parameters (qdot_max, alpha,
    topic names, etc.) so that no REPLACE_ME placeholders remain.

**Legacy mode** (``use_rt_blender:=false``):
    Spawns ``fr3_forward_velocity_controller`` + optional Python
    ``velocity_blender`` node.  Behavior identical to the original wrapper.

Reads robot defaults from ``franka_bringup/config/franka.config.yaml``
(ROBOT1 section) so that::

    ros2 launch franka_experiments wrapper_forward_velocity.launch.py

works out-of-the-box.  Every argument is still overridable from the CLI::

    ros2 launch … use_rt_blender:=false                         # legacy
    ros2 launch … qdot_max:=0.15 max_accel:=10.0               # RT tuning
    ros2 launch … controllers_yaml:=/tmp/my.yaml                # full override
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# ╔═════════════════════════════════════════════════════════════════════════╗
# ║  Load defaults from franka_bringup config                              ║
# ╚═════════════════════════════════════════════════════════════════════════╝

def load_franka_config_defaults(robot_key='ROBOT1'):
    """Read franka_bringup/config/franka.config.yaml and return defaults.

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
        bringup_share = get_package_share_directory('franka_bringup')
        config_path = os.path.join(bringup_share, 'config', 'franka.config.yaml')
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
        print(f'[wrapper_forward_velocity] WARN: could not read '
              f'{config_path}: {exc} — using hard-coded defaults.')

    return defaults, config_path


# Load once at module level (runs when ros2 launch parses the file)
_DEFAULTS, _CONFIG_PATH = load_franka_config_defaults()


# ╔═════════════════════════════════════════════════════════════════════════╗
# ║  RT mode: generate a complete runtime YAML (solution #A)                ║
# ╚═════════════════════════════════════════════════════════════════════════╝

def _generate_rt_controllers_yaml(
    is_real, arm_id, qdot_max, alpha,
    trk_topic, avd_topic, alpha_topic,
    max_accel, timeout_threshold_s, timeout_ramp_s,
    gazebo, enable_interpolation,
):
    """Build a complete, self-contained controllers YAML for RT blender mode.

    Written to a temp file and passed as ``controllers_yaml`` to
    ``franka.launch.py``.  All parameters are fully resolved — no
    REPLACE_ME placeholders.
    """
    lines = [
        '# Auto-generated at launch time by wrapper_forward_velocity.launch.py',
        '# Mode: RT velocity blender  —  DO NOT EDIT (regenerated every launch)',
        '',
    ]

    # ── controller_manager ──────────────────────────────────────────
    lines += [
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

    # ── broadcaster parameters ──────────────────────────────────────
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
    ]

    # ── rt_velocity_blender_controller parameters ───────────────────
    lines += [
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


# ╔═════════════════════════════════════════════════════════════════════════╗
# ║  YAML selection logic                                                   ║
# ╚═════════════════════════════════════════════════════════════════════════╝

def _pick_controllers_yaml(context, use_rt):
    """Select (or generate) the correct controllers YAML.

    Priority:
      1. Explicit ``controllers_yaml`` CLI override  →  use as-is (any mode)
      2. RT mode  →  generate runtime temp YAML with resolved params
      3. Legacy mode  →  static YAML from franka_experiments/config/
    """
    explicit = LaunchConfiguration('controllers_yaml').perform(context)
    if explicit != '__auto__':
        return explicit

    use_fake = (LaunchConfiguration('use_fake_hardware')
                .perform(context).lower() == 'true')

    if use_rt:
        # ── Resolve all RT parameters ────────────────────────────────
        arm_id = LaunchConfiguration('arm_id').perform(context)
        qdot_max = LaunchConfiguration('qdot_max').perform(context)
        alpha = LaunchConfiguration('alpha').perform(context)
        max_accel = LaunchConfiguration('max_accel').perform(context)
        timeout_threshold_s = LaunchConfiguration(
            'timeout_threshold_s').perform(context)
        timeout_ramp_s = LaunchConfiguration(
            'timeout_ramp_s').perform(context)
        gazebo = LaunchConfiguration('gazebo').perform(context)
        enable_interpolation = LaunchConfiguration(
            'enable_interpolation').perform(context)
        alpha_topic = LaunchConfiguration('alpha_topic').perform(context)

        # Topic names: __auto__ → relative name (namespace-resolved by
        # ros2_control inside the controller_manager namespace).
        trk_raw = LaunchConfiguration('tracking_topic').perform(context)
        avd_raw = LaunchConfiguration('avoidance_topic').perform(context)
        trk_topic = 'tracking_qdot' if trk_raw == '__auto__' else trk_raw
        avd_topic = 'avoidance_qdot' if avd_raw == '__auto__' else avd_raw

        content = _generate_rt_controllers_yaml(
            is_real=not use_fake,
            arm_id=arm_id,
            qdot_max=qdot_max,
            alpha=alpha,
            trk_topic=trk_topic,
            avd_topic=avd_topic,
            alpha_topic=alpha_topic,
            max_accel=max_accel,
            timeout_threshold_s=timeout_threshold_s,
            timeout_ramp_s=timeout_ramp_s,
            gazebo=gazebo,
            enable_interpolation=enable_interpolation,
        )

        fd, path = tempfile.mkstemp(
            prefix='franka_rt_blender_', suffix='.yaml')
        with os.fdopen(fd, 'w') as f:
            f.write(content)
        return path

    # ── Legacy mode: static YAML ─────────────────────────────────────
    yaml_file = ('controllers_velocity_forward.yaml' if use_fake
                 else 'controllers_velocity_forward_real.yaml')
    return PathJoinSubstitution([
        FindPackageShare('franka_experiments'), 'config', yaml_file,
    ]).perform(context)


# ╔═════════════════════════════════════════════════════════════════════════╗
# ║  OpaqueFunction: resolve everything at runtime                          ║
# ╚═════════════════════════════════════════════════════════════════════════╝

def _launch_all(context):
    """Resolve YAML + namespace, include bringup, spawn controller."""
    use_rt = (LaunchConfiguration('use_rt_blender')
              .perform(context).lower() == 'true')

    namespace = LaunchConfiguration('namespace').perform(context)
    use_fake = LaunchConfiguration('use_fake_hardware').perform(context)
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    arm_id = LaunchConfiguration('arm_id').perform(context)
    fake_sensor = LaunchConfiguration('fake_sensor_commands').perform(context)
    load_gripper = LaunchConfiguration('load_gripper').perform(context)
    cm_name = (('/' + namespace + '/controller_manager')
               if namespace else '/controller_manager')
    start_blender = (LaunchConfiguration('start_blender')
                     .perform(context).lower() == 'true')

    controllers_yaml = _pick_controllers_yaml(context, use_rt)

    # ── Controller name (determined exclusively by use_rt_blender) ────
    controller_name = ('rt_velocity_blender_controller' if use_rt
                       else 'fr3_forward_velocity_controller')

    # ── Include franka bringup ────────────────────────────────────────
    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py',
        ]).perform(context)),
        launch_arguments={
            'arm_id': arm_id,
            'robot_ip': robot_ip,
            'namespace': namespace,
            'use_fake_hardware': use_fake,
            'fake_sensor_commands': fake_sensor,
            'load_gripper': load_gripper,
            'controllers_yaml': controllers_yaml,
        }.items(),
    )

    # ── Spawner for the selected controller ───────────────────────────
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            controller_name,
            '--controller-manager', cm_name,
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    # ── Common log header ─────────────────────────────────────────────
    mode_str = 'RT (rt_velocity_blender_controller)' if use_rt else 'LEGACY (fr3_forward_velocity_controller)'
    actions = [
        LogInfo(msg=['╔══ wrapper_forward_velocity ══════════════════════════╗']),
        LogInfo(msg=['[wrapper] mode               : ', mode_str]),
        LogInfo(msg=['[wrapper] franka.config.yaml : ', _CONFIG_PATH]),
        LogInfo(msg=['[wrapper] arm_id             : ', arm_id]),
        LogInfo(msg=['[wrapper] robot_ip           : ', robot_ip]),
        LogInfo(msg=['[wrapper] namespace          : ',
                     namespace if namespace else '<none>']),
        LogInfo(msg=['[wrapper] use_fake_hardware  : ', use_fake]),
        LogInfo(msg=['[wrapper] controllers_yaml   : ', controllers_yaml]),
        LogInfo(msg=['[wrapper] controller_to_spawn: ', controller_name]),
        LogInfo(msg=['[wrapper] controller-manager : ', cm_name]),
    ]

    # ── RT-mode specific log & safety checks ──────────────────────────
    if use_rt:
        qdot_max = LaunchConfiguration('qdot_max').perform(context)
        alpha = LaunchConfiguration('alpha').perform(context)
        max_accel = LaunchConfiguration('max_accel').perform(context)
        timeout_threshold_s = LaunchConfiguration(
            'timeout_threshold_s').perform(context)
        timeout_ramp_s = LaunchConfiguration(
            'timeout_ramp_s').perform(context)
        alpha_topic = LaunchConfiguration('alpha_topic').perform(context)
        gazebo = LaunchConfiguration('gazebo').perform(context)
        enable_interpolation = LaunchConfiguration(
            'enable_interpolation').perform(context)

        trk_raw = LaunchConfiguration('tracking_topic').perform(context)
        avd_raw = LaunchConfiguration('avoidance_topic').perform(context)
        trk_resolved = ('tracking_qdot' if trk_raw == '__auto__'
                        else trk_raw)
        avd_resolved = ('avoidance_qdot' if avd_raw == '__auto__'
                        else avd_raw)

        actions += [
            LogInfo(msg=['[wrapper] ── RT controller parameters ──']),
            LogInfo(msg=['[wrapper]   tracking_topic   : ', trk_resolved,
                         '  (relative to controller NS)']),
            LogInfo(msg=['[wrapper]   avoidance_topic  : ', avd_resolved,
                         '  (relative to controller NS)']),
            LogInfo(msg=['[wrapper]   alpha_topic      : ', alpha_topic]),
            LogInfo(msg=['[wrapper]   alpha            : ', alpha]),
            LogInfo(msg=['[wrapper]   qdot_max         : ', qdot_max,
                         ' rad/s (scalar, same for all 7 joints)']),
            LogInfo(msg=['[wrapper]   max_accel        : ', max_accel,
                         ' rad/s²',
                         '  [ACTIVE]' if float(max_accel) > 0 else '  [DISABLED]']),
            LogInfo(msg=['[wrapper]   timeout_threshold: ', timeout_threshold_s,
                         ' s',
                         '  [ACTIVE]' if (float(timeout_threshold_s) > 0
                                          and float(timeout_ramp_s) > 0)
                         else '  [DISABLED]']),
            LogInfo(msg=['[wrapper]   timeout_ramp     : ', timeout_ramp_s,
                         ' s']),
            LogInfo(msg=['[wrapper]   gazebo           : ', gazebo]),
            LogInfo(msg=['[wrapper]   interpolation    : ',
                         enable_interpolation,
                         '  [ACTIVE]' if enable_interpolation == 'true'
                         else '  [DISABLED]']),
        ]

        # WARN: start_blender is ignored in RT mode
        if start_blender:
            actions.append(
                LogInfo(msg=['[wrapper] ⚠  start_blender=true IGNORED '
                             '— RT controller handles blending internally']))

        actions.append(
            LogInfo(msg=['[wrapper] Python velocity_blender: DISABLED '
                         '(RT mode — blending in C++ update())']))

    actions.append(
        LogInfo(msg=['╚═════════════════════════════════════════════════════╝']))

    # ── Bringup + delayed spawner ─────────────────────────────────────
    actions += [
        franka_launch,
        TimerAction(period=10.0, actions=[controller_spawner]),
    ]

    # ── Legacy mode: optional Python velocity_blender ─────────────────
    if not use_rt:
        if start_blender:
            trk_topic = LaunchConfiguration(
                'tracking_topic').perform(context)
            avd_topic = LaunchConfiguration(
                'avoidance_topic').perform(context)
            cmd_topic = LaunchConfiguration(
                'output_command_topic').perform(context)
            rate_hz = LaunchConfiguration(
                'blender_rate_hz').perform(context)
            qdot_max_legacy = LaunchConfiguration(
                'blender_qdot_max').perform(context)
            watchdog_s = LaunchConfiguration(
                'blender_watchdog_s').perform(context)

            # Auto-resolve __auto__ topics based on namespace
            prefix = ('/' + namespace) if namespace else ''
            if trk_topic == '__auto__':
                trk_topic = prefix + '/tracking_qdot'
            if avd_topic == '__auto__':
                avd_topic = prefix + '/avoidance_qdot'
            if cmd_topic == '__auto__':
                cmd_topic = (prefix +
                             '/fr3_forward_velocity_controller/commands')

            blender_node = Node(
                package='franka_experiments',
                executable='velocity_blender',
                name='velocity_blender',
                output='screen',
                parameters=[{
                    'command_topic': cmd_topic,
                    'tracking_topic': trk_topic,
                    'avoidance_topic': avd_topic,
                    'rate_hz': float(rate_hz),
                    'qdot_max': float(qdot_max_legacy),
                    'watchdog_s': float(watchdog_s),
                }],
            )

            blender_on_spawner_exit = RegisterEventHandler(
                OnProcessExit(
                    target_action=controller_spawner,
                    on_exit=[
                        LogInfo(msg=['[wrapper] Controller spawner done '
                                     '— starting velocity_blender']),
                        blender_node,
                    ],
                )
            )

            actions.extend([
                LogInfo(msg=['[wrapper] velocity_blender  : ENABLED']),
                LogInfo(msg=['[wrapper]   tracking  <- ', trk_topic]),
                LogInfo(msg=['[wrapper]   avoidance <- ', avd_topic]),
                LogInfo(msg=['[wrapper]   command   -> ', cmd_topic]),
                LogInfo(msg=['[wrapper]   rate_hz=', rate_hz,
                             '  qdot_max=', qdot_max_legacy,
                             '  watchdog_s=', watchdog_s]),
                blender_on_spawner_exit,
            ])
        else:
            actions.append(
                LogInfo(msg=['[wrapper] velocity_blender  : DISABLED '
                             '(start_blender:=false)']))

    return actions


# ╔═════════════════════════════════════════════════════════════════════════╗
# ║  generate_launch_description                                            ║
# ╚═════════════════════════════════════════════════════════════════════════╝

def generate_launch_description():

    declared_args = [
        # ── Robot / hardware arguments ──────────────────────────────
        DeclareLaunchArgument(
            'arm_id',
            default_value=_DEFAULTS['arm_id'],
            description='Robot arm model identifier '
                        f'(default from franka.config.yaml: {_DEFAULTS["arm_id"]})'),
        DeclareLaunchArgument(
            'robot_ip',
            default_value=_DEFAULTS['robot_ip'],
            description='IP address of the robot '
                        f'(default from franka.config.yaml: {_DEFAULTS["robot_ip"]})'),
        DeclareLaunchArgument(
            'namespace',
            default_value=_DEFAULTS['namespace'],
            description='Namespace for the robot '
                        f'(default from franka.config.yaml: '
                        f'"{_DEFAULTS["namespace"]}" — empty = no namespace)'),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value=_DEFAULTS['use_fake_hardware'],
            description='Use fake hardware '
                        f'(default from franka.config.yaml: {_DEFAULTS["use_fake_hardware"]})'),
        DeclareLaunchArgument(
            'fake_sensor_commands',
            default_value=_DEFAULTS['fake_sensor_commands'],
            description='Fake sensor commands'),
        DeclareLaunchArgument(
            'load_gripper',
            default_value=_DEFAULTS['load_gripper'],
            description='Load Franka Gripper'),
        DeclareLaunchArgument(
            'controllers_yaml',
            default_value='__auto__',
            description='Path to controllers YAML (overrides auto-selection). '
                        '__auto__ = select based on use_rt_blender + use_fake_hardware.'),

        # ── Mode selection ──────────────────────────────────────────
        DeclareLaunchArgument(
            'use_rt_blender',
            default_value='true',
            description='true = RT C++ blending controller (no jitter, '
                        'no Python blender). '
                        'false = legacy ForwardCommandController + Python blender.'),

        # ── RT controller protection parameters ─────────────────────
        # These match the ACTUAL parameters declared in
        # franka_rt_controllers::RtVelocityBlenderController::on_init().
        # qdot_max is a SCALAR double (same limit for all 7 joints).
        DeclareLaunchArgument(
            'qdot_max',
            default_value='0.2',
            description='[RT mode] Per-joint velocity clamp [rad/s]. '
                        'Scalar — same limit for all 7 joints. '
                        '0.0 = DISABLED. Default 0.2 matches legacy blender.'),
        DeclareLaunchArgument(
            'alpha',
            default_value='1.0',
            description='[RT mode] Blend weight: '
                        'qdot = alpha*tracking + (1-alpha)*avoidance. '
                        'Changeable at runtime via topic or ros2 param set.'),
        DeclareLaunchArgument(
            'max_accel',
            default_value='0.0',
            description='[RT mode] Max joint acceleration [rad/s²] for '
                        'rate-limiter. 0.0 = DISABLED.'),
        DeclareLaunchArgument(
            'timeout_threshold_s',
            default_value='0.0',
            description='[RT mode] Seconds before smooth timeout ramp starts. '
                        '0.0 = DISABLED (last command held indefinitely).'),
        DeclareLaunchArgument(
            'timeout_ramp_s',
            default_value='0.0',
            description='[RT mode] Duration of linear ramp to zero [s] after '
                        'timeout threshold. 0.0 = DISABLED.'),
        DeclareLaunchArgument(
            'gazebo',
            default_value='false',
            description='[RT mode] Set true when running in Gazebo \u2014 skips '
                        'SetFullCollisionBehavior service call.'),
        DeclareLaunchArgument(
            'enable_interpolation',
            default_value='false',
            description='[RT mode] Linear interpolation between consecutive '
                        'low-rate samples in the 1 kHz RT loop. '
                        'Eliminates sample-and-hold velocity steps.'),
        DeclareLaunchArgument(
            'alpha_topic',
            default_value='blend_alpha',
            description='[RT mode] Topic name for runtime alpha updates '
                        '(std_msgs/Float64, 0..1).'),

        # ── Topic arguments (shared by RT and legacy modes) ─────────
        DeclareLaunchArgument(
            'tracking_topic',
            default_value='__auto__',
            description='Tracking qdot topic. __auto__ = tracking_qdot '
                        '(relative in RT mode, absolute in legacy mode).'),
        DeclareLaunchArgument(
            'avoidance_topic',
            default_value='__auto__',
            description='Avoidance qdot topic. __auto__ = avoidance_qdot '
                        '(relative in RT mode, absolute in legacy mode).'),

        # ── Legacy-only arguments ───────────────────────────────────
        DeclareLaunchArgument(
            'start_blender',
            default_value='true',
            description='[Legacy mode only] Start Python velocity_blender '
                        'node. Ignored when use_rt_blender:=true.'),
        DeclareLaunchArgument(
            'output_command_topic',
            default_value='__auto__',
            description='[Legacy mode only] Blender output topic '
                        '(__auto__ = /<ns>/fr3_forward_velocity_controller/commands).'),
        DeclareLaunchArgument(
            'blender_rate_hz',
            default_value='200.0',
            description='[Legacy mode only] Python blender publish rate [Hz]'),
        DeclareLaunchArgument(
            'blender_qdot_max',
            default_value='0.2',
            description='[Legacy mode only] Python blender per-joint '
                        'velocity clamp [rad/s]'),
        DeclareLaunchArgument(
            'blender_watchdog_s',
            default_value='0.2',
            description='[Legacy mode only] Python blender per-channel '
                        'watchdog timeout [s]'),
    ]

    launch_all = OpaqueFunction(function=_launch_all)

    return LaunchDescription(declared_args + [launch_all])
