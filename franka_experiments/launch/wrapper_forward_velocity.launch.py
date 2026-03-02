"""Wrapper: bringup + fr3_forward_velocity_controller spawner + velocity_blender.

Reads robot defaults (robot_ip, use_fake_hardware, namespace, arm_id …) from
  franka_bringup/config/franka.config.yaml  (ROBOT1 section)
so that  `ros2 launch franka_experiments wrapper_forward_velocity.launch.py`
works out-of-the-box without passing any argument.

Automatically selects the correct controllers YAML based on use_fake_hardware:
  - real hardware  → controllers_velocity_forward_real.yaml  (1000 Hz + broadcaster)
  - fake hardware  → controllers_velocity_forward.yaml       (100 Hz, mock)

After bringup, an optional **velocity_blender** node is started once
the ``fr3_forward_velocity_controller`` spawner process exits successfully
(via ``OnProcessExit``).  The blender subscribes to
``/<ns>/tracking_qdot``, clamps joint velocities, and publishes to the
controller's ``/commands`` topic.  Disable with ``start_blender:=false``.

Every argument is still overridable from the CLI, e.g.:
  ros2 launch … robot_ip:=10.0.0.1 use_fake_hardware:=true controllers_yaml:=/tmp/my.yaml
  ros2 launch … start_blender:=false
"""

import os

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


# ── Load defaults from franka_bringup config ────────────────────────
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
    # Hard-coded fallbacks in case the file is missing or unreadable
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
                    # Normalize booleans → lowercase string
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


# ── Helpers ──────────────────────────────────────────────────────────
def _pick_controllers_yaml(context):
    """Select real vs fake YAML unless the user already provided an override."""
    explicit = LaunchConfiguration('controllers_yaml').perform(context)
    if explicit != '__auto__':
        return explicit
    use_fake = LaunchConfiguration('use_fake_hardware').perform(context).lower() == 'true'
    yaml_file = ('controllers_velocity_forward.yaml' if use_fake
                 else 'controllers_velocity_forward_real.yaml')
    return PathJoinSubstitution([
        FindPackageShare('franka_experiments'), 'config', yaml_file,
    ]).perform(context)


# ── OpaqueFunction: resolve everything at runtime ────────────────────
def _launch_all(context):
    """Resolve YAML + namespace, include bringup, spawn controller, start blender."""
    controllers_yaml = _pick_controllers_yaml(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    use_fake = LaunchConfiguration('use_fake_hardware').perform(context)
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    arm_id = LaunchConfiguration('arm_id').perform(context)
    fake_sensor = LaunchConfiguration('fake_sensor_commands').perform(context)
    load_gripper = LaunchConfiguration('load_gripper').perform(context)
    cm_name = ('/' + namespace + '/controller_manager') if namespace else '/controller_manager'
    start_blender = (
        LaunchConfiguration('start_blender').perform(context).lower() == 'true')

    # --- Include bringup with the resolved YAML -----------------------
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

    # --- Forward velocity controller spawner --------------------------
    velocity_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'fr3_forward_velocity_controller',
            '--controller-manager', cm_name,
            '--controller-manager-timeout', '30',
        ],
        output='screen',
    )

    actions = [
        LogInfo(msg=['[wrapper] franka.config.yaml : ', _CONFIG_PATH]),
        LogInfo(msg=['[wrapper] arm_id             : ', arm_id]),
        LogInfo(msg=['[wrapper] robot_ip           : ', robot_ip]),
        LogInfo(msg=['[wrapper] namespace          : ',
                     namespace if namespace else '<none>']),
        LogInfo(msg=['[wrapper] use_fake_hardware  : ', use_fake]),
        LogInfo(msg=['[wrapper] controllers_yaml   : ', controllers_yaml]),
        LogInfo(msg=['[wrapper] controller-manager : ', cm_name]),
        franka_launch,
        # Delay to let bringup spawners finish first
        TimerAction(period=10.0, actions=[velocity_spawner]),
    ]

    # --- Velocity blender (optional) ----------------------------------
    if start_blender:
        # Resolve blender topics from launch arguments
        trk_topic = LaunchConfiguration('tracking_topic').perform(context)
        avd_topic = LaunchConfiguration('avoidance_topic').perform(context)
        cmd_topic = LaunchConfiguration(
            'output_command_topic').perform(context)
        rate_hz = LaunchConfiguration('blender_rate_hz').perform(context)
        qdot_max = LaunchConfiguration('blender_qdot_max').perform(context)
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

        # Native ROS 2 Node (managed by launch, visible in ros2 node list)
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
                'qdot_max': float(qdot_max),
                'watchdog_s': float(watchdog_s),
            }],
        )

        # Start blender ONLY after the controller spawner exits (= active).
        # OnProcessExit fires when velocity_spawner terminates with rc=0,
        # which means the controller has been successfully activated.
        blender_on_spawner_exit = RegisterEventHandler(
            OnProcessExit(
                target_action=velocity_spawner,
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
                         '  qdot_max=', qdot_max,
                         '  watchdog_s=', watchdog_s]),
            blender_on_spawner_exit,
        ])
    else:
        actions.append(
            LogInfo(msg=['[wrapper] velocity_blender  : DISABLED '
                         '(start_blender:=false)']))

    return actions


# ── generate_launch_description ──────────────────────────────────────
def generate_launch_description():

    declared_args = [
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
            description='Path to controllers YAML '
                        '(default: auto-select based on use_fake_hardware)'),
        # ── velocity_blender arguments ──────────────────────────────
        DeclareLaunchArgument(
            'start_blender',
            default_value='true',
            description='Start the velocity_blender node (true/false)'),
        DeclareLaunchArgument(
            'tracking_topic',
            default_value='__auto__',
            description='Blender input: tracking qdot topic '
                        '(__auto__ = /<ns>/tracking_qdot)'),
        DeclareLaunchArgument(
            'avoidance_topic',
            default_value='__auto__',
            description='Blender input: avoidance qdot topic '
                        '(__auto__ = /<ns>/avoidance_qdot)'),
        DeclareLaunchArgument(
            'output_command_topic',
            default_value='__auto__',
            description='Blender output: controller command topic '
                        '(__auto__ = /<ns>/fr3_forward_velocity_controller/commands)'),
        DeclareLaunchArgument(
            'blender_rate_hz',
            default_value='200.0',
            description='Blender publish rate [Hz]'),
        DeclareLaunchArgument(
            'blender_qdot_max',
            default_value='0.2',
            description='Blender per-joint velocity clamp [rad/s]'),
        DeclareLaunchArgument(
            'blender_watchdog_s',
            default_value='0.2',
            description='Blender per-channel watchdog timeout [s]'),
    ]

    # Everything resolved inside OpaqueFunction for runtime YAML selection
    launch_all = OpaqueFunction(function=_launch_all)

    return LaunchDescription(declared_args + [launch_all])
