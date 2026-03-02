"""All-in-one: bringup + fr3_forward_velocity_controller + velocity_commander.

Automatically selects the correct controllers YAML based on use_fake_hardware:
  - real hardware  → controllers_velocity_forward_real.yaml  (1000 Hz, franka_robot_state_broadcaster)
  - fake hardware  → controllers_velocity_forward.yaml       (100 Hz, no broadcaster needed)

Override with  controllers_yaml:=<path>  on the command line.
"""

import ast

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# ── Helpers ──────────────────────────────────────────────────────────
def _resolve_cm_name(context):
    """Return the absolute controller_manager node path."""
    ns = LaunchConfiguration('namespace').perform(context)
    return ('/' + ns + '/controller_manager') if ns else '/controller_manager'


def _parse_float_list(raw: str):
    """Parse a string like '[0.1, 0.2, ...]' into a Python list of floats."""
    return [float(x) for x in ast.literal_eval(raw)]


def _pick_controllers_yaml(context):
    """Select real vs fake YAML unless the user already provided an override."""
    explicit = LaunchConfiguration('controllers_yaml').perform(context)
    # If the user passed a value on the CLI it won't equal our sentinel
    if explicit != '__auto__':
        return explicit
    use_fake = LaunchConfiguration('use_fake_hardware').perform(context).lower() == 'true'
    if use_fake:
        yaml_file = 'controllers_velocity_forward.yaml'
    else:
        yaml_file = 'controllers_velocity_forward_real.yaml'
    return PathJoinSubstitution([
        FindPackageShare('franka_experiments'), 'config', yaml_file,
    ]).perform(context)


# ── OpaqueFunction: bringup + spawn controller + commander ───────────
def _launch_all(context):
    controllers_yaml = _pick_controllers_yaml(context)
    cm_name = _resolve_cm_name(context)
    use_fake = LaunchConfiguration('use_fake_hardware').perform(context)

    # --- Include bringup with the resolved YAML -----------------------
    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py',
        ]).perform(context)),
        launch_arguments={
            'arm_id': LaunchConfiguration('arm_id').perform(context),
            'robot_ip': LaunchConfiguration('robot_ip').perform(context),
            'namespace': LaunchConfiguration('namespace').perform(context),
            'use_fake_hardware': use_fake,
            'fake_sensor_commands': LaunchConfiguration('fake_sensor_commands').perform(context),
            'load_gripper': LaunchConfiguration('load_gripper').perform(context),
            'controllers_yaml': controllers_yaml,
        }.items(),
    )

    # --- Forward velocity controller spawner (t ≈ 10 s) ---------------
    spawner_args = [
        'fr3_forward_velocity_controller',
        '--controller-manager', cm_name,
        '--controller-manager-timeout', '30',
    ]
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=spawner_args,
        output='screen',
    )

    # --- velocity_commander node (t ≈ 14 s) ---------------------------
    command_topic = LaunchConfiguration('command_topic').perform(context)
    rate_hz = float(LaunchConfiguration('commander_rate_hz').perform(context))
    amplitudes = _parse_float_list(
        LaunchConfiguration('commander_amplitudes').perform(context))
    frequencies = _parse_float_list(
        LaunchConfiguration('commander_frequencies').perform(context))
    offsets = _parse_float_list(
        LaunchConfiguration('commander_offsets').perform(context))

    commander_node = Node(
        package='franka_experiments',
        executable='velocity_commander',
        name='velocity_commander',
        output='screen',
        parameters=[{
            'command_topic': command_topic,
            'rate_hz': rate_hz,
            'amplitudes': amplitudes,
            'frequencies': frequencies,
            'offsets': offsets,
        }],
    )

    return [
        LogInfo(msg=['[experiment] controllers_yaml: ', controllers_yaml]),
        LogInfo(msg=['[experiment] controller-manager: ', cm_name]),
        LogInfo(msg=['[experiment] use_fake_hardware: ', use_fake]),
        LogInfo(msg=['[experiment] command_topic: ', command_topic]),
        franka_launch,
        # Delay controller spawn so bringup spawners finish first
        TimerAction(period=10.0, actions=[controller_spawner]),
        # Delay commander so controller is active before we publish
        TimerAction(period=14.0, actions=[commander_node]),
    ]


# ── generate_launch_description ──────────────────────────────────────
def generate_launch_description():

    # --- Bringup / robot arguments ------------------------------------
    declared_args = [
        DeclareLaunchArgument(
            'arm_id', default_value='fr3',
            description='Robot arm model identifier'),
        DeclareLaunchArgument(
            'robot_ip', default_value='192.168.2.10',
            description='IP address of the robot'),
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Namespace for the robot (empty = no namespace)'),
        DeclareLaunchArgument(
            'use_fake_hardware', default_value='false',
            description='Use fake (mock) hardware'),
        DeclareLaunchArgument(
            'fake_sensor_commands', default_value='false',
            description='Fake sensor commands'),
        DeclareLaunchArgument(
            'load_gripper', default_value='false',
            description='Load Franka Gripper'),
        DeclareLaunchArgument(
            'controllers_yaml', default_value='__auto__',
            description='Path to controllers YAML (default: auto-select based on use_fake_hardware)'),
        # --- Commander arguments --------------------------------------
        DeclareLaunchArgument(
            'commander_enabled', default_value='true',
            description='Start the velocity_commander node'),
        DeclareLaunchArgument(
            'commander_rate_hz', default_value='100.0',
            description='Publish rate for velocity commands (Hz)'),
        DeclareLaunchArgument(
            'commander_amplitudes',
            default_value='[0.0, 0.0, 0.0, 0.3, 0.3, 0.0, 0.0]',
            description='Sinusoidal amplitudes (rad/s) per joint'),
        DeclareLaunchArgument(
            'commander_frequencies',
            default_value='[0.0, 0.0, 0.0, 0.5, 0.5, 0.0, 0.0]',
            description='Sinusoidal frequencies (Hz) per joint'),
        DeclareLaunchArgument(
            'commander_offsets',
            default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
            description='Constant velocity offsets (rad/s) per joint'),
        DeclareLaunchArgument(
            'command_topic',
            default_value='/fr3_forward_velocity_controller/commands',
            description='Topic for ForwardCommandController commands'),
    ]

    # Everything is resolved inside the OpaqueFunction so that
    # controllers_yaml can be auto-selected at runtime.
    launch_all = OpaqueFunction(function=_launch_all)

    return LaunchDescription(declared_args + [launch_all])
