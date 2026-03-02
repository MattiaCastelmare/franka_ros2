"""Wrapper: bringup + fr3_forward_velocity_controller spawner.

Automatically selects the correct controllers YAML based on use_fake_hardware:
  - real hardware  → controllers_velocity_forward_real.yaml  (1000 Hz, franka_robot_state_broadcaster)
  - fake hardware  → controllers_velocity_forward.yaml       (100 Hz, no broadcaster needed)

Override with  controllers_yaml:=<path>  on the command line.
"""

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


def _launch_all(context):
    """Resolve YAML + namespace, include bringup and spawn the controller."""
    controllers_yaml = _pick_controllers_yaml(context)
    namespace = LaunchConfiguration('namespace').perform(context)
    use_fake = LaunchConfiguration('use_fake_hardware').perform(context)
    cm_name = ('/' + namespace + '/controller_manager') if namespace else '/controller_manager'

    # --- Include bringup with the resolved YAML -----------------------
    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py',
        ]).perform(context)),
        launch_arguments={
            'arm_id': LaunchConfiguration('arm_id').perform(context),
            'robot_ip': LaunchConfiguration('robot_ip').perform(context),
            'namespace': namespace,
            'use_fake_hardware': use_fake,
            'fake_sensor_commands': LaunchConfiguration('fake_sensor_commands').perform(context),
            'load_gripper': LaunchConfiguration('load_gripper').perform(context),
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

    return [
        LogInfo(msg=['[wrapper_forward_velocity] controllers_yaml: ', controllers_yaml]),
        LogInfo(msg=['[wrapper_forward_velocity] controller-manager: ', cm_name]),
        LogInfo(msg=['[wrapper_forward_velocity] use_fake_hardware: ', use_fake]),
        franka_launch,
        # Delay to let bringup spawners finish first
        TimerAction(period=10.0, actions=[velocity_spawner]),
    ]


def generate_launch_description():

    # --- Declare wrapper arguments ----------------------------------------
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
            description='Use fake hardware'),
        DeclareLaunchArgument(
            'fake_sensor_commands', default_value='false',
            description='Fake sensor commands'),
        DeclareLaunchArgument(
            'load_gripper', default_value='false',
            description='Load Franka Gripper'),
        DeclareLaunchArgument(
            'controllers_yaml', default_value='__auto__',
            description='Path to controllers YAML (default: auto-select based on use_fake_hardware)'),
    ]

    # Everything resolved inside OpaqueFunction for runtime YAML selection
    launch_all = OpaqueFunction(function=_launch_all)

    return LaunchDescription(declared_args + [launch_all])
