"""Wrapper: bringup + joint_velocity_example_controller spawner."""

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


def spawn_velocity_controller(context):
    """Resolve namespace and spawn with explicit --controller-manager path.

    Delayed via TimerAction so the bringup spawners (joint_state_broadcaster)
    finish first — concurrent configure_controller calls deadlock the CM
    executor on Humble.

    When use_fake_hardware is true, passes a param file that sets gazebo:=true
    on the controller so it skips the set_full_collision_behavior service call
    (that service does not exist with mock hardware and would deadlock the CM).
    """
    namespace = LaunchConfiguration('namespace').perform(context)
    use_fake = LaunchConfiguration('use_fake_hardware').perform(context).lower() == 'true'

    # Build absolute controller-manager node path
    if namespace:
        cm_name = '/' + namespace + '/controller_manager'
    else:
        cm_name = '/controller_manager'

    spawner_args = [
        'joint_velocity_example_controller',
        '--controller-manager', cm_name,
        '--controller-manager-timeout', '30',
    ]

    if use_fake:
        fake_params = PathJoinSubstitution([
            FindPackageShare('franka_experiments'),
            'config', 'fake_hw_controller_params.yaml',
        ]).perform(context)
        spawner_args += ['--param-file', fake_params]

    velocity_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=spawner_args,
        output='screen',
    )

    return [
        LogInfo(msg=['[wrapper_velocity] controller-manager path: ', cm_name]),
        LogInfo(msg=['[wrapper_velocity] use_fake_hardware: ',
                     'true' if use_fake else 'false']),
        LogInfo(msg=[
            '[wrapper_velocity] debug: '
            'ros2 service list | grep list_controllers  &&  '
            'ros2 control list_controllers -c ', cm_name,
        ]),
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
            'controllers_yaml',
            default_value=PathJoinSubstitution([
                FindPackageShare('franka_bringup'), 'config', 'controllers.yaml'
            ]),
            description='Path to controllers YAML'),
    ]

    # --- Include the official bringup launch file -------------------------
    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py'
        ])),
        launch_arguments={
            'arm_id': LaunchConfiguration('arm_id'),
            'robot_ip': LaunchConfiguration('robot_ip'),
            'namespace': LaunchConfiguration('namespace'),
            'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
            'fake_sensor_commands': LaunchConfiguration('fake_sensor_commands'),
            'load_gripper': LaunchConfiguration('load_gripper'),
            'controllers_yaml': LaunchConfiguration('controllers_yaml'),
        }.items(),
    )

    # --- Spawn velocity controller via OpaqueFunction ---------------------
    velocity_spawner = OpaqueFunction(function=spawn_velocity_controller)

    return LaunchDescription(declared_args + [franka_launch, velocity_spawner])
