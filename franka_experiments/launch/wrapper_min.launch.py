"""Minimal wrapper: includes franka_bringup/franka.launch.py forwarding args."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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

    return LaunchDescription(declared_args + [franka_launch])
