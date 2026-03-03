"""Minimal launch file for ee_random_waypoints_velocity_commander.

Starts *only* the random-waypoint commander node — assumes the RT velocity
blender (or legacy forward-velocity controller) is already running, e.g. via::

    ros2 launch franka_experiments wrapper_forward_velocity.launch.py

Usage
-----
::

    # defaults:
    ros2 launch franka_experiments random_waypoints_velocity.launch.py

    # with namespace:
    ros2 launch franka_experiments random_waypoints_velocity.launch.py namespace:=NS_1

    # override waypoint parameters:
    ros2 launch franka_experiments random_waypoints_velocity.launch.py \\
        num_waypoints:=20 segment_duration_s:=3.0 loop:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_node(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    num_waypoints = LaunchConfiguration('num_waypoints').perform(context)
    segment_duration_s = LaunchConfiguration('segment_duration_s').perform(context)
    hold_time_s = LaunchConfiguration('hold_time_s').perform(context)
    rate_hz = LaunchConfiguration('rate_hz').perform(context)
    qdot_max = LaunchConfiguration('qdot_max').perform(context)
    loop = LaunchConfiguration('loop').perform(context)

    node = Node(
        package='franka_experiments',
        executable='ee_random_waypoints_velocity_commander',
        name='ee_random_waypoints_velocity_commander',
        namespace=namespace if namespace else None,
        output='screen',
        parameters=[{
            'num_waypoints': int(num_waypoints),
            'segment_duration_s': float(segment_duration_s),
            'hold_time_s': float(hold_time_s),
            'rate_hz': float(rate_hz),
            'qdot_max': float(qdot_max),
            'loop': loop.lower() == 'true',
        }],
    )

    return [
        LogInfo(msg=['╔══ random_waypoints_velocity ════════════════════════╗']),
        LogInfo(msg=['[launch] namespace       : ',
                     namespace if namespace else '<none>']),
        LogInfo(msg=['[launch] num_waypoints   : ', num_waypoints]),
        LogInfo(msg=['[launch] segment_dur     : ', segment_duration_s, ' s']),
        LogInfo(msg=['[launch] hold_time       : ', hold_time_s, ' s']),
        LogInfo(msg=['[launch] rate_hz         : ', rate_hz]),
        LogInfo(msg=['[launch] qdot_max        : ', qdot_max, ' rad/s']),
        LogInfo(msg=['[launch] loop            : ', loop]),
        LogInfo(msg=['╚═════════════════════════════════════════════════════╝']),
        node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='ROS namespace for the node (empty = no namespace).'),
        DeclareLaunchArgument(
            'num_waypoints', default_value='10',
            description='Number of random waypoints to visit.'),
        DeclareLaunchArgument(
            'segment_duration_s', default_value='2.0',
            description='Duration of each point-to-point segment [s].'),
        DeclareLaunchArgument(
            'hold_time_s', default_value='0.3',
            description='Hold time at each waypoint [s].'),
        DeclareLaunchArgument(
            'rate_hz', default_value='200.0',
            description='Publish rate [Hz].'),
        DeclareLaunchArgument(
            'qdot_max', default_value='0.3',
            description='Per-joint velocity clamp [rad/s].'),
        DeclareLaunchArgument(
            'loop', default_value='false',
            description='Loop through waypoints indefinitely.'),
        OpaqueFunction(function=_launch_node),
    ])
