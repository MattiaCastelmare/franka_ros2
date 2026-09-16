#!/usr/bin/env python3

import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
    ExecuteProcess,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from franka_experiments.utils.distance_utils import load_robot_config


DEFAULT_BAG_PATH = '/bags/arm_repeated'


def create_bag_player(context):
    """Create the same rosbag playback process"""
    play_bag = LaunchConfiguration('play_bag').perform(context).lower()
    if play_bag not in ('true', '1', 'yes', 'on'):
        return []

    bag_path = os.path.expanduser(
        LaunchConfiguration('bag_path').perform(context)
    )
    if not bag_path:
        return [LogInfo(msg='bag_path is empty: rosbag was not started.')]

    return [ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--loop', '--clock', '100.0', '--read-ahead-queue-size', '1000'
        ],
        output='screen',
    )]


def generate_launch_description():
    package_share = get_package_share_directory('franka_experiments')
    camera_link_extrinsics_path = os.path.join(
        package_share,
        'config',
        'camera_extrinsics.yaml',
    )
    rviz_config_path = os.path.join(
        package_share,
        'config',
        'human.rviz',
    )
    human_parameters_path = os.path.join(
        package_share,
        'config',
        'human_params.yaml',
    )

    extrinsics = load_robot_config(camera_link_extrinsics_path)
    human_config = load_robot_config(human_parameters_path)

    translation = extrinsics['translation']
    rotation = extrinsics['rotation']
    play_bag = LaunchConfiguration('play_bag')
    rosbag_record = LaunchConfiguration('rosbag_record')
    run_name = LaunchConfiguration('run_name')
    default_run_name = time.strftime("%Y%m%d_%H%M%S")

    use_sim_time = ParameterValue(
        play_bag,
        value_type=bool,
    )

    publish_camera_tf = LaunchConfiguration('publish_camera_tf')

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fr3_to_camera_link',
        condition=IfCondition(publish_camera_tf),
        arguments=[
            '--x', str(translation['x']),
            '--y', str(translation['y']),
            '--z', str(translation['z']),
            '--qx', str(rotation['x']),
            '--qy', str(rotation['y']),
            '--qz', str(rotation['z']),
            '--qw', str(rotation['w']),
            '--frame-id', 'fr3_link0',
            '--child-frame-id', 'camera_color_optical_frame',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    tracker = Node(
        package='franka_experiments',
        executable='human_tracker',
        name='human_tracker',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    distance = Node(
        package='franka_experiments',
        executable='human_distance',
        name='human_distance',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    human_logger_node = Node(
        package='franka_experiments',
        executable='human_logging',
        name='human_logger',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    visualizer = Node(
        package='franka_experiments',
        executable='human_visualizer',
        name='human_visualizer',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    record_bag = ExecuteProcess(
        condition=IfCondition(rosbag_record),
        cmd=[
            'ros2', 'bag', 'record',
            '-o', ['recorded_bags/', run_name],
            '/NS_1/joint_states',
            '/camera/camera/aligned_depth_to_color/camera_info',
            '/camera/camera/aligned_depth_to_color/image_raw',
            '/camera/camera/color/camera_info',
            '/camera/camera/color/image_raw',
            '/tf', 
            '/tf_static'
        ],
        output='screen'
    )

    bag_player = TimerAction(
        period=2.0,
        actions=[OpaqueFunction(function=create_bag_player)],
    )

    camera_tf_delayed = TimerAction(
        period=4.0,
        actions=[camera_tf]
    )

    return LaunchDescription([
        DeclareLaunchArgument('play_bag', default_value='false'),
        DeclareLaunchArgument('rosbag_record', default_value='false'),
        DeclareLaunchArgument('bag_path', default_value=DEFAULT_BAG_PATH),
        DeclareLaunchArgument('publish_camera_tf', default_value='true'),
        DeclareLaunchArgument('run_name', default_value=default_run_name),
        camera_tf_delayed,
        tracker,
        distance,
        human_logger_node,
        visualizer,
        rviz,
        bag_player,
        record_bag,
    ])