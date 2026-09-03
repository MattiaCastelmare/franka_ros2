#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    bag_path = LaunchConfiguration('bag_path')
    rate = LaunchConfiguration('rate')

    rviz_config = PathJoinSubstitution([
        FindPackageShare('franka_experiments'),
        'config',
        'hand_tracker.rviz',
    ])

    tracker = Node(
        package='franka_experiments',
        executable='human_hand_tracker',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'publish_debug_image': True,
            'show_selected_landmarks': True,
        }],
    )

    kalman = Node(
        package='franka_experiments',
        executable='kalman_hand',
        output='screen',
    )

    estimator = Node(
        package='franka_experiments',
        executable='hand_state_estimator',
        output='screen',
    )

    compare_visualizer = Node(
        package='franka_experiments',
        executable='hand_compare_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
    )

    logger = Node(
        package='franka_experiments',
        executable='hand_logger',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': True,
        }],
    )

    bag_player = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'play',
            bag_path,
            '--clock',
            '--rate',
            rate,
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_path',
            default_value='/ros2_ws/rosbags/datasets-001/arm_repeated', 
            # or  /ros2_ws/rosbags/handratacker_objecy /ros2_ws/rosbags/datasets-001/handtracker_poses
            # /ros2_ws/rosbags/datasets-001/arm_complex /ros2_ws/rosbags/datasets-001/arm_repeated
        ),

        DeclareLaunchArgument(
            'rate',
            default_value='1.0',
        ),

        tracker,
        kalman,
        estimator,
        compare_visualizer,
        logger,
        rviz,

        TimerAction(
            period=2.0,
            actions=[bag_player],
        ),
    ])