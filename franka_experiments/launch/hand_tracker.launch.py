#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    bag_path = LaunchConfiguration('bag_path')
    rate = LaunchConfiguration('rate')

    base_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fr3_link0_to_hand_base_tf',
        output='log',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--qx', '0',
            '--qy', '0',
            '--qz', '0',
            '--qw', '1',
            '--frame-id', 'fr3_link0',
            '--child-frame-id', 'base',
        ],
    )

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

    handover_distance = Node(
        package='franka_experiments',
        executable='distance_handover_estimator',
        name='distance_handover_estimator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
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
            default_value='/ros2_ws/rosbags/handratacker_objecy', 
            # or  /ros2_ws/rosbags/handratacker_objecy /ros2_ws/rosbags/datasets-001/handtracker_poses
            # /ros2_ws/rosbags/datasets-001/arm_complex /ros2_ws/rosbags/datasets-001/arm_repeated
        ),

        DeclareLaunchArgument(
            'rate',
            default_value='1.0',
        ),

        base_alias_tf,
        tracker,
        kalman,
        estimator,
        compare_visualizer,
        handover_distance,
        logger,
        rviz,

        TimerAction(
            period=2.0,
            actions=[bag_player],
        ),
    ])