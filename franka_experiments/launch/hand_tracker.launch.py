#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import (
    FindPackageShare,
)


def generate_launch_description():

    bag_path = LaunchConfiguration(
        'bag_path'
    )

    rate = LaunchConfiguration(
        'rate'
    )

    velocity_mode = LaunchConfiguration(
        'velocity_mode'
    )


    model_complexity = LaunchConfiguration(
        'model_complexity'
    )


    static_image_mode = LaunchConfiguration(
        'static_image_mode'
    )

    min_tracking_confidence = LaunchConfiguration(
        'min_tracking_confidence'
    )


    min_detection_confidence = LaunchConfiguration(
        'min_detection_confidence'
    )


    start_rviz = LaunchConfiguration(
        'start_rviz'
    )

    publish_base_alias_tf = (
        LaunchConfiguration(
            'publish_base_alias_tf'
        )
    )

    base_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fr3_link0_to_hand_base_tf',
        output='log',
        condition=IfCondition(
            publish_base_alias_tf
        ),
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
        FindPackageShare(
            'franka_experiments'
        ),
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
            'model_complexity': ParameterValue(
                model_complexity,
                value_type=int,
            ),
            'static_image_mode': ParameterValue(
                static_image_mode,
                value_type=bool,
            ),

            'min_tracking_confidence': ParameterValue(
                min_tracking_confidence,
                value_type=float,
            ),

            'min_detection_confidence': ParameterValue(
                min_detection_confidence,
                value_type=float,
            ),
        }],
    )

    kalman = Node(
        package='franka_experiments',
        executable='kalman_hand',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
    )

    estimator = Node(
        package='franka_experiments',
        executable='hand_state_estimator',
        output='screen',
        parameters=[{'use_sim_time': True, 'velocity_mode': velocity_mode, 'max_position_age_s': 0.1}],
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

    handover_observer = Node(
        package='franka_experiments',
        executable='handover_observer',
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
        parameters=[{
            'use_sim_time': True,
        }],
    )


    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(start_rviz),
        output='screen',
        arguments=[
            '-d',
            rviz_config,
        ],
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
            default_value=(
                '/ros2_ws/rosbags/'
                'datasets-001/'
                'arm_complex'
            ),
        ),

        DeclareLaunchArgument(
            'rate',
            default_value='1.0',
        ),

        DeclareLaunchArgument(
            'velocity_mode',
            default_value='w75',
        ),


        DeclareLaunchArgument(
            'model_complexity',
            default_value='0',
        ),


        DeclareLaunchArgument(
            'static_image_mode',
            default_value='false',
        ),

        DeclareLaunchArgument(
            'min_tracking_confidence',
            default_value='0.5',
        ),


        DeclareLaunchArgument(
            'min_detection_confidence',
            default_value='0.4',
        ),


        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'publish_base_alias_tf',
            default_value='true',
        ),

        base_alias_tf,

        tracker,
        kalman,
        estimator,
        compare_visualizer,
        handover_distance,
        handover_observer,
        logger,
        rviz,

        TimerAction(
            period=2.0,
            actions=[
                bag_player
            ],
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()
