#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
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
    arm_bridge_s = LaunchConfiguration('arm_bridge_s')
    hand_backend = LaunchConfiguration('hand_backend')
    rtmw_mode = LaunchConfiguration('rtmw_mode')
    rtmw_body_cues = LaunchConfiguration('rtmw_body_cues')
    gripper_camera = LaunchConfiguration('gripper_camera')


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

    record_video = LaunchConfiguration(
        'record_video'
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
            'arm_bridge_s': ParameterValue(arm_bridge_s, value_type=float),
            'hand_backend': ParameterValue(hand_backend, value_type=str),
            'rtmw_mode': ParameterValue(rtmw_mode, value_type=str),
            'rtmw_body_cues': ParameterValue(rtmw_body_cues, value_type=bool),
            'gripper_camera': ParameterValue(gripper_camera, value_type=bool),
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
        parameters=[{'use_sim_time': True, 'velocity_mode': velocity_mode}],
    )


    end_effector_state = Node(
        package='franka_experiments',
        executable='end_effector_state',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
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

    proximity_estimator = Node(
        package='franka_experiments',
        executable='proximity_estimator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
    )

    grasp = Node(
        package='franka_experiments',
        executable='grasp',
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

    video_recorder = ExecuteProcess(
        condition=IfCondition(record_video),
        cmd=[
            'python3',
            '/ros2_ws/src/franka_experiments/scripts/bag_to_mp4.py',
            '--live',
        ],
        output='screen',
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
            '--topics',

            # Main camera - current naming
            '/camera/camera/color/image_raw',
            '/camera/camera/color/camera_info',
            '/camera/camera/aligned_depth_to_color/image_raw',
            '/camera/camera/aligned_depth_to_color/camera_info',
            '/camera/camera/extrinsics/depth_to_color',

            # Main camera - legacy bag naming
            '/camera/color/image_raw',
            '/camera/color/camera_info',
            '/camera/aligned_depth_to_color/image_raw',
            '/camera/aligned_depth_to_color/camera_info',
            '/camera/extrinsics/depth_to_color',

            # Wrist D405
            '/d405/d405/color/image_raw',
            '/d405/d405/color/camera_info',
            '/d405/d405/aligned_depth_to_color/image_raw',
            '/d405/d405/aligned_depth_to_color/camera_info',
            '/d405/d405/extrinsics/depth_to_color',

            # Robot state
            '/NS_1/franka/joint_states',
            '/NS_1/joint_states',

            # Transforms
            '/tf',
            '/tf_static',
        ],
        output='screen',
    )

    return LaunchDescription([

        # Replay isolated from the lab network: a real robot / camera on the
        # same ROS domain would mix its /tf and images with the bag's.
        # Other terminals: export ROS_DOMAIN_ID=73 to see these topics.
        DeclareLaunchArgument('isolate', default_value='true'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '73', condition=IfCondition(LaunchConfiguration('isolate'))),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1', condition=IfCondition(LaunchConfiguration('isolate'))),
        # numpy's OpenBLAS keeps one spinning thread per core on the tiny matrices of
        # these nodes (kalman_hand alone took ~2 cores): one thread is enough.
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '1'),

        # Available bags:
        # /ros2_ws/rosbags/datasets-001/{arm_complex,arm_repeated,handtracker_poses,handratacker_objecy,handover_rosbag2}
        # /ros2_ws/src/rosbags_external/handover_20260924_154930
        # /ros2_ws/src/rosbags_external/handover_20260929_161100
        DeclareLaunchArgument(
            'bag_path',
            default_value='/ros2_ws/src/rosbags_external/handover_20260924_154930',
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
            default_value='1',
        ),

        # palm rebuilt from the arm when the hand is lost [s], 0 = off
        DeclareLaunchArgument('arm_bridge_s', default_value='1.0'),

        # hand front-end: mediapipe | rtmw
        DeclareLaunchArgument('hand_backend', default_value='rtmw'),
        # RTMW size: lightweight (RTMW-m) | balanced (RTMW-l)
        DeclareLaunchArgument('rtmw_mode', default_value='lightweight'),
        # RTMW: body wrist / forearm cues against ghost hands
        DeclareLaunchArgument('rtmw_body_cues', default_value='false'),
        # D405 on the gripper as second view (used only if its images + TF arrive)
        DeclareLaunchArgument('gripper_camera', default_value='true'),


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
            'record_video',
            default_value='false',
        ),

        DeclareLaunchArgument(
            'publish_base_alias_tf',
            default_value='true',
        ),

        base_alias_tf,

        tracker,
        kalman,
        estimator,
        end_effector_state,
        handover_distance,
        handover_observer,
        proximity_estimator,
        grasp,
        compare_visualizer,
        logger,
        rviz,
        video_recorder,

        TimerAction(
            period=2.0,
            actions=[
                bag_player
            ],
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()
