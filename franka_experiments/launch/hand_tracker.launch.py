#!/usr/bin/env python3
"""Replay of a bag through the handover perception pipeline (no robot).

  bag -> human_hand_tracker -> kalman_hand -> hand_state_estimator -> distance_handover_estimator
         grasp (Hands23 object), hand_visualizer (debug images), hand_logger (CSV in results/)
         optional: grasp pose (start_grasp_pose), rviz (start_rviz), debug video (record_video)
Node parameters: config/hand_tracking.yaml; the arguments below override the most used ones.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

SCRIPTS = '/ros2_ws/src/franka_experiments/scripts/'
BAG_TOPICS = [
    # main camera (current and legacy naming), wrist D405
    *[f'{ns}/{topic}' for ns in ('/camera/camera', '/camera', '/d405/d405') for topic in (
        'color/image_raw', 'color/camera_info', 'aligned_depth_to_color/image_raw',
        'aligned_depth_to_color/camera_info', 'extrinsics/depth_to_color')],
    '/NS_1/franka/joint_states', '/NS_1/joint_states', '/tf', '/tf_static',
]


def generate_launch_description():
    arg = LaunchConfiguration
    hand_tracking_yaml = PathJoinSubstitution(
        [FindPackageShare('franka_experiments'), 'config', 'hand_tracking.yaml'])

    def pipeline_node(executable, overrides=None, yaml=True, **kwargs):
        return Node(package='franka_experiments', executable=executable, output='screen',
                    parameters=(([hand_tracking_yaml] if yaml else [])
                                + [{'use_sim_time': True, **(overrides or {})}]),
                    **kwargs)

    return LaunchDescription([
        # Replay isolated from the lab network: a real robot / camera on the same ROS domain would mix
        # its /tf and images with the bag's. Other terminals: export ROS_DOMAIN_ID=73 to see these topics.
        DeclareLaunchArgument('isolate', default_value='true'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '73', condition=IfCondition(arg('isolate'))),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1', condition=IfCondition(arg('isolate'))),
        # numpy's OpenBLAS keeps one spinning thread per core on the tiny matrices of
        # these nodes (kalman_hand alone took ~2 cores): one thread is enough.
        SetEnvironmentVariable('OPENBLAS_NUM_THREADS', '1'),

        # Available bags:
        # /ros2_ws/rosbags/datasets-001/{arm_complex,arm_repeated,handtracker_poses,handratacker_objecy,handover_rosbag2}
        # /ros2_ws/src/rosbags_external/handover_20260924_154930
        # /ros2_ws/src/rosbags_external/handover_20260929_161100
        DeclareLaunchArgument('bag_path', default_value='/ros2_ws/src/rosbags_external/handover_20260924_154930'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        # palm rebuilt from the arm when the hand is lost [s], 0 = off
        DeclareLaunchArgument('arm_bridge_s', default_value='1.0'),
        # hand detector: rtmw | mediapipe
        DeclareLaunchArgument('hand_detector', default_value='rtmw'),
        # RTMW size: lightweight (RTMW-m) | balanced (RTMW-l)
        DeclareLaunchArgument('rtmw_mode', default_value='lightweight'),
        # D405 on the gripper as second view (used only if its images + TF arrive)
        DeclareLaunchArgument('gripper_camera', default_value='true'),
        DeclareLaunchArgument('start_grasp_pose', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('record_video', default_value='false'),
        DeclareLaunchArgument('publish_base_alias_tf', default_value='true'),

        # perception pipeline
        Node(package='tf2_ros', executable='static_transform_publisher', name='fr3_link0_to_hand_base_tf',
             output='log', condition=IfCondition(arg('publish_base_alias_tf')),
             arguments=['--x', '0', '--y', '0', '--z', '0', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                        '--frame-id', 'fr3_link0', '--child-frame-id', 'base']),
        pipeline_node('human_hand_tracker', {
            'arm_bridge_s': ParameterValue(arg('arm_bridge_s'), value_type=float),
            'hand_detector': ParameterValue(arg('hand_detector'), value_type=str),
            'rtmw_mode': ParameterValue(arg('rtmw_mode'), value_type=str),
            'gripper_camera': ParameterValue(arg('gripper_camera'), value_type=bool)}),
        pipeline_node('kalman_hand'),
        pipeline_node('hand_state_estimator'),
        pipeline_node('distance_handover_estimator', name='distance_handover_estimator'),
        pipeline_node('grasp', yaml=False),

        # grasp pose -> /handover/grasp_pose (GSNet, or AnyGrasp with backend:=anygrasp)
        ExecuteProcess(cmd=['python3', SCRIPTS + 'handover_grasp.py', 'pose',
                            '--ros-args', '-p', 'use_sim_time:=true'],
                       output='screen', condition=IfCondition(arg('start_grasp_pose'))),

        # visualisation and logging
        pipeline_node('hand_visualizer', yaml=False),
        pipeline_node('hand_logger'),
        Node(package='rviz2', executable='rviz2', output='screen', condition=IfCondition(arg('start_rviz')),
             arguments=['-d', PathJoinSubstitution(
                 [FindPackageShare('franka_experiments'), 'config', 'hand_tracker.rviz'])],
             parameters=[{'use_sim_time': True}]),
        ExecuteProcess(cmd=['python3', SCRIPTS + 'bag_to_mp4.py', '--live'], output='screen',
                       condition=IfCondition(arg('record_video'))),

        # bag replay, after the nodes are up
        TimerAction(period=2.0, actions=[ExecuteProcess(
            cmd=['ros2', 'bag', 'play', arg('bag_path'), '--clock', '--rate', arg('rate'),
                 '--topics', *BAG_TOPICS],
            output='screen')]),
    ])
