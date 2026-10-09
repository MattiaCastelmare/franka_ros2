#!/usr/bin/env python3
"""robot_handover.launch.py + rosbag of both cameras, started together."""

import datetime
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

_CAMERA_TOPICS = [
    'color/image_raw',
    'color/camera_info',
    'aligned_depth_to_color/image_raw',
    'aligned_depth_to_color/camera_info',
    'extrinsics/depth_to_color',
]

ROSBAG_TOPICS = [
    f'/{cam}/{cam}/{topic}'
    for cam in ('camera', 'd405')
    for topic in _CAMERA_TOPICS
]

ROSBAG_TOPICS += [
    # Robot state
    '/NS_1/franka/joint_states',
    '/NS_1/joint_states',
    '/tf',
    '/tf_static',

    # Hand perception/state
    '/handover/hand_tracking_raw',
    '/handover/hand_tracking_filtered',
    '/handover/hand_state',

    # Robot-relative handover state
    '/handover/end_effector_state',
    '/handover/distance',
    '/handover/observer',
    '/handover/hand_object',

    # Control
    '/NS_1/qddot_nom',
    '/NS_1/torque_cmd',
]


def generate_launch_description():
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.join(
        os.path.expanduser('~'),
        'ros2_bags',
        f'handover_{stamp}',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('franka_experiments'),
                'launch',
                'robot_handover.launch.py',
            ]))
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '--output', bag_dir] + ROSBAG_TOPICS,
            output='screen',
            name='rosbag_record',
        ),
    ])
