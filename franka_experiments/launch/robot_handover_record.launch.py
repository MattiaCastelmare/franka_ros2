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


def generate_launch_description():
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.join(os.path.expanduser('~'), 'ros2_bags', f'handover_{stamp}')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('franka_experiments'),
                'launch', 'robot_handover.launch.py',
            ]))
        ),
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '--output', bag_dir] + ROSBAG_TOPICS,
            output='screen',
            name='rosbag_record',
        ),
    ])
