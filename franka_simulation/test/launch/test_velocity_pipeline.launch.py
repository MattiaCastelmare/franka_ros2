#!/usr/bin/env python3
"""Test launch: velocity pipeline + automatic test publisher.

Starts the full velocity simulation pipeline and automatically publishes
sinusoidal joint-velocity commands to verify the pipeline end-to-end.

Pipeline under test:
    test_velocity_publisher → /fr3_velocity_controller/commands
                            → fr3_velocity_controller → Gazebo

What to expect:
    - Gazebo opens with FR3 robot
    - RViz opens showing robot state
    - After ~10 s: fr3_joint1 starts oscillating (±0.15 rad/s, 0.1 Hz)
    - After ~40 s: publisher stops, robot holds current position
    - Check with: ./check_pipeline.sh velocity

Topics to monitor:
    /fr3_velocity_controller/commands   Float64MultiArray (7)
    /joint_states                       JointState

Controllers that must be ACTIVE:
    joint_state_broadcaster
    fr3_velocity_controller
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('franka_simulation')

    sim_velocity = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'sim_velocity.launch.py')
        ),
    )

    params_yaml = os.path.join(pkg, 'test', 'config', 'test_publishers.yaml')

    test_publisher = Node(
        package='franka_simulation',
        executable='test_velocity_publisher',
        name='velocity_test_publisher',
        output='screen',
        parameters=[params_yaml],
    )

    return LaunchDescription([
        sim_velocity,
        # Wait for Gazebo + controllers to be fully up before publishing
        TimerAction(period=10.0, actions=[test_publisher]),
    ])
