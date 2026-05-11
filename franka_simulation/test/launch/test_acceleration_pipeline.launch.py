#!/usr/bin/env python3
"""Test launch: acceleration pipeline + automatic test publisher.

Starts the full acceleration simulation pipeline and automatically publishes
sinusoidal joint-acceleration commands to verify the pipeline end-to-end.

Pipeline under test:
    test_acceleration_publisher → /sim_acceleration_bridge/accel_cmd
                                → sim_acceleration_bridge (q̈→q̇ integrator)
                                → /fr3_velocity_controller/commands
                                → fr3_velocity_controller → Gazebo

What to expect:
    - Gazebo opens with FR3 robot
    - RViz opens showing robot state
    - After ~10 s: fr3_joint1 starts accelerating/decelerating sinusoidally
    - The bridge integrates: q̇(t+1) = clamp(q̇(t) + q̈·dt, -qdot_max, +qdot_max)
    - Joint velocity ramps up, reaches clamping limit, then decelerates
    - After ~40 s: publisher stops

Topics to monitor:
    /sim_acceleration_bridge/accel_cmd    Float64MultiArray (7)  — input to bridge
    /fr3_velocity_controller/commands     Float64MultiArray (7)  — bridge output
    /joint_states                         JointState

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

    sim_acceleration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'sim_acceleration.launch.py')
        ),
    )

    params_yaml = os.path.join(pkg, 'test', 'config', 'test_publishers.yaml')

    test_publisher = Node(
        package='franka_simulation',
        executable='test_acceleration_publisher',
        name='acceleration_test_publisher',
        output='screen',
        parameters=[params_yaml],
    )

    return LaunchDescription([
        sim_acceleration,
        TimerAction(period=10.0, actions=[test_publisher]),
    ])
