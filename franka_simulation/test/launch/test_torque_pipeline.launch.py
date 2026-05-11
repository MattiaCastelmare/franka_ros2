#!/usr/bin/env python3
"""Test launch: torque pipeline + automatic test publisher.

Starts the full torque simulation pipeline and automatically publishes
torque commands to verify the pipeline end-to-end.

Pipeline under test:
    test_torque_publisher → /fr3_effort_controller/commands
                          → fr3_effort_controller → Gazebo (effort interface)

IMPORTANT — gravity semantics in Gazebo:
    Gazebo simulates gravity via physics. The fr3_effort_controller adds torques
    ON TOP of what the physics engine already does.
    Publishing zeros → robot stays at current pose (Gazebo equilibrium).
    Do NOT add gravity compensation (g(q)) to commanded torques.
    This contrasts with real hardware where rt_torque_controller adds g(q).

What to expect:
    Phase 0 (0–20 s after controller up):
        publisher not started yet — robot holds default pose
    Phase 1 (20–30 s):
        test publisher starts, publishes zeros for 10 s
        robot should stay still (Gazebo equilibrium)
    Phase 2 (30–50 s):
        ±2 Nm sinusoidal torque on joint 1 → joint1 oscillates
    Phase 3 (50+ s):
        zeros published again → robot stops

Prerequisites:
    sim_torque.launch.py uses gazebo_effort:=true in the xacro.
    The effort command interface must be listed by:
        ros2 control list_hardware_interfaces

Topics to monitor:
    /fr3_effort_controller/commands    Float64MultiArray (7)
    /joint_states                      JointState

Controllers that must be ACTIVE:
    joint_state_broadcaster
    fr3_effort_controller
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('franka_simulation')

    sim_torque = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'sim_torque.launch.py')
        ),
    )

    params_yaml = os.path.join(pkg, 'test', 'config', 'test_publishers.yaml')

    test_publisher = Node(
        package='franka_simulation',
        executable='test_torque_publisher',
        name='torque_test_publisher',
        output='screen',
        parameters=[params_yaml],
    )

    return LaunchDescription([
        sim_torque,
        # Extra delay — torque pipeline needs effort interface to be registered
        TimerAction(period=12.0, actions=[test_publisher]),
    ])
