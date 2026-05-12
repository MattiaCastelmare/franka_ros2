#!/usr/bin/env python3
"""Test launch: velocity pipeline with fake hardware.

Starts the minimal bringup with fake hardware and launches the pentagon
velocity commander to validate the velocity pipeline end-to-end without
needing a physical robot.

Pipeline under test:
    ee_pentagon_velocity_commander
        → /NS_1/qdot_cmd  (resolved by resolve_tracking_topic())
        → rt_velocity_executor_controller
        → fake Franka hardware (franka_bringup fake_hw)

What to expect:
    - franka_bringup starts with use_fake_hardware:=true
    - rt_velocity_executor_controller spawns (takes ~15 s)
    - commander starts after ~20 s with warmup + cosine ramp
    - Pentagon EE trajectory in YZ plane (radius=0.15 m, cycle=15 s)
    - /NS_1/joint_states publishes (fake encoder feedback)

Validation:
    ./test/scripts/check_topics.sh velocity

Topics to monitor:
    /NS_1/joint_states                JointState (fake hardware)
    /NS_1/qdot_cmd                    Float64MultiArray (7) — commander output

Controllers that must be ACTIVE:
    joint_state_broadcaster
    rt_velocity_executor_controller
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from franka_experiments.utils.ros import load_franka_config_defaults, load_launch_defaults

_LAUNCH_DEFAULTS, _ = load_launch_defaults()
_BRINGUP_DEFAULTS, _ = load_franka_config_defaults()
_DEFAULTS = {**_LAUNCH_DEFAULTS, **_BRINGUP_DEFAULTS}


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value=_DEFAULTS.get('namespace', 'NS_1'))
    control_delay_arg = DeclareLaunchArgument(
        'control_spawner_delay_s',
        default_value=_DEFAULTS.get('control_spawner_delay_s', '10.0'))

    namespace      = LaunchConfiguration('namespace')
    control_delay  = LaunchConfiguration('control_spawner_delay_s')

    minimal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('franka_experiments'), 'launch', 'minimal.launch.py',
        ])),
        launch_arguments={
            'use_fake_hardware':       'true',
            'fake_sensor_commands':    'true',
            'namespace':               namespace,
            'control_spawner_delay_s': control_delay,
            'use_torque_controller':   'false',
        }.items(),
    )

    commander = Node(
        package='franka_experiments',
        executable='ee_pentagon_velocity_commander',
        name='ee_pentagon_velocity_commander',
        output='screen',
        parameters=[{
            'rate_hz':    200.0,
            'warmup_s':   2.0,
            'ramp_s':     2.0,
            'qdot_max':   0.3,
            'radius':     0.15,
            'cycle_time': 15.0,
            'plane':      'front',
        }],
    )

    return LaunchDescription([
        namespace_arg,
        control_delay_arg,
        minimal,
        # Commander starts after controllers are up
        TimerAction(
            period=20.0,
            actions=[commander],
        ),
    ])
