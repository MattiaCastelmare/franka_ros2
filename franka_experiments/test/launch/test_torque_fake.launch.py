#!/usr/bin/env python3
"""Test launch: torque pipeline with fake hardware.

Starts the minimal bringup with fake hardware and launches the pentagon
torque commander to validate the torque pipeline end-to-end.

Pipeline under test:
    pentagon_torque_commander
        → /NS_1/torque_cmd  (τ_nom without gravity)
        → rt_torque_controller
            ├─ optional LPF + per-joint clipping
            └─ applies effort to fake Franka hardware

IMPORTANT — gravity compensation:
    pentagon_torque_commander sends τ = M(q)·q̈ + C(q,q̇)·q̇ (NO gravity).
    rt_torque_controller does NOT add g(q): on real hardware the Franka
    firmware compensates gravity internally (adding it again would double-
    compensate). The same applies in fake mode, where the controller path
    is identical.

What to expect:
    - franka_bringup starts with use_fake_hardware:=true
    - rt_torque_controller spawns and listens on /NS_1/torque_cmd
    - Commander starts after ~20 s with warmup + ramp
    - Pentagon trajectory at ±0.3 m radius, 10 s cycle
    - /NS_1/torque_cmd publishes at 100 Hz

Validation:
    ./test/scripts/check_topics.sh torque

Topics to monitor:
    /NS_1/joint_states    JointState (fake feedback)
    /NS_1/torque_cmd      Float64MultiArray (7) — τ_nom (no gravity)

Controllers that must be ACTIVE:
    joint_state_broadcaster
    rt_torque_controller
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

    namespace     = LaunchConfiguration('namespace')
    control_delay = LaunchConfiguration('control_spawner_delay_s')

    minimal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('franka_experiments'), 'launch', 'minimal.launch.py',
        ])),
        launch_arguments={
            'use_fake_hardware':       'true',
            'fake_sensor_commands':    'true',
            'namespace':               namespace,
            'control_spawner_delay_s': control_delay,
            'use_torque_controller':   'true',
            'torque_command_topic':    'torque_cmd',
        }.items(),
    )

    commander = Node(
        package='franka_experiments',
        executable='pentagon_torque_commander',
        name='pentagon_torque_commander',
        output='screen',
        parameters=[{
            'cbf_mode':          False,
            'torque_topic':      '/NS_1/torque_cmd',
            'joint_state_topic': '/NS_1/joint_states',
            'ee_frame':          'fr3_hand_tcp',
            'rate_hz':           100.0,
            'warmup_s':          3.0,
            'ramp_s':            3.0,
            'radius':            0.3,
            'kp_cart':           80.0,
            'kd_cart':           15.0,
            'kp_rot':            20.0,
            'kd_rot':            5.0,
        }],
    )

    return LaunchDescription([
        namespace_arg,
        control_delay_arg,
        minimal,
        TimerAction(period=20.0, actions=[commander]),
    ])
