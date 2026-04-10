"""Bringup: velocity controller + AprilTag detection + hand-eye calibration.

Launches the full hand-eye calibration pipeline in a single command::

    ros2 launch franka_experiments handeye_calibration_bringup.launch.py

This replaces the manual three-terminal workflow:

1. ``ros2 launch franka_experiments wrapper_forward_velocity.launch.py``
2. ``ros2 run apriltag_ros apriltag_node --ros-args …``
3. ``ros2 run franka_experiments handeye_calibration_node``

The ``handeye_calibration_node`` is delayed by a configurable number of
seconds (default 3 s) so that TF, the robot driver, and the AprilTag
detector are ready before calibration acquisition begins.

Automatic shutdown
------------------
When ``handeye_calibration_node`` exits (calibration finished or aborted),
the launch system emits a ``Shutdown`` event via ``OnProcessExit``, which
cleanly terminates ``apriltag_node`` and every other process in the launch
tree.  No manual killing required.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    # ── Launch arguments ──────────────────────────────────────────────
    apriltag_family_arg = DeclareLaunchArgument(
        'apriltag_family', default_value='36h11',
        description='AprilTag family (e.g. 36h11, 25h9).')

    apriltag_size_arg = DeclareLaunchArgument(
        'apriltag_size', default_value='0.10',
        description='Physical size of the AprilTag in metres.')

    calibration_delay_arg = DeclareLaunchArgument(
        'calibration_delay', default_value='3.0',
        description='Seconds to wait before starting the calibration node.')

    # ── 1) Include wrapper_forward_velocity.launch.py ─────────────────
    wrapper_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('franka_experiments'),
                    'launch',
                    'wrapper_forward_velocity.launch.py',
                ])
            ),
            launch_arguments={
                'start_real_time_distance': 'false',
                'max_accel': '0.5',
            }.items(),
        )

    # ── 2) AprilTag detection node ────────────────────────────────────
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        remappings=[
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info'),
        ],
        parameters=[{
            'family': LaunchConfiguration('apriltag_family'),
            'size': LaunchConfiguration('apriltag_size'),
            'image_transport': 'raw',
        }],
        output='screen',
    )

    # ── 3) Hand-eye calibration node (delayed) ───────────────────────
    handeye_node = Node(
        package='franka_experiments',
        executable='handeye_calibration_node',
        name='handeye_calibration_node',
        output='screen',
    )

    delayed_handeye = TimerAction(
        period=LaunchConfiguration('calibration_delay'),
        actions=[handeye_node],
    )

    # ── 4) Shutdown everything when calibration node exits ────────────
    #    OnProcessExit fires when handeye_calibration_node terminates
    #    (success or failure).  EmitEvent(Shutdown()) tells the launch
    #    system to SIGINT all remaining processes (apriltag_node, robot
    #    driver, etc.) and then exit cleanly.
    shutdown_on_calibration_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=handeye_node,
            on_exit=[EmitEvent(event=Shutdown(
                reason='handeye_calibration_node finished — '
                       'shutting down calibration pipeline.'))],
        )
    )

    # ── Compose ───────────────────────────────────────────────────────
    return LaunchDescription([
        apriltag_family_arg,
        apriltag_size_arg,
        calibration_delay_arg,
        wrapper_launch,
        apriltag_node,
        delayed_handeye,
        shutdown_on_calibration_exit,
    ])
