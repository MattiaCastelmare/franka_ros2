"""Bringup: velocity controller + AprilTag detection + hand-eye calibration.

Launches the full hand-eye calibration pipeline in a single command::

    ros2 launch franka_experiments handeye_calibration_bringup.launch.py

This replaces the manual three-terminal workflow:

1. ``ros2 launch franka_experiments minimal.launch.py use_torque_controller:=false command_topic:=tracking_qdot``
2. ``ros2 run apriltag_ros apriltag_node --ros-args …``
3. ``ros2 run franka_experiments handeye_calibration_node``

Root-cause fix (2026-05-11)
---------------------------
The previous version included ``cbf_velocity.launch.py``, which
unconditionally starts ``pentagon_qddot_commander`` and
``cbf_safety_filter``.  These fight the calibration node's waypoint
controller, and ``cbf_safety_filter`` publishes torques [N·m] on
``/NS_1/qdot_cmd`` — which ``rt_velocity_executor_controller`` interprets as
rad/s, triggering ``joint_velocity_violation`` within seconds.

Fix: include ``minimal.launch.py`` (robot driver + bare velocity controller,
no CBF, no trajectory commander) with ``command_topic=tracking_qdot`` so
the controller subscribes to the same topic that ``handeye_calibration_node``
publishes on via ``resolve_tracking_topic()``.

Startup sequence
----------------
  t=0 s    — franka bringup (robot driver + joint_state_broadcaster)
  t=1 s    — world→fr3_link0 static TF + camera extrinsics TF
  t=0 s    — RealSense camera driver (camera_delay_s=0 default)
  t=10 s   — rt_velocity_executor_controller spawner
  t≈12 s   — controller becomes active
  t=2 s    — RViz (early, cosmetic — will show errors until TFs are ready)
  t=calibration_delay (default 15 s) — handeye_calibration_node starts

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

    # ── Launch arguments ──────────────────────────────────────────────────────
    apriltag_family_arg = DeclareLaunchArgument(
        'apriltag_family', default_value='36h11',
        description='AprilTag family (e.g. 36h11, 25h9).')

    apriltag_size_arg = DeclareLaunchArgument(
        'apriltag_size', default_value='0.10',
        description='Physical size of the AprilTag in metres.')

    calibration_delay_arg = DeclareLaunchArgument(
        'calibration_delay', default_value='15.0',
        description='Seconds to wait before starting the calibration node. '
                    'Must be > control_spawner_delay_s (~10 s) + controller '
                    'activation time (~2 s). Default 15 s is safe.')

    # ── 1) Minimal bringup: robot driver + bare velocity controller ───────────
    #   command_topic=tracking_qdot  → controller subscribes to /NS_1/tracking_qdot,
    #                                   the same topic handeye_calibration_node publishes.
    #   use_torque_controller=false  → rt_velocity_executor_controller (no CBF, no torques).
    #   enable_camera=true           → RealSense driver for AprilTag detection.
    #   start_real_time_distance=false → no distance estimation needed.
    #   start_experiment_logger=false  → no data logging during calibration.
    minimal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('franka_experiments'),
                'launch',
                'minimal.launch.py',
            ])
        ),
        launch_arguments={
            'command_topic':            'tracking_qdot',
            'use_torque_controller':    'false',
            'max_accel':                '0.5',
            'enable_camera':            'true',
            'start_real_time_distance': 'false',
            'start_experiment_logger':  'false',
        }.items(),
    )

    # ── 2) AprilTag detection node ────────────────────────────────────────────
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

    # ── 3) RViz (early start — cosmetic; TF errors resolve once controller is up) ──
    rviz_config = PathJoinSubstitution([
        FindPackageShare('franka_experiments'),
        'config', 'handeye_rviz.rviz',
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='log',
    )
    delayed_rviz = TimerAction(period=2.0, actions=[rviz_node])

    # ── 4) Hand-eye calibration node (delayed until controller is active) ─────
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

    # ── 5) Shutdown everything when calibration node exits ────────────────────
    shutdown_on_calibration_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=handeye_node,
            on_exit=[EmitEvent(event=Shutdown(
                reason='handeye_calibration_node finished — '
                       'shutting down calibration pipeline.'))],
        )
    )

    # ── Compose ───────────────────────────────────────────────────────────────
    return LaunchDescription([
        apriltag_family_arg,
        apriltag_size_arg,
        calibration_delay_arg,
        minimal_launch,
        apriltag_node,
        delayed_rviz,
        delayed_handeye,
        shutdown_on_calibration_exit,
    ])
