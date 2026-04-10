"""Complete launch for the human-robot avoidance framework.

Includes the full robot bringup via ``wrapper_forward_velocity.launch.py``
(Franka hardware, RT velocity-blender controller, camera pipeline,
human-pose estimator), then launches the avoidance-specific nodes with
configurable delays:

1. **human_distance_estimator** — back-projects 2-D pose landmarks into
   3-D via depth, computes min human–robot distance, publishes
   ``HumanRobotDistance``.
2. **human_avoidance_controller** — reads the distance estimate and
   generates CBF-QP repulsive joint velocities.

All arguments of the wrapper are forwarded transparently:

.. code-block:: bash

    ros2 launch franka_experiments human_avoidance.launch.py
    ros2 launch franka_experiments human_avoidance.launch.py use_fake_hardware:=true
    ros2 launch franka_experiments human_avoidance.launch.py estimator_delay_s:=10 avoidance_delay_s:=13
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node as LaunchNode
from launch_ros.substitutions import FindPackageShare

from franka_experiments.utils.ros import (
    declare_robot_args,
    declare_rt_blender_args,
    load_franka_config_defaults,
    load_launch_defaults,
)

# ── Load defaults (same source of truth as the wrapper) ───────────────
_LAUNCH_DEFAULTS, _ = load_launch_defaults()
_BRINGUP_DEFAULTS, _ = load_franka_config_defaults()
_DEFAULTS = {**_LAUNCH_DEFAULTS, **_BRINGUP_DEFAULTS}

# Arguments forwarded verbatim to wrapper_forward_velocity.launch.py
_WRAPPER_ARGS = [
    "namespace", "use_fake_hardware", "robot_ip", "arm_id",
    "fake_sensor_commands", "load_gripper", "controllers_yaml",
    "qdot_max", "alpha", "max_accel", "timeout_threshold_s",
    "timeout_ramp_s", "gazebo", "enable_interpolation",
    "alpha_topic", "tracking_topic", "avoidance_topic",
    "enable_camera", "control_spawner_delay_s",
    "start_rviz", "rviz_delay_s",
    "camera_delay_s", "start_human_pose", "human_pose_delay_s",
]


def _launch_all(context):
    """Resolve parameters, include the wrapper, schedule avoidance nodes."""
    pkg_share = get_package_share_directory("franka_experiments")

    # ── Resolve delay parameters ──────────────────────────────────────
    estimator_delay = float(
        LaunchConfiguration("estimator_delay_s").perform(context))
    avoidance_delay = float(
        LaunchConfiguration("avoidance_delay_s").perform(context))
    namespace = LaunchConfiguration("namespace").perform(context)

    # ── Include the full bringup wrapper ──────────────────────────────
    wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("franka_experiments"),
                "launch", "wrapper_forward_velocity.launch.py",
            ]).perform(context),
        ),
        launch_arguments={
            k: LaunchConfiguration(k).perform(context)
            for k in _WRAPPER_ARGS
        }.items(),
    )

    # ── Config files for the two avoidance nodes ──────────────────────
    estimator_config = os.path.join(
        pkg_share, "config", "human_distance_estimator_params.yaml")
    avoidance_config = os.path.join(
        pkg_share, "config", "human_avoidance_params.yaml")

    # ── Distance estimator (producer) ─────────────────────────────────
    estimator_node = LaunchNode(
        package="franka_experiments",
        executable="human_distance_estimator",
        name="human_distance_estimator",
        output="screen",
        parameters=[estimator_config],
    )

    # ── Avoidance controller (consumer) ──────────────────────────────
    avoidance_node = LaunchNode(
        package="franka_experiments",
        executable="human_avoidance_controller",
        name="human_avoidance_controller",
        namespace=namespace,
        output="screen",
        parameters=[avoidance_config],
    )

    # ── Capsule overlay (debug visualisation) ───────────────────
    overlay_node = LaunchNode(
        package="franka_experiments",
        executable="capsule_overlay_node",
        name="capsule_overlay_node",
        output="screen",
        parameters=[estimator_config],
    )

    # ── Assemble: wrapper first, then delayed avoidance nodes ───────
    return [
        wrapper_launch,
        LogInfo(msg=["[human_avoidance] Estimator   scheduled in ",
                     str(estimator_delay), " s"]),
        LogInfo(msg=["[human_avoidance] Controller  scheduled in ",
                     str(avoidance_delay), " s"]),
        TimerAction(period=estimator_delay, actions=[estimator_node]),
        TimerAction(period=estimator_delay, actions=[overlay_node]),
        TimerAction(period=avoidance_delay, actions=[avoidance_node]),
    ]


def generate_launch_description():
    return LaunchDescription(
        # ── Robot & RT-blender args (forwarded to the wrapper) ────────
        declare_robot_args(_DEFAULTS)
        + declare_rt_blender_args(_DEFAULTS)
        + [
            # ── Wrapper scheduling / UI args ──────────────────────────
            DeclareLaunchArgument(
                "enable_camera",
                default_value=str(_DEFAULTS.get("enable_camera", "true")),
                description="Enable RealSense driver + image republisher"),
            DeclareLaunchArgument(
                "control_spawner_delay_s",
                default_value=str(
                    _DEFAULTS.get("control_spawner_delay_s", "10.0")),
                description="Seconds before spawning the RT controller"),
            DeclareLaunchArgument(
                "start_rviz",
                default_value=str(_DEFAULTS.get("start_rviz", "true")),
                description="Launch RViz2"),
            DeclareLaunchArgument(
                "rviz_delay_s",
                default_value=str(_DEFAULTS.get("rviz_delay_s", "5.0")),
                description="Seconds before launching RViz2"),
            DeclareLaunchArgument(
                "camera_delay_s",
                default_value=str(_DEFAULTS.get("camera_delay_s", "0.0")),
                description="Seconds before launching camera pipeline"),
            DeclareLaunchArgument(
                "start_human_pose",
                default_value=str(
                    _DEFAULTS.get("start_human_pose", "true")),
                description="Launch human_pose_node"),
            DeclareLaunchArgument(
                "human_pose_delay_s",
                default_value=str(
                    _DEFAULTS.get("human_pose_delay_s", "0.0")),
                description="Extra seconds before human_pose_node"),
            # ── Avoidance-specific timing ─────────────────────────────
            DeclareLaunchArgument(
                "estimator_delay_s",
                default_value="12.0",
                description="Seconds before launching "
                            "human_distance_estimator "
                            "(needs robot_state_publisher + camera + "
                            "human_pose_node to be up)"),
            DeclareLaunchArgument(
                "avoidance_delay_s",
                default_value="15.0",
                description="Seconds before launching "
                            "human_avoidance_controller "
                            "(needs estimator to be publishing)"),
            # ── Main logic ────────────────────────────────────────────
            OpaqueFunction(function=_launch_all),
        ]
    )
