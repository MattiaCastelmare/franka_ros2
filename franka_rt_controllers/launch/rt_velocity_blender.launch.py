"""Launch: franka bringup + RT velocity blender controller.

This launch file:
  1. Includes franka_bringup/franka.launch.py (loads URDF, spawns hw interface,
     starts joint_state_broadcaster + franka_robot_state_broadcaster).
  2. Spawns ONLY the rt_velocity_blender_controller (NO forward_command_controller).
  3. The RT controller claims fr3_joint{1..7}/velocity — no other velocity
     controller can be active at the same time.

Reads robot defaults from franka_bringup/config/franka.config.yaml (ROBOT1)
so that  `ros2 launch franka_rt_controllers rt_velocity_blender.launch.py`
works out-of-the-box.

IMPORTANT — Before launching, verify that NO other velocity controller is
active on the same joints:
  ros2 control list_controllers
  ros2 control list_hardware_interfaces
If another controller claims fr3_joint*/velocity, deactivate it first:
  ros2 control switch_controllers --deactivate <other_controller_name>
"""

import os

import yaml
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
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# ── Load defaults from franka_bringup config ─────────────────────────
def _load_defaults(robot_key="ROBOT1"):
    defaults = {
        "arm_id": "fr3",
        "robot_ip": "192.168.2.10",
        "use_fake_hardware": "false",
        "namespace": "",
        "fake_sensor_commands": "false",
        "load_gripper": "false",
    }
    config_path = "<not found>"
    try:
        bringup_share = get_package_share_directory("franka_bringup")
        config_path = os.path.join(bringup_share, "config", "franka.config.yaml")
        with open(config_path, "r") as fh:
            config = yaml.safe_load(fh)
        if config and robot_key in config:
            robot = config[robot_key]
            for key in defaults:
                if key in robot:
                    val = robot[key]
                    if isinstance(val, bool):
                        defaults[key] = "true" if val else "false"
                    else:
                        defaults[key] = str(val)
    except Exception as exc:  # noqa: BLE001
        print(
            f"[rt_velocity_blender.launch] WARN: could not read "
            f"{config_path}: {exc} — using hard-coded defaults."
        )
    return defaults, config_path


_DEFAULTS, _CONFIG_PATH = _load_defaults()


# ── OpaqueFunction: resolve everything at runtime ─────────────────────
def _launch_all(context):
    namespace = LaunchConfiguration("namespace").perform(context)
    use_fake = LaunchConfiguration("use_fake_hardware").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    arm_id = LaunchConfiguration("arm_id").perform(context)
    fake_sensor = LaunchConfiguration("fake_sensor_commands").perform(context)
    load_gripper = LaunchConfiguration("load_gripper").perform(context)

    cm_name = (
        ("/" + namespace + "/controller_manager")
        if namespace
        else "/controller_manager"
    )

    # ── Controller YAML from our package ────────────────────────────
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("franka_rt_controllers"), "config", "rt_velocity_blender.yaml"]
    ).perform(context)

    # Allow CLI override
    yaml_override = LaunchConfiguration("controllers_yaml").perform(context)
    if yaml_override != "__auto__":
        controllers_yaml = yaml_override

    # ── Include franka bringup ──────────────────────────────────────
    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("franka_bringup"), "launch", "franka.launch.py"]
            ).perform(context)
        ),
        launch_arguments={
            "arm_id": arm_id,
            "robot_ip": robot_ip,
            "namespace": namespace,
            "use_fake_hardware": use_fake,
            "fake_sensor_commands": fake_sensor,
            "load_gripper": load_gripper,
            "controllers_yaml": controllers_yaml,
        }.items(),
    )

    # ── Spawn the RT velocity blender controller ────────────────────
    rt_blender_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rt_velocity_blender_controller",
            "--controller-manager",
            cm_name,
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
    )

    actions = [
        LogInfo(msg=["[rt_blender] franka.config.yaml : ", _CONFIG_PATH]),
        LogInfo(msg=["[rt_blender] arm_id             : ", arm_id]),
        LogInfo(msg=["[rt_blender] robot_ip           : ", robot_ip]),
        LogInfo(
            msg=[
                "[rt_blender] namespace          : ",
                namespace if namespace else "<none>",
            ]
        ),
        LogInfo(msg=["[rt_blender] use_fake_hardware  : ", use_fake]),
        LogInfo(msg=["[rt_blender] controllers_yaml   : ", controllers_yaml]),
        LogInfo(msg=["[rt_blender] controller-manager : ", cm_name]),
        franka_launch,
        # Delay to let bringup spawners (joint_state_broadcaster etc.) finish
        TimerAction(period=10.0, actions=[rt_blender_spawner]),
    ]

    return actions


# ── generate_launch_description ───────────────────────────────────────
def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument(
            "arm_id",
            default_value=_DEFAULTS["arm_id"],
            description="Robot arm model identifier",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value=_DEFAULTS["robot_ip"],
            description="IP address of the robot",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value=_DEFAULTS["namespace"],
            description="Namespace for the robot (empty = no namespace)",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value=_DEFAULTS["use_fake_hardware"],
            description="Use fake hardware",
        ),
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value=_DEFAULTS["fake_sensor_commands"],
            description="Fake sensor commands",
        ),
        DeclareLaunchArgument(
            "load_gripper",
            default_value=_DEFAULTS["load_gripper"],
            description="Load Franka Gripper",
        ),
        DeclareLaunchArgument(
            "controllers_yaml",
            default_value="__auto__",
            description="Override controllers YAML path (__auto__ = use package default)",
        ),
    ]

    return LaunchDescription(declared_args + [OpaqueFunction(function=_launch_all)])
