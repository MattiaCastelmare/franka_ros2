#!/usr/bin/env python3

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_rsp(context: LaunchContext, arm_id, load_gripper, franka_hand):
    """Crea robot_description da Xacro e avvia il robot_state_publisher."""
    arm_id_str = context.perform_substitution(arm_id)
    load_gripper_str = context.perform_substitution(load_gripper)
    franka_hand_str = context.perform_substitution(franka_hand)

    xacro_file = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        arm_id_str,
        f"{arm_id_str}.urdf.xacro",
    )

    urdf_xml = xacro.process_file(
        xacro_file,
        mappings={
            "arm_id": arm_id_str,
            "hand": load_gripper_str,
            "ros2_control": "true",
            "gazebo": "true",
            "ee_id": franka_hand_str,
        },
    ).toxml()

    robot_description = {"robot_description": urdf_xml}

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    return [rsp]


def generate_launch_description():
    # Argomenti
    load_gripper_arg = DeclareLaunchArgument("load_gripper", default_value="true")
    franka_hand_arg = DeclareLaunchArgument("franka_hand", default_value="franka_hand")
    arm_id_arg = DeclareLaunchArgument("arm_id", default_value="fr3")

    load_gripper = LaunchConfiguration("load_gripper")
    franka_hand = LaunchConfiguration("franka_hand")
    arm_id = LaunchConfiguration("arm_id")

    # RSP (pubblica /robot_description)
    rsp = OpaqueFunction(function=make_rsp, args=[arm_id, load_gripper, franka_hand])

    # Gazebo (Ignition/GZ) mondo vuoto
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.path.dirname(
        get_package_share_directory("franka_description")
    )
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "empty.sdf -r"}.items(),
    )

    # Spawn del robot in Gazebo dal topic /robot_description
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "/robot_description"],
        output="screen",
    )

    # Spawner dei controller (parlano a /controller_manager dentro Gazebo)
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_joint_state_broadcaster",
        arguments=[
            "joint_state_broadcaster", 
            "--controller-manager", "/controller_manager",
            "--param-file",
            os.path.join(get_package_share_directory("franka_simulation"), "config", "controllers.yaml")
        ],
        output="screen",
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawn_fr3_arm_controller",
        arguments=[
            "fr3_arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            os.path.join(get_package_share_directory("franka_simulation"), "config", "fr3_arm_controller.yaml"),
        ],
        output="screen",
    )

    # RViz (fixed frame = fr3_link0)
    rviz_file = os.path.join(
        get_package_share_directory("franka_description"), "rviz", "visualize_franka.rviz"
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["--display-config", rviz_file, "-f", "fr3_link0"],
        output="screen",
    )

    # Sequenza: spawn -> JSB -> ARM -> RViz
    delayed_spawn = TimerAction(period=3.0, actions=[spawn])
    delayed_jsb = TimerAction(period=5.0, actions=[jsb_spawner])
    delayed_arm = TimerAction(period=7.0, actions=[arm_spawner])
    delayed_rviz = TimerAction(period=8.5, actions=[rviz])

    return LaunchDescription(
        [
            load_gripper_arg,
            franka_hand_arg,
            arm_id_arg,
            gz,
            rsp,
            delayed_spawn,
            delayed_jsb,
            delayed_arm,
            delayed_rviz,
        ]
    )
