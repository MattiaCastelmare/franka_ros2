#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from franka_experiments.utils.config import (
    load_franka_config_defaults,
)


_BRINGUP_DEFAULTS, _ = load_franka_config_defaults()


def generate_launch_description():

    robot_ip = LaunchConfiguration('robot_ip')
    arm_id = LaunchConfiguration('arm_id')
    namespace = LaunchConfiguration('namespace')
    load_gripper = LaunchConfiguration('load_gripper')

    start_rviz = LaunchConfiguration('start_rviz')
    start_logger = LaunchConfiguration('start_logger')


    # ==============================================================
    # 1. REAL FR3
    # ==============================================================
    #
    # Driver + robot state / joint state broadcasters.
    #
    # IMPORTANT:
    # questo launch NON avvia controller di velocita/coppia
    # e NON avvia alcun commander di movimento.
    #

    franka = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('franka_bringup'),
                'launch',
                'franka.launch.py',
            ])
        ),
        launch_arguments={
            'arm_id': arm_id,
            'robot_ip': robot_ip,
            'namespace': namespace,

            'use_fake_hardware': 'false',
            'fake_sensor_commands': 'false',

            'load_gripper': load_gripper,
        }.items(),
    )


    # ==============================================================
    # 2. REALSENSE LIVE
    # ==============================================================
    #
    # Produce:
    #
    # /camera/camera/color/image_raw
    # /camera/camera/aligned_depth_to_color/image_raw
    # /camera/camera/aligned_depth_to_color/camera_info
    #

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py',
            ])
        ),
        launch_arguments={
            'align_depth.enable': 'true',
        }.items(),
    )


    # ==============================================================
    # 3. BASE FRAME ALIAS
    # ==============================================================
    #
    # La tua hand pipeline usa:
    #
    #     base
    #
    # mentre fr3_complete.yaml usa:
    #
    #     fr3_link0
    #
    # Nel framework attuale sono lo stesso frame fisico.
    #

    base_alias_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fr3_link0_to_hand_base_tf',
        output='log',

        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',

            '--qx', '0',
            '--qy', '0',
            '--qz', '0',
            '--qw', '1',

            '--frame-id', 'fr3_link0',
            '--child-frame-id', 'base',
        ],
    )


    # ==============================================================
    # 4. HUMAN HAND TRACKER
    # ==============================================================

    tracker = Node(
        package='franka_experiments',
        executable='human_hand_tracker',
        output='screen',

        parameters=[{
            'use_sim_time': False,

            'publish_debug_image': True,
            'show_selected_landmarks': True,

            'model_complexity': 0,
            'static_image_mode': False,

            'min_tracking_confidence': 0.5,
            'min_detection_confidence': 0.4,
        }],
    )


    # ==============================================================
    # 5. LANDMARK KALMAN
    # ==============================================================

    kalman = Node(
        package='franka_experiments',
        executable='kalman_hand',
        output='screen',

        parameters=[{
            'use_sim_time': False,
        }],
    )


    # ==============================================================
    # 6. HAND STATE ESTIMATOR
    # ==============================================================

    estimator = Node(
        package='franka_experiments',
        executable='hand_state_estimator',
        output='screen',

        parameters=[{
            'use_sim_time': False,
            'velocity_mode': 'w75',
        }],
    )


    # ==============================================================
    # 7. HAND <-> ROBOT RELATIVE KINEMATICS
    # ==============================================================

    handover_distance = Node(
        package='franka_experiments',
        executable='distance_handover_estimator',
        name='distance_handover_estimator',
        output='screen',

        parameters=[{
            'use_sim_time': False,
        }],
    )


    # ==============================================================
    # 8. HANDOVER OBSERVER
    # ==============================================================

    handover_observer = Node(
        package='franka_experiments',
        executable='handover_observer',
        output='screen',

        parameters=[{
            'use_sim_time': False,
        }],
    )


    # ==============================================================
    # 9. VISUAL DEBUG
    # ==============================================================
    #
    # Lo avviamo soltanto se parte RViz.
    #

    compare_visualizer = Node(
        package='franka_experiments',
        executable='hand_compare_visualizer',
        output='screen',

        condition=IfCondition(start_rviz),

        parameters=[{
            'use_sim_time': False,
        }],
    )


    # ==============================================================
    # 10. LOGGER
    # ==============================================================

    logger = Node(
        package='franka_experiments',
        executable='hand_logger',
        output='screen',

        condition=IfCondition(start_logger),

        parameters=[{
            'use_sim_time': False,
        }],
    )


    # ==============================================================
    # 11. RVIZ
    # ==============================================================

    rviz_config = PathJoinSubstitution([
        FindPackageShare('franka_experiments'),
        'config',
        'hand_tracker.rviz',
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',

        condition=IfCondition(start_rviz),

        arguments=[
            '-d',
            rviz_config,
        ],

        parameters=[{
            'use_sim_time': False,
        }],

        additional_env={
            '__NV_PRIME_RENDER_OFFLOAD': '1',
            '__GLX_VENDOR_LIBRARY_NAME': 'nvidia',
        },
    )


    # ==============================================================
    # 12. PIPELINE DELAY
    # ==============================================================
    #
    # Diamo 2 secondi a FR3 + RealSense per iniziare il bringup.
    #

    pipeline = TimerAction(
        period=2.0,

        actions=[
            tracker,
            kalman,
            estimator,

            handover_distance,
            handover_observer,

            compare_visualizer,
            logger,
            rviz,
        ],
    )


    return LaunchDescription([

        DeclareLaunchArgument(
            'robot_ip',
            default_value=_BRINGUP_DEFAULTS.get(
                'robot_ip',
                '192.168.2.10',
            ),
        ),

        DeclareLaunchArgument(
            'arm_id',
            default_value=_BRINGUP_DEFAULTS.get(
                'arm_id',
                'fr3',
            ),
        ),

        DeclareLaunchArgument(
            'namespace',
            default_value=_BRINGUP_DEFAULTS.get(
                'namespace',
                '',
            ),
        ),

        DeclareLaunchArgument(
            'load_gripper',
            default_value='false',
        ),

        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
        ),

        DeclareLaunchArgument(
            'start_logger',
            default_value='true',
        ),


        # Hardware
        franka,
        realsense,

        # Frame compatibility
        base_alias_tf,

        # Human handover pipeline
        pipeline,
    ])


if __name__ == '__main__':
    generate_launch_description()