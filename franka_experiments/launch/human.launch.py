#!/usr/bin/env python3

import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
    ExecuteProcess,
    LogInfo,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from franka_experiments.utils.distance_utils import load_robot_config
from franka_experiments.utils.ros import (
    declare_robot_args,
    declare_rt_blender_args,
    load_franka_config_defaults,
    load_launch_defaults,
    pick_controllers_yaml,
    resolve_controller_manager_name,
)

DEFAULT_BAG_PATH = '/bags/arm_repeated'

# Caricamento dei default per il robot (necessari per la parte real)
_LAUNCH_DEFAULTS, _LAUNCH_DEFAULTS_PATH = load_launch_defaults()
_BRINGUP_DEFAULTS, _CONFIG_PATH = load_franka_config_defaults()
_DEFAULTS = {**_LAUNCH_DEFAULTS, **_BRINGUP_DEFAULTS}


def create_bag_player(context):
    """Create the same rosbag playback process if real:=false"""
    is_real = LaunchConfiguration('real').perform(context).lower() in ('true', '1', 'yes', 'on')
    if is_real:
        return []

    bag_path = os.path.expanduser(
        LaunchConfiguration('bag_path').perform(context)
    )
    if not bag_path:
        return [LogInfo(msg='bag_path is empty: rosbag was not started.')]

    return [ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--loop', '--clock', '100.0', '--read-ahead-queue-size', '1000'
        ],
        output='screen',
    )]


def _launch_real_robot(context):
    """Avvia il driver del robot e il controller se real è true"""
    is_real = LaunchConfiguration('real').perform(context).lower() in ('true', '1', 'yes', 'on')
    if not is_real:
        return []

    p = {k: LaunchConfiguration(k).perform(context) for k in [
        'namespace', 'use_fake_hardware', 'robot_ip', 'arm_id',
        'fake_sensor_commands', 'load_gripper', 'controllers_yaml',
        'qdot_max', 'max_accel', 'timeout_threshold_s', 'timeout_ramp_s',
        'gazebo', 'enable_interpolation', 'command_topic',
        'use_torque_controller', 'torque_command_topic', 'lpf_alpha', 'tau_max_scale',
        'control_spawner_delay_s'
    ]}

    use_fake = str(p['use_fake_hardware']).strip().lower() in ('1', 'true', 'yes', 'y', 'on')
    use_torque_val = p['use_torque_controller'].strip().lower()
    use_torque = use_torque_val in ('1', 'true', 'yes', 'y', 'on')
    cbf_pipeline = (use_torque_val == 'cbf')

    if cbf_pipeline:
        controller_name = 'cbf_torque_controller'
        rt_params = dict(is_real=not use_fake, arm_id=p['arm_id'], controller_type='cbf')
    elif use_torque:
        controller_name = 'rt_torque_controller'
        rt_params = dict(
            is_real=not use_fake, arm_id=p['arm_id'],
            qdot_max=p['qdot_max'], command_topic=p['command_topic'],
            max_accel=p['max_accel'], timeout_threshold_s=p['timeout_threshold_s'],
            timeout_ramp_s=p['timeout_ramp_s'], gazebo=p['gazebo'],
            enable_interpolation=p['enable_interpolation'], controller_type='torque',
            torque_command_topic=p['torque_command_topic'], lpf_alpha=p['lpf_alpha'],
            tau_max_scale=p['tau_max_scale']
        )
    else:
        controller_name = 'rt_velocity_executor_controller'
        rt_params = dict(
            is_real=not use_fake, arm_id=p['arm_id'],
            qdot_max=p['qdot_max'], command_topic=p['command_topic'],
            max_accel=p['max_accel'], timeout_threshold_s=p['timeout_threshold_s'],
            timeout_ramp_s=p['timeout_ramp_s'], gazebo=p['gazebo'],
            enable_interpolation=p['enable_interpolation']
        )

    controllers_yaml = pick_controllers_yaml(p['controllers_yaml'], use_fake, rt_params)
    cm_name = resolve_controller_manager_name(p['namespace'])

    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py',
        ]).perform(context)),
        launch_arguments={
            'arm_id': p['arm_id'],
            'robot_ip': p['robot_ip'],
            'namespace': p['namespace'],
            'use_fake_hardware': p['use_fake_hardware'],
            'fake_sensor_commands': p['fake_sensor_commands'],
            'load_gripper': p['load_gripper'],
            'controllers_yaml': controllers_yaml,
        }.items(),
    )

    world_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_robot_base_tf',
        output='log',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'world',
            '--child-frame-id', 'fr3_link0',
        ],
    )

    actions = [
        franka_launch,
        TimerAction(period=1.0, actions=[world_tf_node]),
    ]

    if controller_name is not None:
        controller_spawner = Node(
            package='controller_manager', executable='spawner',
            arguments=[controller_name, '--controller-manager', cm_name,
                       '--controller-manager-timeout', '30'],
            output='screen',
        )
        actions.append(TimerAction(period=float(p['control_spawner_delay_s']),
                                   actions=[controller_spawner]))

    # Avvio del driver RealSense (solo quando siamo sul robot reale)
    realsense_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py',
        ]).perform(context)),
        launch_arguments={'align_depth.enable': 'true'}.items(),
    )
    actions.append(realsense_driver)

    return actions


def generate_launch_description():
    package_share = get_package_share_directory('franka_experiments')
    camera_link_extrinsics_path = os.path.join(
        package_share,
        'config',
        'camera_extrinsics.yaml',
    )
    rviz_config_path = os.path.join(
        package_share,
        'config',
        'human.rviz',
    )

    extrinsics = load_robot_config(camera_link_extrinsics_path)
    translation = extrinsics['translation']
    rotation = extrinsics['rotation']

    rosbag_record = LaunchConfiguration('rosbag_record')
    run_name = LaunchConfiguration('run_name')
    default_run_name = time.strftime("%Y%m%d_%H%M%S")
    real = LaunchConfiguration('real', default='true')

    use_sim_time = ParameterValue(
        PythonExpression(["'", real, "'.lower() not in ['true', '1', 'yes', 'on']"]),
        value_type=bool,
    )

    publish_camera_tf = LaunchConfiguration('publish_camera_tf')

    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fr3_to_camera_link',
        condition=IfCondition(publish_camera_tf),
        arguments=[
            '--x', str(translation['x']),
            '--y', str(translation['y']),
            '--z', str(translation['z']),
            '--qx', str(rotation['x']),
            '--qy', str(rotation['y']),
            '--qz', str(rotation['z']),
            '--qw', str(rotation['w']),
            '--frame-id', 'fr3_link0',
            '--child-frame-id', 'camera_color_optical_frame',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    tracker = Node(
        package='franka_experiments',
        executable='human_tracker',
        name='human_tracker',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    distance = Node(
        package='franka_experiments',
        executable='human_distance',
        name='human_distance',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    human_logger_node = Node(
        package='franka_experiments',
        executable='human_logging',
        name='human_logger',
        parameters=[{
            'use_sim_time': use_sim_time,
            'run_name': ParameterValue(run_name, value_type=str)
        }],
        output='screen'
    )

    visualizer = Node(
        package='franka_experiments',
        executable='human_visualizer',
        name='human_visualizer',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    record_bag = ExecuteProcess(
        condition=IfCondition(rosbag_record),
        cmd=[
            'ros2', 'bag', 'record',
            '-o', ['experiment_bags/', run_name],
            '/NS_1/joint_states',
            '/camera/camera/aligned_depth_to_color/camera_info',
            '/camera/camera/aligned_depth_to_color/image_raw',
            '/camera/camera/color/camera_info',
            '/camera/camera/color/image_raw',
            '/tf', 
            '/tf_static'
        ],
        output='screen'
    )

    bag_player = TimerAction(
        period=2.0,
        actions=[OpaqueFunction(function=create_bag_player)],
    )

    real_robot_action = OpaqueFunction(function=_launch_real_robot)

    camera_tf_delayed = TimerAction(
        period=4.0,
        actions=[camera_tf]
    )

    return LaunchDescription(
        declare_robot_args(_DEFAULTS)
        + declare_rt_blender_args(_DEFAULTS)
        + [
            DeclareLaunchArgument('real', default_value='true', description='True for real robot, False for simulation with bag'),
            DeclareLaunchArgument('rosbag_record', default_value='false'),
            DeclareLaunchArgument('bag_path', default_value=DEFAULT_BAG_PATH),
            DeclareLaunchArgument('publish_camera_tf', default_value='true'),
            DeclareLaunchArgument('run_name', default_value=default_run_name),
            DeclareLaunchArgument('control_spawner_delay_s', default_value=str(_DEFAULTS.get('control_spawner_delay_s', '10.0'))),
            DeclareLaunchArgument('use_torque_controller', default_value=str(_DEFAULTS.get('use_torque_controller', 'true'))),
            DeclareLaunchArgument('torque_command_topic', default_value=str(_DEFAULTS.get('torque_command_topic', 'torque_cmd'))),
            DeclareLaunchArgument('lpf_alpha', default_value=str(_DEFAULTS.get('lpf_alpha', '1.0'))),
            DeclareLaunchArgument('tau_max_scale', default_value=str(_DEFAULTS.get('tau_max_scale', '1.0'))),
            real_robot_action,
            camera_tf_delayed,
            tracker,
            distance,
            human_logger_node,
            visualizer,
            rviz,
            bag_player,
            record_bag,
        ]
    )