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
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from tf2_msgs.msg import TFMessage

from franka_experiments.utils.distance_utils import load_robot_config
from franka_experiments.utils.ros import (
    declare_robot_args, declare_rt_blender_args,
    load_franka_config_defaults, load_launch_defaults,
    pick_controllers_yaml, resolve_controller_manager_name,
)

DEFAULT_BAG_PATH = '/bags/varied'

# Loading default values for the robot
_LAUNCH_DEFAULTS, _LAUNCH_DEFAULTS_PATH = load_launch_defaults()
_BRINGUP_DEFAULTS, _CONFIG_PATH = load_franka_config_defaults()
_DEFAULTS = {**_LAUNCH_DEFAULTS, **_BRINGUP_DEFAULTS}

# Camera frame of the tracker; the bags also carry the RealSense-internal parent of it
# (camera_color_frame), not connected to the robot, which must not reach /tf_static
_CAMERA_FRAME = 'camera_color_optical_frame'


def _static_tf_node(name, parent, child, translation, rotation, use_sim_time):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', str(translation[0]), '--y', str(translation[1]), '--z', str(translation[2]),
            '--qx', str(rotation[0]), '--qy', str(rotation[1]),
            '--qz', str(rotation[2]), '--qw', str(rotation[3]),
            '--frame-id', parent, '--child-frame-id', child,
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='log',
    )


def bag_static_transforms(bag_path):
    """Static transforms recorded in a bag: {child: (parent, translation, rotation)}."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_path, storage_id=''), ConverterOptions('', ''))
    reader.set_filter(StorageFilter(topics=['/tf_static']))
    transforms = {}
    while reader.has_next():
        _, data, _ = reader.read_next()
        for tf in deserialize_message(data, TFMessage).transforms:
            t, q = tf.transform.translation, tf.transform.rotation
            transforms.setdefault(tf.child_frame_id, []).append(
                (tf.header.frame_id, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)))
    return transforms


def create_bag_player(context):
    """Replay the bag if real:=false, with the camera calibration it was recorded with.

    The bag's /tf_static is diverted: in it the camera frame has two parents (the robot
    base and the RealSense driver's own frames), and which one TF keeps decides whether
    the tracker can place the camera at all. The camera (re-parented to fr3_link0; in the
    bags base == fr3_link0) and fr3_link8 are republished from it instead; the
    camera_extrinsics.yaml of today would be wrong for a bag recorded before the latest
    calibration.
    """
    is_real = LaunchConfiguration('real').perform(context).lower() in ('true', '1', 'yes', 'on')
    if is_real:
        return []

    bag_path = os.path.expanduser(
        LaunchConfiguration('bag_path').perform(context)
    )
    if not bag_path:
        return [LogInfo(msg='bag_path is empty: rosbag was not started.')]

    actions = []
    statics = bag_static_transforms(bag_path)
    camera = [tf for tf in statics.get(_CAMERA_FRAME, []) if tf[0] in ('base', 'fr3_link0')]
    if camera:
        _, translation, rotation = camera[-1]
        actions.append(_static_tf_node(
            'bag_camera_tf', 'fr3_link0', _CAMERA_FRAME, translation, rotation, True))
        actions.append(LogInfo(msg=f'[human] camera from the bag: t={translation}'))
    else:
        actions.append(LogInfo(msg='[human] no camera transform in the bag: camera_extrinsics.yaml'))
        actions.append(_camera_tf_from_extrinsics(True))
    for parent, translation, rotation in statics.get('fr3_link8', [])[-1:]:
        actions.append(_static_tf_node(
            'bag_link8_tf', parent, 'fr3_link8', translation, rotation, True))

    actions.append(ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--loop', '--clock', '100.0', '--read-ahead-queue-size', '1000',
            '--remap', '/tf_static:=/bag/tf_static',
        ],
        output='screen',
    ))
    return actions


def _camera_tf_from_extrinsics(use_sim_time):
    """fr3_link0 -> camera from today's calibration (config/camera_extrinsics.yaml)."""
    extrinsics = load_robot_config(os.path.join(
        get_package_share_directory('franka_experiments'), 'config', 'camera_extrinsics.yaml'))
    t, r = extrinsics['translation'], extrinsics['rotation']
    return _static_tf_node(
        'fr3_to_camera_link', 'fr3_link0', _CAMERA_FRAME,
        (t['x'], t['y'], t['z']), (r['x'], r['y'], r['z'], r['w']), use_sim_time)


def create_camera_tf(context):
    """Camera TF on the real robot (real:=false takes it from the bag, see create_bag_player)."""
    is_real = LaunchConfiguration('real').perform(context).lower() in ('true', '1', 'yes', 'on')
    publish = LaunchConfiguration('publish_camera_tf').perform(context).lower() in (
        'true', '1', 'yes', 'on')
    return [_camera_tf_from_extrinsics(False)] if is_real and publish else []


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
        'control_spawner_delay_s', 'rt_pin_cpu'
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

    # Pin the ros2_control RT thread onto the isolated core (see scripts/pin_rt_thread.sh).
    # Unpinned, it shares cores with MediaPipe and IRQs -> communication_constraints_violation.
    rt_pin_cpu = str(p['rt_pin_cpu']).strip()
    if rt_pin_cpu and not use_fake:
        pin_rt_thread = ExecuteProcess(
            cmd=['bash', PathJoinSubstitution([
                FindPackageShare('franka_experiments'), 'scripts', 'pin_rt_thread.sh',
            ]).perform(context), rt_pin_cpu, '60'],
            output='screen',
        )
        actions.append(TimerAction(period=float(p['control_spawner_delay_s']),
                                   actions=[pin_rt_thread]))

    # Avvio del driver RealSense (solo quando siamo sul robot reale)
    realsense_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py',
        ]).perform(context)),
        # publish_tf off: the driver's camera_color_frame -> camera_color_optical_frame
        launch_arguments={'align_depth.enable': 'true', 'publish_tf': 'false'}.items(),
    )
    actions.append(realsense_driver)

    return actions


def generate_launch_description():
    package_share = get_package_share_directory('franka_experiments')
    rviz_config_path = os.path.join(
        package_share,
        'config',
        'human.rviz',
    )

    rosbag_record = LaunchConfiguration('rosbag_record')
    run_name = LaunchConfiguration('run_name')
    default_run_name = time.strftime("%Y%m%d_%H%M%S")
    real = LaunchConfiguration('real', default='true')

    use_sim_time = ParameterValue(
        PythonExpression(["'", real, "'.lower() not in ['true', '1', 'yes', 'on']"]),
        value_type=bool,
    )
    # On the robot the 1 kHz state feed; recorded bags only carry the 30 Hz republished one
    joint_state_topic = PythonExpression([
        "'/NS_1/franka/joint_states' if '", real,
        "'.lower() in ['true', '1', 'yes', 'on'] else '/NS_1/joint_states'",
    ])

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
        parameters=[{
            'use_sim_time': use_sim_time,
            'joint_state_topic': ParameterValue(joint_state_topic, value_type=str),
        }],
        output='screen',
    )

    human_logger_node = Node(
        package='franka_experiments',
        executable='human_logging',
        name='human_logger',
        parameters=[{
            'use_sim_time': use_sim_time,
            'run_name': ParameterValue(run_name, value_type=str),
            'robot_state_topic': ParameterValue(joint_state_topic, value_type=str),
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
            '-o', ['recorded_bags/', run_name],
            '/NS_1/joint_states',
            '/NS_1/franka/joint_states',
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
        actions=[OpaqueFunction(function=create_camera_tf)]
    )

    return LaunchDescription(
        declare_robot_args(_DEFAULTS)
        + declare_rt_blender_args(_DEFAULTS)
        + [
            DeclareLaunchArgument('real', default_value='true', description='True for real robot, False for simulation with bag'),
            DeclareLaunchArgument('rosbag_record', default_value='false'),
            DeclareLaunchArgument('bag_path', default_value=DEFAULT_BAG_PATH),
            DeclareLaunchArgument('publish_camera_tf', default_value='true',
                                  description='real:=true only: publish config/camera_extrinsics.yaml'),
            DeclareLaunchArgument('run_name', default_value=default_run_name),
            DeclareLaunchArgument('control_spawner_delay_s', default_value=str(_DEFAULTS.get('control_spawner_delay_s', '10.0'))),
            DeclareLaunchArgument('use_torque_controller', default_value=str(_DEFAULTS.get('use_torque_controller', 'true'))),
            DeclareLaunchArgument('torque_command_topic', default_value=str(_DEFAULTS.get('torque_command_topic', 'torque_cmd'))),
            DeclareLaunchArgument('lpf_alpha', default_value=str(_DEFAULTS.get('lpf_alpha', '1.0'))),
            DeclareLaunchArgument('tau_max_scale', default_value=str(_DEFAULTS.get('tau_max_scale', '1.0'))),
            DeclareLaunchArgument('rt_pin_cpu', default_value=str(_DEFAULTS.get('rt_pin_cpu', '3')),
                                  description="Isolated CPU for the ros2_control RT thread ('' = no pinning)"),
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