#!/usr/bin/env python3
"""Handover on the real FR3: robot, both cameras, perception pipeline and the handover commander.

Startup order
  1. franka bringup (rt_torque_controller config: accel_topic = /NS_1/qddot_nom, NO CBF), D455 + D405,
     alias fr3_link0 -> base; after 2 s the perception pipeline (parameters: config/hand_tracking.yaml)
  2. controller_manager up -> qddot_to_torque, rt_torque_controller spawner, gripper_controller
     (use_gripper / grasp_executor), RT thread pinning (rt_pin_cpu)
  3. rt_torque_controller active -> handover commander (follow_hand false: hold / test offset),
     or the grasp cycle with grasp_executor:=true
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from franka_experiments.utils.ros import (load_franka_config_defaults, load_launch_defaults,
                                          pick_controllers_yaml, resolve_controller_manager_name)

_DEFAULTS = {**load_launch_defaults()[0], **load_franka_config_defaults()[0]}
_QDDOT_NOM_TOPIC = '/NS_1/qddot_nom'
_TORQUE_CONTROLLER = 'rt_torque_controller'
SCRIPTS = '/ros2_ws/src/franka_experiments/scripts/'

# numpy's OpenBLAS keeps one spinning thread per core on the tiny matrices of
# these nodes (kalman_hand alone took ~2 cores): one thread is enough.
PERCEPTION_ENV = {'OPENBLAS_NUM_THREADS': '1'}


def _as_bool(value) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'y', 'on')


def _on_success(actions):
    def on_exit(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            raise RuntimeError(
                f'[handover] {event.process_name} failed (exit={event.returncode}); aborting startup')
        return actions

    return on_exit


def _poll(name: str, test: str, timeout_s: str) -> ExecuteProcess:
    """Wait until a ROS condition becomes true (at most timeout_s)."""
    return ExecuteProcess(name=name, output='screen',
                          cmd=['timeout', str(timeout_s), 'bash', '-o', 'pipefail', '-c',
                               f'until {test}; do sleep 1; done'])


def _include(package, launch_file, context, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file]).perform(context)),
        launch_arguments=arguments.items())


def _launch_setup(context):
    def p(key):
        return LaunchConfiguration(key).perform(context)

    ns = p('namespace').strip('/')
    cm = resolve_controller_manager_name(ns)
    use_fake = _as_bool(p('use_fake_hardware'))
    timeout_s = p('timeout_s')
    # grasp cycle (take / release / home) instead of palm following; needs the gripper
    grasp_executor = _as_bool(p('grasp_executor'))
    # Franka hand + gripper_controller (open / close service) also without the grasp cycle
    use_gripper = grasp_executor or _as_bool(p('use_gripper'))

    # ------------------------------------------------------------ robot and cameras
    # NO CBF in handover mode: qddot_to_torque and the rt_torque_controller watchdog use the same
    # nominal acceleration, accel_topic = /NS_1/qddot_nom
    rt_params = dict(is_real=not use_fake, arm_id=p('arm_id'), controller_type='torque',
                     torque_command_topic=p('torque_command_topic'), gazebo=p('gazebo'),
                     lpf_alpha=float(p('lpf_alpha')), tau_max_scale=float(p('tau_max_scale')),
                     accel_topic=_QDDOT_NOM_TOPIC)
    franka = _include('franka_bringup', 'franka.launch.py', context, {
        'arm_id': p('arm_id'), 'robot_ip': p('robot_ip'), 'namespace': ns,
        'use_fake_hardware': p('use_fake_hardware'), 'fake_sensor_commands': p('fake_sensor_commands'),
        # the gripper needs franka_gripper (gripper_controller actions)
        'load_gripper': 'true' if use_gripper else p('load_gripper'),
        'controllers_yaml': pick_controllers_yaml(p('controllers_yaml'), use_fake, rt_params)})

    # serial needed with the D405 also plugged in, otherwise the driver may open the D405 instead of the D455
    realsense = _include('realsense2_camera', 'rs_launch.py', context,
                         {'align_depth.enable': 'true', 'serial_no': '_318122300288'})
    # Wrist D405 on a USB 2 extension: 640x480x30 colour+depth fits, 848x480x30 and infra streams do not.
    realsense_wrist = _include('realsense2_camera', 'rs_launch.py', context, {
        'camera_namespace': 'd405', 'camera_name': 'd405', 'serial_no': '_126122270738',
        'align_depth.enable': 'true', 'enable_infra1': 'false', 'enable_infra2': 'false',
        'depth_module.color_profile': '640x480x30', 'depth_module.depth_profile': '640x480x30'})
    base_alias_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', name='fr3_link0_to_hand_base_tf',
        output='log',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                   '--frame-id', 'fr3_link0', '--child-frame-id', 'base'])

    # ------------------------------------------------------------ perception pipeline
    hand_tracking_yaml = PathJoinSubstitution(
        [FindPackageShare('franka_experiments'), 'config', 'hand_tracking.yaml'])

    def pipeline_node(executable, overrides=None, yaml=True, **kwargs):
        return Node(package='franka_experiments', executable=executable, output='screen',
                    additional_env=PERCEPTION_ENV,
                    parameters=(([hand_tracking_yaml] if yaml else [])
                                + [{'use_sim_time': False, **(overrides or {})}]),
                    **kwargs)

    perception_pipeline = TimerAction(period=2.0, actions=[
        pipeline_node('human_hand_tracker', {'hand_detector': p('hand_detector'),
                                             'rtmw_mode': p('rtmw_mode'),
                                             'gripper_camera': _as_bool(p('gripper_camera'))}),
        pipeline_node('kalman_hand'),
        pipeline_node('hand_state_estimator'),
        pipeline_node('distance_handover_estimator', name='distance_handover_estimator'),

        # object tracking -> /handover/hand_object (needs Hands23)
        pipeline_node('grasp', yaml=False, condition=IfCondition(LaunchConfiguration('start_grasp'))),
        # grasp pose -> /handover/grasp_pose (GSNet, or AnyGrasp with backend:=anygrasp)
        ExecuteProcess(cmd=['python3', SCRIPTS + 'handover_grasp.py', 'pose',
                            '--ros-args', '-p', 'use_sim_time:=false'],
                       output='screen', additional_env=PERCEPTION_ENV,
                       condition=IfCondition(LaunchConfiguration('start_grasp_pose'))),

        # visualisation and logging
        pipeline_node('hand_visualizer', yaml=False, condition=IfCondition(LaunchConfiguration('start_rviz'))),
        pipeline_node('hand_logger', condition=IfCondition(LaunchConfiguration('start_logger'))),
        Node(package='rviz2', executable='rviz2', name='handover_rviz', output='screen',
             condition=IfCondition(LaunchConfiguration('start_rviz')),
             arguments=['-d', PathJoinSubstitution(
                 [FindPackageShare('franka_experiments'), 'config', 'hand_tracker.rviz'])],
             parameters=[{'use_sim_time': False}],
             additional_env={'__NV_PRIME_RENDER_OFFLOAD': '1', '__GLX_VENDOR_LIBRARY_NAME': 'nvidia'}),
    ])

    # ------------------------------------------------------------ control
    # qddot_to_torque reads /NS_1/qddot_safe: without CBF it gets /NS_1/qddot_nom
    qddot_to_torque = Node(package='franka_experiments', executable='qddot_to_torque', name='qddot_to_torque',
                           namespace=ns or None, output='screen',
                           remappings=[('/NS_1/qddot_safe', _QDDOT_NOM_TOPIC)])
    controller_spawner = Node(package='controller_manager', executable='spawner', output='screen',
                              arguments=[_TORQUE_CONTROLLER, '--controller-manager', cm,
                                         '--controller-manager-timeout', timeout_s])

    after_cm = [qddot_to_torque, controller_spawner]
    if use_gripper:  # open / close through gripper_controller/set_gripper
        after_cm.append(Node(
            package='controller_manager', executable='spawner', output='screen',
            arguments=['gripper_controller', '--controller-manager', cm,
                       '--controller-type', 'franka_rt_controllers/GripperController',
                       '--param-file', PathJoinSubstitution([FindPackageShare('franka_rt_controllers'), 'config',
                                                             'gripper_controller.yaml']).perform(context),
                       '--controller-manager-timeout', timeout_s]))

    rt_pin_cpu = str(p('rt_pin_cpu')).strip()
    if rt_pin_cpu and not use_fake:
        after_cm.append(ExecuteProcess(
            name='pin_rt_torque_thread', output='screen',
            cmd=['bash',
                 PathJoinSubstitution([FindPackageShare('franka_experiments'), 'scripts', 'pin_rt_thread.sh']),
                 rt_pin_cpu, '60']))

    # the Python class inherits the Pentagon commander, so the ROS node is renamed here
    handover_commander = ExecuteProcess(
        name='handover_qddot_commander_process', output='screen',
        cmd=['python3', SCRIPTS + ('handover_grasp.py' if grasp_executor else 'handover_qddot_commander.py'),
             '--ros-args', '-r', '__node:=handover_qddot_commander',
             '-p', f'gripper_service:={"/" + ns if ns else ""}/gripper_controller/set_gripper'])

    # ------------------------------------------------------------ startup gates
    wait_cm = _poll('wait_controller_manager',
                    f'ros2 service list 2>/dev/null | grep -Fx "{cm}/list_controllers" >/dev/null', timeout_s)
    wait_torque = _poll(
        'wait_rt_torque_controller',
        f'ros2 control list_controllers --controller-manager {cm} 2>/dev/null '
        r"| sed -E 's/\x1B\[[0-9;]*m//g' "
        f'| grep -E "^[[:space:]]*{_TORQUE_CONTROLLER}[[:space:]]+[^[:space:]]+[[:space:]]+active[[:space:]]*$" '
        '>/dev/null',
        timeout_s)

    return [
        LogInfo(msg=['[handover] namespace=', ns or '<none>', '  robot_ip=', p('robot_ip'),
                     '  qddot=', _QDDOT_NOM_TOPIC, '  torque_topic=', p('torque_command_topic'),
                     '  CBF=DISABLED']),
        franka, realsense, realsense_wrist, base_alias_tf, perception_pipeline,
        RegisterEventHandler(OnProcessExit(target_action=wait_cm, on_exit=_on_success(after_cm))),
        # spawner finished -> wait for the controller to be ACTIVE -> commander
        RegisterEventHandler(OnProcessExit(target_action=controller_spawner,
                                           on_exit=_on_success([wait_torque]))),
        RegisterEventHandler(OnProcessExit(target_action=wait_torque,
                                           on_exit=_on_success([handover_commander]))),
        wait_cm,
    ]


def generate_launch_description():
    defaults = [('namespace', 'NS_1'), ('arm_id', 'fr3'), ('robot_ip', '192.168.2.10'),
                ('use_fake_hardware', 'false'), ('fake_sensor_commands', 'false'), ('controllers_yaml', ''),
                ('gazebo', 'false'), ('torque_command_topic', 'torque_cmd'), ('lpf_alpha', '1.0'),
                ('tau_max_scale', '1.0'), ('rt_pin_cpu', '3')]

    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=str(_DEFAULTS.get(name, value))) for name, value in defaults] + [
            DeclareLaunchArgument('load_gripper', default_value='false'),
            DeclareLaunchArgument('timeout_s', default_value='60'),
            DeclareLaunchArgument('start_rviz', default_value='true'),
            DeclareLaunchArgument('start_logger', default_value='true'),
            # hand detector: rtmw | mediapipe
            DeclareLaunchArgument('hand_detector', default_value='rtmw'),
            # RTMW size: lightweight (RTMW-m) | balanced (RTMW-l)
            DeclareLaunchArgument('rtmw_mode', default_value='lightweight'),
            # D405 on the gripper as second view of the hand
            DeclareLaunchArgument('gripper_camera', default_value='true'),
            DeclareLaunchArgument('start_grasp', default_value='false'),
            DeclareLaunchArgument('start_grasp_pose', default_value='false'),
            DeclareLaunchArgument('grasp_executor', default_value='false'),
            # Franka hand + gripper_controller (open / close) without the grasp cycle
            DeclareLaunchArgument('use_gripper', default_value='false'),
            OpaqueFunction(function=_launch_setup),
        ])
