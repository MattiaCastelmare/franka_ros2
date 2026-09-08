

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from franka_experiments.utils.ros import (
    load_franka_config_defaults,
    resolve_controller_manager_name,
)

_DEFAULTS, _CONFIG_PATH = load_franka_config_defaults()

_MODULE = 'franka_experiments.nodes.workspace_gripper_cycle'
_CONTROLLER = 'gripper_controller'
_CONTROLLER_TYPE = 'franka_rt_controllers/GripperController'
_TORQUE_CONTROLLER = 'rt_torque_controller'

# Launch arguments forwarded to the node as parameters of the same name.
# The velocity clamp is NOT among them: launch configurations propagate into
# included launch files, and an argument named qdot_max would shadow the one
# minimal.launch.py declares for the RT velocity executor.  It is exposed as
# 'cycle_qdot_max' and forwarded explicitly below.
_NODE_PARAMS = ['segment_duration_s', 'grip_hold_s', 'loop', 'use_gripper',
                'qddot_max']


def _as_bool(value) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'y', 'on')


def _poll(name: str, test: str, timeout_s: str) -> ExecuteProcess:
    """Run *test* until it succeeds, or give up after *timeout_s*."""
    return ExecuteProcess(
        name=name,
        cmd=['timeout', str(timeout_s), 'bash', '-c',
             f'until {test}; do sleep 1; done'],
        output='screen',
    )


def _launch_setup(context):
    def p(key):
        return LaunchConfiguration(key).perform(context)

    ns = p('namespace').strip('/')
    cm = resolve_controller_manager_name(ns)
    timeout_s = p('timeout_s')
    grip_srv = f'/{ns}/{_CONTROLLER}/set_gripper' if ns \
        else f'/{_CONTROLLER}/set_gripper'
    use_gripper = _as_bool(p('use_gripper'))

    cycle = ExecuteProcess(
        name='workspace_gripper_cycle',
        cmd=['python3', '-m', _MODULE, '--ros-args']
            + (['-r', f'__ns:=/{ns}'] if ns else [])
            + [arg for key in _NODE_PARAMS for arg in ('-p', f'{key}:={p(key)}')]
            + ['-p', f'qdot_max:={p("cycle_qdot_max")}'],
        output='screen',
    )

    # Existing node, unmodified: qddot_safe → torque_cmd (τ = M·q̈ + C·q̇).
    # Its topics are absolute (fr3_control.yaml); the namespace only keeps the
    # node name tidy.
    qddot_to_torque = Node(
        package='franka_experiments', executable='qddot_to_torque',
        name='qddot_to_torque', namespace=ns or None, output='screen',
    )

    wait_cm = _poll(
        'wait_controller_manager',
        f'ros2 service list 2>/dev/null | grep -q "^{cm}/list_controllers$"',
        timeout_s)
    wait_torque = _poll(
        'wait_rt_torque_controller',
        f'ros2 control list_controllers --controller-manager {cm} 2>/dev/null'
        f' | grep -q "{_TORQUE_CONTROLLER}.*active"',
        timeout_s)

    actions = [
        LogInfo(msg=['[wp_cycle] ns=', ns or '<none>',
                     '  robot=', p('start_robot'),
                     '  gripper=', p('use_gripper'),
                     '  segment=', p('segment_duration_s'), 's',
                     '  loop=', p('loop')]),
    ]

    if _as_bool(p('start_robot')):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('franka_experiments'),
                'launch', 'minimal.launch.py',
            ]).perform(context)),
            launch_arguments={
                'arm_id':                   p('arm_id'),
                'robot_ip':                 p('robot_ip'),
                'namespace':                ns,
                'use_fake_hardware':        p('use_fake_hardware'),
                'load_gripper':             'true',    # franka_gripper actions
                'use_torque_controller':    'true',    # rt_torque_controller
                # This launch is a gripper/motion test: keep perception out.
                'enable_camera':            'false',
                'start_real_time_distance': 'false',
                'start_experiment_logger':  'false',
            }.items(),
        ))

    actions.append(wait_cm)

    if not use_gripper:
        actions += [
            LogInfo(msg='[wp_cycle] gripper off — controller not spawned'),
            RegisterEventHandler(OnProcessExit(
                target_action=wait_cm, on_exit=[qddot_to_torque, wait_torque])),
            RegisterEventHandler(OnProcessExit(
                target_action=wait_torque, on_exit=[cycle])),
        ]
        return actions

    spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=[_CONTROLLER,
                   '--controller-manager', cm,
                   '--controller-type', _CONTROLLER_TYPE,
                   '--param-file', PathJoinSubstitution([
                       FindPackageShare('franka_rt_controllers'),
                       'config', 'gripper_controller.yaml',
                   ]).perform(context),
                   '--controller-manager-timeout', timeout_s],
        output='screen',
    )
    wait_grip = _poll(
        'wait_gripper_service',
        f'ros2 service list 2>/dev/null | grep -q "^{grip_srv}$"',
        timeout_s)

    actions += [
        LogInfo(msg=['[wp_cycle] spawn ', _CONTROLLER, ' → wait ', grip_srv,
                     ' → wait ', _TORQUE_CONTROLLER, ' active']),
        RegisterEventHandler(OnProcessExit(
            target_action=wait_cm, on_exit=[qddot_to_torque, spawner])),
        # A non-zero spawner exit (typically: already loaded by another launch)
        # still moves on — the gates below are what actually matter.
        RegisterEventHandler(OnProcessExit(
            target_action=spawner, on_exit=[wait_grip])),
        RegisterEventHandler(OnProcessExit(
            target_action=wait_grip, on_exit=[wait_torque])),
        RegisterEventHandler(OnProcessExit(
            target_action=wait_torque, on_exit=[cycle])),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value=_DEFAULTS.get('namespace', ''),
            description='Namespace of the controller_manager and of the nodes'),
        DeclareLaunchArgument(
            'arm_id', default_value=_DEFAULTS.get('arm_id', 'fr3'),
            description='Robot arm model identifier'),
        DeclareLaunchArgument(
            'robot_ip', default_value=_DEFAULTS.get('robot_ip', '192.168.2.10'),
            description='IP address of the robot'),
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value=_DEFAULTS.get('use_fake_hardware', 'false'),
            description='Fake hardware: no motion and no gripper actions — '
                        'pair with use_gripper:=false'),
        DeclareLaunchArgument(
            'start_robot', default_value='true',
            description='false = skip the bringup, for when the stack is '
                        'already running elsewhere'),
        DeclareLaunchArgument(
            'timeout_s', default_value='60',
            description='Seconds each startup gate waits before giving up'),
        DeclareLaunchArgument(
            'use_gripper', default_value='true',
            description='false = motion only: no gripper controller, no grasps'),
        DeclareLaunchArgument(
            'segment_duration_s', default_value='3.0',
            description='Duration of each waypoint-to-waypoint move [s]'),
        DeclareLaunchArgument(
            'grip_hold_s', default_value='2.0',
            description='Time the fingers get to close, and to open [s]'),
        DeclareLaunchArgument(
            'loop', default_value='false',
            description='true = repeat the waypoint list forever'),
        DeclareLaunchArgument(
            'cycle_qdot_max', default_value='0.3',
            description="Per-joint velocity clamp of the cycle [rad/s]. Not "
                        "named 'qdot_max': that belongs to the RT velocity "
                        "executor and would be shadowed in the bringup"),
        DeclareLaunchArgument(
            'qddot_max', default_value='2.0',
            description='Per-joint acceleration clamp of the cycle [rad/s^2]'),
        OpaqueFunction(function=_launch_setup),
    ])
