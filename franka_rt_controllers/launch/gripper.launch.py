

from franka_experiments.utils.ros import (
    load_franka_config_defaults,
    resolve_controller_manager_name,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_DEFAULTS, _CONFIG_PATH = load_franka_config_defaults()

_CONTROLLER = 'gripper_controller'


def _launch_all(context):
    def p(key):
        return LaunchConfiguration(key).perform(context)

    namespace = p('namespace')
    cm_name = resolve_controller_manager_name(namespace)

    controllers_yaml = p('controllers_yaml')
    if not controllers_yaml:
        controllers_yaml = PathJoinSubstitution([
            FindPackageShare('franka_rt_controllers'),
            'config', 'gripper_controller.yaml',
        ]).perform(context)

    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py',
        ]).perform(context)),
        launch_arguments={
            'arm_id':               p('arm_id'),
            'robot_ip':             p('robot_ip'),
            'namespace':            namespace,
            'use_fake_hardware':    p('use_fake_hardware'),
            'fake_sensor_commands': p('fake_sensor_commands'),
            'load_gripper':         'true',   # required: provides move/grasp
            'controllers_yaml':     controllers_yaml,
        }.items(),
    )

    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[_CONTROLLER,
                   '--controller-manager', cm_name,
                   '--controller-manager-timeout', '30'],
        output='screen',
    )

    spawner_delay = float(p('spawner_delay_s'))
    service_name = (f'/{namespace}/{_CONTROLLER}/set_gripper' if namespace
                    else f'/{_CONTROLLER}/set_gripper')

    actions = [
        LogInfo(msg=['[gripper] arm_id=', p('arm_id'),
                     '  ip=', p('robot_ip'),
                     '  fake=', p('use_fake_hardware'),
                     '  action=', p('gripper_action'),
                     '  service=', service_name]),
        franka_launch,
        TimerAction(period=spawner_delay, actions=[gripper_spawner]),
    ]

    action = p('gripper_action').strip().lower()
    if action in ('open', 'close'):
        data = 'true' if action == 'close' else 'false'
        actions.append(TimerAction(
            period=spawner_delay + float(p('command_delay_s')),
            actions=[ExecuteProcess(
                cmd=['ros2', 'service', 'call', service_name,
                     'std_srvs/srv/SetBool', '{data: ' + data + '}'],
                output='screen',
            )],
        ))
    elif action not in ('', 'none'):
        raise RuntimeError(
            f"gripper_action must be 'open', 'close' or 'none', got {action!r}")

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arm_id',
            default_value=_DEFAULTS.get('arm_id', 'fr3'),
            description='Robot arm model identifier'),
        DeclareLaunchArgument('robot_ip',
            default_value=_DEFAULTS.get('robot_ip', '192.168.2.10'),
            description='IP address of the robot'),
        DeclareLaunchArgument('namespace',
            default_value=_DEFAULTS.get('namespace', ''),
            description='Namespace for the robot'),
        DeclareLaunchArgument('use_fake_hardware',
            default_value=_DEFAULTS.get('use_fake_hardware', 'false'),
            description='Use fake hardware (no move/grasp action server)'),
        DeclareLaunchArgument('fake_sensor_commands',
            default_value=_DEFAULTS.get('fake_sensor_commands', 'false'),
            description='Fake sensor commands'),
        DeclareLaunchArgument('controllers_yaml',
            default_value='',
            description='Controllers YAML. Empty = '
                        'franka_rt_controllers/config/gripper_controller.yaml'),
        DeclareLaunchArgument('gripper_action',
            default_value='none',
            description="Command sent once after startup: 'open', 'close' or 'none'"),
        DeclareLaunchArgument('spawner_delay_s',
            default_value='10.0',
            description='Seconds before spawning gripper_controller'),
        DeclareLaunchArgument('command_delay_s',
            default_value='3.0',
            description='Seconds after the spawner before sending the command'),
        OpaqueFunction(function=_launch_all),
    ])
