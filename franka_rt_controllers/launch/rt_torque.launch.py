"""Launch: franka bringup + RT torque controller.

Spawns joint_state_broadcaster (via franka.launch.py) and
rt_torque_controller.  YAML is generated at launch time using
generate_rt_controllers_yaml() from franka_experiments.

Accepted arguments: arm_id, robot_ip, use_fake_hardware, namespace,
                    gazebo, lpf_alpha, tau_max_scale, torque_command_topic
"""

from franka_experiments.utils.ros import (
    generate_rt_controllers_yaml,
    load_franka_config_defaults,
    resolve_controller_manager_name,
    write_temp_yaml,
)

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

_DEFAULTS, _CONFIG_PATH = load_franka_config_defaults()


def _launch_all(context):
    def p(key):
        return LaunchConfiguration(key).perform(context)

    use_fake   = p('use_fake_hardware').strip().lower() in ('1', 'true', 'yes')
    namespace  = p('namespace')
    arm_id     = p('arm_id')
    cm_name    = resolve_controller_manager_name(namespace)

    yaml_content = generate_rt_controllers_yaml(
        is_real=not use_fake,
        arm_id=arm_id,
        qdot_max='0.0',
        command_topic='torque_cmd',
        max_accel='0.0',
        timeout_threshold_s='0.0',
        timeout_ramp_s='0.0',
        gazebo=p('gazebo'),
        enable_interpolation='false',
        controller_type='torque',
        torque_command_topic=p('torque_command_topic'),
        lpf_alpha=p('lpf_alpha'),
        tau_max_scale=p('tau_max_scale'),
    )
    controllers_yaml = write_temp_yaml(yaml_content, prefix='franka_rt_torque_')

    franka_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('franka_bringup'), 'launch', 'franka.launch.py',
        ]).perform(context)),
        launch_arguments={
            'arm_id':               arm_id,
            'robot_ip':             p('robot_ip'),
            'namespace':            namespace,
            'use_fake_hardware':    p('use_fake_hardware'),
            'fake_sensor_commands': p('fake_sensor_commands'),
            'load_gripper':         p('load_gripper'),
            'controllers_yaml':     controllers_yaml,
        }.items(),
    )

    torque_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['rt_torque_controller',
                   '--controller-manager', cm_name,
                   '--controller-manager-timeout', '30'],
        output='screen',
    )

    return [
        LogInfo(msg=['[rt_torque] arm_id=', arm_id,
                     '  ip=', p('robot_ip'),
                     '  fake=', p('use_fake_hardware'),
                     '  topic=', p('torque_command_topic'),
                     '  lpf_alpha=', p('lpf_alpha'),
                     '  yaml=', controllers_yaml]),
        franka_launch,
        TimerAction(period=10.0, actions=[torque_spawner]),
    ]


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
            description='Use fake hardware'),
        DeclareLaunchArgument('fake_sensor_commands',
            default_value=_DEFAULTS.get('fake_sensor_commands', 'false'),
            description='Fake sensor commands'),
        DeclareLaunchArgument('load_gripper',
            default_value=_DEFAULTS.get('load_gripper', 'false'),
            description='Load Franka Gripper'),
        DeclareLaunchArgument('gazebo',
            default_value='false',
            description='Set true when running in Gazebo'),
        DeclareLaunchArgument('torque_command_topic',
            default_value='torque_cmd',
            description='Topic for torque commands (Float64MultiArray, size=7)'),
        DeclareLaunchArgument('lpf_alpha',
            default_value='1.0',
            description='Low-pass filter alpha for tau_cmd. 1.0 = off (pass-through)'),
        DeclareLaunchArgument('tau_max_scale',
            default_value='1.0',
            description='Scale factor applied to per-joint torque limits'),
        OpaqueFunction(function=_launch_all),
    ])
