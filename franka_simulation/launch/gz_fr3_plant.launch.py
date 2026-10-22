"""The Gazebo plant of the CBF experiments: Ignition Fortress + FR3 + ros2_control.

SIM. This is what replaces ``franka_bringup/franka.launch.py`` + libfranka when
pipeline A runs against Gazebo (docs/gz_cbf_experiments_plan.md §5). It
provides the same contract as the bringup (plan §1):

  /NS_1/controller_manager        gz_ros2_control's, at the YAML update_rate
  /NS_1/robot_state_publisher     base-rooted URDF, TF on /tf and /tf_static
  /NS_1/joint_state_publisher     the bringup's 30 Hz republisher of
                                  franka/joint_states -> /NS_1/joint_states
  joint_state_broadcaster         -> /NS_1/franka/joint_states (1 kHz)

and nothing else. Controllers beyond the JSB are spawned by whoever includes
this (``spawn_controllers``), e.g. torque_control_stack's own spawner.

Headless by default (server only, ``--headless-rendering`` with the NVIDIA
EGL ICD, plan §3.2); ``gui:=true`` adds the Ignition client.

    ros2 launch franka_simulation gz_fr3_plant.launch.py \
        controller_params:=/path/rt_controllers.yaml spawn_controllers:=rt_torque_controller

No dependency on franka_experiments: the controller YAML is an argument.
"""
import os
import sys
import tempfile

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

Q0 = '0.0, -0.70, 0.0, -2.35, 0.0, 1.57, 0.70'

_ARGS = {
    'namespace': ('NS_1', 'Robot namespace (the contract: NS_1)'),
    'world_sdf': ('', 'World SDF to load as is. Empty = worlds/fr3_cbf_lab.sdf'),
    'initial_q': (Q0, '7 start joint positions [rad], comma-separated (plan H16)'),
    'controller_params': ('', 'Extra parameter YAMLs for the gz controller_manager, '
                              'comma-separated (e.g. the generated rt_torque_controller YAML)'),
    'spawn_controllers': ('', 'Controllers to spawn after joint_state_broadcaster, comma-separated'),
    'spawn_delay_s': ('2.0', '[s] after Gazebo starts, before the robot is spawned'),
    'gui': ('false', 'Start the Ignition GUI client too'),
    'headless': ('true', 'Server-side headless rendering (EGL); false = use $DISPLAY'),
    'gz_verbosity': ('1', 'ign gazebo -v level'),
    'run_dir': ('', 'Where the generated URDFs are written. Empty = a temp directory'),
    'partition': ('', 'IGN_PARTITION. Empty = the inherited one, else fr3cbf_<pid>, so '
                      'parallel sims cannot cross-talk'),
    'joint_state_rate': ('30', '[Hz] the bringup republisher rate (franka.config.yaml)'),
    'camera': ('true', 'Bridge the D455 depth stream and run depth_sim_adapter, so the plant '
                       'publishes /camera/camera/depth/* like the RealSense driver does'),
    'depth_adapter_params': ('', 'Parameter YAML for depth_sim_adapter. Empty = '
                                 'config/sim/depth_adapter.yaml'),
    'depth_seed': ('0', 'depth_sim_adapter noise/dropout seed'),
    'depth_gz_topic': ('/d455/depth/image', 'Gazebo depth image topic (world SDF <topic>)'),
    'scenario_bridge': ('true', 'Bridge the ScenarioObstacles topics: the episode start time '
                                '(ROS -> gz) and the obstacle truth (gz -> ROS)'),
}


def _utils():
    lib = os.path.join(get_package_prefix('franka_simulation'), 'lib', 'franka_simulation')
    if lib not in sys.path:
        sys.path.insert(0, lib)
    from utils import gz_robot_description
    return gz_robot_description


def _launch(context):
    p = {k: LaunchConfiguration(k).perform(context) for k in _ARGS}
    ns = p['namespace'].strip('/')
    share = get_package_share_directory('franka_simulation')
    world = p['world_sdf'] or os.path.join(share, 'worlds', 'fr3_cbf_lab.sdf')
    run_dir = p['run_dir'] or tempfile.mkdtemp(prefix='gz_fr3_plant_')
    os.makedirs(run_dir, exist_ok=True)
    q0 = [float(v) for v in p['initial_q'].replace(';', ',').split(',') if v.strip()]
    params = [s.strip() for s in p['controller_params'].split(',') if s.strip()]

    grd = _utils()
    gz_urdf, rsp_urdf = grd.build_descriptions(
        grd.process_xacro('fr3', hand=True), initial_q=q0, namespace=ns,
        extra_param_files=params)
    gz_urdf_path = os.path.join(run_dir, 'fr3_gazebo.urdf')
    with open(gz_urdf_path, 'w') as f:
        f.write(gz_urdf)
    with open(os.path.join(run_dir, 'fr3_rsp.urdf'), 'w') as f:
        f.write(rsp_urdf)

    desc_parent = os.path.dirname(get_package_share_directory('franka_description'))
    resource = ':'.join(filter(None, [desc_parent, os.path.join(share, 'worlds'),
                                      os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')]))
    plugin_path = ':'.join(filter(None, [os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''),
                                         os.environ.get('LD_LIBRARY_PATH', '')]))
    partition = p['partition'] or os.environ.get('IGN_PARTITION') or f'fr3cbf_{os.getpid()}'
    headless = p['headless'].lower() in ('1', 'true', 'yes')
    env = [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', plugin_path),
        SetEnvironmentVariable('IGN_PARTITION', partition),
    ]
    if headless:
        # Mesa is the only EGL vendor registered in the container and it cannot
        # render here (no depth frame at all); point glvnd at NVIDIA's ICD.
        env.append(SetEnvironmentVariable(
            '__EGL_VENDOR_LIBRARY_FILENAMES',
            os.path.join(share, 'config', 'sim', 'nvidia_egl_vendor.json')))

    server = ['ign', 'gazebo', '-s', '-r', '-v', p['gz_verbosity']]
    if headless:
        server.append('--headless-rendering')
    actions = env + [
        LogInfo(msg=f'[gz_fr3_plant] SIM  world={world}  partition={partition}  '
                    f'urdf={gz_urdf_path}  q0={q0}'),
        ExecuteProcess(cmd=server + [world], output='log', name='gz_server'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             parameters=[{'use_sim_time': True}], output='log'),
        # The two bringup nodes a Gazebo plant has to provide itself
        # (franka.launch.py:105-139), with the same parameters.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=ns, output='log',
             parameters=[{'robot_description': rsp_urdf, 'use_sim_time': True}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             name='joint_state_publisher', namespace=ns, output='log',
             parameters=[{'source_list': ['franka/joint_states', 'franka_gripper/joint_states'],
                          'rate': int(p['joint_state_rate']),
                          'use_robot_description': False, 'use_sim_time': True}]),
        TimerAction(period=float(p['spawn_delay_s']), actions=[
            Node(package='ros_gz_sim', executable='create', output='log',
                 arguments=['-file', gz_urdf_path, '-name', 'fr3']),
        ]),
        TimerAction(period=float(p['spawn_delay_s']) + 1.0, actions=[
            Node(package='controller_manager', executable='spawner', namespace=ns, output='log',
                 arguments=['joint_state_broadcaster', '--controller-manager',
                            f'/{ns}/controller_manager', '--controller-manager-timeout', '60']),
        ]),
    ]
    if p['camera'].lower() in ('1', 'true', 'yes'):
        gz_img = p['depth_gz_topic']
        gz_info = gz_img.rsplit('/', 1)[0] + '/camera_info'
        adapter_params = p['depth_adapter_params'] or os.path.join(
            share, 'config', 'sim', 'depth_adapter.yaml')
        actions += [
            Node(package='ros_gz_bridge', executable='parameter_bridge', name='d455_bridge',
                 output='log', parameters=[{'use_sim_time': True}],
                 arguments=[f'{gz_img}@sensor_msgs/msg/Image[gz.msgs.Image',
                            f'{gz_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
                 remappings=[(gz_img, '/sim/d455/depth/image'),
                             (gz_info, '/sim/d455/depth/camera_info')]),
            Node(package='franka_simulation', executable='depth_sim_adapter',
                 name='depth_sim_adapter', output='log',
                 additional_env={'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'},
                 parameters=[adapter_params, {'use_sim_time': True,
                                              'seed': int(p['depth_seed'])}]),
        ]
    if p['scenario_bridge'].lower() in ('1', 'true', 'yes'):
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='scenario_bridge',
            output='log', parameters=[{'use_sim_time': True}],
            arguments=['/sim/episode_start@std_msgs/msg/Float64]gz.msgs.Double',
                       '/sim/truth/obstacle_poses@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
                       '/sim/truth/obstacle_poses_vel@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V']))
    for name in [s.strip() for s in p['spawn_controllers'].split(',') if s.strip()]:
        actions.append(TimerAction(period=float(p['spawn_delay_s']) + 2.0, actions=[
            Node(package='controller_manager', executable='spawner', namespace=ns, output='log',
                 arguments=[name, '--controller-manager', f'/{ns}/controller_manager',
                            '--controller-manager-timeout', '60'])]))
    if p['gui'].lower() in ('1', 'true', 'yes'):
        actions.append(ExecuteProcess(cmd=['ign', 'gazebo', '-g'], output='log', name='gz_gui'))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in _ARGS.items()]
        + [OpaqueFunction(function=_launch)])
