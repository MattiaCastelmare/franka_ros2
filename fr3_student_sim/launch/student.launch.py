"""Gazebo FR3 + effort controller + virtual wall + the student's controller.

    ros2 launch fr3_student_sim student.launch.py                       # my_controller.py
    ros2 launch fr3_student_sim student.launch.py controller:=examples/joint_pd.py
    ros2 launch fr3_student_sim student.launch.py wall:=false duration:=10 gui:=false

``controller`` is a path to a Python file; relative paths are looked up in the
package's ``students/`` folder. When the run ends (duration, ``done``, safety
stop) the log is saved in ``log_dir`` and everything shuts down.
"""
import math
import os
import tempfile

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'fr3_student_sim'

ARGS = {
    'controller': ('my_controller.py', 'Controller file (relative to students/ or absolute); '
                                       '"none" = only the robot, commanded from the terminal'),
    'task_config': ('', 'Task YAML passed to the controller as self.params '
                        '(empty = config/task.yaml)'),
    'wall_config': ('', 'Wall YAML (empty = config/wall.yaml)'),
    'wall': ('true', 'Enable the virtual wall'),
    'stop_at_detachment': ('false', 'End the run when the tool first leaves the wall'),
    'duration': ('20.0', '[s] run length after the settle phase'),
    'control_rate': ('1000.0', '[Hz] how often Controller.update is called (<= 1000)'),
    'gui': ('true', 'Open the Gazebo window'),
    'log_dir': ('', 'Where the .npz logs go (empty = runs/ in the fr3_student_sim source folder)'),
    'run_name': ('', 'Prefix of the log file (empty = controller name)'),
    'real_time_factor': ('0.5', 'Gazebo speed vs wall clock. 0.5 leaves the Python controller '
                                'time to run at 1 kHz; raise it only for light controllers'),
    # realism (all off by default: the plant is then exactly the students' model)
    'joint_friction': ('false', "Keep franka_description's joint friction/damping in Gazebo "
                                '(the model does not know it: ~0.3 N m unmodelled torque)'),
    'mass_error': ('0.0', 'Scale each link mass/inertia IN GAZEBO by a random factor in '
                          '[1-e, 1+e]; the model keeps the nominal values (e.g. 0.05)'),
    'noise_q': ('0.0', '[rad] std of Gaussian noise on the q the controller sees'),
    'noise_dq': ('0.0', '[rad/s] std of Gaussian noise on the dq the controller sees'),
    'seed': ('0', 'Random seed for mass_error and the noise'),
    'partition': ('', 'IGN_PARTITION (to run several simulations side by side)'),
}

WORLD = """<?xml version="1.0"?>
<sdf version="1.8">
  <world name="student_world">
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>{rtf}</real_time_factor>
      <real_time_update_rate>{rate}</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.85 0.88 0.92 1</background></scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <direction>-0.4 0.2 -0.9</direction>
    </light>
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material></visual>
      </link>
    </model>
{wall}  </world>
</sdf>
"""

WALL = """    <model name="virtual_wall">
      <static>true</static>
      <pose>{x} {y} {z} {r} {p} {yaw}</pose>
      <link name="link">
        <visual name="v"><geometry><box><size>0.01 0.5 0.5</size></box></geometry>
          <material><ambient>0.25 0.25 0.28 1</ambient><diffuse>0.25 0.25 0.28 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular></material></visual>
      </link>
    </model>
"""


def _wall_sdf(wall: dict) -> str:
    """A thin visual-only box whose +x face lies on the wall plane (no collision:
    the force comes from the runner's spring-damper model)."""
    n = np.asarray(wall['normal'], float)
    n /= np.linalg.norm(n)
    x = -n                                   # box x axis points into the wall
    up = np.array([0.0, 0.0, 1.0]) if abs(x[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    y = np.cross(up, x)
    y /= np.linalg.norm(y)
    z = np.cross(x, y)
    R = np.column_stack([x, y, z])
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = -math.asin(max(-1.0, min(1.0, R[2, 0])))
    yaw = math.atan2(R[1, 0], R[0, 0])
    c = np.asarray(wall['point'], float) + 0.005 * x
    return WALL.format(x=c[0], y=c[1], z=c[2], r=roll, p=pitch, yaw=yaw)


def _launch(context):
    a = {k: LaunchConfiguration(k).perform(context) for k in ARGS}
    on = lambda k: a[k].lower() in ('1', 'true', 'yes')  # noqa: E731
    share = get_package_share_directory(PKG)
    from fr3_student_sim.paths import runs_dir
    from fr3_student_sim.robot_description import build_urdf, process_xacro

    controller = a['controller']
    sim_only = controller.lower() == 'none'
    if not sim_only and not os.path.isabs(controller):
        controller = os.path.join(share, 'students', controller)
    task_cfg = a['task_config'] or os.path.join(share, 'config', 'task.yaml')
    wall_cfg = a['wall_config'] or os.path.join(share, 'config', 'wall.yaml')
    with open(task_cfg) as f:
        task = yaml.safe_load(f) or {}
    with open(wall_cfg) as f:
        wall = yaml.safe_load(f) or {}
    q0 = [float(v) for v in task.get('initial_q', [0, -0.785, 0, -2.356, 0, 1.571, 0.785])]

    run_dir = tempfile.mkdtemp(prefix='fr3_student_')
    xacro_xml = process_xacro()
    # Gazebo (and the plugin's gravity compensation) simulate the "real" robot,
    # possibly with mass errors; the students' model gets the nominal URDF.
    urdf = build_urdf(xacro_xml, initial_q=q0, joint_friction=on('joint_friction'),
                      mass_error=float(a['mass_error']), seed=int(a['seed']),
                      param_files=[os.path.join(share, 'config', 'controllers.yaml')])
    urdf_path = os.path.join(run_dir, 'fr3.urdf')
    model_urdf_path = os.path.join(run_dir, 'fr3_nominal.urdf')
    with open(urdf_path, 'w') as f:
        f.write(urdf)
    with open(model_urdf_path, 'w') as f:
        f.write(build_urdf(xacro_xml, initial_q=q0))
    rtf = float(a['real_time_factor'])
    world_path = os.path.join(run_dir, 'world.sdf')
    with open(world_path, 'w') as f:
        f.write(WORLD.format(rtf=rtf, rate=1000.0 * rtf,
                             wall=_wall_sdf(wall) if on('wall') else ''))

    desc_parent = os.path.dirname(get_package_share_directory('franka_description'))
    resource = ':'.join(filter(None, [desc_parent, os.environ.get('IGN_GAZEBO_RESOURCE_PATH')]))
    plugins = ':'.join(filter(None, [os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH'),
                                     os.environ.get('LD_LIBRARY_PATH')]))
    env = [SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource),
           SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource),
           SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', plugins)]
    if a['partition']:
        env.append(SetEnvironmentVariable('IGN_PARTITION', a['partition']))

    spawner = lambda name: Node(  # noqa: E731
        package='controller_manager', executable='spawner', output='log',
        arguments=[name, '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'])
    spawn_robot = Node(package='ros_gz_sim', executable='create', output='log',
                       arguments=['-file', urdf_path, '-name', 'fr3'])
    jsb, effort = spawner('joint_state_broadcaster'), spawner('fr3_effort_controller')
    runner = Node(
        package=PKG, executable='runner', name='student_runner', output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'urdf_path': model_urdf_path, 'controller': controller,
                     'noise_q': float(a['noise_q']), 'noise_dq': float(a['noise_dq']),
                     'seed': int(a['seed']), 'mass_error': float(a['mass_error']),
                     'joint_friction': on('joint_friction'),
                     'task_config': task_cfg, 'wall_config': wall_cfg,
                     'wall_enabled': on('wall'),
                     'stop_at_detachment': on('stop_at_detachment'), 'duration': float(a['duration']),
                     'control_rate': float(a['control_rate']),
                     'log_dir': a['log_dir'] or runs_dir(),
                     'run_name': a['run_name']}])

    actions = env + [
        LogInfo(msg=f'[{PKG}] controller={controller}  world={world_path}'),
        ExecuteProcess(cmd=['ign', 'gazebo', '-s', '-r', '-v', '1', world_path],
                       output='log', name='gazebo'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='log'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='log', parameters=[{'robot_description': urdf, 'use_sim_time': True}]),
        spawn_robot,
        # robot -> joint_state_broadcaster -> effort controller -> runner -> shutdown
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[effort])),
    ]
    if sim_only:
        # only the robot: torques come from the terminal (ros2 topic pub ...)
        actions.append(RegisterEventHandler(OnProcessExit(target_action=effort, on_exit=[
            LogInfo(msg=f'[{PKG}] robot ready: publish 7 torques on /fr3_effort_controller/commands')])))
    else:
        actions += [
            RegisterEventHandler(OnProcessExit(target_action=effort, on_exit=[runner])),
            RegisterEventHandler(OnProcessExit(target_action=runner, on_exit=[
                EmitEvent(event=Shutdown(reason='run finished'))])),
        ]
    if on('gui'):
        actions.append(ExecuteProcess(cmd=['ign', 'gazebo', '-g', '-v', '1'],
                                      output='log', name='gazebo_gui'))
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in ARGS.items()]
        + [OpaqueFunction(function=_launch)])
