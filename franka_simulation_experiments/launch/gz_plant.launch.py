"""SIM. The Gazebo plant with the rt_torque_controller registered: Phase 1 of the plan.

franka_simulation's ``gz_fr3_plant.launch.py`` (Gazebo + FR3 + RSP + the 30 Hz
republisher + JSB), with what only this package can add:

* the rt_torque_controller YAML from franka_experiments'
  ``launch_support.generate_rt_controllers_yaml`` (hardware gains, 1000 Hz,
  ``gazebo: true``), given to the plugin as a second ``<parameters>`` file;
* the world configured from ``config/sim/camera_extrinsics_sim.yaml`` and the
  D455 depth intrinsics.

``spawn_rt_controller:=true`` also spawns it (the Phase 1 tests); the full stack
leaves that to torque_control_stack's own spawner.

    ros2 launch franka_simulation_experiments gz_plant.launch.py spawn_rt_controller:=true
"""
import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

_ARGS = {
    'run_dir': ('', 'Run directory (generated files). Empty = a new one under sim_runs/'),
    'scene': ('rig', 'rig (tables as measured on the rig) | bare (floor only, H9)'),
    'spawn_rt_controller': ('false', 'Spawn rt_torque_controller here (Phase 1 tests)'),
    'gui': ('false', 'Ignition GUI client'),
    'initial_q': ('0.0, -0.70, 0.0, -2.35, 0.0, 1.57, 0.70', 'Start pose [rad]'),
    'depth_rate_hz': ('90', 'D455 depth update rate (camera_depth_profile fps)'),
}


def _simexp():
    lib = os.path.join(get_package_prefix('franka_simulation_experiments'), 'lib',
                       'franka_simulation_experiments')
    if lib not in sys.path:
        sys.path.insert(0, lib)
    from simexp import sim_config
    return sim_config


def _launch(context):
    p = {k: LaunchConfiguration(k).perform(context) for k in _ARGS}
    sc = _simexp()
    run_dir = p['run_dir'] or sc.make_run_dir(name='plant')
    os.makedirs(run_dir, exist_ok=True)
    ctrl = sc.rt_controllers_yaml(run_dir)
    world = sc.world_sdf(run_dir, scene=p['scene'], depth_rate_hz=float(p['depth_rate_hz']))
    spawn = 'rt_torque_controller' if p['spawn_rt_controller'].lower() == 'true' else ''
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('franka_simulation'), 'launch', 'gz_fr3_plant.launch.py')),
        launch_arguments={'world_sdf': world, 'controller_params': ctrl, 'run_dir': run_dir,
                          'spawn_controllers': spawn, 'gui': p['gui'],
                          'initial_q': p['initial_q']}.items())]


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in _ARGS.items()]
        + [OpaqueFunction(function=_launch)])
