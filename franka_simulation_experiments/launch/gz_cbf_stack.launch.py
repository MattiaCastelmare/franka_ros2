"""SIM. Pipeline A, closed loop, against the Gazebo FR3: one experiment, one command.

    ros2 launch franka_simulation_experiments gz_cbf_stack.launch.py \\
        condition:=kf [motion:=pentagon|scenario|rl] [scenario:=<name>] [seed:=0] \\
        [perception:=true|false|emulated] [scene:=rig|bare] [gui:=false] [duration_s:=0]

What runs (docs/gz_cbf_experiments_plan.md §5):

  franka_simulation/gz_fr3_plant.launch.py   Gazebo + FR3 + RSP + 30 Hz republisher + JSB
  franka_experiments/torque_control_stack    INCLUDED, unchanged, with external_plant:=true:
      real_time_distance, cbf_safety_filter, qddot_to_torque, the commander,
      experiment_logger, and the rt_torque_controller spawner
  sim-only nodes                              reflex_monitor, episode_supervisor, a sim bag

Every node runs on sim time (``SetParameter(use_sim_time)``, plan H6). Every
toggle of the condition is passed explicitly (plan H12, ground rule 3), and the
run directory gets a run_manifest_sim.json saying so. Outputs go under
``franka_simulation_experiments/sim_runs/``, never next to hardware logs.
"""
import json
import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

_ARGS = {
    'condition': ('', 'residual | kf | imm | imm+pred (REQUIRED, config/conditions.yaml)'),
    'set': ('', 'Per-run overrides, "key=value;key=value" (cbf flags, imm_enabled, prediction)'),
    'motion': ('', 'pentagon | scenario | rl: who publishes /NS_1/qddot_nom. Empty = the '
                   "scenario's (catalog/throws/tests: scenario commander; pentagon_*: pentagon), "
                   'pentagon without a scenario'),
    'scenario': ('', 'Scenario name in config/scenarios.yaml'),
    'seed': ('0', 'Episode seed (start-phase offset, optional jitter, adapter noise)'),
    'perception': ('true', 'true: simulated D455 + depth_sim_adapter + real_time_distance; '
                           "emulated: cbf_scenarios' perception emulation from truth "
                           '(emulated_perception, needs a scenario); false: none'),
    'emulated_fps': ('90', '[Hz] perception:=emulated frame rate (the full chain measures 90)'),
    'scene': ('', "rig (measured tables) | bare (floor only). Empty = the scenario's, else rig"),
    'gui': ('false', 'Ignition GUI client (headless otherwise)'),
    'duration_s': ('', "[s sim] stop the run after this long. Empty = the scenario's T "
                       '(0 = run until stopped)'),
    'run_dir': ('', 'Run directory. Empty = a new one under sim_runs/'),
    'record_bag': ('true', 'ros2 bag record --use-sim-time of the non-image topics'),
    'filter_cpu': ('', 'Pin cbf_safety_filter (all threads) to this CPU, e.g. an isolated core. '
                       'Empty = unpinned (the Phase 5 batch). On a shared host the QP loop '
                       'loses ticks: 245 Hz unpinned vs 300 Hz pinned, measured. Avoid CPU 3 '
                       'while the real robot runs on this host (its rt_pin_cpu core)'),
    'rl_onnx_model': ('', 'motion:=rl only'),
}

_ONE_THREAD = {'OPENBLAS_NUM_THREADS': '1', 'OMP_NUM_THREADS': '1'}

#: What smoothness_report.py, latency_budget.py bag and the scoring read; no images.
BAG_TOPICS = ['/NS_1/franka/joint_states', '/NS_1/joint_states', '/NS_1/qddot_nom',
              '/NS_1/qddot_safe', '/NS_1/torque_cmd', '/NS_1/cbf_status', '/NS_1/ee_actual',
              '/NS_1/ee_desired', '/cbf/per_link_distances', '/tf', '/tf_static', '/clock',
              '/sim/truth/obstacles', '/sim/truth/cp_gaps', '/sim/truth/obstacle_poses',
              '/sim/truth/obstacle_poses_vel', '/sim/reflex', '/sim/episode',
              '/sim/episode_start']


def _simexp():
    lib = os.path.join(get_package_prefix('franka_simulation_experiments'), 'lib',
                       'franka_simulation_experiments')
    if lib not in sys.path:
        sys.path.insert(0, lib)
    from simexp import conditions, sim_config
    return conditions, sim_config


def _resolve_scenario(name: str, seed: int, sc, paths: dict):
    """(resolved scenario, d_safe): catalog entry, seeded, outside the barrier at start."""
    import yaml
    from simexp import scenario_catalog
    from simexp.truth_geometry import TruthGeometry, load_perception_config
    _, grd = sc.franka_simulation_utils()
    with open(paths['fr3_control']) as f:
        d_safe = float(yaml.safe_load(f)['params']['d_safe'])
    r = scenario_catalog.resolve(name, seed=seed)
    _, rsp = grd.build_descriptions(grd.process_xacro('fr3', hand=True), initial_q=r['q0'])
    cps = TruthGeometry(rsp, load_perception_config(paths['fr3_complete'])).control_points(r['q0'])
    r = scenario_catalog.enforce_start_clearance(r, cps, d_safe)
    return r, d_safe


def _parse_set(s: str) -> dict:
    import yaml
    out = {}
    for kv in [x for x in s.replace(',', ';').split(';') if x.strip()]:
        k, v = kv.split('=', 1)
        out[k.strip()] = yaml.safe_load(v)
    return out


def _launch(context):
    p = {k: LaunchConfiguration(k).perform(context) for k in _ARGS}
    cond_mod, sc = _simexp()
    cond = cond_mod.resolve(p['condition'], _parse_set(p['set']))
    paths = sc.default_paths()
    seed = int(p['seed'])
    scen, d_safe = (_resolve_scenario(p['scenario'], seed, sc, paths) if p['scenario']
                    else (None, None))
    motion = (p['motion'].strip().lower() or (scen['motion'] if scen else 'pentagon'))
    if motion not in ('pentagon', 'scenario', 'rl'):
        raise RuntimeError(f'motion:={motion}: expected pentagon, scenario or rl')
    if motion == 'scenario' and scen is None:
        raise RuntimeError('motion:=scenario needs scenario:=<name>')
    scene = p['scene'] or (scen['scene'] if scen else 'rig')
    duration = (float(p['duration_s']) if p['duration_s'] != ''
                else (scen['T'] if scen else 0.0))
    perception_mode = p['perception'].strip().lower()
    if perception_mode not in ('true', 'false', 'emulated'):
        raise RuntimeError(f'perception:={perception_mode}: expected true, false or emulated')
    if perception_mode == 'emulated' and scen is None:
        raise RuntimeError('perception:=emulated needs scenario:=<name> (it emulates from truth)')
    perception = perception_mode == 'true'          # the simulated camera chain
    rows = perception_mode != 'false'               # someone publishes per-link rows
    run_dir = p['run_dir'] or sc.make_run_dir(name=f'{p["scenario"] or motion}_{p["condition"]}'
                                                   f'_s{seed}')
    os.makedirs(run_dir, exist_ok=True)

    ctrl_yaml = sc.rt_controllers_yaml(run_dir)
    obstacles = scen['obstacles'] if scen else []
    gz_world, _ = sc.franka_simulation_utils()
    world = sc.world_sdf(
        run_dir, scene=scene, depth_rate_hz=90.0,
        extra_models=gz_world.obstacle_models(obstacles),
        extra_world_plugins=[gz_world.scenario_obstacles_plugin(obstacles)] if obstacles else [])
    gui = p['gui'].lower() == 'true'
    # gui:=true has a display: give real_time_distance its window back
    # (config/sim/perception_sim.yaml turns it off for headless runs). The
    # scenario's Gazebo counterpart of cbf_scenarios' max_thresh lands here too.
    rtd_over = sc.deep_merge(dict(cond['perception']), scen['perception'] if scen else {})
    rtd_cfg = sc.perception_yaml(run_dir, sc.deep_merge(
        rtd_over, {'booleans': {'visualize': True}} if gui else {}))
    adapter_yaml = sc.depth_adapter_yaml(run_dir, scen['depth_adapter'] if scen else {})

    manifest = {
        'plant': 'gazebo', 'SIM': True,
        'git': sc.git_provenance(), 'run_dir': run_dir,
        'motion': motion, 'scenario': p['scenario'], 'seed': seed, 'duration_s': duration,
        'scenario_resolved': scen, 'perception': perception, 'perception_mode': perception_mode,
        'emulated_fps': float(p['emulated_fps']) if perception_mode == 'emulated' else None,
        'filter_cpu': p['filter_cpu'] or None,
        'scene': scene,
        'condition': cond['toggles'],
        'condition_description': cond['description'],
        'files': {'world': world, 'controllers': ctrl_yaml, 'perception_config': rtd_cfg,
                  'camera_extrinsics': paths['camera_extrinsics_sim'],
                  'depth_adapter': adapter_yaml},
    }
    with open(os.path.join(run_dir, 'run_manifest_sim.json'), 'w') as f:
        json.dump(manifest, f, indent=2, sort_keys=True, default=str)

    stack_args = dict(cond['stack_args'])
    stack_args.update({
        'external_plant': 'true',
        'use_fake_hardware': 'false',
        'motion_source': {'pentagon': 'pentagon', 'rl': 'rl', 'scenario': 'none'}[motion],
        'enable_camera': 'false',            # no RealSense driver: Gazebo is the camera
        'start_real_time_distance': 'true' if perception else 'false',
        'camera_extrinsics_yaml': paths['camera_extrinsics_sim'],
        'robot_config_yaml': rtd_cfg,
        'camera_depth_profile': '848x480x90',
        'rt_pin_cpu': '',                    # no isolcpus core to pin to in sim
        'start_move_group': 'false',         # the commander does not use it (ARCHITECTURE §12.3)
        'trajectory_overlay_window': 'false',
        'start_rosbag': 'false',             # its recorder is wall-time and records images
    })
    if motion == 'rl' and p['rl_onnx_model']:
        stack_args['rl_onnx_model'] = p['rl_onnx_model']

    fs_share = get_package_share_directory('franka_simulation')
    fe_share = get_package_share_directory('franka_experiments')
    actions = [
        LogInfo(msg=f'[gz_cbf_stack] SIM run -> {run_dir}  condition={p["condition"]}  '
                    f'motion={motion}  scenario={p["scenario"] or "-"}  seed={p["seed"]}'),
        SetEnvironmentVariable('FRANKA_LOGS_DIR', os.path.join(run_dir, 'logger')),
        SetParameter(name='use_sim_time', value=True),
        # experiment_logger's default-off truth columns (only the logger
        # declares these; every other node ignores an undeclared override)
        *([SetParameter(name='truth_topic', value='/sim/truth/cp_gaps'),
           SetParameter(name='truth_n_cp', value=11)] if scen is not None else []),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(fs_share, 'launch', 'gz_fr3_plant.launch.py')),
            launch_arguments={'world_sdf': world, 'controller_params': ctrl_yaml,
                              'run_dir': run_dir, 'gui': p['gui'],
                              'camera': 'true' if perception else 'false',
                              'depth_adapter_params': adapter_yaml,
                              'initial_q': ', '.join(str(v) for v in (scen['q0'] if scen else
                                                                      sc.Q0)),
                              'depth_seed': str(seed)}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(fe_share, 'launch',
                                                       'torque_control_stack.launch.py')),
            launch_arguments=stack_args.items()),
    ]
    reflex = Node(package='franka_simulation_experiments', executable='reflex_monitor',
                  name='reflex_monitor', output='log', additional_env=_ONE_THREAD,
                  parameters=[{'run_dir': run_dir,
                               'urdf_path': os.path.join(run_dir, 'fr3_rsp.urdf')}])
    supervisor = Node(package='franka_simulation_experiments', executable='episode_supervisor',
                      name='episode_supervisor', output='screen',
                      parameters=[{'run_dir': run_dir, 'duration_s': float(duration),
                                   'perception': rows,
                                   'start_offset_s': float(scen['start_offset_s'] if scen
                                                           else 0.0)}])
    if motion == 'scenario':
        actions.append(TimerAction(period=3.0, actions=[Node(
            package='franka_simulation_experiments', executable='scenario_commander',
            name='scenario_commander', output='log', additional_env=_ONE_THREAD,
            parameters=[{'nominal_json': json.dumps(scen['nominal']),
                         'q0': [float(v) for v in scen['q0']]}])]))
    if perception_mode == 'emulated':
        actions.append(TimerAction(period=3.0, actions=[Node(
            package='franka_simulation_experiments', executable='emulated_perception',
            name='emulated_perception', output='log', additional_env=_ONE_THREAD,
            parameters=[{'perception_config': rtd_cfg,
                         'urdf_path': os.path.join(run_dir, 'fr3_rsp.urdf'),
                         'obstacles_json': json.dumps(obstacles),
                         'fps': float(p['emulated_fps']),
                         'cam_noise': bool(scen['emulation'].get('cam_noise', False)),
                         'seed': seed}])]))
    if scen is not None:
        actions.append(TimerAction(period=3.0, actions=[Node(
            package='franka_simulation_experiments', executable='ground_truth_node',
            name='ground_truth', output='log', additional_env=_ONE_THREAD,
            parameters=[{'run_dir': run_dir, 'urdf_path': os.path.join(run_dir, 'fr3_rsp.urdf'),
                         'perception_config': paths['fr3_complete'], 'd_safe': d_safe,
                         'obstacles_json': json.dumps(obstacles)}])]))
    actions += [
        TimerAction(period=3.0, actions=[reflex]),
        TimerAction(period=3.0, actions=[supervisor]),
        RegisterEventHandler(OnProcessExit(target_action=supervisor, on_exit=[
            LogInfo(msg=f'[gz_cbf_stack] episode finished -> {run_dir}'),
            EmitEvent(event=Shutdown(reason='episode done'))])),
    ]
    if p['filter_cpu'].strip():
        actions.append(TimerAction(period=4.0, actions=[ExecuteProcess(
            cmd=[os.path.join(get_package_prefix('franka_simulation_experiments'), 'lib',
                              'franka_simulation_experiments', 'pin_process'),
                 '--cpu', p['filter_cpu'].strip(), '--pattern', 'cbf_safety_filter --ros-args'],
            output='screen', name='pin_filter')]))
    if p['record_bag'].lower() == 'true':
        actions.append(TimerAction(period=4.0, actions=[ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '--use-sim-time', '-o', os.path.join(run_dir, 'bag'),
                 *BAG_TOPICS], output='log', name='sim_bag', sigterm_timeout='30')]))
    # NOT wrapped in a scoped GroupAction: the stack starts its nodes from
    # TimerActions, which fire after a group's scope has been popped, and they
    # would lose both use_sim_time and FRANKA_LOGS_DIR (measured: the logger
    # then wrote into franka_experiments/franka_logs, next to hardware runs).
    return actions


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v, description=d) for k, (v, d) in _ARGS.items()]
        + [OpaqueFunction(function=_launch)])
