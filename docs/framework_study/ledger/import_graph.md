# Import graph and reachability — static, NOT yet verified

> Built from every `import`/`from … import` statement (including function-local and conditional imports) in the Python files in scope. Static analysis cannot see `importlib` string imports or `sys.path` manipulation targets; those are listed by hand below.

## Entry points (`franka_experiments/setup.py`)

| Console script | Module | Started by a production launch file? | Started by a test launch file? | Internal modules in closure |
|---|---|---|---|---:|
| `capsule_overlay_node` | `franka_experiments/nodes/capsule_overlay_node.py` | **no** |  | 19 |
| `cbf_oscbf_filter` | `franka_experiments/nodes/cbf_OSCBF_filter.py` | **no** |  | 18 |
| `cbf_safety_filter` | `franka_experiments/nodes/cbf_safety_filter.py` | yes |  | 27 |
| `cbf_velocity_filter` | `franka_experiments/nodes/cbf_velocity_filter.py` | yes |  | 6 |
| `ee_circle_velocity_commander` | `franka_experiments/nodes/ee_circle_velocity_commander.py` | **no** |  | 21 |
| `ee_pentagon_velocity_commander` | `franka_experiments/nodes/ee_pentagon_velocity_commander.py` | yes | yes | 21 |
| `ee_random_waypoints_velocity_commander` | `franka_experiments/nodes/ee_random_waypoints_velocity_commander.py` | **no** |  | 21 |
| `experiment_logger` | `franka_experiments/nodes/experiment_logger.py` | yes |  | 14 |
| `frame_grabber` | `franka_experiments/nodes/frame_grabber.py` | **no** |  | 21 |
| `handeye_calibration_node` | `franka_experiments/nodes/handeye_calibration_node.py` | yes |  | 21 |
| `handeye_eye_in_hand_node` | `franka_experiments/nodes/handeye_eye_in_hand_node.py` | yes |  | 19 |
| `image_rectifier_node` | `franka_experiments/nodes/image_rectifier_node.py` | yes |  | 0 |
| `iso_evidence_logger` | `franka_experiments/nodes/iso_evidence_logger.py` | yes |  | 20 |
| `iso_safety_monitor` | `franka_experiments/nodes/iso_safety_monitor.py` | yes |  | 20 |
| `pentagon_qddot_commander` | `franka_experiments/nodes/pentagon_qddot_commander.py` | yes |  | 21 |
| `pentagon_torque_commander` | `franka_experiments/nodes/pentagon_torque_commander.py` | **no** | yes | 21 |
| `qddot_to_torque` | `franka_experiments/nodes/qddot_to_torque.py` | yes |  | 7 |
| `real_time_distance` | `franka_experiments/nodes/real_time_distance.py` | yes |  | 29 |
| `rl_policy_commander` | `franka_experiments/nodes/rl_policy_commander.py` | yes |  | 23 |
| `tag_overlay_node` | `franka_experiments/nodes/tag_overlay_node.py` | yes |  | 0 |
| `trajectory_overlay_node` | `franka_experiments/nodes/trajectory_overlay_node.py` | yes |  | 8 |
| `trajectory_visualization_node` | `franka_experiments/nodes/trajectory_visualization_node.py` | yes |  | 7 |

## Executables of `franka_simulation_experiments` (CMake installs every `scripts/*.py`)

| Executable | Started by an in-package launch file? | Internal modules in closure |
|---|---|---:|
| `check_self_occlusion` | no (CLI tool / batch) | 18 |
| `emulated_perception` | yes | 13 |
| `episode_supervisor` | yes | 4 |
| `ground_truth_node` | yes | 11 |
| `gz_trajectory_markers` | yes | 1 |
| `pin_process` | no (CLI tool / batch) | 0 |
| `plot_control_regression` | no (CLI tool / batch) | 0 |
| `reflex_monitor` | yes | 18 |
| `run_gz_scenarios` | no (CLI tool / batch) | 40 |
| `scenario_commander` | yes | 7 |

## Per-module internal dependencies (fan-out → / fan-in ←)

| Module | Imports (internal) | Imported by (non-test) | Imported by tests |
|---|---|---|---:|
| `nodes/__init__.py` |  |  | 0 |
| `nodes/capsule_overlay_node.py` | camera_yaml, constants, kinematics, logging_utils, simulation_imports |  | 0 |
| `nodes/cbf_OSCBF_filter.py` | cbf_kinematics, constants, kinematics, logging_utils |  | 0 |
| `nodes/cbf_safety_filter.py` | cbf_diagnostics, cbf_hard_limits, cbf_qp_assembly, cbf_state_rows, cbf_zones, config, go_around, kinematics, livelock, logging_utils, perception_msgs, rate_scaling, state_governor | ball_closed_loop | 10 |
| `nodes/cbf_velocity_filter.py` | cbf_utils, ros_setup |  | 0 |
| `nodes/distance_visualization_node.py` | distance_utils, mask_builder, tf_manager, visualization |  | 0 |
| `nodes/ee_circle_velocity_commander.py` | constants, kinematics, logging_utils, math_utils, ros |  | 0 |
| `nodes/ee_pentagon_velocity_commander.py` | constants, kinematics, logging_utils, math_utils, ros, trajectory |  | 0 |
| `nodes/ee_random_waypoints_velocity_commander.py` | constants, kinematics, logging_utils, math_utils, ros, trajectory |  | 0 |
| `nodes/experiment_logger.py` | cbf_utils, config, kinematics, math_utils, node_runtime, params, perception_msgs | thales.launch | 3 |
| `nodes/frame_grabber.py` | ros |  | 0 |
| `nodes/handeye_calibration_node.py` | constants, kinematics, logging_utils, math_utils, ros, trajectory |  | 0 |
| `nodes/handeye_eye_in_hand_node.py` | __init__, constants, handeye_solver, kinematics, logging_utils, math_utils, node_runtime |  | 0 |
| `nodes/image_rectifier_node.py` |  |  | 0 |
| `nodes/iso_evidence_logger.py` | cbf_state_rows, config, iso_ssm, kinematics, params, perception_msgs |  | 0 |
| `nodes/iso_safety_monitor.py` | cbf_state_rows, config, iso_ssm, kinematics, perception_msgs |  | 1 |
| `nodes/pentagon_qddot_commander.py` | cbf_utils, config, constants, kinematics, logging_utils, node_runtime, singularity_recovery, timing_law, trajectory |  | 0 |
| `nodes/pentagon_torque_commander.py` | constants, kinematics, logging_utils, math_utils, ros, trajectory |  | 0 |
| `nodes/qddot_to_torque.py` | cbf_utils, config, constants, kinematics, node_runtime, params |  | 1 |
| `nodes/real_time_distance.py` | ballistic_prediction, calibration_check, distance_engine, distance_utils, logging_utils, mask_builder, obstacle_sim, obstacle_track_pipeline, params, perception_msgs, rate_scaling, self_detection, tf_manager, visualization | approach_probe | 1 |
| `nodes/rl_policy_commander.py` | cbf_utils, constants, kinematics, logging_utils, rl_policy, ros |  | 1 |
| `nodes/tag_overlay_node.py` |  |  | 0 |
| `nodes/trajectory_overlay_node.py` | cbf_utils, config, node_runtime, params, trajectory_overlay, trajectory_trace |  | 0 |
| `nodes/trajectory_visualization_node.py` | cbf_utils, node_runtime, params, trajectory_trace |  | 0 |
| `utils/__init__.py` |  | ball_closed_loop, handeye_eye_in_hand_node, ros | 1 |
| `utils/avoidance.py` |  |  | 1 |
| `utils/avoidance_math.py` | math_utils | ros_setup | 0 |
| `utils/ballistic_prediction.py` | distance_engine | real_time_distance | 1 |
| `utils/calibration_check.py` |  | iso_constants_measure, real_time_distance | 1 |
| `utils/camera_yaml.py` | config | capsule_overlay_node | 0 |
| `utils/cbf_constraints.py` | cbf_kinematics |  | 0 |
| `utils/cbf_diagnostics.py` |  | cbf_safety_filter, provenance | 3 |
| `utils/cbf_evasion.py` |  | cbf_state_rows | 1 |
| `utils/cbf_hard_limits.py` |  | ball_closed_loop, cbf_safety_filter, cbf_scenarios, latency_budget, reflex_checks, smoothness_report, state_governor | 10 |
| `utils/cbf_kinematics.py` | kinematics | cbf_OSCBF_filter, cbf_constraints | 0 |
| `utils/cbf_qp.py` |  |  | 0 |
| `utils/cbf_qp_assembly.py` | cbf_state_rows | ball_closed_loop, cbf_safety_filter, cbf_scenarios, latency_budget | 16 |
| `utils/cbf_singularity.py` |  | cbf_state_rows | 1 |
| `utils/cbf_state_rows.py` | cbf_evasion, cbf_singularity, cbf_zones, evasion_direction, iso_ssm, kinematics, rate_scaling, self_collision, self_shadow | cbf_qp_assembly, cbf_safety_filter, cbf_scenarios, check_self_occlusion, compare_range_noise, compare_vobs, evasion_direction, iso_constants_measure, iso_evidence_logger, iso_safety_monitor, latency_budget, logging_utils, reflex_checks | 35 |
| `utils/cbf_utils.py` | config, math_utils | cbf_velocity_filter, experiment_logger, pentagon_qddot_commander, qddot_to_torque, rl_policy_commander, trajectory_overlay_node, trajectory_visualization_node | 1 |
| `utils/cbf_zones.py` |  | cbf_safety_filter, cbf_state_rows, logging_utils, state_governor | 1 |
| `utils/config.py` | params | bag_replay.launch, ball_closed_loop, camera_yaml, cbf_safety_filter, cbf_scenarios, cbf_utils, check_self_occlusion, distance_utils, experiment_logger, gen_zero_gain_controllers_yaml, iso_constants_measure, iso_evidence_logger, iso_safety_monitor, kinematics, latency_budget, launch_support, node_runtime, pentagon_qddot_commander, qddot_to_torque, reflex_checks, ros, sim_config, torque_control_stack.launch, trajectory_overlay_node | 2 |
| `utils/constants.py` |  | capsule_overlay_node, cbf_OSCBF_filter, ee_circle_velocity_commander, ee_pentagon_velocity_commander, ee_random_waypoints_velocity_commander, handeye_calibration_node, handeye_eye_in_hand_node, kinematics, node_runtime, pentagon_qddot_commander, pentagon_torque_commander, qddot_to_torque, rl_policy, rl_policy_commander, ros | 1 |
| `utils/distance_engine.py` | obstacle_clusters, rate_scaling, self_shadow | ballistic_prediction, cbf_scenarios, compare_range_noise, compare_vobs, emulation, latency_budget, perception_msgs, real_time_distance | 10 |
| `utils/distance_utils.py` | config, perception_msgs | ball_throw_eval, cbf_scenarios, check_control_derivatives, compare_range_noise, compare_vobs, distance_visualization_node, latency_budget, real_time_distance, tf_manager, truth_geometry | 2 |
| `utils/evasion_direction.py` | cbf_state_rows | cbf_state_rows | 2 |
| `utils/go_around.py` |  | cbf_safety_filter | 1 |
| `utils/handeye_solver.py` |  | handeye_eye_in_hand_node | 1 |
| `utils/iso_ssm.py` |  | cbf_state_rows, iso_evidence_logger, iso_pfl_speed, iso_safety_monitor | 3 |
| `utils/kinematics.py` | config, constants, math_utils, node_runtime, params | ball_closed_loop, capsule_overlay_node, cbf_OSCBF_filter, cbf_kinematics, cbf_safety_filter, cbf_scenarios, cbf_state_rows, check_control_derivatives, check_self_occlusion, compare_range_noise, compare_vobs, ee_circle_velocity_commander, ee_pentagon_velocity_commander, ee_random_waypoints_velocity_commander, estimator_metrics, experiment_logger, handeye_calibration_node, handeye_eye_in_hand_node, iso_constants_measure, iso_evidence_logger, iso_safety_monitor, latency_budget, pentagon_qddot_commander, pentagon_torque_commander, qddot_to_torque, rl_policy_commander, ros, scenario_laws, throw_clearance | 6 |
| `utils/launch_support.py` | config | gen_zero_gain_controllers_yaml, ros, sim_config, torque_control_stack.launch | 0 |
| `utils/livelock.py` |  | cbf_safety_filter, cbf_scenarios | 1 |
| `utils/logging_utils.py` | cbf_state_rows, cbf_zones | capsule_overlay_node, cbf_OSCBF_filter, cbf_safety_filter, ee_circle_velocity_commander, ee_pentagon_velocity_commander, ee_random_waypoints_velocity_commander, handeye_calibration_node, handeye_eye_in_hand_node, node_utils, pentagon_qddot_commander, pentagon_torque_commander, real_time_distance, rl_policy_commander, ros | 2 |
| `utils/mask_builder.py` |  | cbf_scenarios, compare_range_noise, compare_vobs, distance_visualization_node, latency_budget, real_time_distance | 1 |
| `utils/math_utils.py` |  | avoidance_math, cbf_utils, ee_circle_velocity_commander, ee_pentagon_velocity_commander, ee_random_waypoints_velocity_commander, experiment_logger, handeye_calibration_node, handeye_eye_in_hand_node, kinematics, pentagon_torque_commander, ros, trajectory | 0 |
| `utils/node_runtime.py` | config, constants | experiment_logger, handeye_eye_in_hand_node, kinematics, pentagon_qddot_commander, qddot_to_torque, ros, trajectory_overlay_node, trajectory_visualization_node | 0 |
| `utils/node_utils.py` | logging_utils, perception_msgs |  | 0 |
| `utils/obstacle_clusters.py` |  | distance_engine, obstacle_track_pipeline, self_shadow | 2 |
| `utils/obstacle_sim.py` |  | cbf_scenarios, compare_vobs, latency_budget, real_time_distance | 0 |
| `utils/obstacle_track_pipeline.py` | obstacle_clusters, obstacle_tracker | approach_probe, cbf_scenarios, compare_vobs, latency_budget, perception_msgs, real_time_distance | 2 |
| `utils/obstacle_tracker.py` |  | approach_probe, obstacle_track_pipeline | 7 |
| `utils/params.py` |  | config, experiment_logger, iso_evidence_logger, kinematics, qddot_to_torque, real_time_distance, trajectory_overlay_node, trajectory_visualization_node | 0 |
| `utils/perception_msgs.py` | distance_engine, obstacle_track_pipeline | cbf_safety_filter, distance_utils, emulated_perception, estimator_metrics, experiment_logger, iso_evidence_logger, iso_safety_monitor, latency_budget, node_utils, real_time_distance | 10 |
| `utils/rate_scaling.py` |  | approach_probe, cbf_safety_filter, cbf_state_rows, distance_engine, real_time_distance | 1 |
| `utils/rl_policy.py` | constants | rl_policy_commander | 2 |
| `utils/ros.py` | __init__, config, constants, kinematics, launch_support, logging_utils, math_utils, node_runtime, trajectory | ee_circle_velocity_commander, ee_pentagon_velocity_commander, ee_random_waypoints_velocity_commander, frame_grabber, handeye_calibration_node, minimal.launch, pentagon_torque_commander, rl_policy_commander, thales.launch, velocity_cbf_control_stack.launch | 2 |
| `utils/ros_setup.py` | avoidance_math | cbf_velocity_filter | 0 |
| `utils/rtd_debug.py` |  |  | 0 |
| `utils/run_metadata.py` |  |  | 1 |
| `utils/self_collision.py` |  | cbf_state_rows | 3 |
| `utils/self_detection.py` |  | compare_vobs, real_time_distance | 2 |
| `utils/self_shadow.py` | obstacle_clusters | cbf_state_rows, distance_engine | 3 |
| `utils/simulation_imports.py` |  | capsule_overlay_node | 0 |
| `utils/singularity_recovery.py` |  | pentagon_qddot_commander | 1 |
| `utils/state_governor.py` | cbf_hard_limits, cbf_zones | cbf_safety_filter | 3 |
| `utils/static_scene.py` |  |  | 2 |
| `utils/surface_velocity.py` |  |  | 1 |
| `utils/tf_manager.py` | distance_utils | approach_probe, cbf_scenarios, compare_range_noise, compare_vobs, distance_visualization_node, iso_constants_measure, latency_budget, real_time_distance | 0 |
| `utils/timing_law.py` |  | pentagon_qddot_commander | 0 |
| `utils/trajectory.py` | math_utils | ee_pentagon_velocity_commander, ee_random_waypoints_velocity_commander, handeye_calibration_node, pentagon_qddot_commander, pentagon_torque_commander, ros | 0 |
| `utils/trajectory_overlay.py` |  | trajectory_overlay_node | 1 |
| `utils/trajectory_trace.py` |  | gz_trajectory_markers, trajectory_overlay_node, trajectory_visualization_node | 2 |
| `utils/visualization.py` |  | distance_visualization_node, real_time_distance | 0 |
| `simexp/__init__.py` |  | check_self_occlusion, emulated_perception, gz_cbf_stack.launch, gz_circle_demo.launch, gz_plant.launch, run_gz_scenarios | 19 |
| `simexp/analysis.py` |  | attribution, batch, check_self_occlusion, closed_loop, control_regression, estimator_metrics, runner | 4 |
| `simexp/attribution.py` | analysis, estimator_metrics | batch | 1 |
| `simexp/batch.py` | analysis, attribution, closed_loop, estimator_metrics, runner, scenario_catalog, sim_config | run_gz_scenarios | 2 |
| `simexp/closed_loop.py` | analysis, reflex_checks | batch, control_regression | 1 |
| `simexp/conditions.py` |  | estimator_metrics, gz_cbf_stack.launch, offline | 2 |
| `simexp/control_regression.py` | analysis, closed_loop |  | 1 |
| `simexp/emulation.py` | distance_engine | emulated_perception, estimator_metrics, offline | 2 |
| `simexp/estimator_metrics.py` | analysis, conditions, emulation, kinematics, offline, perception_msgs | attribution, batch | 1 |
| `simexp/graph_contract.py` |  |  | 1 |
| `simexp/obstacle_laws.py` |  | scenario_catalog | 2 |
| `simexp/offline.py` | conditions, emulation | estimator_metrics, report | 1 |
| `simexp/procs.py` |  | runner | 2 |
| `simexp/provenance.py` | __init__, cbf_diagnostics |  | 1 |
| `simexp/reflex_checks.py` | cbf_hard_limits, cbf_state_rows, config | closed_loop, reflex_monitor | 1 |
| `simexp/report.py` | offline, scenario_catalog, sim_config | run_gz_scenarios | 1 |
| `simexp/runner.py` | analysis, procs | batch | 3 |
| `simexp/scenario_catalog.py` | obstacle_laws, truth_geometry | batch, gz_cbf_stack.launch, report | 5 |
| `simexp/scenario_laws.py` | kinematics | scenario_commander | 1 |
| `simexp/sim_config.py` | config, launch_support | batch, episode_supervisor, gz_cbf_stack.launch, gz_circle_demo.launch, gz_plant.launch, report, run_gz_scenarios | 7 |
| `simexp/truth_geometry.py` | distance_utils | emulated_perception, ground_truth_node, gz_cbf_stack.launch, scenario_catalog | 2 |

## Reachability classes for library modules

| Module | entry | sim-exec | launch-file | script | tests only | none |
|---|:-:|:-:|:-:|:-:|:-:|:-:|
| `nodes/__init__.py` |  |  |  |  |  | ● |
| `nodes/capsule_overlay_node.py` | ● |  |  |  |  |  |
| `nodes/cbf_OSCBF_filter.py` | ● |  |  |  |  |  |
| `nodes/cbf_safety_filter.py` | ● |  |  | ● |  |  |
| `nodes/cbf_velocity_filter.py` | ● |  |  |  |  |  |
| `nodes/distance_visualization_node.py` |  |  |  |  |  | ● |
| `nodes/ee_circle_velocity_commander.py` | ● |  |  |  |  |  |
| `nodes/ee_pentagon_velocity_commander.py` | ● |  |  |  |  |  |
| `nodes/ee_random_waypoints_velocity_commander.py` | ● |  |  |  |  |  |
| `nodes/experiment_logger.py` | ● |  | ● |  |  |  |
| `nodes/frame_grabber.py` | ● |  |  |  |  |  |
| `nodes/handeye_calibration_node.py` | ● |  |  |  |  |  |
| `nodes/handeye_eye_in_hand_node.py` | ● |  |  |  |  |  |
| `nodes/image_rectifier_node.py` | ● |  |  |  |  |  |
| `nodes/iso_evidence_logger.py` | ● |  |  |  |  |  |
| `nodes/iso_safety_monitor.py` | ● |  |  |  |  |  |
| `nodes/pentagon_qddot_commander.py` | ● |  |  |  |  |  |
| `nodes/pentagon_torque_commander.py` | ● |  |  |  |  |  |
| `nodes/qddot_to_torque.py` | ● |  |  |  |  |  |
| `nodes/real_time_distance.py` | ● |  |  | ● |  |  |
| `nodes/rl_policy_commander.py` | ● |  |  |  |  |  |
| `nodes/tag_overlay_node.py` | ● |  |  |  |  |  |
| `nodes/trajectory_overlay_node.py` | ● |  |  |  |  |  |
| `nodes/trajectory_visualization_node.py` | ● |  |  |  |  |  |
| `utils/__init__.py` | ● |  | ● | ● |  |  |
| `utils/avoidance.py` |  |  |  |  | ● |  |
| `utils/avoidance_math.py` | ● |  |  |  |  |  |
| `utils/ballistic_prediction.py` | ● |  |  | ● |  |  |
| `utils/calibration_check.py` | ● |  |  | ● |  |  |
| `utils/camera_yaml.py` | ● |  |  |  |  |  |
| `utils/cbf_constraints.py` |  |  |  |  |  | ● |
| `utils/cbf_diagnostics.py` | ● |  |  | ● |  |  |
| `utils/cbf_evasion.py` | ● | ● | ● | ● |  |  |
| `utils/cbf_hard_limits.py` | ● | ● |  | ● |  |  |
| `utils/cbf_kinematics.py` | ● |  |  |  |  |  |
| `utils/cbf_qp.py` |  |  |  |  |  | ● |
| `utils/cbf_qp_assembly.py` | ● |  |  | ● |  |  |
| `utils/cbf_singularity.py` | ● | ● | ● | ● |  |  |
| `utils/cbf_state_rows.py` | ● | ● | ● | ● |  |  |
| `utils/cbf_utils.py` | ● |  | ● |  |  |  |
| `utils/cbf_zones.py` | ● | ● | ● | ● |  |  |
| `utils/config.py` | ● | ● | ● | ● |  |  |
| `utils/constants.py` | ● | ● | ● | ● |  |  |
| `utils/distance_engine.py` | ● | ● | ● | ● |  |  |
| `utils/distance_utils.py` | ● | ● | ● | ● |  |  |
| `utils/evasion_direction.py` | ● | ● | ● | ● |  |  |
| `utils/go_around.py` | ● |  |  | ● |  |  |
| `utils/handeye_solver.py` | ● |  |  |  |  |  |
| `utils/iso_ssm.py` | ● | ● | ● | ● |  |  |
| `utils/kinematics.py` | ● | ● | ● | ● |  |  |
| `utils/launch_support.py` | ● | ● | ● | ● |  |  |
| `utils/livelock.py` | ● |  |  | ● |  |  |
| `utils/logging_utils.py` | ● |  | ● | ● |  |  |
| `utils/mask_builder.py` | ● |  |  | ● |  |  |
| `utils/math_utils.py` | ● | ● | ● | ● |  |  |
| `utils/node_runtime.py` | ● | ● | ● | ● |  |  |
| `utils/node_utils.py` |  |  |  |  |  | ● |
| `utils/obstacle_clusters.py` | ● | ● | ● | ● |  |  |
| `utils/obstacle_sim.py` | ● |  |  | ● |  |  |
| `utils/obstacle_track_pipeline.py` | ● | ● | ● | ● |  |  |
| `utils/obstacle_tracker.py` | ● | ● | ● | ● |  |  |
| `utils/params.py` | ● | ● | ● | ● |  |  |
| `utils/perception_msgs.py` | ● | ● | ● | ● |  |  |
| `utils/rate_scaling.py` | ● | ● | ● | ● |  |  |
| `utils/rl_policy.py` | ● |  |  |  |  |  |
| `utils/ros.py` | ● |  | ● |  |  |  |
| `utils/ros_setup.py` | ● |  |  |  |  |  |
| `utils/rtd_debug.py` |  |  |  |  |  | ● |
| `utils/run_metadata.py` |  |  |  |  | ● |  |
| `utils/self_collision.py` | ● | ● | ● | ● |  |  |
| `utils/self_detection.py` | ● |  |  | ● |  |  |
| `utils/self_shadow.py` | ● | ● | ● | ● |  |  |
| `utils/simulation_imports.py` | ● |  |  |  |  |  |
| `utils/singularity_recovery.py` | ● |  |  |  |  |  |
| `utils/state_governor.py` | ● |  |  | ● |  |  |
| `utils/static_scene.py` |  |  |  |  | ● |  |
| `utils/surface_velocity.py` |  |  |  |  | ● |  |
| `utils/tf_manager.py` | ● |  |  | ● |  |  |
| `utils/timing_law.py` | ● |  |  |  |  |  |
| `utils/trajectory.py` | ● |  | ● |  |  |  |
| `utils/trajectory_overlay.py` | ● |  |  |  |  |  |
| `utils/trajectory_trace.py` | ● | ● |  |  |  |  |
| `utils/visualization.py` | ● |  |  | ● |  |  |
| `simexp/__init__.py` |  | ● | ● |  |  |  |
| `simexp/analysis.py` |  | ● |  |  |  |  |
| `simexp/attribution.py` |  | ● |  |  |  |  |
| `simexp/batch.py` |  | ● |  |  |  |  |
| `simexp/closed_loop.py` |  | ● |  |  |  |  |
| `simexp/conditions.py` |  | ● | ● |  |  |  |
| `simexp/control_regression.py` |  |  |  |  | ● |  |
| `simexp/emulation.py` |  | ● |  |  |  |  |
| `simexp/estimator_metrics.py` |  | ● |  |  |  |  |
| `simexp/graph_contract.py` |  |  |  |  | ● |  |
| `simexp/obstacle_laws.py` |  | ● | ● |  |  |  |
| `simexp/offline.py` |  | ● |  |  |  |  |
| `simexp/procs.py` |  | ● |  |  |  |  |
| `simexp/provenance.py` |  |  |  |  | ● |  |
| `simexp/reflex_checks.py` |  | ● |  |  |  |  |
| `simexp/report.py` |  | ● |  |  |  |  |
| `simexp/runner.py` |  | ● |  |  |  |  |
| `simexp/scenario_catalog.py` |  | ● | ● |  |  |  |
| `simexp/scenario_laws.py` |  | ● |  |  |  |  |
| `simexp/sim_config.py` |  | ● | ● |  |  |  |
| `simexp/truth_geometry.py` |  | ● | ● |  |  |  |

**Library modules reached by no executable (candidates only — Stage 7 confirms):** `__init__.py`, `distance_visualization_node.py`, `avoidance.py` (tests), `cbf_constraints.py`, `cbf_qp.py`, `node_utils.py`, `rtd_debug.py`, `run_metadata.py` (tests), `static_scene.py` (tests), `surface_velocity.py` (tests), `control_regression.py` (tests), `graph_contract.py` (tests), `provenance.py` (tests)

## External (non-internal) imports per file

| File | Top-level packages imported |
|---|---|
| `franka_experiments/nodes/capsule_overlay_node.py` | ament_index_python, cv2, cv_bridge, numpy, pinocchio, rclpy, sensor_msgs, yaml |
| `franka_experiments/nodes/cbf_OSCBF_filter.py` | ament_index_python, franka_msgs, numpy, pinocchio, qpsolvers, rclpy, sensor_msgs, std_msgs, yaml |
| `franka_experiments/nodes/cbf_safety_filter.py` | franka_msgs, numpy, osqp, pinocchio, rclpy, scipy, sensor_msgs, std_msgs |
| `franka_experiments/nodes/cbf_velocity_filter.py` | franka_msgs, numpy, pinocchio, qpsolvers, rclpy, sensor_msgs, std_msgs |
| `franka_experiments/nodes/distance_visualization_node.py` | ament_index_python, cv2, cv_bridge, franka_msgs, numpy, rclpy, sensor_msgs, tf2_ros, trimesh |
| `franka_experiments/nodes/ee_circle_velocity_commander.py` | numpy, rclpy, std_msgs |
| `franka_experiments/nodes/ee_pentagon_velocity_commander.py` | numpy, rclpy, std_msgs |
| `franka_experiments/nodes/ee_random_waypoints_velocity_commander.py` | numpy, rclpy, std_msgs |
| `franka_experiments/nodes/experiment_logger.py` | franka_msgs, matplotlib, numpy, pinocchio, rclpy, scipy, sensor_msgs, std_msgs |
| `franka_experiments/nodes/frame_grabber.py` | cv2, cv_bridge, rclpy, sensor_msgs, std_msgs |
| `franka_experiments/nodes/handeye_calibration_node.py` | ament_index_python, apriltag_msgs, cv2, geometry_msgs, numpy, rclpy, scipy, std_msgs, tf2_ros, yaml |
| `franka_experiments/nodes/handeye_eye_in_hand_node.py` | ament_index_python, apriltag_msgs, numpy, pinocchio, rclpy, scipy, std_msgs, tf2_ros, yaml |
| `franka_experiments/nodes/image_rectifier_node.py` | cv2, cv_bridge, numpy, rclpy, sensor_msgs |
| `franka_experiments/nodes/iso_evidence_logger.py` | franka_msgs, numpy, pinocchio, rclpy, sensor_msgs, std_msgs |
| `franka_experiments/nodes/iso_safety_monitor.py` | franka_msgs, numpy, pinocchio, rclpy, sensor_msgs, std_msgs, std_srvs |
| `franka_experiments/nodes/pentagon_qddot_commander.py` | geometry_msgs, numpy, pinocchio, rclpy, sensor_msgs, std_msgs |
| `franka_experiments/nodes/pentagon_torque_commander.py` | numpy, pinocchio, rclpy, sensor_msgs, std_msgs |
| `franka_experiments/nodes/qddot_to_torque.py` | numpy, pinocchio, rclpy, sensor_msgs, std_msgs |
| `franka_experiments/nodes/real_time_distance.py` | ament_index_python, cv2, cv_bridge, franka_msgs, numpy, rclpy, sensor_msgs, tf2_ros, trimesh, yaml |
| `franka_experiments/nodes/rl_policy_commander.py` | franka_msgs, geometry_msgs, numpy, onnxruntime, pinocchio, rclpy, sensor_msgs, std_msgs |
| `franka_experiments/nodes/tag_overlay_node.py` | apriltag_msgs, cv2, cv_bridge, numpy, rclpy, sensor_msgs |
| `franka_experiments/nodes/trajectory_overlay_node.py` | ament_index_python, cv2, cv_bridge, geometry_msgs, numpy, rclpy, sensor_msgs |
| `franka_experiments/nodes/trajectory_visualization_node.py` | geometry_msgs, rclpy, visualization_msgs |
| `franka_experiments/utils/avoidance.py` | numpy |
| `franka_experiments/utils/avoidance_math.py` | geometry_msgs, numpy, pinocchio, std_msgs, visualization_msgs |
| `franka_experiments/utils/ballistic_prediction.py` | numpy |
| `franka_experiments/utils/calibration_check.py` | numpy |
| `franka_experiments/utils/cbf_constraints.py` | numpy |
| `franka_experiments/utils/cbf_diagnostics.py` | numpy |
| `franka_experiments/utils/cbf_evasion.py` | numpy |
| `franka_experiments/utils/cbf_hard_limits.py` | numpy |
| `franka_experiments/utils/cbf_qp.py` | numpy, qpsolvers |
| `franka_experiments/utils/cbf_qp_assembly.py` | numpy, osqp, scipy |
| `franka_experiments/utils/cbf_singularity.py` | numpy, pinocchio |
| `franka_experiments/utils/cbf_state_rows.py` | numpy, pinocchio |
| `franka_experiments/utils/cbf_utils.py` | ament_index_python, numpy, yaml |
| `franka_experiments/utils/cbf_zones.py` | numpy |
| `franka_experiments/utils/config.py` | ament_index_python, numpy, yaml |
| `franka_experiments/utils/distance_engine.py` | numpy |
| `franka_experiments/utils/distance_utils.py` | numpy, yaml |
| `franka_experiments/utils/evasion_direction.py` | numpy |
| `franka_experiments/utils/go_around.py` | numpy |
| `franka_experiments/utils/handeye_solver.py` | numpy, scipy |
| `franka_experiments/utils/iso_ssm.py` | numpy |
| `franka_experiments/utils/kinematics.py` | ament_index_python, numpy, pinocchio, rclpy, sensor_msgs |
| `franka_experiments/utils/launch_support.py` | ament_index_python, launch, yaml |
| `franka_experiments/utils/livelock.py` | numpy |
| `franka_experiments/utils/logging_utils.py` | numpy |
| `franka_experiments/utils/mask_builder.py` | cv2, numpy |
| `franka_experiments/utils/math_utils.py` | numpy |
| `franka_experiments/utils/node_runtime.py` | ament_index_python, numpy, rclpy, yaml |
| `franka_experiments/utils/node_utils.py` | franka_msgs, geometry_msgs, numpy |
| `franka_experiments/utils/obstacle_clusters.py` | numpy, scipy |
| `franka_experiments/utils/obstacle_sim.py` | numpy |
| `franka_experiments/utils/obstacle_track_pipeline.py` | numpy |
| `franka_experiments/utils/obstacle_tracker.py` | numpy |
| `franka_experiments/utils/params.py` | rclpy |
| `franka_experiments/utils/perception_msgs.py` | franka_msgs, geometry_msgs, numpy |
| `franka_experiments/utils/rl_policy.py` | numpy, yaml |
| `franka_experiments/utils/ros.py` | numpy, rclpy, yaml |
| `franka_experiments/utils/ros_setup.py` | moveit_msgs, numpy, pinocchio, rcl_interfaces, rclpy, sensor_msgs |
| `franka_experiments/utils/rtd_debug.py` | numpy |
| `franka_experiments/utils/run_metadata.py` | numpy |
| `franka_experiments/utils/self_collision.py` | numpy |
| `franka_experiments/utils/self_detection.py` | numpy |
| `franka_experiments/utils/self_shadow.py` | numpy, scipy |
| `franka_experiments/utils/simulation_imports.py` | ament_index_python, utils |
| `franka_experiments/utils/singularity_recovery.py` | numpy |
| `franka_experiments/utils/state_governor.py` | numpy |
| `franka_experiments/utils/static_scene.py` | numpy |
| `franka_experiments/utils/surface_velocity.py` | numpy |
| `franka_experiments/utils/tf_manager.py` | numpy, rclpy, tf2_ros |
| `franka_experiments/utils/trajectory.py` | numpy |
| `franka_experiments/utils/trajectory_overlay.py` | cv2, numpy |
| `franka_experiments/utils/trajectory_trace.py` | geometry_msgs, numpy, visualization_msgs |
| `franka_experiments/utils/visualization.py` | cv2, numpy |
| `launch/bag_replay.launch.py` | ament_index_python, launch, launch_ros, yaml |
| `launch/handeye_calibration_bringup.launch.py` | launch, launch_ros |
| `launch/handeye_eye_in_hand_calibration.launch.py` | launch, launch_ros, yaml |
| `launch/minimal.launch.py` | launch, launch_ros, yaml |
| `launch/thales.launch.py` | launch, launch_ros, yaml |
| `launch/torque_control_stack.launch.py` | launch, launch_ros, yaml |
| `launch/velocity_cbf_control_stack.launch.py` | launch, launch_ros, yaml |
| `scripts/analyze_isolation_ab.py` | numpy |
| `scripts/approach_envelope.py` | cbf_scenarios, numpy |
| `scripts/approach_probe.py` | ball_throw_eval, compare_vobs, cv_bridge, numpy, rclpy, rosidl_runtime_py, yaml |
| `scripts/bag_to_mp4.py` | cv2, numpy, rclpy, rosbag2_py, rosidl_runtime_py, sensor_msgs |
| `scripts/ball_closed_loop.py` | ball_throw_eval, builtin_interfaces, franka_msgs, numpy, pinocchio, rclpy, sensor_msgs, std_msgs, yaml |
| `scripts/ball_throw_eval.py` | numpy, rclpy, rosidl_runtime_py, scipy, yaml |
| `scripts/cbf_phase4_check.py` | franka_msgs, numpy, rclpy, std_msgs |
| `scripts/cbf_scenarios.py` | ament_index_python, compare_vobs, cv_bridge, numpy, osqp, pinocchio, scipy, trimesh, yaml |
| `scripts/check_control_derivatives.py` | numpy, pinocchio, yaml |
| `scripts/compare_range_noise.py` | ament_index_python, compare_vobs, cv_bridge, numpy, pinocchio, range_noise_calibration, trimesh |
| `scripts/compare_vobs.py` | ament_index_python, cv_bridge, matplotlib, numpy, pinocchio, rclpy, rosidl_runtime_py, tf2_ros, trimesh |
| `scripts/iso_constants_measure.py` | ament_index_python, cv_bridge, numpy, pinocchio, rclpy, sensor_msgs, std_msgs, tf2_ros, trimesh, yaml |
| `scripts/iso_preflight_check.py` | rclpy, std_msgs, yaml |
| `scripts/latency_budget.py` | ament_index_python, compare_vobs, cv_bridge, franka_msgs, numpy, osqp, pinocchio, rclpy, realsense2_camera_msgs, scipy, sensor_msgs, std_msgs, trimesh |
| `scripts/pass_plot.py` | ball_throw_eval, matplotlib, numpy, smoothness_report |
| `scripts/pipeline_rate_monitor.py` | matplotlib, psutil, rclpy, rosidl_runtime_py |
| `scripts/plot_rl_timing.py` | matplotlib, numpy |
| `scripts/range_noise_calibration.py` | cv_bridge, numpy, rclpy, sensor_msgs |
| `scripts/smoothness_report.py` | ball_throw_eval, numpy |
| `scripts/throw_actuation.py` | ball_throw_eval, numpy |
| `scripts/throw_clearance.py` | ball_throw_eval, numpy, pinocchio |
| `scripts/throw_lead.py` | ball_closed_loop, numpy |
| `scripts/watch_obstacle_tracks.py` | franka_msgs, numpy, rclpy |
| `setup.py` | setuptools |
| `launch/gz_fr3_plant.launch.py` | ament_index_python, launch, launch_ros, utils |
| `launch/gz_cbf_stack.launch.py` | ament_index_python, launch, launch_ros, yaml |
| `launch/gz_circle_demo.launch.py` | ament_index_python, launch, launch_ros |
| `launch/gz_plant.launch.py` | ament_index_python, launch |
| `scripts/check_self_occlusion.py` | ament_index_python, franka_msgs, numpy, pinocchio, sensor_msgs |
| `scripts/emulated_perception.py` | franka_msgs, geometry_msgs, numpy, rclpy, sensor_msgs, yaml |
| `scripts/episode_supervisor.py` | controller_manager_msgs, franka_msgs, rclpy, std_msgs |
| `scripts/ground_truth_node.py` | geometry_msgs, numpy, rclpy, sensor_msgs, std_msgs |
| `scripts/gz_trajectory_markers.py` | geometry_msgs, rclpy |
| `scripts/plot_control_regression.py` | matplotlib, numpy |
| `scripts/reflex_monitor.py` | numpy, pinocchio, rclpy, rosgraph_msgs, sensor_msgs, std_msgs |
| `scripts/run_gz_scenarios.py` | yaml |
| `scripts/scenario_commander.py` | geometry_msgs, numpy, rclpy, sensor_msgs, std_msgs |
| `scripts/simexp/analysis.py` | franka_msgs, numpy, rclpy, sensor_msgs, std_msgs |
| `scripts/simexp/attribution.py` | approach_probe, numpy |
| `scripts/simexp/batch.py` | numpy |
| `scripts/simexp/closed_loop.py` | franka_msgs, numpy, std_msgs |
| `scripts/simexp/conditions.py` | ament_index_python, yaml |
| `scripts/simexp/control_regression.py` | geometry_msgs, numpy, std_msgs |
| `scripts/simexp/emulation.py` | cbf_scenarios, geometry_msgs, numpy |
| `scripts/simexp/estimator_metrics.py` | ament_index_python, ball_throw_eval, compare_vobs, franka_msgs, geometry_msgs, numpy, pinocchio, sensor_msgs, yaml |
| `scripts/simexp/graph_contract.py` | rclpy, tf2_msgs |
| `scripts/simexp/obstacle_laws.py` | numpy |
| `scripts/simexp/reflex_checks.py` | numpy |
| `scripts/simexp/report.py` | numpy |
| `scripts/simexp/scenario_catalog.py` | ament_index_python, numpy, yaml |
| `scripts/simexp/scenario_laws.py` | numpy, pinocchio |
| `scripts/simexp/sim_config.py` | ament_index_python, utils, yaml |
| `scripts/simexp/truth_geometry.py` | ament_index_python, numpy, pinocchio, yaml |

**Third-party / ROS packages by number of importing files:** numpy (220), pytest (81), yaml (56), rclpy (56), sensor_msgs (35), ament_index_python (35), std_msgs (34), pinocchio (32), franka_msgs (27), _cbf_builder_harness (25), geometry_msgs (22), scipy (21), launch (17), cv_bridge (14), launch_ros (13), cv2 (12), utils (8), builtin_interfaces (8), trimesh (7), tf2_ros (7), ball_throw_eval (7), matplotlib (6), qpsolvers (5), osqp (5), rosidl_runtime_py (5), compare_vobs (5), visualization_msgs (4), apriltag_msgs (3), range_noise_calibration (3), onnxruntime (2), rcl_interfaces (2), cbf_scenarios (2), approach_probe (2), std_srvs (1), moveit_msgs (1), rosbag2_py (1), realsense2_camera_msgs (1), smoothness_report (1), psutil (1), ball_closed_loop (1), setuptools (1), compare_range_noise (1), test_iso_status_contract (1), _launch_snapshot (1), controller_manager_msgs (1), rosgraph_msgs (1), tf2_msgs (1), launch_testing (1), _rosprobe (1)
