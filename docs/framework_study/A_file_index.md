# Appendix A — File index (draft, Stage 0)

> Inverse view of [ledger/coverage_ledger.md](ledger/coverage_ledger.md): for each chapter, the files it must explain. Assignments are **provisional** (rule-based from file names, Stage 0). A file appears under its *primary* chapter only; cross-references between chapters are added as each chapter is written. Tests are listed under the chapter of the subsystem they specify. Chapter files that do not exist yet are marked *(not written)*.

Roles: **node** = ROS executable; **util** = library module; **sim** = simulation package; **ext** = executed-path file of a workspace dependency.


## 01 — Architecture → `01_architecture.md` *(not written)*

7 files, 3567 lines.

| File | Role | Lines |
|---|---|---:|
| `franka_experiments/launch/bag_replay.launch.py` | launch file | 201 |
| `franka_experiments/launch/handeye_calibration_bringup.launch.py` | launch file | 166 |
| `franka_experiments/launch/handeye_eye_in_hand_calibration.launch.py` | launch file | 335 |
| `franka_experiments/launch/minimal.launch.py` | launch file | 526 |
| `franka_experiments/launch/thales.launch.py` | launch file | 371 |
| `franka_experiments/launch/torque_control_stack.launch.py` | launch file | 1527 |
| `franka_experiments/launch/velocity_cbf_control_stack.launch.py` | launch file | 441 |

## 02 — ROS graph & data flow → `02_ros_graph_and_dataflow.md` *(not written)*

4 files, 246 lines.

| File | Role | Lines |
|---|---|---:|
| `franka_msgs/msg/HumanRobotDistance.msg` | message definition (external) | 30 |
| `franka_msgs/msg/LinkDistance.msg` | message definition (external) | 186 |
| `franka_msgs/msg/MultiDistance.msg` | message definition (external) | 19 |
| `franka_msgs/msg/MultiLinkDistance.msg` | message definition (external) | 11 |

## 03 — Configuration → `03_configuration.md` *(not written)*

16 files, 6415 lines.

| File | Role | Lines |
|---|---|---:|
| `franka_experiments/config/fake_hw_controller_params.yaml` | config (parameters) | 7 |
| `franka_experiments/config/fr3_complete.yaml` | config (parameters) | 930 |
| `franka_experiments/config/fr3_control.yaml` | config (parameters) | 2672 |
| `franka_experiments/config/fr3_distance.yaml` | config (parameters) | 23 |
| `franka_experiments/config/launch_defaults.yaml` | config (parameters) | 333 |
| `franka_experiments/config/oscbf_params.yaml` | config (parameters) | 63 |
| `franka_experiments/franka_experiments/__init__.py` | package init | 0 |
| `franka_experiments/franka_experiments/utils/__init__.py` | util module | 6 |
| `franka_experiments/franka_experiments/utils/config.py` | util module | 806 |
| `franka_experiments/franka_experiments/utils/launch_support.py` | util module | 589 |
| `franka_experiments/franka_experiments/utils/node_runtime.py` | util module | 170 |
| `franka_experiments/franka_experiments/utils/node_utils.py` | util module | 42 |
| `franka_experiments/franka_experiments/utils/params.py` | util module | 293 |
| `franka_experiments/franka_experiments/utils/ros.py` | util module | 245 |
| `franka_experiments/franka_experiments/utils/ros_setup.py` | util module | 200 |
| `franka_experiments/franka_experiments/utils/run_metadata.py` | util module | 36 |

## 04 — Mathematical foundations → `04_math_foundations.md` *(not written)*

5 files, 852 lines.

| File | Role | Lines |
|---|---|---:|
| `franka_experiments/franka_experiments/utils/cbf_kinematics.py` | util module | 9 |
| `franka_experiments/franka_experiments/utils/constants.py` | util module | 18 |
| `franka_experiments/franka_experiments/utils/kinematics.py` | util module | 665 |
| `franka_experiments/franka_experiments/utils/math_utils.py` | util module | 87 |
| `franka_experiments/scripts/check_control_derivatives.py` | offline/aux script | 73 |

## 05 — Perception & distance → `05_perception_and_distance.md` *(not written)*

59 files, 18808 lines.

| File | Role | Lines |
|---|---|---:|
| `franka_experiments/config/camera_EE_extrinsic.yaml` | config (calibration) | 51 |
| `franka_experiments/config/camera_extrinsics.yaml` | config (calibration) | 11 |
| `franka_experiments/config/camera_intrinsics.yaml` | config (calibration) | 56 |
| `franka_experiments/config/camera_link_extrinsics.yaml` | config (calibration) | 16 |
| `franka_experiments/config/dataset_handeye_eye_in_hand.yaml` | config (calibration) | 310 |
| `franka_experiments/config/depth_intrinsics.yaml` | config (calibration) | 56 |
| `franka_experiments/config/ee_tag_extrinsics.yaml` | config (calibration) | 11 |
| `franka_experiments/config/handeye_eye_in_hand_rviz.rviz` | config (calibration) | 120 |
| `franka_experiments/config/handeye_rviz.rviz` | config (calibration) | 150 |
| `franka_experiments/config/rgb_intrinsics.yaml` | config (calibration) | 56 |
| `franka_experiments/franka_experiments/nodes/frame_grabber.py` | ROS node | 155 |
| `franka_experiments/franka_experiments/nodes/handeye_calibration_node.py` | ROS node | 2561 |
| `franka_experiments/franka_experiments/nodes/handeye_eye_in_hand_node.py` | ROS node | 962 |
| `franka_experiments/franka_experiments/nodes/image_rectifier_node.py` | ROS node | 107 |
| `franka_experiments/franka_experiments/nodes/real_time_distance.py` | ROS node | 1255 |
| `franka_experiments/franka_experiments/utils/ballistic_prediction.py` | util module | 202 |
| `franka_experiments/franka_experiments/utils/calibration_check.py` | util module | 235 |
| `franka_experiments/franka_experiments/utils/camera_yaml.py` | util module | 17 |
| `franka_experiments/franka_experiments/utils/distance_engine.py` | util module | 1014 |
| `franka_experiments/franka_experiments/utils/distance_utils.py` | util module | 307 |
| `franka_experiments/franka_experiments/utils/handeye_solver.py` | util module | 422 |
| `franka_experiments/franka_experiments/utils/mask_builder.py` | util module | 312 |
| `franka_experiments/franka_experiments/utils/obstacle_clusters.py` | util module | 376 |
| `franka_experiments/franka_experiments/utils/obstacle_sim.py` | util module | 153 |
| `franka_experiments/franka_experiments/utils/obstacle_track_pipeline.py` | util module | 312 |
| `franka_experiments/franka_experiments/utils/obstacle_tracker.py` | util module | 1048 |
| `franka_experiments/franka_experiments/utils/perception_msgs.py` | util module | 484 |
| `franka_experiments/franka_experiments/utils/rtd_debug.py` | util module | 370 |
| `franka_experiments/franka_experiments/utils/self_detection.py` | util module | 226 |
| `franka_experiments/franka_experiments/utils/self_shadow.py` | util module | 685 |
| `franka_experiments/franka_experiments/utils/static_scene.py` | util module | 505 |
| `franka_experiments/franka_experiments/utils/surface_velocity.py` | util module | 279 |
| `franka_experiments/franka_experiments/utils/tf_manager.py` | util module | 223 |
| `franka_experiments/test/test_ballistic_prediction.py` | test | 108 |
| `franka_experiments/test/test_calibration_check.py` | test | 186 |
| `franka_experiments/test/test_cbf_obstacle_identity.py` | test | 251 |
| `franka_experiments/test/test_cbf_sensor_range_uncertainty.py` | test | 227 |
| `franka_experiments/test/test_compare_range_noise.py` | test | 121 |
| `franka_experiments/test/test_cp_label_agreement.py` | test | 99 |
| `franka_experiments/test/test_distance_surface_semantics.py` | test | 355 |
| `franka_experiments/test/test_handeye_solver.py` | test | 209 |
| `franka_experiments/test/test_link_distance_track_fields.py` | test | 125 |
| `franka_experiments/test/test_mask_depth_gate.py` | test | 338 |
| `franka_experiments/test/test_multi_obstacle_rows.py` | test | 294 |
| `franka_experiments/test/test_obstacle_clusters.py` | test | 263 |
| `franka_experiments/test/test_obstacle_imm.py` | test | 270 |
| `franka_experiments/test/test_obstacle_kalman.py` | test | 257 |
| `franka_experiments/test/test_obstacle_scene_guard.py` | test | 111 |
| `franka_experiments/test/test_obstacle_track_pipeline.py` | test | 267 |
| `franka_experiments/test/test_obstacle_tracker.py` | test | 363 |
| `franka_experiments/test/test_range_noise_calibration.py` | test | 123 |
| `franka_experiments/test/test_self_detection.py` | test | 213 |
| `franka_experiments/test/test_self_reject_and_obstacle_floor.py` | test | 179 |
| `franka_experiments/test/test_self_shadow_derivatives.py` | test | 302 |
| `franka_experiments/test/test_self_shadow_geometry.py` | test | 237 |
| `franka_experiments/test/test_sensor_range_field.py` | test | 268 |
| `franka_experiments/test/test_ssb_perception.py` | test | 327 |
| `franka_experiments/test/test_track_eviction.py` | test | 70 |
| `franka_experiments/test/test_track_lifecycle.py` | test | 198 |

## 06 — Nominal motion generation → `06_nominal_motion.md` *(not written)*

25 files, 10197 lines.

| File | Role | Lines |
|---|---|---:|
| `franka_experiments/franka_experiments/nodes/ee_circle_velocity_commander.py` | ROS node | 472 |
| `franka_experiments/franka_experiments/nodes/ee_pentagon_velocity_commander.py` | ROS node | 360 |
| `franka_experiments/franka_experiments/nodes/ee_random_waypoints_velocity_commander.py` | ROS node | 537 |
| `franka_experiments/franka_experiments/nodes/pentagon_qddot_commander.py` | ROS node | 1688 |
| `franka_experiments/franka_experiments/nodes/rl_policy_commander.py` | ROS node | 753 |
| `franka_experiments/franka_experiments/utils/avoidance.py` | util module | 165 |
| `franka_experiments/franka_experiments/utils/avoidance_math.py` | util module | 1309 |
| `franka_experiments/franka_experiments/utils/evasion_direction.py` | util module | 264 |
| `franka_experiments/franka_experiments/utils/go_around.py` | util module | 160 |
| `franka_experiments/franka_experiments/utils/livelock.py` | util module | 163 |
| `franka_experiments/franka_experiments/utils/rl_policy.py` | util module | 499 |
| `franka_experiments/franka_experiments/utils/timing_law.py` | util module | 135 |
| `franka_experiments/franka_experiments/utils/trajectory.py` | util module | 887 |
| `franka_experiments/test/smoke_rl_policy_commander.py` | test | 268 |
| `franka_experiments/test/test_avoidance.py` | test | 191 |
| `franka_experiments/test/test_directional_fade.py` | test | 28 |
| `franka_experiments/test/test_evasion_direction.py` | test | 192 |
| `franka_experiments/test/test_fr3_velocity_envelope.py` | test | 186 |
| `franka_experiments/test/test_go_around.py` | test | 64 |
| `franka_experiments/test/test_livelock.py` | test | 218 |
| `franka_experiments/test/test_phase_governor.py` | test | 195 |
| `franka_experiments/test/test_phase_governor_staging.py` | test | 97 |
| `franka_experiments/test/test_rl_policy.py` | test | 772 |
| `franka_experiments/test/test_trajectory_overlay.py` | test | 290 |
| `franka_experiments/test/test_trajectory_trace.py` | test | 304 |

## 07 — CBF obstacle barrier → `07_cbf_obstacle_barrier.md` *(not written)*

30 files, 12376 lines.

| File | Role | Lines |
|---|---|---:|
| `franka_experiments/franka_experiments/utils/cbf_constraints.py` | util module | 218 |
| `franka_experiments/franka_experiments/utils/cbf_evasion.py` | util module | 306 |
| `franka_experiments/franka_experiments/utils/cbf_state_rows.py` | util module | 4354 |
| `franka_experiments/franka_experiments/utils/cbf_utils.py` | util module | 46 |
| `franka_experiments/franka_experiments/utils/cbf_zones.py` | util module | 356 |
| `franka_experiments/test/test_cbf_curvature_term.py` | test | 346 |
| `franka_experiments/test/test_cbf_diag_line.py` | test | 191 |
| `franka_experiments/test/test_cbf_evidence_trust.py` | test | 457 |
| `franka_experiments/test/test_cbf_fast_track.py` | test | 81 |
| `franka_experiments/test/test_cbf_latency_compensation.py` | test | 174 |
| `franka_experiments/test/test_cbf_lateral_evasion.py` | test | 296 |
| `franka_experiments/test/test_cbf_link_speed.py` | test | 151 |
| `franka_experiments/test/test_cbf_link_speed_relative.py` | test | 39 |
| `franka_experiments/test/test_cbf_normal_rotation_guard.py` | test | 216 |
| `franka_experiments/test/test_cbf_outrun_evasion.py` | test | 173 |
| `franka_experiments/test/test_cbf_phase1_reactivity.py` | test | 286 |
| `franka_experiments/test/test_cbf_retreat_authority.py` | test | 204 |
| `franka_experiments/test/test_cbf_retreat_cap.py` | test | 347 |
| `franka_experiments/test/test_cbf_selfcollision_anticipation.py` | test | 243 |
| `franka_experiments/test/test_cbf_state_rows.py` | test | 828 |
| `franka_experiments/test/test_cbf_static_scene.py` | test | 325 |
| `franka_experiments/test/test_cbf_surface_velocity.py` | test | 561 |
| `franka_experiments/test/test_cbf_tangential_bias.py` | test | 353 |
| `franka_experiments/test/test_cbf_tracker_source.py` | test | 400 |
| `franka_experiments/test/test_cbf_uncertainty_margin.py` | test | 176 |
| `franka_experiments/test/test_cbf_velocity_feedforward.py` | test | 159 |
| `franka_experiments/test/test_cbf_velocity_standoff.py` | test | 81 |
| `franka_experiments/test/test_cbf_vobs_rate_independent.py` | test | 414 |
| `franka_experiments/test/test_cbf_zone_ladder.py` | test | 468 |
| `franka_experiments/test/test_vobs_in_hdot.py` | test | 127 |

## 08 — Constraints & QP → `08_constraints_and_qp.md` *(not written)*

28 files, 8380 lines.

| File | Role | Lines |
|---|---|---:|
| `franka_experiments/franka_experiments/nodes/cbf_safety_filter.py` | ROS node | 2085 |
| `franka_experiments/franka_experiments/utils/cbf_hard_limits.py` | util module | 456 |
| `franka_experiments/franka_experiments/utils/cbf_qp.py` | util module | 114 |
| `franka_experiments/franka_experiments/utils/cbf_qp_assembly.py` | util module | 598 |
| `franka_experiments/franka_experiments/utils/cbf_singularity.py` | util module | 214 |
| `franka_experiments/franka_experiments/utils/rate_scaling.py` | util module | 199 |
| `franka_experiments/franka_experiments/utils/self_collision.py` | util module | 224 |
| `franka_experiments/franka_experiments/utils/singularity_recovery.py` | util module | 205 |
| `franka_experiments/franka_experiments/utils/state_governor.py` | util module | 266 |
| `franka_experiments/test/_cbf_builder_harness.py` | test | 390 |
| `franka_experiments/test/smoke_cbf_construct.py` | test | 110 |
| `franka_experiments/test/smoke_cbf_safety_filter.py` | test | 180 |
| `franka_experiments/test/test_accel_box_decoupling.py` | test | 67 |
| `franka_experiments/test/test_brake_slew_limited.py` | test | 100 |
| `franka_experiments/test/test_cbf_hard_constraints.py` | test | 405 |
| `franka_experiments/test/test_cbf_multi_cp_qp.py` | test | 303 |
| `franka_experiments/test/test_cbf_qp_fallback.py` | test | 199 |
| `franka_experiments/test/test_cbf_risk_weighted_slack.py` | test | 330 |
| `franka_experiments/test/test_cbf_row_l1_slack.py` | test | 129 |
| `franka_experiments/test/test_cbf_row_padding.py` | test | 222 |
| `franka_experiments/test/test_cbf_singularity.py` | test | 198 |
| `franka_experiments/test/test_rate_scaling.py` | test | 388 |
| `franka_experiments/test/test_retreat_excess_speed.py` | test | 42 |
| `franka_experiments/test/test_safety_failure_policy.py` | test | 78 |
| `franka_experiments/test/test_shipped_smoothness_config.py` | test | 54 |
| `franka_experiments/test/test_singularity_recovery.py` | test | 162 |
| `franka_experiments/test/test_ssb_qp_rows.py` | test | 463 |
| `franka_experiments/test/test_state_governor.py` | test | 199 |

## 09 — Torque realization → `09_torque_realization.md` *(not written)*

6 files, 1361 lines.

| File | Role | Lines |
|---|---|---:|
| `franka_experiments/franka_experiments/nodes/qddot_to_torque.py` | ROS node | 317 |
| `franka_experiments/scripts/pin_rt_thread.sh` | shell script | 73 |
| `franka_rt_controllers/config/cbf_torque_controller.yaml` | C++ controller (external, executed) | 6 |
| `franka_rt_controllers/franka_rt_controllers.xml` | C++ controller (external, executed) | 10 |
| `franka_rt_controllers/src/cbf_torque_controller.cpp` | C++ controller (external, executed) | 332 |
| `franka_rt_controllers/src/rt_torque_controller.cpp` | C++ controller (external, executed) | 623 |

## 10 — Alternative controllers → `10_alternative_controllers.md` *(not written)*

8 files, 2950 lines.

| File | Role | Lines |
|---|---|---:|
| `franka_experiments/franka_experiments/nodes/cbf_OSCBF_filter.py` | ROS node | 971 |
| `franka_experiments/franka_experiments/nodes/cbf_velocity_filter.py` | ROS node | 623 |
| `franka_experiments/franka_experiments/nodes/pentagon_torque_commander.py` | ROS node | 496 |
| `franka_rt_controllers/config/rt_velocity_blender.yaml` | C++ controller (external, executed) | 89 |
| `franka_rt_controllers/rt_velocity_blender_controller.xml` | C++ controller (external, executed) | 25 |
| `franka_rt_controllers/src/rt_velocity_blender_controller.cpp` | C++ controller (external, executed) | 424 |
| `franka_experiments/test/test_cbf_velocity_filter.py` | test | 205 |
| `franka_experiments/test/test_torque_stack_external_plant_off_identical.py` | test | 117 |

## 11 — ISO safety layer → `11_iso_safety_layer.md` *(not written)*

18 files, 5111 lines.

| File | Role | Lines |
|---|---|---:|
| `franka_experiments/config/safety/watchman_profile.md` | doc (lead) | 153 |
| `franka_experiments/franka_experiments/nodes/iso_evidence_logger.py` | ROS node | 621 |
| `franka_experiments/franka_experiments/nodes/iso_safety_monitor.py` | ROS node | 417 |
| `franka_experiments/franka_experiments/utils/iso_ssm.py` | util module | 210 |
| `franka_experiments/scripts/iso_constants_measure.py` | offline/aux script | 644 |
| `franka_experiments/scripts/iso_evidence_report.py` | offline/aux script | 678 |
| `franka_experiments/scripts/iso_pfl_speed.py` | offline/aux script | 155 |
| `franka_experiments/scripts/iso_preflight_check.py` | offline/aux script | 254 |
| `franka_experiments/test/test_iso_dsafe_floor.py` | test | 155 |
| `franka_experiments/test/test_iso_evidence_report.py` | test | 246 |
| `franka_experiments/test/test_iso_flags_off_identical.py` | test | 142 |
| `franka_experiments/test/test_iso_monitor.py` | test | 267 |
| `franka_experiments/test/test_iso_perception_failclosed.py` | test | 243 |
| `franka_experiments/test/test_iso_pfl_ceiling.py` | test | 119 |
| `franka_experiments/test/test_iso_speed_rows.py` | test | 206 |
| `franka_experiments/test/test_iso_ssm.py` | test | 158 |
| `franka_experiments/test/test_iso_status_contract.py` | test | 232 |
| `franka_experiments/test/test_iso_torque_limits.py` | test | 211 |

## 12 — Simulation → `12_simulation.md` *(not written)*

63 files, 10429 lines.

| File | Role | Lines |
|---|---|---:|
| `franka_simulation/launch/gz_fr3_plant.launch.py` | launch file (external, executed) | 180 |
| `franka_simulation_experiments/config/conditions.d/mppi_it08.yaml` | config (sim experiment axes) | 57 |
| `franka_simulation_experiments/config/conditions.yaml` | config (sim experiment axes) | 1516 |
| `franka_simulation_experiments/config/control_regression_scenarios.yaml` | config (sim experiment axes) | 29 |
| `franka_simulation_experiments/config/scenarios.yaml` | config (sim experiment axes) | 629 |
| `franka_simulation_experiments/launch/gz_cbf_stack.launch.py` | launch file | 273 |
| `franka_simulation_experiments/launch/gz_circle_demo.launch.py` | launch file | 145 |
| `franka_simulation_experiments/launch/gz_plant.launch.py` | launch file | 66 |
| `franka_simulation_experiments/scripts/check_self_occlusion.py` | sim node / tool | 107 |
| `franka_simulation_experiments/scripts/emulated_perception.py` | sim node / tool | 149 |
| `franka_simulation_experiments/scripts/episode_supervisor.py` | sim node / tool | 224 |
| `franka_simulation_experiments/scripts/ground_truth_node.py` | sim node / tool | 141 |
| `franka_simulation_experiments/scripts/gz_trajectory_markers.py` | sim node / tool | 256 |
| `franka_simulation_experiments/scripts/pin_process.py` | sim node / tool | 72 |
| `franka_simulation_experiments/scripts/plot_control_regression.py` | sim node / tool | 70 |
| `franka_simulation_experiments/scripts/reflex_monitor.py` | sim node / tool | 155 |
| `franka_simulation_experiments/scripts/run_gz_scenarios.py` | sim node / tool | 94 |
| `franka_simulation_experiments/scripts/scenario_commander.py` | sim node / tool | 95 |
| `franka_simulation_experiments/scripts/simexp/__init__.py` | sim library (simexp) | 1 |
| `franka_simulation_experiments/scripts/simexp/analysis.py` | sim library (simexp) | 217 |
| `franka_simulation_experiments/scripts/simexp/attribution.py` | sim library (simexp) | 132 |
| `franka_simulation_experiments/scripts/simexp/batch.py` | sim library (simexp) | 305 |
| `franka_simulation_experiments/scripts/simexp/closed_loop.py` | sim library (simexp) | 292 |
| `franka_simulation_experiments/scripts/simexp/conditions.py` | sim library (simexp) | 135 |
| `franka_simulation_experiments/scripts/simexp/control_regression.py` | sim library (simexp) | 131 |
| `franka_simulation_experiments/scripts/simexp/emulation.py` | sim library (simexp) | 121 |
| `franka_simulation_experiments/scripts/simexp/estimator_metrics.py` | sim library (simexp) | 308 |
| `franka_simulation_experiments/scripts/simexp/graph_contract.py` | sim library (simexp) | 165 |
| `franka_simulation_experiments/scripts/simexp/obstacle_laws.py` | sim library (simexp) | 114 |
| `franka_simulation_experiments/scripts/simexp/offline.py` | sim library (simexp) | 66 |
| `franka_simulation_experiments/scripts/simexp/procs.py` | sim library (simexp) | 105 |
| `franka_simulation_experiments/scripts/simexp/provenance.py` | sim library (simexp) | 49 |
| `franka_simulation_experiments/scripts/simexp/reflex_checks.py` | sim library (simexp) | 149 |
| `franka_simulation_experiments/scripts/simexp/report.py` | sim library (simexp) | 365 |
| `franka_simulation_experiments/scripts/simexp/runner.py` | sim library (simexp) | 131 |
| `franka_simulation_experiments/scripts/simexp/scenario_catalog.py` | sim library (simexp) | 225 |
| `franka_simulation_experiments/scripts/simexp/scenario_laws.py` | sim library (simexp) | 87 |
| `franka_simulation_experiments/scripts/simexp/sim_config.py` | sim library (simexp) | 217 |
| `franka_simulation_experiments/scripts/simexp/truth_geometry.py` | sim library (simexp) | 87 |
| `franka_simulation_experiments/test/conftest.py` | test | 11 |
| `franka_simulation_experiments/test/launch/launch_test_graph_contract.py` | test | 142 |
| `franka_simulation_experiments/test/slow/_rosprobe.py` | test | 97 |
| `franka_simulation_experiments/test/slow/test_phase1_plant.py` | test | 215 |
| `franka_simulation_experiments/test/slow/test_phase2_closed_loop.py` | test | 81 |
| `franka_simulation_experiments/test/slow/test_phase3_perception.py` | test | 176 |
| `franka_simulation_experiments/test/slow/test_phase4_obstacles.py` | test | 128 |
| `franka_simulation_experiments/test/slow/test_phase5_batch.py` | test | 90 |
| `franka_simulation_experiments/test/test_analysis.py` | test | 31 |
| `franka_simulation_experiments/test/test_batch.py` | test | 76 |
| `franka_simulation_experiments/test/test_closed_loop.py` | test | 144 |
| `franka_simulation_experiments/test/test_conditions.py` | test | 173 |
| `franka_simulation_experiments/test/test_control_regression.py` | test | 43 |
| `franka_simulation_experiments/test/test_emulation.py` | test | 102 |
| `franka_simulation_experiments/test/test_estimator_metrics.py` | test | 108 |
| `franka_simulation_experiments/test/test_gz_trajectory_markers.py` | test | 143 |
| `franka_simulation_experiments/test/test_obstacle_law_wander.py` | test | 143 |
| `franka_simulation_experiments/test/test_offline_and_attribution.py` | test | 102 |
| `franka_simulation_experiments/test/test_reflex_checks.py` | test | 76 |
| `franka_simulation_experiments/test/test_run_provenance.py` | test | 28 |
| `franka_simulation_experiments/test/test_scenario_catalog.py` | test | 193 |
| `franka_simulation_experiments/test/test_scenario_laws.py` | test | 92 |
| `franka_simulation_experiments/test/test_sim_config.py` | test | 158 |
| `franka_simulation_experiments/test/test_ssb_scenarios.py` | test | 217 |

## 13 — Observability & tools → `13_observability_and_tools.md` *(not written)*

42 files, 12711 lines.

| File | Role | Lines |
|---|---|---:|
| `franka_experiments/config/trajectory_rviz.rviz` | config (rviz) | 120 |
| `franka_experiments/docs/make_ros_node_graph.py` | doc tool | 298 |
| `franka_experiments/franka_experiments/nodes/__init__.py` | package init | 0 |
| `franka_experiments/franka_experiments/nodes/capsule_overlay_node.py` | ROS node | 796 |
| `franka_experiments/franka_experiments/nodes/distance_visualization_node.py` | ROS node | 402 |
| `franka_experiments/franka_experiments/nodes/experiment_logger.py` | ROS node | 1125 |
| `franka_experiments/franka_experiments/nodes/tag_overlay_node.py` | ROS node | 160 |
| `franka_experiments/franka_experiments/nodes/trajectory_overlay_node.py` | ROS node | 449 |
| `franka_experiments/franka_experiments/nodes/trajectory_visualization_node.py` | ROS node | 171 |
| `franka_experiments/franka_experiments/utils/cbf_diagnostics.py` | util module | 65 |
| `franka_experiments/franka_experiments/utils/logging_utils.py` | util module | 504 |
| `franka_experiments/franka_experiments/utils/simulation_imports.py` | util module | 96 |
| `franka_experiments/franka_experiments/utils/trajectory_overlay.py` | util module | 322 |
| `franka_experiments/franka_experiments/utils/trajectory_trace.py` | util module | 258 |
| `franka_experiments/franka_experiments/utils/visualization.py` | util module | 154 |
| `franka_experiments/scripts/analyze_isolation_ab.py` | offline/aux script | 300 |
| `franka_experiments/scripts/approach_envelope.py` | offline/aux script | 124 |
| `franka_experiments/scripts/approach_probe.py` | offline/aux script | 416 |
| `franka_experiments/scripts/bag_to_mp4.py` | offline/aux script | 160 |
| `franka_experiments/scripts/ball_closed_loop.py` | offline/aux script | 780 |
| `franka_experiments/scripts/ball_throw_eval.py` | offline/aux script | 408 |
| `franka_experiments/scripts/cbf_phase4_check.py` | offline/aux script | 285 |
| `franka_experiments/scripts/cbf_scenarios.py` | offline/aux script | 734 |
| `franka_experiments/scripts/compare_range_noise.py` | offline/aux script | 464 |
| `franka_experiments/scripts/compare_vobs.py` | offline/aux script | 730 |
| `franka_experiments/scripts/experiment_summary.py` | offline/aux script | 255 |
| `franka_experiments/scripts/gen_zero_gain_controllers_yaml.py` | offline/aux script | 79 |
| `franka_experiments/scripts/latency_budget.py` | offline/aux script | 695 |
| `franka_experiments/scripts/pass_plot.py` | offline/aux script | 38 |
| `franka_experiments/scripts/pipeline_rate_monitor.py` | offline/aux script | 414 |
| `franka_experiments/scripts/plot_rl_timing.py` | offline/aux script | 123 |
| `franka_experiments/scripts/range_noise_calibration.py` | offline/aux script | 283 |
| `franka_experiments/scripts/smoothness_report.py` | offline/aux script | 163 |
| `franka_experiments/scripts/throw_actuation.py` | offline/aux script | 71 |
| `franka_experiments/scripts/throw_clearance.py` | offline/aux script | 46 |
| `franka_experiments/scripts/throw_lead.py` | offline/aux script | 46 |
| `franka_experiments/scripts/watch_obstacle_tracks.py` | offline/aux script | 153 |
| `franka_experiments/test/data/experiment_log_header_94cd7b0.json` | test | 341 |
| `franka_experiments/test/test_control_point_derivatives.py` | test | 109 |
| `franka_experiments/test/test_experiment_log_columns.py` | test | 333 |
| `franka_experiments/test/test_experiment_logger_truth_off_identical.py` | test | 64 |
| `franka_experiments/test/test_kinematics_fix_regression.py` | test | 177 |

## 14 — Build / launch / test → `14_build_launch_test.md` *(not written)*

22 files, 4414 lines.

| File | Role | Lines |
|---|---|---:|
| `franka_experiments/.gitignore` | build metadata | 34 |
| `franka_experiments/MANIFEST.in` | build metadata | 4 |
| `franka_experiments/package.xml` | build metadata | 47 |
| `franka_experiments/resource/franka_experiments` | build metadata | 0 |
| `franka_experiments/setup.cfg` | build metadata | 8 |
| `franka_experiments/setup.py` | build metadata | 69 |
| `franka_simulation_experiments/.gitignore` | build metadata | 2 |
| `franka_simulation_experiments/CMakeLists.txt` | build metadata | 23 |
| `franka_simulation_experiments/package.xml` | build metadata | 44 |
| `franka_simulation_experiments/pytest.ini` | build metadata | 7 |
| `franka_experiments/test/README.md` | test | 401 |
| `franka_experiments/test/_launch_snapshot.py` | test | 138 |
| `franka_experiments/test/config/test_defaults.yaml` | test | 38 |
| `franka_experiments/test/data/README_snapshot.md` | test | 32 |
| `franka_experiments/test/data/torque_control_stack_94cd7b0_snapshot.json` | test | 2756 |
| `franka_experiments/test/e2e_pentagon_moveit.sh` | test | 48 |
| `franka_experiments/test/launch/test_rl_fake.launch.py` | test | 78 |
| `franka_experiments/test/launch/test_torque_fake.launch.py` | test | 108 |
| `franka_experiments/test/launch/test_velocity_fake.launch.py` | test | 99 |
| `franka_experiments/test/scripts/check_topics.sh` | test | 229 |
| `franka_experiments/test/test_node_self_references.py` | test | 219 |
| `franka_experiments/test/test_run_metadata.py` | test | 30 |

## 16 — Issues & discrepancies (lead documents) → `16_issues_and_discrepancies.md` *(not written)*

13 files, 12081 lines.

| File | Role | Lines |
|---|---|---:|
| `franka_experiments/LEGACY.md` | doc (lead) | 193 |
| `franka_experiments/README.md` | doc (lead) | 438 |
| `franka_experiments/SAFETY.md` | doc (lead) | 302 |
| `franka_experiments/docs/FRAMEWORK_COMPLETE_TECHNICAL_GUIDE.md` | doc (lead) | 5813 |
| `franka_experiments/docs/approach_speed_envelope.md` | doc (lead) | 227 |
| `franka_experiments/docs/ball_throw_closed_loop.md` | doc (lead) | 144 |
| `franka_experiments/docs/plots/bt3_nom_vs_safe.png` | doc asset | 323 |
| `franka_experiments/docs/plots/bt3_passes.png` | doc asset | 1148 |
| `franka_experiments/docs/reactivity_evasion_report.md` | doc (lead) | 220 |
| `franka_experiments/docs/ros_node_graph.png` | doc asset | 2334 |
| `franka_experiments/docs/ros_node_graph.svg` | doc asset | 195 |
| `franka_simulation_experiments/README.md` | doc (lead) | 673 |
| `franka_simulation_experiments/next_step.md` | doc (lead) | 71 |
