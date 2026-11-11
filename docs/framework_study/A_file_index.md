# Appendix A — File index

> **Generated** (Stage 7) by `tools/stage7_verification/gen_file_index.py` from [ledger/ledger.json](ledger/ledger.json), `tools/stage7_verification/results/inventory.json` and [15_code_status.md](15_code_status.md). Do not edit by hand: change the ledger (`tools/ledger_status.py`) or chapter 15 and regenerate. Baseline `ba68077`.

Each in-scope file appears once, under its **primary** chapter. Columns:
- **Status** (coverage): `documented` = covered by its chapter; `read` = every function read; `partial` = the parts named in PROGRESS §2 were read (for tests: static traceability only, C-043); `pending` = inventoried only, not read.
- **Class** (ch. 15): CURRENT / ALTERNATIVE / EXPERIMENTAL / LEGACY / UNUSED CANDIDATE / UNKNOWN; "—" where ch. 15 classifies by kind (tests, data, build files) and not per file.
- **Prod / Launch / Tests**: production importers, launch files that reference it, test files that import it (static, `inventory.py`). **Cited in**: chapters that name the path.

| Status | Files |
|---|---:|
| documented | 100 |
| read | 3 |
| partial | 225 |
| pending | 18 |
| **total** | **346** |

## 01 — Architecture → [01_architecture.md](01_architecture.md)

5 files, 3066 lines; documented 2, partial 3.

| File | Role | Lines | Status | Class | Prod | Launch | Tests | Cited in |
|---|---|---:|---|---|---:|---:|---:|---|
| `franka_experiments/launch/bag_replay.launch.py` | launch file | 201 | partial | CURRENT | 0 | 0 | 0 | 01, 05, 13, 14 |
| `franka_experiments/launch/minimal.launch.py` | launch file | 526 | partial | CURRENT | 0 | 4 | 2 | 01, 02, 09, 10, 14, 16 |
| `franka_experiments/launch/thales.launch.py` | launch file | 371 | partial | ALTERNATIVE | 0 | 0 | 0 | 01, 02, 14 |
| `franka_experiments/launch/torque_control_stack.launch.py` | launch file | 1527 | documented | CURRENT | 0 | 3 | 4 | 01, 02, 03, 06, 10, 12, 14, 16 |
| `franka_experiments/launch/velocity_cbf_control_stack.launch.py` | launch file | 441 | documented | ALTERNATIVE | 0 | 0 | 0 | 01, 02, 06, 10, 14, 16 |

## 02 — ROS graph and data flow → [02_ros_graph_and_dataflow.md](02_ros_graph_and_dataflow.md)

2 files, 49 lines; partial 2.

| File | Role | Lines | Status | Class | Prod | Launch | Tests | Cited in |
|---|---|---:|---|---|---:|---:|---:|---|
| `franka_msgs/msg/HumanRobotDistance.msg` | message definition (external) | 30 | partial | — | 0 | 0 | 0 |  |
| `franka_msgs/msg/MultiDistance.msg` | message definition (external) | 19 | partial | — | 0 | 0 | 0 |  |

## 03 — Configuration → [03_configuration.md](03_configuration.md)

13 files, 6137 lines; documented 3, read 2, partial 8.

| File | Role | Lines | Status | Class | Prod | Launch | Tests | Cited in |
|---|---|---:|---|---|---:|---:|---:|---|
| `franka_experiments/config/fake_hw_controller_params.yaml` | config (parameters) | 7 | partial | UNUSED CANDIDATE | 0 | 0 | 0 | 03, 15, 16 |
| `franka_experiments/config/fr3_complete.yaml` | config (parameters) | 930 | partial | CURRENT | 0 | 0 | 0 | 01, 02, 03, 04, 05, 12, 15, 16 |
| `franka_experiments/config/fr3_control.yaml` | config (parameters) | 2672 | partial | CURRENT | 0 | 0 | 0 | 01, 02, 03, 05, 06, 08, 10, 11, 12, 14, 15, 16 |
| `franka_experiments/config/fr3_distance.yaml` | config (parameters) | 23 | documented | ALTERNATIVE | 0 | 0 | 0 | 02, 03, 15, 16 |
| `franka_experiments/config/launch_defaults.yaml` | config (parameters) | 333 | documented | CURRENT | 0 | 0 | 0 | 01, 02, 03, 05, 10, 14, 15, 16 |
| `franka_experiments/config/oscbf_params.yaml` | config (parameters) | 63 | partial | UNUSED CANDIDATE | 0 | 0 | 0 | 01, 03, 15, 16 |
| `franka_experiments/franka_experiments/__init__.py` | package init | 0 | read | CURRENT | 1 | 0 | 19 | 14, 15 |
| `franka_experiments/franka_experiments/utils/__init__.py` | util module | 6 | read | CURRENT | 3 | 0 | 2 | 14, 15 |
| `franka_experiments/franka_experiments/utils/config.py` | util module | 806 | partial | CURRENT | 24 | 0 | 2 | 01, 02, 03, 05, 09, 12, 14, 15, 16 |
| `franka_experiments/franka_experiments/utils/launch_support.py` | util module | 589 | partial | CURRENT | 4 | 0 | 0 | 01, 02, 09, 15 |
| `franka_experiments/franka_experiments/utils/node_runtime.py` | util module | 170 | partial | CURRENT | 8 | 0 | 0 | 15 |
| `franka_experiments/franka_experiments/utils/params.py` | util module | 293 | documented | CURRENT | 8 | 0 | 0 | 03, 06, 07, 08, 15, 16 |
| `franka_experiments/franka_experiments/utils/ros.py` | util module | 245 | partial | LEGACY | 10 | 0 | 2 | 10, 15 |

## 04 — Mathematical foundations → [04_math_foundations.md](04_math_foundations.md)

5 files, 852 lines; documented 5.

| File | Role | Lines | Status | Class | Prod | Launch | Tests | Cited in |
|---|---|---:|---|---|---:|---:|---:|---|
| `franka_experiments/franka_experiments/utils/cbf_kinematics.py` | util module | 9 | documented | LEGACY | 2 | 0 | 0 | 04, 15 |
| `franka_experiments/franka_experiments/utils/constants.py` | util module | 18 | documented | CURRENT | 15 | 0 | 1 | 04, 15 |
| `franka_experiments/franka_experiments/utils/kinematics.py` | util module | 665 | documented | CURRENT | 29 | 0 | 6 | 03, 04, 15, 16 |
| `franka_experiments/franka_experiments/utils/math_utils.py` | util module | 87 | documented | CURRENT | 12 | 0 | 0 | 04, 15 |
| `franka_experiments/scripts/check_control_derivatives.py` | offline/aux script | 73 | documented | — | 0 | 0 | 0 | 04, 15 |

## 05 — Perception and distance → [05_perception_and_distance.md](05_perception_and_distance.md)

64 files, 19548 lines; documented 18, partial 40, pending 6.

| File | Role | Lines | Status | Class | Prod | Launch | Tests | Cited in |
|---|---|---:|---|---|---:|---:|---:|---|
| `franka_experiments/config/camera_EE_extrinsic.yaml` | config (calibration) | 51 | partial | CURRENT | 0 | 0 | 0 | 03, 05, 15 |
| `franka_experiments/config/camera_extrinsics.yaml` | config (calibration) | 11 | documented | CURRENT | 0 | 0 | 0 | 02, 03, 05, 13, 15 |
| `franka_experiments/config/camera_intrinsics.yaml` | config (calibration) | 56 | pending | UNUSED CANDIDATE | 0 | 0 | 0 | 03, 05, 15 |
| `franka_experiments/config/camera_link_extrinsics.yaml` | config (calibration) | 16 | documented | CURRENT | 0 | 0 | 0 | 02, 03, 05, 15 |
| `franka_experiments/config/dataset_handeye_eye_in_hand.yaml` | config (calibration) | 310 | pending | CURRENT | 0 | 0 | 0 | 03, 15 |
| `franka_experiments/config/depth_intrinsics.yaml` | config (calibration) | 56 | documented | CURRENT / UNUSED CANDIDATE | 0 | 0 | 0 | 03, 05, 15 |
| `franka_experiments/config/ee_tag_extrinsics.yaml` | config (calibration) | 11 | pending | CURRENT | 0 | 0 | 0 | 03, 05, 15 |
| `franka_experiments/config/handeye_eye_in_hand_rviz.rviz` | config (calibration) | 120 | pending | CURRENT | 0 | 0 | 0 | 15 |
| `franka_experiments/config/handeye_rviz.rviz` | config (calibration) | 150 | pending | CURRENT | 0 | 0 | 0 | 15 |
| `franka_experiments/config/rgb_intrinsics.yaml` | config (calibration) | 56 | pending | CURRENT / UNUSED CANDIDATE | 0 | 0 | 0 | 03, 05, 15 |
| `franka_experiments/franka_experiments/nodes/frame_grabber.py` | ROS node | 155 | partial | UNUSED CANDIDATE | 0 | 0 | 0 | 05, 15 |
| `franka_experiments/franka_experiments/nodes/handeye_calibration_node.py` | ROS node | 2561 | partial | CURRENT | 0 | 1 | 0 | 05, 15 |
| `franka_experiments/franka_experiments/nodes/handeye_eye_in_hand_node.py` | ROS node | 962 | partial | CURRENT | 0 | 1 | 0 | 05, 15 |
| `franka_experiments/franka_experiments/nodes/image_rectifier_node.py` | ROS node | 107 | partial | CURRENT | 0 | 1 | 0 | 05, 15 |
| `franka_experiments/franka_experiments/nodes/real_time_distance.py` | ROS node | 1255 | documented | CURRENT | 1 | 5 | 2 | 01, 02, 05, 14, 15, 16 |
| `franka_experiments/franka_experiments/utils/ballistic_prediction.py` | util module | 202 | partial | EXPERIMENTAL | 1 | 0 | 1 | 05, 15 |
| `franka_experiments/franka_experiments/utils/calibration_check.py` | util module | 235 | documented | CURRENT | 2 | 0 | 1 | 05, 15 |
| `franka_experiments/franka_experiments/utils/camera_yaml.py` | util module | 17 | documented | LEGACY | 1 | 0 | 0 | 15 |
| `franka_experiments/franka_experiments/utils/distance_engine.py` | util module | 1014 | documented | CURRENT | 8 | 0 | 10 | 15, 16 |
| `franka_experiments/franka_experiments/utils/distance_utils.py` | util module | 307 | documented | CURRENT | 10 | 0 | 2 | 04, 15 |
| `franka_experiments/franka_experiments/utils/handeye_solver.py` | util module | 422 | partial | CURRENT | 1 | 0 | 1 | 05, 15 |
| `franka_experiments/franka_experiments/utils/mask_builder.py` | util module | 312 | documented | CURRENT | 6 | 0 | 1 | 15, 16 |
| `franka_experiments/franka_experiments/utils/node_utils.py` | util module | 42 | documented | LEGACY | 0 | 0 | 0 | 15 |
| `franka_experiments/franka_experiments/utils/obstacle_clusters.py` | util module | 376 | documented | CURRENT | 3 | 0 | 2 | 05, 15 |
| `franka_experiments/franka_experiments/utils/obstacle_sim.py` | util module | 153 | partial | EXPERIMENTAL / CURRENT | 4 | 0 | 0 | 15 |
| `franka_experiments/franka_experiments/utils/obstacle_track_pipeline.py` | util module | 312 | documented | CURRENT | 6 | 0 | 2 | 15 |
| `franka_experiments/franka_experiments/utils/obstacle_tracker.py` | util module | 1048 | documented | CURRENT | 2 | 0 | 7 | 05, 15 |
| `franka_experiments/franka_experiments/utils/perception_msgs.py` | util module | 484 | documented | CURRENT | 10 | 0 | 10 | 06, 15, 16 |
| `franka_experiments/franka_experiments/utils/rtd_debug.py` | util module | 370 | partial | UNUSED CANDIDATE | 0 | 0 | 0 | 15 |
| `franka_experiments/franka_experiments/utils/self_detection.py` | util module | 226 | documented | CURRENT | 2 | 0 | 2 | 05, 15 |
| `franka_experiments/franka_experiments/utils/self_shadow.py` | util module | 685 | partial | EXPERIMENTAL | 2 | 0 | 3 | 05, 14, 15 |
| `franka_experiments/franka_experiments/utils/static_scene.py` | util module | 505 | partial | EXPERIMENTAL | 0 | 0 | 2 | 05, 14, 15, 16 |
| `franka_experiments/franka_experiments/utils/surface_velocity.py` | util module | 279 | partial | EXPERIMENTAL | 0 | 0 | 1 | 05, 14, 15, 16 |
| `franka_experiments/franka_experiments/utils/tf_manager.py` | util module | 223 | documented | CURRENT | 8 | 0 | 0 | 02, 05, 15 |
| `franka_experiments/launch/handeye_calibration_bringup.launch.py` | launch file | 166 | partial | CURRENT | 0 | 0 | 0 | 01, 14 |
| `franka_experiments/launch/handeye_eye_in_hand_calibration.launch.py` | launch file | 335 | partial | CURRENT | 0 | 0 | 0 | 01, 13, 14 |
| `franka_experiments/test/test_ballistic_prediction.py` | test | 108 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_calibration_check.py` | test | 186 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_obstacle_identity.py` | test | 251 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_sensor_range_uncertainty.py` | test | 227 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_compare_range_noise.py` | test | 121 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cp_label_agreement.py` | test | 99 | partial | — | 0 | 0 | 0 | 05 |
| `franka_experiments/test/test_distance_surface_semantics.py` | test | 355 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_handeye_solver.py` | test | 209 | partial | — | 0 | 0 | 0 | 05 |
| `franka_experiments/test/test_link_distance_track_fields.py` | test | 125 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_mask_depth_gate.py` | test | 338 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_multi_obstacle_rows.py` | test | 294 | partial | — | 0 | 0 | 0 | 05 |
| `franka_experiments/test/test_obstacle_clusters.py` | test | 263 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_obstacle_imm.py` | test | 270 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_obstacle_kalman.py` | test | 257 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_obstacle_scene_guard.py` | test | 111 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_obstacle_track_pipeline.py` | test | 267 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_obstacle_tracker.py` | test | 363 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_range_noise_calibration.py` | test | 123 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_self_detection.py` | test | 213 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_self_reject_and_obstacle_floor.py` | test | 179 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_self_shadow_derivatives.py` | test | 302 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_self_shadow_geometry.py` | test | 237 | partial | — | 0 | 0 | 0 | 05 |
| `franka_experiments/test/test_sensor_range_field.py` | test | 268 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_ssb_perception.py` | test | 327 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_track_eviction.py` | test | 70 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_track_lifecycle.py` | test | 198 | partial | — | 0 | 0 | 0 |  |
| `franka_msgs/msg/LinkDistance.msg` | message definition (external) | 186 | documented | — | 0 | 0 | 0 | 05 |
| `franka_msgs/msg/MultiLinkDistance.msg` | message definition (external) | 11 | documented | — | 0 | 0 | 0 |  |

## 06 — Nominal motion → [06_nominal_motion.md](06_nominal_motion.md)

26 files, 10402 lines; documented 5, partial 21.

| File | Role | Lines | Status | Class | Prod | Launch | Tests | Cited in |
|---|---|---:|---|---|---:|---:|---:|---|
| `franka_experiments/franka_experiments/nodes/ee_circle_velocity_commander.py` | ROS node | 472 | partial | UNUSED CANDIDATE | 0 | 0 | 0 | 15 |
| `franka_experiments/franka_experiments/nodes/ee_pentagon_velocity_commander.py` | ROS node | 360 | partial | ALTERNATIVE | 0 | 2 | 1 | 02, 06, 15 |
| `franka_experiments/franka_experiments/nodes/ee_random_waypoints_velocity_commander.py` | ROS node | 537 | partial | UNUSED CANDIDATE | 0 | 0 | 0 | 15 |
| `franka_experiments/franka_experiments/nodes/pentagon_qddot_commander.py` | ROS node | 1688 | partial | CURRENT | 0 | 2 | 2 | 01, 02, 06, 15 |
| `franka_experiments/franka_experiments/nodes/rl_policy_commander.py` | ROS node | 753 | partial | ALTERNATIVE | 0 | 1 | 3 | 01, 02, 06, 15 |
| `franka_experiments/franka_experiments/utils/avoidance.py` | util module | 165 | partial | UNUSED CANDIDATE | 0 | 0 | 1 | 06, 15, 16 |
| `franka_experiments/franka_experiments/utils/avoidance_math.py` | util module | 1309 | partial | ALTERNATIVE | 1 | 0 | 0 | 06, 15 |
| `franka_experiments/franka_experiments/utils/evasion_direction.py` | util module | 264 | documented | CURRENT | 1 | 0 | 2 | 06, 15 |
| `franka_experiments/franka_experiments/utils/go_around.py` | util module | 160 | documented | CURRENT | 1 | 0 | 1 | 06, 14, 15 |
| `franka_experiments/franka_experiments/utils/livelock.py` | util module | 163 | documented | CURRENT | 2 | 0 | 1 | 06, 15 |
| `franka_experiments/franka_experiments/utils/rl_policy.py` | util module | 499 | partial | ALTERNATIVE | 1 | 0 | 2 | 03, 06, 10, 15 |
| `franka_experiments/franka_experiments/utils/singularity_recovery.py` | util module | 205 | documented | CURRENT | 1 | 0 | 1 | 15, 16 |
| `franka_experiments/franka_experiments/utils/timing_law.py` | util module | 135 | documented | CURRENT | 1 | 0 | 0 | 06, 15 |
| `franka_experiments/franka_experiments/utils/trajectory.py` | util module | 887 | partial | CURRENT | 6 | 0 | 0 | 06, 15 |
| `franka_experiments/test/smoke_rl_policy_commander.py` | test | 268 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_avoidance.py` | test | 191 | partial | — | 0 | 0 | 0 | 06, 15 |
| `franka_experiments/test/test_directional_fade.py` | test | 28 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_evasion_direction.py` | test | 192 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_fr3_velocity_envelope.py` | test | 186 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_go_around.py` | test | 64 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_livelock.py` | test | 218 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_phase_governor.py` | test | 195 | partial | — | 0 | 0 | 0 | 06, 14, 16 |
| `franka_experiments/test/test_phase_governor_staging.py` | test | 97 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_rl_policy.py` | test | 772 | partial | — | 0 | 0 | 0 | 10 |
| `franka_experiments/test/test_trajectory_overlay.py` | test | 290 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_trajectory_trace.py` | test | 304 | partial | — | 0 | 0 | 0 |  |

## 07 — CBF obstacle barrier → [07_cbf_obstacle_barrier.md](07_cbf_obstacle_barrier.md)

30 files, 12376 lines; documented 5, partial 25.

| File | Role | Lines | Status | Class | Prod | Launch | Tests | Cited in |
|---|---|---:|---|---|---:|---:|---:|---|
| `franka_experiments/franka_experiments/utils/cbf_constraints.py` | util module | 218 | documented | LEGACY | 0 | 0 | 0 | 03, 07, 15, 16 |
| `franka_experiments/franka_experiments/utils/cbf_evasion.py` | util module | 306 | documented | CURRENT | 1 | 0 | 1 | 06, 15 |
| `franka_experiments/franka_experiments/utils/cbf_state_rows.py` | util module | 4354 | documented | CURRENT | 13 | 0 | 35 | 04, 05, 07, 08, 14, 15, 16 |
| `franka_experiments/franka_experiments/utils/cbf_utils.py` | util module | 46 | documented | CURRENT | 7 | 0 | 1 | 15 |
| `franka_experiments/franka_experiments/utils/cbf_zones.py` | util module | 356 | documented | CURRENT | 4 | 0 | 1 | 07, 15 |
| `franka_experiments/test/test_cbf_curvature_term.py` | test | 346 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_diag_line.py` | test | 191 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_evidence_trust.py` | test | 457 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_fast_track.py` | test | 81 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_latency_compensation.py` | test | 174 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_lateral_evasion.py` | test | 296 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_link_speed.py` | test | 151 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_link_speed_relative.py` | test | 39 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_normal_rotation_guard.py` | test | 216 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_outrun_evasion.py` | test | 173 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_phase1_reactivity.py` | test | 286 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_retreat_authority.py` | test | 204 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_retreat_cap.py` | test | 347 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_selfcollision_anticipation.py` | test | 243 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_state_rows.py` | test | 828 | partial | — | 0 | 0 | 0 | 14 |
| `franka_experiments/test/test_cbf_static_scene.py` | test | 325 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_surface_velocity.py` | test | 561 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_tangential_bias.py` | test | 353 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_tracker_source.py` | test | 400 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_uncertainty_margin.py` | test | 176 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_velocity_feedforward.py` | test | 159 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_velocity_standoff.py` | test | 81 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_vobs_rate_independent.py` | test | 414 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_zone_ladder.py` | test | 468 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_vobs_in_hdot.py` | test | 127 | partial | — | 0 | 0 | 0 |  |

## 08 — Constraints and QP → [08_constraints_and_qp.md](08_constraints_and_qp.md)

27 files, 8175 lines; documented 8, partial 19.

| File | Role | Lines | Status | Class | Prod | Launch | Tests | Cited in |
|---|---|---:|---|---|---:|---:|---:|---|
| `franka_experiments/franka_experiments/nodes/cbf_safety_filter.py` | ROS node | 2085 | documented | CURRENT | 1 | 2 | 13 | 01, 02, 06, 07, 08, 14, 15, 16 |
| `franka_experiments/franka_experiments/utils/cbf_hard_limits.py` | util module | 456 | documented | CURRENT | 7 | 0 | 10 | 08, 15, 16 |
| `franka_experiments/franka_experiments/utils/cbf_qp.py` | util module | 114 | documented | LEGACY | 0 | 0 | 0 | 08, 15 |
| `franka_experiments/franka_experiments/utils/cbf_qp_assembly.py` | util module | 598 | documented | CURRENT | 4 | 0 | 16 | 07, 08, 15, 16 |
| `franka_experiments/franka_experiments/utils/cbf_singularity.py` | util module | 214 | documented | CURRENT | 1 | 0 | 1 | 08, 15 |
| `franka_experiments/franka_experiments/utils/rate_scaling.py` | util module | 199 | documented | CURRENT | 5 | 0 | 1 | 05, 08, 15, 16 |
| `franka_experiments/franka_experiments/utils/self_collision.py` | util module | 224 | documented | LEGACY | 1 | 0 | 3 | 15, 16 |
| `franka_experiments/franka_experiments/utils/state_governor.py` | util module | 266 | documented | CURRENT | 1 | 0 | 3 | 06, 15 |
| `franka_experiments/test/_cbf_builder_harness.py` | test | 390 | partial | — | 0 | 0 | 0 | 03, 14 |
| `franka_experiments/test/smoke_cbf_construct.py` | test | 110 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/smoke_cbf_safety_filter.py` | test | 180 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_accel_box_decoupling.py` | test | 67 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_brake_slew_limited.py` | test | 100 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_hard_constraints.py` | test | 405 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_multi_cp_qp.py` | test | 303 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_qp_fallback.py` | test | 199 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_risk_weighted_slack.py` | test | 330 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_row_l1_slack.py` | test | 129 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_row_padding.py` | test | 222 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_cbf_singularity.py` | test | 198 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_rate_scaling.py` | test | 388 | partial | — | 0 | 0 | 0 | 16 |
| `franka_experiments/test/test_retreat_excess_speed.py` | test | 42 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_safety_failure_policy.py` | test | 78 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_shipped_smoothness_config.py` | test | 54 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_singularity_recovery.py` | test | 162 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_ssb_qp_rows.py` | test | 463 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_state_governor.py` | test | 199 | partial | — | 0 | 0 | 0 |  |

## 09 — Torque realization → [09_torque_realization.md](09_torque_realization.md)

5 files, 1288 lines; documented 5.

| File | Role | Lines | Status | Class | Prod | Launch | Tests | Cited in |
|---|---|---:|---|---|---:|---:|---:|---|
| `franka_experiments/franka_experiments/nodes/qddot_to_torque.py` | ROS node | 317 | documented | CURRENT | 0 | 2 | 3 | 01, 02, 04, 09, 15 |
| `franka_rt_controllers/config/cbf_torque_controller.yaml` | C++ controller (external, executed) | 6 | documented | — | 0 | 0 | 0 | 09 |
| `franka_rt_controllers/franka_rt_controllers.xml` | C++ controller (external, executed) | 10 | documented | — | 0 | 0 | 0 | 09 |
| `franka_rt_controllers/src/cbf_torque_controller.cpp` | C++ controller (external, executed) | 332 | documented | — | 0 | 0 | 0 | 02, 09, 16 |
| `franka_rt_controllers/src/rt_torque_controller.cpp` | C++ controller (external, executed) | 623 | documented | — | 0 | 0 | 0 | 01, 02, 09 |

## 10 — Alternative controllers → [10_alternative_controllers.md](10_alternative_controllers.md)

9 files, 3150 lines; documented 7, partial 2.

| File | Role | Lines | Status | Class | Prod | Launch | Tests | Cited in |
|---|---|---:|---|---|---:|---:|---:|---|
| `franka_experiments/franka_experiments/nodes/cbf_OSCBF_filter.py` | ROS node | 971 | documented | ALTERNATIVE | 0 | 0 | 0 | 01, 15 |
| `franka_experiments/franka_experiments/nodes/cbf_velocity_filter.py` | ROS node | 623 | documented | ALTERNATIVE | 0 | 1 | 0 | 01, 02, 10, 14, 15 |
| `franka_experiments/franka_experiments/nodes/pentagon_torque_commander.py` | ROS node | 496 | partial | ALTERNATIVE | 0 | 1 | 1 | 15 |
| `franka_experiments/franka_experiments/utils/ros_setup.py` | util module | 200 | partial | ALTERNATIVE | 1 | 0 | 0 | 15 |
| `franka_experiments/test/test_cbf_velocity_filter.py` | test | 205 | documented | — | 0 | 0 | 0 | 01, 10, 14 |
| `franka_experiments/test/test_torque_stack_external_plant_off_identical.py` | test | 117 | documented | — | 0 | 0 | 0 | 01, 10 |
| `franka_rt_controllers/config/rt_velocity_blender.yaml` | C++ controller (external, executed) | 89 | documented | — | 0 | 0 | 0 | 10, 16 |
| `franka_rt_controllers/rt_velocity_blender_controller.xml` | C++ controller (external, executed) | 25 | documented | — | 0 | 0 | 0 | 09 |
| `franka_rt_controllers/src/rt_velocity_blender_controller.cpp` | C++ controller (external, executed) | 424 | documented | — | 0 | 0 | 0 | 01, 02, 10, 16 |

## 11 — ISO safety layer → [11_iso_safety_layer.md](11_iso_safety_layer.md)

18 files, 5111 lines; documented 4, partial 14.

| File | Role | Lines | Status | Class | Prod | Launch | Tests | Cited in |
|---|---|---:|---|---|---:|---:|---:|---|
| `franka_experiments/config/safety/watchman_profile.md` | doc (lead) | 153 | partial | — | 0 | 0 | 0 | 03, 11, 15 |
| `franka_experiments/franka_experiments/nodes/iso_evidence_logger.py` | ROS node | 621 | documented | EXPERIMENTAL | 0 | 1 | 0 | 02, 15 |
| `franka_experiments/franka_experiments/nodes/iso_safety_monitor.py` | ROS node | 417 | documented | EXPERIMENTAL | 0 | 1 | 1 | 02, 15 |
| `franka_experiments/franka_experiments/utils/iso_ssm.py` | util module | 210 | documented | EXPERIMENTAL | 4 | 0 | 3 | 11, 15 |
| `franka_experiments/scripts/iso_constants_measure.py` | offline/aux script | 644 | partial | — | 0 | 0 | 0 | 11, 13, 15, 16 |
| `franka_experiments/scripts/iso_evidence_report.py` | offline/aux script | 678 | partial | — | 0 | 0 | 1 | 11, 13, 15 |
| `franka_experiments/scripts/iso_pfl_speed.py` | offline/aux script | 155 | partial | — | 0 | 0 | 1 | 11, 13, 15, 16 |
| `franka_experiments/scripts/iso_preflight_check.py` | offline/aux script | 254 | documented | — | 0 | 1 | 0 | 11, 13, 14, 15 |
| `franka_experiments/test/test_iso_dsafe_floor.py` | test | 155 | partial | — | 0 | 0 | 0 | 14 |
| `franka_experiments/test/test_iso_evidence_report.py` | test | 246 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_iso_flags_off_identical.py` | test | 142 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_iso_monitor.py` | test | 267 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_iso_perception_failclosed.py` | test | 243 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_iso_pfl_ceiling.py` | test | 119 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_iso_speed_rows.py` | test | 206 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_iso_ssm.py` | test | 158 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_iso_status_contract.py` | test | 232 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_iso_torque_limits.py` | test | 211 | partial | — | 0 | 0 | 0 |  |

## 12 — Simulation → [12_simulation.md](12_simulation.md)

63 files, 10429 lines; documented 23, read 1, partial 39.

| File | Role | Lines | Status | Class | Prod | Launch | Tests | Cited in |
|---|---|---:|---|---|---:|---:|---:|---|
| `franka_simulation/launch/gz_fr3_plant.launch.py` | launch file (external, executed) | 180 | documented | — | 0 | 2 | 0 | 00, 01, 02, 12, 14 |
| `franka_simulation_experiments/config/conditions.d/mppi_it08.yaml` | config (sim experiment axes) | 57 | partial | CURRENT | 0 | 0 | 0 | 03, 15, 16 |
| `franka_simulation_experiments/config/conditions.yaml` | config (sim experiment axes) | 1516 | partial | CURRENT | 0 | 0 | 0 | 01, 03, 12, 14, 15 |
| `franka_simulation_experiments/config/control_regression_scenarios.yaml` | config (sim experiment axes) | 29 | read | UNUSED CANDIDATE | 0 | 0 | 0 | 03, 15 |
| `franka_simulation_experiments/config/scenarios.yaml` | config (sim experiment axes) | 629 | partial | CURRENT | 0 | 0 | 0 | 01, 03, 12, 14, 15 |
| `franka_simulation_experiments/launch/gz_cbf_stack.launch.py` | launch file | 273 | documented | CURRENT | 0 | 1 | 1 | 01, 02, 03, 12, 14, 15 |
| `franka_simulation_experiments/launch/gz_circle_demo.launch.py` | launch file | 145 | partial | CURRENT | 0 | 0 | 0 | 01, 12, 14, 15 |
| `franka_simulation_experiments/launch/gz_plant.launch.py` | launch file | 66 | partial | CURRENT | 0 | 1 | 1 | 12, 14, 15 |
| `franka_simulation_experiments/scripts/check_self_occlusion.py` | sim node / tool | 107 | partial | CURRENT | 0 | 0 | 0 | 15 |
| `franka_simulation_experiments/scripts/emulated_perception.py` | sim node / tool | 149 | documented | — | 0 | 1 | 0 | 02 |
| `franka_simulation_experiments/scripts/episode_supervisor.py` | sim node / tool | 224 | documented | — | 0 | 1 | 1 | 02 |
| `franka_simulation_experiments/scripts/ground_truth_node.py` | sim node / tool | 141 | documented | — | 0 | 1 | 0 | 02 |
| `franka_simulation_experiments/scripts/gz_trajectory_markers.py` | sim node / tool | 256 | partial | CURRENT | 0 | 1 | 1 | 12, 15 |
| `franka_simulation_experiments/scripts/pin_process.py` | sim node / tool | 72 | partial | — | 0 | 1 | 0 | 14 |
| `franka_simulation_experiments/scripts/plot_control_regression.py` | sim node / tool | 70 | partial | CURRENT | 0 | 0 | 0 | 15 |
| `franka_simulation_experiments/scripts/reflex_monitor.py` | sim node / tool | 155 | documented | — | 0 | 1 | 0 | 02 |
| `franka_simulation_experiments/scripts/run_gz_scenarios.py` | sim node / tool | 94 | documented | CURRENT | 0 | 0 | 0 | 15 |
| `franka_simulation_experiments/scripts/scenario_commander.py` | sim node / tool | 95 | documented | — | 0 | 1 | 0 | 02 |
| `franka_simulation_experiments/scripts/simexp/__init__.py` | sim library (simexp) | 1 | documented | CURRENT | 14 | 0 | 20 | 14, 15 |
| `franka_simulation_experiments/scripts/simexp/analysis.py` | sim library (simexp) | 217 | documented | — | 7 | 0 | 4 | 12 |
| `franka_simulation_experiments/scripts/simexp/attribution.py` | sim library (simexp) | 132 | partial | — | 1 | 0 | 1 | 12 |
| `franka_simulation_experiments/scripts/simexp/batch.py` | sim library (simexp) | 305 | documented | — | 1 | 0 | 2 | 12 |
| `franka_simulation_experiments/scripts/simexp/closed_loop.py` | sim library (simexp) | 292 | documented | — | 2 | 0 | 1 | 12, 13, 15 |
| `franka_simulation_experiments/scripts/simexp/conditions.py` | sim library (simexp) | 135 | documented | — | 3 | 0 | 2 | 03, 12, 16 |
| `franka_simulation_experiments/scripts/simexp/control_regression.py` | sim library (simexp) | 131 | partial | CURRENT | 0 | 0 | 1 | 03, 12, 15 |
| `franka_simulation_experiments/scripts/simexp/emulation.py` | sim library (simexp) | 121 | documented | — | 3 | 0 | 2 | 12, 14 |
| `franka_simulation_experiments/scripts/simexp/estimator_metrics.py` | sim library (simexp) | 308 | partial | — | 2 | 0 | 1 | 12, 14 |
| `franka_simulation_experiments/scripts/simexp/graph_contract.py` | sim library (simexp) | 165 | partial | CURRENT | 0 | 0 | 1 | 12, 14, 15 |
| `franka_simulation_experiments/scripts/simexp/obstacle_laws.py` | sim library (simexp) | 114 | documented | — | 1 | 0 | 2 | 12, 16 |
| `franka_simulation_experiments/scripts/simexp/offline.py` | sim library (simexp) | 66 | partial | — | 2 | 0 | 1 |  |
| `franka_simulation_experiments/scripts/simexp/procs.py` | sim library (simexp) | 105 | documented | — | 1 | 0 | 2 |  |
| `franka_simulation_experiments/scripts/simexp/provenance.py` | sim library (simexp) | 49 | documented | UNUSED CANDIDATE | 0 | 0 | 1 | 12, 14, 15, 16 |
| `franka_simulation_experiments/scripts/simexp/reflex_checks.py` | sim library (simexp) | 149 | documented | — | 2 | 0 | 1 | 12 |
| `franka_simulation_experiments/scripts/simexp/report.py` | sim library (simexp) | 365 | partial | — | 1 | 0 | 1 | 11, 13, 15 |
| `franka_simulation_experiments/scripts/simexp/runner.py` | sim library (simexp) | 131 | documented | — | 1 | 0 | 3 |  |
| `franka_simulation_experiments/scripts/simexp/scenario_catalog.py` | sim library (simexp) | 225 | documented | — | 3 | 0 | 5 | 03, 12 |
| `franka_simulation_experiments/scripts/simexp/scenario_laws.py` | sim library (simexp) | 87 | documented | — | 1 | 0 | 1 | 12 |
| `franka_simulation_experiments/scripts/simexp/sim_config.py` | sim library (simexp) | 217 | documented | — | 7 | 0 | 7 | 03, 05, 09, 12 |
| `franka_simulation_experiments/scripts/simexp/truth_geometry.py` | sim library (simexp) | 87 | documented | — | 4 | 0 | 2 |  |
| `franka_simulation_experiments/test/conftest.py` | test | 11 | partial | — | 0 | 0 | 0 | 14 |
| `franka_simulation_experiments/test/launch/launch_test_graph_contract.py` | test | 142 | partial | — | 0 | 0 | 0 | 12, 14, 15 |
| `franka_simulation_experiments/test/slow/_rosprobe.py` | test | 97 | partial | — | 0 | 0 | 1 |  |
| `franka_simulation_experiments/test/slow/test_phase1_plant.py` | test | 215 | partial | — | 0 | 0 | 0 |  |
| `franka_simulation_experiments/test/slow/test_phase2_closed_loop.py` | test | 81 | partial | — | 0 | 0 | 0 |  |
| `franka_simulation_experiments/test/slow/test_phase3_perception.py` | test | 176 | partial | — | 0 | 0 | 0 |  |
| `franka_simulation_experiments/test/slow/test_phase4_obstacles.py` | test | 128 | partial | — | 0 | 0 | 0 |  |
| `franka_simulation_experiments/test/slow/test_phase5_batch.py` | test | 90 | partial | — | 0 | 0 | 0 |  |
| `franka_simulation_experiments/test/test_analysis.py` | test | 31 | partial | — | 0 | 0 | 0 | 12 |
| `franka_simulation_experiments/test/test_batch.py` | test | 76 | partial | — | 0 | 0 | 0 | 12 |
| `franka_simulation_experiments/test/test_closed_loop.py` | test | 144 | partial | — | 0 | 0 | 0 | 12 |
| `franka_simulation_experiments/test/test_conditions.py` | test | 173 | partial | — | 0 | 0 | 0 | 03, 12 |
| `franka_simulation_experiments/test/test_control_regression.py` | test | 43 | partial | — | 0 | 0 | 0 | 03, 12, 15 |
| `franka_simulation_experiments/test/test_emulation.py` | test | 102 | partial | — | 0 | 0 | 0 | 12 |
| `franka_simulation_experiments/test/test_estimator_metrics.py` | test | 108 | partial | — | 0 | 0 | 0 | 12 |
| `franka_simulation_experiments/test/test_gz_trajectory_markers.py` | test | 143 | partial | — | 0 | 0 | 0 | 12 |
| `franka_simulation_experiments/test/test_obstacle_law_wander.py` | test | 143 | partial | — | 0 | 0 | 0 | 12 |
| `franka_simulation_experiments/test/test_offline_and_attribution.py` | test | 102 | partial | — | 0 | 0 | 0 | 12 |
| `franka_simulation_experiments/test/test_reflex_checks.py` | test | 76 | partial | — | 0 | 0 | 0 | 12 |
| `franka_simulation_experiments/test/test_run_provenance.py` | test | 28 | partial | — | 0 | 0 | 0 | 12 |
| `franka_simulation_experiments/test/test_scenario_catalog.py` | test | 193 | partial | — | 0 | 0 | 0 | 12 |
| `franka_simulation_experiments/test/test_scenario_laws.py` | test | 92 | partial | — | 0 | 0 | 0 | 12 |
| `franka_simulation_experiments/test/test_sim_config.py` | test | 158 | partial | — | 0 | 0 | 0 | 05, 12 |
| `franka_simulation_experiments/test/test_ssb_scenarios.py` | test | 217 | partial | — | 0 | 0 | 0 | 12 |

## 13 — Observability and tools → [13_observability_and_tools.md](13_observability_and_tools.md)

43 files, 12747 lines; documented 4, partial 39.

| File | Role | Lines | Status | Class | Prod | Launch | Tests | Cited in |
|---|---|---:|---|---|---:|---:|---:|---|
| `franka_experiments/config/trajectory_rviz.rviz` | config (rviz) | 120 | partial | CURRENT | 0 | 0 | 0 | 03, 13, 15 |
| `franka_experiments/docs/make_ros_node_graph.py` | doc tool | 298 | partial | — | 0 | 0 | 0 | 13, 15 |
| `franka_experiments/franka_experiments/nodes/__init__.py` | package init | 0 | documented | CURRENT | 0 | 0 | 1 | 14, 15 |
| `franka_experiments/franka_experiments/nodes/capsule_overlay_node.py` | ROS node | 796 | partial | UNUSED CANDIDATE | 0 | 0 | 0 | 15 |
| `franka_experiments/franka_experiments/nodes/distance_visualization_node.py` | ROS node | 402 | partial | UNUSED CANDIDATE | 0 | 0 | 0 | 01, 15 |
| `franka_experiments/franka_experiments/nodes/experiment_logger.py` | ROS node | 1125 | partial | CURRENT | 1 | 4 | 3 | 01, 02, 15, 16 |
| `franka_experiments/franka_experiments/nodes/tag_overlay_node.py` | ROS node | 160 | partial | CURRENT | 0 | 1 | 0 | 15 |
| `franka_experiments/franka_experiments/nodes/trajectory_overlay_node.py` | ROS node | 449 | partial | CURRENT | 0 | 1 | 0 | 02, 15 |
| `franka_experiments/franka_experiments/nodes/trajectory_visualization_node.py` | ROS node | 171 | partial | CURRENT | 0 | 1 | 0 | 02, 15 |
| `franka_experiments/franka_experiments/utils/cbf_diagnostics.py` | util module | 65 | documented | CURRENT | 2 | 0 | 3 | 02, 14, 15 |
| `franka_experiments/franka_experiments/utils/logging_utils.py` | util module | 504 | partial | CURRENT | 14 | 0 | 2 | 15, 16 |
| `franka_experiments/franka_experiments/utils/run_metadata.py` | util module | 36 | documented | UNUSED CANDIDATE | 0 | 0 | 1 | 14, 15 |
| `franka_experiments/franka_experiments/utils/simulation_imports.py` | util module | 96 | documented | LEGACY | 1 | 0 | 0 | 15 |
| `franka_experiments/franka_experiments/utils/trajectory_overlay.py` | util module | 322 | partial | CURRENT | 1 | 0 | 1 | 13, 15 |
| `franka_experiments/franka_experiments/utils/trajectory_trace.py` | util module | 258 | partial | CURRENT | 3 | 0 | 2 | 13, 15 |
| `franka_experiments/franka_experiments/utils/visualization.py` | util module | 154 | partial | CURRENT | 2 | 0 | 0 | 13, 15 |
| `franka_experiments/scripts/analyze_isolation_ab.py` | offline/aux script | 300 | partial | — | 0 | 0 | 0 | 12, 13, 15 |
| `franka_experiments/scripts/approach_envelope.py` | offline/aux script | 124 | partial | — | 0 | 0 | 0 | 13, 15 |
| `franka_experiments/scripts/approach_probe.py` | offline/aux script | 416 | partial | — | 0 | 0 | 0 | 13, 15 |
| `franka_experiments/scripts/bag_to_mp4.py` | offline/aux script | 160 | partial | — | 0 | 0 | 0 | 13, 15 |
| `franka_experiments/scripts/ball_closed_loop.py` | offline/aux script | 780 | partial | — | 1 | 0 | 0 | 13, 15 |
| `franka_experiments/scripts/ball_throw_eval.py` | offline/aux script | 408 | partial | — | 6 | 0 | 0 | 13, 15 |
| `franka_experiments/scripts/cbf_phase4_check.py` | offline/aux script | 285 | partial | — | 0 | 0 | 0 | 13, 15 |
| `franka_experiments/scripts/cbf_scenarios.py` | offline/aux script | 734 | partial | — | 1 | 0 | 0 | 03, 12, 13, 15 |
| `franka_experiments/scripts/compare_range_noise.py` | offline/aux script | 464 | partial | — | 0 | 0 | 1 | 13, 15 |
| `franka_experiments/scripts/compare_vobs.py` | offline/aux script | 730 | partial | — | 4 | 0 | 0 | 13, 15 |
| `franka_experiments/scripts/experiment_summary.py` | offline/aux script | 255 | partial | — | 0 | 0 | 1 | 12, 13, 15 |
| `franka_experiments/scripts/gen_zero_gain_controllers_yaml.py` | offline/aux script | 79 | partial | — | 0 | 0 | 0 | 13, 15 |
| `franka_experiments/scripts/latency_budget.py` | offline/aux script | 695 | partial | — | 0 | 0 | 0 | 01, 11, 13, 15 |
| `franka_experiments/scripts/pass_plot.py` | offline/aux script | 38 | partial | — | 0 | 0 | 0 | 13, 15 |
| `franka_experiments/scripts/pipeline_rate_monitor.py` | offline/aux script | 414 | partial | — | 0 | 0 | 0 | 13, 14, 15 |
| `franka_experiments/scripts/plot_rl_timing.py` | offline/aux script | 123 | partial | — | 0 | 0 | 0 | 13, 15 |
| `franka_experiments/scripts/range_noise_calibration.py` | offline/aux script | 283 | partial | — | 1 | 0 | 2 | 13, 15 |
| `franka_experiments/scripts/smoothness_report.py` | offline/aux script | 163 | partial | — | 1 | 0 | 0 | 13, 15 |
| `franka_experiments/scripts/throw_actuation.py` | offline/aux script | 71 | partial | — | 0 | 0 | 0 | 13 |
| `franka_experiments/scripts/throw_clearance.py` | offline/aux script | 46 | partial | — | 0 | 0 | 0 | 13 |
| `franka_experiments/scripts/throw_lead.py` | offline/aux script | 46 | partial | — | 0 | 0 | 0 | 13 |
| `franka_experiments/scripts/watch_obstacle_tracks.py` | offline/aux script | 153 | partial | — | 0 | 0 | 0 | 13, 15 |
| `franka_experiments/test/data/experiment_log_header_94cd7b0.json` | test | 341 | partial | — | 0 | 0 | 0 | 13 |
| `franka_experiments/test/test_control_point_derivatives.py` | test | 109 | partial | — | 0 | 0 | 0 | 04 |
| `franka_experiments/test/test_experiment_log_columns.py` | test | 333 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_experiment_logger_truth_off_identical.py` | test | 64 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_kinematics_fix_regression.py` | test | 177 | partial | — | 0 | 0 | 0 | 04 |

## 14 — Build, launch and test → [14_build_launch_test.md](14_build_launch_test.md)

23 files, 4487 lines; documented 11, partial 12.

| File | Role | Lines | Status | Class | Prod | Launch | Tests | Cited in |
|---|---|---:|---|---|---:|---:|---:|---|
| `franka_experiments/.gitignore` | build metadata | 34 | documented | — | 0 | 0 | 0 |  |
| `franka_experiments/MANIFEST.in` | build metadata | 4 | documented | — | 0 | 0 | 0 |  |
| `franka_experiments/package.xml` | build metadata | 47 | documented | — | 0 | 0 | 0 | 14, 16 |
| `franka_experiments/resource/franka_experiments` | build metadata | 0 | documented | — | 0 | 0 | 0 | 00, 01, 02, 03, 04, 06, 07, 08, 09, 12, 13, 14, 15, 16 |
| `franka_experiments/scripts/pin_rt_thread.sh` | shell script | 73 | documented | — | 0 | 0 | 0 | 01, 14, 15 |
| `franka_experiments/setup.cfg` | build metadata | 8 | documented | — | 0 | 0 | 0 | 14 |
| `franka_experiments/setup.py` | build metadata | 69 | documented | — | 0 | 0 | 0 | 01, 03, 13, 14, 15 |
| `franka_experiments/test/README.md` | test | 401 | partial | — | 0 | 0 | 0 | 00, 04, 05, 06, 07, 08, 09, 10, 11, 12, 13 |
| `franka_experiments/test/_launch_snapshot.py` | test | 138 | partial | — | 0 | 0 | 0 | 14 |
| `franka_experiments/test/config/test_defaults.yaml` | test | 38 | partial | — | 0 | 0 | 0 | 14 |
| `franka_experiments/test/data/README_snapshot.md` | test | 32 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/data/torque_control_stack_94cd7b0_snapshot.json` | test | 2756 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/e2e_pentagon_moveit.sh` | test | 48 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/launch/test_rl_fake.launch.py` | test | 78 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/launch/test_torque_fake.launch.py` | test | 108 | partial | — | 0 | 0 | 0 | 01, 06, 09, 10, 14, 15 |
| `franka_experiments/test/launch/test_velocity_fake.launch.py` | test | 99 | partial | — | 0 | 0 | 0 | 01 |
| `franka_experiments/test/scripts/check_topics.sh` | test | 229 | partial | — | 0 | 0 | 0 |  |
| `franka_experiments/test/test_node_self_references.py` | test | 219 | partial | — | 0 | 0 | 0 | 14 |
| `franka_experiments/test/test_run_metadata.py` | test | 30 | partial | — | 0 | 0 | 0 | 15 |
| `franka_simulation_experiments/.gitignore` | build metadata | 2 | documented | — | 0 | 0 | 0 |  |
| `franka_simulation_experiments/CMakeLists.txt` | build metadata | 23 | documented | — | 0 | 0 | 0 | 14 |
| `franka_simulation_experiments/package.xml` | build metadata | 44 | documented | — | 0 | 0 | 0 | 14, 16 |
| `franka_simulation_experiments/pytest.ini` | build metadata | 7 | documented | — | 0 | 0 | 0 | 14 |

## 16 — Issues and discrepancies → [16_issues_and_discrepancies.md](16_issues_and_discrepancies.md)

13 files, 12081 lines; partial 1, pending 12.

| File | Role | Lines | Status | Class | Prod | Launch | Tests | Cited in |
|---|---|---:|---|---|---:|---:|---:|---|
| `franka_experiments/LEGACY.md` | doc (lead) | 193 | pending | — | 0 | 0 | 0 | 00, 03, 15 |
| `franka_experiments/README.md` | doc (lead) | 438 | pending | — | 0 | 0 | 0 | 00, 04, 05, 06, 07, 08, 09, 10, 11, 12, 13 |
| `franka_experiments/SAFETY.md` | doc (lead) | 302 | pending | — | 0 | 0 | 0 | 00, 11 |
| `franka_experiments/docs/FRAMEWORK_COMPLETE_TECHNICAL_GUIDE.md` | doc (lead) | 5813 | partial | — | 0 | 0 | 0 | 00, 16 |
| `franka_experiments/docs/approach_speed_envelope.md` | doc (lead) | 227 | pending | — | 0 | 0 | 0 | 13 |
| `franka_experiments/docs/ball_throw_closed_loop.md` | doc (lead) | 144 | pending | — | 0 | 0 | 0 |  |
| `franka_experiments/docs/plots/bt3_nom_vs_safe.png` | doc asset | 323 | pending | — | 0 | 0 | 0 |  |
| `franka_experiments/docs/plots/bt3_passes.png` | doc asset | 1148 | pending | — | 0 | 0 | 0 |  |
| `franka_experiments/docs/reactivity_evasion_report.md` | doc (lead) | 220 | pending | — | 0 | 0 | 0 |  |
| `franka_experiments/docs/ros_node_graph.png` | doc asset | 2334 | pending | — | 0 | 0 | 0 |  |
| `franka_experiments/docs/ros_node_graph.svg` | doc asset | 195 | pending | — | 0 | 0 | 0 |  |
| `franka_simulation_experiments/README.md` | doc (lead) | 673 | pending | — | 0 | 0 | 0 | 00, 04, 05, 06, 07, 08, 09, 10, 11, 12, 13 |
| `franka_simulation_experiments/next_step.md` | doc (lead) | 71 | pending | — | 0 | 0 | 0 |  |

