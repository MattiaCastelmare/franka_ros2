# 15 — Code status: what runs, what is optional, what may be dead

> Stage 7, 2026-11-11. Baseline `ba68077`. Labels as in [00_INDEX.md](00_INDEX.md) §2. This chapter
> classifies; it authorizes nothing. **No deletion is authorized in this stage**, and "unused" never
> means "safe to delete" here (§6).

---

## 1. Intuition

A refactor needs to know, for every file, what breaks if it changes or disappears. Whether the main
launch file mentions a file is a poor proxy. In this repository, code is reached in at least eight
ways:
1. a launch file starts an executable;
2. an executable imports a module;
3. a module is imported through a **dynamic name** (`load_robot_config('distance')` reads
   `fr3_distance.yaml`, F-1-14);
4. a sim script imports a `franka_experiments` script by **relative source path** (F-0-5);
5. a user runs a script by hand (`python3 scripts/…`);
6. a test imports it;
7. a configuration flag switches a branch on (most safety mechanisms are such branches);
8. another package or a sim condition sets that flag.

A file can be absent from every launch file and still be load-bearing, through routes 3, 4 or 7.

---

## 2. Classes and evidence

| Class | Definition used here | Typical evidence |
|---|---|---|
| **CURRENT** | Reached by the shipped hardware or simulation pipeline with default arguments, or a tool the documented workflow needs (calibration, replay, analysis of shipped runs) | a launch reference with default arguments; a production importer; an entry point used by a CURRENT launch |
| **EXPERIMENTAL** | Present and wired, but off by default (a flag, a non-default argument, a sim-only condition) | a default `false` in `fr3_control.yaml`/node defaults; enabled only by a `conditions.yaml` entry |
| **ALTERNATIVE** | A supported alternative path with its own entry point (Pipelines B, C, D, RL) | its own launch file or a non-default launch argument |
| **LEGACY** | Superseded, with **explicit** evidence of the successor (commit, docstring and code agreeing) | a `TODO[LEGACY]` marker **confirmed** against the import graph (§5) |
| **UNUSED CANDIDATE** | No consumer found by any of the eight routes of §1, except tests | the static inventory + manual search for dynamic names |
| **UNKNOWN** | Insufficient evidence to choose | stated per row |

**Confidence:**
- **high**: two independent kinds of evidence agree (for example, the import graph and a launch
  default);
- **medium**: one kind of evidence, or a flag whose run-time value depends on the install (F-7-4);
- **low**: inferred from naming, comments or history only.

**Evidence sources used** (all static, at the baseline):
- `tools/stage7_verification/inventory.py`, which collects the static and string imports, the 32
  entry points (22 `console_scripts` plus 10 sim programs), the references in 20 launch files, and the
  test importers;
- the chapters' Verified flag values (ch. 03, 06–13);
- the 22 in-code `TODO[LEGACY]` markers, used only as **leads** (§5, F-7-2);
- `franka_experiments/LEGACY.md` (a lead, 193 lines);
- manual search for the dynamic routes 3–5.

**Test counts** are from `test_traceability.py`, transitive through the test harness modules: the
number of test files that reach the file.

---

## 3. `franka_experiments` — nodes (`franka_experiments/franka_experiments/nodes/`)

| File | Function | Class | Evidence (launch / config) | Key deps | Tests | If modified or removed | Conf. |
|---|---|---|---|---|---:|---|---|
| `cbf_safety_filter.py` | Pipeline A HOCBF-QP filter, q̈_nom → q̈_safe (ch. 07–08) | CURRENT | `torque_control_stack` (default), `bag_replay`, `gz_cbf_stack` via the stack | `cbf_state_rows`, `cbf_qp_assembly`, `cbf_hard_limits`, `state_governor`, `go_around`, `livelock`, `kinematics`, OSQP | 13 | the safety function itself; every change goes through the protocol with closed-loop Gazebo evidence | high |
| `real_time_distance.py` | depth → per-CP surface distances, tracks (ch. 05) | CURRENT | every stack launch, `bag_replay`, `gz_cbf_stack` (`perception:=true`) | `distance_engine`, `mask_builder`, `obstacle_track_pipeline`, `tf_manager`, `self_detection` | 2 (+10 through `distance_engine`) | all obstacle rows depend on it; the only perception path on hardware | high |
| `pentagon_qddot_commander.py` | nominal task-space acceleration, phase governor, singularity recovery (ch. 06) | CURRENT | `torque_control_stack` (`motion_source:=pentagon`, default), `gz_circle_demo` | `trajectory`, `timing_law`, `singularity_recovery`, `kinematics` | 2 | defines q̈_nom; F-3-3, F-3-9, **F-7-1** live here | high |
| `qddot_to_torque.py` | q̈_safe → τ via M(q)q̈ + c + g (ch. 09) | CURRENT | `torque_control_stack`, `gz_circle_demo` | `kinematics`, Pinocchio | 3 | the actuation path; the torque checks of ch. 09 pin it | high |
| `experiment_logger.py` | per-tick CSV + manifest (ch. 13) | CURRENT | every stack launch | `perception_msgs`, `config`, `cbf_utils` | 3 | evidence quality; F-5-21, F-6-13 | high |
| `iso_safety_monitor.py`, `iso_evidence_logger.py` | ISO-inspired SSM monitor and its evidence (ch. 11) | EXPERIMENTAL | started by `torque_control_stack` **only** with `iso_monitor_enabled:=true` / `start_iso_evidence_logger:=true`; both are `false` in `launch_defaults.yaml` (L292, L301), and `iso_enabled: false` in `fr3_control.yaml` | `iso_ssm`, `cbf_state_rows` | 1 / 0 | with `iso_enabled` the filter stops on a stale monitor (`_iso_stop_active`); the shipped `d_safe` fails the ISO floor (test pins it) | high |
| `rl_policy_commander.py` | ONNX policy → q̈_nom (ch. 06 §7) | ALTERNATIVE | `torque_control_stack motion_source:=rl`; needs a model path (F-0-10) | `rl_policy`, onnxruntime | 3 | the RL path only; parity tests depend on `franka_sim` naming (F-0-10) | high |
| `trajectory_visualization_node.py`, `trajectory_overlay_node.py` | RViz markers, camera overlay of the desired/actual paths (ch. 13) | CURRENT (visualisation) | `torque_control_stack` | `trajectory_trace`, `trajectory_overlay` | 0 | display only; no control effect | high |
| `cbf_velocity_filter.py` | Pipeline B velocity ZCBF-QP (ch. 10) | ALTERNATIVE | `velocity_cbf_control_stack` (default QP on) | `ros_setup`, `avoidance_math` (model), `cbf_utils.select_gamma`, qpsolvers | 1 (a replica, §7) | Pipeline B only; carries a stale `TODO[LEGACY]` (F-7-2) | high |
| `ee_pentagon_velocity_commander.py` | Pipeline B nominal (ch. 06 §8) | ALTERNATIVE | `velocity_cbf_control_stack`, `test_velocity_fake` | `trajectory`, `ros` | 1 | Pipeline B only; stale `TODO[LEGACY]` (F-7-2) | high |
| `cbf_OSCBF_filter.py` | Pipeline C Dynamic-OSCBF torque QP (ch. 10) | ALTERNATIVE (unwired) | entry point `cbf_oscbf_filter`, **no** launch (F-0-2) | `cbf_kinematics`, qpsolvers | 0 | a manual `ros2 run` only; obstacle rows inert even when on (F-5-5) | high |
| `pentagon_torque_commander.py` | Pipeline C nominal torque (ch. 10) | ALTERNATIVE (test-only launch) | `test/launch/test_torque_fake.launch.py` only | `ros`, `kinematics` | 1 | test launch only | high |
| `ee_circle_velocity_commander.py`, `ee_random_waypoints_velocity_commander.py` | Pipeline B nominal variants | UNUSED CANDIDATE (entry point, no launch) | entry points only (F-0-2); `TODO[LEGACY]` "superseded by pentagon_qddot_commander" | `ros`, `trajectory` | 0 | none found; could be run manually by `ros2 run` | medium |
| `handeye_calibration_node.py`, `handeye_eye_in_hand_node.py` | eye-to-hand and eye-in-hand calibration (ch. 05) | CURRENT (manual tooling) | the two hand-eye launches | `handeye_solver`, `trajectory`, `ros`/`node_runtime` | 0 / 1 | produce `camera_extrinsics.yaml` / `camera_EE_extrinsic.yaml`; the first feeds the perception TF | high |
| `image_rectifier_node.py`, `tag_overlay_node.py` | calibration image helpers | CURRENT (manual tooling) | `handeye_eye_in_hand_calibration` | cv_bridge, OpenCV | 0 | calibration only | high |
| `capsule_overlay_node.py` | capsule projection overlay (ch. 13) | UNUSED CANDIDATE (entry point, no launch) | entry point only (F-0-2) | `simulation_imports`, `camera_yaml` | 0 | none found in launches; keeps the `simulation_imports`/`camera_yaml` shims alive | medium |
| `frame_grabber.py` | saves camera frames on CBF events (ch. 13) | UNUSED CANDIDATE (entry point, no launch) | entry point only (F-0-2) | `ros` | 0 | manual tool | medium |
| `distance_visualization_node.py` | standalone distance rendering (ch. 13) | UNUSED CANDIDATE | **no entry point**, no importer (F-0-4) | `visualization`, `tf_manager`, `mask_builder`, `distance_utils` | 0 | none found; runnable only as `python3 -m` | high |
| `__init__.py` (package, nodes) | package markers | CURRENT | Python packaging | — | — | required by `setup.py packages` | high |

---

## 4. `franka_experiments` — libraries (`franka_experiments/franka_experiments/utils/`)

### 4.1 Pipeline A core (CURRENT)

| File | Function | Evidence | Tests | If modified or removed | Conf. |
|---|---|---|---:|---|---|
| `cbf_state_rows.py` | every HOCBF row family, margins, snapshots (ch. 07–08) | imported by `cbf_safety_filter` (and ISO nodes, `cbf_qp_assembly`, `evasion_direction`) | 35 | the barrier mathematics; the most-tested module | high |
| `cbf_qp_assembly.py` | row RHS, slack layout, padding, tangential bias (ch. 08) | `cbf_safety_filter`; scripts `cbf_scenarios`, `ball_closed_loop`, `latency_budget` | 16 | QP structure; offline scripts share it, so offline and online results move together | high |
| `cbf_hard_limits.py` | velocity/acceleration/position boxes, braking, slew (ch. 08); `workspace_face_rows` unused (marker confirmed) | `cbf_safety_filter`, `state_governor`, `simexp.reflex_checks`, 3 scripts | 10 | the hard backstop of F-7-1 and every soft-row failure; `workspace_face_rows` is a candidate | high |
| `state_governor.py` | β scaling of the nominal (ch. 06 §6.2) | `cbf_safety_filter` | 3 | nominal shaping | high |
| `cbf_zones.py` | zone ladder (ch. 07 §5) | `cbf_safety_filter`, `cbf_state_rows`, `logging_utils`, `state_governor` | 1 | per-row gains | high |
| `cbf_evasion.py`, `evasion_direction.py` | lateral evasion row and direction (ch. 06 §6.4) | `cbf_state_rows` | 1 / 2 | evasion on by default | high |
| `go_around.py` | go-around (ch. 06 §6.5, ON) | `cbf_safety_filter` | 1 | nominal shaping; Phase 5 conditions turn it on in sim (F-6-4) | high |
| `livelock.py` | livelock escape (ch. 06 §6.7, ON in the default launch) | `cbf_safety_filter`, `cbf_scenarios` | 1 | — | high |
| `cbf_singularity.py` | σ_min row (8.11) | `cbf_state_rows` | 1 | singularity HOCBF (ON) | high |
| `self_collision.py` | URDF capsules, segment/segment distance (8.10) | `cbf_state_rows` (module level); `self_collision_rows_enabled: true` | 3 | self-collision rows; **its `TODO[LEGACY]` is false** (F-7-2) | high |
| `singularity_recovery.py` | joint-space return to home (ch. 06, F-7-1) | `pentagon_qddot_commander`, `singularity_recovery_enabled` default true | 1 | **F-7-1**: fix before hardware use; the test covers v₀ = 0 only | high |
| `kinematics.py` | `CBFKinematics` (Pinocchio FK, Jacobians, J̇q̇, CPs; ch. 04) | 6+ nodes | 6 | every Jacobian in the stack | high |
| `rate_scaling.py` | rate-independent gates (`frames_for`, `ema_alpha_for`, `FrameRateEstimator`) | `cbf_safety_filter`, `real_time_distance`, `cbf_state_rows`, `distance_engine` | 1 | the evidence gates scale with the measured rate (`set_input_rate`) | high |
| `config.py`, `params.py`, `node_runtime.py`, `launch_support.py` | YAML loading, typed parameter declaration, runtime helpers, YAML generation for the stacks (ch. 03) | every node; `torque_control_stack`, `sim_config` | 2 / 0 / 0 / 0 | the parameter surface; `launch_support` writes the controller YAML (F-5-16) | high |
| `perception_msgs.py`, `logging_utils.py`, `cbf_diagnostics.py` | message builders, throttled logging + CBFDIAG, family residuals (ch. 05, 13) | `cbf_safety_filter`, `real_time_distance`, logger, ISO nodes | 10 / 2 / 3 | diagnostics and the message contract | high |
| `constants.py`, `math_utils.py`, `cbf_utils.py` | shared constants, `skew`, `load_robot_config` alias + `select_gamma` | many nodes | 1 / 0 / 1 | `select_gamma` is Pipeline B only (F-7-3 docstring) | high |
| `iso_ssm.py` | ISO SSM mathematics (ch. 11) | ISO nodes, `cbf_state_rows` (`ssm_speed_cap`, OFF) | 3 | EXPERIMENTAL behaviour behind `iso_enabled` | high |

### 4.2 Perception chain (CURRENT unless noted)

| File | Function | Evidence | Tests | If modified or removed | Conf. |
|---|---|---|---:|---|---|
| `distance_engine.py` | per-CP nearest surface (cKDTree), results (ch. 05) | `real_time_distance`; scripts | 10 | every measured gap | high |
| `mask_builder.py`, `self_detection.py` | robot self-mask (dilated silhouette + depth gate) | `real_time_distance` | 1 / 2 | the N2 self-mask ≠ occlusion lead | high |
| `tf_manager.py` | camera/robot TF handling | `real_time_distance`, `distance_visualization_node`, scripts | 0 | camera extrinsics path (ch. 05, F-6-14) | high |
| `obstacle_clusters.py`, `obstacle_track_pipeline.py`, `obstacle_tracker.py` | clustering, tracking, velocity estimation (ch. 05) | `real_time_distance`, `distance_engine` | 2 / 2 / 7 | v_obs and every velocity-dependent margin | high |
| `distance_utils.py`, `visualization.py` | helpers, `VisFrame` overlay | `real_time_distance` | 2 / 0 | — | high |
| `calibration_check.py` | extrinsic sanity check | `real_time_distance`, `iso_constants_measure` | 1 | start-up check | high |
| `obstacle_sim.py` | a synthetic sphere rendered into the depth frame (ch. 05) | `real_time_distance`, only when `sim_obstacle.enabled` (default false, with a loud start-up warning); scripts | 0 | EXPERIMENTAL inside a CURRENT node. If it is ever on by accident, the arm avoids something that is not there | high |
| `ballistic_prediction.py` | ballistic impact prediction (ch. 05 §7.5) | `real_time_distance`, **OFF** | 1 | EXPERIMENTAL | high |
| `self_shadow.py` | self-shadow candidates and barrier (ch. 05 §8.2, ch. 08 G_SSB) | `cbf_state_rows`, `distance_engine`; **OFF** | 3 | EXPERIMENTAL; a pre-commit copy sits in the install (F-7-4) | high |
| `camera_yaml.py`, `simulation_imports.py` | compatibility shims | `capsule_overlay_node` only | 0 | LEGACY (marker confirmed: contents moved to `config.py` / package imports); removable together with `capsule_overlay_node` | high |

### 4.3 Alternative pipelines

| File | Function | Class | Evidence | Tests | Conf. |
|---|---|---|---|---:|---|
| `ros_setup.py` | Pinocchio model and joint-state callback for Pipeline B (ch. 10) | ALTERNATIVE | `cbf_velocity_filter` | 0 | high |
| `avoidance_math.py` | reduced-model builder, joint ordering, legacy capsule/AABB geometry (28 functions) | ALTERNATIVE (partly) | `ros_setup` imports 4 functions, of which `build_reduced_pinocchio_model_from_urdf` and `ordered_joint_positions_from_joint_state` run in Pipeline B; the rest are reached via `simulation_imports` only. **Stale marker** (F-7-2) | 0 | high |
| `cbf_kinematics.py` | shim to `kinematics.CBFKinematics` | LEGACY (shim, confirmed) | `cbf_OSCBF_filter`, `cbf_constraints` | 0 | high |
| `rl_policy.py` | ONNX policy wrapper, observation layout (ch. 06 §7) | ALTERNATIVE | `rl_policy_commander` | 2 | high |
| `ros.py` | facade over `node_runtime`/`launch_support`/`config` | LEGACY but still imported (marker confirmed) | 5 nodes + `minimal`, `thales`, `velocity_cbf_control_stack` | 2 | high |
| `trajectory.py`, `timing_law.py` | paths and timing laws (ch. 06 §3) | CURRENT | `pentagon_qddot_commander`, Pipeline B commanders, calibration | 0 | high |
| `trajectory_overlay.py`, `trajectory_trace.py` | overlay drawing, marker traces | CURRENT (visualisation) | overlay/visualisation nodes, `gz_trajectory_markers` | 1 / 2 | high |
| `handeye_solver.py` | AX = YB solver (ch. 05) | CURRENT (calibration) | `handeye_eye_in_hand_node` | 1 | high |

### 4.4 No production consumer (UNUSED CANDIDATE or LEGACY)

| File | What it is | Consumers found | Class | Before deleting, check | Conf. |
|---|---|---|---|---|---|
| `cbf_constraints.py` | velocity-era ZCBF rows (F-3-1) | none (a comment in `cbf_OSCBF_filter`) | LEGACY (marker confirmed: superseded by the HOCBF rows) | the `franka_rl`/`fr3_student_sim` trees, out of scope | high |
| `cbf_qp.py` | standalone OSQP wrapper | none | LEGACY (marker confirmed: the filter builds OSQP inline) | as above | high |
| `node_utils.py` | re-export shim (`PerfTimer`, message builders) | none | LEGACY (shim, confirmed) | external user scripts | high |
| `rtd_debug.py` | RTD profiling helpers (`debug_extreme`) | none; `debug_extreme` appears nowhere else | UNUSED CANDIDATE | whether `real_time_distance` once declared `debug_extreme` (history) | high |
| `avoidance.py` | avoidance-first shaping helpers | `test_avoidance.py` only; the governor using it was reverted (`4d4d450`) | UNUSED CANDIDATE | the test file | high |
| `static_scene.py`, `surface_velocity.py` | static-scene hold, surface-normal speed (F-0-3) | each other + tests; "not yet wired" (`2402e58`) | EXPERIMENTAL (unwired) | `stash@{1}` mentions them; the user's intent (Q-7-2) | medium |
| `run_metadata.py` | per-node provenance | `test_run_metadata.py` only | UNUSED CANDIDATE | `simexp.provenance` (F-6-11: also never called) | high |

---

## 5. The `TODO[LEGACY]` markers against the evidence (F-7-2)

All 22 markers carry "flagged: 2026-09-01" and predate many later commits.

| Marker on | Marker claim | Baseline evidence | Verdict |
|---|---|---|---|
| `utils/self_collision.py` | "imported only by test_cbf_hard_constraints" | `cbf_state_rows` L49, module level; rows ON | **false**: CURRENT |
| `utils/avoidance_math.py` | "only importer is simulation_imports" | `ros_setup` → `cbf_velocity_filter` | **false**: ALTERNATIVE |
| `nodes/cbf_velocity_filter.py`, `ee_pentagon_velocity_commander.py` | "velocity-space mode; superseded" | started by `velocity_cbf_control_stack` | **misleading**: ALTERNATIVE, not dead |
| `nodes/ee_circle_…`, `ee_random_waypoints_…` | same | entry points only | consistent: UNUSED CANDIDATE |
| `nodes/cbf_OSCBF_filter.py`, `pentagon_torque_commander.py` | "no launch file starts it" | true (test launch for the commander) | consistent: ALTERNATIVE (unwired) |
| `utils/cbf_constraints.py`, `cbf_qp.py`, `rtd_debug.py`, `node_utils.py`, `cbf_kinematics.py`, `camera_yaml.py`, `ros.py` | no importer / shim | confirmed by the inventory | consistent |
| `utils/simulation_imports.py` | single consumer (`capsule_overlay_node`) | confirmed | consistent |
| `utils/avoidance.py` | test-only | confirmed | consistent |
| `utils/cbf_hard_limits.py::workspace_face_rows` | test-only | confirmed (grep) | consistent |
| `utils/config.py` L271 | duplicate loader | not re-examined | UNKNOWN |
| `nodes/experiment_logger.py` L112 | `cfg_topics` built, never read | consistent with ch. 13 | consistent (dead local code) |
| `nodes/qddot_to_torque.py` L197 | callback name is a misnomer | consistent with ch. 09 | naming only |
| `nodes/pentagon_qddot_commander.py` L128 | `reset_thr_m` read, never used | not re-examined | UNKNOWN |

**Rule:** a marker is a lead. Four of 22 are false or misleading, so none can justify a deletion on
its own.

---

## 6. Launch files, configuration and scripts

### 6.1 Launch files

The table is in [14 §6](14_build_launch_test.md#6-launch-files), with the classes: CURRENT
(`torque_control_stack`, `minimal`, `bag_replay`, the hand-eye pair (`handeye_calibration_bringup`, `handeye_eye_in_hand_calibration`), `gz_cbf_stack`, `gz_plant`,
`gz_circle_demo`) and ALTERNATIVE (`velocity_cbf_control_stack`, `thales`).

### 6.2 Configuration (`franka_experiments/config/`, `franka_simulation_experiments/config/`)

| File | Reader (Verified) | Class | Conf. |
|---|---|---|---|
| `fr3_control.yaml` | every Pipeline A node, ISO nodes, `simexp` | CURRENT: the main parameter file (2672 lines; ch. 03) | high |
| `fr3_complete.yaml` | `real_time_distance`, the stacks, `kinematics`, `distance_utils` | CURRENT: perception and robot geometry | high |
| `launch_defaults.yaml` | every stack launch and the hand-eye launches | CURRENT | high |
| `camera_extrinsics.yaml`, `camera_link_extrinsics.yaml` | stacks' TF; `real_time_distance`, `self_detection`, `trajectory_overlay`; `torque_control_stack` | CURRENT (calibration output) | high |
| `depth_intrinsics.yaml`, `rgb_intrinsics.yaml` | `config.py`, `camera_yaml`, `gz_world`, `sim_config`; `capsule_overlay_node` | CURRENT / UNUSED CANDIDATE (rgb: overlay only) | medium |
| `fr3_distance.yaml` | `cbf_velocity_filter` via `load_robot_config('distance')`, a **dynamic name** | ALTERNATIVE | high |
| `trajectory_rviz.rviz`, `handeye_rviz.rviz`, `handeye_eye_in_hand_rviz.rviz` | `launch_support`, hand-eye launches | CURRENT (display) | high |
| `camera_EE_extrinsic.yaml`, `dataset_handeye_eye_in_hand.yaml`, `ee_tag_extrinsics.yaml` | written by the calibration nodes; the D405 is "recording only" (`launch_defaults.yaml` L54) | CURRENT (calibration artefacts) | medium |
| `oscbf_params.yaml`, `fake_hw_controller_params.yaml`, `camera_intrinsics.yaml` | **no reader** (ch. 03, F-1-14) | UNUSED CANDIDATE | high |
| `safety/watchman_profile.md` | documentation (not installed) | — | — |
| `fse/config/conditions.yaml`, `scenarios.yaml`, `conditions.d/mppi_it08.yaml` | `simexp.conditions`, `scenario_catalog` (ch. 12) | CURRENT (simulation axes) | high |
| `fse/config/control_regression_scenarios.yaml` | `test_control_regression.py` only (the batch that would use it is `run_control_regression.py`, absent from the baseline but present in the install, F-7-4) | UNUSED CANDIDATE | medium |

### 6.3 `franka_experiments/scripts/` (manual tools; none is started by default except `iso_preflight_check.py`)

| Group | Scripts | Class | Evidence | Conf. |
|---|---|---|---|---|
| launch-time guard | `iso_preflight_check.py`, `pin_rt_thread.sh` | CURRENT | `torque_control_stack` | high |
| offline closed-loop reproduction | `cbf_scenarios.py`, `ball_closed_loop.py`, `compare_vobs.py`, `latency_budget.py` | CURRENT (analysis); `cbf_scenarios`/`compare_vobs` are **imported by `simexp`** (F-0-5): renaming them breaks the sim harness | `simexp.emulation`, `estimator_metrics` | high |
| run analysis | `ball_throw_eval.py`, `approach_probe.py`, `approach_envelope.py`, `smoothness_report.py`, `pass_plot.py`, `throw_*.py`, `experiment_summary.py`, `analyze_isolation_ab.py`, `cbf_phase4_check.py`, `check_control_derivatives.py`, `compare_range_noise.py`, `range_noise_calibration.py`, `plot_rl_timing.py`, `bag_to_mp4.py` | CURRENT (manual analysis); `experiment_summary` and `smoothness_report` are imported by `simexp.runner` | inventory | medium |
| live monitors | `pipeline_rate_monitor.py`, `watch_obstacle_tracks.py` | CURRENT (manual) | docstrings | medium |
| ISO measurement | `iso_constants_measure.py`, `iso_evidence_report.py`, `iso_pfl_speed.py` | EXPERIMENTAL (ISO layer off) | ch. 11 | high |
| generators | `gen_zero_gain_controllers_yaml.py`, `docs/make_ros_node_graph.py` | UNKNOWN / documentation tool | no consumer; manual | low |
| log plotter (outside the ledger, F-7-11) | `franka_logs/plot_franka_log.py` (468 lines; plots a `pentagon_qddot_commander` CSV) | CURRENT (manual analysis) | docstring; no importer at the baseline (`git grep`) | medium |

### 6.4 `franka_simulation_experiments`

| Component | Class | Evidence |
|---|---|---|
| `gz_cbf_stack.launch.py`, the six sim nodes (`episode_supervisor`, `ground_truth_node`, `reflex_monitor`, `scenario_commander`, `emulated_perception`, `pin_process`), `run_gz_scenarios.py`, `simexp/{analysis, batch, closed_loop, conditions, emulation, obstacle_laws, procs, reflex_checks, runner, scenario_catalog, scenario_laws, sim_config, truth_geometry, report, estimator_metrics, attribution, offline}` | CURRENT (ch. 12) | launch references and the import chain from `run_gz_scenarios` → `batch` |
| `gz_plant.launch.py`, `gz_circle_demo.launch.py`, `gz_trajectory_markers.py` | CURRENT (sim tooling) | `gz_circle_demo` includes `gz_plant` |
| `check_self_occlusion.py`, `plot_control_regression.py` | CURRENT (manual tools) | installed programs; no launch |
| `simexp/control_regression.py`, `graph_contract.py` | CURRENT (test infrastructure) | `launch_test_graph_contract.py`, tests |
| `simexp/provenance.py` | UNUSED CANDIDATE (F-6-11: never called) | tests only |

---

## 7. Configuration-dependent mechanisms (the EXPERIMENTAL layer inside CURRENT files)

Shipped values in the hardware-default launch (Verified in the cited chapters):

| Mechanism | Default | Chapter |
|---|---|---|
| obstacle, joint-limit, self-collision and singularity HOCBF rows; hard boxes; slew | ON | 08 §2 |
| retreat cap, task-space speed cap | ON (hw), OFF (`minimal_l1`) | 08 §2 |
| zone ladder, standoff margin h_std, `enable_vobs_in_hdot` | ON | 07 §3–5 |
| go-around, livelock escape, lateral evasion, singularity recovery | ON | 06 §4–6 |
| curvature term κ, sensor-range margin h_rng, evidence margin h_ev, feedforward h_brake | OFF | 07 §3.5–3.6 |
| memory rows, weighted slack (`enable_weighted_slack`, `fr3_control.yaml` L2475, read in Stage 7), outrun evasion | OFF | 07 §6, 06 §6.6 |
| self-shadow barrier (G_SSB), ballistic prediction | OFF | 05 §7.5, §8.2; 08 |
| ISO layer (`iso_enabled`), SSM speed rows | OFF | 11 |
| OSCBF obstacle/velocity/workspace rows | OFF | 10 §3 |

**These are classifications of the source at the baseline. The installed tree (F-7-4) carries other
values**, for example `d_safe` 0.15. A flag's run-time value must be read from the running node
(`ros2 param get`) after a clean build, not from this table.

---

## 8. Summary

| Class | Production files (approx.) | Notes |
|---|---:|---|
| CURRENT | ≈ 115 | including visualisation, calibration, analysis tools and sim infrastructure |
| EXPERIMENTAL | ≈ 10 files + 14 mechanisms of §7 | ISO layer, SSB, ballistic, static scene |
| ALTERNATIVE | ≈ 14 | Pipelines B, C, RL; `thales`; `fr3_distance.yaml` |
| LEGACY (confirmed) | 8 | `cbf_constraints`, `cbf_qp`, `node_utils`, `cbf_kinematics`, `camera_yaml`, `simulation_imports`, `ros` (still imported), `workspace_face_rows` |
| UNUSED CANDIDATE | ≈ 14 | §3, §4.4, §6.2, §6.4 |
| UNKNOWN | 3 + 2 markers | `gen_zero_gain_controllers_yaml`, `make_ros_node_graph`, two unexamined markers |

The counts are approximate because some files carry two classes: an EXPERIMENTAL mechanism inside
a CURRENT node. `check_docs.py` (Stage 7) verifies that every non-test Python, launch and
configuration file of both packages is named in this chapter.

---

## 9. Consequences of modification and removal

1. **Before removing an UNUSED CANDIDATE:**
   - rerun `inventory.py` on the branch being changed;
   - search the out-of-scope trees (`franka_rl/`, `fr3_student_sim/`, `integration_launch_testing/`)
     and the user's scripts;
   - check the git history for the reason it exists;
   - **ask the user**: several files encode experiment intent (Q-7-2).
2. **A deletion does not reach an existing install** (F-7-4). Validate removals only after the clean
   build of 14 §5.3.
3. **The LEGACY shims** (`ros.py`, `cbf_kinematics`, `camera_yaml`, `simulation_imports`,
   `node_utils`) are import indirections. Repoint their importers first, one change each, as
   `LEGACY.md` §I already proposes (a lead).
4. **Do not remove EXPERIMENTAL safety mechanisms because they are off.** SSB, memory rows, the ISO
   layer and the evidence margin are documented answers to observed failures (ch. 05 §8.2, 07 §6,
   11). Whether they stay is a research decision for the user, not a cleanup.
5. **`cbf_scenarios.py` and `compare_vobs.py` look like scripts but are libraries** for `simexp`
   (F-0-5). Treat them as CURRENT production code.
