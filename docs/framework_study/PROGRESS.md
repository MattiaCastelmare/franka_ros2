# Progress tracker

This file is the single place that records where the documentation stands. Every stage ends by
updating it and then **stopping for review**. A stage starts only after explicit approval.
Project rules: [/CLAUDE.md](../../CLAUDE.md). Session history: [CHECKPOINTS.md](CHECKPOINTS.md).

## 0. Resume card (overwritten at every checkpoint)

- **Last updated:** 2026-11-11, checkpoint C-035 (Stage 5 complete; parallel-session reconciliation recorded; awaiting review)
- **Project status:** read-only documentation phase. Stages 0–4 are done, committed and pushed (HEAD `a66a134`).
  Stage 5 is done in the working tree, **not committed** (the user did not ask for a commit).
- **Last completed stage:** Stage 5 (awaiting review).
- **Next pending stage:** Stage 6 (Simulation): `12`. **Not approved.**
- **Stage in progress:** none. Stage 5 (approved 2026-11-11, ONLY Stage 5) finished at C-035.
- **Pinned branch:** `humble-mattia`
- **Documented source baseline:** `ba68077052e8d0fb3d4606c05dff553b75baa641`. This is the commit whose
  ROS 2 source the chapters describe; the history is in §8.
  - It is **not** the current HEAD. Commits that touch only `docs/framework_study/` or `CLAUDE.md`
    move HEAD but leave the baseline valid.
  - Any change to the source since this commit makes `tools/check_state.py` report REVIEW (exit code 2).
  - Stashes: 3, never touched.
- **Completed documentation files:**
  - `00_INDEX.md`, `01_architecture.md`, `02_ros_graph_and_dataflow.md`, `03_configuration.md`,
    `04_math_foundations.md`, `05_perception_and_distance.md`, `06_nominal_motion.md`,
    `07_cbf_obstacle_barrier.md`, `08_constraints_and_qp.md`, `09_torque_realization.md`,
    `10_alternative_controllers.md`, `11_iso_safety_layer.md`, `13_observability_and_tools.md`;
  - `ledger/*`;
  - the tools `stage0_extract.py`, `ledger_status.py` and `check_state.py` (baseline-aware since C-003);
  - `tools/stage2_verification/`: 9 read-only scripts reproducing every numerical result of ch. 04 and
    05, `run_all.sh`, a README, and the recorded output `results/observed_2026-11-11.txt`
    (126 of 126 checks passed);
  - Stage 3 audit records: 06 §13, 07 §14;
  - `tools/stage4_verification/`: 5 scripts + `_common4.py` (4 container scripts on the deployed
    osqp 0.6.7 and Pinocchio 4.0.0, 1 host), `run_all.sh`, a README, and
    `results/observed_2026-11-11.txt` (61 of 61 passed);
  - `tools/stage5_verification/`: 5 scripts + `_common5.py` (4 container scripts calling production
    methods unbound on stubs, with the deployed qpsolvers 4.3.3 / osqp 0.6.7; 1 host), `run_all.sh`,
    a README, and `results/observed_2026-11-11.txt` (92 of 92 passed);
  - `tools/stage3_verification/`: 6 scripts + 2 helpers (4 host, 2 container driving the real
    `ConstraintBuilder`), `run_all.sh`, a README, `results/observed_2026-11-11.txt` (85 of 85 passed);
  - `CHECKPOINTS.md` and `RESUME_PROMPT.md`;
  - the root `CLAUDE.md`.
- **Draft or stale:** `A_file_index.md` was generated in Stage 0 and does not reflect the Stage 1 chapter
  reassignments (§2). It gets regenerated in Stage 7.
- **Incomplete work carried forward:**
  - 107 ledger files are `partial`. For nodes, that mostly means the constructor interfaces only;
    the algorithms belong to the stage that owns the chapter. Stage 3 partials are listed in §2.
  - Ch. 05 files still `pending` or `partial`, read at interface level only: the eye-in-hand node, the
    hand-eye launch files, `rtd_debug.py`, `node_utils.py`, the remaining calibration YAMLs, and most
    ch. 05 tests (names cross-referenced only). They are covered by the chapter at interface level;
    a deeper read is optional (Stage 7 coverage pass).
  - Unreachability candidates (F-0-4) are not yet confirmed (Stage 7); Stage 2 confirmed
    `static_scene.py`, `surface_velocity.py` and `self_shadow.void_spheres_from_rays` as unused.
  - The guide-versus-code cross-check has not started (Stage 7, chapter 16).
  - User questions: Q-1-4, Q-1-5, Q-2-2, Q-3-1, Q-4-1, Q-4-2. External or hardware: Q-4-3. Runtime
    tests that need approval: Q-2-3, Q-2-4, Q-3-2. Answered: Q-2-1 (F-3-2, F-3-13), Q-1-1 (09 §5.1).
  - Ch. 08 files not read in Stage 4: `self_collision.py` (capsule parsing and segment distance,
    used through `SelfCollisionRowBuilder`), `singularity_recovery.py` (the commander's recovery, 06
    §4.4), `pin_rt_thread.sh`, and the ch. 08 tests (names only). Stage 7 coverage pass.
  - Stage 5 partials (§2): the 25 ch. 13 analysis scripts, the visualisation nodes and the ch. 13
    tests were read at interface level; `iso_evidence_report.py`, `iso_constants_measure.py`,
    `iso_pfl_speed.py`, `watchman_profile.md` and the ten `test_iso_*` in the parts ch. 11 cites;
    `pentagon_torque_commander.py` (the orientation law and the interface).
  - Stage 5 questions: Q-5-1 to Q-5-4 (§5).
  - A parallel session wrote into Stage 5 files on 2026-11-11; it was reconciled at C-035 (its
    findings are F-5-24…F-5-27, its ch. 11 draft is in `drafts/`). Before resuming, make sure only
    one session writes to `docs/framework_study/`.
- **Exact next action:** wait for the user's review of Stage 5 and, separately, an explicit approval
  of Stage 6 (`12_simulation.md`). Do not start Stage 6 without it.
- **Resume procedure:** follow "Resuming a session" in `/CLAUDE.md`, or paste
  [RESUME_PROMPT.md](RESUME_PROMPT.md).

## 1. Stages

| Stage | Deliverables | Files in ledger | Status | Verified against baseline | Reviewed / approved |
|---|---|---:|---|---|---|
| 0 — Inventory | `00_INDEX.md`, `PROGRESS.md`, `A_file_index.md` (draft), `ledger/*`, `tools/stage0_extract.py` | 346 enumerated | **done** | `ba68077` | approved 2026-11-11 |
| 1 — Architecture & graph | `01_architecture.md`, `02_ros_graph_and_dataflow.md`, `03_configuration.md`; `tools/ledger_status.py` | ch. 01: 5 · 02: 4 · 03: 13, after reassignment; plus about 30 node, launch and C++ files read at interface level | **done** | `ba68077` (tree clean apart from `docs/framework_study/`) | acknowledged complete 2026-11-11 (Q-1-4, Q-1-5 still open) |
| 2 — Foundations & perception | `04`, `05` | ch. 04: 5 · 05: 62 | **done** (awaiting review) | `ba68077` | approved 2026-11-11 |
| 3 — Nominal motion & barrier | `06`, `07`; `tools/stage3_verification/` | ch. 06: 25 · 07: 30 (see §2 for what was read) | **done** (awaiting review) | `ba68077` | approved 2026-11-11 (re-approved in a new session after an interruption) |
| 4 — QP & actuation | `08`, `09`; `tools/stage4_verification/` | ch. 08: 28 · 09: 6 (§2 lists what was read) | **done** (awaiting review) | `ba68077` | approved 2026-11-11 |
| 5 — Alternatives, ISO, tools | `10`, `11`, `13`; `tools/stage5_verification/` | ch. 10: 9 · 11: 18 · 13: 43 (§2 lists what was read) | **done** (awaiting review) | `ba68077` | approved 2026-11-11 |
| 6 — Simulation | `12` | ch. 12: 63 | not started | | |
| 7 — Synthesis | `14`, `15`, `16`, `B`, `C`, final `00`/`A` | ch. 14: 22 · 16: 13 | not started | | |

The per-chapter file counts come from the provisional assignment in
[ledger/coverage_ledger.md](ledger/coverage_ledger.md). Files are reassigned by hand when reading
shows a better home, and the counts here are then updated.

## 2. Coverage status

| Status | Files |
|---|---:|
| pending | 178 |
| partial | 107 |
| read | 4 |
| documented | 57 |

`partial` was added in Stage 1. It means the parts the stage needed were read; for nodes, that is
the constructor interfaces. Chapter reassignments made in Stage 1:
- `utils/ros_setup.py` → ch. 10 (it is the velocity filter's helper);
- `utils/node_utils.py` → ch. 05;
- `utils/run_metadata.py` → ch. 13;
- the two hand-eye launch files → ch. 05.

Stage 3 (2026-11-11):
- `documented`: `utils/cbf_zones.py`, `utils/cbf_constraints.py` (unused; F-3-1).
- `partial`, chapter 07: `utils/cbf_state_rows.py`. The obstacle family, the assembly and the
  helpers were read; the self-collision, joint-limit, singularity, SSB, rate-cap and speed-row
  builders belong to ch. 08. `utils/cbf_evasion.py` was read by a subagent; its call sites were
  Verified.
- `partial`, chapter 06: `pentagon_qddot_commander.py` (everything except `_tick_isolation` and
  the logger internals). Through subagent reading with spot checks: `rl_policy_commander.py`, the
  three `ee_*` commanders, `timing_law.py` (`LinearTimingLaw` read by me), `trajectory.py` (Circular,
  Linear, Composite read by me), `evasion_direction.py`, `go_around.py`, `livelock.py`,
  `rl_policy.py`, `avoidance.py`, `avoidance_math.py`. Also `test/test_phase_governor.py`
  (constants only).
- `partial`, chapter 08, read for ch. 06/07: `cbf_qp_assembly.py` (`build_row_rhs`,
  `tangential_bias`), `state_governor.py`, `rate_scaling.py` (`frames_for`, `ema_alpha_for`).
- The ch. 06/07 tests remain `pending`: they were cited by name only.
- Stage 3 audit (C-017 to C-019): `cbf_evasion.py`, `go_around.py`, `evasion_direction.py` and
  `livelock.py` were read in full by the main session → `documented`. `rl_policy.py`,
  `rl_policy_commander.py` and `ee_pentagon_velocity_commander.py` were re-read in the parts the
  chapter uses, so they stay `partial`.

Stage 4 (2026-11-11):
- `documented`, chapter 09: `qddot_to_torque.py`, `rt_torque_controller.cpp`,
  `cbf_torque_controller.cpp`, `franka_rt_controllers.xml`, `config/cbf_torque_controller.yaml`
  (unreferenced).
- `documented`, chapter 08: `cbf_qp_assembly.py`, `cbf_singularity.py`, `cbf_hard_limits.py`
  (`workspace_face_rows` and `velocity_accel_box` are noted as unused or superseded),
  `state_governor.py`, `cbf_qp.py` (no importer).
- `partial`: `cbf_safety_filter.py` (everything on the QP path; `_report`, `_publish_status`,
  `_iso_configure` and `_elevate_thread_priority` belong to ch. 11/13), `rate_scaling.py`
  (`FrameRateEstimator` not read).
- Also read, outside the Stage 4 assignment: `launch_support._generate_torque_yaml` and
  `_generate_cbf_base_yaml`, `minimal.launch.py` (controller selection), `gz_fr3_plant.launch.py`,
  `gz_robot_description` (docstring, ARMATURE), `sim_config.rt_controllers_yaml`,
  `franka_ign_ros2_control/src/ign_system.cpp::write` (interface-only package).

Stage 5 (2026-11-11):
- `documented`, chapter 10: `cbf_velocity_filter.py`, `cbf_OSCBF_filter.py`,
  `rt_velocity_blender_controller.cpp`, `rt_velocity_blender.yaml`, `rt_velocity_blender_controller.xml`,
  `test_cbf_velocity_filter.py`, `test_torque_stack_external_plant_off_identical.py`.
- `documented`, chapter 11: `iso_ssm.py`, `iso_safety_monitor.py`, `iso_evidence_logger.py`,
  `iso_preflight_check.py`.
- `documented`, chapter 13: `cbf_diagnostics.py`, `run_metadata.py`, `simulation_imports.py`,
  `nodes/__init__.py` (empty).
- `partial`, chapter 10: `ros_setup.py`, `pentagon_torque_commander.py`.
- `partial`, chapter 11: `iso_evidence_report.py`, `iso_constants_measure.py`, `iso_pfl_speed.py`,
  `watchman_profile.md`, the ten `test_iso_*` (the assertions ch. 11 cites).
- `partial`, chapter 13: `experiment_logger.py` (constructor, header, every callback, `sample` up to
  the row build, the manifest; the plot functions not read), `logging_utils.py` (`format_cbf_diag`),
  the five visualisation nodes and three utils modules, `trajectory_rviz.rviz`, the 25 scripts
  (interface level: docstring, inputs, outputs, whether they move the arm), the ch. 13 tests and the
  golden header (names and docstrings), `docs/make_ros_node_graph.py`.
- Also read, outside the assignment: the torque-stack rosbag block and finger publisher, the
  `bag_replay.launch.py` header, `frame_grabber`'s status test, `cbf_safety_filter._iso_configure`,
  `_publish_status` and the ISO-stop branch of `_qp_tick` (ch. 11), the velocity stack launch and
  `franka_rt_controllers/launch/rt_velocity_blender.launch.py`.

## 3. How later stages are tracked (procedure)

1. **At stage start:** run `python3 -I docs/framework_study/tools/check_state.py`.
   - Exit code 0: proceed.
   - Exit code 2 (REVIEW: the source changed since the documented baseline): follow §8 before
     writing new chapters.
   - Exit code 1: fix the documentation state first.

   Record the HEAD and the baseline in the checkpoint.
2. **While reading:** set each file's status with
   `python3 -I docs/framework_study/tools/ledger_status.py <status> <path…> [--ch NN --stage N]`.
   That updates both `ledger.json` and `coverage_ledger.md`. Use `partial` for parts read, `read`
   once every function in the file has been read, and `documented` once the chapter text covers
   it. Each top-level symbol in
   `ledger/module_definitions.md` must be either explained or explicitly marked "not relevant:
   <reason>" in its chapter.
3. **Interfaces:** each row in `ledger/ros_interfaces.md` and `ledger/launch_inventory.md` that a
   chapter resolves (concrete topic name, type, rate, QoS) moves into that chapter as
   **Verified**. The ledger keeps the raw extraction.
4. **Findings:** every bug, inconsistency or math-vs-code deviation found along the way is
   appended to §4 below with an ID (`F-<stage>-<n>`), and Stage 7 consolidates them into chapter 16.
5. **At stage end:** update §1 and §2, list the open questions in §5, write a checkpoint (§6), run
   `tools/check_state.py`, and stop for review.
6. **Regenerating the extraction** (optional, read-only):
   `python3 -I docs/framework_study/tools/stage0_extract.py . <out_dir>`, then diff against `ledger/`.
   The script overwrites only `<out_dir>`, so point it at a scratch directory and do not regenerate
   in place once statuses have been edited by hand.

## 4. Findings log (raw; consolidated in Stage 7)

| ID | Finding | Evidence | Label |
|---|---|---|---|
| F-0-1 | An untracked test file listed at session start (`franka_experiments/test/test_status_residuals_and_qp_scheduling.py`) no longer exists; it is not in any commit or stash. | `git status`, `git log --all -- <path>`, `git stash show` | Verified |
| F-0-2 | 6 of 22 console scripts are started by **no** production launch file in the two packages: `capsule_overlay_node`, `cbf_oscbf_filter`, `ee_circle_velocity_commander`, `ee_random_waypoints_velocity_commander`, `frame_grabber` and `pentagon_torque_commander`. `pentagon_torque_commander` is started only by `test/launch/test_torque_fake.launch.py`. The whole OSCBF torque-level pipeline therefore has no production launch path at HEAD. | `ledger/import_graph.md` (entry-point table); `grep` over every `*.launch.py` | Verified (static) |
| F-0-3 | `utils/static_scene.py` and `utils/surface_velocity.py` are imported only by each other and by tests. Commit `2402e58` says "not yet wired", and `stash@{1}` mentions "static-scene-truth". | `ledger/import_graph.md`; `git log` | Verified |
| F-0-4 | Library modules reached by no executable: `nodes/distance_visualization_node.py` (no entry point), `utils/avoidance.py` (tests only), `cbf_constraints.py` (mentioned only in a comment of `cbf_OSCBF_filter.py`), `cbf_qp.py`, `node_utils.py`, `rtd_debug.py`, `run_metadata.py` (tests only). In the simulation package, `simexp/control_regression.py`, `graph_contract.py` and `provenance.py` are reached by tests only. These are candidates; Stage 7 confirms them against dynamic use. | `ledger/import_graph.md` | Extracted |
| F-0-5 | `franka_simulation_experiments` imports `franka_experiments/scripts/cbf_scenarios.py` and `compare_vobs.py` **by relative source path** (`simexp/emulation.py`, `simexp/estimator_metrics.py`). This works only from a `src/<pkg>` source layout. | source | Verified |
| F-0-6 | `franka_experiments/package.xml` does not declare `franka_rt_controllers`, although the shipped torque and velocity pipelines spawn its plugins and `launch_support.py` writes their YAML. | `package.xml`, `utils/launch_support.py` | Verified |
| F-0-7 | Naming mismatch: the velocity pipeline spawns `rt_velocity_executor_controller` (type `franka_rt_controllers/RtVelocityExecutorController`), implemented in `rt_velocity_blender_controller.cpp`. The node and docs say "blender". | `launch_support.py` L182; `.cpp` L423 | Verified |
| F-0-8 | The large nodes (`cbf_safety_filter`, `real_time_distance`, `qddot_to_torque`, `cbf_OSCBF_filter`) show 0–2 direct `declare_parameter` calls. Their parameters are declared through `utils/params.py` (5 call sites) and `utils/config.py`, so the parameter surface is not visible per node until Stage 1 resolves the helpers. | `ledger/ros_interfaces.md` | Extracted |
| F-0-9 | The existing `FRAMEWORK_COMPLETE_TECHNICAL_GUIDE.md` was last changed in `ce80f36`. 19 commits touching the two packages have landed since, including the curvature term, per-row ℓ1 slack with brake fallback, evidence conditioning, and the self-shadow barrier, so parts of it are probably stale. | `git log ce80f36..HEAD` | Verified (that commits exist); staleness Inferred |
| F-0-10 | `utils/rl_policy.py::find_sim_root` locates a directory named `franka_sim`, while the directory on disk is `franka_rl/`. A project note says 9 parity tests skip silently because of this. | source; project note | source Verified, test-skip effect Unverified |

### Stage 1 findings

All are Static unless labelled otherwise. Chapter references point to the full discussion.

| ID | Finding | Where | Label |
|---|---|---|---|
| F-1-1 | The namespace `NS_1` comes from `franka_bringup/config/franka.config.yaml` (`ROBOT1.namespace`), not from this repository. It is then hard-coded as `/NS_1/...` in `fr3_control.yaml`, node defaults, `launch_support.py` (`accel_topic`), simulation scripts and `gz_fr3_plant.launch.py`. | 02 §2.1 | Verified |
| F-1-2 | `launch_defaults.yaml` `robot_ip: 192.168.1.10` (and `namespace: ""`) are always overridden by the bringup file (`192.168.2.10`, `NS_1`), because of the merge order `{**launch, **bringup}`. | 02 §2.1, 03 §2 | Verified; runtime (recorded) launch log shows `ip=192.168.2.10` |
| F-1-3 | The `torque_control_stack.launch.py` docstring says `rt_torque_controller` "adds g(q)". Neither the controller nor `qddot_to_torque` adds gravity: the FR3 firmware does, according to `test_torque_fake.launch.py`. | 01 §2 | Verified (source); firmware behaviour from documentation |
| F-1-4 | The `velocity_cbf_control_stack.launch.py` docstring says the filter reads `/human_robot/multi_distance`; the code subscribes to `/cbf/per_link_distances`. | 01 §4.2 | Verified |
| F-1-5 | `start_move_group` defaults to `true`, and the torque launch comment says the commander uses `compute_fk`/`compute_cartesian_path`. No Pipeline A node uses MoveIt. | 01 §4.1, 02 §3.2 | Verified (grep) |
| F-1-6 | Pipeline C (OSCBF): no production launch; `/NS_1/torque_safe` has no configured consumer; its `/cbf/per_link_distances` subscription is RELIABLE against BEST_EFFORT publishers. | 01 §4.3, 02 §7.2 | Static; no reception is Inferred |
| F-1-7 | `minimal` and `thales` publish `world → fr3_link0`, which conflicts with RSP's `base → fr3_link0`. `minimal`, `thales` and the velocity stack publish a static `base → camera_color_optical_frame`, which conflicts with the RealSense driver's own chain if it publishes TF. | 02 §5 | Static; conflict Inferred |
| F-1-8 | `lpf_alpha`, `tau_max_scale` and `urdf_path` are written into the `rt_torque_controller` YAML but never declared by the C++, so the launch arguments `lpf_alpha` and `tau_max_scale` are inert. | 02 §3.6, 03 §3.5 | Verified |
| F-1-9 | Topics with no consumer in the repository: `/human_robot/distance`, `/NS_1/rl_status`, and `/NS_1/q_des_state` (consumed only by `CBFTorqueController`, which is spawned only by `minimal use_torque_controller:=cbf`). | 02 §4 | Verified (grep); runtime (recorded) graph shows `q_des_state` with 0 subscribers |
| F-1-10 | `d_safe` is read from the **installed** YAML by the commander's governor (L357) and by the simulation start-clearance check, so a `d_safe` override sent through ROS parameters reaches the filter only. | 03 §5.5, §6.1 | Derived |
| F-1-11 | `fr3_control.yaml` and the launch defaults disagree on 9 filter mechanisms, plus `tracking.enabled` and `multi_obstacle_k`. The YAML alone misdescribes a default launch. | 03 §5.1 | Verified |
| F-1-12 | `robot_config_path` is passed to `cbf_safety_filter` by the torque launch and by `bag_replay`, but the node never declares it, so it is ignored. | 03 §4, §5.5 | Verified (no declaration); "ignored" Inferred from rclpy behaviour |
| F-1-13 | `config/conditions.d/mppi_it08.yaml` has no reader (`conditions.load` reads `conditions.yaml` only; `shaper` and `mppi` appear in no code). | 03 §3.6 | Verified |
| F-1-14 | Dead configuration: `oscbf_params.yaml`, `fake_hw_controller_params.yaml`, `camera_intrinsics.yaml`; 6 keys under `params:` (`d_safe_default`, `distance_ema_alpha`, `ema_alpha`, `gamma`, `max_tau_delta`, `qp_smooth_weight`); 2 launch defaults (`start_human_pose`, `human_pose_delay_s`). | 03 §3.1, §5.2, §5.3 | Verified (static search) |
| F-1-15 | `/NS_1/joint_states` has two publishers in Pipeline A: the 30 Hz republisher and a 10 Hz finger-only publisher. | 02 §3.8 | Static; runtime (recorded) 40.0 Hz = 30 + 10 |
| F-1-16 | In one recorded simulation cell, the constraint rebuild ran at 35.1 Hz against 50 configured, while the QP ran at 99.2 Hz against 100. | 01 §7, 02 §6 | Runtime (recorded), single cell |
| F-1-17 | There are two joint-limit sources: `franka_description` `joint_limits.yaml` for the filter, and `fr3_control.yaml → joint_limits` for 4 other nodes. | 03 §5.4 | Verified |
| F-1-18 | The Stage 0 extractor's markdown grouping missed `franka_experiments/franka_experiments/__init__.py`. Fixed in the tool, and the row was added by hand. | ledger | Verified |

### Stage 2 findings

| ID | Finding | Where | Label |
|---|---|---|---|
| F-2-1 | Segment 5 (`fr3_link5 → fr3_link6`) has length 0 for every q, because the two frame origins coincide. Its 2 control points `fr3_link6#0` and `#1` are therefore the same point, with identical J and J̇: 2 of the 11 control points are duplicates. The effect on the QP rows and slack is to be examined in Stage 3. | 04 §4.2 | Derived + numerically verified (200 random q) |
| F-2-2 | The container's **installed** `franka_experiments` (`/ros2_ws/install/...`, `kinematics.py` dated 2026-11-10 14:39) is stale relative to the baseline source: it lacks `CBFKinematics.configure_control_points`. A run in the container right now would not execute the baseline code. The numerical checks therefore imported the source via `PYTHONPATH`. | 04 §7 | Verified (runtime observation, 2026-11-11) |
| F-2-3 | `qddot_to_torque` uses `data.M` from `computeAllTerms` without symmetrising; `pentagon_qddot_commander` mirrors the upper triangle, on the assumption that "CRBA fills only the upper triangle". With Pinocchio 4.0.0 both triangles are filled, so both are correct. The code nonetheless depends on the version and no test pins it. | 04 §4.5 | Numerically verified (4.0.0); other versions Unverified |
| F-2-5 | Five consumers cache the joint-state index map from the first full message (`JointStateManager`, `pentagon_qddot_commander`, `rl_policy_commander`, `pentagon_torque_commander`, `cbf_OSCBF_filter`). `cbf_safety_filter` and `qddot_to_torque` map per message. Two arm publishers with different orders on one topic would be mis-mapped silently. | 04 §2.2 | Verified (code); risk Inferred |
| F-2-6 | `CBFKinematics.resolve_frame_id` falls back to the first frame whose name *contains* the string, so a typo can select another frame silently. | 04 §4.6 | Verified (code); impact Inferred |
| F-2-4 | The end-effector sphere (r = 0.05 m at link8 + 0.10 m · z₈) does not cover the hand's lateral half-extent (about 0.10 m, `hand.stl` bounding box). More generally, the distance geometry is 11 **spheres**, not capsules: on the forearm segment the gap between spheres is up to 3.2 cm at the 5 cm tube, and links 0–3 carry no control point. **Added in the reproducibility pass (2026-11-11):** at q_home, with the URDF-correct hand placement, 94 % of the hand's vertices lie outside every sphere; the largest excess is 6.84 cm (`check_geometry_and_so3.py`; home pose only). | 05 §3.2 | Derived; magnitudes numerically computed and reproduced; risk Inferred |
| F-2-7 | `camera_link_extrinsics.yaml` (static TF `base → camera_link`) was derived from `camera_extrinsics.yaml` as of `d4e4028` (2026-06-19; recomposition error 0.000 m). It was not regenerated after the three recalibrations of September. The TF camera pose is 0.276 m and 5.14° away from the pose `real_time_distance` uses. Distances are unaffected, because the node reads the file; RViz and anything using TF camera frames are affected. Answers Q-1-3. **Correction (2026-11-11, reproducibility pass, `tools/stage2_verification/check_camera_extrinsics.py`, full git-history scan):** the matching calibration is `e5fcdb2` (2026-05-21), which was the content of `camera_extrinsics.yaml` at `d4e4028`. There were **five** later calibrations (`8bfd96b` 06-19 15:28, `0c7e2b9`, `72d2753`, `08710eb`, `fda4af7`), not three. The text above that says "as of `d4e4028`" and "three recalibrations of September" is superseded on those two points; the 0.276 m / 5.14° conclusion stands. | 05 §5.2 | Verified + numerically checked; reproduced |
| F-2-8 | The mask-dilation margin (12 px body / 24 px EE, converted at the obstacle pixel's depth) is subtracted from **every** obstacle pixel, not only from pixels at the mask edge. Published distances are biased low by about 2.8 / 4.2 / 5.6 cm at Z = 1.0 / 1.5 / 2.0 m. This is conservative; it was probably not intended to be uniform. Input to the Stage 3 over-reactivity analysis. | 05 §3.4 | Derived (code) |
| F-2-9 | `MaskBuilder` poses `hand.stl` (the collision mesh of `fr3_hand`) with the TF of `fr3_link8`. The URDF joint `fr3_hand_joint` has rpy (0, 0, −π/4), so the modelled hand is rotated by 45°: up to 7.96 cm vertex displacement. The rotated samples also feed the mask, the z-buffer, `calibration_check` and the SSB model points. Effect on self-leak: Inferred; a runtime overlay check is proposed. | 05 §4.2 | Verified (URDF + code) + numerically measured |
| F-2-10 | `labelled_links` tells obstacle ranks apart from control points by `closest_point_robot` position. The duplicate control points `fr3_link6#0` and `#1` (F-2-1) have exactly equal positions, so with `multi_obstacle_k ≥ 2` (the launch default) and a second cluster, `fr3_link6#1` is labelled `fr3_link6#0.2` (with its extra as `#0.3`). The geometry is still correct; per-control-point state keyed on labels is mis-assigned. The self-detection keys (`_self_detected`, arrival order) keep `fr3_link6#1`, which then matches no published label, so a self-detection verdict for it never suppresses its track fields. | 05 §6.2 | Numerically reproduced with the baseline `build_cp_messages` + `labelled_links`; self-detection part Derived |
| F-2-11 | During the bounded LPF hold (≤ `iso_distance_hold_max_s` = 0.10 s), `DistanceEngine._lpf_pass` emits `closest_obstacle_point=None` with a valid direction. `build_cp_messages` then leaves `closest_point_human` at its wire default (0, 0, 0), but the entry is still `valid = True`. `cbf_safety_filter` reads it into `Obstacle.ph`; `cbf_state_rows` (around L2509) builds the normal from `pr − ph`. For the home-pose fingertip the normal is 122° away from the published direction (radially away from the base). Effect on the barrier: Inferred; Stage 3 must check whether anything rejects ph = 0 (Q-2-1). | 05 §3.5 | Verified (code path) + numerically reproduced (message); filter effect Inferred |
| F-2-12 | `handeye_calibration_node`: the docstring says Levenberg–Marquardt, the code calls `least_squares(method='trf', loss='huber')` with the default `f_scale` = 1. For residuals of a few mm or mrad, Huber is in its quadratic zone, so the fit is plain least squares; robustness comes only from the MAD rejection. Rotation (rad) and translation (m) residuals have unit weights. Minor, documentation-level. | 05 §9.1 | Verified (code) + Derived |

### Stage 3 findings

| ID | Finding | Where | Label |
|---|---|---|---|
| F-3-1 | `utils/cbf_constraints.py` (`build_simple_accel_cbf_constraints`, `build_hocbf_constraints`, `predict_state`, `CBFParams`) is imported by no node, script or test. Its HOCBF differs from the live one: c_i clipped to ±5, n̂̇ divided by max(d, 0.05), rows skipped at cond(J) > 1e5, no margins. Confirms the F-0-4 candidate for this module. The live rows come from `cbf_state_rows.ConstraintBuilder` + `cbf_qp_assembly.build_row_rhs`. | 07 §2 | Verified (grep over both packages) |
| F-3-2 | **F-2-11 downstream (answers Q-2-1).** No stage rejects or repairs `ph = (0,0,0)`. In the held entry `valid` = True, confidence = `find_pt_confidence` ≥ 0.2, so it passes `min_confidence` (0.2, test `<`). The track fields are zero (`cluster_id` −1, and `track_for_point((0,0,0))` finds no track), so `v_vec` is a zero **array**, not None. `ConstraintBuilder.build` then uses n̂ = pr/‖pr‖ and h = held d − d_safe − margins. Further effects to quantify: the identity guard (0.10 m) resets the per-label velocity state; the residual speed is computed with the wrong a. Numerical reproduction: `stage3_verification/check_held_row.py`. **Added at C-014 (`check_held_row.py`, 21/21):** scenario A (arm approaching at 0.10 m/s): the true approach reads +0.053 m/s along the held normal; the k1 term's ḣ goes from −0.100 to +0.034…+0.053 m/s. Scenario B (tracked obstacle closing at 0.5 m/s): see F-3-13. **Audit (C-018), Derived:** the go-around bias takes its geometry from the closest row (`around_best`), so a held row can also hand go-around the base-radial normal (06 §6.5). | 07 §8 | Verified (code path); effect numerically checked (see 07 §8) |
| F-3-3 | **Phase governor breaks the chain rule.** `pentagon_qddot_commander._tick` calls `_timing.step(actual_dt·σ)`, so in real time s advances at σ·rate. `LinearTimingLaw.step` still returns `s_dot = rate` (and s̈ for timing-law time). `v_d = P′(s)·s_dot` and `a_d = P″ s_dot² + P′ s̈` are therefore **not** the time derivatives of `p_d(t)` while σ < 1: v_d is overstated by 1/σ and the centripetal term by 1/σ². `test_phase_governor.py` assumes the opposite ("v_d = P'(s)*s_dot" carries σ) and never calls the timing law. | 06 §5 | Derived; numerically reproduced (`check_timing_governor.py`) |
| F-3-4 | `ZoneLadder.weights` builds the partition of unity from `bounds[:3]` only. `d_notice` (= `zone_r_notice`·d_safe) only validates the ordering and is printed by `describe()`. The notice gains apply for every gap > d_active, not from d_notice as the module and class docstrings say. `zone_r_notice` has no effect on the gains. **Correction (C-014):** the YAML comment on `zone_r_active` ("Outside it the notice gains") agrees with the code. Only the module table and the `ZoneLadder` docstring ("d_notice is where the ladder starts having an opinion") imply otherwise. The statement "not from d_notice as the module and class docstrings say" stands for those two docstrings only. Reproduced in `check_hocbf_gains.py`. | 07 §6.4 | Verified |
| F-3-5 | The priority/hold rungs of the zone ladder use (k0, k1) = (50, 14): k1² − 4k0 = −4 < 0. The characteristic polynomial s² + k1 s + k0 has complex roots, so the two-stage class-K decomposition (ψ1 = ḣ + α1 h, ψ̇1 + α2 ψ1 ≥ 0) of the HOCBF does not exist. The blended gains cross into this region for priority weight w > 0.504. The comparison-lemma guarantee then becomes an underdamped bound whose worst undershoot is numerically negligible (ζ = 0.99). **Precision (C-014):** complex roots only for gaps below 0.0499 m, i.e. more than 5 cm inside d_safe = 0.10 m. The negative lobe of the impulse response is 2.8e-10 of its peak (`check_hocbf_gains.py`). | 07 §6.4 | Derived; numerically evaluated (`check_hocbf_gains.py`) |
| F-3-6 | Two different evidence gates on the same track. The `v_obs` / curvature / `v_hdot` gate uses `_min_frames_eff` = `frames_for(obstacle_velocity_min_span_s, rate)` = 24 frames at 90 Hz. The uncertainty margin, latency compensation, velocity standoff and evidence class compare `frames_seen` with the raw `obstacle_velocity_min_frames` = 8 (89 ms at 90 Hz), whereas the YAML states that 8 frames "was ~265 ms at 30 Hz". The conditions `minimal_l1` and others set `obstacle_velocity_min_frames: 8` through `cbf_params`, which does not change the span-derived gate. | 07 §5 | Verified (code) + Derived |
| F-3-7 | The commander's CSV column `qddot_nom` (and `q_nom`, `dq_nom`) is the **open-loop model** of `_step_nominal`. The q̈ actually published on `/NS_1/qddot_nom` is `_q_ddot` from `_tick`. An analysis that reads the CSV column as the published nominal is wrong whenever the arm deviates. **Correction (C-014):** the CSV also logs the published command as `qddot_des_*`, and its header comments call `q_nom/dq_nom/qddot_nom` "ideal". So the defect is a **name collision** with the topic `/NS_1/qddot_nom` (which carries `qddot_des`), not a mislabelled column. `_step_nominal` also omits the `cart_err_max` clip of `_tick`. | 06 §4.6 | Verified |
| F-3-8 | The `fr3_control.yaml` comments give `path_kp_cart` in N/m and `path_kd_cart` in N·s/m. In the acceleration-level law ẍ = a_d + Kp e + Kd ė they are s⁻² and s⁻¹. | 06 §4.2 | Verified |
| F-3-9 | `CompositePath` joins the straight lead-in and the cycle with only C⁰ continuity of v_d: P′ jumps from (entry − p0)/S_a to the cycle tangent at s = S_a, so v_d and a_d step at the junction. | 06 §3.3 | Derived; magnitude computed (`check_timing_governor.py`) |
| F-3-10 | F-2-11 also reaches `rl_policy_commander`. `_obs_cb` keeps valid entries with a unit direction, so a held entry with `p_human` = 0 can become the nearest one. `obstacle_centre` = −r·n̂ (near the base origin), and the finite-difference `v_obs` observation spikes. The effect on the policy is Unverified. **Correction (audit, C-018):** Δt in the finite difference is the 100 Hz policy tick (`_tick` L570–579), not the 90 Hz message interval. The spike is ‖p_h,prev‖/0.01 s = 71 m/s for ‖p_h‖ = 0.71 m, not 64 m/s. Independently of the defect, 100 Hz sampling of a 90 Hz stream makes v_obs a sawtooth (≈ 1 tick in 10 reads 0, the others ≈ 1.11× the true speed). Derived. | 06 §7 | Derived |
| F-3-11 | `test/test_phase_governor.py` mirrors `_governor_sigma` with stale defaults (D_FULL 0.05 m, D_STOP 0.02 m) instead of the YAML ratios (1.5, 0.8 × d_safe = 0.15, 0.08 m), and does not exercise the timing law (see F-3-3). | 06 §5 | Verified |
| F-3-12 | F-2-10 downstream. Labels are per-label state keys in `ConstraintBuilder`: `_h_smooth`, `_obs_vel`, `_obs_vmed`, `_unc_ema`, `_stand_ema`, `_obs_ident`, `_ev_seen` and the memory. With `multi_obstacle_k` = 2 the label of the second wrist control point flips between `fr3_link6#1` (one cluster) and `fr3_link6#0.2` (two clusters), resetting that state. The rows of `#0` and `#1` are identical duplicates. Reproduction: `check_cp_label_rows.py`. **Correction (C-014, `check_cp_label_rows.py`, 8/8):** "resetting that state" is only half right. On the flip to a NEW key (`#1` → `#0.2`) `_h_smooth` is empty, so h_raw is accepted at once: an 11.9 mm loosening in one frame in the test, while the obstacle recedes. On the flip BACK, `#1`'s STALE state from before the flip is resumed, so the EMA restarts from an old value; its direction depends on the scene. Rows `#0`/`#1` agree to 5.6e-17. | 07 §8.2 | Derived; numerically reproduced |
| F-3-13 | **Held row loosens while a tracked obstacle closes (F-2-11 × evidence gates).** In a held frame `frames_seen` = 0, so the tracker v_obs is 0. The residual sees a frozen d. Every margin gated on `n_seen ≥ obstacle_velocity_min_frames` drops: latency at once, standoff through its EMA. In `check_held_row.py` scenario B (0.5 m/s approach, confirmed track), the row's v_obs goes 0.5 → 0 and h goes −0.116 → +0.039 m over the 0.10 s hold, while the true gap shrinks 4.4 cm. After the hold, the identity guard resets the velocity state a second time. **Correction (audit, C-018):** "while the true gap shrinks 4.4 cm" mixes two intervals. Over the same frames as the 15.5 cm (last normal frame 44 → last held frame 53) the gap shrinks **5.0 cm**; 4.4 cm is frames 45 → 53. The numbers are unchanged in `check_held_row.py`, which prints both. | 07 §8.1 | Numerically reproduced with the real builder; closed-loop effect Unverified (runtime) |
| F-3-14 | The commander's null-space term uses N = I − J⁺J with the **damped** J⁺. Then J·N ≠ 0: its singular values are λ²σ/(σ² + λ²) ≤ λ/2, so the posture task leaks into the task space by up to 0.005 (λ² = 1e-4) and 0.11 (λ² = 5e-2, near singularities) m/s² per rad/s² of null-space command. | 06 §4.3 | Derived + numerically verified (`check_nominal_shaping.py`) |
| F-3-15 | `StateGovernor.velocity_margin` uses `np.where(q̇ ≥ 0, up, −lo)`, so a STOPPED joint is scored against its UPPER envelope only. Joint 4 at the filter's position-box edge (q_ref,up − 0.05 rad) gives target w = 0.81 when stopped and 1.0 with q̇₄ = −1e-6. At q_ref,up − 0.02 it gives 0.12, and at q_ref,up it gives 0. The module docstring says a parked joint has full margin. `test_state_governor` covers the lower side only. (A subagent reported this as a latch at w = 0; the w = 0 case is at the box limit itself.) | 06 §6.2 | Verified (code) + numerically evaluated |
| F-3-16 | The lateral-evasion and outrun biases are EMA-filtered in `_qp_tick` (L1366–1386), but the filtered state is ADDED only while `con.esc_bias` / `con.outrun_bias` is not None. On disengagement the whole filtered bias (≤ 3 rad/s²) drops in one tick, contrary to the "fade out" comment. The livelock bias is added while n_c > 0, so it does fade. The go-around inward `cut` is not jerk-limited (`go_around_step`: only `bias` passes the rate limit). | 06 §6.4–6.6 | Verified |
| F-3-17 | Lateral evasion maps its Cartesian direction to joint space with J_pᵀ (`evasion_bias`, normalised), whereas outrun and go-around use a damped pseudoinverse. In Cartesian terms, J_pJ_pᵀê is not parallel to ê. | 06 §6.4 | Verified (code) + Derived |
| F-3-18 | `utils/avoidance.py` (`tangential_redirect`, `feasibility_beta_target`) is imported only by `test/test_avoidance.py`; its docstring "Used by pentagon_qddot_commander" is false. `ee_pentagon_velocity_commander`'s docstring promises minimum-jerk C² sides; the code uses constant speed with Bézier corner blends. | 06 §8 | Verified (grep, code) |
| F-3-19 | RL commander control-point names: the robot labels each control point by its segment's `end_link` (fr3_link4 … fr3_link8, `fr3_complete.yaml`); `franka_rl/config.yaml` lists `fr3_link3` "mirroring segment 3". So `cp:fr3_link3` never matches and stays at (clip, 0, 0, 0). **Audit (C-018):** the matching code was re-read (`perception_msgs.py` L212–216 sets `robot_link_name` = segment end link; `obs_spec_from_config` uses `robot_link` or `body`; `control_point_geometry` matches by exact name). Moreover, each policy slot fr3_link4…7 receives the robot samples of the segment ENDING at that link. | 06 §7 | Verified (config + code, audit C-018) |
| F-3-20 | Lateral evasion takes its trigger from the conditioned v_o (frame-gated or residual floor) but its direction from the raw `ob.v_vec`, gated only on ‖v_vec‖ ≥ 0.15 m/s, not on `frames_seen` (`escape_direction`; `_escape_row` L3846–3852). A young track's prior velocity can therefore set the escape direction while the residual floor supplies the urgency. | 06 §6.4 | Verified (code) + Derived (consequence); effect Unverified |

### Stage 4 findings

| ID | Finding | Where | Label |
|---|---|---|---|
| F-4-1 | **The obstacle barrier is a penalty, not a constraint, in the shipped `family` slack mode.** With ½ρs² on a family slack, a binding row aᵀq̈ + s ≥ b is always solved with s = (b − aᵀq̈_n)/(1 + ρ‖a‖²) > 0 (inside the box): the unrelaxed HOCBF inequality is violated by that amount on every binding tick (0.35 % of the demand at ρ = 1000, ‖a‖ = 0.54). Exact enforcement exists only in `row_l1` mode (exact penalty while the multiplier stays below ρ_l1), which only `minimal_l1` uses. | 08 §3 | Derived + numerically verified (`check_slack_and_priority.py` A, C) |
| F-4-2 | **One family slack per family couples the rows.** When one obstacle row cannot be met (box, slew, conflict), the shared s_obs also relaxes every other obstacle row: in the test, a second row satisfiable on its own was violated by its full demand (0.5 of 0.5 m/s²). With one obstacle row per control point and per obstacle rank (07 §6), one unreachable row loosens the barrier for the whole arm. | 08 §3.3 | Numerically reproduced (`check_slack_and_priority.py` B) |
| F-4-3 | **The hard box has priority over every CBF row**, including the slew window ±`max_qddot_delta` = 0.5 rad/s² per 3.33 ms tick (150 rad/s³) around the previous command. An obstacle row that needs a faster change of q̈ is relaxed by exactly the shortfall. Every CBF row is soft; the box (position, velocity, acceleration, slew) is hard. | 08 §2.4, §3.4 | Verified (code) + numerically reproduced (`check_slack_and_priority.py` D; `check_qp_assembly.py`: s_obs = 1.40 when the slew window excluded the barrier's demand) |
| F-4-4 | `accept_iterate` is meant to return None for a rejected iterate, but with OSQP 0.6.7 an infeasible problem returns `res.x` as an object array of None, and `np.isfinite` raises `TypeError`. `_solve` still reaches the fallback, but only through its generic `try/except`; the logged reason is the TypeError, not the OSQP status. Reachable when a family slack is bounded (`obstacle_slack_max` or `ssb_slack_max` ≤ 0). | 08 §5 | Numerically reproduced (`check_slack_and_priority.py` F) |
| F-4-5 | **Stale-joint-state fallback is open-loop.** `_qp_tick` L1099–1104 publishes clip(−k_brake·q̇_stale, static acc box) with the same stale q̇ every tick, without the CBF rows, the position/velocity box or the slew limit. Integrated, the commanded velocity reverses after 1/k_brake = 0.33 s and keeps accelerating backward while the stall lasts. From the box's fastest state 0.10 rad before the position margin it overshoots that margin on joints 1, 5, 6 and 7 (by up to 0.040 rad, still inside q_max). | 08 §5.2 | Derived + numerically reproduced, ideal tracking (`check_box_and_brake.py` 5); closed loop Unverified |
| F-4-6 | `qddot_to_torque`: the effort clip and torque-rate limit act only when `iso_enabled` (shipped false), so τ goes out unclipped. There is no staleness check on its joint state (it computes τ from the last received q, q̇ whenever a `/qddot_safe` arrives). The rate limit is scaled by the installed YAML's `qp_rate_hz`, not the effective one (cf. F-1-10). | 09 §2 | Verified (code) + numerically reproduced (`check_torque_realization.py` 4) |
| F-4-7 | `RtTorqueController` writes τ = ffScale·τ_ff + K_p p + K_d e with **no torque clamp and no low-pass filter**, although its plugin description says it "applies an optional low-pass filter, clips to per-joint limits". Torque saturation is therefore left to `franka_hardware`/libfranka/firmware (Unverified here) on hardware, and to Gazebo's joint limits in simulation. | 09 §3 | Verified (`check_controller_sources.py` 1–2) |
| F-4-8 | **`CBFTorqueController` bypasses `/qddot_safe` while `/q_des_state` is fresh.** It computes τ = RNEA(q, q̇, q̈_sp) − g with q̈_sp = the `effort` field of `/q_des_state`, which `pentagon_qddot_commander` fills with its own **nominal** q̈. `/qddot_safe` is used only when `/q_des_state` is older than 0.1 s. It is spawned only by `minimal.launch.py use_torque_controller:=cbf` (F-1-9), which itself starts neither the commander nor the filter, so the bypass needs a user-composed pipeline. | 09 §3.3 | Verified (`check_controller_sources.py` 6–7); runtime use Unverified (Q-1-4) |
| F-4-9 | **Gazebo plant inertia ≠ dynamics model.** `gz_robot_description` adds the reflected rotor inertia (ARMATURE 0.195 / 0.074 kg·m²) to the plant; `qddot_to_torque`'s Pinocchio model has none. At q_home the feedforward alone realises 63–86 % of q̈_safe on joints 1–4, 32–34 % on joints 5–6 and **3 %** on joint 7 (diagonal approximation). The 1 kHz PD on the integrated reference recovers 99 % of the commanded velocity within 0.3 s (transcribed loop). Barrier rows assume the realised q̈ equals the chosen one, which here holds only through the PD. | 09 §5 | Numerically evaluated, illustrative (`check_torque_realization.py` 6, `check_controller_sources.py` 8) |
| F-4-10 | The QP solves at OSQP 0.6.7's default tolerance (eps_abs = eps_rel = 1e-3; `max_iter` 20000; `polish` off). Relaxed-row residuals of 7.8e-4 and warm-vs-cold differences of 6.4e-3 rad/s² were observed, and `accept_iterate` then clips q̈ into the box, which can move a relaxed row by the same order. `row_l1` alone tightens eps_rel to 1e-5. | 08 §4 | Numerically reproduced (`check_qp_assembly.py`) |
| F-4-11 | No workspace rows and no torque rows exist in the running QP. `cbf_hard_limits.workspace_face_rows` has no caller, and the acceleration box is the only actuator-side constraint. At q̇ = 0 no box vertex needs more than effort_max (0 of 12 800 vertex × pose cases); Coriolis and gravity are not in that check. | 08 §2 | Verified (grep) + numerically evaluated |
| F-4-12 | **No rows at all and no fault before the first distance message.** `_update_constraints` (L958–961) needs both `_js` and `_obs`. Until perception publishes once, `_qp_tick` takes the "no snapshot" branch: a box-only QP with the unchanged nominal, without joint-limit, self-collision or singularity rows, and fault = 0. Stale distances, by contrast, brake (L1153–1161). The joint-state-only families depend on perception being alive. | 08 §5.1 | Verified (code) |
| F-4-13 | `qddot_to_torque`'s module docstring states "`rt_torque_controller` adds g(q) internally". It does not (F-4-7); on hardware the firmware compensates gravity, in Gazebo the plugin does. This is the node-side counterpart of F-1-3. | 09 §2 | Verified |

### Stage 5 findings

| ID | Finding | Where | Label |
|---|---|---|---|
| F-5-1 | **`bypass_cbf` defaults to `false` in `velocity_cbf_control_stack.launch.py`** (`DeclareLaunchArgument` L434–436), so the velocity QP is active by default. The launch docstring (L19: "Phase 1 (bypass_cbf:=true, default)") and **ch. 01 §4.2** state the opposite; ch. 01 is corrected with the old wording kept. | 10 §2.1, 01 §4.2 | Verified (`check_velocity_cbf.py` 4) |
| F-5-2 | **The velocity filter fails open.** Stale or never-received distances → clip(q̇_nom) pass-through; QP failure → 0.5·clip(q̇_nom). No braking path. It reads q, q̇ from the 30 Hz `/NS_1/joint_states` with no age check, and keys activation hysteresis by `robot_link_name`, which is shared by several control points. | 10 §2 | Verified (`check_velocity_cbf.py` 4) |
| F-5-3 | **The velocity filter's post-QP rate clamp can undo the barrier.** The published q̇ is q̇_prev + clip(q̇_QP − q̇_prev, ±q̈_max·dt), with q̈_max = min(decel limit, jerk·1 ms·0.8/2) = (3, 1.5, 2, 2.5, 3, 4, 4) rad/s². Approaching at 0.3 m/s, the QP output met the ZCBF row (residue 4e-4) but the published command violated it by 0.099 m/s in that tick. The ZCBF row is also a single-slack penalty, like F-4-1. | 10 §2.3 | Numerically reproduced with the production QP (`check_velocity_cbf.py` 1–2) |
| F-5-4 | **The OSCBF obstacle row omits α₁ḣ.** `_build_obstacle_cbf` enforces ḧ ≥ −α₂ḣ − α₁α₂h; the HOCBF chain (and the same file's joint-position and workspace rows) requires ḧ ≥ −(α₁+α₂)ḣ − α₁α₂h. At α = 10, 10 the loop is s² + 10s + 100 (ζ = 0.5, 16.3 % impulse undershoot). In a 1-D binding approach at 1 m/s from h = 0.20 m, the coded law reaches h = −29.8 mm while the HOCBF law stops at 0. | 10 §3.3 | Numerically reproduced with the production builder (`check_oscbf.py` 2–3) |
| F-5-5 | **The OSCBF obstacle path is inert or optimistic as shipped.** Its h = ‖p_r − p_h‖ − 0.15: the centre distance with a hard-coded d_safe, ignoring `LinkDistance.distance` (the surface gap). `enable_obstacle_cbf` defaults to false; the distance subscription is RELIABLE against a BEST_EFFORT publisher (F-1-6); stale distances skip the rows silently. An infeasible QP passes τ_nom unfiltered; a stale nominal gives τ = 0; the output `/NS_1/torque_safe` has no consumer. The node reads joint states from the 30 Hz topic. | 10 §3 | Verified + numerically checked (`check_oscbf.py` 4, 6) |
| F-5-6 | **`pentagon_torque_commander` has an orientation-loop sign error.** e_rot = ½vee(R_dᵀR − RᵀR_d) ≈ +θ (desired → current, in R_d's frame) is fed back with +K_rot, so θ̈ = +K_p θ − K_d θ̇ is unstable (root 2.62 s⁻¹ at 20/5). It is the form `pentagon_qddot_commander` replaced (its comment: "frame/segno errati"). There is no a_d feedforward. The node is test-only (F-0-2). | 10 §3.1 | Verified + numerically illustrated (`check_torque_commander.py`) |
| F-5-7 | **`RtVelocityExecutorController` has no command watchdog.** `timeout_threshold_s` and `timeout_ramp_s` are read but "reserved – not used in RT path" (header L88–89). The last q̇ is held indefinitely if the filter stops publishing, although `launch_defaults` sets 0.5 / 1.0 s. On hardware, libfranka's own communication checks are Unverified. | 10 §2.4 | Verified (`check_velocity_cbf.py` 5) |
| F-5-8 | **The ISO S_h uses the measured approach speed, not the normative human speed.** The monitor (`_evaluate`), the QP SSM rows (`spd_pts` v_app) and the evidence logger feed `protective_separation` / `ssm_speed_cap` with v_app = max(n̂ᵀv_track, 0): 0 for an untracked person, and not gated on `frames_seen`. `iso_v_human` = 2.0 is used only for the standoff (F-5-14). At the shipped constants S_p shrinks by v_h(T_r + T_s): by 1.20 m at v_r = 0.5 m/s, and the cap at d = 1.5 m rises from 0.174 to 0.982 m/s. | 11 §3 | Verified + numerically evaluated (`check_iso.py` 3, 7) |
| F-5-9 | **The ISO layer cannot be enabled in this cell as shipped.** C = 0.85 m (body detection, no detection-capability record) gives the floor C + Z_d + Z_r = 0.92 m, which exceeds the FR3 reach (0.855 m). `_iso_configure` raises for d_safe = 0.10 (fail closed); passing needs d_safe ≥ 0.92, i.e. a barrier larger than the workspace. This is the "STRUCTURAL / D1" verdict of `iso_evidence_report`. | 11 §4 | Verified + numerically reproduced (`check_iso.py` 4) |
| F-5-10 | **ISO wiring gaps (subagent report, spot-checked).** (a) The launch preflight is given only `--config <share>/fr3_control.yaml`: it sees the YAML `iso_enabled: false` and exits 0, so a command-line `iso_enabled:=true` and the speed overrides are never preflight-checked; the filter's `_iso_configure` is the effective gate. (b) The evidence logger is not passed `iso_enabled` / `iso_mode` / `iso_ssm_speed_rows` / `iso_monitor_enabled`, so its manifest and reduced-mode checks use the YAML values. | 11 §5 | Verified (launch L536–556, L833–850; `check_iso.py` 6) |
| F-5-11 | `iso_v_pfl` = 0.68 m/s is the hands-and-fingers value of `iso_pfl_speed.py`'s own table. Other regions give chest 0.328, thigh 0.349, abdomen 0.408, forearm 0.482, upper-arm 0.521 m/s, and the script itself says the minimum over reachable regions binds. | 11 §3.4 | Numerically reproduced (`check_iso.py` 2) |
| F-5-12 | `iso_distance_hold_max_s` (0.10 s, the F-2-11 hold) is read by `real_time_distance` without any `iso_enabled` check, contrary to `launch_defaults`' "ALL FALSE … adds no behaviour". | 11 §6 | Verified |
| F-5-13 | `fr3_control.yaml` comment (L91) says "d_safe stays at its tuned research value (0.15 m, above)"; the value above is 0.10 (L66). `test_iso_dsafe_floor` repeats 0.15. A second comment in the same block (L109–112, "IN SYNC WITH SIM … franka_sim/config.yaml carries 0.15 m and so does this file") is also contradicted by L66 (added in the resumed session). | 11 §4 | Verified |
| F-5-14 | The ISO standoff sets time = T_r + v_h/a_s, using the HUMAN speed in the stopping-time slot, where the standard's T_s = v_r/a_s is the robot's. It is conservative only while v_r ≤ v_h (true with `link_speed_max` ≤ v_PFL < v_h). Max = min(v_h·t, 2 m). | 11 §3.3 | Derived (`check_iso.py` 4) |
| F-5-15 | The OSCBF cost Hessian P_arm is positive definite but badly conditioned (eigenvalues 0.06 … 2.7·10⁵ at q_home), relevant to OSQP's ~1e-3 accuracy. | 10 §3.2 | Numerically evaluated (`check_oscbf.py` 5) |
| F-5-16 | **Stale velocity-executor configuration and diagnostics.** (a) `franka_rt_controllers/config/rt_velocity_blender.yaml` (`qdot_max` 0.4, `max_accel` 4.0, timeouts 0.3/0.8) is loaded only by `franka_rt_controllers/launch/rt_velocity_blender.launch.py`; the velocity stack generates its own YAML from `launch_defaults.yaml` (1.5 / 1.5 / 0.5 / 1.0). (b) `launch_support.build_wrapper_log_actions` would print the timeouts as `[ACTIVE]`, although the controller ignores them (F-5-7); no launch file calls it. (c) The `ee_pentagon_velocity_commander` docstring names `rt_velocity_blender_controller` as its consumer; the consumer is `cbf_velocity_filter`. | 10 §5.1 | Verified (`check_velocity_cbf.py` 6) |
| F-5-17 | **ISO stop-latch deadlock (dormant: needs `iso_enabled` and `iso_monitor_enabled`).** While the monitor's latch is set, `cbf_safety_filter._qp_tick` brakes and publishes `/cbf_status` with `fault_braking = 1.0` (L1071–1088). The monitor reads `data[2] ≥ 1` as a live safety-chain fault (`_tick`), so `_live = REASON_FAULT`; `_on_reset` refuses while `_live ≠ NONE`, and auto-resume (`iso_stop_requires_reset: false`) also needs `_live = NONE`. The latch can therefore never be cleared from inside the system. The same loop turns any transient filter fault (stale perception, frozen joint state) into a permanent stop once it lasts 9 ticks. That includes `_brake_authority_fault`, documented as "Diagnostic only — no stop": it also sets `fault = 1.0` (L1504), so with the monitor on it becomes a stop. `test_iso_monitor` tests the monitor alone (`test_a_chain_fault_also_blocks_the_reset` pins the refusal), so the loop between the two nodes is untested. | 11 §5.3 | Derived; reproduced with the production monitor methods, the filter side transcribed from verified source (`check_iso.py` 8); runtime Unverified |
| F-5-18 | **Two meanings of "S_p" on `/cbf_status[5]`.** With a fresh monitor message the field is the monitor's S_p at the current closing speed of the most binding point. Otherwise it is the row builder's `diag_ssm_sp` = max over points of `protective_separation(cap, v_app)`, which equals the gap d itself wherever the cap is not saturated (identity to 2e-16). Example: d = 1.2 m, closing at 0.2 m/s gives monitor 0.960 m vs builder 1.200 m. Also, the SSM speed row bounds ‖ṗ‖ along the current travel direction, while the monitor trips on the closing component −n̂ᵀṗ (consistent, the row being stricter); the rows use the conditioned v_obs, the monitor and the evidence logger the raw `obstacle_velocity`. | 11 §3.5, 13 | Numerically reproduced (`check_iso.py` 9) |
| F-5-19 | **The monitor's liveness model is one-sided.** (a) It trips STALE 9 ticks (30 ms) after start when any input is missing, and latches. (b) `_on_status` stores the fault flag with no timestamp: a filter that stops publishing is invisible to the monitor, and the last flag is kept forever. (c) Inferred boot race: the filter (started at `torque_dynamics_delay_s` 2 s) brakes with fault = 1 once no `/iso_safety` has arrived for `distance_timeout` 0.5 s, and the monitor starts at `torque_iso_monitor_delay_s` 3 s. Whether the monitor sees 9 fault ticks before the filter sees its first unlatched message, and whether perception is publishing by then, is timing-dependent; if either happens, F-5-17 locks the stop from boot. | 11 §5.3 | (a)(b) Verified + reproduced (`check_iso.py` 10); (c) Inferred, runtime Unverified |
| F-5-20 | **The preflight's placeholder gate exists only in the preflight.** `iso_preflight_check` refuses the four shipped placeholder constants (T_r 0.10, a_s 1.0, Z_d 0.06, Z_r 0.01), but the launch runs it on the YAML only (F-5-10a) and never with `--wait`, and `_iso_configure` has no placeholder check. A command-line `iso_enabled:=true` therefore runs on placeholders as long as the d_safe floor and the PFL ceiling pass. | 11 §6 | Verified (source) |
| F-5-21 | **`experiment_logger` records corrupted arm rows in about a quarter of the samples.** `torque_control_stack` starts, unconditionally, a 10 Hz `ros2 topic pub` of a finger-only `JointState` (`fr3_finger_joint1/2`, all zeros) on `/NS_1/joint_states` (L613–633). That topic is the logger's default `joint_state_topic`, and the launch passes the logger no parameters. For a finger message, `_vector_from_joint_state` fails the by-name mapping and falls back to **positional** copying: q₁ = q₂ = 0 and q₃…q₇ = NaN. The same happens to q̇ and τ_effort, and the Cartesian block (`tcp_*`, `tcp_speed`) becomes NaN. The row stays like that until the next 30 Hz arm message. Recorded: 26.3–26.7 % of rows in four Gazebo runs (`sim_runs/{leakfix_20261106,ra2,ra3,rb3}`) show exactly q₁ = q₂ = 0 with q₃ and `tcp_speed` NaN. The plots built from these columns, and any offline metric that does not drop the pattern, inherit it. | 13 §2.2 | Verified (code) + Runtime (recorded; the logger is unchanged since 7da68ac of 2026-11-05, before the 2026-11-06 run; the manifests' `git.sha` is null); reproduced on the production method (`check_observability.py` 1) |
| F-5-22 | **The run bag misses the channels that explain a stop.** `torque_control_stack start_rosbag:=true` records 19 topics, but not `/rosout` (so no CBFDIAG line, fallback warning or ISO error), `/NS_1/iso_safety`, `/NS_1/torque_saturation` or `/NS_1/q_des_state`. A bag of an ISO-enabled run therefore holds only `/cbf_status[8]` for the monitor (and F-5-18 applies to `[5..7]`). The recorder writes zstd file compression, while the shared `ball_throw_eval.read_bag` (also used by `throw_*`, `pass_plot`, `smoothness_report`) opens only uncompressed `*.db3` through sqlite3 and asks for a manual `zstd -d`. | 13 §3 | Verified (`check_observability.py` 3) |
| F-5-23 | **Misleading or missing diagnostics in the loggers.** (a) `experiment_logger.sample` holds every last value with no age check: NaN means "never received", but a channel that dies mid-run keeps printing its last value. (b) `qddot_*` is a finite difference of the 30 Hz `/NS_1/joint_states` velocities, EMA-smoothed (α 0.35), not the 1 kHz realised acceleration. (c) The legacy `min_h`, `cbf{k}_h` and `cbf_violation_count` are `MultiDistance.distance − d_safe` from `/human_robot/multi_distance`, without the filter's margins, beside `cbf_n_violated`, which counts the QP rows with h̄ < 0: two "violation" counts with different meanings. (d) `iso_evidence_logger.tau_max_abs` is max \|τ\| of `/NS_1/torque_cmd`, which excludes gravity (09), so it is not comparable with the effort limits. (e) The recorded runs' `run_manifest.json` carry `git.sha = null` (the container install is not a git checkout), and `utils/run_metadata.write_runtime_metadata`, documented as "enabled by the experiment runner", has no caller outside tests (see also F-5-27). (f) `frame_grabber only_when_cbf_active` tests `cbf_status[0]` (`n_rows`, every row in the QP, binding or not) instead of `[3]` (`n_violated`). (g) The logger's default `experiment_name` is `bag_02_dynamic_test`, and the torque stack passes no parameters, so every run directory is named after that one bag (all the recorded `sim_runs/*/logger/*_bag_02_dynamic_test`). | 13 §2–§4 | Verified (`check_observability.py` 2, 4) + Runtime (recorded) for (e) |
| F-5-24 | *(Recorded as F-5-16 by a parallel session on 2026-11-11 and renumbered at the reconciliation, C-035; the ID F-5-16 stays with the earlier row of that number.)* **The experiment log does not hold the QP's own diagnostics.** `/cbf_status` has 41 fields (schema v3), but `experiment_logger` reads only data[0..10]. Fallback level, solve time, tick gap, the three perception/snapshot ages, the per-family slacks and the per-family raw/relaxed violations (data[11..40]) are never written to the CSV. The torque launch's bag (`start_rosbag`) records `/NS_1/cbf_status`, so they survive only in bags. It does not record `/NS_1/iso_safety`, `/NS_1/torque_saturation`, `/NS_1/q_des_state` or `/rosout`, so CBFDIAG lines are lost. The logger also samples at 100 Hz the 300 Hz status (single-tick events can be missed) and holds stale values without an age check. | 13 §2.4, §3, §8, §9 | Verified (`check_observability.py` 5, 9) + subagent inventory (sampling, holding) |
| F-5-25 | *(Recorded as F-5-17 by a parallel session on 2026-11-11 and renumbered at the reconciliation, C-035; the ID F-5-17 stays with the earlier row of that number.)* **Misleading logger columns.** (a) `min_h`, `cbf{k}_h` and `cbf_violation_count` are the logger's OWN d − d_safe, computed on the legacy per-segment `/human_robot/multi_distance` with its own d_safe, not the filter's h̄. (b) `qddot_nom_*` is the topic (the commander's published command), while the commander's own CSV uses `qddot_nom_*` for its ideal open-loop channel (F-3-7); the commander's publish parameter is even named `qddot_safe_topic`. (c) `qddot_delta_norm` compares the commander's command with q̈_safe, not with the filter's shaped nominal. (d) `qddot_i` is an EMA finite difference of the 30 Hz `/NS_1/joint_states` velocities. (e) `experiment_name` defaults to `bag_02_dynamic_test` and the torque launch passes no parameters, so every run carries that name. | 13 §2.2, §2.4, §9 | Verified (a, b, d, e: `check_observability.py` 6, 7, 11) + subagent (c) |
| F-5-26 | *(Recorded as F-5-18 by a parallel session on 2026-11-11 and renumbered at the reconciliation, C-035; the ID F-5-18 stays with the earlier row of that number.)* Stale tool defaults: `watch_obstacle_tracks` listens to `/cbf/per_link_distances_tracked` (published by nothing); `throw_clearance` loads `<pkg>/rosbag/<BAG>_truth.npz` (no such directory); `pipeline_rate_monitor` expects `/NS_1/qddot_safe` and `/NS_1/cbf_status` at 100 Hz (`qp_rate_hz` 300) and distances at 30 Hz (camera 90); `iso_constants_measure uncertainty` defaults to `/cams/d455/...` (the shipped topics are `/camera/camera/...`). | 13 §7 | Verified (`check_observability.py` 10) |
| F-5-27 | *(Recorded as F-5-19 by a parallel session on 2026-11-11 and renumbered at the reconciliation, C-035; the ID F-5-19 stays with the earlier row of that number.)* `utils/run_metadata.write_runtime_metadata` has no caller outside its test on this branch, and nothing sets `FRANKA_RUN_METADATA_DIR`. 297 archived node copies under `sim_runs/*/provenance` (data, out of scope) do call it; they probably come from other code versions (not on this branch's history, `git log -S`). | 13 §2.1 | Verified (`check_observability.py` 4, 8); origin of the archived copies Inferred |

## 5. Decisions and open questions

**Open questions added in Stage 5:**

| ID | Question | Who can close it |
|---|---|---|
| Q-5-1 | Extends Q-1-4. Pipelines B (velocity CBF) and C (OSCBF) carry the defects F-5-2, F-5-3, F-5-4, F-5-6 and F-5-7. Should they be fixed, kept as historical, or removed in the refactor? The answer decides whether ch. 10 is reference or archaeology. | **user** |
| Q-5-2 | F-5-17: what is the intended ISO reset semantics? Should the filter publish `fault_braking` only for its own faults, should the monitor ignore a fault the ISO stop caused, or should the reset clear both? | **user** |
| Q-5-3 | F-5-19: does the monitor trip STALE at boot (it starts at 3 s, the filter at 2 s; STALE after 9 ticks of a missing input)? Needs a Gazebo run with `iso_enabled` and `iso_monitor_enabled`, which first needs the F-5-9 gate satisfied. | runtime (user approval needed) |
| Q-5-4 | F-5-21: were any reported results computed from the affected logger columns (`q_*`, `qdot_*` of joints 1–2, `tcp_speed`), e.g. through `analyze_isolation_ab.py`? If so, they need re-computing on rows with `q_3` finite. | **user** |

**Open questions added in Stage 4:**

| ID | Question | Who can close it |
|---|---|---|
| Q-4-1 | F-4-1/F-4-2: is the family-slack (penalty) barrier intended for hardware, or is `row_l1` meant to become the default? With family slack a single unreachable obstacle row relaxes all of them. | **user** |
| Q-4-2 | F-4-5: is the open-loop stale-joint-state brake acceptable, given that the published deceleration reverses the joint after 1/k_brake? | **user** / runtime |
| Q-4-3 | Where is torque saturated on hardware (F-4-7: not in the controller)? `franka_hardware`/libfranka limits, and whether the firmware compensates gravity in the effort command mode used. | external docs / hardware |

**Open questions added in Stage 3:**

| ID | Question | Who can close it |
|---|---|---|
| Q-3-1 | F-3-6: should the margin gates (8 measurements) follow the span-derived 24-measurement gate of the velocity they protect, or is 8 at 90 Hz (89 ms) intended? | **user** |
| Q-3-2 | Closed-loop effect of F-3-13 (held row loosens while a tracked obstacle closes) and F-3-3 (governor lead): a Gazebo cell with an occlusion during approach, and a static-obstacle cell with σ < 1. | runtime (user approval needed) |

**Decisions recorded at the Stage 0 review (2026-11-11):**
- The missing untracked test is recorded and must not be recreated.
- Only HEAD is analysed. Stashes are never applied, modified or deleted.
- `ledger.json` and the tools stay in the repository, and no automatic commits are made.
- A pipeline's status is classified from code evidence. Where the evidence is ambiguous, it is
  marked as requiring user confirmation.

**Open questions (Stage 1 and Stage 2):**

| ID | Question | Who can close it |
|---|---|---|
| Q-1-1 (**answered in Stage 4**, 09 §5) | Who compensates gravity in Gazebo? Neither `qddot_to_torque` nor `rt_torque_controller` adds `g(q)`. **Answer:** `franka_ign_ros2_control/src/ign_system.cpp::IgnitionSystem::write` adds the KDL gravity torque g(q) to every effort command (Verified). On hardware the FR3 firmware compensates gravity (Franka documentation; Unverified here). | Stage 4/6 (read `franka_ign_ros2_control` / the plant) |
| Q-1-2 (**answered in Stage 2, 05 §5.1**: a deliberate choice, recorded in `fr3_complete.yaml`; root cause open; runtime test proposed) | `real_time_distance` projects the depth image (`camera_depth_optical_frame`) with an extrinsic whose child frame is `camera_color_optical_frame`. Is the depth-to-colour offset handled? | Stage 2 |
| Q-1-3 (**answered in Stage 2: NO**, F-2-7) | Do `camera_extrinsics.yaml` (projection) and `camera_link_extrinsics.yaml` (TF) describe the same camera pose consistently? | Stage 2 (numeric check) |
| Q-1-4 | Is Pipeline B (velocity) still run on hardware? Is Pipeline C (OSCBF) abandoned or parked? Is `CBFTorqueController` (`use_torque_controller:=cbf`) still used? | **user** |
| Q-1-5 | Is `start_move_group:=true` intended (for example, for another tool), or is it a leftover? | **user** |
| Q-2-1 (**answered in Stage 3: NO**, F-3-2) | Does anything in `cbf_safety_filter` / `cbf_state_rows` reject or repair an obstacle with `ph = (0, 0, 0)` (F-2-11), or does the held row really use the base-radial normal? | Stage 3 (ch. 07) |
| Q-2-2 | `gate_max_m` (0.5 m) against `max_coast_s` (0.8 s): after about 0.5 s of coast the 4σ Mahalanobis gate exceeds 0.5 m, so only the Euclidean gate binds (05 §7.4; the YAML records the same caveat). Is a re-derivation planned before hardware use? | **user** / runtime |
| Q-2-3 | Q-1-2 root cause: does the hand-eye result really approximate base → depth optical, or was the 2026-09-30 offset trial composed with the wrong direction? Runtime test in 05 §5.1, using the mask coverage and leak statistics (05 §9.3). | runtime (user approval needed) |
| Q-2-4 | F-2-9 runtime confirmation: compare the mask overlay and the `leak=` counter with joint 7 rotated, with and without the −45° hand rotation. | runtime (user approval needed) |

**Decisions recorded on 2026-11-11, after Stage 2 (reproducibility pass, at the user's request):**
- Numerical evidence lives in the repository, under `tools/<stage>_verification/`, never only in a
  session scratchpad.
- Each script tags its results `NUM` (identity verified numerically), `REPRO` (a reported number
  reproduced) or `FACT` (a code or configuration fact evaluated). Runtime validation is listed
  separately as still open.
- Scripts must write nothing into the repository; bytecode caching is disabled.
- When a rerun disagrees with a chapter, the chapter and the finding are corrected, and the superseded
  wording is kept visible.

**Decisions recorded on 2026-11-11, after Stage 1:**
- The repository files are the single source of truth, so that work can resume across sessions and
  accounts.
- The root `CLAUDE.md` holds the project rules.
- Checkpoints are written during stages, following §6.
- The rules that used to live only in account memory are now in the repository:
  - no `Co-Authored-By` trailer on commits the user requests;
  - the technical leads in §7.

## 6. Checkpoint protocol

**When:**
- after completing a significant subsystem (roughly one chapter section or one major source file);
- before starting another major subsystem;
- before ending a session;
- before context compaction, whenever possible;
- at the end of every stage.

**How:**
1. Update the ledger statuses of the files just read, with `tools/ledger_status.py`.
2. Append any new findings to §4 and any open questions to §5.
3. Append an entry to [CHECKPOINTS.md](CHECKPOINTS.md), using the template given there. It records:
   1. what has been analysed;
   2. what remains;
   3. the files read, with status;
   4. the documentation files updated;
   5. findings and uncertainties, by ID;
   6. the **exact next action**: the file, the section and the first thing to read or write.
4. Overwrite §0 (the resume card) so that it agrees with that entry.
5. Run `python3 -I docs/framework_study/tools/check_state.py`. It must print `RESULT: consistent`.

A half-written chapter is acceptable at a checkpoint. Mark the unfinished sections in the chapter
itself with `> **TODO (checkpoint C-NNN):** …`, so the next session knows where to continue.

## 7. Leads carried over from earlier account memory (Unverified until a stage checks them)

These facts were recorded in the user's per-account notes before this documentation project started.
They are **leads only**: each must be re-verified against HEAD by the stage named here before a
chapter states it.

| Lead | Target |
|---|---|
| `qpsolvers==4.3.3` with `osqp<1.0` are pinned; otherwise `available_solvers == []` and the velocity and OSCBF filters die with `SolverNotFound`. | ch. 08, 14 |
| Setting `SCHED_FIFO` on a Python thread throttles the whole rclpy node (GIL inversion). FIFO for the QP thread was turned off on 2026-11-07. | ch. 08 |
| `/NS_1/franka/joint_states` arrives in the order `[j1, j3, j6, j7, j2, j4, j5]`. Consumers must map joints by name, or the Jacobians silently decorrelate. | ch. 04, 09 |
| colcon **copies** `config/` (it does not symlink it), so a new `fr3_control.yaml` key needs a rebuild, or every Gazebo cell fails with `ok=False` while the unit tests pass. `install(PROGRAMS)` also copies scripts. | ch. 14 |
| `rl_policy.py` looks for `franka_sim`, but the directory on disk is `franka_rl`, so 9 of 44 sim/real parity tests SKIP silently (see F-0-10). | ch. 14 |
| The simulation `truth.npz` is sampled at 448–688 Hz under load, not 1 kHz, with gaps up to 207 ms. Derivatives taken on it break. | ch. 12 |
| Leftover ROS nodes (for example `/clock` bridges) make every simulation cell time out with `ok=False`. | ch. 12, 14 |
| Commit `e9ff7cd` dropped the `truth_*` columns of `experiment_logger` (an undeclared override); they were restored on 5 Nov 2026. | ch. 13 |
| Surface-point velocity must come from a fitted slope (least squares over about 0.22 s): differencing an argmin attenuates a real approach about 9×. | ch. 05 |
| Pentagon tracking looked "random" because `cbf_safety_filter` was starved to 18 Hz on 28 Oct 2026. | ch. 02, 08 |
| Brake on QP failure and slew-yields-to-state-box are default-on but dormant in healthy runs. Braking does not avoid a **lateral** obstacle. | ch. 08 |
| Over-reactivity fixes P1–P4: age and floor gates, track hold, PLACE memory, surface cap (the cap is inert in simulation). | ch. 05, 07 |
| The three evidence-trust novelties are default-off and none has been shown to work. The baseline σ is 2.27 cm, which rules out claims from a single seed. | ch. 07, 15 |
| The `diag` furnished scene is the simulation default. The `static` scene fails because phantom `v_obs` appears on the furniture. | ch. 12 |
| Gazebo markers: `scale` multiplies the points rather than setting a line width, and a misspelt field drops every field after it. | ch. 12, 13 |

## 8. Documented source baseline and its history

Two revisions are distinct:
- **The documented source baseline** is the commit whose ROS 2 source the chapters describe. It
  changes only through the procedure below.
- **HEAD** is whatever is checked out now.

Commits that touch only the documentation paths (`docs/framework_study/` and the root `CLAUDE.md`)
move HEAD and nothing else.

`tools/check_state.py` compares the **working tree** with the baseline, so it covers committed,
uncommitted and untracked changes. It classifies every changed path:

| Class | Paths | Effect |
|---|---|---|
| doc | `docs/framework_study/`, `CLAUDE.md` | ignored |
| source | every ledger file; anything under `franka_experiments/`, `franka_simulation_experiments/`, `franka_rt_controllers/`, `franka_msgs/msg/` (data directories excluded) | **REVIEW** (exit code 2). Lists the file's primary chapter and the written chapters that mention it |
| interface | other workspace dependencies documented by interface only (`franka_simulation/`, `franka_rl/`, `franka_bringup/`, `franka_description/`, …) | NOTE. Check whether a documented interface changed, for example `franka_bringup` namespace → F-1-1 or `joint_limits.yaml` → F-1-17 |
| other | everything else (root reports, `.gitignore`, …) | NOTE |

It reports a PROBLEM (exit code 1) when:
- the baseline is not an ancestor of HEAD (a branch switch, rebase or reset);
- §0 and the last row of the table below disagree.

**When REVIEW appears.** This may happen during the documentation phase, if the user edits code in
parallel. During the later refactor it is expected:
1. Do not move the baseline yet. Run `git diff <baseline> -- <path>` for each REVIEW path.
2. For each affected chapter (primary plus mentioning), re-check the statements that depend on the
   changed code. Update the text, its "Consequences of modification" section and its findings.
   - When a finding is fixed by the change, mark it superseded with the commit; never delete it.
   - Set the ledger status of the changed files back to `partial` if their chapter text is no longer
     verified.
3. If files were added or removed, update `ledger/` by hand. You can regenerate the extraction into a
   scratch directory and compare.
4. Only when every affected chapter has been re-checked, append a row below and update §0 to the new
   baseline. Write a checkpoint.
5. If a stage is approved while the review is pending, the review comes first. Chapters written on top
   of an unreviewed change would mix two revisions.

| ID | Baseline commit | Adopted | Reason | Chapters re-checked |
|---|---|---|---|---|
| B-1 | `ba68077052e8d0fb3d4606c05dff553b75baa641` | 2026-11-11 | Stage 0 start: HEAD of `humble-mattia`, tree clean | — (initial) |
