# Progress tracker

This file is the single place that records where the documentation stands. Every stage ends by
updating it and then **stopping for review**. A stage starts only after explicit approval.
Project rules: [/CLAUDE.md](../../CLAUDE.md). Session history: [CHECKPOINTS.md](CHECKPOINTS.md).

## 0. Resume card (overwritten at every checkpoint)

- **Last updated:** 2026-11-11, checkpoint C-003 (baseline-aware consistency check; no stage work)
- **Project status:** read-only documentation phase. Stages 0 and 1 are done.
- **Last completed stage:** Stage 1 (Architecture and graph). The user acknowledged it as complete on
  2026-11-11.
- **Next pending stage:** Stage 2 (Foundations and perception): `04_math_foundations.md` and
  `05_perception_and_distance.md`. It is **NOT approved yet**.
- **Stage in progress:** none.
- **Pinned branch:** `humble-mattia`
- **Documented source baseline:** `ba68077052e8d0fb3d4606c05dff553b75baa641`. This is the commit whose
  ROS 2 source the chapters describe; the history is in §8.
  - It is **not** the current HEAD. Commits that touch only `docs/framework_study/` or `CLAUDE.md`
    move HEAD but leave the baseline valid.
  - Any change to the source since this commit makes `tools/check_state.py` report REVIEW (exit code 2).
  - Stashes: 3, never touched.
- **Completed documentation files:**
  - `00_INDEX.md`, `01_architecture.md`, `02_ros_graph_and_dataflow.md`, `03_configuration.md`;
  - `ledger/*`;
  - the tools `stage0_extract.py`, `ledger_status.py` and `check_state.py` (baseline-aware since C-003);
  - `CHECKPOINTS.md` and `RESUME_PROMPT.md`;
  - the root `CLAUDE.md`.
- **Draft or stale:** `A_file_index.md` was generated in Stage 0 and does not reflect the Stage 1 chapter
  reassignments (§2). It gets regenerated in Stage 7.
- **Incomplete work carried forward:**
  - 47 ledger files are `partial`: for nodes, only the constructor interfaces were read. Their algorithms
    belong to the stage that owns their chapter.
  - Unreachability candidates (F-0-4) are not yet confirmed (Stage 7).
  - The guide-versus-code cross-check has not started (Stage 7, chapter 16).
  - Q-1-4 and Q-1-5 need the user's answer (§5).
- **Exact next action:** wait for the user to approve Stage 2. Once approved:
  1. run `tools/check_state.py`;
  2. record the stage start in `CHECKPOINTS.md`;
  3. begin with `04_math_foundations.md`, reading `franka_experiments/franka_experiments/utils/kinematics.py`
     and the other ch. 04 rows of the ledger (`grep '| 04 |' ledger/coverage_ledger.md`);
  4. then move to ch. 05, which opens with Q-1-2 and Q-1-3.
- **Resume procedure:** follow "Resuming a session" in `/CLAUDE.md`, or paste
  [RESUME_PROMPT.md](RESUME_PROMPT.md).

## 1. Stages

| Stage | Deliverables | Files in ledger | Status | Verified against baseline | Reviewed / approved |
|---|---|---:|---|---|---|
| 0 — Inventory | `00_INDEX.md`, `PROGRESS.md`, `A_file_index.md` (draft), `ledger/*`, `tools/stage0_extract.py` | 346 enumerated | **done** | `ba68077` | approved 2026-11-11 |
| 1 — Architecture & graph | `01_architecture.md`, `02_ros_graph_and_dataflow.md`, `03_configuration.md`; `tools/ledger_status.py` | ch. 01: 5 · 02: 4 · 03: 13, after reassignment; plus about 30 node, launch and C++ files read at interface level | **done** | `ba68077` (tree clean apart from `docs/framework_study/`) | acknowledged complete 2026-11-11 (Q-1-4, Q-1-5 still open) |
| 2 — Foundations & perception | `04`, `05` | ch. 04: 5 · 05: 62 | not started | | |
| 3 — Nominal motion & barrier | `06`, `07` | ch. 06: 25 · 07: 30 | not started | | |
| 4 — QP & actuation | `08`, `09` | ch. 08: 28 · 09: 6 | not started | | |
| 5 — Alternatives, ISO, tools | `10`, `11`, `13` | ch. 10: 9 · 11: 18 · 13: 43 | not started | | |
| 6 — Simulation | `12` | ch. 12: 63 | not started | | |
| 7 — Synthesis | `14`, `15`, `16`, `B`, `C`, final `00`/`A` | ch. 14: 22 · 16: 13 | not started | | |

The per-chapter file counts come from the provisional assignment in
[ledger/coverage_ledger.md](ledger/coverage_ledger.md). Files are reassigned by hand when reading
shows a better home, and the counts here are then updated.

## 2. Coverage status

| Status | Files |
|---|---:|
| pending | 290 |
| partial | 47 |
| read | 5 |
| documented | 4 |

`partial` was added in Stage 1. It means the parts the stage needed were read; for nodes, that is
the constructor interfaces. Chapter reassignments made in Stage 1:
- `utils/ros_setup.py` → ch. 10 (it is the velocity filter's helper);
- `utils/node_utils.py` → ch. 05;
- `utils/run_metadata.py` → ch. 13;
- the two hand-eye launch files → ch. 05.

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

## 5. Decisions and open questions

**Decisions recorded at the Stage 0 review (2026-11-11):**
- The missing untracked test is recorded and must not be recreated.
- Only HEAD is analysed. Stashes are never applied, modified or deleted.
- `ledger.json` and the tools stay in the repository, and no automatic commits are made.
- A pipeline's status is classified from code evidence. Where the evidence is ambiguous, it is
  marked as requiring user confirmation.

**Open after Stage 1:**

| ID | Question | Who can close it |
|---|---|---|
| Q-1-1 | Who compensates gravity in Gazebo? Neither `qddot_to_torque` nor `rt_torque_controller` adds `g(q)`. | Stage 4/6 (read `franka_ign_ros2_control` / the plant) |
| Q-1-2 | `real_time_distance` projects the depth image (`camera_depth_optical_frame`) with an extrinsic whose child frame is `camera_color_optical_frame`. Is the depth-to-colour offset handled? | Stage 2 |
| Q-1-3 | Do `camera_extrinsics.yaml` (projection) and `camera_link_extrinsics.yaml` (TF) describe the same camera pose consistently? | Stage 2 (numeric check) |
| Q-1-4 | Is Pipeline B (velocity) still run on hardware? Is Pipeline C (OSCBF) abandoned or parked? Is `CBFTorqueController` (`use_torque_controller:=cbf`) still used? | **user** |
| Q-1-5 | Is `start_move_group:=true` intended (for example, for another tool), or is it a leftover? | **user** |

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
