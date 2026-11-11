# Studying the FR3 safety framework — index and reading order

This is a study guide to two ROS 2 packages: `franka_experiments` and
`franka_simulation_experiments`, together with the parts of their workspace dependencies that
actually execute in the pipeline. It is written **from the source code**. Existing documents
(READMEs, `LEGACY.md`, `SAFETY.md`, `franka_experiments/docs/FRAMEWORK_COMPLETE_TECHNICAL_GUIDE.md`,
root-level reports) are used only as leads. Every claim taken from them is re-verified, and the
places where they disagree with the code are collected in chapter 16.

It is meant to be studied, not only looked up. Later, during a refactor, it also serves as a
reference for what each change will affect.

> **Status: Stages 0–3 complete (Stage 3 awaiting review); Stage 4 not yet approved.** The live status is in the resume card,
> [PROGRESS.md](PROGRESS.md) §0. Project rules and the resume procedure are in
> [/CLAUDE.md](../../CLAUDE.md). The session history is in [CHECKPOINTS.md](CHECKPOINTS.md). To
> resume in a new session, paste [RESUME_PROMPT.md](RESUME_PROMPT.md).

---

## 1. Source revision this documentation describes

| Item | Value (verified 2026-11-11 11:18 CET) |
|---|---|
| Branch | `humble-mattia` |
| HEAD | `ba68077052e8d0fb3d4606c05dff553b75baa641`: "Add the self-shadow barrier, off by default, with what it can and cannot do" (10 Nov 2026) |
| Working tree | clean (`git status --porcelain` empty) before `docs/framework_study/` was created |
| Stashes | 3, kept as they are and **not** described here: `stash@{0}` (10 Nov, `OPEN_ISSUES.md` only), `stash@{1}` (30 Oct, "P1-P4 gates + self-reject/obstacle-floor/static-scene-truth"), `stash@{2}` (2 Jul) |
| Discrepancy (decision: record it, do not recreate) | The session-start snapshot listed an untracked file, `franka_experiments/test/test_status_residuals_and_qp_scheduling.py`. When re-checked, the file no longer existed. It is not ignored, not in any commit and not in any stash. It was removed outside this documentation session; nothing in this work touched it. |

This commit is the **documented source baseline**. Later commits that touch only the documentation
(`docs/framework_study/`, `CLAUDE.md`) move HEAD without invalidating it. A change to the source
since the baseline is reported by `tools/check_state.py` as REVIEW. The affected chapters are then
re-checked, and only after that does the baseline move ([PROGRESS.md](PROGRESS.md) §8, which also
holds the baseline history).

## 2. Evidence labels

| Label | Meaning |
|---|---|
| **Verified** | Read directly in executable source, a declaration or a configuration value, with file and function named. |
| **Derived** | An algebraic consequence of verified statements. The derivation is shown. |
| **Inferred** | Likely behaviour or engineering intent that was not executed or observed. |
| **Unverified** | Needs runtime, hardware, an external package's behaviour, or a measurement. It is never presented as fact. |
| *Extracted* | Produced mechanically by the Stage 0 scripts and not yet checked by reading the code. This appears only in `ledger/`. |
| **Runtime (recorded)** | Taken from an artefact that a previous run wrote, such as a `rates.txt`, a `graph.json` or a `cell_result.json`. The artefact's revision and scope are named, and it was not re-run here. Added in Stage 1. |

Code references use repository-relative paths plus `file::function`, because line numbers drift in
this actively edited tree. Line numbers, when given, refer to the HEAD recorded in that chapter.

## 3. Template of every subsystem chapter

Each major subsystem is explained in five parts, followed by a refactoring note:

1. **Intuition.** What problem is being solved, why it exists, and what would go wrong without it.
2. **Mathematical foundations.** Complete derivations. Every symbol is defined with its frame, its
   unit and its assumptions, and every simplification is stated.
3. **Implementation.** Equation by equation, which `file::function` computes it, which parameters
   enter, and where the code departs from the mathematics.
4. **Integration.** Inputs and outputs (topics, messages, parameters, rates), upstream and
   downstream subsystems, timing, and failure behaviour.
5. **Practical example.** Concrete numbers or a scenario traced through the code.
6. **Consequences of modification.** What depends on this subsystem, which tests pin it down, and
   which hidden couplings exist (shared constants, mirrored copies, the `franka_sim` mirror).

## 4. Chapters and recommended reading order

| # | File | Topic | Stage | Status |
|---|---|---|---|---|
| 01 | [01_architecture.md](01_architecture.md) | Both packages and how they relate; the four control pipelines; which launch file selects what | 1 | **written (Stage 1)** |
| 02 | [02_ros_graph_and_dataflow.md](02_ros_graph_and_dataflow.md) | Nodes, topics, services, parameters, TF tree, rates, QoS, timing chain | 1 | **written (Stage 1)** |
| 03 | [03_configuration.md](03_configuration.md) | YAML → launch → node parameter flow; configuration key families; dead or overridden keys | 1 | **written (Stage 1)** |
| 04 | [`04_math_foundations.md`](04_math_foundations.md) | Frames, FR3 kinematics (Pinocchio), control-point Jacobians J and J̇, dynamics M, C, g | 2 | **written** (Stage 2) |
| 05 | [`05_perception_and_distance.md`](05_perception_and_distance.md) | Calibration, depth → points, self-masking, distance engine, tracking, prediction | 2 | **written** (Stage 2) |
| 06 | [`06_nominal_motion.md`](06_nominal_motion.md) | Commanders, paths and timing laws, Cartesian PD + damped IK + null space, phase governor, filter-side shaping (zone fade, state governor, tangential bias, go-around, evasion, livelock), RL policy, Pipeline B commanders | 3 | **written** (Stage 3) |
| 07 | [`07_cbf_obstacle_barrier.md`](07_cbf_obstacle_barrier.md) | HOCBF derivation, the implemented obstacle rows, velocity estimation, margins, evidence gates, zone ladder, F-2-10/F-2-11 traced through the barrier | 3 | **written** (Stage 3) |
| 08 | `08_constraints_and_qp.md` | Other constraint families, QP assembly, slack, fallback, governor | 4 | not written |
| 09 | `09_torque_realization.md` | q̈ → τ, C++ real-time controllers, rates and threading | 4 | not written |
| 10 | `10_alternative_controllers.md` | Velocity CBF plus executor, Dynamic-OSCBF, RL path | 5 | not written |
| 11 | `11_iso_safety_layer.md` | ISO speed-and-separation math, monitor, evidence logger | 5 | not written |
| 12 | `12_simulation.md` | Gazebo plant, spawn, controller activation, sim nodes, experiment axes, batch runs, scoring | 6 | not written |
| 13 | `13_observability_and_tools.md` | Logger, overlays, offline scripts, bag replay | 5 | not written |
| 14 | `14_build_launch_test.md` | Build, launch recipes, test suites, validation | 7 | not written |
| 15 | `15_code_status.md` | Current / experimental / legacy / unused, with the evidence for each | 7 | not written |
| 16 | `16_issues_and_discrepancies.md` | Bugs, inconsistencies, math vs. code, undocumented assumptions, existing guide vs. code | 7 | not written |
| A | [A_file_index.md](A_file_index.md) | File → role → chapter | 0 → 7 | **draft** |
| B | `B_equation_index.md` | Equation → `file::function` | 7 | not written |
| C | `C_glossary.md` | Symbols and acronyms | 7 | not written |

**Reading paths** (to be refined once the chapters exist):

- **First pass, the whole picture:** 01 → 02 → 06 (intuition sections) → 07 (intuition) → 08 (intuition) → 12 → 14.
- **Mathematical path:** 04 → 05 → 07 → 08 → 09 → 10, with B and C open alongside.
- **Before refactoring a module:** 15 (its status) → its chapter's "Consequences of modification" → 16 (known issues) → A (every file involved).

## 5. Stage 0 artefacts (inventory)

| File | Content |
|---|---|
| [ledger/coverage_ledger.md](ledger/coverage_ledger.md) | Every in-scope file (346): lines, role, provisional chapter and stage, static reachability, status |
| [ledger/dependencies.md](ledger/dependencies.md) | Declared and runtime dependencies, the direction between the two packages, `sys.path` shims, third-party libraries |
| [ledger/ros_interfaces.md](ledger/ros_interfaces.md) | Every publisher, subscription, service, client, timer, parameter declaration and TF call site (*Extracted*) |
| [ledger/launch_inventory.md](ledger/launch_inventory.md) | `Node`, `ExecuteProcess`, include and launch-argument call sites per launch file (*Extracted*) |
| [ledger/import_graph.md](ledger/import_graph.md) | Entry points, executables, the internal import graph, reachability classes, external imports (*Extracted*) |
| [ledger/module_definitions.md](ledger/module_definitions.md) | Top-level classes and functions per module (791 symbols). This is the symbol checklist for the chapters. |
| [ledger/ledger.json](ledger/ledger.json) | Machine-readable form of the above, used to update status in later stages |
| [tools/stage0_extract.py](tools/stage0_extract.py) | The read-only extractor that regenerates the `ledger/*.md` tables (`python3 -I tools/stage0_extract.py <repo> <out_dir>`) |
| [tools/ledger_status.py](tools/ledger_status.py) | Sets a file's status, chapter or stage in `ledger.json` and `coverage_ledger.md` at once (added in Stage 1) |
| [tools/check_state.py](tools/check_state.py) | Read-only consistency check, run at every session start and checkpoint: git position against the pinned HEAD, stray working-tree changes, agreement of the ledger totals, broken links |
| [tools/stage2_verification/](tools/stage2_verification/README.md) | Read-only scripts that reproduce every numerical result of chapters 04 and 05 (`run_all.sh`; host and container parts), with expected vs observed values and the list of what still needs runtime validation (added after Stage 2) |
| [tools/stage3_verification/](tools/stage3_verification/README.md) | Read-only scripts reproducing the numerical results of chapters 06 and 07, two of which drive the real `ConstraintBuilder` in the container (`run_all.sh`; 85 of 85 checks on 2026-11-11) |
| [CHECKPOINTS.md](CHECKPOINTS.md) / [RESUME_PROMPT.md](RESUME_PROMPT.md) | Append-only session log, and the prompt for resuming in a new session |

## 6. Scope

- **In scope (346 files):** everything git tracks in both packages, except data directories. It
  also includes the executed-path files of workspace dependencies: the three `franka_rt_controllers`
  C++ sources with their YAML and plugin XML, `franka_simulation/launch/gz_fr3_plant.launch.py`, and
  the four `franka_msgs` distance messages.
- **Interface only:** the rest of `franka_simulation` (its `scripts/utils` modules move to "in
  scope" when Stage 5/6 confirms they execute), `franka_rl`/`franka_sim`, `franka_description`,
  `franka_bringup`, `franka_fr3_moveit_config`, `franka_hardware`, `franka_gripper`, `libfranka`.
- **Excluded as data:** `sim_runs/` (its copied `provenance/` code is not re-read),
  `experiment_logs/`, `franka_logs/`, `docs/paper_draft/`, `Papers/`, `Videos/`, `.pytest_cache/`.
  Their layouts are described from the code that writes them.
