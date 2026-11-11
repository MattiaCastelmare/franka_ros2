# Studying the FR3 safety framework — index and reading order

This is a study guide to two ROS 2 packages: `franka_experiments` and
`franka_simulation_experiments`, together with the parts of their workspace dependencies that
actually execute in the pipeline. It is written **from the source code**. Existing documents
(READMEs, `LEGACY.md`, `SAFETY.md`, `franka_experiments/docs/FRAMEWORK_COMPLETE_TECHNICAL_GUIDE.md`,
root-level reports) are used only as leads. Every claim taken from them is re-verified, and the
places where they disagree with the code are collected in chapter 16.

It is meant to be studied, not only looked up. Later, during a refactor, it also serves as a
reference for what each change will affect.

> **Status: Stages 0–7 complete (Stage 7, the synthesis, awaiting review; not committed).** The
> read-only documentation phase is finished; the refactor has **not** started and needs separate,
> per-change approval ([REFACTORING_READINESS.md](REFACTORING_READINESS.md)). The live status is in the resume card,
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
| 08 | [`08_constraints_and_qp.md`](08_constraints_and_qp.md) | The QP actually solved (objective, OSQP form), every constraint family (hard box vs soft rows), slack modes and their safety meaning, OSQP numerics, fallback ladder, command ordering | 4 | **written** (Stage 4) |
| 09 | [`09_torque_realization.md`](09_torque_realization.md) | Inverse dynamics (`qddot_to_torque`), the C++ `ros2_control` controllers, multi-rate timing, Gazebo vs FR3 (gravity: Q-1-1; inertia) | 4 | **written** (Stage 4) |
| 10 | [`10_alternative_controllers.md`](10_alternative_controllers.md) | Velocity CBF plus executor, Dynamic-OSCBF, RL path; the barrier family across relative degrees | 5 | **written** (Stage 5) |
| 11 | [`11_iso_safety_layer.md`](11_iso_safety_layer.md) | ISO-inspired speed-and-separation and PFL math, the configuration gate, monitor, preflight, evidence logger; monitoring vs enforcement (no compliance claim) | 5 | **written** (Stage 5) |
| 12 | [`12_simulation.md`](12_simulation.md) | Gazebo plant, spawn, controller activation, sim nodes, perception modes, scenarios and obstacle laws, conditions, ground truth, reflex monitor, batch runs, what each metric means | 6 | **written** (Stage 6) |
| P | [`SIMULATION_VALIDATION_PROTOCOL.md`](SIMULATION_VALIDATION_PROTOCOL.md) | The per-modification procedure of the refactor: the ten-step rule, Phases 1–6, acceptance criteria (thresholds marked for user approval), evidence, revert. Documented, not executed | 6 | **written** (Stage 6) |
| 13 | [`13_observability_and_tools.md`](13_observability_and_tools.md) | Logger, `/cbf_status` schema, bags and replay, overlays, analysis-tool inventory, misleading diagnostics | 5 | **written** (Stage 5) |
| 14 | [`14_build_launch_test.md`](14_build_launch_test.md) | Workspace, Docker, how colcon installs each package, source vs install (F-6-3/F-7-4), launch files, test suites and which code a test run exercises (F-7-5), runtime diagnostics | 7 | **written** (Stage 7) |
| 15 | [`15_code_status.md`](15_code_status.md) | Every production file classified CURRENT / ALTERNATIVE / EXPERIMENTAL / LEGACY / UNUSED CANDIDATE / UNKNOWN, with evidence and confidence; the `TODO[LEGACY]` markers checked (F-7-2); removal rules | 7 | **written** (Stage 7) |
| 16 | [`16_issues_and_discrepancies.md`](16_issues_and_discrepancies.md) | All 125 findings by category with severity, priority, confidence, reproduction and direction (Part B); shared causes and corrections (C); P0–P3 (D); the old technical guide vs. the code (E) | 7 | **written** (Stage 7) |
| R | [`REFACTORING_READINESS.md`](REFACTORING_READINESS.md) | Is the framework ready for refactoring: prerequisites, removable vs protected code, regression tests and gaps, baseline simulations, user decisions, workstreams, safest order | 7 | **written** (Stage 7) |
| A | [A_file_index.md](A_file_index.md) | File → role → chapter → coverage status → code class → importers/launch/tests (generated) | 0 → 7 | **generated** (Stage 7) |
| B | [`B_equation_index.md`](B_equation_index.md) | 122 numbered equations → chapter line → code pointers (generated) | 7 | **generated** (Stage 7) |
| C | [`C_glossary.md`](C_glossary.md) | Symbols, terms, and the symbols that mean different things in different chapters | 7 | **written** (Stage 7) |

**Reading paths:**

- **First pass, the whole picture:** 01 → 02 → 06 (intuition sections) → 07 (intuition) → 08
  (intuition) → 09 (intuition) → 12 (intuition) → 14 §1 and §5 → 16 Part D. Keep C open.
- **Mathematical path:** 04 → 05 → 06 → 07 → 08 → 09 → 10 → 11, with B and C open alongside.
  Each chapter's verification scripts (`tools/stageN_verification/`) reproduce its numbers.
- **The safety argument and where it breaks:** 07 §2 (HOCBF) → 08 §2 and §5 (soft rows, box, brake)
  → 09 (torque) → 16 Part D.1 (P0 list) → 11 (what the ISO layer does and does not add).
- **Before refactoring anything:** [REFACTORING_READINESS.md](REFACTORING_READINESS.md) → 14 §5
  (make the install match the source) → [SIMULATION_VALIDATION_PROTOCOL.md](SIMULATION_VALIDATION_PROTOCOL.md).
- **Before refactoring one module:** A (the file's row: chapter, class, importers, tests) → 15 (its
  class and evidence) → its chapter's "Consequences of modification" → 16 (its findings) → B (its
  equations).
- **Do not use** `franka_experiments/docs/FRAMEWORK_COMPLETE_TECHNICAL_GUIDE.md` as a reference: it
  is stale at the baseline (16 Part E, F-7-8).

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
| [tools/stage4_verification/](tools/stage4_verification/README.md) | QP assembly, slack semantics, box and brake, inverse dynamics and controller-source checks of chapters 08–09, built from the production QP builders and solved with the deployed OSQP 0.6.7 in the container (`run_all.sh`; 61 of 61 on 2026-11-11) |
| [tools/stage5_verification/](tools/stage5_verification/README.md) | Velocity-CBF, OSCBF, torque-commander, ISO and observability checks of chapters 10, 11 and 13, calling production methods unbound on stubs, with the deployed qpsolvers/OSQP in the container (`run_all.sh`; 92 of 92 on 2026-11-11) |
| [tools/stage6_verification/](tools/stage6_verification/README.md) | Conditions, obstacle laws and catalog, scorers, reflex checker, emulation, depth conversion, robot/world builders of chapter 12 on production code; the recorded-run audit; the installed-vs-source state check (`run_all.sh`; 69 of 69 on 2026-11-11) |
| [tools/stage7_verification/](tools/stage7_verification/README.md) | Static inventory and test traceability; the Stage 7 defect reproductions (F-7-1, F-7-3, F-7-4); the old guide against the baseline; documentation consistency (`check_docs.py`); the generators of appendices A and B (`run_all.sh`; 47 of 47 on 2026-11-11) |
| [drafts/](drafts/11_iso_safety_layer.parallel_session.md) | A parallel session's draft of chapter 11, kept verbatim at the reconciliation (C-035); not a chapter |
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
  Their layouts are described from the code that writes them. Exception found in Stage 7:
  `franka_logs/` holds one tracked tool, `plot_franka_log.py`, which is therefore outside the ledger
  (F-7-11; classified in ch. 15 §6.3).
