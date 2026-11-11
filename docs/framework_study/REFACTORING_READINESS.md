# Refactoring readiness

> Stage 7, 2026-11-11. Baseline `ba68077`. This document answers one question: **can the refactor
> be planned now, and in what order?** It does not start the refactor. It is built from chapters
> 14 (build and tests), 15 (code classes), 16 (findings, Parts D and E) and
> [SIMULATION_VALIDATION_PROTOCOL.md](SIMULATION_VALIDATION_PROTOCOL.md). Every refactoring change
> remains **incremental and individually approved by the user**.

---

## 1. Is the framework documented well enough to start *planning*?

**Yes, for planning. No, for executing a change yet.**

What is in place (Verified at the baseline unless stated):
- every production file of both packages is classified (ch. 15) and indexed (appendix A);
- the shipped control path (Pipeline A: perception → barrier → QP → torque → plant) is documented
  function by function, with its mathematics (ch. 04–09; 122 numbered equations, appendix B) and
  reproduced numerically by 480 checks (all passing on 2026-11-11, C-051) in `tools/stage2…7_verification/`;
- 125 findings are consolidated, prioritized and cross-linked (ch. 16);
- the old guide has been compared with the code (ch. 16 Part E): it must not be used as reference;
- a validation protocol exists for every future change (the protocol, phases 1–6).

What blocks **execution** of the first change (each is a prerequisite, §2 and §8):
1. **The install is not the source** (F-7-4, F-6-3): `/ros2_ws/install` is a mixed build of the
   branch `fr3-student-sim`. No run or test since 2026-11-10 15:42 +0100 tested the baseline.
2. **No baseline evidence exists**: no test result and no simulation run at `ba68077` (F-6-1; ch. 14
   §7.2). A regression check needs a reference to compare with.
3. **The acceptance thresholds are not chosen** (Q-6-3; protocol §10).
4. Several design questions decide what a "fix" even is (Q-4-1, Q-5-1, Q-7-1, Q-7-2; §8).

## 2. Issues to investigate before modifying production code

These are investigations (reading, building, running), not code changes. Each one changes what a
later change means or how it can be validated.

| # | Investigation | Why first | Needs |
|---|---|---|---|
| I-1 | Clean, revision-stamped rebuild of the baseline (14 §5.3); `check_install_provenance.py` must then report the install = `ba68077` | every later result depends on it (F-7-4, F-6-3, F-2-2) | user approval of `colcon build` (Q-7-3) |
| I-2 | Full unit-test run from each package directory, with the import guard of 14 §7.3; record pass/fail per test | the regression reference; F-7-7 (must fail), F-0-10 (skips), F-7-9 | approval of `pytest` (Q-7-3) |
| I-3 | Baseline Gazebo runs of the reference cell set, seeds as approved (protocol §6.1) | the regression reference for behaviour | approval of the runs, Q-6-3 |
| I-4 | P0 investigations of ch. 16 D.1, in order: F-4-12 (no rows before the first distance message), F-7-1 (recovery reference), F-4-5 (stale-state brake), F-2-11/F-3-2/F-3-13 (held row), F-4-1/F-4-2 (soft, coupled obstacle barrier) | each is a candidate S1 defect on the shipped path; their closed-loop effect is Unverified | sim runs (I-3 first) |
| I-5 | Q-6-4: decide whether the harness defects F-6-5 (truth holes), F-6-4 (QP-rate gate), F-6-11 (provenance), F-6-2 (dotted literal) are fixed **before** the refactor | they weaken every later comparison | user decision |
| I-6 | Q-4-3: where torque is saturated on hardware (libfranka, firmware) | F-4-6/F-4-7/F-7-6: no software clamp exists with `iso_enabled` false | external documentation / hardware |

## 3. Potentially removable code (candidates only)

Removal requires the procedure of ch. 15 §9: rerun `inventory.py` on the branch, search the
out-of-scope trees and the user's scripts, read the history, and **ask the user** (Q-7-2). A
deletion does not reach an existing install (F-7-4).

| Candidate | Class (ch. 15) | Evidence | Caveat |
|---|---|---|---|
| `utils/cbf_constraints.py` | LEGACY | no importer (F-3-1) | named in a comment of `cbf_OSCBF_filter.py` |
| `utils/cbf_qp.py` | LEGACY | no importer | — |
| `cbf_hard_limits.workspace_face_rows`, `velocity_accel_box` | LEGACY (functions) | no caller (F-4-11) | the module itself is CURRENT |
| `utils/node_utils.py`, `cbf_kinematics.py`, `camera_yaml.py`, `simulation_imports.py`, `ros.py` | LEGACY shims | import indirections | repoint importers first, one change each; `ros.py` is still imported |
| `utils/avoidance.py` | UNUSED CANDIDATE | tests only (F-3-18) | its docstring claims a use |
| `utils/static_scene.py`, `utils/surface_velocity.py` | EXPERIMENTAL (unwired) | "not yet wired" (`2402e58`, F-0-3) | intent known only to the user |
| `nodes/distance_visualization_node.py` | UNUSED CANDIDATE | no entry point (F-0-4) | — |
| `config/oscbf_params.yaml`, `fake_hw_controller_params.yaml`, `camera_intrinsics.yaml`; dead `params:` keys | UNUSED CANDIDATE | no reader (F-1-14) | `d_safe_default` is dead but misleading (0.15) |
| `config/conditions.d/mppi_it08.yaml` | UNUSED CANDIDATE | no reader (F-1-13) | — |
| `simexp/control_regression.py`, `graph_contract.py`, `provenance.py` | test-only | F-0-4 | `provenance.snapshot` *should* be called (F-6-11), not removed |
| launch-arg pass-throughs `lpf_alpha`, `tau_max_scale`, `urdf_path` | dead parameters | never declared by the C++ (F-1-8) | removing them changes generated YAML; textual tests may pin them |

The full list with confidence is in [15_code_status.md](15_code_status.md) §3–§6.

## 4. Active or safety-critical code: do not remove or weaken without evidence

- **Everything on Pipeline A** (ch. 15 §4.1): `cbf_safety_filter`, `cbf_state_rows`,
  `cbf_qp_assembly`, `cbf_hard_limits`, `cbf_singularity`, `self_collision`, `state_governor`,
  `cbf_zones`, `kinematics`, `distance_engine`, `mask_builder`, `obstacle_tracker`,
  `obstacle_track_pipeline`, `real_time_distance`, `pentagon_qddot_commander`,
  `singularity_recovery`, `qddot_to_torque`, `rt_torque_controller.cpp`, `launch_support`, `config`,
  `params`, `rate_scaling`, the two main YAMLs and `torque_control_stack.launch.py`.
- **Files flagged `TODO[LEGACY]` that are in fact live** (F-7-2): `self_collision.py` (row builder,
  `self_collision_rows_enabled: true`), `avoidance_math.py` (Pipeline B's model and joint order),
  `cbf_velocity_filter.py` and `ee_pentagon_velocity_commander.py` (Pipeline B, started by
  `velocity_cbf_control_stack.launch.py`).
- **Libraries that look like scripts**: `scripts/cbf_scenarios.py`, `scripts/compare_vobs.py` are
  imported by `simexp` by relative path (F-0-5).
- **EXPERIMENTAL safety mechanisms that are off** (SSB, obstacle memory, evidence margin, curvature
  term, ISO layer, `row_l1`): documented answers to observed failures. Keeping them is a research
  decision, not cleanup (ch. 15 §9.4).
- **Alternative pipelines** B, C and RL: keep until Q-1-4 / Q-5-1 / Q-7-1 are answered.

## 5. Tests usable as regression checks

Status: **none has been run at the baseline** (I-2). The useful kinds, by the static inventory
(ch. 14 §7.2, `test_traceability.py`; test files that import the module):

| Component | Direct test files | Usable as regression for |
|---|---:|---|
| `cbf_state_rows.py` | 35 | row construction, margins, evasion, link speed, SSB rows, evidence, curvature |
| `cbf_qp_assembly.py` | 16 | RHS, slack layout, padding, `row_l1`, fallback |
| `cbf_hard_limits.py` | 10 | boxes, envelopes, slew |
| `distance_engine.py` | 10 | distance semantics, depth gate, fail-closed |
| `obstacle_tracker.py` | 7 | Kalman, IMM, lifecycle, eviction |
| `kinematics.py` | 6 | Jacobians and derivatives (`test_kinematics_fix_regression`, `test_control_point_derivatives`) |
| `iso_ssm.py`, ISO nodes | 3 + 10 `test_iso_*` | ISO formulas and flags-off identity |
| `simexp` (sim package) | 16 files | conditions, catalog, laws, scoring, batch |
| **Identity tests** | `test_torque_stack_external_plant_off_identical`, `test_iso_flags_off_identical`, `test_experiment_logger_truth_off_identical` | a refactor that must not change behaviour with flags off |
| **Static lint** | `test_node_self_references` | a renamed method that a node still calls |

Use with care (ch. 14 §7.4): **replica tests** (`test_phase_governor.py`, `test_gamma_continuous`, a
helper in `test_cbf_state_rows.py`) cannot fail on a production change; **textual tests** (20 files)
fail on harmless rewording and pass on behaviour changes; F-7-7 is a known failing test.

## 6. Missing tests (gaps found in Stages 0–7)

| Gap | Why it matters | Finding |
|---|---|---|
| No test file imports `rt_torque_controller.cpp`, `timing_law.py`, `trajectory.py`, `launch_support.py`, `params.py`, `ground_truth_node.py` | the torque law, the nominal path, the generated controller YAML and the declared parameter surface have no direct regression | ch. 09, 06, 03, 12 |
| `pentagon_qddot_commander` has 2 test importers; the governor test is a replica | the nominal motion has no real regression test | F-3-3, F-3-11 |
| `singularity_recovery` tested only with v₀ = 0 | the default-on recovery's defect is invisible to the suite | **F-7-1** |
| No test of the QP with no distance message | the "no rows, no fault" start-up state | **F-4-12** |
| No test of the stale-joint-state fallback over time | the open-loop brake reversal | **F-4-5** |
| No test of a held distance entry (`ph = 0`) reaching the filter | the loosening row during a hold | **F-2-11, F-3-2, F-3-13** |
| No test of the per-row violation in `family` mode under a conflict | the soft, coupled obstacle barrier | **F-4-1, F-4-2** |
| No torque-saturation test (there is no clamp to test) | F-4-6, F-4-7, F-7-6 | Q-4-3 |
| No test that the install equals the source | F-7-4 went unnoticed for a day | `check_install_provenance.py` is a model |
| No closed-loop test outside Gazebo `slow` tests | every closed-loop claim depends on the simulator | protocol |
| `cbf_param_overrides` launch route has no assertion (lead from the old guide §13.1, not re-checked) | the ablation harness's parameter route | ch. 16 E.6 |

These are proposals for tests the user may approve, not tests to write now.

## 7. Baseline simulations to perform (after I-1, before any change)

From [SIMULATION_VALIDATION_PROTOCOL.md](SIMULATION_VALIDATION_PROTOCOL.md) §6.1 (the set itself
REQUIRES USER APPROVAL):
- the reference cell set: the pentagon motion with perception on, at least one static obstacle on the
  path, one moving obstacle (oscillate or wander), one ballistic throw, one occlusion case; each on the
  `full` condition and the shipped `qp_rate_hz`;
- the approved number of seeds, knowing that a seed mostly changes only the start phase (F-6-9);
- emulated **and** real perception (`perception:=true`), since emulation bypasses the estimator
  (F-6-8);
- the P0 probes of §2 I-4 as dedicated cells: delayed perception at start (F-4-12), a forced hold
  during an approach (F-3-13), a stall that triggers the recovery (F-7-1), frozen joint states (F-4-5).

Every run records: the revision, the install provenance check, the run manifest, the truth series,
the bag with `/rosout` (F-5-22) and the evidence directory of protocol §7.

## 8. Unresolved problems that need a user decision

| ID | Decision |
|---|---|
| Q-7-3 | approve the clean rebuild and the baseline test run (I-1, I-2) |
| Q-6-3 | reference cells, seeds, thresholds, statistical rule (protocol §10) |
| Q-6-4 | fix the harness defects first? |
| Q-4-1 | `family` or `row_l1` as the obstacle slack mode (the barrier is a penalty in `family`) |
| Q-4-2 | is the open-loop stale-state brake acceptable? |
| Q-1-4, Q-5-1, Q-7-1 | fate of Pipelines B, C, RL and `CBFTorqueController`; are any used on hardware? |
| Q-7-2 | fate of experimental, unwired and no-consumer files |
| Q-5-2 | ISO reset semantics (F-5-17) |
| Q-3-1 | margin-gate frame counts |
| Q-2-2 | re-derive `gate_max_m` for the 0.8 s coast (and then F-7-7) |
| Q-1-5 | `start_move_group:=true` intended? |
| Q-5-4 | were reported results computed from the corrupted logger columns (F-5-21)? |
| Q-7-4 | update, retire or replace the old technical guide |

Runtime questions that need an approved run: Q-2-3, Q-2-4, Q-3-2, Q-5-3, Q-6-5.

## 9. Separable refactoring workstreams

Each workstream can be planned and validated on its own; the dependencies are explicit.

| WS | Scope | Main findings | Depends on |
|---|---|---|---|
| W0 | **Reproducibility**: revision-stamped builds, install check in every run, test cwd guard, provenance snapshot | F-7-4, F-7-5, F-6-3, F-6-11, F-2-2 | — (first) |
| W1 | **Validation harness**: truth holes, QP-rate gate, scoring semantics, logger columns | F-6-5, F-6-4, F-6-6, F-5-21, F-6-13, F-6-2 | W0 |
| W2 | **Start-up and fault safety of the filter**: rows before perception, stale-state brake, failure ladder | F-4-12, F-4-5, F-4-4 | W0, W1 |
| W3 | **Barrier formulation**: slack mode, coupling, held row, label state, margin gates | F-4-1, F-4-2, F-2-11, F-3-2, F-3-13, F-3-12, F-3-6, F-3-5 | W2; Q-4-1 |
| W4 | **Nominal motion**: governor chain rule, singularity recovery, composite path, null space | F-3-3, F-7-1, F-3-9, F-3-14, F-3-11 | W0, W1 |
| W5 | **Torque realization**: saturation, comments, gravity notes, sim inertia | F-4-6, F-4-7, F-7-6, F-4-13, F-1-3, F-4-9 | Q-4-3 |
| W6 | **Perception**: hand-eye consistency, mask pose, sphere coverage, label collisions | F-2-7, F-2-9, F-2-4, F-2-10, F-2-8 | runtime Q-2-3/Q-2-4 |
| W7 | **Configuration hygiene**: dead keys and files, launch/YAML disagreement, duplicate joint limits, `d_safe` read from install | F-1-14, F-1-11, F-1-17, F-1-10, F-1-12, F-1-8 | W0 |
| W8 | **Code-structure cleanup**: legacy shims, unused modules, stale markers, docstrings | F-7-2, F-3-1, F-3-18, F-0-4, F-7-3, F-1-4 | Q-7-2; after W2–W4 |
| W9 | **Alternative pipelines and ISO layer** | F-5-2…F-5-7, F-4-8, F-5-8…F-5-20 | Q-5-1, Q-7-1, Q-5-2 |
| W10 | **Documentation**: the old guide, READMEs, `LEGACY.md` | F-7-8, F-0-9 | Q-7-4 |

## 10. Safest sequence

1. **W0, with no production change**: approved clean build (I-1) → `check_install_provenance.py`
   reports the baseline → full test run from each package directory (I-2) → record the outcomes as the
   test baseline (expected failures listed: F-7-7; expected skips: F-0-10).
2. **Approve the protocol's open items** (Q-6-3, Q-6-4) and run the **baseline simulations** (§7).
   Without them no change can be judged.
3. **W1** harness fixes, if approved (Q-6-4), each as one modification, then re-run the baseline.
4. **W2** (filter start-up and fault behaviour), one finding per change.
5. **W4** items that are local and well-tested numerically (F-7-1 first: its defect is reproduced by
   `check_unread_functions.py` and a v₀ ≠ 0 test can be added before the fix).
6. **W3** after the slack-mode decision (Q-4-1); this changes the barrier and needs the full cell set.
7. **W5, W6** as their external or runtime questions are answered.
8. **W7, W8, W10** last: low risk to safety but high risk of silently changing what runs (textual
   tests, installed copies, generated YAML).
9. **W9** only after Q-5-1 / Q-7-1.

**For every single change**, the protocol's ten steps apply, and the change is accepted only if all
of these pass and are recorded:
- the ROS 2 build (and the install-provenance check);
- the automated tests, run from each package directory;
- a complete Gazebo start-up;
- controller activation;
- perception and CBF operating (rows built, QP solving at its rate, status published);
- the robot actually moving along the nominal motion;
- the safety and regression checks of protocol Phase 6 against the baseline runs.

A change is **not** successful because the build passes or because Gazebo opens. After each change:
update the affected chapters ("Consequences of modification" sections), mark fixed findings
superseded (never delete them), move the baseline only through PROGRESS §8, and stop for the next
approval.
