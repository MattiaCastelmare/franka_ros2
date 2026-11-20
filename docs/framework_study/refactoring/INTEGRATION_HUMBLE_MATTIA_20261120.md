# Integration of the executor investigation into `humble-mattia` (C-080)

Message 18, 2026-11-20. Labels follow `00_INDEX.md` §2. The integration put the R1 and H2 work, and
all the documentation, into `humble-mattia`. **It deliberately left out the R2 executor-default
change.**

## 1. Branches and what they contained (Verified, git)

The three branches form one straight line of history (no divergence):

```
humble-mattia (625c179) ── R1 / H2 / M1 / M3 / M3b ──> refactor/cbf-executor-scheduling (168b163)
                                                         └── R2 config + R2 tests + R2 docs ──> refactor/r2-single-executor-default (9ba26d2)
```

- `humble-mattia` was an ancestor of both refactor branches; the R1 branch was an ancestor of the R2
  branch. **R2 already contained every R1 / H2 commit**, so one integration covers both.
- 22 commits were in R2 and not in `humble-mattia`. Only five touch code:

| Commit | What | Integrated? |
|---|---|---|
| `9bb2531` | R1: `executor_mode` parameter (`single` / `multi`) in `cbf_safety_filter`; **default `multi` = the historical 3-worker executor**; tests | **Yes** |
| `73ed807` | Gazebo harness: the `full` condition pins `executor_mode: multi` so `--set` can select the executor | **Yes** |
| `186b823` | H2: opt-in phase-aligned obstacle birth (`episode_start:=phase`; default unchanged), tests | **Yes** |
| `cadaf62` | **R2: default executor → `single`** (`fr3_control.yaml`, `conditions.yaml` pin) | **No: left out** |
| `359dd7c` | R2 tests that assert the `single` default | **No: left out** (they test `cadaf62`) |

The other 17 commits touch only `docs/framework_study/` (and `CLAUDE.md` in `9bb2531`).

## 2. How it was integrated (Verified, git)

1. **Fast-forward merge** of `refactor/cbf-executor-scheduling` (`168b163`) into `humble-mattia`.
   There were no conflicts, by construction.
2. **Cherry-picks (`-x`) of the three R2 documentation-only commits**, so the R2 experiments are
   recorded on `humble-mattia` without the R2 code:

   | R2 branch | humble-mattia |
   |---|---|
   | `6d5caca` | `0895a2c` |
   | `e4e5331` | `68e2cdd` |
   | `9ba26d2` | `62c0c5f` |

   They applied without conflict.
3. This record, the C-080 checkpoint and finding updates were added in one documentation commit.

**Checks after the integration:**
- **Outside `docs/framework_study/`, `humble-mattia` is byte-identical to `168b163`.** `git diff` is
  empty.
- The documentation is identical to the R2 branch head, until this commit.
- **The whole-tree difference from the R2 branch is exactly the diff of `cadaf62` + `359dd7c`**
  (5 files).
- Shipped default on `humble-mattia`: `executor_mode: multi` (`fr3_control.yaml` L317,
  `conditions.yaml` L109).
- `SingleThreadedExecutor` stays selectable:
  - `--ros-args -p executor_mode:=single` on the node;
  - `--set executor_mode=single` for the Gazebo `full` condition (F-8-58: only `full`).

**Note on the commit hashes.** The cherry-picked documents still cite the R2-branch hashes
(`cadaf62`, `359dd7c`, `6d5caca`, `e4e5331`, `9ba26d2`). Those commits exist on
`refactor/r2-single-executor-default` (local and `origin`), not on `humble-mattia` (F-8-65).

## 3. Build and tests on the integrated source (Runtime, recorded)

**Build.** As documented (R2 report §3):
- the previous R2 build of both packages was moved to `/ros2_ws/quarantine_hm_20261120/` (with
  `COLCON_IGNORE`), not deleted;
- `colcon build --base-paths src --packages-select franka_experiments franka_simulation_experiments
  --cmake-args -DCMAKE_BUILD_TYPE=Release` gave rc 0, 2 packages, 0 symlinks;
- installed `fr3_control.yaml` and `conditions.yaml` are SHA-256-equal to the source; the installed
  default is `multi`.

**Tests.** The same runner and environment as S5 / R1 / M3 (`run_suites.sh`, `ROS_DOMAIN_ID=77`,
localhost only). Evidence: `~/franka_baseline_evidence/integration_humble_mattia_20261120/`.

| Suite | Sealed M3 baseline (same R1 + H2 code) | humble-mattia | Compared test by test |
|---|---|---|---|
| 1 `franka_experiments` | 1405 / 35 / 9 | **1405 / 35 / 9** | 0 IDs added or removed, **0 outcomes changed**; failing set identical |
| 2 `franka_simulation_experiments` | 179 / 0 / 0 (22 deselected) | **179 / 0 / 0** (22 deselected) | 0 changed |
| 3 `franka_simulation/test/sim` | 39 / 0 / 0 | **39 / 0 / 0** | 0 changed |
| Smoke `smoke_cbf_safety_filter.py` (real node, rclpy) | 3 / 3 | **3 / 3** rc 0 | — |

Columns are pass / fail / skip.

**The 35 known failures, reported separately.** They are the same IDs as the baseline (F-8-2,
triaged in F-8-55), and all are test infrastructure, not new defects:

| Module | Failures | Cause |
|---|---:|---|
| `test_cbf_surface_velocity` | 14 | Unwired specification |
| `test_cbf_static_scene` | 5 | Unwired specification |
| `test_self_reject_and_obstacle_floor` | 6 | Specification |
| `test_torque_stack_external_plant_off_identical` | 8 | Stale launch golden, F-8-1 |
| `test_rate_scaling` | 1 | Outdated `max_coast_s` |
| `test_node_self_references` | 1 | Checker false positive |

**New failures: none.**

**Areas covered by suite 1.** This is an indicative keyword grouping of test IDs; the areas overlap:

| Area | Results |
|---|---|
| Executor selection | 22 pass |
| CBF mathematics | 51 pass |
| QP construction and constraints | 156 pass, 2 skip |
| Kinematics | 31 pass |
| Perception and distance | 272 pass, 8 fail (the stale golden) |
| Safety and fallback | 99 pass |
| Configuration loading | 48 pass, 1 fail (`max_coast_s`), 3 skip |

Python imports and real ROS 2 node start-up are exercised by the smoke test and by the node tests in
suite 1.

**Documentation checks.**
- `check_docs.py`: 14 of 14.
- `check_state.py`: integrity consistent, exit 2 (REVIEW) for the 11 R1 / H2 source paths. This is
  expected: see §5.

Not run in this phase: Gazebo, the robot.

## 4. Status of the investigation

1. **Implemented.**
   - R1, a configurable CBF executor: integrated.
   - H2, opt-in phase-aligned obstacle birth: integrated.
   - R2, default `single`: on its branch only.
   - The F-8-41 moving-obstacle correction: in the experimental harness only
     (`tools/f841/`, `refactor_r2/06_f841/`).
   - Analysis and replay tools M1, M3, M3b, R2 and the static Gazebo validation (`tools/`).
2. **Experiments performed.**
   - R1 offline A/B.
   - The E3 Gazebo pilot and three H2 pairs (E3-05…10).
   - M1, an executor-thread replay.
   - M3, replay under synthetic host load.
   - M3b, real-time playback.
   - The F-8-44 isolated tests.
   - The R2 offline suites, smoke and sequential identity.
   - The R2 sealed performance regression.
   - The R2 static Gazebo campaign (4 runs).
3. **Validated.**
   - R1 with default `multi`: outputs unchanged; suites unchanged.
   - H2: unit tests; six valid E3 runs.
   - Single holds about 100 QP ticks per sim s where multi degrades:
     - E3, Gazebo: 99.0–99.8 vs 66.8–89.1;
     - M3 under load: 99.94 vs 79.95;
     - R2 Gazebo: 98–99 vs 61–65.
   - Single has fresher constraints and smoother commands.
   - F-8-41 correction: exact static equivalence on 10 sealed runs, plus synthetic moving cases.
4. **Acceptance criteria that failed** (unchanged):
   - **R2 offline A_L1:** NOT_REPRODUCED (gap p99 13 vs ≤ 12 ms), not accepted (F-8-60);
   - **R2 Gazebo static Q1:** NO (D − M clearance −9.3 / −7.1 mm vs 5 mm), so Q6 DOES_NOT_SUPPORT
     (F-8-61);
   - M3b L1, single at rate 1.0 under load: FAIL, 89.44 ticks per sim s (F-8-52).
5. **Unresolved.**
   - Moving-obstacle validation: **not performed**.
   - F-8-44: governor σ / snapshot coherence, under both executors; fix P2 not made.
   - Q1 tolerance calibration (F-8-63).
   - F-8-56: OSQP ρ interval.
   - F-8-58: `--set` works only for `full`.
   - F-8-64: summary references.
   - F-8-1: launch golden.
   - Documented-baseline review, §5.
   - Q-7-2, Q-2-2.
6. **Ready for integration, and now integrated:** R1, H2, the tools and the documentation.
7. **Needs further validation before integration:**
   - **R2**, the default change: a pre-registered Q1 repetition, then the moving-obstacle phase. The
     merge needs your explicit decision.
   - F-8-41 needs a moving run.
   - F-8-44 / P2 needs its own step.

## 5. Documented source baseline (pending review)

The chapters describe baseline B-1 (`ba68077`). R1 and H2 are now in `humble-mattia`, so
`check_state.py` lists 11 changed source paths, each with its primary and mentioning chapters:
- `cbf_safety_filter.py`, `config.py`, `fr3_control.yaml`, `test_cbf_executor_mode.py`;
- `conditions.yaml`, `gz_cbf_stack.launch.py`, `episode_supervisor.py`, `run_gz_scenarios.py`,
  `batch.py`, `phase_start.py`, `test_phase_start.py`.

Their changes are documented in the refactoring reports (`R1_EXECUTOR_CONFIGURATION.md`, the C-065…
C-072 records). **The chapters themselves are not yet re-checked.** Under PROGRESS §8 the baseline
moves (to B-2) only after that review. By §8 rule 5, it comes **before** any refactoring stage.

## 6. Branches after the integration

See the C-080 checkpoint for the final state.
- **Kept:** `refactor/r2-single-executor-default` (local and `origin`). It holds the only copy of
  `cadaf62` and `359dd7c`, R2 is not approved, and safe deletion (`-d`) would refuse it.
- **Fully merged** and eligible for safe deletion after the push: `refactor/cbf-executor-scheduling`.
- **Not touched:** the other people's branches (`dani-thesis`, `fr3-student-sim`,
  `tesi_federico_urbini`, `thesis_TAS`, `humble`).

## 7. Preparation for the monolithic-node refactoring (no code changed)

Sizes and structure were measured on `humble-mattia` (Verified: `wc -l`, `ast`). Length alone is not
the criterion. Each candidate below combines several responsibilities, or very long single functions,
with shared mutable state.

| # | File (chapter) | Lines | Main responsibilities | Why it may be monolithic | Helps novelty development? |
|---|---|---:|---|---|---|
| 1 | `franka_experiments/franka_experiments/utils/cbf_state_rows.py` (ch. 07) | 4354 | Builds every QP constraint row from a state snapshot. The rows: obstacle barriers with tracking, smoothing, memory and criticality weights; self-collision; shadow; joint limits; workspace; retreat and ISO speed caps; stuck-joint bookkeeping. It also defines `ConstraintSnap` | **`ConstraintBuilder.build` is one 1,482-line method** (L2335–3816) with 25 interleaved concerns and about 97 instance attributes carried between calls (track and memory state). F-8-44 / F-8-45 come from state shared between this build and the tick | **High.** Most novelty work, such as a new barrier, row type or obstacle model, lands here. Splitting it into per-row-family builders, behind tests that check identical row output, would make such additions local and testable |
| 2 | `franka_experiments/franka_experiments/nodes/cbf_safety_filter.py` (ch. 08) | 2137 | ROS wiring, 4 subscriptions and 3 callback groups. The rebuild timer and the QP tick. Governor, go-around, livelock, tangential bias, slew and state box, OSQP solve, fallback ladder, publishing, diagnostics | It already delegates the mathematics to about 10 utility modules. The remaining issue is orchestration and state: **`_qp_tick` is 474 lines, `__init__` 396, 89 attributes**, shared by callbacks that may run on different threads under `multi` (F-8-44, F-8-45) | **High.** Making the per-tick data flow explicit (one immutable snapshot in, one command out) is what P2 / F-8-44 needs, and it would make the executor choice irrelevant to correctness |
| 3 | `franka_experiments/franka_experiments/nodes/pentagon_qddot_commander.py` (ch. 06) | 1688 | Nominal motion: trajectory and timing law, Pinocchio kinematics, task-space control, null-space and avoidance-first redirection, anti-windup, singularity recovery, safety clamp, logging | **`__init__` is 594 lines with about 67 parameters and 181 attributes; `_tick` is 352 lines.** Nominal tracking and avoidance shaping are mixed in one loop | **Medium to high.** A novelty in the nominal or avoidance layer, and the RL commander, would reuse the pieces if task-space control and avoidance shaping were separate components |
| 4 | `franka_experiments/franka_experiments/nodes/real_time_distance.py` + `utils/distance_engine.py` (ch. 05) | 1255 + 1014 | Depth-to-distance perception: masks, self-detection, obstacle tracks, ballistic prediction, rate scaling, calibration checks, publishing | Already partly split into modules (`DistanceEngine`, `MaskBuilder`, track pipeline). It still has a **433-line `__init__`, a 253-line `_process_depth_impl`, and a worker thread with a lock**; `DistanceEngine.compute` is 338 lines | **Medium.** Useful if novelty touches perception (e.g. surface velocity, F-8-41's moving obstacles). Otherwise lower priority, because its interface (`MultiLinkDistance`) is already clean |
| 5 | `cbf_velocity_filter.py` (623) and `cbf_OSCBF_filter.py` (971) (ch. 10) | 1594 | Alternative velocity-level and operational-space CBF filters | Each has its **own QP and CBF construction** (`qpsolvers` or their own builders); they do not share `cbf_qp_assembly` / `cbf_state_rows`. This is duplication rather than one oversized node | **Low to medium.** Only if novelty compares filter formulations. They are not in the shipped torque pipeline |

Not candidates for this purpose:
- `handeye_calibration_node.py` (2561 lines): an offline calibration tool, outside the control loop;
- `experiment_logger.py` (1125 lines): 9 subscriptions, but a single responsibility.

**Suggested order** (each step needs its own approval):
1. The §5 baseline review (B-2).
2. Characterisation tests that freeze the outputs of #1 and #2 on recorded inputs. The executor-free
   sequential replay (`r1_sequential_outputs.py`) already does this, apart from the OSQP timing noise,
   F-8-56.
3. Split #1 by row family.
4. Make #2's tick snapshot explicit. This includes P2 / F-8-44.
5. #3 only if novelty needs it.

This complements `REFACTORING_READINESS.md` §9–§10. Workstream W8 there covers cleanup, not
decomposition.
