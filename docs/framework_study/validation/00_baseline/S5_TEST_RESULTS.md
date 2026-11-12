# 00_baseline — S5: automated test execution (result)

> Approved by the user on 2026-11-12 (S5 only). Every test that needs neither Gazebo nor hardware
> was run against the S3/S4 install (`INSTALLED == SOURCE`, HEAD `f708991`, documented baseline
> `ba68077`). **No Gazebo, no launch file, no ros2_control process was started.** No source, test or
> configuration file was changed. Evidence: `~/franka_baseline_evidence/baseline_f708991/03_tests/`
> (31 files, `SHA256SUMS`, write-protected; SHA-256 of `SHA256SUMS`
> `fb68c59fe9ff8470bdb48349a73dfb34c1ced2ab01807543943ca1eccb185b6b`).
>
> **Headline (Runtime, recorded).** Functional pytest: **1587 tests, 1543 passed, 35 failed, 9
> skipped.** Every failure falls into one of four causes. All four are pre-existing and confined to
> tests or test data; none points to a defect of the rebuilt install. The 35 failures reproduce
> identically under `colcon test`. Smoke scripts: 1 passed, 1 failed (an outdated stub), 1 not run
> (no model). Style linters: `franka_simulation` fails 1155 lint checks; the other packages pass.

## 1. Preconditions (all passed, before any test)

| Check | Result |
|---|---|
| Branch / HEAD / baseline | `humble-mattia` / `f7089911decf` / `ba68077` is an ancestor; 0 non-documentation paths changed |
| `git status` outside `docs/framework_study/` | empty; 3 stashes (recorded, untouched) |
| `check_state.py` | consistent (exit 0) |
| Evidence `00_pre_rebuild`, `01_build`, `02_install_verification` | `SHA256SUMS` verify |
| Install vs S3 | `sha256sum -c` of 1158 files: OK. So `INSTALLED == SOURCE` holds (S4) |
| Container processes | only `/bin/bash`; no Gazebo, ROS, colcon or pytest process on the host or in the container |

## 2. Environment (`03_tests/environment.txt`)

- Container `franka_ros2`, Ubuntu 22.04, kernel 6.17.0-35, 28 CPUs.
- Python 3.10.12, pytest 6.2.5.
- Libraries: numpy 1.26.4, scipy 1.15.3, osqp 0.6.7.post3, qpsolvers 4.3.3 (solver `osqp`),
  pinocchio 4.0.0, PyYAML 5.4.1, onnxruntime 1.23.2, torch 2.13.0+cu130.
- Every run did `source /ros2_ws/install/setup.bash` and set
  `PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1`.
  - Why the isolation: the container uses host networking and `ROS_DOMAIN_ID` was unset (domain 0),
    so a test node could otherwise talk to a lab machine on the LAN.
  - Only `test_run_metadata.py` reads `ROS_DOMAIN_ID`, and it sets the variable itself
    (monkeypatch).
- `FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/src/fastdds_no_shm.xml` (inherited).
- **Import guard** (F-7-5): which copy each run imported.

  | Run (cwd) | `franka_experiments` resolved to |
  |---|---|
  | `src/franka_experiments` | the source |
  | `src/franka_simulation_experiments` | the install |
  | smoke scripts from `src` | the source (the script inserts the package directory) |

  Install == source (S4), so every resolution is a baseline file.

## 3. Suite inventory and classification

Each package has these kinds of tests:
- **U**: pure unit;
- **N**: numerical or mathematical;
- **R**: ROS 2 integration, single process;
- **L**: launch;
- **G**: needs Gazebo;
- **H**: needs hardware;
- **S**: style lint.

| # | Suite (package) | Kind | Executed? | Command (cwd) |
|---|---|---|---|---|
| 1 | `franka_experiments/test/test_*.py` (96 modules) | U, N; launch-*description* expansion without starting anything | **yes** | `python3 -m pytest test -p no:cacheprovider -v -rfEsxX --tb=long --durations=30 --junitxml=…` (`src/franka_experiments`) |
| 2 | `franka_simulation_experiments/test/` without `slow` | U, N (offline scenario laws, metrics, attribution, catalog) | **yes** | `python3 -m pytest -v -rfEsxX --tb=long --durations=30 --junitxml=…`. `pytest.ini` adds `-m "not slow" -p no:cacheprovider` (`src/franka_simulation_experiments`) |
| 3 | `franka_simulation/test/sim/` | U (robot-description generation, sim config) | **yes** | `python3 -m pytest -v -rfEsxX --tb=long --durations=15 --junitxml=…` (`src/franka_simulation/test/sim`) |
| 4 | `franka_experiments/test/smoke_cbf_construct.py` | U (stubs; constructs `CBFSafetyFilter` without ROS) | **yes** | `timeout 180 python3 -B franka_experiments/test/smoke_cbf_construct.py` (`src`; see §5.2) |
| 5 | `franka_experiments/test/smoke_cbf_safety_filter.py` | R (one process: rclpy, a stimulus node and the real `CBFSafetyFilter`; no launch) | **yes** | `timeout 180 python3 -B test/smoke_cbf_safety_filter.py` (`src/franka_experiments`) |
| 6 | `franka_experiments/test/smoke_rl_policy_commander.py` | R (needs an ONNX model) | **attempted; NOT RUN** | `timeout 180 python3 -B test/smoke_rl_policy_commander.py` (`src/franka_experiments`) |
| 7 | `colcon test` of the six packages | S (ament_lint_auto via ctest) + a re-run of suite 1 (ament_python) | **yes** | `colcon test --base-paths src --packages-select <six> --event-handlers console_cohesion+`, then `colcon test-result --test-result-base build/<pkg> --verbose` (`/ros2_ws`) |
| — | `franka_simulation_experiments/test/slow/test_phase1…5_*.py` (22 tests) | G (start the Gazebo plant and the stack) | **no** | deselected by `pytest.ini`; listed in `suite2_deselected_slow_collection.txt` |
| — | `franka_simulation_experiments/test/launch/launch_test_graph_contract.py` | L + G (starts `gz_cbf_stack`) | **no** | — |
| — | `franka_experiments/test/launch/*.launch.py` | L (ros2_control fake-hardware stack) | **no** | a complete stack is excluded by the S5 approval |
| — | `franka_experiments/test/e2e_pentagon_moveit.sh` | L, multi-process (`ros2 run` / `topic pub`), no assertions | **no** | — |
| — | `franka_experiments/test/scripts/check_topics.sh`, `franka_simulation/test/scripts/check_pipeline.sh` | need a running stack | **no** | — |
| — | `franka_simulation/test/*.py` demos, `test/launch/*`, `test/scripts/test_*_publisher.py` | G | **no** | — |
| — | real-robot tests (`minimal`/`torque_control_stack` on hardware) | H | **no** | — |

`colcon test` does **not** run suite 2. `franka_simulation_experiments` is an `ament_cmake` package
with no registered test, so colcon reports 0 tests for it. Its 121 non-slow tests are reachable only
through a direct pytest run (part of F-8-2).

## 4. Results

| # | Suite | Collected | Passed | Failed | Skipped | Other | Exit | Time |
|---|---|---:|---:|---:|---:|---|---|---:|
| 1 | `franka_experiments` pytest | 1427 | 1383 | **35** | 9 | 23 warnings | 1 | 26.67 s |
| 2 | `franka_simulation_experiments` pytest (not slow) | 143 | 121 | 0 | 0 | 22 deselected (slow, G); 1 warning | 0 | 19.73 s |
| 3 | `franka_simulation/test/sim` pytest | 39 | 39 | 0 | 0 | 1 warning | 0 | 0.43 s |
| 4 | `smoke_cbf_construct.py` | — | — | **1** (CONSTRUCTION FAILED, TICK FAILED) | — | the script exits **0** anyway | 0 | 0.38 s |
| 5 | `smoke_cbf_safety_filter.py` | 3 phases | **SMOKE TEST PASSED** | 0 | — | — | 0 | 6.9 s |
| 6 | `smoke_rl_policy_commander.py` | — | — | — | **NOT RUN** | "No ONNX policy found" before `rclpy.init` | 1 | 0.28 s |
| 7a | colcon `franka_msgs` (lint) | 5 | 5 | 0 | 0 | — | — | 1.2 s |
| 7b | colcon `franka_ign_ros2_control` (lint) | 37 | 30 | 0 | 7 | cppcheck self-skips ("cppcheck 2.7 performance issues") | — | 1.1 s |
| 7c | colcon `franka_rt_controllers` | 0 | — | — | — | the package registers no tests | — | 0.03 s |
| 7d | colcon `franka_simulation` (lint) | 1164 | 8 | **1155** | 1 | cppcheck self-skip | — | 1.7 s |
| 7e | colcon `franka_experiments` (pytest re-run) | 1427 | 1383 | **35** | 9 | the same 35 test IDs as suite 1 (junit set comparison) | — | 26.6 s |
| 7f | colcon `franka_simulation_experiments` | 0 | — | — | — | no registered test (see §3) | — | 0.04 s |

How to read these numbers:
- **colcon's exit code.** `colcon test` exited **0** (29.8 s in total) despite the failures. This is
  colcon's default without `--return-code-on-test-failure`. The counts above come from
  `colcon test-result`.
- **Totals.**
  - Functional pytest (suites 1–3): 1587 executed, 1543 passed, 35 failed, 9 skipped.
  - Smoke scripts: 2 executed (1 passed, 1 failed), 1 not run.
  - Lint: 1206 checks, 43 passed, 1155 failed, 8 skipped.
  - Suite 7e repeats suite 1 and is not added to the totals.
- **Time.** Suite 1 is dominated by `test_cbf_row_padding` (4.2 s). Suite 2 is dominated by
  `test_throw_sweep_matches_approach_envelope_cells` (13.6 s).
- **Warnings: none relates to a failure.**
  - Suite 1:
    - scipy `delta_grad == 0.0` in `test_cbf_row_padding`;
    - qpsolvers/osqp "Converted P/G to csc_matrix" in `test_cbf_velocity_filter` and
      `test_ssb_qp_rows`;
    - numpy `invalid value encountered in matmul` in
      `test_self_shadow_derivatives::test_a_non_finite_input_is_refused…`: the test feeds a
      non-finite input on purpose, and passes.
  - Suites 2 and 3: a `construct` private-variable deprecation.

## 5. Failure analysis (static comparison with the production code; nothing modified)

Each failure has its full traceback in `suite1_franka_experiments_pytest.log`.
`suite1_failures_and_skips_classified.txt` maps all 35 test IDs to a cause; none is unclassified.
Every failure reproduced: it failed in both the direct run and the colcon run.

### 5.1 Suite 1: 35 failures in four causes

| Cause | n | Failing assertion (first line) | Comparison with production code | Class | Finding |
|---|---:|---|---|---|---|
| **A**: tests written ahead of unwired features | 25 | `_cbf_builder_harness.py:290 AssertionError: unknown parameter 'enable_static_scene_hold'` (also `enable_static_scene_voxels`, `enable_surface_velocity_cap`, `obstacle_velocity_floor_min_frames`); `AttributeError: 'DistanceEngine' object has no attribute 'last_self_reject_px'` / `last_floor_drop_px` / `_self_reject_tol`; `assert 1600 == 0` | The parameters and attributes do not exist in `cbf_safety_filter` / `DistanceEngine`. `static_scene.py` and `surface_velocity.py` are not wired (F-0-3). Commit `2402e58` (2026-11-05) states verbatim that these three files "FAIL in this state, by 25 cases. They are committed as the specification of the wiring, not as a passing suite." | pre-existing; outdated **production** relative to the tests (deliberate) | F-0-3 (outcome observed), F-8-2 |
| **B**: private rclpy API | 1 | `test_node_self_references::test_every_node_self_call_resolves`: "handeye_eye_in_hand_node.py:200 HandEyeEyeInHandNode._parameters is used but never defined or assigned" | L200 `{name: self.get_parameter(name).value for name in self._parameters}` uses `rclpy.node.Node._parameters`. That attribute exists in the installed rclpy, so the node does not crash. The lint flags any `self._x` the class never defines. `5731c47` (2026-10-22) had replaced it with `get_parameters_by_prefix('')`; `e9ff7cd` (2026-10-30) reverted the fix. | pre-existing production defect (regression of a fix; outside the control path) | F-7-9 (outcome observed) |
| **C**: test against the shipped YAML | 1 | `test_rate_scaling.py:336 assert 0.8 == 0.167 ± 1.7e-07` | Shipped `fr3_complete.yaml` `tracking.max_coast_s: 0.8` (changed 2026-11-01, L896–900); the test still asserts 0.167 | outdated test | F-7-7 (outcome observed) |
| **D**: stale launch golden | 8 | `test_torque_stack_external_plant_off_identical`: `assert [...] == [...]` in all 8 cases (3 variants × 2 tests + the `external_plant` and `motion_source:=none` tests) | Exactly **2 differing leaves** per variant (`diag_launch_snapshot_diff.txt`): (1) the `real_time_distance` node: `robot_config_path` `<tmp:df2a01c32d756c6b>` → `<tmp:3a253d3f7d80e364>`, the content hash of the temp copy of `fr3_complete.yaml`, which changed after the golden (`e9ff7cd`, `8bf658f`, `ba68077`); (2) the `cbf_safety_filter` node: `robot_config_path` `<absent>` → `<install>/…/config/fr3_complete.yaml`, added by `25be305` (2026-11-05, also to `iso_safety_monitor` and `iso_evidence_logger`, which are not in the default expansion). The golden was last generated in `97db307` (2026-10-22). The two variant tests fail at the shifted index of the same `real_time_distance` element (Derived from the reported index) | pre-existing; outdated test **data** after deliberate launch/YAML changes | **F-8-1** |

### 5.2 Smoke scripts

**`smoke_cbf_construct.py` fails, and its exit code hides it.**
- **First run, from `src/franka_experiments` (planned cwd).** `FileNotFoundError:
  …/franka_experiments/franka_experiments/config/fr3_control.yaml`.
  - The stub (L28–30) returns `os.path.abspath('franka_experiments')` as the package share
    directory. From the package directory that path is the Python package, not the package root.
  - This cwd was my error: the stubs assume the repository root.
- **Run from `src`, the cwd the stubs assume (L4, L30).** Construction fails with
  `kinematics.py:600 configure_control_points_from_node … AttributeError: 'FakeKin' object has no
  attribute 'configure_control_points'`.
  - The stub class `FakeKin` (L64–71) predates that API. `cbf_safety_filter.__init__` L212 has
    called `configure_control_points_from_node` since `aad4c86`/`341929b` (2026-11-03).
  - The tick section then fails with `NameError: node`.
- **Exit code.** Both runs print `=== CONSTRUCTION FAILED ===`, and the script **exits 0**: it
  catches every exception and has no assertion or `sys.exit`.
- **Classification.** An outdated test (stale stub), pre-existing, reproducible. The exit code
  makes it a silent false pass for any automated runner. → **F-8-3**.
- **Reproducibility.** The two runs are not retries with a modified configuration; they are the two
  candidate working directories. The second is the intended one. Both logs are kept.

**`smoke_cbf_safety_filter.py` passes.** The real node runs in one process at a measured 300 Hz.
- Phase 1: 898 outputs, all finite. `max|out−nom| = 1.000`: the obstacle row binds (n_c = 2;
  the QP status is solved, 25 iterations, 0.30 ms).
- Phase 2: the distance stream stops. The node brakes ("distance stale → braking fallback (CBF
  inactive)"), output ≈ 0 against a nominal of 5.0.
- Phase 3: the nominal stops. The node brakes, output ≈ 0.

This is the only R-class evidence of S5. It runs the node without Gazebo and without the real
perception pipeline.

**`smoke_rl_policy_commander.py` is NOT RUN.**
- It printed "No ONNX policy found. Train + export one first: `python3 -m franka_sim.train` …" and
  exited 1 before `rclpy.init`.
- Two reasons:
  - the models are gitignored and not part of the baseline;
  - `find_sim_root` looks for `franka_sim/`, while the directory is `franka_rl/` (F-0-10).
- Classification: missing artifact plus the F-0-10 naming defect; environment, not install.
- Running it needs a model chosen by the user (`--model`), which is outside S5.

### 5.3 Skips: 9, all NOT RUN

All nine are in `test_rl_policy.py`:
- "franka_sim checkout not available in this layout" (and variants);
- the sim/real parity tests: observation layout, extended observation, shipped sim config, real
  configs in sync, accel cap, workspace box, shield families, shield parity, EE frame.

Cause: F-0-10 (`franka_sim` versus `franka_rl`). The skip effect stated in F-0-10 as Unverified is
now **observed**: 9 skips, against the "9 of 44" of the project note. The sim/real parity of the RL
observation and shield is therefore **untested at the baseline**.

### 5.4 Lint (colcon)

`franka_simulation` declares `ament_lint_auto` and fails 1155 checks
(`lint_franka_simulation_breakdown.txt`):

| Linter | Failures | Where | Typical code |
|---|---:|---|---|
| flake8 | 760 | 30 files | W293 280, E501 155, W191 74, E221 65 |
| pep257 | 313 | 14 files | D213 114 |
| copyright | 60 | 60 files | no copyright header |
| cpplint | 12 | 2 files | include order 11 |
| lint_cmake | 3 | 2 files | — |
| uncrustify | 1 | `src/scenario_obstacles_system.cpp` | — |

The remaining 6 are the per-linter ctest wrappers.

- Style only; no functional implication was identified. Pre-existing. → **F-8-4**.
- `franka_msgs` and `franka_ign_ros2_control` pass.
- In both C++ packages, `cppcheck` disables itself for cppcheck 2.7: the C++ code is not
  statically analysed by the shipped lint.

## 6. Findings and observations recorded

The findings use the series **F-8-n**, "stage 8 = baseline validation". IDs F-0…F-7 belong to the
documentation stages.

**New findings in `PROGRESS.md` §4 and chapter 16 B.10:**

| ID | Summary |
|---|---|
| F-8-1 | The launch golden `torque_control_stack_94cd7b0_snapshot.json` is stale: 8 failures, 2 leaves |
| F-8-2 | No green automated gate exists at the baseline. pytest exits 1 (35 known failures), `colcon test` exits 0 whatever happens, and colcon does not run the `franka_simulation_experiments` suite. A regression is detectable only by comparing against the recorded set of 35 test IDs |
| F-8-3 | `smoke_cbf_construct.py` has a stale stub and exits 0 on failure |
| F-8-4 | 1155 `ament_lint_auto` failures in `franka_simulation` |

**Observed outcomes appended** (original text kept): F-0-3, F-0-10, F-7-7, F-7-9; Q-7-3.

**Observations, not findings (the O-series convention of S1–S4):**
- **O-16.** `utils/config.py::resolve_log_dir` probes writability by touching and unlinking
  `.write_test` in `franka_experiments/franka_logs/` (the source tree, gitignored). It runs at
  import time of `experiment_logger` (`DEFAULT_OUTPUT_DIR`).
  - Effect: the directory's mtime changed during the pytest runs (12:53:28).
  - What was checked: no file in that directory is newer than the pre-test marker, and
    `git status --ignored` lists only pre-existing files.
  - Limit: the file count before the run was not recorded.
  - Consequence: importing the package writes, briefly, into the source tree.
- **O-17.** Production code leaves temporary files in the container's `/tmp` and never deletes
  them: 63 `fr3_cbf_*.urdf`, 55 `fr3_sc_*.urdf`, 23 `franka_rt_blender_*.yaml`, and 5
  `fr3_complete_launch_<pid>.yaml` (the launch snapshot tests).
- **O-18.** pytest's basetemp rotation (keep 3) deleted three older `/tmp/pytest-of-user/pytest-329…331`
  directories in the container. They predate S5 and are not evidence. The new
  `pytest-333…335` were kept.

## 7. Side-effect verification (after all runs)

| Check | Result |
|---|---|
| `git status --porcelain --untracked-files=all` | byte-identical to the pre-test record |
| Stashes | 3, identical; branch `humble-mattia`, HEAD `f708991` |
| Repository files newer than the pre-test marker (tracked, untracked, ignored; outside `.git` and `docs/framework_study`) | only the directory entry `franka_experiments/franka_logs` (mtime, O-16), no file |
| Container `docker diff` (before → after) | additions only in `/ros2_ws/build/<pkg>/` (colcon test: `test_results`, `Testing`, `ament_*`, `colcon_test.rc`, `pytest.xml`, `.pytest_cache`), `/ros2_ws/log/test*` + `latest_test*`, `/home/user/.ros/log` (rclpy logs of suite 5), `/tmp` (O-17, pytest basetemps, `/tmp/s5` junit); removals only O-18 |
| Install | `sha256sum -c` against the S3 list: all 1158 OK (unchanged) |
| Quarantine `/ros2_ws/quarantine_s3_20261112/` | listing identical to `01_build/quarantine_listing.txt` |
| Gazebo / launch / ros2_control | never started; at the end only `/bin/bash` runs in the container |

## 8. Readiness for a Gazebo smoke test (assessment, not a decision)

**Derived.** On the evidence of S5, nothing blocks a Gazebo smoke test:
- the install equals the source;
- every non-Gazebo functional failure is explained, pre-existing, and confined to tests or test
  data;
- the CBF filter node runs, solves and brakes correctly in a single process;
- the offline harness (scenario laws, catalog, metrics) passes in full.

**Caveats to carry into that step:**
1. The launch-structure regression test is red (F-8-1). The torque stack's launch expansion has no
   green guard. The current difference is fully explained (2 leaves), but a further unintended
   change would be hidden among the 8 failures until the golden is regenerated (the user's
   decision).
2. F-8-2: judge any later test run by comparing its failing set against the 35 IDs in
   `suite1_failures_and_skips_classified.txt`, never by the exit code.
3. The 22 slow Gazebo tests and the launch graph contract are the next unexecuted layer. They are
   the natural content of the smoke step.
4. The open P0 findings (ch. 16 Part D) are unchanged by S5. Tests at this level do not exercise
   them.

## 9. Not done (needs approval)

- Gazebo and every G/L/H test in §3.
- Regenerating the golden; fixing any test or the stub.
- Choosing a model for the RL smoke test.
- `--filter-cpu`, thresholds and seeds (Q-6-3, Q-6-4).
- The fate of the quarantine.
