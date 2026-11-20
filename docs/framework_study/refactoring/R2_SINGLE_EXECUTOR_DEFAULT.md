# R2: SingleThreadedExecutor as the shipped default (configuration only); offline validation and performance regression

> **Status (2026-11-20, C-077).** Approved by the user: STEP 1 to STEP 8.
>
> **Done:**
> - M3b committed;
> - R2 implemented on its own branch (configuration only, no CBF or QP code);
> - tests updated and extended;
> - offline validation;
> - a sealed, prospective performance protocol, executed once without retries;
> - a Gazebo validation plan (not executed).
>
> **Not done:**
> - nothing merged or pushed;
> - R2 is **not committed**: it is the working-tree diff on `refactor/r2-single-executor-default`, and
>   committing needs approval;
> - no Gazebo, no robot, no CPU-setting change;
> - no F-8-44 fix, no F-8-41 correction.
>
> **Labels:** Verified / Derived / Inferred / Unverified / Runtime (recorded).
>
> **Evidence:** `~/franka_baseline_evidence/refactor_r2/`.
>
> | Directory | Content |
> |---|---|
> | `01_offline_tests/` | Suites, smoke tests, diff |
> | `02_sequential_identity/` | Executor-free output comparison |
> | `03_f844/` | The F-8-44 tests on the R2 install |
> | `04_perf/` | Protocol, scripts, pre-flight, smoke, data and analysis; sealed |

## 1. STEP 1: M3b finalised (Verified)

**Git state before.**
- Branch `refactor/cbf-executor-scheduling`, HEAD `c21a570`, 3 stashes.
- The uncommitted set was exactly:
  - the four M3b tools (`m3b_run.sh`, `m3b_batch.sh`, `m3b_analyze.py`, `test_m3b_tools.py`);
  - the tools README;
  - the C-076 docs (PROGRESS, CHECKPOINTS, chapter 16, `R2_READINESS_M3B_AND_TEST_TRIAGE.md`).
- No production diff (0 lines outside `docs/framework_study`).

**Tools.** The four M3b tools, and the M1 / M3 tools they import, are byte-identical to the sealed
`refactor_r1_m3b/scripts/` copies (11 files).

**Seals.** Every M3b seal verifies:
- `protocol/` (SEALED_AT 2026-11-20T13:06:20Z);
- `scripts/`;
- `preflight.SHA256SUMS`;
- `smoke.SHA256SUMS`;
- `data/SHA256SUMS_raw_data` (67 files);
- `data/SHA256SUMS_analysis`.

One residue: `preflight.SHA256SUMS` was still writable, so it was made read-only (content unchanged).
`scripts/__pycache__/` (from the sealed test run) is not in any manifest.

**Checks.**
- `test_m3b_tools.py`: 17/17.
- `test_m3_tools.py`: 21/21.
- `check_docs.py`: 14/14.
- `check_state.py`: exit 2 (REVIEW for the R1 / H2 source paths, expected).

**Commits** (local, user authorship only, no trailer, not pushed):
- `0581aa9` tools: M3b real-time-playback replay harness, analysis and tests;
- `168b163` docs: M3b results, triage of the 35 package-test failures and R2 plan; findings F-8-52 to
  F-8-55 (C-076).

## 2. STEP 2: the R2 branch (Verified)

- `refactor/r2-single-executor-default`, created with `git switch -c` from **`168b163`** (the baseline
  commit = `refactor/cbf-executor-scheduling` after the M3b commits).
- `humble-mattia` is untouched, nothing was merged, and the stashes are untouched (3).

## 3. STEP 3: the change (Verified)

`grep` over `*.py`, `*.yaml`, `*.xml`, `*.sh` outside `docs/`: only two configuration files set
`executor_mode`.
- No launch file passes it.
- The parameter spec (`utils/config.py` L733) has no default.
- `OPEN_ISSUES.md` L187 mentions `MultiThreadedExecutor` only as a historical measurement.

| File | Change |
|---|---|
| `franka_experiments/config/fr3_control.yaml` L305–321 | `executor_mode: multi` → **`single`**. The comment now says that `single` is the default since R2 and `multi` the default until R2, cites the offline evidence (R1_M1 / R1_M3 / M3b), states that under heavy host load at 1.0× neither mode holds 100 Hz, that this is offline evidence only, and how to run `multi` without editing the file |
| `franka_simulation_experiments/config/conditions.yaml` L109 (`full.cbf_params`) | `executor_mode: multi` → **`single`**, the key kept (F-8-53), plus a 4-line comment: it equals the shipped default; the key stays so that `--set executor_mode=multi` works |

**Not changed:**
- CBF equations, QP formulation, constraints, perception, kinematics, commander, fallbacks, limits;
- any `.py` under `franka_experiments/franka_experiments/`;
- `make_executor` and `main`;
- `qp_thread_fifo_prio` (0; `single` refuses a positive value).

**Multi stays selectable.**
- `cbf_param_overrides:=executor_mode=multi` (torque stack);
- `--ros-args -p executor_mode:=multi`;
- `--set executor_mode=multi` (Gazebo `full`).

**Limitation found (F-8-58, pre-existing):** no other condition accepts `--set executor_mode`.

**Build.**
- The clean-workspace procedure, as for R1 and H2.
- The old `build/` and `install/` of both packages went to `/ros2_ws/quarantine_r2_20261120/` (with
  `COLCON_IGNORE`).
- Command: `colcon build --base-paths src --packages-select franka_experiments franka_simulation_experiments --cmake-args -DCMAKE_BUILD_TYPE=Release`: rc 0, 2 packages, 0 symlinks.
- **Install diff, pre-R2 vs R2:** exactly `fr3_control.yaml` and `conditions.yaml`, plus the ament
  `parent_prefix_path` metadata of `franka_simulation_experiments`. That file lists 17 instead of 12
  parent prefixes because the full overlay was sourced at build time; it is build-environment
  metadata, not a runtime input (Inferred).
- **Install vs source:** 109 files compared, 1 differs: `watchman_profile.md`, which is not installed,
  as at baseline. The installed YAMLs equal the source (sha256).

## 4. STEP 4: tests (Verified)

| Required | Test | Kind |
|---|---|---|
| 1. Shipped default | `test_cbf_executor_mode.py::test_shipped_default_is_single`: the source YAML says `single` **and** `qp_thread_fifo_prio == 0` (so the default can start) | replaces `test_shipped_default_is_multi` (assertion changed to the new intended value, docstring and module docstring updated) |
| 2. Main config vs Gazebo conditions | `test_conditions.py::test_the_full_condition_runs_the_shipped_executor_and_multi_stays_selectable`: `fr3_control.yaml` == `full.cbf_params` == `single`; it reaches `cbf_param_overrides`; `--set executor_mode=multi` gives `multi`. `::test_no_other_condition_pins_the_executor`: only `full` pins it, no global pin | new |
| 3. `main()` construction | `::test_main_with_the_shipped_configuration_spins_a_single_threaded_executor`: **real** `CBFSafetyFilter`, no override → `SingleThreadedExecutor`, node mode `single`, and the **installed** YAML (`P.config_path`) == source == `single` (catches a stale install) | new |
| 4. Explicit multi | `::test_main_explicit_multi_override_spins_the_three_thread_executor`: real node, `--ros-args -p executor_mode:=multi` → MT with 3 workers. `::test_ros_override_selects_multi_over_a_single_yaml` | new |
| 5. Smoke on the shipped default | `test/smoke_cbf_safety_filter_shipped_executor.py`: the filter alone on `make_executor(P.executor_mode, P.qp_thread_fifo_prio)`, the stimulus on its own executor. Phase 0 checks the executor (`--expect`); phases 1–3 are the original smoke checks (rate, CBF binding, braking on stale distances, braking on stale nominal) | new; the original 4-thread smoke is kept |

- `test_main_default_spins_the_three_thread_executor` was renamed
  `test_main_multi_spins_the_three_thread_executor`. Its assertion is unchanged and verified: it passes
  `multi` explicitly to a stub.
- No other test changed. No test was weakened or removed.
- Every other assertion in `test_cbf_executor_mode.py` was re-read against the new intended behaviour.
  They concern `declare_from_spec` and `make_executor` mechanics, independent of the default, and
  remain correct.

## 5. STEP 5: offline validation (Runtime (recorded) unless stated)

### 5.1 Package suites, R2 install (`01_offline_tests/run1/`)

The commands and environment are those of S5 / R1 / M3 (`run_suites.sh`). Each suite is compared test by
test with the sealed M3 run (`refactor_r1_m3/00_regression_tests/junit/`).

| Suite | M3 baseline (pass / fail / skip) | R2 (pass / fail / skip) | New failures | No longer failing |
|---|---|---|---|---|
| 1 `franka_experiments` | 1405 / 35 / 9 | **1408 / 35 / 9** | **0** | 0 |
| 2 `franka_simulation_experiments` | 179 / 0 / 0 | **181 / 0 / 0** | 0 | 0 |
| 3 `franka_simulation/test/sim` | 39 / 0 / 0 | **39 / 0 / 0** | 0 | 0 |

**The 35 known failures, reported separately.** They are the same 35 IDs as the baseline (F-8-2,
triaged in F-8-55). By module:

| Module | Failures | Cause |
|---|---:|---|
| `test_cbf_surface_velocity` | 14 | Unwired specification |
| `test_cbf_static_scene` | 5 | Unwired specification |
| `test_self_reject_and_obstacle_floor` | 6 | Specification |
| `test_node_self_references` | 1 | Checker false positive |
| `test_rate_scaling` | 1 | Outdated `max_coast_s` |
| `test_torque_stack_external_plant_off_identical` | 8 | Stale launch golden, F-8-1 |

Suite 1 therefore **does not pass as a whole**: it has 35 known failures and 0 new ones.

**Test-set changes.**
- Suite 1 lost 2 test IDs, the renamed ones (`test_shipped_default_is_multi`,
  `test_main_default_spins_the_three_thread_executor`).
- It gained 5 (`test_shipped_default_is_single`, `test_main_multi_spins_…`,
  `test_main_with_the_shipped_configuration_…`, `test_main_explicit_multi_override_…`,
  `test_ros_override_selects_multi_over_a_single_yaml`).
- Suite 2 gained the 2 condition tests.
- All 7 new tests pass. `test_cbf_executor_mode.py`: 25/25.

**Required areas, all passing.**

| Area | Modules (tests) |
|---|---|
| CBF mathematics | `test_cbf_state_rows` (43), `test_cbf_zone_ladder` (30), `test_cbf_lateral_evasion` (32), `test_cbf_uncertainty_margin` (17), `test_cbf_vobs_rate_independent` (23), `test_cbf_latency_compensation` (12), `test_kinematics_fix_regression` (15) |
| QP constraints | `test_cbf_hard_constraints` (26), `test_cbf_multi_cp_qp` (11), `test_cbf_row_l1_slack` (6), `test_cbf_singularity` (11), `test_cbf_selfcollision_anticipation` (11) |
| Safety margins | `test_iso_dsafe_floor` (10), `test_cbf_velocity_standoff` (10), `test_cbf_retreat_cap` (20) |
| Fallbacks | `test_cbf_qp_fallback` (10), `test_brake_slew_limited` (7) |
| Stale-data handling | `test_cbf_evidence_trust` (28), `test_cbf_fast_track` (7), the ISO fail-closed tests |
| Executor selection | `test_cbf_executor_mode` (25) |

### 5.2 Smoke tests (closed loop on synthetic inputs, real node, real executor)

**The original `smoke_cbf_safety_filter.py`** (its own 4-thread executor): 3/3 PASSED, rc 0, as in M3.

**The new shipped-executor smoke, run 1** (`01_offline_tests/run1/`).
- Every one of the 6 runs (3 default, 3 explicit multi) printed **SMOKE TEST PASSED**:
  - phase 0: the correct executor;
  - phase 1: ~300 Hz = `qp_rate_hz` of the shipped YAML, finite, CBF binding;
  - phases 2 and 3: braking output ~1e-323.
- 3 of the 6 then aborted at teardown (rc 134, `terminate called without an active exception`), in
  **both** modes.
- **Cause (F-8-59):** a defect of the test script I wrote, not of R2. It called `Executor.shutdown()`
  while the spin threads were still in the wait set.
- **Fix:** the threads run `spin_once` until a stop event and are joined before shutdown and destroy.
  `Executor.spin` itself is `while ok and not shutdown: spin_once()`.

**Run 2, after the fix** (`01_offline_tests/smoke_run2/`, script sha256 recorded): **10/10 PASSED,
rc 0**.

| Mode | Runs | Phase 0 |
|---|---|---|
| Default | 5 | `executor_mode=single (SingleThreadedExecutor)`, config = the installed YAML |
| `-p executor_mode:=multi` | 5 | `multi (MultiThreadedExecutor)` |

Production `main()` is not exposed to that teardown race (Verified, code): it spins in the main thread
and destroys the node only after `spin()` returns.

### 5.3 Control-output equivalence, executor-free (`02_sequential_identity/`)

**Method.** `r1_sequential_outputs.py`, unchanged (sha256 = the R1 sealed copy), runs the real node's
callbacks in recorded sim-time order on the S6B bag:
- no executor and no thread, so the scheduling is fixed;
- it records every published command, every solver return (q̈, slack, level, iterations, status,
  objective) and the row count per tick;
- 4068 records each.

**Runs.**
- **Pre-R2:** the quarantined R1 install, made explicit with `AMENT_PREFIX_PATH` / `PYTHONPATH`.
  - A first attempt that sourced the quarantine's `package.bash` silently loaded the **R2** install
    (module path checked); it is kept, relabelled `r2_default_b`.
- **R2:** the default, and `executor_mode=multi`.
- 4 more of each, for characterisation.

**Comparison: bitwise against R1's sealed `base_a`.**

| Runs | Result |
|---|---|
| Pre-R2 (5) | **All bit-identical** |
| R2 default (`r2_default`, `r2_rep1–4`) and R2 multi | **All bit-identical** |
| `r2_default_b`, the first replay after the build | Differs on **165 consecutive ticks** (sim 30.29–32.13 s) |

Details of the one deviation:
- OSQP reports "solved" in both runs, with different iteration counts (25 / 50 vs 75 / 100);
- max |Δq̈_safe| = **0.0376 rad/s²**;
- identical row counts;
- the outputs re-converge after 32.13 s.

**Parameter namespaces.** All 265 keys equal R1's, except:
- `executor_mode`, which is absent before R1 and otherwise as configured;
- `config_path` for the pre-R2 run.

**Interpretation (F-8-56).**
- *Verified:* OSQP 0.6.7 is set up without `adaptive_rho_interval`, so its automatic ρ-adaptation
  applies.
- *Inferred, from the OSQP documentation of `adaptive_rho_fraction`:* in that mode OSQP decides when to
  update ρ from the measured setup time, so its iteration path depends on wall time.
- One deviating ρ update changes one solve inside tolerance. Through the filter's state (the slew box
  uses the previous command) it then propagates until the state realigns. The deviation starts 17
  ticks into a constant-size problem (25 rows), which fits a timing-triggered update, not a setup
  difference.
- The code under test is identical before and after R2 (install diff = two YAML files; the executor
  setting is read only by `main`, `test_executor_mode_is_read_only_by_main`). So the deviation is
  **not an R2 effect**, and both installs are exposed to it (Derived).
- **Size:** 0.0376 rad/s² is inside R1 §9.2's timing-noise floor (scheduling-ideal p95 0.055).

**Conclusion for R2.**
- The control equations are unchanged: Verified (code), and Runtime in 11 of 12 bit-identical replays.
- The single deviating replay is a solver-timing effect of the kind measured in R1 §9.2, not
  bitwise-reproducible. The cause is pre-existing.

**Tolerance policy for later steps.**
- A bit-identity criterion ("max |Δ| = 0") must be judged on repeated runs, or with a harness-only fixed
  `adaptive_rho_interval`.
- Changing that interval in production would be a QP-setting change and needs its own approval.

### 5.4 Constraint-snapshot consistency (F-8-44), R2 install (`03_f844/`)

- `test_f844_governor_sigma_snapshot.py`, unchanged, with the E3-05 parameter files: **7 tests, 6
  pass, 1 expected failure (T6)**, exactly as in M3.
- `f844_results.json` is **identical** to the sealed M3 result.
- R2 neither fixes nor worsens F-8-44. Under `single` the normal-path mismatch cannot occur (T2);
  only the singularity-row exception path (T3) remains, as before. The F-8-44 correction (P2) stays a
  separate step.

### 5.5 Executor selection and callback behaviour

- **Executor selection:**
  - §4 tests 1–4;
  - the smoke phase 0 in both modes;
  - the dry runs of §6.3: the R2 install gives `SingleThreadedExecutor`, and the pre-R2 install (the
    negative control) gives MT(3).
- **Callback behaviour:**
  - the same callbacks, timers (10 ms QP, 20 ms rebuild) and subscriptions in every replay (`_m1.json`);
  - per-callback timing in §6.

## 6. STEP 6: performance regression of the shipped default (`04_perf/`)

### 6.1 Protocol, sealed before any run

- **Protocol:** `protocol/R2_PERF_PROTOCOL.md`, SHA-256 `13a7a9b5…`, SEALED_AT **2026-11-20T14:40:55Z**.
  The smoke run started after it, at 14:41:02, and the batch at 14:42:10.
- **Scripts:** `scripts/` (15 files, `SHA256SUMS`).
- **Tests:** `test_r2_tools.py` 18/18 at sealing time and again after the batch
  (`data/analysis/tests/`). The mutation check caught 4 of 4 mutations.
- **Thresholds:** those of `R2_READINESS…` §6.3 (commit `168b163`, before any R2 run). For C_L1, the same
  band rule adds the freshness and jitter limits.

**Harness: the arm `default` (`r2_default_replay.py`).** It is M3b's harness with the executor source
moved to the install. Two overrides would otherwise set the executor (F-8-57):
- the harness's own `-p executor_mode:=…`;
- the E3-05 parameter file's `executor_mode: multi`.

So the arm removes and records the first, and uses a copy of the parameter file without that one line.

**Pre-flight** (`preflight/`, sealed):
- **Repository:** branch, HEAD `168b163`, and the R2 diff with its SHA-256.
- **Install:** install == source.
- **Inputs:**
  - the bag and two parameter files are equal to the M3b reference hashes;
  - the third parameter file differs only by the removed line.
- **Host:**
  - governor 28 powersave and EPP 28 performance (unchanged);
  - no ROS or Gazebo process;
  - AC power;
  - the inhibitor probe works.
- **Dry runs:**
  - the R2 install gives `SingleThreadedExecutor`, mode `single`, and all 265 parameters equal to M3b
    `m3b_01`;
  - the pre-R2 install gives `MultiThreadedExecutor`/3, `multi`. This is the negative control: the arm
    follows the install.

**Smoke run** `smoke_r2_B_L0` (not counted): valid, 100.00 ticks per sim s, sealed.

**Batch.**
- 14:42:10–15:09:10 UTC under `systemd-inhibit --who=R2_BATCH`.
- **12/12 OK and valid**, 0 aborts, 0 retries, no suspend.
- Every run: `SingleThreadedExecutor`, node mode `single`, no override, parameters == M3b reference.
- At the end: inhibitor released, 0 load workers, 0 container ROS processes, CPU settings unchanged.
- Raw data (145 files) and analysis are sealed and read-only.

### 6.2 Results (Runtime (recorded)). Mean [min, max] of the 3 valid runs, with the sealed references (means)

**A_L0, rate 0.70, idle host.** Reference: M1 single and M3 single L0 (both: 100 ticks, 0 missed,
p99 11 ms, age p95 11 ms). Verdict: **REPRODUCED**.

| Metric | R2 |
|---|---|
| Ticks per sim s | **100.00** [100, 100] |
| Missed slots | 0 |
| Gap p95 / p99 | 11 / 11 ms |
| Gaps ≥ 50 ms | 0 |
| Snapshot age p95 | 11 ms |
| Python threads | 0.42 core |

**A_L1, rate 0.70, E3-like load.** Reference: M3 single L1 (99.94 ticks, 2.0 missed, p99 12 ms in all
three runs, age p95 11 ms). Verdict: **NOT_REPRODUCED**, because r2_04 has gap p99 = 13 ms against a
threshold of 12 ms.

| Metric | R2 |
|---|---|
| Ticks per sim s | **99.96** [99.96, 99.96] |
| Missed slots | 1.3 [1, 2] |
| Gap p99 | 12.3 [12, **13**] ms |
| Gaps ≥ 50 ms | 0 |
| Snapshot age p95 | 11 ms |
| Python threads | 0.86 core |

**B_L0, rate 1.0, idle host.** Reference: M3b L0 (99.99 ticks, 0.33 missed, p99 11 ms, age p95 11 ms).
Verdict: **REPRODUCED** (M3b PASS).

| Metric | R2 |
|---|---|
| Ticks per sim s | **100.00** [100, 100] |
| Missed slots | 0 |
| Gap p99 | 11 ms |
| Gaps ≥ 50 ms | 0 |
| Snapshot age p95 | 11 ms |
| Python threads | 0.58 core |

**C_L1, rate 1.0, E3-like load.** Reference: M3b L1 (89.44 [86.36, 92.21] ticks, 319 missed, p99
23 ms, 0.33 gaps ≥ 50 ms, age p95 15.7 ms, 0.95 core). Verdict: **REPRODUCED** (within the band:
every run ≥ 80.4 ticks, ≤ 571 missed, p99 ≤ 26 ms, age p95 ≤ 17.15 ms).

| Metric | R2 |
|---|---|
| Ticks per sim s | **94.14** [90.54, 96.54], **not 100**, as expected |
| Missed slots | 183 [111, 288] |
| Gap p95 / p99 | 17 / 21 ms |
| Gaps ≥ 50 ms | 0 |
| Snapshot age p95 | 15.3 [15, 16] ms |
| Python threads | 0.98 core (process 102 %) |

**Overall, by the pre-registered rule: NOT_REPRODUCED** (A_L1).

### 6.3 Investigation of the A_L1 shortfall (existing evidence only; no re-run, threshold unchanged)

What failed is **one 1 ms quantum of the gap p99 in one of three runs**. Ticks, missed slots, the
absence of 50 ms gaps and freshness met their criteria in all three runs.

**Derived, from the raw tick times.** The p99 of 2798 gaps is the 28th largest, so p99 = 13 ms exactly
when ≥ 28 gaps are ≥ 13 ms.

| Run | Gaps ≥ 13 ms | Gaps ≥ 15 ms | Gaps ≥ 20 ms |
|---|---:|---:|---:|
| r2_04 | 43 | 9 | 1 |
| r2_06, r2_09 | 20, 21 | 3, 6 | 0 |
| M3 m3_05 / m3_09 / m3_16 | 25 / 25 / 9 | 5 / 10 / 3 | 0 / 3 / 0 |

- r2_04's far tail (≥ 15 ms, ≥ 20 ms) lies inside M3's range.
- Only the 13–14 ms band is denser than in M3.
- M3 itself came within 3 gaps of the boundary (25 of 28).

**Runtime (recorded), host.** r2_04 had the longest rebuilds of all six rate-0.70 L1 runs and the
highest busy-core count:

| Runs | Rebuild p50 / p95 | Busy cores |
|---|---|---|
| r2_04 | 9.31 / 11.84 ms | 13.36 |
| The other five (R2 and M3) | 8.84–9.05 / 11.19–11.54 ms | 13.01–13.19 |

The P-core frequency median was 2466 MHz, against 2422–2577 MHz in the others.

**Inferred.** A tick that falls due during a rebuild waits for it, and longer rebuilds push more of
those ticks from 12 ms gaps to 13 ms gaps. This is run-to-run variation of the host load within the
pre-registered gate (11.5–15 busy cores), not a property of R2.
- R2's executor, callbacks, parameters (all 265 equal) and code are those of M3's single arm, which ran
  on the identical R1 code.
- The only difference is where `single` comes from: the YAML instead of `-p`.

**Derived.** The criterion "gap p99 ≤ 12 ms" (R1_M3 §7.3 item 7) was set at exactly M3's observed value
(12.0 in all three runs), with no allowance for run-to-run spread. The C_L1 band, by contrast, was
derived from the spread.

**Verdict as pre-registered:** NOT_REPRODUCED. It is reported as observed.

**User decision (message 16, 2026-11-20).** The 13 ms result is **not** accepted as passing the
original criterion. The R2 offline verdict **remains NOT_REPRODUCED** and the deviation stays
**unresolved**. The user considers it non-blocking for further experimental investigation (the Gazebo
static phase), not proof of successful validation. No threshold was changed and no repetition was run.

### 6.4 Other observations (reported, not gated)

- **C_L1 is better than M3b, not worse:**
  - 94.1 against 89.4 ticks per sim s;
  - 183 against 319 missed slots;
  - 0 against 0.33 gaps ≥ 50 ms;
  - the same freshness.

  But fewer joint states were processed (19.5 % against 26.3 %), with Python threads at 0.98 against
  0.95 core. The single thread is saturated in both cases; under saturation, a different interleaving
  shifts work between ticks and input callbacks (Inferred). The one-sided band does not reward this,
  and no claim is made from it (n = 3 per side; host variation as in §6.3).
- At rate 0.70 and 1.0 on the idle host, every metric equals the references to within the 1 ms
  quantum. CPU and per-operation times are within a few percent of M1, M3 and M3b.

## 7. STEP 7: answers

**1. Is SingleThreadedExecutor now the default everywhere intended?** **Yes** (Verified + Runtime).
- The shipped `fr3_control.yaml` (source and install).
- The Gazebo `full` condition.
- Every other Gazebo condition, which inherits the installed default.
- `main()` with the real node: `SingleThreadedExecutor` (test 3).
- The smoke phase 0, the dry run and all 12 performance runs.

No launch file overrides it.

**2. Can MultiThreadedExecutor still be selected explicitly?** **Yes**, through:
- `cbf_param_overrides:=executor_mode=multi`;
- `--ros-args -p executor_mode:=multi`, giving MT(3) (real-node `main()` test; 5 smoke runs; the
  executor-free replay);
- `--set executor_mode=multi` in Gazebo `full` (test 2).

**Limitation, pre-existing (F-8-58):** the other Gazebo conditions do not accept `--set executor_mode`.

**3. Did the CBF mathematical behaviour remain unchanged?** **Yes.**
- **Verified:** no `.py` changed. The install diff is two YAML files. The executor setting is read only
  by `main` (pinned by a test).
- **Runtime:**
  - 11 of 12 executor-free replays are bit-identical to R1's sealed reference on every command, solver
    return and row count;
  - the one deviating replay is OSQP's timing-dependent ρ-adaptation (F-8-56): pre-existing, both
    installs exposed, 0.038 rad/s² max, inside R1's timing floor;
  - all CBF, QP, margin and fallback suites pass;
  - the F-8-44 tests give identical results;
  - the parameter namespace is identical (265 keys) in all performance runs.

**4. Did all relevant offline regression tests pass?**
- The R2-relevant tests: **all pass**.
  - executor 25/25;
  - conditions 2 new;
  - smoke 3/3 (original) and 10/10 (shipped executor, after the test-script fix);
  - F-8-44 6 + 1 expected failure.
- Suite 2 (181) and suite 3 (39) pass completely.
- **Suite 1 does not pass as a whole:** 1408 pass, **35 fail (the known baseline failures, unchanged)**,
  9 skip.

**5. Were any new failures introduced?**
- **No new package-test failure** (failing set identical to M3's, test by test).
- **One new test's first version had a teardown defect** (F-8-59): rc 134 after PASSED, fixed in the
  test, and production is not exposed.
- **One pre-registered performance criterion was missed** (A_L1 gap p99, §6.3).

**6. Does R2 reproduce the established performance improvement?**
- **Pre-registered overall verdict: NOT_REPRODUCED.** The only miss is A_L1, where one of three runs
  has gap p99 13 ms against a 12 ms threshold.
- **Substantively, the improvement is reproduced** (Derived). Against M3's multi arm under the same
  load at 0.70 (79.95 ticks per sim s), R2 still gives:
  - **99.96** ticks per sim s in every A_L1 run;
  - ≤ 2 missed slots;
  - no gap ≥ 50 ms;
  - fresh constraints.

  At 1.0 R2 gives 100 on the idle host, and 94.1 under load (M3b 89.4; not 100, as expected).
- The A_L1 excess is one 1 ms quantum in the 99th percentile. It coincides with the run's longer
  rebuilds (host variation, Inferred). It sits against a threshold set at M3's observed value with no
  spread allowance.
- User decision (message 16): not accepted; the verdict remains NOT_REPRODUCED, unresolved,
  non-blocking for further investigation.

**7. Does R2 preserve constraint freshness?** **Yes.** Snapshot age p95:

| Condition | R2 | Reference |
|---|---|---|
| A_L0, A_L1, B_L0 | 11 ms in every run | M1 / M3 / M3b: 11 ms |
| C_L1 | 15–16 ms | M3b: 15–16 ms; one 20 ms rebuild period is the limit |

Joint-state ages are reported in `r2_analysis.json`.

**8. Are there any new safety or correctness concerns?**
- **None from R2 itself.**
- Concerns surfaced and recorded:
  - **F-8-56:** bit-identity is not guaranteed under OSQP's automatic ρ-adaptation. It matters for P2's
    acceptance design. A production fix is a QP-setting decision.
  - **F-8-57:** the E3-05 parameter file pins the executor. Earlier experiments are unaffected.
  - **F-8-58:** `--set` limitation.
  - **F-8-59:** test teardown race.
- Unchanged from before R2:
  - F-8-44, now reduced to the singularity-row path;
  - under E3-like host load at real time the filter is saturated (C_L1, Python threads 0.98 core), so
    hardware still needs the load-reduction work (F-8-52).
- Offline statistics are not real-time guarantees: no robot, no 1 kHz real joint states, no closed loop
  in these replays.

**9. Is R2 ready for Gazebo validation?** **Yes, conditionally.**
- Ready:
  - the configuration, tests and offline evidence are in place;
  - the static scenario can run with the existing H2 gates once a harness copy supports arm D (no
    `--set`, §8).
- Two conditions:
  - **(a)** the user's decision on the A_L1 p99 excess (§6.3). Decided in message 16: not accepted,
    verdict NOT_REPRODUCED kept, non-blocking for the Gazebo static phase;
  - **(b)** for the moving-obstacle scenario, the F-8-41 correction and addendum (§8.4) must be approved
    and sealed first.
- R2 must also be committed on its branch first (approval needed), so that Gazebo runs have a clean git
  provenance (rule R05 / R18).


## 8. STEP 8: minimal Gazebo validation plan (prepared, NOT executed; each item needs approval)

**What Gazebo must add to the offline evidence.** It must show the shipped default in the **closed
loop**, with the real plant, perception and commander:
- the QP rate in Gazebo;
- safety (clearance, contact, faults, fallbacks, reflex);
- tracking.

These are all things the bag replay cannot show. The claim stays relative: single against multi in
pairs, and the shipped default end to end.

### 8.1 Scenarios (one static, one moving)

| | Static | Moving |
|---|---|---|
| Scenario | **`pentagon_static_baseline_bare`**, condition `full`. The E3 cell: its H2 gates (R01–R22, V1–V7) are validated, and Gazebo runs of both modes exist for comparison: single E3-04 (a STOP, startup race), E3-06, E3-08, E3-09; multi E3-03, E3-05, E3-07, E3-10 | **`pentagon_wander_close`**, condition `full`. The ball tours the task circle at a median of 0.31 m/s and a maximum of 0.44 m/s (`scenarios.yaml` L332–357) |
| Why this one | The lowest-risk check that the default path works in closed loop; the F-8-41 correction is not needed (static obstacle) | Staleness matters most when v_obs ≠ 0 (V4 of the R1_E3 plan) |
| Alternative | V2 (`pentagon_static_baseline`, scene `diag`, about 2× rows) tests generalisation to higher load. Known risk: phantom obstacle velocities on furniture (memory lead, Unverified at this revision) | — |

**Recommendation.** Use `bare` for R2's minimal validation. Keep V2 for a later generalisation step.

### 8.2 Arms and design

- **Arm D (R2 default):** `--conditions full` **without** `--set`.
  - The `full` pin (`single`) reaches the stack through `cbf_param_overrides` (Verified, §4 test 2).
  - F-8-57: this tests the shipped value. A bare hardware launch has no override at all; offline that
    path was covered by the default arm of §6.
- **Arm M (rollback):** `--set executor_mode=multi`.
- **Design:** 2 seeds × (D, M), counterbalanced ABBA / BAAB per scenario, giving **8 Gazebo runs**
  (about 60 min).
- H2 phase-aligned birth (`--episode-start phase --start-phase 0.0`). For `wander_close`, the same seed
  within a pair gives the same jittered p0.

### 8.3 Gates

- The H2 gates **R01–R22 unchanged** for the static scenario. R05 expects `single` for arm D.
- **For the moving scenario,** the F-8-41 addendum (§8.4) first.
- Pre-registered acceptance, as in R2_READINESS §6.4:
  - arm D: QP rate R15 ≥ 95 Hz;
  - R12 no contact;
  - R14 clearance ≥ d_safe = 0.10 m with the conservative bound (§8.4 for moving);
  - R10 / R11 zero faults and fallbacks;
  - R08 reflex PASS;
  - tracking |e| p95 within ±5 mm of the pair partner.
- **Record:** `executor_mode` and the start-up executor line per cell, and the achieved Gazebo RTF
  (E3 ran at ≈ 0.63–0.70).
- **Expectation, from E3 and M3:**
  - D ≈ 99–100 ticks per sim s, M ≈ 67–89 at E3's RTF;
  - at an RTF near 1.0 under E3's host load, D is **not** expected to hold 100 Hz (M3b).

### 8.4 F-8-41: the validity-check correction needed before the moving-obstacle runs (identified, not implemented)

The E3 / H2 gate code (sealed, `refactor_r1_e3/scripts_h2/`) assumes a static obstacle in five
places (Verified, code).

| # | Where | Static assumption | Correction (to pre-register in an addendum) |
|---|---|---|---|
| 1 | `e3_h2_metrics.py` L40–46, check V4 (L210) | g_min = d_safe + v_ref·τ_budget = 0.1875 m ignores the obstacle's speed | g_min = d_safe + (v_ref + v_obs,max)·τ_budget. With v_obs,max = 0.44 m/s (the scenario's maximum, from `scenarios.yaml`, or the measured truth maximum, whichever is larger), that is 0.10 + 0.79 × 0.25 = **0.2975 m** |
| 2 | `s6b1_supplementary.py` L44–50 (R12 / R13 / R14 inputs) | The inter-sample dip bound uses the robot control-point speed only: `vmax·dt/2` | Use the relative speed (cp speed + |v_obs|max) |
| 3 | `s6b1_reconstruct_clearance.py`, causal pose pairing | Each joint sample is paired with the last obstacle pose. With the pose stream at up to 50 ms, the obstacle may have moved by |v_obs|·Δt_pose (F-8-13) | Subtract |v_obs|max·Δt_pose,max (0.44 × 0.05 = **22 mm**) in the conservative bound used by R12 / R14 |
| 4 | `e3_analyze.sh` R13 (L266–282) | Truth vs reconstruction must agree to `EXACT` | With a moving obstacle, the causal-pose reconstruction and the truth differ by up to |v_obs|·Δt_pose. R13 needs that as its tolerance, or it STOPs spuriously |
| 5 | `s6b1_reconstruct_clearance.py` L114–122 and `e3_run.sh` L51 / L57 / L252 | Scenario p0 `[0.55, -0.25, 0.449]` (diagnostics only), scenario, start phase and `--set executor_mode=$MODE` are hard-coded | Make scenario, phase, p0 and the arm (D = no `--set`) recorded arguments of a parameterised harness copy (`scripts_h3/`); drop or condition the p0 diagnostics for moving cells |

**Procedure.**
- A harness-only copy (`scripts_h3/`, the sealed `scripts_h2/` untouched).
- Unit tests of the corrected bounds against the sealed E3-05…10 data: on a static cell, the corrected
  gate must reproduce every sealed verdict, because v_obs = 0.
- A prospective addendum sealed before any moving run.

None of this touches production code. It needs its own approval.

*C-078 note (2026-11-20).* Implemented in the harness only (`refactor_r2/06_f841/scripts_h3/`) and
tested; no moving run was made. Three corrections to this table:
- **v_obs,max = 0.4464 m/s, not 0.44.** 0.44 m/s is the maximum sampled over one 40 s episode, while
  0.4464 m/s is the law's analytic supremum. g_min is therefore **0.2991 m**, not 0.2975.
- **Row 3 uses the per-sample pose age, not a fixed 22 mm.**
- **Two more static assumptions:**
  - `e3_h2_bag.py` refuses any non-static law (`SystemExit`), so every moving run would have
    GATE_STOPped;
  - `e3_h2_metrics.py` computes the closing speed at p0.

See [R2_GAZEBO_STATIC_AND_F841.md](R2_GAZEBO_STATIC_AND_F841.md) §6 and F-8-62.

### 8.5 Kept separate

- **F-8-44 / P2:** σ carried in `ConstraintSnap`; its own step, with T1–T7, after R2. R2 is not blocked
  by it (§5.4).
- The launch golden (F-8-1), Q-7-2, Q-2-2.
- P3 / P5 load reduction (needed for 100 Hz at RTF 1.0 under load).
- F-8-56: the OSQP ρ-interval, a QP-setting decision.
- F-8-58: `--set` for other conditions.

## 9. Rollback

- **Per run, no code change:**
  - `cbf_param_overrides:=executor_mode=multi`;
  - `--ros-args -p executor_mode:=multi`;
  - `--set executor_mode=multi` (Gazebo `full`).
- **Default:** discard or revert the R2 diff, then rebuild both packages (config is copied at build
  time).
  - The pre-R2 install is kept in `/ros2_ws/quarantine_r2_20261120/install/`.
  - Then re-run `test_cbf_executor_mode.py` and `test_conditions.py`.
- R2 touches no control code, so a revert restores the R1 behaviour exactly, up to F-8-56's solver
  timing.

## 10. Findings and consequences of modification

**New findings:**
- **F-8-56:** OSQP ρ-adaptation and replay determinism.
- **F-8-57:** the E3-05 parameter-file executor pin and the default arm.
- **F-8-58:** `--set executor_mode` only for `full`.
- **F-8-59:** executor teardown race in the new smoke v1.
- **F-8-60:** the R2 performance result, including the A_L1 p99 miss.

**Consequences:**
- **Changing the default again** means editing `fr3_control.yaml` and `conditions.yaml` together. The
  test `test_the_full_condition_runs_the_shipped_executor_and_multi_stays_selectable` fails otherwise.
- **Any test that spins executors on helper threads** must stop and join them before `shutdown()`
  (F-8-59).
- **Bit-identity acceptance** in later steps (P2) must be designed for F-8-56.
- **Offline replays of the shipped default** must strip the executor key from E3-05's parameter file
  (F-8-57). `r2_default_replay.py` does this and refuses otherwise.
- **Pre-registered thresholds** taken from a reference's observed value need a spread allowance (the
  C_L1 rule); A_L1's did not (§6.3).
