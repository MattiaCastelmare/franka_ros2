# R2 readiness: M3 finalised, triage of the package-test failures, M3b (single at real-time playback under load), R2 plan

> **Status (2026-11-20, C-076).** Approved by the user (message 14): STEP 1 to STEP 6.
>
> **Not done:**
> - R2 is not implemented; no executor default changed; no F-8-44 fix;
> - no production code changed;
> - no Gazebo or robot; no CPU-setting change; no retry;
> - nothing pushed or merged;
> - the M3b tools and these docs are **not committed** (needs approval).
>
> **Labels:** Verified / Derived / Inferred / Unverified / Runtime (recorded).
>
> **M3b evidence:** `~/franka_baseline_evidence/refactor_r1_m3b/`.
>
> | Directory / file | Content | Seal |
> |---|---|---|
> | `protocol/M3B_PROTOCOL.md` | The protocol | sealed 2026-11-20 13:06:20 UTC, SHA-256 `8d5206c1…`, before any run |
> | `scripts/` | The harness | sealed |
> | `preflight/` | Pre-flight checks | sealed |
> | `smoke/` | One smoke run, not counted | sealed |
> | `data/` | The 6 counted runs: `SHA256SUMS_raw_data` (67 files) and `SHA256SUMS_analysis`, read-only | sealed |

## 1. STEP 1: M3 finalised (Verified)

**Uncommitted changes, reviewed.** They were exactly:
- the M3 tools (`m3_load.sh`, `m3_host_sampler.py`, `m3_run.sh`, `m3_batch.sh`, `m3_analyze.py`),
  `test_m3_tools.py` and the tools README;
- the isolated F-8-44 test;
- the M3 report and the PROGRESS / chapter 16 / CHECKPOINTS updates.

**Byte-identity.** The M3 tools are byte-identical to the sealed `scripts_a01/` copies that ran
attempt 1. The F-8-44 test is byte-identical to its sealed copy in `02_f844/`.

**Production code.** No diff in the working tree outside `docs/framework_study`. The only non-doc
commits since R1 (`9bb2531`) are the two harness commits approved earlier:
- `73ed807` (`conditions.yaml`);
- `186b823` (H2, `franka_simulation_experiments` harness only).

No CBF, QP, perception or commander file changed.

**Evidence integrity.** Every M3 seal verifies:
- `00_regression_tests`;
- `01_design_probe`;
- `02_f844`;
- `attempt0`;
- `attempt1` raw (181 files) and analysis (13 files);
- `protocol`, `scripts`, `scripts_a01`, `smoke`.

One residue: the `02_f844.SHA256SUMS` file was still writable, so it was made read-only (its content
is unchanged).

**The laptop suspend.**
- It is documented as F-8-51 and in R1_M3 §4.
- Sensitivity (`attempt1/analysis/m3_sensitivity.txt`): without m3_18, MT(3) under L1 is 76.5 ticks
  per sim s instead of 80.0 (68.7 / 84.4 / 86.8 → 68.7 / 84.4). Every M3 verdict is unchanged:
  - H1 is unaffected (single only);
  - H2: the mean order is kept and it is still not separated;
  - H3 / H5: the m3_18 values lie inside the other two MT(3) runs;
  - H4 is still reproduced.
- **Cause (new, Inferred from `journalctl`):**
  - there is no idle suspend on AC (`sleep-inactive-ac-type 'nothing'`) and no lid event;
  - a USB volume was unmounted at 17:08:20 local time, 4 s before "The system will suspend now!";
  - most likely the suspend was requested from the desktop.

**Commits** (local, the user as the only author, no trailer, not pushed):
- `8566d28` tools: M3 harness, analysis and tests; the isolated F-8-44 test;
- `c21a570` docs: the M3 results, the regression suites, the F-8-44 tests and the R2 decision
  report, findings F-8-48…F-8-51 (C-075).

## 2. STEP 2: the 35 failures of the main package suite

**Baseline comparison (Runtime, `refactor_r1_m3/00_regression_tests/comparison_with_R1_H2.txt`).**
- The failing set (35 IDs), the skipped set (9) and all 65 assertion lines are identical to the R1
  baseline and to S5 (`baseline_f708991/03_tests/suite1_failures_and_skips_classified.txt`).
- No failure is new; no pass is lost.

**Classification**, re-examined in this step against the code, the shipped configuration and git:

| Group | n | Tests | Cause (evidence) | Class | Safety relevance | Blocks R2? |
|---|---|---|---|---|---|---|
| A1 | 19 | `test_cbf_static_scene` (5), `test_cbf_surface_velocity` (14) | Specification tests for the static-scene hold and the surface-velocity cap. `utils/static_scene.py` and `utils/surface_velocity.py` are unwired (commit `2402e58`, "not yet wired"; F-0-3). The test harness's `P` has no `enable_static_scene_hold`, `enable_static_scene_voxels`, `enable_surface_velocity_cap` or `obstacle_velocity_floor_min_frames`. **No shipped config, launch or node sets these keys** (grep, Verified). | **Test infrastructure:** specification tests merged ahead of an unimplemented feature | None for shipped behaviour (the features do not exist, so nothing ships them) | No |
| A2 | 6 | `test_self_reject_and_obstacle_floor` | Specifications for `DistanceEngine` self-rejection and a base-frame obstacle floor (`last_self_reject_px`, `last_floor_drop_px`, `_self_reject_tol`, `obstacle_min_z_m` are absent). The spec itself says both default to inert. | **Test infrastructure** (unimplemented specification) | Adjacent to the open perception-safety item F-8-21 (self-mask blind zone). The failing tests describe a feature that does not exist, not a broken one | No |
| B | 1 | `test_node_self_references::test_every_node_self_call_resolves` | The static checker flags `HandEyeEyeInHandNode._parameters` (L200) as undefined. It is rclpy's own private attribute, initialised in `rclpy.node.Node.__init__` (`self._parameters: dict = {}`, Verified in the container's Humble rclpy), so the node works. Fix `5731c47` was reverted by `e9ff7cd` (F-7-9). | **Test infrastructure:** checker false positive. Plus a code-quality note: the node depends on a private API that a future rclpy may break | None: the hand-eye calibration node is not in the control pipeline | No |
| C | 1 | `test_rate_scaling::test_the_shipped_config_states_durations` | Expects `tracking.max_coast_s == 0.167`; the shipped `fr3_complete.yaml` was deliberately changed to 0.8 on 2026-11-01 (the YAML comment records it; F-7-7) | **Test infrastructure:** outdated expectation | The test is outdated. The related open question **Q-2-2** (was `gate_max_m` re-derived after the coast change?) is a **potential correctness issue in the tracker, Unresolved**, independent of the executor | No |
| D | 8 | `test_torque_stack_external_plant_off_identical` (all) | Stale launch golden. Exactly two leaves differ per variant, both from deliberate launch changes: the `fr3_complete.yaml` temp-copy content hash, and the new `robot_config_path` parameter (`25be305`; F-8-1) | **Test infrastructure:** stale golden | Indirect: the torque-stack launch has **no green regression guard**, so another launch change would hide among the 8 failures | No (see below) |

- **Totals:** 35 test-infrastructure failures.
  - 25 are specification tests for features that do not exist;
  - 1 is a checker false positive;
  - 1 is an outdated expectation;
  - 8 come from a stale golden.
- **Other categories:**
  - pre-existing functional defects shown by these failures: 0;
  - environment or dependency failures: 0 (the 9 *skips* are environment: no `franka_sim` checkout
    in this layout);
  - direct safety or correctness failures: 0;
  - adjacent open items: Q-2-2, F-8-21.
- **Decision left to the user:** whether to implement or retire the A1 / A2 specifications (Q-7-2).

**Does any failure block R2? No.**

1. None touches the executor, the QP, the CBF rows or `cbf_safety_filter`'s spin path. The 22
   executor-mode tests and the 123 / 69 / 108 CBF, QP and constraint tests pass.
2. **Does the stale golden (D) hide an R2 change?** No (Derived, static).
   - The node loads `fr3_control.yaml` itself from the package share (`utils/config.py`
     `load_cbf_config` L777–801, which reads `config/fr3_control.yaml` from
     `get_package_share_directory`, then calls `declare_from_spec`).
   - The launch refers to it only by path (`torque_control_stack.launch.py` L546), and
     `_launch_snapshot._TMP` content-hashes only `/tmp/franka_rt_blender_*.yaml` and
     `fr3_complete_launch_*.yaml`.
   - So the snapshot is identical before and after R2's one-line edit, and D neither masks nor is
     changed by R2.
3. **Residual risk:** suite 1 can only be judged as "the failing set equals these 35 IDs" (F-8-2).
   R2's acceptance keeps that rule.
4. **Recommended as a separate step (not part of R2):** regenerate the launch golden with a reviewed
   diff (F-8-1), so the torque-stack launch regains a green guard. This updates an expected result,
   so it needs its own approval and a diff review showing that only the two known leaves change.

## 3. STEP 3: M3b design (sealed `protocol/M3B_PROTOCOL.md`; summary)

**Condition.**
- `SingleThreadedExecutor` via the production `make_executor` (`-p executor_mode:=single`).
- **Playback rate 1.0.**
- E3-05's bag and three parameter files, all hash-identical (§4).
- CBF and QP parameters unchanged.
- Window sim [t0+2, t0+30] s.

**Loads (M3's, unchanged):**
- **L0:** idle host.
- **L1:** 12 Python spin workers, calibrated to E3's measured non-filter host load.
- Synthetic: no memory, iGPU or DDS pressure.

**Order and repetitions.** 3 runs per load, in the order L0 L1 L1 L0 L0 L1 (M3's block loads), plus
one smoke run that is not counted.

**Harness.** `m3b_run.sh` / `m3b_batch.sh` are M3's scripts with only four changes:
- rate 1.0;
- a fresh container workdir;
- arm fixed to single;
- suspend detection and an inhibitor check.

`test_m3b_tools.py::TestScripts` pins this diff. `m3b_analyze.py` reuses `m1_analyze.py` and the
helpers of `m3_analyze.py`, all unchanged.

**Hypotheses:**
- **Hb1:** L0 PASS.
- **Hb2 (primary):** L1 PASS. The recorded prediction from M3's linear projection (saturation at
  0.83) was *not supported*.
- **Hb3:** saturation evidence under L1.
- **Hb4:** demand scaling, descriptive.

**Validity:** M1's checks with the rate gate re-centred to [0.90, 1.10], executor single, M3's load
gates, sampler coverage, no suspend, inhibitor held.

**Acceptance:** A1 (rate) to A4 (freshness) decide the verdict; A5 (margin) and the saturation
indicator are reported alongside it.

| Code | Criterion |
|---|---|
| A1 rate | Every run ≥ 95 ticks per sim s and cell mean ≥ 99 |
| A2 deadlines | Missed 10 ms slots ≤ 28 and 0 gaps ≥ 50 ms |
| A3 jitter | Gap p99 ≤ 15 ms |
| A4 freshness | Snapshot age p95 ≤ 20 ms |
| A5 margin (Derived, reported) | Projected saturation ≥ 1.10 |
| Saturation indicator | Python callback threads ≥ 0.95 core |

| Verdict | Condition |
|---|---|
| PASS | A1–A4 hold |
| MARGINAL | Every run ≥ 95, but PASS fails |
| FAIL | Any run < 95 |
| INCONCLUSIVE | Fewer than 2 valid runs |

**Abort conditions:**
- a pre-check fails;
- the inhibitor is missing or lost;
- a suspend is detected;
- the thermal guard fires;
- load workers survive cleanup;
- the governor or EPP changed;
- harness failure.

**No retry.**

**Tool tests** (written and run before sealing): `test_m3b_tools.py` 17 / 17, 0 warnings.
- Includes an integration test on the sealed M3 run m3_05: M3b's metrics reproduce M3's to 1e-9, and
  the re-centred gate rejects a 0.70 run with exit code 10, not 1.
- **Pre-data correction:** the test's own whitelist missed one echo line. It was fixed before sealing.
- **Mutation check:** a wrong rate gate, a wrong missed-slot threshold or a disabled suspend tolerance
  each make the tests fail (3, 2 and 1 failures).

## 4. STEP 4: execution (Runtime, recorded)

**Pre-flight** (`preflight/`, sealed):

| Check | Result |
|---|---|
| Repository | HEAD `c21a570`, 0 production diff lines, 3 stashes untouched |
| Install vs source (container) | Every `.py`, YAML and launch file of `franka_experiments` is identical. The only difference outside `__pycache__` is `config/safety/watchman_profile.md`, a note the package does not install |
| Inputs | Container workdir replay scripts = sealed M1 scripts (`1a690708…`, `eb053c83…`); parameter files = E3-05 `container_tmp` (`1ea07454…`, `5e18b7bf…`, `6162eadc…`); bag `8a7ef1e8…` |
| CPU | `powersave` × 28, EPP `performance` × 28 (not changed) |
| Processes | 0 ROS / Gazebo processes (host and container); 0 load workers |
| Host | 58 °C; on AC power |
| Inhibitor | Probe lock granted |

**Suspend inhibition (experiment-scoped, reversible):**
- The batch ran under
  `systemd-inhibit --what=sleep:idle:handle-lid-switch --mode=block --who=M3B_BATCH` (self
  re-exec), and so did the smoke run.
- Every run recorded `inhibitor_held 1` before and after.
- `CLOCK_BOOTTIME − CLOCK_MONOTONIC` = 71505.811 s, constant across all 6 runs, so no suspend.
- After the batch: 0 inhibitor locks, 0 load workers, 0 container ROS processes.

**Smoke run** (single / L1 / rate 1.0, not counted): valid; the harness worked end to end. No
addendum was needed.

**Batch:**
- 2026-11-20 13:08:56 → 13:21:20 UTC; **6 / 6 runs OK and valid**; no INVALID, abort or retry.
- Effective playback rate 1.000 in every run.
- Workload vs E3-05: rows p50 +1…+2; \|Δrows\| ≤ 2 on 97.0–98.1 % of matched ticks.
- Host load: L0 0.94–1.22 busy cores; L1 13.66–14.54, with 12 workers alive at the start and end.
- At most 69 °C.
- 0 stale or brake warnings in the window.

## 5. STEP 5: results and answers

### 5.1 Measured (Runtime). Mean [min, max] of 3 valid runs, next to M3's single arm at rate 0.70 (Runtime, sealed M3)

**QP rate and timing:**

| Metric | **L0, rate 1.0** | L0, 0.70 (M3) | **L1, rate 1.0** | L1, 0.70 (M3) |
|---|---|---|---|---|
| QP ticks per sim s | **99.99** [99.96, 100.00] | 100.00 | **89.44** [86.36, 92.21] | 99.94 |
| QP ticks per wall s | 99.99 | 70.0 | 89.49 | 69.97 |
| Missed 10 ms deadlines | 0.3 [0, 1] | 0 | **319** [239, 405] | 2 |
| Late ticks (> 15 ms) | 1 | 0 | 517 | 6 |
| Tick gap p95 / p99 / max (ms) | 11 / 11 / 18 | 11 / 11 / 13 | **19.3 / 23.0 / 38.7** | 11 / 12 / 21 |
| Gaps ≥ 50 ms (sum over runs) | 0 | 0 | 1 (m3b_03) | 0 |

**Per-operation cost:**

| Metric | **L0, rate 1.0** | L0, 0.70 (M3) | **L1, rate 1.0** | L1, 0.70 (M3) |
|---|---|---|---|---|
| Rebuild wall p50 / p95 (ms) | 2.56 / 3.66 | 2.40 / 3.29 | 8.83 / 12.72 | 8.95 / 11.35 |
| Rebuild CPU p50 (ms) | 2.57 | 2.41 | 8.84 | 8.95 |
| Rebuilds per sim s | 50.0 | 50.0 | 49.86 | 50.0 |
| QP solve wall p50 / p95 (ms) | 0.37 / 0.60 | 0.34 / 0.52 | 0.99 / 1.88 | 0.98 / 1.64 |
| Tick CPU p50 (ms) | 0.95 | 0.89 | 2.76 | 2.91 |

**Freshness and inputs:**

| Metric | **L0, rate 1.0** | L0, 0.70 (M3) | **L1, rate 1.0** | L1, 0.70 (M3) |
|---|---|---|---|---|
| Snapshot age p50 / p95 (ms) | 2 / 11 | 3.5 / 11 | 0 / **15.7** | 1.3 / 11 |
| Joint states / `/clock` processed | 0.78 / 0.67 | 0.86 / 0.73 | **0.26 / 0.24** | 0.48 / 0.43 |
| Joint-state age p95 (ms) | 7.8 | 6.6 | 19.2 | 12.1 |

**CPU, host and headroom:**

| Metric | **L0, rate 1.0** | L0, 0.70 (M3) | **L1, rate 1.0** | L1, 0.70 (M3) |
|---|---|---|---|---|
| Process CPU (% of a core) | 65.1 | 45.7 | **98.9** | 87.8 |
| Python callback threads (cores) | 0.59 | 0.41 | **0.95** [0.92, 0.96] | 0.85 |
| CPU per sim s: callbacks / loop / remainder (ms) | 276 / 204 / 172 | 259 / 220 / 174 | 726 / 115 / 148 | 807 / 241 / 206 |
| Effective playback rate | 1.000 | 0.700 | 1.000 | 0.700 |
| Host busy cores | 1.10 | 0.61 | 14.13 | 13.10 |
| P-core p50 (MHz) | 800 (idle cores) | 800 | 2181 | 2535 |

**Late ticks by `n_rows` tercile, L1 at rate 1.0** (low / mid / high):
- m3b_02: 0.09 / 0.22 / 0.32;
- m3b_03: 0.15 / 0.25 / 0.40;
- m3b_06: 0.11 / 0.13 / 0.19.

**Verdicts (§8 of the protocol):**

| Load | Verdict | Detail |
|---|---|---|
| **L0** | **PASS** | A1–A4 in every run; **with margin** (A5, projected saturation 1.71); no saturation |
| **L1** | **FAIL** | No run ≥ 95. A2 and A3 fail in every run; **A4 (freshness) holds in every run**. No margin. Saturation evidence in 2 of 3 runs (0.96, 0.96; the third run 0.92) |

**Hypotheses:**
- **Hb1:** supported.
- **Hb2:** **not supported**, as M3's projection predicted.
- **Hb3:** supported.
- **Hb4:** Python-thread demand ratio (rate 1.0 vs 0.70):
  - L0: 1.425, i.e. linear (1.43);
  - L1: **1.12**, i.e. sublinear: demand is capped near one core and inputs are coalesced.

### 5.2 Answers

1. **Does single hold about 100 Hz at playback rate 1.0? On an idle host, yes** (Runtime):
   - 99.96–100.00 ticks per sim s, which at rate 1.0 is also ticks per wall second;
   - ≤ 1 missed deadline in 2800;
   - gap p99 11 ms;
   - Python threads 0.59 core.
2. **Under E3-like host load? No** (Runtime).
   - 89.4 ticks per sim s on average [86.4, 92.2]: about 10.6 % of the QP ticks are lost.
   - Every run is below E3's R15 gate (95 Hz).
   - Single at 0.70 under the same load held 99.94 (M3).
   - For scale (different rates, so not a like-for-like comparison): MT(3) at 0.70 under L1 gave
     80.0 and MT(1) 89.4.
3. **Missed deadlines:**
   - idle: 0–1 of 2800 slots;
   - under load: 239–405 (8.5–14.5 %), 517 ticks later than 15 ms;
   - one gap ≥ 50 ms in 3 runs (max 54 ms in m3b_03).
4. **Freshness: the constraints stay fresh even under load.**
   - Snapshot age p95 is 15–16 ms, below one rebuild period (20 ms).
   - The rebuild timer keeps 49.7–50.0 rebuilds per sim s.
   - What degrades is the **QP tick rate**, the **input processing** (26 % of joint states and 24 %
     of `/clock` messages handled, against 48 % and 43 % at 0.70) and the joint-state age (p95
     19 ms).
5. **Computational saturation: yes, under load** (measured; mechanism Inferred).
   - The Python callback threads use 0.92–0.96 core and the process 96–101 % of one core.
   - Per sim second, all callbacks together cost 0.66–0.77 s of CPU in a serial executor: rebuild
     57–59 % of that, QP tick 36–37 %.
   - Under load each operation costs about 3.4× its idle CPU, as in M3.
   - Demand scales sublinearly (Hb4), because the executor sheds input events.
   - **In a 20 ms period**, one rebuild (8.8 ms p50, 12.7 ms p95) plus two ticks (2 × 2.8 ms) plus
     the event handling leave no slack. Because the executor is serial, a long rebuild delays the
     next tick. This is consistent with the late fraction rising with `n_rows` (rebuild cost grows
     with the rows).
   - Interpreter serialisation is Inferred; the GIL was not instrumented.
6. **Remaining performance margin (Derived):**
   - **Idle:** projected saturation at 1.71× real time (1.0 / 0.585 core); measured linear scaling
     from 0.70 to 1.0 supports it. That is about 40 % spare interpreter capacity.
   - **Under E3-like load: none.** The threshold lies between rate 0.70 (held) and 1.0 (lost).
     M3's linear projection put it at 0.83, but the sublinear scaling measured here means that
     figure is only approximate.
   - The A5 "projected 1.056" value under load is not a usable margin, because the threads are
     already saturated at 0.95 core.
7. **Does it support proceeding with R2? Yes, for R2 as a relative improvement; no, for any claim
   that R2 achieves real-time 100 Hz under load.**
   - Under every measured condition, single is better than or equal to multi (M1, M3) and holds
     100 Hz at real-time playback on an idle host (M3b).
   - Under E3-like load, M3b shows that **the default change alone does not deliver 100 Hz at real
     time**. Multi was not measured at rate 1.0. That multi would be worse there is Inferred from
     M3, where it lost rate already at 0.70.
   - R2 should therefore proceed as a configuration step with a correctly scoped claim (§6). Reaching
     100 Hz at real time under load needs separate work:
     - P3 (OSQP `Ax` fill);
     - P5 / F-8-43 (event load, about 60 % of the CPU);
     - rebuild cost;
     - process isolation or CPU affinity on the target machine (each a separate approval).
8. **Limits for the real robot (all Unverified):**
   - **Inputs:** the robot has no `/clock` (the 1 kHz `/clock` callback is a large part of the
     "remainder") and real 1 kHz `franka_ros2` joint states. Hardware timing is not the same as bag
     playback.
   - **Other load:** the other pipeline nodes (perception, commander, ISO layer) share the CPU, and
     DDS and network jitter add to it. This host's load differs from Gazebo's (no iGPU or memory
     pressure).
   - **Machine:** a different CPU, governor and kernel. rclpy and Python give **no real-time
     guarantee**: the SCHED_FIFO per-thread option is off, and `single` refuses it.
   - **Loop:** the replay is open-loop, so there is no closed-loop effect of a late tick on tracking
     or clearance.
   - **What these results are:** offline throughput and latency statistics on one recorded scene.
     They are not worst-case execution-time bounds.

## 6. STEP 6: R2 recommendation (proposal; not implemented)

### 6.1 Exact config-only change set

| # | File | Change | Why |
|---|---|---|---|
| 1 | `franka_experiments/config/fr3_control.yaml` L317 | `executor_mode: multi` → `executor_mode: single` | The shipped default (read by `cbf_safety_filter.main` through `P.executor_mode`) |
| 2 | `franka_experiments/config/fr3_control.yaml` L305–316 (comment) | Update the text: `single` is the default from R2, with references to R1_M1 / R1_M3 / M3b, and how to select `multi` (`cbf_param_overrides:=executor_mode=multi`) | Documentation must not contradict the value |
| 3 | `franka_simulation_experiments/config/conditions.yaml` L109 (`full.cbf_params.executor_mode`) | `multi` → `single`. **Keep the key**: `conditions.resolve` lets `--set` override only keys already present (L106–109), and E3 needs `--set executor_mode=multi` for the multi arm | The `full` condition is documented as "exactly as a bare hardware launch runs it". Without this, every Gazebo `full` cell stays `multi` after R2 |

No other file sets `executor_mode` (grep over `*.py`, `*.yaml`, `*.xml` outside `docs/`, Verified).
- No launch file passes it.
- The parameter spec (`utils/config.py` L733) has no default of its own.
- `qp_thread_fifo_prio` is 0 in the only YAML that sets it, so `single`'s refusal of a FIFO
  priority (`make_executor` L2103) cannot trigger with shipped settings.

### 6.2 Existing tests affected

| Test | Effect of R2 | Required action (no weakening) |
|---|---|---|
| `franka_experiments/test/test_cbf_executor_mode.py::test_shipped_default_is_multi` | **Fails** (asserts `'multi'`) | Rename to `test_shipped_default_is_single` and assert `'single'`. The module docstring says "default unchanged / the shipped default is multi": update it, citing R2 |
| `…::test_main_default_spins_the_three_thread_executor` | Passes (it passes `'multi'` explicitly), but its name says "default" | Rename to `test_main_multi_spins_the_three_thread_executor`; assertion unchanged |
| `franka_simulation_experiments/test/test_phase_start.py::test_batch_forwards_episode_start_only_when_set` | Unaffected (`sets={'executor_mode': 'multi'}` is plumbing only) | None |
| `test_torque_stack_external_plant_off_identical` (8, already failing) | Unchanged (§2 point 2) | None in R2 |
| `smoke_cbf_safety_filter.py` | Unaffected: it builds its own `MultiThreadedExecutor(num_threads=4)` and does **not** exercise the shipped executor | Add (new test, below) |
| Every other suite 1, 2 and 3 test | No dependence on the default (grep) | Judge by failing set = the 35 IDs |

### 6.3 Additional offline regression tests (new; added, never weakening)

1. **T-R2-1:** `fr3_control.yaml` `executor_mode` equals `conditions.yaml` `full.cbf_params.executor_mode`
   (suite 2). The sim baseline cannot silently diverge from the shipped default again.
2. **T-R2-2:** `main()` with the shipped YAML builds a `SingleThreadedExecutor` (monkeypatched spin,
   as `_run_main`; no ROS graph).
3. **T-R2-3:** a smoke variant of `smoke_cbf_safety_filter.py` that spins the node with
   `make_executor(P.executor_mode)` (single). The existing 4-thread smoke run stays.
4. **Sequential identity:** `r1_sequential_outputs.py` on the M1 bag, old vs new install: every
   published `qddot_safe` bit-identical. Expected trivially, since executor-free; it guards against
   accidental edits.
5. **Default-build replay** (after a `colcon build`, because config is copied, not symlinked):
   - **M3 harness, rate 0.70, L0 and L1, 3 runs each, without `-p executor_mode`.** Acceptance (R1_M3
     §7.3 item 7): every run ≥ 99.0 ticks per sim s, missed slots ≤ 10, gap p99 ≤ 12 ms, snapshot
     age p95 ≤ 11 ms.
   - **M3b harness, rate 1.0, L0 and L1, 3 runs each, without `-p executor_mode`.**
     - L0: the M3b §8 verdict PASS.
     - L1: **an equivalence check, not an absolute target.** M3b shows that single does not reach
       100 Hz there (§5). Each run's ticks per sim s and missed slots must fall inside the M3b L1
       range widened by its spread: ticks ≥ 86.4 − 2 × 3.0 = 80.4; missed ≤ 405 + 2 × 83 = 571.
       The run must also be `SingleThreadedExecutor` by default.
     - Its purpose is to prove that the default build *is* the configuration M3b measured.
   - The M3 / M3b harnesses run `m1_executor_replay.py`, which passes `-p executor_mode:=<arm>`
     explicitly. The default-build check therefore needs a new, separately approved arm `default`
     (no `-p`), which also verifies T-R2-2 end to end.

### 6.4 Mandatory Gazebo validation (before merge; each run needs its own approval)

0. Prerequisite: the F-8-41 gate addendum (v_obs in g_min; |v_obs| × 50 ms in the conservative
   clearance bound).
1. Counterbalanced single / multi pairs, ≥ 3 seeds each:
   - **V2** (furnished static scene, `diag`);
   - **V4** (`pentagon_wander_close`, moving obstacle).

   Use H2 phase-aligned obstacle birth and the E3 gates:
   - R15 ≥ 95 Hz (single);
   - R14 clearance ≥ 0.10 m (with the F-8-41 bound);
   - R12 no contact;
   - R10 / R11 zero faults and fallbacks;
   - R08 reflex PASS;
   - tracking |e| p95 within ±5 mm of the pair partner.
2. Record `executor_mode` and RTF per cell. Report the Gazebo RTF achieved (E3 ran at ≈ 0.63–0.70).

### 6.5 Rollback

- **Per run (no code change):** `cbf_param_overrides:=executor_mode=multi` (torque stack),
  `--ros-args -p executor_mode:=multi` (`ros2 run`), or `--set executor_mode=multi` (Gazebo harness).
  R1 keeps both modes; `multi` is the historical MT(3).
- **Default:** `git revert <R2 commit>`, then `colcon build --packages-select franka_experiments
  franka_simulation_experiments` (config is copied at build time), then re-run
  `test_cbf_executor_mode.py`.
- R2 touches no control code, so a revert restores bit-identical behaviour (R1 §8–§9 identity).

### 6.6 Kept separate from R2

- P2, the F-8-44 fix (σ in `ConstraintSnap`): its own step, with T1–T7 and bit-identity under
  `single`.
- The launch-golden regeneration (F-8-1).
- The A1 / A2 specification decision (Q-7-2).
- Q-2-2.
- P3 (OSQP `Ax`).
- P5 (event load).
- F-8-45 / F-8-46.
- FIFO and priority changes.
- The campaign suspend inhibitor (now used by M3b only).

### 6.7 Do F-8-44 or the package-test failures block R2?

- **F-8-44:** no. R2 *reduces* it: under `single` the normal-path mismatch cannot occur (T2). Only the
  singularity-row exception path remains, and it exists in both modes today.
  - The effect is on the governor's nominal shaping, bounded and ramp-limited in the permissive
    direction, never on the constraints (T5).
  - Fix it in P2 separately so that R2 stays a pure configuration change with a clean bit-identity
    argument.
- **The 35 failures:** no (§2).

## 7. Findings and consequences of modification

**New findings** (PROGRESS §4, chapter 16):
- **F-8-52:** the M3b result.
- **F-8-53:** the Gazebo `full` condition pins `executor_mode: multi`, so a default change in
  `fr3_control.yaml` alone would not reach the `full` cells.
- **F-8-54:** the safety-filter smoke test spins its own 4-thread executor, so the shipped executor
  is not exercised by any smoke test.
- **F-8-55:** the triage of the 35 package-test failures (no new defect). It adds the R2 analysis of
  the launch golden.
- **Note added to F-8-51:** the suspend cause (Inferred, desktop-initiated).

**Consequences:**
- **R2 must change three places, not one:** the `fr3_control.yaml` value, its comment, and the
  `conditions.yaml` `full` pin. It must also update two tests, by renaming and changing an
  assertion, never by weakening.
- **The claim of R2 is relative:** better rate, freshness and CPU than multi, and 100 Hz at real time
  on an idle host. Under E3-like load at real time, single still loses about 10 % of the QP ticks
  (M3b), so any hardware deployment needs the load-reduction work in §5.2 item 7 or a less loaded
  machine.
- **The suspend inhibitor** in `m3b_batch.sh` is the pattern for any future campaign harness on this
  laptop.
