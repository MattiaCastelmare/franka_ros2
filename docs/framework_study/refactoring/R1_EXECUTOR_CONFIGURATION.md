# R1: Configurable CBF executor

| | |
|---|---|
| Step | R1, the first production change of the refactoring phase |
| Approved | by the user, 2026-11-12, for this step only (no Gazebo, no R2) |
| Branch | `refactor/cbf-executor-scheduling`, created at `625c179` (= `humble-mattia` = `origin/humble-mattia`) |
| State | implemented and tested, **uncommitted**. `humble-mattia` is untouched. |
| Documented source baseline | unchanged, `ba68077` (`PROGRESS.md` §8). The chapters still describe the baseline, and the R1 notes in ch. 02, 03 and 16 are dated. |
| Evidence | `~/franka_baseline_evidence/refactor_r1_executor/` (outside Git, write-protected, `SHA256SUMS`) |
| Motivating analysis | [QP_PERFORMANCE_ROOT_CAUSE.md](../validation/00_baseline/QP_PERFORMANCE_ROOT_CAUSE.md) §7; F-8-14, F-8-15 |

Labels follow [00_INDEX.md](../00_INDEX.md) §2: Verified, Derived, Inferred, Unverified,
Runtime (recorded).

## Summary

| Metric (S6B bag replay, 5 runs per mode and condition; mean [min..max]) | Original executor (`multi`, MT3) | Single-threaded executor (`single`) |
|---|---|---|
| QP ticks per sim s, RTF 0.62, unpinned | 100 [99.9..100] | 100 [100..100] |
| QP ticks per sim s, RTF 1.0, E-cores | 94 [73.9..99.1] | 100 [100..100] |
| Late ticks (sim gap ≥ 15 ms), of ~3000; RTF 0.62 / E-cores | 140 [86..205] / 635 [613..661] | 0 [0..0] / 4 [0..9] |
| Missed 10 ms slots; RTF 0.62 / E-cores | 19 [2..45] / 558 [394..1120] | 0 / 0.6 [0..3] |
| Maximum tick gap, ms; RTF 0.62 / E-cores | 22.6 [20..26] / 63.6 [34..144] | 13.0 [13..13] / 16.8 [14..19] |
| Process CPU, % of one core; RTF 0.62 / E-cores | 67.4 [63.6..72.5] / 104 [103..107] | 44.2 [39.7..49.6] / 86.6 [84.5..88.5] |
| Command deviation from the schedule-ideal sequential reference, p95 / max, rad/s² (RTF 0.62) | 0.414 / 0.88 | 0.055 / 0.50 |
| Command difference between the modes, p95 (noise floors: MT3 vs MT3 0.0046, single vs single 0.0135) | 0.419 rad/s² (RTF 0.62), 0.437 (E-cores) | ← same |
| Control equations | — | **bit-identical** to the baseline install in a deterministic replay (4068 / 4068 rows) |

All rows are Runtime (recorded, isolated benchmark). The replay is not a closed loop: it is not
an end-to-end result (§10, §11).

**Answers to the five questions of the R1 request:**

- **Correctly implemented:** yes, to the extent tested here.
- **All new tests pass:** yes, 22/22.
- **Default unchanged:** yes.
- **Better offline performance:** yes, in the replay.
- **Ready for Gazebo validation:** yes, as an A/B with an explicit override. It is not ready to
  become the default (§10).

## 1. Files changed

Production, on the R1 branch (`git diff --stat`: 3 files, +73 / −4; patch
`01_build/production_diff_final.patch`):

| File | Change |
|---|---|
| `franka_experiments/franka_experiments/nodes/cbf_safety_filter.py` | Imports `SingleThreadedExecutor` (L51). New `EXECUTOR_MODES`, `MULTI_EXECUTOR_THREADS = 3` and `make_executor(mode, qp_thread_fifo_prio)` (L2070–2112). `main()` (L2115) builds its executor through `make_executor` from the node's `executor_mode` parameter, logs the choice, and on refusal logs the error, destroys the node, shuts rclpy down and re-raises. |
| `franka_experiments/franka_experiments/utils/config.py` | One entry in `CBF_PARAM_SPEC` (L733): `'executor_mode': ('str', dict(choices=('multi', 'single')))` |
| `franka_experiments/config/fr3_control.yaml` | One key in `params:` (L317): `executor_mode: multi`, with a comment (L305–316) |

New test: `franka_experiments/test/test_cbf_executor_mode.py` (22 tests, §7).

Documentation (this step):
- this file;
- dated notes in ch. 02 (§2 and §9), ch. 03 (§3.3, §5.2) and ch. 16 (F-8-14, F-8-17, F-8-18);
- `PROGRESS.md` (§0, §4) and `CHECKPOINTS.md` (C-063);
- `CLAUDE.md` (the refactoring exception);
- four new scripts in `tools/baseline_validation/` (`r1_*.py`) and its README.

**Not changed (Verified, `git diff`):**
- launch files, `setup.py`, `package.xml`, the harness and `conditions.yaml`;
- every other node;
- the CBF, QP, slack, geometry, perception, margins, limits, gains, nominal command and fallback
  code;
- every other configuration value.

## 2. Motivation

F-8-14 (Runtime, isolated benchmark; GIL by elimination): the filter's `MultiThreadedExecutor(3)`
does not let the QP tick run independently of the 50 Hz constraint rebuild.
- All callback work is Python, Pinocchio or small NumPy, which hold the GIL. The three workers
  therefore serialise.
- A tick that falls due during a rebuild waits for it.
- In the QP-performance replay, a harness-only `SingleThreadedExecutor` removed the late ticks and
  cut process CPU by 27–37 %.
- F-8-15: most of the CPU is executor overhead, not callback work.

R1 makes that executor selectable without changing the default, so that a Gazebo A/B (E3) can test
it under the real launch. It is the minimal change: the callback groups are left as they are.
- `SingleThreadedExecutor` ignores callback groups.
- MT3 keeps using them exactly as before.

## 3. Implementation

- **Parameter.** `executor_mode` is a normal CBF parameter.
  - Its value lives in `fr3_control.yaml → params:`, as every other CBF value does.
  - Its kind and choices live in `CBF_PARAM_SPEC`.
  - `declare_from_spec` (`utils/params.py`) declares it as a ROS parameter, with the YAML value as
    the default (Verified, the same path as the other 260 spec keys).
- **Validation at declaration.**
  - `declare_str` raises `ValueError` and logs `parameter "executor_mode": invalid value ... —
    expected one of [...]` for any value other than `multi` or `single`. The comparison is
    case-sensitive and does not strip whitespace (Verified by the tests: `Multi`, `SINGLE`, `''`
    and `'multi '` are refused).
  - A YAML without the key raises `KeyError` (Verified).
  - The node never constructs in either case.
- **`make_executor(mode, qp_thread_fifo_prio=0)`**, Verified:
  - `multi` → `MultiThreadedExecutor(num_threads=3)`, the historical object.
  - `single` → `SingleThreadedExecutor()`.
  - Anything else → `ValueError`. This guards callers that bypass the parameter layer.
  - `single` with `qp_thread_fifo_prio > 0` → `ValueError`. `_elevate_thread_priority` raises the
    priority of the thread that runs `_qp_tick`. In single mode that thread is the whole node, the
    configuration behind the GIL inversion noted on 2026-11-07 (ch. 02 §2). The default
    `qp_thread_fifo_prio` is 0, so this applies only on an explicit override.
- **`main()`** logs `executor_mode=<mode> (<ExecutorClass>)` at start-up. Otherwise its sequence is
  unchanged: construct, add, spin, destroy, shut down.
- **Reachability** (Verified, AST test): `executor_mode` is read only by
  `cbf_safety_filter.py::main`. No control code can see it.

## 4. How to select each executor

| Where | `multi` (default) | `single` |
|---|---|---|
| Nothing set | used | — |
| `fr3_control.yaml → params: executor_mode` | `multi` (shipped) | `single` (changes the default for every launch; **not recommended** before Gazebo validation) |
| `torque_control_stack.launch.py` | — | `cbf_param_overrides:=executor_mode=single` (YAML-parsed, `_cbf_param_overrides`) |
| Bag replay launch | — | `cbf_overrides:=executor_mode=single` |
| Direct | — | `ros2 run franka_experiments cbf_safety_filter --ros-args ... -p executor_mode:=single` |
| Simulation harness | — | a condition in `conditions.yaml` with `cbf_param_overrides: executor_mode=single`. Not added: changing the harness was out of R1's scope. |

The start-up log line shows which executor is in effect (Verified on the real entry point, §7.4).

## 5. Default behaviour

The default is unchanged (Verified):
- `fr3_control.yaml` ships `executor_mode: multi`.
- `multi` builds `MultiThreadedExecutor` with `_max_workers == 3`.
- `main()` with the default spins exactly that executor (test).
- The real entry point logs `executor_mode=multi (MultiThreadedExecutor)` with no override.

The deterministic replay with the R1 install and `executor_mode: multi` is bit-identical to the
baseline install (§9.1). The parameter namespace differs only by the added key: 264 → 265 keys,
0 changed values (Verified, `seq/*.params.json`).

## 6. Build result

The clean-workspace procedure of S3 was used, for `franka_experiments` only (Runtime (recorded),
`01_build/`):
- the old build and install were quarantined to `/ros2_ws/quarantine_r1_20261112/`, with
  `COLCON_IGNORE`;
- `colcon build --packages-select franka_experiments` returned rc 0 in 0.88 s, with only the
  byte-compile warnings already seen in S3;
- the install differs from the pre-R1 install in exactly the three changed files, `SOURCES.txt` and
  the new test;
- install == source for 101 of 102 package files. The exception, `watchman_profile.md`, was never
  installed (the same at baseline);
- the installed install has 0 symlinks;
- final check after all runs: the source and installed SHA-256 of the three changed files are equal.

## 7. Tests

### 7.1 New tests: `test/test_cbf_executor_mode.py`, 22/22 passed

| Required check | Tests |
|---|---|
| The default is MT3 | `test_shipped_default_is_multi`, `test_multi_is_the_historical_three_thread_executor`, `test_main_default_spins_the_three_thread_executor`, `test_yaml_value_is_used_without_override` |
| `single` selects `SingleThreadedExecutor` | `test_single_builds_a_single_threaded_executor`, `test_ros_override_selects_single`, `test_main_single_spins_a_single_threaded_executor` |
| Invalid values are handled explicitly | `test_invalid_override_refuses_to_start` (×5), `test_missing_key_refuses_to_start`, `test_make_executor_rejects_unknown_modes` (×4), `test_single_with_fifo_priority_is_refused`, `test_multi_keeps_accepting_a_fifo_priority` |
| Start-up stays compatible | `test_main_*` (stub node, real executor class, monkeypatched `spin`), `test_main_refusal_cleans_up`, plus the real entry point (§7.4) |
| No control equation or default changes | `test_spec_and_helper_agree_on_the_modes`, `test_executor_mode_is_read_only_by_main`; the equation identity itself is shown on recorded data (§9.1), not in pytest |

### 7.2 Existing suites against S5

Runtime (recorded), `02_tests/comparison_with_S5.txt`.

| Suite | S5 failed / skipped / passed | R1 failed / skipped / passed | Failed set | Lost passes |
|---|---|---|---|---|
| 1: `franka_experiments` | 35 / 9 / 1383 | 35 / 9 / 1405 | identical (the 35 IDs of F-8-2) | none; the 22 new passes are the R1 tests |
| 2: `franka_simulation_experiments` | 0 / 0 / 121 | 0 / 0 / 121 | identical | none |
| 3: `franka_simulation` sim | 0 / 0 / 39 | 0 / 0 / 39 | identical | none |

All 65 assertion (`E `) lines of the 35 failures are byte-identical to S5's. They are the known
baseline failures (F-8-2), **not R1 regressions**. None was fixed (not in scope).

### 7.3 Smoke scripts

- The smoke construct fails exactly as in S5 (F-8-3, known).
- The smoke safety filter passed 5/5. In one of the five runs it printed a "never retrieved …
  Destroyable" message. That is the script's own `MultiThreadedExecutor(4)` on a daemon thread
  racing node destruction (Inferred from the trace; it occurs in the script, not in the node's
  executor).

### 7.4 Real entry point

Container, DDS domain 231, `02_tests/startup_entry_point.log`; Runtime (recorded):

| Arguments | Result |
|---|---|
| none | `executor_mode=multi (MultiThreadedExecutor)` |
| `executor_mode:=multi` | the same |
| `executor_mode:=single` | `executor_mode=single (SingleThreadedExecutor)` |
| `executor_mode:=fast` | `ValueError`, exit 1, node not constructed |
| `executor_mode:=single`, `qp_thread_fifo_prio:=10` | refused with the R1 message, exit 1 |

The traceback on SIGINT is the pre-existing F-8-6, present in both modes.

## 8. Replay comparison

### 8.1 Method

The method follows the QP-performance phase 2, with the production executor choice. Runtime
(recorded) unless marked.

- **Input.** The S6B cell bag (`pentagon_static_baseline_bare__full__gz__s0`, the same copy as in
  QP_PERFORMANCE_ROOT_CAUSE).
  - `ros2 bag play` replays `/clock`, the joint states, `qddot_nom` and the distances.
  - The node runs with the S6B parameter files. The only difference between the arms is
    `-p executor_mode:=<mode>`.
- **Executor.** It is built by the production `make_executor` from the node's own parameter,
  exactly as `main()` does (`r1_executor_replay.py`). Methods are wrapped at class level for
  timing; no production file is modified.
- **Conditions.** Isolated DDS domain 231, `ROS_LOCALHOST_ONLY=1`. Host idle at the start (load
  0.58), governor `powersave`, the same container and install for every run.
  - `s6b`: RTF 0.62 (the S6B run's RTF), unpinned.
  - `ecore`: RTF 1.0, `taskset -c 16-27` (E-cores only), a deliberately stressed condition.
- **Repetitions.** 5 per mode per condition, 20 runs in total. The mode order alternated per
  repetition (`multi, single` on odd repetitions, `single, multi` on even ones). Plus one
  unpinned RTF 1.0 trial pair (`03_replay/ab_trial/`), not pooled.
- **Window.** The S6B episode, sim [10.55, 40.56] s, about 3000 ticks.
- **True time.** The node's sim clock is frozen while a callback runs, so ages are computed against
  the true playback time reconstructed from the `/clock` callbacks: T(w) = a + rate·w, with
  a = max(clock_i − rate·w_i) (Derived).
- **Reference.** `r1_sequential_outputs.py` replays the same bag with **no executor**: the node's
  own callbacks in recorded sim-time order, with the rebuild before the tick at each 20 ms point. It
  gives the schedule-ideal output that both modes are compared against, per 10 ms slot.
- **Script.** `r1_compare.py` computes everything below. Each published command is paired with the
  latest tick that started before it (wall time).

### 8.2 Timing

Mean [min..max] over 5 runs; Runtime (recorded); `03_replay/analysis/r1_compare_stdout.txt`.

| Metric | s6b multi | s6b single | ecore multi | ecore single |
|---|---|---|---|---|
| ticks / sim s | 100 [99.9..100] | 100 | 94 [73.9..99.1] | 100 |
| late ticks (gap ≥ 15 ms) | 140 [86..205] | 0 | 635 [613..661] | 4 [0..9] |
| missed slots | 19 [2..45] | 0 | 558 [394..1120] | 0.6 [0..3] |
| max gap, ms | 22.6 [20..26] | 13.0 | 63.6 [34..144] | 16.8 [14..19] |
| process CPU, % core | 67.4 | 44.2 | 104 | 86.6 |
| executor-loop CPU, % | 24.7 | 14.8 | 24.6 | 18.2 |
| tick wall p50 / p95, ms | 2.42 / 8.22 | 0.94 / 1.53 | 3.61 / 16.2 | 1.79 / 2.86 |
| tick CPU p50, ms | 1.44 | 0.94 | 2.18 | 1.80 |
| tick wait (wall − CPU) p95, ms | 5.73 | ≈ 0 | 12.7 | ≈ 0 |
| `_solve` wall p50, ms | 1.26 | 0.36 | 1.53 | 0.62 |
| rebuild wall p50 / p95, ms | 7.0 / 12.1 | 2.5 / 3.8 | 9.95 / 14.4 | 5.34 / 7.12 |
| rebuild CPU p50, ms | 3.79 | 2.52 | 6.54 | 5.35 |
| rebuild gaps ≥ 30 ms | 0 | 0 | 0.2 [0..1] | 0 |
| rebuild start delay vs its 20 ms point, p95, ms | 4.61 | 4.30 | 6.07 | 4.61 |
| joint-state callbacks / sim s | 854 | 870 | 471 | 548 |
| joint-state callback wall p95, ms | 0.078 | 0.062 | 0.075 | 0.112 |

**Reading (Derived):**
- In single mode the wall time of every callback equals its CPU time: there is no in-callback
  wait. Under MT3, callbacks wait 2–13 ms at p95 inside their own bodies. That wait is GIL
  contention, as attributed by elimination in F-8-14; the attribution is not instrumented.
- Single mode also spends less CPU per callback (tick and rebuild p50: −33 to −35 % unpinned,
  −17 to −18 % on E-cores) and less in the executor loop.
- `ecore multi r5` was the worst run: 2217 episode ticks, max gap 144 ms. MT3's spread is far wider
  than single's in every metric.
- Single mode processed more joint-state callbacks than MT3 in both conditions. On E-cores, both
  modes lose about half of the 1 kHz stream at RTF 1 (Runtime). The losses are an existing load
  effect of the replay, larger under MT3.

### 8.3 Input ages at tick start (true time)

Mean of the per-run statistic, ms; Runtime (recorded), Derived for the true-time reconstruction.

| Age | s6b multi | s6b single | ecore multi | ecore single |
|---|---|---|---|---|
| node clock lag behind playback, p95 / max | 4.84 / 8.1 | 6.01 / 9.2 | 8.77 / 13.1 | 10.5 / 14.3 |
| joint state used (header), p50 / p95 / max | 4.35 / 5.22 / 9.5 | 4.98 / 6.60 / 9.7 | 4.89 / 9.38 / 18.2 | 6.85 / 11.2 / 18.5 |
| nominal command used, p95 | 13.9 | 14.9 | 16.5 | 18.8 |
| distance capture, p95 | 113 | 114 | 115 | 117 |
| **constraint snapshot used, p50 / p95 / max** | **8.72 / 17.6 / 25.0** | **0.17 / 9.5 / 11.0** | **5.69 / 15.9 / 25.2** | **0.18 / 6.1 / 12.3** |
| tick start after its 10 ms slot, p95 | 5.13 | 6.21 | 10.3 | 10.8 |
| command latency (publish − joint-state header), p50 / p95 / max | 5.70 / 8.66 / 16.5 | 5.48 / 7.21 / 11.0 | 8.16 / 15.4 / 32.6 | 8.77 / 13.2 / 22.9 |

The bag's record time minus the joint-state header is p50 0.00 ms and max 3 ms (Runtime), so the
header is a valid proxy for the measurement time.

**Reading (Derived): single mode does introduce stale measurements, in one specific and bounded
way.**
- **Joint state.** The joint state a tick uses is 0.6 ms older at p50 (unpinned) and 2.0 ms older
  on E-cores; at p95 the difference is 1.4–1.8 ms. The maximum is unchanged (9.5 vs 9.7 ms; 18.2
  vs 18.5 ms).
- **Cause (Verified in rclpy).** rclpy's `_wait_for_ready_callbacks` yields ready timers before
  subscriptions in each wait cycle (`executors.py` L712–728). When a tick falls due during a
  rebuild, it runs right after the rebuild, before the joint-state and `/clock` messages that
  arrived during the rebuild. The node's clock lag (p95 +1.2 to +1.7 ms) has the same cause.
- **Constraint snapshot.** It is much fresher in single mode: p50 0.17 ms vs 8.7 ms, and max
  11–12 ms vs 25 ms.
- **Overall freshness.** The command latency, from the measurement to the published command, is
  lower in single mode at p95 and max in both conditions. On E-cores its p50 is 0.6 ms higher.
- **Net.** Single mode trades about 1–2 ms of joint-state freshness for about 8 ms (p50) of
  constraint-snapshot freshness and a shorter tail.
- **Severity.** At 1 kHz the joint-state difference is about one sample. Whether that matters for
  the CBF's robustness margin (the `k_sigma` and latency terms) is not established here
  (Unverified, §11).

### 8.4 Deadlock and starvation

Runtime (recorded) unless marked.

- **Deadlock.**
  - The node holds no Python lock (Verified in the QP-performance analysis; unchanged by R1).
  - A single-threaded executor has no second thread to wait on.
  - No single-mode run hung. All 22 player processes (20 A/B runs and 2 trial runs) returned 0, and
    every run reached the end of the episode.
- **Starvation of the tick.** None in single mode.
  - The maximum gap is 13 ms unpinned and 14–19 ms on E-cores.
  - 0 missed slots unpinned and 0–3 on E-cores, against 2–45 and 394–1120 under MT3.
- **Starvation of the rebuild.** None: 0 rebuild gaps ≥ 30 ms in single mode, and the start delay
  is no worse than MT3's.
- **Starvation of the subscriptions.**
  - More joint-state callbacks were processed than under MT3 (above).
  - Distance callbacks behave the same: p95 wall time 0.21 vs 0.27 ms.
  - Each rclpy wait cycle drains every ready entity before waiting again (Verified, the same
    `executors.py` loop), so subscriptions are delayed by at most one callback, not starved.
  - That holds only while the total work stays below one core.
- **Ticks without a command.** In 2 of the 10 single runs one tick, at node time 0.046 s, published
  nothing. That is before the first joint state: `_qp_tick` returns early when `js is None`
  (L1052–1053, Verified). It is outside the episode and is the intended start-up behaviour.
- **What single mode cannot do (Derived).** A callback that blocks, such as a long rebuild or a
  stall in Pinocchio, delays **every** other callback, including the tick, by its whole duration.
  Under MT3 it delays them by the GIL share only. Nothing in the current callbacks blocks on I/O
  or on another callback (Inferred from the code read in ch. 08 and ch. 13), and the longest
  rebuild observed in single mode was 12.4 ms (episode window, 10 runs). Under MT3 it was 30 ms,
  wall time inflated by waits.

## 9. Numerical differences

### 9.1 Control equations: identical

Runtime (recorded); `03_replay/seq/`.

- `r1_sequential_outputs.py` was run four times on the S6B bag, with no executor:
  1. the baseline install;
  2. the baseline install again;
  3. the R1 install with `executor_mode=multi`;
  4. the R1 install with `executor_mode=single`.
- In all four, each of the 4068 command (`pub`), solver (`solve`: q̈, slack, level, iterations,
  status, objective) and row-count (`tick`) records is **bit-identical** to the corresponding record
  in the other runs.
- Therefore no output difference between the modes comes from the equations, the parameters or the
  QP. Every difference comes from **which inputs each tick sees and when** (Derived).

### 9.2 Executor runs: tolerances and attribution

Runtime (recorded); Derived for the tolerances. Deviations are the per-slot max-abs over the 7
joints of q̈_safe, in rad/s². The reference command magnitude is p50 0.90, p95 2.09 and max 5.49.

**Tolerances**, from the data itself:
- **Noise floor of each mode:** run-to-run differences within a mode. In the unpinned condition the
  p95 is 0.0046 (MT3 vs MT3) and 0.0135 (single vs single); the fraction of slots above 0.05 is
  1.0 % and 1.4 %.
  - Run-to-run variation exists in both modes: message arrival interleaving differs between runs.
- **Scheduling-ideal floor:** single mode against the sequential reference, p95 0.055 (0.050–0.060).
  - It sits at the level of the within-mode noise because single mode follows the reference's
    order: the rebuild first, then the tick.
- **Tolerance used:** a difference is attributed to scheduling unless it exceeds both floors and
  cannot be traced to a different input. Since §9.1 rules out the equations, no difference here
  exceeds that test.

| Comparison, s6b (ecore) | p50 | p95 | p99 | max | share > 0.05 |
|---|---|---|---|---|---|
| multi vs multi (10 pairs) | 3e-7 (0.005) | 0.0046 (0.27) | 0.050 (0.80) | 0.60 (2.23) | 1.0 % (20.6 %) |
| single vs single (10 pairs) | 0.0003 (0.0004) | 0.0135 (0.063) | 0.070 (0.22) | 0.71 (1.47) | 1.4 % (5.8 %) |
| **multi vs single (25 pairs)** | 0.0073 (0.016) | **0.419 (0.437)** | 0.63 (0.85) | 1.12 (2.57) | **23.2 % (31.9 %)** |
| multi vs sequential reference | 0.0093 | 0.414 | — | 0.88 | — |
| single vs sequential reference | 0.0018 | 0.055 | — | 0.50 | — |

**Attribution** (`r1_snapshot_attribution.py`, all 20 runs; Runtime (recorded); Derived):

- **Under MT3:**
  - 49.3–50.6 % of the episode ticks use a constraint snapshot built **before** their own 20 ms grid
    point, that is, the previous rebuild's;
  - those ticks carry 78 % (unpinned) and 55–67 % (E-cores) of the deviations above 0.05;
  - deviation p95 is 0.47 on those ticks vs 0.21 on the others.
- **In single mode:** 0.4 % of the ticks do so, and their deviation is ≤ 0.0023 at p95.
- **Cause:** the rebuild and every second tick fall due together. Under MT3 the tick starts on
  another thread while the rebuild is still computing, so it reads the old `self._con`. In single
  mode the rebuild runs to completion first, as in the reference.
- **New finding F-8-17:** this is a property of the **current default**. Its effect, about
  0.4 rad/s² p95 on a command of about 2 rad/s² p95, is a large share of the per-tick command.
- **The MT3 noise on E-cores:** p95 0.27 between two MT3 runs reflects the 394–1120 missed slots
  (§8.2). The E-core MT3 output is not reproducible run to run.

**Conclusions on the differences:**
- They are scheduling effects: which constraint snapshot, joint state and nominal command each tick
  uses.
- No change to the control law is involved.
- Single mode reproduces the schedule-ideal sequence closely.
- MT3 does not, about half of the time.
- Which ordering is preferable for closed-loop safety is **not** shown by an open-loop replay
  (§11).

## 10. Safety implications

- **Default runs are unaffected (Verified).** They use the same executor object and the same
  callback code, and give a bit-identical deterministic output. R1 is inert unless `single` is
  selected.
- **Invalid configurations fail closed (Verified).**
  - The node refuses to construct or spin, with an explicit error, for an invalid or missing
    `executor_mode` and for `single` combined with a FIFO priority.
  - A filter that does not start publishes no `torque_cmd`. The controller's command timeout then
    applies (ch. 02 §8; Verified there).
  - The launch is not blocked by this: a misconfigured launch shows up as a missing filter, not as
    a filter running a wrong executor.
- **What `single` changes for a running filter** (Derived from §8):
  - (+) No in-callback waits, no late or missed ticks in the replay, and a fresh constraint
    snapshot.
  - (+) A lower command latency at p95 and max, and less CPU.
  - (−) The joint state used is about one 1 kHz sample older at p50 and p95.
  - (−) A blocking callback would stall the tick for its whole duration; under MT3, only for the
    GIL share.
- **Not evaluated here:**
  - the closed loop (the plant reacting to the command);
  - the hardware timing (no `/clock`; real-time kernel, if any);
  - the interaction with `qddot_to_torque` and the controller timeouts under load;
  - the margin effect of the extra about 1–2 ms of joint-state age.
- **Recommendation.**
  - Keep `multi` as the default.
  - Validate `single` in Gazebo (E3) with the protocol §6.2 metrics before any wider use.
  - Use it on hardware only after that, and after measuring its timing on the hardware.

## 11. Uncertainties

- **Replay, not closed loop.** The joint states come from the bag, not from a plant reacting to the
  filter. Performance and deviation numbers do not transfer to an end-to-end result. **No
  end-to-end improvement is claimed.**
- **The S6B magnitude is not reproduced.** S6B's per-call costs were 1.3–2.9× the replay's
  (QP_PERFORMANCE_ROOT_CAUSE §3.3), with Gazebo and the other nodes co-located. Whether single mode
  keeps its margin under that load is unknown.
- **Host.**
  - Governor `powersave`; one machine; isolated domain.
  - Only the filter ran in-process. The other pipeline nodes were absent, because the bag replaces
    them.
  - The E-core condition was chosen as a stress test, not as a production configuration.
- **GIL attribution.** As in F-8-14, the GIL is the cause by elimination; the attribution is not
  instrumented (E1 remains open).
- **Staleness severity.** Whether 1–2 ms of extra joint-state age at p50 and p95 matters against the
  CBF's latency and robustness terms is not established (Unverified).
- **Hardware.** There is no 1 kHz `/clock` on hardware, so the executor-loop load (F-8-15) differs.
  The relative benefit there is unknown (Unverified).
- **The reference ordering.** The sequential reference runs the rebuild before the tick at each
  20 ms point. That matches the timer creation order and single mode's behaviour, but it is a
  modelling choice, not a documented contract of the node (Inferred). Under MT3 neither order is
  guaranteed.
- **Pre-existing side effect seen during R1 (F-8-18).** Every node start leaves temporary URDFs in
  `/tmp`. The R1 runs added 181 to the container's 38 743. They were not removed (cleanup is
  outside R1's scope).

## Consequences of modification

- **Making `single` the default** changes the behaviour of every launch that does not set
  `executor_mode`, including hardware launches. It needs a Gazebo A/B (E3), a hardware timing check
  and a dated note in ch. 02, ch. 03 and F-8-14.
- **Adding a mode**, for example `MultiThreadedExecutor(2)` or an events executor: extend
  `EXECUTOR_MODES`, `CBF_PARAM_SPEC['executor_mode']['choices']` and the tests together.
  `test_spec_and_helper_agree_on_the_modes` fails if they disagree.
- **Removing MT3 entirely** removes the default. That is a different change from R1 and needs its
  own approval.
- **Changing the callback groups** (for example merging `grp_perc` and `grp_ctrl` under MT3) is an
  alternative fix for F-8-17. It is not part of R1.
- **Raising `qp_thread_fifo_prio` with `single`** is refused by design. Removing that guard needs
  the 2026-11-07 GIL-inversion analysis redone.
- **Merging R1.** On merge to `humble-mattia`, the documented baseline must move by the §8
  procedure: re-check ch. 02 §2, ch. 03 §3.3 and §5.2, ch. 08 (`main`) and ch. 16, then append a
  baseline row.
