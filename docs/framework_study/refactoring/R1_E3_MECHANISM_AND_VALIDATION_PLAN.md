# R1 after E3: mechanism analysis and validation plan (proposal only)

> **Scope.** Requested by the user on 2026-11-18 (C-073). Analysis of **existing** evidence and code
> only: no Gazebo, no new run, no build, no production / R1 / H2 / harness change, CPU settings
> untouched. Everything in §4–§7 is a **proposal that needs explicit approval**.
>
> **Sources.**
> - E3 runs 05–10: `~/franka_baseline_evidence/refactor_r1_e3/runs/`, sealed; comparisons
>   `compare_05_06`, `compare_07_08`, `compare_09_10`; addendum `protocol/ADDENDUM_01_…`.
> - Isolated benchmarks: `validation/00_baseline/QP_PERFORMANCE_ROOT_CAUSE.md` (F-8-14, F-8-15,
>   F-8-16) and `refactoring/R1_EXECUTOR_CONFIGURATION.md` §8 (F-8-17).
> - Code at HEAD (`cbf_safety_filter.py`; rclpy Humble).
>
> **Per-tick analysis.** It is new in this step and written to the scratchpad only, not into the
> sealed evidence. The input is `analysis/s6b_series.npz` (`/NS_1/cbf_status`, 41 fields per tick;
> field 0 `n_rows`, 13 `solve_wall_ms`, 18 `constraint_snapshot_age_s`, 37
> `constraint_build_wall_ms`), `wall_pairs` and `s6b_host.json`. The window is t0 + 2 s … t0 + 30 s.
> A "late" tick is a sim gap > 15 ms; a "new rebuild" is a tick whose build time changed.
>
> **Labels** (00_INDEX §2): Verified · Derived · Inferred · Unverified · Runtime (recorded).

---

## 1. What R1 has demonstrated, and what it has not

**Demonstrated: a performance result (Runtime (recorded), n = 3 H2 pairs, one scenario).**
- `executor_mode: single` held its QP period: 99.0 / 99.8 / 99.7 ticks per sim s, 5–30 missed slots
  per episode, gap p99 15–18 ms, max ≤ 31 ms.
- `multi` reached 66.8 / 82.7 / 89.1, with 319–987 missed slots, gap p99 36–67 ms and max 57–94 ms.
- On matched inputs, single's per-operation wall times were shorter: rebuild 8.6–9.3 vs 16.3 ms p50,
  solve 1.05–1.17 vs 2.94–3.01 ms. Constraint snapshot age p50 was 0 vs 20–21 ms, and filter CPU was
  about 15 points lower.
- The advantage held with single run first (seed 2).

**Not demonstrated: any obstacle-avoidance improvement.**
- Clearance was the same in both modes (single − multi = −0.3 ± 1.0 mm; every lower bound
  0.118–0.121 m, above d_safe 0.10 m), and so was tracking (+1.2 ± 5.2 mm at p95).
- Faster, more regular QP ticks did not change the avoidance outcome in this scenario.
- No safety guarantee follows from one static obstacle on one path in simulation.

**Unproven.**
- The mechanism: GIL contention is the leading explanation but is not instrumented (§2, §3).
- Generalisation to moving obstacles, other positions, other phases, heavier constraint loads, other
  hosts and hardware.
- The cause of multi's run-to-run spread (F-8-39) and of the startup race (F-8-38).

---

## 2. What the existing evidence shows

### 2.1 Per-tick results for the six H2 runs (Runtime (recorded); Derived from the series)

| Run | Mode | Ticks/sim s | P(late \| new rebuild) | P(late \| no rebuild) | Late fraction by rows tercile (low / mid / high) | Rebuild p50, rows ≤ 30 / > 30 (ms) | 1-s windows: ticks mean ± sd | corr(wall rate, local RTF) |
|---|---|---:|---:|---:|---|---|---|---:|
| E3-05 | multi | 66.8 | 0.469 | **0.157** | 0.251 / 0.338 / 0.375 | 15.8 / 18.4 | 66.8 ± 13.2 | −0.17 |
| E3-07 | multi | 82.6 | 0.267 | **0.113** | 0.156 / 0.189 / 0.230 | 15.9 / 19.2 | 82.2 ± 9.2 | −0.21 |
| E3-10 | multi | 89.1 | 0.194 | **0.103** | 0.100 / 0.164 / 0.189 | 15.8 / 18.7 | 88.8 ± 7.2 | −0.07 |
| E3-06 | single | 99.0 | 0.485 | **0.000** | 0.223 / 0.238 / 0.281 | 9.0 / 10.8 | 99.0 ± 2.3 | **+0.91** |
| E3-08 | single | 99.8 | 0.426 | **0.000** | 0.192 / 0.234 / 0.211 | 8.6 / 9.4 | 99.8 ± 0.8 | **+0.99** |
| E3-09 | single | 99.7 | 0.399 | **0.000** | 0.184 / 0.190 / 0.226 | 8.3 / 10.5 | 99.7 ± 1.1 | **+0.98** |

Host, per run (`s6b_host.json`):
- P-core and E-core frequency medians were **identical in all six runs** (2100 / 1900 MHz).
- The filter's threads ran on E-cores 43–48 % of the time under multi and 43–70 % under single.
- Busy host cores: 13.8 / 13.0 / 12.7 in the multi runs and 13.9 / 12.8 / 12.7 in the single runs,
  in time order.
- Filter threads: multi has 4 Python threads at about 24–28 % each plus 2 small ones (process
  105–110 % of a core); single has 1 thread at 85–91 % (process 91–97 %).

### 2.2 The seven questions

**Q1. Why does multi's rate differ between nominally identical runs?**
- *Measured:* 66.8 / 82.7 / 89.1 ticks per sim s; 46.8 / 53.6 / 56.1 ticks per wall s; inputs and
  configuration identical (F-8-35); per-operation times stable (rebuild p50 16.28–16.36 ms).
- *Correlation (n = 3, no inference possible):*
  - the rate rose in session order (F-8-39);
  - fewer busy host cores and a lower RTF went with a higher rate;
  - P(late | new rebuild) fell 0.47 → 0.27 → 0.19.
  - The two single runs that came later also show slightly shorter rebuilds (9.3 → 8.7 → 8.6 ms), so
    whatever changed affected both modes, and multi more.
- *Within runs:* multi's tick rate swings ±7–13 per 1-s window. Its wall rate is uncorrelated with the
  local RTF, which is the signature of a wall-clock throughput ceiling.
- *Hypothesis (Inferred):* multi is bound by in-process serialisation whose cost depends on host
  conditions (co-located load, core placement, hand-off latency); small host differences move its
  ceiling. Single is not near its ceiling, so the same differences do not show up.
- *Cannot be established from this evidence:* which host variable matters. Frequency is ruled out at
  the median level; placement is not monotonic (0.43 / 0.48 / 0.47 E-core share against
  66.8 / 82.6 / 89.1).

**Q2. Why does single stay near 100 Hz?**
- *Measured:* its tick rate is set by the timer, not by throughput. Its wall rate tracks the local RTF
  almost perfectly (r = 0.91–0.99), so it delivers exactly the sim-time period.
- Its late ticks occur **only** when a rebuild falls due at the same time (P = 0.40–0.49 with a
  rebuild, 0.000 without). Even then the delay is bounded by one rebuild (gap p95 17–21 ms), and the
  timer catches up without skipping a slot.
- *Derived:* with one thread, a tick waits at most for the rest of the running callback. It then runs
  uninterrupted. The budget (about 9 ms rebuild every 20 ms plus 1 ms ticks) leaves headroom.

**Q3. Why are rebuild and solve slower under multi?**
- *Measured:* the same callbacks on the same inputs take about 1.9× (rebuild) and 2.7× (solve) the
  wall time. Single's wall time equals its CPU time (replay, R1 §8.2); multi's has 2–13 ms of wait
  inside the callback (replay).
- *Measured:* four Python threads each at about 25 % sum to about one core, while 15 of 28 host cores
  are idle.
- *Inferred, the leading explanation:* the callbacks are interleaved by a process-wide lock. The node
  holds no Python lock (Verified, `QP_PERFORMANCE_ROOT_CAUSE.md` §1.2). The candidates are the GIL
  and rclpy's internal executor locks. Every component except OSQP's C solve holds the GIL
  (§2.4 there: Pinocchio, small NumPy and SciPy-sparse hold it). This is GIL contention by
  elimination. **No GIL-wait counter or GIL-aware profile exists, so it is not proven.**
- A solve that releases the GIL must re-acquire it to return (convoy, H3 there). That would explain
  why the 0.28 ms native solve shows up as about 3 ms wall: Inferred.

**Q4. Do CPU use, RTF or host load correlate with the differences?**
- *Between modes (Measured):* multi uses 105–110 % of a core against single's 91–97 %, for fewer
  ticks.
- *Between multi runs:* rate up as busy cores and RTF went down (n = 3, correlation only).
- *Within runs:* multi ticks per sim s fall with local RTF (r = −0.22 … −0.61), consistent with a
  wall-throughput ceiling. Single's wall rate rises with RTF (r ≈ +0.95), consistent with a sim-period
  timer.
- *Not established:* any causal host variable.

**Q5. Is the improvement tied to the number of obstacle constraints?**
- *Measured:* in multi the late-tick fraction rises from the low to the high rows tercile in all three
  runs (+0.12, +0.07, +0.09), and rows > 30 add about 2.5–3.4 ms to a rebuild. In the heaviest
  curve-phase window (phase 4/6, 38–46 rows) multi drops to 49 / 73 / 78 ticks per sim s while
  single holds 98–100.
- In single the tercile effect is smaller (+0.06, +0.02, +0.04) and the rate does not drop.
- *Inferred:* more rows → a longer rebuild → longer overlap with ticks → more contention under multi.
  Single absorbs it because it has headroom.
- *Caveat:* rows co-vary with geometry (closeness to the obstacle) and with the curve phase. This is
  a correlation, not a manipulation.

**Q6. Can existing measurements explain the session trend in multi?**
- Partly at most.
- *Ruled out at the median level:* CPU frequency (identical) and the filter's core placement (not
  monotonic).
- *Co-varying:* host busy cores (13.8 → 13.0 → 12.7), episode RTF (0.70 → 0.65 → 0.63) and
  P(late | rebuild).
- *Not separable:* the seed-2 pair reversed the order, but each mode has only three points, all on one
  afternoon.
- **Unverified.**

**Q7. Can the startup race still affect results after H2 qualification?**
- *Measured:* in the race runs (E3-06, 08) the last cycle before birth was clean (V3: max |e| 4.7 mm,
  no HARD pair), and the controller was active throughout the episode (R06, R10, 0 faults). The
  spawner exits after activation.
- Single with the race (06, 08) against single without it (09): 99.0 / 99.8 vs 99.7 ticks per sim s,
  rebuild 9.3 / 8.7 vs 8.6 ms, late 0.245 / 0.214 vs 0.200. E3-06 is the worst, but it also had the
  highest host load and RTF, a confound.
- *Conclusion:* no measurable effect after qualification. A subtle effect cannot be excluded. Multi
  never had the race under H2, so the effect cannot have favoured single.

---

## 3. Architecture: what the code supports (Verified at HEAD unless marked)

| Element | Code | Consequence |
|---|---|---|
| Executor | `make_executor`: `MultiThreadedExecutor(num_threads=3)` (`MULTI_EXECUTOR_THREADS`, a module constant, not a parameter) or `SingleThreadedExecutor`; `main()` adds the one node and spins | Changing multi's thread count needs a code change, or a replay tool that builds the executor itself |
| rclpy Humble MT executor | a `ThreadPoolExecutor` of generic workers. The spinning thread rebuilds a Python `WaitSet` on each iteration and hands ready callbacks to workers (`QP_PERFORMANCE_ROOT_CAUSE.md` §1.1) | Up to 4 Python threads (main + 3 workers) compete for the GIL; executor bookkeeping is Python (F-8-15) |
| Callback groups | three `MutuallyExclusiveCallbackGroup`s: `grp_io` (joint states, `qddot_nom`, distances, ISO), `grp_perc` (`_update_constraints` timer, `cbf_update_rate_hz` 50), `grp_ctrl` (`_qp_tick` timer, `qp_rate_hz` 100 in condition `full`), plus the default group for `/clock` (1 kHz in Gazebo) | Groups only stop two callbacks *of the same group* overlapping. Rebuild, tick and I/O may overlap under multi, and no ordering is enforced (F-8-17) |
| Shared data | snapshot objects swapped by attribute assignment (`self._con = self._rows.build(…)`); no `threading.Lock`, `Event` or `sleep` | No user lock: any in-process wait is the GIL or rclpy internals (Derived) |
| Rebuild | `_update_constraints` → `ConstraintBuilder.build` (per-control-point Python loops, Pinocchio, small NumPy) | GIL-holding throughout (§2.4 there); cost grows with rows (§2.2 Q5) |
| Solve | `_solve`: `build_osqp_A` rebuilds the sparse `A` every tick (F-8-16), then OSQP `update` + `solve` | Only the C solve releases the GIL (about 0.04–0.3 ms); the rest holds it |
| Timers | rcl timers on sim time; a timer that is more than one period late skips the missed periods | Under multi, delays > 10 ms become **missed slots**; under single the delay stays below a period |
| FIFO option | `_elevate_thread_priority`, off by default; refused with `single` | Not involved in E3 |

**Supported by the code:**
- callback overlap under multi;
- no user-level locks;
- GIL-holding work in every callback except the C solve;
- a Python executor loop driven by 1 kHz inputs;
- no ordering between rebuild and tick.

**Needs experiment:**
- whether the waits are GIL waits or other executor-internal contention;
- the share of executor overhead against callback overlap;
- the role of host load and core placement in Gazebo.

---

## 4. Mechanism-isolation experiment (proposal; not implemented)

### 4.1 Design principle

The smallest reliable design is **offline**. Replay a recorded E3 cell bag into the production node in
an isolated domain, as the R1 replay did.

- Each E3 cell bag contains every input: `/clock` 52 k messages, `/NS_1/franka/joint_states` 52 k,
  `qddot_nom`, `per_link_distances`.
- Every arm then gets **identical inputs**, so factor D (workload) is held fixed by construction.
- It needs no Gazebo and no production change.
- What it does need, and needs approved:
  1. a replay-tool variant under `docs/framework_study/tools/` that builds
     `MultiThreadedExecutor(num_threads=n)` itself (the production `make_executor` fixes n = 3);
  2. for M2 only, installing a GIL-aware sampler (`py-spy`) in the container, which is an
     environment change.

### 4.2 Matrix

| ID | Hypothesis tested | Independent variable | Arms | Reps | Cost (estimate) |
|---|---|---|---|---:|---|
| **M1** (essential) | A executor overhead vs B callback overlap / GIL | executor | `single`, `MT(1)`, `MT(3)` | 3 each, order rotated (ABC, BCA, CAB) | 9 replays × about 1.5 min ≈ 15–20 min + analysis |
| **M2** (essential) | B: are the waits GIL waits? | sampler on / off | `MT(3)` and `single` under `py-spy record --gil --native` (or periodic `py-spy dump`) | 1 each | 2 replays ≈ 5 min, after approval to install py-spy |
| M3 (optional) | C: host load amplifies multi | background CPU load | `MT(3)`, `single` × {idle, synthetic load on the cores Gazebo used} | 2 each | 8 replays ≈ 15 min |

**Controlled in all of them:**
- the same bag (E3-05's, the worst multi run, so any effect has room to show) and replay rate (RTF
  0.65, the E3 median);
- the same container, install and parameter files;
- domain 231 with `ROS_LOCALHOST_ONLY=1`, and the governor as found (recorded, not changed);
- no pinning except in M3.

**Measurements**, the same instrumentation as the R1 replay (`r1_compare.py`):
- ticks per sim s, missed slots, gap p50 / p95 / p99 / max;
- P(late | rebuild overlap);
- per-callback wall and CPU, and wall − CPU (in-callback wait);
- executor-loop CPU, process and per-thread CPU, snapshot age.
- M2 adds: the fraction of samples in which the worker or tick thread waits for the GIL (`take_gil`
  or `PyEval_RestoreThread`), split by whether a rebuild is running.

### 4.3 Expected results under each explanation

| Explanation | M1: `MT(1)` (one worker + main loop; callbacks never overlap) | M1: `MT(3)` | M2 | M3 |
|---|---|---|---|---|
| **A** executor dispatch overhead dominates | as bad as `MT(3)` (wait − CPU large, P(late) high even without overlap) | bad | little GIL waiting; time in executor Python frames | load scales all arms similarly |
| **B** overlap / GIL contention dominates | close to `single` (small in-callback wait; P(late \| overlap) small) | bad; late ticks concentrated on overlap | tick and worker threads frequently in `take_gil` during rebuild overlap | load widens the MT(3) − single gap (longer hand-offs) |
| **C** host scheduling dominates | all arms close on an idle host | close to `single` on an idle host | waits off-CPU but not in `take_gil` | gap appears or grows only under load |
| **D** workload differs | excluded by identical inputs; any residual difference is an executor effect | — | — | — |

The prior replay already showed `MT(3)` late ticks only with a rebuild overlap, 0–1 late under
`single` and 2–13 ms in-callback waits under `MT(3)`. That favours B over A and C. M1 and M2 test it
directly.

**Confounders:**
- the replay is milder than Gazebo (§3.5 of the root-cause document: per-call costs 1.3–2.9×
  smaller), which is why M3 exists;
- the replay is open loop, so the joint states do not react;
- `MT(1)` still has two Python threads (main + worker), so it isolates callback overlap, not all
  threading;
- py-spy sampling adds overhead, so M2's timings are not compared with M1's.

---

## 5. Broader validation (proposal; not implemented)

### 5.1 What exists (Verified, installed `scenarios.yaml`)

- **H2 needs `motion:=pentagon`, so only the `pentagon.scenarios` entries are eligible.** They all
  share the same circle (centre (0.4, 0, 0.45), r 0.25 m, plane yz, cycle 6 s).
- **Static:**
  - `pentagon_static_baseline_bare` (E3);
  - `pentagon_static_baseline` (the same obstacle in the furnished `diag` scene);
  - `pentagon_on_path` (0.4, −0.25, 0.45), on the path;
  - `pentagon_beside_path`;
  - `pentagon_pos_p1…p8`, `pentagon_occl_o1`;
  - `pentagon_multi_static`, `pentagon_two_static`.
- **Moving:**
  - `pentagon_crossing` (linear, 0.3 m/s);
  - `pentagon_lateral` (linear);
  - `pentagon_lateral_osc`, `pentagon_lateral_y`, `pentagon_sweep` (oscillate);
  - `pentagon_pursuer` (pursuit, 0.3 m/s);
  - `pentagon_wander`, `pentagon_wander_close` (wander, **`jitter_m` 0.04**);
  - `pentagon_multi_mixed` (static + oscillating).
- **Seeds:** a seed changes the obstacle position only where `jitter_m` > 0 and seed ≠ 0
  (`scenario_catalog.resolve` L126–131). Otherwise, under H2, seeds are nominal repeats (F-8-35).
- **Trajectory phase:** the H2 CLI accepts `--start-phase` (any value in cycles), but `e3_run.sh`
  hard-codes `0.0` and the scenario.
- **Perception workload:** `--perception true|emulated` exists (`run_gz_scenarios`). Emulated replaces
  the depth pipeline, changing both host load and input timing. The furnished `diag` scene adds
  furniture rows. A memory lead says about 44 vs 22 QP rows and phantom obstacle velocities on the
  furniture; that is Unverified at this revision.

### 5.2 Proposed cells

| # | Cell | Tests | Why it matters | Initial conditions | Priority |
|---|---|---|---|---|---|
| V1 | `pentagon_static_baseline_bare`, `--start-phase 0.5` | a different trajectory phase at birth, same obstacle | the only new information about this cell is phase, not seed (F-8-35) | H2, phase 0.5 | **essential** |
| V2 | `pentagon_static_baseline` (scene `diag`) | constraint load about 2× (furniture rows): factor D in closed loop | multi's deficit grew with rows (Q5); does single keep 100 Hz at higher load? | H2, phase 0.0 | **essential** |
| V3 | `pentagon_on_path` | obstacle on the path; strong avoidance at a different position | avoidance demand and rows peak differently | H2, phase 0.0 | **essential** |
| V4 | `pentagon_wander_close`, seeds 1 and 2 | moving obstacle near the path, with real seed variation (`jitter_m`) | moving obstacles stress staleness (snapshot age 0 vs 20 ms matters more when v_obs ≠ 0) | H2, phase 0.0; jitter gives different, recorded p0 | **essential** |
| V5 | `pentagon_crossing` or `pentagon_pursuer` | a fast approach (0.3 m/s) | short reaction window; the latency difference could matter here first | H2, phase 0.0 | optional |
| V6 | `pentagon_multi_mixed` | two obstacles, identity, rows | the heaviest existing load with motion | H2, phase 0.0 | optional |
| V7 | V2 cell with `--perception emulated` | perception host load removed | separates host load (factor C) from executor in closed loop | H2 | optional |
| V8 | `pentagon_pos_p1…p8` subset | positions | coverage, low information per run | H2 | optional, last |

**Per cell: 2 pairs**, i.e. 4 runs (single, multi × 2).

**Order:** within each cell, ABBA for the first pair and BAAB for the second. The cell order across
the session is randomised with a fixed RNG seed, written into the pre-registration before the first
run.

**Interleaving:** no cell is completed in one block. Pairs of different cells alternate, so session
drift (F-8-39) spreads across cells and modes.

**Metrics:**
- *Executor:* ticks per sim s and per wall s, missed slots, gap p50 / p95 / p99 / max, gaps ≥ 50 ms,
  P(late | rebuild), rebuild and solve p50 / p95, snapshot age, filter CPU and threads, RTF, rows
  terciles and the six phase windows.
- *Safety:* min clearance with the conservative lower bound (R14), contact (R12), faults and
  fallbacks (R10, R11), reflex (R08), the negative-h obstacle-row counts.
- *Tracking:* EE error p50 / p95 per window.

**Comparability:** `e3_h2_compare.py` criteria per pair (VALID both, birth phase within 0.01 cycle,
same configuration except `executor_mode`). For V4 the jittered p0 is the same within a pair, because
the same seed is used for both arms.

**Mandatory gates:** R01–R22 unchanged, including R14, R19 and R20 as STOP rules, plus a
**new pre-registration addendum before any run**, because two gate inputs do not carry over to moving
obstacles:
- **V4's g_min** (d_safe + v_ref · τ) assumes a static obstacle. For moving cells it must add the
  obstacle's closing speed: g_min = d_safe + (v_ref + v_obs) · τ.
- **R13/R14's reconstruction** pairs joint samples with the last obstacle pose (F-8-13: up to
  |v| · 50 ms of error). The conservative bound for moving cells must subtract |v_obs| · 0.05 m
  (15 mm at 0.3 m/s).

**Harness:** `e3_run.sh` hard-codes the scenario, the start phase and the timeout. Running V1–V8 needs
a parameterised harness copy (for example `scripts_h3/`, the same checks plus the scenario and phase
as recorded arguments). That is a harness change and needs approval. R1 and H2 stay unchanged.

**Known risks (memory leads, Unverified at this revision):**
- `pentagon_sweep` collided at shipped settings in an earlier campaign;
- the furnished scene produced phantom obstacle velocities.

An R14 STOP in such a cell would be an avoidance finding in both modes, not an executor result. The
design keeps the two apart by comparing within pairs.

---

## 6. Prioritisation

| Step | Experiments | Runs | Value | Cost | Confound control |
|---|---|---:|---|---|---|
| 1 (essential) | **M1** | 9 replays | separates A from B; no Gazebo | about 30 min incl. analysis | identical inputs (D fixed); rotated order |
| 2 (essential) | **M2** | 2 replays | the only direct test of the GIL attribution | about 10 min after a py-spy install | identical inputs |
| 3 (essential) | **V2, V4** | 8 Gazebo runs | higher load (V2) and moving obstacles with real initial-condition variation (V4): the two generalisation questions most likely to change the conclusion | about 50–60 min | ABBA/BAAB, interleaved |
| 4 (essential) | **V1, V3** | 8 Gazebo runs | phase and position | about 50–60 min | same |
| 5 (optional) | M3, V5, V6, V7, V8 | 8 + 4 per cell | host load; harder moving cases; perception load; coverage | as listed | same |

**Minimum, mechanism:** M1 + M2 (11 replays).

**Minimum, generalisation:** V2 + V4 (8 runs). V1 + V3 complete the essential set (16 runs in all).

---

## 7. Recommended next experiment

**M1, the offline executor-thread replay (`single`, `MT(1)`, `MT(3)` × 3, on E3-05's recorded bag).**

Why first:
- It is the cheapest experiment that discriminates between the two leading explanations, executor
  overhead (A) and callback overlap / GIL (B).
- It holds the workload exactly fixed (D).
- It needs no Gazebo, no production change and no environment change; only a replay-tool variant under
  `docs/framework_study/tools/`.

Its outcome decides how M2 is read. The Gazebo campaign (§5) answers a different question, whether the
benefit generalises; it does not depend on M1 and can follow in either order.

---

## Consequences of modification

- **R1 itself is unchanged by this plan.** `multi` stays the default. Nothing here justifies changing
  the default before the essential validation cells have run.
- If M1 and M2 confirm B (GIL contention from callback overlap), `single` (or explicit serialisation)
  is the structural fix. FIFO priority, a shorter switch interval and more threads are not fixes
  (`QP_PERFORMANCE_ROOT_CAUSE.md` §3.2).
- If A dominates, executor overhead is the target: fewer events, no `/clock` on hardware (F-8-15),
  F-8-16.
- On hardware there is no 1 kHz `/clock`, so the executor load differs. The benefit there is
  Unverified until it is measured.
