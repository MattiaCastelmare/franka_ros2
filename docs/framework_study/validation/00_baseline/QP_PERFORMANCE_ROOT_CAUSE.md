# QP performance: root-cause analysis of the acceleration-level CBF filter

> **Scope.** Analysis only, requested by the user on 2026-11-12. **No production, test, launch or
> configuration file was modified; no Gazebo, no launch file, no build.** Isolated benchmarks only:
> the production node (`franka_experiments.nodes.cbf_safety_filter.CBFSafetyFilter`, installed
> copy, sha256 equal to the source) was run in-process inside the `franka_ros2` container on an
> isolated DDS domain (231, `ROS_LOCALHOST_ONLY=1`). It was fed the **recorded** S6B cell
> (`pentagon_static_baseline_bare / full / s0`). The bag and manifest hashes equal the sealed S6B
> evidence, and the launch parameter files are identical too. Instrumentation is by class-level
> wrappers installed before construction. No file of the node was changed. Baseline `ba68077`,
> HEAD `2636e31`.
>
> **Evidence.**
> - Outputs: `~/franka_baseline_evidence/baseline_f708991/08_qp_performance/` (README_QPPERF.txt,
>   49 files, 7.4 MB, `SHA256SUMS` hash `cf0738b2…25b2`, write-protected).
> - Scripts: [tools/baseline_validation/](../../tools/baseline_validation/README.md) (`qpperf_*`).
> - `06_static_obstacle/` and `07_clearance_audit/` were re-verified against their `SHA256SUMS` and
>   are unchanged.
>
> **Labels** (00_INDEX §2):
> - **Verified**: read in code;
> - **Measured**: a number from a benchmark run in this step, recorded in the evidence [Runtime (recorded)];
> - **Recorded**: a number from the S6A/S6B runs [Runtime (recorded)];
> - **Derived**: computed from the above;
> - **Inferred**: an explanation consistent with the evidence but not directly measured;
> - **Unverified**: not established.

---

## 0. Answer in brief

| # | Question | Answer | Label |
|---|---|---|---|
| 1 | Is the filter compute-bound? | **No.** Uncontended, a constraint rebuild costs 2.0 ms of CPU (p95 2.75), a QP tick 0.72 ms (p95 1.07) and the `_solve` call 0.28 ms. All callbacks together use 0.20 CPU-s per sim s, about 20 % of one core at RTF 1. The periods are 20 ms and 10 ms. | Measured (phase 1) |
| 2 | Where does the time go? | Under the production `MultiThreadedExecutor(3)`, the same callbacks take 2–3× their uncontended wall time. Ticks wait **while a rebuild runs**: a median of 2.3–2.6 ms of waiting, against 0.5–0.6 ms without a rebuild. Late ticks (sim gap ≥ 15 ms) occur only when a rebuild overlaps: P = 7–27 % with an overlap, 0–0.5 % without. Running the identical node and callbacks under a **`SingleThreadedExecutor` removes the late ticks**: 0 and 1 of 3001, against 195 and 656 in the matched MT3 runs. It also cuts process CPU by 27–37 % at the same RTF. | Measured (phase 2) |
| 3 | Is the GIL the cause? | **The mechanism is in-process serialisation.** Three measurements show it: the waits occur while the process uses less than one core on an otherwise idle host; they occur only when another Python callback is running; and they disappear when the callbacks are serialised by the executor instead. The node holds no Python lock (Verified), so the only lock held across these callbacks is the GIL. **GIL contention is therefore the leading explanation, Derived. It is not directly instrumented** (no GIL-wait counter); §5 gives the experiment. | Measured; Derived; GIL wait itself Unverified |
| 4 | Solver or preparation? | **Preparation.** OSQP's C solve is about 3.5 % of a tick's CPU. Re-building the sparse `A = [G; I]` with SciPy on every tick (`build_osqp_A`) is about 27 %; `tangential_bias` 21 %; logging, status and diagnostics about 19 %. In the rebuild the cost is spread over many small NumPy calls in per-control-point Python loops; no single hotspot dominates. | Measured (cProfile, relative) |
| 5 | Which operations release the GIL? | OSQP `solve()` **releases** it (probe at 0.95–1.03 of its solo rate). Pinocchio FK and Jacobian calls **hold** it (0.51, the same as pure Python; no `PyEval_SaveThread` in the module). Small-array NumPy and SciPy-sparse construction hold it. At the filter's size the solver's native work is about 0.04 ms, so releasing the GIL gives no useful parallelism. | Measured (phase 3); Verified (symbols) |
| 6 | Are 50 Hz rebuilds and 100 Hz QP compatible with the measured costs? | **Yes, computationally:** the budget is 20–60 % of a core in every configuration measured. The deficit is a **scheduling** effect. Single-threaded, the worst tick delay equals the rebuild duration, p95 4.0 ms (P-cores, RTF 0.62) and 7.8 ms (E-cores pinned, RTF 1), both under the 10 ms period. With S6B's slower clocks that margin is thin (§4.3). | Measured; Derived |
| 7 | Does the benchmark reproduce S6B? | **The mechanism, yes; the magnitude, no.** S6B lost 11.6 % of its ticks, with build p50 16.8 ms and solve 2.95 ms wall. The replay at the same RTF loses at most 1.9 % and shows build 6–10 ms and solve 1.0–1.6 ms. The residual factor of about 1.3–2.9 is **Unverified**; candidates are §4.3. | Measured; Recorded; Unverified |
| 8 | First modification proposed | An **opt-in** executor choice in `cbf_safety_filter.main()`. The default stays as today, the callbacks are unchanged, and no CBF equation, geometry or margin is touched. **Not implemented; needs explicit approval** (§7). | Proposal |

---

## 1. Current architecture (Verified, `cbf_safety_filter.py` at `ba68077`)

### 1.1 Executor and threads

- `main()` (L2070–2081) builds **`MultiThreadedExecutor(num_threads=3)`** and spins the single node.
  The comment there reads: "One thread per callback group: I/O, constraint builder, QP loop. The QP
  thread must never wait on perception."
- In rclpy Humble, `MultiThreadedExecutor` is a `ThreadPoolExecutor` of 3 **generic** workers, not one
  thread per group (Verified, `rclpy/executors.py` L822–859).
  - The spinning thread runs `wait_for_ready_callbacks`, which rebuilds a `_rclpy.WaitSet` in Python
    on every iteration (L516 ff.).
  - Each ready entity is taken (C take plus deserialisation), then handed to a worker as a task (L450–515).
  - A `MutuallyExclusiveCallbackGroup` only prevents two callbacks **of the same group** from
    running at once.
- Because the node sets `use_sim_time`, rclpy's `TimeSource` adds a `/clock` subscription
  (`QoSProfile(depth=1, BEST_EFFORT)`, `time_source.py` L66–71) in the node's default group. In
  Gazebo, `/clock` arrives at 1 kHz. It does not exist on hardware.
- rclpy Humble has **no** Python `StaticSingleThreadedExecutor` and **no** `EventsExecutor` (Verified
  by grep; a benchmark run asking for the former failed at start).

### 1.2 Callbacks, groups and rates

| Callback | Group | Trigger | Rate in S6B | Work |
|---|---|---|---|---|
| `_on_joint_state` (L737) | `grp_io` | `/NS_1/franka/joint_states` (`joint_states_fast`), depth 1 | 1 kHz offered | name→value dicts, `array_equal` freeze check, `JointSnap`, diagnostic q̈ EMA |
| `_on_qddot_nom` (L773) | `grp_io` | `/NS_1/qddot_nom`, depth 1 | 100 Hz | `NomSnap` |
| `_on_distances` (L820) | `grp_io` | `/cbf/per_link_distances`, BEST_EFFORT depth 1 | about 30–36 per sim s | `ObstacleSnap`, rate estimate |
| `_on_iso_safety` (sub L510) | `grp_io` | ISO topic | not active in S6B | — |
| `_update_constraints` (L958) | `grp_perc` | timer 1/`cbf_update_rate_hz` (L525) | 50 Hz sim | `self._con = self._rows.build(js, obs, now)`: `ConstraintBuilder.build` (`cbf_state_rows.py` L2335) |
| `_qp_tick` (L1038) | `grp_ctrl` | timer 1/`qp_rate_hz` (L527) | 100 Hz sim | nominal shaping, `tangential_bias`, bounds, `_solve` (L1624), `_publish` (L1921), `_publish_status` (L1929), `_report` (L1769) |
| `TimeSource.clock_callback` | default | `/clock`, depth 1 | 1 kHz offered | sets ROS time |

**Shared data.** Snapshots are swapped by attribute assignment (`JointSnap`, `ObstacleSnap`,
`NomSnap`, `ConstraintSnap`): lock-free, and atomic under the GIL. The node has **no**
`threading.Lock`, `Event`, `sleep`, service client or future wait (Verified by grep). Nothing in it
depends on two callbacks running at the same time.

**Solver.** `_solve` creates and `setup()`s a new OSQP problem **only when the row count changes**
(L1684–1696); otherwise it calls `update(q, l, u, Ax)` (L1697–1701). The `Ax` passed in is
`build_osqp_A(G, NV, n_slack).data`, so the full `csc` matrix is rebuilt on every tick: `csc_matrix`
from COO, `sparse.identity` and `sparse.vstack` (`cbf_qp_assembly.py` L73–84). Only `.data` is then
used.

**Optional FIFO priority.** `_elevate_thread_priority` (L963, called from `_qp_tick` L1042) is off by
default. Its docstring records an earlier GIL-inversion observation (46 vs 70 Hz); that is a lead, not
re-tested here.

---

## 2. Timing evidence

### 2.1 Recorded (S6A, S6A.1, S6B)

| Quantity | S6A | S6B | Label |
|---|---|---|---|
| QP ticks per sim s (target 100) | 73.6 | 88.4 (2654 / 3001; 11.6 % missed) | Recorded |
| Episode RTF | 0.62–0.64 | 0.618 | Recorded |
| Constraint build wall (`/cbf_status`) p50 / p95 | 16.1 / 26.7 ms | 16.8 ms p50 | Recorded |
| Solve wall p50 / p95 | 3.0 / 9.9 ms | 2.95 / 8.99 ms | Recorded |
| Late ticks after a fresh rebuild vs otherwise | 51 % vs 2 % (first tick seeing a new snapshot) | 33.4 % vs 0.68 % | Recorded; Derived |
| Filter process CPU (host sampler) | not sampled | 106 % of a core; main thread 28 %, three workers about 24 % each | Recorded |
| Host | load ≈ 0.4 | 45 % busy; P-cores p50 2.07 GHz, E-cores 1.90 GHz; governor `powersave`; filter threads migrated over all 28 CPUs, E-cores included | Recorded |
| Perception | about 30 Hz sim | about 30–36 per sim s | Recorded |

The `/cbf_status` wall fields are **elapsed** time. They include time spent waiting for the GIL or a
core, so on their own they cannot separate compute from contention (S6A.1 §1).

### 2.2 Phase 1: uncontended cost (Measured)

The S6B bag was replayed **sequentially**: no executor, the node's own callbacks called in sim-time
order, the rebuild every 20 ms and the tick every 10 ms of sim time, with the ROS clock set to each
event. Episode window: 30.01 sim s.

| Callback | Calls per sim s | CPU p50 / p95 / max (ms) | CPU ms per sim s |
|---|---:|---|---:|
| `_on_joint_state` | 999.5 | 0.010 / 0.021 / 0.21 | 11.8 |
| `_on_qddot_nom` | 100 | 0.005 / 0.009 / 0.07 | 0.6 |
| `_on_distances` | 36.5 | 0.093 / 0.150 / 0.39 | 3.7 |
| `_update_constraints` | 50 | **2.03 / 2.75 / 4.04** | 103.5 |
| `_qp_tick` (includes `_solve`) | 100 | **0.72 / 1.07 / 67.8** | 78.1 |
| of which `_solve` | 100 | 0.28 / 0.43 / 66.8 | 32.5 |
| **Total** | | | **197.8 ms per sim s** |

Rows per rebuild: p50 26, max 56. Wall equals CPU throughout, so there is no contention by
construction. The one 67.8 ms tick lies inside its `_solve`. Its cause is Unverified: one event in
3001, possibly an OSQP re-`setup()` after a row-count change, though most such setups are cheap; p99 is 0.59 ms. It is irrelevant to the rate.

### 2.3 Phase 2: the production executor (Measured)

The real node is spun by `MultiThreadedExecutor(3)` exactly as in `main()`. A separate C++ process,
`ros2 bag play`, replays `/clock` (recorded, 1 kHz), the joint states, `qddot_nom` and the distances
from the S6B bag at a set rate. That rate fixes the RTF exactly, since the node's time is the bag's
`/clock`. Per-call wall and thread CPU are recorded, together with per-thread CPU from
`/proc/self/task` over the episode window and the executor's `wait_for_ready_callbacks` count and
CPU. The host was otherwise idle; CPU frequencies were not sampled. "Late" means a sim gap of
≥ 15 ms between ticks, i.e. a tick more than half a period late.

| Run | Executor | RTF | Cores | Ticks per sim s | Late ticks / max gap (ms) | Rebuild wall p50/p95, CPU p50 (ms) | Tick wall p50/p95, CPU p50 (ms) | Solve wall / CPU p50 (ms) | Process CPU % of a core (main loop; workers) | Callback bodies % |
|---|---|---|---|---:|---|---|---|---|---|---:|
| `base_r1.0` | MT3 | 1.0 | any | 99.9 | 263 / 27 | 6.6 / 10.3, 3.5 | 2.4 / 6.9, 1.33 | 1.23 / 0.54 | 90.7 (32.6; 3 × 17.5) | 36.7 |
| `base_r0.62` | MT3 | 0.62 | any | 99.4 | 89 / 163 | 6.2 / 10.2, 3.5 | 2.1 / 6.6, 1.27 | 1.01 / 0.51 | 62.6 (24.8; 3 × 11.2) | 22.7 |
| `base_r0.62_b` (repeat) | MT3 | 0.62 | any | 99.9 | 195 / 32 | 7.6 / 13.4, 4.1 | 2.7 / 9.8, 1.55 | 1.38 / 0.62 | 71.2 (26.8; 3 × 13.4) | 27.7 |
| `base_r0.62_ecore` | MT3 | 0.62 | E 16–27 | 100.0 | 429 / 23 | 10.3 / 14.1, 6.4 | 3.5 / 12.5, 2.18 | 1.57 / 0.78 | 81.5 (27.2; 3 × 17.1) | 38.4 |
| `base_r1.0_ecore` | MT3 | 1.0 | E 16–27 | **98.1** | **656** / 40 | 9.9 / 13.9, 6.5 | 3.5 / 13.6, 2.18 | 1.56 / 0.78 | **105.0** (26.6; 3 × 24.9) | 59.7 |
| `rebuild25_r1.0` | MT3, `cbf_update_rate_hz` 25 | 1.0 | any | 99.2 | 230 / 42 | 7.3 / 14.4, 3.9 | 2.3 / 7.3, 1.39 | 1.12 / 0.54 | 87.7 (34.9; 3 × 15.7) | 30.4 |
| `rebuild0_r1.0` | MT3, rebuild **compute** removed (one frozen snapshot; timer kept) | 1.0 | any | 100.0 | **18** / 29 | — | 0.8 / 2.7, 0.54 | 0.24 / 0.09 | 81.8 (43.4; 3 × 11.2) | 12.1 |
| `sw0.5ms_r0.62` | MT3, `sys.setswitchinterval(0.5 ms)` | 0.62 | any | 100.0 | 74 / 24 | 6.4 / 9.9, 3.6 | 2.1 / 6.5, 1.33 | 1.05 / 0.53 | 63.7 (25.2; 3 × 11.6) | 23.4 |
| `sw0.5ms_r1.0_ecore` | MT3, 0.5 ms | 1.0 | E 16–27 | 98.5 | 633 / 36 | 10.0 / 15.4, 6.4 | 3.4 / 12.4, 2.11 | 1.53 / 0.75 | 105.3 (27.1; 3 × 24.9) | 59.0 |
| **`single_r0.62`** | **SingleThreadedExecutor** | 0.62 | any | **100.0** | **0** / 13 | 2.6 / 4.0, 2.6 | 0.97 / 1.64, 0.96 | 0.38 / 0.38 | **44.5** (39.7) | 17.5 |
| **`single_r1.0_ecore`** | **SingleThreadedExecutor** | 1.0 | E 16–27 | **100.0** | **1** / 15 | 5.6 / 7.8, 5.6 | 1.86 / 3.19, 1.85 | 0.64 / 0.64 | **89.1** (84.4) | 53.8 |
| `jslow_r1.0` | MT3, joint states remapped to `/NS_1/joint_states` | 1.0 | any | — | **invalid** | — | — | — | 44.1 (29.9; 3 × 3.7) | 0.9 |

Notes:
- **Run-to-run spread is large.** Two identical MT3 runs at RTF 0.62 gave 89 and 195 late ticks, so
  only differences of about 2× or more are read as effects.
- `jslow` is **invalid as a compute test.** The node received 16.3 msgs per sim s on the remapped
  topic but never built or solved (no `_solve` call), so no valid arm snapshot ever reached it. The
  first message on that topic in the bag carries only the two finger joints. Why no arm message
  arrived was not investigated. The run is kept only as a reference for executor load: about 44 % of
  a core with almost no callback work.
- With MT3 the node consumes only 465–874 of the 1000 joint-state messages per sim s, by design
  (keep-last depth 1). The single-threaded runs consume 519–863.
- "Wait" means wall time minus thread CPU, per call.

**Overlap analysis** (`qpperf_late_tick_overlap.py`; `out/phase2/overlap_summary.txt`). Here
"overlap" means a rebuild running between the previous tick's end and this tick's end.

| Run | P(late \| overlap) | P(late \| none) | Tick wait p50 with a rebuild **during** the tick / without | Rebuild wait p50 |
|---|---|---|---|---|
| `base_r0.62_b` | 0.073 (n 2666) | 0.000 (n 332) | 2.29 / 0.51 ms | 3.44 ms |
| `sw0.5ms_r0.62` | 0.027 | 0.000 | 1.52 / 0.24 ms | 2.81 ms |
| `base_r0.62_ecore` | 0.176 | 0.000 | 2.64 / 0.61 ms | 3.89 ms |
| `base_r1.0_ecore` | 0.273 | 0.005 | 2.37 / 0.64 ms | 3.44 ms |
| `sw0.5ms_r1.0_ecore` | 0.253 | 0.000 | 1.96 / 0.59 ms | 3.63 ms |
| `single_r0.62` / `single_r1.0_ecore` | 0 / 0 | 0 / 0.001 | (no concurrent rebuild possible); wait 0 | 0 |

This matches S6B's recorded pattern (33.4 % vs 0.68 %), though the definitions differ slightly.

### 2.4 Phase 3: what releases the GIL (Measured, `qpperf_gil_probe.py`)

A pure-Python probe thread counts as fast as it can while a worker thread loops on one operation at
the filter's problem size. Switch interval 5 ms.

| Operation | ms per call | Probe rate relative to solo | Reading |
|---|---:|---:|---|
| `time.sleep(0.5 ms)` (reference: releases) | 0.556 | 0.98 | releases |
| pure-Python loop (reference: holds) | 0.048 | 0.51 | holds |
| OSQP 0.6.7 `update(q)` + `solve()` on a random 13 × 30 QP (slightly smaller than the filter's: ≥ 14 decision variables = 7 joints + `N_SLACK` 7 + per-row slots, `cbf_safety_filter.py` L269–270; rows p50 26 + box) | 0.045 | 1.03 | releases (the call is too short to tell well) |
| OSQP `solve()`, 300 × 600 dense (same Python path, about 1000× the C work) | 48.5 | 0.95 | **releases during the C solve** |
| Pinocchio 4.0 `computeJointJacobians` + `framesForwardKinematics` (FR3 URDF) | 0.0014 | 0.51 | **holds** (no `PyEval_SaveThread` import in `pinocchio_pywrap_default`; Verified) |
| 20 × (`norm(cross)` + `clip`) on 3-vectors | 0.33 | 0.22 | holds; the probe gets **less** than a fair share (cause Unverified) |

The `_osqp.so` module imports `PyEval_SaveThread` / `RestoreThread` (Verified) and releases the GIL
in practice (above). The rclpy C extension also imports `PyEval_SaveThread`: the blocking `rcl_wait`
inside `wait_for_ready_callbacks` is expected to release the GIL (Inferred; that is how the main loop
can idle at all). Building the wait set and dispatching tasks is Python and holds it.

---

## 3. Analysis

### 3.1 Where the process CPU goes (Derived from §2.2–2.3)

At RTF 0.62 with MT3 (`base_r0.62`, `_b`), the process uses **63–71 % of a core**:

| Component | Share of a core |
|---|---|
| Callback bodies | 23–28 % |
| Executor main loop (`wait_for_ready_callbacks`) | 24–25 % |
| Worker-side dispatch, take and deserialisation, and task overhead | about 10–13 % |
| DDS and other native threads | about 3–4 % |

The same work single-threaded costs **44.5 %**: bodies 17.5 %, wait loop 14.7 %. The extra 18–27
points under MT3 are threading overhead. One part is visible inside the callbacks themselves: their
thread CPU rises from 2.6 to 3.5–4.1 ms per rebuild and from 0.96 to 1.27–1.55 ms per tick. Likely
causes are GIL hand-off, cache migration and eval-breaker checks (Inferred). The other part sits in
the executor's own bookkeeping.

**Main loop drivers.** The wait loop runs 1000–1900 times per wall second. It is driven by the 1 kHz
`/clock` (385–756 callbacks per wall s get through the depth-1 queue), the 1 kHz joint states, the two
timers and the guard conditions. With the rebuild compute removed (`rebuild0`), the loop's share
**rises** to 43 %: it spends the freed time handling more events. Executor overhead therefore scales
with the event rate, not with the filter's compute (Derived).

Scaled to S6B: the replay reproduces 63–71 % of a core at RTF 0.62 where S6B recorded 106 %. It is
81.5 % when pinned to E-cores. The S6B per-thread split (main 28 %, workers 24 % each) has the same
shape as the replay (main 25–27 %, workers 11–17 %), with heavier workers.

### 3.2 Why ticks are late with three threads (Derived; GIL Inferred)

1. **The GIL serialises nearly all of the process's work.** Python bytecode and Pinocchio calls hold
   it, as do small NumPy and SciPy calls (§2.4). OSQP's GIL-free solve is about 0.04 ms per tick,
   about 3.5 % of the tick. GIL-holding CPU is therefore close to the process CPU. In S6B that was
   about 1.0 core over four Python threads, so the GIL was held **almost all the time** (Derived from
   the recorded per-thread CPU). It was then saturated, and any ready callback had to queue for it.
2. **The extra threads add no useful parallelism.** The only GIL-free work, the solve, is too small to
   overlap anything. The three workers and the main loop take turns on one GIL, and CPython 3.10 hands
   it over only at the 5 ms switch interval or on a blocking call. A 1–2 ms tick that becomes ready
   while a 3–6 ms rebuild runs therefore waits for one or more switch intervals (median 2.3–2.6 ms;
   §2.3). It may also lose the GIL again during its own run, for example after OSQP releases it
   (a convoy effect, Inferred).
3. **The premise in `main()` does not hold.** "The QP thread must never wait on perception" is not
   achieved: the tick waits on the rebuild **through the GIL** (Measured: late ticks only with an
   overlap; `rebuild0` reduces them from 263 to 18).
4. **Serialising explicitly is better than serialising implicitly.** With a single thread, a tick that
   becomes ready during a rebuild waits for at most the rest of that rebuild. It then runs without
   interruption, with no hand-off delays and no thread overhead. Measured: 0–1 late ticks, wall equal
   to CPU, and 27–37 % less process CPU.
5. **A shorter switch interval is not a fix.** At 0.5 ms the late ticks fall at RTF 0.62 (195/89 → 74,
   within the run-to-run spread). At RTF 1 on E-cores there is no change (656 → 633).

### 3.3 Solver versus constraint preparation (Measured, cProfile; relative shares under the profiler)

**QP tick** (profiled total 1.13 ms/tick):

| Function | Share |
|---|---:|
| `_solve` | 38 % |
| of which `build_osqp_A` (`csc_matrix`, `identity`, `vstack`, format checks) | 27 % |
| of which OSQP `solve()` itself | 3.5 % |
| `tangential_bias` (`cbf_qp_assembly.py` L340) | 21 % |
| `_report` (rcutils logging with `_find_caller`; 580 lines in 30 s) | 8 % |
| `_publish_status` | 6 % |
| `family_violations` | 5 % |
| `position_velocity_accel_box` | 3 % |
| many `np.clip` and `norm` calls | the rest |

**Rebuild** (profiled total 3.2 ms/rebuild; 12 control points and about 59 `retreat_speed_available`
calls per rebuild):

| Function | Share |
|---|---:|
| `ConstraintBuilder.build` self time | 13 % |
| `_livelock_direction` → `lateral_direction` | 17 % |
| `SelfCollisionRowBuilder.build` (`segment_segment_closest`, 24 per rebuild) | 14 % |
| `latency_compensation_terms` | 10 % |
| `np.median` on small arrays (12 per rebuild) | 9 % |
| `retreat_speed_available` | 8 % |
| `point_jacobian` / `control_point_jacobian` (Pinocchio) | 6 % / 5 % |
| `np.linalg.norm`, `cross`, `sum`, `all`, `clip` | roughly 250 k reductions over 1501 rebuilds |

The cost is Python overhead on 3-vectors, not arithmetic.

**Conclusion.** The solver is not the bottleneck. In the QP tick the dominant avoidable cost is
re-creating an unchanged sparse structure on every tick. In the rebuild the cost is spread out
(Derived).

### 3.4 Rates (Derived)

- **Budget.**
  - Single-threaded at RTF 0.62 (P-cores): 0.72 CPU-s per sim s in all.
  - At RTF 1 pinned to E-cores: 0.89 CPU-s per sim s, i.e. 89 % of one E-core.
  - The 50 Hz rebuild and the 100 Hz QP fit both.
- **Latency.**
  - With one thread the worst extra tick delay is one rebuild: p95 4.0 ms (P) or 7.8 ms (E), against
    a 10 ms period.
  - With three threads, rebuild wall p95 is 10–14 ms. Some ticks then slip by more than half a period
    (§2.3), and a few by more than a period (98.1 ticks per sim s on E-cores at RTF 1).
- **On hardware.** RTF is 1, there is no `/clock`, and the joint states remain at 1 kHz. Removing
  `/clock` lowers the event load. Whether the margin holds on the target computer is Unverified.

### 3.5 Why the replay is milder than S6B (Unverified; candidates)

S6B's in-run costs are about 1.3–2.9× the replay's at the same RTF: build wall 16.8 vs 6.2–10.3 ms;
solve 2.95 vs 1.0–1.6 ms; process 106 % vs 63–82 %. The candidates below are listed in the order in
which the evidence supports them.

1. **CPU clocks.** S6B ran under `powersave` with P-cores at p50 2.07 GHz and E-cores at 1.90 GHz
   while Gazebo loaded the host. The idle-host replay probably ran near turbo (up to 5.3–5.5 / 3.9 GHz;
   not sampled). Pinning the replay to E-cores alone raised the per-call CPU by 1.4–1.8× and the late
   ticks by 2.2–4.8× (Measured), so clock speed is a strong lever (Inferred for S6B).
2. **Core placement.** S6B's threads migrated over all 28 CPUs, E-cores included (Recorded).
3. **Co-location with Gazebo, perception and the other nodes:** cache and memory-bandwidth
   pressure, and real DDS subscribers to `qddot_safe`, `cbf_status` and `/rosout`. None of these is
   present in the replay (Unverified).

None of these changes the mechanism. They scale the per-call cost, and with three threads the late
ticks grow faster than linearly with it (the E-core rows).

---

## 4. Bottlenecks and hypotheses

### 4.1 Confirmed (Measured in the isolated benchmark; Derived where marked)

| # | Bottleneck | Evidence |
|---|---|---|
| B1 | **In-process serialisation under `MultiThreadedExecutor(3)` delays QP ticks while a rebuild runs.** This is the direct cause of the late ticks in the replay. | P(late \| overlap) 7–27 % vs ≤ 0.5 %; tick wait 2.3–2.6 ms vs 0.5–0.6; removing the rebuild compute takes late ticks from 263 to 18; one thread takes them from 195/656 to 0/1 |
| B2 | **Executor overhead is the largest non-callback cost.** The Python wait loop and dispatch, at 1–1.9 k iterations per wall s, are driven by the 1 kHz `/clock` and the 1 kHz joint states. | Main loop 24–43 % of a core; process CPU minus bodies 40–43 % (MT3) vs 27 % (single) |
| B3 | **The threading itself costs CPU.** MT3 uses 18–27 points of a core more than one thread for the same work. | 62.6–71.2 % vs 44.5 % at RTF 0.62 |
| B4 | **Per-tick sparse rebuild of `A`.** About 27 % of the tick's CPU; OSQP itself is about 3.5 %. | cProfile |
| B5 | **Clock speed and core class amplify all of the above.** | E-core pinning: per-call CPU ×1.4–1.8, late ticks ×2.2–4.8 |

### 4.2 Unverified hypotheses

| # | Hypothesis | What would settle it |
|---|---|---|
| H1 | The waits in B1 are **GIL** waits specifically, rather than another shared resource. Derived by elimination; not instrumented. | GIL-wait sampling of the replay (§5, E1) |
| H2 | S6B's larger magnitude (11.6 % missed) comes mostly from low clocks and E-core placement (§3.5). | The replay with per-core frequency sampling, and under a fixed low frequency or co-located with a running Gazebo cell (§5, E2) |
| H3 | OSQP's GIL release adds a re-acquire wait to each tick (convoy). | Per-tick timing of the solve with and without a concurrent rebuild, from the phase-2 raw data plus E1 |
| H4 | The 1 kHz joint-state stream is a large part of the executor load. The `jslow` attempt was invalid. | A valid variant: a throttled relay of `/NS_1/franka/joint_states` at 250 Hz into the replay (harness-only) |
| H5 | The earlier FIFO-priority inversion (docstring, 46 vs 70 Hz) is the same GIL mechanism. | Not needed if the first modification is adopted; otherwise E1 with FIFO on |
| H6 | The 67.8 ms one-off solve in phase 1. | Re-run phase 1 with per-call timestamps |

---

## 5. Recommended measurement to establish causality

**E1: GIL attribution, offline, no production change.** Re-run `qpperf_executor_replay.py`
(`base_r0.62`, `base_r1.0_ecore`) while sampling the process with a GIL-aware profiler. Either
`py-spy record --gil --native` (not installed in the container, so installing it needs approval) or
`perf sched` / `offcputime` on the worker TIDs. Expected if H1 holds: in the ticks that overlap a
rebuild, the waiting time is spent in `take_gil`. Needs no Gazebo.

**E2: magnitude, offline.** Repeat `base_r0.62` and `single_r0.62` while sampling
`/sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq`. Then repeat them at a capped frequency, or
alongside a CPU load that reproduces S6B's 45 % host busy. A frequency cap is a host change and needs
approval. This tests H2 and bounds what to expect in Gazebo.

**E3: closed loop, needs the first modification and a Gazebo approval.** A/B on the S6B cell
(`pentagon_static_baseline_bare / full / s0`, same host policy, unpinned). Run `executor_mode`
`multi` (today) against `single`, at least 3 seeds each. Judge by every metric of protocol §6.2, not
by `ok`:
- primary: ticks per sim s, sim-gap tail, and the late-after-rebuild conditional;
- secondary: process and per-thread CPU (host sampler), build and solve wall, clearance from the
  bag reconstruction (S6B.1), and tracking.

Seeds are needed because the replay spread is about 2× (§2.3).

---

## 6. Candidate optimisations (none implemented)

These keep the CBF equations, geometry and margins unchanged. Ranked by expected benefit relative to
complexity and risk.

| Rank | Change | Benefit | Complexity | Safety risk | Regression testing |
|---:|---|---|---|---|---|
| 1 | **Opt-in `SingleThreadedExecutor`** in `main()`, selected by a parameter; default unchanged | **High** (Measured: late ticks to 0–1; −27–37 % CPU) | Very low (about 10 lines in `main()`) | Low. The callbacks are unchanged. A tick can now wait for at most one rebuild (p95 4–8 ms measured), a bound that is explicit instead of implicit. There are no blocking calls (Verified). With `_elevate_thread_priority` on, it would elevate the only thread: keep it off. | Easy: the replay harness (bit-equal outputs per tick for the same inputs, Derived, since the callback code is identical), then E3 |
| 2 | **Cache the OSQP `A` pattern:** fill `Ax` from `G.ravel()` and the constant identity entries directly, with no `csc`/`vstack` rebuild when `n_rows` is unchanged | Low–medium (about 27 % of tick CPU, about 0.2–0.3 ms per tick) | Low | Very low if the data are bit-identical (unit-testable against `build_osqp_A(G).data`) | Easy: an equality test on recorded `G` |
| 3 | **Host policy:** P-cores and `performance` for the filter | Medium (E-core pinning gave 2.2–4.8× the late ticks) | None in code | None to the control code; it changes the experimental conditions (F-8-10) | Easy, but must be recorded per run |
| 4 | **Diagnostics cadence:** `_report` caller lookup, `_publish_status`, `family_violations` | Low–medium (about 19 % of a tick) | Low | Low for control. **The batch scorer reads CBFDIAG** (L1764), so the evaluation tooling changes. | Medium: scorer parity |
| 5 | **Vectorise the rebuild's per-control-point Python** (`retreat_speed_available`, `np.median` on small arrays, `latency_compensation_terms`, `lateral_direction`, self-collision pairs) | Medium (rebuild perhaps −30–50 %, Inferred) | Medium | Medium: it touches CBF row code, even though the maths is unchanged | Feasible: replay the bag and compare every `ConstraintSnap` bitwise, or to a tolerance |
| 6 | **Vectorise `tangential_bias`** | Low–medium (21 % of a tick) | Low–medium | Medium: it shapes the nominal | As 5, per tick |
| 7 | **Lower the event load:** a throttled joint-state input or a smaller `/clock` rate in sim | Unknown (H4) | Low–medium | Medium: changes the staleness semantics of `joint_states_fast` (L425–431 comment) | Needs a valid E-H4 first |
| 8 | **Rebuild in a separate process** (true parallelism) | High for the tick, if rebuild work grows | High | Medium–high: IPC latency, snapshot age, failure modes | Hard |
| 9 | **Port the hot path to C++** | Highest | Very high | High | Hard |
| — | `sys.setswitchinterval` | Inconsistent (§3.2.5) | Very low | Low | — |
| — | FIFO priority (existing parameter) | Negative in the earlier record (docstring) | — | GIL inversion | — |
| — | `cbf_update_rate_hz` 25 | Little (263 → 230 late) and it changes constraint freshness | — | Changes behaviour | — |

---

## 7. Proposed first modification (**requires explicit approval; not implemented**)

**Change.** In `franka_experiments/nodes/cbf_safety_filter.py::main` (L2070–2081), choose the
executor from a node parameter, for example `executor_mode` (`'multi'` default = today's
`MultiThreadedExecutor(num_threads=3)`; `'single'` = `SingleThreadedExecutor()`). Update the comment
at L2073–2074 to state the measured behaviour. No callback, group, timer, equation, geometry, margin,
gain or default changes.

**Why first.**
- It is the largest measured effect (§2.3).
- It is the smallest diff.
- It is fully reversible by a parameter.
- It is the only way to run E3 and so establish causality in closed loop.

**What it does not do.** It does not make the rebuild cheaper. In single mode a tick can still be
delayed by one rebuild, and §3.5's slow clocks could push a long rebuild near the period. That is why
E3 measures the sim-gap tail and not only the mean rate.

**Validation plan** (each step needs its own approval, per CLAUDE.md):
1. Rebuild `franka_experiments` (copy mode; `check_install_provenance` expectations change).
2. Run the existing unit tests. The failing set must equal the 35 IDs of S5 (F-8-2). The launch
   golden (F-8-1) is unaffected unless the launch passes the parameter.
3. Run the replay harness against the patched install in both modes. `multi` must reproduce
   `base_r*`; `single` must reproduce `single_r*`; per-tick outputs must be equal for equal inputs.
4. Run E3 in Gazebo.

**Rollback.** Set `executor_mode:=multi`, or revert the single commit.

---

## 8. Consequences for later refactoring

- Any change that adds Python work to a callback, or another subscription or timer, raises the GIL
  load for **all** callbacks under MT3. Measure it with the replay harness, not only with `/cbf_status`
  wall times, which mix compute and waiting.
- `/cbf_status`'s `constraint_build_wall_ms` and `solve_wall_ms` are elapsed times. Use them as
  latency, not as cost (S6A.1).
- The replay harness (`qpperf_executor_replay.py`) is a cheap, Gazebo-free regression test for rate
  and CPU. Its absolute numbers depend on clocks (§3.5), so compare like with like.
- Findings: F-8-14 (the executor premise does not hold), F-8-15 (most CPU outside the callback
  bodies; executor event load), F-8-16 (per-tick sparse rebuild); a note on F-8-5.
