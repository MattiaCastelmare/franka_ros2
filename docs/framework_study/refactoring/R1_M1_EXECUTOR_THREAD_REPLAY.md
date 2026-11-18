# R1 / M1: executor-thread replay. Results, mechanism, improvement plan and test plan

> **Status (2026-11-18, C-074).** M1 was approved by the user, run (9/9 valid) and analysed. No
> production, R1, H2 or harness change was made. Nothing is committed.
> - **Labels** (00_INDEX §2): **Verified** (code / checksums), **Runtime (recorded)**, **Derived**
>   (arithmetic on recorded data), **Inferred**, **Unverified**.
> - **Separation:** §1–§4 are facts and their interpretation. §5 is a **proposal** (production
>   changes, none made). §6 is the test plan. §7 records the analysis-tool changes that were
>   actually made and tested.
>
> **Evidence:** `~/franka_baseline_evidence/refactor_r1_m1/`
> - `protocol/`:
>   - `M1_PROTOCOL.md` (sealed before any run, `ac674f7f…`);
>   - Addendum 01 (analysis JSON fix, before the data runs);
>   - Addendum 02 (launch permission, before the data runs);
>   - Addendum 03 (post-data analysis boundary fix, no result change).
> - `scripts/`, `scripts_a01/` (the scripts the runs used), `scripts_a03/`.
> - `setup/` (dry construction), `smoke/`.
> - `runs/`, `logs/`, `host/`: raw data, sealed in `SHA256SUMS_raw_data` (91 files) and
>   write-protected.
> - `analysis/`: `m1_analysis.json` (reported), `m1_analysis_a03.json` (identical), `m1_tables.txt`,
>   `tests/`.
>
> **Tools:** `docs/framework_study/tools/baseline_validation/`: `m1_executor_replay.py`,
> `m1_analyze.py`, `m1_run.sh`, `m1_batch.sh`, `test_m1_tools.py`.

---

## 1. Validity of M1

### 1.1 Design (as sealed)

**Arms:**
- `single`: `SingleThreadedExecutor`;
- `mt1`: `MultiThreadedExecutor(num_threads=1)`, obtained by setting the module constant
  `MULTI_EXECUTOR_THREADS = 1` in the replay process only;
- `mt3`: `MultiThreadedExecutor(3)`, the R1 default.

All three arms are built by the production `cbf_safety_filter.make_executor`.

**Inputs and playback:**
- E3-05's recorded cell bag (sha `8a7ef1e8…`), with its three exact filter parameter files;
- `ros2 bag play --rate 0.70` (the bag's timestamps are sim time; E3-05's episode RTF was 0.698);
- isolated DDS domain 231.

**Repetitions:** 3 per arm in a rotated order: single, mt1, mt3 / mt1, mt3, single / mt3, single,
mt1.

### 1.2 Checks (all Verified on the recorded files; `test_m1_tools.py` Part 2)

| # | Check | Result |
|---|---|---|
| 1 | All nine runs present, each with 5 files | PASS |
| 2 | Execution order = sealed order (pre-check timestamps) | PASS: m1_01 … m1_09, 13:15:36 → 13:29:21 UTC |
| 3 | Identical recorded inputs | PASS: same cell, bag, parameter files (hashes = E3-05's sealed `container_tmp` copies) and rate |
| 4 | Intended executor configuration | PASS: class, worker count, node `executor_mode` and module constant per arm |
| 5 | Same harness version | PASS: `m1_executor_replay.py` `1a690708…` and `r1_executor_replay.py` `eb053c83…` in every run; `scripts/` = `scripts_a01/` except the analysis script |
| 6 | Equivalent constraint workload | PASS: the 264 resolved parameters other than `executor_mode` are identical (`0f58563b…`); `n_rows` median 27 vs E3-05's 25 (gate ±3), mean +1.36; at matched sim times \|Δrows\| ≤ 2 on 97.8–98.1 % of ticks; obstacle in range on 100 % of rows; 0 fault ticks; fallback level 0 on every tick |
| 7 | Valid playback timing | PASS: effective RTF 0.700 (± 2 × 10⁻⁶) in every run |
| 8 | Relevant messages | Distances 840/840 and `qddot_nom` 2798–2800/2800 processed in every run. `/clock` (64–72 %) and joint states (76–86 %) are **not** all processed: depth-1 subscriptions coalesce while the executor is busy, so these drops are part of the executor's behaviour and are reported per arm (§2) |
| 9 | Expected metrics available | PASS: every §6 protocol metric is present for every run |
| 10 | Evidence complete, checksummed, preserved | PASS: raw data sealed and read-only; protocol and scripts sealed; analysis sealed at the end of this session |

The perception input stream is also identical to E3-05's own: both log the same three distance-rate
re-derivations (26.0, 30.3, 36.4 Hz).

### 1.3 The harness corrections and their effect on the meaning of M1

| Correction | When | What | Effect on the experiment |
|---|---|---|---|
| Addendum 01, D-M1-1 | after the smoke run, **before** any data run | Validity entries were numpy booleans, so JSON writing failed; they are now cast to `bool` | None: two lines of the analysis output code, no metric or threshold change. The smoke data were re-analysed, not re-run |
| Addendum 02 | before m1_01 started | The sealed copies were mode 444, so `m1_batch.sh` could not execute `m1_run.sh` (rc 126); they are now mode 555, with unchanged hashes | None: no replay had started and no data existed |
| Addendum 03, D-M1-2 (F-8-47) | **after** the data runs | `missed_slots` assumed exactly 2800 slots and could go negative at the inclusive window edges; it now counts the due grid points in [W0, W1) | None on the results: a recomputation from the immutable data is **identical** (0 differences). Found by a new test (§7) |

None of the three changed the arms, the inputs, the playback, the measurement or a decision rule.

---

## 2. Results (F-8-42, F-8-43; Runtime (recorded); n = 3 per arm; mean ± sd [min, max])

### 2.1 Comparison table

**Sim time (per sim s; window t0 + 2 … t0 + 30 s):**

| Metric | single | mt1 | mt3 |
|---|---|---|---|
| QP ticks per sim s | 100.00 ± 0 | 100.00 ± 0 | 99.98 ± 0.04 [99.93, 100.00] |
| Missed 10 ms slots | 0, 0, 0 | 1, 0, 0 | 1, 2, 0 |
| Late ticks (gap > 15 ms) | 0, 1, 0 | 2, 0, 0 | 3, 2, 0 |
| Tick gap p50 / p95 / p99 (ms) | 10 / 11 / 11 | 10 / 11 / 11 | 10 / 11 / 11 |
| Tick gap max (ms) | 13, 16, 13 | 20, 13, 14 | 19, 23, 13 |
| Constraint snapshot age p50 / p95 (ms) | 3.5 / 11 | 3.5 / 11 | **19 / 24** |
| Rebuilds per sim s | 50 | 50 | 50 |
| `n_rows` p50 | 27 | 27 | 27 |

**Wall clock (per wall s; RTF 0.700):**

| Metric | single | mt1 | mt3 |
|---|---|---|---|
| QP ticks per wall s | 70.00 | 70.00 | 69.99 |
| Rebuild wall p50 (ms) | 2.57 ± 0.13 | **4.10 ± 0.06** | **6.80 ± 0.18** |
| Rebuild CPU p50 (ms) | 2.58 ± 0.13 | 2.97 ± 0.04 | 3.70 ± 0.06 |
| Rebuild wait (wall − CPU) p95 (ms) | 0.00 | 2.42 ± 0.07 | 5.48 ± 0.18 |
| Rebuild wall p95 (ms) | 3.75 | 7.03 | 11.41 |
| Solve wall p50 (ms) | 0.37 ± 0.02 | 0.67 ± 0.02 | 1.20 ± 0.03 |
| Solve CPU p50 (ms) | 0.37 | 0.44 | 0.56 |
| Solve wall p95 (ms) | 0.60 | 1.23 | 4.24 |
| Tick wall / CPU p50 (ms) | 0.95 / 0.95 | 1.40 / 1.11 | 2.32 / 1.42 |
| Tick wait p95 (ms) | 0.00 | 1.67 ± 0.62 [1.26, 2.38] | 5.14 ± 0.19 |
| Process CPU (% of a core) | 49.3 ± 2.4 | 64.1 ± 0.6 | 71.6 ± 0.9 |
| Executor-loop CPU (`wait_for_ready_callbacks`, % of a core) | 16.4 | **28.5** | 25.8 |

**Threads (% of a core):**
- single: 1 thread at 43–47;
- mt1: spin thread 29.5–30.3 and worker 29.2–29.8;
- mt3: spin thread 27.0–27.5 and 3 workers at 13.0–13.7.

**Input processing:**

| Metric | single | mt1 | mt3 |
|---|---|---|---|
| Joint states processed | 0.85 | 0.77 | 0.84 |
| `/clock` processed | 0.72 | 0.65 | 0.70 |
| Joint-state age at the tick, p95 (ms) | 6.9 | 8.8 | 5.3 |

### 2.2 CPU budget per sim second (Derived from the per-callback thread CPU and the replay summary)

| Run(s) | Callbacks (rebuild + tick + inputs) | Executor loop | Remainder (`/clock` callback, dispatch, other) | Process |
|---|---:|---:|---:|---:|
| single (m1_01, 06, 08) | 270–302 ms | 228–247 | 180–194 | **679–743 ms** |
| mt1 (02, 04, 09) | 309–319 | 403–414 | 192–198 | **906–921** |
| mt3 (03, 05, 07) | 395–404 | 366–373 | 246–261 | **1010–1036** |

The executor loop and the remainder (the 1 kHz `/clock` and joint-state traffic) account for about
60 % of the filter's CPU even in `single`. The rebuild is 129–144 ms and the tick 99–110 ms.

### 2.3 Pre-registered mt1-position rule (protocol §8)

f = (mt1 − single) / (mt3 − single), applied only when single and mt3 are separated:

| Metric | Result |
|---|---|
| Rebuild wall p50 | f = 0.36 → intermediate |
| Solve wall p50 | f = 0.37 → intermediate |
| Tick wait p95 | f = 0.32 → like single (borderline) |
| Snapshot age p50 | f = 0.00 → like single |
| Late ticks, missed slots, ticks per sim s, gap p99, P(late \| otherwise) | not separated: no throughput penalty at this load in any arm |

**Pre-registered reading: H-AB.** Both the executor implementation and the worker count contribute.

### 2.4 Answers

1. **Does mt1 behave like single or mt3?** Neither: it is intermediate on callback cost and like
   single on snapshot freshness and tick waiting. Its distinctive signature is the **highest**
   executor-loop CPU (28.5 % vs 16.4 / 25.8) together with the **fewest** input messages processed.
2. **Executor type, worker count, or both? Both, in different ways.**
   - *Executor type* (mt1 − single): +230 ms CPU per sim s, of which +177 ms in the executor loop
     (77 %) and +45 ms in callbacks. Callback wall time rises +1.53 ms (rebuild) and +0.31 ms
     (solve), mostly as wait.
   - *Worker count* (mt3 − mt1): +100 ms CPU per sim s, mostly callbacks (+88 ms); the loop costs
     −35 ms. Callback wall time rises a further +2.70 ms (rebuild) and +0.53 ms (solve). The
     snapshot age goes from 3.5 to 19 ms, **only** with 3 workers.
3. **Which operations account for the extra time?** Both callbacks grow in wall time and in CPU:
   - rebuild: wait 0 → 1.1 → 3.0 ms p50, CPU 2.58 → 2.97 → 3.70 ms;
   - solve path: wait 0 → 0.23 → 0.58 ms, CPU 0.37 → 0.44 → 0.56 ms;
   - executor loop: +12 points under MT.

   The CPU inflation of identical callbacks is real. It also appears in the existing E-core-pinned
   R1 replay (multi rebuild CPU 6.54 vs 5.35 ms), so it is **not** only P/E-core placement.
4. **Does it depend on constraint load?**
   - Rebuild wall time rises with `n_rows` in every arm (single 2.2 → 2.9 ms, mt1 3.6 → 5.2, mt3
     6.1 → 8.2, from ≤ 24 to > 35 rows).
   - The penalty is **multiplicative**: the mt3 / single ratio is about 2.7–2.8 in every band, so
     the absolute penalty grows with load.
   - Late ticks are too few in the replay to show a load dependence.
5. **Consistent across repetitions?** Yes:
   - Within-arm sd is 1–5 % of the mean for every cost metric; the between-arm differences are
     60–165 %.
   - There is no monotonic round effect.
   - Within-run halves vary about ±10 %. m1_06's first half was 2.98 vs 2.50 ms (a host transient).
   - m1_02 has the largest mt1 tick-wait p95 (2.38 ms vs 1.26 / 1.37), with the lowest sampled
     start frequency (1099 MHz); it is noted, not excluded.
6. **Can replay timing explain it?** No. The effective RTF was 0.700 in all nine runs, the inputs
   were identical, and the differences are in the per-operation costs measured inside the node.
7. **Plausible mechanisms:** see §3.

---

## 3. Mechanism evaluation

| Hypothesis | Evidence for | Evidence against / limits | Status |
|---|---|---|---|
| **A. MT dispatch overhead** (thread-pool submit, Task per callback, guard-condition triggers, spin/worker hand-off) | mt1's executor loop costs +12 points of a core while it processes fewer messages (Runtime). rclpy Humble `MultiThreadedExecutor._spin_once_impl` submits every ready handler to a `ThreadPoolExecutor`. The handler triggers the guard condition twice (`executors.py` `_make_handler`), so the spin thread runs Python in parallel with the worker (Verified, rclpy source) | — | **Supported** as one component (Runtime + Verified) |
| **B. Callback concurrency** (several workers running callbacks at once) | mt3 − mt1: callbacks +88 ms CPU per sim s; rebuild wait 1.1 → 3.0 ms; snapshot age 3.5 → 19 ms, only with 3 workers (Runtime) | — | **Supported** as the second component, and the only cause of staleness |
| **B′. GIL serialisation** as the specific waiting mechanism | In-callback off-CPU wait grows with the number of runnable Python threads (1 → 2 → 4). The host was idle (load about 1 on 28 cores), so OS preemption is unlikely. The node has no Python locks (Verified). rclpy's `_tasks_lock` is not held during callbacks (Verified). Pinocchio and small NumPy hold the GIL (`qpperf_gil_probe`, earlier) | **No GIL-wait instrumentation.** Cache-line contention on shared Python objects (reference counts), condition-variable hand-off costs or `with clock.handle` contention cannot be excluded | **Inferred, not proven** |
| **C. Host scheduling / load** | E3 per-operation costs are 2.4× the replay's (rebuild 16.3 vs 6.8 ms under mt3). E-core pinning raises costs 1.4–1.8× (earlier) | M1 ran on an idle host, so it cannot quantify C. The CPU inflation persists under E-core pinning | **Amplifier**, plausible (Inferred) |
| **D. Workload differences** | — | Excluded by design: identical inputs, parameters and rows, verified | **Ruled out** for M1 |
| **Throughput ceiling** (Derived) | Python work is serialised, so a node can use little more than one core of interpreter time. mt3 needs 1.01–1.04 core-seconds per sim second; the existing pinned replay at RTF 1 shows mt3 losing ticks (94/s) while single held 100/s | M1 at RTF 0.7 is below the ceiling for every arm; it shows the costs, not the collapse | **Most plausible chain** for E3 (Inferred) |

**Most plausible explanation of the E3 slowdown (Inferred).** Under `MultiThreadedExecutor(3)`,
the same callbacks cost about 40–50 % more CPU and about 2.6× the wall time. The extra comes from:
- MT dispatch (a Python spin thread running in parallel with the callbacks);
- concurrent callbacks contending for the interpreter, by elimination most likely the GIL.

The filter's demand per sim second then exceeds the serialised interpreter capacity. Gazebo's host
load raises every cost about 2.4×, so in E3 multi hit that ceiling (66.8–89.1 ticks per sim s),
while single (1.3 core-seconds per sim s in E3, period-bound at RTF about 0.66) did not.

**Ruled out or weakened:**
- D (workload) for M1;
- "frequency differences" (identical medians in E3; M1's rate was exact);
- "replay timing";
- "pure E-core placement" (the CPU inflation persists when pinned).

---

## 4. The production execution path (read-only, Verified at HEAD `a756c76`)

### 4.1 Data flow

```
real_time_distance → /cbf/per_link_distances (MultiLinkDistance, BEST_EFFORT, depth 1)
  cbf_safety_filter.py::_on_distances        (grp_io)    Python loop per entry → ObstacleSnap (immutable)
                     ::_track_input_rate                 may call ConstraintBuilder.set_input_rate (mutates builder)
/NS_1/franka/joint_states (1 kHz, depth 1)
  ::_on_joint_state                          (grp_io)    dict(zip) ×2, np.array, array_equal → JointSnap
/NS_1/qddot_nom → ::_on_qddot_nom            (grp_io)    → NomSnap
/clock (sim only, 1 kHz) → rclpy TimeSource.clock_callback (default group) → clock.set_ros_time_override
timer 20 ms → ::_update_constraints          (grp_perc)  self._con = ConstraintBuilder.build(js, obs, now)
  cbf_state_rows.py::ConstraintBuilder.build (L2335)     resets ~58 diag_* fields; Pinocchio update (+J̇);
       singularity row (sets self.diag_sigma L3350); self-collision FK pass + rows; per-CP obstacle rows,
       retreat caps, speed rows → ConstraintSnap (NamedTuple, L1842; built at L3795)
timer 10 ms → ::_qp_tick                     (grp_ctrl)
  ISO stop / stale state / stale nominal / empty-frame / stale-distance branches → brake paths
  build_row_rhs(con, q̇) → zone ladder → state governor (reads self._rows.diag_sigma, L1244)
  → tangential_bias (writes self._rows.diag_tan*, L1317) → evasion / outrun / livelock biases
  → position_velocity_accel_box → apply_slew_limit
  → ::_solve (L1624): pad_rows_to_block → build_osqp_bounds → build_osqp_A (sparse rebuild, every tick)
       → OSQP setup (row-block change) or update → solve → accept_iterate / brake fallback
  → family_violations → ::_publish (qddot_safe) → ::_publish_status → ::_report (CBFDIAG via format_cbf_diag)
executor: cbf_safety_filter.py::make_executor (L2074) → MT(3) | Single ; main() L2115
```

### 4.2 Classified findings

| ID | Location | Finding | Class |
|---|---|---|---|
| F-8-44 | `cbf_safety_filter.py::_qp_tick` L1241–1247; `cbf_state_rows.py::ConstraintBuilder.build` L3350; `ConstraintSnap` L1842 | The state governor's σ is read from the **builder's mutable** `diag_sigma`, not from the snapshot `con` it pairs with `d_sc_min`. Under `multi`, a tick that overlaps a rebuild uses σ from the in-progress build (set early in `build`) with the previous snapshot. It is also stale-by-one in `single` when `_sing_rows.build` raises | **Confirmed defect** (consistency; Verified). Behavioural magnitude Unverified, probably small (σ changes slowly) |
| F-8-45 | `ConstraintBuilder.build` (58 `diag_*` resets); `_qp_tick` L1317–1321, 1345–1348; `logging_utils::format_cbf_diag` | Builder diagnostics are written by both the rebuild and the tick and read by `_report`. Under `multi`, CBFDIAG lines (read by the batch scorer) can mix two snapshots | **Confirmed defect**, diagnostic integrity only, `multi` only (Verified) |
| F-8-46 | `_on_distances` → `_track_input_rate` → `ConstraintBuilder.set_input_rate` | The evidence gates of the builder are mutated from the I/O callback, which under `multi` can run during a `build()`. Rare: 3 re-derivations per run in M1 and in E3-05 | Confirmed hazard (Verified); consequence Unverified |
| B4 / F-8-16 | `cbf_qp_assembly.py::build_osqp_A` L73–84, called by `_solve` every tick | Sparse `identity` + `csc_matrix` + `vstack` per tick, although the pattern is fixed per row block. About 27 % of tick CPU in the earlier profile | **Measured bottleneck** (cProfile) |
| B2 / F-8-43, F-8-15 | rclpy executor + 1 kHz `/clock` + 1 kHz joint states | The executor loop plus remainder are about 60 % of the filter CPU even in `single` (§2.2) | **Measured bottleneck** |
| B1 / F-8-14 | `make_executor` (MT(3)) + three `MutuallyExclusiveCallbackGroup`s, L421–423 | Groups do not prevent rebuild, tick and I/O overlap; overlap inflates every callback and makes the tick use the previous snapshot (19 ms) | **Measured** (M1, E3) |
| F-8-17 | Timer ordering | Under single or mt1 the rebuild runs before the tick at shared due times only because of rclpy's ready-order; nothing in the node guarantees it | Confirmed design property (Verified, Runtime) |
| — | `_report` / `_publish_status` / `family_violations` | About 19 % of a tick (earlier profile); `_report` formats CBFDIAG at `diag_period_s` | Measured cost, low priority |
| — | `ConstraintBuilder.build` per-CP Python loops | About 3 ms CPU per rebuild uncontended; grows with rows (§2.4.4) | Measured cost |
| — | Stale-data handling | Each callback swaps an immutable snapshot by attribute assignment, which is atomic under the GIL; the tick reads `self._con` / `self._obs` once per tick | Verified: no torn snapshot; the only inconsistencies are F-8-44 / 45b / 46 |
| — | GC pauses | `diag_disable_gc` exists; never measured in E3 / M1 | Unverified hypothesis |

---

## 5. Candidate improvements (PROPOSALS; none implemented; each needs its own approval)

The ranking weighs benefit by evidence strength, then complexity and risk. Smallest first.

### P1. Make `executor_mode: single` the default

- **Location:** `franka_experiments/config/fr3_control.yaml:317` (`executor_mode: multi`), plus the
  sim-condition overrides (`franka_simulation_experiments/config/conditions.yaml`, `73ed807`).
- **Problem:** B1 / F-8-14; E3 66.8–89.1 vs 99.0–99.8 ticks per sim s; M1 rebuild 2.6× and solve
  3.3× the single wall time; snapshot age 19 vs 3.5 ms.
- **Evidence:** E3 3 pairs (Runtime), M1 9 runs, the R1 replay A/B (20 runs), and the
  bit-identical-callback argument of R1 §8.
- **Change:** a one-line default. R1 already implements and tests both modes
  (`test_cbf_executor_mode.py`).
- **Expected benefit:**
  - the QP rate held at target in sim;
  - fresh constraints;
  - −15 to −22 points of CPU.
- **Drawbacks:**
  - one thread serialises everything, so a tick can wait up to one rebuild (E3: gap p99 15–18 ms,
    max ≤ 31 ms);
  - joint-state age at the tick is slightly higher than mt3's (6.9 vs 5.3 ms p95 in M1);
  - **headroom:** in E3, single used 1.3 core-seconds per sim s at RTF about 0.66. At RTF 1 under
    that load it could not keep 100 Hz (Derived). On an idle host it needs about 0.69 core-seconds
    per sim s (M1).
  - The FIFO priority option stays refused in single.
- **Safety risk:** low. The control equations are unchanged (R1). Timing becomes more regular,
  never less, in every measurement so far. It is not validated with moving obstacles or on
  hardware.
- **Real-time risk:** low in sim. On hardware it is unknown: no `/clock`, real 1 kHz joint states.
- **Tests:** §6 A1–A3, B1–B7, C1–C6, D1–D6, E1–E5.
- **Acceptance:** §6.

### P2. Carry σ in the snapshot (fixes F-8-44)

- **Location:** `cbf_state_rows.py::ConstraintSnap` (new field, default `nan`) and its constructor
  at L3795 (`sigma_min=self.diag_sigma`); `cbf_safety_filter.py::_qp_tick` L1244 (read
  `snap.sigma_min`).
- **Problem:** the governor pairs a σ and a `d_sc_min` from different builds under `multi`, and
  after a singularity-row exception under `single`.
- **Change:** about 3 lines; no maths change.
- **Benefit:** snapshot consistency independent of the executor.
- **Drawbacks:** none known.
- **Safety risk:** very low. The value is identical in the normal single path.
- **Tests:**
  - a new unit test: the tick uses the σ of the snapshot it uses, even with a builder σ that has
    moved on;
  - replay parity: under `single`, every published `qddot_safe` is bit-identical to the current
    code on the M1 bag, except after a singularity-row exception (none in M1: 0 faults).
- **Acceptance:** both pass; no `/cbf_status` field changes on the M1 bag under `single`.

### P3. Fill OSQP `Ax` directly (B4 / F-8-16)

- **Location:** `cbf_qp_assembly.py::build_osqp_A`, and `_solve` L1694–1699 (the `update(Ax=…)`
  path).
- **Problem:** sparse matrix construction every tick, about 27 % of tick CPU (earlier profile).
- **Change:** precompute the CSC pattern per row block and fill `data` from `G` (column-major) plus
  the constant identity entries. Keep `build_osqp_A` for `setup()`.
- **Benefit:** about 0.2–0.3 ms per tick (Derived from the profile), i.e. 20–30 ms CPU per sim s
  (about 3–4 % of single's budget). This is **the** largest solve-path saving.
- **Drawbacks:** a pattern-order bug would corrupt constraints.
- **Safety risk:** medium if wrong, very low if bit-identical. Gate it on an equality test.
- **Tests:**
  - A5: `data` bit-equal to `build_osqp_A(G).data` for every recorded `G` and every row block;
  - B: QP solutions bit-identical on the M1 bag under `single`;
  - C: tick CPU p50 reduced.
- **Acceptance:** bit-identity on 100 % of recorded ticks, and a tick CPU p50 reduction ≥ 0.1 ms,
  outside the replay noise (sd 0.06 ms).

### P4. Make the rebuild → tick order explicit (F-8-17)

- **Location:** `cbf_safety_filter.py` timers L525–528.
- **Change:** one 100 Hz timer whose callback runs `_update_constraints()` on every second tick
  before the QP. Alternatively, keep two timers but assert the order.
- **Benefit:** freshness that no longer depends on rclpy's ready-order (age ≤ 10 ms by
  construction) and determinism.
- **Drawbacks:**
  - the tick that rebuilds becomes about 3 ms longer;
  - it changes the timing semantics (a rebuild exactly at tick time, with the freshest joint state).
- **Safety risk:** low to medium (timing changes).
- **Tests:** B, C and D in full, including Gazebo pairs.
- **Priority:** after P1–P3, only if C2/C5 show a residual ordering problem.

### P5. Reduce event load (B2)

- **Problem:** the executor loop plus `/clock` / joint-state traffic are about 60 % of the CPU.
- **Options:**
  - on hardware `/clock` disappears by itself;
  - a joint-state decimation upstream changes staleness semantics (the L425–431 comment) and
    needs an experiment first.
- **Priority:** measure first (an offline replay with a decimated joint-state stream; a new
  protocol and approval needed). Do not change it blind.

### P6. Builder-diagnostics ownership (F-8-45, F-8-46)

Move the tick-written `diag_tan*` / `diag_ga*` to node attributes, and apply `set_input_rate` from
inside `_update_constraints`. This is moot for control if P1 lands, but it keeps `multi` honest.
Low priority.

### Not recommended (evidence against)

- More worker threads: worker count is the main cost driver (M1).
- FIFO priority: a GIL inversion in the earlier record.
- `sys.setswitchinterval`: inconsistent effect.
- Lowering `cbf_update_rate_hz`: little gain, and it changes freshness.

---

## 6. Test plan (before any production change)

**Common justification for thresholds:**
- **QP rate:** the E3 gate R15 uses ±5 % of the 100 Hz target (≥ 95 ticks per sim s). Single
  achieved 99.0–99.8 in E3 and 100.0 in M1, so a change must not lower single's rate by more than
  the E3 between-run spread (about 1 tick per sim s).
- **Timing metrics:** compared against the per-arm sd measured here (rebuild sd 0.13 ms, solve sd
  0.02 ms, tick sd 0.06 ms). A difference counts only if it exceeds 2 × the larger sd **and** the
  ranges do not overlap: the M1 rule, n ≥ 3.
- **Safety thresholds** are the existing E3 gates: d_safe 0.10 m via R14, no contact via R12,
  zero faults or fallbacks via R10 / R11, reflex PASS via R08.

**Legend for "Where":** Off = offline (pytest, replay or analysis on recorded data); Gz = needs
Gazebo; Auth = needs a new approval. Running `pytest` or `colcon` needs approval under CLAUDE.md.

### A. Mathematical correctness (P2, P3, any refactor)

| # | Verifies | How | Expected / threshold | Where |
|---|---|---|---|---|
| A1 | Kinematics and Jacobians unchanged | existing `test_kinematics_fix_regression.py`, `test_control_point_derivatives.py` | all pass (today's pass set; S5 baseline, F-8-2) | Off, Auth (pytest) |
| A2 | Distances and CBF rows unchanged | `test_cbf_state_rows.py`, `test_multi_obstacle_rows.py`, `test_cbf_hard_constraints.py`; plus a **new** snapshot-equality replay: every `ConstraintSnap` field bit-equal on the M1 bag, old vs new code | 100 % bit-identical (the code paths are deterministic; R1 §8 showed bit-identity under the sequential replay) | Off, Auth |
| A3 | QP constraints, joint, acceleration and slew limits, margins | `test_cbf_row_padding.py`, `test_accel_box_decoupling.py`, `test_brake_slew_limited.py`, `test_fr3_velocity_envelope.py` | all pass | Off, Auth |
| A4 | Slack behaviour | `test_cbf_row_l1_slack.py`, `test_cbf_risk_weighted_slack.py` | all pass | Off, Auth |
| A5 | (P3) `Ax` fill equals `build_osqp_A(G).data` | a new unit test over all recorded `G` (M1 bag, every row block) and random `G` | bit-equal 100 % | Off, Auth |
| A6 | Published command identical | `r1_sequential_outputs.py`-style executor-free replay, old vs new: `qddot_safe` per 10 ms slot | max \|Δ\| = 0 (P2, P3); for P1, the executor-induced deviation must not exceed the single-vs-single noise floor of R1 §8 | Off, Auth (new replay run) |

### B. Functional correctness

| # | Verifies | How | Expected / threshold | Where |
|---|---|---|---|---|
| B1 | Executor construction and mode validation | `test_cbf_executor_mode.py` | pass | Off, Auth |
| B2 | Stale joint state, nominal or distance → brake | `test_safety_failure_policy.py`, `test_cbf_qp_fallback.py` | pass | Off, Auth |
| B3 | Constraint updates use the snapshot consistently (P2) | a new unit test: builder σ ≠ snapshot σ, and the governor uses the snapshot's | pass | Off, Auth |
| B4 | Command publication every tick, including the fallback paths | the existing smoke `smoke_cbf_safety_filter.py` (5 repetitions, R1 §7) | pass 5/5 | Off, Auth |
| B5 | Fallback ladder | `test_cbf_qp_fallback.py`; replay with injected QP failure (`osqp_max_iter` low; a harness parameter only) | brake level on failure, never the nominal | Off, Auth |
| B6 | Startup | node start in both modes; warm-up before timers; executor log line | first tick ≤ 1 period after the first state | Off, Auth |
| B7 | Shutdown | spin interrupted → `destroy_node`, no traceback | clean exit | Off, Auth |

### C. Scheduling and performance

| # | Verifies | How | Expected / threshold | Where |
|---|---|---|---|---|
| C1 | Target QP frequency | the M1 harness (this protocol), new arm vs `single` baseline, 3 reps, rotated | ticks per sim s ≥ 99.0 at RTF 0.7 (M1 single 100.0) | Off, Auth (new replays) |
| C2 | Missed deadlines and gap tails | the same | missed ≤ 2 per 28 s; gap p99 ≤ 11 ms, max ≤ 20 ms (M1 single max 16) | Off, Auth |
| C3 | Rebuild duration | the same | p50 not above single's 2.57 ms + 2 sd | Off, Auth |
| C4 | Solve duration | the same | P3: p50 ≤ 0.37 − 0.1 ms; others not above 0.37 + 2 sd | Off, Auth |
| C5 | Constraint freshness | the same | snapshot age p50 ≤ 3.5 ms and p95 ≤ 11 ms | Off, Auth |
| C6 | CPU | the same | process CPU not above single's 49 % + 2 sd at RTF 0.7 | Off, Auth |
| C7 | Load sensitivity | the same harness at RTF 1.0, and E-core-pinned (as R1 §8); M3 synthetic load | single: ≥ 99 ticks per sim s at RTF 1 pinned (R1: 100.0); report the margin | Off, Auth (M3 not yet approved) |
| C8 | Closed loop | E3/H2 pairs on V2 (`pentagon_static_baseline`, furnished) and V4 (`pentagon_wander_close`, seeds 1–2), ABBA/BAAB | R15 PASS (≥ 95 Hz) in single; the pair comparison as E3 | **Gz, Auth**, after the F-8-41 addendum for moving cells |

### D. Safety regression

| # | Verifies | How | Expected / threshold | Where |
|---|---|---|---|---|
| D1 | Constraints not weakened | A2 + A5 bit-identity; a `ConstraintSnap` row count per tick equal | identical | Off, Auth |
| D2 | No increase in violations | `/cbf_status` `n_violated`, `violation_*` columns on replay; R12 / R14 in Gazebo | no row with \|Δ\| > 0 offline; R14 lower bound ≥ 0.10 m in every Gazebo run | Off + Gz, Auth |
| D3 | No stale or inconsistent constraints | C5 + B3; under `multi`, an attribution of ticks using the previous snapshot (`r1_snapshot_attribution.py`) | single: 0 ticks older than one rebuild period | Off, Auth |
| D4 | Fallbacks enabled | B2 / B5; `fallback_levels` distribution on replay | level 0 on 100 % of nominal ticks; brake on injected failure | Off, Auth |
| D5 | Safety gates not bypassed | ISO paths (`test_iso_*`), empty-frame and distance-stale branches unchanged (diff review + tests) | pass; no diff in those branches | Off, Auth |
| D6 | No new instability | E3 tracking \|e\| p95, reflex R08, `qddot_safe` jerk (slew bite counts) | \|e\| p95 within the E3 pair spread (±5 mm); R08 PASS | Gz, Auth |

### E. Experimental reproducibility

| # | Verifies | How | Expected | Where |
|---|---|---|---|---|
| E1 | Identical inputs | `test_m1_tools.py::TestRecordedEvidence::test_identical_parameters_and_inputs`, `test_parameter_files_are_the_e3_05_files` | pass | Off (done) |
| E2 | Identical parameters except the variable | parameter-dump hash per run | equal | Off (done) |
| E3 | Controlled executor configuration | `test_executor_configuration_per_arm` | pass | Off (done) |
| E4 | Repeatable measurement | within-arm sd ≤ 5 % of the mean for cost metrics | pass (1–5 %) | Off (done) |
| E5 | Preserved evidence and provenance | checksum tests, write protection, sealed protocol and addenda, reproducibility of the analysis | pass | Off (done) |

---

## 7. Implemented analysis-tool changes and tests actually executed

**Changed (analysis and experimental tooling only):**
- **New** `m1_executor_replay.py`, `m1_run.sh`, `m1_batch.sh`, `m1_analyze.py` (pre-data, sealed).
- `m1_analyze.py` fixes D-M1-1 (Addendum 01, pre-data) and D-M1-2 (Addendum 03, post-data, with
  no result change).
- **New** `test_m1_tools.py` (31 tests).
- `README.md` rows.

No production, R1, H2, E3-harness or protocol file was changed (§8 of the checkpoint).

**Executed** with `python3 -I -X dev docs/framework_study/tools/baseline_validation/test_m1_tools.py -v`
(host, stdlib `unittest` + numpy; no ROS, no pytest):

| Suite | Tests | Result |
|---|---:|---|
| Synthetic metric semantics | 9 | PASS |
| Validity gates (each rejects its case) | 5 | PASS |
| Window boundary (D-M1-2 regression) | 1 | PASS after the fix (FAIL, −1, before) |
| Arm summary / mt1 rule | 3 | PASS |
| Recorded evidence | 13 | PASS |
| **Total** | **31** | **31 PASS, 0 FAIL, 0 SKIP, 0 warnings** (dev mode) |

**Mutation check** (the same tests against the historical script versions):
- sealed original `scripts/m1_analyze.py`: 19 of 31 fail (every synthetic test hits D-M1-1);
- `scripts_a01/m1_analyze.py`: 1 of 31 fails, exactly the D-M1-2 test.

The tests therefore detect both known defects.

**Not executed:** the existing `franka_experiments` pytest suites. They are not needed for M1 (no
production change), and CLAUDE.md requires approval before running pytest. They are part of the
§6 plan for any production change.

---

## 8. Final assessment

**A. What M1 demonstrated (Runtime, offline, idle host, one bag).**
- With identical inputs, parameters and constraint workload, the executor configuration alone
  changes the per-operation cost of the filter, in steps:
  - rebuild 2.6 / 4.1 / 6.8 ms;
  - solve 0.37 / 0.67 / 1.20 ms;
  - CPU 0.69 / 0.92 / 1.02 core-seconds per sim s.
- The concurrency of three workers alone makes the constraints about 15 ms staler.
- At RTF 0.7 on an idle host none of the arms loses QP ticks.
- **This is a performance result, not a safety or avoidance result.**

**B. Best configuration:** `single`, on every cost, freshness and CPU metric, with equal QP rate.

**C. Most plausible cause of the original slowdown:** §3. The MT executor's parallel Python
dispatch plus concurrent callbacks inflate every operation (by elimination most plausibly GIL
contention). That pushes the filter's serialised demand past what one interpreter can deliver
once Gazebo's host load multiplies the costs.

**D. Evidence for it:**
- the stepwise cost increase with the number of runnable Python threads (M1);
- in-callback wait on an idle host;
- the CPU inflation also under E-core pinning (R1);
- the throughput loss when the load rises (R1 RTF 1 pinned, E3);
- single's period-bound behaviour in every setting.

**E. Unproven:**
- that the waits are GIL waits (needs M2 or another direct measurement);
- the size of host-load amplification (M3);
- any closed-loop effect beyond the one static scene (E3 only);
- the behaviour on hardware;
- whether F-8-44 has any measurable behavioural effect.

**F. What to improve first:** the executor choice (P1). Then the snapshot consistency (P2, which
is independent of the executor). Then the per-tick `A` rebuild (P3).

**G. Exact recommended modification:** P1 as a single-line default change,
`fr3_control.yaml:317` `executor_mode: multi` → `single`, together with the matching sim-condition
defaults. Implement it only after the §6 tests below are approved. P2 next, as a separate step.

**H. Tests that must pass for P1:**
- A6 (output deviation within the single-vs-single noise);
- B1, B2, B4, B5;
- C1–C7;
- D2, D4, D5;
- E1–E5;
- and, before calling it validated, C8 / D2 / D6 in Gazebo on V2 and V4.

**I. Additional experiments needed:**
- M2 (GIL attribution; needs a py-spy install);
- M3 (host-load sensitivity; `single` headroom at RTF 1 under load);
- the V2 / V4 Gazebo pairs (after the F-8-41 addendum);
- a hardware timing check (no `/clock`).

**J. Next step:** review this report. Then decide between (i) approving P1 with the §6 offline
tests (A6, B, C1–C6 on the M1 harness) and (ii) approving M3 first if `single`'s headroom under
load is the deciding concern. M2 is useful for the mechanism claim but is **not** required to
choose P1, because P1 rests on measured performance, not on the GIL attribution.

| Proposed change | Expected benefit | Risk | Required tests | Priority |
|---|---|---|---|---|
| P1 `executor_mode: single` default | QP rate held in sim (E3 99.5 vs 79.5); fresh constraints (3.5 vs 19 ms); −15 to −22 CPU points | Low (control code unchanged); headroom at RTF 1 under load and on hardware unverified | A6, B1–B5, C1–C8, D2–D6, E1–E5 | **1** |
| P2 σ in the snapshot (F-8-44) | Executor-independent snapshot consistency | Very low | A2, A6, B3, D1 | **2** |
| P3 direct `Ax` fill (B4) | About 0.2–0.3 ms per tick, 3–4 % of single's CPU | Medium if the pattern is wrong; bit-equality gated | A5, A6, C3–C6, D1 | **3** |
| P4 explicit rebuild → tick order | Freshness by construction, determinism | Low to medium (timing semantics) | B, C, D full + Gazebo | 4 (only if needed) |
| P5 event-load reduction | Up to about 60 % of the CPU is event handling | Medium (staleness semantics) | a new offline experiment first | 5 (measure first) |
| P6 diagnostics ownership (F-8-45, F-8-46) | Honest CBFDIAG under `multi` | Low | `test_cbf_diag_line.py`, scorer parity | 6 |

---

## 9. Blockers and approvals required

- **Any production change (P1–P6):** a new refactoring step (R2, …) on its own branch, approved by
  the user.
- **Running the `franka_experiments` pytest suites, or `colcon build`:** approval (CLAUDE.md).
- **New offline replays** (C1–C7 for a change; P5 measurement; M3): a new protocol and approval.
- **M2:** approval to install py-spy in the container.
- **Gazebo** (C8, D2, D6; V2 / V4): approval, plus the F-8-41 pre-registration addendum for
  moving obstacles.
- No blocker remains for the M1 analysis itself: all acceptance criteria of this session are met
  (§7, checkpoint C-074).

## Consequences of modification

- R1's default (`multi`) is unchanged. M1 strengthens the case for changing it but does not change
  it.
- `m1_analyze.py` is the analysis of record from Addendum 03 on. Its result is identical to the
  sealed Addendum 01 analysis.
- Any change to `ConstraintSnap` (P2) must update every consumer of the NamedTuple, the CBFDIAG
  formatter and the tests that construct snapshots.
- P1 changes the timing regime of every sim condition that does not override `executor_mode`.
  Earlier sim results (all `multi`) are not directly comparable with new ones; record the mode per
  run.
