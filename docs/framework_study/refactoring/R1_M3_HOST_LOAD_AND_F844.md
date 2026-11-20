# R1 / M3: executor configuration under controlled host load, the regression suites, and F-8-44

> **Status (2026-11-20, C-075).** Approved by the user (message 12): commit M1, run the offline
> regression suites, design and run M3, investigate F-8-44.
> - **Not done:** R2 is not implemented; the executor default is unchanged; no production, R1 or
>   H2 code changed; no Gazebo, robot or py-spy; no CPU-setting change; nothing pushed.
> - **Labels:** Verified / Derived / Inferred / Unverified / Runtime (recorded) (00_INDEX §2).
> - **Separation:** §1–§5 are measurements and their reading. §6 is the F-8-44 analysis. §7 holds
>   the proposals and the decision answers; none of them is implemented.
>
> **Evidence:** `~/franka_baseline_evidence/refactor_r1_m3/`.
>
> | Directory | Content | Seal |
> |---|---|---|
> | `00_regression_tests/` | Regression suites | sealed, read-only |
> | `01_design_probe/` | Load-only probe | checksummed |
> | `protocol/` | `M3_PROTOCOL.md` + `ADDENDUM_01_thermal_guard_and_restart.md` | sealed |
> | `scripts/` | Original harness | sealed |
> | `scripts_a01/` | Addendum-01 harness | sealed |
> | `smoke/` | Not counted | checksummed |
> | `attempt0/` | Aborted at m3_01; not counted | sealed, read-only |
> | `attempt1/` | The 18 counted runs: `runs/`, `host/`, `logs/`, `analysis/` | `SHA256SUMS_raw_data` (181 files) and `SHA256SUMS_analysis`, read-only |
> | `02_f844/` | F-8-44 test, logs, results | sealed, read-only |

## 1. Phase 1: M1 committed

- **Inspection.** The uncommitted changes were only the M1 tools, the M1 tests, the M1 report, and
  the PROGRESS / chapter 16 / CHECKPOINTS edits.
  - The production tree is unchanged since R1 / H2 (`git diff HEAD -- . ':!docs'` empty).
  - The only non-doc commits since R1 are `73ed807` and `186b823`, which existed before.
- **Evidence.** All 9 M1 runs are valid. The sealed raw data, analysis, protocol and scripts
  checksums verify.
- **Tests.** `test_m1_tools.py` passes 31/31.
- **Commits** (local only, the user as author, no trailer):
  - `cc029db` (tools);
  - `8e0d004` (docs, C-074).

## 2. Phase 2: offline regression suites (Runtime, recorded)

- **Command and environment:** the S5 / R1 command and environment (container, installed
  workspace; `franka_experiments` install == source, verified per file). No build was needed.
- **Evidence:** `00_regression_tests/`.

| Suite | Failed / skipped / passed | Baseline | Failed set | Lost passes |
|---|---|---|---|---|
| 1 `franka_experiments` pytest (1449) | 35 / 9 / 1405 | R1: 35 / 9 / 1405 | identical; all 65 assertion lines byte-identical | none |
| 2 `franka_simulation_experiments` (not slow) | 0 / 0 / 179 (22 slow = Gazebo, deselected) | H2: 0 / 0 / 179 | identical | none |
| 3 `franka_simulation/test/sim` | 0 / 0 / 39 | R1: 0 / 0 / 39 | identical | none |
| Smoke `smoke_cbf_safety_filter.py` | 3 / 3 PASSED | R1 5 / 5 | — | — |

- **The 35 failures are the known baseline set** (F-8-2), not regressions:
  - 25 unwired specs (F-0-3);
  - 8 stale launch golden (F-8-1);
  - 1 private rclpy API (F-7-9, `test_node_self_references`);
  - 1 outdated test (F-7-7).
- **Priority areas, all passing** (suite 1, by module):
  - CBF maths and rows: 123;
  - QP construction and slack: 69;
  - constraint handling: 108;
  - executor configuration: 22 / 22;
  - safety fallbacks: 51;
  - governor: 26;
  - callbacks, snapshots and stale data: 61 pass and 1 known failure (F-7-9).
- **One harness incident:** attempt 0 of the runner used `set -u`, and the colcon `setup.bash`
  aborted on it before any test ran. This is recorded in `attempt0_setup_failure.txt`. The flag was
  removed, and the suites then ran once.

## 3. Phase 3: M3 design (sealed `protocol/M3_PROTOCOL.md`)

- **Arms:** single / MT(1) / MT(3), built by the production `make_executor` through the unchanged
  M1 replay.
- **Inputs:** E3-05's bag and its three parameter files, rate 0.70. The window is [t0+2, t0+30] s,
  as in M1 / E3.
- **Independent variable:** host load.
  - L0: idle.
  - L1: 12 pure-Python CPU spin workers (`m3_load.sh`: nice 0, own session, no pinning, 330 s
    `timeout` cap).
  - The target is E3's measured non-filter load, 11.2–12.4 busy cores (E3 total 12.5–13.9).
  - **Not equivalent to Gazebo:** no memory, cache, iGPU or DDS pressure.
- **Order:** 3 repetitions per cell (18 runs). Blocks L0 L1 L1 L0 L0 L1; within each load, each arm
  takes each position once.
- **Host sampler** (`m3_host_sampler.py`, 0.5 s): per-core `/proc/stat`, frequency, package
  temperature, monotonic + epoch time. It is cut to the replay window through the shared
  CLOCK_MONOTONIC.
- **Validity:** the M1 replay gates, plus load gates (L0 ≤ 3.0 busy cores; L1 11.5–15.0 and 12
  workers alive at the start and end of the replay).
- **Hypotheses H1–H5** with pre-registered rules (§5).
- **Design probe** (load only): 12.1 busy cores, P / E p50 2.8 / 2.3 GHz, at most 61 °C.

## 4. Phase 4: execution, deviations and validity

| Event | What happened | Effect |
|---|---|---|
| Smoke (not counted) | mt3 under L1: 73.5 ticks per sim s, rebuild 17.2 ms | The pipeline worked end to end |
| **Attempt 0** | Aborted at m3_01 (single, L0). The single-sample thermal guard (≥ 95 °C) fired on one 0.5 s spike (60 → 73 → **98** → 60 °C) at replay start-up on an **idle** host. The replay itself completed (rc 0) | Batch stopped per protocol §8. Data kept in `attempt0/`, not counted. m3_01 (100 ticks per sim s, rebuild 2.53 ms) agrees with attempt-1 single / L0 |
| **Addendum 01** (user's choice) | Guard fires on ≥ 95 °C in 4 consecutive samples (2 s); new unit tests (incl. the exact spike); full restart as attempt 1 with `scripts_a01/` | Harness only; no measurement or gate change |
| **Attempt 1** | 18 / 18 runs exit OK, in the sealed order | — |
| **Host suspend (unplanned)** | systemd-logind suspended the host 2026-11-18 16:08:27 → 2026-11-19 12:00:17 UTC (`journalctl`), **inside the 60 s pause** after m3_17 (ended 16:07:50). m3_18 began 20 s after resume. No replay was running during the suspend | m3_18 passed every gate (13.26 busy cores, 58 °C). Its values lie inside the other mt3 / L1 runs (ticks 86.8 vs 68.7 and 84.4). Excluding it leaves every hypothesis verdict unchanged (H4: 76.5 ticks per sim s). Kept, as registered |

**Validity (Runtime, `attempt1/analysis/m3_analysis.json`): 18 / 18 valid.**
- RTF 0.700 in every run.
- `n_rows` within ±3 of E3-05; matched-time \|Δrows\| ≤ 2 on 97.2–98.4 % of ticks.
- Obstacle in range on 100 %; 0 fault ticks.
- Host busy cores: L0 0.54–0.85; L1 13.0–13.9.
- 12 workers alive at the start and end of every L1 replay.
- At most 72 °C.
- **0** stale or brake WARN lines in any window. The 5 start-up "qddot_nom stale" lines per run are
  all outside the window.

## 5. Phase 5: results (Runtime, recorded; mean [min, max], n = 3 per cell)

**QP rate and timing:**

| Metric | L0 single | L0 MT(1) | L0 MT(3) | **L1 single** | **L1 MT(1)** | **L1 MT(3)** |
|---|---|---|---|---|---|---|
| QP ticks per sim s | 100.00 | 100.00 | 100.00 | **99.94** [99.86, 100.00] | **89.39** [82.86, 93.86] | **79.95** [68.68, 86.82] |
| QP ticks per wall s | 70.0 | 70.0 | 70.0 | 69.97 | 62.6 | 56.0 |
| Missed 10 ms slots | 0 | 0 | 0 | 2 [0, 4] | 298 [172, 482] | 580 [384, 903] |
| Late ticks (> 15 ms) | 0 | 0 | 0 | 6 | 425 | 560 |
| Gap p50 / p95 / p99 / max (ms) | 10 / 11 / 11 / 13 | 10 / 11 / 11 / 13 | 10 / 11 / 11 / 14 | 10 / 11 / 12 / 21 | 10 / 20 / 25 / 34 | 10 / 27 / 43 / 76 |
| Gaps ≥ 50 ms | 0 | 0 | 0 | 0 | 0 | 14 [1, 35] |

**Per-operation cost:**

| Metric | L0 single | L0 MT(1) | L0 MT(3) | **L1 single** | **L1 MT(1)** | **L1 MT(3)** |
|---|---|---|---|---|---|---|
| Rebuild wall / CPU p50 (ms) | 2.40 / 2.41 | 3.80 / 2.81 | 6.12 / 3.40 | 8.95 / 8.95 | 12.53 / 10.33 | 16.82 / 10.66 |
| Rebuild wait p95 (ms) | 0.0 | 1.9 | 4.5 | 0.1 | 4.6 | 12.0 |
| Solve wall / CPU p50 (ms) | 0.34 / 0.35 | 0.62 / 0.41 | 1.01 / 0.51 | 0.98 / 1.00 | 2.08 / 1.22 | 2.76 / 1.27 |
| Tick wait p95 (ms) | 0.0 | 1.4 | 4.3 | 0.02 | 6.0 | 29.5 |

**Freshness and inputs:**

| Metric | L0 single | L0 MT(1) | L0 MT(3) | **L1 single** | **L1 MT(1)** | **L1 MT(3)** |
|---|---|---|---|---|---|---|
| Snapshot age p50 / p95 (ms) | 3.5 / 11 | 3.5 / 11 | 16 / 23 | 1.3 / 11 | 0 / 15 | 20 / 33 |
| `n_rows` p50 | 27 | 27 | 27 | 27 | 25.7 | 25 |
| Joint states / `/clock` processed | 0.86 / 0.73 | 0.79 / 0.67 | 0.86 / 0.72 | 0.48 / 0.43 | 0.19 / 0.17 | 0.39 / 0.36 |
| Joint-state age p95 (ms) | 6.6 | 7.9 | 5.0 | 12.1 | 17.0 | 9.4 |

**CPU, host and headroom:**

| Metric | L0 single | L0 MT(1) | L0 MT(3) | **L1 single** | **L1 MT(1)** | **L1 MT(3)** |
|---|---|---|---|---|---|---|
| Process CPU (% of a core) | 45.7 | 59.7 | 66.3 | 87.8 | 107.6 | 108.1 |
| Executor-loop CPU (%) | 15.4 | 26.7 | 24.8 | 16.9 | 36.2 | 24.5 |
| Python-callback-thread cores | 0.41 | 0.55 | 0.62 | 0.85 | **1.04** | **1.05** |
| Projected saturation rate (Derived) | 1.71 | 1.27 | 1.13 | **0.83** | 0.67 | 0.67 |
| Host busy cores | 0.61 | 0.73 | 0.79 | 13.1 | 13.5 | 13.5 |
| P / E frequency p50 (MHz) | 800 / 800 (idle cores) | 800 / 800 | 807 / 800 | 2535 / 2169 | 2322 / 2065 | 2270 / 2055 |

**E3 Gazebo comparison** (H2 runs 05–10, each in its own window; Runtime):

| | E3 multi (3 runs) | **M3 L1 MT(3)** | E3 single (3 runs) | **M3 L1 single** |
|---|---|---|---|---|
| Ticks per sim s | 79.5 [66.8, 89.1] | 79.95 [68.7, 86.8] | 99.5 [99.0, 99.8] | 99.94 [99.86, 100.0] |
| Rebuild p50 (ms) | 16.3 | 16.8 | 8.6–9.3 | 8.95 |
| Solve p50 (ms) | 2.94–3.01 | 2.76 | 1.05–1.17 | 0.98 |
| Filter CPU (%) | 105–110 | 108 | 91–97 | 88 |
| Snapshot age p50 (ms) | 20–21 | 20 | 0 | 1.3 |
| Host busy cores / P-core p50 (MHz) | 12.7–13.8 / 2100 | 13.5 / 2270 | 12.7–13.9 / 2100 | 13.1 / 2535 |

### Pre-registered hypotheses

| | Verdict |
|---|---|
| H1 (single holds its rate under load) | **Supported**: 99.96, 99.86, 100.00, all ≥ 95 |
| H2 (loss order mt3 > mt1 > single) | **Supported by mean** (loss 20.0 > 10.6 > 0.06). **Neither step is separated** by the M1 rule: mt3 vs mt1 ranges overlap; mt1 vs single ranges do not overlap, but \|Δ\| = 10.5 < 2 × sd 5.8 |
| H3 (the gap grows under load) | **Supported** for rebuild wall, solve wall, tick wait, missed slots, late ticks and rebuild CPU, for both MT arms (growth > 2 SE). Snapshot age grows for mt3 only. Process CPU grows for mt1 only. The *ratio* mt3 / single for rebuild falls (2.55 → 1.88) while the absolute gap doubles (3.7 → 7.9 ms) |
| H4 (E3 degradation reproduced) | **Reproduced**: mt3 79.95 < 95 (76.5 without m3_18) |
| H5 (load inflates cost) | **Supported in every arm**: rebuild CPU ×3.7 / ×3.7 / ×3.1; solve CPU ×2.9 / ×3.0 / ×2.5; tick CPU ×3.3 / ×3.3 / ×2.9. All separated |

**mt1 position.**
- L0: rebuild and solve intermediate (f 0.38 / 0.42); tick wait and freshness like single, as in M1.
- L1: late ticks like mt3 (f 0.76); ticks per sim s, missed slots, gap p99, rebuild and solve
  intermediate (f 0.42–0.62); tick wait and freshness like single.

### Answers A–I

- **A. Does single maintain its QP rate under load? Yes** (Runtime): 99.86–100.00 ticks per sim s,
  ≤ 4 missed slots, gap p99 12 ms, max ≤ 26 ms.
- **B. Does mt3 lose throughput before mt1? By mean, yes** (80.0 vs 89.4 ticks per sim s; 580 vs
  298 missed slots). It is not separated by the pre-registered rule, because run-to-run spread under
  load is large (mt3 68.7–86.8).
- **C. Does mt1 lose throughput before single? Yes** (89.4 vs 99.9; ranges disjoint). It narrowly
  fails the 2-sd part of the rule.
- **D. Does the gap grow under load? Yes**, in absolute terms, for every timing metric (H3). At L0
  there is no throughput gap; at L1 there is a 10–20 tick per sim s gap.
- **E. What becomes the bottleneck?**
  - Measured:
    - Under load, every callback's CPU cost rises 2.5–3.7× in every arm (the P-cores ran at a
      median 2.3–2.5 GHz instead of turbo).
    - The Python callback threads of both MT arms reach **1.04–1.05 cores**, against 0.85 for single.
    - The executor loop stays flat for single (15 → 17 %) and mt3 (25 → 25 %) but rises for mt1
      (27 → 36 %).
    - Both MT arms are inflated by an extra rebuild and solve **wait** (p95 4.6 / 12.0 ms) that
      single does not have.
  - Inferred: the binding limit is the serialised Python callback demand against a capacity of about
    one interpreter core. The MT arms saturate (projected saturation rate 0.67 < 0.70 playback);
    single stays below it (0.83).
- **F. Is freshness affected? Yes, for mt3 only:**
  - mt3 snapshot age p50 16 → 20 ms and p95 23 → 33 ms;
  - single and mt1 stay ≤ 11 / 15 ms p95.

  Input coalescing rises in every arm under load (joint states processed 48 / 19 / 39 %).
- **G. Is the E3 degradation reproduced? Yes.** Offline, with host CPU load alone, mt3 reproduced
  E3's multi rate (79.95 vs 79.5 ticks per sim s) and its per-operation costs within about 3–8 %,
  and single reproduced E3's single.
  - Gazebo-specific causes (physics, rendering, closed loop) are therefore **not needed** to explain
    the E3 slowdown (Derived from the agreement; Inferred as an explanation).
  - **What still differs from Gazebo:** an open loop; no physics; no other DDS nodes; no
    memory / iGPU pressure; P-core 2.3–2.5 vs 2.1 GHz.
- **H. Headroom (Derived)** from the Python-callback-thread cores at playback 0.70:
  - single saturates at about **1.71×** real-time-factor 0.7 equivalents on an idle host, i.e. at a
    playback rate of about 1.7, and at about **0.83** under the L1 load;
  - MT(1) / MT(3) at about 0.67 under load, which is already below the 0.70 that was played.

  Under E3-like host load, **single is unlikely to hold 100 Hz at RTF 1.0** (Derived and untested).
  Gazebo itself runs at RTF ≈ 0.63–0.70, so the E3 regime has about 18 % margin.
  - The assumption (Python threads ≤ 1 core) is approximate: the MT arms measured 1.04–1.05 because
    OSQP and some NumPy release the GIL.
- **I. Mechanism: a combination.**
  - **Host effects (measured):** host load reduces core frequency and adds contention, which
    multiplies every operation's CPU cost about 3×.
  - **Executor overhead (measured):** the MT loop costs +10 to +20 points of CPU at L0 (mt1 the
    most).
  - **Callback contention (measured as wait):** extra in-callback wait grows with workers and with
    load.
  - **Throughput loss** appears when the serialised Python demand reaches about 1 core (measured
    1.04–1.05). That is consistent with interpreter serialisation (Inferred), but **GIL contention
    is not proven**, because no GIL instrumentation was used.

## 6. Phase 6: F-8-44 (the governor's σ vs the snapshot's `d_sc_min`)

### 6.1 Read / write sequence (Verified, code at HEAD)

**Writer** (rebuild thread, `grp_perc`): `_update_constraints` → `ConstraintBuilder.build`
(`cbf_state_rows.py`):
1. L2336: return `None` on a stale distance.
2. L2343: `self._kin.update(…)`.
3. Obstacle rows.
4. L3347: `self._sing_rows.build(q, q̇)`, which sets `last_sigma` (`cbf_singularity.py` L176).
5. **L3350: `self.diag_sigma = last_sigma`**, only if the row loop completed without an exception.
6. L3356+: the self-collision FK pass (`self._sc_kin.update`) and the self-collision rows, … more
   families.
7. **L3795: `return ConstraintSnap(…, d_sc_min, …)`**. The snapshot has no σ field (L1842).
8. **The node assigns `self._con = …`** (`cbf_safety_filter.py` L961).

**Reader** (tick, `grp_ctrl`), `_qp_tick`:
1. `con = self._con` (one read per tick).
2. L1241: `snap = con if n_c > 0`.
3. **L1244: `sigma = getattr(self._rows, 'diag_sigma')`**; **L1246: `d_sc = snap.d_sc_min`**.
4. `self._gov.weight(…)`.
5. If `w < 1` and the binding term is `'sing'`: the directional fade uses `snap.A` rows of group
   `G_SING`, from the snapshot.

`diag_sigma` is never reset per build. It is initialised to NaN in `__init__` (L2283).

### 6.2 Answers (test `docs/framework_study/tools/f844/test_f844_governor_sigma_snapshot.py`, real node and builder, E3-05 parameters; Runtime, `02_f844/run2.log`)

| Question | Answer | Evidence |
|---|---|---|
| 1. Can the two values come from different builds? | **Yes** | **T1**: a tick placed deterministically inside build B, after L3350 and before L3795, receives σ = 0.086 (build B) with `d_sc` = 0.1175 from snapshot A (σ_A = 0.070) |
| 2. Which executor configurations? | **MT with ≥ 2 workers** (MT(3), the default): rebuild and tick are in different groups and run concurrently. **single and MT(1)** serialise callbacks: consistent on the normal path (**T2**). **But single is also exposed on the exception path:** when `_sing_rows.build` raises, `diag_sigma` keeps the older build's σ while the new snapshot is used (**T3**: σ 0.070 paired with snapshot B) | T1–T3; code |
| 3. Can it change the governor's command? | **Yes.** **T4** (demonstration pair σ 0.070 vs 0.086): w = 0.50 vs 0.97, nominal difference up to 0.94 rad/s². That pair needs 7.3 rad/s in 20 ms, beyond FR3 limits, so it is a mechanism demonstration only. **Realistic bound (T7, Derived):** within one rebuild period, \|Δσ\| ≤ 0.0062 at the FR3 velocity limits → \|Δw\| ≤ 0.23; at 0.5 rad/s per joint, \|Δw\| ≤ 0.044. **Asymmetry (Derived from `StateGovernor.weight`):** a too-low σ tightens at once (conservative); a too-high σ can relax w by at most dt / `resume_s` = 0.01 / 0.5 = **0.02 per tick** | T4, T7; `state_governor.py` |
| 4. Can it affect safety-related behaviour? | **The constraints are not affected:** the QP is solved with the snapshot's own rows, including its singularity row (**T5**). The governor shapes only the nominal (the QP target). The effect is a task-level attenuation that is slightly too strong or (ramp-limited) too weak near a singularity or self-collision band. **In E3** the sampled σ_min (CBFDIAG, every run) stayed ≥ 0.091 > 0.09 (floor + band), so the σ term never engaged on a sampled line | T5; E3 CBFDIAG (Runtime) |
| 5. Deterministic reproduction? | **Yes**, single-threaded, with no executor (T1, T3). **T6**, the specification "σ belongs to the snapshot it uses", is an **expected failure** today | `run2.log`: 7 tests, 6 pass, 1 expected failure, 0 skipped |

**Test history (recorded):**
- The first run skipped every test: the pose search on q6 could not reach σ = 0.07, because σ_min
  along q6 is at least 0.0565.
- The search was moved to q4 (σ from 0.143 down to 0.0006) before any result existed. Assertions and
  thresholds were unchanged.
- Both versions and the probe script are kept in `02_f844/`.

**Classification:** a **correctness (coherence) defect** of the snapshot model, independent of
executor performance. It has low safety impact: constraints are untouched, and the permissive
direction is ramp-limited. It is not observed in the recorded E3 scene.

### 6.3 Smallest correction (proposal P2; not implemented)

1. `ConstraintSnap`: add a trailing field `sigma_min: float = float('nan')`. A trailing default
   keeps both construction sites (`cbf_state_rows.py` L3795 and `test_cbf_tangential_bias.py`) valid.
2. `ConstraintBuilder.build`:
   - keep a local `sigma_build = nan`;
   - set it to `last_sigma` where L3350 sets `diag_sigma`;
   - pass `sigma_min=sigma_build` at L3795.

   Keep `diag_sigma` for CBFDIAG (P6 is separate).
3. `_qp_tick` L1244: `sigma=(snap.sigma_min if snap is not None else None)`.

**On the exception path** σ becomes NaN, meaning "no opinion" (`StateGovernor.weight` docstring)
instead of an older σ. This is a deliberate, documented behaviour change. Failing closed instead
would be a separate decision.

**Equivalence checks:**
- **Mathematical:**
  - on the normal single path, σ_snapshot ≡ `diag_sigma` at every tick;
  - executor-free sequential replay (`r1_sequential_outputs.py`) of the M1 bag, old vs new: every
    published `qddot_safe`, every `_solve` result and `n_rows` must be **bit-identical**.
- **Behavioural:**
  - T6 passes;
  - T1 must then show snapshot coherence (its expectation inverts, which is documented);
  - T3 shows NaN, not the old σ;
  - T2, T4, T5 unchanged;
  - suite 1 failing set = the 35 IDs;
  - `test_state_governor`, `test_directional_fade` and `test_cbf_tangential_bias` pass.
- Under `multi`, behaviour changes only on interleaved ticks, which is intended.

## 7. Phase 7: decision report (recommendations; nothing implemented)

### 7.1 Facts

- **M1:** the executor alone changes per-operation cost (single < MT(1) < MT(3)). No rate loss on an
  idle host.
- **M3:**
  - under E3-like CPU load, single holds 100 Hz;
  - MT(1) loses about 11 % of the rate and MT(3) about 20 %;
  - MT(3) matches E3's rate, costs and staleness;
  - freshness degrades only in MT(3).
- **Regression suites:** identical to the R1 / H2 baselines.
- **F-8-44:** reproduced deterministically. MT(≥ 2) on the normal path, and single on the
  singularity-row exception path. The command can change; the constraints cannot.

### 7.2 Remaining uncertainties

**Performance:**
- headroom at RTF 1.0 under load (projected about 0.83 saturation for single, untested);
- hardware (no `/clock`; real 1 kHz joint states; no Gazebo load);
- the GIL attribution (M2);
- only one scene / bag.

**Safety and correctness:**
- no closed-loop validation with moving obstacles (F-8-41 gate addendum pending);
- F-8-44 (coherence);
- F-8-45 / F-8-46 (multi-only diagnostics and gate races);
- F-8-20 / F-8-21 (start-up race, self-mask blind zone) are independent of the executor.

### 7.3 Answers

1. **Is single sufficiently validated offline for R2?**
   - **Yes for performance** in the E3 regime (RTF about 0.7, idle and E3-like load: M1, M3), and
     for equation identity (R1 §8–§9) and the regression suites.
   - **No for headroom at RTF 1.0 under load**, and no closed-loop safety evidence with moving
     obstacles.
2. **Mandatory Gazebo tests before changing the default:**
   - first, the F-8-41 addendum (v_obs in g_min; \|v_obs\| × 50 ms in the conservative bound);
   - then counterbalanced single / multi pairs (≥ 3 seeds) on **V2** (furnished static) and **V4**
     (`pentagon_wander_close`, moving), using the E3 gates: R15 ≥ 95 Hz (single), R14 clearance
     ≥ 0.10 m (bound), R12 no contact, R10 / R11 zero faults and fallbacks, R08 reflex PASS, and
     tracking \|e\| p95 within ±5 mm of the pair partner.
3. **Should F-8-44 be fixed before R2? Not required.** single removes the normal-path mismatch;
   only the exception path remains. Fix it as its **own** step (P2, after or alongside R2 on a
   separate branch). It changes behaviour only under multi and on the exception path.
4. **Is M2 necessary? No** for the R2 decision: R2 rests on measured performance, not on the GIL
   attribution. M2 is optional, for the mechanism claim.
5. **R2 should contain exactly:**
   - `franka_experiments/config/fr3_control.yaml:317` `executor_mode: multi` → `single`;
   - the sim-condition defaults that pin `multi`, if any (`conditions.yaml`, `73ed807`);
   - the default-expectation tests in `test_cbf_executor_mode.py` (`test_shipped_default_is_multi`
     and similar), updated with the reason, because the shipped default is what they assert;
   - documentation and a record `R2_*.md`.
6. **Keep separate:**
   - P2 (F-8-44);
   - P3 (OSQP `Ax`);
   - P4 (explicit order);
   - P5 (event load);
   - P6 / F-8-45 / F-8-46 (diagnostics and gates);
   - the F-8-41 gate addendum;
   - a suspend inhibitor for future campaigns (§4);
   - any FIFO or thread-priority change.
7. **R2 acceptance criteria (objective; thresholds from the recorded baselines):**
   - **Suites:**
     - suite 1 failing set = the 35 F-8-2 IDs; the only changed expectations are the documented
       default tests;
     - suites 2 / 3 unchanged;
     - smoke 5 / 5.
   - **Equation identity:** sequential-replay outputs bit-identical old vs new; no control code
     changes.
   - **Offline M3 harness, default build:** L0 and L1 ticks per sim s ≥ 99.0 every run (M3 single
     min 99.86); missed slots ≤ 10; gap p99 ≤ 12 ms; snapshot age p95 ≤ 11 ms.
   - **Gazebo (item 2):** every single run R15 PASS, R14 / R12 / R10 / R11 / R08 PASS, \|e\| p95
     within ±5 mm.
   - The executor mode is recorded per run.

### 7.4 Smallest safe next implementation step

- **R2 as a configuration-only change** on its own branch: the one-line default, plus the
  default-expectation tests and the record.
- Verify it offline against §7.3 item 7 (suites, sequential identity, the M3 harness at L0 / L1).
- **Do not merge** until the F-8-41 addendum and the V2 / V4 Gazebo pairs pass.
- P2 (F-8-44) follows as a separate small step, with a bit-identity check under single.

Each step needs the user's approval.

| Proposed change | Expected benefit | Risk | Required tests | Priority |
|---|---|---|---|---|
| R2: `executor_mode: single` default | Rate held under E3-like load (99.9 vs 80 ticks per sim s), fresh constraints (≤ 11 vs 33 ms p95), −20 CPU points | Low (no control code); headroom at RTF 1.0 and on hardware unverified | Suites, sequential identity, M3 harness L0 / L1, Gazebo V2 / V4 pairs | 1 |
| P2: σ in `ConstraintSnap` (F-8-44) | Coherent governor input in every mode | Very low; deliberate NaN on the exception path | F-8-44 T1–T7, bit-identity under single, suites | 2 |
| F-8-41 gate addendum | Valid gate for moving obstacles | None (analysis) | Gate unit tests on recorded cells | 2 (before the Gazebo pairs) |
| P3: OSQP `Ax` fill | −0.2 to −0.3 ms per tick (more headroom) | Medium if wrong; bit-equality gated | A5 / A6 bit-equality, timing | 3 |
| Suspend inhibitor in the campaign harness | No unplanned pauses | None | Harness test | 3 |
| M3b: rate 1.0 under L1 (single) | Measures the projected headroom limit | None (offline experiment) | New protocol | 3 (before hardware) |

## 8. Implemented tool changes and tests executed

**New tools:**
- **M3** (`tools/baseline_validation/`): `m3_load.sh`, `m3_host_sampler.py`, `m3_run.sh`,
  `m3_batch.sh`, `m3_analyze.py`, `test_m3_tools.py`;
- **F-8-44** (`tools/f844/`): `test_f844_governor_sigma_snapshot.py`.

**Results:**

| Suite | Result |
|---|---|
| `test_m1_tools.py` | 31 / 31 pass (dev mode, 0 warnings) |
| `test_m3_tools.py` | **21 / 21 pass**, 0 skip, 0 warnings. Mutation check: against the original sampler, the 2 new guard tests fail (errors) and the rest pass |
| `test_f844_governor_sigma_snapshot.py` | **6 pass + 1 expected failure (T6, the defect), 0 skip** (run 2). Run 1: 7 skipped (q6 search, §6.2); not counted as passing |
| Package suites | §2 |

**Noted artefact:** `scripts_a01/__pycache__/` was created when the sealed copy's tests ran
(2026-11-18 15:28 UTC, before attempt 1). It is an added cache directory; every sealed file still
verifies.

## Consequences of modification

- M3 makes the executor default the dominant performance lever in the E3 regime. R2 changes the
  timing regime of every condition that does not override `executor_mode`, so record the mode per
  run.
- P2 changes the `ConstraintSnap` interface: every constructor, the CBFDIAG formatter and the tests
  that build snapshots must be checked.
- Future campaigns on this laptop need a suspend inhibitor (`systemd-inhibit`). That is a harness
  change, which needs approval.
