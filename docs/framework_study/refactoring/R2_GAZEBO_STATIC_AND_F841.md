# R2 Gazebo static validation and the F-8-41 harness correction (C-078)

Message 16, 2026-11-20. Branch `refactor/r2-single-executor-default`. The system under test is HEAD
`6d5caca`.

The scope covered:
- R2 committed locally;
- one sealed static Gazebo campaign of 4 runs;
- the F-8-41 correction, in the experimental harness only.

The scope excluded:
- production CBF, QP or perception changes;
- moving-obstacle runs;
- the robot;
- retries;
- push or merge.

Labels follow `00_INDEX.md` §2. These runs are **preliminary evidence**: 2 pairs, one static scenario.
**The R2 offline verdict stays NOT_REPRODUCED and the A_L1 deviation stays unresolved (F-8-60).**
Nothing here changes either.

## 0. Summary

| | Result |
|---|---|
| STEP 1, commits | `cadaf62` configuration, `359dd7c` tests, `6d5caca` docs and tools. User authorship only; not pushed, not merged |
| STEP 2, protocol | `05_gz_static/protocol/R2GZ_PROTOCOL.md` sealed 17:00:53Z, before any run. Pre-flight: 4 arms resolved; arms differ only in the executor keys; 18 of 18 committed files match the install; INSTALLED == SOURCE |
| STEP 3, campaign | 17:00:56–17:10:42Z, order D M M D. **4 of 4 runs complete and GATE_CONTINUE; both pairs COMPARABLE, with pair checks P1–P6 PASS.** No abort, no retry. Campaign sealed |
| STEP 4, pre-registered answers | Q1 clearance preserved: **NO**. Q2 QP timing: YES. Q3 staleness: YES. Q4 oscillation and tracking: YES. Q5 safety: YES. **Q6 supports the moving phase: DOES_NOT_SUPPORT** (rule: Q1 NO) |
| Why Q1 is NO | D's minimum clearance was 9.3 mm (seed 0) and 7.1 mm (seed 1) **below** M's, against a 5 mm tolerance. Both arms stayed ≥ 16 mm above d_safe = 0.10 m, with the continuous lower bound ≥ d_safe and no contact |
| STEP 5, F-8-41 | Corrections C1–C6 and rule R23 implemented in a harness copy (`06_f841/scripts_h3/`). Host tests 17 of 17; container tests 10 of 10. Static equivalence is exact on 10 sealed runs. Threshold **0.2991 m**, not 0.2975. Addendum sealed, **not executed** |

## 1. Commits (STEP 1)

The tree was clean before the commits, and the stashes were untouched (3). The installed copies of
both YAML files hash-equal the source.

| Commit | Content |
|---|---|
| `cadaf62` config | `fr3_control.yaml` `executor_mode: single` (+comment); `conditions.yaml` `full` pin `single` |
| `359dd7c` test | `test_cbf_executor_mode.py`, `test_conditions.py`, `smoke_cbf_safety_filter_shipped_executor.py` |
| `6d5caca` docs | C-077 documentation (with the user's message-16 decision recorded in F-8-60 and report §6.3), `r2_*` tools |

Sealed evidence was not modified.

## 2. Protocol and pre-run verification (STEP 2)

The sealed protocol is `~/franka_baseline_evidence/refactor_r2/05_gz_static/protocol/R2GZ_PROTOCOL.md`,
SHA-256 `c8122e91…`, sealed at 2026-11-20T17:00:53.88Z. The batch started 3 s later.

- **Harness.** The H2 phase-aligned harness, unchanged, from sealed copies of `scripts_h2/`. On top
  of it:
  - `r2gz_run.sh` / `r2gz_analyze.sh` / `r2gz_resolve.py`, copies of `e3_*` with only the arm, root
    and branch changed;
  - `r2gz_extra.py`: oscillation and summary metrics;
  - `r2gz_pair_check.py`: identity checks P1–P6;
  - `r2gz_summary.py`: Q1–Q6 with constants fixed in the protocol;
  - `r2gz_batch.sh`: order, stop rules, `systemd-inhibit`.
  Repository copies are in [`tools/gz_validation/`](../tools/gz_validation/README.md).
- **Arms.**
  - **D** = `default`: no `--set`; `full` resolves to `single` through the R2 pin.
  - **M** = `multi`: `--set executor_mode=multi`, 3 workers.
  - Order: 01 s0 D, 02 s0 M, 03 s1 M, 04 s1 D (counterbalanced).
- **Identity (Verified, pre-flight and per pair).**
  - Resolved specs differ only in `cbf_params.executor_mode`, `stack_args.cbf_param_overrides` and
    `toggles.cbf_params.executor_mode`.
  - Seeds differ only in seed and `start_offset_s`.
  - Obstacle: static, p0 (0.55, −0.25, 0.449), r = 0.05.
  - q0 (0, −0.7, 0, −2.35, 0, 1.57, 0.7); T = 30 s; scene `diag_bare`.
  - Per pair:
    - P1: launch parameters equal apart from the executor;
    - P2: cell YAML and URDF equal;
    - P3: manifest equal and git SHA equal, not dirty;
    - P4: filter start-up lines equal, so the CBF parameters are identical;
    - P5: commander parameters equal;
    - P6: the executor lines differ.
- **Install.** Committed == installed for 18 files; INSTALLED == SOURCE.
- **Validity.** Unchanged H2 rules R01–R22 and V1–V7, including:
  - V4: g0 ≥ 0.1875 m;
  - R12 / R14: reconstructed minimum and continuous lower bound ≥ d_safe;
  - R13: truth vs reconstruction;
  - R19: phase start.
- **Abort rules.**
  - Any run rc ≠ 0, or analysis rc ≠ 0, stops the campaign.
  - Safety rules R08 (reflex), R10 (fault), R11 (fallback), R12 (contact), R14 (clearance) are named
    in the log.
  - A pair that is not COMPARABLE, or that fails the pair check, stops the campaign.
  - No automatic retry.
- **Pre-registered answers.** Protocol §7, constants in `r2gz_summary.py`:
  - D_SAFE 0.10 m;
  - Q1 clearance tolerance 5 mm (D ≥ M − 5 mm);
  - Q2 ≥ 95 ticks/sim s, D > M and no gap ≥ 50 ms;
  - Q3 snapshot age p95 ≤ 20 ms and D < M;
  - Q4 oscillation ratios ≤ 1.15 × M and tracking p95 ≤ M + 10 mm. The tracking criterion was revised
    from ±5 mm to +10 mm before any data, from the E3 spread.

## 3. Execution (STEP 3)

`logs/batch.txt` (Runtime, recorded): each run took about 2 min; each analysis about 25 s.

| Run | Run rc | Gate | Validity | g0 [m] | Non-PASS rules |
|---|---|---|---|---|---|
| 01_s0_default | 0 | CONTINUE | VALID | 0.4197 | R09, R16, R21 (RECORD) |
| 02_s0_multi | 0 | CONTINUE | VALID | 0.4197 | R09, R15 RATE_FAIL, R16, R21 (RECORD) |
| 03_s1_multi | 0 | CONTINUE | VALID | 0.4197 | R09, R15 RATE_FAIL, R16, R21 (RECORD) |
| 04_s1_default | 0 | CONTINUE | VALID | 0.4197 | R09, R16, R21 (RECORD) |

- **Pairs.** s0 and s1 are both COMPARABLE (`e3_h2_compare.py`), and the pair checks pass.
- **Runtime state.**
  - Host: governor powersave × 28, EPP performance (unchanged, as in E3).
  - Batch under `systemd-inhibit`.
  - At the end: 0 inhibitor locks, 0 container ROS processes.
- **Sealing.** Each run directory carries its own `SHA256SUMS` (all four verified after the
  campaign). `SHA256SUMS_campaign` covers 31 files: pairs, summary, logs, host_env and batch stdout.
  Its SHA-256 is `043ebfba…`, sealed 17:11:47Z, read-only.

## 4. Results and the pre-registered answers (STEP 4)

From `summary/r2gz_summary.txt` (Runtime, recorded; window t0+2 s to the end):

| Metric | s0 D | s0 M | s1 D | s1 M |
|---|---|---|---|---|
| QP ticks per sim s | **99.43** | 65.05 | **98.34** | 60.75 |
| Missed slots | 16 | 979 | 46.6 | 1099.6 |
| Tick gap p95 / p99 / max [ms] | 15 / 17 / 36 | 46 / 67 / 109 | 16 / 19 / 31 | 55 / 90 / 126 |
| Gaps ≥ 50 ms | 0 | 80 | 0 | 103 |
| Snapshot age p50 / p95 / max [ms] | 0 / 14 / 19 | 21 / 37 / 50 | 0 / 15 / 19 | 21 / 37 / 66 |
| Reconstructed minimum clearance [m] | **0.1161** | 0.1254 | **0.1181** | 0.1252 |
| Continuous lower bound [m] | 0.1155 | 0.1249 | 0.1175 | 0.1246 |
| Truth minimum [m] | 0.1161 | 0.1254 | 0.1181 | 0.1252 |
| Reversals per command, safe (nominal) | 0.133 (0.079) | 0.179 (0.079) | 0.152 (0.084) | 0.179 (0.075) |
| HF power fraction > 10 Hz | 0.007 | 0.006 | 0.009 | 0.004 |
| Command step p99 / jerk p99 | 1.10 / 106 | 1.90 / 169 | 1.23 / 110 | 1.82 / 159 |
| Smoothness verdict | PASS | PASS | PASS | WATCH |
| Pentagon p95 / tracking p95 [m] | 0.124 / 0.126 | 0.140 / 0.139 | 0.135 / 0.134 | 0.130 / 0.130 |
| RTF | 0.708 | 0.693 | 0.677 | 0.697 |
| Filter CPU [% of one core] | 96.0 | 110.2 | 96.6 | 110.2 |
| Build / solve wall p50 [ms] | 9.7 / 1.15 | 16.6 / 3.09 | 10.4 / 1.24 | 16.8 / 3.04 |
| Rebuilds per sim s | 50.0 | 33.8 | 50.0 | 31.8 |
| Reflex, fault ticks, fallback | PASS, 0, full only | PASS, 0, full only | PASS, 0, full only | PASS, 0, full only |

The answers below are as computed by the sealed rules.

1. **Clearance preserved? NO.**
   - The sub-criteria R12 / R14 PASS and lower bound ≥ d_safe were met.
   - The sub-criterion "D ≥ M − 5 mm" failed in both pairs: D − M = −9.3 mm (s0) and −7.1 mm (s1).
2. **QP timing improved? YES.** D held 98–99 ticks per sim s with no gap ≥ 50 ms. M ran at 61–65.
3. **Staleness reduced? YES.** Snapshot age p95: D 14–15 ms, M 37 ms.
4. **Oscillations or tracking degradation? NO, so the criterion is met (YES).**
   - Every D oscillation and smoothness metric is at or below M.
   - Tracking p95, D − M: −13 mm (s0) and +5 mm (s1); the limit is +10 mm.
5. **Safety constraints respected? YES.** In every run:
   - no reflex, fault or fallback below `full`;
   - no sample below d_safe;
   - the lower bound ≥ d_safe.
6. **Proceed to moving-obstacle validation? DOES_NOT_SUPPORT**, by the pre-registered rule
   "Q1 or Q5 NO".

## 5. Post-hoc analysis (not pre-registered; it does not change §4)

All of this was done after the data was seen. It is labelled, and it changes no verdict.

**Same-extractor reference values for E3.** `r2gz_summary` printed `None` for the E3 references
(F-8-64). `r2gz_extra.py` was run unchanged on the sealed E3 runs, writing into a new folder
`refactor_r2/05b_gz_static_posthoc/`; the E3 runs themselves were not written.

| Run | Ticks | Gap p95 / p99 [ms] | Snapshot p95 [ms] | Minimum clearance [m] | Reversals per command | Jerk p99 | Pentagon p95 [m] |
|---|---|---|---|---|---|---|---|
| E3-06 single s0 | 99.0 | 15 / 18 | 15 | 0.1202 | 0.151 | 105 | 0.153 |
| **R2-01 default s0** | 99.4 | 15 / 17 | 14 | 0.1161 | 0.133 | 106 | 0.124 |
| E3-08 single s1 | 99.8 | 13 / 15 | 13 | 0.1195 | 0.150 | 104 | 0.131 |
| **R2-04 default s1** | 98.3 | 16 / 19 | 15 | 0.1181 | 0.152 | 110 | 0.135 |
| E3-09 single s2 | 99.7 | 12 / 15 | 12 | 0.1188 | 0.134 | 100 | 0.131 |
| E3-05 multi s0 | 66.8 | 48 / 67 | 37 | 0.1217 | 0.172 | 178 | 0.148 |
| **R2-02 multi s0** | 65.0 | 46 / 66.5 | 37 | 0.1254 | 0.179 | 169 | 0.140 |
| E3-07 multi s1 | 82.7 | 31 / 48 | 35 | 0.1189 | 0.158 | 117 | 0.125 |
| **R2-03 multi s1** | 60.8 | 55 / 90 | 37 | 0.1252 | 0.179 | 159 | 0.130 |
| E3-10 multi s2 | 89.1 | 22 / 36 | 35 | 0.1189 | 0.147 | 133 | 0.137 |

- **Derived: the default arm reproduces E3's explicit single arm.**
  - It matches timing, staleness, smoothness and CPU.
  - So the R2 default really runs `SingleThreadedExecutor`; P6 and R05 also verify that.
- **Derived: clearance spread.**
  - Across 10 static runs, the run minima span 0.1161–0.1254 m (9.3 mm).
  - Within each arm, per pass (4 passes per run): sd 1.5 mm (E3 single), 2.4 mm (E3 multi), 3.7 mm
    (R2 default), 1.2 mm (R2 multi).
  - The 5 mm Q1 tolerance was derived from the three E3 same-seed differences (0.1, 0.6, 1.5 mm).
    That sample was too small to show this spread (F-8-63).
  - Relative to E3, D moved −4.1 / −1.4 mm and M moved +3.7 / +6.3 mm.
- **Inferred: a conservative side-effect of starvation, not a hypothesis test.**
  - Over the 5 multi runs, a lower QP rate goes with a larger clearance:
    - 60.8 → 0.1252 and 65.0 → 0.1254 (R2);
    - 66.8 → 0.1217, 82.7 → 0.1189 and 89.1 → 0.1189 (E3).
  - The two R2 multi runs are the most starved of all 5 and have the largest clearances.
  - One reading: a starved filter keeps more distance. The mechanism is not instrumented, and n = 5.
- **What this does not show.** It does not show that single is less safe. The pre-registered
  comparison still says D kept less clearance than M by more than the tolerance, so Q1 stays NO.

## 6. F-8-41: the moving-obstacle harness correction (STEP 5)

The work is in `~/franka_baseline_evidence/refactor_r2/06_f841/`:
- `scripts_h3/`, tests and `ADDENDUM_F841.md`;
- `SHA256SUMS` (86 files), sealed 17:20:17Z, `SHA256SUMS` hash `2a847895…`;
- addendum SHA-256 `7765e883…`.

Repository copies are in [`tools/f841/`](../tools/f841/README.md). No production, launch or
perception file was touched. The sealed `scripts_h2/` is unchanged.

### 6.1 Static assumptions (Verified, code)

Seven places. R2 report §8.4 listed five; this session found the last two of the gated rows and
re-located row 2.

| # | Where | Static assumption | Effect on a moving run |
|---|---|---|---|
| 1 | `e3_h2_metrics.py` V4 | g_min = d_safe + v_ref·τ = 0.1875 m | Accepts births that can reach d_safe within τ |
| 2 | `e3_h2_metrics.py` placement | Closing speed and time-to-d_safe computed against the fixed p0 | Wrong recorded values |
| 3 | `s6b1_supplementary.py` | Dip bound v_cp·dt/2; the causal pose treated as current | The "lower bound" (R12 / R14) is not a bound |
| 4 | `e3_analyze.sh` R13 | Truth and reconstruction must agree to 1e-9 | Spurious STOP |
| 5 | **`e3_h2_bag.py`** | `raise SystemExit('mask replay implemented for ONE static obstacle only')`; ball at p0 | **Every moving run would GATE_STOP at the h2_bag step** |
| 6 | `s6b1_reconstruct_clearance.py` | Hard-coded E3 ball (0.55, −0.25, 0.449), r 0.05 | Wrong diagnostics |
| 7 | `e3_run.sh`, `e3_resolve.py` | Scenario and arm hard-coded | Scenario cannot run |

Recorded-only diagnostics, not corrected: `s6b_host.py` near/far `s_dot`, and `s6b_analyze.py`
`obstacle_p` (first sample).

### 6.2 Corrections (harness copies only)

Each correction reduces exactly to the static formula when v_obs,max = 0.

- **C1, V4.** g_min = d_safe + (v_ref + v_obs,max)·τ. The ball position at each sample comes from its
  law (law time t − t0). Time to d_safe uses v_ref + v_obs,max.
- **C2, lower bound.** min_k [g_k − v_obs,max·age_k] − (v_cp,max + v_obs,max)·dt_max/2, where age_k
  is the causal pose's age, which is already in the reconstruction npz. This replaces the R2 plan's
  fixed 22 mm by the per-sample age.
- **C3, R13.** |Δg| ≤ v_obs,max·|Δt_pose|.
- **C4, mask replay.** The ball is rendered at its law position at each probe.
- **C5, diagnostics.** Obstacle values come from the run's scenario, and a new law check compares the
  bag poses with the analytic law.
- **C6, harness.** Scenario as a recorded argument, from an allow-list.
- **R23 (A1).** The gate rule: law deviation ≤ 2 mm on moving runs.

v_obs,max is computed from the law, not measured (`h3_moving.speed_bound`). Ballistic and pursuit
laws are refused.

### 6.3 The threshold: 0.1875 → 0.2991 m (Derived; Verified by computation)

The 0.10 + (0.35 + 0.44)·0.25 = 0.2975 m arithmetic is correct, but 0.44 m/s is the maximum *sampled*
over one 40 s episode (also the `scenarios.yaml` comment), not a bound.

The wander law's per-axis peaks are (0.0718, 0.3195, 0.3034) m/s. Its frequency ratios
(1, φ, 1 + √2) are incommensurate, so the supremum is their norm, **0.4464 m/s**:
- it is never exceeded in 1 h of samples at 1 ms;
- it is approached to 1 mm/s.

That gives **g_min = 0.2991 m**, 1.6 mm higher.

**Physical justification.** d/dt‖p − c‖ ≥ −‖ṗ − ċ‖ ≥ −(‖ṗ‖ + ‖ċ‖), so the closing speed is at most
v_ref + v_obs,max. Property test A8: 2000 random closings never go below d_safe. The bound is
conservative: it assumes a head-on approach at both maxima for the whole 0.25 s, and no reaction. At the
φ0 = 0 birth the ball moves at 0.31 m/s with no component towards the hand. Tighter rigorous variants
exist; the law's maximum over [0, τ] gives 0.2655 m. The global bound is kept because it is scenario-
and phase-independent.

**Per-seed V4 margins** (pre-birth information only; Inferred: assumes the static phase's arm pose at
the phase-0 birth):
- the real jitter (`scenario_catalog.resolve`, seeds 0–20) gives predicted g0 of 0.3204–0.3897 m,
  margins **+2.1 to +9.1 cm**;
- seeds 0 and 1: +6.1 and +2.6 cm;
- the theoretical worst jitter corner (+0.8 mm) is not reached by any of these seeds.

### 6.4 Tests (Runtime, recorded; `06_f841/test_results/`)

**`test_f841.py`: 17 of 17.**
- **A:** units and properties.
- **B:** three bug demonstrations, each using a synthetic 0.44 m/s approach with a 50 ms pose stream:
  - the sealed lower bound is 2.2 cm *above* the true minimum; the corrected one is below it by
    ≤ 0.6 mm;
  - the sealed R13 STOPs on exact geometry; the corrected one passes, and still catches a 1 mm error;
  - the sealed V4 accepts a birth that can reach d_safe.
- **C:** on all 10 sealed static runs (E3-05…10 and R2-01…04):
  - corrected `s6b1_supplementary` and `e3_h2_metrics` reproduce the sealed JSON exactly (new keys
    only);
  - the corrected gate gives every sealed verdict, with identical R12 / R14 text;
  - R23 gives PASS on static runs, STOP at 5 mm and PASS at 1 mm.
- **D:** the scenario numbers (0.4464, 0.2991, 0.2975 at 0.44, the 40 s sampled maximum) and a check
  that `scenarios.yaml` still holds this spec.

Three earlier attempts failed in the test code only. Their logs are kept, and no correction changed
between attempts:
- attempt 1: output-name collision;
- attempt 2: passed, before R23 existed;
- attempt 3: the copies kept the read-only mode.

**`test_f841_container.sh`: 10 of 10 PASS.** The corrected `e3_h2_bag` and
`s6b1_reconstruct_clearance` reproduce the sealed JSON and npz exactly on the 10 sealed cells. The
law deviation is 1.6e-15 m.

**Not covered.** C4 and C5 on a real moving bag. No moving run is authorised; R23 is the first thing a
moving run checks.

### 6.5 Prospective addendum

`06_f841/ADDENDUM_F841.md` is sealed and not executed. It contains:
- the seven assumptions;
- C1–C6 and A1 / R23;
- the threshold derivation;
- per-seed margins, with seeds **0, 1** proposed by the pre-birth rule;
- open items for a moving-phase protocol:
  - scene `diag` (furnished), not `diag_bare`: a confound against the static phase, with leads of
    about 2× the QP rows and phantom velocities on furniture (Unverified at this revision);
  - provisional moving-case tolerances.

## 7. Limitations

- 2 pairs, 1 static scenario, 1 host, simulation at RTF about 0.7. This is not hardware evidence and
  not a proof of general safety.
- Q1 and Q4 tolerances come from three E3 pairs (F-8-63). The same small-sample calibration issue
  affected the offline A_L1 criterion (F-8-60).
- The rate–clearance association is post hoc, with n = 5 multi runs and no instrumented mechanism.
- The F-8-41 correction is verified for exact static equivalence and on synthetic moving cases; it has
  never run on a moving bag.
- The per-seed g0 prediction assumes the static phase's arm pose at the birth.
- The moving scenario uses scene `diag`, which the static phase did not.
- F-8-44 (σ / snapshot coherence) is still present in both executors' code paths (single via the
  exception path). It is not addressed here.

## 8. Recommendation for the moving-obstacle phase

**By the pre-registered rule, this campaign does not support proceeding** (Q6 DOES_NOT_SUPPORT,
because of Q1). The harness side is ready: F-8-41 is implemented and tested, and the addendum is
sealed.

Recommended order; each step needs its own approval:
1. **Resolve Q1 prospectively.** Run a new sealed static repetition (e.g. seeds 2 and 3, plus a
   repeat of 0 and 1, order counterbalanced). Its Q1 tolerance must come from the spread now observed
   across 10 runs, and it must be pre-registered before data. Alternatively, pre-register Q1 as "D's
   lower bound ≥ d_safe + margin", an absolute safety criterion, instead of a D-vs-M relative one.
   That choice is the user's.
2. Only then run the moving phase on `pentagon_wander_close`, with:
   - `scripts_h3`;
   - seeds 0 and 1;
   - a moving-phase protocol that references the sealed addendum and decides the `diag` scene
     question.

The alternative is for the user to declare the moving phase exploratory, with no support claim.
That is not recommended without step 1.

## 9. Does any unresolved issue block merging R2?

- **No new safety defect was found.** In every D run:
  - clearance stayed ≥ d_safe + 16 mm;
  - the lower bound ≥ d_safe;
  - no reflex, fault or fallback occurred;
  - timing, staleness and smoothness were better than M.
- **But two pre-registered criteria are unmet:**
  - the offline A_L1 verdict is NOT_REPRODUCED (F-8-60, not accepted by the user);
  - the Gazebo Q1 is NO, so Q6 is DOES_NOT_SUPPORT.
- Both look like tolerance-calibration problems (Derived). Declaring them passed after seeing the data
  is what the user rejected for the 13 ms result.
- **Recommendation:** do not merge R2 yet. Merge after a pre-registered repetition settles Q1 (§8
  step 1), or after an explicit decision by the user. F-8-44, F-8-56 and F-8-58 do not block R2.

**User decision (message 17, 2026-11-20): R2 is not approved for merging into the main development
branch.** The branch is pushed as a record only.

## 9a. R2 status at the end of the session (C-079)

| Item | State | Where |
|---|---|---|
| Configuration (`executor_mode: single` default; `full` pin `single`) | Committed `cadaf62` | R2_SINGLE_EXECUTOR_DEFAULT.md §2–§3 |
| Executor regression tests (shipped default, config consistency, real `main()`, shipped-executor smoke) | Committed `359dd7c`; passing | idem §4–§5; F-8-59 |
| Offline performance regression | **NOT_REPRODUCED** (A_L1 gap p99 13 vs ≤ 12 ms). Not accepted by the user; unresolved | idem §6; F-8-60 |
| Gazebo static validation | **Q1 failed** (D − M clearance −9.3 / −7.1 mm vs 5 mm); Q2–Q5 YES; Q6 DOES_NOT_SUPPORT | §4; F-8-61, F-8-63 |
| Moving-obstacle validation | **Not performed** | §8 |
| F-8-41 harness correction | Implemented and tested in the harness only; addendum sealed, not executed | §6; F-8-62 |
| F-8-44 (governor σ / snapshot coherence) | Reproduced by isolated tests; present under both executors; production fix P2 not made | R1_M3_HOST_LOAD_AND_F844.md §6; F-8-44, F-8-49 |
| Other open items | F-8-56 (OSQP ρ interval), F-8-58 (`--set` only for `full`), F-8-64 (summary references), F-8-1 (launch golden) | PROGRESS §4–§5 |
| Merge | **Not approved** (message 17) | — |

## 10. Consequences of modification

- `tools/gz_validation/` (r2gz) needs no change for further static pairs. Change only the protocol
  constants, and do that before data.
- `scripts_h3` supersedes `scripts_h2` for any run with a moving obstacle. For static runs the two give
  identical results. Any later change to `h3_moving.py` must keep the exact static reduction; test C
  checks it.
- A new obstacle law needs a `speed_bound` entry, or `h3_moving` refuses it.

## 11. Findings

| ID | Summary |
|---|---|
| F-8-61 | R2 static Gazebo campaign: 4 of 4 runs valid and comparable. Q2–Q5 YES; Q1 NO (D − M clearance −9.3 / −7.1 mm vs 5 mm); Q6 DOES_NOT_SUPPORT |
| F-8-62 | F-8-41 correction implemented in the harness: two further static assumptions (`e3_h2_bag` refusal, closing speed at p0); threshold 0.2991 m, not 0.2975 m; tests; addendum |
| F-8-63 | The Q1 (and Q4) tolerances were calibrated on three E3 same-seed differences; the 10-run spread is 9.3 mm. Same pattern as F-8-60 |
| F-8-64 | `r2gz_summary.py` prints `None` for the E3 references: it reads `r2gz_extra.json`, which E3 runs lack. Recording gap; computed post hoc |
