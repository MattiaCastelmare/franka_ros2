# Simulation validation protocol — one modification at a time

> **Status:** documented in Stage 6 (2026-11-11), at source baseline `ba68077`. **Not executed.**
> Stage 6 documents this procedure. It does not run the simulation and does not modify the
> framework. Every command below is to be run during the refactor phase, and only with the user's
> approval.
>
> **Basis:** [12_simulation.md](12_simulation.md) (what the simulation does and what its numbers
> mean), the findings F-6-1…F-6-14 (PROGRESS §4), and the checks in
> [tools/stage6_verification/](tools/stage6_verification/README.md).
>
> **Thresholds.** No numerical threshold is invented here. A number is used only when the code or
> the documentation already states it, and its source is cited. Every other criterion is marked
> **REQUIRES USER APPROVAL**. Section 6 collects these items, to be decided before the first
> modification.

---

## 0. The mandatory rule

Every refactoring change follows these ten steps, in order. **If any required check fails, STOP.
Do not proceed to another modification.** Do not skip a test silently, and do not assume
success: a step that could not run is recorded as **NOT RUN**, with the reason, and the
modification is not accepted.

1. **Propose ONE modification.** State what changes, why, which findings or chapter sections
   motivate it, and the expected observable effect.
2. **Explain the files affected.** List every file that will change. List the files that depend
   on them, using the "Consequences of modification" section of the relevant chapter and the
   coverage ledger. Name the hidden couplings: mirrored copies, positional parsers, shared
   constants, and installed copies.
3. **Wait for explicit approval.** An approval covers that one modification only.
4. **Implement only the approved modification.** No unrelated cleanup and no "while I'm here"
   edits.
5. **Build** (Phase 1).
6. **Run the relevant tests** (Phase 2).
7. **Launch the full Gazebo simulation** (Phase 3).
8. **Verify robot, controllers, perception, CBF and movement** (Phases 3–5).
9. **Check the safety and regression metrics** (Phase 6).
10. **Record the outcome** (§8): the evidence directory, every phase's result, and the decision.

**A simulation that merely opens Gazebo, or merely activates the controllers, does NOT count as a
successful test.** A modification is validated only when Phases 1–6 have all passed on the cells
of the reference set (§6.1).

---

## Phase 1 — Build and installed-state check

Purpose: the code that will run is the code that was changed. The simulation reads **installed**
configuration files and programs (12 §2.4), and on 2026-11-11 the installed tree did not match the
source (F-6-3).

| # | Step | Command (run from the workspace root, in the container) | Pass condition | Kind |
|---|---|---|---|---|
| 1.1 | Record the starting point | `git rev-parse HEAD; git status --short; git diff HEAD > <evidence>/working_tree.patch` | written | automatic |
| 1.2 | Build the affected packages | `colcon build --packages-select <the packages listed in step 2 of §0>`. Add `franka_simulation_experiments` and `franka_simulation` when their files changed | exit code 0, no package failed | automatic |
| 1.3 | Source the new install | `source install/setup.bash` | — | — |
| 1.4 | Installed == source | `docs/framework_study/tools/stage6_verification/run_all.sh` (the container part runs `check_installed.py`) | the line `INSTALLED == SOURCE` (12 §2.4) | automatic |
| 1.5 | Documentation state | `python3 -I docs/framework_study/tools/check_state.py` | exit 0, or exit 2 with the changed files being exactly the approved ones | automatic |

**STOP** if 1.2 fails, or if 1.4 reports a difference in a file the change did not intend to
touch.

---

## Phase 2 — Automated tests

| # | Suite | Command | Pass condition | Kind |
|---|---|---|---|---|
| 2.1 | Stack unit tests | `colcon test --packages-select franka_experiments` (or `pytest franka_experiments/test`) | every test passes; the skip count is unchanged from the baseline record | automatic |
| 2.2 | Sim harness unit tests | `pytest franka_simulation_experiments/test` (the 16 fast test files; 12 §12) | every test passes; skip count unchanged | automatic |
| 2.3 | Documentation checks | `run_all.sh` of `tools/stage2…stage6_verification` | every check passes. A check that fails because the code changed **under a finding** is not "fixed": the finding is re-assessed (marked superseded with the reason) | automatic |
| 2.4 | Slow Gazebo tests (when the change touches the plant, the controller, perception or the harness) | `pytest franka_simulation_experiments/test/slow` | every test passes | automatic, launches Gazebo |
| 2.5 | Graph contract (when topics, QoS, node names or TF change) | `launch_test franka_simulation_experiments/test/launch/launch_test_graph_contract.py` | `diff(ref, sim) == []` on the safety path | automatic |

**Skips are failures to report.** A suite that is skipped (for example the parity tests that skip
because the directory is named `franka_rl` rather than `franka_sim`, a memory lead) is listed as
NOT RUN with the reason. **STOP** on any failure.

---

## Phase 3 — Gazebo startup

One cell of the reference set, launched as the batch launches it
(`run_gz_scenarios --scenarios <s> --conditions <c> --seeds 0 --perception true --batch-dir <evidence>/batch`),
or directly with `ros2 launch franka_simulation_experiments gz_cbf_stack.launch.py …`.

| # | Check | How | Pass condition | Kind |
|---|---|---|---|---|
| 3.0 | No orphans before the start | `ps -eo pid,args \| grep -E 'ign gazebo\|parameter_bridge\|ros2 launch'` | nothing from an earlier run (stale `/clock` publishers make every cell time out; memory lead) | manual |
| 3.1 | The launch resolved | `launch.log`: no exception from `conditions.resolve`, `scenario_catalog.resolve` or `build_descriptions`; `run_manifest_sim.json` exists with the intended condition, scenario, seed and git SHA | present and correct | automatic (file) + manual (content) |
| 3.2 | Sim time advances | `/clock` in the bag; `reflex_summary.json → rtf.rtf_run` | rtf_run not null. **The RTF bar for acceptance: REQUIRES USER APPROVAL** (the code flags < 0.9, `RtfMeter.summary(flag_below=0.9)`) | automatic |
| 3.3 | Robot spawned and controllers active | `run_manifest_sim.json → episode.controller_active_at_start` | `true` | automatic |
| 3.4 | All stack nodes up | `launch.log` / `ros_log`: no node died (no "process has died", no traceback); for a parameter type change, `ros2 param get` of the changed key | none died | automatic (log grep) + manual |
| 3.5 | The episode started and ended | `run_manifest_sim.json → episode.t_start_sim`, `t_end_sim`; `launch.log` has `SIM episode START` and `SIM episode DONE` | both present; `t_end − t_start ≥ T` of the scenario | automatic |
| 3.6 | Clean stop | `cell_result.json → timed_out`, `leftovers` | `false`, `[]` | automatic |

**A run that passes Phase 3 and nothing else is NOT a successful test.**

---

## Phase 4 — Perception and CBF are live

| # | Check | How | Pass condition | Kind |
|---|---|---|---|---|
| 4.1 | Rows arrive (mode `true` or `emulated`) | `closed_loop → rows_rate_hz`, `rows_latency_ms_p50/p95` | rows present. **Minimum rate and maximum latency: REQUIRES USER APPROVAL** (rendered at 90 Hz, 12 §4.1) | automatic |
| 4.2 | Perception is the intended mode | `run_manifest_sim.json → perception`; for `true`, the node log of `real_time_distance` shows frames processed; for the scenario, the depth adapter's `n_in/n_out` if logged | as intended. **Emulated runs do not validate perception** (F-6-8) | manual |
| 4.3 | The filter solves at the nominal rate | `closed_loop → qp_rate_hz`, `qp_rate_ok` (±5 %: `QP_RATE_TOL`, Task 5.5 of `OSCBF_ALIGNMENT_PLAN.md`, cited in `closed_loop.py`) | `qp_rate_ok == True`. **`None` is not a pass:** for a condition without `qp_rate_hz` (F-6-4), compare by hand against the effective rate (`C3.effective_params`) with the same 5 % band | automatic, plus manual for UNKNOWN |
| 4.4 | Obstacle rows reach the QP when an obstacle is near | `/NS_1/cbf_status → n_rows` in the bag while the truth gap is small; `closed_loop → n_rows_max` | n_rows_max > 0 in a scenario with an obstacle within `max_thresh` | automatic |
| 4.5 | No QP failure or fallback beyond the baseline | `fault_ticks`, `fallback_levels`, `slack_max_any`, `slack_max` per family | **equal to or better than the paired baseline cell** (§6.2). Absolute limits: **REQUIRES USER APPROVAL** | automatic (fields), comparison manual |
| 4.6 | Stale perception | the filter's sensing age `capage_ms_p50/p95` (CBFDIAG) and the rows rate | **REQUIRES USER APPROVAL** (no documented bound in sim); needs CBFDIAG enabled | instrumentation |

---

## Phase 5 — The robot really moves as intended

Opening Gazebo, activating the controllers, or publishing rows is not enough. The arm must
execute the commanded motion, and the safety behaviour must come from the filter, not from a
stalled loop.

| # | Check | How | Pass condition | Kind |
|---|---|---|---|---|
| 5.1 | The arm moved | the bag's `/NS_1/franka/joint_states`: displacement of q over the episode. For pentagon runs, `pentagon.p95_m` and `n` > 0 | non-zero motion, of the same character as the baseline cell. **"Moved enough": REQUIRES USER APPROVAL** | automatic (computable) + manual |
| 5.2 | Commanded vs realised | `runner.digest_cell → qdd_projection.frac_median/p10/p90` (12.22); `cbfdiag.qdd_ratio_0.5` | close to the baseline cell. **Band: REQUIRES USER APPROVAL** | automatic (`digest_cell` must be called; the batch does not call it) |
| 5.3 | Nominal vs actual | scenario runs: `task_final_err`, `task_done` (< 0.05 rad, the `cbf_scenarios.summarize` definition); pentagon runs: `pentagon.rms_m/p95_m` | `task_done` as in the baseline cell; tracking no worse than baseline. **Degradation allowed: REQUIRES USER APPROVAL** | automatic |
| 5.4 | Smoothness | `jerk_p99` (12.17); `smoothness_report.py` verdicts (`digest_cell`) | no worse than baseline. **Band: REQUIRES USER APPROVAL** | automatic |
| 5.5 | The pentagon phase is comparable | pentagon runs: the EE position at t_start | **Note F-6-12:** the pentagon ignores the episode clock, so compare per-cell distributions over seeds, not single runs | manual |

---

## Phase 6 — Safety and regression acceptance

### 6.1 Reference set of cells (REQUIRES USER APPROVAL)

The cells on which every modification is validated must be fixed **once**, before the first
modification, and then never changed silently. The proposal below is to be approved, edited or
replaced:

| Cell (scenario) | Why | Kind |
|---|---|---|
| `pentagon_clear` | the nominal loop alone; no obstacle | none |
| `static`, `pentagon_on_path` | static barrier, scene without and with furniture | static |
| `slow`, `fast`, `throw_ballistic_v4` | moving: pursuit, fast linear, ballistic | moving |
| `pentagon_wander_close` | a long moving tour near the path (memory lead: 2.2 mm near miss at κ 1.0) | moving |
| `p4_laws` or `pentagon_multi_mixed` | combined, identity separation | combined |

The proposal also covers:
- **Conditions:** the condition the change targets, plus `full`. Note that the four Phase 5
  conditions run at 300 Hz with go-around (F-6-4).
- **Perception:** `true` for every cell; `emulated` additionally when the change is in the filter
  only.
- **Seeds:** at least seeds 0, 1, 2 (a seed varies the start phase only, F-6-9). **The number of
  seeds: REQUIRES USER APPROVAL.** The memory lead says σ = 2.27 cm, so single-seed claims are
  not reliable.
- **Baseline:** the same cells run on the **unmodified** baseline, built and installed (Phase 1
  passed), in the same container, with the same `--filter-cpu`. The recorded runs in `sim_runs/`
  are **not** a baseline (F-6-1).

### 6.2 Acceptance criteria

The "Automatic?" column says whether a production scorer computes the value today. "Needs
instrumentation" means a new or changed scorer, a log setting or a manual step.

| Category | Metric (12 §8.2) | Threshold | Source of the threshold | Automatic? |
|---|---|---|---|---|
| **Collision / penetration** | `contact` (truth gap < 0) | **none allowed** in a cell where the baseline has none | definition of contact (12.14) | automatic, **but** valid only if the truth grid has no hole around the closest approach (F-6-5): check the largest Δt of `truth.npz` near `t_min_h` |
| | penetration depth `min_gap` when < 0 | REQUIRES USER APPROVAL | — | automatic |
| | penetration duration | REQUIRES USER APPROVAL | — | **needs instrumentation** (no field; `neg_duration_s` is time inside d_safe, F-6-7) |
| **True clearance** | `min_h`, `min_gap`, `t_min_h`, `min_h_cp` | not lower than the baseline beyond a tolerance: **REQUIRES USER APPROVAL** | — | automatic |
| | time inside d_safe `neg_duration_s` | not longer than baseline: **tolerance REQUIRES USER APPROVAL** | — | automatic (biased by holes, F-6-5) |
| **Constraint violations** | `slack_max_any`, per-family `slack_max`, `fault_ticks` | REQUIRES USER APPROVAL | — | automatic (`slack_max` sampled) |
| **Solver failures** | `fallback_levels` (box-only, brake), `fault_ticks` | REQUIRES USER APPROVAL (the baseline count is the reference) | — | automatic (100-tick resolution) |
| **QP rate** | `qp_rate_ok` | ±5 % of the nominal | `closed_loop.QP_RATE_TOL` (Task 5.5) | automatic only when the condition declares `qp_rate_hz` (F-6-4) |
| **Stale perception** | `capage_ms_p95`, `rows_rate_hz` | REQUIRES USER APPROVAL | — | automatic only with CBFDIAG lines in `launch.log` |
| **Controller faults / robot limits** | `reflex_verdict`, `reflex_trips`, `rate_clips` | verdict **PASS** (FAIL = the FR3 would have aborted; NO_DATA = not evaluated) | `reflex_checks` (libfranka envelope, effort limits) | automatic. **Not** included in `ok` (F-6-6): check it explicitly |
| | controller active at start | `true` | supervisor | automatic |
| **Nominal vs actual motion** | `task_done`, `task_final_err`, `pentagon.p95_m`, `qdd_projection.frac_*` | `task_done` 0.05 rad (cbf_scenarios definition); others REQUIRES USER APPROVAL | `closed_loop` | automatic (`qdd_projection` only through `digest_cell`) |
| **Scenario completion** | `timed_out`, `leftovers`, episode start/end, `ok` | no timeout; no leftovers; START and DONE present | `batch.cell_ok` | automatic. **`ok` alone is not acceptance** (F-6-6) |
| **Simulation validity** | `rtf_run`, `rtf_p10`, the reflex `n_samples` | RTF flag at 0.9 (code); acceptance bar REQUIRES USER APPROVAL | `RtfMeter` | automatic |
| **Comparison statistics** | per-cell median and [min, max] over seeds (`report.py`) | the decision rule (e.g. paired difference, test, effect size): **REQUIRES USER APPROVAL** | — | partly automatic |

**Hard rules that need no threshold:**
1. A cell whose truth is missing or holed at the closest approach cannot support a "no contact"
   claim: re-run it (F-6-5).
2. `perception:=emulated` cannot accept a perception change (F-6-8).
3. An UNKNOWN rate verdict is not a pass (F-6-4).
4. A recorded run from another revision is not a baseline (F-6-1).
5. Never `--rescore` the evidence directory in place: copy it first (F-6-10).

---

## 7. Preserving diagnostic evidence

For every modification, create `<evidence>` = `docs/framework_study/validation/<NN>_<short-name>/`.
That directory is to be **proposed** at the first modification: Stage 6 creates no directory. It
holds:

1. `working_tree.patch` (`git diff HEAD --binary`), `HEAD.txt` (`git rev-parse HEAD`, `git status`),
   because `simexp.provenance.snapshot` is not called by the launch (F-6-11);
2. the output of `run_all.sh` (stages 2–6), and the `check_installed.py` lines;
3. the colcon build log and the test reports, **including the skip lists**;
4. the batch directory of the reference cells, untouched: `cells/*/` with `run_manifest_sim.json`,
   `truth.npz`, `reflex_*`, `bag/`, `launch.log`, `ros_log/`, `logger/`, plus `cells.csv` and
   `batch_manifest.json`. Large bags may be kept outside git, but their path and checksum go in
   the record;
5. the baseline batch the comparison used (or its path and checksum);
6. `RESULT.md` (§8).

Do not edit, rescore or "clean" these files. If a scorer changes later, score a **copy**.

---

## 8. Recording the outcome

`RESULT.md`, one per modification:

```
Modification: <NN> <title>          Approved: <date, by whom>
Files changed: <list>               Commit/patch: <sha or patch file>
Phase 1 build: PASS/FAIL/NOT RUN (<reason>)   installed==source: yes/no
Phase 2 tests: <suite: passed/failed/skipped (+names)>
Phase 3 startup: <per cell>
Phase 4 perception/CBF: <per cell>
Phase 5 movement: <per cell>
Phase 6 acceptance: <per criterion: value, baseline, threshold, PASS/FAIL/UNDECIDED>
Decision: ACCEPTED / REJECTED / STOPPED at phase <n>
Findings created or superseded: <F-ids>
```

Then add a line to `CHECKPOINTS.md`, and to the findings in PROGRESS §4 if the modification
supersedes one.

---

## 9. Reverting a failed change

A failed change is reverted before anything else is attempted. The protocol never jumps to a
second modification on top of a failed one.

1. **Preserve first.** Complete §7 for the failed attempt, including the patch. The failure is
   evidence.
2. **Revert, with the user's approval.**
   - If the change was committed: `git revert <sha>`, which creates a new commit and rewrites
     nothing.
   - If it was not committed: `git apply -R <evidence>/working_tree.patch` limited to the files of
     the modification, after confirming with `git diff` that nothing else is in the patch.
   - Never use `reset --hard`, checkout of unrelated files, or stash operations without explicit
     approval.
3. **Rebuild and re-verify the baseline:** Phase 1, including `INSTALLED == SOURCE` against the
   baseline, and Phase 3 on one reference cell. The revert counts only when the installed tree
   matches the baseline again.
4. **Record** `Decision: REJECTED` with the phase that failed and why. Then propose a new single
   modification, or stop.

---

## 10. Items requiring user approval before the first modification

1. The reference cell set, conditions, perception modes and seed count (§6.1).
2. Every threshold marked REQUIRES USER APPROVAL in Phases 3–6, in particular:
   - the clearance tolerance against the baseline;
   - the penetration depth and duration allowed (normally none);
   - the slack, fallback and fault counts;
   - the stale-perception bounds;
   - the motion and tracking bands;
   - the RTF bar;
   - the statistical decision rule.
3. Whether to fix the known harness defects **before** the refactor, since they weaken the
   evidence every later step relies on:
   - F-6-5, the truth holes;
   - F-6-4, a `qp_rate_hz` for the Phase 5 conditions;
   - F-6-11, calling the provenance snapshot;
   - F-6-2, the dotted literal.

   Each of them would itself go through this protocol as one modification.
4. The evidence directory location and what may be stored in git (bags are large).
