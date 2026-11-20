# CBF monoliths: analysis, characterisation tests and extraction plan

*Refactoring preparation, message 19 (2026-11-20), branch `refactor/cbf-monoliths` from `humble-mattia`
`34d8daa` = documented baseline B-2. **No production code was changed.** Tests committed as `f3d9841`
(baseline B-3); checkpoint C-082.
Labels as in [00_INDEX.md](../00_INDEX.md) §2.*

Subjects: `franka_experiments/franka_experiments/utils/cbf_state_rows.py` (4,354 lines) and
`franka_experiments/franka_experiments/nodes/cbf_safety_filter.py` (2,137 lines). The mathematics is
described in chapters [07](../07_cbf_obstacle_barrier.md) and [08](../08_constraints_and_qp.md); the
ROS graph and executor in [02](../02_ros_graph_and_dataflow.md). This document covers structure,
state, failure handling and the extraction plan, and it does not repeat the equations.

---

## 1. Measurements (Verified, `ast` on the B-2 source)

### 1.1 `cbf_state_rows.py`

| Part | Lines | Size | What it is |
|---|---|---:|---|
| 17 free functions (`joint_limit_rows` … `risk_slack_weight`) | L59–946 | ~890 | pure row and margin formulas (already extracted, tested) |
| `SelfCollisionRowBuilder` | L951–1168 | 218 | capsule self-collision rows (own FK model) |
| `Shadow`, `ShadowRowBuilder` | L1175–1364 | 190 | SSB rows (default off) |
| 12 free helpers (`build_self_collision_builder` … `memory_surface_gap`) | L1369–1765 | ~400 | evidence, curvature, self-mask, memory geometry |
| `JointSnap`, `NomSnap`, `Obstacle`, `ObstacleSnap`, `ConstraintSnap` | L1768–1988 | ~220 | immutable data types (`NamedTuple`) |
| `build_optional_row_builders` | L1994–2101 | 108 | wiring of the optional row builders |
| **`ConstraintBuilder`** | L2104–4354 | **2,251** | `__init__` 182, **`build` 1,482 (L2335–3816)**, 10 helpers |

`ConstraintBuilder` holds 48 private attributes and about 60 public `diag_*` attributes.

### 1.2 `cbf_safety_filter.py`

| Part | Lines | Size |
|---|---|---:|
| `CBFSafetyFilter` (one class, 28 methods, ~90 private attributes) | L136–2067 | 1,932 |
| `__init__` | | 396 |
| **`_qp_tick`** | L1038–1511 | **474** |
| `_solve` | L1624–1767 | 144 |
| `_publish_status` | | 136 |
| `make_executor`, `main` (R1) | L2074–2133 | 60 |

There are 4 subscriptions, 2 publishers and 2 timers, in 3 callback groups: `grp_io` (subscriptions),
`grp_perc` (`_update_constraints`, 50 Hz) and `grp_ctrl` (`_qp_tick`, `qp_rate_hz` = 300). There is no
lock: `threading` is imported only for `get_native_id` (Verified, `grep`).

---

## 2. `ConstraintBuilder.build`: regions, state and data flow

`build(js, obs, now)` returns a `ConstraintSnap` or `None`. It is one method with 25 section comments.
The partition below is by top-level statement (Verified, `ast`). "Writes" covers assignment,
subscript stores and in-place calls on `self.*` (`append`, `pop`, …); `*` marks in-place mutation.

| Region | Lines | Family / job | Persistent state written (besides `diag_*`) | Locals consumed from earlier regions |
|---|---|---|---|---|
| R0 | 2335–2488 | staleness gate (`distance_timeout`), `kin.update`, reset of ~35 `diag_*`, read-once trust/evidence/curvature flags, row accumulators created | `_kin*` (FK state) | — |
| **R1** | **2489–3158 (670)** | **obstacle loop**: per `Obstacle` → gating (valid, confidence, horizon, leverage, finiteness), closing-speed estimation (`_obstacle_speed[_tracked]`, median, rotation hold, identity reset, fast track), margins (latency, uncertainty, sensor range, standoff, evidence), h-recovery EMA, row `(a, h̄, J̇q̇)`, retreat cap, escape, outrun, livelock and go-around candidates | `_obs_frames*`, `_obs_vel*`, `_obs_vmed*`, `_obs_nhat*`, `_obs_rothold*`, `_obs_ident[]` (via helpers), `_h_smooth[]`, `_stand_ema[]`, `_unc_ema`, `_ev_seen[]`, `_curv_nhat_hist*`, `_obs_mem[]`, `_cp_labels*` | 43 locals (row lists, caps, flags, best-candidate trackers) |
| R2 | 3160–3302 | N2 occlusion-aware obstacle memory rows (default off) | `_obs_mem[]` | row lists, `live_tracks`, gains |
| R3 | 3304–3335 | joint-limit rows | — | row lists |
| R4 | 3336–3355 | singularity row (`try/except`) | — (`diag_sigma`) | row lists |
| R5 | 3356–3379 | self-collision rows (own FK; `try/except`) | `_sc_kin*`, `_sc_q[]`, `_sc_v[]` | row lists |
| R6 | 3380–3452 | SSB rows (`try/except`; default off) | — | row lists, `n_weak` |
| R7 | 3453–3470 | stuck joint-limit bookkeeping (warnings) | `_qlim_stuck[]` | row lists |
| R8 | 3471–3567 | task-space link-speed rows, ISO Annex L speed rows, ISO reduced-mode TCP row | — | `spd_pts`, cap lists, `d_sc_min` |
| R9 | 3568–3612 | rate-cap rows, appended **last** (the QP addresses them by slice) | — | cap lists, row lists |
| R10 | 3613–3744 | zone-ladder gains, depth push, weighted-slack weights, slack slots, velocity feed-forward | — | `G`, `n_c`, gains |
| R11 | 3745–3816 | curvature scatter, lateral-evasion bias, `ConstraintSnap(...)` | — | 28 locals |

**Data flow (Derived).** Six parallel lists (`rows_a`, `rows_h`, `rows_jdq`, `rows_link`,
`rows_group`, `rows_vobs`) travel from R0 to R11, and every family appends to them. Their order is
part of the contract:
- obstacle rows come first, because the per-obstacle extras are indexed by obstacle-row position;
- retreat-cap rows are spliced as the trailing block (`n_rtr`);
- rate-cap rows come last (`n_cap`), because `build_row_rhs` and the QP address them by slice.

43 locals cross from the obstacle loop into later regions. That coupling, not the line count, is
the real size of `build`.

**Helpers with state (Verified).**
- `_obstacle_speed` writes `_obs_frames`, `_obs_nhat`, `_obs_rothold[_t0]`, `_obs_vel` and
  `diag_rot_reject`.
- `_obstacle_identity_changed` writes `_obs_ident`.
- `_outrun_bias` writes `_outrun_warned` and `diag_outrun_*`.
- `_frame_id` writes `_fid_cache`.
- `_escape_row`, `_livelock_direction`, `_retreat_cap`, `_fast_track` and
  `_obstacle_speed_tracked` write nothing.

## 3. `_qp_tick`: steps and state

| Step | Lines | Job | Exits |
|---|---|---|---|
| prologue | 1038–1056 | one-shot FIFO elevation, tick-gap watch, NaN reset of the residual diagnostics | `js is None` → return (no publish) |
| STEP 0 | 1057–1093 | ISO stop → `box_only_solve` brake, level BRAKE | publish + status, return |
| STEP 1 | 1094–1107 | joint state stale → −k_brake q̇ | publish + status, return |
| STEP 2 | 1108–1121 | nominal fresh? (else −k_brake q̇); frozen state overrides | — |
| STEP 3 | 1122–1162 | mode: frozen / empty-frame fault / **fresh snapshot → `build_row_rhs`** / distance stale → brake nominal | — |
| STEP 3a | 1163–1210 | zone-ladder task fade (`_zone_held`) | — |
| STEP 3a+ | 1211–1281 | state governor (σ from `self._rows.diag_sigma`, d_sc from the snapshot: F-8-44), directional fade | — |
| STEP 3b / 3b′ | 1282–1349 | tangential bias EMA; go-around (writes `self._rows.diag_tan*`, `diag_ga*`: F-8-45) | — |
| STEP 3c / 3d / 3e | 1350–1434 | escape, outrun, livelock biases (EMA state on the node) | — |
| STEP 4 | 1435–1474 | state box (position/velocity/acceleration), slew limit, conflict counters | — |
| STEP 5 | 1475–1498 | `_solve` (OSQP setup/update, padding, fallback ladder), per-family residuals (`try`) | — |
| STEP 6 | 1499–1511 | slew diagnostics, braking-authority fault, publish, status, report | — |

`_qp_tick` writes 28 node attributes and reads 46 (Verified, `ast`). It reads `self._con` **once**
(L1125) and solves with that snapshot's rows, which `test_tick_solves_with_the_snapshot_it_read_first`
now pins. Three things it uses do **not** come from that snapshot:
- σ (F-8-44);
- the `diag_*` fields that it writes into the builder object while the builder may be rebuilding
  (F-8-45);
- `self._obs`, read for the staleness branch only.

## 4. Shared mutable state between threads (Verified from the callback groups; races Inferred)

| State | Writer (group) | Readers | Handoff |
|---|---|---|---|
| `_js`, `_nom`, `_obs` | subscriptions (`grp_io`) | rebuild, tick | whole immutable `NamedTuple` swapped by one assignment (atomic under the GIL) |
| `_con` | `_update_constraints` (`grp_perc`) | tick | same; read once per tick |
| builder `diag_*` (~60) | `build` (`grp_perc`) resets and fills; the tick (`grp_ctrl`) writes `diag_tan*`, `diag_ga*` | tick (σ), `format_cbf_diag`, `/cbf_status` | **no handoff:** individual attributes of a shared object (F-8-44, F-8-45) |
| builder per-track dicts (`_obs_vel`, …) | `build` only | `build` only | single writer |
| `_qddot_prev`, `_box_*`, `_qvec`, EMA biases | tick only | tick only (`_brake_box` from `_solve`) | single thread; arrays mutated **in place** (`np.copyto`, `out=`, slices of `_box_lb[:NV]` passed as `out`) |

Under `executor_mode: multi` (the shipped default) a rebuild can interleave with a tick. Under
`single` it cannot, but F-8-44 still occurs through the exception path (now pinned in a test).

## 5. Exception handling (Verified)

- **Guarded in `build`:** the singularity (L3341), self-collision (L3361) and SSB (L3405) families,
  each with `except Exception` → log and skip the family's rows; two dictionary lookups in the memory
  family (L3242, L3260).
- **Guarded in the tick:** `_solve` catches every solver exception, so a failure becomes the fallback
  ladder (brake, or box-only under `qp_failure_policy: box_only`). `family_violations` is diagnostic
  and guarded.
- **Not guarded (F-8-67, new):**
  - the rest of `build`: the obstacle loop, joint-limit rows, speed and cap rows, gains, assembly;
  - `_update_constraints`;
  - the rest of `_qp_tick`.

  rclpy Humble re-raises a callback's exception from `spin()`: `SingleThreadedExecutor` at
  `executors.py` L807–809, `MultiThreadedExecutor` at L860–864, read in the container. `main`'s
  `finally` then destroys the node. The filter therefore fails *stop*: it stops publishing, and the
  chain falls back on the controller's stale-command behaviour ([09](../09_torque_realization.md)
  §3.2, `command_timeout` 0.1 s; Derived).

  This is a property of the current design, not something the refactor should change silently.
  `test_f867_known_behaviour_unguarded_build_exception_propagates` pins it.

## 6. Performance-sensitive operations (Verified from the code; magnitudes Runtime (recorded) in C-062)

- **Per rebuild (50 Hz):**
  - one Pinocchio FK and Jacobian pass (`kin.update(..., with_jdot=True)`);
  - one point Jacobian per obstacle row (`_control_point_jacobian`);
  - a second FK pass on the hand model for self-collision;
  - per-track median and EMA updates.
- **Per tick (300 Hz requested):**
  - `build_row_rhs` (refreshes aᵀq̇ and the cap right-hand sides only);
  - the state box and slew limit, into preallocated arrays;
  - OSQP `update` with `pad_rows_to_block` (row count quantised to `qp_row_block` = 16), so that
    `setup` and its factorisation run only when the padded size changes.
- An extraction must keep:
  - the in-place array semantics and the aliasing of `_box_lb[:NV]` / `_box_ub[:NV]` views;
  - the padding;
  - the single `build_osqp_A` per tick.

  The reason is measured: a tick's cost is small NumPy work that holds the GIL, not the solve
  (C-062, sequential replay: `build_osqp_A` ≈ 27 %, `tangential_bias` ≈ 21 %, diagnostics ≈ 19 %,
  OSQP `solve` ≈ 3.5 % of a tick), and the achieved rate is already marginal under host load (F-8-52).
  Any allocation added per tick is therefore a real cost.

## 7. Architectural problems confirmed

| # | Problem | Evidence | Consequence for the refactor |
|---|---|---|---|
| A1 | `build` is one 1,482-line method with 12 regions, 6 parallel accumulators and 43 cross-region locals | §2 (F-8-66) | extract regions as functions with explicit inputs and outputs; keep the row order |
| A2 | Per-track estimator state (11 dicts) lives on the builder and is mutated from both `build` and helpers | §2 | move into one `ObstacleTrackState` owner; the private names that tests use (`_obs_vel`, `_obs_vmed`, `_min_frames_eff`, …) must stay reachable |
| A3 | Diagnostics are ~60 mutable attributes on a shared object, written by two threads | §4, F-8-45 | collect them per build in a value object and swap it **atomically** (a behaviour-visible change only for diagnostics: separate approval) |
| A4 | The governor's σ bypasses the snapshot | F-8-44, test pinned | fix = P2, a behaviour change, not an extraction |
| A5 | `_qp_tick` mixes mode selection, nominal shaping (7 stateful stages), box, solve and reporting | §3 | split into a pure mode decision, a shaping pipeline with explicit state, a `QPSolver`, a reporter |
| A6 | No local fault containment for most of `build` / `_qp_tick` | §5, F-8-67 | keep fail-stop during extraction; any change of policy is a separate decision |
| A7 | Tests and production call private methods unbound (`CBFSafetyFilter._brake_box(self)`, `_range_field`, `_latency_fields`, …): 37 private node names and 8 builder names are used by 28 existing test files | `ast` + `grep` | keep thin delegates with the same names; do not edit existing tests |
| A8 | `__init__` (396 lines) mixes parameter checks (ISO Annex L / M refusals), limits, kinematics, builder wiring, QP preallocation and ROS wiring | reading | extract configuration validation and QP preallocation; keep ROS wiring in the node |

## 8. Proposed module structure (in-process Python only; no new ROS node)

All ROS interfaces stay in `CBFSafetyFilter`: topics, parameter names, timers, callback groups,
`executor_mode` and its default. `cbf_state_rows.py` stays as a **façade** that re-exports every
public name, so no importer changes.

```
utils/cbf_state_rows.py            façade: re-exports + ConstraintBuilder (orchestrator, ~200 lines)
utils/cbf_snapshots.py             JointSnap, NomSnap, Obstacle, ObstacleSnap, ConstraintSnap, Shadow
utils/cbf_row_set.py               RowSet: the six parallel lists + trailing retreat/cap blocks, order invariant
utils/cbf_robot_rows.py            joint-limit (R3), singularity (R4), self-collision (R5), SSB (R6), stuck bookkeeping (R7)
utils/cbf_self_collision.py        SelfCollisionRowBuilder, build_self_collision_builder
utils/cbf_shadow_rows.py           ShadowRowBuilder
utils/cbf_obstacle_tracking.py     ObstacleTrackState (owns _obs_* dicts) + _obstacle_speed[_tracked], identity, fast track
utils/cbf_obstacle_rows.py         one obstacle → gated row + extras (R1 body, staged: gate, speed, margins, row, extras)
utils/cbf_obstacle_memory.py       N2 memory rows (R2)
utils/cbf_speed_rows.py            link-speed / ISO speed rows (R8) + rate-cap block (R9)
utils/cbf_snapshot_assembly.py     gains, slack weights, depth push, curvature, lateral bias, ConstraintSnap (R10–R11)
nodes/cbf_safety_filter.py         CBFSafetyFilter: ROS wiring, callbacks, _qp_tick orchestrator, delegates
utils/cbf_tick_mode.py             STEP 0–3 decision as a pure function → (mode, nominal, fault)
utils/cbf_nominal_shaping.py       STEP 3a–3e: ShapingState + pipeline (zone, governor, tangential, go-around, escape, outrun, livelock)
utils/cbf_qp_solver.py             QPSolver: OSQP lifetime, padding, setup/update, fallback ladder, slack bookkeeping (_solve)
utils/cbf_status_report.py         /cbf_status row, CBFDIAG and solve-time lines
```

Existing utilities stay as they are: `cbf_qp_assembly`, `cbf_hard_limits`, `cbf_diagnostics`,
`cbf_zones`, `go_around`, `livelock`, `state_governor` and `rate_scaling`.

## 9. Exact extraction sequence (each step = one approval, its own commit, no behaviour change)

Gate for every step:
1. `colcon build` of `franka_experiments`, since the tests import the install;
2. `test_cbf_characterisation.py` all green **without** regenerating goldens;
3. suite 1 failing set = the 35 known IDs only;
4. smoke ×3;
5. `check_docs` 14/14.

A step that needs a golden change is not an extraction: stop and report.

| Step | Move | Risk | Why this order |
|---|---|---|---|
| E1 | data types → `cbf_snapshots.py` (re-exported) | very low | no logic; everything else imports them |
| E2 | `SelfCollisionRowBuilder`, `ShadowRowBuilder` and their factories → own modules | low | already self-contained classes |
| E3 | `RowSet` replaces the six parallel lists inside `build` (same append order) | low–medium | makes every later region extraction a signature change only |
| E4 | R3–R7 robot-state families → `cbf_robot_rows.py` functions | low | few inputs, already partly guarded |
| E5 | R8–R9 speed / ISO / rate-cap rows → `cbf_speed_rows.py` | medium | trailing-block order and the `n_cap` / `n_rtr` slices |
| E6 | R10–R11 → `cbf_snapshot_assembly.py` | medium | 28 inputs; produces the snapshot |
| E7 | R2 memory rows → `cbf_obstacle_memory.py` | medium | default off; covered by `optional_families` |
| E8 | per-track state + stateful helpers → `ObstacleTrackState` (aliases keep `_obs_vel`, `_obs_vmed`, `_min_frames_eff` reachable) | medium–high | 11 dicts, two writers |
| E9 | R1 obstacle loop → staged functions in `cbf_obstacle_rows.py` | **high** | largest region, 43 outgoing locals; last builder step |
| E10 | `_solve` → `QPSolver` (node keeps `_solve` as delegate) | medium | OSQP lifetime and warm start must be identical |
| E11 | STEP 3a–3e → `cbf_nominal_shaping.py` with explicit `ShapingState` | medium–high | 7 stateful stages, EMA order |
| E12 | STEP 0–3 decision → `cbf_tick_mode.py` | medium | early returns and status rows |
| E13 | status / report → `cbf_status_report.py` | low | wall-time fields excluded from the goldens: check them by schema |
| E14 | `__init__` validation and QP preallocation helpers | low–medium | the ISO refusals are pinned |

**Behaviour changes are not extraction steps.** They come after, each with its own approval and
evidence, and each updates exactly the test that pins the old behaviour:
- **P2:** σ from the snapshot (F-8-44);
- an atomic diagnostics snapshot (F-8-45);
- an exception-containment policy (F-8-67).

## 10. Characterisation tests added (STEP 4)

- `franka_experiments/test/_cbf_characterisation.py`: the harness. Real node, real config (SHA-256
  checked), real Pinocchio and OSQP. The fake clock replaces the instance's `_now`, callbacks run
  sequentially, the plant is a double integrator, and perception comes from the harness's own
  kinematics. It refuses a stale install.
- `franka_experiments/test/gen_cbf_characterisation_goldens.py`: `--write` and `--check`.
- `franka_experiments/test/data/cbf_characterisation/*.npz|json`: 11 scenarios, 3,900 ticks, 650 rebuilds,
  5.1 MB, written at `34d8daa`.
- `franka_experiments/test/test_cbf_characterisation.py`: 29 tests.

**Results (Runtime (recorded), evidence `~/franka_baseline_evidence/cbf_monoliths_prep_20261120/`).**
The new module passes 29 / 29 in 12.7 s. Suite 1 is 1434 pass / 35 fail / 9 skip, against
1405 / 35 / 9 in C-080:
- the 29 added tests all pass;
- no existing test changed outcome;
- the failing set is identical, test by test, to the 35 known IDs.

Suites 2 and 3 (179 / 0 / 0, 39 / 0 / 0) and smoke ×3 are unchanged.

| Priority (message 19) | Covered by |
|---|---|
| 1 obstacle rows | goldens `obstacle_approach`, `obstacle_static_close`, `optional_families`, `iso_speed_rows`, `input_edge_cases` (every `ConstraintSnap` field, every `diag_*`) |
| 2 self-collision | golden `self_collision` (7 rows, gap 11.8 mm → binding); `test_self_collision_failure_is_contained` |
| 3 joint limits | golden `joint_limit` (joints 1 and 7 driven past q_max) |
| 4 workspace | `test_no_workspace_family_in_the_shipped_filter` (the shipped filter has none) |
| 5 QP matrices and bounds | every golden: each OSQP `setup` / `update` (P, q, A, l, u, settings) and `solve` (x, status, iterations) per tick |
| 6 nominal and safe commands | every golden: q̈_nom sent, q̈_safe published, `/cbf_status` (39 of 41 fields), governor and zone weights, box, biases |
| 7 fallback | goldens `faults` (stale nominal, stale / frozen joint state, stale distance, OSQP failure → brake), `faults_box_only` (level 1) |
| 8 snapshot consistency, F-8-44 | `test_tick_solves_with_the_snapshot_it_read_first`, `test_f844_known_defect_governor_sigma_not_from_the_snapshot` |
| 9 exceptional and boundary cases | golden `input_edge_cases` (NaN distance, 5 s old capture stamp, invalid and low-confidence entries); `test_f867_…`; `test_iso_enabled_is_refused_…` |

**Exactness.** Every comparison is exact (`np.array_equal`, NaN == NaN); no tolerance is used.

- **Determinism.**
  - *Runtime (recorded):* two `--check` runs in separate processes with different `PYTHONHASHSEED`
    were bit-identical to the goldens for all 11 scenarios.
  - `test_harness_is_deterministic_in_process` checks the in-process case.
- **The single test-only deviation:** OSQP `adaptive_rho_interval` is fixed at 25. Production's 0
  makes OSQP choose the ρ-update interval from the measured setup time (F-8-56). Every production
  matrix, vector and setting given to OSQP is still recorded and compared.
- **Excluded from comparison:** only the wall-clock fields (`solve_wall_ms`,
  `constraint_build_wall_ms`, the snapshot's `build_wall_ms`).
- **Sensitivity.** *Runtime (recorded), mutation check.* A 1e-6 relative change was detected in
  each of `k0_cbf`, `k1_cbf`, `position_brake_eta`, `joint_limit_row_margin`, `k_brake`,
  `max_qddot_delta`, `zone_k0_notice`, `memory_hold_s` and `outrun_evasion_gain`, with first
  differences down to 1e-11. `zone_k0_priority` produced no difference: see §11.
- **Environment.** The goldens are bit-exact for the recorded fingerprint (Python 3.10, numpy 1.26.4,
  scipy 1.15.3, osqp 0.6.7.post3, pinocchio 4.1.0, x86_64). Under a different one the golden tests
  **skip** with that reason instead of failing.

## 11. Not covered, or not reliably testable yet

- **Multi-threaded interleavings as such.** The tests run the callbacks sequentially. The one
  interleaving that matters for the rows is emulated: a snapshot swap inside the tick. The others,
  such as the σ race and diagnostics mixing under real threads, are timing-dependent; they are pinned
  only through their single-thread reproductions (F-8-44 exception path).
- **SSB rows** (need shadow fields), the **ISO stop** (`/iso_safety` input), the **empty-frame fault**
  (ISO-gated) and the **braking-authority fault**: exercised only to the point of no false trigger.
- **The zone ladder's priority and hold bands.** The ball is never inside 0.05 m: the
  `zone_k0_priority` mutation was not detected.
- **The curvature term**: its κ diagnostics stayed 0 in `optional_families`.
- **Wall-clock behaviour**: tick gaps, solve and build times, `slew_rate_compensation` under real
  jitter, and the OSQP ρ adaptation itself (F-8-56).
- **The ROS wiring**: topics, QoS, timers, callback groups. It is covered by the existing launch and
  executor tests (`test_cbf_executor_mode.py`), not by these.
- **Real robot, Gazebo and perception**: out of scope (no Gazebo, no robot).

## 12. Risks and mitigations

| Risk | Mitigation |
|---|---|
| An extraction reorders floating-point operations (for example a sum regrouped) and changes the last bits | goldens are exact by design; a bit change stops the step, which is then reviewed: either the order is restored or the step is reclassified as a behaviour change |
| Tests compare the install, not the edited source | the harness refuses a stale install; the gate starts with `colcon build` (memory: colcon copies) |
| Goldens regenerated to "make tests pass" | the generator's docstring and this record forbid it; the goldens carry `source_commit` = `34d8daa` |
| Private names that tests use disappear | §7 A7 inventory; keep delegates; run all of suite 1 at every step |
| Row order or trailing-block slices broken (`n_cap`, `n_rtr`, slack slots) | `RowSet` (E3) encodes the order; snapshot fields `n_cap`, `n_rtr`, `group` and `links` are in the goldens |
| Allocation added in the tick (rate regression, F-8-52) | review rule: no new arrays per tick; optional timing comparison against C-062 (Runtime, not a gate) |
| Thread-safety worsened by splitting state across objects | keep the single-assignment handoffs of §4; no new shared mutable object without a lock or an atomic swap |
| Fixing F-8-44 / F-8-45 / F-8-67 "on the way" | not allowed in extraction steps; the pinned tests fail and make it visible |
| Coverage gaps of §11 | extraction steps touching those paths (E6 zone gains, R6 SSB) get an added scenario first, before the move |
