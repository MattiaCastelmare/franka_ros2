# Self-Shadow Barrier — validation report

**Status: PARTIAL.** This file records only measurements that were executed. Every
number below was read out of a Gazebo run or a test run in this workspace;
nothing is projected. Section 8 lists what has **not** been run, and the report
must not be read as if those rows were empty rather than absent.

Nothing in this work was committed or pushed, and nothing was run on the
physical FR3. `ssb_rows_enabled` and `distance.ssb_enabled` are both `false` in
the shipped configs, so a default build is bit-identical to before (asserted by
test, §3).

---

## 1. The finding that bounds every claim in this report

A return is classified as a self-shadow CANDIDATE when, in optical-axis depth,

    bias + guard  <  z_model − z_near  <  bias + tol        (0.10, 0.16) m shipped

For a sphere of radius *r* on the camera ray to a control point, the barrier's
own surface gap is one diameter deeper, so the deleted band corresponds to

    true surface gap  ∈  (bias + guard − 2r,  bias + tol − 2r)   = (0.02, 0.08) m at r = 4 cm

and `d_safe = 0.10 m`. **The band that the self-filter deletes lies entirely
inside `d_safe`.** An obstacle that first becomes invisible because of
self-filtering is therefore *already* inside the safety distance. SSB can bound
the approach from there; it cannot prevent the `d_safe` violation that the
deletion causes. This is asserted by
`franka_simulation_experiments/test/test_ssb_scenarios.py::test_the_deleted_band_lies_inside_d_safe`
so it cannot be quietly lost.

## 2. Mathematical validation (executed)

Analytic barrier, gradient, ḣ and the full predicted ḧ against central finite
differences, over random valid FR3 configurations and velocities
(`log/ssb_work/ssb_deriv_table.py`, raw output in `log/ssb_work/deriv_table.txt`):

| regime | ε | ∇h abs / rel | ḣ abs / rel | full ḧ abs / rel |
|---|---|---|---|---|
| frozen region (`J_x = 0`) | 1e-5 | 1.105e-9 / 2.800e-9 | 1.701e-9 / 6.292e-9 | 1.798e-6 / 1.681e-6 |
| region attached to a non-adjacent link | 1e-5 | 1.199e-11 / 1.513e-10 | 2.472e-11 / 1.776e-9 | 1.115e-7 / 5.043e-6 |
| frozen, near-coincident (L ≈ 1–4 mm) | 1e-6 | 1.663e-8 / 4.213e-8 | 3.039e-8 / 6.205e-8 | 9.350e-3 / 7.002e-5 |

The third row is the honest one: within a few millimetres of coincidence the
second derivative loses ~4 digits, as 1/L² predicts. `shadow_row` returns `None`
at exact coincidence rather than emitting a row with an undefined normal.

The adjacency exclusion is not a heuristic: a region owned by a segment's own
start link gives `‖a‖ ≈ 7e-17`, i.e. `dh/dq ≡ 0`, because control points
interpolate frame *origins* and a revolute child's origin is rigid in its
parent. Such rows carry no authority and are dropped by `cbf_min_leverage`.

Derivation, the three corrections to the proposed wedge (z-slab not spherical
shell; one-sided not `|·| ≤ tol`; dilated mesh silhouette not a per-control-point
cone), and the resolution of the zero-gradient problem of §5.2 are in
`SSB_MATHEMATICAL_DERIVATION.md`.

## 3. Unit / integration tests (executed)

| suite | result |
|---|---|
| `test_ssb_qp_rows.py` (17 tests) | pass — one row per shadow, `a == n̂ᵀJ_p`, own slack column only, horizon, release hysteresis, row cap, hold erosion + expiry, backwards clock, leverage gate, binding row, hard row |
| `test_self_shadow_geometry.py`, `test_self_shadow_derivatives.py`, `test_ssb_perception.py`, `test_status_residuals_and_qp_scheduling.py` | pass |
| `test_ssb_scenarios.py` (4), `test_conditions.py` (+2) | pass |
| full `franka_experiments` suite | 35 failed / 1389 passed / 9 skipped, against a pre-existing baseline of 35 failed / 1329 passed / 9 skipped — `comm -23` on the sorted FAILED lists is empty, i.e. **zero new failures** |
| `franka_simulation_experiments` suite | 111 passed |

Backward compatibility is asserted, not assumed: with `ssb_rows_enabled: false`
and shadows present on the wire the QP is bit-identical, and with the flag on
and no shadow it is bit-identical again.

## 4. Gazebo: does SSB see what the ordinary pipeline loses? (executed)

Scenario `ssb_poke` (`diag_bare`, a 4 cm ball driven along the camera ray into
the hand at 8 cm/s), seed 0, three arms differing in **one** key each.
Window = the interval in which the obstacle's *true* gap is below the band's
outer edge, 0.16 m. Ground truth from `truth.npz` at 1 kHz; perception numbers
from the node's own `SSBDIAG`/`CBFDIAG` lines (`log/ssb_work/ssb_verify.py`).

The six checks §10 asks for, on `ssb_poke__minimal_l1_ssb__gz__s0`:

1. **obstacle exists** — present on 7936 truth samples, true gap 0.4000 → 0.0126 m.
2. **it reaches the region** — true gap < 0.16 m for 4.95 s.
3./4. **perception removes it and the ordinary pipeline loses it** — during that
   window the obstacle CBF's `d_min` ran from +0.084 m to **+inf**: on 19 of 91
   ticks the pipeline reported **no obstacle at all** while the true gap was as
   small as 1.3 cm. This is the failure the whole exercise is about, reproduced
   under ground truth.
5. **SSB identifies the unknown space** — band candidates on **50 of 77**
   perception frames in the window, *all* of them unexplained by the 3-D robot
   model, cluster-supported on 17; `front_max = 0.100 m`, i.e. exactly the band's
   near edge, which ties the measurement back to §1's derivation.
   **Outside the window: 0 of 56 frames produced a candidate.** No false
   positives on this cell.
6. **SSB modifies motion** — up to **4 SSB rows**, on 18 of 91 ticks, closest
   shadow gap 0.109 m; `slack_ssb` max 0.0126 with `violation_without_slack_ssb`
   > 0 on 4.0 % of ticks and `violation_with_slack_ssb` ≤ 6.0e-4. The family
   neither starves nor is starved by the obstacle family (`slack_obs` max 0.47).

All 41 `/cbf_status` v3 columns publish finite values on all 792 ticks
(`log/ssb_work/status_report.py`), so the new family is observable, not inferred.

### The support threshold was wrong, and the measurement is what caught it

The first shipped value, `ssb_min_support_px: 6`, **rejected every candidate on
every frame** of the first grid: `band=5 unexplained=5 supported=0 max_blob=3`.
The threshold counts *grid samples*, and at `pixel_step = 6` one sample stands
for 36 px, so it was 36× stricter than its name suggests; worse, the band is
only 6 cm deep while a sphere's near-surface depth varies by its own radius
across its disc, so only an *annulus* of the ball is ever in the band — 1–5
samples, not the ~11 of the whole disc. The default is now 3, derived (a 2-sample
connected blob already spans 12 px, which isolated speckle cannot produce) and
the old value is kept as an explicit arm, `minimal_l1_ssb_sup6`, which measures
2 supported frames of 80 versus 17 of 77 — i.e. the mechanism really was
support-limited, not absent.

### Cost (executed, §9)

| quantity | value |
|---|---|
| `ssb_ms` (geometry pass) | p50 1.14 ms, p95 9.48 ms |
| when the p95 occurs | mid-episode (t 10.5–13.8 s), i.e. **when the band is populated** — not a warm-up artifact (first frames: 2.89, 0.89, 1.16, 0.15, 0.30 ms) |
| `constraint_build_wall_ms` | p50 6.0 ms, max 36.1 ms |
| `solve_wall_ms` | p50 3.2 ms, max 13.2 ms |
| rows | 11 base, max 15 with SSB — bounded by `ssb_max_rows` |
| QP rate | 97–99 Hz, every cell inside the harness's ±5 % gate |
| `fallback_level` | 0 on 100 % of ticks — no infeasibility, no brake |
| distance rows published | 34–39 Hz baseline → 29–33 Hz with `ssb_enabled` (`ssb_poke`) |

A p95 of 9.5 ms in the perception pass is **not** comfortable, and §6 shows it
is not merely a budget worry: it costs 10-15 % of the distance-row rate and is
the most likely cause of the one seed on which the SSB-enabled arms did worse
than the baseline.

## 5. What is NOT claimed

- **Not a formal guarantee.** SSB rows are soft (own slack, price `rho_slack_ssb`)
  unless `ssb_slack_max` is set to 0. A satisfied QP row is not a proof of
  collision avoidance, and §1 means SSB cannot restore the clearance the
  deletion already cost.
- **No single-seed comparison of arms.** On `ssb_poke` seed 0 the arms gave
  `min_h` −0.190 (baseline), −0.086 (observe-only), −0.087 (acting), −0.090
  (support 6). The observe-only arm builds *no rows* yet matches the acting arm,
  so that seed's spread is run-to-run variance, not SSB — consistent with this
  harness's recorded σ = 2.27 cm. **No claim that SSB improves clearance is made
  in this report**, because the seeds needed to support one are still running.
- **Not a reason to loosen `depth_gate_tol_m`.**
- The 15 % of arm pixels the sim reports *in front of* the 16 cm gate, and
  `self:0` on every frame (the sim's calibration residual is −0.2 cm against a
  6 cm bias), mean **the simulator does not reproduce the false-positive
  pressure the real rig has.** SSB's specificity result above is a sim result.

## 6. The 24-cell grid (executed): 2 scenarios x 4 arms x 3 seeds

Batch `franka_simulation_experiments/sim_runs/batches/ssb_p2`, **24/24 cells
ok**, every cell inside the QP-rate gate (98-99 Hz), `fallback_level` 0
everywhere -- no infeasibility and no brake in any arm. Mean +/- sd over 3 seeds
(`log/ssb_work/ssb_table.py`).

| scenario | arm | min true gap [m] | t < d_safe [s] | t < 0 [s] | SSB rows max | ticks with a row | band frames in / out | slack obs | slack ssb |
|---|---|---|---|---|---|---|---|---|---|
| ssb_poke | minimal_l1 | -0.0468 +/-0.032 | 2.29 +/-0.96 | 1.50 +/-0.85 | 0 | 0 | - / - | 0.000 | 0 |
| ssb_poke | ..._ssb_obs | -0.0150 +/-0.021 | 1.39 +/-0.27 | 0.61 +/-0.44 | 0 | 0 | 46 of 72 / **0 of 57** | 0.069 | 0 |
| ssb_poke | ..._ssb | **-0.0072 +/-0.014** | 1.40 +/-0.18 | **0.48 +/-0.35** | 4.0 +/-0.0 | 18 +/-2 | 50 of 75 / **0 of 57** | 0.303 | 0.0103 |
| ssb_poke | ..._ssb_sup6 | -0.0164 +/-0.020 | 1.48 +/-0.25 | 0.59 +/-0.45 | 4.0 +/-0.0 | 6 +/-2 | 41 of 70 / 0 of 61 | 0.089 | 0 |
| ssb_band_sweep | minimal_l1 | +0.1257 +/-0.001 | 0.00 | 0.00 | 0 | 0 | - / - | 0.470 | 0 |
| ssb_band_sweep | ..._ssb_obs | +0.0708 +/-0.041 | 0.84 +/-0.61 | 0.00 | 0 | 0 | 54 of 552 / **0 of 109** | 0.496 | 0 |
| ssb_band_sweep | ..._ssb | +0.1023 +/-0.033 | **0.34 +/-0.48** | 0.00 | 2.7 +/-1.9 | 10 +/-10 | 54 of 537 / **3 +/-4.7 of 109** | 0.785 | 0.0010 |
| ssb_band_sweep | ..._ssb_sup6 | +0.0959 +/-0.040 | 0.51 +/-0.73 | 0.00 | 1.3 +/-1.9 | 9 +/-12 | 44 of 534 / 0 of 112 | 0.951 | 0 |

**How this must be read.**

- **No difference between any pair of arms is established by these 3 seeds.**
  Welch and paired t-tests on the per-seed values (`log/ssb_work/ssb_stats.py`),
  two-sided, on the minimum true gap:

  | scenario | comparison | d(mean) | Welch p | paired p |
  |---|---|---|---|---|
  | ssb_poke | baseline -> obs | +0.0318 m | 0.313 | 0.484 |
  | ssb_poke | **obs -> acting** | **+0.0078 m** | **0.686** | **0.310** |
  | ssb_poke | baseline -> acting | +0.0396 m | 0.215 | 0.342 |
  | ssb_band_sweep | baseline -> obs | -0.0549 m | 0.196 | 0.191 |
  | ssb_band_sweep | **obs -> acting** | **+0.0315 m** | **0.445** | **0.261** |
  | ssb_band_sweep | baseline -> acting | -0.0234 m | 0.422 | 0.420 |

  Every p >= 0.19. The eye-catching baseline-to-`_ssb_obs` gap in the table
  above is **not** a measured effect either, and must not be quoted as one.
- The **clean one-key comparison is `_ssb_obs` vs `_ssb`** (`ssb_rows_enabled`,
  asserted by `test_conditions.py::test_the_ssb_arms_differ_in_exactly_one_key`).
  `minimal_l1` and `_ssb_obs` also differ *only* in SSB keys -- verified by
  flattening both resolved conditions and diffing; an earlier draft of this
  report claimed a `perception.tracking` confound between them, and that was
  **wrong**. There is no confound; there is simply no resolving power at n = 3.
- Direction, offered as direction only: the acting arm beats the observe-only
  arm on both scenarios and on every metric (min gap -0.0150 -> -0.0072 m,
  penetration 0.61 -> 0.48 s; min gap +0.071 -> +0.102 m, time inside `d_safe`
  0.84 -> 0.34 s). Seeds 3-9 are running to test whether that survives n = 10.
- **The perception half is not free, and on one seed it hurt.** On
  `ssb_band_sweep` the baseline is nearly deterministic (min gap +0.125 on all
  three seeds, sd 0.001) while *all three* SSB arms collapse on seed 2
  (+0.056 / +0.028 / +0.039). The failure is shared by the arm that builds no
  rows, so the cause is the **measurement** pass, not the barrier. The
  mechanism is measured: the published distance-row rate drops from 34-39 Hz
  (baseline) to 29-33 Hz on `ssb_poke` when `distance.ssb_enabled` is on -- a
  10-15 % loss of perception frames, consistent with `ssb_ms` p95 ~9.5 ms
  against a ~28 ms frame period. **Turning on the SSB measurement pass can make
  avoidance worse by slowing perception.** That is an argument for making the
  geometry pass cheaper before anything else, and it is why the feature stays
  off by default on evidence rather than on caution.
- **False positives are not zero.** On `ssb_band_sweep` the acting arm produced
  band candidates outside the window on 3 +/-4.7 frames of 109, i.e. one of the
  three seeds fired ~8 spurious frames. `ssb_poke` had 0 of 57 in all arms. The
  "no false positives" result of §4 is a `ssb_poke` result and does not
  generalise.
- The SSB family stays cheap and contained: `slack_ssb` <= 0.0103 against
  `slack_obs` 0.07-0.95, `ssb_rows` <= 4, no fallback anywhere. But the acting
  arm also consumed **more** obstacle slack than the observe-only arm on both
  scenarios (0.069 -> 0.303, 0.496 -> 0.785), which is the conservatism cost
  showing up where it should and is not yet explained.
- `ssb_min_support_px: 6` is confirmed support-limited rather than inert: 4 +/-2
  supported frames versus 18 +/-1 at the derived value of 3.

## 7. A prediction, recorded before the sweep that tests it

Written down here **before** the sensitivity batch finished, so that it is a
prediction and not a story told afterwards.

`front_m = z_model - bias - z` is each candidate's depth behind the robot
surface, and a candidate needs `guard < front_m < tol` = (0.04, 0.10) m. The
measured per-frame maximum does not spread across that interval; it ramps up
and piles against the FAR edge (0.093, 0.094, 0.095, 0.096, 0.097, 0.098,
0.099, 0.100 m on 1, 2, 1, 4, 4, 8, 10 and 7 frames of
`ssb_poke__minimal_l1_ssb__gz__s0`). That is the annulus geometry again: the
part of the ball that is inside the band is its rim, and the deepest rim pixels
sit as far behind the surface as the band allows.

If that reading is right, then the arm that moves `depth_gate_tol_m`:

- `minimal_l1_ssb_tol20` (tol 0.20, band 14 cm deep) should produce **more**
  candidates per frame and a `front_max` that moves up with the new edge;
- `minimal_l1_ssb_tol08` (tol 0.08, band 4 cm deep) should produce **fewer**,
  and its `front_max` should pile against 0.08 instead;
- `minimal_l1_ssb_guard12` (guard 0.12 > tol 0.10, band empty) should produce
  **none at all** — and if it produces any, the classifier is not reading the
  band it claims to read and every positive result above is void.

## 8. Not executed
- §7.1 obstacle-slack sweep. Confirmed again here that the obstacle row is paid
  for rather than enforced (`violation_without_slack_obs` > 0 on 34.5 % of ticks,
  max 0.47, matching the 34.8 % already on record), but the sweep that decides
  whether it is *under-priced* or *box-infeasible*
  (`log/ssb_work/run_slack_sweep.sh`: `rho_slack_l1` ∈ {1e3, 1e4, 1e5} plus
  `obstacle_slack_max=0`) has **not** been run, so no correction has been made.
- §8 control-point coverage. Measured and diagnosed, not changed: segments 0–2
  (`fr3_link0→1`, `1→2`, `2→3`) declare `control_points: 0` in
  `fr3_complete.yaml`, so links 0–3 carry no protected point at all. This is a
  config choice, not a code limit; raising it would add obstacle rows too, which
  is outside this change.
- Sensitivity to depth tolerance, mask dilation, `d_safe`, obstacle speed,
  camera noise and missing measurements.
- Static-obstacle cells, path-deviation and task-completion metrics.
