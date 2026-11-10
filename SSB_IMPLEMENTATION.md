# The Self-Shadow Barrier: implementation reference

Companion to `SSB_MATHEMATICAL_DERIVATION.md` (the maths and its corrections)
and `SSB_VALIDATION_REPORT.md` (what was measured). This document is the map:
which file does what, which parameter controls it, and what is guaranteed when
the flags are off.

**Both halves are OFF by default.** `distance.ssb_enabled` arms the
measurement; `cbf.ssb_rows_enabled` arms the constraint. Neither implies the
other, and the measurement half alone builds no row.

---

## 1. Files

| file | role | state |
|---|---|---|
| `franka_experiments/franka_experiments/utils/self_shadow.py` | **new.** Ray classification, candidate extraction, the barrier and its row, the void-ball covering. Pure numpy (+ optional scipy), no ROS. | ~430 lines |
| `…/utils/mask_builder.py` | **modified.** Optionally keeps the robot's mesh samples in camera frame (`model_points_cam`) for the 3-D model test. | +12 lines |
| `…/utils/distance_engine.py` | **modified.** Runs the shadow pass inside the depth-gate branch and annotates each `ControlPointResult` with five shadow fields. | +~120 lines |
| `…/nodes/real_time_distance.py` | **modified.** Config, warnings, `SSBDIAG` log line, passes the model points, publishes the shadow fields. | +~75 lines |
| `franka_msgs/msg/LinkDistance.msg` | **modified.** Six appended fields. | +6 fields |
| `…/utils/perception_msgs.py` | **modified.** Writes the shadow fields on rank-0 entries only. | +~20 lines |
| `…/utils/cbf_state_rows.py` | **modified.** `G_SSB`, `Shadow`, `ShadowRowBuilder`, the rows in `ConstraintBuilder.build`, `ConstraintSnap.d_ssb_min`/`n_ssb`. | +~200 lines |
| `…/utils/cbf_diagnostics.py` | **modified.** `FAMILIES_ALL`, schema v3 (three appended fields). | +~25 lines |
| `…/nodes/cbf_safety_filter.py` | **modified.** Parses the wire fields, prices the family, writes the v3 status tail. | +~45 lines |
| `…/utils/logging_utils.py` | **modified.** `ssb[n/d/s]` on the CBFDIAG line. | +8 lines |
| `franka_simulation_experiments/scripts/simexp/closed_loop.py` | **modified.** Scores the new diag key. | +~25 lines |
| `franka_simulation_experiments/config/scenarios.yaml` | **modified.** `ssb_poke`, `ssb_band_sweep`. | +2 scenarios |
| `franka_simulation_experiments/config/conditions.yaml` | **modified.** `minimal_l1_ssb_obs`, `minimal_l1_ssb`. | +2 arms |

Tests: `test_self_shadow_geometry.py` (20), `test_self_shadow_derivatives.py`
(9), `test_ssb_perception.py` (15), `test_ssb_qp_rows.py` (17).

---

## 2. Data flow

```
depth frame ─► MaskBuilder.rebuild()
                 │  search_exclusion_mask (dilated silhouette)
                 │  robot_depth            (z-buffer of the model)
                 │  model_points_cam       (mesh samples, only if stride > 0)
                 ▼
            DistanceEngine.compute()
                 │  depth-gate branch, BEFORE `excluded &= ~in_front`
                 ├─► _shadow_pass()
                 │     1 classify_masked_rays   → 6 classes, counts
                 │     2 reject_model_explained → drop what the mesh explains
                 │     3 cluster_support        → drop speckle
                 │     └─► ShadowCloud (p_cam, u, v, z, counts, front_m)
                 │
                 ├─► nearest_candidate_per_cp() → (gap, idx) per control point
                 └─► _annotate_shadow()         → 5 fields on each result
                 ▼
         build_cp_messages()  → LinkDistance.shadow_* (rank 0 only)
                 ▼
     cbf_safety_filter._on_distances()  → Shadow tuples (independent of `valid`)
                 ▼
         ConstraintBuilder.build()
                 └─► ShadowRowBuilder.build()  → rows in family G_SSB
                 ▼
                OSQP
```

Two ordering details are load-bearing:

* **`_shadow_pass` runs before `excluded &= ~in_front`.** After that line the
  in-front pixels have been removed from the mask, and the information about
  what was *deleted* is gone.
* **The per-control-point shadow gap is computed before the "no obstacle pixel"
  early return.** A hand entirely inside the dilated silhouette produces no
  obstacle pixel at all, so every entry of that frame has `valid = False` —
  which is precisely the frame the shadow is the only evidence on. The filter
  node therefore parses `shadow_*` **before** its `if not ld.valid: continue`.

---

## 3. Parameters

### 3.1 `fr3_complete.yaml`, `distance:` — the measurement half

| key | default | meaning |
|---|---|---|
| `ssb_enabled` | `false` | arms the whole measurement half. Off ⇒ no cloud, no stats, no fields, and the obstacle path is bit-identical. |
| `ssb_guard_m` | `0.04` | how far in front of the debiased surface a return must be to be a candidate. ≈ 2σ of the measured residual. With 0 roughly half the arm qualifies (`depth_gate_bias_m` is a median). |
| `ssb_model_clearance_m` | `0.05` | 3-D rejection radius: a candidate within this of any sampled mesh point is the robot. |
| `ssb_model_points_stride` | `8` | subsampling of the mesh points used for that test. 0 keeps none (and is what the flag-off path uses, making `mask_builder` bit-identical). |
| `ssb_min_support_px` | `3` | minimum connected blob, in **grid samples** on the engine's sampled grid -- one sample is `pixel_step`² = 36 px, so the threshold is 36x coarser than the name suggests. The first value shipped here was 6, and it rejected **every** candidate on **every** frame of the first Gazebo grid (`max_blob` peaked at 3): a sphere's near-surface depth varies by its own radius across its disc, so only an *annulus* of it is inside the 6 cm band at any instant -- 1-5 samples, not the ~11 of the whole disc. 3 is the smallest value that still rejects isolated speckle, since a 2-sample connected blob already spans 12 px. The old value is kept as the `minimal_l1_ssb_sup6` arm, which measures 4±2 supported frames against 18±1 at 3. |
| `ssb_log_throttle_s` | `0.5` | throttle of the `SSBDIAG` line. |

### 3.2 `fr3_control.yaml`, `params:` — the constraint half

| key | default | meaning |
|---|---|---|
| `ssb_rows_enabled` | `false` | arms the `G_SSB` family. |
| `ssb_d_safe` | `0.05` | the family's **own** safety distance. Smaller than `d_safe` (0.10) because a shadow row engages close by construction and a large value would make a false positive violent rather than merely conservative. |
| `ssb_row_horizon` | `0.20` | gap below which a shadow produces a row. |
| `ssb_row_release` | `1.5` | hysteresis multiplier on the way out. |
| `ssb_max_rows` | `4` | bounded worst case. |
| `ssb_hold_s` | `0.10` | survival of a frame with no measurement. |
| `ssb_inflate_speed_mps` | `0.20` | erosion of the held surface per second of age. |
| `rho_slack_ssb` | `400.0` | slack price. **Below** `rho_slack` (1000) so that when a shadow row and an obstacle row disagree, the obstacle row — backed by a measurement the pipeline believed — wins. |
| `ssb_slack_max` | `1e6` | cap on the family's slack. `0.0` makes the rows **hard**, which can render the QP infeasible; the fallback ladder then reports it. |

---

## 4. Backward compatibility, and exactly what is guaranteed

| claim | how it is checked |
|---|---|
| Both flags off ⇒ the constraint matrices are **bit-identical**, including with shadow measurements present on the wire | `test_ssb_qp_rows.py::test_flag_off_is_bit_identical_even_with_shadows_on_the_wire` (`array_equal` on `A`, `h_bar`, `G`, `group`, `v_obs`, `links`) |
| Rows armed but nothing shadowed ⇒ still bit-identical | `…::test_flag_on_with_no_shadow_is_bit_identical_too` |
| `distance.ssb_enabled` off ⇒ no cloud, no stats, no fields; obstacle path identical | `test_ssb_perception.py::test_the_flag_off_is_inert`, `…::test_the_obstacle_path_is_unchanged_with_ssb_on` |
| The `ssb` slack **column** exists either way | by construction (`N_SLACK = 7`), so the decision-vector layout is a constant of the build rather than a function of a flag, and a flags-off run is comparable with a flags-on one |
| Every pre-existing family keeps its index; the three v3 status fields are appended **last** | `test_ssb_qp_rows.py::test_the_family_is_appended_not_inserted`; `FAMILIES_ALL == GROUP_NAMES` is asserted |
| No regressions | full `franka_experiments` suite: **35 failed / 1389 passed / 9 skipped**, against a pre-existing baseline of **35 failed / 1329 passed / 9 skipped** — `comm -23` on the sorted FAILED lists is empty, i.e. zero new failures. The 35 are pre-existing and unrelated (launch-file include tests). |

---

## 5. Diagnostics

**`SSBDIAG`** (from `real_time_distance`, every `ssb_log_throttle_s`):

```
SSBDIAG cls[candidate:N/kept:N/outside:N/self:N/through:N/void:N]
        unknown_frac=F band=N unexplained=N supported=N model_pts=N
        front_max=F cloud=N n_cp_shadow=N d_shadow_min=F
```

`band → unexplained → supported` are the three filters' survivor counts, in
order, so a candidate lost to the mesh test and one lost to the blob test are
distinguishable. This line is the **only** record the mechanism leaves when
`ssb_rows_enabled` is off: with no row the constraint builder never runs, so
`CBFDIAG`'s `ssb[n/d/s]` is `0/inf/0.000` by construction in that arm.

**`CBFDIAG`** gains `ssb[n/d/s]` = rows built / closest shadow gap / family
slack. Appended as its own key rather than widened into
`s[obs/sc/qlim/sing/cap/spd]`, because every existing parser of that bracketed
group counts six.

**`/cbf_status`** schema v3 appends `slack_ssb`,
`violation_without_slack_ssb`, `violation_with_slack_ssb` after
`constraint_build_wall_ms`. A v2 bag still zips correctly against the first 38
names.

**`LinkDistance`** gains `shadow_valid`, `shadow_distance`, `shadow_point`,
`shadow_direction`, `shadow_front_m`, `shadow_unknown_frac`. The default
(`shadow_valid = false`) is the safe one and there is no value of any of these
fields that can *loosen* a constraint.

---

## 6. Experiment arms and scenarios

All in `franka_simulation_experiments/config/`. Each arm changes exactly **one**
key from the arm above it, asserted by `test/test_conditions.py`
(`test_the_ssb_arms_differ_in_exactly_one_key`,
`test_each_ssb_sensitivity_arm_moves_exactly_one_parameter`) so that a measured
difference is attributable.

| condition | differs by | purpose |
|---|---|---|
| `minimal_l1` | — | the A baseline, untouched |
| `minimal_l1_ssb_obs` | `distance.ssb_enabled` | measurement half only: does SSB *see* the blind region, and what does classifying alone cost |
| `minimal_l1_ssb` | `cbf.ssb_rows_enabled` | the acting arm |
| `minimal_l1_ssb_sup6` | `ssb_min_support_px` 6 | the rejected support threshold, kept as measured evidence |
| `minimal_l1_ssb_guard12` | `ssb_guard_m` 0.12 | **falsification arm**: with `guard ≥ tol` the band `(bias+guard, bias+tol)` is empty, so SSB must classify nothing. `ssb_guard_m` is SSB-only, so the self-filter and the obstacle pipeline are untouched — this is also a clean "present but inert" control. |
| `minimal_l1_ssb_tol08` / `_tol20` | `mask.depth_gate_tol_m` | sensitivity to the depth tolerance: band 4 cm / 14 cm deep instead of 6. A **shared** parameter — the self-filter changes too. |
| `minimal_l1_ssb_dil8` | `mask.ee_mask_dilate_px` 8 | sensitivity to the gripper dilation. Shared parameter. |

The band is non-empty exactly when `guard < tol`; the bias cancels. Getting that
wrong is how a draft of this file mislabelled the `tol08` arm as the empty-band
arm, and `test_the_empty_band_arm_really_has_an_empty_band` now asserts the
inequality rather than trusting prose.

| scenario | obstacle | purpose |
|---|---|---|
| `ssb_poke` | 4 cm ball, 0.08 m/s along the camera ray | the reference crossing: 0.75 s in the band |
| `ssb_band_sweep` | same ray, oscillating, T = 32 s | repeated long dwells in the band |
| `ssb_poke_fast` / `_slow` | 0.16 / 0.04 m/s | obstacle-speed sensitivity; the fast crossing is ~12 perception frames |
| `ssb_poke_noisy` | `depth_adapter.range_noise` | camera-noise sensitivity |
| `ssb_poke_dropout` | `depth_adapter.dropout_p` 0.1 | missing measurements. Dropped pixels inside the silhouette must land in `RAY_VOID`, not `RAY_CANDIDATE` — this is the cell that tests "a missing measurement is not evidence of free space" against the implementation rather than against the docstring. |

## 7. Cost, and the one algorithmic choice that mattered

The hot path per perception frame is: classify the masked rays, unproject the
candidates, reject the ones the robot's own mesh explains, require cluster
support. Measured per call at the shipped sizes (13500 model samples,
`pixel_step` 6, `log/ssb_work/bench_ssb.py`, `OMP_NUM_THREADS=1`):

| stage | cost |
|---|---|
| `classify_masked_rays` (2000 samples) | 0.15 ms |
| `unproject` | 0.06 ms |
| `cluster_support` | 0.62 ms |
| `reject_model_explained`, **as first written** | **5.09 ms**, of which **4.84 ms was building a cKDTree** and 0.22 ms querying it |
| `reject_model_explained`, now | **0.55 ms**, flat in candidate count |

The tree was rebuilt every frame, because the model samples move with `q`. At
the 1–5 candidates the runs actually report, paying `O(N log N)` to serve five
queries is the wrong trade by 2–16×:

| candidates | 1 | 3 | 5 | 10 | 25 | 100 | 425 |
|---|---|---|---|---|---|---|---|
| brute force | 0.32 | 1.16 | 1.12 | 2.16 | 5.84 | 53.6 | 257.3 ms |
| tree (build + query) | 5.29 | 2.66 | 1.89 | 2.07 | 1.82 | 4.96 | 4.94 ms |

so the function now brute-forces while `n_candidates × n_model ≤ 120000` and
builds the tree above that. Brute force alone was only ~2× better, though; what
made it ~9× was an **exact** prefilter: a model sample within `clearance` of any
candidate must lie inside the candidates' bounding box inflated by `clearance`,
so everything outside is discarded without changing the answer. One frame's
candidates sit on one obstacle a few centimetres across, so 13500 samples become
a few hundred.

This is a performance switch inside the filter that stops the arm being
classified as its own shadow, so
`test_ssb_perception.py::test_the_two_model_rejection_paths_agree_exactly`
checks the two branches against each other on the same data, including
candidates placed exactly at the clearance radius where a `>` and a `>=` would
disagree. Writing that test also surfaced a real bug: the documented
"pass a prebuilt cKDTree" path raised `TypeError`, because the size guard used
`len()` and a `cKDTree` has only `.n`. That path had never been exercised.

## 8. Known limitations

1. **The `VOID` regime builds no row.** Classified, counted, published as
   `shadow_unknown_frac`; the covering-ball machinery exists and is tested, but
   a `VOID` slab is unbounded in depth and reducing it to balls is a modelling
   choice that has not been validated. §4.3 of the derivation.
2. **The deleted band is inside `d_safe`.** See §6 of the derivation: at the
   shipped calibration SSB cannot *prevent* the violation, only bound the
   approach after it.
3. **Links 0–3 carry no protected point at all.** Measured, not assumed
   (`log/ssb_work/cp_coverage.py`): the control set is 11 points on
   `fr3_link4`, `5`, `6`, `7`, `8`, and the cause is one column of
   `robot.segments` in `fr3_complete.yaml` — segments 0, 1 and 2
   (`fr3_link0→1`, `1→2`, `2→3`) declare `control_points: 0` while segments
   3–7 declare 2, 2, 2, 2 and 3. So this is a **configuration** choice, not a
   code limitation: `define_control_points` honours the column, and raising
   those three zeros is a one-line change.
   It is left alone here because those points feed the *ordinary* obstacle rows
   as well, so changing them would move the baseline every SSB arm is compared
   against. Nothing in this work is specific to the upper arm — SSB reuses the
   control set and its Jacobians unchanged — so the gap is inherited, not
   introduced. No scenario aimed at the lower arm was run, so there is **no
   measurement** of how much a lower-arm shadow would matter.
4. **The "Gray Area" baseline of §10.1 cannot be run** — that mechanism has
   been removed from the repository. The comparison is A (`minimal_l1`),
   C (`minimal_l1_mem`, the occlusion-aware memory) and D (`minimal_l1_ssb`),
   plus the observe-only arm `minimal_l1_ssb_obs`.
5. **`shadow_unknown_frac` is a frame-level scalar** stamped on every entry; it
   is not per control point.
