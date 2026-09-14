# ISO alignment roadmap — implementation report

Implementation of `franka_experiments/ISO_COMPLIANCE_ROADMAP.md` (the v2
roadmap, the one carrying the `[R]`/`[S]`/`[E]` tags; the superseded v1 lives
outside the repo at `~/Downloads/ISO_COMPLIANCE_ROADMAP_v1_superseded.md`).

**Date:** 2026-09-14  **Package:** `franka_experiments`

**No ISO compliance or certification is claimed.** What is claimed, per item, is
in `franka_experiments/SAFETY.md`. Every `iso_*` flag ships `false`; with them
off the filter's numerical output is unchanged, and that is enforced by
`test/test_iso_flags_off_identical.py` rather than asserted in prose.

---

## 1. Implemented steps

### Step 1 — measure the ISO input constants ✅

`scripts/iso_constants_measure.py`, four sub-commands plus a `yaml` writer.

| sub-command | tag | what it does |
|---|---|---|
| `reaction` | [E] | shells out to `latency_budget.py perception`, parses the p95 total, reports `T_r = p95 + 1/qp_rate_hz + 0.001` |
| `stop` | [S] procedure, [E] values | drives the arm over the Annex H 33/66/100 % grid × 2 directions, commands `q̈ = −q̇/τ` clipped to `±decel_max`, reports per-run `v_tcp`, `T_s`, `S_s`, `a_s`, and the **minimum** `a_s`. Refuses without `--i-am-supervising` |
| `detection` | [R] input for `C` | records a detection-capability test and derives `C` per ISO 13855:2024 (`C = 8·(d−14)` mm for `d ≤ 40` mm, else 850 mm). Without arguments it prints the procedure |
| `uncertainty` | [E] | p95 of `calibration_check.calibration_residual` against a static target → `Z_d` |
| `yaml` | — | assembles the paste-ready block from the JSON results file, flagging every value still at its placeholder |

`iso_z_robot` is documented as a datasheet transcription (FR3 product manual,
"worst case safe Cartesian position accuracy for stopping functions"), not
something the script can measure.

### Step 2 — ISO parameter block ✅

- `config/fr3_control.yaml`: the full `iso_*` block inside `params:`, every entry
  tagged, with the long notes on `C`, `v_h` and reduced speed carried verbatim.
- `utils/config.py`: 21 matching `CBF_PARAM_SPEC` entries with the ranges the
  roadmap specifies (`iso_a_stop` ≤ 20, `iso_v_human` ≤ 5, distances ≤ 2, times
  ≤ 5, `iso_mode` a choice, the two counters `int` positive ≤ 1000).
- `launch/torque_control_stack.launch.py` + `config/launch_defaults.yaml`:
  `iso_enabled`, `iso_mode`, `iso_ssm_speed_rows`, `iso_monitor_enabled`,
  `torque_iso_monitor_delay_s`, all defaulting off/`automatic`.

### Step 3 — `utils/iso_ssm.py` ✅

Exactly the four specified functions, numpy only, no classes, no state:
`stopping_distance`, `protective_separation`, `ssm_speed_cap`, `pfl_speed`.
17 tests pin the exact inversion, the stop region, the three monotonicities, the
NaN-free boundary cases and the worked `v_PFL` example.

One deliberate addition: `ssm_speed_cap` short-circuits `d ≤ c+z_d+z_r` to
literal `0.0`. The quadratic alone returns `9.7e-17` at exactly that boundary,
and a cap of `1e-17` reads as "creep allowed" to every consumer.

### Step 4 — barrier offset = irreducible part of `S_p` ✅ (with a documented deviation)

`cbf_safety_filter._iso_configure` — one place, run before anything reads `P`:

1. raises `ValueError` naming `d_safe`, `C`, `Z_d`, `Z_r`, the floor, and all
   three admissible resolutions, when `d_safe < C + Z_d + Z_r`. No bypass flag.
2. sets `velocity_standoff_time_s = T_r + v_h/a_s` and
   `velocity_standoff_max = v_h·(T_r+T_s)`, forcing `enable_velocity_standoff`
   on — this is `S_h`.
3. (Step 9) raises when `link_speed_max > iso_v_pfl` or the ordering invariant
   `retreat_cap_max_speed < link_speed_max` is broken.

**Deviation — see §3.1.** `d_safe` stays at 0.10 m rather than being raised to
the 0.92 m floor.

### Step 5 — SSM cap drives the task-space speed rows ✅

`utils/cbf_state_rows.py`: `spd_pts` now carries `(Jp, clearance, label, v_app)`
so the row consumes the **same** conditioned closing speed the obstacle row and
the retreat cap do — passed through, never recomputed. With
`iso_enabled and iso_ssm_speed_rows`, the obstacle term becomes
`ssm_speed_cap(...)`; the self-collision term is untouched (not an ISO distance)
and the two are still combined with `min()`.

`rho_slack_link_speed` is raised 500 → 2000 (above `rho_slack` = 1000) in the
filter when the flag is on (`_ISO_RHO_SLACK_SPEED`). The row stays
slack-relaxable, as specified.

### Step 6 — independent SSM monitor and non-safety-rated stop ✅

New node `nodes/iso_safety_monitor.py` (+ `setup.py` console script,
`std_srvs` in `package.xml`):

- own `CBFKinematics` over `build_urdf_no_hand`, own copy of the constants;
- trips on: closing speed > cap + tol, inside `C+Z_d+Z_r`, `cbf_status[2] == 1`,
  each after `iso_monitor_ticks` consecutive ticks;
- publishes `/NS_1/iso_safety` = `[stop_latched, trip_reason, S_p, v_cap_min,
  v_closing_max]` **every tick**;
- `/NS_1/safety_reset` (`std_srvs/Trigger`) refuses while any condition is live.

Consumers: `cbf_safety_filter._qp_tick` STEP 0 skips the rows and brakes on
`−q̇/iso_stop_tau` through `box_only_solve` (never zeros — with firmware gravity,
zero torque is coasting); `pentagon_qddot_commander._tick` publishes `q̈_nom = 0`
and holds the phase at `σ = 0`, below the `governor_sigma_min` floor.

The term *protective stop* is not used anywhere; the node's docstring says why.

### Step 7 — fail-closed perception ✅

1. **Contact-regime publish hole** — `real_time_distance` splits `in_band`
   (legacy `MultiDistance`/logging) from `publishable` (safety topic, no lower
   bound). Gated on `distance.publish_contact_regime: true` in
   `fr3_complete.yaml`.
2. **Unbounded hold** — `DistanceEngine` accumulates a per-CP hold age; past
   `iso_distance_hold_max_s` the result is emitted with `direction = None`, which
   `build_cp_messages` already turns into `valid = False`.
3. **Dead confidence gate** — `LinkDistance.confidence` now carries
   `find_pt_confidence(...)`, except in the contact regime where it is a flat
   0.5 (above `min_confidence`, below any clean measurement). Verified to drop
   **no** row at `min_confidence = 0.2`.
4. **Empty-frame run** — `cbf_safety_filter._empty_frame_fault` faults when a run
   of empty frames exceeds `iso_empty_frame_max_s` and the last non-empty frame
   had `min h < zone_r_active·d_safe`. Checked **before** the row branch, because
   with joint-limit rows on an empty distance frame still yields a fresh,
   non-empty snapshot of joint-limit rows only.

### Step 8 — command feasibility ✅

`qddot_to_torque._limit_and_report`: effort clip from
`load_franka_joint_limits`, then `|Δτ| ≤ iso_tau_rate_max`, then
`/NS_1/torque_saturation` (7 flags). `max_tau_delta` left in place as the dead
legacy key it is, with its TODO updated to name the live one.

`cbf_safety_filter._brake_authority_fault`: `frac = (q̈_real·q̈_cmd)/‖q̈_cmd‖²`
below `iso_brake_frac_min` for `iso_brake_frac_ticks` → `fault_braking = 1` and
an ERROR with both vectors. Diagnostic only, no stop, as specified.

### Step 9 — absolute ceilings ✅ (with a documented deviation)

`scripts/iso_pfl_speed.py` computes `iso_v_pfl` through `iso_ssm.pfl_speed`,
prints every input with its tag, and prints all three required warnings
(quasi-static vs transient, mass, and `[R]` PFMD measurement per
ISO 10218-2:2025 6.3.3 / Annex N). A convenience copy of the most-cited Annex M
rows is included with an explicit "do not cite this dict" note.

Reduced speed: an extra `G_SPD` row at `iso_tcp_reduced_speed` on `FR3_TCP_LINK`
(falling back to the furthest reporting CP), built with the existing
`link_speed_row`; `iso_safety_monitor` applies the same cap to the same link.
Mode logged at startup.

**Deviation — see §3.2.** `link_speed_max` stays at 1.3 in the YAML.

### Step 10 — status contract, diagnostics, logging ✅

- `/NS_1/cbf_status` gains `data[5..8] = S_p, v_cap_min, v_closing_max,
  iso_stop_latched`, appended; the contract comment in `fr3_control.yaml` is
  updated. The monitor's numbers win when fresh, the filter's own are the
  fallback, and `v_closing_max` is measured from the obstacle rows directly.
- CBFDIAG gains `sp= vcap= vcls= isostop=` (with an `inf`-safe formatter, since
  `vcap` is legitimately infinite with the rows off).
- `experiment_logger`: 10 new scalar columns + `tau_sat_1..7`, from
  `/NS_1/cbf_status`, `/NS_1/iso_safety` and `/NS_1/torque_saturation`. Filter
  view and monitor view logged separately — when they disagree, that is the
  interesting part of the run.

### Step 11 — hardware safety layer ✅

- `config/safety/watchman_profile.md`: SLS-J table derived from
  `velocity_box_margin · qdot_max` + 20 %, SLD/SLP-J, safe inputs X3.1/X3.2/X3.3
  and the enabling device, an Annex H stopping-data table to fill in, the
  Class II / PL d context, and a **VERIFY, DO NOT ASSUME** section on FCI
  compatibility with SLS-J and SLD, with sign-off lines.
- `scripts/iso_preflight_check.py`: fails on placeholder constants, on a lowered
  `C` with no detection record, on `d_safe < C+Z_d+Z_r`, on
  `link_speed_max > iso_v_pfl` or a broken ordering invariant, and on
  `/NS_1/iso_safety` silent within `--wait`.
- The launch file runs it at `t = 0`, **before** the controller spawner, and
  `Shutdown`s the launch on a non-zero exit.

### Step 12 — `SAFETY.md` ✅

`franka_experiments/SAFETY.md`: the 5-column requirement table (clause / tag /
file+symbol / status / residual gap), a 5-entry deviation register (D1 `C`, D2
`d_safe`, D3 calculated PFL, D4 cell-wide reduced speed, D5 manual reset latch),
and 8 residual gaps including all five the roadmap lists. Plus a "turning it on"
checklist.

### Beyond the roadmap — evidence recording and analysis

Added on request, after the 12 steps: a way to find out from a real run whether
the cell respects the limits its configuration claims.

- `franka_experiments/nodes/iso_evidence_logger.py` — a PASSIVE observer
  (subscribes, computes, writes; publishes nothing). Per sample it records TCP
  Cartesian speed, the fastest control point, the worst joint-speed ratio, the
  separation margin `d - S_p` per control point, the SSM cap excess, torque
  saturation, the realized/commanded acceleration ratio, and what the stack
  itself said on `cbf_status` / `iso_safety`. It writes a run directory with a
  **manifest** (every threshold the run was judged against, plus the git SHA), a
  rate-sampled **CSV**, and a sparse **event log** so a 30 ms excursion is not
  lost to sampling. It evaluates the ISO criteria **whether or not the ISO layer
  is on** — which is what makes it useful with the flags off.
  Independent of `iso_safety_monitor` by design: two readings of one scene, and
  `check_two_channels` compares them.
- `scripts/iso_evidence_report.py` — turns a run directory into a verdict per
  check: **PASS / FAIL / INCONCLUSIVE / STRUCTURAL / NOT FROM LOGS**, each with
  its clause and tag, plus an explicit list of what logs can never settle
  (PL d, PFMD measurement, detection capability, Cat 0/1 stopping data, the risk
  assessment).
- `test/test_iso_evidence_report.py` — 23 tests on the verdict logic.
- Launch: `start_iso_evidence_logger` (default **false**) +
  `torque_iso_evidence_delay_s`, `iso_evidence_dir`, `iso_evidence_run_name`.

Two design decisions worth recording:

- **`STRUCTURAL` is a separate verdict from `FAIL`.** With the conformant
  `C = 0.85 m` the floor is 0.92 m against a 0.855 m reach, so the separation
  checks are foregone conclusions and would be red on every run for a reason no
  tuning can fix — burying the findings that ARE about the run. The report now
  detects deviation D1 from the manifest, says so once, and reports the margin
  **excluding C** — the part the control system actually governs, and the only
  actionable number in that situation.
- **A channel that never published must read INCONCLUSIVE, not PASS.** Found by
  the live smoke test: `/torque_saturation` was absent and the report said
  "PASS, inside the joint limits in 100.0% of samples", because the logger
  initialised the column to zero and a column of zeros reads as "measured, and
  fine". The logger now writes NaN until the first message on each optional
  channel; `test_an_absent_channel_is_inconclusive_not_a_pass` pins it.

### Beyond the roadmap — the per-experiment log

`experiment_logger` already ran on every experiment and already wrote one
directory per run — but it subscribed to `/NS_1/tracking_qdot` and
`/NS_1/qdot_cmd`, which are **velocity-pipeline** topics that nothing in the
torque stack publishes. Fourteen of its columns were dead in the very stack it
is launched in, and the acceleration pipeline was not logged at all. Extended
rather than replaced — a fourth logger next to `experiment_logger`,
`iso_evidence_logger` and the commander's own CSV would have been worse:

- **Cartesian block** — `tcp_x/y/z`, the orientation quaternion, `tcp_speed`
  and `tcp_omega`, from the Jacobian rather than a finite difference of pose
  (which at 100 Hz is dominated by differencing noise, against a 250 mm/s
  limit). Nothing in the package logged TCP speed before, and joint speed is not
  a substitute: a folded or near-singular arm decouples the two in both
  directions. Fails soft — a missing model costs the Cartesian columns, not the
  run.
- **Acceleration pipeline** — `qddot_nom_1..7`, `qddot_safe_1..7`, their norms,
  and `qddot_delta_norm` = ‖q̈_safe − q̈_nom‖, which is the single number saying
  how hard the barrier fought the task.
- **Per-control-point avoidance** — from `/cbf/per_link_distances`, the
  barrier's own input: valid/total entry counts (the difference between
  "nothing was near" and "perception produced entries the QP threw away"),
  closest gap and its label, fastest tracked obstacle, tracked-entry count.
- **`run_manifest.json`** next to the CSV — configuration, topics, git SHA. A
  CSV with no record of which configuration produced it is a pile of numbers.
- `scripts/experiment_summary.py` — a one-screen digest (motion, torque,
  avoidance, health) with peaks and their timestamps, which also **names every
  channel that was not recording** instead of reporting its absence as zero.
- `test/test_experiment_log_columns.py` — 24 tests on the column contract and
  the summary.

Same principle as the evidence logger, and it is the one that matters in both:
**NaN means the channel was not running; 0 means it was and read zero.** They are
different statements, and collapsing them is what makes a reader conclude the
arm was stationary when in fact nothing was recording it.

---

## 2. Tests performed and results

Everything ran in the `franka_ros2` container
(`python3 -m pytest test -q`, `OPENBLAS_NUM_THREADS=1`), incrementally after
each step and again at the end.

| | baseline (before) | final |
|---|---|---|
| passed | 653 | **828** |
| failed | 1 | **0** |

The baseline failure (`test_rl_policy::test_real_configs_are_in_sync`, a `d_safe`
drift against `franka_sim/config.yaml`) disappeared when `d_safe` was set to
0.15, which is the value `franka_sim` already carried. The suite is now fully
green.

The single failure is **pre-existing and unrelated**:
`test_rl_policy.py::test_real_configs_are_in_sync` — `franka_sim/config.yaml`
has `d_safe: 0.15` against `fr3_control.yaml`'s `0.10`. It fails identically on
a clean checkout (verified with `git stash`), and `franka_sim` is explicitly out
of scope for this roadmap.

### New tests — 174 across 11 files

| file | n | what it pins |
|---|---|---|
| `test_iso_ssm.py` | 17 | exact inversion of `S_p`, exact-zero stop region, monotonicity in `v_h`/`d`/`a_s`, no NaN on boundaries, the `v_PFL` worked example |
| `test_iso_dsafe_floor.py` | 10 | flag off ⇒ not one value touched; the floor raises and names all three terms; the **shipped config refuses to run**; `S_h` parameterisation and its clamp; PFL ceiling raises rather than clamps |
| `test_iso_speed_rows.py` | 12 | rows bit-identical with the flag off; the row carries exactly `ssm_speed_cap` of its own gap; `v_app` reaches it through `spd_pts`; the stop region drives the RHS negative; reduced mode adds exactly one TCP row at 0.25 m/s |
| `test_iso_monitor.py` | 18 | consecutive-tick requirement; two flickering conditions do not accumulate a trip; each of the four conditions trips; the latch holds; auto-resume when unlatched; reset **refuses** while live and clears the streaks |
| `test_iso_perception_failclosed.py` | 17 | contact-regime entries published valid and flagged 0.5; **no row dropped** at `min_confidence = 0.2`; confidence is the real figure; the hold expires (with and without frame stamps) and clears on a finite measurement; the empty-frame fault and its three inert cases |
| `test_iso_torque_limits.py` | 17 | effort clip, rate limit, first-command exemption, `tau_prev` tracks what went out, flag-off passthrough, saturation still reported; braking-authority projection, window, small-command and no-row exemptions |
| `test_iso_pfl_ceiling.py` | 9 | the shipped `iso_v_pfl` **is** what the script computes; all three warnings present; payload lowers it; transient ≈ 2× quasi-static; the configuration the raise is armed on |
| `test_iso_status_contract.py` | 15 | first five `cbf_status` fields unchanged; the ISO tail; monitor wins when fresh, not when stale; `v_closing_max` from the obstacle rows; the four CBFDIAG fields and their off-state; **every column `sample()` writes is in the header** |
| `test_iso_evidence_report.py` | 23 | the four-way verdict; STRUCTURAL vs FAIL under deviation D1; the margin excluding C; the coverage floor that stops a PASS on unrecorded data; an absent channel is INCONCLUSIVE |
| `test_experiment_log_columns.py` | 24 | the per-joint blocks, the Cartesian and avoidance blocks, absent-vs-zero, and the summary's peak/coverage helpers |
| `test_iso_flags_off_identical.py` | 12 | the ground rule itself: shipped config + deliberately "loud" ISO constants ⇒ identical `A`, `h_bar`, `cap_v`, `group`, `v_obs`, labels **and** RHS, at five distances, in reduced mode, and with the velocity standoff on |

### Beyond pytest

- **Build:** `colcon build --packages-select franka_experiments` — clean, after
  every step and at the end.
- **Launch composition:** the `OpaqueFunction` body executed headless with flags
  off (20 actions, no ISO nodes) and on (24 actions, `iso_safety_monitor`
  present). `ros2 launch ... --show-args` lists all six new arguments.
  `iso_monitor_enabled:=true` without `iso_enabled:=true` raises with an
  explanation, as intended.
- **Monitor, live:** `ros2 run franka_experiments iso_safety_monitor` — starts,
  logs its constants, publishes `/NS_1/iso_safety`, registers
  `/NS_1/safety_reset`, and correctly latches `trip_reason = 4` (inputs stale)
  with no robot attached. Fail-closed, as designed.
- **Preflight, live:** fails with the shipped config (3 findings: placeholders,
  `d_safe` floor, PFL ceiling); fails on a corrected config when the monitor is
  **not** running; passes when it is.
- **Config/interface consistency:** scripted check — every `CBF_PARAM_SPEC` key
  has a YAML value, every `iso_*` YAML key has a spec entry, every value is
  inside its declared bounds and of its declared type, every new topic exists in
  the `topics:` block, every new launch default has a declared argument and is in
  `_ALL_PARAMS`, the entry point is in `setup.py` and `std_srvs` in
  `package.xml`. **OK.**
- **Scripts:** `iso_constants_measure.py yaml`/`detection`,
  `iso_pfl_speed.py` (default, `--payload`, `--transient`) and
  `iso_preflight_check.py` all exercised end to end.

---

## 3. Deviations from the roadmap's literal wording

Three, all forced by the roadmap's own ground rule *"with all flags off,
numerical output must be bit-identical to today"* — restated by the task as
*"preserve existing behavior when the new safety/ISO functionality is
disabled"*. In each case the mechanism the step asks for is implemented in full;
what is not done is writing an ISO-derived number into a YAML key that is read
with the flags off.

### 3.1 `d_safe` stays 0.10 m (Step 4 item 2)

The step says *"Set `d_safe` to at least `d_floor`"* — that is resolution **(b)**
of the three it lists. (b) is **not physically available here**: `d_floor = 0.92 m`
exceeds the FR3's 0.855 m reach, which is the roadmap's own stated conclusion
("at `C = 0.85 m` the protective separation distance exceeds this arm's reach,
i.e. ISO-conformant SSM is not achievable in this cell"). Resolution **(c)** is
taken instead: `d_safe` unchanged, `iso_enabled` false, deviation recorded in
`SAFETY.md` §D1/§D2, no ISO claim. The `ValueError` is implemented and armed —
`test_the_shipped_configuration_refuses_to_run_with_iso_enabled` asserts it
fires — and the full derivation is in the `d_safe` comment in
`fr3_control.yaml`. **No bypass flag was added.**

### 3.2 `link_speed_max` / `retreat_cap_max_speed` stay at 1.3 / 1.1 (Step 9 item 2)

Writing `min(link_speed_max, iso_v_pfl) = 0.68` into the YAML would change the
speed rows with every flag off. The **assertion** the step asks for is
implemented and raises (does not clamp); `iso_preflight_check.py` catches it a
process earlier; and two pass-through launch arguments were added so the error
message's instruction —
`link_speed_max:=0.68 retreat_cap_max_speed:=0.60` — works verbatim.

### 3.3 `velocity_standoff_time_s` is overridden at runtime (Step 4 item 3)

`T_r + v_h/a_s` is applied in `_iso_configure` under `iso_enabled` rather than
written into the YAML, for the same reason (`velocity_standoff` is `true` in
`launch_defaults.yaml`, so the key is live with the ISO flags off). Two
`CBF_PARAM_SPEC` **validation ranges** were widened to accommodate the ISO
arithmetic — `velocity_standoff_time_s` 2.0 → 5.0 s, `velocity_standoff_max`
1.0 → 2.0 m. No value moved; a bound that rejects the standard's own arithmetic
is the wrong bound.

### Smaller, non-normative

- `/NS_1/iso_safety` field `[2]` is documented as `S_p` (the demand of the CP
  with the smallest margin) rather than `S_p_min` (the numerically smallest
  `S_p`, which would be the least interesting number in the frame). The
  roadmap's name is quoted in the docstring so it is findable.
- The monitor has a **fourth** trip reason, `REASON_STALE` (4), for missing or
  stale inputs. The roadmap lists three; failing closed on a dead input channel
  was judged to be within the step's intent, and it is what the live smoke test
  above exercises.
- Step 8's effort clip and rate limit are **applied** only under `iso_enabled`;
  `/NS_1/torque_saturation` is published unconditionally, since observability
  changes no number.

---

## 4. Remaining issues and gaps

### Blocking any ISO claim (by design, documented in `SAFETY.md`)

1. **Conformant SSM is unachievable in this cell.** `C + Z_d + Z_r = 0.92 m` >
   FR3 reach 0.855 m. Nothing in software fixes this.
2. **No constant has been measured.** `iso_t_reaction`, `iso_a_stop`,
   `iso_z_depth`, `iso_z_robot` are all placeholders. The tooling exists
   (Step 1); running it needs the robot and a supervised session.
3. **No detection capability demonstrated,** and the depth pipeline has no
   IEC/TS 61496-4-3 assessment — it is not a rated protective device.
4. **PFL is calculated, not measured** with a PFMD (ISO 10218-2:2025 6.3.3,
   Annex N), for one region, with no payload or gripper.

### Implemented but not validated on hardware

5. **Nothing in the ISO layer has run on the FR3.** Every flag ships `false`.
   The roadmap's own sequencing (fake hardware first, then supervised) has not
   been started.
6. **`config/safety/watchman_profile.md` is a specification, not a
   configuration.** Nothing in this repo touches Watchman. In particular,
   **SLS-J / SLD compatibility with FCI is unverified** — the datasheet excludes
   SLP-C, SLS-C and SLP-J and is silent about the other two, and silence is not
   compatibility. If SLS-J turns out to be incompatible with FCI, the rated
   backstop this design assumes does not exist during operation.
7. **The braking-authority check does not stop** (`fault_braking` + ERROR only),
   deliberately, until its false-positive rate on hardware is known.

### Structural, unclosable by this roadmap

8. **The chain cannot be a safety function** — single-channel Python over
   best-effort DDS cannot meet PL d / SIL 2, which a Class II robot's safety
   functions require.
9. **The FR3's rated functions are certified to EN ISO 10218-1:2011,** not the
   2025 editions.
10. **The safe set is not forward-invariant** — obstacle rows stay
    slack-relaxable. The monitor bounds the consequence; it does not restore the
    guarantee. Hard rows are not the fix (infeasible QP exactly when it matters).
11. **`S_h` is clamped** at `velocity_standoff_max = 2.0 m` against a full
    `v_h·(T_r+T_s) = 4.2 m` at the placeholder constants. `_iso_configure` warns.
12. **`a_s` will be a minimum over a finite grid,** not a worst case over all
    payloads, configurations and directions.
13. **`franka_sim/envs/cbf_filter.py` still mirrors the pre-ISO row math** —
    explicitly out of scope per the roadmap's ground rules, and now a wider
    divergence than before. Anything trained against it is trained against the
    pre-ISO filter.

### Pre-existing, untouched

14. `test_rl_policy.py::test_real_configs_are_in_sync` fails on a `d_safe` drift
    between `franka_sim/config.yaml` (0.15) and `fr3_control.yaml` (0.10). Out of
    scope; failing before this work and after it.
15. `test/smoke_cbf_construct.py` is broken at `HEAD` (`NameError: name 'N' is
    not defined`, plus a stale `franka_msgs` import). Verified pre-existing with
    `git stash`; not touched.

---

## 5. Files

### New (17)

```
SAFETY.md                                     the claim table + deviation register
ISO_IMPLEMENTATION_REPORT.md                  this file
franka_experiments/utils/iso_ssm.py           the four Annex L / Annex M closed forms
franka_experiments/nodes/iso_safety_monitor.py  the independent SSM channel
scripts/iso_constants_measure.py              T_r / a_s / d / Z_d
scripts/iso_pfl_speed.py                      v_PFL
scripts/iso_preflight_check.py                refuses an ISO launch that cannot deliver
config/safety/watchman_profile.md             the rated envelope, to be applied by a person
test/test_iso_ssm.py
test/test_iso_dsafe_floor.py
test/test_iso_speed_rows.py
test/test_iso_monitor.py
test/test_iso_perception_failclosed.py
test/test_iso_torque_limits.py
test/test_iso_pfl_ceiling.py
test/test_iso_status_contract.py
test/test_iso_flags_off_identical.py
```

### Modified (17)

| file | change |
|---|---|
| `config/fr3_control.yaml` | the `iso_*` block; `d_safe` derivation comment; `min_confidence` rewritten (the gate is live again); `max_tau_delta` TODO updated; three new topics; `cbf_status` contract extended |
| `config/fr3_complete.yaml` | `distance.publish_contact_regime`; the contact-regime note rewritten |
| `config/launch_defaults.yaml` | five ISO defaults + the two ceiling overrides |
| `franka_experiments/utils/config.py` | 21 `CBF_PARAM_SPEC` entries; two standoff ranges widened |
| `franka_experiments/utils/iso_ssm.py` | *(new)* |
| `franka_experiments/utils/cbf_state_rows.py` | `FR3_TCP_LINK`; `spd_pts` carries `v_app`; SSM cap replaces the obstacle term; reduced-mode TCP row; two SSM diagnostics |
| `franka_experiments/utils/perception_msgs.py` | `LinkDistance.confidence` populated, contact regime flagged 0.5 |
| `franka_experiments/utils/distance_engine.py` | bounded hold (age counter, `direction = None` on expiry, `RTDDIAG hold expired`) |
| `franka_experiments/utils/logging_utils.py` | `sp= vcap= vcls= isostop=` + the `inf`-safe formatter |
| `franka_experiments/nodes/cbf_safety_filter.py` | `_iso_configure`, `_ISO_RHO_SLACK_SPEED`, `_on_iso_safety`, `_iso_stop_active`, `_qp_tick` STEP 0, `_empty_frame_fault`, `_brake_authority_fault`, extended `_publish_status` |
| `franka_experiments/nodes/iso_safety_monitor.py` | *(new)* |
| `franka_experiments/nodes/real_time_distance.py` | `in_band` / `publishable` split, `publish_contact_regime`, hold bound forwarded |
| `franka_experiments/nodes/qddot_to_torque.py` | `_limit_and_report`, effort limits, `/NS_1/torque_saturation` |
| `franka_experiments/nodes/pentagon_qddot_commander.py` | `_iso_safety_cb`, phase held at σ = 0 while latched |
| `franka_experiments/nodes/experiment_logger.py` | three subscriptions, 17 new columns |
| `launch/torque_control_stack.launch.py` | six launch args, `_speed_ceiling_overrides`, the monitor node, the preflight + `Shutdown` handler |
| `setup.py` / `package.xml` | `iso_safety_monitor` console script; `std_srvs` |
| `test/_cbf_builder_harness.py` | eleven `iso_*` attributes, all off |
