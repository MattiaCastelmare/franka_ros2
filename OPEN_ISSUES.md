# Open issues — FR3 CBF framework

Carried over from the kinematics/dynamics audit of 2026-11-03, after the
audit scratch files were deleted. Nothing here is fixed.

---

## S1. The CBF admits a ~6.5 cm barrier violation (SAFETY, highest priority)

With a static ball sitting on the pentagon path, `h` goes negative and the
arm closes to roughly a third of the intended standoff. Reproduced twice,
on different obstacle positions and different solver rates:

| run | obstacle | QP rate | min_h | closest gap | time h<0 |
|---|---|---|---|---|---|
| `pentagon_on_path` s0 | (0.4, -0.25, 0.45) | 60.8 Hz | -0.0673 m | 0.033 m | 0.813 s |
| `pentagon_ball_left`  | (0.4, +0.25, 0.45) | 102.2 Hz | -0.0656 m | 0.034 m | 0.524 s |
| `pentagon_on_path` s2 | (0.4, -0.25, 0.45) | 136.6 Hz | +0.0174 m | 0.117 m | 0 |

No contact and no reflex trip in any run, so nothing was hit — but
`d_safe` is not being enforced. The two breaches agree to within 1.7 mm
despite a 68% difference in solver rate, so this is NOT simply compute
starvation (an earlier hypothesis, now doubted). Seed 2 most likely just
never got close: its `d_obstacle_min` bottomed at 0.076 m, so the barrier
was never seriously tested.

Candidate causes, none investigated:
- HOCBF gains `k0 = 25`, `k1 = 10.5` (poles -3.65, -6.85).
- The curvature term omitted from `d_dot` — verified conservative in sign
  (>= 0 on all 44 samples tested) but possibly too slack here.
- Perception latency feeding a stale normal.
- Finding F5 below (1-25 deg normal error).
- The same stale/lost-track mechanism as S5, which is now measured: in
  S5 perception was 0.157 m OPTIMISTIC while the obstacle was within
  0.15 m. If that also holds for a static ball, S1 is a perception
  failure rather than a barrier-tuning one. Check this FIRST.

Reproduce: `ros2 launch franka_simulation_experiments gz_cbf_stack.launch.py
condition:=kf scenario:=pentagon_ball_left seed:=0 gui:=true`

## S3. `scene: rig` reports a zero-distance obstacle on every tick

With `obstacles: []` and `scene: rig`, perception still reports
`d_obstacle_min = 0.000` on 622/622 and 923/923 ticks, from the rig
structure itself. Result: 36-76 constraint rows per tick, slack up to
45.7, 71-303 reflex trips, QP starved to 20-36 Hz, EE tracking degraded to
189 mm p50. The same code on `scene: diag_bare` gives 0/7200 detections
and 180-240 Hz. The rig is not being masked.

Workaround in place: `pentagon_ball_left` is pinned to `scene: diag_bare`.

## S4. `/cbf_status` publishes 11 of its 38 declared fields

`cbf_diagnostics.STATUS_FIELDS` declares 38 names at
`STATUS_SCHEMA_VERSION = 2`, but `cbf_safety_filter._publish_status`
(`cbf_safety_filter.py:1679`) writes only the first 11. The 27 missing
include `fallback_level`, `solve_wall_ms` and every per-family `slack_*`
and `violation_*` column.

Any analysis reading those columns silently gets NaN instead of an error,
and `simexp/provenance.py` writes all 38 names into each run manifest, so
manifests claim data the bags do not contain. Fix the producer or trim the
schema.

---

## S5. Obstacle lost inside the robot self-mask -> CONTACT (SAFETY)

Scenario: `pentagon_sweep` (one ball oscillating 0.5 m laterally, period
10 s), `scene: diag`, seed 0.

Measured 2026-11-03 on a verified-clean machine (both conditions run back
to back, cleanup before and after each, controller spawners confirmed
alive). An EARLIER measurement of the `kf` case gave min_gap -0.086 m but
ran alongside 13 orphaned processes from previous runs; it is superseded
by the table below and must not be cited.

| | `kf` | `imm` |
|---|---|---|
| contact | **True** | False |
| min_h | **-0.1143 m** | +0.0147 m |
| min_gap | **-0.0143 m** | +0.1147 m |
| time h<0 | 1.802 s | 0 |
| QP rate | 51.0 Hz | 56.4 Hz |
| reflex trips | 0 | 0 |
| EE p50 | 48.5 mm | 59.9 mm |

So `kf` interpenetrates by 14 mm; `imm` keeps the full `d_safe` margin and
never leaves the safe set. The reflex monitor said PASS in both: it
watches joint limits, not geometry.

**The CBF was not blind.** 36-71 constraint rows every tick, a finite
`d_obstacle_min` on every tick, never a tick with zero rows. It was told
the wrong distance. Perception error against ground truth (`truth['gap']`,
the ball being the only obstacle):

| | `kf` whole run | `kf` ball within 0.15 m | `imm` within 0.15 m |
|---|---|---|---|
| mean error (seen - true) | -0.229 m | **+0.157 m optimistic** | -0.006 m |
| max optimism | +0.262 m | +0.262 m | +0.040 m |
| correlation(seen, true) | +0.454 | **-0.743** | +0.042 |
| ticks in the danger zone | - | 118 | 30 |

The negative correlation in the danger zone is the mechanism: under `kf`,
as the ball closes, perception reports it moving AWAY. Under `imm` the
correlation collapses to zero and the bias to -6 mm (conservative), and
the danger zone is entered on 30 ticks instead of 118 because the arm
retreats earlier.

Cause: the ball enters the robot's own self-mask during close approach and
the `kf` track dies. This is already documented in `fr3_complete.yaml`
next to `max_coast_s`, raised 0.167 -> 0.8 s for exactly this reason, with
a measured residual over 72 runs in the DIAG scene:

    tracker  coast  contacts     worst gap
    kf       0.8    1/12  (8%)   -0.094 m
    imm      0.8    1/24  (4%)   -0.053 m

This run is the documented 8% `kf` case. So S5 is a REPRODUCTION of a
known residual failure, not a new defect -- but it is unresolved, and `kf`
is what `gz_cbf_stack` runs by DEFAULT in these scenarios. One clean `imm`
run is not proof that `imm` is safe: the table above says `imm` still
fails 4% of the time, so this single pass is consistent with either
outcome.

Aggravating factor: DIAG lab clutter. Perception carried 8-11 tracks with
39 births / 28 reaps, so the tracker thrashes on furniture; that is why
the whole-run error is -0.23 m (it usually reports furniture, nearer than
the ball) and only flips optimistic when the ball is close.

Also visible here and NOT explained by the obstacle: the QP runs at
51-56 Hz in DIAG against 180-240 Hz in `diag_bare`. Same root as S3 --
clutter inflates the row count.

Next steps, in order:
1. Decide whether the self-mask may erase a CONFIRMED track at all,
   instead of relying on a coast timer to outlast the occlusion. This is
   the actual fix; `imm` only makes the failure rarer.
2. Repeat both conditions over >= 12 seeds to get a contact RATE, not a
   single outcome. One run per condition cannot separate 8% from 4%.
3. If `imm` holds up, change the default condition in `gz_cbf_stack`.

Reproduce: `ros2 launch franka_simulation_experiments gz_cbf_stack.launch.py
condition:=kf scenario:=pentagon_sweep scene:=diag seed:=0 gui:=true`
(runs: `franka_simulation_experiments/sim_runs/jac_audit_claude/sweep2_{kf,imm}`)

---

## S6. The filter reaches 12% of its QP rate in sim, and the jerk limit is not rate-compensated (SAFETY, invalidates S1/S5)

Measured 2026-11-03 across four identical-configuration runs of
`pentagon_sweep` / `scene: diag` / seed 0:

| run | achieved QP | /clock | joint_states_fast | reflex trips |
|---|---|---|---|---|
| `sweep2_kf`  | 52.4 Hz | 999.7 Hz | 998.1 Hz | 0 |
| `sweep2_imm` | 56.3 Hz | 999.8 Hz | 998.0 Hz | 0 |
| `gui_kf`     | 29.7 Hz | 999.8 Hz | 997.9 Hz | 151 |
| `prof_nogui` | 35.9 Hz | 999.7 Hz | 998.7 Hz | 193 |

The node asks for 300 Hz (`CBF filter QP=300 Hz` in every launch log) and
fires 1369 of 11435 expected ticks: **12%**. Run-to-run spread is 30-56 Hz
on an unchanged configuration, so this is a scheduling effect, not a
deterministic cost.

**The QP solver is NOT the bottleneck.** Benchmarked offline, no ROS, no
Gazebo, at the filter's real problem size (NV=7, N_SLACK=6, OSQP,
`max_iter=20000`), build + update + solve:

    rows   setup_ms   update+solve_ms    p95     max    -> Hz
      16      0.987             0.253   0.601   0.681    3951
      32      0.465             0.232   0.331   0.489    4318
      48      0.467             0.244   0.337   0.536    4102
      64      0.489             0.281   0.391   0.645    3562
      80      0.539             0.354   0.498   0.661    2822

0.25 ms at the row counts actually seen (36-71). The solve is ~1% of a
300 Hz budget and would sustain ~4000 Hz. Even a full re-factorization is
0.5 ms. `fr3_control.yaml:229` independently claims ~0.05 ms.

Other candidates ruled out:
- Clock granularity: `/clock` at 999.7 Hz, physics `max_step_size` 1 ms.
- Input starvation: the filter reads `joint_states_fast`
  (`/NS_1/franka/joint_states`) at 998 Hz, NOT the 46 Hz
  `/NS_1/joint_states`.
- rclpy sim-time timers: isolated test, 300 Hz timer, 3-thread
  MultiThreadedExecutor -> 299.9 Hz on wall clock, 254.6 Hz on sim clock.
  Real but far too small to explain 12%.
- Core exhaustion: 28 cores; the whole stack averages ~5.6.
- The GUI: an EARLIER claim that the renderer halved the rate is
  WITHDRAWN. All four runs launched the same GUI client and the same
  `ign gazebo -s -r --headless-rendering` server. The spread is variance.

Per-process CPU during `prof_nogui` (`ps` pcpu, lifetime-average, so a
steady-state UNDER-estimate): GZSERVER 197%, real_time_distance 123%,
commander 69%, qddot_to_torque 64%, depth adapter 54%, **filter 49%**.
The filter is blocked, not saturated. Exact mechanism NOT established --
and the two diagnostics that would settle it, `solve_wall_ms` and
`filter_cpu`, are both unpopulated (`filter_cpu: null` in all manifests).
See S4.

**Why this looked like a safety issue.**
`max_qddot_delta: 0.5` is documented at `fr3_control.yaml:600` as
"[rad/s^2 per tick] PER ACTUAL QP TICK: the jerk bound is
max_qddot_delta * qp_rate_hz". Nothing scaled it by the ACHIEVED rate, so
at 36 Hz the designed 150 rad/s^3 silently became 18 rad/s^3.

FIXED 2026-11-04: `cbf_safety_filter._slew_delta()` now scales the step by
the measured sim-clock tick interval, capped by `slew_dt_ratio_max`.
Unit-verified: the jerk bound holds at exactly 150 rad/s^3 from 300 Hz
down to 30 Hz, then clamps. `slew_rate_compensation: false` reproduces the
old behaviour bit-for-bit.

**The hypothesis that this caused the S1/S5 breaches is REFUTED.**
A/B over 6 seeds, `condition: kf`, `pentagon_sweep`, `scene: diag`,
compensation on vs off, run back to back:

| | contacts | worst gap | median gap | median QP |
|---|---|---|---|---|
| compensation ON  | 2/6 (33%) | -0.1056 m | +0.0887 m | 51.7 Hz |
| compensation OFF | 3/5 (60%) | -0.0852 m | -0.0176 m | 110.6 Hz |

(one OFF run produced no closed-loop metrics and is excluded)

That looks like a win, and it is NOT one: the two arms also differ 2x in
achieved QP rate, because they ran sequentially rather than interleaved.
Pooling all 11 scored runs by rate instead of by treatment:

| | n | contacts | median EE p50 |
|---|---|---|---|
| QP > 80 Hz  | 7 | **5 (71%)** | 39 mm |
| QP <= 80 Hz | 4 | **0 (0%)**  | 115 mm |

**Every contact happened at a HIGH tick rate; none at a low one** -- the
opposite of what S6 predicted. The low-rate runs track 3x worse (115 mm
vs 39 mm EE error), i.e. the arm lags so far off the commanded pentagon
path that it never reaches where the ball is. They are not safe, they are
sluggish, and `min_gap` cannot tell the difference.

**The real finding is the uncontrolled variable.** Achieved QP rate swings
42-120 Hz across seeds of an IDENTICAL configuration, and it drives both
tracking quality and contact outcome. Until a run's rate is held fixed,
no safety comparison in this scenario means anything -- including the
kf-vs-imm table in S5 (those two happened to match at 52.4 / 56.3 Hz with
48.5 / 59.9 mm tracking, so that one comparison is better controlled than
most, but it is still n=1 per arm).

Next steps, in order:
1. Report achieved QP rate and EE tracking error ALONGSIDE every safety
   number, and reject any run whose rate falls outside a declared band.
   A `min_gap` without its tick rate and tracking error is not evidence.
2. Find and fix the 42-120 Hz variance itself. `solve_wall_ms` and
   `filter_cpu` are both unpopulated (S4), which is why 88% of missing
   ticks still cannot be attributed.
3. Only then re-run S1 and S5, interleaving conditions seed by seed
   rather than arm by arm, so machine state cannot align with treatment.

The compensation fix stays in (it is correct, and it removes a silent
8x authority loss), but it must NOT be credited with a safety improvement
on this evidence.

Reproduce the benchmark: `scratchpad/qp_bench.py`. A/B runs:
`sim_runs/jac_audit_claude/ab_kf_{true,false}_s{0..5}`.

---

## Audit findings F1-F6 (reported, deliberately not fixed)

- **F1 (MEDIUM, dormant).** `cbf_OSCBF_filter.py:489-498` still applies the
  RIGID point-Jacobian formula to interpolated control points AND omits the
  `-[omega x r]x Jw` transport term — both defects corrected elsewhere in
  `aad4c86`. Dormant only because this filter is not currently launched.
  Note its line 508 DOES carry the curvature term the main filter omits.

- **F2 (MEDIUM, live in the MoveIt stack).** `avoidance_math.py:627-651`
  (and its duplicate in `franka_simulation/scripts/utils/`) is correct only
  by accident: `computeFrameJacobian(..., WORLD)` leaves `data.oMf` at
  zero, so the offset `r` happens to come out right. Given already-updated
  data the error reaches **1.199**. Live-wrong in
  `online_avoidance_controller.py`, which calls FK +
  `updateFramePlacements` first and then passes the same `data`.

- **F3 (LOW).** Segment `fr3_link5 -> fr3_link6` has **zero length at every
  configuration**, so its two control points collapse onto one. Confirmed
  independently in the trajectory replay: `fr3_link6#0` and `#1` return
  identical Jacobians. Latent label flip to `#0.1`.

- **F4 (MEDIUM).** Links 0->3 carry no control points, so they are
  unmonitored. Worst-case optimism measured at 0.27-0.60 m against a
  `d_safe` of 0.10.

- **F5 (MEDIUM).** The normal is taken from the capture time `t_cap` while
  the Jacobian is evaluated at `q_now`, giving a 1-25 degree normal error.
  A candidate cause of S1.

- **F6.** Test gaps; see S4 and the items below.

---

## Test debt

Half-wired features — the module exists, the config parameter does not:

| test | failures | missing |
|---|---|---|
| `test_cbf_static_scene.py` | 5 | param `enable_static_scene_hold` (`static_scene.py` exists) |
| `test_cbf_surface_velocity.py` | 14 | param `enable_vobs_in_hdot` (`surface_velocity.py` exists) |
| `test_self_reject_and_obstacle_floor.py` | 6 | `DistanceEngine.last_self_reject_px`, `last_floor_drop_px` |
| `test_safety_failure_policy.py` | 5 | numeric mismatch, NOT a missing attribute — investigate, it reads `STATUS_FIELDS` and may be catching S4 |

`torque_control_stack.launch.py` (uncommitted) breaks 8 golden-snapshot
tests pinned to commit `94cd7b0`; regenerate the snapshots in the same
commit that lands it.

Deleted 2026-11-03 as orphaned (tested code that exists nowhere in the
tree): `test_governed_timing.py` (`GovernedTimingLaw`),
`test_runner_startup_timeout.py` and `run_control_regression.py`
(`run_cell(startup_timeout_s=...)`), `test_static_scene_truth.py`
(`box_gaps`), `test_obstacle_laws.py` (scenarios `pentagon_sweeping`,
`pentagon_crossing_train`).

---

## Not an issue, but worth knowing

`real_time_distance` does **not** implement Flacco's depth-space distance.
It implements the Cartesian point-cloud method the 2015 paper benchmarks
against. The README calls it "Flacco depth-space method"; a TODO already
exists at `docs/paper_draft/method_notes.md:117-120`.
