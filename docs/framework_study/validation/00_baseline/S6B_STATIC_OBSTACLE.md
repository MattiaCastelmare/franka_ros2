# S6B — One static-obstacle avoidance experiment

> **Scope.** One Gazebo cell, run **exactly once**, approved by the user on 2026-11-12. No production
> code, test, configuration, launch file or scenario definition was modified. No second seed, no
> other scenario, no reference (obstacle-free) run. Baseline `ba68077`, HEAD `f708991`, branch
> `humble-mattia`.
>
> **Evidence.** `~/franka_baseline_evidence/baseline_f708991/06_static_obstacle/` (README_S6B.txt,
> SHA256SUMS, write-protected). Scripts: `scripts/s6b_run.sh`, `s6b_proc_sampler.py`,
> `s6b_analyze.py` (container, rosbag2_py), `s6b_host.py` (host, numpy). Outputs: `analysis/`.
>
> **Labels** (00_INDEX §2): **Verified** (code read / exact recorded value), **Derived** (computed
> from recorded data), **Inferred** (reasoned, not measured: a hypothesis), **Unverified** (not
> established), **Runtime (recorded)** (from this recording).

> **Note (S6B.1, 2026-11-12).** The truth holes were filled by an offline reconstruction from the
> bag's measured 1 kHz joint states with the truth node's own geometry, which matches every recorded
> truth sample exactly. Pass 3's minimum lies inside the 395 ms hole: **0.12313 m** at t = 30.041 s,
> ≥ d_safe. No episode interval remains unverified under the sphere model; the episode minimum is
> unchanged (0.12272 m). See [S6B1_CLEARANCE_AUDIT.md](S6B1_CLEARANCE_AUDIT.md). The rows and
> sections below are kept as written.

---

## 0. Answer in brief

| # | Question | Answer | Label |
|---|---|---|---|
| 1 | Pentagon completed? | **Yes.** 4.00 laps of the reference in the 30 s episode, 4.00 laps of the measured EE; final EE error 3.8 mm; no timeout, no fault. The governor slowed the reference near the ball, so not the 4.76 laps that full speed would give. | Runtime (recorded); Derived |
| 2 | Did the CBF modify the motion **because of the obstacle**? | **Yes, with the attribution limits of §5.** Near the ball the obstacle-barrier rows were binding and relaxed (`slack_obs` > 0 on 1214 of 2654 ticks, 14 of them far from the ball), retreat-demanding obstacle rows appeared (h_qp < 0, 41 CBFDIAG lines, each with a positive Cartesian change along n̂), and the QP's own bending (`dnorm`) was p50 3.95 rad/s² near versus p95 0 far. The EE left the path plane toward −x (away from the ball) by a median of 5.2 cm near the ball versus +0.2 cm far from it. Joint-limit, singularity, self-collision and link-speed rows were not relaxed and not at their bounds in the sampled lines. Slew and joint-velocity rows touched a few sampled lines. The commander's phase governor and the filter's tangential bias also shaped the motion, upstream of the QP. | Runtime (recorded); Derived; per-tick active set Unverified |
| 3 | Minimum clearance? | **12.27 cm** (true gap, sphere model, CP `fr3_link8#2`, t = 36.927 s sim) on the gridded truth samples. One pass (t ≈ 30.0 s) had a 395 ms hole in the truth grid starting at its minimum; that window is **Unverified** (an EE proxy suggests it was not closer, Inferred). | Runtime (recorded); hole Unverified |
| 4 | 10 cm maintained? | **Met in the observed data**: the time below d_safe was 0 on the 1 kHz truth samples, and the four passes were 12.27–12.32 cm. **Not fully verified**: 6 truth-grid holes (91–562 ms), one at a pass minimum. Perceived distances were conservative by 2.3 cm at the median and **optimistic by up to 7.8 mm** at moments. | Runtime (recorded); holes Unverified |
| 5 | Constraints violated? | The **soft** obstacle rows were violated without slack on 1173 ticks and relaxed by slack (max 1.31). The residual after slack was ≤ 3.3e-3, i.e. solver tolerance. The retreat cap was relaxed on 1024 ticks (max 1.66; overruled by design). No true-gap violation, no contact, no reflex trip, no joint-limit trip. There were 82 torque-rate clips (reflex counter, PASS). | Runtime (recorded) |
| 6 | QP frequency | **88.4 ticks per sim s** (2654 / 3001; 11.6 % missed; `RATE_FAIL`), **54.7 per wall s** (episode RTF 0.618 from the wall bag). Worst sim gap 64 ms, below `accel_timeout` 100 ms. Eight gaps ≥ 50 ms. | Runtime (recorded); Derived |
| 7 | Did perception latency affect the result? | **No evidence that it reduced safety here.** The capture age at the filter was p50 108 / p95 132 / max 198 ms of sim time (above `latency_t_blind` 85 ms, F-8-9), and the barrier still held 12.3 cm. The perceived gap was conservative on net. Its influence on the trajectory (later reactions, more slack) is **not separable** with one run. | Runtime (recorded); effect Unverified |
| 8 | Fallbacks or faults? | **None**: fallback level 0 (full QP) on all 2654 ticks, 0 `fault_braking`, 0 slew conflicts, 0 livelock escapes. The reflex monitor passed with 0 trips. The controller was active at START. | Runtime (recorded) |
| 9 | Most likely cause of the QP rate problem | **Measured:** late ticks follow a fresh constraint rebuild 33 % of the time, against 0.7 % otherwise. The filter **process** used 105 % of one core (1 s windows p5–p95 101–110 %), split across 4 threads of 24–28 % each, while the host was 45 % busy (12.7 of 28 cores). **Hypothesis (Inferred):** the Python GIL serialises the rebuild and QP callbacks, capping the node near one core. It is not established as the cause, since per-thread GIL time was not measured. | Runtime (recorded); mechanism Inferred |
| 10 | Before accepting the baseline | Fix or tolerate the truth-grid holes (F-6-5); decide the QP-rate policy (F-8-5) and the RTF policy (F-8-10); log the per-tick active set or shaped nominal, so that the box / slew / obstacle attribution is per tick rather than per CBFDIAG line; decide whether a 20 % margin over d_safe is the intended behaviour or over-conservatism; run the obstacle-free reference (C2/C3 controls) and more seeds. See §10. | Recommendation |

---

## 1. Configuration (exact)

| Item | Value | Label |
|---|---|---|
| Scenario | `pentagon_static_baseline_bare` (the existing empty-room variant of `pentagon_static_baseline`; scene `diag_bare`) | Verified (catalog resolved: `preflight/resolved_pentagon_static_baseline_bare_full_s0.json`) |
| Condition / seed / perception | `full` / `0` / `true` (the real `depth_sim_adapter` → `real_time_distance` pipeline) | Verified |
| Episode | T = 30 s sim, motion `pentagon` (circle r = 0.25 m, centre (0.4, 0, 0.45), yz plane, 6 s per lap) | Verified |
| Obstacle | sphere r = 0.05 m, fixed at (0.55, −0.25, 0.449), placed at t = 10.551 s sim | Verified (truth.npz) |
| Condition overrides | `enable_go_around: false`, `qp_rate_hz: 100.0` (the rest from the installed `fr3_control.yaml`, install == source) | Verified (`container_tmp/tmp__launch_params_40egyj6p`; `preflight/install_sha256_check.txt` rc 0) |
| Filter startup | d_safe 0.1 m, horizon 1.2 m, k0 25.0, k1 10.5, rho 1000; zone ladder hold < 0.025, prio < 0.050, active < 0.100, notice < 0.150 m | Verified (`launch.log`) |
| Commander governor | full speed above 0.150 m, floor below 0.080 m (`governor_r_full`/`r_stop` × d_safe) | Verified (`launch.log`) |
| CPU pinning | **none** (`--filter-cpu` not used: no documented configuration requires it) | Verified |
| Plant | Gazebo only: "franka bringup SKIPPED (external_plant:=true)"; no `franka_hardware` / `ros2_control_node` | Verified (`launch.log`) |

**Command** (`command.txt`):

```
docker exec -w /tmp -e PYTHONDONTWRITEBYTECODE=1 -e ROS_LOCALHOST_ONLY=1 franka_ros2 bash -c \
 'source /ros2_ws/install/setup.bash; ros2 run franka_simulation_experiments run_gz_scenarios \
  --scenarios pentagon_static_baseline_bare --conditions full --seeds 0 --perception true \
  --batch-dir /home/user/s6b_20261112/batch'
```

Runner output: `[1/1 80s] pentagon_static_baseline_bare__full__gz__s0 ok=False min_h=+0.023 rtf=0.65
qp=88Hz!=100 RATE_FAIL wall=74s`, exit 0. **`ok=False` is caused only by the QP-rate gate**
(`batch.cell_ok` requires `qp_rate_ok` not False): there was no timeout, no leftovers, and min_h > 0
(Verified, `cell_result.json`).

**Side monitoring (no production change).**
- A host `/proc` sampler at 4 Hz on the container cgroup. Its overhead was 4.89 CPU-s over 85 s,
  ≈ 6 % of one core, Runtime (recorded).
- A second `ros2 bag record` **without** sim time of `/NS_1/cbf_status` and `/cbf/per_link_distances`,
  for wall-clock receive stamps. Its CPU was not captured by the sampler (README).

---

## 2. Robot motion

| Quantity | Value | Label |
|---|---|---|
| Laps (reference / measured EE, unwrapped angle in the yz plane) | 4.001 / 4.003 | Derived |
| Approach segment (reference off the circle) | 10.55–12.00 s (1.46 s) | Derived |
| EE tracking error, whole episode | p50 2.6 cm, p95 13.3 cm, max 17.8 cm | Derived |
| … far from the ball (true gap > 0.30 m, on the circle) | p50 0.33 cm, p95 1.6 cm, max 3.3 cm | Derived |
| … near the ball (true gap < 0.20 m) | p50 7.5 cm, p95 12.2 cm, max 16.8 cm | Derived |
| Out-of-plane deviation x_act − x_des, near / far / +y side | median −5.2 cm / +0.18 cm / +0.17 cm (min −12.3 cm near) | Derived |
| Commander phase governor σ (Δs / (ṡ Δt) from the commander CSV) | near (EE–ball centre < 0.25 m): p50 0.71, p5 0.38, min 0.20; far (> 0.40 m): 1.00 | Derived |
| Final EE error | 3.8 mm | Derived |

The logged `s_dot` stays 1/6 near the ball because the governor scales **dt**, not ṡ (Verified,
`pentagon_qddot_commander.py` L939; F-3-3). σ was therefore reconstructed from Δs.

**Pass-by-pass** (local minima of the true gap, `analysis/s6b_analysis.json` `passes`):

| Pass | t (sim s) | True min gap | Truth hole within ±0.5 s | 1st-order "nominal" CP gap † | EE desired gap ‡ | Max EE error | x deviation min |
|---|---|---|---|---|---|---|---|
| 1 | 15.601 | 0.1228 | 1 ms (91 ms hole at 14.998, 0.6 s before) | 0.0515 | 0.0500 | 15.4 cm | −11.4 cm |
| 2 | 23.025 | 0.1228 | none | 0.0515 | 0.0500 | 14.7 cm | −9.2 cm |
| 3 | 29.999 | 0.1232 | **395 ms hole starting at the minimum** | 0.0429 | 0.0500 | 17.8 cm | −12.3 cm |
| 4 | 36.927 | **0.1227** | none | 0.0475 | 0.0500 | 12.2 cm | −6.8 cm |

† Each actual CP shifted by the EE tracking error (translation only), Derived; an approximation of
what the unfiltered reference would have produced, not a counterfactual simulation.
‡ Distance from the **reference** EE point to the ball surface minus 0.05 m: the reference is designed
to pass at 5.0 cm (Derived; it matches S6A.1 §6).

---

## 3. Minimum clearance and safety classification

| Item | Value | Label |
|---|---|---|
| True minimum gap (sphere model, `truth.npz`, 27 949 samples in the episode) | **0.12272 m**, `fr3_link8#2`, t = 36.927 s | Runtime (recorded) |
| Recomputed from CP and obstacle positions and radii | identical (max error 0.0) | Derived |
| Time below d_safe / below 0 (on the samples) | 0 s / 0 s | Derived |
| Contact (harness) | false; `neg_duration` 0 | Runtime (recorded) |
| Truth-grid holes > 10 ms in the episode | 6: 91, 174, 336, 395, 484, 562 ms, at t ≈ 15, 20, 25, 30, 35, 40 s | Runtime (recorded) |
| `/sim/truth/cp_gaps` (the other truth topic) | same holes (e.g. 493 ms around 30.0 s) | Runtime (recorded) |
| Gap at the edges of the 30.0 s hole | 0.1232 → 0.1280 m | Runtime (recorded) |
| EE–ball-centre distance inside that hole (`/NS_1/ee_actual`, no hole) | ≥ 0.2230 m, versus 0.2218–0.2224 m at the other passes' minima | Derived |

*(Superseded for pass 3 by S6B.1, 2026-11-12: reconstructed minimum 0.12313 m inside the hole; requirement met. Original text kept.)*

**Classification (the user's scheme, thresholds unchanged).**
- **Passes 1, 2 and 4:** clearance ≥ 0.10 m, **requirement met in the observed data** (a complete
  1 kHz grid within ±0.5 s of each minimum).
- **Pass 3:** **Unverified.** The grid is complete up to the minimum (0.1232 m) and resumes at
  0.1280 m. The EE proxy did not come closer than at the other passes. The proxy is not a bound,
  though: elsewhere the true CP gap differs from (EE–centre − 0.10) by −7.0 to +0.3 cm, because the
  closest CP changes.
- **Geometry caveat:** the truth gap is between spheres (CP spheres, obstacle sphere). The real link
  meshes are not checked, so "no contact" is with respect to the sim's geometric model only
  (Verified, `truth.npz` keys `cp_radius`, `obstacle_radius`).

**Truth-hole pattern (new detail for F-6-5).** The holes fall every 5.0 s of sim time and grow
monotonically (91 → 562 ms) in a scene without furniture. They are therefore not a scene-load effect
(Runtime (recorded); the cause is Unverified).

---

## 4. Barrier activation evidence

### 4.1 Per tick, from `/NS_1/cbf_status` (2654 episode ticks)

| Family | Ticks with slack > 1e-6 | Max slack | Violation without slack (ticks / max) | Violation with slack (ticks / max) |
|---|---|---|---|---|
| obs (obstacle barrier) | **1214** | 1.312 | 1173 / 1.313 | 648 / 3.3e-3 |
| cap (retreat cap) | **1024** | 1.660 | 91 / 1.661 | 44 / 2.5e-3 |
| spd (link-speed cap) | 170 | 2.3e-5 | 0 | 0 |
| qlim (joint position/velocity barrier) | 32 | 0.097 | 2 / 0.095 | 0 |
| sc (self-collision) | 4 | 1.5e-6 | 0 | 0 |
| sing (singularity) | 0 | 8.5e-7 | 0 | 0 |
| ssb | 0 | 2.3e-9 | 0 | 0 |

- `n_violated` (= count of h̄ < 0, not of binding rows; F-8-11) > 0 on 297 ticks, max 6 (Runtime
  (recorded)).
- Perceived `d_obstacle_min`: min 0.0868 m (below d_safe on the **perceived** scale), p5 0.106
  (Runtime (recorded)).
- A slack on the obstacle row is direct evidence that the row was **binding** on that tick: it could
  not be met together with the other constraints at zero slack (Verified: slack enters only where the
  row is not met, chapter 08). Obstacle-row slack occurred on 14 ticks far from the ball (true gap
  > 0.25 m), against 1200 near it (Derived).
- The slack values are in the row's units (m/s² of aᵀq̈); "violation with slack" ≤ 3.3e-3 is solver
  tolerance (Derived).

### 4.2 Per pass

| Pass | Status ticks (±1.5 s) | `slack_obs` > 0 | `slack_cap` > 0 | n_act > 0 | min h_qp (CBFDIAG) | obstacle rows with h_qp < 0 | max `s_obs` / `s_cap` (CBFDIAG) |
|---|---|---|---|---|---|---|---|
| 1 | 261 | 259 | 165 | 64 | −0.95 | 10 | 0.415 / 1.231 |
| 2 | 257 | 245 | 185 | 70 | −0.31 | 4 | 0.108 / 0.562 |
| 3 | 249 | 243 | 157 | 86 | −1.77 | 7 | 1.312 / 0.908 |
| 4 | 272 | **272** | 164 | 77 | −0.27 | 11 | 0.247 / 0.000 |

All CBFDIAG lines near the passes are labelled with `fr3_link8#0/#1/#2` (the hand CPs) or, twice,
the self-collision pair `_link2/_link5`. Zone labels are `notice` and `active`; there was no `hold`
or `prio` rung (Runtime (recorded)).

### 4.3 Direction of the correction (CBFDIAG, ≈ 17 lines per sim s, 516 in the episode)

- On the 41 lines where an **obstacle** row has h_qp < 0 (retreat demanded), `cart_rad` (Cartesian
  acceleration change along n̂ at that CP) is **positive on 41 of 41**: away from the ball (Derived).
- Of the 230 lines with `dnorm` > 0.2, `|dq_rad|` > `|dq_ort|` on 178: the correction lies mostly
  along the labelled row (Derived).
- `dq_ort` reached 7.8 rad/s² (pass 1), so a large part of the correction also went into directions
  orthogonal to the labelled row (cf. the "throws itself backward" note in `format_cbf_diag`)
  (Runtime (recorded)).

---

## 5. Constraint attribution (obstacle barrier versus the rest)

**What the logs can and cannot establish.**
- `/NS_1/qddot_nom` in the bag is the **commander's** nominal.
- The QP minimises the distance to a **shaped** nominal, after the zone fade, the state governor,
  the tangential bias, go-around (off), escape, outrun and livelock (`cbf_safety_filter._qp_tick`
  L1198–1430, Verified).
- The shaped nominal is not published. CBFDIAG's `dnorm` = ‖q̈_safe − q̈_nom,shaped‖ (Verified, the
  `_report` call at L1510 passes the reassigned `qddot_nom`), but CBFDIAG is throttled to about 1
  line in 5 ticks.
- The per-row active set is not logged. Only the per-family slack and violation sums are, per tick.

Consequently the attribution is per family per tick, and per mechanism only on the sampled lines.

| Candidate | Evidence near the ball (true gap < 0.20 m; 216 CBFDIAG lines, 1106 status ticks) | Evidence far (> 0.30 m; 192 lines, 1028 ticks) | Conclusion | Label |
|---|---|---|---|---|
| **Obstacle barrier rows** | `s_obs` > 0 on 206/216 lines; obstacle h_qp < 0 on 41 lines, all with `cart_rad` > 0 | `s_obs` 0/192 lines; 14 slack ticks in total | **Active and binding near the ball** | Runtime (recorded); Derived |
| Retreat cap (G_CAP, obstacle-derived, an upper bound on separation) | `s_cap` > 0 on 20 lines; retreat ≥ 0.95 × cap on 49 | 0 / 0 | Bound and **overruled** by the barrier (by design, `format_cbf_diag` docstring). It limits how fast the arm retreats; it does not cause the deviation | Runtime (recorded) |
| Link-speed cap (G_SPD, distance-dependent) | 81 lines with cap < `link_speed_max` 1.3 (geometric term on), **0** with the CP speed ≥ 0.95 × cap; slack ≤ 2.3e-5 | 44 / 0 | Emitted; **not at its bound** in the sampled lines | Runtime (recorded); per-tick Unverified |
| Joint acceleration box | the commander's nominal never exceeded the box (0 ticks) | — | The box does not clip the **commander's** nominal. Whether it bound the QP solution is not logged | Derived; Unverified |
| Slew (per-tick acceleration change) | `slew` non-empty on 12/216 lines | 2/192 | Touched occasionally | Runtime (recorded) |
| Joint position / velocity limits | `vbite` (joint 4 velocity at its braking curve) on 11/216 lines; `s_qlim` > 0 on 0 lines; qlim slack on 32 ticks (max 0.097) | 0 | Minor, near the ball only (joint 4 at about 50 % of its envelope) | Runtime (recorded) |
| Self-collision / singularity rows | slack ≤ 1.5e-6; labels `sc:_link5/3_hand` and `sing:fr3_link8` dominate the **far** lines | — | Not binding | Runtime (recorded) |
| Zone fade + state governor (pre-QP) | `task` and `gov` < 1 on 19 lines (min 0.30; binding term `vel`) | 0 | Upstream reshaping of the nominal near the ball | Runtime (recorded) |
| Tangential bias (pre-QP) | applied p50 1.64 rad/s² (raw p95 7.2, capped at ≈ 1.97) | p50 0.19, max 0.55 (from sc / sing rows) | Large near the ball; present far | Runtime (recorded) |
| Commander phase governor (pre-filter) | σ p50 0.71, min 0.20 | 1.00 | Slows the reference near the ball | Derived |

**Bag-level difference q̈_safe − q̈_nom (commander).** ‖Δ‖ > 0.2 rad/s² on 2132 of 2654 ticks
(p50 0.64, max 8.38). That is true **also far from the ball**, on 727 of 1246 ticks. Far from it the
QP itself does not bend the shaped nominal (`dnorm` p95 0). The far difference therefore comes from
the pre-QP shaping (the tangential bias from the sc / sing rows) and from pairing a 100 Hz nominal
with 88 Hz safe ticks (Derived; the split between the two is Unverified). The S6A.1 box + slew
residual test is **not discriminating** here (701 of 1246 far ticks > 0.2), because the shaping
terms are not in the bag. **The bag difference alone is not evidence of barrier activity**. The
evidence is the slack, h_qp, `cart_rad` and `dnorm` near/far contrast above.

**Effect on the path (Derived).**
- The reference passes 5.0 cm from the ball (EE point).
- Shifting the actual CPs by the tracking error gives a 1st-order "nominal" CP gap of 4.3–5.2 cm.
- The realised gap was 12.3 cm on every pass, i.e. +7.1 to +8.0 cm.
- The arm was displaced toward −x (away from the ball, which sits at x = 0.55 against the plane at
  x = 0.40) by a median of 5.2 cm near the ball, against ≈ 0 far and on the +y side (the in-run
  control).
- Several mechanisms contributed: the governor (timing only, not the path), the zone and state
  fades, the tangential bias, and the QP rows. **They cannot be separated into individual
  centimetres from these logs** (Unverified).

**S6A.1 §6.4 criteria (proposals; evaluated, not used as gates).**
- V1 pass.
- V2: 3 of 4 passes pass; pass 3 fails (395 ms hole).
- V3: episode RTF 0.618, not below 0.5.
- V4: `RATE_FAIL`, max gap 64 ms < 100 ms.
- V5: no fallbacks, faults or trips.
- B1 pass on all passes.
- B2: ‖Δ‖ > 0.2 on ≥ 5 consecutive ticks on all passes, but the residual test is not discriminating
  (above).
- B3: reported.
- C1: 12.3 ≥ 7.0 cm, pass.
- C2: −5.2 cm near versus +0.17 cm at +y, pass.
- C3: far / +y-side error p95 1.6–1.9 cm. The recovery time was not computed per pass (Unverified).
- S1: pass on the samples; pass 3 is Unverified.
- S2: 0 s below d_safe.
- S3: PASS, 82 rate clips.

---

## 6. QP and perception performance

### 6.1 QP loop

| Metric | Value | Label |
|---|---|---|
| Ticks / expected (episode) | 2654 / 3001 → **88.4 per sim s**, 11.6 % missed | Runtime (recorded) |
| Per wall s (wall bag pairing, 2602 messages paired) | **54.7**; episode wall 48.56 s, RTF **0.618** | Derived |
| Tick gap, sim | p50 10, p95 24, p99 36, **max 64 ms**; 8 ≥ 50 ms; 0 ≥ 100 ms | Runtime (recorded) |
| Tick gap, wall | p50 17.7, p95 33.3, max 107 ms | Derived |
| Near / far from the ball (rate from gaps) | 86.0 / 91.4 per sim s | Derived |
| Solve wall time | p50 2.95, p95 8.99, max 21.98 ms (near p50 3.04, far 2.80) | Runtime (recorded) |
| Rows | p50 25, p95 41, max 56 | Runtime (recorded) |
| Constraint rebuilds | 1327 = **44.2 per sim s** of 50 | Derived (change of `constraint_build_wall_ms`) |
| Build wall time | p50 16.8, p95 25.5, max 34.5 ms | Runtime (recorded) |
| Late tick (> 15 ms) after a fresh rebuild vs otherwise | **33.4 % vs 0.68 %** | Derived |
| 1 s windows (sim) | ticks 66–96 per sim s; RTF 0.59–0.73 | Derived |
| Fallback / fault / slew conflict / livelock | 0 / 0 / 0 / 0 | Runtime (recorded) |
| Ages at the tick (sim ms) | joint receipt p50 0 / p95 4; distance **capture** p50 108 / p95 132 / max 198; distance receipt p50 13 / p95 33 / max 110; constraint snapshot p50 20 / p95 33 / max 42 | Runtime (recorded) |

### 6.2 External CPU (host sampler, episode window, 194 samples)

| Process | % of one core | Label |
|---|---|---|
| `ign gazebo` (server + rendering) | 230 | Runtime (recorded) |
| `real_time_distance` | 138 (main processing thread 63 %, main 46 %, six workers ≈ 4 % each) | Runtime (recorded) |
| **`cbf_safety_filter`** | **106**: four threads at 28 / 24 / 24 / 24 %; 1 s windows p5–p95 101–110 % | Runtime (recorded) |
| `ground_truth` | 95 | Runtime (recorded) |
| experiment_logger / commander / reflex / qddot_to_torque / joint_state_publisher / depth adapter / supervisor / harness bag / overlay | 72 / 72 / 67 / 65 / 60 / 57 / 56 / 52 / 49 | Runtime (recorded) |
| Host total | 12.7 of 28 core-equivalents busy; per core 32–61 %. Frequencies: P-cores p50 2.07 GHz (p95 2.48), E-cores p50 1.90 GHz; governor `powersave` | Runtime (recorded) |

The filter's threads migrated across all 28 CPUs, E-cores included (last-CPU histogram), Runtime
(recorded).

### 6.3 Perception

| Metric | Value | Label |
|---|---|---|
| Distance messages | 36.5 per sim s; 22.4 per wall s; 1 per distinct capture (1095) | Runtime (recorded); Derived |
| Capture intervals | on the 90 Hz sim grid: 1, 2, 3, 4, 5, 6+ periods = 295, 292, 278, 158, 58, 13 | Derived |
| Bag receive − capture (sim) | p50 66, p95 88, max 113 ms | Derived |
| Node timing lines | `all` 39–55 ms wall, `wait` 0.0 ms (compute-bound); the node logged a 18.2 → 45.5 frames/s rescale at START | Runtime (recorded) |
| Perceived − true min gap (same sim time) | median −2.26 cm (conservative), p5 −5.0, min −8.0, **max +0.78 cm** (optimistic) | Derived |

---

## 7. Safety violations

- **None** on the recorded truth samples: no penetration and no time below d_safe. The four
  pass minima were 12.27–12.32 cm.
- **Not verified:** the 395 ms truth hole at pass 3; mesh-level contact (sphere model only).
- **Soft-constraint relaxation:** the obstacle barrier was relaxed by slack on 46 % of the ticks. The
  safety margin observed is the joint result of the relaxed barrier, the tightening terms
  (`hunc` ≤ 3.1 cm, `hlat` ≤ 2.8 cm near the ball) and the conservative perception. One run cannot
  say how much margin each contributes (Inferred).
- **Reflex:** PASS; worst velocity ratio 0.54, torque ratio 0.33, position margin 0.15 rad; 82
  torque-rate clips (worst rate 2873 N·m/s) (Runtime (recorded)).

---

## 8. Limitations

1. **One seed, one phase** (F-6-12: the pentagon phase depends on boot timing). Nothing statistical
   is claimed.
2. **No obstacle-free reference run.** The "nominal" clearance is a first-order reconstruction (§2 †),
   and the +y side of the same run serves as the in-run control.
3. **Attribution granularity:**
   - the shaped nominal and the per-row active set are not recorded per tick;
   - CBFDIAG samples about 1 tick in 5;
   - a box-bound or slew-bound **QP solution** cannot be detected per tick from the bag.
4. **Truth grid holes** (F-6-5) and the sphere-only geometry.
5. **Sim time versus wall time:** all rates and ages are per sim s at RTF 0.62. Wall-bound components
   (perception, the filter) would look worse at RTF 1 (F-8-10, Inferred).
6. **Monitoring:**
   - the wall-time recorder's own CPU was not captured (sampler race, README);
   - the sampler cost ≈ 6 % of one core on the host;
   - both add load that a normal cell does not have.
7. The GIL hypothesis is consistent with the measurements (process ≈ 1 core, threads sharing it) but
   was not tested (no py-spy, no per-thread GIL wait).

---

## 9. Reproducibility

- Re-run: the command of §1 from a clean container directory (`s6b_run.sh` refuses to overwrite).
  Same HEAD, install == source (`preflight/`), no pinning.
- Re-analysis:
  - container: `python3 -B s6b_analyze.py <cell> <wallbag> <out>`;
  - host: `python3 -I s6b_host.py <evidence_dir> <out.json>`.
- Integrity:
  - git HEAD, status and stash are identical before and after;
  - 0 repository files are newer than the launch marker outside `docs/framework_study`;
  - `docker diff` delta: 97 entries, all in the run's scratch, launch temp files and logs
    (`postflight/`).
- Container scratch left in place: `/home/user/s6b_20261112/`, `/home/user/s6b_20261112_analysis/`,
  `/home/user/s6b_analyze.py`, `/tmp/s6b_marker`, `/tmp/launch_params_*`.
- Processes: after the run the container ran only its PID 1 shell; no gz / ros2 process on the host.

---

## 10. What to investigate or correct before the baseline can be accepted (proposals; nothing done)

1. **Truth grid (F-6-5):** periodic (every 5 s) and growing holes in `ground_truth`. Until fixed,
   "no contact" holds only on the samples.
2. **QP-rate policy (F-8-5):** 88.4 per sim s. Decide whether the gate is 100 Hz in sim time, and
   test the GIL hypothesis (e.g. py-spy on the filter, or a run that moves the rebuild out of
   process): measurement first, no code change implied.
3. **Per-tick attribution:** publish or log the shaped nominal and the binding-row identity (a
   diagnostics change, so a production change needing approval). Without it, "barrier versus box /
   slew / shaping" stays per family.
4. **Margin semantics:** 12.3 cm against d_safe 10 cm with the reference at 5 cm. Decide whether
   the extra 2.3 cm (≈ the perception bias) is intended. Decide also whether the frequent slack on
   the obstacle rows is acceptable for a "hard" safety claim.
5. **Perception:** capture age p95 132 ms against `latency_t_blind` 85 ms (F-8-9). The
   occasionally optimistic perceived gap (+7.8 mm).
6. **Reference and seeds:** the obstacle-free `pentagon_clear` cell for C2/C3, and more seeds or
   phases (F-6-12).
7. **RTF and host:** the governor is `powersave`; episode RTF 0.62; decide the host policy
   (F-8-10).
