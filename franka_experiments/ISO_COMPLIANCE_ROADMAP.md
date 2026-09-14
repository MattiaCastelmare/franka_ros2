# ISO Safety Alignment — Implementation Roadmap for `franka_experiments`

Target: acceleration-space torque stack
(`pentagon_qddot_commander` → `cbf_safety_filter` → `qddot_to_torque` → `rt_torque_controller`),
with `real_time_distance` as the perception source.

## Normative basis

- **ISO 10218-1:2025**, *Robotics — Safety requirements — Part 1: Industrial robots*
  (supersedes ISO 10218-1:2011). Relevant: robot class (5.1.17), safety functions (5.3, 5.5),
  speed limit monitoring (5.5.3), start/restart interlock and reset (5.5.2), monitored standstill
  (5.5.5), stopping time / distance limiting (5.5.6, 5.5.7), **normative Annex H** (stopping time
  and distance metrics), **normative Annex C** (functional safety performance).
- **ISO 10218-2:2025**, *… Part 2: Industrial robot applications and robot cells*
  (supersedes ISO 10218-2:2011 and absorbs most of ISO/TS 15066:2016). Relevant: collaborative
  applications (5.14), **SSM (5.14.5) with the separation-distance formulas in normative
  Annex L**, **PFL (5.14.6) with biomechanical limits in informative Annex M and the measurement
  methodology in informative Annex N**, speed monitoring as a safety function (5.5.6:
  *reduced speed* ≤ 250 mm/s vs *monitored speed*), biomechanical-limit verification (6.3.3).
- **ISO 13855:2024** — approach speeds and intrusion distance `C`.
- **ISO 13849-1:2023** — PL / Category of safety-related control system parts.
- **IEC/TS 61496-4-3** — vision-based protective devices.
- ISO/TS 15066:2016 is the historical source of the SSM and PFL numbers; it is superseded by the
  above and must not be cited as the governing document for new work.

Terminology follows the 2025 editions: **collaborative application** / **collaborative task**
(the terms *collaborative robot* and *collaborative operation* are deprecated), **protected
space** (formerly *safeguarded space*), **separation distance**, **protective stop**,
**monitored standstill**, **quasi-static contact** / **transient contact**.

## Normative classification used throughout

Every requirement, formula and number below carries one tag. Do not silently promote a tag.

- **[R]** — explicitly required by the standard.
- **[S]** — value or formula provided / recommended by the standard.
- **[E]** — engineering assumption or design choice made here; not traceable to a standard.

## Ground rules

- Do **not** rename existing symbols, parameters, or topics. Add, do not refactor.
- Every new parameter goes in `franka_experiments/config/fr3_control.yaml` under `params:`
  **and** in `CBF_PARAM_SPEC` in `franka_experiments/franka_experiments/utils/config.py`
  (a key missing from either side is a startup failure — that is the intended behaviour).
- Every new behaviour is behind an `iso_*` flag, **default `false`**, until validated on hardware.
  With all flags off, numerical output must be bit-identical to today.
- Every step adds `pytest` tests under `franka_experiments/test/` and must pass
  `colcon test --packages-select franka_experiments` before the next step starts.
- Steps 1–3 and 10–12 touch no safety path. Steps 4–9 do: run them on fake hardware first
  (`torque_control_stack.launch.py use_fake_hardware:=true`), then supervised on the FR3.
- `franka_sim/envs/cbf_filter.py` is a deliberate duplicate of the filter math and is **out of
  scope**. If Step 5 changes row math, note the divergence in `franka_sim/config.yaml` comments;
  do not re-sync in this roadmap.
- Claims must be labelled **implemented** / **validated** / **certified**. Nothing here produces
  a certified safety function: the CBF chain is single-channel Python over best-effort DDS, and
  the FR3 itself is certified to EN ISO 10218-1:**2011**, not to the 2025 editions.

---

## Step 1 — Measure the ISO input constants

**Objective.** Replace guessed constants with measured ones. Every later step consumes these
numbers; guessing them makes the whole chain decorative.

**Files**
- new: `franka_experiments/scripts/iso_constants_measure.py`
- read-only use: `franka_experiments/scripts/latency_budget.py`

**Changes**
1. New script, four sub-commands, no publishing to the robot except in `stop`:
   - `reaction` **[E]** — wrap `latency_budget.py` outputs (depth→CBF→torque chain) and report
     `T_r = p95(total blind time) + 1/qp_rate_hz + 0.001` s.
   - `stop` **[S] procedure, [E] values** — with the arm in free space, drive joint-space motions
     at **33 %, 66 % and 100 %** of `velocity_box_margin * qdot_max`, at **33/66/100 % of rated
     payload and of arm extension**, then command the stop profile (`q̈ = −q̇ / 0.05` clipped to
     `±decel_max`); record `/NS_1/joint_states` and `/NS_1/qddot_safe`; report per run: realized
     Cartesian TCP speed at trip, stopping time `T_s`, stopping distance `S_s`, and realized
     Cartesian deceleration `a_s = v_tcp / T_s`. Report the **minimum** `a_s` over all runs and
     directions. The 33/66/100 % grid and the inclusion of a **Category 2 stop** follow ISO
     10218-1:2025 **Annex H**; the FR3 manual's stopping data is measured to the same annex
     (numbered Annex B in the 2011 edition) — cross-check against it and keep the worse value.
   - `detection` **[R] input for `C`** — determine the **detection capability `d`** of the depth
     pipeline: the smallest object reliably detected, at the far end of the working range, over
     the whole field of view, across the worst-case surface and reflectivity in the cell. Report
     `d` in mm with the test conditions. This is the input ISO 13855:2024 requires for `C`;
     without it, no ISO-conformant `C` exists (see Step 2).
   - `uncertainty` **[E]** — from a static calibration target: report the depth/TF residual p95
     → `Z_d`.
2. Output a ready-to-paste YAML block with `iso_t_reaction`, `iso_a_stop`, `iso_v_human`,
   `iso_c_intrusion`, `iso_z_depth`, `iso_z_robot`, each annotated with its measurement date and
   its **[R]/[S]/[E]** tag.
3. `iso_z_robot` **[E]**: take the FR3 "worst case safe Cartesian position accuracy for stopping
   functions" from the Franka Research 3 product manual; record the source in the YAML comment.

**Rationale.** Hardware diagnostics show commanded deceleration far above realized
(`qdd_cmd_rad` vs `qdd_real_rad` in CBFDIAG). `a_s` must be the *realized* value or `S_p` is
optimistic by exactly that factor.

---

## Step 2 — ISO parameter block

**Objective.** One declared, validated source of truth for the ISO constants.

**Files**
- `franka_experiments/config/fr3_control.yaml`
- `franka_experiments/franka_experiments/utils/config.py`

**Changes**
1. In `fr3_control.yaml`, new section inside `params:`. Every value carries its tag; the
   placeholders must be overwritten from Step 1:

```yaml
  # ── ISO 10218-1/-2:2025 layer ─────────────────────────────────────────────
  # Tags: [R] required by the standard, [S] value/formula from the standard,
  #       [E] engineering assumption made here.
  iso_enabled: false            # [E] master flag for steps 4-9
  iso_mode: automatic           # [E] automatic | reduced (see iso_tcp_reduced_speed)
  iso_t_reaction: 0.10          # [E] T_r, measured (Step 1 `reaction`)
  iso_a_stop: 1.0               # [E] a_s, MEASURED realized Cartesian decel (Step 1 `stop`)

  # [S] ISO 13855:2024 approach speeds: 2.0 m/s for a hand/arm reach toward the
  # hazard, 1.6 m/s for a whole-body walking approach. A benchtop FR3 is reached
  # INTO, so 2.0 is the applicable value here. Lowering it requires a validated
  # measurement of the operator's directed speed, which this cell does not have.
  iso_v_human: 2.0              # [m/s]

  # [R] C is the intrusion distance defined by ISO 13855:2024 and is NOT a free
  # parameter: with detection capability d (Step 1 `detection`),
  #     d <= 40 mm  ->  C = 8*(d - 14) mm
  #     d >  40 mm  ->  C = 850 mm   (body detection only)
  # The depth pipeline has no demonstrated detection capability and is not a
  # rated protective device (IEC/TS 61496-4-3), so 0.85 m is the conformant
  # value until `detection` says otherwise. STATE THIS HONESTLY: at C = 0.85 m
  # the protective separation distance exceeds this arm's reach, i.e.
  # ISO-conformant SSM is not achievable in this cell. Any smaller value is a
  # documented research deviation and must be recorded in SAFETY.md (Step 12).
  iso_c_intrusion: 0.85         # [m]

  iso_z_depth: 0.06             # [E] Z_d, depth+calibration position uncertainty
  iso_z_robot: 0.01             # [E] Z_r, robot position accuracy (FR3 manual)

  # [E] derived from [S] inputs; see Step 9. Recompute per body region and per
  # payload; PFL compliance requires MEASUREMENT (ISO 10218-2:2025 6.3.3 and
  # Annex N), never this calculation alone.
  iso_v_pfl: 0.68               # [m/s]

  # [S] ISO 10218-1:2025 (5.5.3) / -2:2025 (5.5.6) reduced speed = max 250 mm/s,
  # required for manual modes. Using it as a cell-wide derate is [E].
  iso_tcp_reduced_speed: 0.25   # [m/s]

  iso_ssm_speed_rows: false     # [E] Step 5
  iso_monitor_enabled: false    # [E] Step 6
  iso_speed_tol: 0.05           # [E] monitor tolerance above the cap
  iso_monitor_ticks: 3          # [E] consecutive violating ticks before a trip
  iso_stop_tau: 0.05            # [E] braking time constant of the stop profile
  # [E] SSM PERMITS motion to resume automatically once S >= S_p; latching is a
  # local choice for supervised experiments, consistent with the start/restart
  # interlock and reset function (ISO 10218-1:2025, 5.5.2) but not required.
  iso_stop_requires_reset: true
  iso_distance_hold_max_s: 0.10 # [E] Step 7
  iso_empty_frame_max_s: 0.20   # [E] Step 7
  iso_brake_frac_min: 0.5       # [E] Step 8
  iso_brake_frac_ticks: 10      # [E] Step 8
  iso_tau_rate_max: 40.0        # [E] N*m per 100 Hz tick, Step 8
```

2. In `config.py`, add matching `CBF_PARAM_SPEC` entries:
   - floats with `positive=True` and a `maximum` for all distances/speeds/times
     (`iso_a_stop` max 20.0, `iso_v_human` max 5.0, distances max 2.0, times max 5.0),
   - `iso_mode`: `('str', dict(choices=('automatic', 'reduced')))`,
   - bools with `dict()`,
   - `iso_monitor_ticks`, `iso_brake_frac_ticks`: `('int', dict(positive=True, maximum=1000))`.
3. Add the corresponding launch arguments in
   `franka_experiments/launch/torque_control_stack.launch.py` (`_ALL_PARAMS` + the
   `parameters=[{...}]` block of `cbf_node`) for `iso_enabled`, `iso_mode`,
   `iso_ssm_speed_rows`, `iso_monitor_enabled`, and defaults in
   `franka_experiments/config/launch_defaults.yaml` (all `false` / `automatic`).

---

## Step 3 — `utils/iso_ssm.py` (pure math, no ROS)

**Objective.** One tested implementation of the SSM equations, importable by the filter, the
monitor, and the offline scripts.

**Normative status of the math.** ISO 10218-2:2025 **Annex L** (normative) gives `S_p` as the sum
of three integrals plus `C + Z_d + Z_r` **[S]**. The closed forms below assume constant `v_h`,
constant `v_r` during `T_r`, and constant deceleration `a_s` during `T_s = v_r/a_s` **[E]**; the
inversion in `ssm_speed_cap` is exact with respect to those closed forms, **not** with respect to
the standard's integrals **[E]**. `S_s` computed as `v_r²/(2·a_s)` must be checked against the
measured stopping distances from Step 1, and is conservative only if `a_s` is the **minimum**
measured deceleration.

**Files**
- new: `franka_experiments/franka_experiments/utils/iso_ssm.py`
- new: `franka_experiments/test/test_iso_ssm.py`

**Changes** — implement exactly these four functions (numpy only, no classes, no state):

```python
def stopping_distance(v_r, *, t_r, a_s) -> float:
    """[m] S_r + S_s = v_r*t_r + v_r^2/(2*a_s).  v_r clamped at >= 0.  [E] model."""

def protective_separation(v_r, v_h, *, t_r, a_s, c, z_d, z_r) -> float:
    """[m] S_p = S_h + S_r + S_s + C + Z_d + Z_r   ([S] structure, [E] closed form)
       with T_s = v_r/a_s:
       S_h = v_h*(t_r + v_r/a_s);  S_r = v_r*t_r;  S_s = v_r^2/(2*a_s).
       v_r, v_h clamped at >= 0 (a receding part never shrinks S_p)."""

def ssm_speed_cap(d, v_h, *, t_r, a_s, c, z_d, z_r, v_max) -> float:
    """[m/s] largest robot speed TOWARD the obstacle satisfying d >= S_p.  [E]
       Exact inversion of protective_separation in v_r:
           b    = a_s*t_r + v_h
           disc = b*b + 2*a_s*(d - c - z_d - z_r - v_h*t_r)
           v    = sqrt(disc) - b            (0.0 when disc <= 0)
       Returns min(v, v_max), floored at 0.0."""

def pfl_speed(f_max, k_body, m_robot, m_human) -> float:
    """[m/s] v_PFL = f_max / sqrt(mu*k_body),  mu = 1/(1/m_human + 1/m_robot).
       m_robot = M/2 + payload.  [S] formula and simplification (ISO 10218-2:2025
       Annex M; formerly ISO/TS 15066 Annex A)."""
```

**Tests** (`test_iso_ssm.py`) must pin:
- `ssm_speed_cap` is the exact inverse of `protective_separation`: for random
  `(v_r, v_h, t_r, a_s)`, `protective_separation(ssm_speed_cap(d, ...), v_h, ...) ≈ d`.
- `d <= c + z_d + z_r` ⇒ cap is exactly `0.0` (stop region).
- Monotonicity: cap decreases in `v_h`, increases in `d` and in `a_s`.
- No NaN for `d = 0`, `v_h = 0`, `a_s` small, `d` huge.
- `pfl_speed` reproduces the worked example in the module docstring (Step 9).

---

## Step 4 — Barrier offset = irreducible part of `S_p`

**Objective.** Make `d_safe` mean something ISO-defined instead of a tuned constant, without
double-counting the dynamic terms.

**Split of the `S_p` terms across existing mechanisms — [E], a design choice.** The requirement
`S ≥ S_p` is **[R]**; applying it per control point rather than to "any hazardous moving part" as
a whole is a conservative interpretation **[E]**.

| Term | Enforced by |
|---|---|
| `C + Z_d + Z_r` | `d_safe` (this step) |
| `S_h = v_h·(T_r + T_s)` | existing `enable_velocity_standoff` (`velocity_standoff()`) |
| `S_r + S_s` | the SSM speed cap (Step 5) |
| sensing latency | existing `enable_latency_compensation` |

**Files**
- `franka_experiments/franka_experiments/nodes/cbf_safety_filter.py` (`__init__`, after
  `load_cbf_config`)
- `franka_experiments/config/fr3_control.yaml`
- new test: `franka_experiments/test/test_iso_dsafe_floor.py`

**Changes**
1. In `__init__`, when `P.iso_enabled`: compute
   `d_floor = P.iso_c_intrusion + P.iso_z_depth + P.iso_z_robot`; if `P.d_safe < d_floor`,
   **raise `ValueError`** naming both values and all three contributions (same failure style as
   `declare_from_spec`). With the conformant `iso_c_intrusion = 0.85` this assertion **will**
   fail on this cell — that is the correct outcome, not a bug. The only three admissible
   resolutions, in order: (a) reduce `C` by demonstrating a detection capability `d ≤ 40 mm`
   (Step 1 `detection`) and setting `C = 8·(d−14)` mm; (b) raise `d_safe` to `d_floor`;
   (c) record a research deviation in `SAFETY.md` (Step 12) and run with `iso_enabled: false`,
   making no ISO claim. **Do not add a bypass flag.**
2. Set `d_safe` in `fr3_control.yaml` to at least `d_floor` and add a comment deriving it, naming
   which of (a)/(b)/(c) applies.
3. Set `velocity_standoff_time_s = iso_t_reaction + iso_v_human/iso_a_stop` rounded up, and keep
   `velocity_standoff: true` in `launch_defaults.yaml` — this is `S_h` (**[S]** structure,
   **[E]** parameterisation).

**Note.** The zone ladder boundaries are multiples of `d_safe`, so the whole ladder follows
automatically; no ladder parameter changes.

---

## Step 5 — SSM cap drives the task-space speed rows

**Objective.** Replace the heuristic control-point speed cap with the SSM bound, so that every
control point can stop before reaching the obstacle. Requirement **[R]**, mechanism **[E]**.

**Files**
- `franka_experiments/franka_experiments/utils/cbf_state_rows.py`
  (the "Task-space speed rows" block inside `ConstraintBuilder.build`, and the
  `link_speed_cap` / `obstacle_link_speed_cap` neighbourhood)
- new test: `franka_experiments/test/test_iso_speed_rows.py`

**Changes**
1. `from franka_experiments.utils.iso_ssm import ssm_speed_cap` at module top.
2. In the `for Jp_i, clear_i, lbl_i in spd_pts:` loop, when `self._P.iso_ssm_speed_rows` is true,
   replace the obstacle term only:

```python
v_obst = ssm_speed_cap(
    clear_i, v_app_i,                      # v_app_i = the same conditioned closing
    t_r=P.iso_t_reaction, a_s=P.iso_a_stop, # speed this CP's obstacle row uses
    c=P.iso_c_intrusion, z_d=P.iso_z_depth, z_r=P.iso_z_robot,
    v_max=P.link_speed_max)
```
   keeping `min(v_obst, link_speed_cap(d_sc_min, v_max=..., reaction_s=...))` for the
   self-collision term unchanged (self-collision is not an ISO term — **[E]**). When the flag is
   false, the existing `obstacle_link_speed_cap` path runs untouched.
3. `v_app_i` must be the per-control-point conditioned closing speed already computed for the
   obstacle rows (`con.v_obs` source); if it is not in scope at that point in `build`, pass it
   through the existing `spd_pts` tuple — do not recompute it.
4. **[E]** Raise `rho_slack_link_speed` from `500.0` to `2000.0` (above `rho_slack = 1000.0`)
   when `iso_ssm_speed_rows` is on, so the barrier can no longer buy its way past the SSM cap
   with slack. Do not make the row hard: a hard row plus the state box can be infeasible when the
   arm is already over the cap; Step 6 is what makes the bound enforced rather than preferred.
5. Keep `link_speed_max` as the flat ceiling above the SSM cap (Step 9 lowers it).

**Tests**: at `v_h = 0` and large `d` the cap equals `link_speed_max`; at `d = C+Z_d+Z_r` the row
demands deceleration (RHS through `retreat_cap_rhs` is negative); the cap decreases monotonically
as `v_app` grows; flag off ⇒ rows are bit-identical to the current output.

---

## Step 6 — Independent SSM monitor and non-safety-rated stop

**Objective.** Below `S_p` the robot system must stop **[R]**; a speed reduction alone is not
enough. The QP shapes; a separate channel enforces.

**Terminology.** What is implemented here is a **non-safety-rated stop**, not a *protective stop*
in the sense of ISO 10218 — that term is reserved for a rated function (PL d / SIL 2 on this
robot class). Do not use it in logs, topic names, or documentation.

**Files**
- new: `franka_experiments/franka_experiments/nodes/iso_safety_monitor.py`
- `franka_experiments/setup.py` (console_scripts entry
  `iso_safety_monitor = franka_experiments.nodes.iso_safety_monitor:main`)
- `franka_experiments/franka_experiments/nodes/cbf_safety_filter.py`
- `franka_experiments/franka_experiments/nodes/pentagon_qddot_commander.py`
- `franka_experiments/launch/torque_control_stack.launch.py`
- new test: `franka_experiments/test/test_iso_monitor.py`

**Changes**
1. **Monitor node** — subscribes `/NS_1/joint_states` (RELIABLE, depth 1),
   `/cbf/per_link_distances` (BEST_EFFORT, depth 1), `/NS_1/cbf_status` (depth 1).
   At `qp_rate_hz`, for every valid `LinkDistance` entry: compute the control-point speed
   `‖J_p q̇‖` with its own `CBFKinematics` instance (`utils/kinematics.py`, same URDF builder
   `build_urdf_no_hand`), the closing component along `direction`, and
   `ssm_speed_cap(ld.distance, v_app, ...)` from `utils/iso_ssm.py`.
   Trip conditions, each requiring `iso_monitor_ticks` consecutive ticks **[E]**:
   - closing speed > cap + `iso_speed_tol`;
   - `ld.distance < iso_c_intrusion + iso_z_depth + iso_z_robot` (inside the stop region);
   - `cbf_status[2] == 1.0` (safety-chain fault) held for the same window.
   Publishes `/NS_1/iso_safety` `Float64MultiArray = [stop_latched, trip_reason, S_p_min,
   v_cap_min, v_closing_max]` at the monitor rate, always (a silent monitor is a dead monitor).
   Exposes `/NS_1/safety_reset` (`std_srvs/srv/Trigger`): clears the latch only when no trip
   condition is currently true; refuses with a message otherwise.
   Record in the module docstring **[E]**: SSM permits motion to resume automatically once
   `S ≥ S_p`; the latch (`iso_stop_requires_reset`) is a local choice for supervised experiments,
   and setting it false is equally standard-conformant.
2. **Filter** — subscribe to `/NS_1/iso_safety` (depth 1). In `_qp_tick`, when
   `stop_latched == 1.0` **or** the topic is older than `distance_timeout`: skip the row path and
   take the existing brake branch with `q̈_nom = −q̇ / iso_stop_tau`, solved through
   `box_only_solve` (`OSQP_LEVEL_BRAKE`) so the state box clips it to `±decel_max`.
   Do **not** publish zeros: with gravity added by the firmware, zero torque is coasting.
   Set `fault_braking = 1.0`.
3. **Commander** — in `_cbf_status_cb`'s neighbourhood add a `/NS_1/iso_safety` subscription;
   while latched, publish `q̈_nom = 0` and hold the phase (`self._gov_sigma = 0.0`, bypassing the
   `governor_sigma_min` floor). On reset, resume through the existing soft-reset path
   (`soft_reset_alpha`) so the reference does not step.
4. **Launch** — start `iso_safety_monitor` in `torque_control_stack.launch.py` after
   `real_time_distance`, with the same `_SINGLE_THREAD_BLAS` env and a delay parameter
   `torque_iso_monitor_delay_s` in `launch_defaults.yaml`.

---

## Step 7 — Fail-closed perception

**Objective.** Remove the three paths where perception silently produces "no constraint" instead
of "fault". Fault-behaviour requirements **[R]** apply to rated safety functions under ISO
13849-1:2023; this chain is **not** a safety function, so failing closed here is **[E]** good
practice, not compliance.

**Files**
- `franka_experiments/franka_experiments/nodes/real_time_distance.py`
- `franka_experiments/franka_experiments/utils/distance_engine.py`
- `franka_experiments/franka_experiments/utils/perception_msgs.py`
- `franka_experiments/franka_experiments/nodes/cbf_safety_filter.py`
- `franka_experiments/config/fr3_complete.yaml`
- new test: `franka_experiments/test/test_iso_perception_failclosed.py`

**Changes**
1. **Contact-regime publish hole.** In `real_time_distance`, the `valid` comprehension currently
   requires `min_thresh <= r.distance <= max_thresh`, so a frame in which every control point is
   closer than `min_thresh` (0.08 m) publishes only an empty heartbeat. Split the two roles:
   - `in_band` — unchanged, still decides the legacy `MultiDistance` / logging path;
   - `publishable` — `np.isfinite(r.distance) and r.distance <= max_thresh`.
   Publish `MultiLinkDistance` whenever `publishable` is non-empty. Keep the self-detection guard
   by flagging entries below `min_thresh` (`ld.confidence = 0.5`) rather than dropping them.
   Gate on a new `distance.publish_contact_regime: true` in `fr3_complete.yaml`.
2. **Unbounded hold.** In `DistanceEngine`, the "hold last value when no finite measurement"
   branch never expires. Add a per-control-point age counter; once the held value is older than
   `iso_distance_hold_max_s`, emit the result with `direction = None` so
   `build_cp_messages` marks it `valid = False` (that path already exists and is tested).
3. **Dead confidence gate.** `build_cp_messages` sets `LinkDistance.confidence = 1.0`
   unconditionally. Populate it with `find_pt_confidence(...)` (already in `perception_msgs.py`)
   so the filter's `min_confidence` gate stops being inert. Verify no regression in row count
   with `min_confidence = 0.2`.
4. **Empty-frame run.** In `cbf_safety_filter._on_distances`, count consecutive empty frames.
   If the previous non-empty frame had `min h < zone_r_active * d_safe` and the empty run exceeds
   `iso_empty_frame_max_s`, raise the same fault as a stale frame (brake + `fault_braking = 1`)
   instead of running with zero obstacle rows.

---

## Step 8 — Command feasibility: torque limits and braking authority

**Objective.** `S_p` assumes `a_s` is actually delivered. The `q̈ → τ` chain is open-loop (pure
feed-forward `M q̈ + C q̇`, no PD, no friction model), so this must be measured, bounded, and
faulted on. All **[E]**; the rated analogues are *stopping time limiting* and *stopping distance
limiting* (ISO 10218-1:2025, 5.5.6 / 5.5.7), which this is not.

**Files**
- `franka_experiments/franka_experiments/nodes/qddot_to_torque.py`
- `franka_experiments/franka_experiments/nodes/cbf_safety_filter.py`
- new test: `franka_experiments/test/test_iso_torque_limits.py`

**Changes**
1. In `qddot_to_torque.__init__`, load `effort_max` via
   `load_franka_joint_limits(FR3_JOINT_KEYS)` (`utils/config.py` already returns it).
2. In `_on_qddot_nom`, after `_compute_tau`: clip `tau` to `±effort_max`, then rate-limit
   `|tau[k] − tau_prev[k]| <= iso_tau_rate_max`, store `tau_prev`. Publish the saturation state on
   a new `/NS_1/torque_saturation` `Float64MultiArray[7]` (`1.0` per joint hitting either bound).
   `rt_torque_controller` already clips; the point here is that saturation becomes **observable**.
3. In `cbf_safety_filter`, using the existing `_diag_qddot_real` / commanded `qddot_safe` pair:
   while at least one obstacle row is active and `‖q̈_cmd‖ > 0.5 rad/s²`, compute
   `frac = (q̈_real · q̈_cmd) / ‖q̈_cmd‖²`. If `frac < iso_brake_frac_min` for
   `iso_brake_frac_ticks` consecutive ticks, publish `fault_braking = 1.0` and log at ERROR with
   both vectors. Keep it diagnostic-only (no stop) behind `iso_enabled` until the fault rate is
   known on hardware.
4. Delete nothing: `max_tau_delta` stays as the dead legacy key it is flagged to be;
   `iso_tau_rate_max` is the live one.

---

## Step 9 — Absolute ceilings: PFL and reduced speed

**Objective.** Bound the worst case that survives every software layer, including a perception
failure.

**Files**
- `franka_experiments/config/fr3_control.yaml`
- `franka_experiments/franka_experiments/utils/cbf_state_rows.py`
- new: `franka_experiments/scripts/iso_pfl_speed.py`
- new test: `franka_experiments/test/test_iso_pfl_ceiling.py`

**Changes**
1. New script computes `iso_v_pfl` with `iso_ssm.pfl_speed`. Inputs **[S]**: `F_max` and `k` for
   the chosen body region, read from ISO 10218-2:2025 **Annex M** (informative; formerly ISO/TS
   15066 Annex A) — read them from the standard, do not hard-code the example — plus
   `m_robot = M/2 + payload` with `M` the manipulator mass from the FR3 datasheet, and `m_human`
   the region's effective mass. Worked example for the module docstring **[E]**, using the
   Annex M **hands-and-fingers** row (quasi-static `F_max = 140 N`, `k = 75 N/mm`,
   `m_H = 0.6 kg`) with `M = 17.8 kg` and **no payload or gripper**:
   `m_R = 8.9 kg`, `μ = 0.562 kg`, `v_PFL = 140/√(0.562·75000) ≈ 0.68 m/s`.
   The script must print the inputs used, the region, and three warnings:
   - the transient limit for a region is roughly twice the quasi-static one; the quasi-static
     value is used here because a clamping event cannot be excluded in this cell **[E]**;
   - a mounted gripper and payload raise `m_R` and lower `v_PFL` — recompute per configuration;
   - **[R]** claiming PFL requires force/pressure **measurement** with a PFMD per
     ISO 10218-2:2025 clause 6.3.3 and Annex N; this calculation supports a preliminary risk
     assessment only.
2. Set `link_speed_max = min(link_speed_max, iso_v_pfl)` in `fr3_control.yaml`, and assert the
   same at startup in `cbf_safety_filter.__init__` when `iso_enabled` (raise, do not clamp
   silently). Keep the ordering invariant `retreat_cap_max_speed < link_speed_max`; lower
   `retreat_cap_max_speed` alongside it if the new ceiling breaks it.
3. Reduced speed: when `iso_mode == 'reduced'`, add one extra `G_SPD` row on the TCP control
   point with `v_allow = iso_tcp_reduced_speed`, built with the existing `link_speed_row(...)`,
   and have the Step 6 monitor apply the same cap to the TCP. Log the active mode at startup.
   **[S]** 250 mm/s is the standard's reduced-speed limit, required for manual modes;
   **[E]** applying it as a cell-wide derate during automatic operation is a local choice and is
   not a substitute for a rated speed-monitoring function.

---

## Step 10 — Status contract, diagnostics, logging

**Objective.** Make every ISO quantity observable in real time and in the bags. All **[E]** —
except that ISO 10218-2:2025 clause 7 **[R]** requires SSM and PFL parameters (stopping
distances, effective masses, contact areas, response times) to be documented for the user, which
Step 12 covers.

**Files**
- `franka_experiments/franka_experiments/nodes/cbf_safety_filter.py` (`_publish_status`)
- `franka_experiments/franka_experiments/utils/logging_utils.py` (`format_cbf_diag`)
- `franka_experiments/franka_experiments/nodes/experiment_logger.py`
- `franka_experiments/config/fr3_control.yaml` (the `cbf_status` contract comment block)

**Changes**
1. Append to `/NS_1/cbf_status`, preserving positional order (consumers index behind a `len()`
   guard): `data[5] = S_p_min`, `data[6] = v_cap_min`, `data[7] = v_closing_max`,
   `data[8] = iso_stop_latched`. Update the contract comment in `fr3_control.yaml`.
2. Add to the CBFDIAG line: `sp=`, `vcap=`, `vcls=`, `isostop=`.
3. `experiment_logger`: log the new `cbf_status` fields plus `/NS_1/iso_safety` and
   `/NS_1/torque_saturation` as CSV columns.

---

## Step 11 — Hardware safety layer (FR3 rated functions)

**Objective.** Put a rated envelope underneath the single-channel software filter. The software
cannot reach that rating; the robot's own functions can.

**Normative context.** ISO 10218-1:2025 (5.1.17, Annex C) classifies robots as Class I
(manipulator mass ≤ 10 kg, maximum force ≤ 50 N, speed ≤ 250 mm/s) or Class II. The FR3
manipulator exceeds 10 kg, so it is **Class II**, for which required safety functions must reach
**PL d / SIL 2** per ISO 13849-1:2023 / IEC 62061:2021 **[R]**. The FR3's own functions are rated
PL d / Cat 3 and are certified against **EN ISO 10218-1:2011**; re-assessment against the 2025
editions is the integrator's responsibility and is not claimed here.

**Files**
- new: `franka_experiments/config/safety/watchman_profile.md`
- new: `franka_experiments/scripts/iso_preflight_check.py`
- `franka_experiments/launch/torque_control_stack.launch.py`

**Changes**
1. `watchman_profile.md` records the Watchman configuration to be applied on the FR3 and
   validated by the Safety Operator:
   - **SLS-J** per-joint safe speed limits set ~20 % above the software cap
     (`velocity_box_margin * qdot_max`), so the software bites first and the rated layer is the
     backstop **[E]**;
   - **SLD** safe distance and/or **SLP-J** envelope for the cell;
   - safe inputs X3.2 / X3.3 mapped to the cell's presence-sensing device and to SMSS;
   - E-stop X3.1 and the enabling device present and reachable;
   - stopping times and distances (Category 0/1/2) from the product manual, cross-checked against
     Step 1 — the manual's data is measured to EN ISO 10218-1:2011 Annex B, renumbered
     **Annex H** in the 2025 edition;
   - **verify on the robot, do not assume:** the FR3 datasheet states that FCI cannot control the
     robot while **SLP-C**, **SLS-C** or **SLP-J** is active. It does not list SLS-J or SLD among
     those exclusions, but that is an absence of a note, not a statement of compatibility —
     confirm in Watchman on this unit before the cell design depends on either being active
     during FCI operation, and record the result in this file.
2. `iso_preflight_check.py`: fails loudly if `iso_enabled` is true while
   `iso_a_stop`/`iso_t_reaction`/`iso_z_depth` still hold the placeholder defaults, if
   `iso_c_intrusion` was lowered below the ISO 13855:2024 value without a `detection` measurement
   on record, if `d_safe < C + Z_d + Z_r`, if `link_speed_max > iso_v_pfl`, or if
   `/NS_1/iso_safety` is not being published within 5 s of startup.
3. Run the check from the launch file before the controller spawner; a non-zero exit aborts the
   launch.

---

## Step 12 — `SAFETY.md` and the deviation register

**Objective.** State exactly what is and is not claimed, so a reader (or a reviewer) cannot
mistake this for a compliant — let alone certified — system.

**Files**
- new: `franka_experiments/SAFETY.md`

**Changes**
1. One table with a row per requirement addressed, five columns: requirement with its clause
   (e.g. ISO 10218-2:2025 5.14.5 / Annex L) / tag **[R]|[S]|[E]** / where it is implemented
   (file + symbol) / status (**implemented** / **validated on hardware** / **certified**) /
   residual gap.
2. A **deviation register** listing every place this cell departs from the standards, each with
   its reason and risk-assessment reference:
   - `iso_c_intrusion` set below the ISO 13855:2024 value — or kept at 0.85 m, with the
     consequence that conformant SSM is unachievable in this workspace;
   - PFL limits derived by calculation, not measured with a PFMD (ISO 10218-2:2025 6.3.3,
     Annex N);
   - the depth pipeline is not a rated presence-sensing device (no IEC/TS 61496-4-3 assessment)
     and has no demonstrated detection capability;
   - reduced speed applied as a cell-wide derate rather than as a rated speed-monitoring
     function;
   - automatic resumption replaced by a manual reset latch (permitted either way; recorded for
     transparency).
3. Residual gaps that no step in this roadmap can close:
   - the CBF chain is single-channel Python over best-effort DDS — it is not a safety function
     and cannot meet the **PL d / SIL 2** required of a Class II robot's safety functions;
   - the FR3's rated functions are certified against EN ISO 10218-1:2011, not the 2025 editions;
   - obstacle rows remain slack-relaxable; forward invariance of the declared safe set is not
     guaranteed by the QP alone — Step 6 bounds the consequence, it does not restore the
     guarantee;
   - `a_s` is a measured minimum over a finite test grid, not a worst case over all payloads,
     configurations and directions;
   - `franka_sim` mirrors the pre-ISO math and is not a validation of the deployed filter.
