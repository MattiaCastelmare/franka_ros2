# Ball throws: closed-loop evaluation, smoothing, and what limits the dodge

2026-09-30 (night). Everything below was measured with `scripts/ball_closed_loop.py` on the recorded
throws (`rosbag/ball_throws_2`: 9 passes, `ball_throws_3`: 6 passes). **Nothing here has been run on the
robot.** The plant is a model; read the numbers as *relative* comparisons (see "How far to trust it").

## 1. The bench

`bag_replay.launch.py` is open loop: q and q̇ are the recorded ones, so the arm never moves out of the way
and nothing can be said about what a command *does* to the arm. `ball_closed_loop.py` closes the loop:

| part | real or modelled |
|---|---|
| `cbf_safety_filter` | **real** — the node class, built in-process with the launch's own parameters, ticked on a simulated clock (`_qp_tick` 100 Hz, `_update_constraints` 50 Hz) |
| perception | **real** — the `/cbf/per_link_distances` messages a replay of the recorded depth produces (obstacle points, tracks, velocities, covariances, capture stamps, the measured capture→receipt latency of each message) |
| ball | colour-tracked ground truth, used only to score clearance |
| arm | **modelled**: q̈ = 0.96 · first-order-lag(10 ms)(q̈_safe). Identified on `ball_throws_3` (best of a delay × τ × gain grid; no dead time, residual 0.6 rad/s² on a 1.1 rad/s² signal) |
| task | **modelled**: `pentagon_qddot_commander`'s measured-channel law (Cartesian PD 20/9, cart_err_max 0.08, DLS pseudo-inverse, posture null-space, phase governor on the filter's published gap). Replica vs the recorded q̈_nom on quiet stretches: median 0.41, p90 1.09 rad/s² |
| obstacle rows for the simulated arm | control points carried on their links to the simulated pose; obstacle point and track as recorded; surface gap re-derived with the recorded capsule offset |

Validation: with the filter off, the modelled loop tracks the circle to 1–2 cm and reproduces the recorded
clearance to ~2 cm. With the filter on and the old parameters it reproduces the live run's qualitative
behaviour (saturated ±10 rad/s² square waves, sign flips every 100–150 ms).

Modes: default (as recorded) · `--threat M` (translate each recorded flight, perpendicular to itself, until it
passes M m from the axis of the control point it would have passed nearest — **an aimed throw**; a dead-centre
throw is `--threat 0`) · `--ball-only` (drop every row that is not on the ball) · `--oracle`, `--probe`,
`--trace`, `--qtrace`, `--gtrace` (diagnostics). Helper scripts: `smoothness_report.py` (jerk / chatter /
sign flips of any bag), `pass_plot.py` (one figure per pass from a live bag).

## 2. What was wrong (live, `ball_throws_3`, old parameters)

* 22 % of the ticks inside a ball pass sat at |q̈| ≥ 5.9 rad/s²; 77 % of them had the filter overriding the
  nominal. The command was a **±10 / ±6 / ±3.5 rad/s² square wave**, flipping sign every 100–150 ms and
  then swinging back (`docs/plots/bt3_passes.png`, `bt3_nom_vs_safe.png`).
* Command jerk p99 = **500 rad/s³** (= the slew limit: 0 → 10 rad/s² in 20 ms), realised 441–453.
* Cause: the obstacle row for a 3–4 m/s ball demands 20–40 m/s² of retreat against an authority of ~5
  (`h_qp` = −19…−39, slack 17–40, priced 1000×), so the QP is at the corner of its box on every joint that
  helps, and every 50 Hz row update moves it to another corner.
* `ball_throws_4` **cannot be used for smoothness**: the arm never moved in that recording (|q̇| ≤ 0.02 for
  135 s, `ee_actual` constant) while the command sat at 6–7 rad/s², so its `trk_err` is "command − 0". The
  conclusion "the wider acceleration box made the arm jerky" was drawn from it and is **not supported**
  (the box was still the wrong thing to widen — see §4 — but not for that reason). Sampled colour frames show an idle cell and a replay through the current perception finds no fast track within 0.7 m of the arm anywhere in the recording: it carries no throws either.

## 3. What changed

| key | before | now | effect (closed loop, 15 recorded passes) |
|---|---|---|---|
| `max_qddot_delta` | 5.0 (500 rad/s³) | **1.5** (150 rad/s³) | command jerk p99 500 → 150, realised 450 → 144, sign reversals 24/29 → 16/22 (bt2/bt3); clearance −0.3 cm |
| `qddot_accel_limits` | `[]` (j5, j7 at 10) | **[6, 2.585, 3.5, 4, 6, 5.5, 6]** | peak command 10 → 6; clearance −0.2 cm (braking curve keeps 10) |
| `velocity_box_margin` | 0.9 | **0.7** | aimed throws: peak \|q̇\|/firmware envelope 0.92–0.94 → 0.72–0.73; clearance −0.1…−0.3 cm per 0.15 |

Synthetic scenarios (`scripts/cbf_scenarios.py`, `--set` to compare): same contact outcomes; jerk p99 of the
command 824–1053 → 301–367 rad/s³; wedge livelock time below the barrier 4.8 → 3.1 s.
Open-loop replay of the two bags (real recorded states): command jerk p99 500 → 150, chatter p99 3.0 → 1.3
rad/s², sign flips/s 1.9 → 0.8, peak 10 → 6, same activity (42 % of ticks) and same deviation from nominal.

`franka_sim/config.yaml` follows (`max_qddot_delta`, `velocity_box_margin`): a policy trained before today saw
the old values.

## 4. What was tried and did NOT pay (all removed or left off)

Measured on the same passes; replay-to-replay noise of the *perception* is ±2 hits / ±2 cm, so only large
effects count. "Aimed" = `--threat 0` (dead-centre), hits = clearance < 0.

| experiment | result |
|---|---|
| demand cap (`aᵀq̈ ≥ η · authority`), η 0.4–0.9 | smaller excursion, **less** clearance; nothing once the velocity box was tightened — removed |
| ballistic prediction rows (`tracking.prediction`) | aimed hits 7/9, 5/6 vs 5/9, 5/6 — worse — stays off |
| collision-cone mode (`replace_current`) | hits 7/9, 5/6, excursion halved, clearance lower — stays off |
| wider detection band (`max_thresh` 1.0 / 1.3) | within noise |
| predicted-miss gate on the anticipatory terms | −10…15 % excursion, −1 cm clearance — removed |
| sideways dodge planner (track position/velocity/ballistic closest approach → J⁺ push on q̈_nom) | **no effect**; with the QP at the corner of its box nothing added to the nominal has room — removed |
| raising lateral / outrun evasion gains 10× | changes the 4th digit |
| direction-preserving scaling of q̈_nom into the box | no effect — removed |
| wider acceleration box ([8,5,6,6,10,7,10]) | the earlier "jerky" verdict rested on `ball_throws_4` (arm not moving); on the model it adds ≤ 0.3 cm of clearance for 40 % more peak command — left reverted |

## 5. What actually limits the dodge

An **oracle** that knows the true flight and pushes the threatened control point sideways with a smooth
raised-cosine acceleration (filter still active), aimed throws, ball-only:

| warning before closest approach | hits (bt2 / bt3) | mean clearance |
|---|---|---|
| none (current filter, ball-only) | 4/9, 2/6 | −0.6 / +0.5 cm |
| 0.30 s, 4 m/s² | 3/9, 3/6 | 0.0 / +0.6 cm |
| 0.45 s, 8 m/s² | 2/9, 1/6 | +3.7 / +7.2 cm |

So at the ~0.3 s the perception gives today (first row on the ball 350 ms before closest approach, trusted
~330 ms, median filter reaction ~250 ms) **a dead-centre 4 m/s throw cannot be dodged by any control law** with
this arm's authority; ~0.45 s is where it starts to work. More lead is the lever, not a cleverer reaction:
seeing the ball earlier (RGB detection of the ball, which the colour camera sees at 2–3 m; a longer depth
search band did not help), or a less batched depth stream (the D455 delivers 90 fps in bursts of 3 every 33 ms).
A person standing near the arm (the catcher) costs about as much again: with their rows left in, aimed hits
go 2/6 → 5/6 on `ball_throws_3`.

### Where the 0.3 s comes from (open)

The colour+depth ground truth sees the flying ball **0.4–1.0 s (median ~0.5 s)** before closest approach; the depth
pipeline's first row on it is at **0.29 s (bt3) / 0.35 s (bt2)**, a track with 3 frames ~20 ms later. That
moment did not move in any replay of any perception setting tried tonight: `pixel_step` 4/5/6,
`cluster_min_points` 3/4/5, `roi_pad_px` 180/400/700, `max_thresh` 0.7/1.0/1.3/2.0/2.5, `cbf_obstacle_horizon`
1.2/2.5, `perception.multi_obstacle_max_rows` 24/60 (tracks >= 3 frames move by at most 0.05 s). So it is gated by something none of those touch (what the
robot-silhouette / depth gate lets through for a small fast object, or the stereo depth on a ball that far
away) and ~0.2 s of warning is sitting there. Finding that gate is worth more than any control change. Note
`rosbag/replay_cfg/p_thr10.yaml` and the earlier "max_thresh 1.0: no gain" never changed the first-row time
either, so that experiment said nothing about the band itself.

Three prediction-row replays vs three baseline replays (aimed, bt2): hits 7,7,6 of 9 vs 3,5,4; bt3 5,5,5 of 6 in both;
mean clearance −1.1/−2.5 cm vs −0.2/−1.6 — `tracking.prediction` is **worse**, not merely useless.

## 6. How far to trust it

* The modelled arm exaggerates excursions by ~1.6–2× (simulated off-path 21 cm vs recorded 12 cm mean on
  `ball_throws_3`); use it for ordering variants, not for absolute centimetres.
* Recorded people do not react to the simulated arm moving, so scene-driven excursions are overstated.
* Clearance uses fixed capsule/ball radii (5 cm + 3.3 cm) and inherits the 5.9 cm depth→colour offset (see
  `fr3_complete.yaml`).
* Every perception replay differs slightly (real-time playback): ±2 hits, ±2 cm between replays of the same
  configuration. Average several replays before believing a perception change.

## 7. Next hardware run

1. `colcon build --packages-select franka_experiments` (the install is a copy), launch the torque stack as usual.
2. Record with `start_rosbag` (now includes `/NS_1/franka/joint_states`), check the arm actually moves in the
   recording (`python3 scripts/smoothness_report.py <bag>`: realised accel p50 should be ~0.5–1, not 0.03).
3. Expect: peak `qddot_safe` 6, command jerk ≤ 150 rad/s³, `vrat` ≤ ~0.75 in CBFDIAG, `slew=` bites more often.
4. If the dodge looks too timid: raise `max_qddot_delta` to 2–3 first (jerk 200–300, nearly free in the model),
   then `velocity_box_margin` to 0.8. If it still looks jerky: lower `max_qddot_delta` to 1.0.
5. Score a live bag the same way the model was scored: `scripts/pass_plot.py <bag> <replay of it> <truth.npz> out.png`.
