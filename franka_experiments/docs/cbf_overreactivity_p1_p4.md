# CBF over-reactivity: four gates on the velocity-derived margins

The acceleration QP tightens every obstacle row by five additive
velocity-derived terms (`h_brake`, `h_unc`, `h_lat`, `h_rng`, `h_std`) and a
`k_sigma` covariance term. On `experiments_irim2.zip` those terms were doing most
of the work, and most of that work was wrong. This change adds four independent
gates that withhold a velocity claim the perception stack cannot support. It does
not change the barrier itself, the gains, the row geometry, or the message
contract, and every gate ships **off**.

## What the recorded run showed

| Measurement | Value |
|---|---:|
| Obstacle rows per tick | 46 median, 67 p95 |
| `obstacle_velocity` on the wire | up to 7.78 m/s |
| Conditioned `v_obs` reaching the rows | up to 3.11 m/s |
| Ticks with at least one row violated | 23.1 % |
| ... of those, with the geometric gap still above `d_safe` | **90.6 %** |
| Tracks created in 91 s | 1727, median life 70 ms |
| Nearest-obstacle rows carrying **no** `track_id` | 48–52 % |
| Confirmed track ids, median lifetime | 0.27 s |
| Argmin surface-point hops above 10 cm | 14–35 % of updates |

On `irim1` the `k_sigma` term alone accounted for 40 % of `d_safe`; removing that
tightening moved the binding fraction from 43 % to 5.6 %.

The common cause is that a cluster centroid's velocity is not the velocity of the
surface the row is measuring. Merges, splits, short-lived ids and a sliding argmin
all fabricate closing speed in a room where nothing is moving.

## The four gates

All four reduce or withhold `v_obs` and `n_seen`. Because four of the five
tightenings are gated on those two quantities, zeroing them disables the lot in
one place. The geometric barrier, `d_safe` and the gains are untouched: a gated
row keeps its full `k0 * h_bar` response and loses only the anticipation.

### P1 — age and residual-floor persistence

`obstacle_velocity_min_age_s` (0.0) withholds a track's velocity until the track
has existed that long; with a 0.27 s median lifetime, most ids never earn it.
`obstacle_velocity_floor_min_frames` (1) makes the residual-velocity floor prove
itself over consecutive **perception** frames rather than rebuilds — at a 50 Hz
builder on a 90 Hz camera, counting rebuilds passed the gate in half the
configured time.

### P2 — track-keyed scene hold

`enable_static_scene_hold` classifies a *track* as scene from its own velocity
history (`static_scene_window_s` 1.0, `static_scene_speed_m_s` 0.08,
`static_scene_drift_m` 0.04, `static_scene_min_samples` 8) and releases it after
`static_scene_release_hits` 3 consecutive samples above
`static_scene_release_speed_m_s` 0.25.

### P3 — place-keyed scene memory

P2 can only act on rows that have a track, and 48–52 % of nearest-obstacle rows
do not. `enable_static_scene_voxels` keys the same judgement to a **place**: the
observed surface point is quantised to a `static_scene_voxel_m` 0.15 m cell, and a
cell whose median surface speed stays at or below `static_scene_speed_m_s` over
`static_scene_voxel_window_s` 0.5 s (minimum `static_scene_voxel_min_samples` 5)
is scene. The cell gate is deliberately looser than P4's estimator gate: sharing
P4's strict 8-sample window starved the cells and coverage collapsed from 27.5 %
to 3.5 %. The speed and release thresholds *are* shared with P2, so the same wall
cannot be scene by one definition and a body by the other.

### P4 — surface-point velocity cap

A tracked row's claim is capped at `surface_velocity_cap_k` 2.0 times the speed of
the surface point the row actually measures, plus `surface_velocity_cap_floor`
0.05 m/s. The cap can only lower a claim, never raise one, and never applies to a
fast-trusted row.

The estimator shape is the whole result here. Differencing an argmin is dominated
by per-frame sliding; a least-squares slope over `surface_velocity_window_s`
0.22 s is a telescoping difference that cancels it:

| Estimator | p50 read on a 0.158 m/s truth | Correlation | Capped **below** the true closing speed |
|---|---:|---:|---:|
| Median of 5 consecutive differences | 0.020 | +0.47 | **69.5 %** |
| Least squares over 0.22 s | 0.131 | **+0.94** | 3.4 % |
| Cluster-centroid tracker (for reference) | 0.148 | +0.68 | 15.8 % |

The first form shipped in this branch briefly and was unsound: it read 0.020 m/s
against a 0.158 m/s truth, pulled 1055 ball rows below their real closing speed,
and drove `min_h` to −0.035. After the rewrite the harm metric — the tracker was
right *and* the cap broke it — is 0, 0 and 2 rows across three runs.

## Configuration

| Key | Default | Gate |
|---|---:|---|
| `obstacle_velocity_min_age_s` | 0.0 | P1 |
| `obstacle_velocity_floor_min_frames` | 1 | P1 |
| `enable_static_scene_hold` | false | P2 |
| `enable_static_scene_voxels` | false | P3 |
| `static_scene_voxel_m` | 0.15 | P3 |
| `static_scene_voxel_window_s` | 0.5 | P3 |
| `static_scene_voxel_min_samples` | 5 | P3 |
| `static_scene_max_cells` | 4096 | P3 |
| `enable_surface_velocity_cap` | false | P4 |
| `surface_velocity_cap_k` | 2.0 | P4 |
| `surface_velocity_cap_floor` | 0.05 | P4 |
| `surface_velocity_window_s` | 0.22 | P4 |
| `surface_velocity_min_samples` | 8 | P4 |
| `surface_velocity_min_span_s` | 0.08 | P4 |
| `surface_velocity_jump_m` | 0.10 | P4 |

The launch arguments are named separately from the node parameters:
`vobs_min_age_s`, `vobs_floor_min_frames`, `static_scene_hold`,
`static_scene_voxels`, `vox_m`, `surface_velocity_cap`, `surf_cap_k`.
Simulation batches pass them through `set:=`, e.g.
`set:="static_scene_voxels=true;surface_velocity_cap=true"`. `CBFDIAG` reports
`vox=<rows>/<static>/<cells> scap=<rows> vsurf=<max |v_n|>`.

## What was verified

**Suites.** franka_experiments 1281, franka_simulation_experiments 111 (23 slow
Gazebo tests deselected), franka_simulation 39, fr3_student_sim 13. franka_rl has
2 failures (`blocking_fraction` default, MuJoCo gravity sag) that reproduce
with this branch's `franka_rl/config.yaml` reverted to `HEAD` — pre-existing.

**Off is inert.** The row-identity tests pin the off path bit-for-bit, and every
baseline run reports `vox_cells=0 scap=0 vsurf=0`: the classifiers are not
constructed.

**P3 against ground truth.** Six scenario runs in the furnished `diag` lab
(sweeping ball, pursuer chasing the flange, ball train, static sphere on the
path): roughly 41,000 rows held as scene, **none of them ever on a moving
obstacle**, including the pursuer run where P3 held 45.5 % of rows.

**P3's boundary, measured rather than assumed.** An untracked body closing on the
arm:

| Closing speed | Velocity claim withheld |
|---|---|
| 0.05 m/s | yes (82 of 180 frames) |
| 0.08 m/s (= `static_scene_speed_m_s`) | yes (65 of 180) |
| 0.10 m/s | no |
| 0.15, 0.25 m/s | no |

The band at and below the threshold is a deliberate trade: at 0.08 m/s the row
loses 0.8 mm of braking margin, 1.6 cm of standoff and the `k1` anticipation
term, and keeps its entire geometric barrier. Lower `static_scene_speed_m_s` to
narrow it — shared with P2, so it narrows both.

**The effect the gates were built to have.** `pentagon_clear` in the furnished
lab, where nothing moves, so every margin is phantom:

| | baseline (4 runs) | P3+P4 (3 runs) |
|---|---|---|
| Ticks with a velocity margin firing | 96.0 – 98.4 % | **77.1 – 82.5 %** |
| Rows whose place was recognised as scene | 0 | 53 – 60 % |

No overlap between the groups.

## What is not verified, and why

**Closed-loop numbers are not attributable.** Torque-reflex trips track the QP
rate, not the configuration:

| QP rate | 58 | 54 | 51 | 44 | 36 | 27 | 20 |
|---|---:|---:|---:|---:|---:|---:|---:|
| arm | base | base | P3+P4 | base | P3+P4 | P3+P4 | base |
| trips | 0 | 0 | 0 | 0 | 32 | 17 | **73** |

Both arms appear on both sides of a ~40 Hz threshold, and the worst run is a
baseline. Root cause found: **19 of 28 cores sit on the `powersave` governor**.
Until that is fixed, `min_h`, jerk, tracking error and verdict are measuring the
host, not the filter.

**P4 is inert in simulation.** Gazebo's sphere plus an accurate tracker produce no
merge/split phantom, so the room's tracked `v_obs` p95 moves only 0.169 to 0.162.
The 3.11 m/s the cap exists for lives on the hardware bag. P4 is safe here and
unproven here.

**`min_h` cannot see the furniture.** `truth.npz` tracks only the ball, so in
`pentagon_clear` (no ball) `min_h` is `+inf` by construction. Furniture contact
would not appear in it.

**One smoothness anomaly.** At matched rate, `ra1` (51 Hz, P3+P4) gave jerk p50
32.4 and EE RMS 0.337 against baseline runs at 54–58 Hz giving 4.0–11.3 and
0.094–0.196. One sample, outside the baseline range, consistent with the trade
being real — less anticipation means correcting later and harder.

## What is left to do

1. **Fix the measurement floor.** `sudo cpupower frequency-set -g performance`
   (needs the user's sudo), then repeat the `pentagon_clear` pairs. Nothing
   closed-loop can be claimed before this.
2. **Fix the QP starvation itself.** `cbf_safety_filter` runs at 18–58 Hz against
   its configured rate; a `SCHED_FIFO` thread under the Python GIL is the leading
   suspect for the `_qp_tick` gaps.
3. **Settle the smoothness question** with 3–4 more matched-rate pairs once 1 is
   done. If the `ra1` anomaly reproduces, the anticipation the gates remove is
   worth something and the thresholds need retuning rather than the gates being
   enabled as-is.
4. **Validate P4 offline on `experiments_irim2.zip`**, replaying the recorded
   `LinkDistance` stream through the builder. That is the only dataset where the
   phantom the cap targets actually occurs.
5. **Decide the P3 slow-approach band.** Whether withholding velocity from a body
   closing at 0.05–0.08 m/s is acceptable is a safety call, not a code question.
6. **Guard the runner against orphan nodes.** A leftover `/clock` bridge makes
   every cell time out with `ok=False`; two runs in this session also hung at
   startup with the controller manager never coming up.
7. **P5 — compose the five velocity margins as a worst case** instead of summing
   them. Summing is why the margins reached 40 % of `d_safe`.
8. **P6 — row budget / nearest-K selection.** 46 median and 67 p95 rows per tick
   is most of the QP cost, and the QP rate is the binding problem.
9. **Hardware validation.** Everything above is simulation or replay.

## Reproducing

```bash
# the harness, and the filter logic a simulation executes
cd /ros2_ws/src/franka_simulation_experiments && python3 -m pytest test -q
cd /ros2_ws/src/franka_experiments && python3 -m pytest \
    test/test_cbf_surface_velocity.py test/test_cbf_static_scene.py \
    test/test_cbf_state_rows.py -q

# a paired before/after run in the furnished lab
ros2 launch franka_simulation_experiments gz_cbf_stack.launch.py \
    condition:=kf motion:=pentagon scenario:=pentagon_clear scene:=diag \
    perception:=true gui:=false record_bag:=true run_dir:=<dir>
ros2 launch franka_simulation_experiments gz_cbf_stack.launch.py \
    condition:=kf motion:=pentagon scenario:=pentagon_clear scene:=diag \
    perception:=true gui:=false record_bag:=true \
    set:="static_scene_voxels=true;surface_velocity_cap=true" run_dir:=<dir>
```

## Source

New: `franka_experiments/utils/static_scene.py` (P2 and P3),
`franka_experiments/utils/surface_velocity.py` (P4), with
`test/test_cbf_static_scene.py` and `test/test_cbf_surface_velocity.py`.
Changed: `utils/cbf_state_rows.py` (the gates and their diagnostics),
`utils/config.py`, `config/fr3_control.yaml`, `config/launch_defaults.yaml`,
`launch/torque_control_stack.launch.py`, `utils/logging_utils.py`, and the
simulation harness mirrors in `franka_simulation_experiments`.
