# S6A.1 — Offline performance and timing investigation

> **Scope.** Offline analysis only, approved by the user on 2026-11-12. **No simulation, launch or
> build was run.** No source, test, launch or configuration file was modified. The input is the S6A
> recording of one cell (`static / full / seed 0 / perception true`, unpinned, baseline `ba68077`,
> HEAD `f708991`); see [S6A_GAZEBO_SMOKE.md](S6A_GAZEBO_SMOKE.md).
>
> **Evidence.** `~/franka_baseline_evidence/baseline_f708991/05_offline_timing/`: 4 scripts, their
> outputs, a README, 12 files, write-protected; `SHA256SUMS` hash
> `39821592ba4b28615ed519b161cb4cbb2a0ea1e8ad6e00f32a2e0986fb330127`. The S6A evidence
> (`04_gazebo_smoke/`) was hash-verified before use and not modified.
>
> **Labels** (00_INDEX §2): Verified (code read / exact recorded value), Derived (computed from
> recorded data or code), Inferred (reasoned, not measured), Unverified (not established),
> Runtime (recorded) (from the S6A recording).

---

## 0. Answer in brief

| Question | Answer | Label |
|---|---|---|
| Does the 73.6 Hz figure mix wall and sim time? | **No.** The CBF timer, the bag stamps, the episode window and the 100 Hz nominal are all in **simulated** time. 73.6 is ticks per **simulated** second. | Verified (code, launch log, bag metadata) |
| The user's hypothesis (73.6 wall → 106.7 per sim s) | **Refuted.** In wall time the loop ran at **≈ 46 ticks/s** (45.5–47.1, depending on the RTF anchor), not 107. | Derived |
| Is it a control-loop problem? | **Yes**, in the time base the plant integrates: 106 of the 401 scheduled ticks (26 %) never ran. The worst gap (91 ms sim) was 9 ms short of the torque controller's `accel_timeout` (100 ms). | Runtime (recorded); Derived |
| What limits it? | **Not Gazebo's slowdown**: a lower RTF gives each 10 ms sim period *more* wall time. The deficit starts exactly when the constraint builder starts working. Before START the same timer held 99.6 ticks/sim s. Late ticks follow a fresh rebuild 51 % of the time, against 2 % otherwise. Leading hypothesis: in-process contention between the rebuild thread and the QP thread (GIL / executor). | Derived (association); mechanism Inferred |
| Perception | The camera captures on the 90 Hz **sim** grid. The node processes 1 frame in 3 (30.7 per sim s, ≈ 19 per wall s) and drops the rest **by design** (`Queue(maxsize=1)`). Its throughput is wall-bound: about 58–65 ms of wall time per frame, with `wait = 0.0 ms`. | Verified (stamps, code); Derived |
| RTF 0.69 | 0.69 is the **whole-run** figure, startup included. Before START ≈ 0.82; in the episode **0.62–0.64**; no 1 s window reached 0.9. The contributors were not measured. | Runtime (recorded); attribution Inferred |
| Correction to S6A | S6A's "no obstacle row bound; the only intervention was the acceleration box" is **not supported**. `n_act` counts h̄ < 0, not binding rows. The slew bound also shaped the command. The obstacle-gap-dependent link-speed cap was emitted in 7.43–7.70 s. Box + slew leave a residual up to 1.17 rad/s². | Verified (code); Derived (residual); F-8-11 |
| Next experiment (not run) | `pentagon_static_baseline_bare / full / seed 0 / perception true`. The nominal passes at a 5.0 cm gap against `d_safe` = 10 cm, with a local 5 cm correction available. The acceptance criteria in §6.4 need the user's approval. | Derived; lead from a non-baseline run |

New findings: **F-8-8 … F-8-11**. Dated notes were added to F-8-5 and F-8-7 (PROGRESS §4). Nothing
was deleted.

---

## 1. QP rate analysis

### 1.1 Which clock drives the CBF timer? → simulated ROS time (Verified)

- `cbf_safety_filter.__init__` L525–528 creates both timers with `Node.create_timer` and no
  explicit clock:
  - `create_timer(1/cbf_update_rate_hz, _update_constraints)`;
  - `create_timer(1/qp_rate_hz, _qp_tick)`.

  An rclpy timer created this way runs on the node's ROS clock (rclpy API; external, not re-read
  here).
- Every node of the stack runs with `use_sim_time = True`:
  - `gz_cbf_stack.launch.py` L194 has `SetParameter(name='use_sim_time', value=True)`;
  - the node command lines in `launch.log` L665–672 contain `-p use_sim_time:=True` for
    `cbf_safety_filter`, `real_time_distance`, `qddot_to_torque` and `joint_state_publisher`.
- The ROS clock is therefore Gazebo's `/clock`, which the bag shows at 1000 messages per sim
  second with a constant 1 ms interval.
- **Runtime confirmation.** Before START the filter ran at about RTF 0.82. Its tick gap was
  p5 / p50 / p95 = 9 / 10 / 11 ms of **sim** time (`s6a1_phase.json`). A wall-driven timer would
  have produced gaps near 10 ms × 0.82 = 8.2 ms of sim time.

### 1.2 Which clock measures `qp_rate_hz`? → simulated time (Verified)

`closed_loop.py` L240–275 computes the rate as:

  `qp_rate_hz = (number of /NS_1/cbf_status messages with bag time in [t0, t1]) / (t1 − t0)`

- `t0` and `t1` are the manifest's `t_start_sim` / `t_end_sim` (7.170 / 11.180 s). The code
  comment says "achieved QP rate in SIM time".
- The bag is recorded with `ros2 bag record --use-sim-time` (`gz_cbf_stack.launch.py` L261). The
  recorder logged "use_sim_time set, waiting for /clock" (`launch.log` L132), and the bag's
  `starting_time` is 0.198 s.
- The bag's receive time agrees with the filter's own `tick_gap_ros_s` to within ±11 ms per tick
  (p5–p95). This is recorder jitter; it cannot move the count by more than about one tick at
  each edge of the window.

### 1.3 Which clock does the harness use, and in which clock is 100 Hz defined? → both sim (Verified)

- The nominal is `condition.cbf_params.qp_rate_hz = 100.0`. The `full` condition overrides the
  YAML's 300 Hz (`cbf_param_overrides: enable_go_around=false;qp_rate_hz=100.0`), and the filter
  logged `QP=100 Hz rows=50 Hz`.
- This nominal sets the period of the sim-time timer. `rate_verdict` (±5 %, `QP_RATE_TOL`,
  `closed_loop.py` L53–75) compares a sim-time count with a sim-time nominal, so the gate is
  self-consistent.
- On hardware, ROS time is wall time, and the same 100 means 100 per wall second.

### 1.4 The measured numbers

| Quantity | Value | Label |
|---|---|---|
| Scheduled periods in the episode (4.010 s sim × 100) | 401 | Derived |
| Ticks delivered (`/cbf_status`, = `qddot_safe` = `torque_cmd`) | 295 | Runtime (recorded) |
| Missed periods | 106 (26.4 %) | Derived |
| Ticks per **sim** second (the harness figure) | **73.6** | Runtime (recorded) |
| Ticks per **wall** second | **45.5–47.1** (episode wall 6.49 s from the supervisor's START/DONE lines; 6.27 s from an LSQ fit of 65 filter `CBFDIAG` wall/sim pairs) | Derived |
| Tick gap, sim (p5 / p50 / p95 / p99 / max) | 4 / 10.5 / 34.5 / 56 / 91 ms | Runtime (recorded) |
| Ticks **before START** (sim 2.0–7.17 s, no obstacle in the scene) | **99.6 per sim s**, gap p95 11 ms, max 28 ms | Runtime (recorded) |
| Rebuilds in the episode (nominal 50 Hz) | 145 (36.2 per sim s, 72 % of nominal) | Derived (change of `constraint_build_wall_ms`) |
| Rebuilds before START | 0 (no obstacle rows: `n_rows` = 0, build field NaN) | Runtime (recorded) |
| Build wall time per rebuild (p5 / p50 / p95 / max) | 9.6 / 16.5 / 25.7 / 34.8 ms | Runtime (recorded) |
| Solve wall time per tick, episode / pre-START (p50) | 3.0 ms / 1.1 ms | Runtime (recorded) |
| `n_rows`, episode / pre-START (p50) | 22 / 0 | Runtime (recorded) |

### 1.5 Is 73.6 Hz a real control-loop deficit? → yes (Derived)

The plant, the C++ controller and the commander all run in sim time:
- `joint_states` arrived at 991.5 per sim s with a 1 ms header interval (p5–p99);
- the commander ran at 299.8 per sim s against its 300.

So the robot received 73.6 safe accelerations per **simulated** second instead of 100.
Consequences:
- Each command was held for 13.6 ms on average and for up to 91 ms.
- `rt_torque_controller` keeps integrating its reference only while q̈_safe is younger than
  `accel_timeout` = 0.100 s (configure log, `launch.log` L260; ch. 09 §2). The worst gap left
  9 ms of margin.
- The slew bound stays physical because of the per-tick ratio compensation
  (`_slew_delta` L1513–1548, `slew_rate_compensation: true`).
- The discrete-time CBF argument assumes the nominal 10 ms hold. A 91 ms hold is 9 periods.

In this cell the deficit had no visible task cost: `task_done`, final error 9.1e-4 rad. That is one
cell, with no binding barrier (§5.4).

### 1.6 What limits the rate? (each mechanism, with its evidence)

| Mechanism | Evidence | Verdict | Label |
|---|---|---|---|
| **Wall/sim clock mixing in the gate** | §1.1–1.3 | excluded | Verified |
| **Gazebo slowdown (low RTF)** | The timer is sim-driven, so a lower RTF gives each 10 ms sim period more wall time: 15.6–16.2 ms at the episode's RTF. Before START the RTF was higher (≈ 0.82) and the rate was 99.6 per sim s. | excluded as a **direct** cause. It can still share a root cause (host load) with the deficit | Derived |
| **Work inside the filter process** | (a) The rate falls from 99.6 to 73.6 per sim s exactly when the constraint builder starts (rebuilds 0 → 36 per sim s, `n_rows` 0 → 22). (b) Of the ticks that are the first to see a new snapshot, **51 %** come after a gap > 15 ms sim; of the other ticks, **2 %** do (n = 145 / 150). (c) The rebuild timer itself misses 28 % of its periods. (d) The instrumented work alone is 0.886 s of wall time per sim second (rebuild 36.2 × 16.5 ms + solve 73.6 × 3.7 ms), 55–57 % of the 1.56–1.62 wall s available, before joint-state callbacks (991 per sim s), the node's own `/clock` handling, distance callbacks, the non-solve part of `_qp_tick` and executor overhead. | **leading explanation** | Derived (a–d) |
| ... through the GIL / executor | The three callback groups are threads of one Python process (`MultiThreadedExecutor(num_threads=3)`, L2073–2075), so pure-Python parts of the 16 ms rebuild serialise with the QP tick. Whether OSQP and NumPy release the GIL was not measured. | plausible mechanism | Inferred |
| **OS scheduling / host CPU contention** | Unpinned; 28 logical CPUs, none isolated; governor `powersave` / EPP `performance`; host load ≈ 0.4 at preflight. No CPU sampling during the run. `solve_wall_ms` and `constraint_build_wall_ms` are **elapsed** wall time: they include time spent waiting for the GIL or for a core, so compute and contention cannot be separated. | not excluded, not measured | Unverified |
| **Pure computation (CPU time)** | Same limitation: no CPU-time measurement exists in the recording. | not established | Unverified |

**Conclusion (Derived / Inferred).** The rate deficit is not a clock artefact and not a direct
effect of Gazebo's slowdown. It appears when the in-process constraint rebuild starts and is
statistically tied to rebuild delivery. Whether the binding resource is the GIL or a CPU core
cannot be decided from this recording. F-8-5's earlier wording ("compute-bound", budget 14.5 ms)
added a per-rebuild cost to a per-tick cost and used the whole-run RTF. It is corrected by a
dated note (PROGRESS §4).

Correlation of ticks per sim s with the window RTF: only four 1-s windows lie inside the CBFDIAG map
(r = −0.51). This is **inconclusive** and not used.

### 1.7 What a future approved run would need to measure

None of these needs a code change. Each adds a small external load, which must be recorded.

1. **Per-thread CPU time of `cbf_safety_filter`.** Sample `/proc/<pid>/task/*/stat`
   (utime + stime) at about 20 Hz with wall stamps, in a separate shell. A QP thread that sits
   idle while the rebuild thread burns CPU points to GIL serialisation. Both threads short of CPU
   points to the host.
2. **Host CPU.** `mpstat -P ALL 1` and `pidstat -u -t 1`, to detect saturated cores and to rank
   gz_server, `real_time_distance`, `depth_sim_adapter`, the filter, the commander and the recorder.
3. **A wall-clock count of the same ticks.** A second `ros2 bag record` of `/NS_1/cbf_status`
   **without** `--use-sim-time`. This gives exact ticks per wall second; the current wall/sim
   map rests on 65 log lines.
4. **Gazebo's own statistics.** Echo Gazebo's world-statistics topic (sim time, real time,
   iterations, RTF) instead of the reflex monitor's 1-s windows.
5. **Optional, needs approval.** `py-spy record --gil --idle` on the filter, if installing
   py-spy is approved.
6. **Not to be done silently.** A `--filter-cpu` or governor change is a different
   configuration. It is a separate, approved experiment (F-8-5 direction).

---

## 2. Wall time versus simulated time

The wall/sim map comes from three independent sources:
- **`CBFDIAG` lines.** The filter writes 65 of them during the episode (sim 7.36–11.33 s). Each
  carries an rcutils wall stamp and the filter's sim `t=`.
- **The episode supervisor.** It logs `START at t=7.1700` (wall …304.794) and `DONE at
  t=11.180` (wall …311.282).
- **The reflex monitor.** It reports RTF over 1-s wall windows.

| Topic / loop | Per sim s (episode) | Per wall s (episode) | Nominal | Label |
|---|---:|---:|---|---|
| `/clock` | 1000.0 | 618–640 | 1000 per sim s (`real_time_update_rate` 1000, 1 ms step) | Runtime |
| `joint_states` (sim header) | 991.5 | 613–635 | 1000 (controller manager `update_rate` 1000) | Runtime |
| `qddot_nom` (commander) | 299.8 | 185–192 | 300 | Runtime |
| `cbf_status` = `qddot_safe` = `torque_cmd` | **73.6** | **45.5–47.1** | 100 | Runtime / Derived |
| constraint rebuilds | 36.2 | 22.4–23.2 | 50 | Derived |
| `per_link_distances` | 30.7 | **≈ 19.0–19.6** | camera 90 per sim s | Runtime / Derived |

Per-wall values = per-sim value × episode RTF 0.618–0.640 (Derived).

**Episode RTF.**

| Estimator | Value |
|---|---|
| Supervisor START/DONE lines | 0.618 |
| LSQ slope of the 65 `CBFDIAG` pairs | 0.640 |
| Local slope between consecutive `CBFDIAG` lines (p50) | 0.617 (p5–p95 0.46–0.99, log-emission jitter) |
| Reflex windows overlapping the episode | 0.61–0.73 |
| *Clamped first estimate in `s6a1_timing.json`* | *0.679, superseded: the map starts at 7.36 s* |

**Clock-mixing inside the filter's own diagnostic (F-8-8, Verified).**
- `_watch_tick_gap` (L1551–1559) measures the gap with `time.perf_counter()`, which is **wall**
  time.
- Its threshold is `tick_gap_warn_factor × 1000 × dt_qp` (L417), i.e. 3 × the **sim** period =
  30 ms.
- In the episode the nominal wall period was 15.6–16.2 ms, so the warning fires at about 1.9
  nominal periods.
- It logged 54 warnings in the episode, while only 26 sim-time gaps exceeded 30 ms.

The warnings are diagnostic only (no control effect), but their count is not comparable across runs
with different RTF, nor with hardware.

---

## 3. Perception throughput

| Quantity | Value | Label |
|---|---|---|
| Camera `update_rate` | 90 (`world.sdf` L62), in **sim** time: the 122 intervals between processed captures lie within −0.78 … −0.11 ms of an integer multiple of 1/90 s (stamps quantised by the 1 ms physics step) | Verified (stamps) |
| Camera publication rate, either clock | **not measurable**: the image topics are not in the bag (`BAG_TOPICS` = non-image). Every processed capture is on the 90 Hz grid, and 24 consecutive-frame intervals exist. That every grid frame was rendered is **Unverified**. | Unverified |
| If every grid frame was rendered | 90 per sim s ≈ 57 per wall s at RTF 0.63 | Derived (conditional) |
| Processed frames = distance messages | 123 distinct capture stamps = 123 messages in the episode (one message per processed frame, no empty heartbeats) | Runtime (recorded) |
| Processing / publication rate | 30.7 per sim s; 19.0–19.6 per wall s | Runtime / Derived |
| Skipped frames between processed ones | intervals of 1–7 camera periods: 1: 24, 2: 22, 3: 33, 4: 24, 5: 16, 6: 2, 7: 1; mean **1.97 frames dropped per processed frame** (34 % of the ≈ 361 grid frames processed) | Derived |
| Dropped or skipped? | **Deliberately dropped.** `depth_callback` puts frames in `Queue(maxsize=1)` and on `queue.Full` discards the queued frame for the new one (`real_time_distance.py` L690–696; module docstring L34–36). Only the freshest frame is processed. | Verified (code) |
| Per-frame wall cost | `all` = 57.1 / 65.3 / 61.8 / 57.1 ms (4 timing lines in `launch.log`), mask stage 27.6–30.9 ms, `wait = 0.0 ms` on all four | Runtime (recorded) |
| Overloaded? | **Yes, compute-saturated.** `wait ≈ 0` is the node's own definition of "the compute is the ceiling" (L733–736). Measured throughput is about 1 frame per 51 ms wall (19.0–19.6 frames per wall s), and the logged per-frame cost is 57–65 ms. | Derived |
| Wall-bound, not sim-bound | Before START (no obstacle) the node logged "processing 22.7 depth frames/s" (rate from capture stamps, i.e. sim). At RTF ≈ 0.82 that is ≈ 18.6 per wall s; in the episode it is 19.0–19.6 per wall s. The wall throughput is about constant across a 25 % change of RTF. | Derived (one pre-START sample) |
| Capture → message received | p5 / p50 / p95 = 55 / 77 / 99 ms sim | Runtime |
| Age at the filter, per QP tick | capture age p50 112.5 / p95 154 / max 176 ms; receipt age p50 17 ms; snapshot age p50 21 / p95 36 ms (sim) | Runtime |

### 3.1 Does staleness affect the CBF constraints? → yes, in sim time (Verified / Derived)

- **The barrier value is capture-time geometry.** The main obstacle row computes
  `delta = ob.pr − ob.ph` from the message (`cbf_state_rows.py` L2509), and the robot-side point
  `pr` is the perception's, at capture time.
  - h̄ therefore does not contain the robot's own motion during the capture age.
  - Only aᵀq̇ is refreshed at the QP rate (ch. 07 §7, Derived there).
- **Size of the effect in this cell.** The fastest closing rate of the true minimum gap was
  0.142 m/s (20 ms difference on the 1 kHz truth grid). At that rate the h̄ optimism is bounded
  by about **16 mm** at the median age and **22 mm** at the p95 age (Derived upper bound).
  - S6A measured perception about 13.5 mm *conservative* in the same cell. The two partly
    offset, but their net effect per tick was not established.
- **Latency compensation covers 85 ms, but the measured age is longer.**
  - `enable_latency_compensation` predicts over a fixed `latency_t_blind` = **0.085 s**
    (YAML L2348: "the measured p95 at 30 fps", a hardware figure).
  - The measured sim capture age is 112 ms at the median and 154 ms at p95.
  - The compensation also covers only the obstacle's own motion (h_pred = 0 for a static
    obstacle), and it is gated on a confirmed track.
  - Logged as **F-8-9**.
- **Staleness gate.** Rows are dropped when the distance data is older than 0.5 s (ch. 08 §5.1);
  `distance_capture_age_max` = 1.0 s. Neither was approached: the maximum age was 176 ms.

---

## 4. Real-time factor

### 4.1 What was measured (Runtime (recorded), `reflex_summary.json`; Derived split)

- `rtf_run` = 0.690 = 11.317 s sim / 16.40 s wall, startup included.
- The 16 one-second windows were:

  0.008, 0.85, 0.78, 0.86, 0.87, 0.71, 0.82, 0.78, 0.88, 0.84, 0.73, 0.67, 0.63, 0.61, 0.61, 0.61.

  - The first window is Gazebo startup (8 ms of sim time in 1 s of wall time). It sets
    `rtf_min_window` = 0.0076, which therefore says nothing about the episode.
  - Without that window the run is 0.73.
- By phase:
  - **before START** ≈ 0.82;
  - **episode** 0.62–0.64 (§2);
  - **no window reached the 0.9 flag.**

### 4.2 Likely contributors (all Inferred; none measured)

There are no per-process CPU samples, so the apportioning below is a list of candidates, not a result.

1. **Depth rendering.** A 848 × 480 depth camera at 90 Hz of sim time (`ogre2`, EGL on the NVIDIA
   GPU), plus an advertised point-cloud topic (`DepthCameraSensor.cc:353` in the console).
   Whether Gazebo Fortress's physics loop waits for the sensor render thread is **Unverified**
   (external library behaviour). The camera was active before START too, which fits an RTF that
   was already below 0.9 then.
2. **Python nodes competing for cores.**
   - `real_time_distance` is saturated (about one core or more, §3).
   - `depth_sim_adapter` converts every 90 Hz frame (32FC1 → 16UC1).
   - Also: the CBF filter (§1), the commander at 300 Hz, `ground_truth` and `reflex_monitor` at
     about 1 kHz, the experiment logger and trajectory overlay, and `ros2 bag record` (about
     5,000 messages per sim s).
3. **The 1 kHz `/clock` and `joint_states` fan-out over DDS** to every subscriber.
4. **CPU frequency.** The governor was `powersave` (EPP `performance`). Whether clocks were
   sustained under this load is Unverified.

**Episode drop 0.82 → 0.63.** It coincides with the obstacle appearing at sim 7.12 s
(`truth.npz`: parked at z = −20 m until then). With it:
- the perception's distance stage starts working (27 ms in the first frame);
- the filter's rebuilds start (16.5 ms × 36 per sim s);
- the arm starts moving.

This **coincidence** is Derived. The causal split is Unverified.

### 4.3 Does the RTF affect…

| Item | Effect | Label |
|---|---|---|
| **validity of the QP-rate check** | The check is self-consistent (sim vs sim) and the low RTF did **not** cause the failure: it gave each period more wall time. But the outcome depends on RTF and host load: at RTF 1 each period has 36–38 % less wall time, so the same code on the same host would probably miss more ticks. A pass at low RTF does not certify real-time capability. | Derived; RTF-1 outcome Inferred |
| **perception age** | Perception is wall-bound (§3), so its sim-time rate and ages scale with RTF. At RTF 1 on this host it would deliver ≈ 19–20 frames per sim s, with ages about 1.5× those measured. The sim results are therefore **optimistic** about perception latency. | Derived (constant wall throughput); extrapolation Inferred |
| **controller execution** | None observed. The C++ controller runs inside Gazebo's step through `gz_ros2_control` (lockstep with physics): 991.5 `joint_states` per sim s, 1 ms header interval p5–p99. | Runtime; lockstep mechanism Inferred |
| **robot trajectory execution** | Indirect only. Plant, controller and commander are lockstep in sim time (commander 299.8 / 300). The trajectory depends on RTF only through the filter's and the perception's sim-time rates. The arm reached its goal. | Derived |
| **reliability of metrics** | Physical sim-time metrics are valid for the simulated system whatever the RTF: clearances, contact, task error, durations, reflex limits. **Timing metrics are not RTF-invariant and are host-dependent**: QP rate, tick gaps, ages, `rows_latency`, all `*_wall_ms`, and the `_qp_tick gap` warning count (F-8-8). `rtf_run` and `rtf_min_window` include the startup window. Comparisons need similar episode RTF, and the RTF must be reported per phase. | Derived; F-8-10 |

---

## 5. Verified findings versus hypotheses

### 5.1 Verified or Runtime (recorded)

- **The QP rate gate is consistent.**
  - The CBF timer, the bag and the harness all use simulated ROS time, and the 100 Hz nominal is
    a sim-time period.
  - 295 ticks in 4.010 s sim; 106 of 401 periods missed; worst gap 91 ms sim.
- **Before START the same timer held 99.6 per sim s**, with no rebuilds and `n_rows` = 0.
- **The camera captures on the 90 Hz sim grid.** The distance node processes 123 of ≈ 361
  frames, drops the rest **by design**, logs 57–65 ms of wall time per frame with `wait = 0`,
  and publishes one message per processed frame.
- **The filter's barrier uses capture-time geometry** (`pr`, `ph` from the message). The capture
  age is p50 112 / p95 154 ms sim, longer than `latency_t_blind` = 85 ms.
- **The tick-gap warning compares a wall gap with a sim-derived threshold** (F-8-8).
- **RTF:** whole run 0.69 (startup included); before START ≈ 0.82; episode 0.62–0.64.
- **S6A correction (F-8-11):**
  - `n_act` = count(h̄ < 0) (L1145), not a count of binding rows.
  - The slew bound was at its limit on 11 ticks.
  - The cap row family reported a gap-dependent cap of 0.564 m/s, against `link_speed_max`
    1.3 m/s, with a capped control point at 0.30–0.41 m/s in 7.43–7.70 s (`CBFDIAG vlink`).
  - One `CBFDIAG` line has h_qp < 0 (7.481 s, `fr3_link7#1`).
  - The zone ladder and the state governor never faded the nominal (`task` = `gov` = 1.00).

### 5.2 Derived

- Ticks per wall s 45.5–47.1, which refutes the 106.7 per sim s reading.
- **The rate loss is tied to rebuild delivery** (51 % vs 2 % late ticks).
- The instrumented wall work is 55–57 % of the wall time available per sim second.
- Perception wall throughput is about constant (≈ 18.6 → 19.0–19.6 frames per wall s) across the
  RTF change.
- Robot-motion optimism in h̄: ≤ 16 mm (p50 age) / 22 mm (p95 age) at this cell's maximum
  closing speed.
- q̈_safe ≠ slew(clip(q̈_nom)) on 61 ticks by more than 0.2 rad/s² (max 1.17), in 7.42–8.87 s.

### 5.3 Inferred (hypotheses)

- **GIL / executor serialisation** between the rebuild and QP threads is the mechanism of the rate
  loss.
- **At RTF 1** on this host, both the QP rate and the perception ages would be worse.
- The **link-speed cap** probably bound during the acceleration phase. Its row is
  `v̂ᵀJ(q̇ + q̈·0.10 s) ≤ cap`, and a 6 rad/s² joint-1 command adds about 0.25–0.3 m/s at the
  wrist.
- **RTF contributors:** depth rendering, Python CPU load, DDS fan-out, CPU frequency.
- `ROS_LOCALHOST_ONLY` may be ineffective (S6A, unchanged).

### 5.4 Unverified (not decidable from this recording)

- Compute versus OS-contention share of the elapsed wall times.
- Whether every 90 Hz frame is rendered, and whether rendering stalls the physics.
- Which constraint caused the residual after 7.70 s. Per-row activity is not recorded; only the
  family slacks (all 0) and `CBFDIAG` at about 16 Hz are.
- Whether an obstacle-dependent row changed the S6A command. **"No obstacle-driven intervention"
  is withdrawn**; it is now "not established" (F-8-11).

---

## 6. Recommended next experiment (NOT executed; requires approval)

### 6.1 Choice

| Field | Value |
|---|---|
| **Scenario** | `pentagon_static_baseline_bare` (`scenarios.yaml`, pentagon section) |
| **Condition** | `full`: the baseline arm (100 Hz, go-around off, tracker velocity, latency compensation, zone ladder…) |
| **Seed** | 0 (`start_offset_s` = 0, no jitter; seeds only change the start phase within one camera frame, ch. 12 §5.4) |
| Perception | `true` (real depth chain); scene `diag_bare` (DIAG room, furniture removed) |
| Duration | T = 30 s sim; at RTF about 0.62 that is about 48 s of wall time plus startup (harness timeout 400 s) |
| Resolved offline | `05_offline_timing/out/resolved_pentagon_static_baseline_bare_full_s0.json`: obstacle p0 = (0.55, −0.25, 0.449), r = 0.05, static, parked before the episode |

**Why this cell** (matching the requested properties):

- **The obstacle is static and slightly outside the nominal path, not on it.**
  - The commanded path is a circle of radius 0.25 m in the plane x = 0.40, centre (0.4, 0, 0.45),
    6 s per lap (`fr3_control.yaml` L2628–2631, `path_type: circle`).
  - The ball sits 0.15 m in **front** of the path's −y point.
  - The catalog states the design explicitly: "a local correction a barrier can actually produce",
    as opposed to `pentagon_on_path`, which "would demand a 0.15 m excursion the filter has no
    means to construct".
- **It is initially collision-free.**
  - `enforce_start_clearance` shifts nothing at d_safe = 0.10 (ch. 12 §5.5: worst start margin
    +2.5 cm over all scenarios).
  - The ball appears at the start of the episode, away from the arm at q0.
- **It is fair.**
  - The room is emptied (`diag_bare`), so phantom obstacles from furniture cannot drive the
    result; the earlier sweep showed large perception biases with furniture (§6.3).
  - The perception chain is the real one.
- **Rejected alternatives:**
  - `around`: the ball sits **on** the flange's circle, so the nominal collides and the cell
    needs a go-around, which is off in `full`.
  - `wedge`: the nominal passes between two balls with a 0.01 m gap, so the filter must stop
    the arm (a trap test).
  - `static`: already run (S6A); the nominal gap is 16.8 cm, so no barrier activity.
  - `approach_hold`, `static_noisy`: no expected crossing of d_safe.

### 6.2 Expected geometry and activation

| Item | Value | Label |
|---|---|---|
| Safety distance d_safe | 0.10 m (`fr3_control.yaml` L66; truth `d_safe` 0.1 in S6A) | Verified |
| Expected nominal (unfiltered) clearance | **5.0 cm**: centre distance 0.15 m − obstacle radius 0.05 − control-point radius 0.05, if the closest control point is the one on the traced point | Derived (geometry) |
| Prior measurement | 5.00 cm "commanded (unfiltered) min gap", closest CP `fr3_link8#1` (`sim_runs/phase1_static_baseline_20261102/PHASE1_REPORT.md`) | Runtime (recorded) **at `2d87ab6`, dirty, condition `minimal`, scene `diag`: not a baseline (F-6-1); lead only** |
| Shortfall the filter must recover | 5.0 cm, along −x (toward the robot, free space) | Derived |
| When the barrier should act | Once per lap near the −y point (y ≈ −0.25, z ≈ 0.45): up to 5 passes in 30 s. The approach is grazing (the path tangent there is ±z), so the closing rate is low and the HOCBF row's h_qp should go ≤ 0 shortly before the gap reaches 0.10 m. | Derived; timing Inferred |
| What other runs saw (leads, not baseline) | `diag_bare` / `minimal` at `2d87ab6`: truth min gap 6.5–6.65 cm (d_safe violated 2.7–5.1 s, no contact, QP ≈ 100 Hz, perception bias +20–35 mm); `diag` / `minimal`: 4.9–6.9 cm | Runtime (recorded), non-baseline |
| Confounder to separate | In `full`, the commander's phase governor and the filter's zone ladder react to `d_obs_min` (ch. 06 §5–6). The governor changes the path **timing** (σ), not its geometry. A displacement of `ee_actual` **out of the path plane** (x < 0.40) near the ball can therefore come only from the filter's modification of q̈. | Derived (ch. 06) |

### 6.3 Signals to record (all already recorded by the harness; no change needed)

- **`/NS_1/ee_desired`, `/NS_1/ee_actual`.** The pentagon commander's path point and the
  forward-kinematics hand position; both are in `BAG_TOPICS` and were empty in S6A. Used to
  measure the out-of-plane displacement.
- **`/NS_1/qddot_nom`, `/NS_1/qddot_safe`, `/NS_1/torque_cmd`, `/NS_1/franka/joint_states`.**
  Map joint states by name (S6A).
- **`/NS_1/cbf_status`.**
  - `n_rows`, `slack_obs`, `violation_without_slack_obs`, `violation_with_slack_obs`;
  - `d_obstacle_min`, `fallback_level`, `fault_braking`;
  - `tick_gap_ros_s`, the four ages, `solve_wall_ms`, `constraint_build_wall_ms`, `slack_cap`.
- **`/cbf/per_link_distances`:** perceived gaps and capture stamps.
- **`truth.npz` and `/sim/truth/cp_gaps`:** true gap per control point on the 1 kHz grid.
  Check for holes around every minimum (F-6-5).
- **`CBFDIAG` lines in `launch.log`.**
  - `n_act`, `h_qp`, `link`, `zone`, `task`, `gov`, `vlink`, `retreat`;
  - `dq_rad` / `dq_ort`, `cart_rad`.
  - Use `launch.log`, not the per-node `ros_log` file, which can lose lines at teardown (§7).
- **Runner outputs:** `pentagon_tracking` (from the commander CSV), `reflex_summary.json` (RTF
  windows, trips), `cell_result.json`.
- **Recommended additions (external, no code change; each needs approval, §1.7):** per-thread CPU
  of the filter, host CPU, a wall-clock bag of `/NS_1/cbf_status`, and Gazebo statistics.

### 6.4 Proposed acceptance criteria (REQUIRE USER APPROVAL; thresholds are proposals, Q-6-3)

**V. Validity of the run**
- V1. It completes: no timeout and no leftovers. `working_tree.patch` is saved first, and
  install == source (F-6-11).
- V2. **Truth grid:** no hole larger than 10 ms within ±0.5 s of every truth-gap minimum (F-6-5).
  Otherwise "no contact" is not claimed.
- V3. **RTF:** report it per phase (pre-START / episode). Proposal: no acceptance bar (Q-6-3), but
  flag an episode RTF below 0.5.
- V4. **QP rate:** report it per sim s and per wall s, together with the worst tick gap.
  Proposal: given F-8-5, a `RATE_FAIL` is recorded but does not void this **diagnostic** run,
  unless any gap reaches `accel_timeout` (0.100 s sim).
- V5. Report fallbacks other than `full`, `fault_braking` > 0 and reflex trips. Each one is
  explained or fails the run.

**B. The barrier is active**
- B1. On every pass near the −y point, at least one obstacle row on the hand / wrist control
  points has h_qp ≤ 0, **or** `slack_obs` > 0, **or** `n_act` ≥ 1.
- B2. During each such window, ‖q̈_safe − q̈_nom‖ > 0.2 rad/s² on ≥ 5 consecutive ticks, and
  slew(clip(q̈_nom)) does **not** reproduce it (the residual test of §5.2).
- B3. The fade terms are logged: if `task` < 1 or `gov` < 1, their share is reported separately.

**C. The behaviour changes because of the barrier**
- C1. The true minimum gap of each −y pass is at least 2.0 cm larger than the nominal 5.0 cm,
  i.e. ≥ 7.0 cm. Proposal; the leads suggest about 6.6 cm, so the bar itself is the user's call.
- C2. Near the ball, `ee_actual` leaves the path plane toward −x (away from it) by at least 1 cm
  more than at the opposite (+y) point of the same lap. The +y side serves as the in-run control.
- C3. Tracking recovers within 1 s after each pass, to the +y-side error band.

**S. Safety**
- S1. No contact: true gap > 0 everywhere, under V2.
- S2. Time below d_safe and penetration depth into d_safe are reported. **Whether d_safe must
  hold is the user's decision:** the non-baseline leads show it was not held (6.5–6.7 cm).
- S3. The reflex monitor passes. Any torque-rate clip is reported.

**Explicitly not claimed by one seed:** statistical performance, robustness to phase or noise, and
equivalence to hardware (F-8-10).

**Decisions for the user before approval:**
- scene `diag_bare` vs `diag`;
- whether an obstacle-free reference run (`pentagon_clear`, a second cell) is wanted for C2/C3;
- the four thresholds above;
- whether to add the §1.7 measurements to the same run.

---

## 7. Remaining uncertainties

1. **Cause of the QP rate loss.** The data shows an association with rebuilds. Whether the
   mechanism is the GIL or host cores is unknown (no CPU sampling; wall times are elapsed).
2. **Camera publication rate.** It was not recorded, so it is not known whether frames are lost
   before the perception queue (render or bridge) in addition to the deliberate drops.
3. **The wall/sim map** rests on 65 log lines and 2 supervisor lines (±0.02 on the episode RTF).
   The pre-START RTF (≈ 0.82) comes from aligning the reflex windows and is approximate.
4. **Command attribution in S6A.** The residual after 7.70 s is unexplained. Per-row duals and
   residuals are not recorded, so the binding constraint cannot be named offline. The
   `vlink` → cap-row link is Inferred.
5. **Perception versus truth.** The −13.5 mm offset (S6A) and the ≤ 22 mm staleness bound use
   different alignments. The net per-tick error of h̄ is not established.
6. **Teardown truncates per-node logs.** `real_time_distance`'s own `ros_log` file holds 33 lines
   against 35 in `launch.log`: its last two timing lines are missing (the node died on SIGINT,
   F-8-6). The filter's file is complete (65 `CBFDIAG` lines, 55 warnings, the same in both).
   Use `launch.log` for counts.
7. **Hardware relevance.** Every timing number here belongs to this host at RTF 0.6–0.8. Nothing
   transfers to the real FR3 without a hardware measurement.
8. **Superseded figures in the evidence.** In `s6a1_timing.json`, the fields `rebuild`,
   `budget_per_sim_s`, `gap_vs_rebuild` and `episode_rtf_filter_clock` used a method that
   over-counted rebuilds or clamped the map. They are kept, marked as superseded in the evidence
   README, and replaced by `s6a1_phase.json` and §2.

---

## 8. Changes to the documentation (this step)

- **New findings:**
  - **F-8-8** tick-gap warning clock mismatch;
  - **F-8-9** capture age longer than `latency_t_blind`, with capture-time robot geometry in h̄;
  - **F-8-10** sim-time timing metrics are RTF- and host-dependent and optimistic for
    wall-bound components;
  - **F-8-11** the S6A report overstated "no obstacle-driven intervention", and F-8-5's budget
    argument was flawed.
- **Dated notes** on F-8-5 and F-8-7 (PROGRESS §4).
- **Correction notes** in [S6A_GAZEBO_SMOKE.md](S6A_GAZEBO_SMOKE.md) (the original text is kept).
- Chapter 16 B.10 / B.11 rows; total 132 → 136.
- Checkpoint C-059 and the resume card.
