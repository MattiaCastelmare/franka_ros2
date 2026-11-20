# Baseline validation analysis tools

Offline, read-only analysis scripts for the verified-baseline phase (S6B and later). They never
modify a recording, a production file or the installed workspace. Large outputs go to
`~/franka_baseline_evidence/baseline_f708991/`, never into the repository.

| Script | Step | Runs in | What it does |
|---|---|---|---|
| `s6b1_reconstruct_clearance.py` | S6B.1 | `franka_ros2` container (rosbag2_py, pinocchio, installed packages) | Recomputes the sphere clearance of every control point from the **measured** joint states in the cell bag, using `simexp.truth_geometry` (the ground-truth node's own code) and the recorded obstacle poses; compares it sample by sample with `truth.npz`; lists the truth holes and the bag samples inside them; reports per-pass minima |
| `s6b1_supplementary.py` | S6B.1 | host or container (numpy) | Disagreements and their cause, a Lipschitz bound between samples, the truth node's pose-pairing age, and the reconstruction-only spans |
| `qpperf_replay_profile.py` | QP performance, phase 1 | container (isolated domain; installed `franka_experiments`) | Instantiates the real `CBFSafetyFilter` with the S6B parameters and calls its own callbacks in recorded sim-time order (rebuild every 20 ms, tick every 10 ms), with **no executor**: uncontended per-callback cost plus cProfile of the rebuild, the tick and the joint-state callback |
| `qpperf_executor_replay.py` | QP performance, phase 2 | container (isolated domain) | Spins the real node in its production `MultiThreadedExecutor(3)` (or, as a harness-only variant, `SingleThreadedExecutor` or another switch interval) while `ros2 bag play` replays `/clock`, the joint states, `qddot_nom` and the distances at a set rate; per-call wall/CPU/TID, per-thread CPU, executor loop counts |
| `qpperf_late_tick_overlap.py` | QP performance, phase 2 | host or container (numpy) | From the phase-2 raw rows: P(late tick \| rebuild overlap) and tick wait with and without a concurrent rebuild |
| `qpperf_gil_probe.py` | QP performance, phase 3 | container | Microbenchmark: does OSQP `solve()`, Pinocchio FK/Jacobian or small-array NumPy release the GIL? (pure-Python probe thread) |
| `r1_sequential_outputs.py` | R1 | container (isolated domain) | Executor-free, deterministic replay of the S6B bag through the installed node (as phase 1, integer-ns event grid; rebuild before tick at each 20 ms point). Saves every published command, every `_solve` result and the row count per tick, plus the full parameter namespace. Used to prove that R1 leaves the equations bit-identical and as the schedule-ideal reference |
| `r1_executor_replay.py` | R1 | container (isolated domain) | As `qpperf_executor_replay.py`, but the executor is built by the **production** `make_executor` from the node's own `executor_mode` (`-p executor_mode:=<mode>`), and each tick records the inputs it used (joint-state header, nominal, distances, constraint-snapshot build time) and each command |
| `r1_compare.py` | R1 | host or container (numpy) | Per-run timing, CPU, input ages in reconstructed true playback time, deviation from the sequential reference per 10 ms slot, and cross-run deviations (within-mode noise floors and multi vs single) |
| `r1_snapshot_attribution.py` | R1 | host or container (numpy) | Splits each run's ticks by whether the constraint snapshot they used was built in their own 20 ms period, and the output deviation of each group (F-8-17) |
| `m1_executor_replay.py` | M1 (C-074) | container (isolated domain) | Runs `r1_executor_replay.py` **unmodified** (via `runpy`) in three arms: `single`, `mt1` (sets the module constant `MULTI_EXECUTOR_THREADS = 1` in its own process only, so the production `make_executor` builds `MultiThreadedExecutor(1)`) and `mt3` (production default). Adds only recording: each `/cbf_status` row, the executor class and worker count, subscriptions and timers, and the resolved parameter dump. `M1_DRY=1` stops after construction |
| `m1_analyze.py` | M1 | host (numpy, sqlite3 read-only) | Per-run metrics in the E3 window [t0+2, t0+30] s (sim and wall separately), the pre-registered validity checks including workload equivalence with E3-05's recorded `n_rows`, per-arm statistics and the position of `mt1` between `single` and `mt3` |
| `m1_run.sh`, `m1_batch.sh` | M1 | host (drives the container) | One replay with pre-checks (no ROS/Gazebo processes; governor/EPP as recorded, never changed), copy-back and validity check; the nine runs in the sealed Latin-square order, stopping at the first abort or invalid run, with no retry |
| `m3_load.sh` | M3 (C-075) | host | Synthetic CPU load: `start <n> <cap_s>` launches n pure-Python spin workers (marker `M3_LOAD_WORKER`, own session, nice 0, no pinning, `timeout --signal=KILL` cap); `stop` kills them and verifies none is left; `count` uses an anchored regex (the `timeout` wrappers are not counted, F-8-50). Never changes a CPU setting. |
| `m3_host_sampler.py` | M3 | host | 0.5 s sampler: per-core `/proc/stat` jiffies, `scaling_cur_freq`, `x86_pkg_temp`, monotonic (= the container's `perf_counter`) and epoch time. Thermal guard (Addendum 01): ≥ `temp_abort_mC` in `n_consecutive` samples kills the load workers and writes `<out>.thermal_abort`. |
| `m3_run.sh`, `m3_batch.sh` | M3 | host (drives the container) | One run: pre-checks (no ROS/Gazebo; governor/EPP as recorded; 0 workers; idle ≤ 2 busy cores; package < 85 °C), sampler, load and 45 s ramp for L1, the unchanged M1 replay, worker counts at start and end, cleanup and copy-back; rc 3 abort, 4 invalid (batch continues), 5 harness failure. The batch runs the 18 runs in the sealed order with 60 s / 15 s pauses and no retry. |
| `m3_analyze.py` | M3 | host (numpy) | Runs `m1_analyze.py` per load, adds the host metrics (busy cores, P/E frequency, temperature) cut to the replay window, the load gates, stale-event counts, per-thread CPU, headroom (projected saturation rate), CPU budget, the E3 H2 reference and the pre-registered H1–H5. |
| `test_m3_tools.py` | M3 | host | 21 unit tests of the M3 tools (load gate, busy-core arithmetic, window cut, separation and growth rules, stale-event window, thermal guard incl. the ABORT_01 spike). |
| `../f844/test_f844_governor_sigma_snapshot.py` | F-8-44 (C-075) | container (`source install/setup.bash`, no `-I`) | Isolated, executor-free tests T1–T7 of the governor-σ / snapshot pairing on the real node and builder with the E3-05 parameter files (`F844_PARAMS`), results to `F844_OUT`. T6 is an `expectedFailure` specification that P2 should make pass. Never modifies production code. |

Reports: [S6B1_CLEARANCE_AUDIT.md](../../validation/00_baseline/S6B1_CLEARANCE_AUDIT.md),
[QP_PERFORMANCE_ROOT_CAUSE.md](../../validation/00_baseline/QP_PERFORMANCE_ROOT_CAUSE.md),
[R1_EXECUTOR_CONFIGURATION.md](../../refactoring/R1_EXECUTOR_CONFIGURATION.md).

The `r1_*` outputs and sealed copies of the scripts are in `~/franka_baseline_evidence/refactor_r1_executor/`
(`03_replay/`, `04_scripts/`), not in the baseline evidence tree. The `m1_*` protocol, sealed scripts
(`scripts/`, and `scripts_a01/` per Addendum 01), runs and analysis are in
`~/franka_baseline_evidence/refactor_r1_m1/`; report:
[R1_M1_EXECUTOR_THREAD_REPLAY.md](../../refactoring/R1_M1_EXECUTOR_THREAD_REPLAY.md).

The `qpperf_*` scripts never modify the node: they wrap its methods at class level before
construction, and every executor or switch-interval variant applies only inside the benchmark
process. They run on an isolated DDS domain (`ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1`). Outputs and
sealed copies of the scripts: `~/franka_baseline_evidence/baseline_f708991/08_qp_performance/`. The
sealed copy of `qpperf_gil_probe.py` predates two comment-only corrections (the large QP is "~1000x", not "~100x", the C work; the small QP is
slightly smaller than the filter's, not equal to it).

The `m3_*` protocol (with Addendum 01), sealed scripts (`scripts/`, `scripts_a01/`), attempts, analysis and the F-8-44 outputs (`02_f844/`) are in `~/franka_baseline_evidence/refactor_r1_m3/`; report: [R1_M3_HOST_LOAD_AND_F844.md](../../refactoring/R1_M3_HOST_LOAD_AND_F844.md).
