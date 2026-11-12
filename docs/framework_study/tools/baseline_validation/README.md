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

Reports: [S6B1_CLEARANCE_AUDIT.md](../../validation/00_baseline/S6B1_CLEARANCE_AUDIT.md),
[QP_PERFORMANCE_ROOT_CAUSE.md](../../validation/00_baseline/QP_PERFORMANCE_ROOT_CAUSE.md),
[R1_EXECUTOR_CONFIGURATION.md](../../refactoring/R1_EXECUTOR_CONFIGURATION.md).

The `r1_*` outputs and sealed copies of the scripts are in `~/franka_baseline_evidence/refactor_r1_executor/`
(`03_replay/`, `04_scripts/`), not in the baseline evidence tree.

The `qpperf_*` scripts never modify the node: they wrap its methods at class level before
construction, and every executor or switch-interval variant applies only inside the benchmark
process. They run on an isolated DDS domain (`ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1`). Outputs and
sealed copies of the scripts: `~/franka_baseline_evidence/baseline_f708991/08_qp_performance/`. The
sealed copy of `qpperf_gil_probe.py` predates two comment-only corrections (the large QP is "~1000x", not "~100x", the C work; the small QP is
slightly smaller than the filter's, not equal to it).
