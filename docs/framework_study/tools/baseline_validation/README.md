# Baseline validation analysis tools

Offline, read-only analysis scripts for the verified-baseline phase (S6B and later). They never
modify a recording, a production file or the installed workspace. Large outputs go to
`~/franka_baseline_evidence/baseline_f708991/`, never into the repository.

| Script | Step | Runs in | What it does |
|---|---|---|---|
| `s6b1_reconstruct_clearance.py` | S6B.1 | `franka_ros2` container (rosbag2_py, pinocchio, installed packages) | Recomputes the sphere clearance of every control point from the **measured** joint states in the cell bag, using `simexp.truth_geometry` (the ground-truth node's own code) and the recorded obstacle poses; compares it sample by sample with `truth.npz`; lists the truth holes and the bag samples inside them; reports per-pass minima |
| `s6b1_supplementary.py` | S6B.1 | host or container (numpy) | Disagreements and their cause, a Lipschitz bound between samples, the truth node's pose-pairing age, and the reconstruction-only spans |

Report: [S6B1_CLEARANCE_AUDIT.md](../../validation/00_baseline/S6B1_CLEARANCE_AUDIT.md).
