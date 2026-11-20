# R2 Gazebo static validation tools (C-078)

Repository copies of the sealed scripts in
`~/franka_baseline_evidence/refactor_r2/05_gz_static/scripts/`. They are byte-identical, checked by
`cmp` at copy time. The sealed copies are the ones of record. They are used together with sealed
copies of the unchanged H2 scripts from `refactor_r1_e3/scripts_h2/`:
`e3_h2_bag.py`, `e3_h2_compare.py`, `e3_h2_metrics.py`, `PREREGISTRATION.md`, `s6b*`.

| File | Runs on | What it does |
|---|---|---|
| `r2gz_run.sh` | host (drives the container) | `e3_run.sh` with only these changes: arms `default` (no `--set`; `full` resolves to the shipped `single`) and `multi` (`--set executor_mode=multi`), the R2 branch, roots and the resolver |
| `r2gz_resolve.py` | container | Resolves the cell; requires the installed `fr3_control.yaml` `executor_mode: single`; the arm diff must be exactly the three executor keys |
| `r2gz_analyze.sh` | host | `e3_analyze.sh` (gate R01–R22, V1–V7) with the run-id pattern and arm mapping, plus the `r2gz_extra` step |
| `r2gz_extra.py` | host | Oscillation metrics (reversals per s and per command, HF power > 10 Hz, step norm, safe/nominal ratios) and the run summary |
| `r2gz_pair_check.py` | host | P1–P6 pair identity: launch parameters, cell files, manifest and SHA, filter start-up lines, commander parameters, executor lines differ |
| `r2gz_summary.py` | host | Q1–Q6 with the constants pre-registered in `R2GZ_PROTOCOL.md` §7. Known gap F-8-64: E3 references print `None` |
| `r2gz_batch.sh` | host | Order D M M D under `systemd-inhibit`; stop rules; pair compare and check; summary; no retry |

Report: [R2_GAZEBO_STATIC_AND_F841.md](../../refactoring/R2_GAZEBO_STATIC_AND_F841.md).
