# Stage 7 verification scripts (chapters 14, 15, 16; appendices A, B; documentation consistency)

Read-only scripts behind the Stage 7 synthesis. They are of three kinds, and the distinction matters
when a result changes:

| Kind | Scripts | A PASS means | A FAIL means |
|---|---|---|---|
| **Defect reproduction** (written to PASS while the defect is present) | `check_unread_functions.py` checks 1–4 (F-7-1), 5 (F-7-3); `check_install_provenance.py` check 2 (F-6-3/F-7-4); `check_guide_vs_code.py` E7 (F-7-7) | the documented defect is still there | **the defect changed** (fixed, or worse): re-assess the finding, mark it superseded if fixed |
| **Correct-behaviour checks** | `check_unread_functions.py` checks 6–9 (slack weight, joint-limit margin, segment distance, retreat displacement) | the production function behaves as ch. 08 states | a behaviour change: re-check ch. 08 |
| **Static evidence and documentation** | `inventory.py`, `test_traceability.py` (refresh `results/`); `check_guide_vs_code.py` (the old guide against the baseline); `check_docs.py` (documentation consistency); `gen_file_index.py`, `gen_equation_index.py` (print appendices A and B) | the evidence was collected / the documentation is consistent | read the message: a documentation edit is needed, never a code edit |

Rules (same as Stages 2–6, PROGRESS §5 decisions):
- the scripts import the **repository source** or read `git show ba68077:<path>`, never an install
  (`check_unread_functions.py` refuses an installed `franka_experiments`, F-2-2);
- they write nothing into the repository except `inventory.py` and `test_traceability.py`, which
  refresh their own `results/` files; bytecode caching is disabled (`-B`);
- the generators print to stdout; writing `A_file_index.md` / `B_equation_index.md` is an explicit
  redirect by the person running them;
- vocabulary: `NUM` (identity verified numerically), `REPRO` (a reported number or defect reproduced),
  `FACT` (a code, configuration or documentation fact evaluated), `INFO`.

**Nothing here starts ROS, Gazebo or the robot, runs pytest, or builds.** `check_install_provenance.py`
only streams the container's install tree out with `docker exec <c> tar -cf - …` and compares bytes
with `git cat-file` blobs.

## Scripts

| Script | Where | What | Recorded result (2026-11-11) |
|---|---|---|---|
| `inventory.py` | host | static evidence per ledger file: definitions, imports (static and string), entry points, launch references, test importers, chapter citations → `results/inventory.{json,md}` | 346 files, 283 Python, 32 entry points, 20 launch files |
| `test_traceability.py` | host | 128 test files: test counts, infrastructure tags, imports, stale imports, replica candidates → `results/test_traceability.{json,md}` | 1487 test functions, 0 stale imports |
| `check_unread_functions.py` | host | production functions first reviewed in Stage 7 (F-7-1, F-7-3, ch. 08 (8.10)) | 9/9 |
| `check_install_provenance.py` | host + `docker exec` (read-only) | which revision the container's install tree is a build of (F-7-4) | 3/3 (install ≠ baseline; mostly `42df80e`) |
| `check_guide_vs_code.py` | host | the old technical guide against the baseline (ch. 16 Part E) | 21/21 |
| `check_docs.py` | host | 11 documentation-consistency checks (findings ↔ ch. 16, IDs, ch. 15 coverage, equation references, "Consequences" sections, links, appendices current, verification directories, resume card, superseded statements) | 14/14 |
| `gen_file_index.py` | host | prints appendix A from the ledger, the inventory and ch. 15 | — (generator) |
| `gen_equation_index.py` | host | prints appendix B from the numbered equations of ch. 04–13 | — (generator) |

## How to run

```bash
docs/framework_study/tools/stage7_verification/run_all.sh             # host + container probe
docs/framework_study/tools/stage7_verification/run_all.sh --host-only
```

Exit codes: 0 every script that ran passed; 1 at least one failed; 2 the container probe was skipped.

To regenerate the appendices after a ledger or chapter change:

```bash
python3 -I -B docs/framework_study/tools/stage7_verification/gen_file_index.py > docs/framework_study/A_file_index.md
python3 -I -B docs/framework_study/tools/stage7_verification/gen_equation_index.py > docs/framework_study/B_equation_index.md
```

## Dependencies

Host: Python 3 with numpy and PyYAML (`check_unread_functions.py` imports the pure-numpy production
modules from the repository); `git`. Container probe: Docker and the `franka_ros2` container running.

## Observed results

`results/observed_2026-11-11.txt` is the full output of `run_all.sh` on 2026-11-11 (C-051).

## Unverified assumptions

- `check_guide_vs_code.py` E7 derives that `test_rate_scaling.py::test_the_shipped_config_states_durations`
  fails from its assertion and the shipped YAML; the test was **not run**.
- `check_install_provenance.py` describes the container on the day it runs; the install can change
  under it (any `colcon build`). Its check 2 is expected to FAIL after an approved clean rebuild.
- `test_traceability.py`'s infrastructure tags and replica detection are regex heuristics; the
  replicas named in ch. 14 §7.4 were confirmed by reading, the others are candidates.
- Appendix B's code pointers are where the chapter points, not a call graph; the `MANUAL` table in
  `gen_equation_index.py` was checked symbol by symbol with `git grep` at the baseline.
