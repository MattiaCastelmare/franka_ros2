# CLAUDE.md

## Active project: framework study documentation (read-only phase)

**Objective.** The user is studying the FR3 safety framework from first principles before any refactor.
The deliverable is educational, code-derived Markdown documentation of two ROS 2 packages:
`franka_experiments` and `franka_simulation_experiments`. It also covers the parts of their workspace
dependencies that actually run in the pipeline: `franka_rt_controllers`, `franka_simulation`'s plant
launch, and the `franka_msgs` distance messages. Later, the same documentation serves as the reference
for what each refactoring change affects.

**Location.** Everything lives in `docs/framework_study/`. The repository files are the single source of
truth: do not rely on conversation history or on account-specific memory.

| File | Use it for |
|---|---|
| `docs/framework_study/PROGRESS.md` | **Start here.** §0 is the resume card: status, last completed stage, next stage, exact next action. Also stages (§1), coverage (§2), procedure (§3), findings (§4), decisions and open questions (§5), checkpoint protocol (§6), leads (§7) and the source baseline (§8). |
| `docs/framework_study/CHECKPOINTS.md` | Append-only session log. Read the newest entry when resuming. |
| `docs/framework_study/00_INDEX.md` | Pinned revision, evidence labels (§2), chapter template (§3), chapter plan and reading order (§4) |
| `docs/framework_study/ledger/coverage_ledger.md` | Every in-scope file with its chapter, stage and status |
| `docs/framework_study/RESUME_PROMPT.md` | The prompt the user pastes to resume in a new session |

## Resuming a session

1. Read `PROGRESS.md` (§0 first), then the newest entry of `CHECKPOINTS.md`, then `00_INDEX.md`.
2. Run `python3 -I docs/framework_study/tools/check_state.py` from the repository root. It is
   read-only.
   - The chapters describe the **documented source baseline** (`PROGRESS.md` §0 and §8), which is not
     the current HEAD. Commits that touch only the documentation do not invalidate it.
   - Exit code 0: consistent.
   - Exit code 2: REVIEW. The ROS 2 source changed since the baseline. Follow `PROGRESS.md` §8 before
     any new stage work.
   - Exit code 1: integrity problem.
3. Interpret the ledger. The status of each file is one of:
   - `pending`: not read;
   - `partial`: the parts an earlier stage needed were read (§2 of `PROGRESS.md` says which);
   - `read`: every function was read;
   - `documented`: covered by its chapter.

   Do not re-read work that is recorded as done unless HEAD has moved.
4. Report the reconstructed status and the next action, then **wait for the user's approval**.

## Rules

- **Run only explicitly approved stages, one per approval.** At the end of a stage, update the progress
  files, report, and STOP. An approval given in an earlier session does not carry over: a stage recorded
  as "in progress" resumes only after the user confirms it in the current session.
- **Keep the code read-only.** Do not modify, move or delete any source, launch, configuration or test
  file, nor `package.xml`, `setup.py`, `CMakeLists.txt` or the existing docs (READMEs,
  `FRAMEWORK_COMPLETE_TECHNICAL_GUIDE.md`, and others). Write only under `docs/framework_study/` and in
  this file. Leave other untracked files alone. Ask before running `colcon build`, `pytest` or any ROS
  launch.
  - **Exception: approved refactoring steps.** The user approves production changes one step at a
    time (R1, R2, …), each on its own branch, never on `humble-mattia`. Only the files that the
    approved step needs may change; the record is `docs/framework_study/refactoring/R<n>_*.md`.
    R1 (configurable CBF executor, branch `refactor/cbf-executor-scheduling`) was approved on
    2026-11-12. An approval covers one step only; Gazebo runs and the next step need their own.
- **Git.**
  - Do not commit unless asked.
  - Never switch branches, reset, check out files, or apply, drop or modify stashes.
  - Analyse HEAD only.
  - Commits the user requests carry only the user's authorship: no `Co-Authored-By` trailer.
- **Label every claim.** Use the labels Verified / Derived / Inferred / Unverified / Runtime (recorded)
  (definitions in `00_INDEX.md` §2). Never present an inferred or unverified claim as fact. Cite
  `file::function`.
- **Preserve findings.**
  - Log every bug, inconsistency or math-versus-code deviation in `PROGRESS.md` §4 as soon as you find
    it, with ID `F-<stage>-<n>`.
  - Never delete a finding. Mark it superseded and say why.
  - Each chapter ends with "Consequences of modification", for the later refactor.
- **Use the existing guide only as a source of leads.** `franka_experiments/docs/FRAMEWORK_COMPLETE_TECHNICAL_GUIDE.md`
  is never trusted on its own. Disagreements with the code go to chapter 16.
- **Checkpoint during the work, not only at the end of a stage.** Write a checkpoint:
  - after each significant subsystem;
  - before starting another one;
  - before ending a session;
  - before context compaction, whenever possible.

  The format is in `PROGRESS.md` §6.
- Write in English.
