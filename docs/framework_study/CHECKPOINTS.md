# Checkpoint log (append-only)

This is the session history of the documentation project. Rules:
- Append a new entry at the **bottom**, and never edit or delete old entries. If an old entry was
  wrong, the new entry says so.
- After appending, overwrite the resume card in [PROGRESS.md](PROGRESS.md) §0 so that it agrees with
  the newest entry.
- See PROGRESS.md §6 for when to checkpoint.

Template:

```
## C-NNN — YYYY-MM-DD HH:MM — Stage N — <short title>
- Baseline / HEAD / check: <baseline sha12> / <HEAD sha12> / check_state exit <0|1|2> (<REVIEW paths, if any>)
1. Analysed: <subsystems, sections written>
2. Remaining in this stage: <list>
3. Files read (ledger status set): <path → status, …>, or "none"
4. Documentation files updated: <list>
5. Findings / uncertainties: <F-… IDs added; Q-… IDs added or closed; doubts not yet logged>
6. Exact next action: <file + section to write, first source file/function to read>
```

---

## C-001 — 2026-11-11 — Stage 1 — Architecture, ROS graph and configuration done

This entry was reconstructed afterwards, at C-002, from PROGRESS.md and the chapter files.

- HEAD / tree: `ba68077052e8`; the tree is clean except for the untracked `docs/framework_study/`.
1. **Analysed:**
   - the four pipelines (A: accel HOCBF → torque; B: velocity; C: OSCBF; D: no filter);
   - the Gazebo harness reusing `torque_control_stack.launch.py` with `external_plant:=true`;
   - the node interfaces, topics, QoS, TF tree and rates (configured, and recorded where available);
   - the configuration precedence model and the `CBF_PARAM_SPEC` validation.
2. **Remaining in this stage:** none.
3. **Files read:** the ledger records them (5 `read`, 47 `partial`, 4 `documented`). The main ones:
   - the launch files;
   - `utils/launch_support.py`, `utils/config.py`, `utils/params.py`;
   - the node constructors;
   - the C++ controller interfaces;
   - `gz_cbf_stack.launch.py`, `gz_fr3_plant.launch.py`, `conditions.py`, `sim_config.py`.
4. **Documentation files updated:**
   - `01_architecture.md`, `02_ros_graph_and_dataflow.md`, `03_configuration.md`;
   - `PROGRESS.md`, `00_INDEX.md`, `ledger/*`;
   - the new `tools/ledger_status.py`;
   - the extractor grouping fix in `tools/stage0_extract.py`.
5. **Findings and uncertainties:** F-1-1 … F-1-18 were added, as were Q-1-1 … Q-1-5. Q-1-4 and Q-1-5
   need the user.
6. **Exact next action:** stop for the user's review of Stage 1.

## C-002 — 2026-11-11 — between Stage 1 and Stage 2 — Persistent-memory system

- HEAD / tree: `ba68077052e8`. The only untracked entries are `docs/framework_study/` and the new
  root `CLAUDE.md`. No tracked file was modified.
1. **Analysed:** no framework code. The user asked for session continuity across sessions and
   accounts.
2. **Remaining:** Stages 2–7. Stage 2 is not approved yet.
3. **Files read:** none of the framework. The ledger is unchanged (pending 290, partial 47, read 5,
   documented 4).
4. **Documentation files updated:**
   - created `/CLAUDE.md`, `CHECKPOINTS.md`, `RESUME_PROMPT.md` and `tools/check_state.py`;
   - `PROGRESS.md`: added §0 (resume card), §6 (checkpoint protocol) and §7 (leads carried over from
     account memory), plus the post-Stage-1 decisions in §5; the Stage 1 row is marked as
     acknowledged;
   - `00_INDEX.md`: status line and artefact table.
5. **Findings and uncertainties:** no new F-IDs. Q-1-4 and Q-1-5 are still open for the user.
6. **Exact next action:** wait for the user's approval of Stage 2. Then:
   1. run `python3 -I docs/framework_study/tools/check_state.py`;
   2. append C-003 "Stage 2 started";
   3. read `franka_experiments/franka_experiments/utils/kinematics.py` for `04_math_foundations.md`
      §Mathematical foundations (frames, FK, control-point Jacobians J and J̇).

## C-003 — 2026-11-11 — between Stage 1 and Stage 2 — Baseline-aware consistency check

- Baseline / HEAD / check: `ba68077052e8` / `ba68077052e8` / check_state exit 0
1. **Analysed:** no framework code. The user asked for the memory system to survive commits that
   touch only the documentation, and to detect real source changes.
2. **Remaining:** Stages 2–7. Stage 2 is not approved yet.
3. **Files read:** none of the framework. The ledger is unchanged.
4. **Documentation files updated:**
   - `tools/check_state.py` now compares the working tree with the *documented source baseline*
     instead of HEAD. It classifies changes as doc, source, interface or other, and has exit codes
     0, 1 and 2;
   - `PROGRESS.md`: §0 "Documented source baseline" replaces "Pinned HEAD"; §3 step 1; the §1
     column renamed; new §8 (the source baseline, its re-check procedure, and the history row B-1);
   - `CLAUDE.md`, `00_INDEX.md` §1, `RESUME_PROMPT.md` step 4, and the template in this file.
5. **Findings and uncertainties:** no new F-IDs. Ten scenarios were tested in a throwaway clone in the
   scratchpad, since deleted:
   - commits that touch only the documentation pass (exit 0);
   - an uncommitted, committed or untracked source change gives REVIEW (exit 2) with its chapters;
   - interface and other paths give a NOTE;
   - a baseline that is not an ancestor of HEAD, and §0 disagreeing with §8, give a PROBLEM (exit 1);
   - re-baselining after review returns exit 0.
6. **Exact next action:** unchanged from C-002. Wait for the user's approval of Stage 2.
