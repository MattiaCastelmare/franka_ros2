# Resume prompt

Paste the block below into a new Claude Code session that was started at the repository root
(`franka_ros2/`). The session can use any account; nothing depends on its conversation history or its
memory.

```text
Resume the framework documentation project in this repository. Do not do any new analysis yet.

1. Read CLAUDE.md at the repository root and follow its rules.
2. Read docs/framework_study/PROGRESS.md (§0 resume card first, then §1–§8), the newest entry of
   docs/framework_study/CHECKPOINTS.md, and docs/framework_study/00_INDEX.md.
3. Inspect docs/framework_study/ledger/coverage_ledger.md. Give the status totals and the files
   that belong to the next stage's chapters.
4. Check the Git state: run `git branch --show-current`, `git rev-parse HEAD`,
   `git status --porcelain` and `git stash list`, then
   `python3 -I docs/framework_study/tools/check_state.py`. Report the documented source
   baseline (PROGRESS.md §0/§8) separately from the current HEAD. Documentation-only commits
   are expected. If the check exits with code 2 (REVIEW), list the changed source files and the
   chapters they affect. If it exits with code 1, report the problem.
5. Reconstruct and report:
   - the current status;
   - the last completed stage;
   - any stage in progress, and how far it got;
   - open questions that need me;
   - incomplete work;
   - the exact next pending task.
6. STOP and wait for my explicit approval before reading more source code, writing documentation,
   or changing any file.

Do not modify source, launch, configuration or test files. Do not commit. Do not switch branches.
Do not touch stashes.
```
