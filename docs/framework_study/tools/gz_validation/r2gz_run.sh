#!/bin/bash
# R2 Gazebo static validation (C-078). Copy of the sealed refactor_r1_e3/scripts_h2/e3_run.sh with ONLY:
#   * arms: default = NO --set (the shipped default reaches the stack: R2 => single), and
#           multi   = --set executor_mode=multi (explicit MT(3));
#   * evidence root refactor_r2/05_gz_static, container root /home/user/r2gz_20261120, branch
#     refactor/r2-single-executor-default, resolver r2gz_resolve.py, required r2gz_* scripts.
# Everything else (preflight, monitors, H2 phase start, postflight, copy verification) is unchanged.
#
# Original header (scripts_h2/e3_run.sh):
# E3 — closed-loop Gazebo A/B of the CBF filter executor (R1). ONE cell per call:
#   pentagon_static_baseline_bare / full / --perception true / seed <0|1|2> /
#   --set executor_mode=<multi|single>   (explicit in EVERY run, both arms)
#
# scripts_h2 VERSION (H2, 2026-11-16): identical to scripts/e3_run.sh (E3-01..04) except
#   * every run uses the phase-aligned obstacle birth: --episode-start phase --start-phase 0.0
#     --start-timeout-s 90 (episode_supervisor start_mode:=phase); the cell name is unchanged;
#   * the expected HEAD is given explicitly (E3_EXPECT_HEAD): H2 must be COMMITTED and BUILT first;
#   * the committed==installed check also covers the H2 files and the read-only production files
#     the start logic and the analysis rely on (commander, path and timing laws, perception);
#   * scripts come from scripts_h2/ (scripts/ is left as it was: it is the record of E3-01..04).
# Monitors as in S6B (s6b_run.sh): host /proc sampler at 4 Hz on the container's CURRENT cgroup,
# and a wall-clock `ros2 bag record` of /NS_1/cbf_status and /cbf/per_link_distances.
# Unpinned (no --filter-cpu); the CPU governor is recorded, never changed.
#
# Usage:
#   E3_EXPECT_HEAD=<40-hex sha> e3_run.sh <NN> <seed> <multi|single>   run one cell; evidence in runs/<NN>_s<seed>_<mode>/
#   E3_EXPECT_HEAD=<sha> e3_run.sh --preflight-only <seed> <multi|single>  every preflight check; nothing is launched, the
#                                                     container is not written, no run directory is made
#                                                     (output goes to a mktemp dir under $TMPDIR)
#
# Outcome: <evidence>/RUN_STATUS
#   INCOMPLETE                   while running (and if the host dies mid-run)
#   ABORTED at stage '<s>': ...  any failed check, signal, runner error or postflight problem
#   COMPLETE_PENDING_ANALYSIS    ONLY when the runner exited 0, the monitors were stopped cleanly, the
#                                run was copied out and verified, and every postflight check passed.
# This script never judges the experiment: e3_analyze.sh applies the pilot stop rules (the gate).
# Exit codes: 0 complete (pending analysis) / preflight OK | 1 aborted | 2 usage
#
# Safety of earlier evidence: the run directory is created with a plain mkdir (fails if it exists),
# `set -C` makes every '>' refuse to overwrite, NN may not be reused, the same seed+mode needs
# E3_ALLOW_REPEAT=1, and a lock prevents two concurrent runs.

set -u -o pipefail
set -C

usage() { echo "usage: E3_EXPECT_HEAD=<sha> r2gz_run.sh [--preflight-only] <NN> <seed 0|1|2> <default|multi>  (--preflight-only takes <seed> <arm>)" >&2; exit 2; }

PREFLIGHT_ONLY=0
if [ "${1:-}" = "--preflight-only" ]; then PREFLIGHT_ONLY=1; NN=00; SEED=${2:-}; MODE=${3:-}
else NN=${1:-}; SEED=${2:-}; MODE=${3:-}; fi
[[ $NN =~ ^[0-9]{2}$ ]] || usage
case $SEED in 0|1|2) ;; *) usage;; esac
case $MODE in default|multi) ;; *) usage;; esac
ARM=$MODE                                   # the arm label: default | multi
case $ARM in default) EXPMODE=single; SETARG='';; multi) EXPMODE=multi; SETARG='--set executor_mode=multi';; esac

# ------------------------------------------------------------------ constants
CONTAINER=franka_ros2
EXPECT_HEAD=${E3_EXPECT_HEAD:-}
[[ $EXPECT_HEAD =~ ^[0-9a-f]{40}$ ]] || { echo "e3_run: set E3_EXPECT_HEAD to the full 40-hex commit of H2" >&2; exit 2; }
START_ARGS='--episode-start phase --start-phase 0.0 --start-timeout-s 90'
EXPECT_BRANCH=refactor/r2-single-executor-default
REPO=$HOME/Git/franka_ros2
ROOT=$HOME/franka_baseline_evidence/refactor_r2/05_gz_static
S=$ROOT/scripts
DOMAIN=77                                   # simexp.runner.run_cell default; batch.run_one keeps it
SCEN=pentagon_static_baseline_bare
ID=${NN}_s${SEED}_${MODE}
case $ARM in default) CELL=${SCEN}__full__gz__s${SEED};; multi) CELL=${SCEN}__full-executor_mode=multi__gz__s${SEED};; esac
CROOT=/home/user/r2gz_20261120
RUN=$CROOT/$ID
INSTALL=/ros2_ws/install
CHECK_INSTALLED=/ros2_ws/src/docs/framework_study/tools/stage6_verification/check_installed.py
ORPH='ign gazebo|gz sim|gzserver|parameter_bridge|ros2 launch|ros2 bag|ros2-daemon|run_gz_scenarios|cbf_safety_filter|real_time_distance|s6b_proc_sampler'
OUTER_TIMEOUT_S=900                         # the runner's own per-cell timeout is 400 s
REQUIRED_SCRIPTS="s6b_proc_sampler.py r2gz_resolve.py s6b_analyze.py s6b_host.py s6b1_reconstruct_clearance.py s6b1_supplementary.py r2gz_analyze.sh e3_h2_bag.py e3_h2_metrics.py e3_h2_compare.py PREREGISTRATION.md r2gz_extra.py r2gz_pair_check.py"
# committed source (git show HEAD:<path>) == installed file, checked explicitly on top of check_installed
INSTALL_PAIRS="franka_simulation_experiments/config/conditions.yaml|$INSTALL/franka_simulation_experiments/share/franka_simulation_experiments/config/conditions.yaml
franka_simulation_experiments/config/scenarios.yaml|$INSTALL/franka_simulation_experiments/share/franka_simulation_experiments/config/scenarios.yaml
franka_simulation_experiments/launch/gz_cbf_stack.launch.py|$INSTALL/franka_simulation_experiments/share/franka_simulation_experiments/launch/gz_cbf_stack.launch.py
franka_simulation_experiments/scripts/simexp/conditions.py|$INSTALL/franka_simulation_experiments/lib/franka_simulation_experiments/simexp/conditions.py
franka_experiments/config/fr3_control.yaml|$INSTALL/franka_experiments/share/franka_experiments/config/fr3_control.yaml
franka_experiments/franka_experiments/nodes/cbf_safety_filter.py|$INSTALL/franka_experiments/lib/python3.10/site-packages/franka_experiments/nodes/cbf_safety_filter.py
franka_experiments/franka_experiments/utils/config.py|$INSTALL/franka_experiments/lib/python3.10/site-packages/franka_experiments/utils/config.py
franka_experiments/launch/torque_control_stack.launch.py|$INSTALL/franka_experiments/share/franka_experiments/launch/torque_control_stack.launch.py
franka_simulation_experiments/scripts/episode_supervisor.py|$INSTALL/franka_simulation_experiments/lib/franka_simulation_experiments/episode_supervisor
franka_simulation_experiments/scripts/run_gz_scenarios.py|$INSTALL/franka_simulation_experiments/lib/franka_simulation_experiments/run_gz_scenarios
franka_simulation_experiments/scripts/simexp/batch.py|$INSTALL/franka_simulation_experiments/lib/franka_simulation_experiments/simexp/batch.py
franka_simulation_experiments/scripts/simexp/phase_start.py|$INSTALL/franka_simulation_experiments/lib/franka_simulation_experiments/simexp/phase_start.py
franka_experiments/franka_experiments/utils/trajectory.py|$INSTALL/franka_experiments/lib/python3.10/site-packages/franka_experiments/utils/trajectory.py
franka_experiments/franka_experiments/utils/timing_law.py|$INSTALL/franka_experiments/lib/python3.10/site-packages/franka_experiments/utils/timing_law.py
franka_experiments/franka_experiments/nodes/pentagon_qddot_commander.py|$INSTALL/franka_experiments/lib/python3.10/site-packages/franka_experiments/nodes/pentagon_qddot_commander.py
franka_experiments/franka_experiments/nodes/real_time_distance.py|$INSTALL/franka_experiments/lib/python3.10/site-packages/franka_experiments/nodes/real_time_distance.py
franka_experiments/franka_experiments/utils/mask_builder.py|$INSTALL/franka_experiments/lib/python3.10/site-packages/franka_experiments/utils/mask_builder.py
franka_experiments/franka_experiments/utils/distance_engine.py|$INSTALL/franka_experiments/lib/python3.10/site-packages/franka_experiments/utils/distance_engine.py"

# ------------------------------------------------------------------ evidence dir and lock
mkdir -p "$ROOT/runs" || exit 1
exec 9>>"$ROOT/.e3_lock" || exit 1
flock -n 9 || { echo "e3_run: another e3_run.sh/e3_analyze.sh holds $ROOT/.e3_lock" >&2; exit 1; }
if [ "$PREFLIGHT_ONLY" = 1 ]; then
  D=$(mktemp -d "${TMPDIR:-/tmp}/e3_preflight_s${SEED}_${MODE}_XXXXXX") || exit 1
else
  if compgen -G "$ROOT/runs/${NN}_*" >/dev/null; then
    echo "e3_run: run number $NN already used: $(ls -d "$ROOT/runs/${NN}_"*)" >&2; exit 1; fi
  if compgen -G "$ROOT/runs/*_s${SEED}_${MODE}" >/dev/null && [ "${E3_ALLOW_REPEAT:-0}" != 1 ]; then
    echo "e3_run: seed $SEED / $MODE already recorded ($(ls -d "$ROOT/runs/"*_s${SEED}_${MODE})); set E3_ALLOW_REPEAT=1 to repeat it deliberately" >&2
    exit 1; fi
  D=$ROOT/runs/$ID
  mkdir "$D" || { echo "e3_run: cannot create $D (exists?)" >&2; exit 1; }
fi
mkdir "$D/preflight" "$D/postflight" "$D/monitor" "$D/run" "$D/run/container_tmp" || exit 1

# ------------------------------------------------------------------ state, helpers, cleanup
STATE=start; ABORT=''; FINAL=0; SAMPLER_PID=''; TAIL_PID=''; WALLBAG_UP=0; RUNNER_UP=0
PROBLEMS=()
status() { printf '%s\n' "$*" >| "$D/RUN_STATUS"; }
log()    { printf '[%s] %s\n' "$(date -u +%T.%N)" "$*" | tee -a "$D/monitor/run_timeline.txt" >&2; }
fail()   { ABORT="$*"; log "ABORT: $*"; exit 1; }
problem(){ PROBLEMS+=("$*"); log "PROBLEM: $*"; }
dx()     { docker exec "$CONTAINER" "$@"; }
dxs()    { docker exec -w /tmp -e PYTHONDONTWRITEBYTECODE=1 "$CONTAINER" bash -c \
             "source $INSTALL/setup.bash >/dev/null 2>&1; $1"; }

runner_pids() { dx pgrep -f -- "--batch-dir $RUN/batch" 2>/dev/null | tr '\n' ' '; }

stop_runner() {          # SIGINT first: run_cell's finally tears the launch group down
  local p i; p=$(runner_pids); [ -n "${p// /}" ] || return 0
  log "stopping the in-container runner with SIGINT: $p"
  dx bash -c "kill -INT $p" 2>/dev/null
  for i in $(seq 1 90); do [ -z "$(runner_pids | tr -d ' ')" ] && return 0; sleep 1; done
  p=$(runner_pids); log "runner still alive after 90 s, SIGTERM: $p"; dx bash -c "kill -TERM $p" 2>/dev/null; sleep 10
  [ -z "$(runner_pids | tr -d ' ')" ] && return 0
  log "RUNNER STILL ALIVE ($(runner_pids)): manual cleanup required"; return 1
}

stop_wallbag() {
  [ "$WALLBAG_UP" = 1 ] || return 0
  local wp i; wp=$(dx cat "$RUN/wallbag.pid" 2>/dev/null) || { log "wall recorder pid file missing"; WALLBAG_UP=0; return 1; }
  dx bash -c "kill -INT $wp" 2>/dev/null
  for i in $(seq 1 30); do dx bash -c "kill -0 $wp" 2>/dev/null || { WALLBAG_UP=0; log "wall recorder stopped"; return 0; }; sleep 0.5; done
  log "wall recorder ignored SIGINT for 15 s: SIGTERM (its bag may lack metadata)"
  dx bash -c "kill -TERM $wp" 2>/dev/null; sleep 2; WALLBAG_UP=0; return 1
}

stop_sampler() {
  [ -n "$SAMPLER_PID" ] || return 0
  local i; touch "$D/monitor/sampler.stop"
  for i in $(seq 1 20); do kill -0 "$SAMPLER_PID" 2>/dev/null || break; sleep 0.5; done
  if kill -0 "$SAMPLER_PID" 2>/dev/null; then log "sampler ignored its stop file: SIGTERM"; kill "$SAMPLER_PID"; fi
  wait "$SAMPLER_PID" 2>/dev/null; SAMPLER_PID=''; log "sampler stopped"
}

on_exit() {
  local rc=$?
  trap - EXIT; trap '' INT TERM HUP
  [ -n "$TAIL_PID" ] && kill "$TAIL_PID" 2>/dev/null
  if [ "$PREFLIGHT_ONLY" = 1 ]; then
    if [ "$FINAL" = 1 ]; then echo "E3 PREFLIGHT OK (seed $SEED, $MODE): $D"; else
      echo "E3 PREFLIGHT FAILED at stage '$STATE': ${ABORT:-unexpected exit rc=$rc} — see $D" >&2; [ "$rc" = 0 ] && rc=1; fi
    exit "$rc"
  fi
  stop_runner || ABORT="${ABORT:+$ABORT; }runner processes could NOT be stopped: manual cleanup required"
  stop_wallbag || ABORT="${ABORT:+$ABORT; }wall recorder did not stop cleanly"
  stop_sampler
  if [ "$FINAL" != 1 ]; then
    status "ABORTED at stage '$STATE': ${ABORT:-unexpected exit (rc=$rc)}"
    { date -u +%FT%TZ; ps -eo pid,args | grep -E "$ORPH" | grep -v grep; } >> "$D/postflight/processes_at_abort.txt" 2>&1
    echo "E3 RUN $ID ABORTED — $(cat "$D/RUN_STATUS")" >&2
    [ "$rc" = 0 ] && rc=1
  fi
  exit "$rc"
}
trap on_exit EXIT
trap 'ABORT="signal received"; exit 130' INT TERM HUP
[ "$PREFLIGHT_ONLY" = 1 ] || status INCOMPLETE

# ------------------------------------------------------------------ preflight: host
STATE=preflight-host
log "E3 $ID: cell $CELL"
cd "$REPO" || fail "repository $REPO missing"
{ git rev-parse HEAD; git branch --show-current; } > "$D/preflight/HEAD.txt"
git status --porcelain --untracked-files=all > "$D/preflight/git_status_before.txt" || fail "git status"
git stash list > "$D/preflight/git_stash_before.txt" || fail "git stash list"
git diff HEAD --binary > "$D/preflight/working_tree.patch" || fail "git diff"          # F-6-11
[ "$(git rev-parse HEAD)" = "$EXPECT_HEAD" ] || fail "HEAD is $(git rev-parse HEAD), expected $EXPECT_HEAD"
[ "$(git branch --show-current)" = "$EXPECT_BRANCH" ] || fail "branch is $(git branch --show-current)"
[ -s "$D/preflight/git_status_before.txt" ] && fail "working tree not clean (see preflight/git_status_before.txt)"
[ "$(wc -l < "$D/preflight/git_stash_before.txt")" = 3 ] || fail "expected 3 stashes"
ps -eo pid,args > "$D/preflight/host_ps_before.txt"
grep -E "$ORPH" "$D/preflight/host_ps_before.txt" | grep -v grep > "$D/preflight/orphans_before.txt"
[ -s "$D/preflight/orphans_before.txt" ] && fail "processes from an earlier run (preflight/orphans_before.txt)"
{ date -u +%FT%TZ; uptime; echo "governor:"; sort /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | uniq -c
  free -m; nproc; } > "$D/preflight/host_resources.txt"
for f in $REQUIRED_SCRIPTS; do [ -r "$S/$f" ] || fail "required script missing: $S/$f"; done
(cd "$S" && sha256sum $REQUIRED_SCRIPTS r2gz_run.sh) > "$D/preflight/scripts_sha256.txt" || fail "hashing scripts"
[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null)" = true ] || fail "container $CONTAINER is not running"
CID=$(docker inspect -f '{{.Id}}' "$CONTAINER") && CPID=$(docker inspect -f '{{.State.Pid}}' "$CONTAINER") || fail "docker inspect"
CG="docker-${CID}.scope"                    # the CURRENT container cgroup, never a remembered id
grep -qF "$CG" "/proc/$CPID/cgroup" || fail "container PID $CPID is not in cgroup $CG ($(cat /proc/$CPID/cgroup))"
printf 'container_id %s\ncontainer_pid1 %s\ncgroup_substring %s\n/proc/%s/cgroup: %s\n' \
  "$CID" "$CPID" "$CG" "$CPID" "$(cat /proc/$CPID/cgroup)" > "$D/preflight/cgroup.txt"

# ------------------------------------------------------------------ preflight: container
STATE=preflight-container
dx ps -eo pid,args > "$D/preflight/container_ps_before.txt" || fail "docker exec ps"
grep -E "$ORPH" "$D/preflight/container_ps_before.txt" | grep -v grep > "$D/preflight/container_orphans_before.txt"
[ -s "$D/preflight/container_orphans_before.txt" ] && fail "processes from an earlier run in the container"
dx test ! -e "$RUN" || fail "container run dir $RUN already exists"
dxs "python3 -B $CHECK_INSTALLED" > "$D/preflight/check_installed.txt" 2>&1
grep -q 'INSTALLED == SOURCE' "$D/preflight/check_installed.txt" || fail "installed != source (preflight/check_installed.txt)"
: > "$D/preflight/committed_vs_installed.txt"
while IFS='|' read -r src inst; do
  c=$(git show "HEAD:$src" | sha256sum | cut -c1-64)
  i=$(dx sha256sum "$inst" 2>/dev/null | cut -c1-64)
  printf '%s committed=%s installed=%s %s\n' "$src" "$c" "${i:-ABSENT}" "$([ "$c" = "$i" ] && echo MATCH || echo MISMATCH)" \
    >> "$D/preflight/committed_vs_installed.txt"
  [ "$c" = "$i" ] || fail "installed $inst differs from committed $src"
done <<< "$INSTALL_PAIRS"
docker exec -i -w /tmp -e PYTHONDONTWRITEBYTECODE=1 "$CONTAINER" bash -c \
  "source $INSTALL/setup.bash >/dev/null 2>&1; python3 -B - $ARM $SEED" < "$S/r2gz_resolve.py" \
  > "$D/preflight/resolved.json" 2> "$D/preflight/resolved.stderr" || fail "r2gz_resolve failed: $(cat "$D/preflight/resolved.stderr")"
python3 -I -c 'import json,sys; r=json.load(open(sys.argv[1])); sys.exit(0 if r["cell"]==sys.argv[2] and r["condition"]["cbf_params"]["executor_mode"]==sys.argv[3] else 1)' \
  "$D/preflight/resolved.json" "$CELL" "$EXPMODE" || fail "resolved cell or executor_mode mismatch"

if [ "$PREFLIGHT_ONLY" = 1 ]; then
  STATE=preflight-done; FINAL=1
  log "preflight only: every check passed; nothing launched, nothing written in the container"
  exit 0
fi

dx mkdir -p "$CROOT" && dx mkdir "$RUN" || fail "cannot create $RUN"
dx touch "/tmp/e3_marker_$ID" || fail "cannot create the container marker"
timeout 180 docker diff "$CONTAINER" | sort > "$D/preflight/docker_diff_before.txt" || fail "docker diff (before)"
touch "$D/preflight/marker_before_launch"

# ------------------------------------------------------------------ monitors (identical in both arms)
STATE=monitors
log "sampler start (cgroup $CG)"
nohup python3 -I "$S/s6b_proc_sampler.py" "$CG" "$D/monitor/proc_samples.jsonl" "$D/monitor/sampler.stop" 4 900 \
  > "$D/monitor/sampler.log" 2>&1 &
SAMPLER_PID=$!
sleep 2
kill -0 "$SAMPLER_PID" 2>/dev/null || fail "sampler died at start (monitor/sampler.log)"
python3 -I -c 'import json,sys; L=[json.loads(x) for x in open(sys.argv[1]) if x.strip()]; sys.exit(0 if any(r.get("procs") for r in L) else 1)' \
  "$D/monitor/proc_samples.jsonl" || fail "sampler sees no container process: wrong cgroup?"
docker exec -d -e PYTHONDONTWRITEBYTECODE=1 -e ROS_DOMAIN_ID=$DOMAIN -e ROS_LOCALHOST_ONLY=1 "$CONTAINER" bash -c \
  "source $INSTALL/setup.bash >/dev/null 2>&1; echo \$\$ > $RUN/wallbag.pid; \
   exec ros2 bag record -o $RUN/wallbag /NS_1/cbf_status /cbf/per_link_distances > $RUN/wallbag.log 2>&1" \
  || fail "cannot start the wall recorder"
WALLBAG_UP=1
sleep 3
WP=$(dx cat "$RUN/wallbag.pid" 2>/dev/null) || fail "wall recorder wrote no pid"
dx bash -c "kill -0 $WP" 2>/dev/null || fail "wall recorder not running ($(dx cat $RUN/wallbag.log 2>/dev/null | tail -3))"
dx bash -c "tr '\\0' '\\n' < /proc/$WP/environ | grep -E '^(ROS_|RMW_)' | sort" > "$D/monitor/wallbag_env.txt"
grep -qx "ROS_DOMAIN_ID=$DOMAIN" "$D/monitor/wallbag_env.txt" && grep -qx 'ROS_LOCALHOST_ONLY=1' "$D/monitor/wallbag_env.txt" \
  || fail "wall recorder environment is not domain $DOMAIN / localhost-only (monitor/wallbag_env.txt)"
log "wall recorder pid $WP"

# ------------------------------------------------------------------ the cell
STATE=runner
CMD="source $INSTALL/setup.bash; ros2 run franka_simulation_experiments run_gz_scenarios --scenarios $SCEN --conditions full --seeds $SEED --perception true $SETARG $START_ARGS --batch-dir $RUN/batch"
printf '%s\n' "docker exec -w /tmp -e PYTHONDONTWRITEBYTECODE=1 -e ROS_LOCALHOST_ONLY=1 $CONTAINER bash -c '$CMD'" > "$D/command.txt"
log "runner start"
RUNNER_UP=1
timeout -s INT "$OUTER_TIMEOUT_S" docker exec -w /tmp -e PYTHONDONTWRITEBYTECODE=1 -e ROS_LOCALHOST_ONLY=1 \
  "$CONTAINER" bash -c "$CMD" > "$D/runner_console.log" 2>&1 &
RUNNER_CLIENT=$!
tail -n +1 -f --pid="$RUNNER_CLIENT" "$D/runner_console.log" >&2 &
TAIL_PID=$!
wait "$RUNNER_CLIENT"; RC=$?
RUNNER_UP=0
sleep 0.5; kill "$TAIL_PID" 2>/dev/null; TAIL_PID=''
printf '%s\n' "$RC" > "$D/runner_exit_code.txt"
log "runner exit $RC"
[ "$RC" = 124 ] && problem "outer timeout ($OUTER_TIMEOUT_S s): the runner was interrupted"
[ "$RC" = 0 ] || problem "runner exit code $RC"
stop_runner || problem "runner processes survive in the container"

# ------------------------------------------------------------------ stop monitors
STATE=stop-monitors
stop_wallbag || problem "wall recorder did not stop cleanly"
stop_sampler

# ------------------------------------------------------------------ postflight: copy out, never modify the cell
STATE=postflight
docker cp "$CONTAINER:$RUN/." "$D/run/" || problem "docker cp of $RUN failed"
[ -f "$D/run/batch/cells/$CELL/cell_result.json" ] || problem "no cell_result.json for $CELL"
# Copy verification: the SAME listing code runs in the container and on the host. One line per entry,
# "<type>\t<size>\t<sha256>\t<path>": type f|d|l|o (file, directory, symlink, other); size and SHA-256
# for regular files; a symlink records its target. Entries are sorted bytewise (LC_ALL=C), so neither
# locale nor directory order matters. find only enumerates (host find is bfs, container find is GNU).
# Any listing or hashing error is an explicit failure (exit != 0), never an empty or partial "match".
# ./container_tmp is skipped on both sides: it is filled on the host after this check.
# >>> E3_LIST
read -r -d '' E3_LIST <<'LISTEOF' || true
set -u -o pipefail
export LC_ALL=C LANG=C
cd -- "$1" || { echo "e3_list: cannot enter $1" >&2; exit 2; }
find . -path ./container_tmp -prune -o -print0 | sort -z | {
  n=0
  while IFS= read -r -d '' p; do
    if [ -L "$p" ]; then
      t=$(readlink -- "$p") || { echo "e3_list: cannot read link $p" >&2; exit 3; }
      printf 'l\t-\t-\t%s -> %s\n' "$p" "$t"
    elif [ -d "$p" ]; then
      printf 'd\t-\t-\t%s\n' "$p"
    elif [ -f "$p" ]; then
      s=$(stat -c %s -- "$p") || { echo "e3_list: cannot stat $p" >&2; exit 3; }
      h=$(sha256sum -- "$p") || { echo "e3_list: cannot hash $p" >&2; exit 3; }
      h=${h%% *}
      [[ $h =~ ^[0-9a-f]{64}$ ]] || { echo "e3_list: malformed hash for $p" >&2; exit 3; }
      printf 'f\t%s\t%s\t%s\n' "$s" "$h" "$p"
    else
      printf 'o\t-\t-\t%s\n' "$p"
    fi
    n=$((n + 1))
  done
  [ "$n" -gt 1 ] || { echo "e3_list: nothing listed under $PWD" >&2; exit 4; }
}
LISTEOF
# <<< E3_LIST
dx bash -c "$E3_LIST" e3_list "$RUN" > "$D/postflight/container_run_files.txt" 2> "$D/postflight/container_run_files.err"
RC_LC=$?
bash -c "$E3_LIST" e3_list "$D/run" > "$D/postflight/copied_run_files.txt" 2> "$D/postflight/copied_run_files.err"
RC_LH=$?
if [ "$RC_LC" != 0 ] || [ "$RC_LH" != 0 ]; then
  problem "copy verification FAILED TO RUN: listing/hashing error (container rc $RC_LC, host rc $RC_LH; postflight/*_run_files.err)"
elif ! cmp -s "$D/postflight/container_run_files.txt" "$D/postflight/copied_run_files.txt"; then
  diff "$D/postflight/container_run_files.txt" "$D/postflight/copied_run_files.txt" > "$D/postflight/run_files.diff"
  problem "the copy differs from the container run dir (paths/types/sizes/SHA-256; postflight/run_files.diff)"
fi
dx bash -c "find /tmp -maxdepth 1 -type f -newer /tmp/e3_marker_$ID; \
            find /home/user/.ignition /home/user/.ros/log -maxdepth 4 -type f -newer /tmp/e3_marker_$ID 2>/dev/null" \
  > "$D/postflight/container_files_written.txt"
while read -r f; do
  [ -n "$f" ] || continue
  docker cp "$CONTAINER:$f" "$D/run/container_tmp/$(printf '%s' "${f#/}" | sed 's#/#__#g')" || problem "cannot copy $f"
done < "$D/postflight/container_files_written.txt"
[ "$(grep -l 'executor_mode:' "$D"/run/container_tmp/tmp__launch_params_* 2>/dev/null | wc -l)" = 1 ] \
  || problem "expected exactly one launch-params file carrying executor_mode"
# wall bag message counts from its own metadata.yaml (no ros2 CLI: it can leave a ros2 daemon behind)
python3 -I -c 'import sys, yaml
m = yaml.safe_load(open(sys.argv[1]))["rosbag2_bagfile_information"]
c = {t["topic_metadata"]["name"]: t["message_count"] for t in m["topics_with_message_count"]}
print(c); sys.exit(0 if all(c.get(t, 0) > 0 for t in ("/NS_1/cbf_status", "/cbf/per_link_distances")) else 1)' \
  "$D/run/wallbag/metadata.yaml" > "$D/postflight/wallbag_counts.txt" 2>&1 \
  || problem "wall bag missing, without metadata, or without messages on a topic (postflight/wallbag_counts.txt)"
timeout 180 docker diff "$CONTAINER" | sort > "$D/postflight/docker_diff_after.txt" || problem "docker diff (after)"
comm -13 "$D/preflight/docker_diff_before.txt" "$D/postflight/docker_diff_after.txt" > "$D/postflight/docker_diff_delta.txt"
comm -23 "$D/preflight/docker_diff_before.txt" "$D/postflight/docker_diff_after.txt" > "$D/postflight/docker_diff_removed.txt"
dx ps -eo pid,args > "$D/postflight/container_ps_after.txt"
ps -eo pid,args > "$D/postflight/host_ps_after.txt"
grep -E "$ORPH" "$D/postflight/host_ps_after.txt" | grep -v grep > "$D/postflight/leftover_processes.txt"
[ -s "$D/postflight/leftover_processes.txt" ] && problem "leftover processes (postflight/leftover_processes.txt)"
cd "$REPO" || problem "repository vanished"
{ git rev-parse HEAD; git branch --show-current; } > "$D/postflight/HEAD.txt"
git status --porcelain --untracked-files=all > "$D/postflight/git_status_after.txt"
git stash list > "$D/postflight/git_stash_after.txt"
cmp -s "$D/preflight/HEAD.txt" "$D/postflight/HEAD.txt" || problem "HEAD or branch changed during the run"
cmp -s "$D/preflight/git_status_before.txt" "$D/postflight/git_status_after.txt" || problem "git status changed"
cmp -s "$D/preflight/git_stash_before.txt" "$D/postflight/git_stash_after.txt" || problem "stashes changed"
find . -newer "$D/preflight/marker_before_launch" -not -path './.git' -not -path './.git/*' > "$D/postflight/repo_files_newer_than_marker.txt"
[ -s "$D/postflight/repo_files_newer_than_marker.txt" ] && problem "repository files written during the run"
dxs "python3 -B $CHECK_INSTALLED" > "$D/postflight/check_installed.txt" 2>&1
grep -q 'INSTALLED == SOURCE' "$D/postflight/check_installed.txt" || problem "installed != source after the run"

((${#PROBLEMS[@]})) && fail "postflight: ${PROBLEMS[*]}"
FINAL=1
status COMPLETE_PENDING_ANALYSIS

# ------------------------------------------------------------------ quick facts (NOT a verdict)
echo "=== E3 $ID recorded; NOT a verdict: run e3_analyze.sh $ID (the pilot gate) ==="
grep -hoE 'executor_mode=[a-z]+ \([A-Za-z]+\)' "$D/run/batch/cells/$CELL/launch.log" | sort | uniq -c
python3 -I -c 'import json,sys; r=json.load(open(sys.argv[1])); c=r.get("closed_loop",{})
print({k: r.get(k) for k in ("ok","timed_out","leftovers","returncode","wall_s")})
print({k: c.get(k) for k in ("qp_rate_hz","qp_rate_ok","min_gap","contact","rtf","reflex_verdict","fault_ticks","fallback_levels")})' \
  "$D/run/batch/cells/$CELL/cell_result.json"
python3 -I -c 'import json,sys; e=json.load(open(sys.argv[1]))["episode"]; s=e.get("start") or {}
print("H2 start:", s.get("status"), s.get("fail_reason") or "", "t_start", e.get("t_start_sim"),
      "phase error", (s.get("placement") or {}).get("phase_error_cycles"))' "$D/run/batch/cells/$CELL/run_manifest_sim.json"
exit 0
