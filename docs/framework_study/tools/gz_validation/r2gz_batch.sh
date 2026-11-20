#!/bin/bash
# R2 Gazebo static validation (C-078): the whole authorised campaign, in the sealed order, ONCE.
#   01 s0 default | 02 s0 multi | 03 s1 multi | 04 s1 default        (counterbalanced ABBA)
# Each run: r2gz_run.sh (record) -> r2gz_analyze.sh (H2 gate R01-R22 + extras; seals the run dir).
# After each pair: e3_h2_compare.py (COMPARABLE) and r2gz_pair_check.py (PASS).
# STOP RULES (protocol §8), checked after every step, never retried:
#   run rc != 0 (ABORTED)                    -> stop the campaign
#   analysis rc != 0 (GATE_STOP or error)    -> stop the campaign; safety rules R08/R10/R11/R12/R14 named
#   pair not COMPARABLE or pair check FAIL   -> stop the campaign
# The summary (r2gz_summary.py) runs on whatever exists when the campaign ends or stops.
# Runs under a systemd sleep/idle/lid inhibitor (the batch re-executes itself under it).
# Usage: R2GZ_EXPECT_HEAD=<40-hex> r2gz_batch.sh
set -u -o pipefail
set -C
R=$HOME/franka_baseline_evidence/refactor_r2/05_gz_static
S=$R/scripts
HEAD=${R2GZ_EXPECT_HEAD:-}
[[ $HEAD =~ ^[0-9a-f]{40}$ ]] || { echo "r2gz_batch: set R2GZ_EXPECT_HEAD" >&2; exit 2; }
if [ -z "${R2GZ_INHIBITED:-}" ]; then
  exec systemd-inhibit --what=sleep:idle:handle-lid-switch --mode=block --who=R2GZ_BATCH \
    --why="R2 Gazebo static validation" env R2GZ_INHIBITED=1 R2GZ_EXPECT_HEAD="$HEAD" "$0" "$@"
fi
export E3_EXPECT_HEAD=$HEAD
ORDER="01:0:default 02:0:multi 03:1:multi 04:1:default"
mkdir -p "$R/logs" "$R/host_env" "$R/pairs" || exit 1
LOG=$R/logs/batch.txt
[ -e "$LOG" ] && { echo "r2gz_batch: $LOG exists: the campaign has already been started (no retries)" >&2; exit 1; }
log() { printf '%s %s\n' "$(date -u +%FT%T.%NZ)" "$*" | tee -a "$LOG"; }
hostenv() {
  { date -u +%FT%TZ; echo governor:; sort /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | uniq -c
    echo epp:; sort /sys/devices/system/cpu/cpu*/cpufreq/energy_performance_preference | uniq -c
    echo "no_turbo=$(cat /sys/devices/system/cpu/intel_pstate/no_turbo 2>/dev/null)"
    echo "pkg_temp_mC=$(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -n | tail -1)"
    echo "on_ac=$(cat /sys/class/power_supply/A*/online 2>/dev/null | head -1)"
    uptime; echo "inhibitors: $(systemd-inhibit --list --no-pager | grep -c R2GZ_BATCH)"; } > "$1"
}
summary() {
  local args=() seed d m
  for seed in 0 1; do
    d=$(ls -d "$R"/runs/*_s${seed}_default 2>/dev/null | head -1); m=$(ls -d "$R"/runs/*_s${seed}_multi 2>/dev/null | head -1)
    args+=(--pair "$seed" "${d:-$R/runs/MISSING_s${seed}_default}" "${m:-$R/runs/MISSING_s${seed}_multi}"
           "$R/pairs/s${seed}/compare/compare.json" "$R/pairs/s${seed}/pair_check.json")
  done
  python3 -I "$S/r2gz_summary.py" "$R/summary" "${args[@]}" \
    --ref 0 "$HOME/franka_baseline_evidence/refactor_r1_e3/runs/06_s0_single" "$HOME/franka_baseline_evidence/refactor_r1_e3/runs/05_s0_multi" \
    --ref 1 "$HOME/franka_baseline_evidence/refactor_r1_e3/runs/08_s1_single" "$HOME/franka_baseline_evidence/refactor_r1_e3/runs/07_s1_multi" \
    > "$R/logs/summary_stdout.txt" 2>&1
  log "summary rc $?"
}
stop_campaign() { log "CAMPAIGN STOPPED: $*"; summary; log "batch end (stopped)"; exit 3; }

log "batch start HEAD $HEAD inhibitor $(systemd-inhibit --list --no-pager | grep -c R2GZ_BATCH)"
for item in $ORDER; do
  IFS=: read -r NN SEED ARM <<< "$item"; ID=${NN}_s${SEED}_${ARM}
  hostenv "$R/host_env/${ID}_before.txt"
  log "== $ID run start"
  bash "$S/r2gz_run.sh" "$NN" "$SEED" "$ARM" > "$R/logs/${ID}_run_stdout.txt" 2>&1
  RC=$?; log "$ID run rc $RC status '$(cat "$R/runs/$ID/RUN_STATUS" 2>/dev/null)'"
  hostenv "$R/host_env/${ID}_after.txt"
  [ "$RC" = 0 ] || stop_campaign "$ID run aborted (rc $RC) — validity stop, no retry"
  bash "$S/r2gz_analyze.sh" "$ID" > "$R/logs/${ID}_analyze_stdout.txt" 2>&1
  RC=$?; AS=$(cat "$R/runs/$ID/ANALYSIS_STATUS" 2>/dev/null)
  log "$ID analysis rc $RC $AS"
  if [ "$RC" != 0 ]; then
    SAFE=$(python3 -I -c 'import json,sys
g=json.load(open(sys.argv[1])); print(" ".join(f"{r[\"id\"]}={r[\"verdict\"]}" for r in g["rules"] if r["id"] in ("R08","R10","R11","R12","R14")))' \
      "$R/runs/$ID/analysis/gate.json" 2>/dev/null)
    BAD=$(python3 -I -c 'import json,sys
g=json.load(open(sys.argv[1])); print(" ".join(r["id"] for r in g["rules"] if r["verdict"] in ("STOP","MISSING","ERROR")))' \
      "$R/runs/$ID/analysis/gate.json" 2>/dev/null)
    stop_campaign "$ID gate $AS; failing rules [$BAD]; safety rules [$SAFE]"
  fi
  if [ "$NN" = 02 ] || [ "$NN" = 04 ]; then
    P=$R/pairs/s$SEED; mkdir -p "$P"
    D=$(ls -d "$R"/runs/*_s${SEED}_default); M=$(ls -d "$R"/runs/*_s${SEED}_multi)
    python3 -I "$S/e3_h2_compare.py" "$M" "$D" "$P/compare" > "$P/compare_stdout.txt" 2>&1
    CV=$(python3 -I -c 'import json,sys; print(json.load(open(sys.argv[1]))["verdict"])' "$P/compare/compare.json" 2>/dev/null)
    python3 -I "$S/r2gz_pair_check.py" "$D" "$M" "$P/pair_check.json" > "$P/pair_check_stdout.txt" 2>&1
    PV=$?
    log "pair s$SEED: compare $CV; pair check rc $PV"
    [ "$CV" = COMPARABLE ] && [ "$PV" = 0 ] || stop_campaign "pair s$SEED not comparable ($CV) or pair check failed (rc $PV)"
  fi
done
summary
log "batch end (complete)"
exit 0
