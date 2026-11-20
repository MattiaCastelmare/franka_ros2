#!/bin/bash
# R2: the twelve runs in the sealed order (protocol refactor_r2/04_perf/protocol/R2_PERF_PROTOCOL.md §6),
# under an experiment-scoped sleep inhibitor. Derived from m3b_batch.sh (M3b, sealed) with exactly these
# changes: the run script is r2_run.sh (arm default) with the rate per run; the inhibitor is R2_BATCH; the
# order below (four conditions x 3 runs, interleaved). Unchanged: the batch re-executes itself under
#   systemd-inhibit --what=sleep:idle:handle-lid-switch --mode=block --who=R2_BATCH
# (released with the process, nothing system-wide is changed); pause after a run: 60 s after L1, 15 s
# after L0; exit 3 (abort) or 5 (harness failure) stops the batch; exit 4 (a validity gate failed) is
# recorded and the batch continues. No retry, no replacement run.
# usage: r2_batch.sh <evidence_dir>
set -u
if [ "${R2_INHIBITED:-0}" != 1 ]; then
  exec systemd-inhibit --what=sleep:idle:handle-lid-switch --mode=block --who=R2_BATCH \
       --why="R2 replay batch (refactor_r2)" env R2_INHIBITED=1 "$0" "$@"
fi
EV=$1
HERE=$(cd "$(dirname "$0")" && pwd)
ORDER="r2_01:0.70:L0 r2_02:1.0:L1 r2_03:1.0:L0 r2_04:0.70:L1 r2_05:1.0:L0 r2_06:0.70:L1 r2_07:0.70:L0 r2_08:1.0:L1 r2_09:0.70:L1 r2_10:1.0:L1 r2_11:0.70:L0 r2_12:1.0:L0"
mkdir -p "$EV/logs"
B="$EV/logs/batch.txt"
echo "batch start $(date -u +%FT%T.%NZ) inhibitor $(systemd-inhibit --list --no-pager | grep -c R2_BATCH)" >> "$B"
for item in $ORDER; do
  IFS=: read -r ID RATE LOAD <<< "$item"
  echo "== $ID default $LOAD $RATE $(date -u +%FT%T.%NZ)" | tee -a "$B"
  "$HERE/r2_run.sh" "$EV" "$ID" "$RATE" "$LOAD" 2>&1 | tee -a "$B"
  RC=${PIPESTATUS[0]}
  echo "rc $ID $RC" >> "$B"
  case $RC in
    0|4) ;;
    *) echo "batch STOPPED at $ID (rc $RC) $(date -u +%FT%T.%NZ)" | tee -a "$B"; exit $RC;;
  esac
  if [ "$LOAD" = L1 ]; then sleep 60; else sleep 15; fi
done
echo "batch end $(date -u +%FT%T.%NZ)" | tee -a "$B"
