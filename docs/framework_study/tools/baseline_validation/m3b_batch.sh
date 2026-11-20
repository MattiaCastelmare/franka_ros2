#!/bin/bash
# M3b: the six runs in the sealed order (protocol refactor_r1_m3b/protocol/M3B_PROTOCOL.md §6), under an
# experiment-scoped sleep inhibitor. Derived from m3_batch.sh (M3, sealed): single arm only, rate 1.0
# (in m3b_run.sh), and the batch re-executes itself under
#   systemd-inhibit --what=sleep:idle:handle-lid-switch --mode=block --who=M3B_BATCH
# so that logind refuses suspend (idle, lid, desktop request) until the batch exits; the lock is released
# with the process, nothing system-wide is changed. Blocks: L0 L1 L1 L0 L0 L1 (as M3's block loads). Pause
# after a run: 60 s after L1, 15 s after L0. Exit 3 (abort) or 5 (harness failure) stops the batch; exit 4
# (a validity gate failed) is recorded and the batch continues. No retry, no replacement run.
# usage: m3b_batch.sh <evidence_dir>
set -u
if [ "${M3B_INHIBITED:-0}" != 1 ]; then
  exec systemd-inhibit --what=sleep:idle:handle-lid-switch --mode=block --who=M3B_BATCH \
       --why="M3b replay batch (refactor_r1_m3b)" env M3B_INHIBITED=1 "$0" "$@"
fi
EV=$1
HERE=$(cd "$(dirname "$0")" && pwd)
ORDER="m3b_01:L0 m3b_02:L1 m3b_03:L1 m3b_04:L0 m3b_05:L0 m3b_06:L1"
mkdir -p "$EV/logs"
B="$EV/logs/batch.txt"
echo "batch start $(date -u +%FT%T.%NZ) inhibitor $(systemd-inhibit --list --no-pager | grep -c M3B_BATCH)" >> "$B"
for item in $ORDER; do
  IFS=: read -r ID LOAD <<< "$item"
  echo "== $ID single $LOAD $(date -u +%FT%T.%NZ)" | tee -a "$B"
  "$HERE/m3b_run.sh" "$EV" "$ID" "$LOAD" 2>&1 | tee -a "$B"
  RC=${PIPESTATUS[0]}
  echo "rc $ID $RC" >> "$B"
  case $RC in
    0|4) ;;
    *) echo "batch STOPPED at $ID (rc $RC) $(date -u +%FT%T.%NZ)" | tee -a "$B"; exit $RC;;
  esac
  if [ "$LOAD" = L1 ]; then sleep 60; else sleep 15; fi
done
echo "batch end $(date -u +%FT%T.%NZ)" | tee -a "$B"
