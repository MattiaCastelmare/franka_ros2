#!/bin/bash
# M3: the 18 runs in the sealed order (protocol refactor_r1_m3/protocol/M3_PROTOCOL.md §6).
# usage: m3_batch.sh <evidence_dir>
# Six blocks of three runs. Block loads L0 L1 L1 L0 L0 L1. The arm order rotates so that, within each
# load, each arm takes each position once. Pause after a run: 60 s after L1 (power budget and
# temperature recover), 15 s after L0. Exit 3 (abort) or 5 (harness failure) stops the batch; exit 4
# (a replay validity gate failed) is recorded and the batch continues. No retry, no replacement run.
set -u
EV=$1
HERE=$(cd "$(dirname "$0")" && pwd)
ORDER="m3_01:single:L0 m3_02:mt1:L0 m3_03:mt3:L0
m3_04:mt3:L1 m3_05:single:L1 m3_06:mt1:L1
m3_07:mt1:L1 m3_08:mt3:L1 m3_09:single:L1
m3_10:mt1:L0 m3_11:mt3:L0 m3_12:single:L0
m3_13:mt3:L0 m3_14:single:L0 m3_15:mt1:L0
m3_16:single:L1 m3_17:mt1:L1 m3_18:mt3:L1"
mkdir -p "$EV/logs"
B="$EV/logs/batch.txt"
echo "batch start $(date -u +%FT%T.%NZ)" >> "$B"
for item in $ORDER; do
  IFS=: read -r ID ARM LOAD <<< "$item"
  echo "== $ID $ARM $LOAD $(date -u +%FT%T.%NZ)" | tee -a "$B"
  "$HERE/m3_run.sh" "$EV" "$ID" "$ARM" "$LOAD" 2>&1 | tee -a "$B"
  RC=${PIPESTATUS[0]}
  echo "rc $ID $RC" >> "$B"
  case $RC in
    0|4) ;;
    *) echo "batch STOPPED at $ID (rc $RC) $(date -u +%FT%T.%NZ)" | tee -a "$B"; exit $RC;;
  esac
  if [ "$LOAD" = L1 ]; then sleep 60; else sleep 15; fi
done
echo "batch end $(date -u +%FT%T.%NZ)" | tee -a "$B"
