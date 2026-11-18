#!/bin/bash
# M1 — the nine pre-registered replays in the sealed order (M1_PROTOCOL.md §4). Stops at the
# first abort or invalid run. No retry.
set -u
EV=$1
HERE=$(cd "$(dirname "$0")" && pwd)
ORDER="m1_01:single m1_02:mt1 m1_03:mt3 m1_04:mt1 m1_05:mt3 m1_06:single m1_07:mt3 m1_08:single m1_09:mt1"
echo "batch start $(date -u +%FT%T.%NZ) order: $ORDER" | tee -a "$EV/logs/batch.log"
for e in $ORDER; do
  ID=${e%%:*}; ARM=${e##*:}
  echo "== $ID $ARM $(date -u +%T)" | tee -a "$EV/logs/batch.log"
  "$HERE/m1_run.sh" "$EV" "$ID" "$ARM" 2>&1 | tee -a "$EV/logs/batch.log"
  rc=${PIPESTATUS[0]}
  if [ "$rc" -ne 0 ]; then echo "STOP after $ID (rc=$rc)" | tee -a "$EV/logs/batch.log"; exit "$rc"; fi
  sleep 10
done
echo "batch done $(date -u +%FT%T.%NZ)" | tee -a "$EV/logs/batch.log"
