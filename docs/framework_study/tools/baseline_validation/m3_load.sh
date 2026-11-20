#!/bin/bash
# M3: controlled synthetic host load (protocol refactor_r1_m3/protocol/M3_PROTOCOL.md §4).
#   m3_load.sh start <n_workers> <cap_s>   start n pure-Python busy-loop workers, each killed after cap_s
#   m3_load.sh stop                        SIGKILL every worker, then verify none remains (exit 1 if any does)
#   m3_load.sh count                       print the number of live worker processes
# Workers are plain user processes at the default nice level, in their own session (setsid), with no
# CPU pinning and no system setting changed. Every worker carries the command-line marker
# M3_LOAD_WORKER, so stop and the sampler's thermal guard find all of them. The `timeout` cap
# guarantees cleanup even if the caller dies.
set -u
MARK=M3_LOAD_WORKER
case "${1:-}" in
  start)
    N=$2; CAP=$3
    for _ in $(seq "$N"); do
      setsid timeout --signal=KILL "${CAP}s" python3 -I -c 'while True: pass' "$MARK" </dev/null >/dev/null 2>&1 &
    done
    sleep 1
    echo "started $(pgrep -fc "^python3 -I -c while True: pass $MARK$")"
    ;;
  stop)
    pkill -KILL -f "$MARK" 2>/dev/null
    for _ in 1 2 3 4 5 6 7 8 9 10; do
      [ "$(pgrep -fc "$MARK")" = 0 ] && { echo "stopped 0 left"; exit 0; }
      sleep 0.5
    done
    echo "STOP FAILED: $(pgrep -fc "$MARK") left"; exit 1
    ;;
  count)
    pgrep -fc "^python3 -I -c while True: pass $MARK$" || true
    ;;
  *) echo "usage: m3_load.sh start <n> <cap_s> | stop | count"; exit 2;;
esac
