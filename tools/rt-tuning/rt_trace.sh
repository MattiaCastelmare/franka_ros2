#!/usr/bin/env bash
# Flight recorder for the isolated cores: keeps the last few minutes of
# scheduler / IRQ / network events of CPU2 (robot NIC) and CPU3 (FCI loop) in
# the kernel ring buffer, so that after a communication_constraints_violation
# the trace shows what the RT thread was doing when it stopped answering.
#
#   sudo rt_trace.sh start      # before launching the stack
#   ... run until it crashes ...
#   sudo rt_trace.sh save       # right after the crash -> /tmp/rt_trace_<time>.dat
#   sudo rt_trace.sh reset      # stop tracing and free the buffer
#
# The buffer overwrites itself, so it can stay on for a whole session.
set -euo pipefail

CPUMASK="${RT_TRACE_CPUMASK:-c}"     # hex mask: c = CPU2 + CPU3
BUF_KB="${RT_TRACE_BUF_KB:-65536}"   # per CPU

case "${1:-}" in
    start)
        trace-cmd start -M "$CPUMASK" -b "$BUF_KB" \
            -e sched:sched_switch -e sched:sched_wakeup -e sched:sched_waking \
            -e sched:sched_migrate_task -e sched:sched_pi_setprio \
            -e irq:irq_handler_entry -e irq:irq_handler_exit \
            -e irq:softirq_entry -e irq:softirq_exit \
            -e net:net_dev_queue -e net:net_dev_xmit -e net:netif_receive_skb \
            -e timer:hrtimer_expire_entry
        echo "[rt_trace] recording CPU mask 0x$CPUMASK, ${BUF_KB} KB/CPU ring buffer"
        ;;
    save)
        out="/tmp/rt_trace_$(date +%H%M%S).dat"
        trace-cmd stop
        trace-cmd extract -o "$out"
        chmod a+r "$out"
        trace-cmd restart
        echo "[rt_trace] saved $out (recording resumed)"
        ;;
    reset)
        trace-cmd reset
        echo "[rt_trace] tracing off, buffer freed"
        ;;
    *)
        echo "usage: $0 start|save|reset" >&2
        exit 1
        ;;
esac
