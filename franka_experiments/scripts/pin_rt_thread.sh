#!/usr/bin/env bash
# Pin the SCHED_FIFO thread of ros2_control_node onto an isolated CPU.
#
# WHY only that thread, and not the whole process:  ros2_control_node carries
# ~60 threads (DDS listeners, executors, service handlers) and exactly one that
# runs the 1 kHz FCI control loop under SCHED_FIFO.  `taskset -cp 3 <PID>` moves
# all of them onto the isolated core, so the very core reserved for the deadline
# ends up hosting the DDS traffic that was supposed to stay away from it.
# Pinning the one FF thread keeps the isolated core exclusively for the loop.
#
# Requires isolcpus in the kernel cmdline; without it the target core is shared
# with the general scheduler and the pin buys almost nothing.  See
# /etc/default/grub -> isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3.
#
# Usage:  pin_rt_thread.sh [CPU] [TIMEOUT_S]
#         CPU        core to pin to           (default 3)
#         TIMEOUT_S  how long to wait for the
#                    controller to come up    (default 60)
set -uo pipefail

CPU="${1:-3}"
TIMEOUT="${2:-60}"

# Finding the right PID is fiddlier than it looks.  `pgrep -x` cannot be used
# because /proc/<pid>/comm truncates at 15 characters ("ros2_control_no"), and
# `pgrep -f` matches ANY process whose command line merely mentions the string
# -- the shell that launched this script included.  So: match on the installed
# executable path, then keep only candidates that actually own a SCHED_FIFO
# thread.  That test is the real filter; a decoy shell never passes it.
find_ff_thread() {
    ps -L -o tid=,cls= -p "$1" 2>/dev/null | awk '$2 == "FF" { print $1; exit }'
}

# Echoes "<pid> <tid>" for the first controller that has its RT thread up.
find_rt_target() {
    local pid tid
    for pid in $(pgrep -f 'controller_manager/ros2_control_node' 2>/dev/null); do
        [[ "$pid" == "$$" ]] && continue
        tid=$(find_ff_thread "$pid")
        if [[ -n "$tid" ]]; then
            echo "$pid $tid"
            return 0
        fi
    done
    return 1
}

deadline=$(( SECONDS + TIMEOUT ))
while (( SECONDS < deadline )); do
    # The FF thread appears only when the controller activates and
    # realtime_tools::configure_sched_fifo() runs, several seconds after the
    # process itself, so an empty result here is normal and worth retrying.
    if target=$(find_rt_target); then
        read -r pid tid <<< "$target"
        if taskset -cp "$CPU" "$tid" >/dev/null 2>&1; then
            echo "[pin_rt_thread] ros2_control_node pid=$pid rt-thread=$tid -> CPU$CPU"
            ps -L -o tid,psr,cls,rtprio,comm -p "$pid" | awk 'NR == 1 || $4 != "-"'
            exit 0
        fi
        echo "[pin_rt_thread] taskset failed on tid=$tid (permissions? cpuset?)" >&2
        exit 1
    fi
    sleep 1
done

echo "[pin_rt_thread] no SCHED_FIFO thread found within ${TIMEOUT}s -- not pinned." >&2
echo "[pin_rt_thread] check that the controller activated: the launch log should" >&2
echo "[pin_rt_thread] contain 'Successful set up FIFO RT scheduling policy'." >&2
exit 1
