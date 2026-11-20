#!/bin/bash
# M3: one executor-thread replay under a host-load condition (protocol refactor_r1_m3/protocol/M3_PROTOCOL.md §5).
# usage: m3_run.sh <evidence_dir> <run_id> <arm> <load>     arm = single | mt1 | mt3, load = L0 | L1
# The replay is the M1 replay, unchanged (m1_executor_replay.py -> r1_executor_replay.py, same cell, the same
# three parameter files, rate 0.70, DDS domain 231). New in M3: the host sampler and, for L1, the load workers.
# Exit codes: 0 = OK, 3 = ABORT (pre-check, cleanup or thermal: the batch stops), 4 = INVALID by a replay
# validity gate (recorded; the batch continues), 5 = harness failure (rc != 0, traceback or missing output:
# the batch stops). No retry in any case.
set -u
EV=$1; ID=$2; ARM=$3; LOAD=$4
HERE=$(cd "$(dirname "$0")" && pwd)
W=/home/user/m3_20261118
CELL='/home/user/e3_20261116/05_s0_multi/batch/cells/pentagon_static_baseline_bare__full-executor_mode=multi__gz__s0'
E305=$HOME/franka_baseline_evidence/refactor_r1_e3/runs/05_s0_multi
RATE=0.70
NWORK=12          # §4: E3 non-filter host load 11.2-12.4 busy cores
RAMP=45           # §4: s of load before the replay starts
CAP=330           # §4: hard kill of every worker (ramp + the replay's own 260 s timeout + margin)
case "$LOAD" in L0|L1) ;; *) echo "ABORT $ID: load $LOAD"; exit 3;; esac
mkdir -p "$EV/runs" "$EV/logs" "$EV/host"
H="$EV/host/${ID}.txt"
STOPF="$EV/host/${ID}_sampler.stop"
cleanup() { "$HERE/m3_load.sh" stop >> "$H" 2>&1; touch "$STOPF"; }
trap 'cleanup; echo "ABORT $ID: interrupted"; exit 3' INT TERM
busy2s() {   # busy cores over 2 s from /proc/stat
  python3 -I -c "
import time
def r():
    s=[l.split() for l in open('/proc/stat') if l.startswith('cpu') and l[3].isdigit()]
    return [(sum(map(int,x[1:9])), int(x[4])+int(x[5])) for x in s]
a=r(); time.sleep(2); b=r()
print('%.2f' % sum(1-(bi-ai)/max(bt-at,1) for (at,ai),(bt,bi) in zip(a,b)))"
}
{
  echo "run $ID arm $ARM load $LOAD pre $(date -u +%FT%T.%NZ)"
  echo "loadavg $(cat /proc/loadavg)"
  echo "governor $(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | sort | uniq -c | xargs)"
  echo "epp $(cat /sys/devices/system/cpu/cpu*/cpufreq/energy_performance_preference | sort | uniq -c | xargs)"
  echo "pkg_temp_mC $(for z in /sys/class/thermal/thermal_zone*; do [ "$(cat $z/type)" = x86_pkg_temp ] && cat $z/temp; done)"
  echo "load_workers $("$HERE/m3_load.sh" count)"
  echo "busy_cores_2s $(busy2s)"
  echo "host_ros_gz $(pgrep -fa 'gz sim|ign gazebo|ros2 (launch|bag)|rviz' | grep -v pgrep | wc -l)"
  echo "container_ros $(docker exec franka_ros2 bash -c "ps -eo args | grep -E 'ros2|gz|ign|_node|cbf|replay|parameter_bridge' | grep -v grep | wc -l")"
} > "$H"
fail_pre() { echo "ABORT $ID: $1 (see $H)"; exit 3; }
grep -q '^host_ros_gz 0$' "$H" && grep -q '^container_ros 0$' "$H" || fail_pre 'unexpected ROS/Gazebo process'
grep -q '^governor 28 powersave$' "$H" && grep -q '^epp 28 performance$' "$H" || fail_pre 'CPU governor/EPP differs from the recorded state'
grep -q '^load_workers 0$' "$H" || fail_pre 'load workers alive before the run'
python3 -I -c "import sys; sys.exit(0 if float('$(grep '^busy_cores_2s' "$H" | cut -d' ' -f2)') <= 2.0 else 1)" || fail_pre 'host not idle before the run (> 2.0 busy cores)'
python3 -I -c "import sys; sys.exit(0 if int('$(grep '^pkg_temp_mC' "$H" | cut -d' ' -f2)') < 85000 else 1)" || fail_pre 'package temperature >= 85 C before the run'
rm -f "$STOPF"
python3 -I "$HERE/m3_host_sampler.py" "$EV/host/${ID}_samples.npz" "$STOPF" 0.5 95000 4 &   # Addendum 01: 4 consecutive samples
SPID=$!
if [ "$LOAD" = L1 ]; then
  echo "load_start $(date -u +%FT%T.%NZ) $("$HERE/m3_load.sh" start $NWORK $CAP)" >> "$H"
  sleep $RAMP
  echo "load_workers_at_replay_start $("$HERE/m3_load.sh" count)" >> "$H"
fi
T_START=$(date -u +%FT%T.%NZ)
docker exec -w /tmp -e PYTHONDONTWRITEBYTECODE=1 -e ROS_DOMAIN_ID=231 -e ROS_LOCALHOST_ONLY=1 franka_ros2 bash -c \
  "source /ros2_ws/install/setup.bash >/dev/null; timeout 260 python3 -B $W/m1_executor_replay.py $ARM '$CELL' $W/runs/$ID $RATE $W/launch_params_quny0qie $W/launch_params_w4adpacz $W/launch_params_xa00zx9n" \
  > "$EV/logs/${ID}.log" 2>&1
RC=$?
T_END=$(date -u +%FT%T.%NZ)
[ "$LOAD" = L1 ] && echo "load_workers_at_replay_end $("$HERE/m3_load.sh" count)" >> "$H"
cleanup
wait $SPID
trap - INT TERM
{
  echo "rc $RC start $T_START end $T_END"
  echo "post loadavg $(cat /proc/loadavg)"
  echo "post governor $(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | sort | uniq -c | xargs)"
  echo "post epp $(cat /sys/devices/system/cpu/cpu*/cpufreq/energy_performance_preference | sort | uniq -c | xargs)"
  echo "post load_workers $("$HERE/m3_load.sh" count)"
  echo "post container_ros $(docker exec franka_ros2 bash -c "ps -eo args | grep -E 'ros2|gz|ign|_node|cbf|replay|parameter_bridge' | grep -v grep | wc -l")"
} >> "$H"
for s in .json _raw.npz _m1.json _m1.npz; do docker cp "franka_ros2:$W/runs/$ID$s" "$EV/runs/" 2>>"$EV/logs/${ID}.log"; done
grep -q '^post load_workers 0$' "$H" || { echo "ABORT $ID: load workers survived cleanup"; exit 3; }
[ -e "$EV/host/${ID}_samples.npz.thermal_abort" ] && { echo "ABORT $ID: thermal guard fired"; exit 3; }
grep -q '^post governor 28 powersave$' "$H" && grep -q '^post epp 28 performance$' "$H" || { echo "ABORT $ID: governor/EPP changed"; exit 3; }
for s in .json _raw.npz _m1.json _m1.npz; do [ -s "$EV/runs/$ID$s" ] || { echo "HARNESS FAILURE $ID: missing $ID$s"; exit 5; }; done
if [ $RC -ne 0 ] || grep -q Traceback "$EV/logs/${ID}.log"; then echo "HARNESS FAILURE $ID: rc=$RC or Traceback (see logs/${ID}.log)"; exit 5; fi
python3 -I "$HERE/m1_analyze.py" "$EV/runs" "$E305" "$EV/runs/${ID}_check.json" "$ID" > "$EV/logs/${ID}_check.txt" 2>&1 || { echo "HARNESS FAILURE $ID: analysis failed"; exit 5; }
cat "$EV/logs/${ID}_check.txt"
if ! python3 -I -c "import json,sys; sys.exit(0 if json.load(open('$EV/runs/${ID}_check.json'))['runs'][0]['valid'] else 1)"; then
  echo "INVALID $ID: replay validity gate failed (recorded; no retry)"; exit 4
fi
echo "OK $ID"
