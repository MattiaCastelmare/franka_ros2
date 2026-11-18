#!/bin/bash
# M1 — one executor-thread replay (protocol refactor_r1_m1/protocol/M1_PROTOCOL.md §5).
# usage: m1_run.sh <evidence_dir> <run_id> <arm>     arm = single | mt1 | mt3
# Runs m1_executor_replay.py in the container (DDS domain 231, localhost only), copies the run's
# four output files to <evidence_dir>/runs/, checks validity with m1_analyze.py. No retry.
set -u
EV=$1; ID=$2; ARM=$3
HERE=$(cd "$(dirname "$0")" && pwd)
W=/home/user/m1_20261118
CELL='/home/user/e3_20261116/05_s0_multi/batch/cells/pentagon_static_baseline_bare__full-executor_mode=multi__gz__s0'
E305=$HOME/franka_baseline_evidence/refactor_r1_e3/runs/05_s0_multi
RATE=0.70
mkdir -p "$EV/runs" "$EV/logs" "$EV/host"
H="$EV/host/${ID}.txt"
{
  echo "run $ID arm $ARM pre $(date -u +%FT%T.%NZ)"
  echo "loadavg $(cat /proc/loadavg)"
  echo "governor $(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | sort | uniq -c | xargs)"
  echo "epp $(cat /sys/devices/system/cpu/cpu*/cpufreq/energy_performance_preference | sort | uniq -c | xargs)"
  echo "freq_mhz $(awk '{s+=$1;n++} END {printf "%.0f", s/n/1000}' /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq)"
  echo "host_ros_gz $(pgrep -fa 'gz sim|ign gazebo|ros2 (launch|bag)|rviz' | grep -v pgrep | wc -l)"
  echo "container_ros $(docker exec franka_ros2 bash -c "ps -eo args | grep -E 'ros2|gz|ign|_node|cbf|replay|parameter_bridge' | grep -v grep | wc -l")"
} > "$H"
if [ "$(grep -c '^host_ros_gz 0$' "$H")" != 1 ] || [ "$(grep -c '^container_ros 0$' "$H")" != 1 ]; then
  echo "ABORT $ID: unexpected ROS/Gazebo process (see $H)"; exit 3
fi
if ! grep -q '^governor 28 powersave$' "$H" || ! grep -q '^epp 28 performance$' "$H"; then
  echo "ABORT $ID: CPU governor/EPP differs from the recorded state (see $H)"; exit 3
fi
T_START=$(date -u +%FT%T.%NZ)
docker exec -w /tmp -e PYTHONDONTWRITEBYTECODE=1 -e ROS_DOMAIN_ID=231 -e ROS_LOCALHOST_ONLY=1 franka_ros2 bash -c \
  "source /ros2_ws/install/setup.bash >/dev/null; timeout 260 python3 -B $W/m1_executor_replay.py $ARM '$CELL' $W/runs/$ID $RATE $W/launch_params_quny0qie $W/launch_params_w4adpacz $W/launch_params_xa00zx9n" \
  > "$EV/logs/${ID}.log" 2>&1
RC=$?
T_END=$(date -u +%FT%T.%NZ)
{
  echo "rc $RC start $T_START end $T_END"
  echo "post loadavg $(cat /proc/loadavg)"
  echo "post governor $(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | sort | uniq -c | xargs)"
  echo "post epp $(cat /sys/devices/system/cpu/cpu*/cpufreq/energy_performance_preference | sort | uniq -c | xargs)"
  echo "post container_ros $(docker exec franka_ros2 bash -c "ps -eo args | grep -E 'ros2|gz|ign|_node|cbf|replay|parameter_bridge' | grep -v grep | wc -l")"
} >> "$H"
for s in .json _raw.npz _m1.json _m1.npz; do docker cp "franka_ros2:$W/runs/$ID$s" "$EV/runs/" 2>>"$EV/logs/${ID}.log"; done
if [ $RC -ne 0 ] || grep -q Traceback "$EV/logs/${ID}.log"; then echo "INVALID $ID: rc=$RC or Traceback (see logs/${ID}.log)"; exit 4; fi
python3 -I "$HERE/m1_analyze.py" "$EV/runs" "$E305" "$EV/runs/${ID}_check.json" "$ID" > "$EV/logs/${ID}_check.txt" 2>&1 || { echo "INVALID $ID: analysis failed"; exit 4; }
cat "$EV/logs/${ID}_check.txt"
if ! python3 -I -c "import json,sys; sys.exit(0 if json.load(open('$EV/runs/${ID}_check.json'))['runs'][0]['valid'] else 1)"; then
  echo "INVALID $ID: validity check failed"; exit 4
fi
echo "OK $ID"
