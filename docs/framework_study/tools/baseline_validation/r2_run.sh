#!/bin/bash
# R2: one replay of the SHIPPED default executor (arm default: no executor override anywhere) at playback
# rate 0.70 or 1.0 under a host-load condition (protocol refactor_r2/04_perf/protocol/R2_PERF_PROTOCOL.md).
# Derived from m3b_run.sh (M3b, sealed) with exactly these changes: RATE is an argument (0.70 | 1.0);
# container workdir r2_20261120/perf; arm default through r2_default_replay.py, with the E3-05 parameter
# file launch_params_xa00zx9n replaced by its copy without the executor_mode line (_noexec); the sleep
# inhibitor is R2_BATCH; _r2.json is copied back; the per-run gate is r2_analyze.py --check.
# usage: r2_run.sh <evidence_dir> <run_id> <rate> <load>     rate = 0.70 | 1.0, load = L0 | L1
# Exit codes: 0 = OK, 3 = ABORT (pre-check, inhibitor, suspend, cleanup or thermal: the batch stops),
# 4 = INVALID by a validity gate (recorded; the batch continues), 5 = harness failure (the batch stops).
# No retry in any case.
set -u
EV=$1; ID=$2; RATE=$3; LOAD=$4; ARM=default
HERE=$(cd "$(dirname "$0")" && pwd)
W=/home/user/r2_20261120/perf
CELL='/home/user/e3_20261116/05_s0_multi/batch/cells/pentagon_static_baseline_bare__full-executor_mode=multi__gz__s0'
E305=$HOME/franka_baseline_evidence/refactor_r1_e3/runs/05_s0_multi
NWORK=12          # §4: E3 non-filter host load 11.2-12.4 busy cores
RAMP=45           # §4: s of load before the replay starts
CAP=330           # §4: hard kill of every worker (ramp + the replay's own 260 s timeout + margin), as M3
case "$LOAD" in L0|L1) ;; *) echo "ABORT $ID: load $LOAD"; exit 3;; esac
case "$RATE" in 0.70|1.0) ;; *) echo "ABORT $ID: rate $RATE"; exit 3;; esac
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
boot_minus_mono() { python3 -I -c "import time; print('%.3f' % (time.clock_gettime(time.CLOCK_BOOTTIME) - time.monotonic()))"; }
inhibitor_held() { systemd-inhibit --list --no-pager 2>/dev/null | grep -q 'R2_BATCH' && echo 1 || echo 0; }
{
  echo "run $ID arm $ARM load $LOAD rate $RATE pre $(date -u +%FT%T.%NZ)"
  echo "boot_minus_mono $(boot_minus_mono)"
  echo "inhibitor_held $(inhibitor_held)"
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
grep -q '^inhibitor_held 1$' "$H" || fail_pre 'R2_BATCH sleep inhibitor not held'
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
  "source /ros2_ws/install/setup.bash >/dev/null; timeout 260 python3 -B $W/r2_default_replay.py '$CELL' $W/runs/$ID $RATE $W/launch_params_quny0qie $W/launch_params_w4adpacz $W/launch_params_xa00zx9n_noexec" \
  > "$EV/logs/${ID}.log" 2>&1
RC=$?
T_END=$(date -u +%FT%T.%NZ)
[ "$LOAD" = L1 ] && echo "load_workers_at_replay_end $("$HERE/m3_load.sh" count)" >> "$H"
cleanup
wait $SPID
trap - INT TERM
{
  echo "rc $RC start $T_START end $T_END"
  echo "post boot_minus_mono $(boot_minus_mono)"
  echo "post inhibitor_held $(inhibitor_held)"
  echo "post loadavg $(cat /proc/loadavg)"
  echo "post governor $(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | sort | uniq -c | xargs)"
  echo "post epp $(cat /sys/devices/system/cpu/cpu*/cpufreq/energy_performance_preference | sort | uniq -c | xargs)"
  echo "post load_workers $("$HERE/m3_load.sh" count)"
  echo "post container_ros $(docker exec franka_ros2 bash -c "ps -eo args | grep -E 'ros2|gz|ign|_node|cbf|replay|parameter_bridge' | grep -v grep | wc -l")"
} >> "$H"
for s in .json _raw.npz _m1.json _m1.npz _r2.json; do docker cp "franka_ros2:$W/runs/$ID$s" "$EV/runs/" 2>>"$EV/logs/${ID}.log"; done
grep -q '^post load_workers 0$' "$H" || { echo "ABORT $ID: load workers survived cleanup"; exit 3; }
python3 -I -c "import sys; a=float('$(grep '^boot_minus_mono' "$H" | cut -d' ' -f2)'); b=float('$(grep '^post boot_minus_mono' "$H" | cut -d' ' -f3)'); sys.exit(0 if abs(b-a) <= 1.0 else 1)" || { echo "ABORT $ID: host suspended during the run (boot-monotonic offset changed)"; exit 3; }
grep -q '^post inhibitor_held 1$' "$H" || { echo "ABORT $ID: sleep inhibitor lost during the run"; exit 3; }
[ -e "$EV/host/${ID}_samples.npz.thermal_abort" ] && { echo "ABORT $ID: thermal guard fired"; exit 3; }
grep -q '^post governor 28 powersave$' "$H" && grep -q '^post epp 28 performance$' "$H" || { echo "ABORT $ID: governor/EPP changed"; exit 3; }
for s in .json _raw.npz _m1.json _m1.npz _r2.json; do [ -s "$EV/runs/$ID$s" ] || { echo "HARNESS FAILURE $ID: missing $ID$s"; exit 5; }; done
if [ $RC -ne 0 ] || grep -q Traceback "$EV/logs/${ID}.log"; then echo "HARNESS FAILURE $ID: rc=$RC or Traceback (see logs/${ID}.log)"; exit 5; fi
python3 -I "$HERE/r2_analyze.py" --check "$EV" "$E305" "$ID" > "$EV/logs/${ID}_check.txt" 2>&1
CRC=$?
cat "$EV/logs/${ID}_check.txt"
case $CRC in
  0) ;;
  10) echo "INVALID $ID: validity gate failed (recorded; no retry)"; exit 4;;
  *) echo "HARNESS FAILURE $ID: analysis failed (rc $CRC)"; exit 5;;
esac
echo "OK $ID"
