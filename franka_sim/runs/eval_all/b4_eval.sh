#!/bin/bash
# b4 evaluation (2026-10-06, from b3_final_eval.sh; + env.cbf_obstacle_on_prob=null so mixing runs are tested with the shield fixed): explicit "tag run ckpt" list, obstacle CBF OFF + ON,
# static + moving, each run with its OWN frozen config.  traces/<OUT>/<tag>_<off|on>_<sc>_<seed0>.json
#   host: runs/eval_all/b4_eval.sh OUT "CHUNKS" P  < list   (list lines: TAG RUN CKPT)
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
OUT=$1; CH=$2; P=${3:-10}
host_avail_gib() { awk '/MemAvailable/{print $2/1048576}' /proc/meminfo; }
mem_gib() { docker stats --no-stream --format '{{.MemUsage}}' franka_ros2 | awk '{v=$1; if (v ~ /MiB/) {sub("MiB","",v); v/=1024} else sub("GiB","",v); print v}'; }
mkdir -p runs/eval_all/traces/$OUT
mapfile -t LIST
JOBS=()
for cond in off on; do for l in "${LIST[@]}"; do for ch in $CH; do for sc in static dynamic; do
  set -- $l; [ -s runs/eval_all/traces/$OUT/${1}_${cond}_${sc}_${ch%%:*}.json ] || JOBS+=("$cond $l $ch $sc")
done; done; done; done
echo "$(date) ${#JOBS[@]} jobs, P=$P"
for j in "${JOBS[@]}"; do
  set -- $j; cond=$1; tag=$2; r=$3; ck=$4; ch=$5; sc=$6
  while [ $(jobs -rp | wc -l) -ge $P ] || awk "BEGIN{exit !($(mem_gib) > 17.0 || $(host_avail_gib) < 4.0)}"; do sleep 10; done
  case $sc in static) mode=static ;; dynamic) mode=sinusoidal ;; esac
  case $cond in off) flag=false ;; on) flag=true ;; esac
  docker exec -e SEEDS=$ch franka_ros2 bash -lc "cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
    python3 franka_sim/runs/eval_all/test_traces.py $r $ck franka_sim/models/$r/config.yaml \
    franka_sim/runs/eval_all/traces/$OUT/${tag}_${cond}_${sc}_${ch%%:*}.json \
    obstacle.mode=$mode obstacle.static_fraction=0 env.cbf_obstacle_enabled=$flag env.cbf_obstacle_on_prob=null 2>&1 | tail -1" &
  sleep 2
done
wait
echo "$(date) ALL DONE"
