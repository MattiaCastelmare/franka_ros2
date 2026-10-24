#!/bin/bash
# c1 evaluation (2026-10-08, from b4_eval.sh): explicit list on stdin, one line per model:
#   TAG RUN CKPT CONFIG      (RUN=ens → CKPT = comma list of models/<run>/checkpoints/<ckpt> paths without .zip)
# For every line: obstacle CBF OFF and/or ON (CONDS), static + moving, the 400 held-out seeds (CHUNKS).
#   host: runs/eval_all/c1/eval.sh OUT P < list      → traces/<OUT>/<tag>_<off|on>_<sc>_<seed0>.json
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
OUT=$1; P=${2:-3}; CONDS=${CONDS:-"off on"}
CH=${CHUNKS:-"3000:25 3025:25 3050:25 3075:25 4000:25 4025:25 4050:25 4075:25 4100:25 4125:25 4150:25 4175:25 4200:25 4225:25 4250:25 4275:25"}
host_avail_gib() { awk '/MemAvailable/{print $2/1048576}' /proc/meminfo; }
mem_gib() { docker stats --no-stream --format '{{.MemUsage}}' franka_ros2 | awk '{v=$1; if (v ~ /MiB/) {sub("MiB","",v); v/=1024} else sub("GiB","",v); print v}'; }
mkdir -p runs/eval_all/traces/$OUT
mapfile -t LIST
JOBS=()
for cond in $CONDS; do for l in "${LIST[@]}"; do for ch in $CH; do for sc in static dynamic; do
  set -- $l; [ -s runs/eval_all/traces/$OUT/${1}_${cond}_${sc}_${ch%%:*}.json ] || JOBS+=("$cond $l $ch $sc")
done; done; done; done
echo "$(date) ${#JOBS[@]} jobs, P=$P"
for j in "${JOBS[@]}"; do
  set -- $j; cond=$1; tag=$2; r=$3; ck=$4; cfg=$5; ch=$6; sc=$7
  while [ $(jobs -rp | wc -l) -ge $P ] || awk "BEGIN{exit !($(mem_gib) > 17.5 || $(host_avail_gib) < 3.0)}"; do sleep 10; done
  case $sc in static) mode=static ;; dynamic) mode=sinusoidal ;; esac
  case $cond in off) flag=false ;; on) flag=true ;; esac
  docker exec -e SEEDS=$ch franka_ros2 bash -lc "cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
    python3 franka_sim/runs/eval_all/test_traces.py $r $ck $cfg \
    franka_sim/runs/eval_all/traces/$OUT/${tag}_${cond}_${sc}_${ch%%:*}.json \
    obstacle.mode=$mode obstacle.static_fraction=0 env.cbf_obstacle_enabled=$flag env.cbf_obstacle_on_prob=null 2>&1 | tail -1" &
  sleep 2
done
wait
echo "$(date) ALL DONE"
