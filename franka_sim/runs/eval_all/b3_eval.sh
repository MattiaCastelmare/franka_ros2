#!/bin/bash
# b3 checkpoint screen (2026-10-05): waits for every run's checkpoint at +STEP steps, then evaluates it on
# SEEDS chunks with the obstacle CBF OFF (first) and ON, static + moving obstacle, each run with its OWN frozen config.
#   host: runs/eval_all/b3_eval.sh STEP [OUTDIR] [CHUNKS] [P]   -> traces/<OUTDIR>/<tag>_<off|on>_<sc>_<seed0>.json
#   summary: python3 b3_summary.py traces/<OUTDIR>
# Runs next to 10 trainings: at most P jobs (default 2), no new job while the container uses > 17 GiB or the HOST has
# < 4 GiB available (docker stats leaves out swap; on 10/05 the trainings had filled the 8 GB host swap).
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
STEP=$1; OUT=${2:-b3_$((STEP / 1000))k}; CH=${3:-"3000:20 3020:20 3040:20 3060:20 3080:20"}; P=${4:-2}
RUNS="b3_base b3_base_s2 b3_base_s3 b3_margin30 b3_wobs5 b3_coll200 b3_prec b3_obs51 b3_noterm b3_ft_v10c"
ck() { case $1 in b3_ft_v10c) echo "checkpoints/sac_$((2500000 + STEP))_steps" ;; *) echo "checkpoints/sac_${STEP}_steps" ;; esac; }
host_avail_gib() { awk '/MemAvailable/{print $2/1048576}' /proc/meminfo; }
mem_gib() { docker stats --no-stream --format '{{.MemUsage}}' franka_ros2 | awk '{v=$1; if (v ~ /MiB/) {sub("MiB","",v); v/=1024} else sub("GiB","",v); print v}'; }
echo "$(date) waiting for checkpoints at +$STEP"
for r in $RUNS; do until [ -s models/sac_$r/$(ck $r).zip ]; do sleep 120; done; done
echo "$(date) all checkpoints present"
mkdir -p runs/eval_all/traces/$OUT
JOBS=()
for cond in off on; do for r in $RUNS; do for ch in $CH; do for sc in static dynamic; do
  [ -s runs/eval_all/traces/$OUT/${r}_${cond}_${sc}_${ch%%:*}.json ] || JOBS+=("$cond $r $ch $sc")
done; done; done; done
echo "$(date) ${#JOBS[@]} jobs, P=$P"
for j in "${JOBS[@]}"; do
  set -- $j; cond=$1; r=$2; ch=$3; sc=$4
  while [ $(jobs -rp | wc -l) -ge $P ] || awk "BEGIN{exit !($(mem_gib) > 17.0 || $(host_avail_gib) < 4.0)}"; do sleep 20; done
  case $sc in static) mode=static ;; dynamic) mode=sinusoidal ;; esac
  case $cond in off) flag=false ;; on) flag=true ;; esac
  docker exec -e SEEDS=$ch franka_ros2 bash -lc "cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 && \
    python3 franka_sim/runs/eval_all/test_traces.py sac_$r $(ck $r) franka_sim/models/sac_$r/config.yaml \
    franka_sim/runs/eval_all/traces/$OUT/${r}_${cond}_${sc}_${ch%%:*}.json \
    obstacle.mode=$mode obstacle.static_fraction=0 env.cbf_obstacle_enabled=$flag 2>&1 | tail -1" &
  sleep 3
done
wait
echo "$(date) ALL DONE"
