#!/bin/bash
# bench_b3.sh NGPU NCPU -> concurrent bench_b3.py runs; samples container memory, GPU memory and CPU temp while they run.
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
G=$1; C=$2; L=runs/eval_all/bench_b3_${G}g${C}c.log; : > $L
( while true; do echo "SAMPLE mem=$(docker stats --no-stream --format '{{.MemUsage}}' franka_ros2) gpu=$(nvidia-smi --query-gpu=memory.used,utilization.gpu,temperature.gpu --format=csv,noheader) cpuT=$(($(cat /sys/class/thermal/thermal_zone5/temp)/1000))C" >> $L; sleep 15; done ) & S=$!
docker exec -e G=$G -e C=$C franka_ros2 bash -lc 'cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
for i in $(seq 1 $G); do python3 franka_sim/runs/eval_all/bench_b3.py franka_sim/config_b3_base.yaml cuda g$i & done
for i in $(seq 1 $C); do python3 franka_sim/runs/eval_all/bench_b3.py franka_sim/config_b3_base.yaml cpu c$i & done
wait' 2>&1 | grep -E "fps|Error|Killed" >> $L
kill $S
echo "== $G GPU + $C CPU: total fps $(grep -oP 'fps \K[0-9.]+' $L | paste -sd+ | bc)" >> $L
