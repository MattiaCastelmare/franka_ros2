#!/bin/bash
# sac_v10c/<model> on the 100 fresh seeds (2000..2099), static + dynamic, as traces100.sh.   traces100_v10c.sh best_model final_model
cd /ros2_ws/src
export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
T=franka_sim/runs/eval_all; C=franka_sim/models/sac_v10c/config.yaml
for m in "$@"; do for i in 0 1 2 3; do s=$((2000 + 25*i)); for sc in static dynamic; do
  case $sc in static) mode=static ;; dynamic) mode=sinusoidal ;; esac
  echo "$m $s $sc $mode"; done; done; done |
xargs -P 16 -L 1 sh -c 'SEEDS=$1:25 python3 franka_sim/runs/eval_all/test_traces.py sac_v10c $0 '"$C"' '"$T"'/traces/v10c/fresh_$0_$2_$1.json obstacle.mode=$3 obstacle.static_fraction=0 2>&1 | tail -1'
