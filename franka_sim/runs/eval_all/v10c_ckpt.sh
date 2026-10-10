#!/bin/bash
# sac_v10c checkpoints on the 40 benchmark seeds (1000..1039), static + dynamic, same settings as traces.sh.
#   v10c_ckpt.sh 2250000 2500000 ...   (a step count, or "final_model" / "best_model")
cd /ros2_ws/src
export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
T=franka_sim/runs/eval_all; C=franka_sim/models/sac_v10c/config.yaml
for s in "$@"; do
  case $s in *_model) m=$s ;; *) m=checkpoints/sac_${s}_steps ;; esac
  echo "$m $T/traces/v10c/${s}_static.json obstacle.mode=static"
  echo "$m $T/traces/v10c/${s}_dynamic.json obstacle.mode=sinusoidal"
done | xargs -P 12 -L 1 sh -c 'python3 franka_sim/runs/eval_all/test_traces.py sac_v10c $0 '"$C"' $1 $2 obstacle.static_fraction=0 2>&1 | tail -1'
