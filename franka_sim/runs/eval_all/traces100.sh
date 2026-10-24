#!/bin/bash
# sac_v10/best_model on 100 fresh seeds (2000..2099), static + dynamic, same settings as traces.sh.
cd /ros2_ws/src
export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
T=franka_sim/runs/eval_all; C=franka_sim/models/sac_v10/config.yaml
for i in 0 1 2 3; do s=$((2000 + 25*i))
  SEEDS=$s:25 python3 $T/test_traces.py sac_v10 best_model $C $T/traces/v10best_static_$s.json obstacle.mode=static obstacle.static_fraction=0 2>&1 | tail -1 &
  SEEDS=$s:25 python3 $T/test_traces.py sac_v10 best_model $C $T/traces/v10best_dynamic_$s.json obstacle.mode=sinusoidal obstacle.static_fraction=0 2>&1 | tail -1 &
done; wait
