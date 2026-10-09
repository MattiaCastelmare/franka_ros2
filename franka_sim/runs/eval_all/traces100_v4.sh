#!/bin/bash
# sac_v4/final_model on the same 100 fresh seeds (2000..2099) as traces100.sh, with its own scenario configs (as traces.sh).
cd /ros2_ws/src
export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
T=franka_sim/runs/eval_all
for i in 0 1 2 3; do s=$((2000 + 25*i))
  for sc in static dynamic; do
    SEEDS=$s:25 python3 $T/test_traces.py sac_v4 final_model $T/sac_v4_$sc.yaml $T/traces/v4final_${sc}_$s.json 2>&1 | tail -1 &
  done
done; wait
