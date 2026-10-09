#!/bin/bash
# Per-step traces of the 40-seed hold test for the plots (see test_traces.py).
cd /ros2_ws/src
export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
T=franka_sim/runs/eval_all
{ for s in static dynamic; do echo "sac_v4 final_model $T/sac_v4_$s.yaml $s"; done
  for r in sac_v10 sac_v10_obs51 sac_v10_g997; do
    echo "$r best_model franka_sim/models/$r/config.yaml static obstacle.mode=static obstacle.static_fraction=0"
    echo "$r best_model franka_sim/models/$r/config.yaml dynamic obstacle.mode=sinusoidal obstacle.static_fraction=0"; done; } |
xargs -P 8 -L 1 sh -c 'python3 franka_sim/runs/eval_all/test_traces.py $0 $1 $2 franka_sim/runs/eval_all/traces/$0__$1__$3.json $4 $5 2>&1 | tail -1' 
