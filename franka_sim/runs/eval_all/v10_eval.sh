#!/bin/bash
# 40-seed hold test of the sac_v10 ablation, static + dynamic scenario, each model with its own config.
cd /ros2_ws/src
export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
for r in sac_v10 sac_v10_obs51 sac_v10_g997; do for m in best_model final_model; do
  echo "$r $m static static"; echo "$r $m dynamic sinusoidal"; done; done |
xargs -P 12 -L 1 sh -c 'python3 franka_sim/runs/eval_all/hold.py $0 $1 franka_sim/models/$0/config.yaml ${2}_v10 obstacle.mode=$3 obstacle.static_fraction=0 > franka_sim/runs/eval_all/V10_$0__$1__$2.txt 2>&1; echo "done $0 $1 $2"'
cat franka_sim/runs/eval_all/V10_*.txt | grep -E "reached-ever" | sort > franka_sim/runs/eval_all/SCENARIOS_v10.txt
