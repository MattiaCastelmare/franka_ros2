#!/bin/bash
# Evaluate every best/final model on the same 40 seeds, 7 jobs at a time.
cd /ros2_ws/src
export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
for d in franka_sim/models/sac_*/; do
  run=$(basename $d)
  for m in best_model final_model; do
    [ -f $d/$m.zip ] || continue
    echo "$run $m"
  done
done | xargs -P 7 -n 2 sh -c 'python3 -m franka_sim.scripts.evaluate_policy --model franka_sim/models/$0/$1.zip --config franka_sim/models/$0/config.yaml --episodes 40 --seed 1000 > franka_sim/runs/eval_all/$0__$1.txt 2>&1; echo "done $0 $1"'
