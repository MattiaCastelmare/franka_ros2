#!/bin/bash
# Re-score the pre-v6 runs on the hard task (blocking_fraction 0.6) with their own obs layout.
cd /ros2_ws/src
export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=2 MKL_NUM_THREADS=2
for r in sac_v3 sac_v4 sac_v5; do
  python3 - "$r" <<'PY'
import sys, yaml
r = sys.argv[1]
c = yaml.safe_load(open(f'franka_sim/models/{r}/config.yaml'))
c.setdefault('task', {})['blocking_fraction'] = 0.6
yaml.safe_dump(c, open(f'franka_sim/runs/eval_all/{r}_hard.yaml', 'w'))
PY
done
for r in sac_v3 sac_v4 sac_v5; do for m in best_model final_model; do echo $r $m; done; done |
  xargs -P 6 -n 2 sh -c 'python3 -m franka_sim.scripts.evaluate_policy --model franka_sim/models/$0/$1.zip --config franka_sim/runs/eval_all/$0_hard.yaml --episodes 40 --seed 1000 > franka_sim/runs/eval_all/HARD_$0__$1.txt 2>&1; echo "done $0 $1"'
