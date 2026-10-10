#!/bin/bash
# Constrained-IK feasibility of the static held-out seeds listed in traces/feas/list.txt ("seed group") -> traces/feas/<group>_<seed>.json
cd /ros2_ws/src; export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
T=franka_sim/runs/eval_all/traces/feas
cat $T/list.txt | xargs -P ${1:-12} -L 1 sh -c 'o='$T'/$1_$0.json; [ -s $o ] || SEEDS=$0 python3 franka_sim/runs/eval_all/feasibility.py static 2>/dev/null | tail -1 > $o'
echo ALL DONE
