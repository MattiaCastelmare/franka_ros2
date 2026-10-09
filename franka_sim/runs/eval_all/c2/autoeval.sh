#!/bin/bash
# c1+c2 auto-eval (2026-10-08): every 10 min, evaluate the checkpoints that exist and have no traces yet → traces/c1_eval.
# ON-recipe runs: shield ON only, ckpts 3.0-3.75M; OFF-recipe runs: OFF + ON, ckpts 3.25-4.0M.
# P = 3 while any c-batch training runs, 10 afterwards.
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
M=models; E=runs/eval_all/c1/eval.sh
ON_RUNS=${ON_RUNS:-"c1_ctrl c1_te14 c1_te14_s2 c2_lr3e5 c2_bs2048"}
OFF_RUNS=${OFF_RUNS:-"c1_off_te14 c2_off_s2 c2_off_s3"}
while true; do
  training=$(docker exec franka_ros2 pgrep -f "exp-name sac_c[0-9]_" | wc -l)
  P=3; [ "$training" -eq 0 ] && P=10
  : > /tmp/c2_on.txt; : > /tmp/c2_off.txt; pending=0
  for r in $ON_RUNS; do for s in 3000000 3250000 3500000 3750000; do
    if [ -s $M/sac_$r/checkpoints/sac_${s}_steps.zip ]; then echo "${r}_$((s/1000))k sac_$r checkpoints/sac_${s}_steps franka_sim/models/sac_$r/config.yaml" >> /tmp/c2_on.txt; else pending=1; fi
  done; done
  for r in $OFF_RUNS; do for s in 3250000 3500000 3750000 4000000; do
    if [ -s $M/sac_$r/checkpoints/sac_${s}_steps.zip ]; then echo "${r}_$((s/1000))k sac_$r checkpoints/sac_${s}_steps franka_sim/models/sac_$r/config.yaml" >> /tmp/c2_off.txt; else pending=1; fi
  done; done
  CONDS=on $E c1_eval $P < /tmp/c2_on.txt
  CONDS="off on" $E c1_eval $P < /tmp/c2_off.txt
  [ $pending -eq 0 ] && break
  sleep 600
done
echo "$(date) AUTOEVAL DONE"
