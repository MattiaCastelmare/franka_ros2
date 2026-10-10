#!/bin/bash
# c1 auto-eval (2026-10-08): every 10 min, evaluate the c1 checkpoints that exist and have no traces yet.
# ON-recipe runs: shield ON only, ckpts 3.0-3.75M; OFF-recipe runs: OFF + ON, ckpts 3.25-4.0M. → traces/c1_eval
# P (default 3 while trainings run) is raised to 10 once no c1 training is left.
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
M=models; E=runs/eval_all/c1/eval.sh
while true; do
  training=$(docker exec franka_ros2 pgrep -f "exp-name sac_c1_" | wc -l)
  P=3; [ "$training" -eq 0 ] && P=10
  : > /tmp/c1_on.txt; : > /tmp/c1_off.txt; pending=0
  for r in c1_ctrl c1_te14 c1_te14_s2 c1_te21 c1_te21_s2 c1_te28; do for s in 3000000 3250000 3500000 3750000; do
    if [ -s $M/sac_$r/checkpoints/sac_${s}_steps.zip ]; then echo "${r}_$((s/1000))k sac_$r checkpoints/sac_${s}_steps franka_sim/models/sac_$r/config.yaml" >> /tmp/c1_on.txt; else pending=1; fi
  done; done
  for r in c1_off_te21 c1_off_te14; do for s in 3250000 3500000 3750000 4000000; do
    if [ -s $M/sac_$r/checkpoints/sac_${s}_steps.zip ]; then echo "${r}_$((s/1000))k sac_$r checkpoints/sac_${s}_steps franka_sim/models/sac_$r/config.yaml" >> /tmp/c1_off.txt; else pending=1; fi
  done; done
  CONDS=on $E c1_eval $P < /tmp/c1_on.txt
  CONDS="off on" $E c1_eval $P < /tmp/c1_off.txt
  [ $pending -eq 0 ] && [ "$training" -eq 0 ] && break
  sleep 600
done
echo "$(date) AUTOEVAL DONE"
