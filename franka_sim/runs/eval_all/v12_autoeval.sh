#!/bin/bash
# Host-side, detached: wait for each v12 run to finish, evaluate its 250k checkpoints on the 400 held-out seeds
# (3000:100 + 4000:300, sac_v10c config, same episodes as ft_prec 3.75M in traces/v11conf), write v12_summary.txt.
# Eval parallelism 8 while any training still runs (container memory limit 20 GiB, OOM on 2026-10-01), 20 after.
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
E=runs/eval_all; LOG=$E/v12_autoeval.log
CH=$(echo -n "3000:20 3020:20 3040:20 3060:20 3080:20 "; for i in $(seq 0 14); do echo -n "$((4000+20*i)):20 "; done)
declare -A STEPS=(
  [v12_cont]="4000000 4250000 4500000 4750000" [v12_hard]="4000000 4250000 4500000 4750000"
  [v12_slack]="4000000 4250000 4500000 4750000" [v12_lr3e5]="4000000 4250000 4500000 4750000"
  [v12_hard_slack]="4000000 4250000 4500000 4750000" [v12_prec4]="4000000 4250000 4500000 4750000"
  [v12_prec_s2]="3000000 3250000 3500000 3750000" [v12_prec_s3]="3000000 3250000 3500000 3750000"
)
mkdir -p $E/traces/v12
echo "$(date) autoeval started" >> $LOG
while :; do
  left=0
  for r in "${!STEPS[@]}"; do
    [ -f $E/traces/v12/.done_$r ] && continue
    left=$((left + 1))
    [ -f models/sac_$r/final_model.zip ] || continue
    ntrain=$(docker exec franka_ros2 pgrep -fc "franka_sim.train" 2>/dev/null || echo 0)
    P=$([ "$ntrain" -gt 0 ] && echo 8 || echo 20)
    M=""; for s in ${STEPS[$r]}; do M="$M${M:+
}sac_${r}_$s|sac_$r|checkpoints/sac_${s}_steps"; done
    echo "$(date) evaluating $r (P=$P, $ntrain trainings running)" >> $LOG
    docker exec -e MODELS="$M" franka_ros2 bash -lc "cd /ros2_ws/src && franka_sim/runs/eval_all/avg_eval.sh $P traces/v12 '$CH'" >> $LOG 2>&1
    touch $E/traces/v12/.done_$r
    docker exec franka_ros2 bash -lc "cd /ros2_ws/src/franka_sim/runs/eval_all && python3 v12_summary.py" > /dev/null 2>&1
    echo "$(date) done $r" >> $LOG
  done
  [ $left -eq 0 ] && break
  sleep 300
done
echo "$(date) ALL DONE" >> $LOG
