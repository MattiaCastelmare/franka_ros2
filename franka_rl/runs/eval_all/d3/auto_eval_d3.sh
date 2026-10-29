#!/bin/bash
# Score d3 checkpoints behind the full shield as the batch produces them (host, detached):
#   setsid nohup runs/eval_all/d3/auto_eval_d3.sh > runs/eval_all/d3/auto_eval.log 2>&1 &
# For each offset, waits until every run has that checkpoint, runs eval_d2.sh MODE=full CONDS=nom on the
# 400 + 400 held-out seeds (cores 20-23; training holds 8-19), then prints the summary against
# big11clip_full and the d2 20-net mix (ens_d1b8_d2k2000_full, 750).
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_rl"
D=runs/eval_all/d3; N=12
for off in ${OFFS:-+0.5 +1.0 +1.5 +2.0}; do
  until [ "$($D/list_d3.sh "$off" | wc -l)" -ge $N ]; do sleep 300; done
  sleep 60   # let the last checkpoint finish writing
  echo "=== $off  $(date)"
  $D/list_d3.sh "$off" | sed 's/^\([^|]*\)/\1_full/' | docker exec -i -e MODE=full -e CONDS=nom franka_ros2 \
    taskset -c 20-23 chrt -i 0 bash /ros2_ws/src/franka_rl/runs/eval_all/d2/eval_d2.sh 4 | tail -1
  k=$(python3 -c "print(int(round(float('$off') * 1000)))")
  (cd runs/eval_all && REF=big11clip_full python3 d2/summary_d2.py "^d3_.*_${k}k_full$|^big11clip_full$|^ens_d1b8_d2k2000_full$")
done
echo "AUTO EVAL DONE $(date)"
