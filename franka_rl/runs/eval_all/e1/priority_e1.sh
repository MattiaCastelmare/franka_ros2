#!/bin/bash
# One-off (2026-10-16): the auto-eval glob never matched, so score the most informative checkpoints first,
# then hand over to the (fixed) auto loop. Cores 20-23.
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_rl"
M=franka_rl/models
ev() { docker exec -i -e SCENE=$1 -e MODE=$2 -e CONDS=nom franka_ros2 taskset -c 20-23 chrt -i 0 bash /ros2_ws/src/franka_rl/runs/eval_all/e1/eval_e1.sh 4; }
L="e1_m1_final|zip:$M/sac_e1_m1/final_model.zip|$M/sac_e1_m1/config.yaml
e1_a2_4000k|zip:$M/sac_e1_a2/checkpoints/sac_3999984_steps.zip|$M/sac_e1_a2/config.yaml
e1_a1_4000k|zip:$M/sac_e1_a1/checkpoints/sac_3999984_steps.zip|$M/sac_e1_a1/config.yaml
e1_a3_4000k|zip:$M/sac_e1_a3/checkpoints/sac_3999984_steps.zip|$M/sac_e1_a3/config.yaml"
echo "=== priority start $(date)"
echo "$L" | ev one full
echo "$L" | ev multi full
printf 'e1_m1_final|zip:%s/sac_e1_m1/final_model.zip|%s/sac_e1_m1/config.yaml\n' $M $M | ev one alone
(cd runs/eval_all && REF=big11clip_full python3 d2/summary_d2.py '^(e1_|ens_d1b8_d2k2000_full|big11clip_full|ens_d2k2000_all12|big11clip)$')
(cd runs/eval_all && DIRS=e1_multi REF=none python3 d2/summary_d2.py 2>/dev/null)
echo "=== priority done $(date)"
exec runs/eval_all/e1/auto_eval_e1.sh
