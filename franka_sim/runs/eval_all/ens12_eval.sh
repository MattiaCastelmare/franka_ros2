#!/bin/bash
# Ensembles with v12 members (2026-10-02) on the 400 held-out seeds (3000:100 + 4000:300) -> traces/avg,
# next to ens_soup3/ens_soup9 (same seeds). Summary: python3 avg_summary.py traces/avg
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
CH=$(echo -n "3000:20 3020:20 3040:20 3060:20 3080:20 "; for i in $(seq 0 14); do echo -n "$((4000+20*i)):20 "; done)
c() { echo "sac_$1/checkpoints/sac_$2_steps"; }
P3=$(c v11_ft_prec 3750000); L3=$(c v11_ft_lr1e4 3750000); M3=$(c v11_ft_lr1e4_s2 3750000)
S3=$(c v12_prec_s3 3750000); S3b=$(c v12_prec_s3 3500000); S2=$(c v12_prec_s2 3500000); R=$(c v12_lr3e5 4750000)
SOUP9=$(for r in v11_ft_prec v11_ft_lr1e4 v11_ft_lr1e4_s2; do for s in 3500000 3750000 4000000; do echo -n "$(c $r $s),"; done; done)
M="ens_v12new3|ens|$S3,$R,$S2
ens_mix3|ens|$P3,$S3,$R
ens_mix6|ens|$P3,$L3,$M3,$S3,$R,$S2
ens_mix13|ens|${SOUP9}$S3,$S3b,$S2,$R"
docker exec -e MODELS="$M" franka_ros2 bash -lc "cd /ros2_ws/src && franka_sim/runs/eval_all/avg_eval.sh 12 traces/avg '$CH'" > runs/eval_all/ens12_eval.log 2>&1
docker exec franka_ros2 bash -lc "cd /ros2_ws/src/franka_sim/runs/eval_all && python3 avg_summary.py traces/avg" > runs/eval_all/ens12_summary.txt 2>&1
echo "$(date) ALL DONE" >> runs/eval_all/ens12_eval.log
