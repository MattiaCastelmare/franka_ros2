#!/bin/bash
# Print eval_d2.sh controller lines for d2 checkpoints:  list_d2.sh "+0.5 +1.0 +2.0" [RUNS]
# A checkpoint is named by its offset from the parent step (s/b runs resume at 4.0M, p runs at 5.0M);
# only checkpoints that exist are printed.  Tag: <run>_<offset in k>, e.g. p_s1_1000k.
cd "$(dirname "$0")/../../.."      # franka_rl
OFFS=${1:-"+2.0"}
RUNS=${2:-"p_s1 p_s5 p_s2 p_s3 p_p1 p_p2 p4_s1 p4_p1 b_1 b_2 c_s1 c_p1"}
for r in $RUNS; do
  case $r in *_p1|*_p2) base=5000000 ;; *) base=4000000 ;; esac
  for o in $OFFS; do
    k=$(python3 -c "print(int(round(float('$o') * 1000)))")
    step=$((base + k * 1000))
    f=models/sac_d2_$r/checkpoints/sac_${step}_steps.zip
    if [ -f $f ]; then echo "${r}_${k}k|zip:franka_rl/$f|franka_rl/models/sac_d2_$r/config.yaml"; fi
  done
done
