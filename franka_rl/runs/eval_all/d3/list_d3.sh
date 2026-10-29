#!/bin/bash
# Print eval_d2.sh controller lines for d3 checkpoints:  list_d3.sh "+0.5 +1.0 +2.0" [RUNS]
# Offsets are from the d2 parent step (6.0M, or 7.0M for the p1/p2 lines). Tag: d3_<run>_<offset in k>.
# Every line uses the d1/d2 eval config, so MODE=full numbers compare with ens_d1b8_d2k2000_full
# (750 / 800) and big11clip_full (731):
#   runs/eval_all/d3/list_d3.sh "+1.0" | sed 's/^\([^|]*\)/\1_full/' | docker exec -i -e MODE=full -e CONDS=nom \
#       franka_ros2 taskset -c 20-23 chrt -i 0 bash /ros2_ws/src/franka_rl/runs/eval_all/d2/eval_d2.sh 4
cd "$(dirname "$0")/../../.."      # franka_rl
OFFS=${1:-"+2.0"}
RUNS=${2:-"d_s5 d_p2 d_b2 d_p4p1 d_s1 d_cp1 d1_p4s1 d1_s3 f_b1 f_s2 m_cs1 m_p1"}
for r in $RUNS; do
  case $r in d_p2|d_p4p1|d_cp1|m_p1) base=7000000 ;; *) base=6000000 ;; esac
  for o in $OFFS; do
    k=$(python3 -c "print(int(round(float('$o') * 1000)))")
    f=models/sac_d3_$r/checkpoints/sac_$((base + k * 1000))_steps.zip
    [ -f $f ] && echo "d3_${r}_${k}k|zip:franka_rl/$f|franka_rl/models/sac_d1_a_s1/config.yaml"
  done
done
