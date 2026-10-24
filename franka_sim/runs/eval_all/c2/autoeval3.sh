#!/bin/bash
# c2 auto-eval v3 (2026-10-08 20:30): SPEC lines "RUN CONDS CKPT,CKPT,..." → traces/c1_eval/<run>_<k>k_<cond>_<sc>_<seed0>.json
# Every 10 min: evaluate what exists; P = 3 while any c-batch training runs, 10 afterwards; exit when all done.
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
E=runs/eval_all/c1/eval.sh
SPEC="c2_lr3e5 on 3000000,3250000,3500000,3750000
c2_bs2048 on 3000000,3250000,3500000,3750000
c2_off_s2 off,on 3250000,3500000,3750000,4000000
c2_off_s3 off,on 3250000,3500000,3750000,4000000
c2_off_s4 off,on 3250000,3500000,3750000,4000000
c2_off_ps3 off,on 4250000,4500000,4750000,5000000
c2_off_ps2 off,on 4250000,4500000,4750000,5000000"
while true; do
  training=$(docker exec franka_ros2 pgrep -f "exp-name sac_c[0-9]_" | wc -l)
  P=3; [ "$training" -eq 0 ] && P=10
  pending=0; : > /tmp/c2v3_on.txt; : > /tmp/c2v3_offon.txt
  while read -r r conds cks; do
    for s in ${cks//,/ }; do
      if [ -s models/sac_$r/checkpoints/sac_${s}_steps.zip ]; then
        line="${r}_$((s/1000))k sac_$r checkpoints/sac_${s}_steps franka_sim/models/sac_$r/config.yaml"
        [ "$conds" = on ] && echo "$line" >> /tmp/c2v3_on.txt || echo "$line" >> /tmp/c2v3_offon.txt
      else pending=1; fi
    done
  done <<< "$SPEC"
  CONDS=on $E c1_eval $P < /tmp/c2v3_on.txt
  CONDS="off on" $E c1_eval $P < /tmp/c2v3_offon.txt
  [ $pending -eq 0 ] && break
  sleep 600
done
echo "$(date) AUTOEVAL DONE"
