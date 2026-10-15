#!/bin/bash
# End-of-batch d2 evaluation (host). Exports the candidate ensembles, then runs eval_d2.sh on them
# (held-out, nom + rnd) and on the single networks at +1.5M / +2.0M (nom + rnd for +2.0M, nom for +1.5M).
#   CORES=8-23 P=16 runs/eval_all/d2/final_d2.sh        (after every sac_d2_* run has its final_model.zip)
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
CORES=${CORES:-8-23}; P=${P:-16}
D=runs/eval_all/d2; M=franka_sim/models; C=$M/sac_d1_a_s1/config.yaml
RUNS="p_s1 p_s5 p_s2 p_s3 p_p1 p_p2 p4_s1 p4_p1 b_1 b_2 c_s1 c_p1"
docker exec franka_ros2 bash -lc "cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src OMP_NUM_THREADS=1 CUDA_VISIBLE_DEVICES= && M=$M &&
  E='taskset -c $CORES chrt -i 0 python3 -m franka_sim.export_ensemble_onnx' &&
  ck() { case \$1 in *_p1|*_p2) echo \$M/sac_d2_\$1/checkpoints/sac_\$((5000000+\$2))_steps.zip ;; *) echo \$M/sac_d2_\$1/checkpoints/sac_\$((4000000+\$2))_steps.zip ;; esac; }
  K1=''; K15=''; K2=''; for r in $RUNS; do K1=\"\$K1 \$(ck \$r 1000000)\"; K15=\"\$K15 \$(ck \$r 1500000)\"; K2=\"\$K2 \$(ck \$r 2000000)\"; done
  D1=''; for v in s1 s2 s3 s4 s5 s6 p1 p2; do D1=\"\$D1 \$M/sac_d1_a_\$v/best_model.zip\"; done
  ex() { o=\$1; shift; [ -s \$M/sac_d2_ens/\$o.onnx ] || \$E --models \$@ --out \$M/sac_d2_ens/\$o.onnx 2>&1 | grep -E 'exported|validation|rror'; }
  ex ens_d2k2000_all12 \$K2
  ex ens_d1b8_d2k2000 \$D1 \$K2
  ex ens_d1b8_d2k1000_k2000 \$D1 \$K1 \$K2
  ex ens_d2k1000_k1500_k2000 \$K1 \$K15 \$K2"
{
  for e in d2k2000_all12 d1b8_d2k2000 d1b8_d2k1000_k2000 d2k1000_k1500_k2000 d1b8_d2k1000; do
    echo "ens_$e|onnx:$M/sac_d2_ens/ens_$e.onnx|$C"; done
  $D/list_d2.sh "+2.0"
} > $D/list_final.txt
$D/list_d2.sh "+1.5" > $D/list_k1500.txt
docker exec -i franka_ros2 taskset -c $CORES chrt -i 0 bash /ros2_ws/src/franka_sim/$D/eval_d2.sh $P < $D/list_final.txt
docker exec -i -e CONDS=nom franka_ros2 taskset -c $CORES chrt -i 0 bash /ros2_ws/src/franka_sim/$D/eval_d2.sh $P < $D/list_k1500.txt
echo FINAL DONE
