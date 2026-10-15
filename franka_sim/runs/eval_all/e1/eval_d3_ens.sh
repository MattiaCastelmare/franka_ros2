#!/bin/bash
# Score the d3 batch as ENSEMBLES (never done on 2026-10-13): mean action of the 12 d3 finals (+2.0M)
# and of the 6 best shield-on singles, behind the full shield and alone, on the held-out seeds
# (same protocol/traces as d2: traces/d2/*_full_nom_* vs ens_d1b8_d2k2000_full 750, big11clip_full 731).
#   runs/eval_all/e1/eval_d3_ens.sh   (host; detached; log runs/eval_all/e1/eval_d3_ens.log)
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
M=franka_sim/models
all12=""; top6=""
for r in d_s5 d_p2 d_b2 d_p4p1 d_s1 d_cp1 d1_p4s1 d1_s3 f_b1 f_s2 m_cs1 m_p1; do
  case $r in d_p2|d_p4p1|d_cp1|m_p1) s=9000000;; *) s=8000000;; esac
  f=$M/sac_d3_$r/checkpoints/sac_${s}_steps.zip; all12="$all12 $f"
  case $r in d_b2|d_cp1|f_s2|d_p2|d1_s3|d_p4p1) top6="$top6 $f";; esac
done
ENVX="cd /ros2_ws/src && export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 CUDA_VISIBLE_DEVICES="
docker exec franka_ros2 bash -lc "$ENVX && mkdir -p $M/sac_d3_ens && \
  python3 -m franka_sim.export_ensemble_onnx --models $all12 --out $M/sac_d3_ens/ens_d3k2000_all12.onnx && \
  python3 -m franka_sim.export_ensemble_onnx --models $top6 --out $M/sac_d3_ens/ens_d3k2000_top6.onnx"
CFG=franka_sim/models/sac_d1_a_s1/config.yaml
printf '%s\n' "ens_d3k2000_all12_full|onnx:$M/sac_d3_ens/ens_d3k2000_all12.onnx|$CFG" \
              "ens_d3k2000_top6_full|onnx:$M/sac_d3_ens/ens_d3k2000_top6.onnx|$CFG" \
  | docker exec -i -e MODE=full -e CONDS=nom franka_ros2 taskset -c 20-23 chrt -i 0 bash /ros2_ws/src/franka_sim/runs/eval_all/d2/eval_d2.sh 4
printf '%s\n' "ens_d3k2000_all12|onnx:$M/sac_d3_ens/ens_d3k2000_all12.onnx|$CFG" \
              "ens_d3k2000_top6|onnx:$M/sac_d3_ens/ens_d3k2000_top6.onnx|$CFG" \
  | docker exec -i -e MODE=alone -e CONDS=nom franka_ros2 taskset -c 20-23 chrt -i 0 bash /ros2_ws/src/franka_sim/runs/eval_all/d2/eval_d2.sh 4
echo "D3 ENS EVAL DONE $(date)"
