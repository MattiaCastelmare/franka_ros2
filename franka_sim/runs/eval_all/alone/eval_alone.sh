#!/bin/bash
# Policy-alone vs model-based comparison (2026-10-09). 400 held-out seeds (3000:100 + 4000:300), static + moving.
# Modes: alone = no QP at all; obstacle_off = the old "shield OFF" (QP without obstacle rows); full = deployed shield.
# Output: traces/alone/<tag>_<mode>_<sc>_<seed0>.json (franka_sim/scripts/eval_constraints.py).   eval_alone.sh P
cd /ros2_ws/src
export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
M=franka_sim/models; O=franka_sim/runs/eval_all/traces/alone; P=${1:-16}; mkdir -p $O
CTRL=(
  "big11|onnx:$M/sac_c2_ens_off/ens_off_big11.onnx|$M/sac_c2_ens_off/config.yaml|alone obstacle_off full"
  "soup9|onnx:$M/sac_v11_ens9/ens_soup9.onnx|$M/sac_v11_ens9/config.yaml|alone full"
  "prec|zip:$M/sac_v11_ft_prec/checkpoints/sac_3750000_steps.zip|$M/sac_v11_ft_prec/config.yaml|alone"
  "pd|baseline:cartesian_pd|$M/sac_c2_ens_off/config.yaml|alone full"
  "zero|zero|$M/sac_c2_ens_off/config.yaml|alone"
)
CH="3000:25 3025:25 3050:25 3075:25 4000:25 4025:25 4050:25 4075:25 4100:25 4125:25 4150:25 4175:25 4200:25 4225:25 4250:25 4275:25"
for c in "${CTRL[@]}"; do IFS='|' read tag spec cfg modes <<< "$c"
  for mode in $modes; do for sc in static dynamic; do for ch in $CH; do
    out=$O/${tag}_${mode}_${sc}_${ch%%:*}.json
    [ -s $out ] || echo "python3 -m franka_sim.scripts.eval_constraints --controller $spec --config $cfg --mode $mode --scenario $sc --seeds $ch --out $out"
  done; done; done
done | xargs -d '\n' -P $P -I{} bash -c '{} 2>&1 | tail -1'
echo ALL DONE
