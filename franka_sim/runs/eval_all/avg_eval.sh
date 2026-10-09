#!/bin/bash
# Checkpoint-averaging screen (2026-10-01): weight soups + action ensembles on held-out seeds.
#   avg_eval.sh P OUTDIR "CHUNKS"    -> OUTDIR/<tag>_<sc>_<seed0>.json
cd /ros2_ws/src
export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
T=franka_sim/runs/eval_all; C=franka_sim/models/sac_v10c/config.yaml; P=$1; O=$T/$2; CH=$3; mkdir -p $O
PR=sac_v11_ft_prec/checkpoints
MODELS="${MODELS:-avg_k3|sac_v11_avg|prec_3.5-4.0_k3
avg_k4|sac_v11_avg|prec_3.25-4.0_k4
avg_k11|sac_v11_avg|prec_3.5-4.0_k11
avg_k5|sac_v11_avg|prec_3.65-3.85_k5
soup3|sac_v11_avg|soup3_3.75
soup9|sac_v11_avg|soup3_3.5-4.0_k9
ens_prec3|ens|$PR/sac_3500000_steps,$PR/sac_3750000_steps,$PR/sac_4000000_steps
ens_soup3|ens|$PR/sac_3750000_steps,sac_v11_ft_lr1e4/checkpoints/sac_3750000_steps,sac_v11_ft_lr1e4_s2/checkpoints/sac_3750000_steps}"
echo "$MODELS" | while IFS='|' read tag run m; do
  for ch in $CH; do for sc in static dynamic; do
    case $sc in static) mode=static ;; dynamic) mode=sinusoidal ;; esac
    out=$O/${tag}_${sc}_${ch%%:*}.json
    [ -s $out ] || echo "$ch|$run|$m|$out|$mode"
  done; done
done | xargs -P $P -d '\n' -n 1 sh -c 'IFS="|"; set -- $0; SEEDS=$1 python3 franka_sim/runs/eval_all/test_traces.py $2 $3 '"$C"' $4 obstacle.mode=$5 obstacle.static_fraction=0 2>&1 | tail -1'
