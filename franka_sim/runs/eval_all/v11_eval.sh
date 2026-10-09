#!/bin/bash
# v11 checkpoints on the 140 paired seeds (1000:40 bench + 2000:100 fresh), static + dynamic, all under the
# sac_v10c config (identical episodes to the v10c 2.5M baseline).   v11_eval.sh P RUN:STEP [RUN:STEP ...]
#   RUN = sac_v11_ft_prec ..., STEP = 3000000 | final_model | best_model;  P = parallel jobs
cd /ros2_ws/src
export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
T=franka_sim/runs/eval_all; C=franka_sim/models/sac_v10c/config.yaml; P=$1; shift
mkdir -p $T/traces/v11
for rs in "$@"; do r=${rs%%:*}; s=${rs##*:}
  case $s in *_model) m=$s ;; *) m=checkpoints/sac_${s}_steps ;; esac
  for ch in 1000:20 1020:20 2000:20 2020:20 2040:20 2060:20 2080:20; do for sc in static dynamic; do
    case $sc in static) mode=static ;; dynamic) mode=sinusoidal ;; esac
    out=$T/traces/v11/${r}_${s}_${sc}_${ch%%:*}.json
    [ -s $out ] || echo "$ch $r $m $out $mode"
  done; done
done | xargs -P $P -L 1 sh -c 'SEEDS=$0 python3 franka_sim/runs/eval_all/test_traces.py $1 $2 '"$C"' $3 obstacle.mode=$4 obstacle.static_fraction=0 2>&1 | tail -1'
