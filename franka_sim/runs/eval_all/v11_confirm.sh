#!/bin/bash
# Held-out confirmation on 100 NEW seeds (3000..3099), static + dynamic, sac_v10c config.   v11_confirm.sh P RUN:STEP ...
cd /ros2_ws/src
export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
T=franka_sim/runs/eval_all; C=franka_sim/models/sac_v10c/config.yaml; P=$1; shift
mkdir -p $T/traces/v11conf
for rs in "$@"; do r=${rs%%:*}; s=${rs##*:}
  case $s in *_model) m=$s ;; *) m=checkpoints/sac_${s}_steps ;; esac
  for ch in ${CHUNKS:-3000:20 3020:20 3040:20 3060:20 3080:20}; do for sc in static dynamic; do
    case $sc in static) mode=static ;; dynamic) mode=sinusoidal ;; esac
    out=$T/traces/v11conf/${r}_${s}_${sc}_${ch%%:*}.json
    [ -s $out ] || echo "$ch $r $m $out $mode"
  done; done
done | xargs -P $P -L 1 sh -c 'SEEDS=$0 python3 franka_sim/runs/eval_all/test_traces.py $1 $2 '"$C"' $3 obstacle.mode=$4 obstacle.static_fraction=0 2>&1 | tail -1'
