#!/bin/bash
# Test 1 (2026-10-05): how much avoidance do the shielded policies own? Same 400 held-out seeds
# (3000:100 + 4000:300) and config as traces/avg + traces/v11conf, but the obstacle CBF rows are OFF at
# test time only (env.cbf_obstacle_enabled=false; joint/vel/slew/workspace/floor/base rows stay on).
#   host: runs/eval_all/noshield_eval.sh [P]   -> traces/noshield/<tag>_<sc>_<seed0>.json
#   summary: python3 noshield_summary.py
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
P=${1:-12}
CH=$(echo -n "3000:20 3020:20 3040:20 3060:20 3080:20 "; for i in $(seq 0 14); do echo -n "$((4000+20*i)):20 "; done)
c() { echo "sac_$1/checkpoints/sac_$2_steps"; }
SOUP9=$(for r in v11_ft_prec v11_ft_lr1e4 v11_ft_lr1e4_s2; do for s in 3500000 3750000 4000000; do echo -n "$(c $r $s),"; done; done)
M="zero|zero|zero
v10c|sac_v10c|checkpoints/sac_2500000_steps
prec3.75|sac_v11_ft_prec|checkpoints/sac_3750000_steps
ens_soup9|ens|${SOUP9%,}"
docker exec -e MODELS="$M" -e CH="$CH" -e P="$P" franka_ros2 bash -lc '
cd /ros2_ws/src
export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
T=franka_sim/runs/eval_all; C=franka_sim/models/sac_v10c/config.yaml; O=$T/traces/noshield; mkdir -p $O
echo "$MODELS" | while IFS="|" read tag run m; do
  for ch in $CH; do for sc in static dynamic; do
    case $sc in static) mode=static ;; dynamic) mode=sinusoidal ;; esac
    out=$O/${tag}_${sc}_${ch%%:*}.json
    [ -s $out ] || echo "$ch|$run|$m|$out|$mode"
  done; done
done | xargs -P $P -d "\n" -n 1 sh -c "IFS=\"|\"; set -- \$0; SEEDS=\$1 python3 $T/test_traces.py \$2 \$3 $C \$4 obstacle.mode=\$5 obstacle.static_fraction=0 env.cbf_obstacle_enabled=false 2>&1 | tail -1"
' > runs/eval_all/noshield_eval.log 2>&1
echo "$(date) ALL DONE" >> runs/eval_all/noshield_eval.log
