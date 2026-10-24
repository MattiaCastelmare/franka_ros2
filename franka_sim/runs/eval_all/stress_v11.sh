#!/bin/bash
# Stress battery (2026-10-01): ft_prec 3.75M vs v10c 2.5M on 100 held-out seeds (3000..3099) under perturbations.
# Output traces/stress/<cond>_<model>_<sc>_<seed0>.json.   stress_v11.sh P
cd /ros2_ws/src
export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
T=franka_sim/runs/eval_all; C=franka_sim/models/sac_v10c/config.yaml; P=$1; O=$T/traces/stress; mkdir -p $O
R="randomization.enabled=true"
declare -A COND=(
  [nominal]=""
  [latency]="$R randomization.latency.enabled=true"
  [obsnoise]="$R randomization.obs_noise.enabled=true"
  [jointnoise]="$R randomization.joint_noise.enabled=true"
  [dynamics]="$R randomization.dynamics.enabled=true"
  [allrand]="$R randomization.latency.enabled=true randomization.obs_noise.enabled=true randomization.joint_noise.enabled=true randomization.dynamics.enabled=true"
  [servo]="actuation.enabled=true"
  [fastobs]="obstacle.speed=0.3"
  [bigobs]="obstacle.radius=0.10"
  [qinit]="task.q_init_noise=0.2"
  [long10s]="env.max_episode_steps=1000"
)
MODELS="prec:sac_v11_ft_prec:checkpoints/sac_3750000_steps v10c:sac_v10c:checkpoints/sac_2500000_steps"
jobs() {
  for cond in "${!COND[@]}"; do for mm in $MODELS; do IFS=: read tag run m <<< "$mm"
    for sc in dynamic static; do
      [ $cond = fastobs ] && [ $sc = static ] && continue
      echo "$cond $tag $run $m $sc"; done; done; done
  for sc in dynamic static; do
    echo "nominal zero zero x $sc"; echo "nominal onnx sac_v11_ft_prec checkpoints/sac_3750000_steps.onnx $sc"
    echo "allrand onnx sac_v11_ft_prec checkpoints/sac_3750000_steps.onnx $sc"; done
}
jobs | while read cond tag run m sc; do
  case $sc in static) mode=static ;; dynamic) mode=sinusoidal ;; esac
  for ch in 3000:20 3020:20 3040:20 3060:20 3080:20; do
    out=$O/${cond}_${tag}_${sc}_${ch%%:*}.json
    [ -s $out ] || echo "$ch|$run|$m|$out|obstacle.mode=$mode obstacle.static_fraction=0 ${COND[$cond]}"
  done
done | xargs -P $P -d '\n' -n 1 sh -c 'IFS="|"; set -- $0; IFS=" "; SEEDS=$1 python3 franka_sim/runs/eval_all/test_traces.py $2 $3 '"$C"' $4 $5 2>&1 | tail -1'
