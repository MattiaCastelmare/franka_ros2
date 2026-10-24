#!/bin/bash
# Stress battery for ens_off_big11 (2026-10-09), shield ON (deployed setting), 100 held-out seeds 3000..3099.
# Same perturbations as stress_v11.sh, plus a 0.5 m/s obstacle. Output traces/stress_big11/<cond>_<sc>_<seed0>.json.
cd /ros2_ws/src
export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
T=franka_sim/runs/eval_all; C=franka_sim/models/sac_b3_ft_v10c/config.yaml; P=${1:-10}; O=$T/traces/stress_big11
R="randomization.enabled=true"
declare -A COND=(
  [nominal]=""
  [latency]="$R randomization.latency.enabled=true"
  [obsnoise]="$R randomization.obs_noise.enabled=true"
  [jointnoise]="$R randomization.joint_noise.enabled=true"
  [dynamics]="$R randomization.dynamics.enabled=true"
  [allrand]="$R randomization.latency.enabled=true randomization.obs_noise.enabled=true randomization.joint_noise.enabled=true randomization.dynamics.enabled=true"
  [fastobs]="obstacle.speed=0.3"
  [fastobs05]="obstacle.speed=0.5"
  [bigobs]="obstacle.radius=0.10"
)
for cond in "${!COND[@]}"; do for sc in dynamic static; do
  case $cond in fastobs*) [ $sc = static ] && continue ;; esac
  case $sc in static) mode=static ;; dynamic) mode=sinusoidal ;; esac
  for ch in 3000:20 3020:20 3040:20 3060:20 3080:20; do
    out=$O/${cond}_${sc}_${ch%%:*}.json
    [ -s $out ] || echo "$ch|$out|obstacle.mode=$mode obstacle.static_fraction=0 env.cbf_obstacle_enabled=true env.cbf_obstacle_on_prob=null ${COND[$cond]}"
  done
done; done | xargs -P $P -d '\n' -n 1 sh -c 'IFS="|"; set -- $0; IFS=" "; SEEDS=$1 python3 franka_sim/runs/eval_all/test_traces.py sac_c2_ens_off ens_off_big11.onnx '"$C"' $2 $3 2>&1 | tail -1'
echo ALL DONE
