#!/bin/bash
# d1 batch evaluation (2026-10-12): policy alone, 400 held-out seeds (3000:100 + 4000:300), static + moving.
# Conditions: nom = randomisation off (same as the big11clip bar in traces/alone)
#             rnd = the d1 training randomisation (latency 60±20 ms, obs/joint noise, dynamics)
# Each model runs with its own config (alone_action clip for SAC, scale for PPO).
# Output: traces/d1/<tag>_<cond>_<sc>_<seed0>.json.      [PPO_CKPTS=19999952] eval_d1.sh [P]
# Run on the E-cores at idle priority so a robot session is not disturbed:
#   docker exec franka_ros2 taskset -c 8-23 chrt -i 0 bash /ros2_ws/src/franka_sim/runs/eval_all/d1/eval_d1.sh 16
cd /ros2_ws/src
export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
M=franka_sim/models; O=franka_sim/runs/eval_all/traces/d1; P=${1:-16}; mkdir -p $O
CTRL=("big11clip|onnx:$M/sac_c2_ens_off/ens_off_big11.onnx|$M/sac_d1_a_s1/config.yaml")
for v in s1 s2 s3 s4 s5 s6 p1 p2; do
  CTRL+=("${v}_best|zip:$M/sac_d1_a_$v/best_model.zip|$M/sac_d1_a_$v/config.yaml"
         "${v}_final|zip:$M/sac_d1_a_$v/final_model.zip|$M/sac_d1_a_$v/config.yaml")
done
CTRL+=("ppo_final|zip:$M/ppo_d1/final_model.zip|$M/ppo_d1/config.yaml")
# parents before the d1 fine-tune, run alone + clip exactly like their children
CTRL+=("v10c_parent|zip:$M/sac_v10c/checkpoints/sac_2500000_steps.zip|$M/sac_d1_a_s1/config.yaml"
       "prec_parent|zip:$M/sac_v12_prec_s3/checkpoints/sac_3750000_steps.zip|$M/sac_d1_a_p1/config.yaml")
for s in ${PPO_CKPTS:-}; do CTRL+=("ppo_$s|zip:$M/ppo_d1/checkpoints/ppo_${s}_steps.zip|$M/ppo_d1/config.yaml"); done
CH="3000:25 3025:25 3050:25 3075:25 4000:25 4025:25 4050:25 4075:25 4100:25 4125:25 4150:25 4175:25 4200:25 4225:25 4250:25 4275:25"
for c in "${CTRL[@]}"; do IFS='|' read tag spec cfg <<< "$c"
  for cond in nom rnd; do
    [ $cond = nom ] && X="--set randomization.enabled=false" || X=""
    for sc in static dynamic; do for ch in $CH; do
      out=$O/${tag}_${cond}_${sc}_${ch%%:*}.json
      [ -s $out ] || echo "python3 -m franka_sim.scripts.eval_constraints --controller $spec --config $cfg --mode alone --scenario $sc --seeds $ch $X --out $out"
    done; done
  done
done | xargs -d '\n' -P $P -I{} bash -c '{} 2>&1 | tail -1'
echo ALL DONE
