#!/bin/bash
# Detached auto-eval of the e2 batch (host): every 10 min, score the new checkpoints of the 12 runs, then — once
# all finals exist — the e2 ensembles and the fresh-seed confirmation. Log: runs/eval_all/e2/auto_eval.log
#   setsid nohup runs/eval_all/e2/auto_eval_e2.sh >> runs/eval_all/e2/auto_eval.log 2>&1 < /dev/null &
# Eval cores: 20-23 (4 workers) while any training runs, 8-23 (14 workers) after.
# Scored: +1.0M and +2.0M of every run in the four protocols (one/multi × alone/full; eval_e2.sh forces the scene).
# Baselines (same episodes): parent ensemble ens_d2top6 (the 6 d2 finals the runs start from) and ens_d2k2000_all12.
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_rl"
M=franka_rl/models
PARENTS="p_s5 p4_s1 p_p2 b_1 b_2 p4_p1"
RUNS=""; for a in g s; do for p in $PARENTS; do RUNS="$RUNS ${a}_$p"; done; done
start() { case ${1#?_} in p_p2|p4_p1) echo 7000000 ;; *) echo 6000000 ;; esac; }
training() { docker exec franka_ros2 pgrep -f 'franka_rl.train --config franka_rl/config_e2_' > /dev/null; }
ev() {  # SCENE MODE  (controllers on stdin)
  if training; then C=20-23; P=4; else C=8-23; P=14; fi
  docker exec -i -e SCENE=$1 -e MODE=$2 -e SEEDS=${SEEDS:-held} franka_ros2 taskset -c $C chrt -i 0 \
    bash /ros2_ws/src/franka_rl/runs/eval_all/e2/eval_e2.sh $P | grep -v '^franka_rl/runs' ; }
all4() { local L; L=$(cat); for sc in multi one; do for mo in full alone; do echo "$L" | ev $sc $mo; done; done; }
summary() {
  (cd runs/eval_all
   for sc in one multi; do echo "--- $sc sphere(s), full shield (traces/e2_$sc)"
     DIRS=e2_$sc REF=ens_d2top6_full python3 d2/summary_d2.py '_full$' 2>/dev/null
     echo "--- $sc sphere(s), policy alone"
     DIRS=e2_$sc REF=ens_d2top6 python3 d2/summary_d2.py '^(?!.*_full$)' 2>/dev/null; done)
}
echo "=== e2 auto-eval start $(date)"
# Baselines first: the parent ensemble (24-D, the e2 starting point) and the 12-net d2 ensemble behind the shield.
if [ ! -f models/sac_d2_top6/ens_d2top6.onnx ]; then
  docker exec franka_ros2 bash -lc "cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src CUDA_VISIBLE_DEVICES= python3 -m franka_rl.export_ensemble_onnx \
    --models $(for p in $PARENTS; do printf '%s ' $M/sac_d2_$p/checkpoints/sac_$(start x_$p)_steps.zip; done) \
    --out $M/sac_d2_top6/ens_d2top6.onnx" | tail -2
fi
CFG24=$M/sac_d1_a_s1/config.yaml
printf '%s\n' "ens_d2top6|onnx:$M/sac_d2_top6/ens_d2top6.onnx|$CFG24" | all4
printf '%s\n' "ens_d2k2000_all12|onnx:$M/sac_d2_ens/ens_d2k2000_all12.onnx|$CFG24" | ev one full
printf '%s\n' "ens_d2k2000_all12|onnx:$M/sac_d2_ens/ens_d2k2000_all12.onnx|$CFG24" | ev multi alone
# e1 for the record, on the corrected protocol (its best checkpoint behind the shield).
printf '%s\n' "e1_a1_6000k|zip:$M/sac_e1_a1/checkpoints/sac_5999976_steps.zip|$M/sac_e1_a1/config.yaml" | ev multi full
printf '%s\n' "e1_a1_6000k|zip:$M/sac_e1_a1/checkpoints/sac_5999976_steps.zip|$M/sac_e1_a1/config.yaml" | ev one full
summary
while true; do
  done_all=1
  for k in 1000 2000; do
    for r in $RUNS; do
      s=$(start $r); f=models/sac_e2_$r/checkpoints/sac_$((s + k * 1000))_steps.zip
      [ -f $f ] || { [ $k = 2000 ] && done_all=0; continue; }
      [ -f runs/eval_all/traces/e2_one/e2_${r}_${k}k_nom_dynamic_4275.json ] && continue
      echo "== e2_${r}_${k}k $(date +%T)"
      printf '%s\n' "e2_${r}_${k}k|zip:franka_rl/$f|$M/sac_e2_$r/config.yaml" | all4
    done
  done
  summary
  [ $done_all = 1 ] && break
  sleep 600
done
# Ensembles of the +2.0M nets: per arm (paired with the 6-parent ensemble) and both arms together.
mk() { docker exec franka_ros2 bash -lc "cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src CUDA_VISIBLE_DEVICES= python3 -m franka_rl.export_ensemble_onnx \
  --models $(for r in $2; do printf '%s ' $M/sac_e2_$r/checkpoints/sac_$(( $(start $r) + 2000000 ))_steps.zip; done) --out $M/sac_e2_ens/$1.onnx" | tail -1; }
G6=""; S6=""; for p in $PARENTS; do G6="$G6 g_$p"; S6="$S6 s_$p"; done
mk ens_e2_g6 "$G6"; mk ens_e2_s6 "$S6"; mk ens_e2_gs12 "$G6 $S6"
for e in g6 s6 gs12; do printf '%s\n' "ens_e2_$e|onnx:$M/sac_e2_ens/ens_e2_$e.onnx|$M/sac_e2_g_p_s5/config.yaml"; done | all4
summary
# Fresh seeds 5000:400 for the parent ensemble and the three e2 ensembles (winner's curse check).
{ printf '%s\n' "ens_d2top6|onnx:$M/sac_d2_top6/ens_d2top6.onnx|$CFG24"
  for e in g6 s6 gs12; do printf '%s\n' "ens_e2_$e|onnx:$M/sac_e2_ens/ens_e2_$e.onnx|$M/sac_e2_g_p_s5/config.yaml"; done; } | SEEDS=fresh all4
(cd runs/eval_all
 for sc in one multi; do for x in full alone; do echo "--- FRESH $sc $x"
   if [ $x = full ]; then DIRS=e2_${sc}_fresh REF=ens_d2top6_full python3 d2/summary_d2.py '_full$'
   else DIRS=e2_${sc}_fresh REF=ens_d2top6 python3 d2/summary_d2.py '^(?!.*_full$)'; fi 2>/dev/null; done; done)
echo "AUTO EVAL E2 DONE $(date)"
