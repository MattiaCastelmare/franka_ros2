#!/bin/bash
# Detached auto-eval of the e3 batch (host), same protocol/traces as e2 (e2/eval_e2.sh → traces/e2_one, e2_multi).
#   setsid nohup runs/eval_all/e3/auto_eval_e3.sh >> runs/eval_all/e3/auto_eval.log 2>&1 < /dev/null &
# +0.75M and +1.5M of every run in the four protocols; then ensembles ens_e3_{all12,g6,s6} (models/sac_e3_ens/)
# and fresh seeds 5000:400 next to ens_d2top6 and ens_e2_gs12 (their fresh traces exist from e2).
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_rl"
M=franka_rl/models
PARENTS="p_s5 p4_s1 p_p2 b_1 b_2 p4_p1"
RUNS=""; for a in g s; do for p in $PARENTS; do RUNS="$RUNS ${a}_$p"; done; done
start() { case ${1#?_} in p_p2|p4_p1) echo 9000000 ;; *) echo 8000000 ;; esac; }
training() { docker exec franka_ros2 pgrep -f 'franka_rl.train --config franka_rl/config_e3_' > /dev/null; }
ev() {  # SCENE MODE  (controllers on stdin)
  if training; then C=20-23; P=4; else C=8-23; P=14; fi
  docker exec -i -e SCENE=$1 -e MODE=$2 -e SEEDS=${SEEDS:-held} franka_ros2 taskset -c $C chrt -i 0 \
    bash /ros2_ws/src/franka_rl/runs/eval_all/e2/eval_e2.sh $P | grep -v '^franka_rl/runs' ; }
all4() { local L; L=$(cat); for sc in multi one; do for mo in full alone; do echo "$L" | ev $sc $mo; done; done; }
summary() {
  (cd runs/eval_all
   for sc in one multi; do echo "--- $sc sphere(s), full shield (traces/e2_$sc)"
     DIRS=e2_$sc REF=ens_d2top6_full python3 d2/summary_d2.py '^(ens_|e3_).*_full$' 2>/dev/null
     echo "--- $sc sphere(s), policy alone"
     DIRS=e2_$sc REF=ens_d2top6 python3 d2/summary_d2.py '^(ens_|e3_)(?!.*_full$)' 2>/dev/null; done)
}
echo "=== e3 auto-eval start $(date)"
while true; do
  done_all=1
  for k in 750 1500; do
    for r in $RUNS; do
      f=models/sac_e3_$r/checkpoints/sac_$(( $(start $r) + k * 1000 ))_steps.zip
      [ -f $f ] || { [ $k = 1500 ] && done_all=0; continue; }
      [ -f runs/eval_all/traces/e2_one/e3_${r}_${k}k_nom_dynamic_4275.json ] && continue
      echo "== e3_${r}_${k}k $(date +%T)"
      printf '%s\n' "e3_${r}_${k}k|zip:franka_rl/$f|$M/sac_e3_$r/config.yaml" | all4
    done
  done
  summary
  [ $done_all = 1 ] && break
  sleep 600
done
mk() { docker exec franka_ros2 bash -lc "cd /ros2_ws/src && PYTHONPATH=/ros2_ws/src CUDA_VISIBLE_DEVICES= python3 -m franka_rl.export_ensemble_onnx \
  --models $(for r in $2; do printf '%s ' $M/sac_e3_$r/checkpoints/sac_$(( $(start $r) + 1500000 ))_steps.zip; done) --out $M/sac_e3_ens/$1.onnx" | tail -1; }
G6=""; S6=""; for p in $PARENTS; do G6="$G6 g_$p"; S6="$S6 s_$p"; done
mk ens_e3_g6 "$G6"; mk ens_e3_s6 "$S6"; mk ens_e3_all12 "$G6 $S6"
for e in g6 s6 all12; do printf '%s\n' "ens_e3_$e|onnx:$M/sac_e3_ens/ens_e3_$e.onnx|$M/sac_e3_g_p_s5/config.yaml"; done | all4
summary
for e in g6 s6 all12; do printf '%s\n' "ens_e3_$e|onnx:$M/sac_e3_ens/ens_e3_$e.onnx|$M/sac_e3_g_p_s5/config.yaml"; done | SEEDS=fresh all4
(cd runs/eval_all
 for sc in one multi; do for x in full alone; do echo "--- FRESH $sc $x"
   if [ $x = full ]; then DIRS=e2_${sc}_fresh REF=ens_d2top6_full python3 d2/summary_d2.py '_full$'
   else DIRS=e2_${sc}_fresh REF=ens_d2top6 python3 d2/summary_d2.py '^(?!.*_full$)'; fi 2>/dev/null; done; done)
echo "AUTO EVAL E3 DONE $(date)"
