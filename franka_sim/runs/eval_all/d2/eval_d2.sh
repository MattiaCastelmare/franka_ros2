#!/bin/bash
# d2 evaluation: same protocol as d1/eval_d1.sh (policy alone, 400 held-out seeds 3000:100 + 4000:300,
# static + moving, nom = randomisation off, rnd = d1 training randomisation), controllers read from stdin:
#     TAG|onnx:PATH or zip:PATH|CONFIG          (CONDS="nom rnd" by default)
# Output: traces/d2/<tag>_<cond>_<sc>_<seed0>.json; existing files are skipped, so it can be re-run.
# FRESH=1: the confirmation set 5000:400 instead, into traces/d2_fresh/.
# MODE=full: the deployed shield (CBF with obstacle rows) instead of the policy alone — give such tags a _full suffix.
#   docker exec -i franka_ros2 taskset -c 20-23 chrt -i 0 bash /ros2_ws/src/franka_sim/runs/eval_all/d2/eval_d2.sh 4 < list.txt
cd /ros2_ws/src
export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 CUDA_VISIBLE_DEVICES=
O=franka_sim/runs/eval_all/traces/d2; P=${1:-4}
CH="3000:25 3025:25 3050:25 3075:25 4000:25 4025:25 4050:25 4075:25 4100:25 4125:25 4150:25 4175:25 4200:25 4225:25 4250:25 4275:25"
if [ -n "$FRESH" ]; then O=${O}_fresh; CH=$(for s in $(seq 5000 25 5375); do printf "%s:25 " $s; done); fi
mkdir -p $O
while IFS='|' read tag spec cfg; do
  [ -z "$tag" ] && continue
  for cond in ${CONDS:-nom rnd}; do
    [ $cond = nom ] && X="--set randomization.enabled=false" || X=""
    for sc in static dynamic; do for ch in $CH; do
      out=$O/${tag}_${cond}_${sc}_${ch%%:*}.json
      [ -s $out ] || echo "python3 -m franka_sim.scripts.eval_constraints --controller $spec --config $cfg --mode ${MODE:-alone} --scenario $sc --seeds $ch $X --out $out"
    done; done
  done
done | xargs -d '\n' -P $P -I{} bash -c '{} 2>&1 | tail -1'
echo ALL DONE
