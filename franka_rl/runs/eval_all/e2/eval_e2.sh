#!/bin/bash
# e2 evaluation (runs INSIDE the container). Controllers from stdin as in d2/eval_d2.sh: TAG|zip:PATH or onnx:PATH|CONFIG.
# Unlike e1/eval_e1.sh the scene is FORCED, whatever the model's config says (e1 models carry count_range [1,3]
# and target_ik_check true, so their "one sphere" rows were 1-3 spheres on other episodes):
#   SCENE=one    count_range null  (one sphere)   → traces/e2_one     — same episodes as traces/d2 (d2 ens 794 alone)
#   SCENE=multi  count_range [2,3] (2-3 spheres)  → traces/e2_multi   — same episodes as the d2 rows of traces/e1_multi
#   both with task.target_ik_check false. SEEDS=held (3000:100 + 4000:300, default) | fresh (5000:400 → *_fresh dirs).
#   MODE=full (deployed shield, default; tag gets _full) | alone.   CONDS=nom (default) | "nom rnd".
#   printf 'e2_g_p_s5_2000k|zip:franka_rl/models/sac_e2_g_p_s5/checkpoints/sac_8000000_steps.zip|franka_rl/models/sac_e2_g_p_s5/config.yaml\n' \
#     | docker exec -i -e SCENE=multi franka_ros2 taskset -c 20-23 chrt -i 0 bash /ros2_ws/src/franka_rl/runs/eval_all/e2/eval_e2.sh 4
cd /ros2_ws/src
export PYTHONPATH=/ros2_ws/src MUJOCO_GL=egl OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 CUDA_VISIBLE_DEVICES=
P=${1:-4}; MODE=${MODE:-full}; SCENE=${SCENE:-one}; SEEDS=${SEEDS:-held}
O=franka_rl/runs/eval_all/traces/e2_$SCENE
case $SCENE in
  one)   EXTRA="--set obstacle.count_range=null" ;;
  multi) EXTRA="--set obstacle.count_range=[2,3]" ;;
  *) echo "SCENE must be one|multi"; exit 1 ;;
esac
EXTRA="$EXTRA --set task.target_ik_check=false"
if [ "$SEEDS" = fresh ]; then
  O=${O}_fresh; CH=$(for s in $(seq 5000 25 5375); do printf "%s:25 " $s; done)
else
  CH="3000:25 3025:25 3050:25 3075:25 4000:25 4025:25 4050:25 4075:25 4100:25 4125:25 4150:25 4175:25 4200:25 4225:25 4250:25 4275:25"
fi
mkdir -p $O
while IFS='|' read tag spec cfg; do
  [ -z "$tag" ] && continue
  if [ "$MODE" = full ]; then case $tag in *_full) ;; *) tag=${tag}_full;; esac; fi
  for cond in ${CONDS:-nom}; do
    [ $cond = nom ] && X="--set randomization.enabled=false" || X=""
    for sc in static dynamic; do for ch in $CH; do
      out=$O/${tag}_${cond}_${sc}_${ch%%:*}.json
      [ -s $out ] || echo "python3 -m franka_rl.scripts.eval_constraints --controller $spec --config $cfg --mode $MODE --scenario $sc --seeds $ch $X $EXTRA --out $out"
    done; done
  done
done | xargs -d '\n' -P $P -I{} bash -c '{} 2>&1 | tail -1'
echo ALL DONE
