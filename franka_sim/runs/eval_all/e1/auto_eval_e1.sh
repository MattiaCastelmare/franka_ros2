#!/bin/bash
# Score e1 checkpoints as they land (detached, cores 20-23): every 500k-step checkpoint of every run,
# behind the full shield, on (a) the single-sphere held-out protocol (comparable with d1/d2/d3) and
# (b) the 2-3-sphere scenes. Finals are also scored alone. Stops when every run has a final_model.
#   setsid nohup runs/eval_all/e1/auto_eval_e1.sh > runs/eval_all/e1/auto_eval.log 2>&1 < /dev/null &
cd "$(git -C "$(dirname "$0")" rev-parse --show-toplevel)/franka_sim"
RUNS=${RUNS:-"a1 a2 a3 m1"}
M=franka_sim/models
run_eval() {  # SCENE MODE  (list on stdin)
  docker exec -i -e SCENE=$1 -e MODE=$2 -e CONDS=nom franka_ros2 taskset -c 20-23 chrt -i 0 \
    bash /ros2_ws/src/franka_sim/runs/eval_all/e1/eval_e1.sh 4 | grep -v "^ALL DONE"
}
summary() {
  echo "=== $(date)"
  (cd runs/eval_all && REF=big11clip_full python3 d2/summary_d2.py '^(e1_|ens_d1b8_d2k2000_full|big11clip_full)' | grep -v "incomplete")
  echo "--- 2-3 spheres (traces/e1_multi)"
  (cd runs/eval_all && DIRS=e1_multi REF=none python3 d2/summary_d2.py 2>/dev/null | grep -v "incomplete")
}
while true; do
  list=""; done_all=1
  for r in $RUNS; do
    [ -f models/sac_e1_$r/final_model.zip ] || done_all=0
    # checkpoint_freq is divided by n_envs in train.py, so the files land at 499998, 999996, ...:
    # round to the nearest 500k for the tag. Newest first, so the latest checkpoints are scored first.
    for f in $(ls models/sac_e1_$r/checkpoints/sac_[0-9]*_steps.zip 2>/dev/null | awk -F'sac_' '{print $NF" "$0}' | sort -n -r | cut -d' ' -f2); do
      k=$(basename $f .zip | sed 's/sac_\([0-9]*\)_steps/\1/'); k=$(( (k + 250000) / 500000 * 500 ))
      list="$list
e1_${r}_${k}k|zip:franka_sim/$f|franka_sim/models/sac_e1_$r/config.yaml"
    done
    [ -f models/sac_e1_$r/final_model.zip ] && list="$list
e1_${r}_final|zip:franka_sim/models/sac_e1_$r/final_model.zip|franka_sim/models/sac_e1_$r/config.yaml"
  done
  if [ -n "$list" ]; then
    echo "$list" | run_eval one full
    echo "$list" | run_eval multi full
    summary
  fi
  [ $done_all = 1 ] && break
  sleep 1200
done
# Finals: alone as well (CBF-RL claims the correction cost makes the policy safe without the filter).
for r in $RUNS; do
  printf 'e1_%s_final|zip:%s/sac_e1_%s/final_model.zip|%s/sac_e1_%s/config.yaml\n' $r $M $r $M $r
done | run_eval one alone
(cd runs/eval_all && python3 d2/summary_d2.py '^(e1_.*final|ens_d2k2000_all12|big11clip)$')
echo "AUTO EVAL DONE $(date)"
