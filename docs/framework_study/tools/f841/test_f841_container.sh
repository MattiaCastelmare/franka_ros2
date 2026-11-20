#!/bin/bash
# F-8-41 container-side static equivalence (C4 e3_h2_bag.py, C5 s6b1_reconstruct_clearance.py).
# For each sealed H2 run whose cell is still in the container, runs the CORRECTED scripts on that cell
# into a fresh container scratch dir, copies the outputs to a host temp dir, and compares them with the
# run's sealed outputs: every sealed key must be identical (new keys are reported, not compared); the
# reconstruction npz arrays must be identical. Read-only on the cells and on the sealed evidence.
# Usage: test_f841_container.sh <out_dir (new)> <run_dir>...
set -u -o pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$1; shift
[ -e "$OUT" ] && { echo "exists: $OUT" >&2; exit 2; }
mkdir -p "$OUT"
C=franka_ros2
CS=/home/user/f841_test_$$
docker exec "$C" mkdir -p "$CS/scripts" || exit 1
for f in e3_h2_bag.py s6b1_reconstruct_clearance.py; do docker cp "$HERE/$f" "$C:$CS/scripts/$f" || exit 1; done
RC=0
for RUN in "$@"; do
  ID=$(basename "$RUN")
  CELLNAME=$(ls "$RUN/run/batch/cells")
  CROOT=$(python3 -I -c 'import json,sys; print(json.load(open(sys.argv[1]))["run_dir"])' "$RUN/run/batch/cells/$CELLNAME/run_manifest_sim.json")
  if ! docker exec "$C" test -d "$CROOT"; then echo "SKIP $ID: container cell $CROOT absent" | tee -a "$OUT/results.txt"; continue; fi
  docker exec -w /tmp -e PYTHONDONTWRITEBYTECODE=1 "$C" bash -c "source /ros2_ws/install/setup.bash >/dev/null 2>&1; \
     mkdir -p $CS/$ID && python3 -B $CS/scripts/s6b1_reconstruct_clearance.py $CROOT $CS/$ID > $CS/$ID/recon.log 2>&1 && \
     python3 -B $CS/scripts/e3_h2_bag.py $CROOT $CS/$ID $CS/$ID > $CS/$ID/bag.log 2>&1"
  r=$?
  mkdir -p "$OUT/$ID" && docker cp "$C:$CS/$ID/." "$OUT/$ID/" >/dev/null
  if [ $r != 0 ]; then echo "FAIL $ID: corrected scripts rc $r (see $OUT/$ID/*.log)" | tee -a "$OUT/results.txt"; RC=1; continue; fi
  python3 -I - "$RUN/analysis" "$OUT/$ID" <<'PY' | tee -a "$OUT/results.txt"
import json, sys, numpy as np
old_d, new_d = sys.argv[1], sys.argv[2]
NEW = {'ball_center', 'law', 'placed_max_dev_from_law_m'}
def strip(d):
    if isinstance(d, dict):
        return {k: strip(v) for k, v in d.items() if k not in NEW}
    if isinstance(d, list):
        return [strip(x) for x in d]
    return d
bad = []
for f in ('s6b1_reconstruction.json', 'h2_bag.json'):
    o, n = json.load(open(f'{old_d}/{f}')), json.load(open(f'{new_d}/{f}'))
    for k in o:
        if k in ('cell_dir', 'truth_geometry_module'):
            continue
        if strip(n.get(k)) != o[k]:
            bad.append(f'{f}:{k}')
zo, zn = np.load(f'{old_d}/s6b1_reconstruction.npz'), np.load(f'{new_d}/s6b1_reconstruction.npz')
for k in zo.files:
    if not np.array_equal(zo[k], zn[k], equal_nan=zo[k].dtype.kind == 'f'):
        bad.append(f'npz:{k}')
nr = json.load(open(f'{new_d}/s6b1_reconstruction.json'))['obstacle']
nb = json.load(open(f'{new_d}/h2_bag.json'))['mask_replay']
centers = {tuple(p['ball_center']) for p in nb['probes']}
print(('PASS' if not bad else 'FAIL'), new_d.rsplit('/', 1)[-1], 'differing:', bad,
      '| law', nr.get('law'), 'placed_max_dev_from_law_m', nr.get('placed_max_dev_from_law_m'),
      '| replay centres', sorted(centers))
PY
  grep -q "^PASS $ID" "$OUT/results.txt" || RC=1
done
docker exec "$C" rm -rf "$CS"
echo "container test rc $RC" | tee -a "$OUT/results.txt"
exit $RC
