#!/usr/bin/env bash
# Run every Stage 7 verification script. Nothing in the repository is written except this
# directory's own results/ (inventory.py and test_traceability.py refresh their JSON/Markdown there).
#
# Usage (from anywhere):
#   docs/framework_study/tools/stage7_verification/run_all.sh            # host + container probe
#   docs/framework_study/tools/stage7_verification/run_all.sh --host-only
#
# Container settings: FS_CONTAINER (default franka_ros2). check_install_provenance.py only streams
# the install tree out of the container with `docker exec … tar -cf -` (read-only).
#
# Exit code: 0 if every script that ran passed; 1 if any failed; 2 if the container probe was skipped.
#
# Order matters: inventory.py and test_traceability.py refresh results/ first, because
# gen_file_index.py (appendix A) reads inventory.json and check_docs.py verifies that A is current.
# If check_docs.py reports appendix A or B stale, regenerate with the command in that generator's
# docstring (a documentation edit, not a code change) and rerun.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER="${FS_CONTAINER:-franka_ros2}"

HOST_SCRIPTS=(inventory.py test_traceability.py check_unread_functions.py check_guide_vs_code.py
              check_docs.py)

status=0
for s in "${HOST_SCRIPTS[@]}"; do
  echo "### $s"
  python3 -I -B "$HERE/$s" || status=1
  echo
done
if [ "${1:-}" = "--host-only" ]; then
  echo "container probe skipped (--host-only)"; exit $(( status ? 1 : 2 ))
fi
if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
  echo "container '$CONTAINER' is not running: check_install_provenance.py skipped"
  exit $(( status ? 1 : 2 ))
fi
echo "### check_install_provenance.py"
FS_CONTAINER="$CONTAINER" python3 -I -B "$HERE/check_install_provenance.py" || status=1
exit $status
