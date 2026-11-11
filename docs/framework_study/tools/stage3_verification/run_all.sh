#!/usr/bin/env bash
# Run every Stage 3 verification script. Read-only: nothing in the repository is written.
#
# Usage (from anywhere):
#   docs/framework_study/tools/stage3_verification/run_all.sh            # host + container checks
#   docs/framework_study/tools/stage3_verification/run_all.sh --host-only
#
# Container settings (same as stage2_verification/run_all.sh):
#   FS_CONTAINER (default franka_ros2), FS_REPO_IN_CONTAINER (default /ros2_ws/src),
#   FS_ROS_SETUP (default /ros2_ws/install/setup.bash)
#
# Exit code: 0 if every script that ran passed; 1 if any failed; 2 if container checks were skipped.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../../.." && pwd)"
REL="${HERE#"$REPO"/}"
CONTAINER="${FS_CONTAINER:-franka_ros2}"
REPO_IN="${FS_REPO_IN_CONTAINER:-/ros2_ws/src}"
ROS_SETUP="${FS_ROS_SETUP:-/ros2_ws/install/setup.bash}"

HOST_SCRIPTS=(effective_params.py check_timing_governor.py check_hocbf_gains.py check_nominal_shaping.py)
CONTAINER_SCRIPTS=(check_held_row.py check_cp_label_rows.py)

status=0
for s in "${HOST_SCRIPTS[@]}"; do
  python3 -I -B "$HERE/$s" || status=1
  echo
done
if [ "${1:-}" = "--host-only" ]; then
  echo "container checks skipped (--host-only)"; exit $(( status ? 1 : 2 ))
fi
if ! docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
  echo "container '$CONTAINER' is not running: container checks skipped"; exit $(( status ? 1 : 2 ))
fi
for s in "${CONTAINER_SCRIPTS[@]}"; do
  # The scripts put the repository source first on sys.path and refuse another copy (F-2-2).
  # `cd /tmp`: xacro writes temporary URDF files in the working directory.
  docker exec "$CONTAINER" bash -c \
    "source '$ROS_SETUP' >/dev/null 2>&1 && cd /tmp && PYTHONDONTWRITEBYTECODE=1 python3 -B '$REPO_IN/$REL/$s'" || status=1
  echo
done
exit $status
