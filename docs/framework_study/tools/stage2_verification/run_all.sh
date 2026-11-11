#!/usr/bin/env bash
# Run every Stage 2 verification script. Read-only: nothing in the repository is written.
#
# Usage (from anywhere):
#   docs/framework_study/tools/stage2_verification/run_all.sh            # host + container checks
#   docs/framework_study/tools/stage2_verification/run_all.sh --host-only
#
# Container settings (override with environment variables if your set-up differs):
#   FS_CONTAINER          name of the running ROS 2 container         (default: franka_ros2)
#   FS_REPO_IN_CONTAINER  where this repository is mounted inside it  (default: /ros2_ws/src)
#   FS_ROS_SETUP          setup file to source inside it              (default: /ros2_ws/install/setup.bash)
#
# Exit code: 0 if every script that ran passed; 1 if any failed; 2 if container checks were skipped.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../../.." && pwd)"
REL="${HERE#"$REPO"/}"
CONTAINER="${FS_CONTAINER:-franka_ros2}"
REPO_IN="${FS_REPO_IN_CONTAINER:-/ros2_ws/src}"
ROS_SETUP="${FS_ROS_SETUP:-/ros2_ws/install/setup.bash}"

HOST_SCRIPTS=(check_camera_extrinsics.py check_hand_mesh.py check_perception_numbers.py check_tracker.py)
CONTAINER_SCRIPTS=(check_kinematics_dynamics.py check_geometry_and_so3.py check_home_example.py
                   check_cp_labels.py check_held_entry.py)

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
  # Not `python3 -I`: the ROS environment is passed through PYTHONPATH. The scripts put the
  # repository source first on sys.path themselves and refuse to run against another copy.
  # `cd /tmp`: xacro writes its temporary URDF files in the container's working directory.
  docker exec "$CONTAINER" bash -c \
    "source '$ROS_SETUP' >/dev/null 2>&1 && cd /tmp && PYTHONDONTWRITEBYTECODE=1 python3 -B '$REPO_IN/$REL/$s'" || status=1
  echo
done
exit $status
