#!/bin/bash
# =============================================================================
# check_pipeline.sh — franka_simulation pipeline validator
#
# Usage:
#   ./check_pipeline.sh velocity
#   ./check_pipeline.sh acceleration
#   ./check_pipeline.sh torque
#   ./check_pipeline.sh cbf
#
# Checks controllers, topics, and publish rates for each pipeline.
# Run AFTER the corresponding launch file is up and stable.
# =============================================================================

set -euo pipefail

PIPELINE="${1:-}"
PASS=0
FAIL=0
WARN=0

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

_pass() { echo -e "${GREEN}[PASS]${NC} $1"; ((PASS++)); }
_fail() { echo -e "${RED}[FAIL]${NC} $1"; ((FAIL++)); }
_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; ((WARN++)); }
_info() { echo -e "      $1"; }

_check_topic_exists() {
    local topic="$1"
    local label="${2:-$topic}"
    if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
        _pass "Topic exists: ${label}"
        return 0
    else
        _fail "Topic missing: ${label}"
        return 1
    fi
}

_check_topic_active() {
    local topic="$1"
    local min_hz="${2:-1.0}"
    local label="${3:-$topic}"
    local hz
    hz=$(timeout 3 ros2 topic hz --window 10 "${topic}" 2>/dev/null | grep 'average rate' | awk '{print $3}' | tr -d ':' || echo "0")
    if [ -z "$hz" ] || [ "$hz" = "0" ]; then
        _warn "No messages on: ${label} (publisher not active yet?)"
    else
        if awk "BEGIN {exit !($hz >= $min_hz)}"; then
            _pass "Topic active at ${hz} Hz (≥${min_hz}): ${label}"
        else
            _fail "Topic rate ${hz} Hz < ${min_hz} Hz: ${label}"
        fi
    fi
}

_check_controller_active() {
    local ctrl="$1"
    local state
    state=$(ros2 control list_controllers 2>/dev/null | grep "^${ctrl}" | awk '{print $2}' || echo "")
    if echo "$state" | grep -qi "active"; then
        _pass "Controller active: ${ctrl}"
    elif echo "$state" | grep -qi "inactive"; then
        _warn "Controller exists but INACTIVE: ${ctrl}"
    else
        _fail "Controller not found: ${ctrl}"
    fi
}

_check_controller_manager() {
    if ros2 control list_controllers 2>/dev/null >/dev/null; then
        _pass "controller_manager is reachable"
    else
        _fail "controller_manager not reachable — is the launch running?"
    fi
}

# =============================================================================
echo "============================================================"
echo "  franka_simulation Pipeline Check: ${PIPELINE:-<none>}"
echo "============================================================"

if [ -z "$PIPELINE" ]; then
    echo "Usage: $0 {velocity|acceleration|torque|cbf}"
    exit 1
fi

echo ""
echo "--- Common checks ---"
_check_controller_manager
_check_topic_exists /joint_states "/joint_states (JSB)"
_check_topic_exists /robot_description "/robot_description (RSP)"
_check_topic_active /joint_states 50.0 "/joint_states (rate)"

echo ""
echo "--- Pipeline-specific checks: ${PIPELINE} ---"

case "$PIPELINE" in

  velocity)
    _check_controller_active fr3_velocity_controller
    _check_topic_exists /fr3_velocity_controller/commands
    _check_topic_active /fr3_velocity_controller/commands 1.0 "/fr3_velocity_controller/commands"
    echo ""
    _info "Manual test command:"
    _info "  ros2 topic pub --once /fr3_velocity_controller/commands std_msgs/Float64MultiArray \"{data: [0.1,0,0,0,0,0,0]}\""
    _info "Expected: fr3_joint1 moves at 0.1 rad/s in Gazebo and RViz"
    ;;

  acceleration)
    _check_controller_active fr3_velocity_controller
    _check_topic_exists /sim_acceleration_bridge/accel_cmd
    _check_topic_exists /fr3_velocity_controller/commands
    _check_topic_active /fr3_velocity_controller/commands 10.0 "/fr3_velocity_controller/commands (bridge output)"
    _check_topic_active /sim_acceleration_bridge/accel_cmd 1.0 "/sim_acceleration_bridge/accel_cmd"
    echo ""
    _info "Manual test command:"
    _info "  ros2 topic pub --once /sim_acceleration_bridge/accel_cmd std_msgs/Float64MultiArray \"{data: [0.2,0,0,0,0,0,0]}\""
    _info "Expected: fr3_joint1 accelerates and then the bridge clamps to qdot_max"
    ;;

  torque)
    _check_controller_active fr3_effort_controller
    _check_topic_exists /fr3_effort_controller/commands
    _check_topic_active /fr3_effort_controller/commands 1.0 "/fr3_effort_controller/commands"
    echo ""
    _info "Manual test command (SAFE — small torque):"
    _info "  ros2 topic pub --once /fr3_effort_controller/commands std_msgs/Float64MultiArray \"{data: [0,0,0,0,0,0,0]}\""
    _info "Expected: robot holds position (Gazebo physics provides gravity equilibrium)"
    _warn "If robot falls: effort interface may not be active — check gazebo_effort:=true in launch"
    ;;

  cbf)
    _check_controller_active fr3_velocity_controller
    _check_topic_exists /fr3_velocity_controller/commands
    _check_topic_exists /avoidance/min_distance      || _warn "CBF avoidance not running yet"
    _check_topic_exists /avoidance/closest_constraint || _warn "CBF avoidance not running yet"
    _check_topic_exists /robot_capsules_markers      || _warn "Capsule markers not active"
    _check_topic_exists /velocity_blender/trajectory || _warn "Velocity blender trajectory not active"
    _check_topic_active /fr3_velocity_controller/commands 10.0 "/fr3_velocity_controller/commands (blender output)"
    echo ""
    _info "Expected in RViz: robot capsule markers (cylinders) visible on arm links"
    _info "Expected: avoidance controller publishing at ~100 Hz"
    ;;

  *)
    echo "Unknown pipeline: ${PIPELINE}"
    echo "Valid options: velocity, acceleration, torque, cbf"
    exit 1
    ;;
esac

# =============================================================================
echo ""
echo "============================================================"
echo "  Results: PASS=${PASS}  FAIL=${FAIL}  WARN=${WARN}"
echo "============================================================"

if [ "$FAIL" -gt 0 ]; then
    exit 1
fi
exit 0
