#!/bin/bash
# =============================================================================
# check_topics.sh — franka_experiments pipeline topic validator
#
# Usage:
#   ./check_topics.sh velocity
#   ./check_topics.sh acceleration
#   ./check_topics.sh torque
#   ./check_topics.sh oscbf
#   ./check_topics.sh oscbf_obstacle   (Phase 3 — with real_time_distance)
#   ./check_topics.sh rl                (Safe-RL ONNX policy pipeline)
#
# Run AFTER the corresponding launch file is stable (controllers spawned).
# Works both with real hardware and fake hardware (use_fake_hardware:=true).
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

# NOTE: `PASS=$((PASS+1))`, never `((PASS++))`. The post-increment form returns
# the OLD value as its exit status, so the very first `((PASS++))` (PASS=0)
# returns 1 and `set -e` aborts the whole script after one successful check.
_pass() { echo -e "${GREEN}[PASS]${NC} $1"; PASS=$((PASS + 1)); }
_fail() { echo -e "${RED}[FAIL]${NC} $1"; FAIL=$((FAIL + 1)); }
_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; WARN=$((WARN + 1)); }
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
    # `ros2 topic hz` prints one "average rate" line per window, so keep only the
    # LAST one — feeding the whole multi-line string to awk below made every
    # rate check fail with a syntax error even on a perfectly healthy topic.
    # `|| true` is required: `timeout` exits 124 (and grep exits 1 on a silent
    # topic), which with `set -e -o pipefail` would abort the whole script
    # instead of reporting a single failed check.
    hz=$(timeout 3 ros2 topic hz --window 10 "${topic}" 2>/dev/null \
         | grep 'average rate' | tail -1 | awk '{print $3}' | tr -d ':' || true)
    hz="${hz:-0}"
    if [ -z "$hz" ] || [ "$hz" = "0" ]; then
        _warn "No messages on: ${label} (node may still be in warmup)"
    else
        if awk "BEGIN {exit !($hz >= $min_hz)}"; then
            _pass "Topic active at ${hz} Hz (≥${min_hz}): ${label}"
        else
            _fail "Topic rate ${hz} Hz < ${min_hz} Hz: ${label}"
        fi
    fi
}

_check_node_exists() {
    local node="$1"
    local label="${2:-$node}"
    if ros2 node list 2>/dev/null | grep -q "${node}"; then
        _pass "Node running: ${label}"
    else
        _fail "Node not found: ${label}"
    fi
}

# Detect namespace from the running joint_states topic.
# The bringup also publishes /<ns>/franka/joint_states (the driver's raw stream),
# so take the SHORTEST prefix: /NS_1, not /NS_1/franka — every command topic
# (qddot_nom, torque_cmd, …) lives under the short one.
_detect_namespace() {
    local ns
    ns=$(ros2 topic list 2>/dev/null | grep '/joint_states$' \
         | sed 's|/joint_states$||' | awk '{print length"\t"$0}' \
         | sort -n | head -1 | cut -f2- || echo "")
    echo "${ns}"
}

echo "============================================================"
echo "  franka_experiments Pipeline Check: ${PIPELINE:-<none>}"
echo "============================================================"

if [ -z "$PIPELINE" ]; then
    echo "Usage: $0 {velocity|acceleration|rl|torque|oscbf|oscbf_obstacle}"
    exit 1
fi

NS=$(_detect_namespace)
if [ -n "$NS" ]; then
    echo "  Detected namespace: ${NS}"
else
    echo "  No namespace detected — using root"
fi

echo ""
echo "--- Common checks ---"
_check_topic_exists "${NS}/joint_states" "joint_states"
# 30 Hz = the `joint_state_rate` configured in franka.config.yaml, and roughly
# what mock hardware delivers. The real joint_state_broadcaster runs at 1 kHz;
# a 100 Hz threshold here only produced a false FAIL on every fake-HW test.
_check_topic_active "${NS}/joint_states" 30.0 "joint_states rate"

echo ""
echo "--- Pipeline-specific checks: ${PIPELINE} ---"

case "$PIPELINE" in

  velocity)
    _check_node_exists "ee_pentagon_velocity_commander" || true
    _check_topic_exists "${NS}/qdot_cmd" || \
      _check_topic_exists "/fr3_velocity_controller/commands" "velocity command topic"
    _check_topic_active "${NS}/qdot_cmd" 50.0 "qdot_cmd" 2>/dev/null || \
      _check_topic_active "/fr3_velocity_controller/commands" 50.0 "velocity commands"
    echo ""
    _info "Expected: ee_pentagon_velocity_commander publishing at ~200 Hz"
    _info "Trajectory: pentagon in EE space, radius 0.2 m"
    ;;

  acceleration)
    _check_node_exists "pentagon_qddot_commander" || true
    _check_node_exists "cbf_safety_filter" || true
    _check_node_exists "qddot_to_torque" || true
    _check_topic_exists "${NS}/qddot_nom"  "qddot_nom (commander output)"
    _check_topic_exists "${NS}/qddot_safe" "qddot_safe (CBF filter output)"
    _check_topic_exists "${NS}/torque_cmd" "torque_cmd (dynamics output)"
    _check_topic_active "${NS}/qddot_nom"  50.0 "qddot_nom"
    _check_topic_active "${NS}/qddot_safe" 50.0 "qddot_safe"
    _check_topic_active "${NS}/torque_cmd" 50.0 "torque_cmd"
    echo ""
    _info "Pipeline: pentagon_qddot_commander → /NS/qddot_nom"
    _info "         cbf_safety_filter (HOCBF-QP) → /NS/qddot_safe"
    _info "         qddot_to_torque (τ = M·q̈ + C·q̇) → /NS/torque_cmd"
    _info "         rt_torque_controller → hardware"
    ;;

  rl)
    # Same accel pipeline, but the q̈_nom source is the ONNX Safe-RL policy
    # (motion_source:=rl). See franka_sim_to_real_roadmap.md Step 3.
    _check_node_exists "rl_policy_commander" || true
    _check_node_exists "cbf_safety_filter" || true
    _check_node_exists "qddot_to_torque" || true
    _check_topic_exists "${NS}/qddot_nom"  "qddot_nom (policy output)"
    _check_topic_exists "${NS}/qddot_safe" "qddot_safe (CBF filter output)"
    _check_topic_exists "${NS}/torque_cmd" "torque_cmd (dynamics output)"
    _check_topic_exists "${NS}/rl_status"  "rl_status (policy diagnostics)"
    _check_topic_active "${NS}/qddot_nom"  50.0 "qddot_nom"
    _check_topic_active "${NS}/qddot_safe" 50.0 "qddot_safe"
    _check_topic_active "${NS}/torque_cmd" 50.0 "torque_cmd"
    _check_topic_active "${NS}/rl_status"  50.0 "rl_status"
    echo ""
    _info "Pipeline: rl_policy_commander (ONNX policy) → /NS/qddot_nom"
    _info "         cbf_safety_filter (HOCBF-QP) → /NS/qddot_safe"
    _info "         qddot_to_torque → /NS/torque_cmd → rt_torque_controller"
    _info "rl_status = [infer_ms, tick_ms, d_min, dist, target_idx, gate]"
    _info "gate: 0=commanding  1=warm-up  2=stale joint state  3=stale distances"
    _info "During the first warmup_s (3 s) an all-zero qddot_nom is EXPECTED."
    ;;

  torque)
    _check_node_exists "pentagon_torque_commander" || true
    _check_topic_exists "${NS}/torque_cmd" "torque_cmd"
    _check_topic_active "${NS}/torque_cmd" 50.0 "torque_cmd rate"
    echo ""
    _info "Pipeline: pentagon_torque_commander → /NS/torque_cmd → rt_torque_controller"
    _info "Gravity compensation: done by Franka firmware (not by commander)"
    ;;

  oscbf)
    _check_node_exists "pentagon_torque_commander" || true
    _check_node_exists "oscbf_filter" || true
    _check_topic_exists "${NS}/torque_cmd"  "torque_cmd (commander output)"
    _check_topic_exists "${NS}/torque_safe" "torque_safe (OSCBF filter output)"
    _check_topic_active "${NS}/torque_cmd"  50.0 "torque_cmd"
    _check_topic_active "${NS}/torque_safe" 50.0 "torque_safe"
    echo ""
    _info "Pipeline: pentagon_torque_commander → /NS/torque_cmd"
    _info "         oscbf_filter (OSCBF-QP) → /NS/torque_safe"
    _info "         rt_torque_controller → hardware (listens on torque_safe)"
    _info "Phase check: if bypass_filter:=true, torque_safe = torque_cmd (no QP)"
    ;;

  oscbf_obstacle)
    _check_node_exists "pentagon_torque_commander" || true
    _check_node_exists "oscbf_filter" || true
    _check_node_exists "real_time_distance" || true
    _check_topic_exists "${NS}/torque_cmd"
    _check_topic_exists "${NS}/torque_safe"
    _check_topic_exists "/cbf/per_link_distances" "per_link_distances (obstacle CBF)"
    _check_topic_active "${NS}/torque_cmd"  50.0 "torque_cmd"
    _check_topic_active "${NS}/torque_safe" 50.0 "torque_safe"
    _check_topic_active "/cbf/per_link_distances" 5.0 "per_link_distances"
    echo ""
    _info "Full OSCBF Phase 3: joint-position CBF + obstacle-avoidance CBF"
    _info "Requires: camera + real_time_distance node"
    ;;

  *)
    echo "Unknown pipeline: ${PIPELINE}"
    echo "Valid: velocity | acceleration | rl | torque | oscbf | oscbf_obstacle"
    exit 1
    ;;
esac

echo ""
echo "============================================================"
echo "  Results: PASS=${PASS}  FAIL=${FAIL}  WARN=${WARN}"
echo "============================================================"

if [ "$FAIL" -gt 0 ]; then
    exit 1
fi
exit 0
