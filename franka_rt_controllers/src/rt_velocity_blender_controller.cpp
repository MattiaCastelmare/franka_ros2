// Copyright (c) 2026 Mattia – RT Velocity Blender Controller
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <franka_rt_controllers/rt_velocity_blender_controller.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

namespace franka_rt_controllers {

// ═══════════════════════════════════════════════════════════════════════════
//  Monotonic timestamp helper  (vDSO on Linux – no syscall, RT-safe)
// ═══════════════════════════════════════════════════════════════════════════

int64_t RtVelocityBlenderController::steadyNowNs() {
  return std::chrono::steady_clock::now().time_since_epoch().count();
}

// ═══════════════════════════════════════════════════════════════════════════
//  Linear interpolation between consecutive samples (RT-safe)
//
//  When a new sample arrives (detected by stamp_ns change), we ramp
//  linearly from the current interpolated value to the new sample over
//  one estimated inter-sample period.  After the ramp completes, we hold
//  at the latest value until the next sample.  No allocations, no locks.
// ═══════════════════════════════════════════════════════════════════════════

std::array<double, RtVelocityBlenderController::kNumJoints>
RtVelocityBlenderController::interpolateInput(
    const VelocityInput& latest,
    InterpState& state,
    int64_t now_ns) const {

  if (!latest.received) {
    return latest.qdot;  // all zeros – no data yet
  }

  // Detect new sample by comparing timestamps
  if (latest.stamp_ns != state.last_sample_stamp_ns) {
    if (state.last_sample_stamp_ns > 0 && state.duration_ns > 0) {
      // Compute current interpolated position as the new ramp start
      const double t = std::clamp(
          static_cast<double>(now_ns - state.start_ns) /
              static_cast<double>(state.duration_ns),
          0.0, 1.0);
      for (size_t i = 0; i < kNumJoints; ++i) {
        state.start[i] =
            state.start[i] + t * (state.target[i] - state.start[i]);
      }
      // Update inter-sample duration estimate from actual timing
      state.duration_ns = latest.stamp_ns - state.last_sample_stamp_ns;
    } else {
      // Very first sample – snap start to the sample value (no ramp)
      state.start = latest.qdot;
    }
    state.target = latest.qdot;
    state.start_ns = now_ns;
    state.last_sample_stamp_ns = latest.stamp_ns;
  }

  // Linear interpolation: t=0 → start, t=1 → target, t>1 → hold at target
  if (state.duration_ns > 0) {
    const double t = std::clamp(
        static_cast<double>(now_ns - state.start_ns) /
            static_cast<double>(state.duration_ns),
        0.0, 1.0);
    std::array<double, kNumJoints> result{};
    for (size_t i = 0; i < kNumJoints; ++i) {
      result[i] = state.start[i] + t * (state.target[i] - state.start[i]);
    }
    return result;
  }
  return latest.qdot;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Interface configuration
// ═══════════════════════════════════════════════════════════════════════════

controller_interface::InterfaceConfiguration
RtVelocityBlenderController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (size_t i = 0; i < kNumJoints; ++i) {
    config.names.push_back(joint_names_[i] + "/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration
RtVelocityBlenderController::state_interface_configuration() const {
  // We claim velocity state interfaces to read actual joint velocity on
  // activation (used to seed prev_cmd_ and avoid initial jumps).
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (size_t i = 0; i < kNumJoints; ++i) {
    config.names.push_back(joint_names_[i] + "/velocity");
  }
  return config;
}

// ═══════════════════════════════════════════════════════════════════════════
//  update()  –  THE REAL-TIME PATH
//
//  Guarantees:
//    • No heap allocations
//    • No mutex / lock
//    • No RCLCPP_* logging
//    • All arrays are fixed-size std::array<double,7>
// ═══════════════════════════════════════════════════════════════════════════

controller_interface::return_type RtVelocityBlenderController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {

  const int64_t now_ns = steadyNowNs();
  const double dt = period.seconds();

  // ── 1. Read latest inputs from realtime buffers (lock-free RT side) ──
  const auto tracking  = *tracking_buf_.readFromRT();
  const auto avoidance = *avoidance_buf_.readFromRT();
  const double alpha   = *alpha_buf_.readFromRT();

  // ── 1b. Optional linear interpolation (reduces sample-and-hold steps) ─
  //    When enabled, smoothly ramps between consecutive low-rate samples
  //    across the 1 kHz RT cycles, eliminating velocity step changes.
  const auto trk_qdot = enable_interpolation_
      ? interpolateInput(tracking, tracking_interp_, now_ns)
      : tracking.qdot;
  const auto avd_qdot = enable_interpolation_
      ? interpolateInput(avoidance, avoidance_interp_, now_ns)
      : avoidance.qdot;

  // ── 2. Compute per-input timeout scaling  ────────────────────────────
  //    If timeout is not configured (threshold or ramp ≤ 0), scale = 1.0
  //    for received inputs, 0.0 for never-received inputs.
  double tracking_scale  = tracking.received  ? 1.0 : 0.0;
  double avoidance_scale = avoidance.received ? 1.0 : 0.0;

  const bool timeout_enabled =
      (timeout_threshold_s_ > 0.0) && (timeout_ramp_s_ > 0.0);

  if (timeout_enabled) {
    if (tracking.received) {
      const double age_s =
          static_cast<double>(now_ns - tracking.stamp_ns) * 1e-9;
      if (age_s > timeout_threshold_s_) {
        const double overtime = age_s - timeout_threshold_s_;
        tracking_scale = std::max(0.0, 1.0 - overtime / timeout_ramp_s_);
      }
    }
    if (avoidance.received) {
      const double age_s =
          static_cast<double>(now_ns - avoidance.stamp_ns) * 1e-9;
      if (age_s > timeout_threshold_s_) {
        const double overtime = age_s - timeout_threshold_s_;
        avoidance_scale = std::max(0.0, 1.0 - overtime / timeout_ramp_s_);
      }
    }
  }

  // ── 3. Blend ─────────────────────────────────────────────────────────
  //    qdot_cmd = alpha * tracking + (1-alpha) * avoidance
  //    Each input is further scaled by its timeout factor.
  std::array<double, kNumJoints> qdot_cmd{};
  const double w_trk = alpha * tracking_scale;
  const double w_avd = (1.0 - alpha) * avoidance_scale;

  for (size_t i = 0; i < kNumJoints; ++i) {
    qdot_cmd[i] = w_trk * trk_qdot[i] + w_avd * avd_qdot[i];
  }

  // ── 4. Rate limiter  (optional – only when max_accel_ > 0) ──────────
  //    Limits the per-joint change in velocity between consecutive cycles
  //    to max_accel_ * dt, ensuring bounded jerk at the hardware level.
  //    Active from the FIRST cycle: prev_cmd_ is seeded from actual joint
  //    velocities in on_activate(), so no initial discontinuity.
  if (max_accel_ > 0.0) {
    const double max_delta = max_accel_ * dt;
    for (size_t i = 0; i < kNumJoints; ++i) {
      const double delta = qdot_cmd[i] - prev_cmd_[i];
      if (delta > max_delta) {
        qdot_cmd[i] = prev_cmd_[i] + max_delta;
      } else if (delta < -max_delta) {
        qdot_cmd[i] = prev_cmd_[i] - max_delta;
      }
    }
  }

  // ── 5. Final clamp  (single, authoritative – only when qdot_max_ > 0)
  if (qdot_max_ > 0.0) {
    for (size_t i = 0; i < kNumJoints; ++i) {
      qdot_cmd[i] = std::clamp(qdot_cmd[i], -qdot_max_, qdot_max_);
    }
  }

  // ── 6. Write to hardware command interfaces ──────────────────────────
  for (size_t i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(qdot_cmd[i]);
  }

  // ── 7. Store for next cycle's rate limiter ───────────────────────────
  prev_cmd_ = qdot_cmd;
  first_update_ = false;

  return controller_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Lifecycle: on_init
// ═══════════════════════════════════════════════════════════════════════════

CallbackReturn RtVelocityBlenderController::on_init() {
  try {
    // All parameters are declared here; defaults disable optional features.
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::vector<std::string>>(
        "joints", std::vector<std::string>{});

    auto_declare<std::string>("tracking_topic", "tracking_qdot");
    auto_declare<std::string>("avoidance_topic", "avoidance_qdot");
    auto_declare<std::string>("alpha_topic", "blend_alpha");

    // Blend weight: 1.0 = 100% tracking, 0.0 = 100% avoidance
    auto_declare<double>("alpha", 0.5);

    // Protection parameters – 0 or negative = DISABLED
    auto_declare<double>("qdot_max", 0.0);
    auto_declare<double>("max_accel", 0.0);
    auto_declare<double>("timeout_threshold_s", 0.0);
    auto_declare<double>("timeout_ramp_s", 0.0);

    // Collision behavior (same as official Franka controllers).
    // Set true when running in Gazebo – skips the firmware service call.
    auto_declare<bool>("gazebo", false);

    // Interpolation: linearly ramp between consecutive low-rate samples
    // in the 1 kHz RT loop (eliminates sample-and-hold velocity steps).
    // Default false = existing behavior (sample-and-hold).
    auto_declare<bool>("enable_interpolation", false);

  } catch (const std::exception& e) {
    fprintf(stderr, "Exception during on_init: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Lifecycle: on_configure
// ═══════════════════════════════════════════════════════════════════════════

CallbackReturn RtVelocityBlenderController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  auto logger = get_node()->get_logger();

  // ── Read scalar parameters ────────────────────────────────────────
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  const auto joints_param =
      get_node()->get_parameter("joints").as_string_array();
  if (joints_param.empty()) {
    // Default: arm_id_joint1 … arm_id_joint7
    joint_names_.clear();
    joint_names_.reserve(kNumJoints);
    for (size_t i = 1; i <= kNumJoints; ++i) {
      joint_names_.push_back(arm_id_ + "_joint" + std::to_string(i));
    }
    RCLCPP_INFO(logger, "Using default joint names: %s_joint[1..7]",
                arm_id_.c_str());
  } else if (joints_param.size() != kNumJoints) {
    RCLCPP_ERROR(logger, "Expected %zu joints, got %zu",
                 kNumJoints, joints_param.size());
    return CallbackReturn::ERROR;
  } else {
    joint_names_.assign(joints_param.begin(), joints_param.end());
  }

  tracking_topic_  = get_node()->get_parameter("tracking_topic").as_string();
  avoidance_topic_ = get_node()->get_parameter("avoidance_topic").as_string();
  alpha_topic_     = get_node()->get_parameter("alpha_topic").as_string();
  default_alpha_   = get_node()->get_parameter("alpha").as_double();
  qdot_max_            = get_node()->get_parameter("qdot_max").as_double();
  max_accel_           = get_node()->get_parameter("max_accel").as_double();
  timeout_threshold_s_ = get_node()->get_parameter("timeout_threshold_s").as_double();
  timeout_ramp_s_      = get_node()->get_parameter("timeout_ramp_s").as_double();  is_gazebo_           = get_node()->get_parameter("gazebo").as_bool();
  enable_interpolation_ = get_node()->get_parameter("enable_interpolation").as_bool();
  // ── Validate & warn ───────────────────────────────────────────────
  if (qdot_max_ <= 0.0) {
    RCLCPP_WARN(logger,
                "qdot_max <= 0 → velocity clamping DISABLED. "
                "Set a positive value in YAML for safety.");
  }
  if (max_accel_ <= 0.0) {
    RCLCPP_WARN(logger,
                "max_accel <= 0 → rate limiting DISABLED. "
                "Set a positive value in YAML for smoothness.");
  }
  if (timeout_threshold_s_ <= 0.0 || timeout_ramp_s_ <= 0.0) {
    RCLCPP_WARN(logger,
                "timeout_threshold_s or timeout_ramp_s <= 0 → "
                "smooth timeout ramp DISABLED. Last received command "
                "will be held indefinitely. Set both to enable safe "
                "automatic ramp-to-zero.");
  }
  // ── SetFullCollisionBehavior (matches official Franka controllers) ─
  //    Raises collision thresholds so the firmware does not trigger
  //    spurious reflex aborts during normal velocity control.
  //    Thresholds are identical to DefaultRobotBehavior in
  //    franka_example_controllers/default_robot_behavior_utils.hpp.
  //    Skipped in Gazebo (no firmware service available).
  if (!is_gazebo_) {
    auto client =
        get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
            "service_server/set_full_collision_behavior");
    auto request = std::make_shared<
        franka_msgs::srv::SetFullCollisionBehavior::Request>();
    request->lower_torque_thresholds_nominal =
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0};
    request->upper_torque_thresholds_nominal =
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0};
    request->lower_torque_thresholds_acceleration =
        {25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0};
    request->upper_torque_thresholds_acceleration =
        {35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0};
    request->lower_force_thresholds_nominal =
        {30.0, 30.0, 30.0, 25.0, 25.0, 25.0};
    request->upper_force_thresholds_nominal =
        {40.0, 40.0, 40.0, 35.0, 35.0, 35.0};
    request->lower_force_thresholds_acceleration =
        {30.0, 30.0, 30.0, 25.0, 25.0, 25.0};
    request->upper_force_thresholds_acceleration =
        {40.0, 40.0, 40.0, 35.0, 35.0, 35.0};

    auto future_result = client->async_send_request(request);
    future_result.wait_for(std::chrono::milliseconds(1000));
    auto response = future_result.get();
    if (!response || !response->success) {
      RCLCPP_FATAL(logger, "Failed to set default collision behavior.");
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(logger,
                "Default collision behavior set "
                "(same thresholds as official Franka controllers).");
  } else {
    RCLCPP_INFO(logger,
                "Gazebo mode \u2014 skipping SetFullCollisionBehavior.");
  }
  // ── Initialize realtime buffers ───────────────────────────────────
  VelocityInput zero_input{};
  tracking_buf_.writeFromNonRT(zero_input);
  avoidance_buf_.writeFromNonRT(zero_input);
  alpha_buf_.writeFromNonRT(default_alpha_);

  // ── Create subscribers (non-RT callbacks, BestEffort depth=1) ─────
  //    SensorDataQoS = BestEffort + Volatile + KeepLast.  Minimises
  //    buffering and avoids reliability handshake latency.
  tracking_sub_ =
      get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
          tracking_topic_, rclcpp::SensorDataQoS().keep_last(1),
          [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            trackingCb(msg);
          });

  avoidance_sub_ =
      get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
          avoidance_topic_, rclcpp::SensorDataQoS().keep_last(1),
          [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            avoidanceCb(msg);
          });

  alpha_sub_ = get_node()->create_subscription<std_msgs::msg::Float64>(
      alpha_topic_, rclcpp::SensorDataQoS().keep_last(1),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        alphaCb(msg);
      });

  // ── Parameter event callback (allows `ros2 param set … alpha 0.5`) ─
  param_cb_handle_ = get_node()->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params)
          -> rcl_interfaces::msg::SetParametersResult {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& p : params) {
          if (p.get_name() == "alpha") {
            const double v = std::clamp(p.as_double(), 0.0, 1.0);
            alpha_buf_.writeFromNonRT(v);
          }
        }
        return result;
      });

  // ── Summary log ───────────────────────────────────────────────────
  RCLCPP_INFO(logger, "╔══ RtVelocityBlenderController configured ══╗");
  RCLCPP_INFO(logger, "  arm_id     : %s", arm_id_.c_str());
  for (size_t i = 0; i < kNumJoints; ++i) {
    RCLCPP_INFO(logger, "  joint[%zu]   : %s", i, joint_names_[i].c_str());
  }
  RCLCPP_INFO(logger, "  tracking   : %s", tracking_topic_.c_str());
  RCLCPP_INFO(logger, "  avoidance  : %s", avoidance_topic_.c_str());
  RCLCPP_INFO(logger, "  alpha_topic: %s", alpha_topic_.c_str());
  RCLCPP_INFO(logger, "  alpha      : %.3f", default_alpha_);
  RCLCPP_INFO(logger, "  qdot_max   : %.4f %s", qdot_max_,
              qdot_max_ > 0 ? "[ACTIVE]" : "[DISABLED]");
  RCLCPP_INFO(logger, "  max_accel  : %.4f %s", max_accel_,
              max_accel_ > 0 ? "[ACTIVE]" : "[DISABLED]");
  RCLCPP_INFO(logger, "  timeout    : thresh=%.3fs ramp=%.3fs %s",
              timeout_threshold_s_, timeout_ramp_s_,
              (timeout_threshold_s_ > 0 && timeout_ramp_s_ > 0)
                  ? "[ACTIVE]"
                  : "[DISABLED]");  RCLCPP_INFO(logger, "  gazebo     : %s",
              is_gazebo_ ? "true (collision behavior skipped)" : "false");
  RCLCPP_INFO(logger, "  interpolation: %s",
              enable_interpolation_ ? "[ACTIVE]" : "[DISABLED]");
  RCLCPP_INFO(logger, "  subscriber QoS: SensorData (BestEffort, depth=1)");  RCLCPP_INFO(logger, "╚════════════════════════════════════════════╝");

  return CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Lifecycle: on_activate
// ═══════════════════════════════════════════════════════════════════════════

CallbackReturn RtVelocityBlenderController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  // Seed prev_cmd_ from current joint velocities (state interfaces) to
  // avoid a velocity discontinuity if the robot is already moving.
  for (size_t i = 0; i < kNumJoints; ++i) {
    prev_cmd_[i] = state_interfaces_[i].get_value();
  }
  first_update_ = true;

  // Reset buffers to zero
  VelocityInput zero_input{};
  tracking_buf_.writeFromNonRT(zero_input);
  avoidance_buf_.writeFromNonRT(zero_input);
  alpha_buf_.writeFromNonRT(default_alpha_);

  // Reset interpolation state
  tracking_interp_ = InterpState{};
  avoidance_interp_ = InterpState{};

  RCLCPP_INFO(get_node()->get_logger(),
              "RtVelocityBlenderController ACTIVATED – writing velocity "
              "commands at controller_manager update_rate.");

  return CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Lifecycle: on_deactivate
// ═══════════════════════════════════════════════════════════════════════════

CallbackReturn RtVelocityBlenderController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  // Send zero velocity to all joints before releasing the interfaces.
  for (size_t i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(0.0);
  }

  RCLCPP_INFO(get_node()->get_logger(),
              "RtVelocityBlenderController DEACTIVATED → zero velocity.");

  return CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Subscriber callbacks  (non-RT thread – may allocate, may log)
// ═══════════════════════════════════════════════════════════════════════════

void RtVelocityBlenderController::trackingCb(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {

  if (msg->data.size() != kNumJoints) {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "tracking_qdot: expected %zu values, got %zu – IGNORING",
        kNumJoints, msg->data.size());
    return;
  }
  VelocityInput input;
  std::copy_n(msg->data.begin(), kNumJoints, input.qdot.begin());
  input.stamp_ns = steadyNowNs();
  input.received = true;
  tracking_buf_.writeFromNonRT(input);
}

void RtVelocityBlenderController::avoidanceCb(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {

  if (msg->data.size() != kNumJoints) {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "avoidance_qdot: expected %zu values, got %zu – IGNORING",
        kNumJoints, msg->data.size());
    return;
  }
  VelocityInput input;
  std::copy_n(msg->data.begin(), kNumJoints, input.qdot.begin());
  input.stamp_ns = steadyNowNs();
  input.received = true;
  avoidance_buf_.writeFromNonRT(input);
}

void RtVelocityBlenderController::alphaCb(
    const std_msgs::msg::Float64::SharedPtr msg) {
  const double val = std::clamp(msg->data, 0.0, 1.0);
  alpha_buf_.writeFromNonRT(val);
}

}  // namespace franka_rt_controllers

// ═══════════════════════════════════════════════════════════════════════════
//  Plugin registration
// ═══════════════════════════════════════════════════════════════════════════
PLUGINLIB_EXPORT_CLASS(franka_rt_controllers::RtVelocityBlenderController,
                       controller_interface::ControllerInterface)
