// Copyright (c) 2026 Mattia – RT Velocity Executor Controller
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

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <franka_msgs/srv/set_full_collision_behavior.hpp>

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_rt_controllers {

/// @brief RT pure executor for Franka FR3.
///
/// Reads Float64MultiArray (7 joint velocities) from a single non-RT topic,
/// applies optional interpolation, rate limiting, and a final clamp — all
/// inside the 1 kHz update() loop.  No allocations, no locks, no logging
/// in the RT path.
///
/// Architecture:
///   Python node  ──topic──▶  RealtimeBuffer  ──readFromRT──▶  update()
///                                                              │
///                 (interp) → rate-limit → clamp → command_interfaces_[i].set_value()
///
class RtVelocityExecutorController
    : public controller_interface::ControllerInterface {
 public:
  static constexpr size_t kNumJoints = 7;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

 private:
  // ─── Aggregate written into RealtimeBuffer by the subscriber callback ─
  struct VelocityInput {
    std::array<double, kNumJoints> qdot{};
    int64_t stamp_ns{0};   ///< std::chrono::steady_clock nanoseconds
    bool received{false};  ///< true after the first valid message
  };

  // ─── Configuration (set once in on_configure, immutable during RT) ──
  std::string arm_id_;
  std::vector<std::string> joint_names_;  // pre-allocated, size == kNumJoints

  // Topic name
  std::string command_topic_;

  // Protection parameters  (0 / negative ⇒ feature disabled)
  double qdot_max_{0.0};             ///< rad/s – final clamp
  double max_accel_{0.0};            ///< rad/s² – rate limiter
  double timeout_threshold_s_{0.0};  ///< reserved – not used in RT path
  double timeout_ramp_s_{0.0};       ///< reserved – not used in RT path

  // ─── Realtime buffer (non-RT writes, RT lock-free reads) ────────────
  realtime_tools::RealtimeBuffer<VelocityInput> command_buf_;

  // ─── Subscriber (owned, created in on_configure) ──────────────────
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      command_sub_;

  // ─── RT-only state (touched exclusively in update()) ───────────────
  std::array<double, kNumJoints> prev_cmd_{};

  // ─── Configuration: collision behavior & interpolation ─────────────
  bool is_gazebo_{false};
  bool enable_interpolation_{false};

  // ─── Interpolation state (RT-only) ─────────────────────────────────
  //     When enable_interpolation is true, linearly ramps between
  //     consecutive samples across 1 kHz cycles (eliminates step changes).
  struct InterpState {
    std::array<double, kNumJoints> start{};    ///< value at ramp start
    std::array<double, kNumJoints> target{};   ///< newest sample value
    int64_t start_ns{0};                       ///< when ramp started
    int64_t duration_ns{5'000'000};            ///< inter-sample estimate (ns)
    int64_t last_sample_stamp_ns{0};           ///< stamp of last processed sample
  };
  InterpState command_interp_{};

  /// @brief Linear interpolation between consecutive samples (RT-safe).
  std::array<double, kNumJoints> interpolateInput(
      const VelocityInput& latest, InterpState& state, int64_t now_ns) const;

  // ─── Non-RT subscriber callback ────────────────────────────────────
  void commandCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  /// Monotonic nanosecond timestamp (RT-safe on Linux / vDSO).
  static int64_t steadyNowNs();
};

}  // namespace franka_rt_controllers
