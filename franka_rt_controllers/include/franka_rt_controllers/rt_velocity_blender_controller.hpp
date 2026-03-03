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
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <franka_msgs/srv/set_full_collision_behavior.hpp>

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_rt_controllers {

/// @brief Real-time velocity blending controller for Franka FR3.
///
/// Subscribes to two non-RT velocity topics (tracking + avoidance) and one
/// alpha weight (topic or parameter), then performs blending, optional rate
/// limiting, optional smooth timeout ramp, and a single final clamp — all
/// inside the 1 kHz update() loop.  No allocations, no locks, no logging
/// in the RT path.
///
/// Architecture:
///   Python nodes  ──topic──▶  RealtimeBuffer  ──readFromRT──▶  update()
///                                                               │
///     blend → rate-limit → clamp → command_interfaces_[i].set_value()
///
class RtVelocityBlenderController
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
  // ─── Aggregate written into RealtimeBuffer by subscriber callbacks ──
  struct VelocityInput {
    std::array<double, kNumJoints> qdot{};
    int64_t stamp_ns{0};   ///< std::chrono::steady_clock nanoseconds
    bool received{false};  ///< true after the first valid message
  };

  // ─── Configuration (set once in on_configure, immutable during RT) ──
  std::string arm_id_;
  std::vector<std::string> joint_names_;  // pre-allocated, size == kNumJoints

  // Topic names
  std::string tracking_topic_;
  std::string avoidance_topic_;
  std::string alpha_topic_;

  // Protection parameters  (0 / negative ⇒ feature disabled)
  double qdot_max_{0.0};             ///< rad/s – final clamp
  double max_accel_{0.0};            ///< rad/s² – rate limiter
  double timeout_threshold_s_{0.0};  ///< seconds before ramp starts
  double timeout_ramp_s_{0.0};       ///< seconds to ramp from 1→0
  double default_alpha_{1.0};        ///< initial / fallback blend weight

  // ─── Realtime buffers (non-RT writes, RT lock-free reads) ──────────
  realtime_tools::RealtimeBuffer<VelocityInput> tracking_buf_;
  realtime_tools::RealtimeBuffer<VelocityInput> avoidance_buf_;
  realtime_tools::RealtimeBuffer<double> alpha_buf_;

  // ─── Subscribers (owned, created in on_configure) ──────────────────
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      tracking_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
      avoidance_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr alpha_sub_;

  // ─── Parameter-event callback for dynamic alpha update ─────────────
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      param_cb_handle_;

  // ─── RT-only state (touched exclusively in update()) ───────────────
  std::array<double, kNumJoints> prev_cmd_{};
  bool first_update_{true};

  // ─── Configuration: collision behavior & interpolation ─────────────
  bool is_gazebo_{false};
  bool enable_interpolation_{false};

  // ─── Interpolation state (RT-only, per input channel) ──────────────
  //     When enable_interpolation is true, linearly ramps between
  //     consecutive samples across 1 kHz cycles (eliminates step changes).
  struct InterpState {
    std::array<double, kNumJoints> start{};    ///< value at ramp start
    std::array<double, kNumJoints> target{};   ///< newest sample value
    int64_t start_ns{0};                       ///< when ramp started
    int64_t duration_ns{5'000'000};            ///< inter-sample estimate (ns)
    int64_t last_sample_stamp_ns{0};           ///< stamp of last processed sample
  };
  InterpState tracking_interp_{};
  InterpState avoidance_interp_{};

  /// @brief Linear interpolation between consecutive samples (RT-safe).
  std::array<double, kNumJoints> interpolateInput(
      const VelocityInput& latest, InterpState& state, int64_t now_ns) const;

  // ─── Non-RT subscriber callbacks ───────────────────────────────────
  void trackingCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void avoidanceCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void alphaCb(const std_msgs::msg::Float64::SharedPtr msg);

  /// Monotonic nanosecond timestamp (RT-safe on Linux / vDSO).
  static int64_t steadyNowNs();
};

}  // namespace franka_rt_controllers
