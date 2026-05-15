#include <franka_rt_controllers/rt_torque_controller.hpp>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace franka_rt_controllers {

static constexpr std::array<double, 7> kDefaultTauMax{87, 87, 87, 87, 12, 12, 12};

// ═══════════════════════════════════════════════════════════════════════════
//  Interface configuration
// ═══════════════════════════════════════════════════════════════════════════

controller_interface::InterfaceConfiguration
RtTorqueController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (size_t i = 0; i < kNumJoints; ++i) {
    config.names.push_back(joint_names_[i] + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
RtTorqueController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // Positions at [0..6], velocities at [7..13] — order matches update() indexing
  for (size_t i = 0; i < kNumJoints; ++i) {
    config.names.push_back(joint_names_[i] + "/position");
  }
  for (size_t i = 0; i < kNumJoints; ++i) {
    config.names.push_back(joint_names_[i] + "/velocity");
  }
  return config;
}

// ═══════════════════════════════════════════════════════════════════════════
//  update()  –  REAL-TIME PATH
//
//  Guarantees:
//    • No heap allocations (q_pin_, data_ pre-sized in on_configure)
//    • No mutex / lock (RealtimeBuffer readFromRT)
//    • No RCLCPP_* logging
// ═══════════════════════════════════════════════════════════════════════════

controller_interface::return_type RtTorqueController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

  // 1. Read latest user torques (lock-free)
  const auto input = *command_buf_.readFromRT();

  // 2. LPF: alpha=1.0 → pass-through; alpha<1.0 → smoothing
  for (size_t i = 0; i < kNumJoints; ++i) {
    tau_filtered_[i] = lpf_alpha_ * input.tau[i]
                     + (1.0 - lpf_alpha_) * tau_filtered_[i];
  }

  // 3. Update q_pin_ from position state_interfaces_[0..6]
  for (size_t i = 0; i < kNumJoints; ++i) {
    q_pin_[arm_q_idx_[i]] = state_interfaces_[i].get_value();
  }

  // 4. Gravity torques → data_.g (Eigen::VectorXd, pre-allocated)
  pinocchio::computeGeneralizedGravity(model_, data_, q_pin_);

  // 5. tau_hw = clip(tau_filtered + g(q), -tau_max, tau_max)
  for (size_t i = 0; i < kNumJoints; ++i) {
    const double tau_raw = tau_filtered_[i] + data_.g[arm_v_idx_[i]];
    command_interfaces_[i].set_value(
        std::clamp(tau_raw, -tau_max_[i], tau_max_[i]));
  }

  return controller_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════════════
//  on_init
// ═══════════════════════════════════════════════════════════════════════════

CallbackReturn RtTorqueController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<std::string>("command_topic", "torque_cmd");
    auto_declare<double>("lpf_alpha", 1.0);
    auto_declare<double>("tau_max_scale", 1.0);
    auto_declare<bool>("gazebo", false);
    // Optional: pre-generated URDF path; empty → auto-generate via xacro
    auto_declare<std::string>("urdf_path", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception in RtTorqueController::on_init: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  on_configure
// ═══════════════════════════════════════════════════════════════════════════

CallbackReturn RtTorqueController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  auto logger = get_node()->get_logger();

  // ── Parameters ─────────────────────────────────────────────────────────
  arm_id_        = get_node()->get_parameter("arm_id").as_string();
  command_topic_ = get_node()->get_parameter("command_topic").as_string();
  lpf_alpha_     = get_node()->get_parameter("lpf_alpha").as_double();
  is_gazebo_     = get_node()->get_parameter("gazebo").as_bool();
  const double tau_max_scale = get_node()->get_parameter("tau_max_scale").as_double();

  const auto joints_param = get_node()->get_parameter("joints").as_string_array();
  if (joints_param.empty()) {
    joint_names_.clear();
    joint_names_.reserve(kNumJoints);
    for (size_t i = 1; i <= kNumJoints; ++i) {
      joint_names_.push_back(arm_id_ + "_joint" + std::to_string(i));
    }
  } else if (joints_param.size() != kNumJoints) {
    RCLCPP_ERROR(logger, "Expected %zu joints, got %zu", kNumJoints, joints_param.size());
    return CallbackReturn::ERROR;
  } else {
    joint_names_.assign(joints_param.begin(), joints_param.end());
  }

  for (size_t i = 0; i < kNumJoints; ++i) {
    tau_max_[i] = kDefaultTauMax[i] * tau_max_scale;
  }

  // ── SetFullCollisionBehavior ────────────────────────────────────────────
  if (!is_gazebo_) {
    auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
        "service_server/set_full_collision_behavior");
    auto req = std::make_shared<franka_msgs::srv::SetFullCollisionBehavior::Request>();
    req->lower_torque_thresholds_nominal      = {25, 25, 22, 20, 19, 17, 14};
    req->upper_torque_thresholds_nominal      = {35, 35, 32, 30, 29, 27, 24};
    req->lower_torque_thresholds_acceleration = {25, 25, 22, 20, 19, 17, 14};
    req->upper_torque_thresholds_acceleration = {35, 35, 32, 30, 29, 27, 24};
    req->lower_force_thresholds_nominal       = {30, 30, 30, 25, 25, 25};
    req->upper_force_thresholds_nominal       = {40, 40, 40, 35, 35, 35};
    req->lower_force_thresholds_acceleration  = {30, 30, 30, 25, 25, 25};
    req->upper_force_thresholds_acceleration  = {40, 40, 40, 35, 35, 35};
    auto future = client->async_send_request(req);
    if (future.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready) {
      RCLCPP_WARN(logger,
          "SetFullCollisionBehavior timed out — no Franka hardware service? "
          "Continuing with default thresholds. (Pass gazebo:=true to suppress.)");
    } else {
      auto resp = future.get();
      if (!resp || !resp->success) {
        RCLCPP_WARN(logger, "SetFullCollisionBehavior failed — using default thresholds.");
      }
    }
  }

  // ── Pinocchio model ─────────────────────────────────────────────────────
  std::string urdf_path = get_node()->get_parameter("urdf_path").as_string();
  const bool auto_urdf  = urdf_path.empty();

  if (auto_urdf) {
    std::string desc_share;
    try {
      desc_share = ament_index_cpp::get_package_share_directory("franka_description");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger, "franka_description not found: %s", e.what());
      return CallbackReturn::ERROR;
    }
    const std::string xacro_file = desc_share + "/robots/fr3/fr3.urdf.xacro";
    urdf_path = "/tmp/franka_rt_torque_" + arm_id_ + ".urdf";
    // Only invoke xacro when the cached file is absent (generated at launch time normally)
    std::ifstream cache_check(urdf_path);
    if (!cache_check.good()) {
      RCLCPP_INFO(logger, "Generating URDF via xacro (cached at %s) …", urdf_path.c_str());
      const std::string cmd =
          "xacro " + xacro_file + " hand:=true ee_id:=franka_hand"
          " > " + urdf_path + " 2>/dev/null";
      if (std::system(cmd.c_str()) != 0) {  // NOLINT(cert-env33-c)
        RCLCPP_ERROR(logger, "xacro failed — check that franka_description is installed.");
        return CallbackReturn::ERROR;
      }
    } else {
      RCLCPP_INFO(logger, "Using cached URDF at %s", urdf_path.c_str());
    }
  }

  try {
    pinocchio::urdf::buildModel(urdf_path, model_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Pinocchio model build failed: %s", e.what());
    return CallbackReturn::ERROR;
  }

  data_  = pinocchio::Data(model_);
  q_pin_ = pinocchio::neutral(model_);

  // Map joint names → pinocchio velocity/position indices
  for (size_t i = 0; i < kNumJoints; ++i) {
    const std::string& jname = joint_names_[i];
    if (!model_.existJointName(jname)) {
      RCLCPP_ERROR(logger,
          "Joint '%s' not found in Pinocchio model. "
          "If arm_id != 'fr3', provide a matching URDF via urdf_path parameter.",
          jname.c_str());
      return CallbackReturn::ERROR;
    }
    const pinocchio::JointIndex jid = model_.getJointId(jname);
    arm_v_idx_[i] = static_cast<int>(model_.joints[jid].idx_v());
    arm_q_idx_[i] = static_cast<int>(model_.joints[jid].idx_q());
  }

  // Warm up: ensures pinocchio's first-call overhead happens outside update()
  pinocchio::computeGeneralizedGravity(model_, data_, q_pin_);

  // ── RealtimeBuffer + subscriber ─────────────────────────────────────────
  TorqueInput zero{};
  command_buf_.writeFromNonRT(zero);

  command_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      command_topic_, rclcpp::SensorDataQoS().keep_last(1),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        commandCb(msg);
      });

  RCLCPP_INFO(logger,
      "RtTorqueController configured  arm=%s  topic=%s  "
      "lpf_alpha=%.2f  tau_max[0]=%.0f  tau_max[4]=%.0f  gazebo=%s",
      arm_id_.c_str(), command_topic_.c_str(), lpf_alpha_,
      tau_max_[0], tau_max_[4], is_gazebo_ ? "true" : "false");

  return CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  on_activate
// ═══════════════════════════════════════════════════════════════════════════

CallbackReturn RtTorqueController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  tau_filtered_.fill(0.0);
  TorqueInput zero{};
  command_buf_.writeFromNonRT(zero);
  for (size_t i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(0.0);
  }

  RCLCPP_INFO(get_node()->get_logger(), "RtTorqueController ACTIVATED.");
  return CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  on_deactivate
// ═══════════════════════════════════════════════════════════════════════════

CallbackReturn RtTorqueController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  for (size_t i = 0; i < kNumJoints; ++i) {
    command_interfaces_[i].set_value(0.0);
  }

  RCLCPP_INFO(get_node()->get_logger(),
      "RtTorqueController DEACTIVATED → zero effort.");
  return CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  commandCb  (non-RT thread)
// ═══════════════════════════════════════════════════════════════════════════

void RtTorqueController::commandCb(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {

  if (msg->data.size() != kNumJoints) {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "torque_cmd: expected %zu values, got %zu — IGNORING",
        kNumJoints, msg->data.size());
    return;
  }
  TorqueInput input;
  std::copy_n(msg->data.begin(), kNumJoints, input.tau.begin());
  input.received = true;
  command_buf_.writeFromNonRT(input);
}

}  // namespace franka_rt_controllers

PLUGINLIB_EXPORT_CLASS(franka_rt_controllers::RtTorqueController,
                       controller_interface::ControllerInterface)
