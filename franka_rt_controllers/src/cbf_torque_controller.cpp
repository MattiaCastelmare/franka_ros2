#include <franka_rt_controllers/cbf_torque_controller.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace franka_rt_controllers {

// ═══════════════════════════════════════════════════════════════════════════
//  Interface configuration
// ═══════════════════════════════════════════════════════════════════════════

controller_interface::InterfaceConfiguration
CBFTorqueController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNJ; ++i)
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  return config;
}

controller_interface::InterfaceConfiguration
CBFTorqueController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= kNJ; ++i)
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  for (int i = 1; i <= kNJ; ++i)
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  return config;
}

// ═══════════════════════════════════════════════════════════════════════════
//  on_init
// ═══════════════════════════════════════════════════════════════════════════

CallbackReturn CBFTorqueController::on_init() {
  try {
    auto_declare<std::string>("arm_id",            "fr3");
    auto_declare<std::string>("qddot_safe_topic",  "/NS_1/qddot_safe");
    auto_declare<std::string>("q_des_state_topic", "/NS_1/q_des_state");
    auto_declare<double>("qddot_timeout_s",        0.1);
    auto_declare<std::vector<double>>("kd",        std::vector<double>(kNJ, 20.0));
    auto_declare<std::vector<double>>("kp_local",  std::vector<double>(kNJ,  0.0));
    auto_declare<std::vector<double>>("kd_local",  std::vector<double>(kNJ,  0.0));
    // FOH + LPF: qddot_lpf_alpha=0 disables LPF, 0.8..0.95 gives light smoothing.
    auto_declare<double>("qddot_lpf_alpha",        0.0);
  } catch (const std::exception& e) {
    fprintf(stderr, "CBFTorqueController::on_init: %s\n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  on_configure
// ═══════════════════════════════════════════════════════════════════════════

CallbackReturn CBFTorqueController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  auto logger = get_node()->get_logger();

  arm_id_           = get_node()->get_parameter("arm_id").as_string();
  qddot_safe_topic_ = get_node()->get_parameter("qddot_safe_topic").as_string();
  q_des_topic_      = get_node()->get_parameter("q_des_state_topic").as_string();
  qddot_timeout_s_  = get_node()->get_parameter("qddot_timeout_s").as_double();
  lpf_alpha_        = get_node()->get_parameter("qddot_lpf_alpha").as_double();
  lpf_alpha_        = std::clamp(lpf_alpha_, 0.0, 0.99);

  auto load_vec7 = [&](const std::string& name, Vec7& out) -> bool {
    auto v = get_node()->get_parameter(name).as_double_array();
    if (static_cast<int>(v.size()) != kNJ) {
      RCLCPP_FATAL(logger, "%s must have %d elements, got %zu", name.c_str(), kNJ, v.size());
      return false;
    }
    for (int i = 0; i < kNJ; ++i) out(i) = v[i];
    return true;
  };
  if (!load_vec7("kd",       kd_))       return CallbackReturn::FAILURE;
  if (!load_vec7("kp_local", kp_local_)) return CallbackReturn::FAILURE;
  if (!load_vec7("kd_local", kd_local_)) return CallbackReturn::FAILURE;

  // ── Pinocchio model ──────────────────────────────────────────────────────
  std::string desc_share;
  try {
    desc_share = ament_index_cpp::get_package_share_directory("franka_description");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "franka_description not found: %s", e.what());
    return CallbackReturn::ERROR;
  }
  const std::string xacro_file = desc_share + "/robots/fr3/fr3.urdf.xacro";
  const std::string urdf_path  = "/tmp/franka_cbf_torque_" + arm_id_ + ".urdf";
  if (std::system(("xacro " + xacro_file + " > " + urdf_path + " 2>/dev/null").c_str()) != 0) {  // NOLINT
    RCLCPP_ERROR(logger, "xacro failed");
    return CallbackReturn::ERROR;
  }
  try {
    pinocchio::urdf::buildModel(urdf_path, pin_model_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "Pinocchio build: %s", e.what());
    std::remove(urdf_path.c_str());
    return CallbackReturn::ERROR;
  }
  std::remove(urdf_path.c_str());

  pin_data_  = pinocchio::Data(pin_model_);
  q_pin_     = pinocchio::neutral(pin_model_);
  dq_pin_    = Eigen::VectorXd::Zero(pin_model_.nv);
  qddot_pin_ = Eigen::VectorXd::Zero(pin_model_.nv);

  for (int i = 0; i < kNJ; ++i) {
    const std::string jname = arm_id_ + "_joint" + std::to_string(i + 1);
    if (!pin_model_.existJointName(jname)) {
      RCLCPP_ERROR(logger, "Joint '%s' not in Pinocchio model", jname.c_str());
      return CallbackReturn::ERROR;
    }
    const pinocchio::JointIndex jid = pin_model_.getJointId(jname);
    arm_q_idx_[i] = static_cast<int>(pin_model_.joints[jid].idx_q());
    arm_v_idx_[i] = static_cast<int>(pin_model_.joints[jid].idx_v());
  }
  pinocchio::rnea(pin_model_, pin_data_, q_pin_, dq_pin_, qddot_pin_);
  pinocchio::computeGeneralizedGravity(pin_model_, pin_data_, q_pin_);

  // ── Legacy Float64MultiArray subscription ────────────────────────────────
  qddot_buf_.writeFromNonRT(Vec7::Zero());
  sub_ = get_node()->create_subscription<MsgT>(
      qddot_safe_topic_, rclcpp::QoS(10).reliable(),
      [this](const MsgT::SharedPtr msg) {
        if (static_cast<int>(msg->data.size()) != kNJ) return;
        Vec7 v;
        for (int i = 0; i < kNJ; ++i) v(i) = msg->data[i];
        qddot_buf_.writeFromNonRT(v);
        last_msg_stamp_s_ = get_node()->get_clock()->now().seconds();
      });

  // ── JointState setpoint subscription — builds FOH interpolation state ────
  // On every new message:
  //   1. the previously-active "next" becomes "prev"
  //   2. the inter-message period is estimated via EMA
  //   3. alpha in update() = (now - arrival_s) / period_s ∈ [0, 1]
  //      → smooth linear blend from prev to next over one Python period
  interp_buf_.writeFromNonRT(InterpState{});
  sp_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
      q_des_topic_, rclcpp::QoS(10).reliable(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->name.size()     < static_cast<size_t>(kNJ) ||
            msg->position.size() < static_cast<size_t>(kNJ) ||
            msg->velocity.size() < static_cast<size_t>(kNJ) ||
            msg->effort.size()   < static_cast<size_t>(kNJ))
          return;

        // Parse into a Setpoint, matching by joint name
        Setpoint sp;
        for (int i = 0; i < kNJ; ++i) {
          const std::string jname = arm_id_ + "_joint" + std::to_string(i + 1);
          auto it = std::find(msg->name.begin(), msg->name.end(), jname);
          if (it == msg->name.end()) return;
          const int idx = static_cast<int>(std::distance(msg->name.begin(), it));
          sp.q_d(i)   = msg->position[idx];
          sp.dq_d(i)  = msg->velocity[idx];
          sp.qddot(i) = msg->effort[idx];
        }

        const double now_s = get_node()->get_clock()->now().seconds();

        // Build new interpolation state from the current one
        const InterpState* cur = interp_buf_.readFromNonRT();
        InterpState istate;
        istate.prev = cur->next;  // old "next" slides into "prev"
        istate.next = sp;
        istate.arrival_s = now_s;

        // EMA on inter-message period; clamp to [5ms, 500ms] for robustness
        const double dt_meas = now_s - cur->arrival_s;
        istate.period_s = (dt_meas > 0.005 && dt_meas < 0.5)
            ? 0.85 * cur->period_s + 0.15 * dt_meas
            : cur->period_s;

        interp_buf_.writeFromNonRT(istate);
        sp_stamp_s_ = now_s;
      });

  RCLCPP_INFO(logger,
    "CBFTorqueController configured  arm=%s  qddot_safe=%s  q_des=%s  "
    "timeout=%.2fs  lpf_alpha=%.2f",
    arm_id_.c_str(), qddot_safe_topic_.c_str(), q_des_topic_.c_str(),
    qddot_timeout_s_, lpf_alpha_);
  return CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  on_activate
// ═══════════════════════════════════════════════════════════════════════════

CallbackReturn CBFTorqueController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  read_joint_states();
  for (int i = 0; i < kNJ; ++i) {
    q_pin_[arm_q_idx_[i]] = q_(i);
    command_interfaces_[i].set_value(0.0);
  }
  last_q_d_         = q_;
  last_msg_stamp_s_ = get_node()->get_clock()->now().seconds();
  sp_stamp_s_       = 0.0;
  qddot_lpf_        = Vec7::Zero();
  qddot_buf_.writeFromNonRT(Vec7::Zero());
  interp_buf_.writeFromNonRT(InterpState{});
  RCLCPP_INFO(get_node()->get_logger(), "CBFTorqueController ACTIVATED.");
  return CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  on_deactivate
// ═══════════════════════════════════════════════════════════════════════════

CallbackReturn CBFTorqueController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  for (int i = 0; i < kNJ; ++i) command_interfaces_[i].set_value(0.0);
  RCLCPP_INFO(get_node()->get_logger(),
      "CBFTorqueController DEACTIVATED → zero effort.");
  return CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  read_joint_states  (RT path)
// ═══════════════════════════════════════════════════════════════════════════

void CBFTorqueController::read_joint_states() {
  for (int i = 0; i < kNJ; ++i) {
    q_(i)  = state_interfaces_[i].get_value();
    dq_(i) = state_interfaces_[kNJ + i].get_value();
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  update  — REAL-TIME PATH  (1 kHz, no alloc, no logging)
// ═══════════════════════════════════════════════════════════════════════════

controller_interface::return_type CBFTorqueController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

  read_joint_states();

  const double now_s       = get_node()->get_clock()->now().seconds();
  const bool   sp_stale    = (now_s - sp_stamp_s_)       > qddot_timeout_s_;
  const bool   qddot_stale = (now_s - last_msg_stamp_s_) > qddot_timeout_s_;

  for (int i = 0; i < kNJ; ++i) {
    q_pin_[arm_q_idx_[i]]  = q_(i);
    dq_pin_[arm_v_idx_[i]] = dq_(i);
  }

  if (!sp_stale) {
    // ── PATH 1: FOH interpolated JointState setpoint ─────────────────────
    // alpha ∈ [0,1]: 0 at prev-message time, 1 at next-message time.
    // This converts the Python 100Hz ZOH into a smooth 1kHz ramp.
    const InterpState& isp = *interp_buf_.readFromRT();
    const double alpha = std::clamp(
        (now_s - isp.arrival_s) / isp.period_s, 0.0, 1.0);
    const double beta  = 1.0 - alpha;

    Setpoint sp;
    for (int i = 0; i < kNJ; ++i) {
      sp.q_d(i)   = beta * isp.prev.q_d(i)   + alpha * isp.next.q_d(i);
      sp.dq_d(i)  = beta * isp.prev.dq_d(i)  + alpha * isp.next.dq_d(i);
      sp.qddot(i) = beta * isp.prev.qddot(i) + alpha * isp.next.qddot(i);
    }

    last_q_d_ = isp.next.q_d;  // hold latest confirmed target for watchdog

    // Optional IIR LPF on qddot_ff (disabled when lpf_alpha_=0)
    if (lpf_alpha_ > 0.0) {
      for (int i = 0; i < kNJ; ++i) {
        qddot_lpf_(i) = lpf_alpha_ * qddot_lpf_(i)
                      + (1.0 - lpf_alpha_) * sp.qddot(i);
        sp.qddot(i) = qddot_lpf_(i);
      }
    }

    for (int i = 0; i < kNJ; ++i)
      qddot_pin_[arm_v_idx_[i]] = sp.qddot(i);

    pinocchio::rnea(pin_model_, pin_data_, q_pin_, dq_pin_, qddot_pin_);
    pinocchio::computeGeneralizedGravity(pin_model_, pin_data_, q_pin_);

    for (int i = 0; i < kNJ; ++i) {
      const double tau_ff = pin_data_.tau[arm_v_idx_[i]] - pin_data_.g[arm_v_idx_[i]];
      const double tau_pd = kp_local_(i) * (sp.q_d(i)  - q_(i))
                          + kd_local_(i) * (sp.dq_d(i) - dq_(i));
      command_interfaces_[i].set_value(tau_ff + tau_pd);
    }

  } else if (!qddot_stale) {
    // ── PATH 2: legacy Float64MultiArray (qddot_safe / CBF filter output)
    const Vec7 qddot_ref = *qddot_buf_.readFromRT();

    for (int i = 0; i < kNJ; ++i)
      qddot_pin_[arm_v_idx_[i]] = qddot_ref(i);

    pinocchio::rnea(pin_model_, pin_data_, q_pin_, dq_pin_, qddot_pin_);
    pinocchio::computeGeneralizedGravity(pin_model_, pin_data_, q_pin_);

    for (int i = 0; i < kNJ; ++i)
      command_interfaces_[i].set_value(
          pin_data_.tau[arm_v_idx_[i]] - pin_data_.g[arm_v_idx_[i]]);

    last_q_d_ = q_;  // sync watchdog hold to current position

  } else {
    // ── PATH 3: WATCHDOG — hold last_q_d_ with pure damping
    for (int i = 0; i < kNJ; ++i)
      command_interfaces_[i].set_value(
          kp_local_(i) * (last_q_d_(i) - q_(i))
        - kd_(i)       * dq_(i));
  }

  return controller_interface::return_type::OK;
}

}  // namespace franka_rt_controllers

PLUGINLIB_EXPORT_CLASS(franka_rt_controllers::CBFTorqueController,
                       controller_interface::ControllerInterface)
