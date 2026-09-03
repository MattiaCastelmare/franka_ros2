#include <franka_rt_controllers/rt_torque_controller.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

namespace franka_rt_controllers {

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
  // Position + velocity per giunto: la velocità alimenta il feedback a 1 kHz
  // (Kd·(q̇_des − q̇)); la posizione è richiesta anche se non usata dalla legge
  // attuale, per non richiedere un secondo refactor delle interfacce se in
  // futuro si aggiungesse un termine Kp. Ordine [pos, vel] per giunto: stesso
  // pattern di joint_impedance_example_controller (indici 2*i, 2*i+1).
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (size_t i = 0; i < kNumJoints; ++i) {
    config.names.push_back(joint_names_[i] + "/position");
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
    const rclcpp::Time& time,
    const rclcpp::Duration& period) {

  // 1. Stato misurato (posizione+velocità) dalle interfacce hardware.
  updateJointStates();   // riempie q_, dq_

  // 2. Ultimo feedforward τ_ff (lock-free) — trasporto invariato.
  const auto input = *command_buf_.readFromRT();

  // 3. Ultimo q̈_safe (lock-free) + check di freschezza.
  const auto accel = *accel_buf_.readFromRT();
  const double now_s   = time.seconds();
  const bool   accel_ok =
      accel.received && (now_s - accel.stamp) < accel_timeout_;

  // dt per l'integrale di q̇_des: il period del controller-manager È il dt
  // esatto del loop RT a 1 kHz (questo è il "dt_RT" del design — NON il tempo
  // Python, quindi il jitter dei 100 Hz non entra qui). Guardia su period non
  // positivo/teleportato (startup, salti di sim-time); il clamp duro sotto
  // limita comunque ogni residuo.
  const double dt = period.seconds();

  // NOTA: la gravità g(q) è compensata dal firmware Franka — NON aggiungerla
  // qui, o verrebbe compensata due volte. Nessun LPF e nessuna saturazione SW:
  // limiti/saturazione coppia restano delegati allo stack low-level Franka.
  for (size_t i = 0; i < kNumJoints; ++i) {
    double e;
    if (accel_ok && dt > 0.0) {
      // Integra q̇_des da q̈_safe, poi CLAMP DURO dell'errore di velocità a
      // ±e_max (anti-windup; NESSUN leak proporzionale — vedi Diagnosi/4b: un
      // leak reintrodurrebbe una frazione k_sync/(Kd+k_sync) proprio del
      // deficit che questo termine esiste per annullare).
      qdot_des_[i] += accel.qddot[i] * dt;

      // TETTO DI VELOCITÀ sul riferimento integrato. q̇_des è un integratore
      // libero di q̈_safe: finché il CBF chiede accelerazione (ostacolo veloce
      // ⇒ q̈_safe grande e SOSTENUTO) q̇_des cresce senza alcun limite, e il
      // clamp e_max lo tiene comunque a dq_+e_max — cioè richiede una velocità
      // di 1 rad/s SOPRA la misurata, con Kd·e_max di coppia costante che la
      // insegue. Il box di velocità del CBF vive a 100 Hz e sul COMANDO q̈, non
      // su questo riferimento: fra i due nulla conosceva q̇_max, e il primo a
      // reagire era il firmware con `joint_velocity_violation`. Qui, a 1 kHz e
      // sulla velocità MISURATA, c'è il backstop. Il clamp e_max sotto può solo
      // avvicinare q̇_des a dq_, quindi non può riportarlo sopra il tetto.
      if (qdot_margin_ > 0.0) {
        qdot_des_[i] = std::min(qdotCeiling(i, q_[i]),
                                std::max(qdotFloor(i, q_[i]), qdot_des_[i]));
      }

      e = qdot_des_[i] - dq_[i];
      if (std::abs(e) > e_max_) {
        e = std::copysign(e_max_, e);   // errore POST-clamp: è quello
        qdot_des_[i] = dq_[i] + e;      // "fisicamente ammesso" → usato per τ
      }
    } else {
      // Nessun q̈_safe fresco: degrada in modo SICURO al puro pass-through del
      // feedforward (comportamento odierno). Congela q̇_des sulla misura, così
      // l'errore — e quindi la coppia di feedback — è esattamente zero, e
      // l'integratore è già riallineato per quando q̈_safe riprende.
      qdot_des_[i] = dq_[i];
      e = 0.0;
    }

    // 4. Legge di controllo: τ = τ_ff + Kd·e   (errore post-clamp).
    command_interfaces_[i].set_value(input.tau[i] + d_gains_[i] * e);
  }

  return controller_interface::return_type::OK;
}

// ═══════════════════════════════════════════════════════════════════════════
//  qdotCeiling / qdotFloor  (RT thread) — inviluppo di velocità del firmware
// ═══════════════════════════════════════════════════════════════════════════
//  Il firmware FR3 non ammette |q̇| ≤ q̇_max costante: vicino a un fine corsa
//  il limite si stringe secondo (franka_description, position_based_velocity_
//  limits)
//      q̇_max(q) = min( q̇_lim ,  v_off + sqrt(2·a_dec·h) ) ,   h = dist. dal
//  limite di posizione in quella direzione. Riprodurlo qui — invece del solo
//  q̇_lim piatto — fa sì che il tetto copra ENTRAMBI i reflex di velocità, non
//  solo quello sul limite assoluto.
//
//  qdot_margin_ (< 1) sposta il tetto sotto la soglia del firmware, così a
//  mordere è questo controller e non il reflex. Nessuna allocazione, una sola
//  sqrt per giunto per tick: RT-safe.

double RtTorqueController::qdotCeiling(size_t i, double q) const {
  const double h = std::max(0.0, q_max_[i] - q);
  return qdot_margin_ * std::min(qdot_max_[i],
                                 v_offset_[i] + std::sqrt(2.0 * decel_[i] * h));
}

double RtTorqueController::qdotFloor(size_t i, double q) const {
  const double h = std::max(0.0, q - q_min_[i]);
  return -qdot_margin_ * std::min(qdot_max_[i],
                                  v_offset_[i] + std::sqrt(2.0 * decel_[i] * h));
}

// ═══════════════════════════════════════════════════════════════════════════
//  on_init
// ═══════════════════════════════════════════════════════════════════════════

CallbackReturn RtTorqueController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    auto_declare<std::string>("command_topic", "torque_cmd");
    auto_declare<bool>("gazebo", false);
    // ── Feedback di velocità a 1 kHz ──────────────────────────────────────
    // Dichiarati qui per NON perpetuare il disallineamento del generatore YAML
    // (che oggi passa lpf_alpha/tau_max_scale/urdf_path senza che il C++ li
    // dichiari → ignorati). Tutti VALORI INIZIALI da validare in hardware.
    auto_declare<std::string>("accel_topic", "/NS_1/qddot_safe");
    auto_declare<std::vector<double>>(
        "d_gains", std::vector<double>{30.0, 30.0, 30.0, 25.0, 10.0, 10.0, 5.0});
    auto_declare<double>("e_max", 1.0);
    auto_declare<double>("accel_timeout", 0.1);
    // ── Tetto di velocità sul riferimento integrato ───────────────────────
    // Default = franka_description/robots/fr3/joint_limits.yaml (stessa fonte
    // che legge il box di stato del cbf_safety_filter). qdot_margin = 0
    // disattiva il tetto e ripristina il comportamento precedente.
    auto_declare<std::vector<double>>(
        "qdot_max", std::vector<double>{2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26});
    auto_declare<std::vector<double>>(
        "q_min", std::vector<double>{-2.9007, -1.8361, -2.9007, -3.0770,
                                     -2.8763, 0.4398, -3.0508});
    auto_declare<std::vector<double>>(
        "q_max", std::vector<double>{2.9007, 1.8361, 2.9007, -0.1169,
                                     2.8763, 4.6216, 3.0508});
    auto_declare<std::vector<double>>(
        "velocity_offset", std::vector<double>{0.6520, 0.2500, 0.2005, 0.3542,
                                               0.5738, 0.4885, 0.4592});
    auto_declare<std::vector<double>>(
        "deceleration_limit", std::vector<double>{6.0, 2.585, 3.5, 4.0,
                                                  17.0, 5.5, 17.0});
    auto_declare<double>("qdot_margin", 0.95);
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
  is_gazebo_     = get_node()->get_parameter("gazebo").as_bool();
  accel_topic_   = get_node()->get_parameter("accel_topic").as_string();
  e_max_         = get_node()->get_parameter("e_max").as_double();
  accel_timeout_ = get_node()->get_parameter("accel_timeout").as_double();

  const auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (d_gains.size() != kNumJoints) {
    RCLCPP_ERROR(logger, "d_gains must have %zu entries, got %zu",
                 kNumJoints, d_gains.size());
    return CallbackReturn::ERROR;
  }
  std::copy_n(d_gains.begin(), kNumJoints, d_gains_.begin());

  // ── Tetto di velocità: carica e valida i cinque array dell'inviluppo ─────
  qdot_margin_ = get_node()->get_parameter("qdot_margin").as_double();
  const std::array<std::pair<const char*, std::array<double, kNumJoints>*>, 5> limit_params{{
      {"qdot_max", &qdot_max_},
      {"q_min", &q_min_},
      {"q_max", &q_max_},
      {"velocity_offset", &v_offset_},
      {"deceleration_limit", &decel_},
  }};
  for (const auto& [name, dest] : limit_params) {
    const auto v = get_node()->get_parameter(name).as_double_array();
    if (v.size() != kNumJoints) {
      RCLCPP_ERROR(logger, "%s must have %zu entries, got %zu", name, kNumJoints, v.size());
      return CallbackReturn::ERROR;
    }
    std::copy_n(v.begin(), kNumJoints, dest->begin());
  }
  if (qdot_margin_ <= 0.0) {
    RCLCPP_WARN(logger,
        "qdot_margin=%.3f <= 0 — tetto di velocità DISATTIVATO: q̇_des integra "
        "q̈_safe senza limite e solo il reflex del firmware ferma il giunto.",
        qdot_margin_);
  }

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

  // ── RealtimeBuffer + subscriber ─────────────────────────────────────────
  TorqueInput zero{};
  command_buf_.writeFromNonRT(zero);
  accel_buf_.writeFromNonRT(AccelInput{});

  command_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      command_topic_, rclcpp::SensorDataQoS().keep_last(1),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        commandCb(msg);
      });

  accel_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      accel_topic_, rclcpp::SensorDataQoS().keep_last(1),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        accelCb(msg);
      });

  RCLCPP_INFO(logger,
      "RtTorqueController configured (τ_ff + Kd·(q̇_des−q̇), clamp e_max=%.3f rad/s, "
      "qdot_margin=%.3f, accel_timeout=%.3f s)  arm=%s  τ_topic=%s  q̈_topic=%s  gazebo=%s",
      e_max_, qdot_margin_, accel_timeout_, arm_id_.c_str(), command_topic_.c_str(),
      accel_topic_.c_str(), is_gazebo_ ? "true" : "false");

  return CallbackReturn::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════════════════
//  on_activate
// ═══════════════════════════════════════════════════════════════════════════

CallbackReturn RtTorqueController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {

  TorqueInput zero{};
  command_buf_.writeFromNonRT(zero);
  accel_buf_.writeFromNonRT(AccelInput{});   // scarta q̈_safe pre-attivazione

  // Inizializza il riferimento integrato sulla velocità misurata, così il
  // primo errore (e quindi la coppia di feedback) parte da zero — niente
  // transitorio all'attivazione (analogo a initial_q_ = q_ nell'esempio).
  updateJointStates();
  qdot_des_ = dq_;

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
//  updateJointStates  (RT thread) — legge q_/dq_ dalle interfacce di stato
//  Ordine [pos, vel] per giunto: indici 2*i, 2*i+1 (vedi
//  state_interface_configuration). Stesso pattern dell'esempio Franka.
// ═══════════════════════════════════════════════════════════════════════════

void RtTorqueController::updateJointStates() {
  for (size_t i = 0; i < kNumJoints; ++i) {
    q_[i]  = state_interfaces_[2 * i].get_value();
    dq_[i] = state_interfaces_[2 * i + 1].get_value();
  }
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

// ═══════════════════════════════════════════════════════════════════════════
//  accelCb  (non-RT thread) — riceve q̈_safe (uscita CBF), gemello di commandCb
// ═══════════════════════════════════════════════════════════════════════════

void RtTorqueController::accelCb(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {

  if (msg->data.size() != kNumJoints) {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "qddot_safe: expected %zu values, got %zu — IGNORING",
        kNumJoints, msg->data.size());
    return;
  }
  AccelInput accel;
  std::copy_n(msg->data.begin(), kNumJoints, accel.qddot.begin());
  accel.received = true;
  accel.stamp    = get_node()->now().seconds();
  accel_buf_.writeFromNonRT(accel);
}

}  // namespace franka_rt_controllers

PLUGINLIB_EXPORT_CLASS(franka_rt_controllers::RtTorqueController,
                       controller_interface::ControllerInterface)
