#pragma once

#include <array>
#include <atomic>
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

class RtTorqueController : public controller_interface::ControllerInterface {
 public:
  static constexpr size_t kNumJoints = 7;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  struct TorqueInput {
    std::array<double, kNumJoints> tau{};
    bool   received{false};
    double stamp{0.0};   // istante di ricezione [s] (node clock). Senza questo
                         // l'ULTIMA coppia ricevuta restava applicata per
                         // sempre se qddot_to_torque smetteva di pubblicare:
                         // accel_timeout copriva solo q̈_safe. Vedi update().
  };

  // q̈_safe transport — parallel, lock-free, gemello di TorqueInput. Porta il
  // riferimento di accelerazione (uscita CBF) per il feedback di velocità a
  // 1 kHz. stamp = istante di ricezione [s] (node clock), usato dal check di
  // freschezza in update().
  struct AccelInput {
    std::array<double, kNumJoints> qddot{};
    bool   received{false};
    double stamp{0.0};
  };

  void updateJointStates();

  // Tetto/pavimento di velocità ammessi per il riferimento integrato sul
  // giunto i alla posizione q. Riproduce l'inviluppo del firmware FR3
  // (position_based_velocity_limits in franka_description), scalato da
  // qdot_margin_ così il tetto morde PRIMA del reflex. Vedi update().
  double qdotCeiling(size_t i, double q) const;   // bound superiore (> 0)
  double qdotFloor(size_t i, double q) const;     // bound inferiore (< 0)
  void commandCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void accelCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  std::string arm_id_;
  std::vector<std::string> joint_names_;
  std::string command_topic_;
  std::string accel_topic_;
  bool is_gazebo_{false};

  // ── Feedback di velocità (vedi on_configure per la semantica) ────────────
  std::array<double, kNumJoints> d_gains_{};   // Kd per giunto [N·m/(rad/s)]
  double e_max_{1.0};                           // clamp errore velocità [rad/s]
  double accel_timeout_{0.1};                   // finestra freschezza q̈_safe [s]
  double command_timeout_{0.1};                 // finestra freschezza τ_ff [s]

  // ── Anello di POSIZIONE (vedi update) ───────────────────────────────────
  // Senza di questo la catena non ha alcun feedback di posizione: τ_ff è
  // feedforward puro e Kd agisce solo sulla velocità, quindi ogni deficit di
  // coppia (attrito, errore di modello) si integra in una deriva di posizione
  // NON limitata — misurato: 0.52 m di errore EE senza alcun ostacolo vicino.
  // Il riferimento q_des_ integra q̇_des_, cioè q̈_SAFE: segue la traiettoria
  // filtrata dal CBF, quindi corregge l'esecuzione e non combatte l'avoidance.
  std::array<double, kNumJoints> p_gains_{};    // Kp per giunto [N·m/rad]
  double p_max_{0.15};                          // clamp errore posizione [rad]
  std::array<double, kNumJoints> q_des_{};      // riferimento posizione integrato

  // ── Hold su perdita di τ_ff ─────────────────────────────────────────────
  std::array<double, kNumJoints> q_hold_{};     // posa catturata all'inizio del hold
  bool hold_latched_{false};                    // RT-only: hold già catturato
  std::atomic<bool> in_hold_{false};            // letto dal timer non-RT che logga
  bool hold_logged_{false};                     // stato del logger (thread del timer)
  rclcpp::TimerBase::SharedPtr fault_timer_;    // logga i fronti di in_hold_

  // ── Tetto di velocità sul riferimento integrato (backstop del reflex) ────
  // q̇_des è un integratore libero di q̈_safe: senza questi limiti nulla, tra
  // il CBF a 100 Hz e il firmware, impedisce a Kd·(q̇_des−q̇) di spingere il
  // giunto oltre q̇_max → "Move command aborted: joint_velocity_violation".
  // I valori di default sono quelli di franka_description/robots/fr3/
  // joint_limits.yaml. qdot_margin_ <= 0 disattiva il tetto.
  std::array<double, kNumJoints> qdot_max_{};   // |q̇| ufficiale [rad/s]
  std::array<double, kNumJoints> q_min_{};      // limite posizione inferiore [rad]
  std::array<double, kNumJoints> q_max_{};      // limite posizione superiore [rad]
  std::array<double, kNumJoints> v_offset_{};   // offset inviluppo firmware [rad/s]
  std::array<double, kNumJoints> decel_{};      // autorità di frenata [rad/s²]
  double qdot_margin_{0.95};                    // frazione di q̇_max concessa

  // ── Stato RT-only (aggiornato dentro update(); mai condiviso tra thread) ──
  std::array<double, kNumJoints> q_{};          // posizione misurata
  std::array<double, kNumJoints> dq_{};         // velocità misurata
  std::array<double, kNumJoints> qdot_des_{};   // riferimento velocità integrato

  realtime_tools::RealtimeBuffer<TorqueInput> command_buf_;
  realtime_tools::RealtimeBuffer<AccelInput>  accel_buf_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr accel_sub_;
};

}  // namespace franka_rt_controllers
