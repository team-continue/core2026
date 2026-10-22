#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

#include "core_hardware/hardware_snapshot.hpp"
#include "core_hardware/ipc_protocol.hpp"
#include "core_hardware/motor_diag_layout.h"
#include "core_msgs/msg/can_array.hpp"
#include "core_msgs/msg/hardware_link_status.hpp"
#include "core_msgs/msg/motor_status_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace core_hardware {

static_assert(std::tuple_size<decltype(HardwareSnapshot::motor_diag)>::value == motor_diag::kNumMotors,
              "motor_diag must have one entry per motor");
static_assert(std::tuple_size<decltype(HardwareSnapshot::motor_voltage)>::value == motor_diag::kNumMotors,
              "motor_voltage must have one entry per motor");
static_assert(std::tuple_size<decltype(HardwareSnapshot::bus_diag)>::value == motor_diag::kNumCanBuses,
              "bus_diag must have one entry per CAN bus");

// bus_diag の並び ([0] = can3, [1] = can2) に対応するバス名
constexpr std::array<const char*, motor_diag::kNumCanBuses> kCanBusNames = {"can3", "can2"};

inline float temp_to_float(const int8_t temp) {
  return temp == motor_diag::kTempInvalid ? std::numeric_limits<float>::quiet_NaN() : static_cast<float>(temp);
}

inline float voltage_to_float(const uint16_t voltage_cv) {
  return voltage_cv == motor_diag::kVoltageInvalid ? std::numeric_limits<float>::quiet_NaN()
                                                   : static_cast<float>(voltage_cv) * 0.01f;
}

// スナップショットの motor_diag / motor_voltage / bus_diag から /hardware/motor_status を作る (header は呼び出し側で埋める)。
// supported / valid が false のときは他の値が無効なので、bool は false、数値は 0 / NaN にそろえる
inline core_msgs::msg::MotorStatusArray make_motor_status_array(const HardwareSnapshot& state) {
  core_msgs::msg::MotorStatusArray msg;
  msg.motors.reserve(motor_diag::kNumMotors);
  for (uint8_t id = 0; id < motor_diag::kNumMotors; ++id) {
    const motor_diag::MotorDiag diag = motor_diag::unpack_motor_diag(state.motor_diag[id], state.motor_voltage[id]);
    core_msgs::msg::MotorStatus motor;
    motor.id = id;
    motor.supported = diag.supported;
    motor.connected = diag.supported && diag.connected;
    motor.enabled = diag.supported && diag.enabled;
    motor.fault = diag.supported && diag.fault;
    motor.error_code = diag.supported ? diag.error_code : 0U;
    motor.temp = diag.supported ? temp_to_float(diag.temp) : std::numeric_limits<float>::quiet_NaN();
    motor.temp_rotor = diag.supported ? temp_to_float(diag.temp_rotor) : std::numeric_limits<float>::quiet_NaN();
    motor.voltage = diag.supported ? voltage_to_float(diag.voltage_cv) : std::numeric_limits<float>::quiet_NaN();
    msg.motors.push_back(motor);
  }

  msg.can_buses.reserve(motor_diag::kNumCanBuses);
  for (std::size_t index = 0; index < motor_diag::kNumCanBuses; ++index) {
    const motor_diag::BusDiag diag = motor_diag::unpack_bus_diag(state.bus_diag[index]);
    core_msgs::msg::CanBusStatus bus;
    bus.name = kCanBusNames[index];
    bus.valid = diag.valid;
    bus.bus_off = diag.valid && diag.bus_off;
    bus.error_passive = diag.valid && diag.error_passive;
    bus.tx_error_count = diag.valid ? diag.tx_error_count : 0U;
    bus.rx_error_count = diag.valid ? diag.rx_error_count : 0U;
    msg.can_buses.push_back(bus);
  }
  return msg;
}

// daemon (IPC) の接続状態と最新のスナップショットから /hardware/link_status を作る (header は呼び出し側で埋める)。
// now_ms は daemon が last_ethercat_rx_ms に使う時計 (system_clock の epoch ms) で渡す
inline core_msgs::msg::HardwareLinkStatus make_link_status(
    const bool daemon_connected, const HardwareSnapshot& state, const int64_t now_ms) {
  core_msgs::msg::HardwareLinkStatus msg;
  msg.daemon_connected = daemon_connected;
  msg.ethercat_connected = false;
  msg.ethercat_rx_age_ms = std::numeric_limits<float>::quiet_NaN();
  msg.bottom_connected = false;
  if (!daemon_connected) {
    // daemon と切れている間は EtherCAT・bottom の状態は分からない
    return msg;
  }

  const std::optional<int64_t> age_ms = ethercat_rx_age_ms(state, now_ms);
  if (age_ms.has_value()) {
    msg.ethercat_rx_age_ms = static_cast<float>(*age_ms);
  }
  msg.ethercat_connected = is_ethercat_connected(state, now_ms);
  // system_state[1] は upper 基板が EtherCAT で送ってくる値なので、EtherCAT 通信中のときだけ信じる
  msg.bottom_connected = msg.ethercat_connected && state.system_state[1] != 0U;
  return msg;
}

}  // namespace core_hardware

class CoreHardware : public rclcpp::Node {
 public:
  CoreHardware();

 private:
  static constexpr uint8_t kJointNum = 15;

  rclcpp::Publisher<core_msgs::msg::CANArray>::SharedPtr can_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr wireless_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr destroy_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr hp_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr color_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hardware_emergency_pub_;
  rclcpp::Publisher<core_msgs::msg::MotorStatusArray>::SharedPtr motor_status_pub_;
  rclcpp::Publisher<core_msgs::msg::HardwareLinkStatus>::SharedPtr link_status_pub_;
  rclcpp::Subscription<core_msgs::msg::CANArray>::SharedPtr can_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr led_upper_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr led_bottom_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr led_bottom2_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  core_hardware::IpcClient client_;
  core_hardware::HardwareSnapshot latest_command_{};
  core_hardware::HardwareSnapshot latest_state_{};
  core_msgs::msg::CANArray pending_can_array_;
  std::string socket_path_;
  bool connected_ = false;
  bool ethercat_connected_ = false;
  uint32_t sequence_ = 0;
  std::chrono::system_clock::time_point last_no_connect_log_tp_{};
  sensor_msgs::msg::JointState joint_states_;
  std_msgs::msg::UInt8MultiArray wireless_;
  std_msgs::msg::Bool destroy_;
  std_msgs::msg::UInt8 hp_;
  std_msgs::msg::UInt8 color_;
  std_msgs::msg::Bool hardware_emergency_;

  void timer_cb();
  void status_timer_cb();
  void can_cb(const core_msgs::msg::CANArray::SharedPtr msg);
  void ensure_connected();
  void update_ethercat_connection_log();
  void handle_message(core_hardware::IpcMessageType type, uint32_t sequence, const std::vector<uint8_t>& payload);
  void handle_state_snapshot(const core_hardware::HardwareSnapshot& snapshot);
  void handle_float_packet(uint8_t id, const std::vector<float>& data);
  void handle_uint8_packet(uint8_t id, const std::vector<uint8_t>& data);
  void led_upper_cb(const std_msgs::msg::UInt8::SharedPtr msg);
  void led_bottom_cb(const std_msgs::msg::UInt8::SharedPtr msg);
  void led_bottom2_cb(const std_msgs::msg::UInt8::SharedPtr msg);
};
