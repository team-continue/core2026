#pragma once

#include <string>

#include "core_hardware/hardware_snapshot.hpp"
#include "core_hardware/ipc_protocol.hpp"
#include "core_msgs/msg/can_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"

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
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hardware_emergency_pub_;
  rclcpp::Subscription<core_msgs::msg::CANArray>::SharedPtr can_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr led_upper_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr led_bottom_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr led_bottom2_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  core_hardware::IpcClient client_;
  core_hardware::HardwareSnapshot latest_command_{};
  core_hardware::HardwareSnapshot latest_state_{};
  core_msgs::msg::CANArray pending_can_array_;
  std::string socket_path_;
  bool connected_ = false;
  uint32_t sequence_ = 0;
  sensor_msgs::msg::JointState joint_states_;
  std_msgs::msg::UInt8MultiArray wireless_;
  std_msgs::msg::Bool destroy_;
  std_msgs::msg::UInt8 hp_;
  std_msgs::msg::Bool hardware_emergency_;

  void timer_cb();
  void can_cb(const core_msgs::msg::CANArray::SharedPtr msg);
  void ensure_connected();
  void handle_message(core_hardware::IpcMessageType type, uint32_t sequence, const std::vector<uint8_t>& payload);
  void handle_state_snapshot(const core_hardware::HardwareSnapshot& snapshot);
  void handle_float_packet(uint8_t id, const std::vector<float>& data);
  void handle_uint8_packet(uint8_t id, const std::vector<uint8_t>& data);
  void led_upper_cb(const std_msgs::msg::UInt8::SharedPtr msg);
  void led_bottom_cb(const std_msgs::msg::UInt8::SharedPtr msg);
  void led_bottom2_cb(const std_msgs::msg::UInt8::SharedPtr msg);
};
