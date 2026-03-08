#pragma once

#include <string>

#include "core_hardware/ecat.hpp"
#include "core_msgs/msg/can.hpp"
#include "core_msgs/msg/can_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

class CoreHardware : public rclcpp::Node {
 public:
  CoreHardware();

 private:
  static constexpr uint8_t kJointNum = 15;
  static constexpr int kReceiveTimeoutUs = 3000;
  static constexpr int kMaxMissedCycles = 10;

  rclcpp::Publisher<core_msgs::msg::CANArray>::SharedPtr can_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr str_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr destroy_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr hp_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hardware_emergency_pub_;
  rclcpp::Subscription<core_msgs::msg::CANArray>::SharedPtr can_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Ecat ecat_;
  uint8_t rx_data_[ECAT_BUFFER_SIZE]{};
  uint8_t tx_data_[ECAT_BUFFER_SIZE]{};
  bool connected_ = false;
  int missed_cycles_ = 0;
  int tx_len_ = 0;
  std::string if_name_;
  sensor_msgs::msg::JointState joint_states_;
  std_msgs::msg::String wireless_;
  std_msgs::msg::Bool destroy_;
  std_msgs::msg::UInt8 hp_;
  std_msgs::msg::Bool hardware_emergency_;

  void timer_cb();
  void can_cb(const core_msgs::msg::CANArray::SharedPtr msg);
};
