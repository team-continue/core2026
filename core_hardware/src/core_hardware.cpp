#include <core_hardware/core_hardware.hpp>

#include <chrono>
#include <cstring>
#include <vector>

using std::placeholders::_1;
using namespace std::chrono_literals;

CoreHardware::CoreHardware()
    : rclcpp::Node("core_hardware") {
  declare_parameter("if_name", "eth0");
  if_name_ = get_parameter("if_name").as_string();

  can_pub_ = create_publisher<core_msgs::msg::CANArray>("can/rx", 10);
  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  str_pub_ = create_publisher<std_msgs::msg::String>("wireless", 10);
  destroy_pub_ = create_publisher<std_msgs::msg::Bool>("destroy", 10);
  hp_pub_ = create_publisher<std_msgs::msg::UInt8>("hp", 10);
  hardware_emergency_pub_ = create_publisher<std_msgs::msg::Bool>("hardware_emergency", 10);

  can_sub_ = create_subscription<core_msgs::msg::CANArray>(
      "can/tx", 10, std::bind(&CoreHardware::can_cb, this, _1));
  timer_ = create_wall_timer(10ms, std::bind(&CoreHardware::timer_cb, this));

  joint_states_.name = std::vector<std::string>{};
  joint_states_.effort.resize(kJointNum, 0.0);
  joint_states_.velocity.resize(kJointNum, 0.0);
  joint_states_.position.resize(kJointNum, 0.0);
}

void CoreHardware::timer_cb() {
  int rx_len = 0;
  int offset = 0;
  core_msgs::msg::CANArray can_array;

  try {
    if (!connected_) {
      ecat_.connect(if_name_.c_str());
      connected_ = true;
      missed_cycles_ = 0;
      RCLCPP_INFO(get_logger(), "Connected to EtherCAT via %s", if_name_.c_str());
      return;
    }

    ecat_.send(tx_data_, static_cast<std::size_t>(tx_len_));
    tx_len_ = 0;
    int wkc = 0;
    if (!ecat_.try_read(rx_data_, kReceiveTimeoutUs, rx_len, wkc)) {
      ++missed_cycles_;
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "EtherCAT cycle missed (%d/%d), wkc=%d", missed_cycles_, kMaxMissedCycles, wkc);
      if (missed_cycles_ >= kMaxMissedCycles) {
        ecat_.close();
        connected_ = false;
        missed_cycles_ = 0;
        tx_len_ = 0;
        RCLCPP_WARN(get_logger(), "EtherCAT reconnecting after %d missed cycles", kMaxMissedCycles);
      }
      return;
    }
    missed_cycles_ = 0;

    while (offset < rx_len) {
      const uint8_t can_id = rx_data_[offset];
      const int uint8_len = rx_data_[offset + 1];
      const uint8_t* uint8_addr = &rx_data_[offset + 2];
      offset += uint8_len + 2;

      if (uint8_len == 0) {
        continue;
      }
      if (can_id < kJointNum && uint8_len == static_cast<int>(sizeof(float) * 6U)) {
        std::vector<float> values(6, 0.0f);
        std::memcpy(values.data(), uint8_addr, static_cast<std::size_t>(uint8_len));

        core_msgs::msg::CAN can;
        can.id = can_id;
        can.data = values;
        can_array.array.push_back(can);
        joint_states_.effort[can_id] = values[1];
        joint_states_.velocity[can_id] = values[3];
        joint_states_.position[can_id] = values[5];
      } else if (can_id == 100 && uint8_len == 1) {
        hp_.data = uint8_addr[0];
        hp_pub_->publish(hp_);
      } else if (can_id == 101 && uint8_len == 1) {
        destroy_.data = (uint8_addr[0] == 1U);
        destroy_pub_->publish(destroy_);
      } else if (can_id == 102) {
        wireless_.data.assign(reinterpret_cast<const char*>(uint8_addr), static_cast<std::size_t>(uint8_len));
        str_pub_->publish(wireless_);
      } else if (can_id == 104 && uint8_len == 1) {
        hardware_emergency_.data = (uint8_addr[0] == 1U);
        hardware_emergency_pub_->publish(hardware_emergency_);
      }
    }

    if (!can_array.array.empty()) {
      joint_states_.header.stamp = now();
      can_pub_->publish(can_array);
      joint_pub_->publish(joint_states_);
    }
  } catch (const std::exception& ex) {
    ecat_.close();
    connected_ = false;
    missed_cycles_ = 0;
    tx_len_ = 0;
    RCLCPP_WARN(get_logger(), "EtherCAT communication error: %s", ex.what());
    rclcpp::sleep_for(1000ms);
  }
}

void CoreHardware::can_cb(const core_msgs::msg::CANArray::SharedPtr msg) {
  for (const auto& can : msg->array) {
    const int uint8_len = static_cast<int>(can.data.size() * sizeof(float));
    if ((tx_len_ + uint8_len + 2) > static_cast<int>(ECAT_BUFFER_SIZE)) {
      RCLCPP_WARN(get_logger(), "Dropping TX packet because buffer is full");
      return;
    }
    tx_data_[tx_len_] = can.id;
    tx_data_[tx_len_ + 1] = static_cast<uint8_t>(uint8_len);
    std::memcpy(tx_data_ + tx_len_ + 2, can.data.data(), static_cast<std::size_t>(uint8_len));
    tx_len_ += uint8_len + 2;
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoreHardware>());
  rclcpp::shutdown();
  return 0;
}
