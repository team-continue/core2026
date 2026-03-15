#include <core_hardware/core_hardware.hpp>

#include <chrono>
#include <utility>

using std::placeholders::_1;
using namespace std::chrono_literals;

CoreHardware::CoreHardware()
    : rclcpp::Node("core_hardware") {
  declare_parameter("socket_path", std::string(core_hardware::kDefaultSocketPath));
  socket_path_ = get_parameter("socket_path").as_string();

  can_pub_ = create_publisher<core_msgs::msg::CANArray>("can/rx", 10);
  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  wireless_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("wireless", 10);
  destroy_pub_ = create_publisher<std_msgs::msg::Bool>("destroy", 10);
  hp_pub_ = create_publisher<std_msgs::msg::UInt8>("hp", 10);
  hardware_emergency_pub_ = create_publisher<std_msgs::msg::Bool>("hardware_emergency", 10);

  can_sub_ = create_subscription<core_msgs::msg::CANArray>(
      "can/tx", 10, std::bind(&CoreHardware::can_cb, this, _1));
  led_upper_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "led/upper", 10, std::bind(&CoreHardware::led_upper_cb, this, _1));
  led_bottom_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "led/bottom", 10, std::bind(&CoreHardware::led_bottom_cb, this, _1));
  led_bottom2_sub_ = create_subscription<std_msgs::msg::UInt8>(
      "led/bottom2", 10, std::bind(&CoreHardware::led_bottom2_cb, this, _1));
  timer_ = create_wall_timer(10ms, std::bind(&CoreHardware::timer_cb, this));

  joint_states_.name = std::vector<std::string>{};
  joint_states_.effort.resize(kJointNum, 0.0);
  joint_states_.velocity.resize(kJointNum, 0.0);
  joint_states_.position.resize(kJointNum, 0.0);
}

void CoreHardware::timer_cb() {
  try {
    ensure_connected();
    const auto payload = core_hardware::encode_snapshot(latest_command_);
    client_.send(core_hardware::IpcMessageType::kCommandSnapshot, sequence_++, payload);
    while (client_.poll_once([this](core_hardware::IpcMessageType type, uint32_t seq, const std::vector<uint8_t>& msg_payload) {
      handle_message(type, seq, msg_payload);
    })) {
    }
    if (!pending_can_array_.array.empty()) {
      joint_states_.header.stamp = now();
      can_pub_->publish(pending_can_array_);
      joint_pub_->publish(joint_states_);
      pending_can_array_.array.clear();
    }
    update_ethercat_connection_log();
  } catch (const std::exception& ex) {
    client_.close();
    connected_ = false;
    ethercat_connected_ = false;
    pending_can_array_.array.clear();
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Hardware bridge IPC error: %s", ex.what());
  }
}

void CoreHardware::can_cb(const core_msgs::msg::CANArray::SharedPtr msg) {
  for (const auto& can : msg->array) {
    if (can.id < latest_command_.motor_ref.size()) {
      latest_command_.motor_ref[can.id] = can.data.empty() ? 0.0f : can.data.back();
      continue;
    }
    if (can.id == 17U) {
      latest_command_.system_ref[0] = (can.data.empty() || can.data.back() == 0.0f) ? 0U : 1U;
    }
  }
}

void CoreHardware::ensure_connected() {
  if (connected_) {
    return;
  }
  client_.connect(socket_path_);
  connected_ = true;
  RCLCPP_INFO(get_logger(), "Connected to hardware daemon via %s", socket_path_.c_str());
}

void CoreHardware::update_ethercat_connection_log() {
  const auto now_tp = std::chrono::system_clock::now();
  const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now_tp.time_since_epoch())
                          .count();
  const bool daemon_connected = latest_state_.ethercat_connected[0] != 0U;
  const uint64_t last_rx_ms = latest_state_.last_ethercat_rx_ms[0];
  const bool ethercat_connected =
      daemon_connected &&
      last_rx_ms != 0U &&
      now_ms >= static_cast<int64_t>(last_rx_ms) &&
      (now_ms - static_cast<int64_t>(last_rx_ms)) <= 1000;

  if (ethercat_connected) {
    if (!ethercat_connected_) {
      RCLCPP_INFO(get_logger(), "EtherCAT connected");
      ethercat_connected_ = true;
    }
    return;
  }

  ethercat_connected_ = false;
  if (last_no_connect_log_tp_.time_since_epoch().count() == 0 ||
      (now_tp - last_no_connect_log_tp_) >= std::chrono::seconds(1)) {
    RCLCPP_WARN(get_logger(), "EtherCAT no connect");
    last_no_connect_log_tp_ = now_tp;
  }
}

void CoreHardware::handle_message(core_hardware::IpcMessageType type, uint32_t, const std::vector<uint8_t>& payload) {
  switch (type) {
    case core_hardware::IpcMessageType::kStateSnapshot:
      handle_state_snapshot(core_hardware::decode_snapshot(payload));
      break;
    case core_hardware::IpcMessageType::kFloatPacket: {
      auto packet = core_hardware::decode_packet_message(payload);
      handle_float_packet(packet.first, packet.second);
      break;
    }
    case core_hardware::IpcMessageType::kUint8Packet: {
      auto packet = core_hardware::decode_uint8_packet_message(payload);
      handle_uint8_packet(packet.first, packet.second);
      break;
    }
    case core_hardware::IpcMessageType::kError:
      RCLCPP_WARN(get_logger(), "Hardware daemon reported an error frame");
      break;
    case core_hardware::IpcMessageType::kHeartbeat:
    case core_hardware::IpcMessageType::kCommandSnapshot:
      break;
  }
}

void CoreHardware::handle_state_snapshot(const core_hardware::HardwareSnapshot& snapshot) {
  latest_state_ = snapshot;
}

void CoreHardware::handle_float_packet(uint8_t id, const std::vector<float>& data) {
  if (id < kJointNum && data.size() == 6U) {
    core_msgs::msg::CAN can;
    can.id = id;
    can.data = data;
    pending_can_array_.array.push_back(can);
    joint_states_.effort[id] = data[1];
    joint_states_.velocity[id] = data[3];
    joint_states_.position[id] = data[5];
    return;
  }
}

void CoreHardware::handle_uint8_packet(uint8_t id, const std::vector<uint8_t>& data) {
  if (id == 100U && !data.empty()) {
    hp_.data = data.front();
    hp_pub_->publish(hp_);
    return;
  }
  if (id == 101U && !data.empty()) {
    destroy_.data = (data.front() != 0U);
    destroy_pub_->publish(destroy_);
    return;
  }
  if (id == 102U) {
    wireless_.data = data;
    wireless_pub_->publish(wireless_);
    return;
  }
  if (id == 104U && !data.empty()) {
    hardware_emergency_.data = (data.front() != 0U);
    hardware_emergency_pub_->publish(hardware_emergency_);
  }
}

void CoreHardware::led_upper_cb(const std_msgs::msg::UInt8::SharedPtr msg) {
  latest_command_.led_tape[0] = msg->data;
}

void CoreHardware::led_bottom_cb(const std_msgs::msg::UInt8::SharedPtr msg) {
  latest_command_.led_tape[1] = msg->data;
}

void CoreHardware::led_bottom2_cb(const std_msgs::msg::UInt8::SharedPtr msg) {
  latest_command_.led_tape[2] = msg->data;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoreHardware>());
  rclcpp::shutdown();
  return 0;
}
