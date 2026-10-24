#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "core_msgs/msg/hardware_link_status.hpp"

class Diagnostic : public rclcpp::Node
{
public:
  Diagnostic()
  : Node("diagnostic")
  {
    //========================================
    // parameters
    //========================================
    microcontroller_diagnostic_time_ = this->declare_parameter(
      "microcontroller_diagnostic_time",
      300);
    bridge_diagnostic_time_ = this->declare_parameter("bridge_diagnostic_time", 500);
    diagnostic_cycle_ = this->declare_parameter("diagnostic_cycle", 10);
    // 通信の死活監視方式: "ros2_bridge"(/bridge/connected を監視) | "none"(監視しない)
    const auto connection_monitor =
      this->declare_parameter<std::string>("connection_monitor", "ros2_bridge");
    if (connection_monitor == "ros2_bridge") {
      monitor_bridge_ = true;
    } else if (connection_monitor == "none") {
      monitor_bridge_ = false;
    } else {
      throw std::invalid_argument(
              "connection_monitor must be \"ros2_bridge\" or \"none\", got \"" +
              connection_monitor + "\"");
    }

    RCLCPP_INFO(
      get_logger(),
      "< diagnostic > micon: %d ms, bridge: %d ms, cycle: %d ms, connection_monitor: %s",
      microcontroller_diagnostic_time_, bridge_diagnostic_time_, diagnostic_cycle_,
      connection_monitor.c_str());

    //========================================
    // publishers
    //========================================
    microcontroller_diagnostic_publisher_ =
      create_publisher<std_msgs::msg::Bool>("microcontroller_timeout", 10);
    bridge_diagnostic_publisher_ =
      create_publisher<std_msgs::msg::Bool>("bridge_disconnected", 10);
    bottom_can_diagnostic_publisher_ =
      create_publisher<std_msgs::msg::Bool>("bottom_can_disconnected", 10);

    //========================================
    // subscribers
    //========================================
    microcontroller_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "microcontroller_monitor", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr) {
        microcontroller_lasttime_ = get_clock()->now();
      });

    // bridge は接続状態を周期送信する。途絶えたらタイムアウト、false なら即時に通信断とする
    if (monitor_bridge_) {
      bridge_subscription_ = create_subscription<std_msgs::msg::Bool>(
        "bridge_connected", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          bridge_lasttime_ = get_clock()->now();
          const bool was_connected = bridge_connected_;
          bridge_connected_ = msg->data;
          if (was_connected && !bridge_connected_) {
            publish_bridge_state();
          }
        });
    }

    // 上下基板間の CAN 断は upper 基板が検出し、EtherCAT 経由で link_status.bottom_connected に載る。
    // EtherCAT 自体が切れている場合は microcontroller_timeout で検出するため、ここでは CAN 断としない
    link_status_subscription_ = create_subscription<core_msgs::msg::HardwareLinkStatus>(
      "link_status", 10,
      [this](const core_msgs::msg::HardwareLinkStatus::SharedPtr msg) {
        const bool was_disconnected = bottom_can_disconnected_;
        bottom_can_disconnected_ = msg->ethercat_connected && !msg->bottom_connected;
        if (!was_disconnected && bottom_can_disconnected_) {
          publish_bottom_can_state();
        }
      });

    //========================================
    // timer
    //========================================
    timer_ = create_wall_timer(
      std::chrono::milliseconds(diagnostic_cycle_),
      std::bind(&Diagnostic::diagnostic, this));

    microcontroller_lasttime_ = get_clock()->now();
    bridge_lasttime_ = get_clock()->now();
  }

private:
  //========================================
  // main logic
  //========================================
  void diagnostic()
  {
    std_msgs::msg::Bool msg;

    // Microcontroller emergency check
    msg.data = is_timeout(microcontroller_lasttime_, microcontroller_diagnostic_time_);
    microcontroller_diagnostic_publisher_->publish(msg);

    // Bridge emergency check
    publish_bridge_state();

    // Bottom CAN emergency check
    publish_bottom_can_state();
  }

  void publish_bottom_can_state()
  {
    std_msgs::msg::Bool msg;
    msg.data = bottom_can_disconnected_;
    bottom_can_diagnostic_publisher_->publish(msg);
  }

  void publish_bridge_state()
  {
    std_msgs::msg::Bool msg;
    msg.data = monitor_bridge_ &&
      (!bridge_connected_ || is_timeout(bridge_lasttime_, bridge_diagnostic_time_));
    bridge_diagnostic_publisher_->publish(msg);
  }

  bool is_timeout(const rclcpp::Time & last_time, int threshold_ms)
  {
    double diff_ms = (get_clock()->now() - last_time).seconds() * 1000.0;
    return diff_ms > threshold_ms;
  }

  //========================================
  // Subscription members
  //========================================
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr microcontroller_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bridge_subscription_;
  rclcpp::Subscription<core_msgs::msg::HardwareLinkStatus>::SharedPtr link_status_subscription_;

  //========================================
  // publisher members
  //========================================
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr microcontroller_diagnostic_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bridge_diagnostic_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bottom_can_diagnostic_publisher_;

  //========================================
  // timer callback member variable
  //========================================
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time microcontroller_lasttime_;
  rclcpp::Time bridge_lasttime_;
  bool bridge_connected_ = false;
  bool bottom_can_disconnected_ = false;

  //========================================
  // parameter variables
  //========================================
  int microcontroller_diagnostic_time_;
  int bridge_diagnostic_time_;
  int diagnostic_cycle_;
  bool monitor_bridge_ = true;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Diagnostic>());
  rclcpp::shutdown();
  return 0;
}
