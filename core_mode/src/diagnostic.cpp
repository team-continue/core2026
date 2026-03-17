#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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
      1000);
    receiver_diagnostic_time_ = this->declare_parameter("receiver_diagnostic_time", 1000);
    diagnostic_cycle_ = this->declare_parameter("diagnostic_cycle", 10);

    RCLCPP_INFO(
      get_logger(),
      "< diagnostic > micon: %d ms, receiver: %d ms, cycle: %d ms",
      microcontroller_diagnostic_time_, receiver_diagnostic_time_, diagnostic_cycle_);

    //========================================
    // subscribers
    //========================================
    microcontroller_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "microcontroller_monitor", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr) {
        microcontroller_lasttime_ = get_clock()->now();
      });

    receive_module_subscription_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
      "receive_module_monitor", 10,
      [this](const std_msgs::msg::UInt8MultiArray::SharedPtr) {
        receiver_lasttime_ = get_clock()->now();
      });

    //========================================
    // publishers
    //========================================
    microcontroller_diagnostic_publisher_ =
      create_publisher<std_msgs::msg::Bool>("microcontroller_emergency", 10);
    receiver_diagnostic_publisher_ =
      create_publisher<std_msgs::msg::Bool>("receiver_emergency", 10);

    //========================================
    // timer
    //========================================
    timer_ = create_wall_timer(
      std::chrono::milliseconds(diagnostic_cycle_),
      std::bind(&Diagnostic::diagnostic, this));

    microcontroller_lasttime_ = get_clock()->now();
    receiver_lasttime_ = get_clock()->now();
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

    // Receiver emergency check
    msg.data = is_timeout(receiver_lasttime_, receiver_diagnostic_time_);
    receiver_diagnostic_publisher_->publish(msg);
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
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr receive_module_subscription_;

  //========================================
  // publisher members
  //========================================
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr microcontroller_diagnostic_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr receiver_diagnostic_publisher_;

  //========================================
  // timer callback member variable
  //========================================
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time microcontroller_lasttime_;
  rclcpp::Time receiver_lasttime_;

  //========================================
  // parameter variables
  //========================================
  int microcontroller_diagnostic_time_;
  int receiver_diagnostic_time_;
  int diagnostic_cycle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Diagnostic>());
  rclcpp::shutdown();
  return 0;
}
