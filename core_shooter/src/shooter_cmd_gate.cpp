#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include <stdexcept>


class ShooterCmdGate : public rclcpp::Node
{
public:
  ShooterCmdGate()
  : Node("shooter_cmd_gate")
  {
    //========================================
    // parameters
    //========================================
    this->declare_parameter<int>("burst_count", 3);
    this->get_parameter("burst_count", burst_count_);
    if (burst_count_ <= 0) {
      RCLCPP_FATAL(
        this->get_logger(), "Invalid parameter burst_count=%d (must be > 0)", burst_count_);
      throw std::runtime_error("invalid burst_count");
    }

    //========================================
    // subscribers ui
    //========================================
    left_once_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "left_shoot_once", 1,
      std::bind(&ShooterCmdGate::leftOnceCallback, this, std::placeholders::_1));
    left_burst_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "left_shoot_burst", 1,
      std::bind(&ShooterCmdGate::leftBurstCallback, this, std::placeholders::_1));
    left_fullauto_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "left_shoot_fullauto", 1,
      std::bind(&ShooterCmdGate::leftFullautoCallback, this, std::placeholders::_1));

    right_once_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "right_shoot_once", 1,
      std::bind(&ShooterCmdGate::rightOnceCallback, this, std::placeholders::_1));
    right_burst_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "right_shoot_burst", 1,
      std::bind(&ShooterCmdGate::rightBurstCallback, this, std::placeholders::_1));
    right_fullauto_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "right_shoot_fullauto", 1,
      std::bind(&ShooterCmdGate::rightFullautoCallback, this, std::placeholders::_1));

    //========================================
    // publishers
    //========================================
    left_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "left_shoot_cmd", 10);
    right_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "right_shoot_cmd", 10);

  }

  void leftOnceCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      publishLeftCmd(1);
      RCLCPP_INFO(this->get_logger(), "On trigger: Left Once");
    }
  }

  void leftBurstCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      publishLeftCmd(burst_count_);
      RCLCPP_INFO(this->get_logger(), "On trigger: Left Burst (%d)", burst_count_);
    }
  }

  void leftFullautoCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    processFullautoInput(msg->data, left_fullauto_input_prev_, left_fullauto_enabled_, true);
  }

  void rightOnceCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      publishRightCmd(1);
      RCLCPP_INFO(this->get_logger(), "On trigger: Right Once");
    }
  }

  void rightBurstCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      publishRightCmd(burst_count_);
      RCLCPP_INFO(this->get_logger(), "On trigger: Right Burst (%d)", burst_count_);
    }
  }

  void rightFullautoCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    processFullautoInput(msg->data, right_fullauto_input_prev_, right_fullauto_enabled_, false);
  }

  void processFullautoInput(
    bool input, bool & prev_input, bool & enabled, bool is_left)
  {
    const bool rising = input && !prev_input;
    const bool falling = !input && prev_input;
    prev_input = input;

    if (rising && !enabled) {
      enabled = true;
      if (is_left) {
        publishLeftCmd(-1);
        RCLCPP_INFO(this->get_logger(), "On trigger: Left Fullauto");
      } else {
        publishRightCmd(-1);
        RCLCPP_INFO(this->get_logger(), "On trigger: Right Fullauto");
      }
    } else if (falling && enabled) {
      enabled = false;
      if (is_left) {
        publishLeftCmd(0);
      } else {
        publishRightCmd(0);
      }
    }
  }

  void publishLeftCmd(int repeat_count)
  {
    std_msgs::msg::Int32 msg;
    msg.data = repeat_count;
    left_cmd_pub_->publish(msg);
  }

  void publishRightCmd(int repeat_count)
  {
    std_msgs::msg::Int32 msg;
    msg.data = repeat_count;
    right_cmd_pub_->publish(msg);
  }

  //========================================
  // Subscription valids
  //========================================
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_once_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_burst_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_fullauto_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_once_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_burst_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_fullauto_sub_;

  //========================================
  // publisher valids
  //========================================
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_cmd_pub_;

  //========================================
  // valids
  //========================================
  bool left_fullauto_enabled_ = false;
  bool right_fullauto_enabled_ = false;
  bool left_fullauto_input_prev_ = false;
  bool right_fullauto_input_prev_ = false;
  int burst_count_ = 3;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShooterCmdGate>());

  rclcpp::shutdown();
  return 0;
}
