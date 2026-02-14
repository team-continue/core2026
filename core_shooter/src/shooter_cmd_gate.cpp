#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"


class ShooterCmdGate : public rclcpp::Node
{
public:
  ShooterCmdGate()
  : Node("shooter_cmd_gate")
  {
    //========================================
    // parameters
    //========================================


    //========================================
    // subscribers ui
    //========================================
    once_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "shoot_once", 1,
      std::bind(&ShooterCmdGate::onceCallback, this, std::placeholders::_1));
    burst_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "shoot_burst", 1,
      std::bind(&ShooterCmdGate::burstCallback, this, std::placeholders::_1));
    fullauto_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "shoot_fullauto", 1,
      std::bind(&ShooterCmdGate::fullautoCallback, this, std::placeholders::_1));
    fullburst_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "shoot_fullburst", 1,
      std::bind(&ShooterCmdGate::fullburstCallback, this, std::placeholders::_1));
    left_shoulder_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "shoot_left_shoulder", 1,
      std::bind(&ShooterCmdGate::shootLeftCallback, this, std::placeholders::_1));
    right_shoulder_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "shoot_right_shoulder", 1,
      std::bind(&ShooterCmdGate::shootRightCallback, this, std::placeholders::_1));

    //========================================
    // publishers
    //========================================
    center_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "center_shoot_cmd", 10);
    left_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "left_shoot_cmd", 10);
    right_cmd_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "right_shoot_cmd", 10);

  }

  void onceCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      publishCenterCmd(1);
      RCLCPP_INFO(this->get_logger(), "On trigger: Shoot");
    }
  }

  void burstCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      publishCenterCmd(3);
      RCLCPP_INFO(this->get_logger(), "On trigger: Burst");
    }
  }

  void fullautoCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && !fullauto_enabled_) {
      fullauto_enabled_ = true;
      publishCenterCmd(-1);
      RCLCPP_INFO(this->get_logger(), "On trigger: Fullauto");
    } else if (!msg->data && fullauto_enabled_) {
      fullauto_enabled_ = false;
      publishCenterCmd(0);
    }
  }

  void fullburstCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      publishCenterCmd(1);
      publishLeftCmd(1);
      publishRightCmd(1);
      RCLCPP_INFO(this->get_logger(), "On trigger: Fullburst");
    }
  }

  void shootLeftCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      publishLeftCmd(1);
      RCLCPP_INFO(this->get_logger(), "On trigger: Left Shoot");
    }
  }

  void shootRightCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) {
      publishRightCmd(1);
      RCLCPP_INFO(this->get_logger(), "On trigger: Right Shoot");
    }
  }

  void publishCenterCmd(int repeat_count)
  {
    std_msgs::msg::Int32 msg;
    msg.data = repeat_count;
    center_cmd_pub_->publish(msg);
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
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr once_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr burst_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fullauto_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fullburst_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr right_shoulder_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr left_shoulder_sub_;

  //========================================
  // publisher valids
  //========================================
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr center_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_cmd_pub_;

  //========================================
  // valids
  //========================================
  bool fullauto_enabled_ = false;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShooterCmdGate>());

  rclcpp::shutdown();
  return 0;
}
