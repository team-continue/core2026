#include "core_cmd_vel_smoother/cmd_vel_smoother_node.hpp"

CmdVelSmootherNode::CmdVelSmootherNode()
: Node("cmd_vel_smoother_node")
{
  alpha_ = this->declare_parameter<double>("alpha", 0.3);
  timeout_sec_ = this->declare_parameter<double>("timeout_sec", 0.2);
  auto input_topic = this->declare_parameter<std::string>("input_topic", "/cmd_vel_raw");
  auto output_topic = this->declare_parameter<std::string>("output_topic", "/cmd_vel");

  RCLCPP_INFO(
    this->get_logger(), "cmd_vel_smoother: alpha=%.2f, timeout=%.2fs, %s -> %s",
    alpha_, timeout_sec_, input_topic.c_str(), output_topic.c_str());

  pub_ = this->create_publisher<geometry_msgs::msg::Twist>(output_topic, 10);
  sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_topic, 10, std::bind(&CmdVelSmootherNode::on_cmd_vel, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&CmdVelSmootherNode::on_timer, this));
}

void CmdVelSmootherNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  const double a = alpha_;
  const double b = 1.0 - a;

  smoothed_.linear.x = a * msg->linear.x + b * smoothed_.linear.x;
  smoothed_.linear.y = a * msg->linear.y + b * smoothed_.linear.y;
  smoothed_.angular.z = a * msg->angular.z + b * smoothed_.angular.z;

  last_msg_time_ = this->now();
  has_received_ = true;

  pub_->publish(smoothed_);
}

void CmdVelSmootherNode::on_timer()
{
  if (!has_received_) {
    return;
  }

  const double elapsed = (this->now() - last_msg_time_).seconds();
  if (elapsed > timeout_sec_) {
    smoothed_ = geometry_msgs::msg::Twist();
    pub_->publish(smoothed_);
    has_received_ = false;
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "cmd_vel_smoother: input timeout (%.2fs), publishing zero", elapsed);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSmootherNode>());
  rclcpp::shutdown();
  return 0;
}
