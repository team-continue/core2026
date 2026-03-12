#ifndef CORE_CMD_VEL_SMOOTHER__CMD_VEL_SMOOTHER_NODE_HPP_
#define CORE_CMD_VEL_SMOOTHER__CMD_VEL_SMOOTHER_NODE_HPP_

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class CmdVelSmootherNode : public rclcpp::Node
{
public:
  CmdVelSmootherNode();

private:
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_timer();

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist smoothed_;
  rclcpp::Time last_msg_time_;
  bool has_received_ = false;

  double alpha_;
  double timeout_sec_;
};

#endif  // CORE_CMD_VEL_SMOOTHER__CMD_VEL_SMOOTHER_NODE_HPP_
