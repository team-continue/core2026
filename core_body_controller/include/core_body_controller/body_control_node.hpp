#include <chrono>
#include <cmath>

#include "core_msgs/msg/can.hpp"
#include "core_msgs/msg/can_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"

class BodyControlNode : public rclcpp::Node
{
public:
  BodyControlNode();

private:
  rclcpp::Publisher<core_msgs::msg::CANArray>::SharedPtr
    body_control_command_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr body_omega_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rotation_flag_sub_;

  void timer_callback();
  std::vector<float> invert_kinematics_calc(
    const geometry_msgs::msg::Twist & cmd_vel, const float & body_angle = 0);
  core_msgs::msg::CANArray gen_body_control_command(
    const std::vector<float> & body_control_command);
  void emergency_stop();

  geometry_msgs::msg::Twist cmd_vel_;
  geometry_msgs::msg::Twist latest_twist_;
  bool emergency_stop_flag_ = true;
  bool has_received_cmd_vel_ = false;
  int rotation_mode_ = 0;
  double latest_body_angle_ = 0;
  std::chrono::steady_clock::time_point last_cmd_vel_time_;

  double ACCELERATION = 2;  // m/s^2
  double ROTATION_ACCELERATION = 1.0;
  double AUTO_ROTATION_VELOCITY = 0.3 * M_PI;
  double HIGH_ROTATION_VELOCITY = 1.0 * M_PI;
  double CMD_VEL_TIMEOUT_SEC = 0.2;

  static constexpr double TIMER_PERIOD = 0.01;  // 10 ms
};
