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
    sub_shooter_angle_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr body_omega_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr rotation_flag_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pad_up_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pad_down_sub_;

  void timer_callback();
  std::vector<float> invert_kinematics_calc(
    const geometry_msgs::msg::Twist & cmd_vel, const float & body_angle = 0);
  core_msgs::msg::CANArray gen_body_control_command(
    const std::vector<float> & body_control_command);
  void emergency_stop();

  geometry_msgs::msg::Twist cmd_vel_;
  geometry_msgs::msg::Twist latest_twist_;
  bool emergency_stop_flag_ = true;
  int rotation_mode_ = 0;
  float body_angle_ = 0;
  float body_target_angle_ = 0;
  double latest_body_angle_ = 0;

  double ACCELERATION = 2;  // m/s
  double ROTATION_ACCELERATION = 1 * M_PI;
  double YAW_ROTATION_VELOCITY = 4 * M_PI;  // yaw
  double AUTO_ROTATION_VELOCITY = 0.3 * M_PI;
  double HIGH_ROTATION_VELOCITY = 1.0 * M_PI;

  static constexpr double TIMER_PERIOD = 0.01; // 10 ms
};
