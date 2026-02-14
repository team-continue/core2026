#include <cmath>

#include "core_msgs/msg/can.hpp"
#include "core_msgs/msg/can_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"

class PID {
 public:
  PID(double kp, double ki, double kd, double max, double min) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    max_ = max;
    min_ = min;
    integral_ = 0;
    previous_error_ = 0;
  }
  double calculate(double target, double current) {
    double error = target - current;
    integral_ += error;
    double derivative = error - previous_error_;
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    if (output > max_) {
      output = max_;
    } else if (output < min_) {
      output = min_;
    }
    previous_error_ = error;
    return output;
  }

 private:
  double kp_;
  double ki_;
  double kd_;
  double max_;
  double min_;
  double integral_;
  double previous_error_;
};

class BodyControlNode : public rclcpp::Node {
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
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
      body_target_angle_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr body_omega_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rotation_flag_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pad_up_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pad_down_sub_;

  void timer_callback();
  std::vector<float> invert_kinematics_calc(
      const geometry_msgs::msg::Twist &cmd_vel, const float &body_angle = 0);
  core_msgs::msg::CANArray gen_body_control_command(
      const std::vector<float> &body_control_command);
  void emergency_stop();

  geometry_msgs::msg::Twist cmd_vel_;
  geometry_msgs::msg::Twist latest_twist_;
  bool emergency_stop_flag_ = true;
  bool rotation_flag_ = false;
  float body_angle_ = 0;
  float body_target_angle_ = 0;
  double latest_body_angle_ = 0;

  double ACCERATION = 3;  // m/s
  double ROTATION_ACCERATION = 3 * M_PI;
  double YAW_ROTATION_VELOCITY = 4 * M_PI;  // yaw
  double AUTO_ROTATION_VELOCITY = 1 * M_PI;

  PID body_angle_pid_ =
      PID(1, 0, 0, YAW_ROTATION_VELOCITY, -YAW_ROTATION_VELOCITY);
};