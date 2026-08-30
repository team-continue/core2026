#include "core_body_controller/body_control_node.hpp"

#include <algorithm>
#include <stdexcept>

constexpr double INITIAL_ANGLE = 3.7822790145874023;

BodyControlNode::BodyControlNode()
: Node("body_control_node")
{
  ACCELERATION = this->declare_parameter<double>("acceleration", ACCELERATION);
  ROTATION_ACCELERATION =
    this->declare_parameter<double>("rotation_acceleration", ROTATION_ACCELERATION);
  AUTO_ROTATION_VELOCITY =
    this->declare_parameter<double>("auto_rotation_velocity", AUTO_ROTATION_VELOCITY);
  HIGH_ROTATION_VELOCITY =
    this->declare_parameter<double>("high_rotation_velocity", HIGH_ROTATION_VELOCITY);
  CMD_VEL_TIMEOUT_SEC =
    this->declare_parameter<double>("cmd_vel_timeout_sec", CMD_VEL_TIMEOUT_SEC);
  if (!std::isfinite(ACCELERATION) || !std::isfinite(ROTATION_ACCELERATION) ||
    !std::isfinite(AUTO_ROTATION_VELOCITY) || !std::isfinite(HIGH_ROTATION_VELOCITY) ||
    !std::isfinite(CMD_VEL_TIMEOUT_SEC) || ACCELERATION <= 0.0 ||
    ROTATION_ACCELERATION <= 0.0 || CMD_VEL_TIMEOUT_SEC <= 0.0)
  {
    throw std::invalid_argument(
            "velocity parameters must be finite; acceleration, rotation_acceleration, and "
            "cmd_vel_timeout_sec must also be positive");
  }

  body_control_command_pub_ = this->create_publisher<core_msgs::msg::CANArray>("can/tx", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(TIMER_PERIOD * 1000)),
    std::bind(&BodyControlNode::timer_callback, this));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) ||
      !std::isfinite(msg->angular.z))
      {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Ignoring cmd_vel containing a non-finite control component");
        return;
      }
      latest_twist_ = *msg;
      last_cmd_vel_time_ = std::chrono::steady_clock::now();
      has_received_cmd_vel_ = true;
    });
  emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/system/emergency/hazard_status", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
      emergency_stop_flag_ = msg->data;
    });
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
      if (msg->position.size() <= 4 || !std::isfinite(msg->position[4])) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Ignoring invalid joint_states: position size=%zu (need finite position[4])",
          msg->position.size());
        return;
      }
      latest_body_angle_ = msg->position[4] - INITIAL_ANGLE;
    });
  body_omega_ = this->create_publisher<std_msgs::msg::Float64>("body_omega", 10);
  rotation_flag_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/rotation", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
      rotation_mode_ = msg->data;
    });
}

void BodyControlNode::timer_callback()
{
  if (emergency_stop_flag_) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Emergency stop flag is set");
    emergency_stop();
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const bool cmd_vel_is_fresh =
    has_received_cmd_vel_ &&
    std::chrono::duration<double>(now - last_cmd_vel_time_).count() <= CMD_VEL_TIMEOUT_SEC;
  geometry_msgs::msg::Twist target_twist;
  if (cmd_vel_is_fresh) {
    target_twist = latest_twist_;
  } else if (has_received_cmd_vel_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "cmd_vel timed out; decelerating body command to zero");
  }

  auto apply_rate_limit = [](double current, double target, double max_step_per_tick) {
      const double error = target - current;
      const double clamped_step = std::clamp(error, -max_step_per_tick, max_step_per_tick);
      return current + clamped_step;
    };

  const double linear_step = ACCELERATION * TIMER_PERIOD;
  cmd_vel_.linear.x = apply_rate_limit(cmd_vel_.linear.x, target_twist.linear.x, linear_step);
  cmd_vel_.linear.y = apply_rate_limit(cmd_vel_.linear.y, target_twist.linear.y, linear_step);

  const double angular_step = ROTATION_ACCELERATION * TIMER_PERIOD;
  double rotation_velocity = 0.0;
  if (cmd_vel_is_fresh && rotation_mode_ == 1) {
    rotation_velocity = AUTO_ROTATION_VELOCITY;
  } else if (cmd_vel_is_fresh && rotation_mode_ == 2) {
    rotation_velocity = HIGH_ROTATION_VELOCITY;
  }
  const double target_angular_z = rotation_velocity + target_twist.angular.z;
  cmd_vel_.angular.z = apply_rate_limit(cmd_vel_.angular.z, target_angular_z, angular_step);

  if (std::abs(cmd_vel_.linear.x) < 0.01) {
    cmd_vel_.linear.x = 0;
  }
  if (std::abs(cmd_vel_.linear.y) < 0.01) {
    cmd_vel_.linear.y = 0;
  }
  if (std::abs(cmd_vel_.angular.z) < 0.01) {
    cmd_vel_.angular.z = 0;
  }

  auto body_control_commands =
    gen_body_control_command(invert_kinematics_calc(cmd_vel_, latest_body_angle_));
  body_control_command_pub_->publish(body_control_commands);

  std_msgs::msg::Float64 body_omega_msg;
  body_omega_msg.data = cmd_vel_.angular.z;
  body_omega_->publish(body_omega_msg);
}

void BodyControlNode::emergency_stop()
{
  core_msgs::msg::CANArray body_control_command_array;
  for (size_t i = 0; i < 4; i++) {
    core_msgs::msg::CAN body_control_command;
    body_control_command.id = i;
    body_control_command.data.push_back(20);
    body_control_command.data.push_back(0);
    body_control_command_array.array.push_back(body_control_command);
  }
  body_control_command_pub_->publish(body_control_command_array);
  cmd_vel_.linear.x = 0;
  cmd_vel_.linear.y = 0;
  cmd_vel_.linear.z = 0;
  cmd_vel_.angular.x = 0;
  cmd_vel_.angular.y = 0;
  cmd_vel_.angular.z = 0;
}

std::vector<float> BodyControlNode::invert_kinematics_calc(
  const geometry_msgs::msg::Twist & cmd_vel,
  const float & body_angle)
{
  std::vector<float> wheel_velocities(4);

  constexpr float WHEEL_RADIUS = 0.13 / 2;
  constexpr float SQRT2 = 1.41421356237f;
  constexpr float BODY_WIDTH = 0.5304f;

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 200,
    "Got cmd_vel: linear.x=%f, linear.y=%f, angular.z=%f, body_angle=%f",
    cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, body_angle);

  // Rotate velocity vector to body frame
  float vx_body = cmd_vel.linear.x * cos(body_angle) + cmd_vel.linear.y * sin(body_angle);
  float vy_body = -cmd_vel.linear.x * sin(body_angle) + cmd_vel.linear.y * cos(body_angle);
  float omega = cmd_vel.angular.z;

  // Standard omni wheel inverse kinematics
  // Wheel arrangement (looking from top):
  //        x
  //   1 [/]  [\] 0
  //   2 [\]  [/] 3
  //

  wheel_velocities[0] =
    -(vx_body * cos(M_PI / 4) - vy_body * sin(M_PI / 4) - SQRT2 * BODY_WIDTH / 2 * omega) /
    WHEEL_RADIUS;
  wheel_velocities[1] =
    -(vx_body * cos(M_PI / 4) + vy_body * sin(M_PI / 4) - SQRT2 * BODY_WIDTH / 2 * omega) /
    WHEEL_RADIUS;
  wheel_velocities[2] =
    -(-vx_body * cos(M_PI / 4) + vy_body * sin(M_PI / 4) - SQRT2 * BODY_WIDTH / 2 * omega) /
    WHEEL_RADIUS;
  wheel_velocities[3] =
    -(-vx_body * cos(M_PI / 4) - vy_body * sin(M_PI / 4) - SQRT2 * BODY_WIDTH / 2 * omega) /
    WHEEL_RADIUS;
  return wheel_velocities;
}

core_msgs::msg::CANArray
BodyControlNode::gen_body_control_command(const std::vector<float> & wheel_vel)
{
  core_msgs::msg::CANArray body_control_commands;
  for (size_t i = 0; i < 4; i++) {
    core_msgs::msg::CAN body_control_command;
    body_control_command.id = i;
    body_control_command.data.push_back(20);
    body_control_command.data.push_back(wheel_vel[i]);
    body_control_commands.array.push_back(body_control_command);
  }
  return body_control_commands;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BodyControlNode>());
  rclcpp::shutdown();
  return 0;
}
