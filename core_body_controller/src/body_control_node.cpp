#include "core_body_controller/body_control_node.hpp"

#include <algorithm>

constexpr double TIMER_PERIOD = 0.01; // 10 ms

BodyControlNode::BodyControlNode() : Node("body_control_node") {
  body_control_command_pub_ = this->create_publisher<core_msgs::msg::CANArray>("can/tx", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(TIMER_PERIOD * 1000)),
                                   std::bind(&BodyControlNode::timer_callback, this));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        latest_twist_ = *msg;
      });
  emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/system/emergency/hazard_status", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        emergency_stop_flag_ = msg->data;
      });

  body_target_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "body_target_angle", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
        body_target_angle_ = msg->data;
      });
  body_omega_ = this->create_publisher<std_msgs::msg::Float64>("body_omega", 10);
  rotation_flag_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "rotation_flag", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        rotation_flag_ = msg->data;
      });
  sub_shooter_angle_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        latest_body_angle_ = msg->position[4];
      });
  pad_up_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "pad/up", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          AUTO_ROTATION_VELOCITY = 1 * M_PI;
        }
      });
  pad_up_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "pad/up", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          AUTO_ROTATION_VELOCITY = 2 * M_PI;
        }
      });
}

void BodyControlNode::timer_callback() {
  if (emergency_stop_flag_) {
    RCLCPP_ERROR(this->get_logger(), "Emergency stop flag is set");
    emergency_stop();
    return;
  }

  auto apply_rate_limit = [](double current, double target, double max_step_per_tick) {
    const double error = target - current;
    const double clamped_step = std::clamp(error, -max_step_per_tick, max_step_per_tick);
    return current + clamped_step;
  };

  const double linear_step = ACCELERATION * TIMER_PERIOD;
  cmd_vel_.linear.x = apply_rate_limit(cmd_vel_.linear.x, latest_twist_.linear.x, linear_step);
  cmd_vel_.linear.y = apply_rate_limit(cmd_vel_.linear.y, latest_twist_.linear.y, linear_step);

  if (rotation_flag_) {
    cmd_vel_.angular.z = AUTO_ROTATION_VELOCITY;
  } else {
    const double angular_step = ROTATION_ACCELERATION * TIMER_PERIOD;
    cmd_vel_.angular.z =
        apply_rate_limit(cmd_vel_.angular.z, latest_twist_.angular.z, angular_step);
  }

  if (std::abs(cmd_vel_.linear.x) < 0.01) {
    cmd_vel_.linear.x = 0;
  }
  if (std::abs(cmd_vel_.linear.y) < 0.01) {
    cmd_vel_.linear.y = 0;
  }
  if (std::abs(cmd_vel_.angular.z) < 0.01) {
    cmd_vel_.angular.z = 0;
  }

  if (rotation_flag_) {
    auto body_control_commands =
        gen_body_control_command(invert_kinematics_calc(cmd_vel_, latest_body_angle_));
    // invert_kinematics_calc(cmd_vel_, body_target_angle_));
    body_control_command_pub_->publish(body_control_commands);
  } else {
    auto body_control_commands =
        gen_body_control_command(invert_kinematics_calc(cmd_vel_, body_target_angle_));
    body_control_command_pub_->publish(body_control_commands);
  }
  std_msgs::msg::Float64 body_omega_msg;
  body_omega_msg.data = cmd_vel_.angular.z;
  body_omega_->publish(body_omega_msg);
}

void BodyControlNode::emergency_stop() {
  RCLCPP_ERROR(this->get_logger(), "Emergency stop flag is set");
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
  emergency_stop_flag_ = true;
  RCLCPP_INFO(this->get_logger(), "Emergency stop completed");
}

std::vector<float> BodyControlNode::invert_kinematics_calc(const geometry_msgs::msg::Twist& cmd_vel,
                                                           const float& body_angle) {
  std::vector<float> wheel_velocities(4);

  constexpr float WHEEL_RADIUS = 0.13 / 2;
  // tentatively set the same value to width and length
  constexpr float BODY_WIDTH = 0.5304;

  RCLCPP_INFO(this->get_logger(),
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
      (vx_body * cos(M_PI / 4) - vy_body * sin(M_PI / 4) + BODY_WIDTH * omega) / WHEEL_RADIUS;
  wheel_velocities[1] =
      (vx_body * cos(M_PI / 4) + vy_body * sin(M_PI / 4) + BODY_WIDTH * omega) / WHEEL_RADIUS;
  wheel_velocities[2] =
      (-vx_body * cos(M_PI / 4) + vy_body * sin(M_PI / 4) + BODY_WIDTH * omega) / WHEEL_RADIUS;
  wheel_velocities[3] =
      (-vx_body * cos(M_PI / 4) - vy_body * sin(M_PI / 4) + BODY_WIDTH * omega) / WHEEL_RADIUS;
  return wheel_velocities;
}

core_msgs::msg::CANArray
BodyControlNode::gen_body_control_command(const std::vector<float>& wheel_vel) {
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

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BodyControlNode>());
  rclcpp::shutdown();
  return 0;
}