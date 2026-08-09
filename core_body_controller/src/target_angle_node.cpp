#include <cmath>
#include <core_msgs/msg/can_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

class PID {
  float kp_, ki_, kd_;
  float integral_;
  float dt_;
  float error_prev_;
  float output_limit_;
  float output_acceleration_limit_;

public:
  PID(float kp, float ki, float kd, float dt, float output_limit, float output_acceleration_limit)
      : kp_(kp), ki_(ki), kd_(kd), integral_(0), dt_(dt), error_prev_(0),
        output_limit_(output_limit), output_acceleration_limit_(output_acceleration_limit) {}

  float update(float error) {
    integral_ += error * dt_;
    float derivative = (error - error_prev_) / dt_;
    float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    output = std::clamp(output, -output_limit_, output_limit_);
    error_prev_ = error;
    return output;
  }
  void reset() {
    integral_ = 0;
    error_prev_ = 0;
  }
  void setGain(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }
};

class TargetAngleNode : public rclcpp::Node {
public:
  TargetAngleNode();

private:
  rclcpp::Publisher<core_msgs::msg::CANArray>::SharedPtr can_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr world_target_angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr body_omega_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_omega_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rotation_sub_;

  constexpr static double MAX_ROTATION = M_PI * 2.5;
  constexpr static double MAX_ROTATE_ACCELERATION = M_PI * 3;
  constexpr static std::chrono::milliseconds TIMER_PERIOD = std::chrono::milliseconds(10);
  constexpr static double TIMER_DT = std::chrono::duration<double>(TIMER_PERIOD).count();

  PID pid_ = PID(2.0, 0.0, 0.00, TIMER_DT, MAX_ROTATION, MAX_ROTATE_ACCELERATION);
  double gimbalControl();

  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();
  double world_target_angle_ = 0;
  double latest_world_angle_ = 0;
  double latest_body_omega_ = 0;
  bool emergency_stop_flag_ = true;
  bool node_initialized_ = false;
  bool rotation_flag_ = false;
  geometry_msgs::msg::Twist latest_twist_;

  static double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }
};

TargetAngleNode::TargetAngleNode() : Node("target_angle_node") {
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        latest_twist_ = *msg;
      });
  timer_ = this->create_wall_timer(TIMER_PERIOD, std::bind(&TargetAngleNode::timer_callback, this));
  can_pub_ = this->create_publisher<core_msgs::msg::CANArray>("can/tx", 10);
  // rotation_sub_ = this->create_subscription<std_msgs::msg::Bool>(
  //     "/rotation", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
  //       if (rotation_flag_ && !msg->data) {
  //         pid_.reset();
  //       }
  //       rotation_flag_ = msg->data;
  //     });
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        node_initialized_ = true;
        // quaternion to euler
        // latest_world_angle_ = std::atan2(2.0 * (msg->orientation.w * msg->orientation.z + msg->orientation.x * msg->orientation.y),
        //                               1.0 - 2.0 * (msg->orientation.y * msg->orientation.y + msg->orientation.z * msg->orientation.z));

        // only use yaw omega
        latest_world_angle_ = normalizeAngle(latest_world_angle_ - msg->angular_velocity.z * TIMER_DT + M_PI * 0.0001);
      });
  world_target_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "yaw_target_angle", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
        world_target_angle_ = msg->data;
      });
  body_omega_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "body_omega", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
        latest_body_omega_ = msg->data;
      });
  emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/system/emergency/hazard_status", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        emergency_stop_flag_ = msg->data;
      });
  target_omega_pub_ = this->create_publisher<std_msgs::msg::Float64>("target_omega", 10);
}

double TargetAngleNode::gimbalControl() {
  world_target_angle_ += latest_twist_.angular.z * TIMER_DT;
  world_target_angle_ = normalizeAngle(world_target_angle_);
  RCLCPP_INFO(this->get_logger(), "world_target_angle_: %f", world_target_angle_);
  RCLCPP_INFO(this->get_logger(), "latest_world_angle_: %f", latest_world_angle_);
  const double world_angle_error =
      normalizeAngle(world_target_angle_ - latest_world_angle_);
  if(rotation_flag_) {
    return -pid_.update(world_angle_error) + latest_body_omega_;
  }else {
    return -pid_.update(world_angle_error);
  }
}

void TargetAngleNode::timer_callback() {
  if (!node_initialized_) {
    RCLCPP_WARN(this->get_logger(), "Waiting for IMU data...");
    return;
  }

  core_msgs::msg::CANArray can_msg;
  can_msg.array.resize(1);
  can_msg.array[0].id = 4;
  can_msg.array[0].data.push_back(3);

  if (emergency_stop_flag_) {
    RCLCPP_ERROR(this->get_logger(), "Emergency stop flag is set");
    can_msg.array[0].data.push_back(0);
  } else {
    auto omega = gimbalControl();
    if (std::abs(omega) < 0.1) {
      omega = 0;
    }
    can_msg.array[0].data.push_back(omega);

    auto omega_msg = std_msgs::msg::Float64();
    omega_msg.data = omega;
    target_omega_pub_->publish(omega_msg);
  }
  can_pub_->publish(can_msg);
  RCLCPP_INFO(this->get_logger(), "omega: %f", can_msg.array[0].data[1]);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetAngleNode>());
  rclcpp::shutdown();
  return 0;
}
