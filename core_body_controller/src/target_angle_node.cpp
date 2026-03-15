#include <cmath>
#include <core_msgs/msg/can_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

class PID {
  float kp_, ki_;
  float integral_;
  float dt_;
  float error_prev_;

public:
  PID(float kp, float ki, float dt) : dt_(dt), error_prev_(0.0) {
    reset();
    setGain(kp, ki);
  }

  float update(float error, float limit) {
    float output, potential, integral_diff;
    // P制御
    potential = kp_ * error;
    // 台形積分
    integral_diff = ki_ * 0.5f * (error + error_prev_) * dt_;
    error_prev_ = error;

    // P制御 + I制御
    output = potential + integral_ + integral_diff;

    // antiwindup - limit the output
    if (output > limit) {
      if (integral_diff < 0) {
        integral_ += integral_diff;
      }
      return limit;
    } else if (output < -limit) {
      if (integral_diff > 0) {
        integral_ += integral_diff;
      }
      return -limit;
    }
    integral_ += integral_diff;
    return output;
  }

  void reset() {
    integral_ = 0;
    error_prev_ = 0;
  }
  void setGain(float kp, float ki) {
    kp_ = kp;
    ki_ = ki;
  }
};

class TargetAngleNode : public rclcpp::Node {
public:
  TargetAngleNode();

private:
  // rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rotation_pub_;
  rclcpp::Publisher<core_msgs::msg::CANArray>::SharedPtr can_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pad_ps_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr rotation_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr body_target_angle_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr body_omega_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr rotation_flag_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_omega_pub_;

  constexpr static double MAX_ROTATION = M_PI * 10;
  constexpr static std::chrono::milliseconds TIMER_PERIOD = std::chrono::milliseconds(100);
  constexpr static double TIMER_DT = std::chrono::duration<double>(TIMER_PERIOD).count();

  PID pid_ = PID(2.595048087059986, 0.0, TIMER_DT);
  double gimbalControl();

  constexpr static double INITIAL_TARGET_ANGLE = 0.0;

  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback();
  double world_target_angle_ = 0;
  double body_target_angle_ = INITIAL_TARGET_ANGLE;
  bool rotation_flag_ = false;
  double latest_imu_omega_ = 0;
  double latest_imu_yaw_estimate_ = 0;
  double latest_body_omega_ = 0;
  double latest_body_angle_ = 0;
  bool emergency_stop_flag_ = true;
  geometry_msgs::msg::Twist latest_twist_;

  static double normalizeAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  double calc_nearlest_target_angle(double current_angle) {
    double remainder = fmod(current_angle - INITIAL_TARGET_ANGLE, 2 * M_PI);
    if (remainder <= M_PI / 2) {
      return current_angle - remainder;
    } else if (remainder <= 3 * M_PI / 2) {
      return current_angle - remainder + M_PI;
    } else {
      return current_angle - remainder + 2 * M_PI;
    }
  }
};

TargetAngleNode::TargetAngleNode() : Node("target_angle_node") {
  // target_angle_pub_ =
  //     this->create_publisher<std_msgs::msg::Float64>("target_angle", 10);
  rotation_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/rotation", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          rotation_flag_ = true;
          world_target_angle_ = latest_imu_yaw_estimate_;
          pid_.reset();
        } else {
          rotation_flag_ = false;
          body_target_angle_ = calc_nearlest_target_angle(latest_body_angle_);
          pid_.reset();
        }
      });
  twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        latest_twist_ = *msg;
      });
  timer_ = this->create_wall_timer(TIMER_PERIOD, std::bind(&TargetAngleNode::timer_callback, this));
  can_pub_ = this->create_publisher<core_msgs::msg::CANArray>("can/tx", 10);
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        // New IMU publishes angular velocity, so estimate yaw by integration in timer callback.
        latest_imu_omega_ = msg->angular_velocity.z;
      });
  body_target_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "yaw_target_angle", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
        body_target_angle_ = msg->data + INITIAL_TARGET_ANGLE;
      });
  body_omega_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "body_omega", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
        latest_body_omega_ = msg->data;
      });
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        latest_body_angle_ = msg->position[4];
      });
  rotation_flag_pub_ = this->create_publisher<std_msgs::msg::Bool>("rotation_flag", 10);
  emergency_stop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/system/emergency/hazard_status", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
        emergency_stop_flag_ = msg->data;
      });
  target_omega_pub_ = this->create_publisher<std_msgs::msg::Float64>("target_omega", 10);
}

double TargetAngleNode::gimbalControl() {
  if (rotation_flag_) {
    world_target_angle_ += latest_twist_.angular.z * TIMER_DT;
    world_target_angle_ = normalizeAngle(world_target_angle_);
    RCLCPP_INFO(this->get_logger(), "world_target_angle_: %f", world_target_angle_);
    RCLCPP_INFO(this->get_logger(), "latest_imu_yaw_estimate_: %f", latest_imu_yaw_estimate_);
    return pid_.update(world_target_angle_ - latest_imu_yaw_estimate_, MAX_ROTATION) -
           latest_body_omega_;
  } else {
    RCLCPP_INFO(this->get_logger(), "body_target_angle_: %f", body_target_angle_);
    return pid_.update(body_target_angle_ - latest_body_angle_, MAX_ROTATION);
  }
}

void TargetAngleNode::timer_callback() {
  latest_imu_yaw_estimate_ =
      normalizeAngle(latest_imu_yaw_estimate_ + latest_imu_omega_ * TIMER_DT);

  core_msgs::msg::CANArray can_msg;
  can_msg.array.resize(1);
  can_msg.array[0].id = 4;
  can_msg.array[0].data.push_back(3);

  if (emergency_stop_flag_) {
    RCLCPP_ERROR(this->get_logger(), "Emergency stop flag is set");
    can_msg.array[0].data.push_back(0);
  } else {
    auto omega = gimbalControl();
    if (std::abs(omega) < 0.2) {
      omega = 0;
    }
    can_msg.array[0].data.push_back(omega);

    auto omega_msg = std_msgs::msg::Float64();
    omega_msg.data = omega;
    target_omega_pub_->publish(omega_msg);

    std_msgs::msg::Bool msg;
    msg.data = rotation_flag_;
    rotation_flag_pub_->publish(msg);
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
