#include <array>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

#include "core_body_controller/dead_reckoning.hpp"
#include "core_body_controller/omni_kinematics.hpp"
#include "core_body_controller/target_angle_controller.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

// 車輪 (joint 0〜3) の実測角速度と腰の角度 (joint 4) から base_link の速度を求め、
// IMU の yaw で向きを決めて積分する推測航法。
// FAST-LIO の odom とは別物なので frame_id は dead_reckoning とし、TF は出さない。
class DeadReckoningNode : public rclcpp::Node
{
public:
  DeadReckoningNode()
  : Node("dead_reckoning_node")
  {
    // 既定値は body_control_node の WHEEL_RADIUS, BODY_WIDTH, INITIAL_ANGLE と同じ
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.13 / 2);
    body_width_ = declare_parameter<double>("body_width", 0.5304);
    body_angle_offset_ = declare_parameter<double>("body_angle_offset", 3.7822790145874023);
    imu_timeout_sec_ = declare_parameter<double>("imu_timeout_sec", 0.2);
    max_integration_dt_sec_ = declare_parameter<double>("max_integration_dt_sec", 0.2);
    frame_id_ = declare_parameter<std::string>("frame_id", "dead_reckoning");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const double init_x = declare_parameter<double>("init_x", 0.0);
    const double init_y = declare_parameter<double>("init_y", 0.0);
    init_yaw_ = declare_parameter<double>("init_yaw", 0.0);
    if (!std::isfinite(wheel_radius_) || !std::isfinite(body_width_) ||
      !std::isfinite(body_angle_offset_) || !std::isfinite(imu_timeout_sec_) ||
      !std::isfinite(max_integration_dt_sec_) || !std::isfinite(init_x) ||
      !std::isfinite(init_y) || !std::isfinite(init_yaw_) || wheel_radius_ <= 0.0 ||
      body_width_ <= 0.0 || imu_timeout_sec_ <= 0.0 || max_integration_dt_sec_ <= 0.0)
    {
      throw std::invalid_argument(
              "dead_reckoning parameters must be finite; wheel_radius, body_width, "
              "imu_timeout_sec and max_integration_dt_sec must also be positive");
    }
    pose_ = core_body_controller::Pose2D{init_x, init_y, init_yaw_};

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("dead_reckoning", 10);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {on_imu(*msg);});
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {on_joint_states(*msg);});
  }

private:
  using SteadyTime = std::chrono::steady_clock::time_point;
  static constexpr std::size_t BODY_JOINT_INDEX = 4;

  void on_imu(const sensor_msgs::msg::Imu & msg)
  {
    double measured_yaw = 0.0;
    if (!core_body_controller::quaternion_to_yaw(
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
        measured_yaw))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Ignoring IMU message containing an invalid orientation quaternion");
      return;
    }

    const auto arrival_time = std::chrono::steady_clock::now();
    const bool continuous = has_imu_ && seconds_since(last_imu_time_, arrival_time) <=
      imu_timeout_sec_;
    // 最初のサンプルを init_yaw とし、途絶後の再開では yaw を飛ばさない
    yaw_tracker_.update(measured_yaw, continuous);
    has_imu_ = true;
    last_imu_time_ = arrival_time;
  }

  void on_joint_states(const sensor_msgs::msg::JointState & msg)
  {
    if (msg.velocity.size() < 4 || msg.position.size() <= BODY_JOINT_INDEX) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Ignoring joint_states: velocity size=%zu (need 4), position size=%zu (need %zu)",
        msg.velocity.size(), msg.position.size(), BODY_JOINT_INDEX + 1);
      return;
    }

    const std::array<double, 4> wheel_velocities{
      msg.velocity[0], msg.velocity[1], msg.velocity[2], msg.velocity[3]};
    const double body_angle = msg.position[BODY_JOINT_INDEX] - body_angle_offset_;
    const auto velocity = core_body_controller::forward_kinematics(
      wheel_velocities, body_angle, wheel_radius_, body_width_);
    if (!velocity) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Ignoring joint_states containing non-finite wheel velocity or body angle");
      return;
    }

    const rclcpp::Time stamp =
      rclcpp::Time(msg.header.stamp).nanoseconds() == 0 ? now() : rclcpp::Time(msg.header.stamp);
    const double angular_z = integrate(stamp, velocity->linear_x, velocity->linear_y);
    previous_stamp_ = stamp;
    publish(stamp, velocity->linear_x, velocity->linear_y, angular_z);
  }

  // 位置を更新し、yaw の角速度を返す。積分できないときは位置を据え置いて 0 を返す
  double integrate(const rclcpp::Time & stamp, double vx, double vy)
  {
    if (!previous_stamp_) {
      return 0.0;
    }
    if (!has_imu_ ||
      seconds_since(last_imu_time_, std::chrono::steady_clock::now()) > imu_timeout_sec_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Waiting for fresh IMU data; holding pose");
      return 0.0;
    }

    const double dt = (stamp - *previous_stamp_).seconds();
    if (dt <= 0.0 || dt > max_integration_dt_sec_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Skipping integration: joint_states interval %.3f s is outside (0, %.3f]", dt,
        max_integration_dt_sec_);
      return 0.0;
    }

    const double yaw = init_yaw_ + yaw_tracker_.yaw();
    const auto next = core_body_controller::integrate_pose(pose_, vx, vy, yaw, dt);
    if (!next) {
      return 0.0;
    }
    const double angular_z = core_body_controller::normalize_angle(yaw - pose_.yaw) / dt;
    pose_ = *next;
    return angular_z;
  }

  void publish(const rclcpp::Time & stamp, double vx, double vy, double angular_z)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose_.x;
    odom.pose.pose.position.y = pose_.y;
    odom.pose.pose.orientation.z = std::sin(pose_.yaw / 2);
    odom.pose.pose.orientation.w = std::cos(pose_.yaw / 2);
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = angular_z;
    odom_pub_->publish(odom);
  }

  static double seconds_since(const SteadyTime & from, const SteadyTime & to)
  {
    return std::chrono::duration<double>(to - from).count();
  }

  double wheel_radius_;
  double body_width_;
  double body_angle_offset_;
  double imu_timeout_sec_;
  double max_integration_dt_sec_;
  double init_yaw_;
  std::string frame_id_;
  std::string base_frame_;

  core_body_controller::Pose2D pose_{};
  core_body_controller::YawTracker yaw_tracker_;
  bool has_imu_{false};
  SteadyTime last_imu_time_{};
  std::optional<rclcpp::Time> previous_stamp_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeadReckoningNode>());
  rclcpp::shutdown();
  return 0;
}
