// 接触を検知するノード。
//
// 次のどちらかで接触とし、hold_s の間 in_contact を保つ。
// - Damiao IMU の水平加速度が閾値を超える（衝突の衝撃）
// - 速度指令と実際の速度の差が閾値を超え、差が縮まらないまま続く（押し付けている状態）
// 加速中も一時的に差が開くので、差が縮まっている間は接触に数えない。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <core_msgs/msg/contact_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "core_nav_common/parameters.hpp"

namespace core_planning
{
namespace
{
using geometry_msgs::msg::Twist;
using core_msgs::msg::ContactStatus;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::Imu;

constexpr auto kPublishPeriod = std::chrono::milliseconds(50);
// 速度の差がこれ以上縮まっていれば、加速中とみなす [m/s]
constexpr double kProgressMps = 0.1;

struct ContactParams
{
  bool enabled{true};
  double accel_threshold_mps2{20.0};
  double velocity_error_mps{1.0};
  double velocity_error_hold_s{0.3};
  double hold_s{1.0};
};
}  // namespace

class ContactMonitorNode final : public rclcpp::Node
{
public:
  explicit ContactMonitorNode(const rclcpp::NodeOptions & options)
  : Node("contact_monitor", options)
  {
    using core_nav_common::declareBool;
    using core_nav_common::declareDouble;
    declareBool(*this, "contact_detection.enabled", true, "接触を検知する");
    declareDouble(
      *this, "contact_detection.accel_threshold_mps2", 20.0, 0.1, 1000.0, "接触とする水平加速度");
    declareDouble(
      *this, "contact_detection.velocity_error_mps", 1.0, 0.01, 20.0, "接触とする速度の差");
    declareDouble(
      *this, "contact_detection.velocity_error_hold_s", 0.3, 0.0, 10.0, "速度の差が続く時間");
    declareDouble(*this, "contact_detection.hold_s", 1.0, 0.0, 60.0, "接触の状態を保つ時間");
    loadParams();
    parameter_callback_ = add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &) {loadParams();});

    publisher_ = create_publisher<ContactStatus>("/planning/contact/status", rclcpp::QoS(10));
    imu_subscription_ = create_subscription<Imu>(
      "/sensing/imu", rclcpp::SensorDataQoS(),
      std::bind(&ContactMonitorNode::onImu, this, std::placeholders::_1));
    cmd_subscription_ = create_subscription<Twist>(
      "/control/cmd_vel", rclcpp::QoS(10),
      [this](const Twist::ConstSharedPtr & cmd) {latest_cmd_ = cmd;});
    odometry_subscription_ = create_subscription<Odometry>(
      "/localization/lio/odom", rclcpp::QoS(10),
      std::bind(&ContactMonitorNode::onOdometry, this, std::placeholders::_1));
    timer_ = rclcpp::create_timer(
      this, get_clock(), kPublishPeriod, std::bind(&ContactMonitorNode::onTimer, this));
  }

private:
  void loadParams()
  {
    ContactParams params;
    params.enabled = get_parameter("contact_detection.enabled").as_bool();
    params.accel_threshold_mps2 =
      get_parameter("contact_detection.accel_threshold_mps2").as_double();
    params.velocity_error_mps = get_parameter("contact_detection.velocity_error_mps").as_double();
    params.velocity_error_hold_s =
      get_parameter("contact_detection.velocity_error_hold_s").as_double();
    params.hold_s = get_parameter("contact_detection.hold_s").as_double();
    params_ = params;
  }

  void trigger(const std::string & reason)
  {
    contact_until_ = now() + rclcpp::Duration::from_seconds(params_.hold_s);
    reason_ = reason;
  }

  void onImu(const Imu::ConstSharedPtr & imu)
  {
    const double accel = std::hypot(imu->linear_acceleration.x, imu->linear_acceleration.y);
    peak_accel_ = std::max(peak_accel_, accel);
    if (params_.enabled && accel > params_.accel_threshold_mps2) {
      trigger("水平加速度が閾値を超えた");
    }
  }

  void onOdometry(const Odometry::ConstSharedPtr & odometry)
  {
    if (!latest_cmd_) {
      return;
    }
    const double error = std::hypot(
      latest_cmd_->linear.x - odometry->twist.twist.linear.x,
      latest_cmd_->linear.y - odometry->twist.twist.linear.y);
    velocity_error_ = error;
    if (error <= params_.velocity_error_mps) {
      error_window_start_.reset();
      return;
    }
    const rclcpp::Time time = now();
    if (!error_window_start_.has_value() || error < error_at_window_start_ - kProgressMps) {
      error_window_start_ = time;
      error_at_window_start_ = error;
      return;
    }
    if (params_.enabled && (time - *error_window_start_).seconds() >= params_.velocity_error_hold_s) {
      trigger("速度指令と実際の速度の差が続いた");
    }
  }

  void onTimer()
  {
    ContactStatus status;
    const rclcpp::Time time = now();
    status.header.stamp = time;
    status.in_contact = params_.enabled && contact_until_.has_value() && time < *contact_until_;
    status.reason = status.in_contact ? reason_ : "";
    status.accel_mps2 = static_cast<float>(peak_accel_);
    status.velocity_error_mps = static_cast<float>(velocity_error_);
    publisher_->publish(status);
    peak_accel_ = 0.0;
  }

  ContactParams params_;
  Twist::ConstSharedPtr latest_cmd_;
  std::optional<rclcpp::Time> contact_until_;
  std::optional<rclcpp::Time> error_window_start_;
  double error_at_window_start_{0.0};
  double peak_accel_{0.0};
  double velocity_error_{0.0};
  std::string reason_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::Publisher<ContactStatus>::SharedPtr publisher_;
  rclcpp::Subscription<Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<Twist>::SharedPtr cmd_subscription_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace core_planning

RCLCPP_COMPONENTS_REGISTER_NODE(core_planning::ContactMonitorNode)
