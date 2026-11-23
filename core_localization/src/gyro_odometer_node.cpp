// 車輪の速度（/vehicle/twist）と IMU の角速度を合わせて、機体の速度（/localization/twist_with_covariance）を
// 出すノード（Autoware の gyro_odometer と同じ役割）。並進は車輪から、回転はジャイロから取る。
// IMU の角速度は TF で base_link の座標系に直す（逆さに付けた IMU もそのまま扱える）。
// 車輪の速度を受けるたびに、前回からためたジャイロの平均と合わせて出す（gyro_odometer.hpp）。

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "core_nav_common/parameters.hpp"
#include "core_localization/gyro_odometer.hpp"

namespace core_localization
{
namespace
{
using geometry_msgs::msg::TwistWithCovarianceStamped;
using sensor_msgs::msg::Imu;

// 共分散の行列（6×6、並びは x, y, z, roll, pitch, yaw）のうち、使わない成分に入れる大きな分散
constexpr double kUnusedVariance = 1.0e4;

double toSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

GyroOdometerParams loadParams(rclcpp::Node & node)
{
  using core_nav_common::declareDouble;
  GyroOdometerParams params;
  params.gyro_variance = declareDouble(
    node, "gyro_variance", params.gyro_variance, 1e-12, 1.0,
    "IMU に角速度の分散がないときに使う分散 [(rad/s)^2]");
  params.wheel_linear_variance = declareDouble(
    node, "wheel_linear_variance", params.wheel_linear_variance, 1e-12, 100.0,
    "車輪の速度に分散がないときに使う並進の分散 [(m/s)^2]");
  params.wheel_angular_variance = declareDouble(
    node, "wheel_angular_variance", params.wheel_angular_variance, 1e-12, 100.0,
    "ジャイロが途絶えて車輪の角速度を使うときの分散 [(rad/s)^2]");
  params.gyro_timeout_s = declareDouble(
    node, "gyro_timeout_s", params.gyro_timeout_s, 0.001, 10.0,
    "最後のジャイロからこの時間を超えたら途絶えたとみなす [s]");
  return params;
}
}  // namespace

class GyroOdometerNode final : public rclcpp::Node
{
public:
  explicit GyroOdometerNode(const rclcpp::NodeOptions & options)
  : Node("gyro_odometer", options),
    odometer_(loadParams(*this)),
    base_frame_(core_nav_common::declareString(*this, "base_frame", "base_link", "機体の座標系", true)),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    using core_nav_common::declareString;
    const std::string wheel_topic =
      declareString(*this, "wheel_topic", "/sensing/vehicle/twist", "車輪の速度（base_link）", true);
    const std::string imu_topic =
      declareString(*this, "imu_topic", "/sensing/imu/fused", "角速度を使う IMU", true);
    const std::string output_topic = declareString(
      *this, "output_topic", "/localization/twist_with_covariance", "合わせた速度", true);
    publisher_ = create_publisher<TwistWithCovarianceStamped>(output_topic, rclcpp::QoS(50));
    imu_subscription_ = create_subscription<Imu>(
      imu_topic, rclcpp::SensorDataQoS(), [this](Imu::ConstSharedPtr msg) {onImu(*msg);});
    wheel_subscription_ = create_subscription<TwistWithCovarianceStamped>(
      wheel_topic, rclcpp::QoS(50),
      [this](TwistWithCovarianceStamped::ConstSharedPtr msg) {onWheel(*msg);});
  }

private:
  // IMU の座標系から base_link への回転。取付けは固定なので、座標系ごとに一度だけ調べる
  std::optional<Eigen::Matrix3d> rotationToBase(const std::string & imu_frame)
  {
    const auto cached = rotations_.find(imu_frame);
    if (cached != rotations_.end()) {
      return cached->second;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(base_frame_, imu_frame, tf2::TimePointZero);
      const Eigen::Matrix3d rotation = tf2::transformToEigen(transform).rotation();
      rotations_.emplace(imu_frame, rotation);
      return rotation;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "IMU の座標系 %s から %s への TF がありません: %s",
        imu_frame.c_str(), base_frame_.c_str(), error.what());
      return std::nullopt;
    }
  }

  void onImu(const Imu & msg)
  {
    // 共分散の先頭が -1 なら、角速度そのものがない（sensor_msgs/Imu の決まり）
    if (msg.angular_velocity_covariance[0] < 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "IMU（%s）に角速度がありません", msg.header.frame_id.c_str());
      return;
    }
    const auto rotation = rotationToBase(msg.header.frame_id);
    if (!rotation.has_value()) {
      return;
    }
    const Eigen::Vector3d angular_base = *rotation * Eigen::Vector3d(
      msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    // 共分散がすべて 0 なら分からないので、0 のまま渡して gyro_variance を使わせる
    const double variance = (*rotation * Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(
        msg.angular_velocity_covariance.data()) * rotation->transpose())(2, 2);
    odometer_.addGyro(GyroRate{toSeconds(msg.header.stamp), angular_base.z(), variance});
  }

  void onWheel(const TwistWithCovarianceStamped & msg)
  {
    if (!msg.header.frame_id.empty() && msg.header.frame_id != base_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "車輪の速度の座標系 %s が %s と違います（そのまま使います）",
        msg.header.frame_id.c_str(), base_frame_.c_str());
    }
    const auto & covariance = msg.twist.covariance;
    const TwistEstimate estimate = odometer_.combine(WheelTwist{
        toSeconds(msg.header.stamp), msg.twist.twist.linear.x, msg.twist.twist.linear.y,
        msg.twist.twist.angular.z, covariance[0], covariance[7], covariance[35]});
    if (!estimate.used_gyro) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "ジャイロが途絶えているので、車輪の角速度を使います");
    }
    TwistWithCovarianceStamped output;
    output.header.stamp = msg.header.stamp;
    output.header.frame_id = base_frame_;
    output.twist.twist.linear.x = estimate.vx;
    output.twist.twist.linear.y = estimate.vy;
    output.twist.twist.angular.z = estimate.wz;
    output.twist.covariance.fill(0.0);
    output.twist.covariance[0] = estimate.var_vx;
    output.twist.covariance[7] = estimate.var_vy;
    output.twist.covariance[14] = kUnusedVariance;
    output.twist.covariance[21] = kUnusedVariance;
    output.twist.covariance[28] = kUnusedVariance;
    output.twist.covariance[35] = estimate.var_wz;
    publisher_->publish(output);
  }

  GyroOdometer odometer_;
  std::string base_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unordered_map<std::string, Eigen::Matrix3d> rotations_;
  rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr wheel_subscription_;
};
}  // namespace core_localization

RCLCPP_COMPONENTS_REGISTER_NODE(core_localization::GyroOdometerNode)
