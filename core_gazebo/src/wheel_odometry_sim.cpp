// シミュレーションの車輪の速度を出すノード。実機の足回りが返す速度（/vehicle/twist、base_link の座標系）の代わり。
// 真値（/ground_truth/odom、200Hz）の姿勢の window_s の差分で機体の速度を求め、実機の車輪で起きる誤差を足して
// rate_hz で出す（誤差は core_gazebo/wheel_odometry_model.hpp）。時刻は差分を取った間の真ん中にする。
// 共分散には、足回りが分かっているはずの誤差（倍率、偏り、雑音）だけを入れる。

#include <cmath>
#include <cstdint>
#include <deque>
#include <optional>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "core_gazebo/wheel_odometry_model.hpp"

namespace core_gazebo
{
namespace
{
using geometry_msgs::msg::TwistWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using wheel_odometry::BodyTwist;
using wheel_odometry::ErrorParams;
using wheel_odometry::PlanarPose;

// 使わない成分（z、roll、pitch）の分散
constexpr double kUnusedVariance = 1.0e4;
// 真値を残す時間 [s]
constexpr double kTruthHistoryS = 0.5;

double toSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time toStamp(const double seconds)
{
  return rclcpp::Time(static_cast<std::int64_t>(std::llround(seconds * 1e9)));
}

ErrorParams declareErrorParams(rclcpp::Node & node)
{
  ErrorParams params;
  const auto declare = [&node](const std::string & name, const double value) {
      return node.declare_parameter<double>("error." + name, value);
    };
  params.scale_std_x = declare("scale_std_x", params.scale_std_x);
  params.scale_std_y = declare("scale_std_y", params.scale_std_y);
  params.scale_std_yaw = declare("scale_std_yaw", params.scale_std_yaw);
  params.drift_std_linear_mps = declare("drift_std_linear_mps", params.drift_std_linear_mps);
  params.drift_std_angular_radps = declare("drift_std_angular_radps", params.drift_std_angular_radps);
  params.drift_time_constant_s = declare("drift_time_constant_s", params.drift_time_constant_s);
  params.accel_slip_s = declare("accel_slip_s", params.accel_slip_s);
  params.burst_rate_hz = declare("burst_rate_hz", params.burst_rate_hz);
  params.burst_speed_min_mps = declare("burst_speed_min_mps", params.burst_speed_min_mps);
  params.burst_speed_max_mps = declare("burst_speed_max_mps", params.burst_speed_max_mps);
  params.burst_duration_min_s = declare("burst_duration_min_s", params.burst_duration_min_s);
  params.burst_duration_max_s = declare("burst_duration_max_s", params.burst_duration_max_s);
  params.noise_std_linear_mps = declare("noise_std_linear_mps", params.noise_std_linear_mps);
  params.noise_std_angular_radps = declare("noise_std_angular_radps", params.noise_std_angular_radps);
  // 範囲の誤りは、意味のあるメッセージで止める
  wheel_odometry::validate(params);
  return params;
}

double positiveParameter(rclcpp::Node & node, const std::string & name, const double value)
{
  const double declared = node.declare_parameter<double>(name, value);
  if (!(declared > 0.0)) {
    throw std::invalid_argument(name + " は正にしてください: " + std::to_string(declared));
  }
  return declared;
}
}  // namespace

class WheelOdometrySim final : public rclcpp::Node
{
public:
  explicit WheelOdometrySim(const rclcpp::NodeOptions & options)
  : Node("wheel_odometry_sim", options),
    model_(declareErrorParams(*this), static_cast<std::uint64_t>(declare_parameter<std::int64_t>("seed", 1))),
    period_s_(1.0 / positiveParameter(*this, "rate_hz", 50.0)),
    window_s_(positiveParameter(*this, "window_s", 0.02)),
    frame_id_(declare_parameter<std::string>("frame_id", "base_link"))
  {
    const auto truth_topic = declare_parameter<std::string>("truth_topic", "/ground_truth/odom");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/sensing/vehicle/twist");
    publisher_ = create_publisher<TwistWithCovarianceStamped>(output_topic, rclcpp::QoS(50));
    subscription_ = create_subscription<Odometry>(
      truth_topic, rclcpp::QoS(50), [this](Odometry::ConstSharedPtr msg) {onTruth(*msg);});
    const BodyTwist & scale = model_.scale();
    RCLCPP_INFO(
      get_logger(), "車輪の速度の倍率: vx %.4f, vy %.4f, wz %.4f", scale.vx, scale.vy, scale.wz);
  }

private:
  void onTruth(const Odometry & msg)
  {
    const auto & q = msg.pose.pose.orientation;
    const Eigen::Vector3d heading =
      Eigen::Quaterniond(q.w, q.x, q.y, q.z) * Eigen::Vector3d::UnitX();
    const PlanarPose pose{
      toSeconds(msg.header.stamp), msg.pose.pose.position.x, msg.pose.pose.position.y,
      std::atan2(heading.y(), heading.x())};
    if (!truth_.empty() && pose.stamp_s <= truth_.back().stamp_s) {
      if (pose.stamp_s + 1.0 < truth_.back().stamp_s) {
        // 時計が戻った（シミュレーションのやり直し）
        truth_.clear();
        last_ = std::nullopt;
      } else {
        return;
      }
    }
    truth_.push_back(pose);
    while (truth_.front().stamp_s < pose.stamp_s - kTruthHistoryS) {
      truth_.pop_front();
    }
    // 真値の周期の半分の揺らぎは許す
    if (last_.has_value() && pose.stamp_s - last_->end_stamp_s < period_s_ - 0.5 * truthPeriod()) {
      return;
    }
    publishAt(pose);
  }

  double truthPeriod() const
  {
    if (truth_.size() < 2) {
      return 0.0;
    }
    return (truth_.back().stamp_s - truth_.front().stamp_s) / static_cast<double>(truth_.size() - 1);
  }

  void publishAt(const PlanarPose & latest)
  {
    // window_s より前の最後の真値と差分を取る
    std::optional<PlanarPose> from;
    for (auto it = truth_.rbegin(); it != truth_.rend(); ++it) {
      if (it->stamp_s <= latest.stamp_s - window_s_ + 1e-6) {
        from = *it;
        break;
      }
    }
    if (!from.has_value()) {
      return;
    }
    const BodyTwist truth = wheel_odometry::bodyTwistBetween(*from, latest);
    const double stamp_s = 0.5 * (from->stamp_s + latest.stamp_s);
    BodyTwist accel;
    double dt = period_s_;
    if (last_.has_value() && stamp_s > last_->stamp_s) {
      dt = stamp_s - last_->stamp_s;
      accel = BodyTwist{
        (truth.vx - last_->truth.vx) / dt, (truth.vy - last_->truth.vy) / dt,
        (truth.wz - last_->truth.wz) / dt};
    }
    const BodyTwist measured = model_.apply(truth, accel, dt);
    const BodyTwist variance = model_.variance(truth);
    last_ = Published{stamp_s, latest.stamp_s, truth};

    TwistWithCovarianceStamped output;
    output.header.stamp = toStamp(stamp_s);
    output.header.frame_id = frame_id_;
    output.twist.twist.linear.x = measured.vx;
    output.twist.twist.linear.y = measured.vy;
    output.twist.twist.angular.z = measured.wz;
    output.twist.covariance.fill(0.0);
    output.twist.covariance[0] = variance.vx;
    output.twist.covariance[7] = variance.vy;
    output.twist.covariance[14] = kUnusedVariance;
    output.twist.covariance[21] = kUnusedVariance;
    output.twist.covariance[28] = kUnusedVariance;
    output.twist.covariance[35] = variance.wz;
    publisher_->publish(output);
  }

  struct Published
  {
    double stamp_s{0.0};
    double end_stamp_s{0.0};
    BodyTwist truth;
  };

  wheel_odometry::WheelErrorModel model_;
  double period_s_;
  double window_s_;
  std::string frame_id_;
  std::deque<PlanarPose> truth_;
  std::optional<Published> last_;
  rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<Odometry>::SharedPtr subscription_;
};
}  // namespace core_gazebo

RCLCPP_COMPONENTS_REGISTER_NODE(core_gazebo::WheelOdometrySim)
