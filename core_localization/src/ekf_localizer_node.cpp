// gyro_odometer の速度（/localization/twist_with_covariance）と、NDT（lidar_localization_ros2）の姿勢
// （/pcl_pose）を EKF で合わせ、地図の座標の姿勢と機体の速度を /localization/kinematic_state
// （nav_msgs/Odometry）に一定の周期で出すノード（Autoware の ekf_localizer と同じ役割）。
// TF は出さない（制御につなぐのは次の段階）。計算は ekf_localizer.hpp。
// /pcl_pose には、照合を捨てた回に LIO で進めた姿勢（bridge）が広い共分散（標準偏差 5m）で入るので、
// 共分散が max_message_pose_std_xy_m を超える姿勢は使わない。

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "core_nav_common/parameters.hpp"
#include "core_localization/ekf_localizer.hpp"

namespace core_localization
{
namespace
{
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::TwistWithCovarianceStamped;
using nav_msgs::msg::Odometry;

// 6×6 の共分散（並びは x, y, z, roll, pitch, yaw）から、平面の 3 成分 [x, y, yaw] を取り出す添字
constexpr int kPlanarIndex[3] = {0, 1, 5};
// 時計がこれより戻ったら（シミュレーションのやり直しなど）、EKF を初めからにする [s]
constexpr double kClockJumpBackS = 1.0;

double toSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time toStamp(const double seconds)
{
  return rclcpp::Time(static_cast<std::int64_t>(std::llround(seconds * 1e9)));
}

Eigen::Matrix3d planarCovariance(const std::array<double, 36> & covariance)
{
  Eigen::Matrix3d planar;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      planar(row, col) = covariance[kPlanarIndex[row] * 6 + kPlanarIndex[col]];
    }
  }
  return planar;
}

void setPlanarCovariance(std::array<double, 36> & covariance, const Eigen::Matrix3d & planar)
{
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      covariance[kPlanarIndex[row] * 6 + kPlanarIndex[col]] = planar(row, col);
    }
  }
}

double yawOf(const geometry_msgs::msg::Quaternion & orientation)
{
  const Eigen::Quaterniond q(orientation.w, orientation.x, orientation.y, orientation.z);
  const Eigen::Vector3d heading = q * Eigen::Vector3d::UnitX();
  return std::atan2(heading.y(), heading.x());
}

const char * describe(const UpdateResult result)
{
  switch (result) {
    case UpdateResult::kAccepted:
      return "accepted";
    case UpdateResult::kRejectedByGate:
      return "rejected_by_gate";
    case UpdateResult::kReset:
      return "reset";
    case UpdateResult::kTooOld:
      return "too_old";
    case UpdateResult::kNotInitialized:
      return "not_initialized";
  }
  return "unknown";
}

EkfParams loadEkfParams(rclcpp::Node & node)
{
  using core_nav_common::declareDouble;
  using core_nav_common::declareInt;
  EkfParams params;
  params.accel_std_mps2 = declareDouble(
    node, "accel_std_mps2", params.accel_std_mps2, 0.01, 100.0, "速度の予測の雑音 [m/s^2]", true);
  params.angular_accel_std_radps2 = declareDouble(
    node, "angular_accel_std_radps2", params.angular_accel_std_radps2, 0.01, 100.0,
    "角速度の予測の雑音 [rad/s^2]", true);
  params.position_noise_std = declareDouble(
    node, "position_noise_std", params.position_noise_std, 0.0, 10.0,
    "位置そのものの予測の雑音 [m/√s]", true);
  params.yaw_noise_std = declareDouble(
    node, "yaw_noise_std", params.yaw_noise_std, 0.0, 10.0, "向きそのものの予測の雑音 [rad/√s]", true);
  params.pose_gate_chi2 = declareDouble(
    node, "pose_gate_chi2", params.pose_gate_chi2, 0.1, 1e6,
    "姿勢を捨てるマハラノビス距離の 2 乗", true);
  params.twist_gate_chi2 = declareDouble(
    node, "twist_gate_chi2", params.twist_gate_chi2, 0.1, 1e6,
    "速度を捨てるマハラノビス距離の 2 乗", true);
  params.min_pose_std_xy_m = declareDouble(
    node, "min_pose_std_xy_m", params.min_pose_std_xy_m, 1e-4, 10.0, "姿勢の位置の標準偏差の下限 [m]", true);
  params.min_pose_std_yaw_rad = declareDouble(
    node, "min_pose_std_yaw_rad", params.min_pose_std_yaw_rad, 1e-5, 1.0,
    "姿勢の向きの標準偏差の下限 [rad]", true);
  params.initial_velocity_std = declareDouble(
    node, "initial_velocity_std", params.initial_velocity_std, 0.01, 100.0,
    "初めの速度の標準偏差 [m/s]、[rad/s]", true);
  params.history_s = declareDouble(
    node, "history_s", params.history_s, 0.05, 10.0, "遅れて届く測定のために残す時間 [s]", true);
  params.pose_rejections_before_reset = static_cast<int>(declareInt(
    node, "pose_rejections_before_reset", params.pose_rejections_before_reset, 0, 1000,
    "姿勢をこの回数続けて捨てたら、測定の姿勢に置き換える（0 で置き換えない）", true));
  return params;
}
}  // namespace

class EkfLocalizerNode final : public rclcpp::Node
{
public:
  explicit EkfLocalizerNode(const rclcpp::NodeOptions & options)
  : Node("ekf_localizer", options),
    ekf_params_(loadEkfParams(*this)),
    ekf_(ekf_params_)
  {
    using core_nav_common::declareDouble;
    using core_nav_common::declareString;
    map_frame_ = declareString(*this, "map_frame", "map", "出力の姿勢の座標系", true);
    base_frame_ = declareString(*this, "base_frame", "base_link", "機体の座標系", true);
    max_extrapolation_s_ = declareDouble(
      *this, "max_extrapolation_s", 0.5, 0.01, 10.0,
      "最後の測定からこの時間を超えたら、先へ進めた姿勢を出さない [s]", true);
    const std::string covariance_source = declareString(
      *this, "pose_covariance_source", "fixed",
      "姿勢の測定の共分散。fixed は下の標準偏差、message は /pcl_pose の共分散", true);
    if (covariance_source != "fixed" && covariance_source != "message") {
      throw std::invalid_argument(
              "pose_covariance_source は fixed か message です（" + covariance_source + "）");
    }
    use_message_pose_covariance_ = covariance_source == "message";
    const double pose_std_xy = declareDouble(
      *this, "pose_std_xy_m", 0.01, 1e-4, 10.0, "fixed のときの姿勢の位置の標準偏差 [m]", true);
    const double pose_std_yaw = declareDouble(
      *this, "pose_std_yaw_rad", 0.01, 1e-5, 1.0, "fixed のときの姿勢の向きの標準偏差 [rad]", true);
    fixed_pose_covariance_ = Eigen::Vector3d(
      pose_std_xy * pose_std_xy, pose_std_xy * pose_std_xy, pose_std_yaw * pose_std_yaw).asDiagonal();
    const double max_message_std = declareDouble(
      *this, "max_message_pose_std_xy_m", 1.0, 0.01, 1000.0,
      "/pcl_pose の位置の標準偏差がこれを超えたら、照合ではない（LIO で進めた）姿勢として使わない [m]", true);
    max_message_pose_variance_xy_ = max_message_std * max_message_std;
    const double rate_hz = declareDouble(
      *this, "publish_rate_hz", 50.0, 1.0, 1000.0, "出力の周期 [Hz]", true);

    const std::string pose_topic = declareString(*this, "pose_topic", "/localization/pcl_pose", "NDT の姿勢", true);
    const std::string twist_topic = declareString(
      *this, "twist_topic", "/localization/twist_with_covariance", "gyro_odometer の速度", true);
    const std::string output_topic = declareString(
      *this, "output_topic", "/localization/kinematic_state", "合わせた姿勢と速度", true);

    publisher_ = create_publisher<Odometry>(output_topic, rclcpp::QoS(10));
    // /pcl_pose は reliable・transient_local で出る。volatile で受け、起動前の古い姿勢は受けない
    pose_subscription_ = create_subscription<PoseWithCovarianceStamped>(
      pose_topic, rclcpp::QoS(10).reliable(),
      [this](PoseWithCovarianceStamped::ConstSharedPtr msg) {onPose(*msg);});
    twist_subscription_ = create_subscription<TwistWithCovarianceStamped>(
      twist_topic, rclcpp::QoS(50),
      [this](TwistWithCovarianceStamped::ConstSharedPtr msg) {onTwist(*msg);});
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / rate_hz), [this]() {onTimer();});
  }

private:
  void onPose(const PoseWithCovarianceStamped & msg)
  {
    if (!msg.header.frame_id.empty() && msg.header.frame_id != map_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "姿勢の座標系 %s が %s と違うので使いません",
        msg.header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    const auto & covariance = msg.pose.covariance;
    if (std::max(covariance[0], covariance[7]) > max_message_pose_variance_xy_) {
      ++counts_.bridge_skipped;
      return;
    }
    PoseMeasurement pose;
    pose.x = msg.pose.pose.position.x;
    pose.y = msg.pose.pose.position.y;
    pose.yaw = yawOf(msg.pose.pose.orientation);
    pose.covariance = use_message_pose_covariance_ ? planarCovariance(covariance) : fixed_pose_covariance_;
    const UpdateResult result = ekf_.updatePose(toSeconds(msg.header.stamp), pose);
    count(result, counts_.pose);
    if (result == UpdateResult::kAccepted) {
      last_pose_z_ = msg.pose.pose.position.z;
      last_pose_covariance_ = covariance;
    } else if (result == UpdateResult::kReset) {
      RCLCPP_WARN(
        get_logger(), "NDT の姿勢を %d 回続けて捨てたので、NDT の姿勢 (%.2f, %.2f, %.1f deg) に置き換えました",
        ekf_params_.pose_rejections_before_reset, pose.x, pose.y, pose.yaw * 180.0 / M_PI);
      last_pose_z_ = msg.pose.pose.position.z;
      last_pose_covariance_ = covariance;
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "NDT の姿勢を使いませんでした（%s、遅れ %.3f s）", describe(result),
        get_clock()->now().seconds() - toSeconds(msg.header.stamp));
    }
  }

  void onTwist(const TwistWithCovarianceStamped & msg)
  {
    TwistMeasurement twist;
    twist.vx = msg.twist.twist.linear.x;
    twist.vy = msg.twist.twist.linear.y;
    twist.wz = msg.twist.twist.angular.z;
    twist.covariance = planarCovariance(msg.twist.covariance);
    const UpdateResult result = ekf_.updateTwist(toSeconds(msg.header.stamp), twist);
    count(result, counts_.twist);
    if (result == UpdateResult::kRejectedByGate || result == UpdateResult::kTooOld) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "速度を使いませんでした（%s）", describe(result));
    }
  }

  void onTimer()
  {
    if (!ekf_.initialized()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "NDT の姿勢を待っています");
      return;
    }
    const double now = get_clock()->now().seconds();
    if (now + kClockJumpBackS < ekf_.stamp()) {
      RCLCPP_WARN(get_logger(), "時計が %.1f s 戻ったので、EKF を初めからにします", ekf_.stamp() - now);
      ekf_ = EkfLocalizer(ekf_params_);
      return;
    }
    if (now - ekf_.stamp() > max_extrapolation_s_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "最後の測定から %.2f s 経ったので、姿勢を出しません",
        now - ekf_.stamp());
      return;
    }
    publish(ekf_.predicted(now));
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "姿勢: 採用 %lu、捨てた %lu、置き換え %lu、古い %lu、bridge %lu／速度: 採用 %lu、捨てた %lu、古い %lu",
      counts_.pose.accepted, counts_.pose.rejected, counts_.pose.reset, counts_.pose.too_old,
      counts_.bridge_skipped, counts_.twist.accepted, counts_.twist.rejected, counts_.twist.too_old);
  }

  void publish(const EkfLocalizer::Belief & belief)
  {
    Odometry odometry;
    odometry.header.stamp = toStamp(belief.stamp_s);
    odometry.header.frame_id = map_frame_;
    odometry.child_frame_id = base_frame_;
    // 状態は平面なので、roll と pitch は 0、z と、その 3 つの分散は最後に使った NDT の値にする
    odometry.pose.pose.position.x = belief.x(0);
    odometry.pose.pose.position.y = belief.x(1);
    odometry.pose.pose.position.z = last_pose_z_;
    odometry.pose.pose.orientation.z = std::sin(0.5 * belief.x(2));
    odometry.pose.pose.orientation.w = std::cos(0.5 * belief.x(2));
    odometry.pose.covariance.fill(0.0);
    odometry.pose.covariance[14] = last_pose_covariance_[14];
    odometry.pose.covariance[21] = last_pose_covariance_[21];
    odometry.pose.covariance[28] = last_pose_covariance_[28];
    setPlanarCovariance(odometry.pose.covariance, belief.p.topLeftCorner<3, 3>());
    odometry.twist.twist.linear.x = belief.x(3);
    odometry.twist.twist.linear.y = belief.x(4);
    odometry.twist.twist.angular.z = belief.x(5);
    odometry.twist.covariance.fill(0.0);
    setPlanarCovariance(odometry.twist.covariance, belief.p.bottomRightCorner<3, 3>());
    publisher_->publish(odometry);
  }

  struct ResultCounts
  {
    std::uint64_t accepted{0};
    std::uint64_t rejected{0};
    std::uint64_t reset{0};
    std::uint64_t too_old{0};
  };
  struct Counts
  {
    ResultCounts pose;
    ResultCounts twist;
    std::uint64_t bridge_skipped{0};
  };

  static void count(const UpdateResult result, ResultCounts & counts)
  {
    switch (result) {
      case UpdateResult::kAccepted:
        ++counts.accepted;
        break;
      case UpdateResult::kRejectedByGate:
        ++counts.rejected;
        break;
      case UpdateResult::kReset:
        ++counts.reset;
        break;
      case UpdateResult::kTooOld:
        ++counts.too_old;
        break;
      case UpdateResult::kNotInitialized:
        break;
    }
  }

  EkfParams ekf_params_;
  EkfLocalizer ekf_;
  std::string map_frame_;
  std::string base_frame_;
  double max_extrapolation_s_{0.5};
  bool use_message_pose_covariance_{false};
  Eigen::Matrix3d fixed_pose_covariance_{Eigen::Matrix3d::Identity()};
  double max_message_pose_variance_xy_{1.0};
  double last_pose_z_{0.0};
  std::array<double, 36> last_pose_covariance_{};
  Counts counts_;
  rclcpp::Publisher<Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr pose_subscription_;
  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr twist_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace core_localization

RCLCPP_COMPONENTS_REGISTER_NODE(core_localization::EkfLocalizerNode)
