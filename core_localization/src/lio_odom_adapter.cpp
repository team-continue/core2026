// FAST-LIO の出力を odom→base_link に変換するノード。
//
// FAST-LIO はフレーム名が camera_init→body に固定で、姿勢は機体ではなく IMU の位置を表す。
// camera_init は起動時の IMU の姿勢なので、odom を起動時の base_link と定めて次のように換算する。
//   T_odom_base = T_base_imu * T_camera_init_imu * T_base_imu^-1
// 起動時に機体が水平である前提で、odom の z 軸が鉛直になる。
// FAST-LIO の点群（camera_init）を同じ TF ツリーで扱えるよう、odom→camera_init も出す。
// lio_extrinsic_T・lio_extrinsic_R を与えると、FAST-LIO に渡した外部パラメータが TF と合うかを確かめる。

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace core_localization
{
namespace
{
using nav_msgs::msg::Odometry;

// FAST-LIO の外部パラメータと TF の差の許容。超えると odom→base_link がずれる
constexpr double kExtrinsicToleranceM = 0.005;
constexpr double kExtrinsicToleranceDeg = 0.5;

Eigen::Isometry3d composeBasePose(
  const Eigen::Isometry3d & base_to_imu, const Eigen::Isometry3d & camera_init_to_imu)
{
  return base_to_imu * camera_init_to_imu * base_to_imu.inverse();
}

// IMU の位置での速度を、base_link の位置の速度に直す。結果は base_link の座標系で表す
geometry_msgs::msg::Twist toBaseTwist(
  const Eigen::Isometry3d & base_to_imu, const geometry_msgs::msg::Twist & imu_twist)
{
  const Eigen::Vector3d linear_imu(
    imu_twist.linear.x, imu_twist.linear.y, imu_twist.linear.z);
  const Eigen::Vector3d angular_imu(
    imu_twist.angular.x, imu_twist.angular.y, imu_twist.angular.z);
  const Eigen::Vector3d angular_base = base_to_imu.rotation() * angular_imu;
  const Eigen::Vector3d linear_base =
    base_to_imu.rotation() * linear_imu - angular_base.cross(base_to_imu.translation());

  geometry_msgs::msg::Twist twist;
  twist.linear.x = linear_base.x();
  twist.linear.y = linear_base.y();
  twist.linear.z = linear_base.z();
  twist.angular.x = angular_base.x();
  twist.angular.y = angular_base.y();
  twist.angular.z = angular_base.z();
  return twist;
}

geometry_msgs::msg::TransformStamped toTransform(const Odometry & odom)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header = odom.header;
  transform.child_frame_id = odom.child_frame_id;
  transform.transform.translation.x = odom.pose.pose.position.x;
  transform.transform.translation.y = odom.pose.pose.position.y;
  transform.transform.translation.z = odom.pose.pose.position.z;
  transform.transform.rotation = odom.pose.pose.orientation;
  return transform;
}
}  // namespace

class LioOdomAdapter final : public rclcpp::Node
{
public:
  explicit LioOdomAdapter(const rclcpp::NodeOptions & options)
  : Node("lio_odom_adapter", options),
    odom_frame_(declare_parameter<std::string>("odom_frame", "odom")),
    base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
    imu_frame_(declare_parameter<std::string>("imu_frame", "livox_imu_link")),
    camera_init_frame_(declare_parameter<std::string>("camera_init_frame", "camera_init")),
    lidar_frame_(declare_parameter<std::string>("lidar_frame", "livox_frame")),
    lio_extrinsic_t_(declare_parameter<std::vector<double>>("lio_extrinsic_T", std::vector<double>{})),
    lio_extrinsic_r_(declare_parameter<std::vector<double>>("lio_extrinsic_R", std::vector<double>{})),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(*this),
    static_tf_broadcaster_(*this),
    publisher_(create_publisher<Odometry>("/localization/lio/odom", rclcpp::QoS(10))),
    subscription_(create_subscription<Odometry>(
        "/localization/Odometry", rclcpp::QoS(20),
        std::bind(&LioOdomAdapter::onOdometry, this, std::placeholders::_1)))
  {
  }

private:
  // base_link から IMU までの位置は固定なので、一度取得したら使い続ける
  const std::optional<Eigen::Isometry3d> & baseToImu()
  {
    if (base_to_imu_.has_value()) {
      return base_to_imu_;
    }
    try {
      auto base_to_imu = tf_buffer_.lookupTransform(base_frame_, imu_frame_, tf2::TimePointZero);
      base_to_imu_ = tf2::transformToEigen(base_to_imu);
      // camera_init は起動時の IMU なので、odom から見ると base_link→IMU と同じ位置にある
      base_to_imu.header.stamp = now();
      base_to_imu.header.frame_id = odom_frame_;
      base_to_imu.child_frame_id = camera_init_frame_;
      static_tf_broadcaster_.sendTransform(base_to_imu);
      checkLioExtrinsic();
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "%s から %s への変換を取得できません: %s",
        base_frame_.c_str(), imu_frame_.c_str(), error.what());
    }
    return base_to_imu_;
  }

  // FAST-LIO の外部パラメータ（IMU の座標系での LiDAR の位置と姿勢）は launch が設定ファイルから渡し、
  // この換算は TF を使う。両方がずれていると odom→base_link が回転のたびにずれるので、起動時に比べる
  void checkLioExtrinsic()
  {
    if (lio_extrinsic_t_.empty() && lio_extrinsic_r_.empty()) {
      return;
    }
    if (lio_extrinsic_t_.size() != 3U || lio_extrinsic_r_.size() != 9U) {
      RCLCPP_ERROR(
        get_logger(), "lio_extrinsic_T は 3 個、lio_extrinsic_R は 9 個の値にしてください（%zu 個と %zu 個）",
        lio_extrinsic_t_.size(), lio_extrinsic_r_.size());
      return;
    }
    try {
      const Eigen::Isometry3d imu_to_lidar = tf2::transformToEigen(
        tf_buffer_.lookupTransform(imu_frame_, lidar_frame_, tf2::TimePointZero));
      const Eigen::Vector3d expected_t(lio_extrinsic_t_[0], lio_extrinsic_t_[1], lio_extrinsic_t_[2]);
      // FAST-LIO と同じく行優先で読む
      const Eigen::Matrix3d expected_r =
        Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(lio_extrinsic_r_.data());
      const double translation_error = (imu_to_lidar.translation() - expected_t).norm();
      const double rotation_error_deg =
        Eigen::AngleAxisd(imu_to_lidar.rotation().transpose() * expected_r).angle() * 180.0 / M_PI;
      if (translation_error > kExtrinsicToleranceM || rotation_error_deg > kExtrinsicToleranceDeg) {
        RCLCPP_ERROR(
          get_logger(),
          "FAST-LIO の外部パラメータが TF と合いません（%s での %s の位置: TF (%.4f, %.4f, %.4f)m、"
          "設定 (%.4f, %.4f, %.4f)m、回転の差 %.2f度）。odom→base_link が回転のたびにずれるので、"
          "imu_source.yaml か URDF を直してください",
          imu_frame_.c_str(), lidar_frame_.c_str(), imu_to_lidar.translation().x(),
          imu_to_lidar.translation().y(), imu_to_lidar.translation().z(), expected_t.x(),
          expected_t.y(), expected_t.z(), rotation_error_deg);
        return;
      }
      RCLCPP_INFO(
        get_logger(), "FAST-LIO の外部パラメータは TF と合っています（%s、差 %.1fmm、%.2f度）",
        imu_frame_.c_str(), translation_error * 1000.0, rotation_error_deg);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN(
        get_logger(), "外部パラメータを確かめるための %s から %s への変換を取得できません: %s",
        imu_frame_.c_str(), lidar_frame_.c_str(), error.what());
    }
  }

  void onOdometry(const Odometry::ConstSharedPtr & input)
  {
    const auto & base_to_imu = baseToImu();
    if (!base_to_imu.has_value()) {
      return;
    }
    Eigen::Isometry3d camera_init_to_imu;
    tf2::fromMsg(input->pose.pose, camera_init_to_imu);

    Odometry output;
    output.header.stamp = input->header.stamp;
    output.header.frame_id = odom_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose = tf2::toMsg(composeBasePose(*base_to_imu, camera_init_to_imu));
    output.twist.twist = toBaseTwist(*base_to_imu, input->twist.twist);

    tf_broadcaster_.sendTransform(toTransform(output));
    publisher_->publish(output);
  }

  const std::string odom_frame_;
  const std::string base_frame_;
  const std::string imu_frame_;
  const std::string camera_init_frame_;
  const std::string lidar_frame_;
  const std::vector<double> lio_extrinsic_t_;
  const std::vector<double> lio_extrinsic_r_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
  std::optional<Eigen::Isometry3d> base_to_imu_;
  rclcpp::Publisher<Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<Odometry>::SharedPtr subscription_;
};
}  // namespace core_localization

RCLCPP_COMPONENTS_REGISTER_NODE(core_localization::LioOdomAdapter)
