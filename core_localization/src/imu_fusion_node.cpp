// 2 つの IMU を測定の段階で融合し、主の IMU の座標系・時刻・周期で出すノード。
//
// 融合の式と、EKF などで平滑化した値を出さない理由は core_localization/imu_fusion.hpp に書いた。
// 補助から主への外部パラメータは TF（URDF）から一度だけ取る。取れるまでは出さない。
// 入力のトピックは primary と secondary、出力は output で、launch で付け替える。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "core_nav_common/parameter_updates.hpp"
#include "core_nav_common/parameters.hpp"
#include "core_localization/imu_fusion.hpp"

namespace core_localization
{
namespace
{
using sensor_msgs::msg::Imu;
namespace fusion = imu_fusion;

// 補助の時刻がこれ以上ずれていたら、時刻の同期の誤りとして知らせる
constexpr double kClockGapWarningS = 1.0;

double toSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

builtin_interfaces::msg::Time toStamp(const double seconds)
{
  return rclcpp::Time(static_cast<std::int64_t>(std::llround(seconds * 1e9)));
}

fusion::ImuSample toSample(const Imu & msg, const double accel_scale)
{
  fusion::ImuSample sample;
  sample.stamp_s = toSeconds(msg.header.stamp);
  sample.gyro = Eigen::Vector3d(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
  sample.accel = accel_scale * Eigen::Vector3d(
    msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
  return sample;
}

Imu toMessage(const fusion::FusedSample & sample, const std::string & frame)
{
  Imu msg;
  msg.header.stamp = toStamp(sample.stamp_s);
  msg.header.frame_id = frame;
  // 姿勢は出さない。sensor_msgs/Imu の決まりで、共分散の先頭を -1 にして知らせる
  msg.orientation_covariance[0] = -1.0;
  msg.angular_velocity.x = sample.gyro.x();
  msg.angular_velocity.y = sample.gyro.y();
  msg.angular_velocity.z = sample.gyro.z();
  msg.linear_acceleration.x = sample.accel.x();
  msg.linear_acceleration.y = sample.accel.y();
  msg.linear_acceleration.z = sample.accel.z();
  for (std::size_t axis = 0U; axis < 3U; ++axis) {
    msg.angular_velocity_covariance[axis * 4U] = sample.gyro_variance;
    msg.linear_acceleration_covariance[axis * 4U] = sample.accel_variance;
  }
  return msg;
}

const char * describe(const fusion::Contribution contribution)
{
  switch (contribution) {
    case fusion::Contribution::kBoth:
      return "主と補助";
    case fusion::Contribution::kPrimary:
      return "主だけ";
    case fusion::Contribution::kSecondary:
      return "補助だけ";
  }
  return "不明";
}

const char * describe(const fusion::OffsetStatus status)
{
  switch (status) {
    case fusion::OffsetStatus::kAccepted:
      return "採用";
    case fusion::OffsetStatus::kNotEnoughData:
      return "測定が足りない";
    case fusion::OffsetStatus::kLowExcitation:
      return "回転が足りない";
    case fusion::OffsetStatus::kLowCorrelation:
      return "相関が低い";
    case fusion::OffsetStatus::kAtSearchEdge:
      return "探す範囲の端";
  }
  return "不明";
}

bool sameEstimate(const fusion::OffsetEstimate & a, const fusion::OffsetEstimate & b)
{
  return a.status == b.status && a.offset_s == b.offset_s && a.correlation == b.correlation &&
         a.samples == b.samples;
}
}  // namespace

class ImuFusionNode final : public rclcpp::Node
{
public:
  ImuFusionNode()
  : Node("imu_fusion"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declareParameters();
    primary_frame_ = get_parameter("primary.frame").as_string();
    secondary_frame_ = get_parameter("secondary.frame").as_string();
    primary_accel_scale_ = get_parameter("primary.accel_scale").as_double();
    secondary_accel_scale_ = get_parameter("secondary.accel_scale").as_double();
    params_ = readParams({});
    if (const auto error = fusion::validateParams(params_)) {
      throw std::invalid_argument("imu_fusion のパラメータが不正です: " + *error);
    }
    // 1 つずつの範囲は宣言で検査する。組み合わせの誤りはここで理由を付けて拒否する
    validation_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & changed) {
        rcl_interfaces::msg::SetParametersResult result;
        const auto error = fusion::validateParams(readParams(changed));
        result.successful = !error.has_value();
        result.reason = error.value_or("");
        return result;
      });
    update_callback_ = core_nav_common::onParametersSet(
      *this, [this](const std::vector<rclcpp::Parameter> &) {
        params_ = readParams({});
        if (fusion_.has_value()) {
          fusion_->setParams(params_);
        }
      });

    // FAST-LIO は IMU を深さ 200 の Reliable で受ける
    publisher_ = create_publisher<Imu>("output", rclcpp::QoS(200));
    // ドライバが Best Effort でも Reliable でも受けられるよう Best Effort にする。
    // 処理が一時的に遅れても落とさないよう、深さは 1 秒分ほど取る
    const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(200)).best_effort();
    primary_subscription_ = create_subscription<Imu>(
      "primary", input_qos, [this](const Imu::ConstSharedPtr & msg) {onPrimary(*msg);});
    secondary_subscription_ = create_subscription<Imu>(
      "secondary", input_qos, [this](const Imu::ConstSharedPtr & msg) {onSecondary(*msg);});
  }

private:
  void declareParameters()
  {
    using core_nav_common::declareBool;
    using core_nav_common::declareDouble;
    using core_nav_common::declareInt;
    using core_nav_common::declareString;
    declareString(*this, "primary.frame", "damiao_imu_link", "主の IMU の座標系（出力の frame_id）", true);
    declareString(
      *this, "secondary.frame", "livox_imu_link", "補助の IMU の取り付け位置の座標系", true);
    for (const std::string role : {"primary", "secondary"}) {
      declareDouble(*this, role + ".accel_scale", 1.0, 0.01, 100.0, "加速度を m/s2 に直す倍率", true);
      declareDouble(
        *this, role + ".gyro_noise_std_radps", 0.002, 1e-6, 1.0, "角速度の雑音の標準偏差 [rad/s]");
      declareDouble(
        *this, role + ".accel_noise_std_mps2", 0.02, 1e-6, 10.0, "加速度の雑音の標準偏差 [m/s2]");
      declareDouble(
        *this, role + ".gyro_saturation_radps", 34.0, 0.1, 1000.0, "飽和とみなす角速度 [rad/s]");
      declareDouble(
        *this, role + ".accel_saturation_mps2", 150.0, 1.0, 10000.0, "飽和とみなす加速度 [m/s2]");
    }
    declareDouble(*this, "timeout_s", 0.05, 0.001, 1.0, "途絶とみなす測定の間隔 [s]");
    declareDouble(*this, "max_wait_s", 0.02, 0.0, 0.5, "補助の測定を待つ上限 [s]");
    declareDouble(
      *this, "secondary_time_offset_s", 0.0, -1.0, 1.0, "補助の時刻に足すと主の時刻になる値 [s]");
    declareDouble(
      *this, "angular_accel_time_constant_s", 0.02, 0.001, 1.0, "角加速度を平滑化する時定数 [s]");
    declareBool(
      *this, "time_offset_estimation.enabled", false, "時刻のずれを相互相関で推定して使う");
    declareDouble(*this, "time_offset_estimation.window_s", 5.0, 0.5, 60.0, "相関を取る長さ [s]");
    declareDouble(
      *this, "time_offset_estimation.search_range_s", 0.05, 0.001, 0.5, "探す幅（片側） [s]");
    declareDouble(*this, "time_offset_estimation.step_s", 0.0005, 0.0001, 0.01, "探す刻み [s]");
    declareDouble(
      *this, "time_offset_estimation.min_correlation", 0.9, 0.0, 1.0, "採用する相関係数の下限");
    declareDouble(
      *this, "time_offset_estimation.min_excitation_radps", 0.2, 0.0, 10.0,
      "推定に要る角速度の大きさの標準偏差 [rad/s]");
    declareDouble(*this, "time_offset_estimation.interval_s", 1.0, 0.1, 60.0, "推定の間隔 [s]");
    declareInt(*this, "unit_check.samples", 200, 0, 100000, "単位の確認に使う起動直後の測定の数", true);
    declareDouble(
      *this, "unit_check.tolerance", 0.3, 0.01, 0.9, "加速度の大きさと重力の差の許容（比）", true);
  }

  // changed に含まれる値はそれを、ほかは今の値を使って設定を組み立てる
  fusion::FusionParams readParams(const std::vector<rclcpp::Parameter> & changed) const
  {
    const auto value = [this, &changed](const std::string & name) {
        const auto found = std::find_if(
          changed.begin(), changed.end(),
          [&name](const rclcpp::Parameter & parameter) {return parameter.get_name() == name;});
        return found != changed.end() ? *found : get_parameter(name);
      };
    const auto sensor = [&value](const std::string & role) {
        fusion::SensorParams params;
        params.gyro_noise_std_radps = value(role + ".gyro_noise_std_radps").as_double();
        params.accel_noise_std_mps2 = value(role + ".accel_noise_std_mps2").as_double();
        params.gyro_saturation_radps = value(role + ".gyro_saturation_radps").as_double();
        params.accel_saturation_mps2 = value(role + ".accel_saturation_mps2").as_double();
        return params;
      };
    fusion::FusionParams params;
    params.primary = sensor("primary");
    params.secondary = sensor("secondary");
    params.timeout_s = value("timeout_s").as_double();
    params.max_wait_s = value("max_wait_s").as_double();
    params.secondary_time_offset_s = value("secondary_time_offset_s").as_double();
    params.angular_accel_time_constant_s = value("angular_accel_time_constant_s").as_double();
    auto & estimation = params.offset_estimation;
    estimation.enabled = value("time_offset_estimation.enabled").as_bool();
    estimation.window_s = value("time_offset_estimation.window_s").as_double();
    estimation.search_range_s = value("time_offset_estimation.search_range_s").as_double();
    estimation.step_s = value("time_offset_estimation.step_s").as_double();
    estimation.min_correlation = value("time_offset_estimation.min_correlation").as_double();
    estimation.min_excitation_radps =
      value("time_offset_estimation.min_excitation_radps").as_double();
    estimation.interval_s = value("time_offset_estimation.interval_s").as_double();
    params.unit_check_samples = static_cast<std::size_t>(value("unit_check.samples").as_int());
    params.unit_check_tolerance = value("unit_check.tolerance").as_double();
    return params;
  }

  // 外部パラメータは固定なので、TF から一度取れたら使い続ける
  bool ensureFusion()
  {
    if (fusion_.has_value()) {
      return true;
    }
    try {
      const auto transform =
        tf_buffer_.lookupTransform(primary_frame_, secondary_frame_, tf2::TimePointZero);
      const Eigen::Isometry3d primary_from_secondary = tf2::transformToEigen(transform);
      fusion::Extrinsic extrinsic;
      extrinsic.rotation = primary_from_secondary.rotation();
      // 補助の原点は主の座標系で translation の位置にあるので、補助から主へはその逆向き
      extrinsic.lever_arm = -primary_from_secondary.translation();
      fusion_.emplace(params_, extrinsic);
      RCLCPP_INFO(
        get_logger(),
        "外部パラメータを TF から取得しました（%s←%s）: 補助から主への位置 (%.4f, %.4f, %.4f)m、"
        "回転 %.3f度",
        primary_frame_.c_str(), secondary_frame_.c_str(), extrinsic.lever_arm.x(),
        extrinsic.lever_arm.y(), extrinsic.lever_arm.z(),
        Eigen::AngleAxisd(extrinsic.rotation).angle() * 180.0 / M_PI);
      return true;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "%s から %s への変換を取得できないので、まだ出しません: %s",
        primary_frame_.c_str(), secondary_frame_.c_str(), error.what());
      return false;
    }
  }

  void onPrimary(const Imu & msg)
  {
    if (!ensureFusion()) {
      return;
    }
    const auto outputs = fusion_->addPrimary(toSample(msg, primary_accel_scale_), now().seconds());
    report();
    publish(outputs);
  }

  void onSecondary(const Imu & msg)
  {
    if (!ensureFusion()) {
      return;
    }
    const auto outputs =
      fusion_->addSecondary(toSample(msg, secondary_accel_scale_), now().seconds());
    report();
    publish(outputs);
  }

  void publish(const std::vector<fusion::FusedSample> & samples)
  {
    for (const auto & sample : samples) {
      publisher_->publish(toMessage(sample, primary_frame_));
      reportContribution(sample);
    }
  }

  // 使う IMU が変わったときだけ知らせる
  void reportContribution(const fusion::FusedSample & sample)
  {
    const auto current = std::make_pair(sample.gyro_from, sample.accel_from);
    if (last_contribution_.has_value() && *last_contribution_ == current) {
      return;
    }
    last_contribution_ = current;
    if (current.first == fusion::Contribution::kBoth && current.second == fusion::Contribution::kBoth) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000, "角速度と加速度に、主と補助の両方を使っています");
      return;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "使う IMU が変わりました: 角速度は%s、加速度は%s（使わない側は飽和、途絶、単位の確認待ちのどれか）",
      describe(current.first), describe(current.second));
  }

  void report()
  {
    reportUnitCheck(
      "主", "primary.accel_scale", fusion_->primaryUnitCheck(), primary_unit_reported_);
    reportUnitCheck(
      "補助", "secondary.accel_scale", fusion_->secondaryUnitCheck(), secondary_unit_reported_);
    reportOffsetEstimate();
    const auto gap = fusion_->clockGap();
    if (gap.has_value() && std::abs(*gap) > kClockGapWarningS) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "補助の時刻が主から %.3f秒ずれていて、補助を使えません。時刻の同期か secondary_time_offset_s "
        "を確かめてください",
        *gap);
    }
  }

  void reportUnitCheck(
    const char * role, const std::string & scale_name, const fusion::UnitCheckResult & result,
    bool & reported)
  {
    if (reported || result.status == fusion::UnitCheckStatus::kPending) {
      return;
    }
    reported = true;
    if (result.status == fusion::UnitCheckStatus::kPassed) {
      RCLCPP_INFO(
        get_logger(), "%sの IMU の起動直後の加速度の大きさ %.3fm/s2 は重力と合っています", role,
        result.mean_accel_norm_mps2);
      return;
    }
    RCLCPP_ERROR(
      get_logger(),
      "%sの IMU の起動直後の加速度の大きさが %.3fm/s2 で、重力 %.3fm/s2 から %.0f%% 以上離れています。"
      "単位の倍率 %s（imu_source.yaml の accel_scale）を確かめてください。この IMU は使いません",
      role, result.mean_accel_norm_mps2, fusion::kStandardGravity, params_.unit_check_tolerance * 100.0,
      scale_name.c_str());
  }

  void reportOffsetEstimate()
  {
    const auto & estimate = fusion_->lastOffsetEstimate();
    if (!estimate.has_value() ||
      (last_estimate_.has_value() && sameEstimate(*last_estimate_, *estimate)))
    {
      return;
    }
    last_estimate_ = *estimate;
    if (estimate->status == fusion::OffsetStatus::kAccepted) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "補助の時刻のずれを推定しました: %.2fms（相関 %.3f、%zu サンプル）",
        estimate->offset_s * 1000.0, estimate->correlation, estimate->samples);
    } else if (estimate->status == fusion::OffsetStatus::kAtSearchEdge) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "時刻のずれの相関が探す範囲の端（%.2fms）で最大になり、採用しませんでした。"
        "secondary_time_offset_s を見直すか search_range_s を広げてください",
        estimate->offset_s * 1000.0);
    } else {
      RCLCPP_DEBUG(
        get_logger(), "時刻のずれを推定しませんでした: %s（相関 %.3f、%zu サンプル）",
        describe(estimate->status), estimate->correlation, estimate->samples);
    }
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string primary_frame_;
  std::string secondary_frame_;
  double primary_accel_scale_{1.0};
  double secondary_accel_scale_{1.0};
  fusion::FusionParams params_;
  std::optional<fusion::ImuFusion> fusion_;
  std::optional<std::pair<fusion::Contribution, fusion::Contribution>> last_contribution_;
  std::optional<fusion::OffsetEstimate> last_estimate_;
  bool primary_unit_reported_{false};
  bool secondary_unit_reported_{false};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr validation_callback_;
  core_nav_common::ParameterUpdateHandle update_callback_;
  rclcpp::Publisher<Imu>::SharedPtr publisher_;
  rclcpp::Subscription<Imu>::SharedPtr primary_subscription_;
  rclcpp::Subscription<Imu>::SharedPtr secondary_subscription_;
};
}  // namespace core_localization

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<core_localization::ImuFusionNode>());
  rclcpp::shutdown();
  return 0;
}
