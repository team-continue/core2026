// 自己位置の異常を判定するノード。
//
// 次の 3 つを見る。
// - 地図照合の評価値と、最後に照合が成功してからの時間
// - Mid-360 内蔵 IMU と Damiao IMU の角速度の差（IMU の故障）
// - LIO の姿勢の変化と Damiao IMU の角速度の差（LIO の発散）
// 取り付けの向きに左右されないよう、角速度は大きさで比べる。
// 照合を捨てた回数と補正の大きさ（跳び）も見られる（health_policy.hpp。既定は切ってある）。

#include <chrono>
#include <deque>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <core_msgs/msg/localization_health.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "core_nav_common/parameter_updates.hpp"
#include "core_nav_common/parameters.hpp"
#include "core_localization/health_policy.hpp"

namespace core_localization
{
namespace
{
using diagnostic_msgs::msg::DiagnosticArray;
using core_msgs::msg::LocalizationHealth;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::Imu;

struct Params
{
  bool enabled{true};
  HealthPolicyParams policy;
  double max_gyro_difference_radps{0.5};
  double fail_timeout_s{2.0};
  // 跳びのあと DEGRADED にしておく時間
  double jump_hold_s{2.0};
  // LIO と Damiao の角速度を、LIO のスキャンの区間でそろえて比べる
  bool align_gyro_with_lio{false};
};

// LIO と比べるために残しておく Damiao の角速度の長さ [s]
constexpr double kGyroHistoryS = 2.0;

// 一定時間の平均を取る。瞬間値だと 2 つの IMU の時刻のずれで差が出る
class RateAverage
{
public:
  void add(const double value)
  {
    sum_ += value;
    ++count_;
  }

  std::optional<double> takeAverage()
  {
    if (count_ == 0) {
      return std::nullopt;
    }
    const double average = sum_ / static_cast<double>(count_);
    sum_ = 0.0;
    count_ = 0;
    return average;
  }

private:
  double sum_{0.0};
  int count_{0};
};

double gyroMagnitude(const Imu & imu)
{
  const auto & w = imu.angular_velocity;
  return std::sqrt(w.x * w.x + w.y * w.y + w.z * w.z);
}

std::optional<std::string> findValue(const DiagnosticArray & array, const std::string & key)
{
  for (const auto & status : array.status) {
    for (const auto & value : status.values) {
      if (value.key == key) {
        return value.value;
      }
    }
  }
  return std::nullopt;
}

// 数として読めない値は、どの値が読めなかったかを例外で知らせる
double parseNumber(const std::string & key, const std::string & text)
{
  try {
    return std::stod(text);
  } catch (const std::exception &) {
    throw std::invalid_argument("/alignment_status の " + key + " を数として読めません: " + text);
  }
}

double findNumber(const DiagnosticArray & array, const std::string & key, const double fallback)
{
  const auto text = findValue(array, key);
  return text.has_value() ? parseNumber(key, *text) : fallback;
}
}  // namespace

class LocalizationHealthNode final : public rclcpp::Node
{
public:
  explicit LocalizationHealthNode(const rclcpp::NodeOptions & options)
  : Node("localization_health", options)
  {
    using core_nav_common::declareBool;
    using core_nav_common::declareDouble;
    declareBool(*this, "localization_health.enabled", true, "自己位置の異常を判定する");
    declareDouble(
      *this, "localization_health.max_fitness", 2.0, 0.01, 1000.0, "地図照合の評価値の上限");
    declareDouble(
      *this, "localization_health.min_inlier_ratio", 0.5, 0.0, 1.0, "地図照合のインライア率の下限");
    declareDouble(
      *this, "localization_health.max_gyro_difference_radps", 0.5, 0.01, 50.0,
      "角速度の大きさの差の上限");
    declareDouble(
      *this, "localization_health.fail_timeout_s", 2.0, 0.1, 3600.0, "異常が続くと失敗とする時間");
    declareBool(
      *this, "localization_health.count_rejected_as_failure", false,
      "地図照合を捨てた回を成功に数えない（捨て続けると FAILED）");
    core_nav_common::declareInt(
      *this, "localization_health.degraded_rejected_updates", 0, 0, 1000,
      "地図照合を続けてこの回数捨てたら DEGRADED（0 で使わない）");
    declareDouble(
      *this, "localization_health.jump_translation_m", 0.0, 0.0, 10.0,
      "採用した照合の補正がこれ以上なら跳びとして DEGRADED（0 で使わない）");
    declareDouble(
      *this, "localization_health.jump_yaw_deg", 0.0, 0.0, 180.0,
      "採用した照合の向きの補正がこれ以上なら跳びとして DEGRADED（0 で使わない）");
    declareDouble(
      *this, "localization_health.jump_hold_s", 2.0, 0.0, 60.0, "跳びのあと DEGRADED にしておく時間");
    declareDouble(
      *this, "localization_health.gyro_relative_tolerance", 0.0, 0.0, 1.0,
      "角速度の差の上限を、大きい方の角速度のこの割合まで広げる（速い旋回での誤報を防ぐ。0 で広げない）");
    declareBool(
      *this, "localization_health.align_gyro_with_lio", false,
      "LIO と Damiao の角速度を、LIO の 2 つのスキャンの時刻の区間でそろえて比べる（回り始めと止まるときの誤報を防ぐ）");
    loadParams();
    parameter_callback_ = core_nav_common::onParametersSet(
      *this, [this](const std::vector<rclcpp::Parameter> &) {loadParams();});

    publisher_ = create_publisher<LocalizationHealth>("/localization/health", rclcpp::QoS(10));
    const auto sensor_qos = rclcpp::SensorDataQoS();
    alignment_subscription_ = create_subscription<DiagnosticArray>(
      "/localization/alignment_status", rclcpp::QoS(10),
      std::bind(&LocalizationHealthNode::onAlignment, this, std::placeholders::_1));
    livox_imu_subscription_ = create_subscription<Imu>(
      "/sensing/livox/imu", sensor_qos,
      [this](const Imu::ConstSharedPtr & imu) {livox_rate_.add(gyroMagnitude(*imu));});
    damiao_imu_subscription_ = create_subscription<Imu>(
      "/sensing/imu", sensor_qos,
      std::bind(&LocalizationHealthNode::onDamiao, this, std::placeholders::_1));
    odometry_subscription_ = create_subscription<Odometry>(
      "/localization/lio/odom", rclcpp::QoS(10),
      std::bind(&LocalizationHealthNode::onOdometry, this, std::placeholders::_1));
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(200),
      std::bind(&LocalizationHealthNode::evaluate, this));
  }

private:
  void loadParams()
  {
    params_.enabled = get_parameter("localization_health.enabled").as_bool();
    params_.policy.max_fitness = get_parameter("localization_health.max_fitness").as_double();
    params_.policy.min_inlier_ratio =
      get_parameter("localization_health.min_inlier_ratio").as_double();
    params_.policy.count_rejected_as_failure =
      get_parameter("localization_health.count_rejected_as_failure").as_bool();
    params_.policy.degraded_rejected_updates = static_cast<int>(
      get_parameter("localization_health.degraded_rejected_updates").as_int());
    params_.policy.jump_translation_m =
      get_parameter("localization_health.jump_translation_m").as_double();
    params_.policy.jump_yaw_deg = get_parameter("localization_health.jump_yaw_deg").as_double();
    params_.policy.gyro_relative_tolerance =
      get_parameter("localization_health.gyro_relative_tolerance").as_double();
    params_.max_gyro_difference_radps =
      get_parameter("localization_health.max_gyro_difference_radps").as_double();
    params_.fail_timeout_s = get_parameter("localization_health.fail_timeout_s").as_double();
    params_.jump_hold_s = get_parameter("localization_health.jump_hold_s").as_double();
    params_.align_gyro_with_lio = get_parameter("localization_health.align_gyro_with_lio").as_bool();
  }

  void onDamiao(const Imu::ConstSharedPtr & imu)
  {
    const double rate = gyroMagnitude(*imu);
    damiao_rate_.add(rate);
    if (!params_.align_gyro_with_lio) {
      return;
    }
    const double stamp_s = rclcpp::Time(imu->header.stamp, get_clock()->get_clock_type()).seconds();
    damiao_history_.push_back(StampedRate{stamp_s, rate});
    while (!damiao_history_.empty() && damiao_history_.front().stamp_s < stamp_s - kGyroHistoryS) {
      damiao_history_.pop_front();
    }
  }

  // LIO の 1 区間の角速度を、同じ区間の Damiao の平均と比べ、評価の周期の中で最も悪い比を残す
  void compareAlignedGyro(const double lio_rate, const double start_s, const double end_s)
  {
    const auto damiao = averageInInterval(damiao_history_, start_s, end_s);
    if (!damiao.has_value()) {
      return;
    }
    const double difference = std::abs(lio_rate - *damiao);
    const double ratio = difference /
      gyroDifferenceLimit(params_.policy, params_.max_gyro_difference_radps, lio_rate, *damiao);
    if (!aligned_worst_ratio_.has_value() || ratio > *aligned_worst_ratio_) {
      aligned_worst_ratio_ = ratio;
      aligned_worst_difference_ = difference;
    }
  }

  void onAlignment(const DiagnosticArray::ConstSharedPtr & array)
  {
    const auto fitness = findValue(*array, "fitness_score");
    const auto converged = findValue(*array, "has_converged");
    if (!fitness.has_value() || !converged.has_value()) {
      return;
    }
    try {
      report_.converged = *converged == "true";
      report_.fitness = parseNumber("fitness_score", *fitness);
      // 出力がない手法では NaN になるので、そのときは比べない
      report_.inlier_ratio = findNumber(*array, "inlier_ratio", std::nan(""));
      report_.consecutive_rejected =
        static_cast<int>(findNumber(*array, "consecutive_rejected_updates", 0.0));
      report_.correction_m = findNumber(*array, "correction_translation_m", 0.0);
      report_.correction_deg = findNumber(*array, "correction_yaw_deg", 0.0);
      report_.motion_gated = findValue(*array, "motion_gate_active").value_or("false") == "true";
    } catch (const std::invalid_argument & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
      return;
    }
    if (isGoodMatch(report_, params_.policy)) {
      last_good_match_ = now();
    }
    if (isJump(report_, params_.policy)) {
      jump_until_ = now() + rclcpp::Duration::from_seconds(params_.jump_hold_s);
      RCLCPP_WARN(
        get_logger(), "地図照合の補正が大きい（%.3f m、%.2f 度）", report_.correction_m,
        report_.correction_deg);
    }
  }

  void onOdometry(const Odometry::ConstSharedPtr & odometry)
  {
    const auto & q = odometry->pose.pose.orientation;
    const tf2::Quaternion orientation(q.x, q.y, q.z, q.w);
    const rclcpp::Time stamp(odometry->header.stamp, get_clock()->get_clock_type());
    if (last_orientation_.has_value() && last_odometry_stamp_.has_value()) {
      const double dt = (stamp - *last_odometry_stamp_).seconds();
      if (dt > 0.0) {
        const double rate = last_orientation_->angleShortestPath(orientation) / dt;
        lio_rate_.add(rate);
        if (params_.align_gyro_with_lio) {
          compareAlignedGyro(rate, last_odometry_stamp_->seconds(), stamp.seconds());
        }
      }
    }
    last_orientation_ = orientation;
    last_odometry_stamp_ = stamp;
  }

  void evaluate()
  {
    LocalizationHealth health;
    health.header.stamp = now();
    health.fitness = static_cast<float>(report_.fitness);
    health.inlier_ratio = static_cast<float>(report_.inlier_ratio);

    const auto livox = livox_rate_.takeAverage();
    const auto damiao = damiao_rate_.takeAverage();
    const auto lio = lio_rate_.takeAverage();
    // 差を、その組の上限に対する比で比べる（速く回っているときは上限を広げられる）
    double difference = 0.0;
    double worst_ratio = 0.0;
    std::string gyro_source;
    const auto compare = [&](const double a, const double b, const char * source) {
        const double limit =
          gyroDifferenceLimit(params_.policy, params_.max_gyro_difference_radps, a, b);
        const double ratio = std::abs(a - b) / limit;
        if (ratio > worst_ratio) {
          worst_ratio = ratio;
          difference = std::abs(a - b);
          gyro_source = source;
        }
      };
    if (livox.has_value() && damiao.has_value()) {
      compare(*livox, *damiao, "内蔵IMUとDamiao IMUの角速度の差");
    }
    if (params_.align_gyro_with_lio) {
      // LIO のスキャンの区間でそろえて比べた結果のうち、この周期で最も悪いもの
      if (aligned_worst_ratio_.has_value() && *aligned_worst_ratio_ > worst_ratio) {
        worst_ratio = *aligned_worst_ratio_;
        difference = aligned_worst_difference_;
        gyro_source = "LIOとDamiao IMUの角速度の差";
      }
      aligned_worst_ratio_.reset();
    } else if (lio.has_value() && damiao.has_value()) {
      compare(*lio, *damiao, "LIOとDamiao IMUの角速度の差");
    }
    health.gyro_difference = static_cast<float>(difference);

    if (!last_good_match_.has_value()) {
      // 照合が始まるまでは判定しない。起動時から失敗扱いにしない
      start_time_ = start_time_.value_or(now());
    }
    const rclcpp::Time reference = last_good_match_.value_or(start_time_.value_or(now()));
    const double since_match = (now() - reference).seconds();
    health.time_since_match_s = static_cast<float>(since_match);

    const bool gyro_bad = worst_ratio > 1.0;
    if (!gyro_bad) {
      last_gyro_ok_ = now();
    }
    const double gyro_bad_duration =
      last_gyro_ok_.has_value() ? (now() - *last_gyro_ok_).seconds() : 0.0;

    if (!params_.enabled) {
      health.level = LocalizationHealth::LEVEL_OK;
      health.reason = "判定は無効";
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "localization_health が無効です。異常を検知できません");
    } else if (since_match > params_.fail_timeout_s) {
      health.level = LocalizationHealth::LEVEL_FAILED;
      health.reason = "地図照合が成功していない";
    } else if (!last_good_match_.has_value()) {
      // 照合が動いていないのに、起動の直後だけ正常と報告しないようにする
      health.level = LocalizationHealth::LEVEL_DEGRADED;
      health.reason = "地図照合の結果を待っている";
    } else if (gyro_bad && gyro_bad_duration > params_.fail_timeout_s) {
      health.level = LocalizationHealth::LEVEL_FAILED;
      health.reason = gyro_source;
    } else if (gyro_bad) {
      health.level = LocalizationHealth::LEVEL_DEGRADED;
      health.reason = gyro_source;
    } else if (rejectsTooMany(report_, params_.policy)) {
      health.level = LocalizationHealth::LEVEL_DEGRADED;
      health.reason = "地図照合を続けて捨てている（" + std::to_string(report_.consecutive_rejected) + " 回）";
    } else if (jump_until_.has_value() && now() < *jump_until_) {
      health.level = LocalizationHealth::LEVEL_DEGRADED;
      health.reason = "地図照合の補正が大きい（跳び）";
    } else if (report_.fitness >= 0.0 && !matchQualityIsGood(report_, params_.policy)) {
      health.level = LocalizationHealth::LEVEL_DEGRADED;
      health.reason = "地図照合の品質が悪い";
    } else {
      health.level = LocalizationHealth::LEVEL_OK;
    }
    publisher_->publish(health);
  }

  Params params_;
  AlignmentReport report_;
  std::optional<rclcpp::Time> jump_until_;
  std::optional<rclcpp::Time> last_good_match_;
  std::optional<rclcpp::Time> start_time_;
  std::optional<rclcpp::Time> last_gyro_ok_;
  std::optional<tf2::Quaternion> last_orientation_;
  std::optional<rclcpp::Time> last_odometry_stamp_;
  RateAverage livox_rate_;
  RateAverage damiao_rate_;
  RateAverage lio_rate_;
  std::deque<StampedRate> damiao_history_;
  std::optional<double> aligned_worst_ratio_;
  double aligned_worst_difference_{0.0};
  core_nav_common::ParameterUpdateHandle parameter_callback_;
  rclcpp::Publisher<LocalizationHealth>::SharedPtr publisher_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr alignment_subscription_;
  rclcpp::Subscription<Imu>::SharedPtr livox_imu_subscription_;
  rclcpp::Subscription<Imu>::SharedPtr damiao_imu_subscription_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace core_localization

RCLCPP_COMPONENTS_REGISTER_NODE(core_localization::LocalizationHealthNode)
