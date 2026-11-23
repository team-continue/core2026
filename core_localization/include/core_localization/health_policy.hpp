// 自己位置の異常の判定に使う、地図照合の結果の読み方と閾値。ROS に依存しない。
//
// 次の 3 つは、コードの既定では切ってある（今までの判定と同じ）。入れるかは設定ファイル
// （core_launch/config/localization_health.yaml）で決める。
// - 照合を捨てた回は「成功した照合」に数えない（捨て続けると、成功からの時間が延びて FAILED になる）
// - 照合を続けて捨てた回数が多いと DEGRADED
// - 採用した照合の補正が大きい（跳び）と、しばらく DEGRADED
// ほかに、速く回っているときに 2 つの角速度の比較が誤報しないよう、差の上限を角速度に比例して広げられる。

#ifndef CORE_LOCALIZATION__HEALTH_POLICY_HPP_
#define CORE_LOCALIZATION__HEALTH_POLICY_HPP_

#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>

namespace core_localization
{

// /alignment_status の 1 回分
struct AlignmentReport
{
  bool converged{false};
  double fitness{-1.0};
  // 出力がない手法では NaN
  double inlier_ratio{std::nan("")};
  // 0 なら、この回の照合は採用された
  int consecutive_rejected{0};
  double correction_m{0.0};
  double correction_deg{0.0};
  // 速く動いていたので、照合の結果を使わなかった（motion_gate_active）。捨てた回にも数えられない
  bool motion_gated{false};
};

struct HealthPolicyParams
{
  double max_fitness{2.0};
  double min_inlier_ratio{0.5};
  // 照合を捨てた回は成功に数えない
  bool count_rejected_as_failure{false};
  // 続けて捨てた回数がこれ以上なら DEGRADED（0 で使わない）
  int degraded_rejected_updates{0};
  // 採用した照合の補正がこれ以上なら跳びとみなす（0 で使わない）
  double jump_translation_m{0.0};
  double jump_yaw_deg{0.0};
  // 角速度の差の上限を、大きい方の角速度のこの割合まで広げる（0 で広げない）
  double gyro_relative_tolerance{0.0};
};

inline bool matchQualityIsGood(const AlignmentReport & report, const HealthPolicyParams & params)
{
  return report.fitness <= params.max_fitness &&
         (std::isnan(report.inlier_ratio) || report.inlier_ratio >= params.min_inlier_ratio);
}

// 成功した照合として、最後に成功した時刻を更新してよいか
inline bool isGoodMatch(const AlignmentReport & report, const HealthPolicyParams & params)
{
  if (!report.converged || !matchQualityIsGood(report, params)) {
    return false;
  }
  return !params.count_rejected_as_failure || report.consecutive_rejected == 0;
}

// 照合を続けて捨てていて DEGRADED にするか
inline bool rejectsTooMany(const AlignmentReport & report, const HealthPolicyParams & params)
{
  return params.degraded_rejected_updates > 0 &&
         report.consecutive_rejected >= params.degraded_rejected_updates;
}

// 採用した照合の補正が大きい（跳び）か
inline bool isJump(const AlignmentReport & report, const HealthPolicyParams & params)
{
  // 捨てた回と、動きで使わなかった回の補正は、機体の位置を動かさない
  if (report.consecutive_rejected != 0 || report.motion_gated) {
    return false;
  }
  const bool translation = params.jump_translation_m > 0.0 &&
    report.correction_m >= params.jump_translation_m;
  const bool yaw = params.jump_yaw_deg > 0.0 && report.correction_deg >= params.jump_yaw_deg;
  return translation || yaw;
}

// 角速度の大きさの差の上限 [rad/s]
inline double gyroDifferenceLimit(
  const HealthPolicyParams & params, double base_limit, double rate_a, double rate_b)
{
  const double relative = params.gyro_relative_tolerance * std::max(std::abs(rate_a), std::abs(rate_b));
  return std::max(base_limit, relative);
}

// 時刻つきの角速度の大きさ
struct StampedRate
{
  double stamp_s{0.0};
  double rate{0.0};
};

// 時刻が [start_s, end_s] の値の平均。1 つもなければ nullopt。
// LIO の角速度（2 つのスキャンの姿勢の差）を、同じ区間の IMU と比べるのに使う。届いた時刻の窓で比べると、
// LIO は数十 ms 遅れて届くので、回り始めや止まるときに違う時刻どうしを比べてしまう
inline std::optional<double> averageInInterval(
  const std::deque<StampedRate> & samples, double start_s, double end_s)
{
  double sum = 0.0;
  int count = 0;
  for (const auto & sample : samples) {
    if (sample.stamp_s >= start_s && sample.stamp_s <= end_s) {
      sum += sample.rate;
      ++count;
    }
  }
  if (count == 0) {
    return std::nullopt;
  }
  return sum / static_cast<double>(count);
}

}  // namespace core_localization

#endif  // CORE_LOCALIZATION__HEALTH_POLICY_HPP_
