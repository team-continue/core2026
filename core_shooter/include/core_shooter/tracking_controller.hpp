#ifndef CORE_SHOOTER__TRACKING_CONTROLLER_HPP_
#define CORE_SHOOTER__TRACKING_CONTROLLER_HPP_

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>

#include "core_shooter/joint_state_history.hpp"

namespace core_shooter
{

// ============================================================================
// 的の角度・角速度の推定
// ============================================================================

/// 的の角速度推定のフィルタ設定。
struct TargetVelocityFilter
{
  /// これ未満のサンプル間隔では速度を更新しない [s]。
  double min_dt_sec = 0.01;
  /// 角速度の上限 [rad/s]。外れ値で予測が飛ぶのを防ぐ。
  double max_yaw_rate = 10.0;
  double max_pitch_rate = 10.0;
  /// EMA 係数 [0, 1]。1 で生値、0 で速度を使わない。
  double ema_alpha = 0.25;
};

/// 砲塔基部から見た的の絶対角（モータ角）とその角速度。
///
/// 画像座標で速度を取ると砲塔自身の回転が混ざるので、撮影時刻の関節角に
/// 画像誤差を足した絶対角の列から速度を求める。
struct TargetTrack
{
  /// 最新の観測。位置の予測はここから伸ばす。
  TimedAngles latest;
  /// 速度を計算した基準の観測。間隔が短すぎる観測では更新しない。
  TimedAngles reference;
  AxisPair velocity;
  bool has_velocity = false;
};

inline TargetTrack startTargetTrack(const TimedAngles & sample)
{
  return TargetTrack{sample, sample, AxisPair{}, false};
}

/// 新しい観測で的の推定を更新する。
inline TargetTrack updateTargetTrack(
  const TargetTrack & track, const TimedAngles & sample, const TargetVelocityFilter & filter)
{
  const double dt = sample.time_sec - track.reference.time_sec;
  if (dt <= 0.0) {
    // 時刻が巻き戻った・同じ時刻が再送された。速度は信用できないので測り直す。
    return startTargetTrack(sample);
  }
  if (dt < filter.min_dt_sec) {
    return TargetTrack{sample, track.reference, track.velocity, track.has_velocity};
  }

  const AxisPair instant{
    std::clamp(
      (sample.angles.yaw - track.reference.angles.yaw) / dt,
      -filter.max_yaw_rate, filter.max_yaw_rate),
    std::clamp(
      (sample.angles.pitch - track.reference.angles.pitch) / dt,
      -filter.max_pitch_rate, filter.max_pitch_rate)};

  AxisPair velocity = instant;
  if (filter.ema_alpha <= 0.0) {
    velocity = AxisPair{};
  } else if (track.has_velocity && filter.ema_alpha < 1.0) {
    const double keep = 1.0 - filter.ema_alpha;
    velocity = AxisPair{
      keep * track.velocity.yaw + filter.ema_alpha * instant.yaw,
      keep * track.velocity.pitch + filter.ema_alpha * instant.pitch};
  }
  return TargetTrack{sample, sample, velocity, true};
}

/// 指定時刻の的の角度を等速で予測する。velocity_gain は速度 FF の掛け率。
inline AxisPair predictTargetAngles(
  const TargetTrack & track, double time_sec, double velocity_gain)
{
  if (!track.has_velocity || velocity_gain == 0.0) {
    return track.latest.angles;
  }
  const double elapsed = time_sec - track.latest.time_sec;
  return AxisPair{
    track.latest.angles.yaw + velocity_gain * track.velocity.yaw * elapsed,
    track.latest.angles.pitch + velocity_gain * track.velocity.pitch * elapsed};
}

// ============================================================================
// 速度 FF + PD（+ I）の追従則
// ============================================================================

/// 1軸ぶんの追従ゲイン。
///
/// モータは位置指令しか受けないので、指令は「目標角 + 補正角」の形にする。
///   command = target + kp * (target - measured)
///                    + ki * ∫(target - measured)
///                    + kd * (target_velocity - measured_velocity)
/// 全ゲイン 0 で指令は目標角そのもの（FF のみ）。kp でモータ位置ループの遅れを
/// 先回りし、kd で行き過ぎを抑える。D 項を速度差で取るので、画像が届くたびに
/// 目標角が段差で変わっても指令は跳ねない。
struct AxisTrackingGains
{
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  /// I 項の出力上限 [rad]。0 なら I 項を使わない。
  double integral_limit = 0.0;
};

struct AxisTrackingState
{
  double integral = 0.0;
};

struct AxisTrackingInput
{
  double target_angle = 0.0;
  /// FF に使う目標角速度 [rad/s]（velocity_gain 適用済み）。
  double target_velocity = 0.0;
  double measured_angle = 0.0;
  double measured_velocity = 0.0;
  double dt_sec = 0.0;
};

struct AxisTrackingResult
{
  double command = 0.0;
  AxisTrackingState state;
};

/// ゲインが使える値かを検査し、だめなら理由を返す。
inline std::optional<std::string> validateAxisTrackingGains(const AxisTrackingGains & gains)
{
  if (!std::isfinite(gains.kp) || !std::isfinite(gains.ki) || !std::isfinite(gains.kd) ||
    !std::isfinite(gains.integral_limit))
  {
    return std::string("gains must be finite");
  }
  if (gains.kp < 0.0 || gains.ki < 0.0 || gains.kd < 0.0 || gains.integral_limit < 0.0) {
    return std::string("kp/ki/kd/integral_limit must be >= 0");
  }
  return std::nullopt;
}

/// 追従則を1周期進める。
///
/// I 項は ki * integral が ±integral_limit を超えないように積分値を抑える（ワインドアップ対策）。
inline AxisTrackingResult stepAxisTracking(
  const AxisTrackingGains & gains, const AxisTrackingState & state,
  const AxisTrackingInput & input)
{
  const double error = input.target_angle - input.measured_angle;

  AxisTrackingState next{0.0};
  if (gains.ki > 0.0 && gains.integral_limit > 0.0) {
    const double integral_cap = gains.integral_limit / gains.ki;
    const double step = input.dt_sec > 0.0 ? error * input.dt_sec : 0.0;
    next.integral = std::clamp(state.integral + step, -integral_cap, integral_cap);
  }

  const double command =
    input.target_angle +
    gains.kp * error +
    gains.ki * next.integral +
    gains.kd * (input.target_velocity - input.measured_velocity);
  return AxisTrackingResult{command, next};
}

}  // namespace core_shooter

#endif  // CORE_SHOOTER__TRACKING_CONTROLLER_HPP_
