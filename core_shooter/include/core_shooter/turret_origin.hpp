#ifndef CORE_SHOOTER__TURRET_ORIGIN_HPP_
#define CORE_SHOOTER__TURRET_ORIGIN_HPP_

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>

#include "core_shooter/turret_stall.hpp"

namespace core_shooter
{

// ============================================================================
// 原点合わせ（ストッパ押し当て）
//
// 角度はすべて joint_states の生角（ファームが出す角度）で扱う。
//
//   生角          = 論理角 + origin_offset
//   origin_offset = ストッパ接触時の生角 - limit_angle
//
// limit_angle は「原点から見たストッパの論理角」で、メカで決まる値。
// ============================================================================

/// 1 軸ぶんの押し当て設定。
struct OriginAxisParams
{
  double direction = 1.0;           ///< 押し当てる向き（生角の符号）。+1 / -1
  double limit_angle = 0.0;         ///< 原点から見たストッパの論理角 [rad]
  bool limit_angle_measured = false;  ///< false の間は原点合わせを拒否する
  double cad_limit_angle = 0.0;     ///< CAD 上のストッパ角 [rad]。実測値の確認に使う
  double cad_tolerance = 0.0;       ///< |実測 - CAD| の許容 [rad]。0 以下で確認しない
  double approach_speed = 0.0;      ///< 目標角の送り速度 [rad/s]
  double lead = 0.0;                ///< 目標角を実角より先に出す上限 [rad]。押し付ける力の上限になる
  StallParams contact;              ///< ストッパ接触の判定
  double min_travel = 0.0;          ///< 接触までに最低限動くべき量 [rad]
  double max_travel = 0.0;          ///< これだけ動いても（追従誤差を除いて）接触しなければ中断 [rad]
  double backoff = 0.0;             ///< 接触後にストッパから戻す量 [rad]
  double settle_sec = 0.0;          ///< 接触後・戻した後に保持する時間 [s]
  double timeout_sec = 0.0;         ///< 押し当て全体のタイムアウト [s]
};

/// 設定の矛盾を調べる。問題なければ nullopt。
inline std::optional<std::string> validateOriginAxisParams(const OriginAxisParams & p)
{
  if (p.direction != 1.0 && p.direction != -1.0) {
    return "direction must be 1.0 or -1.0";
  }
  if (!(p.approach_speed > 0.0) || !(p.lead > 0.0) || !(p.max_travel > 0.0) ||
    !(p.timeout_sec > 0.0))
  {
    return "approach_speed, lead, max_travel and timeout_sec must be > 0";
  }
  if (!(p.contact.error_threshold > 0.0) || !(p.contact.velocity_threshold > 0.0) ||
    p.contact.hold_sec < 0.0)
  {
    return "contact.error_threshold and contact.velocity_threshold must be > 0, "
           "contact.hold_sec must be >= 0";
  }
  if (p.contact.error_threshold >= p.lead) {
    // 目標角は実角 + lead までしか出ないので、それ以上のしきい値は永遠に超えない。
    return "contact.error_threshold must be < lead";
  }
  if (p.min_travel < 0.0 || p.min_travel + p.contact.error_threshold >= p.max_travel) {
    return "must satisfy 0 <= min_travel and min_travel + contact.error_threshold < max_travel";
  }
  if (p.backoff < 0.0 || p.settle_sec < 0.0) {
    return "backoff and settle_sec must be >= 0";
  }
  if (!std::isfinite(p.limit_angle) || !std::isfinite(p.cad_limit_angle) ||
    !std::isfinite(p.cad_tolerance))
  {
    return "limit_angle, cad_limit_angle and cad_tolerance must be finite";
  }
  return std::nullopt;
}

enum class OriginPhase
{
  Approach,  ///< ストッパへ向けて目標角を送っている
  Settle,    ///< 接触した。押し付けをやめてその場で保持している
  Backoff,   ///< ストッパから backoff だけ戻して保持している
  Done,
  Failed,
};

struct OriginAxisState
{
  OriginPhase phase = OriginPhase::Approach;
  double start_raw = 0.0;    ///< 押し当て開始時の生角 [rad]
  double target_raw = 0.0;   ///< この周期に出す生角の指令 [rad]
  double contact_raw = 0.0;  ///< 接触時の生角 [rad]。Settle 以降で有効
  double elapsed_sec = 0.0;
  double phase_sec = 0.0;
  StallState contact;
  std::string error;  ///< Failed のときの理由
};

inline OriginAxisState startOriginAxis(double actual_raw)
{
  OriginAxisState state;
  state.start_raw = actual_raw;
  state.target_raw = actual_raw;
  return state;
}

/// 目標角を direction へ送る。ただし実角より lead 以上は先に出さない。
///
/// 当たった後も目標角を送り続けると誤差が増え、押し付ける力が強くなる。
/// lead で頭打ちにすると、接触後の誤差（= 力）は lead で止まる。
inline double nextApproachTarget(
  double target, double actual, double direction, double speed, double lead, double dt)
{
  const double next = target + direction * speed * dt;
  return direction > 0.0 ? std::min(next, actual + lead) : std::max(next, actual - lead);
}

namespace detail
{
inline OriginAxisState failOriginAxis(OriginAxisState state, double actual, std::string error)
{
  state.phase = OriginPhase::Failed;
  state.target_raw = actual;
  state.error = std::move(error);
  return state;
}

inline std::string formatAngle(double value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4) << value;
  return stream.str();
}
}  // namespace detail

/// 押し当てを 1 周期ぶん進める。戻り値の target_raw を生角の指令として出す。
inline OriginAxisState stepOriginAxis(
  OriginAxisState state, double actual_raw, double velocity, double dt,
  const OriginAxisParams & p)
{
  if (state.phase == OriginPhase::Done || state.phase == OriginPhase::Failed) {
    return state;
  }
  state.elapsed_sec += dt;
  state.phase_sec += dt;

  switch (state.phase) {
    case OriginPhase::Approach: {
        const double travel = std::fabs(actual_raw - state.start_raw);
        // 目標角は max_travel で頭打ちにするので、実角はそこへ追従誤差を残して止まる。
        if (travel >= p.max_travel - p.contact.error_threshold) {
          return detail::failOriginAxis(
            state, actual_raw,
            "moved " + detail::formatAngle(travel) + " rad (max_travel) without contact; "
            "check direction");
        }
        if (state.elapsed_sec > p.timeout_sec) {
          return detail::failOriginAxis(
            state, actual_raw,
            "no contact within timeout_sec=" + detail::formatAngle(p.timeout_sec) + " s");
        }
        double target = nextApproachTarget(
          state.target_raw, actual_raw, p.direction, p.approach_speed, p.lead, dt);
        const double target_limit = state.start_raw + p.direction * p.max_travel;
        if (p.direction * (target - target_limit) > 0.0) {
          target = target_limit;
        }
        state.target_raw = target;
        state.contact = stepStall(state.contact, target, actual_raw, velocity, dt, p.contact);
        if (!state.contact.stalled) {
          return state;
        }
        if (travel < p.min_travel) {
          // 既にストッパに当たっている、またはモータが動いていない（トルクが無い）。
          return detail::failOriginAxis(
            state, actual_raw,
            "contact detected after only " + detail::formatAngle(travel) +
            " rad (< min_travel); move away from the stopper or check the motor");
        }
        state.phase = OriginPhase::Settle;
        state.phase_sec = 0.0;
        state.contact_raw = actual_raw;
        state.target_raw = actual_raw;  // 押し付けをやめる
        return state;
      }

    case OriginPhase::Settle:
      if (state.phase_sec >= p.settle_sec) {
        state.phase = OriginPhase::Backoff;
        state.phase_sec = 0.0;
        state.target_raw = state.contact_raw - p.direction * p.backoff;
      }
      return state;

    case OriginPhase::Backoff:
      if (state.phase_sec >= p.settle_sec) {
        state.phase = OriginPhase::Done;
      }
      return state;

    case OriginPhase::Done:
    case OriginPhase::Failed:
      break;
  }
  return state;
}

inline double computeOriginOffset(double contact_raw, double limit_angle)
{
  return contact_raw - limit_angle;
}

/// 原点に置いた状態の生角 origin_raw から、ストッパの論理角を求める。
inline double computeLimitAngle(double contact_raw, double origin_raw)
{
  return contact_raw - origin_raw;
}

/// 実測したストッパ角を CAD 値と比べる。許容内（または確認しない設定）なら nullopt。
inline std::optional<std::string> checkLimitAgainstCad(
  double measured_limit, const OriginAxisParams & p)
{
  if (p.cad_tolerance <= 0.0) {
    return std::nullopt;
  }
  const double diff = measured_limit - p.cad_limit_angle;
  if (std::fabs(diff) <= p.cad_tolerance) {
    return std::nullopt;
  }
  return "measured limit_angle=" + detail::formatAngle(measured_limit) +
         " differs from cad_limit_angle=" + detail::formatAngle(p.cad_limit_angle) +
         " by " + detail::formatAngle(diff) + " rad (> cad_tolerance=" +
         detail::formatAngle(p.cad_tolerance) + ")";
}

// ============================================================================
// 結果ファイル
// ============================================================================

struct OriginOffsets
{
  double yaw = 0.0;    ///< [rad] 生角 = 論理角 + yaw
  double pitch = 0.0;  ///< [rad] 生角 = 論理角 + pitch
};

/// 結果ファイルを読む。ファイルが無い場合も false（呼び出し側で区別する）。
inline bool loadOriginOffsets(
  const std::string & path, OriginOffsets & offsets, std::string & reason)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    reason = "failed to load '" + path + "': " + e.what();
    return false;
  }
  OriginOffsets loaded;
  try {
    loaded.yaw = root["yaw_origin_offset"].as<double>();
    loaded.pitch = root["pitch_origin_offset"].as<double>();
  } catch (const YAML::Exception &) {
    reason = "'" + path + "' must contain numeric yaw_origin_offset and pitch_origin_offset";
    return false;
  }
  if (!std::isfinite(loaded.yaw) || !std::isfinite(loaded.pitch)) {
    reason = "'" + path + "' contains non-finite origin offsets";
    return false;
  }
  offsets = loaded;
  return true;
}

inline std::string renderOriginOffsets(
  const OriginOffsets & offsets, const std::string & updated_at)
{
  std::ostringstream stream;
  stream << std::setprecision(9);
  stream <<
    "# 砲塔の原点オフセット [rad]。生角 = 論理角 + offset\n"
    "# aim_bot の calibrate_origin / measure_origin_limit が書き換える。手で編集しない。\n"
    "yaw_origin_offset: " << offsets.yaw << "\n"
    "pitch_origin_offset: " << offsets.pitch << "\n"
    "updated_at: \"" << updated_at << "\"\n";
  return stream.str();
}

/// 結果ファイルを書く。途中で落ちても壊れたファイルが残らないよう、一時ファイルから置き換える。
inline bool writeOriginOffsets(
  const std::string & path, const OriginOffsets & offsets, const std::string & updated_at,
  std::string & reason)
{
  namespace fs = std::filesystem;
  const fs::path target(path);
  std::error_code ec;
  if (target.has_parent_path()) {
    fs::create_directories(target.parent_path(), ec);
    if (ec) {
      reason = "failed to create '" + target.parent_path().string() + "': " + ec.message();
      return false;
    }
  }
  const fs::path tmp = target.string() + ".tmp";
  {
    std::ofstream stream(tmp, std::ios::trunc);
    if (!stream) {
      reason = "failed to open '" + tmp.string() + "' for writing";
      return false;
    }
    stream << renderOriginOffsets(offsets, updated_at);
    stream.flush();
    if (!stream) {
      reason = "failed to write '" + tmp.string() + "'";
      return false;
    }
  }
  fs::rename(tmp, target, ec);
  if (ec) {
    reason = "failed to replace '" + path + "': " + ec.message();
    std::error_code remove_ec;
    fs::remove(tmp, remove_ec);
    return false;
  }
  return true;
}

}  // namespace core_shooter

#endif  // CORE_SHOOTER__TURRET_ORIGIN_HPP_
