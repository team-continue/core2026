// 手動と自動の運転モードを決める処理（ROS に依存しない部分）。
//
// - 手動から自動へは、UI の auto（目標の位置と向きを決めて押す）でだけ入る
// - 自動から手動へは、ゴールへの到達、走行の失敗、UI の停止、緊急停止で戻る
// - 緊急停止が入っている間は、auto を受け付けない
// - 速度指令は、今のモードの側のものだけを足回りへ通す
// - 目標は、通過点を順に通ってゴールへ行く経路（route）でも渡せる

#ifndef CORE_BODY_CONTROLLER__CONTROL_MODE_HPP_
#define CORE_BODY_CONTROLLER__CONTROL_MODE_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace core_body_controller
{

enum class ControlMode : std::uint8_t { kManual, kAuto };

// 自動の走行の状態（core_msgs/NavigationStatus の state を、このクラスで使う区分にまとめたもの）
enum class NavigationPhase : std::uint8_t {
  kIdle,     // ゴールなし・到達済み
  kActive,   // 走行中（DRIVING・STUCK_WAIT・BACKOFF・GOAL_WAIT）
  kFailed,   // 失敗
};

enum class CommandSource : std::uint8_t { kManual, kAuto };

inline std::string toString(const ControlMode mode)
{
  return mode == ControlMode::kAuto ? "auto" : "manual";
}

// 経路（通過点…、最後がゴール。各点は map の (x, y)）を受け付けない理由。受け付けるなら nullopt
inline std::optional<std::string> routeError(
  const std::vector<std::array<double, 2>> & points, const std::size_t max_points)
{
  if (points.empty()) {
    return "経路に点がありません";
  }
  if (points.size() > max_points) {
    return "経路の点が " + std::to_string(points.size()) + " 個あり、上限 " +
           std::to_string(max_points) + " 個を超えています";
  }
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (!std::isfinite(points[i][0]) || !std::isfinite(points[i][1])) {
      return std::to_string(i + 1) + " 番目の点の位置が有限でありません";
    }
  }
  return std::nullopt;
}

class ControlModeLogic
{
public:
  ControlMode mode() const {return mode_;}

  // UI の auto。目標を受けたので自動に入る。前の走行の状態は持ち越さない。
  // 緊急停止の間は入らず false を返す
  bool start()
  {
    if (emergency_) {
      return false;
    }
    mode_ = ControlMode::kAuto;
    active_seen_ = false;
    return true;
  }

  // 緊急停止（/system/emergency/hazard_status）。入ったら手動に戻す。解除しても自動には戻らない
  void emergency(const bool active)
  {
    emergency_ = active;
    if (active) {
      mode_ = ControlMode::kManual;
    }
  }

  bool emergencyActive() const {return emergency_;}

  // UI の停止
  void stop() {mode_ = ControlMode::kManual;}

  // ゴールに着いた（global_planner の /behavior/goal_reached）
  void reached() {mode_ = ControlMode::kManual;}

  // 走行の状態。auto を押した直後は、前のゴールの失敗が残って届くことがあるので、
  // 一度走行中になってからの失敗だけで手動に戻す
  void navigation(const NavigationPhase phase)
  {
    if (mode_ != ControlMode::kAuto) {
      return;
    }
    if (phase == NavigationPhase::kActive) {
      active_seen_ = true;
    } else if (phase == NavigationPhase::kFailed && active_seen_) {
      mode_ = ControlMode::kManual;
    }
  }

  // この指令を足回りへ通すか
  bool passes(const CommandSource source) const
  {
    return (source == CommandSource::kAuto) == (mode_ == ControlMode::kAuto);
  }

private:
  ControlMode mode_{ControlMode::kManual};
  bool active_seen_{false};
  bool emergency_{false};
};

}  // namespace core_body_controller

#endif  // CORE_BODY_CONTROLLER__CONTROL_MODE_HPP_
