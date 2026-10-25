#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace core_mode
{

// Teensy の led.h の Pattern / Color と同じ番号
namespace led_pattern
{
constexpr uint8_t kOff = 0;
constexpr uint8_t kChase = 2;
constexpr uint8_t kCenterOutChase = 6;
constexpr uint8_t kBlink = 8;
constexpr uint8_t kDoubleFlash = 9;
constexpr uint8_t kSingleFlash = 11;
constexpr uint8_t kTripleFlash = 12;
constexpr uint8_t kBlinkMedium = 13;
constexpr uint8_t kBlinkSlow = 14;
}  // namespace led_pattern

namespace led_color
{
constexpr uint8_t kWhite = 0;
constexpr uint8_t kRed = 1;
constexpr uint8_t kYellow = 2;
constexpr uint8_t kGreen = 3;
}  // namespace led_color

struct LedCommand
{
  uint8_t pattern = led_pattern::kOff;
  uint8_t color = led_color::kWhite;

  bool operator==(const LedCommand & other) const
  {
    return pattern == other.pattern && color == other.color;
  }
};

// 状態ごとに出す表示。既定値は issue #144 の割り当て
struct LedCommandTable
{
  LedCommand microcontroller_timeout{led_pattern::kTripleFlash, led_color::kRed};
  LedCommand controller_disconnected{led_pattern::kDoubleFlash, led_color::kRed};
  LedCommand destroy{led_pattern::kBlinkSlow, led_color::kRed};
  LedCommand software_emergency{led_pattern::kSingleFlash, led_color::kRed};

  LedCommand turret_error{led_pattern::kBlink, led_color::kYellow};
  LedCommand turret_auto_firing{led_pattern::kDoubleFlash, led_color::kWhite};
  LedCommand turret_manual_firing{led_pattern::kSingleFlash, led_color::kWhite};
  LedCommand turret_enemy_detected{led_pattern::kBlink, led_color::kWhite};
  LedCommand turret_auto_idle{led_pattern::kCenterOutChase, led_color::kWhite};
  LedCommand turret_manual{led_pattern::kChase, led_color::kWhite};

  LedCommand chassis_auto_rotating{led_pattern::kBlinkMedium, led_color::kGreen};
  LedCommand chassis_manual_rotating{led_pattern::kBlinkMedium, led_color::kWhite};
  LedCommand chassis_auto{led_pattern::kChase, led_color::kGreen};
  LedCommand chassis_manual{led_pattern::kChase, led_color::kWhite};
};

// 砲塔 1 つ分の入力。途絶の扱いは呼び出し側で済ませておく
struct TurretInputs
{
  bool manual_mode = true;  // 未受信なら手動扱い
  bool error = false;  // 弾詰まり・装填エラー・発射エラーのどれか
  bool manual_firing = false;  // shooter_controller の発射通知から一定時間内
  bool auto_firing = false;  // aim_bot の aim_status が firing
};

struct LedInputs
{
  // /system/emergency/hazard_states と hazard_status
  bool microcontroller_timeout = false;
  bool controller_disconnected = false;
  bool destroy = false;
  bool software_emergency = false;
  // /behavior_system/state。途絶していれば nullopt (behavior_system が起動していないときも含む)
  std::optional<int32_t> behavior_state;
  bool rotating = false;  // /control/rotation != 0
  bool enemy_detected = false;
  std::array<TurretInputs, 2> turrets{};  // [0] = left, [1] = right
};

struct LedCommandPair
{
  LedCommand upper;  // 砲塔
  LedCommand bottom;  // 足回り
};

// behavior_system の状態 1〜4 (ATTACK / AUTO_SELECTED / AUTO_WAYPOINT / AUTO_IDLE) は auto
inline bool isAutoBehaviorState(int32_t state)
{
  return state >= 1 && state <= 4;
}

// 砲塔 1 つ分の表示と優先順位 (小さいほど優先)
struct RankedLedCommand
{
  int rank;
  LedCommand command;
};

inline RankedLedCommand selectTurretCommand(
  const TurretInputs & turret, bool enemy_detected, const LedCommandTable & table)
{
  if (turret.error) {
    return {0, table.turret_error};
  }
  if (!turret.manual_mode && turret.auto_firing) {
    return {1, table.turret_auto_firing};
  }
  if (turret.manual_mode && turret.manual_firing) {
    return {2, table.turret_manual_firing};
  }
  if (!turret.manual_mode && enemy_detected) {
    return {3, table.turret_enemy_detected};
  }
  if (!turret.manual_mode) {
    return {4, table.turret_auto_idle};
  }
  return {5, table.turret_manual};
}

// behavior の状態が届いていないときは auto ではない (manual) として扱う
inline LedCommand selectChassisCommand(
  const std::optional<int32_t> & behavior_state, bool rotating, const LedCommandTable & table)
{
  const bool is_auto = behavior_state.has_value() && isAutoBehaviorState(*behavior_state);
  if (rotating) {
    return is_auto ? table.chassis_auto_rotating : table.chassis_manual_rotating;
  }
  return is_auto ? table.chassis_auto : table.chassis_manual;
}

// 上 (砲塔) と下 (足回り) の表示を決める。異常は上下とも同じ表示にする
inline LedCommandPair selectLedCommands(const LedInputs & in, const LedCommandTable & table)
{
  const auto both = [](const LedCommand & command) {
      return LedCommandPair{command, command};
    };
  if (in.microcontroller_timeout) {
    return both(table.microcontroller_timeout);
  }
  if (in.controller_disconnected) {
    return both(table.controller_disconnected);
  }
  if (in.destroy) {
    return both(table.destroy);
  }
  if (in.software_emergency) {
    return both(table.software_emergency);
  }

  // 左右の砲塔のうち、優先度の高い方を出す
  const RankedLedCommand left = selectTurretCommand(in.turrets[0], in.enemy_detected, table);
  const RankedLedCommand right = selectTurretCommand(in.turrets[1], in.enemy_detected, table);
  const LedCommand upper = left.rank <= right.rank ? left.command : right.command;
  const LedCommand bottom = selectChassisCommand(in.behavior_state, in.rotating, table);
  return {upper, bottom};
}

}  // namespace core_mode
