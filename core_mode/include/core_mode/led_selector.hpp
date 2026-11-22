#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

namespace core_mode
{

// Teensy の led.h の Pattern / Color と同じ番号
namespace led_pattern
{
constexpr uint8_t kOff = 0;
constexpr uint8_t kChase = 2;  // 一方向に流れる (向きは Teensy の区間ごとの定数)
constexpr uint8_t kMultiChase = 3;
constexpr uint8_t kCenterOutChase = 6;
constexpr uint8_t kBlink = 8;
constexpr uint8_t kDoubleFlash = 9;
constexpr uint8_t kFade = 10;
constexpr uint8_t kSingleFlash = 11;
constexpr uint8_t kTripleFlash = 12;
constexpr uint8_t kBlinkMedium = 13;
constexpr uint8_t kBlinkSlow = 14;
constexpr uint8_t kInwardChase = 15;  // 両端から中央へ流れる
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

// 状態ごとに出す表示。色は 赤 = 異常、白 = 手動、黄 = 自動
// 砲塔の手動・自動は砲塔ごとの manual_mode で決まり、behavior の状態とは別
struct LedCommandTable
{
  LedCommand microcontroller_timeout{led_pattern::kTripleFlash, led_color::kRed};
  LedCommand controller_disconnected{led_pattern::kDoubleFlash, led_color::kRed};
  LedCommand destroy{led_pattern::kBlinkSlow, led_color::kRed};
  LedCommand software_emergency{led_pattern::kSingleFlash, led_color::kRed};

  // 砲塔の異常は手動・自動で同じ
  LedCommand turret_error{led_pattern::kChase, led_color::kRed};
  LedCommand turret_manual_firing{led_pattern::kBlink, led_color::kWhite};
  LedCommand turret_manual_motor_on{led_pattern::kSingleFlash, led_color::kWhite};
  LedCommand turret_manual{led_pattern::kChase, led_color::kWhite};
  LedCommand turret_auto_firing{led_pattern::kBlink, led_color::kYellow};
  LedCommand turret_enemy_detected{led_pattern::kDoubleFlash, led_color::kYellow};
  LedCommand turret_auto_motor_on{led_pattern::kSingleFlash, led_color::kYellow};
  LedCommand turret_auto_fire_off{led_pattern::kChase, led_color::kYellow};  // 待機と同じ
  LedCommand turret_auto_idle{led_pattern::kChase, led_color::kYellow};

  LedCommand chassis_auto_rotating{led_pattern::kBlinkMedium, led_color::kYellow};
  LedCommand chassis_manual_rotating{led_pattern::kBlinkMedium, led_color::kWhite};
  LedCommand chassis_auto{led_pattern::kChase, led_color::kYellow};
  LedCommand chassis_manual{led_pattern::kChase, led_color::kWhite};

  // 上のテープの中央 (全体) に出す behavior の状態。AUTO の 3 つは同じ表示
  LedCommand behavior_manual{led_pattern::kChase, led_color::kWhite};
  LedCommand behavior_attack{led_pattern::kBlink, led_color::kYellow};
  LedCommand behavior_auto_selected{led_pattern::kChase, led_color::kYellow};
  LedCommand behavior_auto_waypoint{led_pattern::kChase, led_color::kYellow};
  LedCommand behavior_auto_idle{led_pattern::kChase, led_color::kYellow};
  LedCommand behavior_unknown{led_pattern::kFade, led_color::kWhite};  // 途絶・範囲外
};

// aim_bot の aim_status を LED の判定に要る分だけに分けたもの
enum class AimStatus
{
  kUnknown,  // 未受信・途絶、または知らない文字列
  kFiring,  // "firing"
  kAutoFireOff,  // "blocked: turret_auto off" (UI で自動射撃がオフ。敵の有無は分からない)
  kTargetInView,  // 敵を捉えているが撃てない (射程外・照準が合っていない)
  kNoTarget,  // "blocked: no target"
  kOtherBlocked,  // 手動・テスト・非常停止・起動中など
};

// aim_bot の publishAimStatus が出す文字列を分ける
inline AimStatus classifyAimStatus(std::string_view status)
{
  constexpr std::string_view kBlocked = "blocked: ";
  if (status == "firing") {
    return AimStatus::kFiring;
  }
  if (status.substr(0, kBlocked.size()) != kBlocked) {
    return AimStatus::kUnknown;
  }
  const std::string_view reason = status.substr(kBlocked.size());
  const auto starts_with = [reason](std::string_view prefix) {
      return reason.substr(0, prefix.size()) == prefix;
    };
  if (reason == "turret_auto off") {
    return AimStatus::kAutoFireOff;
  }
  if (reason == "no target") {
    return AimStatus::kNoTarget;
  }
  if (starts_with("out of range") || starts_with("not aimed") || starts_with("off center")) {
    return AimStatus::kTargetInView;
  }
  return AimStatus::kOtherBlocked;
}

// 砲塔 1 つ分の入力。途絶の扱いは呼び出し側で済ませておく
struct TurretInputs
{
  bool manual_mode = true;  // 未受信なら手動扱い
  bool error = false;  // 弾詰まり・装填エラー・発射エラーのどれか
  bool manual_firing = false;  // shooter_controller の発射通知から一定時間内
  bool shoot_motor_on = false;  // 発射モータに回す指令が出ている (shoot_motor > 0.1)
  AimStatus aim = AimStatus::kUnknown;  // その砲塔の aim_bot の aim_status
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
  std::array<TurretInputs, 2> turrets{};  // [0] = left, [1] = right
};

// 上のテープは 3 区間 (左の砲塔・全体・右の砲塔)、下のテープは 1 区間
struct LedCommands
{
  LedCommand upper_overall;  // behavior の状態
  LedCommand upper_left;  // 左の砲塔
  LedCommand upper_right;  // 右の砲塔
  LedCommand bottom;  // 足回り
};

// behavior_system の状態 1〜4 (ATTACK / AUTO_SELECTED / AUTO_WAYPOINT / AUTO_IDLE) は auto
inline bool isAutoBehaviorState(int32_t state)
{
  return state >= 1 && state <= 4;
}

// 砲塔 1 つ分の表示。上ほど優先。敵の有無はその砲塔の aim_bot で判断する
inline LedCommand selectTurretCommand(const TurretInputs & turret, const LedCommandTable & table)
{
  if (turret.error) {
    return table.turret_error;
  }
  if (turret.manual_mode) {
    if (turret.manual_firing) {
      return table.turret_manual_firing;
    }
    if (turret.shoot_motor_on) {
      return table.turret_manual_motor_on;
    }
    return table.turret_manual;
  }
  if (turret.aim == AimStatus::kFiring) {
    return table.turret_auto_firing;
  }
  if (turret.aim == AimStatus::kTargetInView) {
    return table.turret_enemy_detected;
  }
  // 自動でも発射モータは UI で回すので、手動と同じく見る
  if (turret.shoot_motor_on) {
    return table.turret_auto_motor_on;
  }
  if (turret.aim == AimStatus::kAutoFireOff) {
    return table.turret_auto_fire_off;
  }
  return table.turret_auto_idle;
}

// behavior_system の状態 (0 = MANUAL, 1 = ATTACK, 2 = AUTO_SELECTED, 3 = AUTO_WAYPOINT,
// 4 = AUTO_IDLE)。途絶・範囲外は behavior_unknown
inline LedCommand selectBehaviorCommand(
  const std::optional<int32_t> & behavior_state, const LedCommandTable & table)
{
  if (!behavior_state.has_value()) {
    return table.behavior_unknown;
  }
  switch (*behavior_state) {
    case 0:
      return table.behavior_manual;
    case 1:
      return table.behavior_attack;
    case 2:
      return table.behavior_auto_selected;
    case 3:
      return table.behavior_auto_waypoint;
    case 4:
      return table.behavior_auto_idle;
    default:
      return table.behavior_unknown;
  }
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

// 上の 3 区間と下の表示を決める。異常はすべての区間を同じ表示にする
inline LedCommands selectLedCommands(const LedInputs & in, const LedCommandTable & table)
{
  const auto all = [](const LedCommand & command) {
      return LedCommands{command, command, command, command};
    };
  if (in.microcontroller_timeout) {
    return all(table.microcontroller_timeout);
  }
  if (in.controller_disconnected) {
    return all(table.controller_disconnected);
  }
  if (in.destroy) {
    return all(table.destroy);
  }
  if (in.software_emergency) {
    return all(table.software_emergency);
  }

  return {
    selectBehaviorCommand(in.behavior_state, table),
    selectTurretCommand(in.turrets[0], table),
    selectTurretCommand(in.turrets[1], table),
    selectChassisCommand(in.behavior_state, in.rotating, table),
  };
}

}  // namespace core_mode
