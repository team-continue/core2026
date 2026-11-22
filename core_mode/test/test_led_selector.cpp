#include <gtest/gtest.h>

#include <array>
#include <utility>
#include <vector>

#include "core_mode/led_selector.hpp"

namespace
{

using core_mode::AimStatus;
using core_mode::classifyAimStatus;
using core_mode::LedCommands;
using core_mode::LedCommandTable;
using core_mode::LedInputs;
using core_mode::selectLedCommands;

// behavior の状態を受けていて、異常のない入力 (manual、回転なし、両砲塔とも手動)
LedInputs normalInputs()
{
  LedInputs in;
  in.behavior_state = 0;
  return in;
}

// 判定のテスト用に、状態ごとに表示がすべて違う表を作る。
// 既定の表は同じ表示にまとめた状態があり、間違った方を選んでも通ってしまうため
LedCommandTable distinctTable()
{
  uint8_t n = 0;
  const auto next = [&n]() {
      const core_mode::LedCommand command{
        static_cast<uint8_t>(n % 16U), static_cast<uint8_t>(n / 16U)};
      ++n;
      return command;
    };
  LedCommandTable t;
  for (core_mode::LedCommand * command : {
      &t.microcontroller_timeout, &t.controller_disconnected, &t.destroy, &t.software_emergency,
      &t.turret_error, &t.turret_manual_firing, &t.turret_manual_motor_on, &t.turret_manual,
      &t.turret_auto_firing, &t.turret_enemy_detected, &t.turret_auto_motor_on,
      &t.turret_auto_fire_off, &t.turret_auto_idle,
      &t.chassis_auto_rotating, &t.chassis_manual_rotating, &t.chassis_auto, &t.chassis_manual,
      &t.behavior_manual, &t.behavior_attack, &t.behavior_auto_selected, &t.behavior_auto_waypoint,
      &t.behavior_auto_idle, &t.behavior_unknown})
  {
    *command = next();
  }
  return t;
}

const LedCommandTable kTable = distinctTable();
const LedCommandTable kDefaults{};

// 同じ組の中は同じ表示、組どうしは違う表示
void expectGroups(const std::vector<std::vector<core_mode::LedCommand>> & groups)
{
  for (std::size_t g = 0; g < groups.size(); ++g) {
    for (const auto & command : groups[g]) {
      EXPECT_EQ(command, groups[g].front()) << "group " << g;
    }
    for (std::size_t h = g + 1; h < groups.size(); ++h) {
      EXPECT_FALSE(groups[g].front() == groups[h].front()) << "groups " << g << " " << h;
    }
  }
}

void expectAllSegments(const LedCommands & out, const core_mode::LedCommand & expected)
{
  EXPECT_EQ(out.upper_overall, expected);
  EXPECT_EQ(out.upper_left, expected);
  EXPECT_EQ(out.upper_right, expected);
  EXPECT_EQ(out.bottom, expected);
}

TEST(SelectLedCommands, EmergencyReasonsShowSameOnAllSegmentsInPriorityOrder)
{
  LedInputs in = normalInputs();
  in.software_emergency = true;
  in.destroy = true;
  in.controller_disconnected = true;
  in.microcontroller_timeout = true;
  expectAllSegments(selectLedCommands(in, kTable), kTable.microcontroller_timeout);

  in.microcontroller_timeout = false;
  expectAllSegments(selectLedCommands(in, kTable), kTable.controller_disconnected);
  in.controller_disconnected = false;
  expectAllSegments(selectLedCommands(in, kTable), kTable.destroy);
  in.destroy = false;
  expectAllSegments(selectLedCommands(in, kTable), kTable.software_emergency);
}

TEST(SelectLedCommands, StaleBehaviorStateStillShowsTurretAndManualChassis)
{
  LedInputs in = normalInputs();
  in.behavior_state.reset();
  in.turrets[0].error = true;
  LedCommands out = selectLedCommands(in, kTable);
  EXPECT_EQ(out.upper_left, kTable.turret_error);
  EXPECT_EQ(out.upper_right, kTable.turret_manual);
  EXPECT_EQ(out.upper_overall, kTable.behavior_unknown);
  EXPECT_EQ(out.bottom, kTable.chassis_manual);

  in.rotating = true;
  out = selectLedCommands(in, kTable);
  EXPECT_EQ(out.bottom, kTable.chassis_manual_rotating);
}

TEST(SelectLedCommands, TurretPriority)
{
  LedInputs in = normalInputs();
  auto & turret = in.turrets[0];

  turret.manual_mode = true;
  EXPECT_EQ(selectLedCommands(in, kTable).upper_left, kTable.turret_manual);
  turret.manual_firing = true;
  EXPECT_EQ(selectLedCommands(in, kTable).upper_left, kTable.turret_manual_firing);

  turret.manual_mode = false;
  turret.manual_firing = false;
  in.turrets[1].manual_mode = false;
  EXPECT_EQ(selectLedCommands(in, kTable).upper_left, kTable.turret_auto_idle);
  turret.aim = AimStatus::kNoTarget;
  EXPECT_EQ(selectLedCommands(in, kTable).upper_left, kTable.turret_auto_idle);
  turret.aim = AimStatus::kOtherBlocked;
  EXPECT_EQ(selectLedCommands(in, kTable).upper_left, kTable.turret_auto_idle);
  turret.aim = AimStatus::kAutoFireOff;
  EXPECT_EQ(selectLedCommands(in, kTable).upper_left, kTable.turret_auto_fire_off);
  turret.aim = AimStatus::kTargetInView;
  EXPECT_EQ(selectLedCommands(in, kTable).upper_left, kTable.turret_enemy_detected);
  turret.aim = AimStatus::kFiring;
  EXPECT_EQ(selectLedCommands(in, kTable).upper_left, kTable.turret_auto_firing);
  turret.error = true;
  EXPECT_EQ(selectLedCommands(in, kTable).upper_left, kTable.turret_error);
}

TEST(SelectLedCommands, ManualTurretShootMotor)
{
  LedInputs in = normalInputs();
  auto & turret = in.turrets[0];
  EXPECT_EQ(selectLedCommands(in, kTable).upper_left, kTable.turret_manual);
  turret.shoot_motor_on = true;
  EXPECT_EQ(selectLedCommands(in, kTable).upper_left, kTable.turret_manual_motor_on);
  turret.manual_firing = true;
  EXPECT_EQ(selectLedCommands(in, kTable).upper_left, kTable.turret_manual_firing);
  turret.error = true;
  EXPECT_EQ(selectLedCommands(in, kTable).upper_left, kTable.turret_error);
}

TEST(SelectLedCommands, AutoTurretShootMotor)
{
  LedInputs in = normalInputs();
  auto & turret = in.turrets[0];
  turret.manual_mode = false;
  turret.aim = AimStatus::kNoTarget;
  turret.shoot_motor_on = true;  // 自動でもモータは UI で回す
  EXPECT_EQ(selectLedCommands(in, kTable).upper_left, kTable.turret_auto_motor_on);
  // 自動射撃オフは待機の扱いなので、モータの方を出す
  turret.aim = AimStatus::kAutoFireOff;
  EXPECT_EQ(selectLedCommands(in, kTable).upper_left, kTable.turret_auto_motor_on);
  turret.shoot_motor_on = false;
  EXPECT_EQ(selectLedCommands(in, kTable).upper_left, kTable.turret_auto_fire_off);

  turret.shoot_motor_on = true;
  turret.aim = AimStatus::kTargetInView;
  EXPECT_EQ(selectLedCommands(in, kTable).upper_left, kTable.turret_enemy_detected);
  turret.aim = AimStatus::kFiring;
  EXPECT_EQ(selectLedCommands(in, kTable).upper_left, kTable.turret_auto_firing);
}

TEST(LedCommandTable, DefaultTurretCommandsGroupAndFitInOneByte)
{
  // 自動射撃オフは待機と同じ。全体の異常のうち砲塔の区間にも出るものとも違う
  expectGroups({
      {kDefaults.turret_error},
      {kDefaults.turret_manual_firing},
      {kDefaults.turret_manual_motor_on},
      {kDefaults.turret_manual},
      {kDefaults.turret_auto_firing},
      {kDefaults.turret_enemy_detected},
      {kDefaults.turret_auto_motor_on},
      {kDefaults.turret_auto_idle, kDefaults.turret_auto_fire_off},
      {kDefaults.microcontroller_timeout},
      {kDefaults.controller_disconnected},
      {kDefaults.destroy},
      {kDefaults.software_emergency},
  });
  for (const auto & command : {
      kDefaults.turret_error, kDefaults.turret_manual_firing, kDefaults.turret_manual_motor_on,
      kDefaults.turret_manual, kDefaults.turret_auto_firing, kDefaults.turret_enemy_detected,
      kDefaults.turret_auto_motor_on, kDefaults.turret_auto_fire_off, kDefaults.turret_auto_idle,
      kDefaults.microcontroller_timeout, kDefaults.controller_disconnected, kDefaults.destroy,
      kDefaults.software_emergency})
  {
    EXPECT_LE(command.pattern, 15);
    EXPECT_LE(command.color, 7);
  }
}

TEST(LedCommandTable, DefaultManualAndAutoDifferOnlyInColor)
{
  const std::array<std::pair<core_mode::LedCommand, core_mode::LedCommand>, 4> pairs{{
    {kDefaults.turret_manual_firing, kDefaults.turret_auto_firing},
    {kDefaults.turret_manual_motor_on, kDefaults.turret_auto_motor_on},
    {kDefaults.turret_manual, kDefaults.turret_auto_idle},
    {kDefaults.chassis_manual, kDefaults.chassis_auto},
  }};
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    const auto & [manual, automatic] = pairs[i];
    EXPECT_EQ(manual.pattern, automatic.pattern) << i;
    EXPECT_EQ(manual.color, core_mode::led_color::kWhite) << i;
    EXPECT_EQ(automatic.color, core_mode::led_color::kYellow) << i;
  }
}

TEST(LedCommandTable, DefaultRedIsOnlyForErrors)
{
  for (const auto & command : {
      kDefaults.turret_manual_firing, kDefaults.turret_manual_motor_on, kDefaults.turret_manual,
      kDefaults.turret_auto_firing, kDefaults.turret_enemy_detected,
      kDefaults.turret_auto_motor_on, kDefaults.turret_auto_fire_off, kDefaults.turret_auto_idle,
      kDefaults.chassis_auto_rotating, kDefaults.chassis_manual_rotating, kDefaults.chassis_auto,
      kDefaults.chassis_manual, kDefaults.behavior_manual, kDefaults.behavior_attack,
      kDefaults.behavior_auto_selected, kDefaults.behavior_auto_waypoint,
      kDefaults.behavior_auto_idle, kDefaults.behavior_unknown})
  {
    EXPECT_NE(command.color, core_mode::led_color::kRed);
  }
  for (const auto & command : {
      kDefaults.turret_error, kDefaults.microcontroller_timeout,
      kDefaults.controller_disconnected, kDefaults.destroy, kDefaults.software_emergency})
  {
    EXPECT_EQ(command.color, core_mode::led_color::kRed);
  }
}

TEST(SelectLedCommands, ManualTurretIgnoresAutoInputs)
{
  LedInputs in = normalInputs();
  in.turrets[0].aim = AimStatus::kFiring;
  in.turrets[1].aim = AimStatus::kTargetInView;
  const LedCommands out = selectLedCommands(in, kTable);
  EXPECT_EQ(out.upper_left, kTable.turret_manual);
  EXPECT_EQ(out.upper_right, kTable.turret_manual);
}

TEST(SelectLedCommands, EachTurretHasItsOwnSegment)
{
  LedInputs in = normalInputs();
  in.turrets[0].manual_mode = false;  // left: auto、敵なし
  in.turrets[1].manual_firing = true;  // right: 手動で射撃中
  LedCommands out = selectLedCommands(in, kTable);
  EXPECT_EQ(out.upper_left, kTable.turret_auto_idle);
  EXPECT_EQ(out.upper_right, kTable.turret_manual_firing);

  in.turrets[0].aim = AimStatus::kFiring;  // left: auto で射撃中
  in.turrets[1].error = true;  // right: 異常
  out = selectLedCommands(in, kTable);
  EXPECT_EQ(out.upper_left, kTable.turret_auto_firing);
  EXPECT_EQ(out.upper_right, kTable.turret_error);
}

TEST(SelectLedCommands, EnemyDetectionIsPerTurret)
{
  LedInputs in = normalInputs();
  in.turrets[0].manual_mode = false;
  in.turrets[1].manual_mode = false;
  in.turrets[0].aim = AimStatus::kTargetInView;  // left だけ敵を捉えている
  in.turrets[1].aim = AimStatus::kNoTarget;
  const LedCommands out = selectLedCommands(in, kTable);
  EXPECT_EQ(out.upper_left, kTable.turret_enemy_detected);
  EXPECT_EQ(out.upper_right, kTable.turret_auto_idle);
}

TEST(ClassifyAimStatus, MatchesAimBotStrings)
{
  EXPECT_EQ(classifyAimStatus("firing"), AimStatus::kFiring);
  EXPECT_EQ(classifyAimStatus("blocked: turret_auto off"), AimStatus::kAutoFireOff);
  EXPECT_EQ(classifyAimStatus("blocked: no target"), AimStatus::kNoTarget);
  EXPECT_EQ(
    classifyAimStatus("blocked: out of range: 9.10m >= max_shoot_range 8.00m"),
    AimStatus::kTargetInView);
  EXPECT_EQ(
    classifyAimStatus(
      "blocked: not aimed: miss=(x=0.100, y=0.000, z=0.000)m tol=(0.050, 0.050, 0.050)m"),
    AimStatus::kTargetInView);
  EXPECT_EQ(
    classifyAimStatus("blocked: off center: err=(12.0, 3.0)px tol=(5.0, 5.0)px"),
    AimStatus::kTargetInView);
  EXPECT_EQ(classifyAimStatus("blocked: manual mode"), AimStatus::kOtherBlocked);
  EXPECT_EQ(classifyAimStatus("blocked: test mode"), AimStatus::kOtherBlocked);
  EXPECT_EQ(classifyAimStatus("blocked: emergency stop"), AimStatus::kOtherBlocked);
  EXPECT_EQ(classifyAimStatus("blocked: starting up"), AimStatus::kOtherBlocked);
  EXPECT_EQ(classifyAimStatus(""), AimStatus::kUnknown);
  EXPECT_EQ(classifyAimStatus("firing now"), AimStatus::kUnknown);
}

TEST(SelectLedCommands, OverallSegmentByBehaviorState)
{
  LedInputs in = normalInputs();
  const std::array<std::pair<int32_t, core_mode::LedCommand>, 7> cases{{
    {0, kTable.behavior_manual},
    {1, kTable.behavior_attack},
    {2, kTable.behavior_auto_selected},
    {3, kTable.behavior_auto_waypoint},
    {4, kTable.behavior_auto_idle},
    {5, kTable.behavior_unknown},
    {-1, kTable.behavior_unknown},
  }};
  for (const auto & [state, expected] : cases) {
    in.behavior_state = state;
    EXPECT_EQ(selectLedCommands(in, kTable).upper_overall, expected) << state;
  }
  in.behavior_state.reset();
  EXPECT_EQ(selectLedCommands(in, kTable).upper_overall, kTable.behavior_unknown);
}

TEST(LedCommandTable, DefaultBehaviorCommandsGroupAutoStates)
{
  expectGroups({
      {kDefaults.behavior_manual},
      {kDefaults.behavior_attack},
      {kDefaults.behavior_auto_selected, kDefaults.behavior_auto_waypoint,
        kDefaults.behavior_auto_idle},
      {kDefaults.behavior_unknown},
  });
}

TEST(SelectLedCommands, ChassisByModeAndRotation)
{
  LedInputs in = normalInputs();
  EXPECT_EQ(selectLedCommands(in, kTable).bottom, kTable.chassis_manual);
  in.rotating = true;
  EXPECT_EQ(selectLedCommands(in, kTable).bottom, kTable.chassis_manual_rotating);

  for (int32_t state = 1; state <= 4; ++state) {
    in.behavior_state = state;
    in.rotating = false;
    EXPECT_EQ(selectLedCommands(in, kTable).bottom, kTable.chassis_auto) << state;
    in.rotating = true;
    EXPECT_EQ(selectLedCommands(in, kTable).bottom, kTable.chassis_auto_rotating) << state;
  }
}

}  // namespace
