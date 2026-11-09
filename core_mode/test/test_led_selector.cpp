#include <gtest/gtest.h>

#include "core_mode/led_selector.hpp"

namespace
{

using core_mode::LedCommandPair;
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

const LedCommandTable kTable{};

TEST(SelectLedCommands, EmergencyReasonsShowSameOnBothTapesInPriorityOrder)
{
  LedInputs in = normalInputs();
  in.software_emergency = true;
  in.destroy = true;
  in.controller_disconnected = true;
  in.microcontroller_timeout = true;
  LedCommandPair out = selectLedCommands(in, kTable);
  EXPECT_EQ(out.upper, kTable.microcontroller_timeout);
  EXPECT_EQ(out.bottom, kTable.microcontroller_timeout);

  in.microcontroller_timeout = false;
  EXPECT_EQ(selectLedCommands(in, kTable).upper, kTable.controller_disconnected);
  in.controller_disconnected = false;
  EXPECT_EQ(selectLedCommands(in, kTable).upper, kTable.destroy);
  in.destroy = false;
  out = selectLedCommands(in, kTable);
  EXPECT_EQ(out.upper, kTable.software_emergency);
  EXPECT_EQ(out.bottom, kTable.software_emergency);
}

TEST(SelectLedCommands, StaleBehaviorStateStillShowsTurretAndManualChassis)
{
  LedInputs in = normalInputs();
  in.behavior_state.reset();
  in.turrets[0].error = true;
  LedCommandPair out = selectLedCommands(in, kTable);
  EXPECT_EQ(out.upper, kTable.turret_error);
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
  EXPECT_EQ(selectLedCommands(in, kTable).upper, kTable.turret_manual);
  turret.manual_firing = true;
  EXPECT_EQ(selectLedCommands(in, kTable).upper, kTable.turret_manual_firing);

  turret.manual_mode = false;
  turret.manual_firing = false;
  in.turrets[1].manual_mode = false;
  EXPECT_EQ(selectLedCommands(in, kTable).upper, kTable.turret_auto_idle);
  in.enemy_detected = true;
  EXPECT_EQ(selectLedCommands(in, kTable).upper, kTable.turret_enemy_detected);
  turret.auto_firing = true;
  EXPECT_EQ(selectLedCommands(in, kTable).upper, kTable.turret_auto_firing);
  turret.error = true;
  EXPECT_EQ(selectLedCommands(in, kTable).upper, kTable.turret_error);
}

TEST(SelectLedCommands, ManualTurretIgnoresAutoInputs)
{
  LedInputs in = normalInputs();
  in.enemy_detected = true;
  in.turrets[0].auto_firing = true;
  in.turrets[1].auto_firing = true;
  EXPECT_EQ(selectLedCommands(in, kTable).upper, kTable.turret_manual);
}

TEST(SelectLedCommands, HigherPriorityTurretWins)
{
  LedInputs in = normalInputs();
  in.turrets[0].manual_mode = false;  // left: auto、敵なし
  in.turrets[1].manual_firing = true;  // right: 手動で射撃中
  EXPECT_EQ(selectLedCommands(in, kTable).upper, kTable.turret_manual_firing);

  in.turrets[0].auto_firing = true;  // left: auto で射撃中
  EXPECT_EQ(selectLedCommands(in, kTable).upper, kTable.turret_auto_firing);
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
