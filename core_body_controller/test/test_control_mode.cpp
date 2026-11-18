#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "core_body_controller/control_mode.hpp"

using core_body_controller::CommandSource;
using core_body_controller::ControlMode;
using core_body_controller::ControlModeLogic;
using core_body_controller::NavigationPhase;
using core_body_controller::routeError;

TEST(ControlModeLogic, StartsManualAndPassesOnlyManual)
{
  const ControlModeLogic logic;
  EXPECT_EQ(logic.mode(), ControlMode::kManual);
  EXPECT_TRUE(logic.passes(CommandSource::kManual));
  EXPECT_FALSE(logic.passes(CommandSource::kAuto));
}

TEST(ControlModeLogic, AutoPassesOnlyAuto)
{
  ControlModeLogic logic;
  logic.start();
  EXPECT_EQ(logic.mode(), ControlMode::kAuto);
  EXPECT_TRUE(logic.passes(CommandSource::kAuto));
  EXPECT_FALSE(logic.passes(CommandSource::kManual));
}

TEST(ControlModeLogic, ReachedAndStopReturnToManual)
{
  ControlModeLogic logic;
  logic.start();
  logic.reached();
  EXPECT_EQ(logic.mode(), ControlMode::kManual);
  logic.start();
  logic.stop();
  EXPECT_EQ(logic.mode(), ControlMode::kManual);
}

TEST(ControlModeLogic, IgnoresStaleFailureRightAfterStart)
{
  ControlModeLogic logic;
  logic.start();
  // 前のゴールの失敗が、新しいゴールの計画より先に届く
  logic.navigation(NavigationPhase::kFailed);
  EXPECT_EQ(logic.mode(), ControlMode::kAuto);
  logic.navigation(NavigationPhase::kActive);
  logic.navigation(NavigationPhase::kFailed);
  EXPECT_EQ(logic.mode(), ControlMode::kManual);
}

TEST(ControlModeLogic, RestartForgetsPreviousRun)
{
  ControlModeLogic logic;
  logic.start();
  logic.navigation(NavigationPhase::kActive);
  logic.reached();
  logic.start();
  logic.navigation(NavigationPhase::kFailed);
  EXPECT_EQ(logic.mode(), ControlMode::kAuto);
}

TEST(ControlModeLogic, NavigationDoesNotEnterAuto)
{
  ControlModeLogic logic;
  logic.navigation(NavigationPhase::kActive);
  EXPECT_EQ(logic.mode(), ControlMode::kManual);
}

TEST(ControlModeLogic, EmergencyReturnsToManualAndBlocksStart)
{
  ControlModeLogic logic;
  logic.start();
  logic.emergency(true);
  EXPECT_EQ(logic.mode(), ControlMode::kManual);
  EXPECT_TRUE(logic.passes(CommandSource::kManual));
  // 緊急停止の間は auto を受け付けない
  EXPECT_FALSE(logic.start());
  EXPECT_EQ(logic.mode(), ControlMode::kManual);
  // 解除しても自動には戻らない。もう一度 auto を押せば入る
  logic.emergency(false);
  EXPECT_EQ(logic.mode(), ControlMode::kManual);
  EXPECT_TRUE(logic.start());
  EXPECT_EQ(logic.mode(), ControlMode::kAuto);
}

TEST(RouteError, AcceptsFiniteRouteWithinLimit)
{
  EXPECT_FALSE(routeError({{1.0, 2.0}}, 3).has_value());
  EXPECT_FALSE(routeError({{1.0, 2.0}, {3.0, -1.0}, {0.0, 0.0}}, 3).has_value());
}

TEST(RouteError, RejectsEmptyTooLongAndNonFinite)
{
  EXPECT_TRUE(routeError({}, 3).has_value());
  EXPECT_TRUE(routeError({{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}, {3.0, 3.0}}, 3).has_value());
  const auto error = routeError({{0.0, 0.0}, {std::nan(""), 1.0}}, 3);
  ASSERT_TRUE(error.has_value());
  EXPECT_NE(error->find("2 番目"), std::string::npos);
}
