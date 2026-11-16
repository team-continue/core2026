#include <gtest/gtest.h>

#include <cmath>

#include "core_body_controller/target_angle_controller.hpp"

namespace
{

TEST(TargetAngleControllerTest, QuaternionIsNormalizedBeforeExtractingYaw)
{
  double yaw = 0.0;
  constexpr double HALF_YAW = M_PI / 4.0;

  EXPECT_TRUE(
    core_body_controller::quaternion_to_yaw(
      0.0, 0.0, 2.0 * std::sin(HALF_YAW), 2.0 * std::cos(HALF_YAW), yaw));
  EXPECT_NEAR(yaw, M_PI / 2.0, 1e-12);
}

TEST(TargetAngleControllerTest, InvalidQuaternionsAreRejected)
{
  double yaw = 0.0;

  EXPECT_FALSE(core_body_controller::quaternion_to_yaw(0.0, 0.0, 0.0, 0.0, yaw));
  EXPECT_FALSE(core_body_controller::quaternion_to_yaw(0.0, 0.0, std::nan(""), 1.0, yaw));
  EXPECT_FALSE(core_body_controller::quaternion_to_yaw(0.0, 0.0, 0.0, 1.0e7, yaw));
}

TEST(TargetAngleControllerTest, FirstYawSampleDefinesZero)
{
  core_body_controller::YawTracker tracker;

  EXPECT_TRUE(tracker.update(1.2, false));
  EXPECT_DOUBLE_EQ(tracker.yaw(), 0.0);
  EXPECT_TRUE(tracker.update(1.4, true));
  EXPECT_NEAR(tracker.yaw(), 0.2, 1e-12);
}

TEST(TargetAngleControllerTest, YawIsUnwrappedAcrossPiBoundary)
{
  core_body_controller::YawTracker tracker;
  const double first = 179.0 * M_PI / 180.0;
  const double second = -179.0 * M_PI / 180.0;

  tracker.update(first, false);
  tracker.update(second, true);

  EXPECT_NEAR(tracker.yaw(), 2.0 * M_PI / 180.0, 1e-12);
}

TEST(TargetAngleControllerTest, ResumeSampleRebasesWithoutChangingTrackedYaw)
{
  core_body_controller::YawTracker tracker;

  tracker.update(0.2, false);
  tracker.update(0.7, true);
  tracker.update(-2.5, false);

  EXPECT_NEAR(tracker.yaw(), 0.5, 1e-12);
  tracker.update(-2.4, true);
  EXPECT_NEAR(tracker.yaw(), 0.6, 1e-12);
}

TEST(TargetAngleControllerTest, NonFiniteYawDoesNotChangeState)
{
  core_body_controller::YawTracker tracker;

  tracker.update(0.0, false);
  EXPECT_FALSE(tracker.update(std::nan(""), true));
  EXPECT_DOUBLE_EQ(tracker.yaw(), 0.0);
}

TEST(TargetAngleControllerTest, FinalOutputHonorsVelocityAndAccelerationLimits)
{
  core_body_controller::TargetAngleController controller(2.0, 0.0, 0.0, 1.0, 2.0, 0.0);

  EXPECT_NEAR(controller.update(-10.0, 100.0, 0.1), 0.2, 1e-12);
  EXPECT_NEAR(controller.update(-10.0, 100.0, 0.1), 0.4, 1e-12);
  for (int i = 0; i < 10; ++i) {
    controller.update(-10.0, 100.0, 0.1);
  }
  EXPECT_LE(std::abs(controller.output()), 1.0);
}

TEST(TargetAngleControllerTest, FeedforwardCanBeEnabledWithoutBypassingLimits)
{
  core_body_controller::TargetAngleController controller(2.0, 0.0, 0.0, 1.0, 1000.0, 0.0);

  EXPECT_DOUBLE_EQ(controller.update(0.0, 0.0, 0.01), 0.0);
  EXPECT_NEAR(controller.update(0.0, 0.5, 0.01), 0.5, 1e-12);
  EXPECT_NEAR(controller.update(0.0, 5.0, 0.01), 1.0, 1e-12);
}

TEST(TargetAngleControllerTest, DeadbandDoesNotBypassAccelerationLimit)
{
  constexpr double acceleration_limit = 2.0;
  constexpr double dt = 0.01;
  core_body_controller::TargetAngleController controller(
    0.0, 0.0, 0.0, 1.0, acceleration_limit, 0.1);

  EXPECT_DOUBLE_EQ(controller.update(0.0, 0.05, dt), 0.0);

  double previous = 0.0;
  for (int i = 0; i < 10; ++i) {
    const double command = controller.update(0.0, 1.0, dt);
    EXPECT_LE(std::abs(command - previous), acceleration_limit * dt + 1e-12);
    previous = command;
  }

  for (int i = 0; i < 10; ++i) {
    const double command = controller.update(0.0, 0.0, dt);
    EXPECT_LE(std::abs(command - previous), acceleration_limit * dt + 1e-12);
    previous = command;
  }
}

TEST(TargetAngleControllerTest, YawModeAcceptsOnlyKnownValues)
{
  EXPECT_EQ(core_body_controller::to_yaw_mode(0), core_body_controller::YawMode::kFront);
  EXPECT_EQ(core_body_controller::to_yaw_mode(1), core_body_controller::YawMode::kTurret);
  EXPECT_FALSE(core_body_controller::to_yaw_mode(2).has_value());
  EXPECT_FALSE(core_body_controller::to_yaw_mode(-1).has_value());
}

TEST(TargetAngleControllerTest, FrontModeTurnsTheTargetWithCmdVelOnly)
{
  using core_body_controller::YawMode;
  // FRONT: cmd_vel の angular.z で回り、yaw_target_rate は無視する
  EXPECT_NEAR(
    core_body_controller::next_world_target(0.0, YawMode::kFront, 1.0, 2.0, 0.1), 0.1, 1e-12);
  // cmd_vel が途絶えていれば目標を保つ
  EXPECT_NEAR(
    core_body_controller::next_world_target(0.3, YawMode::kFront, std::nullopt, 2.0, 0.1), 0.3,
    1e-12);
}

TEST(TargetAngleControllerTest, TurretModeTurnsTheTargetWithTargetRateOnly)
{
  using core_body_controller::YawMode;
  // TURRET: 車体の旋回 (cmd_vel の angular.z) では回らず、yaw_target_rate で回る
  EXPECT_NEAR(
    core_body_controller::next_world_target(0.0, YawMode::kTurret, 1.0, -2.0, 0.1), -0.2, 1e-12);
  EXPECT_NEAR(
    core_body_controller::next_world_target(0.5, YawMode::kTurret, 1.0, std::nullopt, 0.1), 0.5,
    1e-12);
}

TEST(TargetAngleControllerTest, TargetUpdateWrapsAndRejectsInvalidInput)
{
  using core_body_controller::YawMode;
  EXPECT_NEAR(
    core_body_controller::next_world_target(M_PI - 0.05, YawMode::kTurret, std::nullopt, 1.0, 0.1),
    -M_PI + 0.05, 1e-12);
  EXPECT_NEAR(
    core_body_controller::next_world_target(0.2, YawMode::kTurret, std::nullopt, std::nan(""), 0.1),
    0.2, 1e-12);
  EXPECT_NEAR(
    core_body_controller::next_world_target(0.2, YawMode::kFront, 1.0, std::nullopt, 0.0), 0.2,
    1e-12);
}

TEST(TargetAngleControllerTest, StepTowardAngleMovesSlowlyAlongTheShorterWay)
{
  // 最大 0.1 rad ずつ近づく
  EXPECT_NEAR(core_body_controller::step_toward_angle(0.0, 1.0, 0.1), 0.1, 1e-12);
  EXPECT_NEAR(core_body_controller::step_toward_angle(0.0, -1.0, 0.1), -0.1, 1e-12);
  // 残りが小さければ goal に着く
  EXPECT_NEAR(core_body_controller::step_toward_angle(0.95, 1.0, 0.1), 1.0, 1e-12);
  // -π/π をまたぐときは近いほうへ回る
  EXPECT_NEAR(
    core_body_controller::step_toward_angle(M_PI - 0.05, -M_PI + 0.05, 0.2), -M_PI + 0.05,
    1e-12);
  EXPECT_NEAR(
    core_body_controller::step_toward_angle(M_PI - 0.05, -M_PI + 0.5, 0.1), -M_PI + 0.05,
    1e-12);
  // 不正な速さでは動かない
  EXPECT_NEAR(core_body_controller::step_toward_angle(0.3, 1.0, 0.0), 0.3, 1e-12);
}

TEST(TargetAngleControllerTest, FrontTargetAddsTheWaistAngleToTheUpperBodyYaw)
{
  // 上半身が 1.0 rad、腰の角度（車体 − 上半身。body_control_node と同じ決め方）が 0.4 rad なら、
  // 車体の正面は 1.4 rad
  EXPECT_NEAR(core_body_controller::front_world_target(1.0, 0.4, 0.0, 1.0), 1.4, 1e-12);
  // 腰の角度 0 が正面からずれている (オフセット 0.1 rad)
  EXPECT_NEAR(core_body_controller::front_world_target(1.0, 0.4, 0.1, 1.0), 1.3, 1e-12);
  // モータの正の向きがワールドの yaw と逆
  EXPECT_NEAR(core_body_controller::front_world_target(1.0, 0.4, 0.0, -1.0), 0.6, 1e-12);
  // 角度は -π〜π に収める
  EXPECT_NEAR(
    core_body_controller::front_world_target(3.0, 0.5, 0.0, 1.0), -2.7831853071795862,
    1e-12);
}

TEST(TargetAngleControllerTest, FeedforwardTurnsTheUpperBodyAtTheWantedRate)
{
  using core_body_controller::waist_feedforward;
  // FRONT: 無限回転 -3.14 rad/s と操作 0.5 rad/s（車体 -2.64 rad/s）。上半身は操作の 0.5 rad/s で回す
  EXPECT_NEAR(waist_feedforward(-2.64, 0.5), -3.14, 1e-12);
  // 無限回転なしの FRONT は打ち消さない（上半身は車体と一緒に回る）
  EXPECT_NEAR(waist_feedforward(0.5, 0.5), 0.0, 1e-12);
  // TURRET: 車体 -2.64 rad/s の間に、操作で上半身を -0.8 rad/s で回す
  EXPECT_NEAR(waist_feedforward(-2.64, -0.8), -1.84, 1e-12);
  // 回したい速さが届いていなければ、車体の回転を全部打ち消す。車体の回転がなければ 0
  EXPECT_NEAR(waist_feedforward(0.5, std::nullopt), 0.5, 1e-12);
  EXPECT_NEAR(waist_feedforward(std::nullopt, 0.5), 0.0, 1e-12);
}

}  // namespace
