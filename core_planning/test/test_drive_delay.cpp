#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "core_planning/drive_delay.hpp"
#include "core_planning/mppi.hpp"

namespace
{
using core_planning::DriveParams;
using core_planning::driveParamsError;
using core_planning::lagGain;
using core_planning::MppiController;
using core_planning::MppiParams;
using core_planning::MppiState;
using core_planning::Point2;
using core_planning::predictDelayedState;
using core_planning::pruneSentCommands;
using core_planning::ReferencePath;
using core_planning::SE2;
using core_planning::SentCommand;

// 加速度の上限が効かないほど大きい駆動
DriveParams quickDrive(const double dead_time_s, const double time_constant_s)
{
  DriveParams params;
  params.predict_delay = true;
  params.dead_time_s = dead_time_s;
  params.time_constant_s = time_constant_s;
  params.max_accel_mps2 = 1e6;
  params.max_angular_accel_radps2 = 1e6;
  return params;
}

TEST(DriveDelay, LagGainFollowsFirstOrderResponse)
{
  EXPECT_DOUBLE_EQ(lagGain(0.1, 0.0), 1.0);
  EXPECT_NEAR(lagGain(0.1, 0.05), 1.0 - std::exp(-2.0), 1e-12);
  EXPECT_DOUBLE_EQ(lagGain(0.0, 0.05), 0.0);
}

TEST(DriveDelay, RejectsNegativeDelayAndNonPositiveAccel)
{
  DriveParams params;
  EXPECT_FALSE(driveParamsError(params).has_value());
  params.dead_time_s = -0.01;
  EXPECT_TRUE(driveParamsError(params).has_value());
  params = DriveParams{};
  params.max_accel_mps2 = 0.0;
  EXPECT_TRUE(driveParamsError(params).has_value());
}

TEST(DriveDelay, KeepsVelocityWithoutCommands)
{
  MppiState measured;
  measured.vx = 2.0;
  measured.vy = -1.0;
  measured.wz = 0.5;
  const MppiState predicted = predictDelayedState(measured, 1.0, 1.1, {}, quickDrive(0.05, 0.0));
  EXPECT_NEAR(predicted.x, 0.2, 1e-9);
  EXPECT_NEAR(predicted.y, -0.1, 1e-9);
  EXPECT_NEAR(predicted.yaw, 0.05, 1e-9);
  EXPECT_DOUBLE_EQ(predicted.vx, 2.0);
}

TEST(DriveDelay, ReturnsMeasuredStateWhenTargetIsNotLater)
{
  MppiState measured;
  measured.x = 1.0;
  measured.vx = 1.0;
  const std::vector<SentCommand> sent{{0.0, 3.0, 0.0, 0.0}};
  const MppiState predicted = predictDelayedState(measured, 1.0, 1.0, sent, quickDrive(0.0, 0.0));
  EXPECT_DOUBLE_EQ(predicted.x, 1.0);
  EXPECT_DOUBLE_EQ(predicted.vx, 1.0);
}

TEST(DriveDelay, CommandTakesEffectAfterDeadTime)
{
  const MppiState rest;
  const std::vector<SentCommand> sent{{0.0, 1.0, 0.0, 0.0}};
  const DriveParams drive = quickDrive(0.05, 0.0);
  // 無駄時間の間は止まったまま
  const MppiState early = predictDelayedState(rest, 0.0, 0.04, sent, drive);
  EXPECT_DOUBLE_EQ(early.vx, 0.0);
  EXPECT_DOUBLE_EQ(early.x, 0.0);
  // 効き始めてからの 0.05 秒だけ進む
  const MppiState later = predictDelayedState(rest, 0.0, 0.1, sent, drive);
  EXPECT_NEAR(later.vx, 1.0, 1e-9);
  EXPECT_NEAR(later.x, 0.05, 1e-6);
}

TEST(DriveDelay, UsesCommandAlreadyInEffectAtMeasurement)
{
  MppiState measured;
  measured.vx = 0.5;
  // 0.2 秒に出した指令は 0.25 秒から効いている。0.35 秒に出した指令は 0.4 秒から効く
  const std::vector<SentCommand> sent{{0.1, 0.0, 0.0, 0.0}, {0.2, 2.0, 0.0, 0.0}, {0.35, 1.0, 0.0, 0.0}};
  const MppiState predicted = predictDelayedState(measured, 0.3, 0.45, sent, quickDrive(0.05, 0.0));
  // 0.3〜0.4 秒は 2.0m/s、0.4〜0.45 秒は 1.0m/s
  EXPECT_NEAR(predicted.vx, 1.0, 1e-9);
  EXPECT_NEAR(predicted.x, 0.2 + 0.05, 1e-6);
}

TEST(DriveDelay, FollowsFirstOrderLagAndAccelLimit)
{
  const MppiState rest;
  const std::vector<SentCommand> sent{{0.0, 1.0, 0.0, 0.0}};
  const MppiState lagged = predictDelayedState(rest, 0.0, 0.1, sent, quickDrive(0.0, 0.1));
  EXPECT_NEAR(lagged.vx, 1.0 - std::exp(-1.0), 1e-9);

  DriveParams limited = quickDrive(0.0, 0.0);
  limited.max_accel_mps2 = 5.0;
  const MppiState capped = predictDelayedState(rest, 0.0, 0.1, sent, limited);
  EXPECT_NEAR(capped.vx, 0.5, 1e-9);
}

TEST(DriveDelay, RotatesBodyCommandWithHeading)
{
  MppiState measured;
  measured.yaw = M_PI / 2.0;
  const std::vector<SentCommand> sent{{0.0, 1.0, 0.0, 0.0}};
  const MppiState predicted = predictDelayedState(measured, 0.0, 0.1, sent, quickDrive(0.0, 0.0));
  EXPECT_NEAR(predicted.vx, 0.0, 1e-9);
  EXPECT_NEAR(predicted.vy, 1.0, 1e-9);
  EXPECT_NEAR(predicted.y, 0.1, 1e-9);
}

TEST(DriveDelay, PruneKeepsCommandInEffect)
{
  std::vector<SentCommand> sent{{0.0, 1.0, 0.0, 0.0}, {0.1, 2.0, 0.0, 0.0}, {0.2, 3.0, 0.0, 0.0},
    {0.3, 4.0, 0.0, 0.0}};
  // 0.27 秒には 0.2 秒の指令（0.25 秒から）が効いている
  pruneSentCommands(sent, 0.27, 0.05);
  ASSERT_EQ(sent.size(), 2U);
  EXPECT_DOUBLE_EQ(sent.front().vx, 3.0);
  // 何も効いていない時刻なら捨てない
  std::vector<SentCommand> future{{1.0, 1.0, 0.0, 0.0}};
  pruneSentCommands(future, 0.5, 0.05);
  EXPECT_EQ(future.size(), 1U);
}

// 予測の軌跡の 1 ステップ目は、出した指令に駆動の一次遅れをかけた分だけ進む
TEST(DriveDelay, PlanFirstStepMatchesLaggedCommand)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{10.0, 0.0}});
  for (const double time_constant : {0.0, 0.08}) {
    MppiParams params;
    params.drive_time_constant_s = time_constant;
    MppiController controller(3U);
    MppiState state;
    state.vx = 0.5;
    const auto command = controller.solve(state, path, {}, nullptr, SE2{}, params, 3.0);
    const auto & trajectory = controller.lastTrajectory();
    ASSERT_GE(trajectory.size(), 2U);
    const double gain = lagGain(params.dt, time_constant);
    EXPECT_NEAR(trajectory[1].vx - state.vx, gain * (command.vx - state.vx), 1e-9);
    EXPECT_NEAR(trajectory[1].vy - state.vy, gain * (command.vy - state.vy), 1e-9);
  }
}
}  // namespace
