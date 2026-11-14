#include <gtest/gtest.h>

#include "core_gazebo/command_delay.hpp"

namespace
{
using core_gazebo::DelayedCommand;

TEST(CommandDelay, ZeroDeadTimeAppliesInSameStep)
{
  // 既定は前の動き: 受けたステップから効く
  DelayedCommand<double> delayed;
  EXPECT_DOUBLE_EQ(delayed.update(0.0), 0.0);
  delayed.push(1.0, 2.0);
  EXPECT_DOUBLE_EQ(delayed.update(1.0), 2.0);
  EXPECT_EQ(delayed.pending(), 0U);
}

TEST(CommandDelay, WaitsForDeadTimeAndKeepsPreviousCommand)
{
  DelayedCommand<double> delayed(0.05);
  delayed.push(0.0, 1.0);
  EXPECT_DOUBLE_EQ(delayed.update(0.049), 0.0);
  EXPECT_DOUBLE_EQ(delayed.update(0.05), 1.0);
  // 次の指令が効くまでは前の指令のまま
  delayed.push(0.06, 2.0);
  EXPECT_DOUBLE_EQ(delayed.update(0.1), 1.0);
  EXPECT_DOUBLE_EQ(delayed.update(0.11), 2.0);
}

TEST(CommandDelay, HandlesDeadTimeLongerThanCommandPeriod)
{
  // 制御の周期（0.1 秒）より長い無駄時間でも、指令を順に効かせる
  DelayedCommand<double> delayed(0.25);
  for (int i = 0; i < 5; ++i) {
    delayed.push(0.1 * i, static_cast<double>(i + 1));
  }
  EXPECT_EQ(delayed.pending(), 5U);
  EXPECT_DOUBLE_EQ(delayed.update(0.2), 0.0);
  EXPECT_DOUBLE_EQ(delayed.update(0.25), 1.0);
  EXPECT_DOUBLE_EQ(delayed.update(0.36), 2.0);
  // 効き始めた指令がいくつあっても、最後のものを使う
  EXPECT_DOUBLE_EQ(delayed.update(0.7), 5.0);
  EXPECT_EQ(delayed.pending(), 0U);
}

TEST(CommandDelay, NegativeDeadTimeIsZero)
{
  DelayedCommand<double> delayed(-1.0);
  EXPECT_DOUBLE_EQ(delayed.deadTime(), 0.0);
}
}  // namespace
