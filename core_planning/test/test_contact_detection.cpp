#include <gtest/gtest.h>

#include "core_planning/contact_detection.hpp"

namespace
{
using core_planning::countsAsImpact;

TEST(ContactDetection, ImpactNeedsAccelAboveThreshold)
{
  EXPECT_FALSE(countsAsImpact(19.9, 20.0, 1.0, 1.0, 0.1));
  EXPECT_FALSE(countsAsImpact(20.0, 20.0, 1.0, 1.0, 0.1));
  EXPECT_TRUE(countsAsImpact(20.1, 20.0, 1.0, 1.0, 0.1));
}

TEST(ContactDetection, ImpactWhileStandingStillIsIgnored)
{
  // 止まっていて指令も 0（行き詰まりの待ちなど）。Gazebo で 28m/s² の振れが出た
  EXPECT_FALSE(countsAsImpact(28.0, 20.0, 0.0, 0.0, 0.1));
  EXPECT_FALSE(countsAsImpact(28.0, 20.0, 0.05, 0.09, 0.1));
}

TEST(ContactDetection, ImpactCountsWhenMovingOrCommanded)
{
  // 走っている（実際の速さ）か、動き出そうとしている（指令の速さ）
  EXPECT_TRUE(countsAsImpact(28.0, 20.0, 1.5, 0.0, 0.1));
  EXPECT_TRUE(countsAsImpact(28.0, 20.0, 0.0, 0.5, 0.1));
}

TEST(ContactDetection, ZeroMinSpeedKeepsPreviousBehavior)
{
  EXPECT_TRUE(countsAsImpact(28.0, 20.0, 0.0, 0.0, 0.0));
}
}  // namespace
