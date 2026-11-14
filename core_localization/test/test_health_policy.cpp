#include <cmath>

#include <gtest/gtest.h>

#include "core_localization/health_policy.hpp"

namespace
{
using core_localization::AlignmentReport;
using core_localization::gyroDifferenceLimit;
using core_localization::HealthPolicyParams;
using core_localization::isGoodMatch;
using core_localization::isJump;
using core_localization::rejectsTooMany;

AlignmentReport accepted(double correction_m = 0.02, double correction_deg = 0.2)
{
  return AlignmentReport{true, 0.1, 0.8, 0, correction_m, correction_deg};
}

AlignmentReport rejected(int consecutive)
{
  return AlignmentReport{true, 0.1, 0.8, consecutive, 0.7, 1.0};
}

TEST(HealthPolicy, DefaultCountsRejectedUpdateAsGoodMatch)
{
  // 今までと同じ: 評価値が良ければ、ガードで捨てた回も成功に数える
  EXPECT_TRUE(isGoodMatch(rejected(3), HealthPolicyParams{}));
}

TEST(HealthPolicy, CanCountRejectedUpdateAsFailure)
{
  HealthPolicyParams params;
  params.count_rejected_as_failure = true;
  EXPECT_FALSE(isGoodMatch(rejected(1), params));
  EXPECT_TRUE(isGoodMatch(accepted(), params));
}

TEST(HealthPolicy, BadQualityIsNeverGoodMatch)
{
  AlignmentReport report = accepted();
  report.fitness = 3.0;
  EXPECT_FALSE(isGoodMatch(report, HealthPolicyParams{}));
  report = accepted();
  report.inlier_ratio = 0.2;
  EXPECT_FALSE(isGoodMatch(report, HealthPolicyParams{}));
  report = accepted();
  report.inlier_ratio = std::nan("");
  EXPECT_TRUE(isGoodMatch(report, HealthPolicyParams{}));
  report.converged = false;
  EXPECT_FALSE(isGoodMatch(report, HealthPolicyParams{}));
}

TEST(HealthPolicy, RejectionCountIsOffByDefault)
{
  EXPECT_FALSE(rejectsTooMany(rejected(50), HealthPolicyParams{}));
  HealthPolicyParams params;
  params.degraded_rejected_updates = 3;
  EXPECT_FALSE(rejectsTooMany(rejected(2), params));
  EXPECT_TRUE(rejectsTooMany(rejected(3), params));
}

TEST(HealthPolicy, JumpIsOnlyAnAcceptedLargeCorrection)
{
  HealthPolicyParams params;
  EXPECT_FALSE(isJump(accepted(1.0, 30.0), params));  // 既定は使わない
  params.jump_translation_m = 0.2;
  params.jump_yaw_deg = 3.0;
  EXPECT_FALSE(isJump(accepted(0.1, 1.0), params));
  EXPECT_TRUE(isJump(accepted(0.25, 0.0), params));
  EXPECT_TRUE(isJump(accepted(0.0, 3.5), params));
  EXPECT_FALSE(isJump(rejected(1), params));  // 捨てた回の補正は機体の位置を動かさない
  AlignmentReport gated = accepted(0.5, 10.0);
  gated.motion_gated = true;
  EXPECT_FALSE(isJump(gated, params));  // 速く動いていて使わなかった回も同じ
}

TEST(HealthPolicy, AveragesRatesInsideInterval)
{
  using core_localization::averageInInterval;
  using core_localization::StampedRate;
  const std::deque<StampedRate> samples{{0.00, 0.0}, {0.05, 1.0}, {0.10, 2.0}, {0.15, 3.0}};
  ASSERT_TRUE(averageInInterval(samples, 0.05, 0.10).has_value());
  EXPECT_DOUBLE_EQ(*averageInInterval(samples, 0.05, 0.10), 1.5);
  EXPECT_DOUBLE_EQ(*averageInInterval(samples, 0.0, 1.0), 1.5);
  EXPECT_FALSE(averageInInterval(samples, 0.2, 0.3).has_value());
}

TEST(HealthPolicy, GyroLimitGrowsWithRateOnlyWhenEnabled)
{
  HealthPolicyParams params;
  EXPECT_DOUBLE_EQ(gyroDifferenceLimit(params, 0.5, 6.0, 5.8), 0.5);
  params.gyro_relative_tolerance = 0.15;
  EXPECT_DOUBLE_EQ(gyroDifferenceLimit(params, 0.5, 1.0, 1.2), 0.5);
  EXPECT_NEAR(gyroDifferenceLimit(params, 0.5, 6.0, -5.8), 0.9, 1e-12);
}
}  // namespace
