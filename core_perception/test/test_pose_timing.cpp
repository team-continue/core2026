#include <cstdint>

#include <gtest/gtest.h>

#include "core_perception/pose_timing.hpp"

namespace
{
using core_perception::kPoseStampToleranceNs;
using core_perception::waitsForNewerPose;

constexpr std::uint64_t kCloudNs = 25'300'000'000ULL;
constexpr std::uint64_t kScanPeriodNs = 100'000'000ULL;

TEST(PoseTiming, ProcessesWithPoseOfSameTime)
{
  EXPECT_FALSE(waitsForNewerPose(kCloudNs, kCloudNs));
}

TEST(PoseTiming, ProcessesWithNewerPose)
{
  // 実機は点群の時刻が走査の始まり、姿勢の時刻が終わり
  EXPECT_FALSE(waitsForNewerPose(kCloudNs, kCloudNs + kScanPeriodNs));
}

TEST(PoseTiming, ToleratesTruncatedPoseStamp)
{
  // FAST-LIO は double の秒から戻すときに ns を切り捨てるので、同じスキャンの姿勢が 1ns 早くなる
  EXPECT_FALSE(waitsForNewerPose(kCloudNs, kCloudNs - 1U));
  EXPECT_FALSE(waitsForNewerPose(kCloudNs, kCloudNs - kPoseStampToleranceNs));
}

TEST(PoseTiming, WaitsBeyondTolerance)
{
  EXPECT_TRUE(waitsForNewerPose(kCloudNs, kCloudNs - kPoseStampToleranceNs - 1U));
}

TEST(PoseTiming, WaitsForPoseOfPreviousScan)
{
  // 実機の Mid-360 は、前のフレームの最後の点が次のフレームの最初の点より数 µs 早い
  EXPECT_TRUE(waitsForNewerPose(kCloudNs, kCloudNs - 4'000U));
  EXPECT_TRUE(waitsForNewerPose(kCloudNs, kCloudNs - kScanPeriodNs));
}

}  // namespace
