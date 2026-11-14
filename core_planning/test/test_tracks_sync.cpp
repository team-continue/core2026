#include <cstdint>
#include <optional>

#include <gtest/gtest.h>

#include "core_planning/tracks_sync.hpp"

namespace
{
using core_planning::isTracksForPose;
using core_planning::kScanStampToleranceNs;
using core_planning::shouldWaitForTracks;
using core_planning::TracksWaitParams;

constexpr std::int64_t kPoseNs = 25'300'000'000LL;
constexpr std::int64_t kScanPeriodNs = 100'000'000LL;

TEST(TracksSync, SameStampIsSameScan)
{
  EXPECT_TRUE(isTracksForPose(kPoseNs, kPoseNs));
  EXPECT_TRUE(isTracksForPose(kPoseNs + kScanPeriodNs, kPoseNs));
}

TEST(TracksSync, ToleratesPoseStampTruncatedByFastLio)
{
  // FAST-LIO の切り捨てで、姿勢の時刻が追跡結果より 1ns 早くなる
  EXPECT_TRUE(isTracksForPose(kPoseNs, kPoseNs - 1));
  // 逆に追跡結果が少し早いときも、余裕の内なら同じスキャン
  EXPECT_TRUE(isTracksForPose(kPoseNs - kScanStampToleranceNs, kPoseNs));
  EXPECT_FALSE(isTracksForPose(kPoseNs - kScanStampToleranceNs - 1, kPoseNs));
}

TEST(TracksSync, WaitsForTracksOfSameScan)
{
  // 最新の追跡結果が 1 つ前のスキャンなら待つ
  EXPECT_TRUE(shouldWaitForTracks(kPoseNs - kScanPeriodNs, kPoseNs, TracksWaitParams{}));
}

TEST(TracksSync, DoesNotWaitWhenTracksAreReady)
{
  EXPECT_FALSE(shouldWaitForTracks(kPoseNs, kPoseNs, TracksWaitParams{}));
  EXPECT_FALSE(shouldWaitForTracks(kPoseNs, kPoseNs - 1, TracksWaitParams{}));
}

TEST(TracksSync, DoesNotWaitWithoutTracks)
{
  // 検出がまだ何も出していないとき（起動直後や、検出を動かしていないとき）
  EXPECT_FALSE(shouldWaitForTracks(std::nullopt, kPoseNs, TracksWaitParams{}));
}

TEST(TracksSync, DoesNotWaitWhenPerceptionStopped)
{
  // 最新の追跡結果が alive_s より古ければ、検出が止まっているとみなす
  const TracksWaitParams params{0.02, 1.0};
  EXPECT_TRUE(shouldWaitForTracks(kPoseNs - 1'000'000'000LL, kPoseNs, params));
  EXPECT_FALSE(shouldWaitForTracks(kPoseNs - 1'000'000'001LL, kPoseNs, params));
}

TEST(TracksSync, DisabledWithZeroWait)
{
  EXPECT_FALSE(shouldWaitForTracks(kPoseNs - kScanPeriodNs, kPoseNs, TracksWaitParams{0.0, 1.0}));
}

}  // namespace
