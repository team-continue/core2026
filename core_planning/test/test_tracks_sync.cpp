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

TEST(TracksSync, StampLagAcceptsTracksStampedAtScanStart)
{
  // 点群のヘッダが走査の始め（姿勢は走査の終わり）。sim の点群は最後の点を走査の終わりの 10µs 前に置く
  const TracksWaitParams params{0.02, 1.0, 0.105};
  const std::int64_t lag_ns = core_planning::stampLagNs(params);
  EXPECT_EQ(lag_ns, 105'000'000LL);
  const std::int64_t scan_start_ns = kPoseNs - kScanPeriodNs + 10'000LL;
  EXPECT_TRUE(isTracksForPose(scan_start_ns, kPoseNs, lag_ns));
  EXPECT_TRUE(isTracksForPose(kPoseNs - kScanPeriodNs, kPoseNs, lag_ns));
  // 1 つ前のスキャン（2 走査前の始め）は同じとみなさない
  EXPECT_FALSE(isTracksForPose(kPoseNs - 2 * kScanPeriodNs, kPoseNs, lag_ns));
  // 同じスキャンの追跡結果が届いていれば待たず、1 つ前のスキャンなら待つ
  EXPECT_FALSE(shouldWaitForTracks(scan_start_ns, kPoseNs, params));
  EXPECT_TRUE(shouldWaitForTracks(kPoseNs - 2 * kScanPeriodNs, kPoseNs, params));
}

TEST(TracksSync, ZeroStampLagKeepsExactMatching)
{
  // stamp_lag_s が 0（既定）なら前の動き: 走査の始めの時刻の追跡結果では待つ
  EXPECT_EQ(core_planning::stampLagNs(TracksWaitParams{}), 0);
  EXPECT_TRUE(shouldWaitForTracks(kPoseNs - kScanPeriodNs + 10'000LL, kPoseNs, TracksWaitParams{}));
  // 負の値は 0 として扱う
  EXPECT_EQ(core_planning::stampLagNs(TracksWaitParams{0.02, 1.0, -0.1}), 0);
}

}  // namespace
