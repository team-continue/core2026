#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "core_perception/lidar_merge.hpp"

namespace
{
using core_perception::decidePairing;
using core_perception::findClosestStamp;
using core_perception::MergeCounts;
using core_perception::mergeScans;
using core_perception::PairingAction;
using core_perception::Scan;
using core_perception::ScanPoint;
using core_perception::voxelDownsample;

constexpr std::uint64_t kMs = 1000000ULL;
constexpr std::uint64_t kBase = 1000ULL * 1000000000ULL;

ScanPoint point(const float x, const float y, const float z, const std::uint32_t offset_ns)
{
  ScanPoint result;
  result.position = Eigen::Vector3f(x, y, z);
  result.offset_ns = offset_ns;
  result.reflectivity = 10U;
  result.line = 1U;
  return result;
}

// 1 台目（前の端 x=0.44）から見た 2 台目（後ろの端 x=-0.44、後ろ向き）の取り付け
Eigen::Isometry3f rearMount()
{
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
  transform.translation() = Eigen::Vector3f(-0.88F, 0.0F, 0.0F);
  transform.linear() = Eigen::AngleAxisf(static_cast<float>(M_PI), Eigen::Vector3f::UnitZ())
    .toRotationMatrix();
  return transform;
}

TEST(FindClosestStamp, PicksClosestWithinTolerance)
{
  const std::vector<std::uint64_t> stamps{kBase - 100 * kMs, kBase - 2 * kMs, kBase + 30 * kMs};
  EXPECT_EQ(findClosestStamp(stamps, kBase, 50 * kMs), std::optional<std::size_t>(1U));
  EXPECT_EQ(findClosestStamp(stamps, kBase + 25 * kMs, 50 * kMs), std::optional<std::size_t>(2U));
  EXPECT_FALSE(findClosestStamp(stamps, kBase + 200 * kMs, 50 * kMs).has_value());
  EXPECT_FALSE(findClosestStamp({}, kBase, 50 * kMs).has_value());
}

TEST(DecidePairing, MergesWaitsOrSendsPrimaryOnly)
{
  // 組にできる 2 台目がある
  auto decision = decidePairing(kBase, {kBase - 100 * kMs, kBase + 1 * kMs}, 50 * kMs, true);
  EXPECT_EQ(decision.action, PairingAction::kMerge);
  EXPECT_EQ(decision.secondary_index, 1U);
  // 手元の 2 台目はすべて古い。2 台目が届いていれば、組になるものを待つ
  EXPECT_EQ(decidePairing(kBase, {kBase - 100 * kMs}, 50 * kMs, true).action, PairingAction::kWait);
  EXPECT_EQ(decidePairing(kBase, {}, 50 * kMs, true).action, PairingAction::kWait);
  // 2 台目が止まっていれば待たない
  EXPECT_EQ(
    decidePairing(kBase, {kBase - 100 * kMs}, 50 * kMs, false).action,
    PairingAction::kPrimaryOnly);
  // 1 台目より新しい 2 台目があるのに組にできないなら、組になるものは来ない
  EXPECT_EQ(
    decidePairing(kBase, {kBase + 80 * kMs}, 50 * kMs, true).action, PairingAction::kPrimaryOnly);
}

TEST(MergeScans, TransformsSecondaryIntoPrimaryFrame)
{
  Scan primary{kBase, {point(2.0F, 0.0F, 0.0F, 0U)}};
  Scan secondary{kBase, {point(1.0F, 0.5F, 0.2F, 0U)}};
  MergeCounts counts;
  const Scan merged = mergeScans(primary, secondary, rearMount(), 0.0, counts);
  ASSERT_EQ(merged.points.size(), 2U);
  // 同じ時刻なら 1 台目の点が前
  EXPECT_TRUE(merged.points[0].position.isApprox(Eigen::Vector3f(2.0F, 0.0F, 0.0F)));
  // 後ろ向きの 2 台目の前 1m は、1 台目のフレームでは後ろ（-0.88 - 1.0）
  EXPECT_NEAR(merged.points[1].position.x(), -1.88F, 1e-5F);
  EXPECT_NEAR(merged.points[1].position.y(), -0.5F, 1e-5F);
  EXPECT_NEAR(merged.points[1].position.z(), 0.2F, 1e-5F);
  EXPECT_EQ(merged.points[1].reflectivity, 10U);
  EXPECT_EQ(merged.points[1].line, 1U);
  EXPECT_EQ(counts.primary_input, 1U);
  EXPECT_EQ(counts.secondary_input, 1U);
  EXPECT_EQ(counts.output, 2U);
}

TEST(MergeScans, KeepsAbsolutePointTimesFromEarlierTimebase)
{
  // 2 台目のスキャンが 20ms 早く始まった。共通の基準は早いほう
  const Scan primary{kBase, {point(1.0F, 0.0F, 0.0F, 0U), point(2.0F, 0.0F, 0.0F, 50 * kMs)}};
  const Scan secondary{
    kBase - 20 * kMs, {point(1.0F, 1.0F, 0.0F, 10 * kMs), point(3.0F, 1.0F, 0.0F, 90 * kMs)}};
  MergeCounts counts;
  const Scan merged = mergeScans(
    primary, secondary, Eigen::Isometry3f::Identity(), 0.0, counts);
  EXPECT_EQ(merged.timebase_ns, kBase - 20 * kMs);
  ASSERT_EQ(merged.points.size(), 4U);
  // 絶対時刻: 2 台目 -10ms、1 台目 0ms、1 台目 +50ms、2 台目 +70ms。時刻の順に並ぶ
  const std::vector<std::uint64_t> expected{
    kBase - 10 * kMs, kBase, kBase + 50 * kMs, kBase + 70 * kMs};
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(merged.timebase_ns + merged.points[i].offset_ns, expected[i]) << i;
  }
  EXPECT_FLOAT_EQ(merged.points[0].position.y(), 1.0F);
  EXPECT_FLOAT_EQ(merged.points[1].position.x(), 1.0F);
}

TEST(MergeScans, WithoutSecondaryReturnsPrimary)
{
  const Scan primary{kBase, {point(1.0F, 0.0F, 0.0F, 5U), point(4.0F, 0.0F, 0.0F, 7U)}};
  MergeCounts counts;
  const Scan merged = mergeScans(primary, std::nullopt, rearMount(), 0.1, counts);
  EXPECT_EQ(merged.timebase_ns, kBase);
  ASSERT_EQ(merged.points.size(), 2U);
  EXPECT_EQ(merged.points[0].offset_ns, 5U);
  EXPECT_TRUE(merged.points[1].position.isApprox(Eigen::Vector3f(4.0F, 0.0F, 0.0F)));
  EXPECT_EQ(counts.secondary_input, 0U);
}

TEST(MergeScans, RejectsTimeDifferenceBeyondOffsetRange)
{
  const Scan primary{kBase, {point(1.0F, 0.0F, 0.0F, 0U)}};
  const Scan secondary{kBase - 5000 * kMs, {point(1.0F, 0.0F, 0.0F, 0U)}};
  MergeCounts counts;
  EXPECT_THROW(
    mergeScans(primary, secondary, Eigen::Isometry3f::Identity(), 0.1, counts),
    std::invalid_argument);
}

TEST(MergeScans, SortsUnorderedInput)
{
  const Scan primary{kBase, {point(1.0F, 0.0F, 0.0F, 30U), point(2.0F, 0.0F, 0.0F, 10U)}};
  const Scan secondary{kBase, {point(5.0F, 0.0F, 0.0F, 20U)}};
  MergeCounts counts;
  const Scan merged = mergeScans(primary, secondary, Eigen::Isometry3f::Identity(), 0.0, counts);
  ASSERT_EQ(merged.points.size(), 3U);
  EXPECT_EQ(merged.points[0].offset_ns, 10U);
  EXPECT_EQ(merged.points[1].offset_ns, 20U);
  EXPECT_EQ(merged.points[2].offset_ns, 30U);
}

TEST(VoxelDownsample, KeepsEarliestRealPointPerVoxel)
{
  // 同じ 0.1m の voxel に 3 点（時刻の順）、別の voxel に 1 点
  const std::vector<ScanPoint> points{
    point(1.01F, 1.01F, 0.01F, 100U), point(1.05F, 1.08F, 0.02F, 200U),
    point(1.09F, 1.02F, 0.09F, 300U), point(1.15F, 1.01F, 0.01F, 400U)};
  std::size_t dropped = 0U;
  const auto output = voxelDownsample(points, 0.1, dropped);
  ASSERT_EQ(output.size(), 2U);
  // 重心ではなく、最も早い点を位置と時刻のまま残す
  EXPECT_TRUE(output[0].position.isApprox(points[0].position));
  EXPECT_EQ(output[0].offset_ns, 100U);
  EXPECT_EQ(output[1].offset_ns, 400U);
  EXPECT_EQ(dropped, 0U);
}

TEST(VoxelDownsample, SeparatesNegativeCoordinates)
{
  // 0 をまたぐ点は別の voxel（floor で番号を付ける）
  const std::vector<ScanPoint> points{
    point(0.05F, 0.0F, 0.0F, 1U), point(-0.05F, 0.0F, 0.0F, 2U), point(-0.15F, 0.0F, 0.0F, 3U)};
  std::size_t dropped = 0U;
  EXPECT_EQ(voxelDownsample(points, 0.1, dropped).size(), 3U);
}

TEST(VoxelDownsample, DropsInvalidPointsAndCanBeDisabled)
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const std::vector<ScanPoint> points{
    point(1.0F, 1.0F, 0.0F, 1U), point(1.01F, 1.0F, 0.0F, 2U), point(nan, 0.0F, 0.0F, 3U),
    point(1.0e6F, 0.0F, 0.0F, 4U)};
  std::size_t dropped = 0U;
  EXPECT_EQ(voxelDownsample(points, 0.1, dropped).size(), 1U);
  // NaN と、voxel の番号の範囲（0.1m で ±約 105km）の外
  EXPECT_EQ(dropped, 2U);
  dropped = 0U;
  // 0 なら間引かない（不正な座標だけ捨てる）
  EXPECT_EQ(voxelDownsample(points, 0.0, dropped).size(), 3U);
  EXPECT_EQ(dropped, 1U);
}

TEST(MergeScans, DownsamplesOverlapKeepingPrimaryPoint)
{
  // 2 台が同じ所を同じ時刻に測った: 1 台目の点が残る
  const Scan primary{kBase, {point(3.01F, 0.01F, 0.01F, 0U)}};
  const Scan secondary{kBase, {point(3.02F, 0.02F, 0.02F, 0U), point(6.0F, 0.0F, 0.0F, 0U)}};
  MergeCounts counts;
  const Scan merged = mergeScans(primary, secondary, Eigen::Isometry3f::Identity(), 0.1, counts);
  ASSERT_EQ(merged.points.size(), 2U);
  EXPECT_FLOAT_EQ(merged.points[0].position.x(), 3.01F);
  EXPECT_EQ(counts.output, 2U);
}
}  // namespace
