#include "core_perception/lidar_merge.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iterator>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace core_perception
{
namespace
{
// voxel の番号は各軸 21 ビット（±2^20 個）に詰める。0.1m なら ±約 105km で、LiDAR の距離には十分
constexpr std::int64_t kVoxelIndexLimit = std::int64_t{1} << 20;

bool byOffset(const ScanPoint & a, const ScanPoint & b)
{
  return a.offset_ns < b.offset_ns;
}

std::optional<std::uint64_t> voxelKey(const Eigen::Vector3f & position, const double voxel_size_m)
{
  std::uint64_t key = 0U;
  for (int axis = 0; axis < 3; ++axis) {
    const double index = std::floor(static_cast<double>(position[axis]) / voxel_size_m);
    if (!(std::abs(index) < static_cast<double>(kVoxelIndexLimit))) {
      return std::nullopt;
    }
    const auto shifted = static_cast<std::uint64_t>(static_cast<std::int64_t>(index) + kVoxelIndexLimit);
    key = (key << 21U) | shifted;
  }
  return key;
}

// スキャンの点の時刻を、共通の基準 base_ns からの差に直す
std::uint32_t shiftedOffset(
  const std::uint64_t scan_timebase_ns, const std::uint32_t offset_ns, const std::uint64_t base_ns)
{
  const std::uint64_t shifted = scan_timebase_ns - base_ns + offset_ns;
  if (shifted > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument(
            "2 台のスキャンの時刻の差が点の時刻（offset_time、uint32 の ns）の範囲を超えます: " +
            std::to_string(scan_timebase_ns - base_ns) + " ns");
  }
  return static_cast<std::uint32_t>(shifted);
}
}  // namespace

std::optional<std::size_t> findClosestStamp(
  const std::vector<std::uint64_t> & candidates_ns, const std::uint64_t target_ns,
  const std::uint64_t max_difference_ns)
{
  std::optional<std::size_t> best;
  std::uint64_t best_difference = std::numeric_limits<std::uint64_t>::max();
  for (std::size_t i = 0; i < candidates_ns.size(); ++i) {
    const std::uint64_t difference = candidates_ns[i] > target_ns ?
      candidates_ns[i] - target_ns : target_ns - candidates_ns[i];
    if (difference <= max_difference_ns && difference < best_difference) {
      best = i;
      best_difference = difference;
    }
  }
  return best;
}

PairingDecision decidePairing(
  const std::uint64_t primary_ns, const std::vector<std::uint64_t> & secondary_ns,
  const std::uint64_t max_difference_ns, const bool secondary_alive)
{
  if (const auto index = findClosestStamp(secondary_ns, primary_ns, max_difference_ns)) {
    return PairingDecision{PairingAction::kMerge, *index};
  }
  // 手元に 1 台目より新しい 2 台目があるなら、組になる 2 台目は来ない（届く順は時刻の順）
  const bool partner_may_come = std::all_of(
    secondary_ns.begin(), secondary_ns.end(),
    [primary_ns](const std::uint64_t stamp) {return stamp < primary_ns;});
  return PairingDecision{
    secondary_alive && partner_may_come ? PairingAction::kWait : PairingAction::kPrimaryOnly, 0U};
}

std::vector<ScanPoint> voxelDownsample(
  const std::vector<ScanPoint> & points, const double voxel_size_m, std::size_t & dropped)
{
  std::vector<ScanPoint> output;
  output.reserve(points.size());
  const bool downsample = voxel_size_m > 0.0;
  std::unordered_set<std::uint64_t> occupied;
  if (downsample) {
    occupied.reserve(points.size());
  }
  for (const auto & point : points) {
    if (!point.position.allFinite()) {
      ++dropped;
      continue;
    }
    if (!downsample) {
      output.push_back(point);
      continue;
    }
    const auto key = voxelKey(point.position, voxel_size_m);
    if (!key.has_value()) {
      ++dropped;
      continue;
    }
    if (occupied.insert(*key).second) {
      output.push_back(point);
    }
  }
  return output;
}

Scan mergeScans(
  const Scan & primary, const std::optional<Scan> & secondary,
  const Eigen::Isometry3f & primary_from_secondary, const double voxel_size_m,
  MergeCounts & counts)
{
  counts = MergeCounts{};
  counts.primary_input = primary.points.size();
  counts.secondary_input = secondary.has_value() ? secondary->points.size() : 0U;

  Scan merged;
  merged.timebase_ns = secondary.has_value() ?
    std::min(primary.timebase_ns, secondary->timebase_ns) : primary.timebase_ns;

  std::vector<ScanPoint> points;
  points.reserve(counts.primary_input + counts.secondary_input);
  std::transform(
    primary.points.begin(), primary.points.end(), std::back_inserter(points),
    [&primary, &merged](ScanPoint point) {
      point.offset_ns = shiftedOffset(primary.timebase_ns, point.offset_ns, merged.timebase_ns);
      return point;
    });
  const auto primary_end = static_cast<std::ptrdiff_t>(points.size());
  if (secondary.has_value()) {
    std::transform(
      secondary->points.begin(), secondary->points.end(), std::back_inserter(points),
      [&secondary, &merged, &primary_from_secondary](ScanPoint point) {
        point.position = primary_from_secondary * point.position;
        point.offset_ns =
        shiftedOffset(secondary->timebase_ns, point.offset_ns, merged.timebase_ns);
        return point;
      });
  }

  // 時刻の順に並べる。各スキャンの中は普通は時刻の順なので、そのときは 2 つを合わせるだけにする。
  // 同じ時刻なら 1 台目の点を前に置く（voxel で 1 台目の点が残る）
  const auto middle = points.begin() + primary_end;
  if (std::is_sorted(points.begin(), middle, byOffset) &&
    std::is_sorted(middle, points.end(), byOffset))
  {
    std::inplace_merge(points.begin(), middle, points.end(), byOffset);
  } else {
    std::stable_sort(points.begin(), points.end(), byOffset);
  }

  merged.points = voxelDownsample(points, voxel_size_m, counts.dropped_invalid);
  counts.output = merged.points.size();
  return merged;
}
}  // namespace core_perception
