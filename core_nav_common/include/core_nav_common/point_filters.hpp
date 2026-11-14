// 点群のボクセルフィルタとランダムサンプル。点の型を問わず使える。

#ifndef CORE_NAV_COMMON__POINT_FILTERS_HPP_
#define CORE_NAV_COMMON__POINT_FILTERS_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <random>
#include <unordered_set>
#include <vector>

namespace core_nav_common
{
// 各軸 21 ビットに詰めて 1 つの整数にする。0.05m 格子でも ±50km まで衝突しない
inline std::uint64_t voxelKey(const double x, const double y, const double z, const double leaf)
{
  constexpr std::int64_t kOffset = std::int64_t{1} << 20;
  constexpr std::uint64_t kMask = (std::uint64_t{1} << 21) - 1;
  const auto pack = [leaf](const double value) {
      const auto index = static_cast<std::int64_t>(std::floor(value / leaf)) + kOffset;
      return static_cast<std::uint64_t>(index) & kMask;
    };
  return (pack(x) << 42) | (pack(y) << 21) | pack(z);
}

// 各ボクセルで最初の点を残す。平均を取らないのは、実機の点は時刻を持っていて、
// 平均すると歪み補正に使う時刻が壊れるため
template<typename Point, typename PositionOf>
std::vector<Point> voxelFilter(
  const std::vector<Point> & points, const double leaf, PositionOf position_of)
{
  std::unordered_set<std::uint64_t> seen;
  seen.reserve(points.size());
  std::vector<Point> output;
  output.reserve(points.size());
  for (const auto & point : points) {
    const std::array<float, 3> position = position_of(point);
    if (seen.insert(voxelKey(position[0], position[1], position[2], leaf)).second) {
      output.push_back(point);
    }
  }
  return output;
}

// target 点を一様に選ぶ。順序は保つ
template<typename Point>
std::vector<Point> randomSample(
  const std::vector<Point> & points, const std::size_t target, std::mt19937 & random)
{
  std::vector<Point> output;
  output.reserve(std::min(target, points.size()));
  std::sample(points.begin(), points.end(), std::back_inserter(output), target, random);
  return output;
}
}  // namespace core_nav_common

#endif  // CORE_NAV_COMMON__POINT_FILTERS_HPP_
