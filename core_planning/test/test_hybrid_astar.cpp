#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "core_planning/grid_map.hpp"
#include "core_planning/hybrid_astar.hpp"

namespace
{
using core_planning::findPathHybridAStar;
using core_planning::GridCell;
using core_planning::GridInfo;
using core_planning::HybridAStarParams;
using core_planning::OccupancyGrid;
using Points = std::vector<std::pair<double, double>>;

// 0.1m 格子の 20m 四方（膨らませ済みとみなす）
OccupancyGrid emptyGrid()
{
  GridInfo info;
  info.resolution = 0.1;
  info.width = 200;
  info.height = 200;
  return OccupancyGrid(info);
}

void fillBox(OccupancyGrid & grid, double x0, double y0, double x1, double y1)
{
  const GridInfo & info = grid.info();
  for (int y = 0; y < info.height; ++y) {
    for (int x = 0; x < info.width; ++x) {
      if (info.centerX(x) >= x0 && info.centerX(x) <= x1 && info.centerY(y) >= y0 &&
        info.centerY(y) <= y1)
      {
        grid.setOccupied(GridCell{x, y});
      }
    }
  }
}

// 道のり前後 0.25m の向きの差から出した曲率の最大 [1/m]（最初と最後の直線のつなぎは除く）
double maxCurvature(const Points & path)
{
  std::vector<std::pair<double, double>> samples;
  double travelled = 0.0;
  double next = 0.0;
  for (std::size_t i = 1U; i < path.size(); ++i) {
    const double length = std::hypot(
      path[i].first - path[i - 1U].first, path[i].second - path[i - 1U].second);
    while (next <= travelled + length) {
      const double t = length > 0.0 ? (next - travelled) / length : 0.0;
      samples.emplace_back(
        path[i - 1U].first + t * (path[i].first - path[i - 1U].first),
        path[i - 1U].second + t * (path[i].second - path[i - 1U].second));
      next += 0.05;
    }
    travelled += length;
  }
  double largest = 0.0;
  for (std::size_t i = 5U; i + 5U < samples.size(); ++i) {
    const double a = std::atan2(
      samples[i].second - samples[i - 5U].second, samples[i].first - samples[i - 5U].first);
    const double b = std::atan2(
      samples[i + 5U].second - samples[i].second, samples[i + 5U].first - samples[i].first);
    largest = std::max(largest, std::abs(std::remainder(b - a, 2.0 * M_PI)) / 0.5);
  }
  return largest;
}

TEST(HybridAStar, ConnectsStraightWhenGoalIsVisible)
{
  const auto grid = emptyGrid();
  const auto path = findPathHybridAStar(grid, {2.0, 2.0}, {15.0, 9.0}, HybridAStarParams{});
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->front(), (std::pair<double, double>{2.0, 2.0}));
  EXPECT_EQ(path->back(), (std::pair<double, double>{15.0, 9.0}));
  EXPECT_LE(path->size(), 3U);
}

TEST(HybridAStar, GoesAroundWallWithinCurvatureLimit)
{
  auto grid = emptyGrid();
  // 下から伸びる壁。右へ回り込む
  fillBox(grid, 9.5, 0.0, 10.5, 14.0);
  HybridAStarParams params;
  params.min_radius_m = 1.0;
  const auto path = findPathHybridAStar(grid, {5.0, 5.0}, {15.0, 5.0}, params);
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->front(), (std::pair<double, double>{5.0, 5.0}));
  EXPECT_EQ(path->back(), (std::pair<double, double>{15.0, 5.0}));
  for (const auto & [x, y] : *path) {
    EXPECT_TRUE(grid.isFree(grid.info().cellAt(x, y))) << x << "," << y;
  }
  // 円弧の曲率の上限 1/min_radius_m（測り方の分だけ余裕を見る）。直線のつなぎの所は 1 点の折れになりうるので、
  // 経路の中ほど（円弧の部分）だけでなく全体で確かめ、上限の 1.6 倍以下とする
  EXPECT_LE(maxCurvature(*path), 1.6 / params.min_radius_m);
}

TEST(HybridAStar, ReturnsNulloptWhenBlockedAndRejectsInvalidParams)
{
  auto grid = emptyGrid();
  fillBox(grid, 9.5, 0.0, 10.5, 20.0);
  EXPECT_FALSE(findPathHybridAStar(grid, {5.0, 5.0}, {15.0, 5.0}, HybridAStarParams{}).has_value());
  HybridAStarParams params;
  params.min_radius_m = 0.0;
  EXPECT_THROW(
    findPathHybridAStar(emptyGrid(), {5.0, 5.0}, {6.0, 5.0}, params), std::invalid_argument);
}
}  // namespace
