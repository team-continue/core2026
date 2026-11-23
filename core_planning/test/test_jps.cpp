#include <cmath>
#include <limits>
#include <optional>
#include <queue>
#include <random>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "core_planning/grid_map.hpp"
#include "core_planning/jps.hpp"

namespace
{
using core_planning::computeDistanceField;
using core_planning::findPath;
using core_planning::GridCell;
using core_planning::GridInfo;
using core_planning::hasLineOfSight;
using core_planning::OccupancyGrid;
using core_planning::pathLength;
using core_planning::expandPath;
using core_planning::shortcutPath;
using core_planning::simplifyPath;
using core_planning::findPathJpsTheta;
using core_planning::findPathThetaStar;
using core_planning::parseSearchMethod;
using core_planning::SearchMethod;
using core_planning::searchPath;

OccupancyGrid emptyGrid(const int width, const int height)
{
  GridInfo info;
  info.resolution = 1.0;
  info.width = width;
  info.height = height;
  return OccupancyGrid(info);
}

// 比較用の素朴なダイクストラ（角を横切らない 8 方向移動）
double dijkstraLength(const OccupancyGrid & grid, const GridCell & start, const GridCell & goal)
{
  const GridInfo & info = grid.info();
  std::vector<double> cost(
    static_cast<std::size_t>(info.width * info.height), std::numeric_limits<double>::infinity());
  using Entry = std::pair<double, int>;
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  cost[static_cast<std::size_t>(info.index(start))] = 0.0;
  open.emplace(0.0, info.index(start));
  while (!open.empty()) {
    const auto [current_cost, index] = open.top();
    open.pop();
    const GridCell cell{index % info.width, index / info.width};
    if (current_cost > cost[static_cast<std::size_t>(index)]) {
      continue;
    }
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if ((dx == 0 && dy == 0) || grid.isOccupied(GridCell{cell.x + dx, cell.y + dy})) {
          continue;
        }
        if (dx != 0 && dy != 0 &&
          (grid.isOccupied(GridCell{cell.x + dx, cell.y}) ||
          grid.isOccupied(GridCell{cell.x, cell.y + dy})))
        {
          continue;
        }
        const int next = info.index(GridCell{cell.x + dx, cell.y + dy});
        const double tentative = current_cost + std::hypot(dx, dy);
        if (tentative < cost[static_cast<std::size_t>(next)]) {
          cost[static_cast<std::size_t>(next)] = tentative;
          open.emplace(tentative, next);
        }
      }
    }
  }
  return cost[static_cast<std::size_t>(info.index(goal))];
}

TEST(FindPath, StraightLineOnEmptyGrid)
{
  const auto path = findPath(emptyGrid(20, 20), GridCell{1, 1}, GridCell{15, 1});
  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(pathLength(*path), 14.0, 1e-9);
}

TEST(FindPath, ReturnsNulloptWhenBlocked)
{
  auto grid = emptyGrid(10, 10);
  for (int y = 0; y < 10; ++y) {
    grid.setOccupied(GridCell{5, y});
  }
  EXPECT_FALSE(findPath(grid, GridCell{1, 1}, GridCell{8, 8}).has_value());
}

TEST(FindPath, MatchesDijkstraOnRandomGrids)
{
  std::mt19937 random(1U);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  std::uniform_int_distribution<int> coordinate(0, 39);
  int compared = 0;
  for (int trial = 0; trial < 200; ++trial) {
    auto grid = emptyGrid(40, 40);
    const double density = 0.05 + 0.25 * uniform(random);
    for (int y = 0; y < 40; ++y) {
      for (int x = 0; x < 40; ++x) {
        if (uniform(random) < density) {
          grid.setOccupied(GridCell{x, y});
        }
      }
    }
    const GridCell start{coordinate(random), coordinate(random)};
    const GridCell goal{coordinate(random), coordinate(random)};
    if (grid.isOccupied(start) || grid.isOccupied(goal)) {
      continue;
    }
    const double expected = dijkstraLength(grid, start, goal);
    const auto path = findPath(grid, start, goal);
    if (std::isinf(expected)) {
      EXPECT_FALSE(path.has_value());
      continue;
    }
    ASSERT_TRUE(path.has_value()) << "trial " << trial;
    EXPECT_NEAR(pathLength(*path), expected, 1e-6) << "trial " << trial;
    ++compared;
  }
  EXPECT_GT(compared, 50);
}

// 任意の角度の探索（Theta* と JPS + Theta*）
using AnyAngleSearch = std::optional<std::vector<GridCell>> (*)(
  const OccupancyGrid &, const GridCell &, const GridCell &);
const std::vector<std::pair<const char *, AnyAngleSearch>> kAnyAngleSearches{
  {"theta_star", &findPathThetaStar}, {"jps_theta", &findPathJpsTheta}};

TEST(AnyAngleSearch, StraightLineOnEmptyGrid)
{
  const auto grid = emptyGrid(20, 10);
  for (const auto & [name, search] : kAnyAngleSearches) {
    const auto path = search(grid, GridCell{1, 1}, GridCell{17, 6});
    ASSERT_TRUE(path.has_value()) << name;
    ASSERT_EQ(path->size(), 2U) << name;
    EXPECT_EQ(path->front(), (GridCell{1, 1})) << name;
    EXPECT_EQ(path->back(), (GridCell{17, 6})) << name;
  }
}

TEST(AnyAngleSearch, ReturnsNulloptWhenBlocked)
{
  auto grid = emptyGrid(10, 10);
  for (int y = 0; y < 10; ++y) {
    grid.setOccupied(GridCell{5, y});
  }
  for (const auto & [name, search] : kAnyAngleSearches) {
    EXPECT_FALSE(search(grid, GridCell{1, 1}, GridCell{8, 8}).has_value()) << name;
  }
}

// 見通しのある区間だけでつながり、8 方向の最短経路より長くならない
TEST(AnyAngleSearch, FindsVisiblePathNoLongerThanGridPathOnRandomGrids)
{
  std::mt19937 random(2U);
  std::uniform_real_distribution<double> uniform(0.0, 1.0);
  std::uniform_int_distribution<int> coordinate(0, 39);
  int compared = 0;
  for (int trial = 0; trial < 200; ++trial) {
    auto grid = emptyGrid(40, 40);
    const double density = 0.05 + 0.25 * uniform(random);
    for (int y = 0; y < 40; ++y) {
      for (int x = 0; x < 40; ++x) {
        if (uniform(random) < density) {
          grid.setOccupied(GridCell{x, y});
        }
      }
    }
    const GridCell start{coordinate(random), coordinate(random)};
    const GridCell goal{coordinate(random), coordinate(random)};
    if (grid.isOccupied(start) || grid.isOccupied(goal)) {
      continue;
    }
    const double grid_length = dijkstraLength(grid, start, goal);
    for (const auto & [name, search] : kAnyAngleSearches) {
      const auto path = search(grid, start, goal);
      if (std::isinf(grid_length)) {
        EXPECT_FALSE(path.has_value()) << name << " trial " << trial;
        continue;
      }
      ASSERT_TRUE(path.has_value()) << name << " trial " << trial;
      EXPECT_EQ(path->front(), start) << name << " trial " << trial;
      EXPECT_EQ(path->back(), goal) << name << " trial " << trial;
      for (std::size_t i = 1U; i < path->size(); ++i) {
        EXPECT_TRUE(hasLineOfSight(grid, (*path)[i - 1U], (*path)[i]))
          << name << " trial " << trial << " segment " << i;
      }
      EXPECT_LE(pathLength(*path), grid_length + 1e-6) << name << " trial " << trial;
    }
    compared += std::isinf(grid_length) ? 0 : 1;
  }
  EXPECT_GT(compared, 50);
}

TEST(SearchPath, JpsMatchesFindPathAndRejectsUnknownMethod)
{
  auto grid = emptyGrid(20, 20);
  for (int y = 0; y < 15; ++y) {
    grid.setOccupied(GridCell{10, y});
  }
  const GridCell start{2, 2};
  const GridCell goal{17, 2};
  const auto expected = simplifyPath(grid, *findPath(grid, start, goal));
  EXPECT_EQ(*searchPath(grid, start, goal, SearchMethod::kJps, false), expected);
  EXPECT_EQ(parseSearchMethod("jps"), SearchMethod::kJps);
  EXPECT_EQ(parseSearchMethod("theta_star"), SearchMethod::kThetaStar);
  EXPECT_EQ(parseSearchMethod("jps_theta"), SearchMethod::kJpsTheta);
  EXPECT_THROW(parseSearchMethod("astar"), std::invalid_argument);
  // 壁を回り込む経路は、どの方法でも JPS の折れ線より長くならない
  const double jps_length = pathLength(expected);
  for (const auto method : {SearchMethod::kThetaStar, SearchMethod::kJpsTheta}) {
    const auto path = searchPath(grid, start, goal, method, false);
    ASSERT_TRUE(path.has_value());
    EXPECT_LE(pathLength(*path), jps_length + 1e-6);
  }
}

TEST(SimplifyPath, RemovesPointsWithLineOfSight)
{
  const auto grid = emptyGrid(30, 30);
  const std::vector<GridCell> path{{1, 1}, {5, 5}, {5, 10}, {20, 20}};
  const auto simplified = simplifyPath(grid, path);
  ASSERT_EQ(simplified.size(), 2U);
  EXPECT_EQ(simplified.back(), path.back());
}

TEST(ShortcutPath, CutsAcrossJumpPointsWhereSimplifyKeepsABend)
{
  // 8 方向の経路 (0,0)→(10,10)→(30,10)。(0,0) から (30,10) は壁でふさがるが、(0,0) から辺の途中の
  // (20,10) へは見通しがある。simplifyPath はジャンプポイントにしか飛べないので 45° の曲がりが残る
  auto grid = emptyGrid(40, 20);
  for (int x = 22; x <= 30; ++x) {
    grid.setOccupied(GridCell{x, 5});
    grid.setOccupied(GridCell{x, 6});
    grid.setOccupied(GridCell{x, 7});
  }
  const std::vector<GridCell> path{{0, 0}, {10, 10}, {30, 10}};
  ASSERT_FALSE(hasLineOfSight(grid, path.front(), path.back()));
  const auto simplified = simplifyPath(grid, path);
  EXPECT_EQ(simplified, path);
  const auto shortcut = shortcutPath(grid, path);
  ASSERT_EQ(shortcut.size(), 3U);
  EXPECT_EQ(shortcut.front(), path.front());
  EXPECT_EQ(shortcut.back(), path.back());
  // 曲がる所はジャンプポイント (10,10) より先の辺の上
  EXPECT_EQ(shortcut[1].y, 10);
  EXPECT_GT(shortcut[1].x, 10);
  for (std::size_t i = 1U; i < shortcut.size(); ++i) {
    EXPECT_TRUE(hasLineOfSight(grid, shortcut[i - 1U], shortcut[i]));
  }
}

TEST(ExpandPath, ListsEveryCellAndRejectsNonOctileSegments)
{
  const auto cells = expandPath({{0, 0}, {2, 2}, {2, 5}});
  const std::vector<GridCell> expected{{0, 0}, {1, 1}, {2, 2}, {2, 3}, {2, 4}, {2, 5}};
  EXPECT_EQ(cells, expected);
  EXPECT_THROW(expandPath({{0, 0}, {3, 1}}), std::invalid_argument);
}

TEST(HasLineOfSight, DetectsWall)
{
  auto grid = emptyGrid(20, 20);
  for (int y = 0; y < 15; ++y) {
    grid.setOccupied(GridCell{10, y});
  }
  EXPECT_FALSE(hasLineOfSight(grid, GridCell{5, 5}, GridCell{15, 5}));
  EXPECT_TRUE(hasLineOfSight(grid, GridCell{5, 18}, GridCell{15, 18}));
}

TEST(DistanceField, MeasuresDistanceToNearestObstacle)
{
  auto grid = emptyGrid(21, 21);
  grid.setOccupied(GridCell{10, 10});
  const auto field = computeDistanceField(grid);
  EXPECT_FLOAT_EQ(field.at(GridCell{10, 10}), 0.0F);
  EXPECT_NEAR(field.at(GridCell{13, 14}), 5.0F, 1e-4F);
}

using core_planning::inflate;
using core_planning::planningInflationRadius;

TEST(PlanningInflation, NarrowPassageUsesHalfWidthPlusSideClearance)
{
  // 0.8m 角の機体: 元は外接円の半径 0.566 + 余裕 0.1。狭い通路を通すときは半幅 0.4 + 片側の余裕 0.2
  // （距離の場の丸めの分 0.001m だけ小さくする）
  const double circumscribed = std::hypot(0.4, 0.4) + 0.1;
  EXPECT_NEAR(planningInflationRadius(0.4, 0.4, 0.1, false, 0.2), circumscribed, 1e-9);
  EXPECT_NEAR(planningInflationRadius(0.4, 0.4, 0.1, true, 0.2), 0.6, 2e-3);
  // しきい値が大きくても、元の半径より大きくはしない
  EXPECT_NEAR(planningInflationRadius(0.4, 0.4, 0.1, true, 0.5), circumscribed, 1e-9);
}

// 0.1m の格子（40 × 40 セル、4m 四方）の真ん中を横切る、幅 width の通路（x: 1〜3m）。壁の外側はすべて埋める
OccupancyGrid corridorGrid(const double width)
{
  GridInfo info;
  info.resolution = 0.1;
  info.width = 40;
  info.height = 40;
  OccupancyGrid grid(info);
  for (int x = 10; x < 30; ++x) {
    for (int y = 0; y < info.height; ++y) {
      if (std::abs(info.centerY(y) - 2.0) >= 0.5 * width) {
        grid.setOccupied(GridCell{x, y});
      }
    }
  }
  return grid;
}

TEST(PlanningInflation, FindsPathThroughCorridorWithEnoughSideClearance)
{
  // 片側の余裕 0.2m（幅 1.2m）の通路は、狭い通路を通す半径で通れる。元の半径（外接円）では通れない。
  // 距離の場はセルの中心どうしの距離なので、0.1m の格子では半セルの丸めで幅 1.1m にも経路ができる
  // （その通路では局所計画が入口の手前で止まる）。幅 1.0m（片側 0.1m）は通れない
  const GridCell start{2, 20};
  const GridCell goal{37, 20};
  const double narrow = planningInflationRadius(0.4, 0.4, 0.1, true, 0.2);
  const double original = planningInflationRadius(0.4, 0.4, 0.1, false, 0.2);
  const auto wide_field = computeDistanceField(corridorGrid(1.2));
  EXPECT_TRUE(findPath(inflate(wide_field, narrow), start, goal).has_value());
  EXPECT_FALSE(findPath(inflate(wide_field, original), start, goal).has_value());
  const auto narrow_field = computeDistanceField(corridorGrid(1.0));
  EXPECT_FALSE(findPath(inflate(narrow_field, narrow), start, goal).has_value());
}
}  // namespace
