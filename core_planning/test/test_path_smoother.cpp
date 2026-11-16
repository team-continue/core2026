#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "core_planning/grid_map.hpp"
#include "core_planning/jps.hpp"
#include "core_planning/path_smoother.hpp"

namespace
{
using core_planning::computeDistanceField;
using core_planning::DistanceField;
using core_planning::FieldBounds;
using core_planning::findPath;
using core_planning::GridCell;
using core_planning::GridInfo;
using core_planning::inflate;
using core_planning::makeFieldGrid;
using core_planning::nearestFreeCell;
using core_planning::OccupancyGrid;
using core_planning::simplifyPath;
using core_planning::parseSmoothingMode;
using core_planning::smoothPath;
using core_planning::SmoothingMode;
using core_planning::SmoothingParams;
using Points = std::vector<std::pair<double, double>>;

// 0.8m 角の機体の外接円に余裕 0.1m を足した、大域計画の膨らみの半径
const double kHardRadiusM = std::hypot(0.4, 0.4) + 0.1;
constexpr double kResolutionM = 0.1;

// 柵で囲った 20m 四方のフィールド
OccupancyGrid fieldGrid()
{
  return makeFieldGrid(FieldBounds{0.0, 20.0, 0.0, 20.0}, kResolutionM);
}

// 中心が箱に入るセルを占有にする
void fillBox(
  OccupancyGrid & grid, const double x_min, const double y_min, const double x_max,
  const double y_max)
{
  const GridInfo & info = grid.info();
  for (int y = 0; y < info.height; ++y) {
    for (int x = 0; x < info.width; ++x) {
      const double cx = info.centerX(x);
      const double cy = info.centerY(y);
      if (cx >= x_min && cx <= x_max && cy >= y_min && cy <= y_max) {
        grid.setOccupied(GridCell{x, y});
      }
    }
  }
}

double distanceAt(const DistanceField & field, const std::pair<double, double> & point)
{
  return static_cast<double>(field.at(point.first, point.second));
}

double length(const Points & points)
{
  double total = 0.0;
  for (std::size_t i = 1U; i < points.size(); ++i) {
    total += std::hypot(
      points[i].first - points[i - 1U].first, points[i].second - points[i - 1U].second);
  }
  return total;
}

// 隣り合う区間の向きの変化の最大 [rad]
double maxTurn(const Points & points)
{
  double largest = 0.0;
  for (std::size_t i = 2U; i < points.size(); ++i) {
    const double before = std::atan2(
      points[i - 1U].second - points[i - 2U].second, points[i - 1U].first - points[i - 2U].first);
    const double after = std::atan2(
      points[i].second - points[i - 1U].second, points[i].first - points[i - 1U].first);
    largest = std::max(largest, std::abs(std::remainder(after - before, 2.0 * M_PI)));
  }
  return largest;
}

double minDistance(const DistanceField & field, const Points & points)
{
  double smallest = std::numeric_limits<double>::infinity();
  for (const auto & point : points) {
    smallest = std::min(smallest, distanceAt(field, point));
  }
  return smallest;
}

// 大域計画のノードと同じ手順で折れ線を作る。始点のセルは機体の位置で置き換え、終点はゴールの位置にする
Points plannedPolyline(
  const OccupancyGrid & inflated, const std::pair<double, double> & start,
  const std::pair<double, double> & goal)
{
  const GridInfo & info = inflated.info();
  const auto start_cell = nearestFreeCell(inflated, info.cellAt(start.first, start.second), 20);
  const GridCell goal_cell = info.cellAt(goal.first, goal.second);
  if (!start_cell.has_value()) {
    throw std::runtime_error("始点の近くに空きセルがありません");
  }
  const auto path = findPath(inflated, *start_cell, goal_cell);
  if (!path.has_value()) {
    throw std::runtime_error("経路がありません");
  }
  const auto cells = simplifyPath(inflated, *path);
  Points points{start};
  for (std::size_t i = 1U; i < cells.size(); ++i) {
    points.emplace_back(info.centerX(cells[i].x), info.centerY(cells[i].y));
  }
  points.back() = goal;
  return points;
}

// 折れ線を点に増やしただけのもの（動かす回数 0）
Points resampled(const Points & polyline, const DistanceField & field, SmoothingParams params)
{
  params.iterations = 0;
  const auto points = smoothPath(polyline, field, kHardRadiusM, params);
  if (!points.has_value()) {
    throw std::runtime_error("折れ線を点に増やせません");
  }
  return *points;
}

TEST(SmoothPath, StraightLineFarFromObstaclesDoesNotMove)
{
  const auto field = computeDistanceField(fieldGrid());
  const Points polyline{{3.0, 10.0}, {9.0, 10.3}, {17.0, 10.7}};
  const auto smoothed = smoothPath(polyline, field, kHardRadiusM, SmoothingParams{});
  ASSERT_TRUE(smoothed.has_value());
  // 14m を 0.1m おき
  EXPECT_GE(smoothed->size(), 140U);
  EXPECT_EQ(smoothed->front(), polyline.front());
  EXPECT_EQ(smoothed->back(), polyline.back());
  // 直線 y = 10 + 0.05 (x - 3) からのずれ
  for (const auto & [x, y] : *smoothed) {
    const double offset = std::abs(y - (10.0 + 0.05 * (x - 3.0))) / std::hypot(1.0, 0.05);
    EXPECT_LT(offset, 0.01) << "x=" << x;
  }
}

TEST(SmoothPath, RoundsPolylineAroundBoxCorners)
{
  auto grid = fieldGrid();
  fillBox(grid, 8.0, 8.0, 12.0, 12.0);
  const auto field = computeDistanceField(grid);
  const auto inflated = inflate(field, kHardRadiusM);
  const Points polyline = plannedPolyline(inflated, {4.0, 10.5}, {16.0, 10.5});
  // 箱の 2 つの角で曲がる折れ線になる
  ASSERT_GE(polyline.size(), 4U);
  const SmoothingParams params;
  const auto smoothed = smoothPath(polyline, field, kHardRadiusM, params);
  ASSERT_TRUE(smoothed.has_value());

  EXPECT_EQ(smoothed->front(), polyline.front());
  EXPECT_EQ(smoothed->back(), polyline.back());
  EXPECT_GE(minDistance(field, *smoothed), kHardRadiusM);
  // 折れ線は 1 点で 30° 以上曲がるが、曲線は点ごとに小さく曲がる
  const double polyline_turn = maxTurn(polyline);
  const double smoothed_turn = maxTurn(*smoothed);
  EXPECT_GT(polyline_turn, 30.0 * M_PI / 180.0);
  EXPECT_LT(smoothed_turn, 8.0 * M_PI / 180.0) << "折れ線 " << polyline_turn * 180.0 / M_PI << "°";
  // 余裕を取る分だけ長くなるが、10% 未満
  EXPECT_LT(length(*smoothed), 1.1 * length(polyline));
  // 膨らみの境界に接していた折れ線より、余裕を取って離れる
  EXPECT_GT(minDistance(field, *smoothed), kHardRadiusM + 0.5 * params.extra_clearance_m);
}

SmoothingParams bendingParams()
{
  SmoothingParams params;
  params.mode = SmoothingMode::kBending;
  return params;
}

TEST(BendingSmoothing, StraightLineDoesNotMove)
{
  const auto field = computeDistanceField(fieldGrid());
  const Points polyline{{3.0, 10.0}, {9.0, 10.3}, {17.0, 10.7}};
  const auto smoothed = smoothPath(polyline, field, kHardRadiusM, bendingParams());
  ASSERT_TRUE(smoothed.has_value());
  EXPECT_EQ(smoothed->front(), polyline.front());
  EXPECT_EQ(smoothed->back(), polyline.back());
  for (const auto & [x, y] : *smoothed) {
    const double offset = std::abs(y - (10.0 + 0.05 * (x - 3.0))) / std::hypot(1.0, 0.05);
    EXPECT_LT(offset, 0.01) << "x=" << x;
  }
}

TEST(BendingSmoothing, SpreadsCornerTurnMoreThanLaplacian)
{
  // 障害物から遠い直角の角。laplacian は角の近くで大きく曲がるが、bending は曲がりを広げる
  const auto field = computeDistanceField(fieldGrid());
  const Points polyline{{4.0, 5.0}, {10.0, 5.0}, {10.0, 15.0}};
  const auto laplacian = smoothPath(polyline, field, kHardRadiusM, SmoothingParams{});
  const auto bending = smoothPath(polyline, field, kHardRadiusM, bendingParams());
  ASSERT_TRUE(laplacian.has_value());
  ASSERT_TRUE(bending.has_value());
  EXPECT_EQ(bending->front(), polyline.front());
  EXPECT_EQ(bending->back(), polyline.back());
  EXPECT_GE(minDistance(field, *bending), kHardRadiusM);
  EXPECT_LT(maxTurn(*bending), 0.5 * maxTurn(*laplacian))
    << "laplacian " << maxTurn(*laplacian) * 180.0 / M_PI << "°、bending "
    << maxTurn(*bending) * 180.0 / M_PI << "°";
  EXPECT_LT(maxTurn(*bending), 5.0 * M_PI / 180.0);
}

TEST(BendingSmoothing, RoundsAroundBoxKeepingClearance)
{
  auto grid = fieldGrid();
  fillBox(grid, 8.0, 8.0, 12.0, 12.0);
  const auto field = computeDistanceField(grid);
  const auto inflated = inflate(field, kHardRadiusM);
  const Points polyline = plannedPolyline(inflated, {4.0, 10.5}, {16.0, 10.5});
  const auto laplacian = smoothPath(polyline, field, kHardRadiusM, SmoothingParams{});
  const auto bending = smoothPath(polyline, field, kHardRadiusM, bendingParams());
  ASSERT_TRUE(laplacian.has_value());
  ASSERT_TRUE(bending.has_value());
  EXPECT_EQ(bending->front(), polyline.front());
  EXPECT_EQ(bending->back(), polyline.back());
  EXPECT_GE(minDistance(field, *bending), kHardRadiusM);
  EXPECT_LE(maxTurn(*bending), maxTurn(*laplacian) + 1e-9)
    << "laplacian " << maxTurn(*laplacian) * 180.0 / M_PI << "°、bending "
    << maxTurn(*bending) * 180.0 / M_PI << "°";
}

TEST(BendingSmoothing, StaysOnCenterlineOfGapPushedFromBothSides)
{
  auto grid = fieldGrid();
  fillBox(grid, 7.0, 0.0, 13.0, 9.2);
  fillBox(grid, 7.0, 10.8, 13.0, 20.0);
  const auto field = computeDistanceField(grid);
  SmoothingParams params = bendingParams();
  params.extra_clearance_m = 0.6;
  for (const double offset : {0.0, 0.05, -0.15}) {
    const Points polyline{{2.0, 10.0 + offset}, {18.0, 10.0 + offset}};
    const auto smoothed = smoothPath(polyline, field, kHardRadiusM, params);
    ASSERT_TRUE(smoothed.has_value()) << "offset=" << offset;
    EXPECT_GE(minDistance(field, *smoothed), kHardRadiusM) << "offset=" << offset;
    Points inside;
    for (const auto & point : *smoothed) {
      if (point.first > 8.0 && point.first < 12.0) {
        inside.push_back(point);
        EXPECT_LE(std::abs(point.second - 10.0), 0.5 * kResolutionM + 1e-3)
          << "offset=" << offset << " x=" << point.first;
      }
    }
    EXPECT_LT(maxTurn(inside), 2.0 * M_PI / 180.0) << "offset=" << offset;
  }
}

TEST(BendingSmoothing, ParsesModeAndRejectsInvalidLength)
{
  EXPECT_EQ(parseSmoothingMode("laplacian"), SmoothingMode::kLaplacian);
  EXPECT_EQ(parseSmoothingMode("bending"), SmoothingMode::kBending);
  EXPECT_THROW(parseSmoothingMode("spline"), std::invalid_argument);
  const auto field = computeDistanceField(fieldGrid());
  SmoothingParams params = bendingParams();
  params.bending_length_m = 0.0;
  EXPECT_THROW(
    smoothPath(Points{{3.0, 3.0}, {5.0, 5.0}}, field, kHardRadiusM, params), std::invalid_argument);
}

SmoothingParams corridorParams()
{
  SmoothingParams params;
  params.mode = SmoothingMode::kCorridor;
  return params;
}

TEST(CorridorSmoothing, SpreadsCornerTurnAndKeepsOutsideInflation)
{
  const auto field = computeDistanceField(fieldGrid());
  const Points polyline{{4.0, 5.0}, {10.0, 5.0}, {10.0, 15.0}};
  const auto laplacian = smoothPath(polyline, field, kHardRadiusM, SmoothingParams{});
  const auto corridor = smoothPath(polyline, field, kHardRadiusM, corridorParams());
  ASSERT_TRUE(laplacian.has_value());
  ASSERT_TRUE(corridor.has_value());
  EXPECT_EQ(corridor->front(), polyline.front());
  EXPECT_EQ(corridor->back(), polyline.back());
  EXPECT_GE(minDistance(field, *corridor), kHardRadiusM);
  EXPECT_LT(maxTurn(*corridor), 0.5 * maxTurn(*laplacian))
    << "laplacian " << maxTurn(*laplacian) * 180.0 / M_PI << "°、corridor "
    << maxTurn(*corridor) * 180.0 / M_PI << "°";
}

TEST(CorridorSmoothing, RoundsAroundBoxKeepingClearance)
{
  auto grid = fieldGrid();
  fillBox(grid, 8.0, 8.0, 12.0, 12.0);
  const auto field = computeDistanceField(grid);
  const auto inflated = inflate(field, kHardRadiusM);
  const Points polyline = plannedPolyline(inflated, {4.0, 10.5}, {16.0, 10.5});
  const auto laplacian = smoothPath(polyline, field, kHardRadiusM, SmoothingParams{});
  const auto corridor = smoothPath(polyline, field, kHardRadiusM, corridorParams());
  ASSERT_TRUE(laplacian.has_value());
  ASSERT_TRUE(corridor.has_value());
  EXPECT_GE(minDistance(field, *corridor), kHardRadiusM);
  EXPECT_LE(maxTurn(*corridor), maxTurn(*laplacian) + 1e-9);
  EXPECT_EQ(parseSmoothingMode("corridor"), SmoothingMode::kCorridor);
}

TEST(SmoothPath, KeepsOutsideInflationInGapNarrowerThanClearance)
{
  // 幅 1.6m のすき間。中央でも膨らみの外までの余裕は 0.3m 取れない
  auto grid = fieldGrid();
  fillBox(grid, 9.0, 0.0, 11.0, 9.2);
  fillBox(grid, 9.0, 10.8, 11.0, 20.0);
  const auto field = computeDistanceField(grid);
  const Points polyline{{3.0, 10.1}, {6.0, 10.1}, {14.0, 10.1}, {17.0, 10.1}};
  ASSERT_GE(minDistance(field, resampled(polyline, field, SmoothingParams{})), kHardRadiusM);

  const auto smoothed = smoothPath(polyline, field, kHardRadiusM, SmoothingParams{});
  ASSERT_TRUE(smoothed.has_value());
  EXPECT_EQ(smoothed->front(), polyline.front());
  EXPECT_EQ(smoothed->back(), polyline.back());
  EXPECT_GE(minDistance(field, *smoothed), kHardRadiusM);
  // 両側から押されて、すき間の中央（y = 10）へ寄る
  for (const auto & [x, y] : *smoothed) {
    if (x > 9.0 && x < 11.0) {
      EXPECT_LT(std::abs(y - 10.0), 0.1) << "x=" << x;
    }
  }
}

TEST(SmoothPath, StaysStraightOnCenterlineOfGapPushedFromBothSides)
{
  // 幅 1.6m の通路（長さ 6m）。取りたい距離（膨らみ + 0.6m、約 1.27m）は真ん中でも足りないので、
  // 両側から押される。押し出しが真ん中を越えると、点が左右に行き来して 1 点ごとに折れる
  auto grid = fieldGrid();
  fillBox(grid, 7.0, 0.0, 13.0, 9.2);
  fillBox(grid, 7.0, 10.8, 13.0, 20.0);
  const auto field = computeDistanceField(grid);
  SmoothingParams params;
  params.extra_clearance_m = 0.6;
  for (const double offset : {0.0, 0.05, 0.1, -0.15}) {
    const Points polyline{{2.0, 10.0 + offset}, {18.0, 10.0 + offset}};
    const auto smoothed = smoothPath(polyline, field, kHardRadiusM, params);
    ASSERT_TRUE(smoothed.has_value()) << "offset=" << offset;
    EXPECT_GE(minDistance(field, *smoothed), kHardRadiusM) << "offset=" << offset;
    Points inside;
    for (const auto & point : *smoothed) {
      if (point.first > 8.0 && point.first < 12.0) {
        inside.push_back(point);
        // 距離場はセルの中心で持つので、真ん中の 2 セル（y = 9.95 と 10.05）の間は距離が同じで、
        // そこより真ん中へは押されない。セルの半分までのずれは許す
        EXPECT_LE(std::abs(point.second - 10.0), 0.5 * kResolutionM + 1e-3)
          << "offset=" << offset << " x=" << point.first;
      }
    }
    ASSERT_GE(inside.size(), 30U);
    EXPECT_LT(maxTurn(inside), 2.0 * M_PI / 180.0) << "offset=" << offset;
  }
}

TEST(SmoothPath, ReturnsNulloptWhenPolylineEntersInflationMidway)
{
  // 幅 1.0m のすき間は膨らみで塞がる。途中の区間でここを通る折れ線は曲線にしない
  auto grid = fieldGrid();
  fillBox(grid, 9.0, 0.0, 11.0, 9.5);
  fillBox(grid, 9.0, 10.5, 11.0, 20.0);
  const auto field = computeDistanceField(grid);
  const Points polyline{{3.0, 10.0}, {6.0, 10.0}, {14.0, 10.0}, {17.0, 10.0}};
  EXPECT_FALSE(smoothPath(polyline, field, kHardRadiusM, SmoothingParams{}).has_value());
}

TEST(SmoothPath, StartAndGoalInsideInflationDoNotMoveCloser)
{
  // 機体が箱の左の面から 0.45m（膨らみの中）にいる。ノードと同じく、近くの空きセルから計画する
  auto grid = fieldGrid();
  fillBox(grid, 10.0, 5.0, 12.0, 15.0);
  const auto field = computeDistanceField(grid);
  const auto inflated = inflate(field, kHardRadiusM);
  const std::pair<double, double> start{9.6, 8.0};
  ASSERT_LT(distanceAt(field, start), kHardRadiusM);
  const Points forward = plannedPolyline(inflated, start, {14.0, 17.0});
  // ゴールの側が膨らみの中にあるときも同じに扱う
  const Points backward(forward.rbegin(), forward.rend());

  for (const Points & polyline : {forward, backward}) {
    const SmoothingParams params;
    const Points before = resampled(polyline, field, params);
    ASSERT_LT(minDistance(field, before), kHardRadiusM);
    const auto smoothed = smoothPath(polyline, field, kHardRadiusM, params);
    // 膨らみの中から始まっても、折れ線に戻らない
    ASSERT_TRUE(smoothed.has_value());
    ASSERT_EQ(smoothed->size(), before.size());
    EXPECT_EQ(smoothed->front(), polyline.front());
    EXPECT_EQ(smoothed->back(), polyline.back());
    int inside_before = 0;
    int inside_after = 0;
    for (std::size_t i = 0U; i < before.size(); ++i) {
      // 元の折れ線より障害物に近づかない。膨らみの外の点は外に保つ
      const double floor = std::min(kHardRadiusM, distanceAt(field, before[i]));
      EXPECT_GE(distanceAt(field, (*smoothed)[i]), floor) << "i=" << i;
      inside_before += distanceAt(field, before[i]) < kHardRadiusM ? 1 : 0;
      inside_after += distanceAt(field, (*smoothed)[i]) < kHardRadiusM ? 1 : 0;
    }
    // 膨らみの中の点は外へ押し出される
    EXPECT_GT(inside_before, 0);
    EXPECT_LT(inside_after, inside_before);
    // 押し出しても、端点の近くで鉤形に曲がらない
    EXPECT_LT(maxTurn(*smoothed), 20.0 * M_PI / 180.0);
  }
}

TEST(SmoothPath, EndpointsInsideClearanceDoNotHook)
{
  // 壁（y < 5）と平行に、0.7m（膨らみの外、余裕を取りたい距離より内）の位置を走る。端点は動かないので、
  // 端点の近くでは取りたい距離を少しずつ上げて、端点で急に曲がらないようにする
  auto grid = fieldGrid();
  fillBox(grid, 0.0, 0.0, 20.0, 5.0);
  const auto field = computeDistanceField(grid);
  const double wall_y = 4.95;
  const Points polyline{{3.0, wall_y + 0.7}, {8.0, wall_y + 0.7}};
  const SmoothingParams params;
  const auto smoothed = smoothPath(polyline, field, kHardRadiusM, params);
  ASSERT_TRUE(smoothed.has_value());
  EXPECT_EQ(smoothed->front(), polyline.front());
  EXPECT_EQ(smoothed->back(), polyline.back());
  EXPECT_GE(minDistance(field, *smoothed), kHardRadiusM);
  EXPECT_LT(maxTurn(*smoothed), 12.0 * M_PI / 180.0);
  // 最初の区間の向きも、元の向き（x 方向）から大きくずれない
  const double first_heading = std::atan2(
    (*smoothed)[1].second - (*smoothed)[0].second, (*smoothed)[1].first - (*smoothed)[0].first);
  EXPECT_LT(std::abs(first_heading), 20.0 * M_PI / 180.0);
  // 端点から離れた中央では、余裕を取った距離まで離れる
  const auto & middle = (*smoothed)[smoothed->size() / 2U];
  EXPECT_NEAR(middle.second - wall_y, kHardRadiusM + params.extra_clearance_m, 0.02);
}

TEST(SmoothPath, DisabledReturnsPolylineUnchanged)
{
  auto grid = fieldGrid();
  fillBox(grid, 8.0, 8.0, 12.0, 12.0);
  const auto field = computeDistanceField(grid);
  const Points polyline = plannedPolyline(inflate(field, kHardRadiusM), {4.0, 10.5}, {16.0, 10.5});
  SmoothingParams params;
  params.enabled = false;
  const auto smoothed = smoothPath(polyline, field, kHardRadiusM, params);
  ASSERT_TRUE(smoothed.has_value());
  EXPECT_EQ(*smoothed, polyline);
}

TEST(SmoothPath, ReturnsShortPolylineAsIs)
{
  const auto field = computeDistanceField(fieldGrid());
  const Points single{{5.0, 5.0}};
  const auto smoothed = smoothPath(single, field, kHardRadiusM, SmoothingParams{});
  ASSERT_TRUE(smoothed.has_value());
  EXPECT_EQ(*smoothed, single);
}

TEST(SmoothPath, RejectsInvalidInput)
{
  const auto field = computeDistanceField(fieldGrid());
  const Points polyline{{3.0, 10.0}, {17.0, 10.0}};
  SmoothingParams zero_spacing;
  zero_spacing.spacing_m = 0.0;
  EXPECT_THROW(smoothPath(polyline, field, kHardRadiusM, zero_spacing), std::invalid_argument);
  SmoothingParams heavy;
  heavy.smooth_weight = 1.5;
  EXPECT_THROW(smoothPath(polyline, field, kHardRadiusM, heavy), std::invalid_argument);
  SmoothingParams negative;
  negative.iterations = -1;
  EXPECT_THROW(smoothPath(polyline, field, kHardRadiusM, negative), std::invalid_argument);
  const Points not_finite{{3.0, 10.0}, {std::nan(""), 10.0}};
  EXPECT_THROW(
    smoothPath(not_finite, field, kHardRadiusM, SmoothingParams{}), std::invalid_argument);
}
}  // namespace
