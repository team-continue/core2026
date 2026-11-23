#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "core_planning/waypoints.hpp"

namespace
{
using core_planning::Waypoint;

TEST(Waypoints, DropsLeadingReachedWaypointsInOrder)
{
  std::vector<Waypoint> waypoints{{1.0, 0.0}, {2.0, 0.0}, {0.1, 0.0}};
  // 機体の近くに 3 番目の点があっても、先頭の点に入るまでは除かない
  EXPECT_EQ(core_planning::dropReachedWaypoints(waypoints, 0.0, 0.0, 0.5), 0U);
  EXPECT_EQ(waypoints.size(), 3U);
  EXPECT_EQ(core_planning::dropReachedWaypoints(waypoints, 0.8, 0.0, 0.5), 1U);
  ASSERT_EQ(waypoints.size(), 2U);
  EXPECT_DOUBLE_EQ(waypoints.front().x, 2.0);
}

TEST(Waypoints, DropsSeveralWhenPointsOverlap)
{
  std::vector<Waypoint> waypoints{{1.0, 0.0}, {1.2, 0.0}, {3.0, 0.0}};
  EXPECT_EQ(core_planning::dropReachedWaypoints(waypoints, 1.1, 0.0, 0.5), 2U);
  ASSERT_EQ(waypoints.size(), 1U);
  EXPECT_DOUBLE_EQ(waypoints.front().x, 3.0);
  EXPECT_EQ(core_planning::dropReachedWaypoints(waypoints, 3.0, 0.0, 0.5), 1U);
  EXPECT_TRUE(waypoints.empty());
  EXPECT_EQ(core_planning::dropReachedWaypoints(waypoints, 3.0, 0.0, 0.5), 0U);
}

TEST(Waypoints, RouteDistanceGoesThroughWaypoints)
{
  EXPECT_DOUBLE_EQ(core_planning::routeDistance(0.0, 0.0, {}, 3.0, 4.0), 5.0);
  EXPECT_DOUBLE_EQ(core_planning::routeDistance(0.0, 0.0, {{3.0, 0.0}}, 3.0, 4.0), 7.0);
}

TEST(Waypoints, JoinSegmentsMergesSharedEnds)
{
  const std::vector<std::vector<std::pair<double, double>>> segments{
    {{0.0, 0.0}, {1.0, 0.0}},
    {{1.0, 0.0}, {1.0, 1.0}},
    {{1.0, 1.0}, {2.0, 1.0}},
  };
  const auto points = core_planning::joinSegments(segments);
  const std::vector<std::pair<double, double>> expected{
    {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {2.0, 1.0}};
  EXPECT_EQ(points, expected);
}

TEST(Waypoints, MovesWaypointOutOfObstacleMargin)
{
  // x ≥ 0 が壁。壁までの距離は -x
  const auto distance = [](const double x, const double) {return std::max(-x, 0.0);};
  // 余裕があればそのまま
  const auto far = core_planning::withClearance(Waypoint{-2.0, 1.0}, 1.0, 2.0, 0.05, distance);
  ASSERT_TRUE(far.has_value());
  EXPECT_DOUBLE_EQ(far->x, -2.0);
  // 壁から 0.45m の点は、余裕 1.0m の所（x = -1.0）まで壁から離す。y は変えない
  const auto near = core_planning::withClearance(Waypoint{-0.45, 1.0}, 1.0, 2.0, 0.05, distance);
  ASSERT_TRUE(near.has_value());
  EXPECT_NEAR(near->x, -1.0, 0.051);
  EXPECT_NEAR(near->y, 1.0, 1e-9);
  // 探す範囲に余裕のある所がなければ nullopt
  EXPECT_FALSE(core_planning::withClearance(Waypoint{3.0, 0.0}, 1.0, 2.0, 0.05, distance).has_value());
}
}  // namespace
