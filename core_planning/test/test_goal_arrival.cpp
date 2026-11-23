#include <gtest/gtest.h>

#include <optional>

#include "core_planning/goal_arrival.hpp"

namespace
{
using core_planning::ArrivalPoint;
using core_planning::arrivedNearTarget;

TEST(GoalArrival, AcceptsNearTarget)
{
  EXPECT_TRUE(arrivedNearTarget(0.5, 0.0, ArrivalPoint{0.0, 0.0}, std::nullopt, 1.1));
  EXPECT_FALSE(arrivedNearTarget(1.5, 0.0, ArrivalPoint{0.0, 0.0}, std::nullopt, 1.1));
}

TEST(GoalArrival, AcceptsNearPathEndWhenGoalWasMoved)
{
  // sim の例: ゴール (-6, -1) は進入禁止ゾーンのそばで占有。経路は (-5.55, -0.85) まで。
  // 局所計画は固定物の余裕で (-5.02, -0.41) で着いたことにした（ゴールから 1.14m）
  const ArrivalPoint goal{-6.0, -1.0};
  const ArrivalPoint path_end{-5.55, -0.85};
  EXPECT_FALSE(arrivedNearTarget(-5.02, -0.41, goal, std::nullopt, 1.1));
  EXPECT_TRUE(arrivedNearTarget(-5.02, -0.41, goal, path_end, 1.1));
  // 経路の終わりからも離れていれば着いていない
  EXPECT_FALSE(arrivedNearTarget(-3.0, 1.0, goal, path_end, 1.1));
}
}  // namespace
