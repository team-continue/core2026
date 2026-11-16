#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "core_planning/footprint.hpp"
#include "core_planning/grid_map.hpp"
#include "core_planning/mppi.hpp"
#include "core_planning/track_motion.hpp"

namespace
{
using core_planning::chooseBackoffDirection;
using core_planning::computeDistanceField;
using core_planning::computeSpeedLimit;
using core_planning::DistanceField;
using core_planning::FieldBounds;
using core_planning::FootprintParams;
using core_planning::makeFieldGrid;
using core_planning::MppiCommand;
using core_planning::MppiController;
using core_planning::MppiParams;
using core_planning::MppiState;
using core_planning::ObjectKind;
using core_planning::Point2;
using core_planning::PredictedObject;
using core_planning::ReferencePath;
using core_planning::SE2;
using core_planning::signedDistanceToRectangle;
using core_planning::SpeedLimitParams;
using core_planning::StuckMonitor;
using core_planning::StuckParams;
using core_planning::StuckState;

// 向き 0、既定の機体（0.8m 角）で速度の上限を求める
double speedLimitAt(
  const SpeedLimitParams & params, const std::vector<PredictedObject> & objects, const double x,
  const double y, const double vx, const double vy, const double max_speed)
{
  MppiState state;
  state.x = x;
  state.y = y;
  state.vx = vx;
  state.vy = vy;
  return computeSpeedLimit(params, objects, state, FootprintParams{}, max_speed);
}

// 指令どおりに動く機体で閉ループを回す
MppiState step(const MppiState & state, const MppiCommand & command, const double dt)
{
  MppiState next = state;
  next.vx = command.vx;
  next.vy = command.vy;
  next.wz = command.wz;
  next.x += next.vx * dt;
  next.y += next.vy * dt;
  next.yaw += next.wz * dt;
  return next;
}

PredictedObject movingObject(const double x, const double y, const double vx, const double vy)
{
  PredictedObject object;
  object.x = x;
  object.y = y;
  object.vx = vx;
  object.vy = vy;
  object.sigma = 0.05;
  object.kind = ObjectKind::kRobot;
  return object;
}

// 向きの決め方を変える前の動き。ゴールの向きへ回り（なければ保ち）、見えない側の速さを抑えない
MppiParams previousHeadingParams()
{
  MppiParams params;
  params.heading.mode = core_planning::HeadingMode::kGoal;
  params.blind_side.enabled = false;
  return params;
}

double footprintGap(const MppiState & state, const PredictedObject & object, const double radius)
{
  const double c = std::cos(state.yaw);
  const double s = std::sin(state.yaw);
  const double dx = object.x - state.x;
  const double dy = object.y - state.y;
  return signedDistanceToRectangle(c * dx + s * dy, -s * dx + c * dy, 0.4, 0.4) - radius;
}

TEST(ReferencePath, ProjectsOntoPolyline)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{10.0, 0.0}, Point2{10.0, 10.0}});
  EXPECT_NEAR(path.length(), 20.0, 1e-9);
  EXPECT_NEAR(path.project(3.0, 1.0), 3.0, 1e-9);
  EXPECT_NEAR(path.project(11.0, 4.0), 14.0, 1e-9);
  EXPECT_NEAR(path.pointAt(15.0).y, 5.0, 1e-9);
  EXPECT_NEAR(path.directionAt(15.0).y, 1.0, 1e-9);
  EXPECT_NEAR(path.remaining(10.0, 5.0), 5.0, 1e-9);
}

TEST(Mppi, FollowsPathWithoutObstaclesAndStopsAtGoal)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{10.0, 0.0}});
  const MppiParams params;
  MppiController controller;
  MppiState state;
  double max_speed = 0.0;
  double max_lateral = 0.0;
  for (int i = 0; i < 80; ++i) {
    const auto command = controller.solve(state, path, {}, nullptr, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    max_speed = std::max(max_speed, std::hypot(state.vx, state.vy));
    max_lateral = std::max(max_lateral, std::abs(state.y));
  }
  EXPECT_NEAR(state.x, 10.0, 0.2);
  EXPECT_LT(max_lateral, 0.3);
  EXPECT_GT(max_speed, 2.5);
  EXPECT_LE(max_speed, 3.0 + 1e-6);
  EXPECT_LT(std::hypot(state.vx, state.vy), 0.2);
}

TEST(Mppi, KeepsTrajectoryOfSelectedControls)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{10.0, 0.0}});
  const MppiParams params;
  MppiController controller;
  MppiState state;
  state.vx = 1.0;
  const auto command = controller.solve(state, path, {}, nullptr, SE2{}, params, 3.0);
  const auto & trajectory = controller.lastTrajectory();
  ASSERT_EQ(trajectory.size(), static_cast<std::size_t>(params.horizon_steps) + 1U);
  EXPECT_DOUBLE_EQ(trajectory.front().x, state.x);
  EXPECT_DOUBLE_EQ(trajectory.front().y, state.y);
  // 最初のステップは、出した指令と同じ加速度の上限で動く
  EXPECT_NEAR(trajectory[1].vx, command.vx, 1e-9);
  EXPECT_GT(trajectory.back().x, 1.0);
  EXPECT_LT(std::abs(trajectory.back().y), 0.3);
  for (std::size_t k = 1; k < trajectory.size(); ++k) {
    const double dv = std::hypot(
      trajectory[k].vx - trajectory[k - 1U].vx, trajectory[k].vy - trajectory[k - 1U].vy);
    EXPECT_LE(dv, params.max_accel_mps2 * params.dt + 1e-9);
  }
  controller.reset();
  EXPECT_TRUE(controller.lastTrajectory().empty());
}

TEST(Mppi, RespectsAccelerationLimit)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{10.0, 0.0}});
  const MppiParams params;
  MppiController controller;
  const auto command = controller.solve(MppiState{}, path, {}, nullptr, SE2{}, params, 3.0);
  EXPECT_LE(std::hypot(command.vx, command.vy), params.max_accel_mps2 * params.dt + 1e-9);
}

TEST(Mppi, AvoidsOncomingRobot)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{30.0, 0.0}});
  const MppiParams params;
  MppiController controller;
  MppiState state;
  auto object = movingObject(12.0, 0.0, -3.0, 0.0);
  double min_gap = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 60; ++i) {
    const auto command = controller.solve(state, path, {object}, nullptr, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    object.x += object.vx * params.dt;
    min_gap = std::min(min_gap, footprintGap(state, object, 0.4));
  }
  EXPECT_GT(min_gap, 0.0);
  // 相手とすれ違った後も前に進んでいる
  EXPECT_GT(state.x, object.x);
}

// 向かってくる相手との距離が 6m になるまでに、横へ動いた量の平均を返す
double meanOffsetBeforeOncomingRobot(const double weight_collision_course)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{30.0, 0.0}});
  MppiParams params;
  params.weight_collision_course = weight_collision_course;
  double offset_sum = 0.0;
  constexpr int kSeeds = 10;
  for (int seed = 0; seed < kSeeds; ++seed) {
    MppiController controller(static_cast<std::uint32_t>(seed));
    MppiState state;
    state.vx = 3.0;
    auto object = movingObject(14.0, 0.0, -3.0, 0.0);
    while (object.x - state.x > 6.0) {
      const auto command = controller.solve(state, path, {object}, nullptr, SE2{}, params, 3.0);
      state = step(state, command, params.dt);
      object.x += object.vx * params.dt;
    }
    offset_sum += std::abs(state.y);
  }
  return offset_sum / kSeeds;
}

TEST(Mppi, StartsAvoidingOncomingRobotEarly)
{
  // ぎりぎりまで経路を追うと、相手が加速して予測が外れたときに避けきれない
  EXPECT_GT(meanOffsetBeforeOncomingRobot(10.0), 0.4);
  EXPECT_GT(meanOffsetBeforeOncomingRobot(10.0), meanOffsetBeforeOncomingRobot(0.0) + 0.1);
}

TEST(Mppi, KeepsHeadingWhileAvoidingWithoutGoalYaw)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{30.0, 0.0}});
  // 前の動き（heading.mode: goal、見えない側の上限なし）。既定の travel では進む向き（0）へ回る
  const MppiParams params = previousHeadingParams();
  MppiController controller;
  MppiState state;
  state.yaw = 0.3;
  auto object = movingObject(12.0, 0.0, -3.0, 0.0);
  for (int i = 0; i < 60; ++i) {
    const auto command = controller.solve(state, path, {object}, nullptr, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    object.x += object.vx * params.dt;
    EXPECT_DOUBLE_EQ(command.wz, 0.0);
  }
  EXPECT_DOUBLE_EQ(state.yaw, 0.3);
}

TEST(Mppi, TurnsTowardGoalYawWithinLimits)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{30.0, 0.0}}, 2.0);
  // 前の動き（heading.mode: goal、見えない側の上限なし）。既定の travel ではゴールの 1m 手前まで
  // 進む向き（0）を保ち、向き 2.0 では進む向きが見えない側になって 0.5m/s までに抑えられる
  const MppiParams params = previousHeadingParams();
  MppiController controller;
  MppiState state;
  for (int i = 0; i < 50; ++i) {
    const MppiState previous = state;
    const auto command = controller.solve(state, path, {}, nullptr, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    EXPECT_LE(std::abs(command.wz), params.max_angular_speed_radps + 1e-9);
    EXPECT_LE(
      std::abs(command.wz - previous.wz), params.max_angular_accel_radps2 * params.dt + 1e-9);
  }
  EXPECT_NEAR(state.yaw, 2.0, 0.05);
  // 向きを変えている間も経路を進む
  EXPECT_GT(state.x, 5.0);
}

TEST(CollisionCourse, DetectsOnlyMovingObjectsThatPassNearby)
{
  const MppiParams params;
  const MppiState state;
  EXPECT_TRUE(core_planning::isOnCollisionCourse(state, {movingObject(4.0, 0.3, -3.0, 0.0)}, params));
  // 横に十分離れて通る
  EXPECT_FALSE(core_planning::isOnCollisionCourse(state, {movingObject(4.0, 3.0, -3.0, 0.0)}, params));
  // 遠ざかる
  EXPECT_FALSE(core_planning::isOnCollisionCourse(state, {movingObject(4.0, 0.0, 3.0, 0.0)}, params));
  // 止まっている
  EXPECT_FALSE(core_planning::isOnCollisionCourse(state, {movingObject(1.5, 0.0, 0.0, 0.0)}, params));
  // 最接近までの時間が長い
  EXPECT_FALSE(core_planning::isOnCollisionCourse(state, {movingObject(9.0, 0.0, -3.0, 0.0)}, params));
  // 自機が動いていて、相手に追いつく
  MppiState moving;
  moving.vx = 3.0;
  EXPECT_TRUE(core_planning::isOnCollisionCourse(moving, {movingObject(3.0, 0.0, 1.0, 0.0)}, params));
}

TEST(Mppi, StationaryRobotDodgesWithHoldReference)
{
  // 元の動き（自機が近づいていない相手も避ける）で、その場を基準に避けられることを確かめる。
  // 既定では止まっている自機は避けない（Responsibility.StationaryRobotDoesNotDodgeApproachingRobot）
  MppiParams params;
  params.avoid_being_hit = true;
  params.merge.enabled = false;
  MppiController controller;
  MppiState state;
  auto object = movingObject(8.0, 0.2, -3.0, 0.0);
  double min_gap = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 50; ++i) {
    const ReferencePath here({Point2{state.x, state.y}, Point2{state.x, state.y}});
    const auto command = controller.solve(state, here, {object}, nullptr, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    object.x += object.vx * params.dt;
    min_gap = std::min(min_gap, footprintGap(state, object, 0.4));
  }
  EXPECT_GT(min_gap, 0.0);
}

TEST(Mppi, StopsBehindBlockingRobotInsteadOfColliding)
{
  // 柵に挟まれた幅の狭い通路をふさぐ相手には、当たらず止まる
  const FieldBounds bounds{0.0, 30.0, -1.0, 1.0};
  const auto grid = makeFieldGrid(bounds, 0.1);
  const auto field = computeDistanceField(grid);
  const ReferencePath path({Point2{1.0, 0.0}, Point2{29.0, 0.0}});
  const MppiParams params;
  MppiController controller;
  MppiState state;
  state.x = 1.0;
  const auto object = movingObject(8.0, 0.0, 0.0, 0.0);
  double min_gap = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 80; ++i) {
    const auto command = controller.solve(state, path, {object}, &field, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    min_gap = std::min(min_gap, footprintGap(state, object, 0.4));
  }
  EXPECT_GT(min_gap, 0.0);
  EXPECT_LT(state.x, 8.0);
}

TEST(Mppi, StaysInsideFenceDistanceField)
{
  // 柵の近くを通る経路でも、機体の角が柵を越えない
  const FieldBounds bounds{-12.0, 12.0, -7.5, 7.5};
  const auto grid = makeFieldGrid(bounds, 0.1);
  const auto field = computeDistanceField(grid);
  const ReferencePath path({Point2{-10.0, 7.3}, Point2{10.0, 7.3}});
  const MppiParams params;
  MppiController controller;
  MppiState state;
  state.x = -10.0;
  state.y = 6.5;
  double max_y = 0.0;
  for (int i = 0; i < 120; ++i) {
    const auto command = controller.solve(state, path, {}, &field, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    max_y = std::max(max_y, state.y);
  }
  EXPECT_LT(max_y + 0.4, bounds.y_max);
  EXPECT_GT(state.x, 5.0);
}

TEST(Mppi, ConvertsMapFieldThroughOdomOffset)
{
  // odom が map から y 方向に 5m ずれていても、map の柵を避ける
  const FieldBounds bounds{-12.0, 12.0, -7.5, 7.5};
  const auto grid = makeFieldGrid(bounds, 0.1);
  const auto field = computeDistanceField(grid);
  const SE2 map_from_odom{0.0, 5.0, 0.0};
  const ReferencePath path({Point2{-10.0, 2.3}, Point2{10.0, 2.3}});
  const MppiParams params;
  MppiController controller;
  MppiState state;
  state.x = -10.0;
  state.y = 1.5;
  double max_map_y = 0.0;
  for (int i = 0; i < 120; ++i) {
    const auto command = controller.solve(state, path, {}, &field, map_from_odom, params, 3.0);
    state = step(state, command, params.dt);
    max_map_y = std::max(max_map_y, state.y + 5.0);
  }
  EXPECT_LT(max_map_y + 0.4, bounds.y_max);
}

// 正面から来る相手をどちらに避けたかを、すれ違うまでの y の符号で返す
double passingOffset(const bool side_right)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{30.0, 0.0}});
  MppiParams params;
  params.passing.enabled = true;
  params.passing.side_right = side_right;
  params.passing.weight = 3.0;
  MppiController controller;
  MppiState state;
  auto object = movingObject(12.0, 0.0, -2.0, 0.0);
  double offset = 0.0;
  for (int i = 0; i < 60; ++i) {
    const auto command = controller.solve(state, path, {object}, nullptr, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    object.x += object.vx * params.dt;
    if (std::abs(state.y) > std::abs(offset)) {
      offset = state.y;
    }
  }
  return offset;
}

TEST(Mppi, PassingRuleChoosesSide)
{
  // +x へ進むとき、右は -y 側
  EXPECT_LT(passingOffset(true), -0.2);
  EXPECT_GT(passingOffset(false), 0.2);
}

TEST(Mppi, PassingRuleYieldsWhenNoRoomOnThatSide)
{
  // 柵の 1.5m 手前を相手が走る。規則の側（柵の側）には機体が入る余裕がない
  const FieldBounds bounds{-12.0, 12.0, -7.5, 7.5};
  const auto grid = makeFieldGrid(bounds, 0.1);
  const auto field = computeDistanceField(grid);
  const ReferencePath path({Point2{10.0, 6.0}, Point2{-10.0, 6.0}});
  const MppiParams params;
  int fence_side = 0;
  constexpr int kSeeds = 10;
  for (int seed = 0; seed < kSeeds; ++seed) {
    MppiController controller(static_cast<std::uint32_t>(seed));
    MppiState state;
    state.x = 10.0;
    state.y = 6.0;
    state.vx = -3.0;
    auto object = movingObject(-4.0, 6.0, 3.0, 0.0);
    double max_y = 0.0;
    while (object.x < state.x) {
      const auto command = controller.solve(state, path, {object}, &field, SE2{}, params, 3.0);
      state = step(state, command, params.dt);
      object.x += object.vx * params.dt;
      max_y = std::max(max_y, state.y);
    }
    EXPECT_LT(max_y + 0.4, bounds.y_max);
    fence_side += state.y > object.y ? 1 : 0;
  }
  EXPECT_LE(fence_side, 1);
}

TEST(Mppi, IsDeterministicWithSameSeed)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{10.0, 0.0}});
  const MppiParams params;
  MppiController first(7U);
  MppiController second(7U);
  const auto a = first.solve(MppiState{}, path, {}, nullptr, SE2{}, params, 3.0);
  const auto b = second.solve(MppiState{}, path, {}, nullptr, SE2{}, params, 3.0);
  EXPECT_DOUBLE_EQ(a.vx, b.vx);
  EXPECT_DOUBLE_EQ(a.vy, b.vy);
}

TEST(Mppi, EmptyPathBrakes)
{
  MppiState state;
  state.vx = 2.0;
  const MppiParams params;
  MppiController controller;
  const auto command = controller.solve(state, ReferencePath{}, {}, nullptr, SE2{}, params, 3.0);
  EXPECT_NEAR(command.vx, 2.0 - params.max_accel_mps2 * params.dt, 1e-9);
}

TEST(SpeedLimit, DisabledReturnsMaxSpeed)
{
  SpeedLimitParams params;
  params.enabled = false;
  const std::vector<PredictedObject> objects{movingObject(1.5, 0.0, 0.0, 0.0)};
  EXPECT_DOUBLE_EQ(speedLimitAt(params, objects, 0.0, 0.0, 3.0, 0.0, 3.0), 3.0);
}

TEST(SpeedLimit, SlowsForObjectAheadOnly)
{
  SpeedLimitParams params;
  params.density_speed_scale = 1.0;
  const std::vector<PredictedObject> ahead{movingObject(2.5, 0.0, 0.0, 0.0)};
  const std::vector<PredictedObject> behind{movingObject(-2.5, 0.0, 0.0, 0.0)};
  const double limit_ahead = speedLimitAt(params, ahead, 0.0, 0.0, 3.0, 0.0, 3.0);
  const double limit_behind = speedLimitAt(params, behind, 0.0, 0.0, 3.0, 0.0, 3.0);
  EXPECT_LT(limit_ahead, 3.0);
  EXPECT_DOUBLE_EQ(limit_behind, 3.0);
  // 止まれる距離の式を満たす
  const double available = 2.5 - params.stopping_margin_m;
  EXPECT_NEAR(
    limit_ahead * params.reaction_time_s +
    limit_ahead * limit_ahead / (2.0 * params.max_decel_mps2), available, 1e-6);
}

TEST(SpeedLimit, SlowsWhenCrowded)
{
  SpeedLimitParams params;
  params.density_speed_scale = 0.7;
  // 真横の相手なので、見通しの制限はかからない
  const std::vector<PredictedObject> one{movingObject(0.0, 2.0, 0.0, 0.0)};
  const std::vector<PredictedObject> two{
    movingObject(0.0, 2.0, 0.0, 0.0), movingObject(0.0, -2.0, 0.0, 0.0)};
  EXPECT_NEAR(speedLimitAt(params, one, 0.0, 0.0, 3.0, 0.0, 3.0), 3.0 * 0.7, 1e-9);
  EXPECT_NEAR(speedLimitAt(params, two, 0.0, 0.0, 3.0, 0.0, 3.0), 3.0 * 0.49, 1e-9);
}

TEST(SpeedLimit, NeverBelowMinimum)
{
  SpeedLimitParams params;
  const std::vector<PredictedObject> objects{movingObject(0.9, 0.0, 0.0, 0.0)};
  EXPECT_DOUBLE_EQ(
    speedLimitAt(params, objects, 0.0, 0.0, 3.0, 0.0, 3.0), params.min_speed_mps);
}

TEST(StuckMonitor, WaitsThenBacksOffThenFails)
{
  StuckParams params;
  params.no_progress_timeout_s = 2.0;
  params.wait_s = 1.0;
  params.backoff_distance_m = 0.5;
  params.max_retries = 1;
  StuckMonitor monitor;
  EXPECT_EQ(monitor.update(0.0, 10.0, 0.0, 0.0, params), StuckState::kDriving);
  EXPECT_EQ(monitor.update(1.9, 10.0, 0.0, 0.0, params), StuckState::kDriving);
  EXPECT_EQ(monitor.update(2.0, 10.0, 0.0, 0.0, params), StuckState::kWait);
  EXPECT_EQ(monitor.update(2.5, 10.0, 0.0, 0.0, params), StuckState::kWait);
  EXPECT_EQ(monitor.update(3.0, 10.0, 0.0, 0.0, params), StuckState::kBackoff);
  EXPECT_EQ(monitor.retries(), 1);
  EXPECT_EQ(monitor.update(3.1, 10.0, 0.2, 0.0, params), StuckState::kBackoff);
  EXPECT_EQ(monitor.update(3.5, 10.0, 0.6, 0.0, params), StuckState::kDriving);
  EXPECT_EQ(monitor.update(5.4, 10.0, 0.6, 0.0, params), StuckState::kDriving);
  EXPECT_EQ(monitor.update(5.5, 10.0, 0.6, 0.0, params), StuckState::kWait);
  EXPECT_EQ(monitor.update(6.5, 10.0, 0.6, 0.0, params), StuckState::kFailed);
  EXPECT_EQ(monitor.update(100.0, 1.0, 0.6, 0.0, params), StuckState::kFailed);
  monitor.reset();
  EXPECT_EQ(monitor.update(101.0, 1.0, 0.6, 0.0, params), StuckState::kDriving);
  EXPECT_EQ(monitor.retries(), 0);
}

TEST(StuckMonitor, AcceptsHardToReachGoalWhenStuckNearIt)
{
  StuckParams params;
  params.no_progress_timeout_s = 2.0;
  params.near_goal_accept_m = 1.0;
  StuckMonitor monitor;
  // ふつうのゴールでは、近くで進めなくても着いたことにしない（待って計画し直す）
  EXPECT_EQ(monitor.update(0.0, 0.3, 0.0, 0.0, params), StuckState::kDriving);
  EXPECT_EQ(monitor.update(2.0, 0.3, 0.0, 0.0, params), StuckState::kWait);
  // 固定物に近いゴールで、0.6m 手前で進めないなら、いちばん近くまで来たとして着いたことにする
  monitor.reset();
  monitor.setGoalHardToReach(true);
  EXPECT_EQ(monitor.update(10.0, 0.6, 0.0, 0.0, params), StuckState::kDriving);
  EXPECT_EQ(monitor.update(12.0, 0.6, 0.0, 0.0, params), StuckState::kNearGoal);
  EXPECT_EQ(monitor.update(13.0, 0.6, 0.0, 0.0, params), StuckState::kNearGoal);
  EXPECT_EQ(monitor.retries(), 0);
  // 到達が難しいゴールでも、遠ければ待つ
  monitor.reset();
  EXPECT_EQ(monitor.update(20.0, 1.5, 0.0, 0.0, params), StuckState::kDriving);
  EXPECT_EQ(monitor.update(22.0, 1.5, 0.0, 0.0, params), StuckState::kWait);
}

TEST(StuckMonitor, WaitsForReplanInsteadOfBackingOff)
{
  StuckParams params;
  params.no_progress_timeout_s = 2.0;
  params.wait_s = 1.0;
  params.backoff_distance_m = 0.0;
  params.replan_timeout_s = 3.0;
  params.max_retries = 2;
  StuckMonitor monitor;
  EXPECT_EQ(monitor.update(0.0, 10.0, 0.0, 0.0, params), StuckState::kDriving);
  EXPECT_EQ(monitor.update(2.0, 10.0, 0.0, 0.0, params), StuckState::kWait);
  // 行き詰まる前に受けた経路では、待ち終えない
  monitor.planReceived(1.5);
  EXPECT_EQ(monitor.update(3.0, 10.0, 0.0, 0.0, params), StuckState::kWait);
  // 計画し直した経路を受けたら、下がらずにその経路で走行に戻る
  monitor.planReceived(3.2);
  EXPECT_EQ(monitor.update(3.3, 10.0, 0.0, 0.0, params), StuckState::kDriving);
  EXPECT_EQ(monitor.retries(), 1);
  // 経路が届かなくても、上限の時間が過ぎたら今の経路で進む
  EXPECT_EQ(monitor.update(5.3, 10.0, 0.0, 0.0, params), StuckState::kWait);
  EXPECT_EQ(monitor.update(7.0, 10.0, 0.0, 0.0, params), StuckState::kWait);
  EXPECT_EQ(monitor.update(8.3, 10.0, 0.0, 0.0, params), StuckState::kDriving);
  EXPECT_EQ(monitor.retries(), 2);
  // 回数を使い切ったら FAILED で止まる
  EXPECT_EQ(monitor.update(10.3, 10.0, 0.0, 0.0, params), StuckState::kWait);
  monitor.planReceived(10.5);
  EXPECT_EQ(monitor.update(11.3, 10.0, 0.0, 0.0, params), StuckState::kFailed);
}

TEST(StuckMonitor, GivesUpWhenNotGettingCloserForLong)
{
  StuckParams params;
  params.no_progress_timeout_s = 2.0;
  params.moving_distance_m = 0.5;
  params.moving_timeout_s = 100.0;
  params.give_up_s = 30.0;
  StuckMonitor monitor;
  EXPECT_EQ(monitor.update(0.0, 10.0, 0.0, 0.0, params), StuckState::kDriving);
  // 動き回っているので行き詰まりとは数えないが、ゴールには近づかない
  for (int i = 1; i < 30; ++i) {
    EXPECT_EQ(
      monitor.update(i, 10.0, (i % 2 == 0) ? 0.0 : 1.0, 0.0, params), StuckState::kDriving) << i;
  }
  EXPECT_EQ(monitor.update(30.0, 10.0, 0.0, 0.0, params), StuckState::kFailed);
  // 近づいていれば、あきらめない
  monitor.reset();
  for (int i = 0; i <= 40; ++i) {
    EXPECT_EQ(monitor.update(100.0 + i, 20.0 - 0.3 * i, 0.0, 0.3 * i, params), StuckState::kDriving);
  }
}

TEST(StuckMonitor, DoesNotCountTimeWhileEvading)
{
  StuckParams params;
  StuckMonitor monitor;
  EXPECT_EQ(monitor.update(0.0, 10.0, 0.0, 0.0, params), StuckState::kDriving);
  for (double t = 0.5; t < 5.0; t += 0.5) {
    monitor.holdProgress(t, 10.0);
    EXPECT_EQ(monitor.update(t, 10.0, 0.0, 0.0, params), StuckState::kDriving);
  }
  // 避け終わった後は、そこから数える
  EXPECT_EQ(monitor.update(6.0, 10.0, 0.0, 0.0, params), StuckState::kDriving);
  EXPECT_EQ(monitor.update(7.0, 10.0, 0.0, 0.0, params), StuckState::kWait);
}

TEST(Mppi, NeverExceedsSpeedLimitWhileEvading)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{30.0, 0.0}});
  const MppiParams params;
  MppiController controller;
  MppiState state;
  state.vx = 1.0;
  auto object = movingObject(6.0, 0.0, -3.0, 0.0);
  for (int i = 0; i < 30; ++i) {
    const auto command = controller.solve(state, path, {object}, nullptr, SE2{}, params, 1.0);
    EXPECT_LE(std::hypot(command.vx, command.vy), 1.0 + 1e-9);
    state = step(state, command, params.dt);
    object.x += object.vx * params.dt;
  }
}

TEST(StuckMonitor, ProgressResetsTimer)
{
  const StuckParams params;
  StuckMonitor monitor;
  double distance = 10.0;
  for (int i = 0; i < 100; ++i) {
    distance -= 0.1;
    EXPECT_EQ(
      monitor.update(0.1 * i, distance, 0.0, 0.0, params), StuckState::kDriving);
  }
}

TEST(StuckMonitor, BackoffTimesOutWhenBlocked)
{
  const StuckParams params;
  StuckMonitor monitor;
  monitor.update(0.0, 10.0, 0.0, 0.0, params);
  monitor.update(2.0, 10.0, 0.0, 0.0, params);
  EXPECT_EQ(monitor.update(3.0, 10.0, 0.0, 0.0, params), StuckState::kBackoff);
  EXPECT_EQ(monitor.update(6.1, 10.0, 0.0, 0.0, params), StuckState::kDriving);
}

TEST(StuckMonitor, DisabledAlwaysDriving)
{
  StuckParams params;
  params.enabled = false;
  StuckMonitor monitor;
  EXPECT_EQ(monitor.update(0.0, 10.0, 0.0, 0.0, params), StuckState::kDriving);
  EXPECT_EQ(monitor.update(100.0, 10.0, 0.0, 0.0, params), StuckState::kDriving);
}

TEST(BackoffDirection, MovesAwayFromObjectAhead)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{10.0, 0.0}});
  const MppiParams params;
  MppiState state;
  const auto direction = chooseBackoffDirection(
    state, path, {movingObject(1.0, 0.0, 0.0, 0.0)}, nullptr, SE2{}, params);
  EXPECT_LT(direction.x, 0.0);
}

TEST(SE2, InverseUndoesApply)
{
  const SE2 transform{1.0, -2.0, 0.7};
  const auto point = transform.apply(3.0, 4.0);
  const auto back = transform.inverse().apply(point.x, point.y);
  EXPECT_NEAR(back.x, 3.0, 1e-9);
  EXPECT_NEAR(back.y, 4.0, 1e-9);
}

// 責任に応じた回避と合流の規則を使わない、元の動きの設定
MppiParams originalParams()
{
  MppiParams params;
  params.avoid_being_hit = true;
  params.merge.enabled = false;
  return params;
}

PredictedObject humanObject(const double x, const double y, const double vx, const double vy)
{
  PredictedObject object = movingObject(x, y, vx, vy);
  object.kind = ObjectKind::kHuman;
  return object;
}

// 自機の速度の、相手の中心の方向の成分
double approachSpeed(const MppiState & state, const PredictedObject & object)
{
  const double dx = object.x - state.x;
  const double dy = object.y - state.y;
  const double distance = std::hypot(dx, dy);
  return distance > 1e-9 ? (state.vx * dx + state.vy * dy) / distance : 0.0;
}

// 2 台の中心を結ぶ線の垂直二等分線から、自機の機体（余裕 0.1m を含む）までの距離。越えていれば負
double bisectorClearance(const MppiState & state, const PredictedObject & object)
{
  const double dx = object.x - state.x;
  const double dy = object.y - state.y;
  const double distance = std::hypot(dx, dy);
  const double c = std::cos(state.yaw);
  const double s = std::sin(state.yaw);
  const double local_x = (c * dx + s * dy) / distance;
  const double local_y = (-s * dx + c * dy) / distance;
  return 0.5 * distance - (0.4 * std::abs(local_x) + 0.4 * std::abs(local_y) + 0.1);
}

struct ClosedLoopResult
{
  double min_speed{std::numeric_limits<double>::infinity()};
  double min_gap{std::numeric_limits<double>::infinity()};
  double max_abs_y{0.0};
  // 自機が相手へ近づく向きに動いているステップだけを見た値
  double min_bisector_approaching{std::numeric_limits<double>::infinity()};
  double min_center_approaching{std::numeric_limits<double>::infinity()};
  // 自機が近づく向きに動いていて重なったステップの数（自機が原因の接触）
  int own_fault_steps{0};
  // 自機が x = cross_x を越えた時刻。越えなければ負
  double cross_time_s{-1.0};
};

// +x へ 3m/s で進む自機と、等速の相手 1 台で閉ループを回す
ClosedLoopResult runAlongX(
  const MppiParams & params, PredictedObject object, const int steps, const double cross_x,
  const std::uint32_t seed, const double radius = 0.4)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{40.0, 0.0}});
  MppiController controller(seed);
  MppiState state;
  state.vx = 3.0;
  ClosedLoopResult result;
  for (int i = 0; i < steps; ++i) {
    const auto command = controller.solve(state, path, {object}, nullptr, SE2{}, params, 3.0);
    const MppiState previous = state;
    state = step(state, command, params.dt);
    object.x += object.vx * params.dt;
    object.y += object.vy * params.dt;
    const double gap = footprintGap(state, object, radius);
    const double approach = approachSpeed(state, object);
    result.min_speed = std::min(result.min_speed, std::hypot(state.vx, state.vy));
    result.min_gap = std::min(result.min_gap, gap);
    result.max_abs_y = std::max(result.max_abs_y, std::abs(state.y));
    if (approach > 0.0) {
      result.min_bisector_approaching =
        std::min(result.min_bisector_approaching, bisectorClearance(state, object));
      result.min_center_approaching = std::min(
        result.min_center_approaching, std::hypot(object.x - state.x, object.y - state.y));
      result.own_fault_steps += gap <= 0.0 ? 1 : 0;
    }
    if (result.cross_time_s < 0.0 && previous.x < cross_x && state.x >= cross_x) {
      result.cross_time_s = static_cast<double>(i + 1) * params.dt;
    }
  }
  return result;
}

// 交点 (cross_x, 0) を、自機（3m/s）より lead_s 秒遅れて通る相手。heading_deg は相手の進む向き
PredictedObject mergingRobot(
  const double cross_x, const double heading_deg, const double speed, const double lead_s)
{
  const double heading = heading_deg * M_PI / 180.0;
  const double distance = speed * (cross_x / 3.0 + lead_s);
  return movingObject(
    cross_x - std::cos(heading) * distance, -std::sin(heading) * distance,
    std::cos(heading) * speed, std::sin(heading) * speed);
}

TEST(Responsibility, StationaryRobotDoesNotDodgeApproachingRobot)
{
  // 止まっている自機に当たるのは相手の責任なので、その場を基準にしても避けない
  const MppiParams params;
  MppiController controller;
  MppiState state;
  auto object = movingObject(8.0, 0.2, -3.0, 0.0);
  double max_displacement = 0.0;
  for (int i = 0; i < 50; ++i) {
    const ReferencePath here({Point2{state.x, state.y}, Point2{state.x, state.y}});
    const auto command = controller.solve(state, here, {object}, nullptr, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    object.x += object.vx * params.dt;
    max_displacement = std::max(max_displacement, std::hypot(state.x, state.y));
  }
  // 元の動きでは 1.5m ほど横へ逃げる（StationaryRobotDodgesWithHoldReference）
  EXPECT_LT(max_displacement, 0.3);
}

TEST(Responsibility, KeepsCourseWhenCaughtFromBehind)
{
  // 1.5m/s で進む自機に、後ろから 3m/s の相手が追いつく。当たるまで進路を変えない。
  // 見えない側の上限は切る。上限は後ろへ向かう系列を縮めるので乱数の結果が変わり、この 5 つの seed では
  // 横の動きの最大が 0.37m（seed 0）、平均 0.27m になる。200 の seed では上限の有無で平均 0.183 と
  // 0.183m、0.35m 以上は 13 と 8 回で、差はない
  for (const bool original : {false, true}) {
    MppiParams params = original ? originalParams() : MppiParams{};
    params.blind_side.enabled = false;
    double max_abs_y_sum = 0.0;
    constexpr int kSeeds = 5;
    for (int seed = 0; seed < kSeeds; ++seed) {
      const ReferencePath path({Point2{0.0, 0.0}, Point2{40.0, 0.0}});
      MppiController controller(static_cast<std::uint32_t>(seed));
      MppiState state;
      state.vx = 1.5;
      auto object = movingObject(-5.0, 0.0, 3.0, 0.0);
      double max_abs_y = 0.0;
      for (int i = 0; i < 40 && footprintGap(state, object, 0.4) > 0.0; ++i) {
        const auto command = controller.solve(state, path, {object}, nullptr, SE2{}, params, 1.5);
        state = step(state, command, params.dt);
        object.x += object.vx * params.dt;
        max_abs_y = std::max(max_abs_y, std::abs(state.y));
      }
      if (!original) {
        EXPECT_LT(max_abs_y, 0.35) << "seed " << seed;
      }
      max_abs_y_sum += max_abs_y;
    }
    if (original) {
      // 元の動きでは横へ 1.6m ほど避ける
      EXPECT_GT(max_abs_y_sum / kSeeds, 1.0);
    } else {
      EXPECT_LT(max_abs_y_sum / kSeeds, 0.25);
    }
  }
}

// 向かってくる相手との距離が 6m になるまでに横へ動いた量の平均と、すれ違うまでの最小距離
std::pair<double, double> headOnOffsetAndGap(const MppiParams & params)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{30.0, 0.0}});
  double offset_sum = 0.0;
  double min_gap = std::numeric_limits<double>::infinity();
  constexpr int kSeeds = 10;
  for (int seed = 0; seed < kSeeds; ++seed) {
    MppiController controller(static_cast<std::uint32_t>(seed));
    MppiState state;
    state.vx = 3.0;
    auto object = movingObject(14.0, 0.0, -3.0, 0.0);
    bool measured = false;
    while (object.x > state.x - 2.0) {
      const auto command = controller.solve(state, path, {object}, nullptr, SE2{}, params, 3.0);
      state = step(state, command, params.dt);
      object.x += object.vx * params.dt;
      min_gap = std::min(min_gap, footprintGap(state, object, 0.4));
      if (!measured && object.x - state.x <= 6.0) {
        offset_sum += std::abs(state.y);
        measured = true;
      }
    }
  }
  return {offset_sum / kSeeds, min_gap};
}

TEST(Responsibility, HeadOnAvoidanceUnchanged)
{
  // 正面から来る相手には、自機が近づく向きに動いているので今までどおり早めに避ける
  const auto [default_offset, default_gap] = headOnOffsetAndGap(MppiParams{});
  const auto [original_offset, original_gap] = headOnOffsetAndGap(originalParams());
  EXPECT_GT(default_offset, 0.4);
  EXPECT_NEAR(default_offset, original_offset, 0.1);
  EXPECT_GT(default_gap, 0.0);
  EXPECT_GT(original_gap, 0.0);
}

// 相手が交点に着く時刻を、今の速さのまま進むとして見積もる設定。先に入るときの寄せ方と速さを、
// 到着時刻の見積もり（加速度の上限で早める）と切り分けて確かめる
MppiParams constantSpeedEstimate()
{
  MppiParams params;
  params.objects.accel_mps2 = 0.0;
  return params;
}

TEST(MergeRule, ApproachesBisectorWithoutCrossing)
{
  // 自機（3m/s）が、右前を 20° で寄ってくる遅い相手（1.5m/s）を追い越して先に交点 (5, 0) を通る。
  // 自機は垂直二等分線まで寄せてよく、近づく向きに動いている間はその線を越えない。
  // 相手が速くならない見積もりで先に入る形にする（既定の見積もりでは、相手が 1.5m/s² で速くなると
  // 先に着きうるので譲る）
  for (std::uint32_t seed = 0; seed < 3; ++seed) {
    const auto merged =
      runAlongX(constantSpeedEstimate(), mergingRobot(5.0, 20.0, 1.5, 0.73), 30, 5.0, seed);
    const auto original =
      runAlongX(originalParams(), mergingRobot(5.0, 20.0, 1.5, 0.73), 30, 5.0, seed);
    EXPECT_GT(merged.min_bisector_approaching, -0.03) << "seed " << seed;
    EXPECT_LT(merged.min_bisector_approaching, 0.1) << "seed " << seed;
    // 元の動きは予測の広がりと余裕の分だけ離れる
    EXPECT_GT(original.min_center_approaching, merged.min_center_approaching + 0.3);
    EXPECT_EQ(merged.own_fault_steps, 0) << "seed " << seed;
  }
}

TEST(MergeRule, KeepsSpeedWhenGoingFirst)
{
  // 右前から 60° で来る相手より、自機が 0.7 秒先に交点 (6, 0) を通れる。減速も横へのずれもしない。
  // 相手が速くならない見積もりで先に入る形にする（既定の見積もりでは 0.7 秒の差では譲る。
  // 1.2 秒の差なら既定でも先に入る: MergeRule.KeepsSpeedWithEnoughLeadOverAccelerating）
  for (std::uint32_t seed = 0; seed < 3; ++seed) {
    const auto merged =
      runAlongX(constantSpeedEstimate(), mergingRobot(6.0, 60.0, 2.5, 0.7), 40, 6.0, seed);
    const auto original =
      runAlongX(originalParams(), mergingRobot(6.0, 60.0, 2.5, 0.7), 40, 6.0, seed);
    EXPECT_GT(merged.min_speed, 2.6) << "seed " << seed;
    EXPECT_LT(merged.max_abs_y, 0.15) << "seed " << seed;
    EXPECT_GT(merged.min_gap, 0.0) << "seed " << seed;
    // 元の動きは相手を理由に横へ譲る
    EXPECT_GT(original.max_abs_y, 0.3) << "seed " << seed;
  }
}

TEST(MergeRule, YieldsWhenOtherGoesFirst)
{
  // 相手が先に交点 (6, 0) を通る（0.3 秒先）か、自機の先行が 0.5 秒に満たない（0.2 秒）なら、
  // 相手が自機の後ろや横にいても今までどおり避けて後ろにつく。
  // 譲る相手は加速したときの位置まで重なりを見るので、0.2 秒の形では横へ 1m 以上よける代わりに、
  // 減速して相手より後に交点を通る（横へ 0.6〜0.8m、交点は相手の 0.6 秒後）
  for (const double lead_s : {-0.3, 0.2}) {
    for (std::uint32_t seed = 0; seed < 3; ++seed) {
      const auto merged =
        runAlongX(MppiParams{}, mergingRobot(6.0, 60.0, 2.5, lead_s), 40, 6.0, seed);
      EXPECT_GT(merged.min_gap, 0.1) << "lead " << lead_s << " seed " << seed;
      if (lead_s < 0.0) {
        // 相手が交点を通る時刻（6 / 3 + lead_s 秒）より後に通る
        EXPECT_GT(merged.cross_time_s, 2.0 + lead_s) << "seed " << seed;
        EXPECT_LT(merged.min_speed, 2.5) << "seed " << seed;
      } else {
        EXPECT_TRUE(merged.max_abs_y > 1.0 || merged.cross_time_s > 2.0 + lead_s) << "seed " << seed;
      }
    }
  }
}

// 交点 (cross_x, 0) を、自機（own_speed）より lead_s 秒遅れて通る相手。heading_deg は相手の進む向き
PredictedObject robotThroughPoint(
  const double cross_x, const double heading_deg, const double speed, const double own_speed,
  const double lead_s)
{
  const double heading = heading_deg * M_PI / 180.0;
  const double distance = speed * (cross_x / own_speed + lead_s);
  return movingObject(
    cross_x - std::cos(heading) * distance, -std::sin(heading) * distance,
    std::cos(heading) * speed, std::sin(heading) * speed);
}

TEST(CrossingRule, YieldsWhenCrossingInFrontOfRobot)
{
  // 自機（2.5m/s）が、右から 100° の向きで来る相手の前を横切る。交点 (5, 0) を自機が 0.2 秒先に通る
  // 形で、相手は自機の横から当たってくる。横切りなので、近づく向きによらず今までどおり避ける
  MppiParams previous;
  previous.crossing.enabled = false;
  // 交点を前回の軌跡で取るので、避けて左へ曲がった軌跡では向きの差が 90° 未満になり、合流（相手が先）
  // として避けることがある。横切りの規則がない場合を見るため、合流の規則も切る
  previous.merge.enabled = false;
  double min_gap_previous = std::numeric_limits<double>::infinity();
  for (std::uint32_t seed = 0; seed < 3; ++seed) {
    const ReferencePath path({Point2{0.0, 0.0}, Point2{40.0, 0.0}});
    for (const bool enabled : {true, false}) {
      const MppiParams params = enabled ? MppiParams{} : previous;
      MppiController controller(seed);
      MppiState state;
      state.vx = 2.5;
      auto object = robotThroughPoint(5.0, 100.0, 2.5, 2.5, 0.2);
      double min_gap = std::numeric_limits<double>::infinity();
      for (int i = 0; i < 40; ++i) {
        const auto command = controller.solve(state, path, {object}, nullptr, SE2{}, params, 2.5);
        state = step(state, command, params.dt);
        object.x += object.vx * params.dt;
        object.y += object.vy * params.dt;
        min_gap = std::min(min_gap, footprintGap(state, object, 0.4));
      }
      if (enabled) {
        EXPECT_GT(min_gap, 0.3) << "seed " << seed;
      } else {
        min_gap_previous = std::min(min_gap_previous, min_gap);
      }
    }
  }
  // 横切りの規則がなければ、横から当てられる
  EXPECT_LE(min_gap_previous, 0.0);
}

TEST(CrossingRule, IgnoresRobotAfterPassingCrossing)
{
  // 1.0m/s の自機の 0.5m 後ろの交点を、相手が右後ろから 120° の向きで横切る。自機は交点を通り過ぎて
  // いるので横切りではなく、相手が当たるまで進路も速さも変えない（元の動きは 4 回中 3 回避ける）
  const ReferencePath path({Point2{-2.0, 0.0}, Point2{40.0, 0.0}});
  const MppiParams params;
  for (std::uint32_t seed = 0; seed < 4; ++seed) {
    MppiController controller(seed);
    MppiState state;
    state.vx = 1.0;
    const double heading = 120.0 * M_PI / 180.0;
    auto object = movingObject(
      -0.5 - std::cos(heading) * 2.0, -std::sin(heading) * 2.0, std::cos(heading) * 2.5,
      std::sin(heading) * 2.5);
    double max_abs_y = 0.0;
    double min_speed = std::numeric_limits<double>::infinity();
    int steps = 0;
    for (; steps < 30 && footprintGap(state, object, 0.4) > 0.0; ++steps) {
      const auto command = controller.solve(state, path, {object}, nullptr, SE2{}, params, 1.0);
      state = step(state, command, params.dt);
      object.x += object.vx * params.dt;
      object.y += object.vy * params.dt;
      max_abs_y = std::max(max_abs_y, std::abs(state.y));
      min_speed = std::min(min_speed, std::hypot(state.vx, state.vy));
    }
    EXPECT_LT(steps, 30) << "seed " << seed;
    EXPECT_LT(max_abs_y, 0.15) << "seed " << seed;
    EXPECT_GT(min_speed, 0.7) << "seed " << seed;
  }
}

TEST(Human, StationaryRobotEvadesApproachingHuman)
{
  // 人には責任に応じた回避を当てない。止まっている自機に人が向かってくれば避ける
  const MppiParams params;
  MppiController controller;
  MppiState state;
  // 最接近まで 2.3 秒（collision_course_lookahead_s 2.5 秒以内）
  auto human = humanObject(3.5, 0.2, -1.5, 0.0);
  EXPECT_TRUE(core_planning::needsEvasionWhileStopped(state, {human}, params, false));
  const auto robot = movingObject(3.5, 0.2, -1.5, 0.0);
  EXPECT_FALSE(core_planning::needsEvasionWhileStopped(state, {robot}, params, false));
  EXPECT_TRUE(core_planning::needsEvasionWhileStopped(state, {robot}, params, true));
  double min_gap = std::numeric_limits<double>::infinity();
  double max_displacement = 0.0;
  for (int i = 0; i < 60; ++i) {
    const ReferencePath here({Point2{state.x, state.y}, Point2{state.x, state.y}});
    const auto command = controller.solve(state, here, {human}, nullptr, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    human.x += human.vx * params.dt;
    min_gap = std::min(min_gap, footprintGap(state, human, 0.3));
    max_displacement = std::max(max_displacement, std::hypot(state.x, state.y));
  }
  EXPECT_GT(min_gap, 0.0);
  EXPECT_GT(max_displacement, 0.3);
}

TEST(StoppedEvasion, DoesNotMoveForHumansByDefault)
{
  // 止まっている間（到達後、行き詰まりで待つ・下がる間）は、既定（どちらも false）なら人が向かってきても
  // 避けない（ユーザーの判断）。evade_humans_while_stopped なら人だけ、evade_while_stopped なら相手すべてを避ける
  const MppiParams params;
  const MppiState state;
  // 最接近まで 2.3 秒（collision_course_lookahead_s 2.5 秒以内）
  const auto human = humanObject(3.5, 0.2, -1.5, 0.0);
  const auto robot = movingObject(3.5, 0.2, -1.5, 0.0);
  EXPECT_FALSE(core_planning::shouldEvadeWhileStopped(state, {human}, params, false, false));
  EXPECT_FALSE(core_planning::shouldEvadeWhileStopped(state, {robot}, params, false, false));
  EXPECT_TRUE(core_planning::shouldEvadeWhileStopped(state, {human}, params, false, true));
  EXPECT_FALSE(core_planning::shouldEvadeWhileStopped(state, {robot}, params, false, true));
  EXPECT_TRUE(core_planning::shouldEvadeWhileStopped(state, {human}, params, true, false));
  EXPECT_TRUE(core_planning::shouldEvadeWhileStopped(state, {robot}, params, true, false));
  // 離れていく人は、どの設定でも避けない
  const auto leaving = humanObject(3.5, 0.2, 1.5, 0.0);
  EXPECT_FALSE(core_planning::shouldEvadeWhileStopped(state, {leaving}, params, true, true));
}

TEST(Human, AvoidsHumanApproachingFromBehind)
{
  // ロボットなら後ろから追いつかれても避けないが、人は避ける。人の近くの速度の上限 1.0m/s で進む自機に、
  // 後ろから 1.5m/s の人が追いつく
  const ReferencePath path({Point2{0.0, 0.0}, Point2{40.0, 0.0}});
  const MppiParams params;
  MppiController controller;
  MppiState state;
  state.vx = 0.5;
  auto human = humanObject(-3.0, 0.0, 1.5, 0.0);
  double min_gap = std::numeric_limits<double>::infinity();
  double max_abs_y = 0.0;
  for (int i = 0; i < 50; ++i) {
    const auto command = controller.solve(state, path, {human}, nullptr, SE2{}, params, 1.0);
    state = step(state, command, params.dt);
    human.x += human.vx * params.dt;
    min_gap = std::min(min_gap, footprintGap(state, human, 0.3));
    max_abs_y = std::max(max_abs_y, std::abs(state.y));
  }
  EXPECT_GT(min_gap, 0.0);
  EXPECT_GT(max_abs_y, 0.3);
}

TEST(SpeedLimit, CapsSpeedInsideHumanRegionInAnyDirection)
{
  SpeedLimitParams params;
  params.density_speed_scale = 1.0;
  // 機体の外形（0.8m 角）が人の中心から 1m 以内なら、後ろ、横、角、遠ざかる人のどれでも 1.0m/s 以下
  for (const auto & human : {humanObject(-1.3, 0.0, 0.0, 0.0), humanObject(0.0, 1.3, 0.0, 0.0),
      humanObject(-0.9, -0.9, 0.0, 0.0), humanObject(1.2, -0.5, 1.2, 0.0)})
  {
    EXPECT_LE(
      speedLimitAt(params, {human}, 0.0, 0.0, 3.0, 0.0, 3.0), params.human_speed_mps + 1e-9);
  }
  // min_speed_mps より優先する
  params.min_speed_mps = 1.8;
  EXPECT_LE(
    speedLimitAt(params, {humanObject(0.0, 1.3, 0.0, 0.0)}, 0.0, 0.0, 3.0, 0.0, 3.0),
    params.human_speed_mps + 1e-9);
}

TEST(SpeedLimit, DoesNotSlowWhenPassingOrLeavingHuman)
{
  SpeedLimitParams params;
  params.density_speed_scale = 1.0;
  // 領域の外を横に通るだけ、後ろの止まっている人から離れる、後ろから自機より遅く同じ向きに歩いてくる人
  // では下げない（前は人の中心から 3m 以内で一律に下げた）
  for (const auto & human : {humanObject(0.0, 1.6, 0.0, 0.0), humanObject(-2.0, 0.0, 0.0, 0.0),
      humanObject(-2.5, 0.0, 0.2, 0.0), humanObject(-2.0, 0.0, 1.2, 0.0)})
  {
    EXPECT_DOUBLE_EQ(speedLimitAt(params, {human}, 0.0, 0.0, 3.0, 0.0, 3.0), 3.0);
  }
}

TEST(SpeedLimit, SlowsBeforeEnteringHumanRegion)
{
  SpeedLimitParams params;
  params.density_speed_scale = 1.0;
  SpeedLimitParams without_rule = params;
  without_rule.human_enabled = false;
  // 正面の止まっている人へ 3m/s で向かう。領域（人の中心から 1m）が機体にかかる時刻（機体の前の端は
  // x = 0.4 なので、境まで x - 1.4 を 3m/s で進む時間）までに、反応 0.2 秒のあと 3m/s² で 1.0m/s に
  // 落とせる速さ。見通しの制限と比べて低いほう
  double previous = std::numeric_limits<double>::infinity();
  for (const double x : {3.4, 2.4, 1.9}) {
    const auto human = humanObject(x, 0.0, 0.0, 0.0);
    const double limit = speedLimitAt(params, {human}, 0.0, 0.0, 3.0, 0.0, 3.0);
    const double entry = (x - 1.4) / 3.0;
    const double want = std::min(
      1.0 + 3.0 * std::max(entry - 0.2, 0.0),
      speedLimitAt(without_rule, {human}, 0.0, 0.0, 3.0, 0.0, 3.0));
    // 時刻は 0.05 秒刻みで調べるので、その分（3m/s² × 0.05 秒）まで高く出る
    EXPECT_NEAR(limit, want, 0.16) << "x=" << x;
    EXPECT_LE(limit, previous);
    previous = limit;
  }
  // 反応の時間のうちに領域がかかるなら human_speed_mps
  EXPECT_NEAR(
    speedLimitAt(params, {humanObject(1.6, 0.0, 0.0, 0.0)}, 0.0, 0.0, 3.0, 0.0, 3.0),
    params.human_speed_mps, 1e-6);
  // 斜めに向かうと、領域がかかるのが遅いので、上限は大きい
  const auto ahead = humanObject(2.4, 0.0, 0.0, 0.0);
  EXPECT_GT(
    speedLimitAt(params, {ahead}, 0.0, 0.0, 3.0 * std::cos(0.6), 3.0 * std::sin(0.6), 3.0),
    speedLimitAt(params, {ahead}, 0.0, 0.0, 3.0, 0.0, 3.0) + 0.2);
}

TEST(SpeedLimit, SlowsWhenHumanApproaches)
{
  SpeedLimitParams params;
  params.density_speed_scale = 1.0;
  // 止まっている自機へ、前から 1.2m/s、横から斜めに 1.17m/s で人が向かってきて、2 秒以内に領域が機体にかかる
  for (const auto & human : {humanObject(3.5, 0.0, -1.2, 0.0), humanObject(1.5, 2.5, -0.6, -1.0)}) {
    EXPECT_DOUBLE_EQ(
      speedLimitAt(params, {human}, 0.0, 0.0, 0.0, 0.0, 3.0), params.human_speed_mps);
  }
  // 2 秒では届かない、離れていく、横を通り過ぎる人では下げない
  for (const auto & human : {humanObject(4.5, 0.0, -1.2, 0.0), humanObject(3.5, 0.0, 1.2, 0.0),
      humanObject(3.5, 2.0, -1.2, 0.0)})
  {
    EXPECT_DOUBLE_EQ(speedLimitAt(params, {human}, 0.0, 0.0, 0.0, 0.0, 3.0), 3.0);
  }
  // 人の向かう先から自機が横へ速く外れていく（人が通るときには自機はいない）なら下げない
  EXPECT_DOUBLE_EQ(
    speedLimitAt(params, {humanObject(3.5, 0.0, -1.2, 0.0)}, 0.0, 0.0, 0.0, 3.0, 3.0), 3.0);
}

TEST(SpeedLimit, KeepsLimitWithoutHumans)
{
  // 同じ位置にいるのがロボットなら、人の制限はかからない
  SpeedLimitParams with_rule;
  with_rule.density_speed_scale = 1.0;
  SpeedLimitParams without_rule = with_rule;
  without_rule.human_enabled = false;
  for (const auto & robot : {movingObject(-2.0, 0.0, 0.0, 0.0), movingObject(0.0, 2.5, 0.0, 0.0),
      movingObject(2.0, -1.5, 1.2, 0.0)})
  {
    EXPECT_DOUBLE_EQ(
      speedLimitAt(with_rule, {robot}, 0.0, 0.0, 3.0, 0.0, 3.0),
      speedLimitAt(without_rule, {robot}, 0.0, 0.0, 3.0, 0.0, 3.0));
  }
  EXPECT_DOUBLE_EQ(speedLimitAt(with_rule, {}, 0.0, 0.0, 3.0, 0.0, 3.0), 3.0);
}

TEST(SpeedLimit, HumanCapCanBeDisabled)
{
  SpeedLimitParams params;
  params.density_speed_scale = 1.0;
  params.human_enabled = false;
  // 領域（人の中心から 1m）の中の人でも下げない
  EXPECT_DOUBLE_EQ(
    speedLimitAt(params, {humanObject(0.0, 1.2, 0.0, 0.0)}, 0.0, 0.0, 3.0, 0.0, 3.0), 3.0);
}

TEST(ReferencePath, TruncatesAtLength)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{10.0, 0.0}, Point2{10.0, 5.0}}, 1.0);
  const auto cut = path.truncated(12.0);
  EXPECT_NEAR(cut.length(), 12.0, 1e-9);
  EXPECT_NEAR(cut.points().back().x, 10.0, 1e-9);
  EXPECT_NEAR(cut.points().back().y, 2.0, 1e-9);
  ASSERT_TRUE(cut.goalYaw().has_value());
  EXPECT_DOUBLE_EQ(*cut.goalYaw(), 1.0);
  EXPECT_NEAR(path.truncated(20.0).length(), 15.0, 1e-9);
  EXPECT_NEAR(path.truncated(-1.0).length(), 0.0, 1e-9);
}

TEST(MergeRule, EstimatesEarliestArrival)
{
  using core_planning::earliestArrivalTime;
  // 加速しなければ今の速さのまま
  EXPECT_NEAR(earliestArrivalTime(6.0, 2.0, 0.0, 3.0), 3.0, 1e-9);
  // 0.5m/s から 1.5m/s² で 3.4m: 0.5t + 0.75t² = 3.4
  EXPECT_NEAR(earliestArrivalTime(3.4, 0.5, 1.5, 10.0), (-0.5 + std::sqrt(0.25 + 10.2)) / 1.5, 1e-9);
  // 2.5m/s から 3.0m/s まで 1/3 秒で 0.9167m、残りは 3.0m/s
  EXPECT_NEAR(earliestArrivalTime(6.75, 2.5, 1.5, 3.0), 1.0 / 3.0 + (6.75 - 0.75 - 1.0 / 6.0) / 3.0, 1e-9);
  // すでに上限より速ければ、そのまま
  EXPECT_NEAR(earliestArrivalTime(7.0, 3.5, 1.5, 3.0), 2.0, 1e-9);
  // 止まっていて加速もしなければ着かない
  EXPECT_TRUE(std::isinf(earliestArrivalTime(1.0, 0.0, 0.0, 3.0)));
}

TEST(MergeRule, YieldsToRobotThatCanAccelerate)
{
  // KeepsSpeedWhenGoingFirst と同じ形（自機が 0.7 秒先）。相手が 1.5m/s² で 3m/s まで速くなると
  // 0.42 秒早く着きうるので、離れている間は先行が 0.5 秒に満たず、今までどおり避ける（横へよける）。
  // 近づくと見積もりの差が縮み、先に入る判定に変わる
  double accelerating_sum = 0.0;
  double constant_sum = 0.0;
  constexpr int kSeeds = 5;
  for (int seed = 0; seed < kSeeds; ++seed) {
    const auto merged = runAlongX(
      MppiParams{}, mergingRobot(6.0, 60.0, 2.5, 0.7), 40, 6.0, static_cast<std::uint32_t>(seed));
    const auto constant = runAlongX(
      constantSpeedEstimate(), mergingRobot(6.0, 60.0, 2.5, 0.7), 40, 6.0,
      static_cast<std::uint32_t>(seed));
    EXPECT_GT(merged.min_gap, 0.1) << "seed " << seed;
    accelerating_sum += merged.max_abs_y;
    constant_sum += constant.max_abs_y;
  }
  // 横へのずれの平均は 0.22m と 0.04m（5 回）
  EXPECT_GT(accelerating_sum / kSeeds, constant_sum / kSeeds + 0.1);
}

TEST(MergeRule, KeepsSpeedWithEnoughLeadOverAccelerating)
{
  // 自機が 1.2 秒先なら、相手が速くなっても 0.5 秒以上先に着けるので、既定でも減速も横へのずれもしない
  for (std::uint32_t seed = 0; seed < 3; ++seed) {
    const auto merged = runAlongX(MppiParams{}, mergingRobot(6.0, 60.0, 2.5, 1.2), 40, 6.0, seed);
    EXPECT_GT(merged.min_speed, 2.6) << "seed " << seed;
    EXPECT_LT(merged.max_abs_y, 0.15) << "seed " << seed;
    EXPECT_GT(merged.min_gap, 0.0) << "seed " << seed;
  }
}

// 合流の相手が、遅い速さ（0.5m/s）から 1.2m/s² で速くなる（3m/s まで）。追跡の速度は 0.5 秒遅れる。
// 相手の進む向きは heading_deg で、自機（+x へ 3m/s）とほぼ同時に交点 (6, 0) に着く。自機が近づく向きに
// 動いていて重なったステップの数を返す
int ownFaultStepsWithAcceleratingRobot(const MppiParams & params, const std::uint32_t seed)
{
  constexpr double kHeading = 65.0 * M_PI / 180.0;
  constexpr double kAccel = 1.2;
  // 追跡の速さは 5 ステップ（0.5 秒）前の真の速さ
  constexpr std::size_t kLagSteps = 5U;
  const ReferencePath path({Point2{0.0, 0.0}, Point2{40.0, 0.0}});
  MppiController controller(seed);
  MppiState state;
  state.vx = 3.0;
  // 0.5t + 0.6t² = 3.4（t = 2 秒）
  double along = -3.4;
  double speed = 0.5;
  std::vector<double> speeds;
  int own_fault_steps = 0;
  for (int i = 0; i < 40; ++i) {
    speeds.push_back(speed);
    const double tracked_speed =
      speeds[speeds.size() > kLagSteps ? speeds.size() - 1U - kLagSteps : 0U];
    auto object = movingObject(
      6.0 + std::cos(kHeading) * along, std::sin(kHeading) * along,
      std::cos(kHeading) * tracked_speed, std::sin(kHeading) * tracked_speed);
    const auto command = controller.solve(state, path, {object}, nullptr, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    along += speed * params.dt + 0.5 * kAccel * params.dt * params.dt;
    speed = std::min(speed + kAccel * params.dt, 3.0);
    object.x = 6.0 + std::cos(kHeading) * along;
    object.y = std::sin(kHeading) * along;
    if (approachSpeed(state, object) > 0.0 && footprintGap(state, object, 0.4) <= 0.0) {
      ++own_fault_steps;
    }
  }
  return own_fault_steps;
}

TEST(MergeRule, YieldsToRobotAcceleratingFromSlowSpeed)
{
  // 追跡の速さ（遅れて 0.5m/s）のまま見積もると自機が先と判定して進み、自機から当たる。
  // 加速度の上限で見積もると相手が先と判定して譲り、自機から当たらない
  int constant_total = 0;
  for (std::uint32_t seed = 0; seed < 3; ++seed) {
    EXPECT_EQ(ownFaultStepsWithAcceleratingRobot(MppiParams{}, seed), 0) << "seed " << seed;
    constant_total += ownFaultStepsWithAcceleratingRobot(constantSpeedEstimate(), seed);
  }
  EXPECT_GT(constant_total, 0);
}

TEST(StuckMonitor, CountsMovementAsProgress)
{
  // すき間を行き過ぎて 0.8m/s で引き返す。残りの距離は増えるが、動いている間は行き詰まりとしない
  const StuckParams params;
  StuckMonitor monitor;
  EXPECT_EQ(monitor.update(0.0, 3.0, 0.0, 0.0, params), StuckState::kDriving);
  for (int i = 1; i <= 40; ++i) {
    const double t = 0.1 * i;
    EXPECT_EQ(monitor.update(t, 3.0 + 0.1 * t, 0.8 * t, 0.0, params), StuckState::kDriving) << t;
  }
  // 止まると、最後に動いた位置から数えて判定する
  StuckState state = StuckState::kDriving;
  double stuck_time = -1.0;
  for (int i = 41; i <= 80 && state == StuckState::kDriving; ++i) {
    state = monitor.update(0.1 * i, 3.4, 3.2, 0.0, params);
    stuck_time = 0.1 * i;
  }
  EXPECT_EQ(state, StuckState::kWait);
  EXPECT_LE(stuck_time, 4.0 + params.no_progress_timeout_s + 0.11);
}

TEST(StuckMonitor, IgnoresSmallBackAndForth)
{
  // その場で 0.2m ほど前後に揺れるだけなら、動いているとはみなさない
  const StuckParams params;
  StuckMonitor monitor;
  EXPECT_EQ(monitor.update(0.0, 5.0, 0.0, 0.0, params), StuckState::kDriving);
  StuckState state = StuckState::kDriving;
  double t = 0.0;
  while (state == StuckState::kDriving && t < 10.0) {
    t += 0.1;
    state = monitor.update(t, 5.0, 0.2 * std::sin(3.0 * t), 0.0, params);
  }
  EXPECT_EQ(state, StuckState::kWait);
  EXPECT_NEAR(t, params.no_progress_timeout_s, 0.11);
}

TEST(StuckMonitor, MovementCountsOnlyUntilTimeout)
{
  // 動き回るだけでゴールへ近づけなければ、moving_timeout_s の後に行き詰まりとする。0 なら数えない
  for (const double moving_distance : {0.5, 0.0}) {
    StuckParams params;
    params.moving_distance_m = moving_distance;
    StuckMonitor monitor;
    monitor.update(0.0, 5.0, 0.0, 0.0, params);
    StuckState state = StuckState::kDriving;
    double t = 0.0;
    while (state == StuckState::kDriving && t < 30.0) {
      t += 0.1;
      // 半径 2m の円を 1m/s で回る
      state = monitor.update(t, 5.0, 2.0 * std::cos(0.5 * t), 2.0 * std::sin(0.5 * t), params);
    }
    EXPECT_EQ(state, StuckState::kWait);
    if (moving_distance > 0.0) {
      EXPECT_GT(t, params.moving_timeout_s);
      EXPECT_LE(t, params.moving_timeout_s + params.no_progress_timeout_s + 0.11);
    } else {
      EXPECT_NEAR(t, params.no_progress_timeout_s, 0.11);
    }
  }
}

using core_planning::GoalApproachParams;
using core_planning::goalHoldLength;
using core_planning::GoalHoldMonitor;

TEST(GoalApproach, HoldsWhenRobotPassesNearGoalDuringArrival)
{
  // ゴール (10, 0) へ 3m/s で向かう。相手は x = 10 を +y へ 2m/s で進み、約 2 秒後にゴールを通る
  const ReferencePath path({Point2{0.0, 0.0}, Point2{10.0, 0.0}});
  const MppiParams params;
  const GoalApproachParams goal;
  MppiState state;
  state.x = 4.0;
  state.vx = 3.0;
  const auto hold = goalHoldLength(state, path, {movingObject(10.0, -4.0, 0.0, 2.0)}, params, goal, 3.0);
  ASSERT_TRUE(hold.has_value());
  EXPECT_NEAR(*hold, 10.0 - goal.hold_distance_m, 1e-9);
  // 相手がゴールの 1m 先（x = 11 の線）を通っても近く（1.5m 以内）
  EXPECT_TRUE(
    goalHoldLength(state, path, {movingObject(11.0, -4.0, 0.0, 2.0)}, params, goal, 3.0).has_value());
  // 3m 先の線なら近くを通らない
  EXPECT_FALSE(
    goalHoldLength(state, path, {movingObject(13.0, -4.0, 0.0, 2.0)}, params, goal, 3.0).has_value());
  // 無効
  GoalApproachParams disabled;
  disabled.enabled = false;
  EXPECT_FALSE(
    goalHoldLength(state, path, {movingObject(10.0, -4.0, 0.0, 2.0)}, params, disabled, 3.0)
    .has_value());
}

TEST(GoalApproach, IgnoresRobotsThatCannotReachGoalInTime)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{10.0, 0.0}});
  const MppiParams params;
  const GoalApproachParams goal;
  MppiState state;
  state.x = 4.0;
  state.vx = 3.0;
  // ゴールを通り過ぎて 3m/s で離れていく（自機が着くまでに戻れない）
  EXPECT_FALSE(
    goalHoldLength(state, path, {movingObject(10.0, 2.5, 0.0, 3.0)}, params, goal, 3.0).has_value());
  // 遠くにいて、自機が着くまでに近くまで来られない
  EXPECT_FALSE(
    goalHoldLength(state, path, {movingObject(10.0, -14.0, 0.0, 1.0)}, params, goal, 3.0).has_value());
  // 止まっている相手は理由にしない（塞がれたゴールは大域計画が扱う）
  EXPECT_FALSE(
    goalHoldLength(state, path, {movingObject(10.0, 0.5, 0.0, 0.0)}, params, goal, 3.0).has_value());
  // 人は理由にしない
  auto human = movingObject(10.0, -3.0, 0.0, 1.2);
  human.kind = ObjectKind::kHuman;
  EXPECT_FALSE(goalHoldLength(state, path, {human}, params, goal, 3.0).has_value());
}

TEST(GoalApproach, DoesNotHoldOnRobotLaneOrInsideHoldDistance)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{10.0, 0.0}});
  const MppiParams params;
  const GoalApproachParams goal;
  MppiState state;
  state.x = 4.0;
  state.vx = 3.0;
  // 自機の経路と同じ線を向かってくる相手。待つ位置も相手の通り道の上なので、待っても離れられない
  EXPECT_FALSE(
    goalHoldLength(state, path, {movingObject(16.0, 0.0, -2.0, 0.0)}, params, goal, 3.0).has_value());
  // 待つ位置より内側にいれば、そのままゴールへ進む
  state.x = 8.5;
  state.vx = 1.5;
  EXPECT_FALSE(
    goalHoldLength(state, path, {movingObject(10.0, -2.0, 0.0, 2.0)}, params, goal, 3.0).has_value());
}

struct GoalApproachResult
{
  // 自機が 0.5m/s 以上で近づく向きに動いていて、0.3m より近づいたステップ数
  int near_steps{0};
  double min_gap{std::numeric_limits<double>::infinity()};
  bool reached{false};
};

// ゴール (10, 0) へ止まった状態から向かう。相手は x = 10 を +y へ 1.5m/s で、自機が着く頃にゴールを通る。
// GoalHoldMonitor で待つ位置まで経路を切る（mppi_lite_node と同じ）。着いてからも 3 秒見る
GoalApproachResult approachGoalWithCrossingRobot(
  const GoalApproachParams & goal, const double start_y)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{10.0, 0.0}});
  const MppiParams params;
  MppiController controller(1U);
  GoalHoldMonitor monitor;
  MppiState state;
  auto object = movingObject(10.0, start_y, 0.0, 1.5);
  GoalApproachResult result;
  int after_reached = 0;
  for (int i = 0; i < 150 && after_reached < 30; ++i) {
    if (result.reached) {
      // 着いた後は止まったまま（到達後は避けない）
      object.y += object.vy * params.dt;
      result.min_gap = std::min(result.min_gap, footprintGap(state, object, 0.4));
      ++after_reached;
      continue;
    }
    const auto hold =
      monitor.update(static_cast<double>(i) * params.dt, state, path, {object}, params, goal, 3.0);
    const auto command = controller.solve(
      state, hold.has_value() ? path.truncated(*hold) : path, {object}, nullptr, SE2{}, params,
      3.0);
    state = step(state, command, params.dt);
    object.y += object.vy * params.dt;
    const double gap = footprintGap(state, object, 0.4);
    result.min_gap = std::min(result.min_gap, gap);
    if (approachSpeed(state, object) > 0.0 && gap < 0.3 && std::hypot(state.vx, state.vy) > 0.5) {
      ++result.near_steps;
    }
    result.reached =
      std::hypot(state.x - 10.0, state.y) < 0.15 && std::hypot(state.vx, state.vy) < 0.2;
  }
  return result;
}

TEST(GoalApproach, WaitsBeforeGoalUntilRobotPasses)
{
  // 待たなければ、ゴールで止まった自機に相手が当たる（止まっている間は避けない方針）。
  // 手前で待てば、相手が通り過ぎてからゴールへ入り、当たらない。相手は 6.0〜6.7 秒後にゴールを通る
  // （自機が待たずに着くのは約 4.6〜8 秒後）
  GoalApproachParams disabled;
  disabled.enabled = false;
  for (const double start_y : {-10.0, -9.0}) {
    const auto without = approachGoalWithCrossingRobot(disabled, start_y);
    const auto with = approachGoalWithCrossingRobot(GoalApproachParams{}, start_y);
    EXPECT_LE(without.min_gap, 0.0) << start_y;
    EXPECT_GT(with.min_gap, 0.5) << start_y;
    EXPECT_EQ(with.near_steps, 0) << start_y;
    EXPECT_TRUE(with.reached) << start_y;
  }
}

TEST(GoalApproach, MonitorKeepsHoldingBrieflyAndGivesUp)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{10.0, 0.0}});
  const MppiParams params;
  const GoalApproachParams goal;
  MppiState state;
  state.x = 4.0;
  const std::vector<core_planning::PredictedObject> crossing{movingObject(10.0, -4.0, 0.0, 2.0)};
  GoalHoldMonitor monitor;
  ASSERT_TRUE(monitor.update(0.0, state, path, crossing, params, goal, 3.0).has_value());
  // 予測が途切れても release_s の間は待ち続け、過ぎたらやめる
  EXPECT_TRUE(monitor.update(0.5, state, path, {}, params, goal, 3.0).has_value());
  EXPECT_FALSE(monitor.update(1.1, state, path, {}, params, goal, 3.0).has_value());
  // 待つ位置（x = 8）へ向かって走っている間は、待った時間に数えない
  double t = 2.0;
  for (; t < 2.0 + 2.0 * goal.max_hold_s; t += 0.1) {
    ASSERT_TRUE(monitor.update(t, state, path, crossing, params, goal, 3.0).has_value()) << t;
  }
  EXPECT_DOUBLE_EQ(monitor.waited(), 0.0);
  // 待つ位置の近くで max_hold_s 待ったら、通る予測があっても待たない
  state.x = 7.7;
  const double arrived = t;
  bool holding = true;
  for (; holding && t < arrived + 2.0 * goal.max_hold_s; t += 0.1) {
    holding = monitor.update(t, state, path, crossing, params, goal, 3.0).has_value();
  }
  EXPECT_FALSE(holding);
  EXPECT_NEAR(t - arrived, goal.max_hold_s + 0.1, 0.11);
  // ゴールが変われば数え直す
  monitor.reset();
  EXPECT_DOUBLE_EQ(monitor.waited(), 0.0);
  EXPECT_TRUE(monitor.update(100.0, state, path, crossing, params, goal, 3.0).has_value());
  // 待つ位置より内側に入れば、途切れたらすぐにやめる
  state.x = 8.5;
  EXPECT_FALSE(monitor.update(100.1, state, path, {}, params, goal, 3.0).has_value());
}

// 幅 width（壁の面どうしの距離）、x ∈ [-5, 5] の通路。壁の外側は柵まで埋める
DistanceField corridorField(const double width)
{
  auto grid = makeFieldGrid(FieldBounds{-12.0, 12.0, -7.5, 7.5}, 0.05);
  const auto & info = grid.info();
  for (int cx = 0; cx < info.width; ++cx) {
    const double x = info.centerX(cx);
    if (x < -5.0 || x > 5.0) {
      continue;
    }
    for (int cy = 0; cy < info.height; ++cy) {
      if (std::abs(info.centerY(cy)) >= 0.5 * width) {
        grid.setOccupied(core_planning::GridCell{cx, cy});
      }
    }
  }
  return computeDistanceField(grid);
}

struct CorridorRun
{
  // 通路の中（x ∈ [-3, 3]）の平均の速さ
  double mean_speed{0.0};
  // 機体の側面から壁の面までの最小距離
  double min_clearance{std::numeric_limits<double>::infinity()};
  bool passed{false};
};

// 通路の中心線の経路を、指令どおりに動く機体で閉ループで走らせる
CorridorRun runCorridor(const double width, const MppiParams & params)
{
  const auto field = corridorField(width);
  const ReferencePath path({Point2{-10.0, 0.0}, Point2{11.0, 0.0}});
  MppiController controller;
  MppiState state;
  state.x = -10.0;
  CorridorRun run;
  double speed_sum = 0.0;
  int count = 0;
  for (int i = 0; i < 300 && !run.passed; ++i) {
    const auto command = controller.solve(state, path, {}, &field, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    if (state.x >= -3.0 && state.x <= 3.0) {
      speed_sum += std::hypot(state.vx, state.vy);
      ++count;
    }
    if (state.x >= -5.4 && state.x <= 5.4) {
      const double half_extent = 0.4 * (std::abs(std::cos(state.yaw)) + std::abs(std::sin(state.yaw)));
      run.min_clearance = std::min(run.min_clearance, 0.5 * width - std::abs(state.y) - half_extent);
    }
    run.passed = state.x >= 7.0;
  }
  run.mean_speed = count > 0 ? speed_sum / count : 0.0;
  return run;
}

TEST(StaticMargin, PassesNarrowCorridorAtSpeed)
{
  // 壁と平行に走るときは静止物へ近づく速さが 0 なので、余裕を増やさない。幅 1.8〜2.0m（片側 0.5〜0.6m）の
  // 通路を、壁がないとき（約 2.9m/s）と同じ速さで通る。既定は false なので、ここで有効にする
  MppiParams params;
  params.static_margin_approach_only = true;
  for (const double width : {1.8, 2.0}) {
    const auto run = runCorridor(width, params);
    EXPECT_TRUE(run.passed) << width;
    EXPECT_GT(run.mean_speed, 2.7) << width;
    EXPECT_GT(run.min_clearance, params.footprint.margin_m) << width;
  }
  // 全方向の速さ（元の動き）では、幅 2.0m でも入口の手前で止まる（狭い通路の扱いも切った元の動き）
  MppiParams original = params;
  original.static_margin_approach_only = false;
  original.narrow.enabled = false;
  const auto run = runCorridor(2.0, original);
  EXPECT_FALSE(run.passed);
  EXPECT_LT(run.mean_speed, 1.0);
}

struct WallApproach
{
  // 機体の前の面から壁までの距離の、最初に止まるまで（5.5 秒）の最小と、10 秒の間の最小
  double first_stop_clearance{std::numeric_limits<double>::infinity()};
  double min_clearance{std::numeric_limits<double>::infinity()};
  // 前の面が壁から 1.5m の所を通ったときの速さ
  double speed_at_1_5m{0.0};
};

// 壁（柵の x = 6）を越えた先のゴールへ、壁に直角に向かう
WallApproach approachWall(const MppiParams & params)
{
  const auto field = computeDistanceField(makeFieldGrid(FieldBounds{-12.0, 6.0, -7.5, 7.5}, 0.05));
  const ReferencePath path({Point2{-6.0, 0.0}, Point2{10.0, 0.0}});
  MppiController controller;
  MppiState state;
  state.x = -6.0;
  WallApproach result;
  for (int i = 0; i < 100; ++i) {
    const auto command = controller.solve(state, path, {}, &field, SE2{}, params, 3.0);
    const double previous_clearance = 6.0 - state.x - 0.4;
    state = step(state, command, params.dt);
    const double clearance = 6.0 - state.x - 0.4;
    if (previous_clearance > 1.5 && clearance <= 1.5) {
      result.speed_at_1_5m = std::hypot(state.vx, state.vy);
    }
    if (i < 55) {
      result.first_stop_clearance = std::min(result.first_stop_clearance, clearance);
    }
    result.min_clearance = std::min(result.min_clearance, clearance);
  }
  return result;
}

TEST(StaticMargin, StopsTowardWallAsBefore)
{
  // 壁へ向かうときは、近づく速さが全方向の速さと同じなので、元の動きと同じ所から減速し、同じ距離で
  // 止まる。止まった後は経路が壁の先へ続くので横へ動き回り、横の速さで余裕を増やさない分だけ、元の動きより
  // 数 cm 寄る（seed 0〜3 で 0.62〜0.64m → 0.56〜0.60m）
  MppiParams original;
  original.static_margin_approach_only = false;
  const auto before = approachWall(original);
  // 既定は false なので、近づく向きの速さだけを使う設定をここで有効にする
  MppiParams approach_only;
  approach_only.static_margin_approach_only = true;
  const auto after = approachWall(approach_only);
  EXPECT_NEAR(after.speed_at_1_5m, before.speed_at_1_5m, 0.3);
  EXPECT_NEAR(after.first_stop_clearance, before.first_stop_clearance, 0.05);
  EXPECT_GT(after.min_clearance, 0.4);
}

TEST(StaticMargin, FollowsFenceWithoutCrossing)
{
  // 柵と平行な経路を追うとき、柵へ寄っても機体の角は余裕の内側へ入らない（近づく向きの速さだけを使う設定。
  // 既定は false なので、ここで有効にする）
  const FieldBounds bounds{-12.0, 12.0, -7.5, 7.5};
  const auto field = computeDistanceField(makeFieldGrid(bounds, 0.05));
  MppiParams params;
  params.static_margin_approach_only = true;
  for (const double offset : {0.2, 0.3, 0.5}) {
    const double y = bounds.y_max - 0.4 - offset;
    const ReferencePath path({Point2{-10.0, y}, Point2{11.0, y}});
    MppiController controller;
    MppiState state;
    state.x = -10.0;
    state.y = y;
    double min_clearance = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 120; ++i) {
      const auto command = controller.solve(state, path, {}, &field, SE2{}, params, 3.0);
      state = step(state, command, params.dt);
      const double half_extent = 0.4 * (std::abs(std::cos(state.yaw)) + std::abs(std::sin(state.yaw)));
      min_clearance = std::min(min_clearance, bounds.y_max - state.y - half_extent);
    }
    EXPECT_GT(min_clearance, params.footprint.margin_m) << offset;
    EXPECT_GT(state.x, 7.0) << offset;
  }
}

TEST(StaticMargin, RejectsTimeBeyondCornerCheck)
{
  // 速さの上限（3m/s + 1.5rad/s × 外接円 0.566m）× 時間が、4 隅を調べる範囲に足す距離を超えれば拒否する
  MppiParams params;
  EXPECT_FALSE(core_planning::staticMarginError(params, 3.0).has_value());
  params.static_margin_time_s = 0.13;
  EXPECT_TRUE(core_planning::staticMarginError(params, 3.0).has_value());
  EXPECT_FALSE(core_planning::staticMarginError(params, 2.5).has_value());
  params.static_margin_time_s = -0.1;
  EXPECT_TRUE(core_planning::staticMarginError(params, 3.0).has_value());
}

// ---- 狭い通路（narrow_passage） ----

using core_planning::passageSideClearance;
using core_planning::passageSpeedLimit;

TEST(NarrowPassage, MeasuresSideClearanceOnlyBetweenTwoWalls)
{
  // 幅 1.4m の通路の真ん中では、片側の余裕は (1.4 − 0.8) / 2 = 0.3m。真ん中から外れても左右の平均で同じ。
  // 隙間なく埋まった壁では、セルの中心までを測るので、最大で半セル（0.025m）広く読む
  const auto field = corridorField(1.4);
  for (const double y : {0.0, 0.15}) {
    const auto clearance = passageSideClearance(field, SE2{}, Point2{0.0, y}, Point2{1.0, 0.0}, 0.4);
    ASSERT_TRUE(clearance.has_value()) << y;
    EXPECT_NEAR(*clearance, 0.3, 0.03) << y;
  }
  // 通路の手前（左右が開いている）は通路ではない
  EXPECT_FALSE(
    passageSideClearance(field, SE2{}, Point2{-8.0, 0.0}, Point2{1.0, 0.0}, 0.4).has_value());
  // 柵に沿うだけ（片側が開いている）も通路ではない
  const auto fence = computeDistanceField(makeFieldGrid(FieldBounds{-12.0, 12.0, -7.5, 7.5}, 0.05));
  EXPECT_FALSE(
    passageSideClearance(fence, SE2{}, Point2{0.0, 6.6}, Point2{1.0, 0.0}, 0.4).has_value());
}

// 地図と同じく、壁の面に 0.2m おきの点だけがある（中は空の）幅 width の通路（x: -5〜5）
DistanceField dottedCorridorField(const double width)
{
  auto grid = makeFieldGrid(FieldBounds{-12.0, 12.0, -7.5, 7.5}, 0.05);
  for (double x = -5.0; x <= 5.0 + 1e-9; x += 0.2) {
    for (const double y : {-0.5 * width, 0.5 * width}) {
      grid.setOccupied(grid.info().cellAt(x, y));
    }
  }
  return computeDistanceField(grid);
}

TEST(NarrowPassage, FindsDottedWallsLikeTheMap)
{
  // 光線が点の間を抜けても、左右 0.1m の幅で点に当たる。点との位置の関係（0.05m ずつずらす）によらず、
  // 幅 1.4m の通路の片側の余裕は約 0.3m
  const auto field = dottedCorridorField(1.4);
  for (const double x : {-3.0, -2.95, -2.9, -2.85}) {
    const auto clearance = passageSideClearance(field, SE2{}, Point2{x, 0.0}, Point2{1.0, 0.0}, 0.4);
    ASSERT_TRUE(clearance.has_value()) << x;
    EXPECT_NEAR(*clearance, 0.3, 0.04) << x;
  }
  // 閉ループでも、点線の壁の通路を真ん中で通り抜ける
  const MppiParams params;
  const ReferencePath path({Point2{-10.0, 0.0}, Point2{11.0, 0.0}});
  MppiController controller;
  MppiState state;
  state.x = -10.0;
  double min_clearance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 200 && state.x < 7.0; ++i) {
    const auto command = controller.solve(state, path, {}, &field, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    if (std::abs(state.x) <= 5.4) {
      const double half_extent = 0.4 * (std::abs(std::cos(state.yaw)) + std::abs(std::sin(state.yaw)));
      min_clearance = std::min(min_clearance, 0.7 - std::abs(state.y) - half_extent);
    }
  }
  EXPECT_GE(state.x, 7.0);
  EXPECT_GT(min_clearance, params.footprint.margin_m);
}

TEST(NarrowPassage, SpeedLimitKeepsSideOutsideStaticMargin)
{
  // 余裕 − 0.1m（footprint.margin_m）− 0.05m（真ん中から外れてよい距離）を 0.1 秒（static_margin_time_s）で
  // 進む速さ。0.2m 未満は 0（手前で止まる）
  MppiParams params;
  EXPECT_DOUBLE_EQ(passageSpeedLimit(0.19, params), 0.0);
  EXPECT_NEAR(passageSpeedLimit(0.2, params), 0.5, 1e-9);
  EXPECT_NEAR(passageSpeedLimit(0.3, params), 1.5, 1e-9);
  params.narrow.min_side_clearance_m = 0.3;
  EXPECT_DOUBLE_EQ(passageSpeedLimit(0.25, params), 0.0);
  params.narrow.enabled = false;
  EXPECT_TRUE(std::isinf(passageSpeedLimit(0.1, params)));
}

TEST(NarrowPassage, EntersPassagesWithEnoughSideClearance)
{
  // 片側の余裕が 0.2m 以上（幅 1.2m 以上）の長い通路は、真ん中を通り抜ける。速さは余裕に合わせて下がり、
  // 機体の側面は静止物の余裕（0.1m）の内側に入らない
  // （隙間なく埋まった壁は、格子の位置によって最大 1 セル広く読むので、速さの上は 1 セル分を足して見る）
  const MppiParams params;
  for (const double width : {1.25, 1.4, 1.6, 2.0, 2.4}) {
    const auto run = runCorridor(width, params);
    const double side = 0.5 * (width - 0.8);
    const double expected = std::min(passageSpeedLimit(side, params), 2.87);
    const double upper = std::min(passageSpeedLimit(side + 0.05, params), 2.87);
    EXPECT_TRUE(run.passed) << width;
    EXPECT_GT(run.min_clearance, params.footprint.margin_m) << width;
    EXPECT_LT(run.mean_speed, upper + 0.15) << width;
    EXPECT_GT(run.mean_speed, 0.7 * expected) << width;
  }
}

TEST(NarrowPassage, StopsBeforePassageNarrowerThanThreshold)
{
  // 片側の余裕が 0.2m 未満（幅 1.1m、片側 0.15m）の通路には入らず、機体の前の端が入口（x = -5）の手前で
  // 止まる（元の動き）
  const MppiParams params;
  const auto field = corridorField(1.1);
  const ReferencePath path({Point2{-10.0, 0.0}, Point2{11.0, 0.0}});
  MppiController controller;
  MppiState state;
  state.x = -10.0;
  double max_front = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < 150; ++i) {
    const auto command = controller.solve(state, path, {}, &field, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    max_front = std::max(max_front, state.x + 0.4);
  }
  EXPECT_LT(max_front, -5.0);
  EXPECT_LT(std::hypot(state.vx, state.vy), 0.1);
}

TEST(NarrowPassage, DisabledKeepsOriginalStopAtLongCorridor)
{
  // narrow_passage.enabled が false なら元の動き。幅 2.0m の長い通路でも入口の手前で止まる
  MppiParams params;
  params.narrow.enabled = false;
  const auto run = runCorridor(2.0, params);
  EXPECT_FALSE(run.passed);
  EXPECT_LT(run.mean_speed, 1.0);
}

// ---- 向きの決め方（heading）と見えない側の速さの上限（blind_side） ----

using core_planning::BlindSideLimit;
using core_planning::BlindSideParams;
using core_planning::HeadingMode;
using core_planning::headingTarget;
using core_planning::wrapAngle;

MppiState stateAt(const double x, const double y, const double yaw)
{
  MppiState state;
  state.x = x;
  state.y = y;
  state.yaw = yaw;
  return state;
}

// 速度の向きの、機体の前の向きからの角度 [rad]（0〜π）
double angleFromFront(const double vx, const double vy, const double yaw)
{
  return std::abs(wrapAngle(std::atan2(vy, vx) - yaw));
}

TEST(BlindSide, ScalesOnlyOutsideVisibleRange)
{
  const BlindSideLimit limit(BlindSideParams{});
  const auto at = [](const double degrees, const double speed) {
      const double angle = degrees * M_PI / 180.0;
      return std::pair<double, double>{speed * std::cos(angle), speed * std::sin(angle)};
    };
  // 横（90°）と 95° は見える範囲。100° と真後ろは 0.5m/s までに縮める
  for (const double degrees : {0.0, 90.0, -90.0, 95.0}) {
    const auto [vx, vy] = at(degrees, 3.0);
    EXPECT_DOUBLE_EQ(limit.scale(vx, vy, 1.0, 0.0), 1.0) << degrees;
    EXPECT_FALSE(limit.isBlind(vx, vy, 1.0, 0.0)) << degrees;
  }
  for (const double degrees : {100.0, -100.0, 180.0}) {
    const auto [vx, vy] = at(degrees, 2.0);
    EXPECT_NEAR(limit.scale(vx, vy, 1.0, 0.0), 0.25, 1e-12) << degrees;
    EXPECT_TRUE(limit.isBlind(vx, vy, 1.0, 0.0)) << degrees;
  }
  // 上限より遅ければ縮めない。機体の前の向きで判定する
  EXPECT_DOUBLE_EQ(limit.scale(-0.4, 0.0, 1.0, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(limit.scale(-2.0, 0.0, -1.0, 0.0), 1.0);
  BlindSideParams disabled;
  disabled.enabled = false;
  EXPECT_DOUBLE_EQ(BlindSideLimit(disabled).scale(-2.0, 0.0, 1.0, 0.0), 1.0);
}

TEST(Heading, TargetFollowsPathAheadAndGoalYawNearGoal)
{
  // L 字の経路。ゴール (10, 10) の向きは π/2 ではなく π（進む向きと違う）
  const ReferencePath path({Point2{0.0, 0.0}, Point2{10.0, 0.0}, Point2{10.0, 10.0}}, M_PI);
  const MppiParams params;
  // 角の 1.5m 手前では、1m 先までの弦は x の向き
  EXPECT_NEAR(*headingTarget(stateAt(8.5, 0.0, 0.0), path, params), 0.0, 1e-9);
  // 角の 0.5m 手前では、弦が角をまたいで 45° になる
  EXPECT_NEAR(*headingTarget(stateAt(9.5, 0.0, 0.0), path, params), M_PI / 4.0, 1e-9);
  // 経路から横へ外れていても、経路の向きを使う（避けている間に回らない）
  EXPECT_NEAR(*headingTarget(stateAt(5.0, -1.5, 0.0), path, params), 0.0, 1e-9);
  // ゴールの 2m 手前は進む向き、0.8m 手前はゴールの向き
  EXPECT_NEAR(*headingTarget(stateAt(10.0, 8.0, 0.0), path, params), M_PI / 2.0, 1e-9);
  EXPECT_NEAR(*headingTarget(stateAt(10.0, 9.2, 0.0), path, params), M_PI, 1e-9);
  // ゴールの向きがなければ、ゴールの近くでは今の向きを保つ
  const ReferencePath no_yaw({Point2{0.0, 0.0}, Point2{10.0, 0.0}});
  EXPECT_FALSE(headingTarget(stateAt(9.5, 0.0, 0.3), no_yaw, params).has_value());
  EXPECT_NEAR(*headingTarget(stateAt(5.0, 0.0, 0.3), no_yaw, params), 0.0, 1e-9);
}

TEST(Heading, KeepsTravelDirectionWhileHoldingBeforeGoal)
{
  // ゴールの手前で待つとき（経路を 2m 手前で切る）は、待つ位置でもゴールの向きへ回らず、ゴールへ
  // 向かう向きを保つ。ゴールまでの残りは切る前の経路で数える
  const ReferencePath path({Point2{0.0, 0.0}, Point2{10.0, 0.0}}, M_PI / 2.0);
  const MppiParams params;
  const ReferencePath held = path.truncated(8.0);
  EXPECT_NEAR(held.distanceBeyondEnd(), 2.0, 1e-9);
  EXPECT_NEAR(*headingTarget(stateAt(7.5, 0.0, 0.0), held, params), 0.0, 1e-9);
  EXPECT_NEAR(*headingTarget(stateAt(8.0, 0.0, 0.0), held, params), 0.0, 1e-9);
  // 切らない経路では、ゴールの 1m 以内でゴールの向き
  EXPECT_NEAR(*headingTarget(stateAt(9.5, 0.0, 0.0), path, params), M_PI / 2.0, 1e-9);
  // 切った経路をさらに切ると、切った長さを足していく
  EXPECT_NEAR(held.truncated(6.0).distanceBeyondEnd(), 4.0, 1e-9);
  EXPECT_NEAR(path.distanceBeyondEnd(), 0.0, 1e-12);
}

TEST(Heading, TurnsTowardTravelDirectionWithinLimits)
{
  // 前（+x）を向いて止まった機体に、左（+y）へ進む経路を与える。左は見える範囲なので速さを抑えず、
  // 回る速さと角加速度の上限の中で、進む向き（π/2）へ回る
  const ReferencePath path({Point2{0.0, 0.0}, Point2{0.0, 20.0}}, 0.0);
  const MppiParams params;
  MppiController controller;
  MppiState state;
  double max_speed = 0.0;
  for (int i = 0; i < 60; ++i) {
    const MppiState previous = state;
    const auto command = controller.solve(state, path, {}, nullptr, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    EXPECT_LE(std::abs(command.wz), params.max_angular_speed_radps + 1e-9);
    EXPECT_LE(
      std::abs(command.wz - previous.wz), params.max_angular_accel_radps2 * params.dt + 1e-9);
    max_speed = std::max(max_speed, std::hypot(state.vx, state.vy));
  }
  EXPECT_NEAR(state.yaw, M_PI / 2.0, 0.05);
  EXPECT_GT(state.y, 8.0);
  EXPECT_GT(max_speed, 2.5);
}

TEST(Heading, AlignsWithGoalYawNearGoal)
{
  // ゴール (6, 0) の向きは π/2。遠い間は進む向き（0）を保ち、残り 1m からゴールの向きへ回る
  const ReferencePath path({Point2{0.0, 0.0}, Point2{6.0, 0.0}}, M_PI / 2.0);
  const MppiParams params;
  MppiController controller;
  MppiState state;
  double max_abs_yaw_far = 0.0;
  for (int i = 0; i < 120; ++i) {
    const auto command = controller.solve(state, path, {}, nullptr, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    if (path.remaining(state.x, state.y) > params.heading.goal_align_distance_m + 0.3) {
      max_abs_yaw_far = std::max(max_abs_yaw_far, std::abs(state.yaw));
    }
  }
  EXPECT_LT(max_abs_yaw_far, 1e-9);
  EXPECT_NEAR(state.yaw, M_PI / 2.0, 0.05);
  EXPECT_NEAR(state.x, 6.0, 0.2);
}

TEST(Heading, FixedModeKeepsFixedYawUntilNearGoal)
{
  // heading.mode: fixed では、遠い間は fixed_yaw を保ち、ゴールの 1m 以内でゴールの向きへ回る
  MppiParams params;
  params.heading.mode = HeadingMode::kFixed;
  params.heading.fixed_yaw = 1.0;
  const ReferencePath path({Point2{0.0, 0.0}, Point2{10.0, 0.0}, Point2{10.0, 10.0}}, M_PI);
  for (const double x : {0.0, 9.5}) {
    EXPECT_NEAR(*headingTarget(stateAt(x, 0.0, 0.3), path, params), 1.0, 1e-12) << x;
  }
  EXPECT_NEAR(*headingTarget(stateAt(10.0, 9.2, 0.3), path, params), M_PI, 1e-12);
  // ゴールの向きがなければ、ゴールの近くでも保つ向きのまま
  const ReferencePath no_yaw({Point2{0.0, 0.0}, Point2{10.0, 0.0}});
  EXPECT_NEAR(*headingTarget(stateAt(9.5, 0.0, 0.3), no_yaw, params), 1.0, 1e-12);
  // 保つ向きが決まっていなければ、今の向きを保つ
  params.heading.fixed_yaw.reset();
  EXPECT_FALSE(headingTarget(stateAt(5.0, 0.0, 0.3), path, params).has_value());
}

TEST(Heading, FixedModeDrivesWithoutTurning)
{
  // 前（+x）を向いて止まった機体に、左（+y）へ進む経路を与える。fixed（保つ向き 0）では回らずに横へ進む
  const ReferencePath path({Point2{0.0, 0.0}, Point2{0.0, 20.0}});
  MppiParams params;
  params.heading.mode = HeadingMode::kFixed;
  params.heading.fixed_yaw = 0.0;
  MppiController controller;
  MppiState state;
  double max_abs_yaw = 0.0;
  for (int i = 0; i < 40; ++i) {
    const auto command = controller.solve(state, path, {}, nullptr, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    max_abs_yaw = std::max(max_abs_yaw, std::abs(state.yaw));
  }
  EXPECT_LT(max_abs_yaw, 1e-9);
  EXPECT_GT(state.y, 5.0);
}

TEST(Heading, ForwardModeLimitsSpeedWhileTurning)
{
  MppiParams params;
  params.heading.mode = HeadingMode::kForward;
  const MppiState state = stateAt(0.0, 0.0, 0.0);
  // 向きの差が 45° を超える間だけ 0.5m/s までに抑える。差の符号と、元の上限が小さいときも見る
  EXPECT_DOUBLE_EQ(core_planning::forwardTurnSpeedLimit(state, M_PI / 2.0, params, 3.0), 0.5);
  EXPECT_DOUBLE_EQ(core_planning::forwardTurnSpeedLimit(state, -M_PI / 2.0, params, 3.0), 0.5);
  EXPECT_DOUBLE_EQ(core_planning::forwardTurnSpeedLimit(state, M_PI / 2.0, params, 0.3), 0.3);
  EXPECT_DOUBLE_EQ(core_planning::forwardTurnSpeedLimit(state, M_PI / 6.0, params, 3.0), 3.0);
  EXPECT_DOUBLE_EQ(core_planning::forwardTurnSpeedLimit(state, std::nullopt, params, 3.0), 3.0);
  // travel では抑えない
  MppiParams travel;
  EXPECT_DOUBLE_EQ(core_planning::forwardTurnSpeedLimit(state, M_PI / 2.0, travel, 3.0), 3.0);

  // 前（+x）を向いて止まった機体に、左（+y）へ進む経路を与える。向きの差が 45° 以下になるまでは 0.5m/s
  // 程度で進み、向いてからは速く走る。travel は回りながら速く走る
  const ReferencePath path({Point2{0.0, 0.0}, Point2{0.0, 20.0}}, 0.0);
  const auto run = [&path](const MppiParams & run_params) {
      MppiController controller;
      MppiState current;
      double turning_max_speed = 0.0;
      double max_speed = 0.0;
      for (int i = 0; i < 60; ++i) {
        const auto command = controller.solve(current, path, {}, nullptr, SE2{}, run_params, 3.0);
        current = step(current, command, run_params.dt);
        const double speed = std::hypot(current.vx, current.vy);
        if (std::abs(wrapAngle(M_PI / 2.0 - current.yaw)) > M_PI / 4.0 + 0.1) {
          turning_max_speed = std::max(turning_max_speed, speed);
        }
        max_speed = std::max(max_speed, speed);
      }
      return std::tuple<double, double, MppiState>{turning_max_speed, max_speed, current};
    };
  const auto [forward_turning, forward_max, forward_end] = run(params);
  EXPECT_LT(forward_turning, 0.5 + 0.2 + 1e-9);
  EXPECT_GT(forward_max, 2.5);
  EXPECT_NEAR(forward_end.yaw, M_PI / 2.0, 0.05);
  const auto [travel_turning, travel_max, travel_end] = run(travel);
  EXPECT_GT(travel_turning, 1.0);
}

TEST(Heading, GoalModeKeepsPreviousBehavior)
{
  // heading.mode: goal では、先を見る距離と切り替える距離によらず、ゴールの向き（なければ保つ）へ回る
  MppiParams goal_mode;
  goal_mode.heading.mode = HeadingMode::kGoal;
  const ReferencePath path({Point2{0.0, 0.0}, Point2{10.0, 0.0}, Point2{10.0, 10.0}}, 2.0);
  for (const double x : {0.0, 9.5}) {
    EXPECT_NEAR(*headingTarget(stateAt(x, 0.0, 0.0), path, goal_mode), 2.0, 1e-12);
  }
  EXPECT_FALSE(
    headingTarget(stateAt(5.0, 0.0, 0.3), ReferencePath({Point2{0.0, 0.0}, Point2{10.0, 0.0}}),
    goal_mode).has_value());
  MppiParams other = goal_mode;
  other.heading.lookahead_m = 3.0;
  other.heading.goal_align_distance_m = 5.0;
  MppiController first(7U);
  MppiController second(7U);
  MppiState a = stateAt(0.0, 0.0, 0.3);
  MppiState b = a;
  for (int i = 0; i < 40; ++i) {
    const auto command_a = first.solve(a, path, {}, nullptr, SE2{}, goal_mode, 3.0);
    const auto command_b = second.solve(b, path, {}, nullptr, SE2{}, other, 3.0);
    EXPECT_DOUBLE_EQ(command_a.vx, command_b.vx);
    EXPECT_DOUBLE_EQ(command_a.vy, command_b.vy);
    EXPECT_DOUBLE_EQ(command_a.wz, command_b.wz);
    a = step(a, command_a, goal_mode.dt);
    b = step(b, command_b, other.dt);
  }
}

TEST(BlindSide, CapsSpeedTowardBlindSideAndKeepsSidewaysSpeed)
{
  // 向きを保つ動き（goal、ゴールの向きなし）で、真後ろ（-x）へ進む経路は 0.5m/s までに抑え、
  // 横（+y、90°）へ進む経路は抑えない
  MppiParams params;
  params.heading.mode = HeadingMode::kGoal;
  for (const auto & [goal, expect_blind] :
    {std::pair<Point2, bool>{Point2{-10.0, 0.0}, true}, {Point2{0.0, 10.0}, false}})
  {
    const ReferencePath path({Point2{0.0, 0.0}, goal});
    MppiController controller;
    MppiState state;
    double max_speed = 0.0;
    for (int i = 0; i < 40; ++i) {
      const auto command = controller.solve(state, path, {}, nullptr, SE2{}, params, 3.0);
      state = step(state, command, params.dt);
      max_speed = std::max(max_speed, std::hypot(command.vx, command.vy));
      EXPECT_NEAR(state.yaw, 0.0, 1e-12);
    }
    if (expect_blind) {
      // 上限で縮めた系列は向きがばらつくので、重み付き平均は上限より少し遅い（約 0.4m/s）
      EXPECT_LE(max_speed, params.blind_side.speed_mps + 1e-9);
      EXPECT_GT(max_speed, 0.3);
      EXPECT_LT(state.x, -1.0);
    } else {
      EXPECT_GT(max_speed, 2.5);
    }
  }
  // 上限を切れば、真後ろへも速く進む（前の動き）
  params.blind_side.enabled = false;
  const ReferencePath back({Point2{0.0, 0.0}, Point2{-10.0, 0.0}});
  MppiController controller;
  MppiState state;
  double max_speed = 0.0;
  for (int i = 0; i < 40; ++i) {
    const auto command = controller.solve(state, back, {}, nullptr, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    max_speed = std::max(max_speed, std::hypot(command.vx, command.vy));
  }
  EXPECT_GT(max_speed, 2.5);
}

TEST(BlindSide, SpeedsUpAfterTurningTowardTravelDirection)
{
  // 前（+x）を向いて止まった機体に、真後ろへ進む経路を与える。進む向きが見えない間は 0.5m/s までで、
  // 回って見える範囲に入ってから速くなる
  const ReferencePath path({Point2{0.0, 0.0}, Point2{-40.0, 0.0}}, 0.0);
  const MppiParams params;
  MppiController controller;
  MppiState state;
  double max_blind_speed = 0.0;
  double max_speed = 0.0;
  for (int i = 0; i < 70; ++i) {
    const auto command = controller.solve(state, path, {}, nullptr, SE2{}, params, 3.0);
    const double speed = std::hypot(command.vx, command.vy);
    if (speed > 1e-6 && angleFromFront(command.vx, command.vy, state.yaw) >
      params.blind_side.visible_half_angle_deg * M_PI / 180.0)
    {
      max_blind_speed = std::max(max_blind_speed, speed);
    }
    max_speed = std::max(max_speed, speed);
    state = step(state, command, params.dt);
  }
  EXPECT_LE(max_blind_speed, params.blind_side.speed_mps + 1e-9);
  EXPECT_NEAR(std::abs(wrapAngle(state.yaw)), M_PI, 0.05);
  EXPECT_GT(max_speed, 2.5);
  EXPECT_LT(state.x, -8.0);
}

TEST(Heading, KeepsHeadingAlongPathWhileDodging)
{
  // 向かってくる相手を横へ避ける間も、向きは経路の向き（0）のまま回らず、横の動きは抑えない
  const ReferencePath path({Point2{0.0, 0.0}, Point2{30.0, 0.0}}, 0.0);
  const MppiParams params;
  MppiController controller;
  MppiState state;
  state.vx = 3.0;
  auto object = movingObject(14.0, 0.0, -3.0, 0.0);
  double min_gap = std::numeric_limits<double>::infinity();
  double max_abs_y = 0.0;
  while (object.x > state.x - 2.0) {
    const auto command = controller.solve(state, path, {object}, nullptr, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    object.x += object.vx * params.dt;
    EXPECT_DOUBLE_EQ(command.wz, 0.0);
    min_gap = std::min(min_gap, footprintGap(state, object, 0.4));
    max_abs_y = std::max(max_abs_y, std::abs(state.y));
  }
  EXPECT_DOUBLE_EQ(state.yaw, 0.0);
  EXPECT_GT(min_gap, 0.0);
  EXPECT_GT(max_abs_y, 0.4);
}

TEST(Heading, RotationIsPartOfPredictedFootprint)
{
  // 下（-y）に止まった物体の 0.1m 横で、真後ろ（-x）へ進む経路を与える。その場で 45° 回ると機体の角が
  // 物体に食い込むが、予測の各ステップで回った向きの機体を物体と比べるので、離れながら回る
  const ReferencePath path({Point2{0.0, 0.0}, Point2{-40.0, 0.0}}, 0.0);
  const MppiParams params;
  PredictedObject box;
  box.x = 0.0;
  box.y = -1.0;
  box.sigma = 0.05;
  box.kind = ObjectKind::kUnknown;
  const double radius = params.objects.unknown_radius_m;
  ASSERT_GT(footprintGap(stateAt(0.0, 0.0, 0.0), box, radius), 0.0);
  ASSERT_LT(footprintGap(stateAt(0.0, 0.0, M_PI / 4.0), box, radius), 0.0);
  MppiController controller;
  MppiState state;
  double min_gap = std::numeric_limits<double>::infinity();
  bool predicted_rotation = false;
  for (int i = 0; i < 40; ++i) {
    const auto command = controller.solve(state, path, {box}, nullptr, SE2{}, params, 3.0);
    const auto & trajectory = controller.lastTrajectory();
    predicted_rotation = predicted_rotation ||
      std::abs(wrapAngle(trajectory.back().yaw - trajectory.front().yaw)) > 0.5;
    state = step(state, command, params.dt);
    min_gap = std::min(min_gap, footprintGap(state, box, radius));
  }
  EXPECT_TRUE(predicted_rotation);
  EXPECT_GT(min_gap, 0.0);
  EXPECT_NEAR(std::abs(wrapAngle(state.yaw)), M_PI, 0.1);
}

TEST(Heading, RotationKeepsCornersInsideFence)
{
  // 柵（y = 7.5）と平行に、機体の左の面が柵から 0.12m の所で止まっている。真後ろ（-x）へ進む経路を与えると
  // 回る途中で角が柵を越える向き（45°）を通るが、予測の各ステップで回った向きの機体の角を距離の場と比べる
  // ので、柵から離れながら回る
  const FieldBounds bounds{-12.0, 12.0, -7.5, 7.5};
  const auto field = computeDistanceField(makeFieldGrid(bounds, 0.05));
  const double y = bounds.y_max - 0.4 - 0.12;
  const ReferencePath path({Point2{0.0, y}, Point2{-30.0, y}}, 0.0);
  const MppiParams params;
  const auto corner_clearance = [&](const MppiState & s) {
      return bounds.y_max - s.y - 0.4 * (std::abs(std::cos(s.yaw)) + std::abs(std::sin(s.yaw)));
    };
  ASSERT_GT(corner_clearance(stateAt(0.0, y, 0.0)), 0.0);
  ASSERT_LT(corner_clearance(stateAt(0.0, y, M_PI / 4.0)), 0.0);
  MppiController controller;
  MppiState state = stateAt(0.0, y, 0.0);
  double min_clearance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 40; ++i) {
    const auto command = controller.solve(state, path, {}, &field, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    min_clearance = std::min(min_clearance, corner_clearance(state));
  }
  EXPECT_GT(min_clearance, 0.0);
  EXPECT_NEAR(std::abs(wrapAngle(state.yaw)), M_PI, 0.1);
  EXPECT_LT(state.x, -3.0);
}

// 東へ straight_m 進み、半径 radius の 1/4 円で左へ曲がって、北へ straight_m 進む経路（0.1m おき）
ReferencePath turnPath(const double straight_m, const double radius)
{
  std::vector<Point2> points;
  for (double x = 0.0; x < straight_m; x += 0.1) {
    points.push_back(Point2{x, 0.0});
  }
  const int arc_steps = static_cast<int>(std::ceil(0.5 * M_PI * radius / 0.1));
  for (int i = 0; i <= arc_steps; ++i) {
    const double angle = -0.5 * M_PI + 0.5 * M_PI * static_cast<double>(i) / arc_steps;
    points.push_back(
      Point2{straight_m + radius * std::cos(angle), radius + radius * std::sin(angle)});
  }
  for (double y = radius + 0.1; y <= radius + straight_m; y += 0.1) {
    points.push_back(Point2{straight_m + radius, y});
  }
  return ReferencePath(points);
}

TEST(CurveSpeed, MeasuresCurvatureOfArcAndStraight)
{
  const ReferencePath path = turnPath(10.0, 3.0);
  // 円弧の真ん中（直線 10m + 1/8 円）
  const double middle = 10.0 + 0.25 * M_PI * 3.0;
  EXPECT_NEAR(core_planning::pathCurvature(path, middle, 0.5), 1.0 / 3.0, 0.03);
  EXPECT_NEAR(core_planning::pathCurvature(path, 3.0, 0.5), 0.0, 1e-9);
  const ReferencePath single({Point2{0.0, 0.0}});
  EXPECT_EQ(core_planning::pathCurvature(single, 0.0, 0.5), 0.0);
}

TEST(CurveSpeed, LimitKeepsLateralAccelUnderMaximum)
{
  MppiParams params;
  // 既定（0）は落とさない（前の動き）
  EXPECT_TRUE(std::isinf(core_planning::curveSpeedLimit(0.5, params)));
  params.curve.max_lateral_accel_mps2 = 2.0;
  EXPECT_NEAR(core_planning::curveSpeedLimit(0.5, params), 2.0, 1e-9);
  EXPECT_TRUE(std::isinf(core_planning::curveSpeedLimit(0.0, params)));
}

struct TurnRun
{
  // 経路の線からの距離の最大（曲がりの前後 5m を含む区間）
  double max_deviation{0.0};
  // 曲がりの外（円の中心から半径より遠い側、出口の先を含む）へのはみ出しの最大
  double max_outward{0.0};
  // 円弧の区間での速さの最大
  double max_arc_speed{0.0};
  bool reached{false};
};

// 速さの上限 speed_limit で、半径 2m の左折を、指令に対して時定数 time_constant_s で遅れて動く機体で
// 走らせる（0 なら指令どおり）
TurnRun runTurn(const MppiParams & params, const double speed_limit, const double time_constant_s)
{
  constexpr double kRadius = 2.0;
  constexpr double kStraight = 15.0;
  const ReferencePath path = turnPath(kStraight, kRadius);
  MppiController controller;
  MppiState state;
  TurnRun run;
  for (int i = 0; i < 400 && !run.reached; ++i) {
    const auto command = controller.solve(state, path, {}, nullptr, SE2{}, params, speed_limit);
    if (time_constant_s > 0.0) {
      // 1 周期を 10 に分けて、指令の速度へ時定数で近づける
      for (int k = 0; k < 10; ++k) {
        const double dt = 0.1 * params.dt;
        const double gain = std::min(dt / time_constant_s, 1.0);
        state.vx += (command.vx - state.vx) * gain;
        state.vy += (command.vy - state.vy) * gain;
        state.wz = command.wz;
        state.x += state.vx * dt;
        state.y += state.vy * dt;
        state.yaw += state.wz * dt;
      }
    } else {
      state = step(state, command, params.dt);
    }
    const double s = path.project(state.x, state.y);
    if (s >= kStraight - 5.0 && s <= kStraight + 0.5 * M_PI * kRadius + 5.0) {
      const Point2 closest = path.pointAt(s);
      run.max_deviation =
        std::max(run.max_deviation, std::hypot(state.x - closest.x, state.y - closest.y));
    }
    // 曲がりの外: 円弧の区間は円の中心から半径より遠い側、出口の先は x = kStraight + kRadius より東
    if (state.x >= kStraight && state.y <= kRadius) {
      run.max_outward = std::max(
        run.max_outward, std::hypot(state.x - kStraight, state.y - kRadius) - kRadius);
      run.max_arc_speed = std::max(run.max_arc_speed, std::hypot(state.vx, state.vy));
    } else if (state.y > kRadius) {
      run.max_outward = std::max(run.max_outward, state.x - (kStraight + kRadius));
    }
    run.reached = state.y >= kRadius + 10.0;
  }
  return run;
}

TEST(CurveSpeed, SlowsForTurnAndStaysCloseToLine)
{
  // 参照軌道が曲がりでも 5m/s のまま進むと、計画の加速度（2.5m/s2）では曲がりきれず、線から大きく
  // 外れる。指令どおりに動く機体では内側を切り（1.4m）、指令に 0.1 秒遅れる機体では外へ膨らむ（3.8m）。
  // 横の加速度 2.0m/s2 の上限なら、半径 2m の曲がりの手前で落とし、ずれは半分未満（0.15m、1.1m）になる
  MppiParams params;
  params.max_accel_mps2 = 2.5;
  params.reference_decel_mps2 = 1.8;
  for (const double time_constant : {0.0, 0.1}) {
    params.curve.max_lateral_accel_mps2 = 0.0;
    const TurnRun original = runTurn(params, 5.0, time_constant);
    params.curve.max_lateral_accel_mps2 = 2.0;
    const TurnRun limited = runTurn(params, 5.0, time_constant);
    ASSERT_TRUE(original.reached) << time_constant;
    ASSERT_TRUE(limited.reached) << time_constant;
    std::printf(
      "時定数 %.1f: 元 線から %.2f m、外 %.2f m、円弧 %.2f m/s / 上限あり 線から %.2f m、外 %.2f m、円弧 %.2f m/s\n",
      time_constant, original.max_deviation, original.max_outward, original.max_arc_speed,
      limited.max_deviation, limited.max_outward, limited.max_arc_speed);
    EXPECT_LT(limited.max_deviation, 0.5 * original.max_deviation) << time_constant;
    EXPECT_LT(limited.max_outward, 0.5 * original.max_outward + 0.05) << time_constant;
    if (time_constant == 0.0) {
      EXPECT_LT(limited.max_deviation, 0.3);
    }
  }
}

TEST(TrackMotion, RequiresSteadyVisibleVelocity)
{
  using core_planning::MotionFilterParams;
  using core_planning::TrackMotionFilter;
  const MotionFilterParams params;
  TrackMotionFilter filter;
  // 0.5 秒続くまでは動いているとみなさない
  EXPECT_FALSE(filter.update(1U, 10.0, 1.0, 0.0, false, params));
  EXPECT_FALSE(filter.update(1U, 10.4, 1.0, 0.0, false, params));
  EXPECT_TRUE(filter.update(1U, 10.5, 1.0, 0.0, false, params));
  // 見えなくなる（coasting）と途切れ、見え直してから数え直す
  EXPECT_FALSE(filter.update(1U, 10.6, 1.0, 0.0, true, params));
  EXPECT_FALSE(filter.update(1U, 10.7, 1.0, 0.0, false, params));
  EXPECT_TRUE(filter.update(1U, 11.2, 1.0, 0.0, false, params));
  // 速度の不確かさが大きい、遅い
  EXPECT_FALSE(filter.update(1U, 11.3, 1.0, 0.5, false, params));
  EXPECT_FALSE(filter.update(2U, 11.3, 0.2, 0.0, false, params));
  // 無効なら速さだけで決める（前の動き）
  MotionFilterParams disabled;
  disabled.enabled = false;
  EXPECT_TRUE(filter.update(3U, 11.3, 1.0, 0.5, true, disabled));
  EXPECT_FALSE(filter.update(3U, 11.3, 0.2, 0.0, false, disabled));
}

TEST(TrackMotion, ForgetsOldTracks)
{
  using core_planning::MotionFilterParams;
  using core_planning::TrackMotionFilter;
  const MotionFilterParams params;
  TrackMotionFilter filter;
  EXPECT_FALSE(filter.update(5U, 0.0, 1.0, 0.0, false, params));
  filter.prune(3.0, 2.0);
  // 忘れたので、0.5 秒後でも数え直し
  EXPECT_FALSE(filter.update(5U, 3.1, 1.0, 0.0, false, params));
}

TEST(OncomingRule, SafeSpeedStopsBeforeTheRobotArrives)
{
  using core_planning::oncomingSafeSpeed;
  // v^2 / (2 a) + c v / a = gap - margin
  const double v = oncomingSafeSpeed(5.0, 3.0, 0.3, 1.8);
  EXPECT_NEAR(v * v / (2.0 * 1.8) + 3.0 * v / 1.8, 4.7, 1e-9);
  // 相手が止まっていれば、止まれる速さ
  EXPECT_NEAR(oncomingSafeSpeed(5.0, 0.0, 0.3, 1.8), std::sqrt(2.0 * 1.8 * 4.7), 1e-9);
  // 隙間が余裕より狭ければ止まる
  EXPECT_DOUBLE_EQ(oncomingSafeSpeed(0.2, 3.0, 0.3, 1.8), 0.0);
}

PredictedObject withId(PredictedObject object, const std::uint32_t id)
{
  object.id = id;
  return object;
}

TEST(OncomingRule, HeadOnRobotOnPathStopsBeforeMidpoint)
{
  using core_planning::OncomingMonitor;
  using core_planning::oncomingSafeSpeed;
  const ReferencePath path({Point2{0.0, 0.0}, Point2{20.0, 0.0}});
  // ゴールの遠さによらず停止線で止まる設定で、停止線と速さの上限を確かめる
  MppiParams params;
  params.oncoming.pass_goal_margin_m = -1.0;
  OncomingMonitor monitor;
  MppiState state;
  state.vx = 2.0;
  const auto first = monitor.update(
    0.0, state, path, {withId(movingObject(10.0, 0.2, -2.0, 0.0), 7U)}, params, 3.0);
  // 中点（5m）より、前の端（0.4 + 余裕 0.1）が 0.3m 手前
  ASSERT_TRUE(first.stop_length.has_value());
  EXPECT_NEAR(*first.stop_length, 5.0 - 0.3 - 0.5, 0.05);
  EXPECT_EQ(first.yielding, 1);
  ASSERT_EQ(first.held_ids.size(), 1U);
  EXPECT_EQ(first.held_ids.front(), 7U);
  // 相手がそのまま来ても、届く前に止まり終える速さ（前の端どうしの隙間 10 - 0.5 - 0.55）
  EXPECT_NEAR(
    first.speed_limit,
    oncomingSafeSpeed(10.0 - 0.5 - 0.55, 2.0, 0.3, params.reference_decel_mps2), 1e-6);
  // 相手が近づいても、中点は出会ったときのまま
  const auto later = monitor.update(
    0.5, state, path, {withId(movingObject(9.0, 0.2, -2.0, 0.0), 7U)}, params, 3.0);
  ASSERT_TRUE(later.stop_length.has_value());
  EXPECT_NEAR(*later.stop_length, *first.stop_length, 0.05);
  EXPECT_LT(later.speed_limit, first.speed_limit);
}

TEST(OncomingRule, HeadOnRobotFarFromGoalIsPassedOnRuleSide)
{
  using core_planning::OncomingMonitor;
  // 出会う所（中点 5m）からゴール（20m）まで 15m。止まらずに、規則の側へよけて通る（予測の位置をよける）
  const ReferencePath path({Point2{0.0, 0.0}, Point2{20.0, 0.0}});
  const MppiParams params;
  OncomingMonitor monitor;
  MppiState state;
  state.vx = 2.0;
  for (const double t : {0.0, 0.5, 1.0}) {
    const auto plan = monitor.update(
      t, state, path, {withId(movingObject(10.0 - 2.0 * t, 0.2, -2.0, 0.0), 7U)}, params, 3.0);
    EXPECT_FALSE(plan.stop_length.has_value());
    EXPECT_TRUE(plan.held_ids.empty());
    EXPECT_EQ(plan.yielding, 0);
    EXPECT_TRUE(std::isinf(plan.speed_limit));
  }
}

TEST(OncomingRule, HeadOnRobotNearGoalStops)
{
  using core_planning::OncomingMonitor;
  // 出会う所（中点 2.25m）からゴール（5m）まで 2.75m で、3m より近い。停止線で止まる
  const ReferencePath path({Point2{0.0, 0.0}, Point2{5.0, 0.0}});
  const MppiParams params;
  OncomingMonitor monitor;
  MppiState state;
  state.vx = 1.0;
  const auto plan = monitor.update(
    0.0, state, path, {withId(movingObject(4.5, 0.0, -1.0, 0.0), 8U)}, params, 3.0);
  ASSERT_TRUE(plan.stop_length.has_value());
  EXPECT_NEAR(*plan.stop_length, 2.25 - 0.3 - 0.5, 0.05);
  EXPECT_EQ(plan.held_ids.size(), 1U);
}

TEST(OncomingRule, HeadOnRobotBesidePathIsHeldWithoutStopping)
{
  using core_planning::OncomingMonitor;
  const ReferencePath path({Point2{0.0, 0.0}, Point2{20.0, 0.0}});
  const MppiParams params;
  OncomingMonitor monitor;
  MppiState state;
  state.vx = 2.0;
  // 経路から 3m 横を、逆向きに通る（自機の半幅 0.4 + 余裕 0.1 + 相手 0.55 より外）
  const auto plan = monitor.update(
    0.0, state, path, {withId(movingObject(10.0, 3.0, -2.0, 0.0), 3U)}, params, 3.0);
  EXPECT_FALSE(plan.stop_length.has_value());
  EXPECT_EQ(plan.yielding, 0);
  ASSERT_EQ(plan.held_ids.size(), 1U);
  EXPECT_TRUE(std::isinf(plan.speed_limit));
}

TEST(OncomingRule, KeepsStopLineForResumeTimeThenForgets)
{
  using core_planning::OncomingMonitor;
  const ReferencePath path({Point2{0.0, 0.0}, Point2{20.0, 0.0}});
  MppiParams params;
  params.oncoming.pass_goal_margin_m = -1.0;
  OncomingMonitor monitor;
  MppiState state;
  const auto object = withId(movingObject(8.0, 0.0, -2.0, 0.0), 4U);
  ASSERT_TRUE(monitor.update(0.0, state, path, {object}, params, 3.0).stop_length.has_value());
  // 相手が見えなくなっても resume_after_s（0.5 秒）は停止線を保つ
  const auto kept = monitor.update(0.3, state, path, {}, params, 3.0);
  EXPECT_TRUE(kept.stop_length.has_value());
  EXPECT_EQ(kept.held_ids.size(), 1U);
  const auto released = monitor.update(0.6, state, path, {}, params, 3.0);
  EXPECT_FALSE(released.stop_length.has_value());
  EXPECT_TRUE(released.held_ids.empty());
  // 止まった相手（0.3m/s 未満）は止まる理由にしない
  auto stopped = object;
  stopped.vx = 0.0;
  EXPECT_FALSE(monitor.update(1.0, state, path, {stopped}, params, 3.0).stop_length.has_value());
}

TEST(OncomingRule, CrossingRobotFirstStopsBeforeCrossing)
{
  using core_planning::OncomingMonitor;
  const ReferencePath path({Point2{0.0, 0.0}, Point2{20.0, 0.0}});
  const MppiParams params;
  OncomingMonitor monitor;
  MppiState state;
  state.vx = 1.0;
  // x=6 で直角に横切る相手。相手は 1.5 秒で交点に着き、自機（1m/s から加速）より先
  const auto plan = monitor.update(
    0.0, state, path, {withId(movingObject(6.0, -3.0, 0.0, 2.0), 9U)}, params, 3.0);
  ASSERT_TRUE(plan.stop_length.has_value());
  // 交点の手前で、前の端（0.4 + 0.1）が相手の通り道（0.55 + 余裕 0.1）にかからない所
  EXPECT_NEAR(*plan.stop_length, 6.0 - 0.5 - 0.65, 0.25);
  EXPECT_EQ(plan.held_ids.size(), 1U);
}

TEST(OncomingRule, CrossingRobotAfterUsKeepsCourse)
{
  using core_planning::OncomingMonitor;
  const ReferencePath path({Point2{0.0, 0.0}, Point2{20.0, 0.0}});
  const MppiParams params;
  OncomingMonitor monitor;
  MppiState state;
  state.vx = 3.0;
  // 遠くから遅く横切る相手。自機が先に交点を抜けるので、止まらずに進路を保つ
  const auto plan = monitor.update(
    0.0, state, path, {withId(movingObject(4.0, -9.0, 0.0, 0.5), 2U)}, params, 3.0);
  EXPECT_FALSE(plan.stop_length.has_value());
  EXPECT_EQ(plan.held_ids.size(), 1U);
}

TEST(OncomingRule, DisabledReturnsEmptyPlan)
{
  using core_planning::OncomingMonitor;
  const ReferencePath path({Point2{0.0, 0.0}, Point2{20.0, 0.0}});
  MppiParams params;
  params.oncoming.enabled = false;
  OncomingMonitor monitor;
  const auto plan = monitor.update(
    0.0, MppiState{}, path, {withId(movingObject(8.0, 0.0, -2.0, 0.0), 1U)}, params, 3.0);
  EXPECT_FALSE(plan.stop_length.has_value());
  EXPECT_TRUE(plan.held_ids.empty());
  EXPECT_EQ(plan.yielding, 0);
}

TEST(OncomingRule, MarkHeldSetsOnlyListedIds)
{
  using core_planning::markHeld;
  const auto marked = markHeld(
    {withId(movingObject(1.0, 0.0, 0.0, 0.0), 1U), withId(movingObject(2.0, 0.0, 0.0, 0.0), 2U)},
    {2U});
  EXPECT_FALSE(marked[0].hold);
  EXPECT_TRUE(marked[1].hold);
}

// 正面から来て、自機の進む線の 1.6m 横を通る相手。すれ違うまでの横のずれの最大。今の位置との余裕
// （自機の半幅 0.4 + 余裕 0.1 + 相手 0.55 + 相手の余裕 0.4 = 1.45m）の外なので、hold なら今の位置の罰はかからない
double offsetForPassingRobot(const bool hold)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{30.0, 0.0}});
  const MppiParams params;
  MppiController controller(1U);
  MppiState state;
  state.vx = 2.0;
  auto object = movingObject(10.0, 1.6, -2.0, 0.0);
  object.hold = hold;
  double offset = 0.0;
  for (int i = 0; i < 35; ++i) {
    const auto command = controller.solve(state, path, {object}, nullptr, SE2{}, params, 2.0);
    state = step(state, command, params.dt);
    object.x += object.vx * params.dt;
    offset = std::max(offset, std::abs(state.y));
  }
  return offset;
}

TEST(OncomingRule, HeldRobotIsNotAvoidedSideways)
{
  // 予測の位置をよける（hold でない）と横へ外れ、hold なら経路を保つ
  const double held = offsetForPassingRobot(true);
  EXPECT_LT(held, 0.15);
  EXPECT_GT(offsetForPassingRobot(false), held + 0.1);
}

// 経路の両側（y = ±lateral）に止まったロボットの間を通る。x = 12m まで進めたかと、機体の外形から相手の
// 外接円（0.55m）までの間の最小
std::pair<bool, double> passBetweenParkedRobots(
  const MppiParams & params, const double lateral, const std::uint32_t seed)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{20.0, 0.0}});
  MppiController controller(seed);
  MppiState state;
  state.vx = 1.5;
  const std::vector<PredictedObject> parked{
    movingObject(8.0, lateral, 0.0, 0.0), movingObject(8.0, -lateral, 0.0, 0.0)};
  double min_gap = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 120 && state.x < 12.0; ++i) {
    const auto command = controller.solve(state, path, parked, nullptr, SE2{}, params, 1.5);
    state = step(state, command, params.dt);
    for (const auto & object : parked) {
      min_gap = std::min(min_gap, footprintGap(state, object, 0.55));
    }
  }
  return {state.x >= 12.0, min_gap};
}

TEST(ObjectMargin, PassesGapAtTargetMarginWithoutGoingBelowMinimum)
{
  // ロボットとの間は目安 0.5m、最小 0.2m（2026-11-14 のユーザーの判断。機体の余裕 0.1m 込みで、相手は
  // 外接円で数える）。両側の外接円まで 0.5m ずつ空いた所は、最小の余裕を足しても通り抜け、最小より
  // 近づかない（0.4m ずつの所は、最小の余裕がなくても通れないことがある）
  for (std::uint32_t seed = 0; seed < 3; ++seed) {
    const auto [passed, gap] = passBetweenParkedRobots(MppiParams{}, 1.45, seed);
    EXPECT_TRUE(passed) << "seed " << seed;
    EXPECT_GT(gap, 0.2) << "seed " << seed;
  }
}

// 経路の上に止まっている相手を回り込む側を、すれ違うまでの y の符号で返す
double stationaryPassingOffset(const bool side_right)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{30.0, 0.0}});
  MppiParams params;
  params.passing.side_right = side_right;
  params.passing.weight = 3.0;
  MppiController controller;
  MppiState state;
  const auto object = movingObject(14.0, 0.0, 0.0, 0.0);
  double offset = 0.0;
  for (int i = 0; i < 80 && state.x < object.x; ++i) {
    const auto command = controller.solve(state, path, {object}, nullptr, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    if (std::abs(state.y) > std::abs(offset)) {
      offset = state.y;
    }
  }
  return offset;
}

TEST(OncomingRule, StationaryRobotIsPassedOnRuleSide)
{
  // +x へ進むとき、右は -y 側
  EXPECT_LT(stationaryPassingOffset(true), -0.2);
  EXPECT_GT(stationaryPassingOffset(false), 0.2);
}

// ---- ゴールの向きに合わせて着く（mppi.goal_yaw_*）と、行き過ぎない回り方（yawRateToward） ----

using core_planning::GoalArrival;
using core_planning::GoalArrivalMonitor;
using core_planning::GoalArrivalParams;
using core_planning::yawRateToward;

MppiState movingStateAt(
  const double x, const double y, const double yaw, const double speed, const double wz)
{
  MppiState state = stateAt(x, y, yaw);
  state.vx = speed;
  state.wz = wz;
  return state;
}

// mppi.yaml の回り方（core2026 の足回りの角加速度 1rad/s²）
MppiParams core2026YawParams()
{
  MppiParams params;
  params.max_angular_speed_radps = 1.5;
  params.max_angular_accel_radps2 = 1.0;
  params.yaw_gain_per_s = 1.5;
  params.yaw_brake_accel_ratio = 0.7;
  return params;
}

TEST(GoalArrival, WithoutGoalYawReachesAtPositionAndSpeedAsBefore)
{
  const GoalArrivalParams params;
  const Point2 goal{5.0, 0.0};
  GoalArrivalMonitor monitor;
  EXPECT_EQ(
    monitor.update(0.0, movingStateAt(4.5, 0.0, 0.0, 0.1, 0.0), goal, std::nullopt, params),
    GoalArrival::kApproaching);
  // 位置の許容の中でも速ければ着いていない
  EXPECT_EQ(
    monitor.update(0.1, movingStateAt(4.9, 0.0, 0.0, 0.5, 0.0), goal, std::nullopt, params),
    GoalArrival::kApproaching);
  // ゴールの向きがなければ、向きと回る速さは見ない
  EXPECT_EQ(
    monitor.update(0.2, movingStateAt(4.9, 0.0, 2.0, 0.1, 1.0), goal, std::nullopt, params),
    GoalArrival::kReached);
  // 許容を負にすると、ゴールの向きがあっても位置と速さだけで着いたとする（前の動き）
  GoalArrivalParams position_only = params;
  position_only.yaw_tolerance_rad = -1.0;
  GoalArrivalMonitor other;
  EXPECT_EQ(
    other.update(0.0, movingStateAt(5.0, 0.0, 0.0, 0.0, 0.0), goal, M_PI / 2.0, position_only),
    GoalArrival::kReached);
}

TEST(GoalArrival, TurnsInPlaceUntilYawAndRateAreWithinTolerance)
{
  const GoalArrivalParams params;
  const Point2 goal{5.0, 0.0};
  const double goal_yaw = M_PI / 2.0;
  GoalArrivalMonitor monitor;
  EXPECT_FALSE(monitor.positionReachedAt().has_value());
  // 位置に着いたが向きが 90° 違う
  EXPECT_EQ(
    monitor.update(10.0, movingStateAt(5.05, 0.0, 0.0, 0.05, 0.0), goal, goal_yaw, params),
    GoalArrival::kAligning);
  ASSERT_TRUE(monitor.positionReachedAt().has_value());
  EXPECT_DOUBLE_EQ(*monitor.positionReachedAt(), 10.0);
  // 向きは許容（5°）の中でも、まだ速く回っている
  const double near_yaw = goal_yaw - 3.0 * M_PI / 180.0;
  EXPECT_EQ(
    monitor.update(11.0, movingStateAt(5.0, 0.0, near_yaw, 0.0, 0.5), goal, goal_yaw, params),
    GoalArrival::kAligning);
  // 向きが 6° 違う
  EXPECT_EQ(
    monitor.update(
      11.5, movingStateAt(5.0, 0.0, goal_yaw - 6.0 * M_PI / 180.0, 0.0, 0.1), goal, goal_yaw,
      params),
    GoalArrival::kAligning);
  EXPECT_EQ(
    monitor.update(12.0, movingStateAt(5.0, 0.0, near_yaw, 0.0, 0.1), goal, goal_yaw, params),
    GoalArrival::kReached);
  // ±π をまたいでも差で比べる
  GoalArrivalMonitor across;
  EXPECT_EQ(
    across.update(0.0, movingStateAt(5.0, 0.0, -M_PI + 0.02, 0.0, 0.0), goal, M_PI - 0.02, params),
    GoalArrival::kReached);
}

TEST(GoalArrival, TimesOutOnlyInsidePositionTolerance)
{
  const GoalArrivalParams params;
  const Point2 goal{5.0, 0.0};
  const double goal_yaw = M_PI;
  GoalArrivalMonitor monitor;
  // 速いまま位置の許容を通り抜けても、着いたとは数えない
  EXPECT_EQ(
    monitor.update(9.0, movingStateAt(5.0, 0.0, 0.0, 1.0, 0.0), goal, goal_yaw, params),
    GoalArrival::kApproaching);
  EXPECT_FALSE(monitor.positionReachedAt().has_value());
  EXPECT_EQ(
    monitor.update(10.0, movingStateAt(5.0, 0.0, 0.0, 0.0, 0.0), goal, goal_yaw, params),
    GoalArrival::kAligning);
  EXPECT_EQ(
    monitor.update(14.9, movingStateAt(5.0, 0.0, 1.0, 0.0, 1.0), goal, goal_yaw, params),
    GoalArrival::kAligning);
  // 押されて位置の許容から出ている間は、時間が過ぎても着いたとしない（行き詰まりの判定に任せる）
  EXPECT_EQ(
    monitor.update(15.5, movingStateAt(5.3, 0.0, 1.0, 0.0, 1.0), goal, goal_yaw, params),
    GoalArrival::kApproaching);
  // 位置に戻れば、速さが許容を超えていても時間切れで着いたとする
  EXPECT_EQ(
    monitor.update(16.0, movingStateAt(5.1, 0.0, 1.0, 0.4, 1.0), goal, goal_yaw, params),
    GoalArrival::kTimedOut);
  monitor.reset();
  EXPECT_FALSE(monitor.positionReachedAt().has_value());
}

TEST(GoalArrival, ClockGoingBackRestartsWaiting)
{
  const GoalArrivalParams params;
  const Point2 goal{5.0, 0.0};
  const MppiState misaligned = movingStateAt(5.0, 0.0, 0.0, 0.0, 0.0);
  GoalArrivalMonitor monitor;
  EXPECT_EQ(monitor.update(100.0, misaligned, goal, M_PI, params), GoalArrival::kAligning);
  // シミュレーションのやり直しで時計が戻った
  EXPECT_EQ(monitor.update(3.0, misaligned, goal, M_PI, params), GoalArrival::kAligning);
  EXPECT_EQ(monitor.update(7.9, misaligned, goal, M_PI, params), GoalArrival::kAligning);
  EXPECT_EQ(monitor.update(8.0, misaligned, goal, M_PI, params), GoalArrival::kTimedOut);
}

TEST(Heading, YawRateTowardCanStopWithinRemainingError)
{
  const MppiParams params = core2026YawParams();
  const double brake = params.max_angular_accel_radps2 * params.yaw_brake_accel_ratio;
  for (int i = -314; i <= 314; ++i) {
    const double error = 0.01 * i;
    const double rate = yawRateToward(error, params);
    EXPECT_GE(rate * error, 0.0) << error;
    EXPECT_LE(std::abs(rate), params.max_angular_speed_radps + 1e-12) << error;
    EXPECT_LE(std::abs(rate), params.yaw_gain_per_s * std::abs(error) + 1e-12) << error;
    // この速さから dt ごとに brake × dt ずつ落とすと、止まるまでに回る角度が誤差を超えない
    EXPECT_LE(
      rate * rate / (2.0 * brake) + std::abs(rate) * params.dt / 2.0, std::abs(error) + 1e-9)
      << error;
  }
  // 誤差が小さい所は比例のまま
  EXPECT_NEAR(yawRateToward(0.1, params), 0.15, 1e-12);
  // 割合 0 なら、比例を回る速さの上限で切るだけ（前の動き）
  MppiParams previous = params;
  previous.yaw_brake_accel_ratio = 0.0;
  EXPECT_NEAR(yawRateToward(0.5, previous), 0.75, 1e-12);
  EXPECT_NEAR(yawRateToward(-2.0, previous), -1.5, 1e-12);
}

struct TurnInPlaceRun
{
  // ゴールの向きを越えて回った角度の最大 [rad]
  double overshoot{0.0};
  // ゴールの位置からずれた距離の最大 [m]
  double drift{0.0};
  // 着いた（kReached）時刻 [s]。着かなければ nullopt
  std::optional<double> reached_s;
  double final_yaw_error{0.0};
};

// ゴールの位置で、向きが goal_yaw と違う止まった機体を、閉ループで回す
TurnInPlaceRun turnInPlace(const MppiParams & params, const double goal_yaw)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{5.0, 0.0}}, goal_yaw);
  MppiController controller;
  GoalArrivalMonitor monitor;
  MppiState state = stateAt(5.0, 0.0, 0.0);
  TurnInPlaceRun run;
  for (int i = 0; i < 80; ++i) {
    const double time_s = i * params.dt;
    if (!run.reached_s.has_value() &&
      monitor.update(time_s, state, Point2{5.0, 0.0}, goal_yaw, GoalArrivalParams{}) ==
      GoalArrival::kReached)
    {
      run.reached_s = time_s;
    }
    const auto command = controller.solve(state, path, {}, nullptr, SE2{}, params, 3.0);
    state = step(state, command, params.dt);
    run.overshoot = std::max(run.overshoot, -std::copysign(1.0, goal_yaw) * (goal_yaw - state.yaw));
    run.drift = std::max(run.drift, std::hypot(state.x - 5.0, state.y));
  }
  run.final_yaw_error = wrapAngle(goal_yaw - state.yaw);
  return run;
}

TEST(Heading, TurnsInPlaceAtGoalWithoutOvershoot)
{
  // 進む向き（+x）と 172° 違うゴールの向きに、位置を保ったままその場で回る
  const MppiParams params = core2026YawParams();
  const TurnInPlaceRun run = turnInPlace(params, 3.0);
  EXPECT_LT(run.overshoot, 0.01);
  EXPECT_LT(run.drift, 0.1);
  ASSERT_TRUE(run.reached_s.has_value());
  EXPECT_LT(*run.reached_s, 5.0);
  EXPECT_LT(std::abs(run.final_yaw_error), 0.01);
  // 前の比例だけの回り方は、減速が間に合わずに行き過ぎる
  MppiParams previous = params;
  previous.yaw_brake_accel_ratio = 0.0;
  EXPECT_GT(turnInPlace(previous, 3.0).overshoot, 0.1);
  // 逆回りも同じ
  EXPECT_LT(turnInPlace(params, -2.0).overshoot, 0.01);
}

// 腰が TURRET（heading.yaw_free）の間は、mode によらず向きの目標を持たない
TEST(Heading, YawFreeHasNoHeadingTarget)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{5.0, 0.0}}, M_PI / 2.0);
  for (const HeadingMode mode :
    {HeadingMode::kTravel, HeadingMode::kGoal, HeadingMode::kForward, HeadingMode::kFixed})
  {
    MppiParams params;
    params.heading.mode = mode;
    params.heading.fixed_yaw = 0.3;
    MppiState state;
    state.x = 4.5;  // ゴールの向きへ切り替える距離の内
    EXPECT_TRUE(headingTarget(state, path, params).has_value());
    params.heading.yaw_free = true;
    EXPECT_FALSE(headingTarget(state, path, params).has_value());
  }
}

// heading.yaw_free では回る速さを指令せずにゴールへ着く。向きを指令するときは、ゴールの向きへ回る
TEST(Heading, YawFreeDrivesToGoalWithoutTurning)
{
  const ReferencePath path({Point2{0.0, 0.0}, Point2{6.0, 0.0}}, M_PI / 2.0);
  const auto run = [&](const bool yaw_free) {
      MppiParams params;
      params.heading.yaw_free = yaw_free;
      MppiController controller;
      MppiState state;
      double max_abs_wz = 0.0;
      for (int i = 0; i < 80; ++i) {
        const auto command = controller.solve(state, path, {}, nullptr, SE2{}, params, 3.0);
        max_abs_wz = std::max(max_abs_wz, std::abs(command.wz));
        state = step(state, command, params.dt);
      }
      return std::make_pair(state, max_abs_wz);
    };
  const auto [free_state, free_wz] = run(true);
  EXPECT_DOUBLE_EQ(free_wz, 0.0);
  EXPECT_DOUBLE_EQ(free_state.yaw, 0.0);
  EXPECT_NEAR(free_state.x, 6.0, 0.2);
  EXPECT_NEAR(free_state.y, 0.0, 0.2);
  const auto [held_state, held_wz] = run(false);
  EXPECT_GT(held_wz, 0.1);
  EXPECT_GT(held_state.yaw, 0.5);
}
}  // namespace
