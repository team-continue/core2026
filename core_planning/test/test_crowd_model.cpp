#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include "core_planning/crowd_model.hpp"

namespace
{
using core_planning::DistanceField;
using core_planning::ObjectKind;
using core_planning::Point2;
using core_planning::computeDistanceField;
using core_planning::makeFieldGrid;
using core_planning::crowd_model::Lane;
using core_planning::crowd_model::LaneState;
using core_planning::crowd_model::RunResult;
using core_planning::crowd_model::Scenario;
using core_planning::crowd_model::StackParams;
using core_planning::crowd_model::Summary;
using core_planning::crowd_model::applyParameter;
using core_planning::crowd_model::applyParameterFile;
using core_planning::crowd_model::fenceClearance;
using core_planning::crowd_model::gapToObject;
using core_planning::crowd_model::laneStateAt;
using core_planning::crowd_model::parseScenario;
using core_planning::crowd_model::simulateRun;
using core_planning::crowd_model::summarize;

DistanceField makeField(const StackParams & params)
{
  return computeDistanceField(makeFieldGrid(params.field, params.grid_resolution_m));
}

Scenario headOnScenario()
{
  return parseScenario(
    YAML::Load(
      "start: [-10.0, 0.0, 0.0]\n"
      "goals:\n"
      "  - [10.0, 0.0, 0.0]\n"
      "  - [-10.0, 0.0, 0.0]\n"
      "timeout_s: 90\n"
      "obstacles:\n"
      "  - {name: other, type: robot, start: [11.0, 0.0], end: [-11.0, 0.0], peak_speed: 3.0}\n"));
}

TEST(CrowdModel, LaneMovesBetweenEndsWithPeakSpeedAtTheMiddle)
{
  Lane lane;
  lane.start = Point2{-11.0, 0.0};
  lane.end = Point2{11.0, 0.0};
  lane.peak_speed_mps = 3.0;
  const double half_period = M_PI * 22.0 / (2.0 * 3.0);

  const LaneState at_start = laneStateAt(lane, 0.0);
  EXPECT_NEAR(at_start.x, -11.0, 1e-9);
  EXPECT_NEAR(at_start.vx, 0.0, 1e-9);

  const LaneState at_middle = laneStateAt(lane, 0.5 * half_period);
  EXPECT_NEAR(at_middle.x, 0.0, 1e-9);
  EXPECT_NEAR(at_middle.vx, 3.0, 1e-9);

  const LaneState at_end = laneStateAt(lane, half_period);
  EXPECT_NEAR(at_end.x, 11.0, 1e-9);
  EXPECT_NEAR(at_end.vx, 0.0, 1e-9);
}

TEST(CrowdModel, ParkedLaneStaysAtStart)
{
  Lane lane;
  lane.start = Point2{3.0, -1.0};
  lane.end = lane.start;
  const LaneState state = laneStateAt(lane, 12.0);
  EXPECT_DOUBLE_EQ(state.x, 3.0);
  EXPECT_DOUBLE_EQ(state.y, -1.0);
  EXPECT_DOUBLE_EQ(state.vx, 0.0);
}

TEST(CrowdModel, GapOfBoxesAndHumans)
{
  // 0.8m 角の箱どうしは、中心が 0.8m 離れると接する
  EXPECT_NEAR(gapToObject(0.0, 0.0, 0.0, ObjectKind::kRobot, 0.8, 0.0), 0.0, 1e-9);
  EXPECT_NEAR(gapToObject(0.0, 0.0, 0.0, ObjectKind::kRobot, 1.0, 0.5), 0.2, 1e-9);
  EXPECT_LT(gapToObject(0.0, 0.0, 0.0, ObjectKind::kRobot, 0.5, 0.0), 0.0);
  // 人は半径 0.25m の円
  EXPECT_NEAR(gapToObject(0.0, 0.0, 0.0, ObjectKind::kHuman, 1.0, 0.0), 0.35, 1e-9);
  EXPECT_LT(gapToObject(0.0, 0.0, 0.0, ObjectKind::kHuman, 0.1, 0.0), 0.0);
  // 45 度回ると、箱の角が相手へ近づく
  EXPECT_LT(
    gapToObject(0.0, 0.0, M_PI_4, ObjectKind::kRobot, 1.0, 0.0),
    gapToObject(0.0, 0.0, 0.0, ObjectKind::kRobot, 1.0, 0.0));
}

TEST(CrowdModel, FenceClearanceUsesTheCorners)
{
  const StackParams params;
  EXPECT_NEAR(fenceClearance(params.field, 0.0, 7.0, 0.0), 0.1, 1e-9);
  EXPECT_LT(fenceClearance(params.field, 0.0, 7.0, M_PI_4), 0.1);
  EXPECT_LT(fenceClearance(params.field, 12.0, 0.0, 0.0), 0.0);
}

TEST(CrowdModel, ParsesScenario)
{
  const Scenario scenario = parseScenario(
    YAML::Load(
      "start: [1.0, 2.0, 0.5]\n"
      "goals: [[3.0, 4.0, 0.0], [5.0, 6.0]]\n"
      "repeat: 2\n"
      "timeout_s: 30\n"
      "obstacles:\n"
      "  - {name: a, type: human, start: [0.0, 0.0], end: [1.0, 1.0], peak_speed: 1.2,"
      " phase: 0.5}\n"
      "  - {name: b, type: robot, start: [7.0, 0.0]}\n"));
  EXPECT_DOUBLE_EQ(scenario.start.yaw, 0.5);
  ASSERT_EQ(scenario.goals.size(), 2U);
  EXPECT_DOUBLE_EQ(scenario.goals[1].y, 6.0);
  EXPECT_EQ(scenario.repeat, 2);
  EXPECT_DOUBLE_EQ(scenario.timeout_s, 30.0);
  ASSERT_EQ(scenario.lanes.size(), 2U);
  EXPECT_EQ(scenario.lanes[0].kind, ObjectKind::kHuman);
  EXPECT_DOUBLE_EQ(scenario.lanes[0].phase, 0.5);
  // end を省くと止まった相手になる
  EXPECT_DOUBLE_EQ(scenario.lanes[1].end.x, 7.0);
}

TEST(CrowdModel, RejectsInvalidScenario)
{
  EXPECT_THROW(parseScenario(YAML::Load("obstacles: []\n")), std::invalid_argument);
  EXPECT_THROW(
    parseScenario(YAML::Load("start: [0.0]\ngoals: [[1.0, 1.0]]\n")), std::invalid_argument);
  EXPECT_THROW(
    parseScenario(
      YAML::Load(
        "start: [0.0, 0.0]\ngoals: [[1.0, 1.0]]\n"
        "obstacles:\n  - {name: a, type: car, start: [0.0, 0.0]}\n")),
    std::invalid_argument);
}

TEST(CrowdModel, AppliesParameters)
{
  StackParams params;
  EXPECT_TRUE(applyParameter(params, "speed_limit.density_speed_scale", "0.5"));
  EXPECT_DOUBLE_EQ(params.speed_limit.density_speed_scale, 0.5);
  EXPECT_TRUE(applyParameter(params, "passing_rule.side", "left"));
  EXPECT_FALSE(params.mppi.passing.side_right);
  EXPECT_TRUE(applyParameter(params, "footprint.width_m", "1.0"));
  EXPECT_DOUBLE_EQ(params.mppi.footprint.half_width_m, 0.5);
  EXPECT_DOUBLE_EQ(params.footprint.half_width_m, 0.5);
  EXPECT_FALSE(applyParameter(params, "mppi.no_such_parameter", "1.0"));
}

TEST(CrowdModel, AppliesResponsibilityMergeAndHumanParameters)
{
  // mppi_lite_node と同じ名前で、責任に応じた回避、合流と横切りの規則、人の近くの減速を設定できる
  StackParams params;
  EXPECT_TRUE(applyParameter(params, "mppi.avoid_being_hit", "true"));
  EXPECT_TRUE(params.mppi.avoid_being_hit);
  EXPECT_TRUE(applyParameter(params, "merge_rule.enabled", "false"));
  EXPECT_FALSE(params.mppi.merge.enabled);
  EXPECT_TRUE(applyParameter(params, "merge_rule.lead_time_s", "1.0"));
  EXPECT_DOUBLE_EQ(params.mppi.merge.lead_time_s, 1.0);
  EXPECT_TRUE(applyParameter(params, "crossing_rule.enabled", "false"));
  EXPECT_FALSE(params.mppi.crossing.enabled);
  EXPECT_TRUE(applyParameter(params, "mppi.evade_while_stopped", "true"));
  EXPECT_TRUE(params.evade_while_stopped);
  EXPECT_TRUE(applyParameter(params, "speed_limit.human_enabled", "false"));
  EXPECT_FALSE(params.speed_limit.human_enabled);
  EXPECT_TRUE(applyParameter(params, "speed_limit.human_radius_m", "2.5"));
  EXPECT_DOUBLE_EQ(params.speed_limit.human_radius_m, 2.5);
  EXPECT_TRUE(applyParameter(params, "speed_limit.human_speed_mps", "0.8"));
  EXPECT_DOUBLE_EQ(params.speed_limit.human_speed_mps, 0.8);
}

TEST(CrowdModel, AppliesStoppedEvasionForHumans)
{
  // mppi_lite_node と同じ名前で、止まっている間に人だけを避けるかを設定できる。既定は避けない
  StackParams params;
  EXPECT_FALSE(params.evade_humans_while_stopped);
  EXPECT_TRUE(applyParameter(params, "mppi.evade_humans_while_stopped", "true"));
  EXPECT_TRUE(params.evade_humans_while_stopped);
}

TEST(CrowdModel, AppliesHeadingParameters)
{
  // mppi_lite_node と同じ名前で向きの決め方を、safety_gate_node と同じく見えない側の上限を安全停止にも設定する
  StackParams params;
  EXPECT_EQ(params.mppi.heading.mode, core_planning::HeadingMode::kTravel);
  EXPECT_TRUE(applyParameter(params, "heading.mode", "goal"));
  EXPECT_EQ(params.mppi.heading.mode, core_planning::HeadingMode::kGoal);
  EXPECT_TRUE(applyParameter(params, "heading.lookahead_m", "1.5"));
  EXPECT_DOUBLE_EQ(params.mppi.heading.lookahead_m, 1.5);
  EXPECT_TRUE(applyParameter(params, "heading.goal_align_distance_m", "0.5"));
  EXPECT_DOUBLE_EQ(params.mppi.heading.goal_align_distance_m, 0.5);
  EXPECT_TRUE(applyParameter(params, "heading.blind_side.enabled", "false"));
  EXPECT_FALSE(params.mppi.blind_side.enabled);
  EXPECT_FALSE(params.safety.blind_side.enabled);
  EXPECT_TRUE(applyParameter(params, "heading.blind_side.visible_half_angle_deg", "100"));
  EXPECT_DOUBLE_EQ(params.mppi.blind_side.visible_half_angle_deg, 100.0);
  EXPECT_DOUBLE_EQ(params.safety.blind_side.visible_half_angle_deg, 100.0);
  EXPECT_TRUE(applyParameter(params, "heading.blind_side.speed_mps", "0.7"));
  EXPECT_DOUBLE_EQ(params.mppi.blind_side.speed_mps, 0.7);
  EXPECT_DOUBLE_EQ(params.safety.blind_side.speed_mps, 0.7);
  EXPECT_THROW(applyParameter(params, "heading.mode", "path"), std::invalid_argument);
}

TEST(CrowdModel, BodyOcclusionHidesRobotBehind)
{
  // 真後ろ 3m に止まったロボットの向こうのゴールへ行く。全周が見えれば避けるが、機体の陰（前 ±96° だけ
  // 見える）では、向きを保って後ろへ進むと最後まで見えずに当たる。進む向きへ回れば見えて避ける
  Scenario scenario;
  scenario.start = core_planning::crowd_model::Goal{0.0, 0.0, 0.0};
  scenario.goals = {core_planning::crowd_model::Goal{-6.0, 0.0, 0.0}};
  scenario.lanes = {Lane{Point2{-3.0, 0.0}, Point2{-3.0, 0.0}, 0.0, 0.0, ObjectKind::kRobot}};
  scenario.timeout_s = 30.0;
  const auto field = makeField(StackParams{});
  const auto run = [&](const std::string & mode, const bool blind, const double visible_deg) {
      StackParams params;
      EXPECT_TRUE(applyParameter(params, "heading.mode", mode));
      EXPECT_TRUE(applyParameter(params, "heading.blind_side.enabled", blind ? "true" : "false"));
      EXPECT_TRUE(applyParameter(params, "model.visible_half_angle_deg", std::to_string(visible_deg)));
      return simulateRun(scenario, params, field, 1U);
    };
  const RunResult hidden_goal_mode = run("goal", false, 96.0);
  EXPECT_GT(hidden_goal_mode.contacts, 0);
  const RunResult visible_goal_mode = run("goal", false, 180.0);
  EXPECT_EQ(visible_goal_mode.contacts, 0);
  EXPECT_EQ(visible_goal_mode.goals_reached, 1);
  const RunResult hidden_travel = run("travel", true, 96.0);
  EXPECT_EQ(hidden_travel.contacts, 0);
  EXPECT_EQ(hidden_travel.goals_reached, 1);
}

TEST(CrowdModel, RejectsUnreadableParameterValues)
{
  StackParams params;
  EXPECT_THROW(applyParameter(params, "mppi.samples", "many"), std::invalid_argument);
  EXPECT_THROW(applyParameter(params, "mppi.dt", "0.1s"), std::invalid_argument);
  EXPECT_THROW(applyParameter(params, "footprint.enabled", "yes"), std::invalid_argument);
  EXPECT_THROW(applyParameter(params, "passing_rule.side", "up"), std::invalid_argument);
}

TEST(CrowdModel, AppliesParameterFileAndReportsUnusedNames)
{
  StackParams params;
  const auto unused = applyParameterFile(
    params,
    YAML::Load(
      "/**:\n"
      "  ros__parameters:\n"
      "    mppi:\n"
      "      samples: 50\n"
      "      latency_compensation_s: 0.05\n"));
  EXPECT_EQ(params.mppi.samples, 50);
  EXPECT_EQ(unused, std::vector<std::string>{"mppi.latency_compensation_s"});
}

TEST(CrowdModel, SameSeedGivesSameResult)
{
  const StackParams params;
  const auto field = makeField(params);
  const Scenario scenario = headOnScenario();
  const RunResult first = simulateRun(scenario, params, field, 3U);
  const RunResult second = simulateRun(scenario, params, field, 3U);
  EXPECT_DOUBLE_EQ(first.duration_s, second.duration_s);
  EXPECT_DOUBLE_EQ(first.distance_m, second.distance_m);
  EXPECT_DOUBLE_EQ(first.min_gap_m, second.min_gap_m);
}

TEST(CrowdModel, ReachesAllGoalsWithoutObjects)
{
  const StackParams params;
  const auto field = makeField(params);
  const Scenario scenario = parseScenario(
    YAML::Load(
      "start: [-6.0, -2.5]\ngoals: [[6.0, -2.5], [0.0, 3.0]]\nrepeat: 2\ntimeout_s: 60\n"));
  const RunResult result = simulateRun(scenario, params, field, 0U);
  EXPECT_EQ(result.goals_total, 4);
  EXPECT_EQ(result.goals_reached, 4);
  EXPECT_EQ(result.contacts, 0);
  EXPECT_GT(result.min_fence_clearance_m, 0.0);
}

TEST(CrowdModel, ContactAtZeroSpeedIsNotOwnFault)
{
  // 開始位置に相手が重なっていて、ゴールも開始位置なので動かない
  const StackParams params;
  const auto field = makeField(params);
  const Scenario scenario = parseScenario(
    YAML::Load(
      "start: [0.0, 0.0]\ngoals: [[0.0, 0.0]]\ntimeout_s: 5\n"
      "obstacles:\n  - {name: a, type: robot, start: [0.3, 0.0]}\n"));
  const RunResult result = simulateRun(scenario, params, field, 0U);
  EXPECT_LT(result.min_gap_m, 0.0);
  EXPECT_EQ(result.contacts, 1);
  EXPECT_EQ(result.own_fault_contacts, 0);
}

TEST(CrowdModel, PassingRuleSetsTheSide)
{
  // 正面から来る相手とすれ違う側。正面の相手に道を開けない規則（oncoming_rule、既定）では止まって
  // すれ違わないので、元の動き（oncoming_rule.enabled: false）で確かめる。既定の動きは
  // HeadOnWithOncomingRule の試験で確かめる
  StackParams right;
  StackParams left;
  ASSERT_TRUE(applyParameter(right, "oncoming_rule.enabled", "false"));
  ASSERT_TRUE(applyParameter(left, "oncoming_rule.enabled", "false"));
  ASSERT_TRUE(applyParameter(left, "passing_rule.side", "left"));
  const auto field = makeField(right);
  const Scenario scenario = headOnScenario();
  const auto evaluate = [&](const StackParams & params) {
      std::vector<RunResult> results;
      for (std::uint32_t seed = 0U; seed < 12U; ++seed) {
        results.push_back(simulateRun(scenario, params, field, seed));
      }
      return summarize(results);
    };
  const Summary with_right = evaluate(right);
  const Summary with_left = evaluate(left);
  ASSERT_GT(with_right.passes_total, 0);
  ASSERT_GT(with_left.passes_total, 0);
  // どちらの設定でも、設定した側を多く通る
  EXPECT_GT(with_right.passes_on_rule_side * 10, with_right.passes_total * 7);
  EXPECT_GT(with_left.passes_on_rule_side * 10, with_left.passes_total * 7);
}

TEST(CrowdModel, SummaryComputesRatesAndCounts)
{
  RunResult clean;
  clean.duration_s = 50.0;
  clean.distance_m = 100.0;
  clean.goals_reached = 2;
  clean.goals_total = 2;
  clean.min_gap_m = 0.5;
  clean.min_fence_clearance_m = 0.4;
  clean.passes_on_rule_side = 3;

  RunResult touched = clean;
  touched.contacts = 2;
  touched.own_fault_contacts = 1;
  touched.min_gap_m = -0.5;
  touched.min_fence_clearance_m = -0.01;
  touched.passes_on_rule_side = 1;
  touched.passes_on_other_side = 1;

  const Summary summary = summarize({clean, touched});
  EXPECT_EQ(summary.runs, 2);
  EXPECT_DOUBLE_EQ(summary.contacts_per_100s, 2.0);
  EXPECT_GT(summary.contacts_per_100s_margin, 0.0);
  EXPECT_DOUBLE_EQ(summary.own_fault_per_100s, 1.0);
  EXPECT_DOUBLE_EQ(summary.contact_free_ratio, 0.5);
  EXPECT_DOUBLE_EQ(summary.deep_contact_ratio, 0.5);
  EXPECT_DOUBLE_EQ(summary.mean_speed_mps, 2.0);
  EXPECT_DOUBLE_EQ(summary.min_fence_clearance_m, -0.01);
  EXPECT_EQ(summary.fence_touch_runs, 1);
  EXPECT_EQ(summary.passes_on_rule_side, 4);
  EXPECT_EQ(summary.passes_total, 5);
}
}  // namespace
