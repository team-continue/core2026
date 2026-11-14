#include "core_planning/crowd_model.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <optional>
#include <random>
#include <stdexcept>
#include <utility>

namespace core_planning::crowd_model
{
namespace
{
// core_nav_check/scenario.py と同じ
constexpr double kRobotHalfSizeM = 0.4;
constexpr double kHumanRadiusM = 0.25;
// run_scenario.py の FAULT_SPEED_MPS と同じ
constexpr double kOwnFaultSpeedMps = 0.5;
// 同じ相手との接触を、この時間より離れていれば別の接触として数える
constexpr double kContactSeparationS = 1.0;
// 重なりがこれより深い接触があった実行を数える
constexpr double kDeepContactM = -0.2;
// ゴールを受けてから計画が届くまでの時間。この間は止まる
constexpr double kPlanDelayS = 0.4;
// omni_force_drive の並進の加速度の上限
constexpr double kRobotMaxAccelMps2 = 5.0;
// check_features.py の passing_sides と同じ条件で、すれ違いを数える
constexpr double kDirectionMinSpeedMps = 0.3;
constexpr double kPassingMinSpeedMps = 1.0;
constexpr double kPassingMaxLateralM = 4.0;
// 相手の動きの位相を変えるために、走り始める時刻をこの範囲で選ぶ
constexpr double kStartTimeMinS = 5.0;
constexpr double kStartTimeMaxS = 80.0;
constexpr double kVelocityNoiseCorrelation = 0.7;
constexpr std::uint32_t kSeedScale = 7919U;
constexpr std::uint32_t kSeedOffset = 13U;

using Setter = std::function<void (StackParams &, const std::string &)>;

double toDouble(const std::string & text)
{
  std::size_t used = 0U;
  const double value = std::stod(text, &used);
  if (used != text.size()) {
    throw std::invalid_argument("数値ではありません");
  }
  return value;
}

int toInt(const std::string & text)
{
  std::size_t used = 0U;
  const int value = std::stoi(text, &used);
  if (used != text.size()) {
    throw std::invalid_argument("整数ではありません");
  }
  return value;
}

bool toBool(const std::string & text)
{
  if (text == "true") {
    return true;
  }
  if (text == "false") {
    return false;
  }
  throw std::invalid_argument("true か false ではありません");
}

HeadingMode toHeadingMode(const std::string & text)
{
  if (const auto mode = parseHeadingMode(text)) {
    return *mode;
  }
  throw std::invalid_argument("travel か goal ではありません");
}

bool toSideRight(const std::string & text)
{
  if (text == "right") {
    return true;
  }
  if (text == "left") {
    return false;
  }
  throw std::invalid_argument("right か left ではありません");
}

// mppi_lite_node と safety_gate_node が読むパラメータと、追跡の近似の値
const std::map<std::string, Setter> & parameterTable()
{
  using P = StackParams;
  using S = std::string;
  static const std::map<std::string, Setter> table{
    {"grid_resolution_m", [](P & p, const S & v) {p.grid_resolution_m = toDouble(v);}},
    {"mppi.samples", [](P & p, const S & v) {p.mppi.samples = toInt(v);}},
    {"mppi.horizon_steps", [](P & p, const S & v) {p.mppi.horizon_steps = toInt(v);}},
    {"mppi.dt", [](P & p, const S & v) {p.mppi.dt = toDouble(v);}},
    {"mppi.temperature", [](P & p, const S & v) {p.mppi.temperature = toDouble(v);}},
    {"mppi.noise_linear", [](P & p, const S & v) {p.mppi.noise_linear = toDouble(v);}},
    {"mppi.noise_correlation", [](P & p, const S & v) {p.mppi.noise_correlation = toDouble(v);}},
    {"mppi.max_speed_mps", [](P & p, const S & v) {p.max_speed_mps = toDouble(v);}},
    {"mppi.max_accel_mps2", [](P & p, const S & v) {p.mppi.max_accel_mps2 = toDouble(v);}},
    {"mppi.max_angular_speed_radps",
      [](P & p, const S & v) {p.mppi.max_angular_speed_radps = toDouble(v);}},
    {"mppi.max_angular_accel_radps2",
      [](P & p, const S & v) {p.mppi.max_angular_accel_radps2 = toDouble(v);}},
    {"mppi.yaw_gain_per_s", [](P & p, const S & v) {p.mppi.yaw_gain_per_s = toDouble(v);}},
    {"mppi.reference_decel_mps2",
      [](P & p, const S & v) {p.mppi.reference_decel_mps2 = toDouble(v);}},
    {"mppi.goal_tolerance_m", [](P & p, const S & v) {p.goal_tolerance_m = toDouble(v);}},
    {"mppi.goal_speed_tolerance_mps",
      [](P & p, const S & v) {p.goal_speed_tolerance_mps = toDouble(v);}},
    {"mppi.evade_while_stopped", [](P & p, const S & v) {p.evade_while_stopped = toBool(v);}},
    {"mppi.evade_humans_while_stopped",
      [](P & p, const S & v) {p.evade_humans_while_stopped = toBool(v);}},
    {"mppi.avoid_being_hit", [](P & p, const S & v) {p.mppi.avoid_being_hit = toBool(v);}},
    {"mppi.backoff_speed_mps", [](P & p, const S & v) {p.backoff_speed_mps = toDouble(v);}},
    {"mppi.keep_out_range_m", [](P & p, const S & v) {p.mppi.keep_out_range_m = toDouble(v);}},
    {"mppi.static_margin_time_s",
      [](P & p, const S & v) {p.mppi.static_margin_time_s = toDouble(v);}},
    {"mppi.static_margin_approach_only",
      [](P & p, const S & v) {p.mppi.static_margin_approach_only = toBool(v);}},
    {"mppi.weight_tracking", [](P & p, const S & v) {p.mppi.weight_tracking = toDouble(v);}},
    {"mppi.weight_terminal", [](P & p, const S & v) {p.mppi.weight_terminal = toDouble(v);}},
    {"mppi.weight_keep_out", [](P & p, const S & v) {p.mppi.weight_keep_out = toDouble(v);}},
    {"mppi.weight_collision", [](P & p, const S & v) {p.mppi.weight_collision = toDouble(v);}},
    {"mppi.weight_object", [](P & p, const S & v) {p.mppi.weight_object = toDouble(v);}},
    {"mppi.weight_effort", [](P & p, const S & v) {p.mppi.weight_effort = toDouble(v);}},
    {"mppi.weight_collision_course",
      [](P & p, const S & v) {p.mppi.weight_collision_course = toDouble(v);}},
    {"mppi.collision_course_lookahead_s",
      [](P & p, const S & v) {p.mppi.collision_course_lookahead_s = toDouble(v);}},
    {"mppi.robot_radius_m", [](P & p, const S & v) {p.mppi.objects.robot_radius_m = toDouble(v);}},
    {"mppi.human_radius_m", [](P & p, const S & v) {p.mppi.objects.human_radius_m = toDouble(v);}},
    {"mppi.unknown_radius_m",
      [](P & p, const S & v) {p.mppi.objects.unknown_radius_m = toDouble(v);}},
    {"mppi.robot_margin_m", [](P & p, const S & v) {p.mppi.objects.robot_margin_m = toDouble(v);}},
    {"mppi.robot_min_margin_m",
      [](P & p, const S & v) {p.mppi.objects.robot_min_margin_m = toDouble(v);}},
    {"mppi.human_margin_m", [](P & p, const S & v) {p.mppi.objects.human_margin_m = toDouble(v);}},
    {"mppi.unknown_margin_m",
      [](P & p, const S & v) {p.mppi.objects.unknown_margin_m = toDouble(v);}},
    {"mppi.prediction_sigma_growth_mps",
      [](P & p, const S & v) {p.mppi.objects.prediction_sigma_growth_mps = toDouble(v);}},
    {"mppi.prediction_velocity_sigma_gain",
      [](P & p, const S & v) {p.mppi.objects.velocity_sigma_gain = toDouble(v);}},
    {"mppi.object_discount", [](P & p, const S & v) {p.mppi.objects.discount = toDouble(v);}},
    {"mppi.object_max_speed_mps",
      [](P & p, const S & v) {p.mppi.objects.max_speed_mps = toDouble(v);}},
    {"footprint.enabled", [](P & p, const S & v) {p.mppi.footprint.enabled = toBool(v);}},
    {"footprint.length_m", [](P & p, const S & v) {
        p.mppi.footprint.half_length_m = 0.5 * toDouble(v);
        p.footprint.half_length_m = 0.5 * toDouble(v);
      }},
    {"footprint.width_m", [](P & p, const S & v) {
        p.mppi.footprint.half_width_m = 0.5 * toDouble(v);
        p.footprint.half_width_m = 0.5 * toDouble(v);
      }},
    {"footprint.margin_m", [](P & p, const S & v) {p.mppi.footprint.margin_m = toDouble(v);}},
    {"passing_rule.enabled", [](P & p, const S & v) {p.mppi.passing.enabled = toBool(v);}},
    {"passing_rule.side", [](P & p, const S & v) {p.mppi.passing.side_right = toSideRight(v);}},
    {"passing_rule.weight", [](P & p, const S & v) {p.mppi.passing.weight = toDouble(v);}},
    {"passing_rule.range_m", [](P & p, const S & v) {p.mppi.passing.range_m = toDouble(v);}},
    {"passing_rule.commit_distance_m",
      [](P & p, const S & v) {p.mppi.passing.commit_distance_m = toDouble(v);}},
    {"passing_rule.lateral_range_m",
      [](P & p, const S & v) {p.mppi.passing.lateral_range_m = toDouble(v);}},
    {"merge_rule.enabled", [](P & p, const S & v) {p.mppi.merge.enabled = toBool(v);}},
    {"merge_rule.lead_time_s", [](P & p, const S & v) {p.mppi.merge.lead_time_s = toDouble(v);}},
    {"goal_approach.enabled", [](P & p, const S & v) {p.goal_approach.enabled = toBool(v);}},
    {"goal_approach.radius_m", [](P & p, const S & v) {p.goal_approach.radius_m = toDouble(v);}},
    {"goal_approach.hold_distance_m",
      [](P & p, const S & v) {p.goal_approach.hold_distance_m = toDouble(v);}},
    {"goal_approach.max_hold_s", [](P & p, const S & v) {p.goal_approach.max_hold_s = toDouble(v);}},
    {"goal_approach.release_s", [](P & p, const S & v) {p.goal_approach.release_s = toDouble(v);}},
    {"crossing_rule.enabled", [](P & p, const S & v) {p.mppi.crossing.enabled = toBool(v);}},
    {"oncoming_rule.enabled", [](P & p, const S & v) {p.mppi.oncoming.enabled = toBool(v);}},
    {"oncoming_rule.stop_before_midpoint_m",
      [](P & p, const S & v) {p.mppi.oncoming.stop_before_midpoint_m = toDouble(v);}},
    {"oncoming_rule.resume_after_s",
      [](P & p, const S & v) {p.mppi.oncoming.resume_after_s = toDouble(v);}},
    {"oncoming_rule.range_m", [](P & p, const S & v) {p.mppi.oncoming.range_m = toDouble(v);}},
    {"oncoming_rule.pass_goal_margin_m",
      [](P & p, const S & v) {p.mppi.oncoming.pass_goal_margin_m = toDouble(v);}},
    {"track_motion.enabled", [](P & p, const S & v) {p.motion_filter.enabled = toBool(v);}},
    {"track_motion.min_speed_mps",
      [](P & p, const S & v) {p.motion_filter.min_speed_mps = toDouble(v);}},
    {"track_motion.max_velocity_sigma_mps",
      [](P & p, const S & v) {p.motion_filter.max_velocity_sigma_mps = toDouble(v);}},
    {"track_motion.persist_s", [](P & p, const S & v) {p.motion_filter.persist_s = toDouble(v);}},
    {"heading.mode", [](P & p, const S & v) {p.mppi.heading.mode = toHeadingMode(v);}},
    {"heading.lookahead_m", [](P & p, const S & v) {p.mppi.heading.lookahead_m = toDouble(v);}},
    {"mppi.curve.max_lateral_accel_mps2",
      [](P & p, const S & v) {p.mppi.curve.max_lateral_accel_mps2 = toDouble(v);}},
    {"mppi.curve.window_m", [](P & p, const S & v) {p.mppi.curve.window_m = toDouble(v);}},
    {"heading.goal_align_distance_m",
      [](P & p, const S & v) {p.mppi.heading.goal_align_distance_m = toDouble(v);}},
    // 安全停止の退避の候補も同じ値を使う。mppi_lite_node と safety_gate_node と同じ
    {"heading.blind_side.enabled", [](P & p, const S & v) {
        p.mppi.blind_side.enabled = toBool(v);
        p.safety.blind_side.enabled = p.mppi.blind_side.enabled;
      }},
    {"heading.blind_side.visible_half_angle_deg", [](P & p, const S & v) {
        p.mppi.blind_side.visible_half_angle_deg = toDouble(v);
        p.safety.blind_side.visible_half_angle_deg = p.mppi.blind_side.visible_half_angle_deg;
      }},
    {"heading.blind_side.speed_mps", [](P & p, const S & v) {
        p.mppi.blind_side.speed_mps = toDouble(v);
        p.safety.blind_side.speed_mps = p.mppi.blind_side.speed_mps;
      }},
    {"speed_limit.enabled", [](P & p, const S & v) {p.speed_limit.enabled = toBool(v);}},
    {"speed_limit.reaction_time_s",
      [](P & p, const S & v) {p.speed_limit.reaction_time_s = toDouble(v);}},
    {"speed_limit.max_decel_mps2",
      [](P & p, const S & v) {p.speed_limit.max_decel_mps2 = toDouble(v);}},
    {"speed_limit.density_radius_m",
      [](P & p, const S & v) {p.speed_limit.density_radius_m = toDouble(v);}},
    {"speed_limit.density_speed_scale",
      [](P & p, const S & v) {p.speed_limit.density_speed_scale = toDouble(v);}},
    {"speed_limit.min_speed_mps",
      [](P & p, const S & v) {p.speed_limit.min_speed_mps = toDouble(v);}},
    {"speed_limit.stopping_margin_m",
      [](P & p, const S & v) {p.speed_limit.stopping_margin_m = toDouble(v);}},
    {"speed_limit.human_enabled",
      [](P & p, const S & v) {p.speed_limit.human_enabled = toBool(v);}},
    {"speed_limit.human_radius_m",
      [](P & p, const S & v) {p.speed_limit.human_radius_m = toDouble(v);}},
    {"speed_limit.human_speed_mps",
      [](P & p, const S & v) {p.speed_limit.human_speed_mps = toDouble(v);}},
    {"speed_limit.human_approach_time_s",
      [](P & p, const S & v) {p.speed_limit.human_approach_time_s = toDouble(v);}},
    {"speed_limit.human_approach_speed_mps",
      [](P & p, const S & v) {p.speed_limit.human_approach_speed_mps = toDouble(v);}},
    {"stuck_recovery.enabled", [](P & p, const S & v) {p.stuck.enabled = toBool(v);}},
    {"stuck_recovery.no_progress_timeout_s",
      [](P & p, const S & v) {p.stuck.no_progress_timeout_s = toDouble(v);}},
    {"stuck_recovery.min_progress_m",
      [](P & p, const S & v) {p.stuck.min_progress_m = toDouble(v);}},
    {"stuck_recovery.wait_s", [](P & p, const S & v) {p.stuck.wait_s = toDouble(v);}},
    {"stuck_recovery.backoff_distance_m",
      [](P & p, const S & v) {p.stuck.backoff_distance_m = toDouble(v);}},
    {"stuck_recovery.max_retries", [](P & p, const S & v) {p.stuck.max_retries = toInt(v);}},
    {"stuck_recovery.moving_distance_m",
      [](P & p, const S & v) {p.stuck.moving_distance_m = toDouble(v);}},
    {"stuck_recovery.moving_timeout_s",
      [](P & p, const S & v) {p.stuck.moving_timeout_s = toDouble(v);}},
    {"safety_stop.enabled", [](P & p, const S & v) {p.safety_enabled = toBool(v);}},
    {"safety_stop.ttc_slow_s", [](P & p, const S & v) {p.safety.ttc_slow_s = toDouble(v);}},
    {"safety_stop.ttc_stop_s", [](P & p, const S & v) {p.safety.ttc_stop_s = toDouble(v);}},
    {"safety_stop.footprint_margin_m",
      [](P & p, const S & v) {p.safety.footprint_margin_m = toDouble(v);}},
    {"safety_stop.max_decel_mps2",
      [](P & p, const S & v) {p.safety.max_decel_mps2 = toDouble(v);}},
    {"safety_stop.reachable.enabled",
      [](P & p, const S & v) {p.safety.reachable.enabled = toBool(v);}},
    // 局所計画（合流とゴールの手前で待つ判定）も同じ値を使う。mppi_lite_node と同じ
    {"safety_stop.reachable.obstacle_accel_mps2", [](P & p, const S & v) {
        p.safety.reachable.obstacle_accel_mps2 = toDouble(v);
        p.mppi.objects.accel_mps2 = p.safety.reachable.obstacle_accel_mps2;
      }},
    {"safety_stop.reachable.velocity_sigma_gain",
      [](P & p, const S & v) {p.safety.reachable.velocity_sigma_gain = toDouble(v);}},
    {"safety_stop.reachable.commit_time_s",
      [](P & p, const S & v) {p.safety.reachable.commit_time_s = toDouble(v);}},
    {"safety_stop.reachable.escape_horizon_s",
      [](P & p, const S & v) {p.safety.reachable.escape_horizon_s = toDouble(v);}},
    {"safety_stop.reachable.escape_change_s",
      [](P & p, const S & v) {p.safety.reachable.escape_change_s = toDouble(v);}},
    {"safety_stop.reachable.directions",
      [](P & p, const S & v) {p.safety.reachable.directions = toInt(v);}},
    // モデルの静止物は柵だけで、距離の場は grid_resolution_m の格子のものを共有する
    {"safety_stop.static.enabled",
      [](P & p, const S & v) {p.safety.static_check.enabled = toBool(v);}},
    {"safety_stop.static.margin_m",
      [](P & p, const S & v) {p.safety.static_check.margin_m = toDouble(v);}},
    {"safety_stop.static.tolerance_m",
      [](P & p, const S & v) {p.safety.static_check.tolerance_m = toDouble(v);}},
    {"safety_stop.oncoming.brake_only",
      [](P & p, const S & v) {p.safety.oncoming.brake_only = toBool(v);}},
    {"safety_stop.mitigation.enabled",
      [](P & p, const S & v) {p.safety.mitigation.enabled = toBool(v);}},
    {"safety_stop.mitigation.min_impact_speed_mps",
      [](P & p, const S & v) {p.safety.mitigation.min_impact_speed_mps = toDouble(v);}},
    {"safety_stop.mitigation.horizon_s",
      [](P & p, const S & v) {p.safety.mitigation.horizon_s = toDouble(v);}},
    {"safety_stop.mitigation.max_shift_m",
      [](P & p, const S & v) {p.safety.mitigation.max_shift_m = toDouble(v);}},
    {"safety_stop.mitigation.max_speed_mps",
      [](P & p, const S & v) {p.safety.mitigation.max_speed_mps = toDouble(v);}},
    {"safety_stop.mitigation.min_reduction_mps",
      [](P & p, const S & v) {p.safety.mitigation.min_reduction_mps = toDouble(v);}},
    {"safety_stop.mitigation.directions",
      [](P & p, const S & v) {p.safety.mitigation.directions = toInt(v);}},
    {"field.x_min", [](P & p, const S & v) {p.field.x_min = toDouble(v);}},
    {"field.x_max", [](P & p, const S & v) {p.field.x_max = toDouble(v);}},
    {"field.y_min", [](P & p, const S & v) {p.field.y_min = toDouble(v);}},
    {"field.y_max", [](P & p, const S & v) {p.field.y_max = toDouble(v);}},
    {"planner.rate_hz", [](P & p, const S & v) {p.plan_period_s = 1.0 / toDouble(v);}},
    {"model.velocity_lag_s", [](P & p, const S & v) {p.tracking.velocity_lag_s = toDouble(v);}},
    {"model.velocity_noise_mps",
      [](P & p, const S & v) {p.tracking.velocity_noise_mps = toDouble(v);}},
    {"model.position_noise_m", [](P & p, const S & v) {p.tracking.position_noise_m = toDouble(v);}},
    {"model.extrapolation_s", [](P & p, const S & v) {p.tracking.extrapolation_s = toDouble(v);}},
    {"model.position_sigma_m", [](P & p, const S & v) {p.tracking.position_sigma_m = toDouble(v);}},
    {"model.visible_half_angle_deg",
      [](P & p, const S & v) {p.tracking.visible_half_angle_deg = toDouble(v);}},
    {"model.lidar_x_m", [](P & p, const S & v) {p.tracking.lidar_x_m = toDouble(v);}},
    {"model.max_coast_s", [](P & p, const S & v) {p.tracking.max_coast_s = toDouble(v);}},
    {"model.coast_sigma_growth_mps",
      [](P & p, const S & v) {p.tracking.coast_sigma_growth_mps = toDouble(v);}},
  };
  return table;
}

void flattenParameters(
  const YAML::Node & node, const std::string & prefix,
  std::vector<std::pair<std::string, std::string>> & parameters)
{
  if (node.IsScalar()) {
    parameters.emplace_back(prefix, node.Scalar());
    return;
  }
  if (!node.IsMap()) {
    return;
  }
  for (const auto & item : node) {
    const auto key = item.first.as<std::string>();
    flattenParameters(item.second, prefix.empty() ? key : prefix + "." + key, parameters);
  }
}

Point2 readPoint(const YAML::Node & node, const std::string & name)
{
  if (!node.IsSequence() || node.size() < 2U) {
    throw std::invalid_argument(name + " は [x, y] の形で指定してください");
  }
  return Point2{node[0].as<double>(), node[1].as<double>()};
}

Goal readGoal(const YAML::Node & node, const std::string & name)
{
  const Point2 point = readPoint(node, name);
  return Goal{point.x, point.y, node.size() >= 3U ? node[2].as<double>() : 0.0};
}

Lane readLane(const YAML::Node & node)
{
  const auto name = node["name"] ? node["name"].as<std::string>() : std::string{"obstacle"};
  if (!node["start"]) {
    throw std::invalid_argument("相手 " + name + " に start がありません");
  }
  const auto type = node["type"] ? node["type"].as<std::string>() : std::string{};
  if (type != "robot" && type != "human") {
    throw std::invalid_argument("相手 " + name + " の type は robot か human にしてください");
  }
  Lane lane;
  lane.start = readPoint(node["start"], name + " の start");
  lane.end = node["end"] ? readPoint(node["end"], name + " の end") : lane.start;
  lane.peak_speed_mps = node["peak_speed"] ? node["peak_speed"].as<double>() : 0.0;
  lane.phase = node["phase"] ? node["phase"].as<double>() : 0.0;
  lane.kind = type == "human" ? ObjectKind::kHuman : ObjectKind::kRobot;
  return lane;
}

safety::BodyVelocity toBody(const double vx, const double vy, const double wz, const double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return safety::BodyVelocity{c * vx + s * vy, -s * vx + c * vy, wz};
}

// 追跡の速度の状態。相手ごとに持つ
struct TrackedVelocity
{
  double vx{0.0};
  double vy{0.0};
  double noise_x{0.0};
  double noise_y{0.0};
};

std::vector<TrackedVelocity> initialTracks(const std::vector<Lane> & lanes, const double time_s)
{
  std::vector<TrackedVelocity> tracks;
  tracks.reserve(lanes.size());
  for (const auto & lane : lanes) {
    const LaneState state = laneStateAt(lane, time_s);
    tracks.push_back(TrackedVelocity{state.vx, state.vy, 0.0, 0.0});
  }
  return tracks;
}

// 追跡の速度を 1 周期進めて、計画に渡す相手の状態を返す
std::vector<PredictedObject> updateTracks(
  const std::vector<Lane> & lanes, const double time_s, const TrackingModel & model,
  const double dt, std::mt19937 & random, std::vector<TrackedVelocity> & tracks)
{
  std::normal_distribution<double> normal(0.0, 1.0);
  const double blend =
    model.velocity_lag_s > 0.0 ? 1.0 - std::exp(-dt / model.velocity_lag_s) : 1.0;
  const double innovation =
    std::sqrt(1.0 - kVelocityNoiseCorrelation * kVelocityNoiseCorrelation);
  std::vector<PredictedObject> objects;
  objects.reserve(lanes.size());
  for (std::size_t i = 0; i < lanes.size(); ++i) {
    const LaneState truth = laneStateAt(lanes[i], time_s);
    TrackedVelocity & track = tracks[i];
    track.vx += (truth.vx - track.vx) * blend;
    track.vy += (truth.vy - track.vy) * blend;
    track.noise_x = kVelocityNoiseCorrelation * track.noise_x +
      innovation * normal(random) * model.velocity_noise_mps;
    track.noise_y = kVelocityNoiseCorrelation * track.noise_y +
      innovation * normal(random) * model.velocity_noise_mps;
    PredictedObject object;
    object.vx = track.vx + track.noise_x;
    object.vy = track.vy + track.noise_y;
    // 走査の時刻の位置を、推定した速度で今の時刻まで進めた結果になる
    object.x = truth.x - (truth.vx - object.vx) * model.extrapolation_s +
      normal(random) * model.position_noise_m;
    object.y = truth.y - (truth.vy - object.vy) * model.extrapolation_s +
      normal(random) * model.position_noise_m;
    object.sigma = model.position_sigma_m;
    object.kind = lanes[i].kind;
    object.id = static_cast<std::uint32_t>(i + 1U);
    objects.push_back(object);
  }
  return objects;
}

// 機体の陰に入っていないか。LiDAR（機体の中心から前へ lidar_x_m）から見た相手の中心の方位から、相手の
// 大きさの分の角度を引いた値が、visible_half_angle_deg 以下なら見える
bool isVisible(
  const MppiState & state, const PredictedObject & object, const TrackingModel & model)
{
  if (model.visible_half_angle_deg >= 180.0) {
    return true;
  }
  const double c = std::cos(state.yaw);
  const double s = std::sin(state.yaw);
  const double dx = object.x - (state.x + c * model.lidar_x_m);
  const double dy = object.y - (state.y + s * model.lidar_x_m);
  const double distance = std::hypot(dx, dy);
  const double radius = object.kind == ObjectKind::kHuman ? kHumanRadiusM : kRobotHalfSizeM;
  if (distance <= radius) {
    return true;
  }
  const double bearing = std::abs(wrapAngle(std::atan2(dy, dx) - state.yaw));
  return bearing - std::asin(radius / distance) <= model.visible_half_angle_deg * M_PI / 180.0;
}

// 最後に見えた時刻と、そのときに計画へ渡した状態。相手ごとに持つ
struct SeenObject
{
  double time_s{-std::numeric_limits<double>::infinity()};
  PredictedObject object;
};

// 機体の陰に入った相手を、最後に見えた状態から max_coast_s まで等速で進めて保ち、過ぎたら除く。
// coasting に、返す相手ごとの、陰に入って予測だけで保っているか（追跡の coasting）を入れる
std::vector<PredictedObject> applyOcclusion(
  const MppiState & state, const std::vector<PredictedObject> & objects, const double time_s,
  const TrackingModel & model, std::vector<SeenObject> & seen, std::vector<std::uint8_t> & coasting)
{
  std::vector<PredictedObject> visible;
  visible.reserve(objects.size());
  coasting.clear();
  for (std::size_t i = 0; i < objects.size(); ++i) {
    if (isVisible(state, objects[i], model)) {
      seen[i] = SeenObject{time_s, objects[i]};
      visible.push_back(objects[i]);
      coasting.push_back(0U);
      continue;
    }
    const double coast = time_s - seen[i].time_s;
    if (coast > model.max_coast_s) {
      continue;
    }
    PredictedObject predicted = seen[i].object;
    predicted.x += predicted.vx * coast;
    predicted.y += predicted.vy * coast;
    predicted.sigma += model.coast_sigma_growth_mps * coast;
    visible.push_back(predicted);
    coasting.push_back(1U);
  }
  return visible;
}

// moving は相手ごとの、動いているとみなすか（TrackMotionFilter）
std::vector<safety::Obstacle> toObstacles(
  const std::vector<PredictedObject> & objects, const std::vector<std::uint8_t> & moving)
{
  std::vector<safety::Obstacle> obstacles;
  obstacles.reserve(objects.size());
  for (std::size_t i = 0; i < objects.size(); ++i) {
    const auto & object = objects[i];
    obstacles.push_back(
      safety::Obstacle{
        object.x, object.y, object.vx, object.vy,
        object.kind == ObjectKind::kHuman ? safety::ObstacleKind::kHuman :
        safety::ObstacleKind::kRobot, object.sigma, object.velocity_sigma, 0.0,
        i < moving.size() && moving[i] != 0U});
  }
  return obstacles;
}

// safety_gate_node と同じ順に、速度指令を安全停止に通す。静止物の距離の場は計画と同じもの（odom = map）。
// 高速で当てられそうなら、衝撃の緩和の速度に替える（mitigator が緩和を始めたら mitigations を数える）
safety::BodyVelocity applySafetyGate(
  const MppiCommand & command, const MppiState & state,
  const std::vector<PredictedObject> & objects, const std::vector<std::uint8_t> & moving,
  const safety::BodyVelocity & last_output, const StackParams & params, const DistanceField & field,
  const double time_s, const double dt, safety::ImpactMitigator & mitigator, int & mitigations)
{
  const safety::BodyVelocity commanded = toBody(command.vx, command.vy, command.wz, state.yaw);
  if (!params.safety_enabled) {
    return commanded;
  }
  const auto obstacles = toObstacles(objects, moving);
  const safety::PlanarPose pose{state.x, state.y, state.yaw};
  const safety::BodyVelocity own_velocity = toBody(state.vx, state.vy, state.wz, state.yaw);
  const safety::SupervisedCommand supervised = safety::superviseCommand(
    obstacles, pose, own_velocity, commanded, params.footprint, params.safety, &field,
    safety::PlanarPose{});
  const bool was_mitigating = mitigator.active();
  const auto mitigation = mitigator.update(
    time_s, obstacles, pose, own_velocity, params.footprint, params.safety, &field,
    safety::PlanarPose{});
  if (mitigator.active() && !was_mitigating) {
    ++mitigations;
  }
  return safety::limitDeceleration(
    last_output, mitigation.value_or(supervised.output), params.safety.max_decel_mps2, dt);
}

// 機体の座標系の速度指令で、機体を 1 周期進める
MppiState stepRobot(const MppiState & state, const safety::BodyVelocity & output, const double dt)
{
  const double c = std::cos(state.yaw);
  const double s = std::sin(state.yaw);
  double dvx = c * output.vx - s * output.vy - state.vx;
  double dvy = s * output.vx + c * output.vy - state.vy;
  const double max_dv = kRobotMaxAccelMps2 * dt;
  const double dv = std::hypot(dvx, dvy);
  if (dv > max_dv) {
    dvx *= max_dv / dv;
    dvy *= max_dv / dv;
  }
  MppiState next = state;
  next.vx += dvx;
  next.vy += dvy;
  next.wz = output.wz;
  next.x += next.vx * dt;
  next.y += next.vy * dt;
  next.yaw += next.wz * dt;
  return next;
}

// 向かい合ってすれ違う瞬間を見つける
class PassDetector
{
public:
  explicit PassDetector(const std::size_t lane_count)
  : previous_along_(lane_count)
  {
  }

  // すれ違った瞬間なら、進行方向から見た相手の横位置（左が正）を返す
  std::optional<double> update(
    const std::size_t lane, const MppiState & state, const LaneState & other)
  {
    const double speed = std::hypot(state.vx, state.vy);
    if (speed < kDirectionMinSpeedMps) {
      previous_along_[lane].reset();
      return std::nullopt;
    }
    const double rel_x = other.x - state.x;
    const double rel_y = other.y - state.y;
    const double along = (rel_x * state.vx + rel_y * state.vy) / speed;
    const double lateral = (state.vx * rel_y - state.vy * rel_x) / speed;
    const bool oncoming = speed >= kPassingMinSpeedMps &&
      std::hypot(other.vx, other.vy) >= kPassingMinSpeedMps &&
      state.vx * other.vx + state.vy * other.vy < 0.0;
    const std::optional<double> previous = previous_along_[lane];
    previous_along_[lane] = along;
    const bool passed = previous.has_value() && *previous > 0.0 && along <= 0.0;
    if (oncoming && passed && std::abs(lateral) < kPassingMaxLateralM) {
      return lateral;
    }
    return std::nullopt;
  }

private:
  std::vector<std::optional<double>> previous_along_;
};

// 周期ごとの接触と、すれ違いと、柵までの距離を記録する
void recordStep(
  const Scenario & scenario, const StackParams & params, const MppiState & state,
  const double time_s, const double dt, std::vector<double> & last_contact_s,
  PassDetector & passes, RunResult & result)
{
  result.min_fence_clearance_m = std::min(
    result.min_fence_clearance_m, fenceClearance(params.field, state.x, state.y, state.yaw));
  bool in_human_region = false;
  for (std::size_t i = 0; i < scenario.lanes.size(); ++i) {
    const Lane & lane = scenario.lanes[i];
    const LaneState other = laneStateAt(lane, time_s);
    const double gap = gapToObject(state.x, state.y, state.yaw, lane.kind, other.x, other.y);
    result.min_gap_m = std::min(result.min_gap_m, gap);
    // gapToObject の人は、機体の外形から人の体（半径 kHumanRadiusM）まで
    const bool human = lane.kind == ObjectKind::kHuman;
    in_human_region = in_human_region || (human && gap + kHumanRadiusM <= kHumanRegionM);
    if (gap <= 0.0) {
      if (time_s - last_contact_s[i] > kContactSeparationS) {
        ++result.contacts;
        result.human_contacts += human ? 1 : 0;
        const double closing = state.vx * (other.x - state.x) + state.vy * (other.y - state.y);
        if (std::hypot(state.vx, state.vy) >= kOwnFaultSpeedMps && closing > 0.0) {
          ++result.own_fault_contacts;
          result.human_own_fault_contacts += human ? 1 : 0;
        }
        // 当たるときの近づく速さ（中心を結ぶ向きの相対速度）
        const double distance = std::hypot(other.x - state.x, other.y - state.y);
        const double impact = distance > 1e-6 ?
          std::max(
          ((state.vx - other.vx) * (other.x - state.x) +
          (state.vy - other.vy) * (other.y - state.y)) / distance, 0.0) : 0.0;
        result.impact_speed_sum_mps += impact;
        if (impact >= kHardImpactSpeedMps) {
          ++result.hard_contacts;
        }
      }
      last_contact_s[i] = time_s;
    }
    const auto lateral = passes.update(i, state, other);
    if (lateral.has_value()) {
      const bool on_rule_side = (*lateral > 0.0) == params.mppi.passing.side_right;
      ++(on_rule_side ? result.passes_on_rule_side : result.passes_on_other_side);
    }
  }
  if (in_human_region) {
    result.human_region_s += dt;
    result.human_region_max_speed_mps =
      std::max(result.human_region_max_speed_mps, std::hypot(state.vx, state.vy));
  }
}
}  // namespace

LaneState laneStateAt(const Lane & lane, const double time_s)
{
  const double dx = lane.end.x - lane.start.x;
  const double dy = lane.end.y - lane.start.y;
  const double length = std::hypot(dx, dy);
  if (length < 1e-6 || lane.peak_speed_mps <= 0.0) {
    return LaneState{lane.start.x, lane.start.y, 0.0, 0.0};
  }
  const double angular_rate = 2.0 * lane.peak_speed_mps / length;
  const double angle = angular_rate * time_s + 2.0 * M_PI * lane.phase;
  const double ratio = 0.5 * (1.0 - std::cos(angle));
  const double rate = 0.5 * angular_rate * std::sin(angle);
  return LaneState{lane.start.x + dx * ratio, lane.start.y + dy * ratio, dx * rate, dy * rate};
}

double gapToObject(
  const double own_x, const double own_y, const double own_yaw, const ObjectKind kind,
  const double x, const double y)
{
  const double c = std::cos(own_yaw);
  const double s = std::sin(own_yaw);
  const double rel_x = x - own_x;
  const double rel_y = y - own_y;
  if (kind == ObjectKind::kHuman) {
    const double local_x = std::abs(c * rel_x + s * rel_y);
    const double local_y = std::abs(-s * rel_x + c * rel_y);
    if (local_x <= kRobotHalfSizeM && local_y <= kRobotHalfSizeM) {
      return -kHumanRadiusM;
    }
    return std::hypot(
      std::max(local_x - kRobotHalfSizeM, 0.0), std::max(local_y - kRobotHalfSizeM, 0.0)) -
           kHumanRadiusM;
  }
  // 分離軸の定理で、回転した 2 つの正方形の距離の下限を求める
  double separation = -std::numeric_limits<double>::infinity();
  for (const auto & [ax, ay] : {std::pair{c, s}, std::pair{-s, c}, std::pair{1.0, 0.0},
      std::pair{0.0, 1.0}})
  {
    const double own_radius =
      kRobotHalfSizeM * (std::abs(c * ax + s * ay) + std::abs(-s * ax + c * ay));
    const double other_radius = kRobotHalfSizeM * (std::abs(ax) + std::abs(ay));
    separation = std::max(
      separation, std::abs(rel_x * ax + rel_y * ay) - own_radius - other_radius);
  }
  return separation;
}

double fenceClearance(const FieldBounds & field, const double x, const double y, const double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  double clearance = std::numeric_limits<double>::infinity();
  for (const auto & [corner_x, corner_y] : {std::pair{1.0, 1.0}, std::pair{1.0, -1.0},
      std::pair{-1.0, 1.0}, std::pair{-1.0, -1.0}})
  {
    const double px = x + kRobotHalfSizeM * (c * corner_x - s * corner_y);
    const double py = y + kRobotHalfSizeM * (s * corner_x + c * corner_y);
    clearance = std::min(
      {clearance, field.x_max - px, px - field.x_min, field.y_max - py, py - field.y_min});
  }
  return clearance;
}

Scenario parseScenario(const YAML::Node & root)
{
  if (!root["start"] || !root["goals"] || !root["goals"].IsSequence() ||
    root["goals"].size() == 0U)
  {
    throw std::invalid_argument(
            "シナリオに start と goals が必要です（相手の動きだけのシナリオは走らせられません）");
  }
  Scenario scenario;
  scenario.start = readGoal(root["start"], "start");
  for (const auto & goal : root["goals"]) {
    scenario.goals.push_back(readGoal(goal, "goals の要素"));
  }
  if (root["obstacles"]) {
    for (const auto & obstacle : root["obstacles"]) {
      scenario.lanes.push_back(readLane(obstacle));
    }
  }
  scenario.timeout_s = root["timeout_s"] ? root["timeout_s"].as<double>() : scenario.timeout_s;
  scenario.repeat = root["repeat"] ? root["repeat"].as<int>() : scenario.repeat;
  if (scenario.repeat <= 0 || scenario.timeout_s <= 0.0) {
    throw std::invalid_argument("シナリオの repeat と timeout_s は正の値にしてください");
  }
  return scenario;
}

bool applyParameter(StackParams & params, const std::string & name, const std::string & value)
{
  const auto & table = parameterTable();
  const auto found = table.find(name);
  if (found == table.end()) {
    return false;
  }
  try {
    found->second(params, value);
  } catch (const std::logic_error & error) {
    // std::stod の例外も logic_error で、その内容からは原因が分からない
    throw std::invalid_argument(name + " の値を読めません: " + value + "（" + error.what() + "）");
  }
  return true;
}

std::vector<std::string> applyParameterFile(StackParams & params, const YAML::Node & root)
{
  std::vector<std::string> unused;
  if (!root.IsMap()) {
    throw std::invalid_argument("パラメータファイルの形式が違います");
  }
  for (const auto & node : root) {
    std::vector<std::pair<std::string, std::string>> parameters;
    if (node.second["ros__parameters"]) {
      flattenParameters(node.second["ros__parameters"], "", parameters);
    }
    for (const auto & [name, value] : parameters) {
      if (!applyParameter(params, name, value)) {
        unused.push_back(name);
      }
    }
  }
  return unused;
}

RunResult simulateRun(
  const Scenario & scenario, const StackParams & params, const DistanceField & field,
  const std::uint32_t seed)
{
  std::mt19937 random(seed * kSeedScale + kSeedOffset);
  const double start_time_s =
    std::uniform_real_distribution<double>(kStartTimeMinS, kStartTimeMaxS)(random);
  const double dt = params.mppi.dt;
  const std::size_t goals_total = scenario.goals.size() * static_cast<std::size_t>(scenario.repeat);
  const double time_limit_s = scenario.timeout_s * static_cast<double>(scenario.repeat);

  MppiController controller(seed);
  StuckMonitor stuck;
  MppiState state;
  state.x = scenario.start.x;
  state.y = scenario.start.y;
  state.yaw = scenario.start.yaw;
  auto tracks = initialTracks(scenario.lanes, start_time_s);
  std::vector<SeenObject> seen(scenario.lanes.size());
  std::vector<double> last_contact_s(
    scenario.lanes.size(), -std::numeric_limits<double>::infinity());
  PassDetector passes(scenario.lanes.size());
  safety::BodyVelocity last_output;
  ReferencePath path;
  std::size_t goal_index = 0U;
  GoalHoldMonitor goal_hold;
  OncomingMonitor oncoming;
  bool yielding = false;
  TrackMotionFilter motion_filter;
  safety::ImpactMitigator mitigator;
  std::vector<std::uint8_t> coasting;
  std::vector<std::uint8_t> moving;
  double time_s = start_time_s;
  double plan_ready_s = start_time_s;
  double next_plan_s = start_time_s;

  RunResult result;
  result.goals_total = static_cast<int>(goals_total);

  const auto next_goal = [&]() {
      ++goal_index;
      stuck.reset();
      controller.reset();
      goal_hold.reset();
      oncoming.reset();
      plan_ready_s = time_s + kPlanDelayS;
      next_plan_s = plan_ready_s;
    };

  while (time_s - start_time_s < time_limit_s && goal_index < goals_total) {
    const auto tracked = applyOcclusion(
      state, updateTracks(scenario.lanes, time_s, params.tracking, dt, random, tracks), time_s,
      params.tracking, seen, coasting);
    // 計画は追跡の速度をそのまま使う（mppi_lite と同じ）。safety_gate と同じく、正面の相手と衝撃の緩和の
    // 判定には、ロボットの追跡の速度を信じてよいか（TrackMotionFilter）を使う。人はそのまま信じる
    const std::vector<PredictedObject> & objects = tracked;
    moving.assign(tracked.size(), 0U);
    for (std::size_t i = 0; i < tracked.size(); ++i) {
      const bool is_moving = motion_filter.update(
        tracked[i].id, time_s, std::hypot(tracked[i].vx, tracked[i].vy), tracked[i].velocity_sigma,
        coasting[i] != 0U, params.motion_filter);
      moving[i] = is_moving || tracked[i].kind != ObjectKind::kRobot ? 1U : 0U;
    }
    const Goal & goal = scenario.goals[goal_index % scenario.goals.size()];
    const auto stop = limitCommand(state, 0.0, 0.0, 0.0, params.mppi);
    MppiCommand command = stop;

    if (time_s >= plan_ready_s) {
      if (time_s >= next_plan_s) {
        path = ReferencePath({Point2{state.x, state.y}, Point2{goal.x, goal.y}}, goal.yaw);
        next_plan_s = time_s + params.plan_period_s;
      }
      const double speed_limit = computeSpeedLimit(
        params.speed_limit, objects, state, params.mppi.footprint, params.max_speed_mps);
      // 止まっている間や下がっている間に向かってくる相手がいれば、その場を基準に避ける。
      // evade_while_stopped なら相手すべて、evade_humans_while_stopped なら人だけ。どちらも false なら避けない
      const auto evade = [&]() -> std::optional<MppiCommand> {
          if (!shouldEvadeWhileStopped(
              state, objects, params.mppi, params.evade_while_stopped,
              params.evade_humans_while_stopped))
          {
            controller.reset();
            return std::nullopt;
          }
          const ReferencePath here({Point2{state.x, state.y}, Point2{state.x, state.y}});
          return controller.solve(
            state, here, objects, &field, SE2{}, params.mppi, speed_limit);
        };

      const double remaining = path.remaining(state.x, state.y);
      const bool reached =
        std::hypot(goal.x - state.x, goal.y - state.y) <= params.goal_tolerance_m &&
        std::hypot(state.vx, state.vy) <= params.goal_speed_tolerance_mps;
      if (reached) {
        ++result.goals_reached;
        next_goal();
      } else {
        if (isOnCollisionCourse(state, objects, params.mppi)) {
          stuck.holdProgress(time_s, remaining);
        }
        // ゴールの近くを相手が通る予測なら手前で待つ（mppi_lite_node の goalHold と同じ）
        const auto hold_length = stuck.state() == StuckState::kDriving ?
          goal_hold.update(
          time_s, state, path, objects, params.mppi, params.goal_approach, speed_limit) :
          std::nullopt;
        if (hold_length.has_value()) {
          stuck.holdProgress(time_s, remaining);
        }
        // 正面・合流・横切りの相手には停止線で止まる（mppi_lite_node の oncomingYield と同じ）
        const YieldPlan yield =
          oncoming.update(time_s, state, path, objects, params.mppi, speed_limit);
        if (yield.yielding > 0) {
          stuck.holdProgress(time_s, remaining);
          result.yields += yielding ? 0 : 1;
        }
        yielding = yield.yielding > 0;
        std::optional<double> cut_length = hold_length;
        if (yield.stop_length.has_value()) {
          cut_length = std::min(cut_length.value_or(*yield.stop_length), *yield.stop_length);
        }
        switch (stuck.update(time_s, remaining, state.x, state.y, params.stuck)) {
          case StuckState::kDriving:
            command = controller.solve(
              state, cut_length.has_value() ? path.truncated(*cut_length) : path,
              markHeld(objects, yield.held_ids), &field, SE2{}, params.mppi,
              std::min(speed_limit, yield.speed_limit));
            break;
          case StuckState::kWait:
            result.recovery_s += dt;
            command = evade().value_or(stop);
            break;
          case StuckState::kBackoff:
            result.recovery_s += dt;
            if (const auto evasion = evade(); evasion.has_value()) {
              command = *evasion;
            } else {
              const Point2 direction = chooseBackoffDirection(
                state, path, objects, &field, SE2{}, params.mppi);
              // mppi_lite_node と同じく、見えない側へ下がるときは上限をかける
              const double speed = params.backoff_speed_mps *
                BlindSideLimit(params.mppi.blind_side).scale(
                direction.x * params.backoff_speed_mps, direction.y * params.backoff_speed_mps,
                std::cos(state.yaw), std::sin(state.yaw));
              command = limitCommand(
                state, direction.x * speed, direction.y * speed, 0.0, params.mppi);
            }
            break;
          case StuckState::kFailed:
            // run_scenario.py と同じく、到達できなかったゴールを飛ばす
            next_goal();
            break;
        }
      }
    }

    const safety::BodyVelocity output =
      applySafetyGate(
      command, state, tracked, moving, last_output, params, field, time_s, dt, mitigator,
      result.mitigations);
    last_output = output;
    state = stepRobot(state, output, dt);
    time_s += dt;
    result.distance_m += std::hypot(state.vx, state.vy) * dt;
    recordStep(scenario, params, state, time_s, dt, last_contact_s, passes, result);
  }
  result.duration_s = time_s - start_time_s;
  return result;
}

Summary summarize(const std::vector<RunResult> & results)
{
  Summary summary;
  summary.runs = static_cast<int>(results.size());
  if (results.empty()) {
    return summary;
  }
  double contacts = 0.0;
  double own_fault = 0.0;
  double duration = 0.0;
  double distance = 0.0;
  double recovery = 0.0;
  double goals = 0.0;
  double contact_free = 0.0;
  double deep = 0.0;
  double impact_sum = 0.0;
  double hard = 0.0;
  double mitigations = 0.0;
  double yields = 0.0;
  double human_contacts = 0.0;
  double human_own_fault = 0.0;
  double human_region = 0.0;
  for (const auto & result : results) {
    human_contacts += result.human_contacts;
    human_own_fault += result.human_own_fault_contacts;
    human_region += result.human_region_s;
    summary.human_region_max_speed_mps =
      std::max(summary.human_region_max_speed_mps, result.human_region_max_speed_mps);
    contacts += result.contacts;
    own_fault += result.own_fault_contacts;
    impact_sum += result.impact_speed_sum_mps;
    hard += result.hard_contacts;
    mitigations += result.mitigations;
    yields += result.yields;
    duration += result.duration_s;
    distance += result.distance_m;
    recovery += result.recovery_s;
    goals += result.goals_reached;
    contact_free += result.contacts == 0 ? 1.0 : 0.0;
    deep += result.min_gap_m < kDeepContactM ? 1.0 : 0.0;
    summary.min_fence_clearance_m =
      std::min(summary.min_fence_clearance_m, result.min_fence_clearance_m);
    summary.fence_touch_runs += result.min_fence_clearance_m <= 0.0 ? 1 : 0;
    summary.passes_on_rule_side += result.passes_on_rule_side;
    summary.passes_total += result.passes_on_rule_side + result.passes_on_other_side;
  }
  const double runs = static_cast<double>(results.size());
  const double rate_per_s = duration > 0.0 ? contacts / duration : 0.0;
  // 比の推定量の標準誤差。実行ごとの長さが違うので、接触数の分散だけでは求められない
  double squared_residuals = 0.0;
  for (const auto & result : results) {
    const double residual = result.contacts - rate_per_s * result.duration_s;
    squared_residuals += residual * residual;
  }
  const double standard_error = results.size() > 1U && duration > 0.0 ?
    std::sqrt(squared_residuals * runs / (runs - 1.0)) / duration : 0.0;

  summary.contacts_per_100s = 100.0 * rate_per_s;
  summary.contacts_per_100s_margin = 100.0 * 1.96 * standard_error;
  summary.own_fault_per_100s = duration > 0.0 ? 100.0 * own_fault / duration : 0.0;
  summary.contact_free_ratio = contact_free / runs;
  summary.deep_contact_ratio = deep / runs;
  summary.goals_reached_mean = goals / runs;
  summary.goals_total = results.front().goals_total;
  summary.mean_duration_s = duration / runs;
  summary.mean_speed_mps = duration > 0.0 ? distance / duration : 0.0;
  summary.mean_recovery_s = recovery / runs;
  summary.mean_impact_speed_mps = contacts > 0.0 ? impact_sum / contacts : 0.0;
  summary.hard_contacts_per_100s = duration > 0.0 ? 100.0 * hard / duration : 0.0;
  summary.mitigations_per_100s = duration > 0.0 ? 100.0 * mitigations / duration : 0.0;
  summary.yields_per_100s = duration > 0.0 ? 100.0 * yields / duration : 0.0;
  summary.human_contacts_per_100s = duration > 0.0 ? 100.0 * human_contacts / duration : 0.0;
  summary.human_own_fault_per_100s = duration > 0.0 ? 100.0 * human_own_fault / duration : 0.0;
  summary.human_region_per_100s = duration > 0.0 ? 100.0 * human_region / duration : 0.0;
  return summary;
}
}  // namespace core_planning::crowd_model
