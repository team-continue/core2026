// 経路の追従（大域計画の線からのはみ出し）を、Gazebo を使わずに比べるコマンド。
//
// 大域計画（JPS と曲線化）、局所計画（MPPI）、安全停止（静止物だけ）を実機と同じライブラリで動かし、
// 機体は core_gazebo の omni_force_drive と同じ式（加速度の上限と、床との滑り）で進める。
// 動く相手は置かない（シナリオの相手は読まない）。crowd_stats と同じ設定ファイルを読み、--case で
// 指定した値だけを変えて、同じ乱数の列で比べる。
//
// Gazebo との違い:
// - 自己位置は真値に速度の雑音を足したもの。LiDAR の走査や FAST-LIO の遅れは、指令の遅れ（bench.command_delay_s。
//   駆動の無駄時間を含む。制御の周期より長くてもよい）と、局所計画の遅れの補償（mppi.latency_compensation_s、
//   または mppi.drive.* で送った指令から先を読む）だけで表す
// - 大域計画は機体の真の位置から計画する。止まっている相手やゴールの代わりの位置は扱わない

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <deque>
#include <exception>
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include <optional>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "core_planning/crowd_model.hpp"
#include "core_planning/drive_delay.hpp"
#include "core_planning/grid_map.hpp"
#include "core_planning/jps.hpp"
#include "core_planning/mppi.hpp"
#include "core_planning/path_smoother.hpp"
#include "core_planning/safety.hpp"
#include "core_planning/traction_estimator.hpp"

namespace
{
namespace crowd_model = core_planning::crowd_model;
namespace fs = std::filesystem;
namespace safety = core_planning::safety;
using core_planning::DistanceField;
using core_planning::DriveParams;
using core_planning::GridCell;
using core_planning::MppiCommand;
using core_planning::MppiController;
using core_planning::MppiParams;
using core_planning::MppiState;
using core_planning::OdometrySample;
using core_planning::OccupancyGrid;
using core_planning::Point2;
using core_planning::ReferencePath;
using core_planning::SE2;
using core_planning::SentCommand;
using core_planning::StuckMonitor;
using core_planning::StuckState;

constexpr const char * kUsage =
  "使い方: ros2 run core_planning tracking_bench [オプション]\n"
  "  --scenario FILE    シナリオ（start と goals だけを使う。既定は core_gazebo の obstacle_goals.yaml）\n"
  "  --config-dir DIR   パラメータファイルの場所（既定は core_launch の config）\n"
  "  --map FILE         静止物の地図（既定は core_launch の maps/arena_obstacles.pcd）\n"
  "  --runs N           条件ごとの実行回数（乱数の種を変える。既定 8）\n"
  "  --jobs N           同時に動かすスレッドの数（既定は CPU の数）\n"
  "  --case A=1,B=2     条件（既定に足す値）。何度でも指定できる。plant.* は機体の滑り、bench.* は試験台の設定、\n"
  "                     mppi.drive.* は局所計画が見込む駆動の遅れ（bench.command_delay_s が本当の遅れ）、\n"
  "                     traction.* は床の推定、zone.* は床の区画（範囲の中だけ床の値を変える）\n"
  "  --base A=1,B=2     すべての条件に先に足す値（例: mppi.max_speed_mps=5.0）\n"
  "  --trace PREFIX     条件ごとに 1 回目の走行の軌跡を PREFIX<番号>.csv に書く\n"
  "  --trace-run N      軌跡を書く走行の番号（0 から。既定 0）\n"
  "  --json FILE        条件ごとの結果を JSON Lines で追記する\n";

constexpr double kFieldInsetM = 0.1;
constexpr double kStaticMinHeightM = 0.15;
constexpr double kStaticMaxHeightM = 2.0;
constexpr double kNearestFreeRadiusM = 2.0;
constexpr double kGoalTimeoutS = 40.0;
// はみ出しを数える間隔と、数える速さの下限（止まる前後の揺れを除く）
constexpr double kSamplePeriodS = 0.05;
constexpr double kSampleMinSpeedMps = 0.3;
// 曲がりの外側へのはみ出しとみなす、経路の前後 kTurnSpanM の向きの差
constexpr double kTurnSpanM = 1.5;
constexpr double kTurnMinRad = 10.0 * M_PI / 180.0;

// 床と車輪の滑りを含む機体の運動。core_gazebo/src/omni_force_drive.cpp と同じ式なので、変えるときは
// 両方を直す。
// - 指令の機体座標の速度に、前後・横・回転の比（床が出せる割合）をかけたものを目標にする
// - 目標と今の速度の差を time_constant_s で埋める加速度（0 なら 1 ステップで埋める）を、駆動の上限
//   max_linear_accel で抑える
// - 床が出せる加速度（摩擦円）static_friction_accel を超えたら滑り、kinetic_friction_accel しか出ない
// - 目標の速度に、時間相関のある雑音（床のむら）を、目標の速さに比例する大きさで足す（止まっていれば 0）
struct PlantParams
{
  double max_linear_accel{5.0};
  double max_angular_accel{10.0};
  double time_constant_s{0.0};
  double static_friction_accel{0.0};
  double kinetic_friction_accel{0.0};
  double longitudinal_ratio{1.0};
  double lateral_ratio{1.0};
  double yaw_ratio{1.0};
  // 床のむらの雑音の大きさ（目標の速さに対する割合、標準偏差）と相関時間 [s]
  double noise_ratio{0.0};
  double noise_time_s{0.5};
  double dt{0.005};
};

// 床の区画。範囲の中では、床の値（摩擦・割合・雑音）を指定した値に変える。機体の値（時定数・駆動の上限）は
// 変えない
struct FloorZone
{
  bool enabled{false};
  double x_min{-std::numeric_limits<double>::infinity()};
  double x_max{std::numeric_limits<double>::infinity()};
  double y_min{-std::numeric_limits<double>::infinity()};
  double y_max{std::numeric_limits<double>::infinity()};
  std::optional<double> static_friction_accel;
  std::optional<double> kinetic_friction_accel;
  std::optional<double> longitudinal_ratio;
  std::optional<double> lateral_ratio;
  std::optional<double> yaw_ratio;
  std::optional<double> noise_ratio;

  bool contains(const double x, const double y) const
  {
    return enabled && x >= x_min && x <= x_max && y >= y_min && y <= y_max;
  }
};

PlantParams zonePlant(const PlantParams & base, const FloorZone & zone)
{
  PlantParams result = base;
  result.static_friction_accel = zone.static_friction_accel.value_or(base.static_friction_accel);
  result.kinetic_friction_accel = zone.kinetic_friction_accel.value_or(base.kinetic_friction_accel);
  result.longitudinal_ratio = zone.longitudinal_ratio.value_or(base.longitudinal_ratio);
  result.lateral_ratio = zone.lateral_ratio.value_or(base.lateral_ratio);
  result.yaw_ratio = zone.yaw_ratio.value_or(base.yaw_ratio);
  result.noise_ratio = zone.noise_ratio.value_or(base.noise_ratio);
  return result;
}

struct PlantState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
  double noise_x{0.0};
  double noise_y{0.0};
  bool sliding{false};
};

PlantState stepPlant(
  PlantState state, const safety::BodyVelocity & command, const PlantParams & params,
  std::mt19937 & random)
{
  const double dt = params.dt;
  const double c = std::cos(state.yaw);
  const double s = std::sin(state.yaw);
  const double bx = command.vx * params.longitudinal_ratio;
  const double by = command.vy * params.lateral_ratio;
  const double target_speed = std::hypot(bx, by);
  const double target_x = c * bx - s * by + params.noise_ratio * target_speed * state.noise_x;
  const double target_y = s * bx + c * by + params.noise_ratio * target_speed * state.noise_y;
  const double tau = std::max(params.time_constant_s, dt);
  double ax = (target_x - state.vx) / tau;
  double ay = (target_y - state.vy) / tau;
  const double demand = std::hypot(ax, ay);
  if (demand > params.max_linear_accel) {
    ax *= params.max_linear_accel / demand;
    ay *= params.max_linear_accel / demand;
  }
  const double accel = std::hypot(ax, ay);
  state.sliding = params.static_friction_accel > 0.0 && accel > params.static_friction_accel;
  if (state.sliding) {
    // 動摩擦を省いたら静止摩擦と同じ（omni_force_drive の既定と同じ）
    const double kinetic = params.kinetic_friction_accel > 0.0 ?
      params.kinetic_friction_accel : params.static_friction_accel;
    ax *= kinetic / accel;
    ay *= kinetic / accel;
  }
  const double alpha = std::clamp(
    (command.wz * params.yaw_ratio - state.wz) / tau, -params.max_angular_accel,
    params.max_angular_accel);
  state.vx += ax * dt;
  state.vy += ay * dt;
  state.wz += alpha * dt;
  state.x += state.vx * dt;
  state.y += state.vy * dt;
  state.yaw += state.wz * dt;
  if (params.noise_ratio > 0.0 && params.noise_time_s > 0.0) {
    // 標準偏差 1 の、時間相関のある雑音（Ornstein-Uhlenbeck）
    std::normal_distribution<double> normal(0.0, 1.0);
    const double decay = dt / params.noise_time_s;
    const double scale = std::sqrt(2.0 * decay);
    state.noise_x += -state.noise_x * decay + scale * normal(random);
    state.noise_y += -state.noise_y * decay + scale * normal(random);
  }
  return state;
}

struct BenchParams
{
  crowd_model::StackParams stack;
  PlantParams plant;
  // 計画の周期と、計画してから指令が機体に届くまでの遅れ（駆動の無駄時間を含む）
  double control_period_s{0.1};
  double command_delay_s{0.02};
  // 自己位置の速度の雑音（標準偏差）
  double velocity_noise_mps{0.02};
  double latency_compensation_s{0.05};
  // 局所計画が見込む駆動の遅れ（mppi_lite_node の mppi.drive.*）。predict_delay なら、送った指令で
  // drive.dead_time_s 先まで状態を進めてから計画する
  DriveParams drive;
  // true なら、予測にも drive.time_constant_s の一次遅れを入れる（mppi.drive_time_constant_s）
  bool drive_lag_in_rollout{true};
  // 床の推定（mppi_lite_node の traction.*）
  core_planning::TractionParams traction;
  // 正なら推定せず、最初からこの床の静止摩擦 [m/s2] で上限を割り当てる（正解の床。試験台だけの設定）
  double traction_fixed_friction_mps2{0.0};
  FloorZone zone;
  // 大域計画
  double planner_resolution_m{0.1};
  core_planning::SmoothingParams smoothing;
  bool narrow_enabled{true};
  double narrow_min_side_clearance_m{0.2};
  // true なら、曲線化の目標の余裕を、狭い通路を使わない半径（外接円 + 余裕）に extra_clearance_m を
  // 足した距離にする（global_planner_node と同じ）
  bool smoothing_keep_clearance{true};
};

double toDouble(const std::string & text)
{
  std::size_t used = 0U;
  const double value = std::stod(text, &used);
  if (used != text.size()) {
    throw std::invalid_argument("数値ではありません: " + text);
  }
  return value;
}

bool toBool(const std::string & text)
{
  if (text == "true" || text == "True" || text == "1") {
    return true;
  }
  if (text == "false" || text == "False" || text == "0") {
    return false;
  }
  throw std::invalid_argument("真偽値ではありません: " + text);
}

// 予測の一次遅れ（mppi.drive_time_constant_s）を、駆動の時定数と lag_in_rollout に合わせる
void syncRolloutLag(BenchParams & params)
{
  params.stack.mppi.drive_time_constant_s =
    params.drive_lag_in_rollout ? params.drive.time_constant_s : 0.0;
}

// 試験台の名前（plant.*、bench.*、mppi.drive.*、大域計画）を設定する。知らなければ false
bool applyBenchParameter(BenchParams & params, const std::string & name, const std::string & value)
{
  PlantParams & plant = params.plant;
  const std::vector<std::pair<std::string, double *>> doubles{
    {"plant.max_linear_accel", &plant.max_linear_accel},
    {"plant.max_angular_accel", &plant.max_angular_accel},
    {"plant.time_constant_s", &plant.time_constant_s},
    {"plant.static_friction_accel", &plant.static_friction_accel},
    {"plant.kinetic_friction_accel", &plant.kinetic_friction_accel},
    {"plant.longitudinal_ratio", &plant.longitudinal_ratio},
    {"plant.lateral_ratio", &plant.lateral_ratio},
    {"plant.yaw_ratio", &plant.yaw_ratio},
    {"plant.noise_ratio", &plant.noise_ratio},
    {"plant.noise_time_s", &plant.noise_time_s},
    {"plant.dt", &plant.dt},
    {"bench.control_period_s", &params.control_period_s},
    {"bench.command_delay_s", &params.command_delay_s},
    {"bench.velocity_noise_mps", &params.velocity_noise_mps},
    {"mppi.latency_compensation_s", &params.latency_compensation_s},
    {"mppi.drive.dead_time_s", &params.drive.dead_time_s},
    {"mppi.drive.time_constant_s", &params.drive.time_constant_s},
    {"mppi.drive.max_accel_mps2", &params.drive.max_accel_mps2},
    {"mppi.drive.max_angular_accel_radps2", &params.drive.max_angular_accel_radps2},
    {"traction.update_period_s", &params.traction.update_period_s},
    {"traction.window_s", &params.traction.window_s},
    {"traction.horizon_s", &params.traction.horizon_s},
    {"traction.min_excitation_mps", &params.traction.min_excitation_mps},
    {"traction.min_improvement_ratio", &params.traction.min_improvement_ratio},
    {"traction.min_improvement_mps", &params.traction.min_improvement_mps},
    {"traction.initial_friction_mps2", &params.traction.initial_friction_mps2},
    {"traction.accel_ratio", &params.traction.accel_ratio},
    {"traction.decel_ratio", &params.traction.decel_ratio},
    {"traction.lateral_ratio", &params.traction.lateral_ratio},
    {"traction.max_accel_mps2", &params.traction.max_accel_mps2},
    {"traction.max_decel_mps2", &params.traction.max_decel_mps2},
    {"traction.max_lateral_accel_mps2", &params.traction.max_lateral_accel_mps2},
    {"traction.min_accel_mps2", &params.traction.min_accel_mps2},
    {"traction.min_decel_mps2", &params.traction.min_decel_mps2},
    {"traction.min_lateral_accel_mps2", &params.traction.min_lateral_accel_mps2},
    {"bench.traction_fixed_friction_mps2", &params.traction_fixed_friction_mps2},
    {"zone.x_min", &params.zone.x_min},
    {"zone.x_max", &params.zone.x_max},
    {"zone.y_min", &params.zone.y_min},
    {"zone.y_max", &params.zone.y_max},
    {"planner.resolution_m", &params.planner_resolution_m},
    {"planner.smoothing.spacing_m", &params.smoothing.spacing_m},
    {"planner.smoothing.extra_clearance_m", &params.smoothing.extra_clearance_m},
    {"planner.smoothing.smooth_weight", &params.smoothing.smooth_weight},
    {"planner.smoothing.clearance_weight", &params.smoothing.clearance_weight},
    {"narrow_passage.min_side_clearance_m", &params.narrow_min_side_clearance_m},
  };
  for (const auto & [key, target] : doubles) {
    if (key == name) {
      *target = toDouble(value);
      if (name == "narrow_passage.min_side_clearance_m") {
        params.stack.mppi.narrow.min_side_clearance_m = *target;
      }
      syncRolloutLag(params);
      return true;
    }
  }
  const std::vector<std::pair<std::string, std::optional<double> *>> zone_values{
    {"zone.static_friction_accel", &params.zone.static_friction_accel},
    {"zone.kinetic_friction_accel", &params.zone.kinetic_friction_accel},
    {"zone.longitudinal_ratio", &params.zone.longitudinal_ratio},
    {"zone.lateral_ratio", &params.zone.lateral_ratio},
    {"zone.yaw_ratio", &params.zone.yaw_ratio},
    {"zone.noise_ratio", &params.zone.noise_ratio},
  };
  for (const auto & [key, target] : zone_values) {
    if (key == name) {
      *target = toDouble(value);
      return true;
    }
  }
  if (name == "zone.enabled") {
    params.zone.enabled = toBool(value);
    return true;
  }
  if (name == "traction.enabled") {
    params.traction.enabled = toBool(value);
    return true;
  }
  if (name == "traction.raise_after") {
    params.traction.raise_after = static_cast<int>(toDouble(value));
    return true;
  }
  if (name == "mppi.drive.predict_delay") {
    params.drive.predict_delay = toBool(value);
    return true;
  }
  if (name == "mppi.drive.lag_in_rollout") {
    params.drive_lag_in_rollout = toBool(value);
    syncRolloutLag(params);
    return true;
  }
  if (name == "planner.smoothing.iterations") {
    params.smoothing.iterations = static_cast<int>(toDouble(value));
    return true;
  }
  if (name == "planner.smoothing.enabled") {
    params.smoothing.enabled = toBool(value);
    return true;
  }
  if (name == "bench.smoothing_keep_clearance") {
    params.smoothing_keep_clearance = toBool(value);
    return true;
  }
  if (name == "narrow_passage.enabled") {
    params.narrow_enabled = toBool(value);
    params.stack.mppi.narrow.enabled = params.narrow_enabled;
    return true;
  }
  if (name == "narrow_passage.lateral_tolerance_m") {
    params.stack.mppi.narrow.lateral_tolerance_m = toDouble(value);
    return true;
  }
  if (name == "mppi.max_accel_mps2") {
    params.stack.mppi.max_accel_mps2 = toDouble(value);
    return true;
  }
  return false;
}

bool applyAnyParameter(BenchParams & params, const std::string & name, const std::string & value)
{
  try {
    if (applyBenchParameter(params, name, value)) {
      return true;
    }
  } catch (const std::logic_error & error) {
    throw std::invalid_argument(name + " の値を読めません: " + value + "（" + error.what() + "）");
  }
  return crowd_model::applyParameter(params.stack, name, value);
}

// "A=1,B=2" の値を設定する
void applyCase(BenchParams & params, const std::string & text)
{
  std::size_t begin = 0U;
  while (begin < text.size()) {
    const std::size_t comma = std::min(text.find(',', begin), text.size());
    const std::string item = text.substr(begin, comma - begin);
    const std::size_t equal = item.find('=');
    if (equal == std::string::npos || equal == 0U || equal + 1U == item.size()) {
      throw std::invalid_argument("条件は 名前=値 の形で指定してください: " + item);
    }
    const std::string name = item.substr(0U, equal);
    if (!applyAnyParameter(params, name, item.substr(equal + 1U))) {
      throw std::invalid_argument("知らないパラメータです: " + name);
    }
    begin = comma + 1U;
  }
}

// ROS のパラメータファイルを平らな名前と値にする
void flatten(
  const YAML::Node & node, const std::string & prefix,
  std::vector<std::pair<std::string, std::string>> & out)
{
  if (node.IsMap()) {
    for (const auto & child : node) {
      const std::string key = child.first.as<std::string>();
      flatten(child.second, prefix.empty() ? key : prefix + "." + key, out);
    }
  } else if (node.IsScalar()) {
    out.emplace_back(prefix, node.as<std::string>());
  }
}

BenchParams loadConfig(const fs::path & config_dir)
{
  BenchParams params;
  const std::vector<std::string> files{
    "mppi.yaml", "passing_rule.yaml", "speed_limit.yaml", "stuck_recovery.yaml", "footprint.yaml",
    "safety_stop.yaml", "field.yaml", "planner.yaml", "narrow_passage.yaml"};
  for (const auto & file : files) {
    const fs::path path = config_dir / file;
    if (!fs::exists(path)) {
      throw std::runtime_error("設定ファイルがありません: " + path.string());
    }
    const YAML::Node root = YAML::LoadFile(path.string());
    for (const auto & node : root) {
      if (!node.second["ros__parameters"]) {
        continue;
      }
      std::vector<std::pair<std::string, std::string>> values;
      flatten(node.second["ros__parameters"], "", values);
      for (const auto & [name, value] : values) {
        // 知らない名前（試験台で使わない設定）は読み飛ばす
        static_cast<void>(applyAnyParameter(params, name, value));
      }
    }
  }
  return params;
}

// 大域計画（global_planner_node と同じ手順）
class GlobalPlanner
{
public:
  GlobalPlanner(const BenchParams & params, const std::string & map_path)
  : smoothing_(params.smoothing)
  {
    OccupancyGrid grid =
      core_planning::makeFieldGrid(params.stack.field, params.planner_resolution_m);
    core_planning::addStaticObstacles(
      grid, map_path, params.stack.field, kFieldInsetM, kStaticMinHeightM, kStaticMaxHeightM);
    distance_ = core_planning::computeDistanceField(grid);
    const auto & footprint = params.stack.mppi.footprint;
    radius_ = core_planning::planningInflationRadius(
      footprint.half_length_m, footprint.half_width_m, footprint.margin_m, params.narrow_enabled,
      params.narrow_min_side_clearance_m);
    if (params.smoothing_keep_clearance) {
      const double open_radius = core_planning::planningInflationRadius(
        footprint.half_length_m, footprint.half_width_m, footprint.margin_m, false,
        params.narrow_min_side_clearance_m);
      smoothing_.extra_clearance_m += std::max(open_radius - radius_, 0.0);
    }
    inflated_ = core_planning::inflate(distance_, radius_);
  }

  std::optional<std::vector<Point2>> plan(const Point2 & robot, const Point2 & goal) const
  {
    const auto & info = inflated_.info();
    const int radius_cells = static_cast<int>(std::ceil(kNearestFreeRadiusM / info.resolution));
    const auto start = core_planning::nearestFreeCell(
      inflated_, info.cellAt(robot.x, robot.y), radius_cells);
    const auto end = core_planning::nearestFreeCell(
      inflated_, info.cellAt(goal.x, goal.y), radius_cells);
    if (!start.has_value() || !end.has_value()) {
      return std::nullopt;
    }
    const auto found = core_planning::findPath(inflated_, *start, *end);
    if (!found.has_value()) {
      return std::nullopt;
    }
    const auto cells = core_planning::simplifyPath(inflated_, *found);
    std::vector<std::pair<double, double>> points;
    points.emplace_back(robot.x, robot.y);
    for (std::size_t i = 1U; i < cells.size(); ++i) {
      points.emplace_back(info.centerX(cells[i].x), info.centerY(cells[i].y));
    }
    const GridCell goal_cell = info.cellAt(goal.x, goal.y);
    if (!cells.empty() && cells.back() == goal_cell) {
      if (cells.size() > 1U) {
        points.back() = {goal.x, goal.y};
      } else {
        points.emplace_back(goal.x, goal.y);
      }
    } else if (cells.size() == 1U) {
      points.emplace_back(info.centerX(cells[0].x), info.centerY(cells[0].y));
    }
    if (smoothing_.enabled) {
      try {
        if (const auto smoothed =
          core_planning::smoothPath(points, distance_, radius_, smoothing_))
        {
          points = *smoothed;
        }
      } catch (const std::invalid_argument &) {
        // global_planner_node と同じく、曲線にできなければ折れ線を使う
      }
    }
    std::vector<Point2> result;
    result.reserve(points.size());
    for (const auto & [x, y] : points) {
      result.push_back(Point2{x, y});
    }
    return result;
  }

  double radius() const {return radius_;}

private:
  core_planning::SmoothingParams smoothing_;
  DistanceField distance_;
  OccupancyGrid inflated_;
  double radius_{0.0};
};

struct Sample
{
  double deviation;
  double outward;
  double speed;
};

struct RunMetrics
{
  int goals_reached{0};
  int goals_total{0};
  double duration_s{0.0};
  double distance_m{0.0};
  double max_speed_mps{0.0};
  std::vector<Sample> samples;
  double first_plan_max_m{0.0};
  double min_static_clearance_m{std::numeric_limits<double>::infinity()};
  int static_contacts{0};
  double stop_s{0.0};
  double slow_s{0.0};
  double sliding_s{0.0};
  int failed_goals{0};
  // 計画の加速度と減速度の上限を時間で積んだもの（床の推定で変わる）と、積んだ時間
  double accel_limit_integral{0.0};
  double decel_limit_integral{0.0};
  double limit_time_s{0.0};
  // 静止物に触れ始めた時刻と位置、速さ、滑っていたか（--verbose で出す）
  std::vector<std::array<double, 6>> contact_events;
};

// 点から折れ線までの距離と、最も近い点の経路に沿った位置
std::pair<double, double> distanceToPath(const ReferencePath & path, const Point2 & point)
{
  const double s = path.project(point.x, point.y);
  const Point2 closest = path.pointAt(s);
  return {std::hypot(point.x - closest.x, point.y - closest.y), s};
}

// 曲がりの外側へのはみ出し（外側でなければ 0）
double outwardDeviation(const ReferencePath & path, const Point2 & point)
{
  const auto [distance, s] = distanceToPath(path, point);
  const Point2 a = path.directionAt(std::max(s - kTurnSpanM, 0.0));
  const Point2 b = path.directionAt(std::min(s + kTurnSpanM, path.length()));
  const double turn = std::atan2(a.x * b.y - a.y * b.x, a.x * b.x + a.y * b.y);
  if (std::abs(turn) < kTurnMinRad) {
    return 0.0;
  }
  const Point2 closest = path.pointAt(s);
  const Point2 d = path.directionAt(s);
  const double side = d.x * (point.y - closest.y) - d.y * (point.x - closest.x);
  // 左へ曲がる（turn > 0）なら右（side < 0）が外側
  return side * turn < 0.0 ? distance : 0.0;
}

safety::BodyVelocity toBody(const double vx, const double vy, const double wz, const double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return safety::BodyVelocity{c * vx + s * vy, -s * vx + c * vy, wz};
}

struct TraceRow
{
  double t;
  PlantState state;
  double deviation;
  double outward;
  int plan;
  // 使っている床の値、最後の推定（推定しなかったら NaN）、本当の床の静止摩擦（0 なら滑らない）[m/s2]
  double friction;
  double estimate;
  double floor;
  // 最後の推定で滑りが見えたか（1 / 0、推定しなかったら -1）と、使っている速度の割合（前後、横）
  int slipping;
  double ratio_long;
  double ratio_lat;
};

RunMetrics simulate(
  const crowd_model::Scenario & scenario, const BenchParams & params, const GlobalPlanner & planner,
  const DistanceField & field, const std::uint32_t seed, std::vector<TraceRow> * trace,
  std::vector<std::vector<Point2>> * trace_plans)
{
  const crowd_model::StackParams & stack = params.stack;
  std::mt19937 random(seed * 7919U + 17U);
  std::normal_distribution<double> normal(0.0, 1.0);
  MppiController controller(seed);
  StuckMonitor stuck;
  PlantState plant;
  plant.x = scenario.start.x;
  plant.y = scenario.start.y;
  plant.yaw = scenario.start.yaw;

  RunMetrics metrics;
  metrics.goals_total = static_cast<int>(scenario.goals.size()) * scenario.repeat;
  ReferencePath path;
  ReferencePath first_path;
  int plan_id = -1;
  std::size_t goal_index = 0U;
  double time_s = 0.0;
  double goal_start_s = 0.0;
  double next_plan_s = 0.0;
  double next_control_s = 0.0;
  double next_sample_s = 0.0;
  safety::BodyVelocity applied;
  safety::BodyVelocity last_output;
  // 機体に届く前の指令（届く時刻の順）と、送った指令の記録（先を読むのと、床の推定に使う）
  std::deque<std::pair<double, safety::BodyVelocity>> in_flight;
  std::vector<SentCommand> sent;
  // 床の区画の中の床と、床の推定
  const PlantParams zone_plant = zonePlant(params.plant, params.zone);
  const bool fixed_traction = params.traction_fixed_friction_mps2 > 0.0;
  const bool estimate_traction = params.traction.enabled && !fixed_traction;
  const bool use_traction = estimate_traction || fixed_traction;
  core_planning::TractionState traction(
    fixed_traction ? params.traction_fixed_friction_mps2 : params.traction.initial_friction_mps2);
  MppiParams mppi = use_traction ?
    core_planning::applyTraction(stack.mppi, traction.friction(), params.traction) : stack.mppi;
  std::vector<OdometrySample> samples;
  double next_traction_s = params.traction.update_period_s;
  double last_estimate = std::numeric_limits<double>::quiet_NaN();
  int last_slipping = -1;
  const double sent_keep_s = std::max(
    1.0, estimate_traction ? params.traction.window_s + params.traction.horizon_s + 1.0 : 0.0);
  bool was_contact = false;
  const double time_limit_s = kGoalTimeoutS * static_cast<double>(metrics.goals_total) + 10.0;
  const safety::Footprint & footprint = stack.footprint;

  while (goal_index < static_cast<std::size_t>(metrics.goals_total) && time_s < time_limit_s) {
    const auto & goal = scenario.goals[goal_index % scenario.goals.size()];
    if (time_s >= next_plan_s) {
      const auto planned = planner.plan(Point2{plant.x, plant.y}, Point2{goal.x, goal.y});
      if (planned.has_value() && planned->size() >= 2U) {
        path = ReferencePath(*planned, goal.yaw);
        ++plan_id;
        if (first_path.empty()) {
          first_path = path;
        }
        if (trace_plans != nullptr) {
          trace_plans->push_back(*planned);
        }
      }
      next_plan_s = time_s + stack.plan_period_s;
    }
    if (time_s >= next_control_s) {
      next_control_s += params.control_period_s;
      // 自己位置: 真値の速度に雑音を足し、mppi_lite_node と同じく遅れの分だけ進める。安全停止には、
      // 先を読むかによらず前と同じ（今の速度で遅れの補償の分だけ進めた）状態を渡す
      MppiState measured;
      measured.vx = plant.vx + params.velocity_noise_mps * normal(random);
      measured.vy = plant.vy + params.velocity_noise_mps * normal(random);
      measured.wz = plant.wz;
      measured.x = plant.x;
      measured.y = plant.y;
      measured.yaw = plant.yaw;
      MppiState compensated = measured;
      compensated.x += measured.vx * params.latency_compensation_s;
      compensated.y += measured.vy * params.latency_compensation_s;
      compensated.yaw += measured.wz * params.latency_compensation_s;
      core_planning::pruneSentCommands(sent, time_s - sent_keep_s, params.drive.dead_time_s);
      if (estimate_traction) {
        samples.push_back(OdometrySample{time_s, measured});
        const double keep_from_s =
          time_s - (params.traction.window_s + params.traction.horizon_s + 1.0);
        const auto keep = std::find_if(
          samples.begin(), samples.end(),
          [keep_from_s](const OdometrySample & sample) {return sample.time_s >= keep_from_s;});
        samples.erase(samples.begin(), keep);
        if (time_s >= next_traction_s) {
          next_traction_s = time_s + params.traction.update_period_s;
          const auto estimate = core_planning::estimateTraction(
            samples, sent, params.drive, params.traction, time_s, traction.ratio());
          last_estimate = estimate.has_value() ?
            estimate->friction_mps2 : std::numeric_limits<double>::quiet_NaN();
          last_slipping = estimate.has_value() ? (estimate->slipping ? 1 : 0) : -1;
          const double before = traction.friction();
          if (traction.update(estimate, params.traction) != before) {
            mppi = core_planning::applyTraction(stack.mppi, traction.friction(), params.traction);
          }
        }
      }
      metrics.limit_time_s += params.control_period_s;
      metrics.accel_limit_integral += mppi.max_accel_mps2 * params.control_period_s;
      metrics.decel_limit_integral += mppi.reference_decel_mps2 * params.control_period_s;
      MppiState state = compensated;
      if (params.drive.predict_delay) {
        state = core_planning::predictDelayedState(
          measured, time_s, time_s + params.drive.dead_time_s, sent, params.drive,
          use_traction ? core_planning::floorFor(traction.friction()) :
          core_planning::FloorTraction{});
      }
      MppiCommand command = core_planning::limitCommand(state, 0.0, 0.0, 0.0, mppi);
      const bool reached =
        std::hypot(goal.x - plant.x, goal.y - plant.y) <= stack.goal_tolerance_m &&
        std::hypot(plant.vx, plant.vy) <= stack.goal_speed_tolerance_mps;
      const bool timed_out = time_s - goal_start_s > kGoalTimeoutS;
      if (reached || timed_out) {
        if (reached) {
          ++metrics.goals_reached;
        } else {
          ++metrics.failed_goals;
        }
        ++goal_index;
        goal_start_s = time_s;
        controller.reset();
        stuck.reset();
        path = ReferencePath();
        first_path = ReferencePath();
        next_plan_s = time_s;
      } else if (!path.empty()) {
        const double remaining = path.remaining(state.x, state.y);
        const double speed_limit = core_planning::computeSpeedLimit(
          stack.speed_limit, {}, state, stack.mppi.footprint, stack.max_speed_mps);
        const StuckState stuck_state =
          stuck.update(time_s, remaining, state.x, state.y, stack.stuck);
        // 行き詰まりの待ちと後退は、相手のいない試験台では止まるだけにする
        if (stuck_state == StuckState::kDriving) {
          command = controller.solve(state, path, {}, &field, SE2{}, mppi, speed_limit);
        } else if (stuck_state == StuckState::kFailed) {
          ++metrics.failed_goals;
          ++goal_index;
          goal_start_s = time_s;
          controller.reset();
          stuck.reset();
          path = ReferencePath();
          first_path = ReferencePath();
          next_plan_s = time_s;
        }
      }
      safety::BodyVelocity output = toBody(command.vx, command.vy, command.wz, state.yaw);
      if (stack.safety_enabled) {
        const auto supervised = safety::superviseCommand(
          {}, safety::PlanarPose{compensated.x, compensated.y, compensated.yaw},
          toBody(compensated.vx, compensated.vy, compensated.wz, compensated.yaw), output, footprint,
          stack.safety, &field, safety::PlanarPose{});
        if (supervised.decision == safety::Decision::kStop) {
          metrics.stop_s += params.control_period_s;
        } else if (supervised.decision == safety::Decision::kSlow) {
          metrics.slow_s += params.control_period_s;
        }
        output = safety::limitDeceleration(
          last_output, supervised.output, stack.safety.max_decel_mps2, params.control_period_s);
      }
      last_output = output;
      in_flight.emplace_back(time_s + params.command_delay_s, output);
      sent.push_back(SentCommand{time_s, output.vx, output.vy, output.wz});
    }
    while (!in_flight.empty() && time_s >= in_flight.front().first) {
      applied = in_flight.front().second;
      in_flight.pop_front();
    }
    const bool in_zone = params.zone.contains(plant.x, plant.y);
    plant = stepPlant(plant, applied, in_zone ? zone_plant : params.plant, random);
    time_s += params.plant.dt;
    const double speed = std::hypot(plant.vx, plant.vy);
    metrics.distance_m += speed * params.plant.dt;
    metrics.max_speed_mps = std::max(metrics.max_speed_mps, speed);
    if (plant.sliding) {
      metrics.sliding_s += params.plant.dt;
    }
    if (time_s >= next_sample_s) {
      next_sample_s += kSamplePeriodS;
      const double clearance = safety::staticClearance(
        field, safety::PlanarPose{}, safety::PlanarPose{plant.x, plant.y, plant.yaw}, footprint);
      metrics.min_static_clearance_m = std::min(metrics.min_static_clearance_m, clearance);
      const bool contact = clearance <= 0.0;
      if (contact && !was_contact) {
        ++metrics.static_contacts;
        metrics.contact_events.push_back(
          {time_s, plant.x, plant.y, speed, plant.sliding ? 1.0 : 0.0,
            static_cast<double>(goal_index)});
      }
      was_contact = contact;
      double deviation = 0.0;
      double outward = 0.0;
      if (!path.empty()) {
        const Point2 point{plant.x, plant.y};
        deviation = distanceToPath(path, point).first;
        outward = outwardDeviation(path, point);
        if (speed >= kSampleMinSpeedMps) {
          metrics.samples.push_back(Sample{deviation, outward, speed});
        }
        if (!first_path.empty()) {
          metrics.first_plan_max_m =
            std::max(metrics.first_plan_max_m, distanceToPath(first_path, point).first);
        }
      }
      if (trace != nullptr) {
        trace->push_back(
          TraceRow{time_s, plant, deviation, outward, plan_id, traction.friction(), last_estimate,
            (in_zone ? zone_plant : params.plant).static_friction_accel, last_slipping,
            traction.ratio()[0], traction.ratio()[1]});
      }
    }
  }
  metrics.duration_s = time_s;
  return metrics;
}

double percentile(std::vector<double> values, const double ratio)
{
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const auto index = static_cast<std::size_t>(
    std::clamp(ratio * static_cast<double>(values.size() - 1U), 0.0,
    static_cast<double>(values.size() - 1U)));
  return values[index];
}

struct Summary
{
  int runs{0};
  double goals_reached_mean{0.0};
  int goals_total{0};
  int failed_goals{0};
  double duration_mean_s{0.0};
  double speed_mean_mps{0.0};
  double max_speed_mps{0.0};
  double deviation_mean_m{0.0};
  double deviation_p95_m{0.0};
  double deviation_max_m{0.0};
  double outward_p95_m{0.0};
  double outward_max_m{0.0};
  double fast_deviation_p95_m{0.0};
  double first_plan_max_m{0.0};
  double min_static_clearance_m{std::numeric_limits<double>::infinity()};
  int static_contacts{0};
  double stop_s_mean{0.0};
  double slide_ratio{0.0};
  // 計画の加速度と減速度の上限の、時間の平均
  double accel_limit_mean{0.0};
  double decel_limit_mean{0.0};
};

Summary summarize(const std::vector<RunMetrics> & runs)
{
  Summary summary;
  summary.runs = static_cast<int>(runs.size());
  std::vector<double> deviations;
  std::vector<double> outwards;
  std::vector<double> fast;
  double driving_s = 0.0;
  double sliding_s = 0.0;
  double limit_time_s = 0.0;
  for (const auto & run : runs) {
    summary.accel_limit_mean += run.accel_limit_integral;
    summary.decel_limit_mean += run.decel_limit_integral;
    limit_time_s += run.limit_time_s;
    summary.goals_reached_mean += run.goals_reached;
    summary.goals_total = run.goals_total;
    summary.failed_goals += run.failed_goals;
    summary.duration_mean_s += run.duration_s;
    summary.speed_mean_mps += run.distance_m / std::max(run.duration_s, 1e-6);
    summary.max_speed_mps = std::max(summary.max_speed_mps, run.max_speed_mps);
    summary.first_plan_max_m = std::max(summary.first_plan_max_m, run.first_plan_max_m);
    summary.min_static_clearance_m =
      std::min(summary.min_static_clearance_m, run.min_static_clearance_m);
    summary.static_contacts += run.static_contacts;
    summary.stop_s_mean += run.stop_s;
    driving_s += run.duration_s;
    sliding_s += run.sliding_s;
    for (const auto & sample : run.samples) {
      deviations.push_back(sample.deviation);
      if (sample.outward > 0.0) {
        outwards.push_back(sample.outward);
      }
      if (sample.speed >= 2.0) {
        fast.push_back(sample.deviation);
      }
    }
  }
  const double n = std::max(static_cast<double>(runs.size()), 1.0);
  summary.goals_reached_mean /= n;
  summary.duration_mean_s /= n;
  summary.speed_mean_mps /= n;
  summary.stop_s_mean /= n;
  summary.slide_ratio = sliding_s / std::max(driving_s, 1e-6);
  summary.accel_limit_mean /= std::max(limit_time_s, 1e-6);
  summary.decel_limit_mean /= std::max(limit_time_s, 1e-6);
  if (!deviations.empty()) {
    double sum = 0.0;
    for (const double d : deviations) {
      sum += d;
    }
    summary.deviation_mean_m = sum / static_cast<double>(deviations.size());
    summary.deviation_p95_m = percentile(deviations, 0.95);
    summary.deviation_max_m = *std::max_element(deviations.begin(), deviations.end());
  }
  summary.outward_p95_m = percentile(outwards, 0.95);
  summary.outward_max_m =
    outwards.empty() ? 0.0 : *std::max_element(outwards.begin(), outwards.end());
  summary.fast_deviation_p95_m = percentile(fast, 0.95);
  return summary;
}

std::string packageShare(const std::string & name)
{
  return ament_index_cpp::get_package_share_directory(name);
}

struct Arguments
{
  bool help{false};
  std::string scenario;
  std::string config_dir;
  std::string map;
  int runs{8};
  int jobs{0};
  std::vector<std::string> cases;
  std::vector<std::string> base;
  std::string trace;
  // 軌跡を書く走行の番号（乱数の種）
  int trace_run{0};
  std::string json;
  bool verbose{false};
};

Arguments parseArguments(const std::vector<std::string> & args)
{
  Arguments arguments;
  for (std::size_t i = 0; i < args.size(); ++i) {
    const std::string & option = args[i];
    const auto value = [&]() -> std::string {
        if (i + 1U >= args.size()) {
          throw std::invalid_argument(option + " に値がありません");
        }
        return args[++i];
      };
    if (option == "-h" || option == "--help") {
      arguments.help = true;
    } else if (option == "--scenario") {
      arguments.scenario = value();
    } else if (option == "--config-dir") {
      arguments.config_dir = value();
    } else if (option == "--map") {
      arguments.map = value();
    } else if (option == "--runs") {
      arguments.runs = std::max(static_cast<int>(toDouble(value())), 1);
    } else if (option == "--jobs") {
      arguments.jobs = std::max(static_cast<int>(toDouble(value())), 1);
    } else if (option == "--case") {
      arguments.cases.push_back(value());
    } else if (option == "--base") {
      arguments.base.push_back(value());
    } else if (option == "--trace") {
      arguments.trace = value();
    } else if (option == "--trace-run") {
      arguments.trace_run = static_cast<int>(toDouble(value()));
    } else if (option == "--json") {
      arguments.json = value();
    } else if (option == "--verbose") {
      arguments.verbose = true;
    } else {
      throw std::invalid_argument("知らないオプションです: " + option);
    }
  }
  return arguments;
}

void writeTrace(
  const std::string & path, const std::vector<TraceRow> & rows,
  const std::vector<std::vector<Point2>> & plans)
{
  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("書けません: " + path);
  }
  out << "t,x,y,yaw,vx,vy,wz,deviation,outward,plan,sliding,friction,estimate,floor,slipping,ratio_long,"
    "ratio_lat\n";
  for (const auto & row : rows) {
    out << row.t << ',' << row.state.x << ',' << row.state.y << ',' << row.state.yaw << ',' <<
      row.state.vx << ',' << row.state.vy << ',' << row.state.wz << ',' << row.deviation << ',' <<
      row.outward << ',' << row.plan << ',' << (row.state.sliding ? 1 : 0) << ',' << row.friction <<
      ',' << row.estimate << ',' << row.floor << ',' << row.slipping << ',' << row.ratio_long << ',' <<
      row.ratio_lat << '\n';
  }
  std::ofstream plan_out(path + ".plans");
  for (std::size_t i = 0; i < plans.size(); ++i) {
    for (const auto & point : plans[i]) {
      plan_out << i << ',' << point.x << ',' << point.y << '\n';
    }
  }
}

int run(const Arguments & arguments)
{
  const fs::path scenario_path = arguments.scenario.empty() ?
    fs::path(packageShare("core_gazebo")) / "scenarios" / "obstacle_goals.yaml" :
    fs::path(arguments.scenario);
  const fs::path config_dir = arguments.config_dir.empty() ?
    fs::path(packageShare("core_launch")) / "config" : fs::path(arguments.config_dir);
  const fs::path map_path = arguments.map.empty() ?
    fs::path(packageShare("core_launch")) / "maps" / "arena_obstacles.pcd" :
    fs::path(arguments.map);
  const int jobs = arguments.jobs > 0 ? arguments.jobs :
    std::max(static_cast<int>(std::thread::hardware_concurrency()), 1);

  const auto scenario = crowd_model::parseScenario(YAML::LoadFile(scenario_path.string()));
  BenchParams base = loadConfig(config_dir);
  for (const auto & text : arguments.base) {
    applyCase(base, text);
  }
  std::vector<std::pair<std::string, BenchParams>> cases{{"既定", base}};
  for (const auto & text : arguments.cases) {
    BenchParams params = base;
    applyCase(params, text);
    cases.emplace_back(text, params);
  }
  std::printf(
    "シナリオ: %s（ゴール %zu）、地図: %s、条件ごとに %d 回\n", scenario_path.filename().c_str(),
    scenario.goals.size(), map_path.filename().c_str(), arguments.runs);
  std::printf(
    "条件 | 到達 | 失敗 | 周回[秒] | 平均[m/s] | 最高[m/s] | はみ出し 平均/95%%/最大[m] | 2m/s以上の95%%[m] | "
    "曲がりの外 95%%/最大[m] | 最初の線から最大[m] | 静止物 最小[m]/接触 | 停止[秒] | 滑り[%%] | "
    "加速度/減速度の上限 平均[m/s2]\n");
  for (std::size_t c = 0; c < cases.size(); ++c) {
    const auto & [name, params] = cases[c];
    const auto field = core_planning::buildDistanceField(
      params.stack.field, params.stack.grid_resolution_m, map_path.string());
    const GlobalPlanner planner(params, map_path.string());
    std::vector<RunMetrics> results(static_cast<std::size_t>(arguments.runs));
    std::vector<TraceRow> trace;
    std::vector<std::vector<Point2>> trace_plans;
    std::atomic<int> next{0};
    const auto worker = [&]() {
        for (int i = next++; i < arguments.runs; i = next++) {
          const bool traced = i == arguments.trace_run && !arguments.trace.empty();
          results[static_cast<std::size_t>(i)] = simulate(
            scenario, params, planner, field, static_cast<std::uint32_t>(i),
            traced ? &trace : nullptr, traced ? &trace_plans : nullptr);
        }
      };
    std::vector<std::future<void>> workers;
    for (int i = 0; i < std::min(jobs, arguments.runs); ++i) {
      workers.push_back(std::async(std::launch::async, worker));
    }
    for (auto & running : workers) {
      running.get();
    }
    const Summary s = summarize(results);
    if (arguments.verbose) {
      for (std::size_t r = 0; r < results.size(); ++r) {
        for (const auto & e : results[r].contact_events) {
          std::printf(
            "  接触 回 %zu t=%.2f (%.2f, %.2f) 速さ %.2f 滑り %.0f ゴール %.0f\n", r, e[0], e[1], e[2],
            e[3], e[4], e[5]);
        }
      }
    }
    std::printf(
      "%s | %.2f/%d | %d | %.1f | %.2f | %.2f | %.2f/%.2f/%.2f | %.2f | %.2f/%.2f | %.2f | "
      "%.2f/%d | %.1f | %.1f | %.2f/%.2f\n",
      name.c_str(), s.goals_reached_mean, s.goals_total, s.failed_goals, s.duration_mean_s,
      s.speed_mean_mps, s.max_speed_mps, s.deviation_mean_m, s.deviation_p95_m, s.deviation_max_m,
      s.fast_deviation_p95_m, s.outward_p95_m, s.outward_max_m, s.first_plan_max_m,
      s.min_static_clearance_m, s.static_contacts, s.stop_s_mean, 100.0 * s.slide_ratio,
      s.accel_limit_mean, s.decel_limit_mean);
    std::fflush(stdout);
    if (!arguments.trace.empty()) {
      writeTrace(arguments.trace + std::to_string(c) + ".csv", trace, trace_plans);
    }
    if (!arguments.json.empty()) {
      std::ofstream out(arguments.json, std::ios::app);
      out << "{\"case\": \"" << name << "\", \"base\": \"";
      for (const auto & text : arguments.base) {
        out << text << ";";
      }
      out << "\", \"scenario\": \"" << scenario_path.filename().string() << "\", \"runs\": " <<
        s.runs << ", \"goals_reached_mean\": " << s.goals_reached_mean << ", \"goals_total\": " <<
        s.goals_total << ", \"failed_goals\": " << s.failed_goals << ", \"duration_mean_s\": " <<
        s.duration_mean_s << ", \"speed_mean_mps\": " << s.speed_mean_mps <<
        ", \"max_speed_mps\": " <<
        s.max_speed_mps << ", \"deviation_mean_m\": " << s.deviation_mean_m <<
        ", \"deviation_p95_m\": " << s.deviation_p95_m << ", \"deviation_max_m\": " <<
        s.deviation_max_m << ", \"fast_deviation_p95_m\": " << s.fast_deviation_p95_m <<
        ", \"outward_p95_m\": " << s.outward_p95_m << ", \"outward_max_m\": " << s.outward_max_m <<
        ", \"first_plan_max_m\": " << s.first_plan_max_m << ", \"min_static_clearance_m\": " <<
        s.min_static_clearance_m << ", \"static_contacts\": " << s.static_contacts <<
        ", \"stop_s_mean\": " << s.stop_s_mean << ", \"slide_ratio\": " << s.slide_ratio <<
        ", \"accel_limit_mean\": " << s.accel_limit_mean << ", \"decel_limit_mean\": " <<
        s.decel_limit_mean << "}\n";
    }
  }
  return 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Arguments arguments = parseArguments(std::vector<std::string>(argv + 1, argv + argc));
    if (arguments.help) {
      std::printf("%s", kUsage);
      return 0;
    }
    return run(arguments);
  } catch (const std::exception & error) {
    std::fprintf(stderr, "[NG] %s\n%s", error.what(), kUsage);
    return 1;
  }
}
