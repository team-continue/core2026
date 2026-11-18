// フィールドの占有格子で JPS を解いてゴールまでの経路を出すノード。
//
// 一定周期のほか、止まっている相手で経路が塞がれたときにも計画し直す。
// 柵はセンサで近距離を見られないので、地図と同じ扱いで占有にしてある。
// x が NaN のゴールを受け取ると、ゴールを解除して空のパスを出す。
// /behavior/goal_route（通過点…、最後がゴール）を受けると、通過点を順に通ってゴールへ行く経路を出す。
// 通過点では止まらない（局所計画は経路の終わりのゴールでだけ止まる）。残りの通過点は /planning/waypoints に出す。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <core_msgs/msg/navigation_status.hpp>
#include <core_msgs/msg/tracked_object.hpp>
#include <core_msgs/msg/tracked_object_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "core_nav_common/field_bounds.hpp"
#include "core_nav_common/parameters.hpp"
#include "core_planning/grid_map.hpp"
#include "core_planning/static_map_params.hpp"
#include "core_planning/hybrid_astar.hpp"
#include "core_planning/jps.hpp"
#include "core_planning/path_smoother.hpp"
#include "core_planning/waypoints.hpp"

namespace core_planning
{
namespace
{
using geometry_msgs::msg::PoseStamped;
using core_msgs::msg::NavigationStatus;
using core_msgs::msg::TrackedObject;
using core_msgs::msg::TrackedObjectArray;
using nav_msgs::msg::Path;

// 地図の固定物とする点の範囲。柵は別に扱うので、フィールドの端は除く
constexpr double kFieldInsetM = 0.1;
// 占有の中にいるときに探す空きセルの範囲
constexpr double kNearestFreeRadiusM = 2.0;
// 経路の閉塞で計画し直す最短の間隔
constexpr double kMinReplanIntervalS = 0.25;
// 新しいゴールを受けた直後に届いた、前のゴールの FAILED・REACHED は使わない（局所計画が新しい経路を受けるまで、
// 前のゴールの状態を出し続ける。REACHED は機体とゴールの距離でも確かめるが、近くに出した新しいゴールは通ってしまう）
constexpr double kStaleStatusIgnoreS = 0.5;
// 追跡結果がこれより古ければ使わない
constexpr double kTracksMaxAgeS = 1.0;
// 代わりのゴールを探す角度の刻み
constexpr int kAlternativeAngleCount = 24;

struct PlanarPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct PlannerParams
{
  double blocked_speed_mps{0.3};
  double reached_clear_radius_m{0.5};
  // 機体がこの距離の中に入った通過点は通過済みにする
  double waypoint_pass_radius_m{0.5};
  // 通過点に要る、静止物（地図・柵・進入禁止ゾーン）までの余裕。足りない通過点は、受けたときに余裕のある一番
  // 近い所へ移す（局所計画は静止物から余裕を取るので、近すぎる通過点には通過済みの距離まで近づけない）
  double waypoint_clearance_m{1.0};
  // ジャンプポイントの間のセルにも直線で飛ぶ（shortcutPath）。false でジャンプポイントの間だけ（simplifyPath）
  bool any_angle_shortcut{true};
  // 探索の方法（planner.search）
  SearchMethod search{SearchMethod::kJps};
  // planner.hybrid.*（search が hybrid_astar のとき）
  HybridAStarParams hybrid;
  // 局所計画が行き詰まった所（/planning/stuck_point）を、通れない円として足す半径と時間。時間 0 で使わない
  double stuck_block_radius_m{0.4};
  double stuck_block_duration_s{20.0};
};

// 局所計画が行き詰まった所。期限までは止まっている相手と同じく通れない円にする
struct StuckBlock
{
  double x{0.0};
  double y{0.0};
  double expires_s{0.0};
};

struct GoalHandlingParams
{
  bool enabled{true};
  double occupied_radius_m{0.8};
  double wait_s{3.0};
  double alternative_radius_m{2.0};
  // true なら、ゴールが wait_s の間塞がれたままのとき、代わりのゴールへ行かずに FAILED で止まる
  bool fail_when_occupied{false};
  // true なら、止まっている相手（と局所計画が行き詰まった所）を避ける経路がない状態が blocked_wait_s 続いたとき、
  // 固定物だけで計画し直さずに FAILED で止まる
  bool fail_when_blocked{false};
  double blocked_wait_s{5.0};
};

struct SlowObstacle
{
  double x{0.0};
  double y{0.0};
  double radius{0.0};
};

double objectRadius(const std::uint8_t classification)
{
  if (classification == TrackedObject::CLASS_ROBOT) {
    return 0.45;
  }
  if (classification == TrackedObject::CLASS_HUMAN) {
    return 0.3;
  }
  return 0.5;
}

PlanarPose toPlanarPose(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & q = transform.transform.rotation;
  return PlanarPose{
    transform.transform.translation.x, transform.transform.translation.y,
    std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
}

// frame（map から見た姿勢 frame_in_map）で表した pose を、map で表す
PlanarPose toMapFrame(const PlanarPose & frame_in_map, const PlanarPose & pose)
{
  const double c = std::cos(frame_in_map.yaw);
  const double s = std::sin(frame_in_map.yaw);
  return PlanarPose{
    frame_in_map.x + c * pose.x - s * pose.y, frame_in_map.y + s * pose.x + c * pose.y,
    std::atan2(std::sin(frame_in_map.yaw + pose.yaw), std::cos(frame_in_map.yaw + pose.yaw))};
}

// 止まっている相手を map 座標系で返す
std::vector<SlowObstacle> slowObstaclesInMap(
  const TrackedObjectArray & tracks, const PlanarPose & map_to_odom, const double blocked_speed)
{
  const double c = std::cos(map_to_odom.yaw);
  const double s = std::sin(map_to_odom.yaw);
  std::vector<SlowObstacle> obstacles;
  for (const auto & object : tracks.objects) {
    if (std::hypot(object.velocity.x, object.velocity.y) >= blocked_speed) {
      continue;
    }
    obstacles.push_back(
      SlowObstacle{
        map_to_odom.x + c * object.position.x - s * object.position.y,
        map_to_odom.y + s * object.position.x + c * object.position.y,
        objectRadius(object.classification)});
  }
  return obstacles;
}

OccupancyGrid withObstacles(
  const OccupancyGrid & grid, const std::vector<SlowObstacle> & obstacles,
  const double inflation_radius)
{
  OccupancyGrid blocked = grid;
  for (const auto & obstacle : obstacles) {
    blocked.fillCircle(obstacle.x, obstacle.y, inflation_radius + obstacle.radius);
  }
  return blocked;
}

bool goalIsOccupied(
  const PlanarPose & goal, const std::vector<SlowObstacle> & obstacles, const double radius)
{
  return std::any_of(
    obstacles.begin(), obstacles.end(),
    [&goal, radius](const SlowObstacle & obstacle) {
      return std::hypot(obstacle.x - goal.x, obstacle.y - goal.y) < radius;
    });
}

// ゴールの周りの円周上で、機体に最も近い空き地点を返す
std::optional<PlanarPose> findAlternativeGoal(
  const OccupancyGrid & grid, const PlanarPose & goal, const PlanarPose & robot, const double radius)
{
  std::optional<PlanarPose> best;
  double best_distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < kAlternativeAngleCount; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / kAlternativeAngleCount;
    const PlanarPose candidate{
      goal.x + radius * std::cos(angle), goal.y + radius * std::sin(angle), goal.yaw};
    const double distance = std::hypot(candidate.x - robot.x, candidate.y - robot.y);
    if (distance < best_distance && grid.isFree(grid.info().cellAt(candidate.x, candidate.y))) {
      best = candidate;
      best_distance = distance;
    }
  }
  return best;
}

// 探した経路。cells は折れ点のセル（塞がれたかの確かめに使う）。hybrid_astar のときは、points に
// 円弧をたどる点の列（始点と終点のセルの中心から）が入る
struct PlannedRoute
{
  std::vector<GridCell> cells;
  std::optional<std::vector<std::pair<double, double>>> points;
};

// 点の列が通るセルの列（続けて同じセルは 1 つにする）
std::vector<GridCell> cellsAlong(
  const OccupancyGrid & grid, const std::vector<std::pair<double, double>> & points)
{
  std::vector<GridCell> cells;
  for (const auto & [x, y] : points) {
    const GridCell cell = grid.info().cellAt(x, y);
    if (cells.empty() || !(cells.back() == cell)) {
      cells.push_back(cell);
    }
  }
  return cells;
}

// 格子で経路を探す。始点と終点が占有の中なら、近くの空きセルから探す。hybrid_astar で見つからなければ
// JPS で探す
std::optional<PlannedRoute> planOnGrid(
  const OccupancyGrid & grid, const PlanarPose & robot, const PlanarPose & target,
  const SearchMethod search, const bool any_angle_shortcut, const HybridAStarParams & hybrid)
{
  const GridInfo & info = grid.info();
  const int radius_cells = static_cast<int>(std::ceil(kNearestFreeRadiusM / info.resolution));
  const auto start = nearestFreeCell(grid, info.cellAt(robot.x, robot.y), radius_cells);
  const auto goal = nearestFreeCell(grid, info.cellAt(target.x, target.y), radius_cells);
  if (!start.has_value() || !goal.has_value()) {
    return std::nullopt;
  }
  if (search == SearchMethod::kHybridAStar) {
    auto points = findPathHybridAStar(
      grid, {info.centerX(start->x), info.centerY(start->y)},
      {info.centerX(goal->x), info.centerY(goal->y)}, hybrid);
    if (points.has_value()) {
      std::vector<GridCell> cells = cellsAlong(grid, *points);
      return PlannedRoute{std::move(cells), std::move(points)};
    }
  }
  auto cells = searchPath(grid, *start, *goal, search, any_angle_shortcut);
  if (!cells.has_value()) {
    return std::nullopt;
  }
  return PlannedRoute{std::move(*cells), std::nullopt};
}

geometry_msgs::msg::Pose makePose(const double x, const double y, const double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.z = std::sin(0.5 * yaw);
  pose.orientation.w = std::cos(0.5 * yaw);
  return pose;
}

// 機体の位置から始まり、ゴールの位置と向きで終わるパスを作る
std::vector<std::pair<double, double>> toPoints(
  const OccupancyGrid & grid, const std::vector<GridCell> & cells, const PlanarPose & robot,
  const PlanarPose & target)
{
  const GridInfo & info = grid.info();
  std::vector<std::pair<double, double>> points;
  points.reserve(cells.size() + 2U);
  points.emplace_back(robot.x, robot.y);
  // 始点のセルは機体の位置で置き換える
  for (std::size_t i = 1U; i < cells.size(); ++i) {
    points.emplace_back(info.centerX(cells[i].x), info.centerY(cells[i].y));
  }
  // 終点のセルが目標のセルなら、セルの中心ではなく目標の正確な位置を使う
  const GridCell target_cell = info.cellAt(target.x, target.y);
  if (!cells.empty() && cells.back() == target_cell) {
    if (cells.size() > 1U) {
      points.back() = {target.x, target.y};
    } else {
      points.emplace_back(target.x, target.y);
    }
  } else if (cells.size() == 1U) {
    points.emplace_back(info.centerX(cells[0].x), info.centerY(cells[0].y));
  }
  return points;
}

Path makePath(
  const std::vector<std::pair<double, double>> & points, const double goal_yaw,
  const std::string & frame, const rclcpp::Time & stamp)
{
  Path path;
  path.header.stamp = stamp;
  path.header.frame_id = frame;
  path.poses.reserve(points.size());
  for (std::size_t i = 0U; i < points.size(); ++i) {
    const auto & [x, y] = points[i];
    double yaw = goal_yaw;
    if (i + 1U < points.size()) {
      yaw = std::atan2(points[i + 1U].second - y, points[i + 1U].first - x);
    }
    PoseStamped pose;
    pose.header = path.header;
    pose.pose = makePose(x, y, yaw);
    path.poses.push_back(pose);
  }
  return path;
}
}  // namespace

class GlobalPlannerNode final : public rclcpp::Node
{
public:
  explicit GlobalPlannerNode(const rclcpp::NodeOptions & options)
  : Node("global_planner", options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    using core_nav_common::declareBool;
    using core_nav_common::declareDouble;
    using core_nav_common::declareInt;
    using core_nav_common::declareString;
    declareString(*this, "map_path", "", "地図の PCD", true);
    declareStaticMapParams(*this);
    declareString(*this, "map_frame", "map", "地図のフレーム", true);
    declareString(*this, "odom_frame", "odom", "追跡結果のフレーム", true);
    declareString(*this, "base_frame", "base_link", "機体のフレーム", true);
    declareDouble(*this, "field.x_min", -12.0, -1000.0, 1000.0, "フィールドの範囲", true);
    declareDouble(*this, "field.x_max", 12.0, -1000.0, 1000.0, "フィールドの範囲", true);
    declareDouble(*this, "field.y_min", -7.5, -1000.0, 1000.0, "フィールドの範囲", true);
    declareDouble(*this, "field.y_max", 7.5, -1000.0, 1000.0, "フィールドの範囲", true);
    declareDouble(*this, "footprint.length_m", 0.8, 0.1, 5.0, "機体の前後の長さ");
    declareDouble(*this, "footprint.width_m", 0.8, 0.1, 5.0, "機体の左右の幅");
    declareDouble(*this, "footprint.margin_m", 0.1, 0.0, 2.0, "機体の余裕");
    // 局所計画（mppi_lite）と同じ値を使う。設定ファイルは全ノードで共通
    declareBool(
      *this, "narrow_passage.enabled", true,
      "片側の余裕が min_side_clearance_m 以上の狭い通路を通る経路も作る（false なら外接円の半径で膨らませる）");
    declareDouble(
      *this, "narrow_passage.min_side_clearance_m", 0.2, 0.0, 2.0,
      "通路を通るのに要る、機体を真ん中に置いたときの片側の余裕 [m]");
    declareDouble(*this, "planner.rate_hz", 2.0, 0.1, 50.0, "計画の周期", true);
    declareDouble(*this, "planner.resolution_m", 0.1, 0.02, 1.0, "格子の大きさ", true);
    declareDouble(*this, "planner.blocked_speed_mps", 0.3, 0.0, 5.0, "止まっているとみなす速度");
    declareDouble(
      *this, "planner.reached_clear_radius_m", 0.5, 0.01, 5.0, "到達でゴールを解除する距離");
    declareDouble(
      *this, "planner.waypoint_pass_radius_m", 0.5, 0.05, 5.0, "通過点を通過したとみなす距離");
    declareDouble(
      *this, "planner.waypoint_clearance_m", 1.0, 0.0, 3.0,
      "通過点に要る静止物までの余裕。足りなければ余裕のある一番近い所へ移す");
    declareBool(
      *this, "planner.any_angle_shortcut", true,
      "ジャンプポイントの間のセルにも直線で飛び、直線で行ける所の折れを除く（search が jps のとき）");
    declareString(
      *this, "planner.search", "jps", "探索の方法（jps / theta_star / jps_theta / hybrid_astar）",
      true);
    declareDouble(*this, "planner.hybrid.step_m", 0.3, 0.01, 5.0, "Hybrid A* の 1 歩の円弧の長さ", true);
    declareDouble(*this, "planner.hybrid.min_radius_m", 1.0, 0.05, 100.0, "Hybrid A* の最小の曲がりの半径", true);
    declareDouble(*this, "planner.hybrid.curvature_weight", 0.5, 0.0, 100.0, "Hybrid A* の曲率の重み", true);
    declareDouble(
      *this, "planner.hybrid.curvature_change_weight", 0.5, 0.0, 100.0, "Hybrid A* の曲率の変化の重み", true);
    declareInt(*this, "planner.hybrid.max_expansions", 100000, 1, 10000000, "Hybrid A* で調べる状態の上限", true);
    declareDouble(
      *this, "planner.stuck_block.radius_m", 0.4, 0.0, 5.0, "局所計画が行き詰まった所を塞ぐ円の半径");
    declareDouble(
      *this, "planner.stuck_block.duration_s", 20.0, 0.0, 600.0,
      "局所計画が行き詰まった所を塞いでおく時間（0 で使わない）");
    declareBool(*this, "planner.smoothing.enabled", true, "静止物の周りで経路を曲線にする");
    declareString(
      *this, "planner.smoothing.mode", "laplacian", "点を動かす方法（laplacian / bending）", true);
    declareDouble(
      *this, "planner.smoothing.bending_length_m", 1.0, 0.05, 20.0,
      "bending で曲がりを広げる長さの目安");
    declareDouble(*this, "planner.smoothing.spacing_m", 0.1, 0.02, 1.0, "曲線の点の間隔");
    declareDouble(
      *this, "planner.smoothing.extra_clearance_m", 0.3, 0.0, 2.0, "膨らみの外にさらに取る余裕");
    declareInt(*this, "planner.smoothing.iterations", 200, 0, 2000, "点を動かす回数の上限");
    declareDouble(
      *this, "planner.smoothing.smooth_weight", 0.25, 0.0, 1.0, "両隣の中点へ引く割合");
    declareDouble(
      *this, "planner.smoothing.clearance_weight", 0.25, 0.0, 1.0, "余裕の不足分を押し戻す割合");
    declareBool(*this, "goal_handling.enabled", true, "塞がれたゴールを扱う");
    declareDouble(
      *this, "goal_handling.occupied_radius_m", 0.8, 0.01, 10.0, "ゴールが塞がれたとする半径");
    declareDouble(*this, "goal_handling.wait_s", 3.0, 0.0, 600.0, "代わりのゴールへ移るまで待つ時間");
    declareDouble(
      *this, "goal_handling.alternative_radius_m", 2.0, 0.1, 20.0, "代わりのゴールを探す半径");
    declareBool(
      *this, "goal_handling.fail_when_occupied", false,
      "ゴールが塞がれたままなら、代わりのゴールへ行かずに FAILED で止まる");
    declareBool(
      *this, "goal_handling.fail_when_blocked", false,
      "止まっている相手を避ける経路がないままなら、FAILED で止まる");
    declareDouble(
      *this, "goal_handling.blocked_wait_s", 5.0, 0.0, 600.0, "経路が塞がれたまま待つ時間");

    loadParams();
    buildGrid();
    parameter_callback_ = add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &) {
        loadParams();
        // 機体の大きさが変わったら、占有を広げ直す
        const double radius = inflationRadius();
        if (std::abs(radius - inflated_radius_) > 1e-9) {
          inflated_ = inflate(distance_field_, radius);
          inflated_radius_ = radius;
        }
      });

    path_publisher_ = create_publisher<Path>("/planning/plan", rclcpp::QoS(1).transient_local());
    status_publisher_ = create_publisher<NavigationStatus>("/planning/planner/status", rclcpp::QoS(10));
    // ゴールに着いたら 1 回だけ true を出す（行動の決定に知らせる）
    goal_reached_publisher_ = create_publisher<std_msgs::msg::Bool>("/behavior/goal_reached", rclcpp::QoS(10));
    goal_subscription_ = create_subscription<PoseStamped>(
      "/behavior/goal_pose", rclcpp::QoS(10),
      std::bind(&GlobalPlannerNode::onGoal, this, std::placeholders::_1));
    route_subscription_ = create_subscription<Path>(
      "/behavior/goal_route", rclcpp::QoS(1).reliable(),
      std::bind(&GlobalPlannerNode::onRoute, this, std::placeholders::_1));
    // 残りの通過点。UI が後から繋いでも分かるよう、最後の値を残して出す
    waypoints_publisher_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/planning/waypoints", rclcpp::QoS(1).reliable().transient_local());
    publishWaypoints();
    tracks_subscription_ = create_subscription<TrackedObjectArray>(
      "/perception/tracks", rclcpp::QoS(10),
      std::bind(&GlobalPlannerNode::onTracks, this, std::placeholders::_1));
    navigation_subscription_ = create_subscription<NavigationStatus>(
      "/planning/navigation/status", rclcpp::QoS(10),
      std::bind(&GlobalPlannerNode::onNavigationStatus, this, std::placeholders::_1));
    stuck_point_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/planning/stuck_point", rclcpp::QoS(10),
      std::bind(&GlobalPlannerNode::onStuckPoint, this, std::placeholders::_1));
    const auto period = std::chrono::duration<double>(
      1.0 / get_parameter("planner.rate_hz").as_double());
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {plan();});
  }

private:
  void loadParams()
  {
    planner_params_.blocked_speed_mps = get_parameter("planner.blocked_speed_mps").as_double();
    planner_params_.reached_clear_radius_m =
      get_parameter("planner.reached_clear_radius_m").as_double();
    planner_params_.waypoint_pass_radius_m =
      get_parameter("planner.waypoint_pass_radius_m").as_double();
    planner_params_.waypoint_clearance_m = get_parameter("planner.waypoint_clearance_m").as_double();
    planner_params_.any_angle_shortcut = get_parameter("planner.any_angle_shortcut").as_bool();
    planner_params_.search = parseSearchMethod(get_parameter("planner.search").as_string());
    planner_params_.hybrid.step_m = get_parameter("planner.hybrid.step_m").as_double();
    planner_params_.hybrid.min_radius_m = get_parameter("planner.hybrid.min_radius_m").as_double();
    planner_params_.hybrid.curvature_weight = get_parameter("planner.hybrid.curvature_weight").as_double();
    planner_params_.hybrid.curvature_change_weight =
      get_parameter("planner.hybrid.curvature_change_weight").as_double();
    planner_params_.hybrid.max_expansions =
      static_cast<int>(get_parameter("planner.hybrid.max_expansions").as_int());
    planner_params_.stuck_block_radius_m = get_parameter("planner.stuck_block.radius_m").as_double();
    planner_params_.stuck_block_duration_s =
      get_parameter("planner.stuck_block.duration_s").as_double();
    goal_params_.enabled = get_parameter("goal_handling.enabled").as_bool();
    goal_params_.occupied_radius_m = get_parameter("goal_handling.occupied_radius_m").as_double();
    goal_params_.wait_s = get_parameter("goal_handling.wait_s").as_double();
    goal_params_.alternative_radius_m =
      get_parameter("goal_handling.alternative_radius_m").as_double();
    goal_params_.fail_when_occupied = get_parameter("goal_handling.fail_when_occupied").as_bool();
    goal_params_.fail_when_blocked = get_parameter("goal_handling.fail_when_blocked").as_bool();
    goal_params_.blocked_wait_s = get_parameter("goal_handling.blocked_wait_s").as_double();
    smoothing_params_.enabled = get_parameter("planner.smoothing.enabled").as_bool();
    smoothing_params_.mode = parseSmoothingMode(get_parameter("planner.smoothing.mode").as_string());
    smoothing_params_.bending_length_m =
      get_parameter("planner.smoothing.bending_length_m").as_double();
    smoothing_params_.spacing_m = get_parameter("planner.smoothing.spacing_m").as_double();
    smoothing_params_.extra_clearance_m =
      get_parameter("planner.smoothing.extra_clearance_m").as_double();
    smoothing_params_.iterations =
      static_cast<int>(get_parameter("planner.smoothing.iterations").as_int());
    smoothing_params_.smooth_weight = get_parameter("planner.smoothing.smooth_weight").as_double();
    smoothing_params_.clearance_weight =
      get_parameter("planner.smoothing.clearance_weight").as_double();
  }

  // 機体の外接円の半径に余裕を足した値。狭い通路を通すときは、機体の半幅に通路の片側の余裕の
  // しきい値を足した値のほうが小さければ、そちらにする（planningInflationRadius）
  double inflationRadius() const
  {
    return inflationRadius(get_parameter("narrow_passage.enabled").as_bool());
  }

  double inflationRadius(const bool narrow_enabled) const
  {
    return planningInflationRadius(
      0.5 * get_parameter("footprint.length_m").as_double(),
      0.5 * get_parameter("footprint.width_m").as_double(),
      get_parameter("footprint.margin_m").as_double(), narrow_enabled,
      get_parameter("narrow_passage.min_side_clearance_m").as_double());
  }

  // 曲線化の設定。取りたい余裕は、狭い通路を使わないときの半径（外接円 + 余裕）に extra_clearance_m を
  // 足した距離のままにする。狭い通路で膨らませる半径を小さくしても、広い所の経路は静止物へ寄らない
  // （守る距離だけが小さくなり、通路の中では両側から押されて真ん中に来る）
  SmoothingParams smoothingParams() const
  {
    SmoothingParams params = smoothing_params_;
    params.extra_clearance_m += std::max(inflationRadius(false) - inflated_radius_, 0.0);
    return params;
  }

  void buildGrid()
  {
    core_nav_common::FieldBounds field;
    field.x_min = get_parameter("field.x_min").as_double();
    field.x_max = get_parameter("field.x_max").as_double();
    field.y_min = get_parameter("field.y_min").as_double();
    field.y_max = get_parameter("field.y_max").as_double();
    OccupancyGrid grid = makeFieldGrid(field, get_parameter("planner.resolution_m").as_double());
    const StaticMapOptions static_map = staticMapOptions(*this);
    const std::string map_path = get_parameter("map_path").as_string();
    if (map_path.empty()) {
      RCLCPP_WARN(get_logger(), "地図がないので、柵だけで計画します");
    } else {
      addStaticObstacles(
        grid, map_path, field, kFieldInsetM, static_map.min_height_m, static_map.max_height_m);
    }
    // 入らない範囲（進入禁止ゾーンなど）は、地図がなくても使う
    addPolygons(grid, static_map.keep_out);
    distance_field_ = computeDistanceField(grid);
    inflated_radius_ = inflationRadius();
    inflated_ = inflate(distance_field_, inflated_radius_);
    // 止まっている相手を足した距離場を作るときに使う
    static_grid_ = std::move(grid);
  }

  // 静止物の周りで折れ線を曲線にする。曲線にできなければ折れ線を返す。
  // obstacles は計画の格子に足した止まっている相手。固定物だけで計画したときは空にする
  std::vector<std::pair<double, double>> smoothedPoints(
    const std::vector<std::pair<double, double>> & points,
    const std::vector<SlowObstacle> & obstacles)
  {
    // 相手がいれば、膨らませる前の格子に相手の円（相手の半径だけ）を足して距離場を作る。
    // 計画の格子（相手の円を膨らみの半径だけ広げたもの）で空いているセルは、この距離場でも
    // 膨らみの外になる。逆は円を格子に並べた分（1 セル未満）だけ緩い
    std::optional<DistanceField> obstacle_field;
    if (smoothing_params_.enabled && !obstacles.empty()) {
      obstacle_field = computeDistanceField(withObstacles(static_grid_, obstacles, 0.0));
    }
    const DistanceField & field = obstacle_field.has_value() ? *obstacle_field : distance_field_;
    try {
      const auto smoothed = smoothPath(points, field, inflated_radius_, smoothingParams());
      if (smoothed.has_value()) {
        return *smoothed;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "経路が途中で膨らみに入っているので、曲線にせず折れ線を使います");
    } catch (const std::invalid_argument & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "経路を曲線にできないので折れ線を使います: %s",
        error.what());
    }
    return points;
  }

  std::optional<PlanarPose> lookupPose(const std::string & target, const std::string & source)
  {
    try {
      return toPlanarPose(tf_buffer_.lookupTransform(target, source, tf2::TimePointZero));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "%s から %s への変換を取得できません: %s",
        target.c_str(), source.c_str(), error.what());
      return std::nullopt;
    }
  }

  // 止まっている相手と、局所計画が行き詰まった所（期限内のもの）
  std::vector<SlowObstacle> currentSlowObstacles()
  {
    std::vector<SlowObstacle> obstacles = trackedSlowObstacles();
    const double time_s = now().seconds();
    stuck_blocks_.erase(
      std::remove_if(
        stuck_blocks_.begin(), stuck_blocks_.end(),
        [time_s](const StuckBlock & block) {return block.expires_s <= time_s;}),
      stuck_blocks_.end());
    for (const auto & block : stuck_blocks_) {
      obstacles.push_back(SlowObstacle{block.x, block.y, planner_params_.stuck_block_radius_m});
    }
    return obstacles;
  }

  std::vector<SlowObstacle> trackedSlowObstacles()
  {
    if (!latest_tracks_ || !last_tracks_time_.has_value() ||
      (now() - *last_tracks_time_).seconds() > kTracksMaxAgeS)
    {
      return {};
    }
    const auto map_to_odom = lookupPose(
      get_parameter("map_frame").as_string(), get_parameter("odom_frame").as_string());
    if (!map_to_odom.has_value()) {
      return {};
    }
    return slowObstaclesInMap(*latest_tracks_, *map_to_odom, planner_params_.blocked_speed_mps);
  }

  // 局所計画が行き詰まった所を、しばらく通れない所にして計画し直す。ゴールの近くの点は足さない
  // （ゴールが塞がれたとみなされ、経路が空になる。ゴールの近くは局所計画が着いたことにする）
  void onStuckPoint(const geometry_msgs::msg::PointStamped::ConstSharedPtr & point)
  {
    if (!goal_.has_value() || planner_params_.stuck_block_duration_s <= 0.0) {
      return;
    }
    const std::string map_frame = get_parameter("map_frame").as_string();
    PlanarPose position{point->point.x, point->point.y, 0.0};
    if (!point->header.frame_id.empty() && point->header.frame_id != map_frame) {
      const auto frame_in_map = lookupPose(map_frame, point->header.frame_id);
      if (!frame_in_map.has_value()) {
        RCLCPP_ERROR(
          get_logger(), "行き詰まった所の座標系 %s を %s に直せないので、使いません",
          point->header.frame_id.c_str(), map_frame.c_str());
        return;
      }
      position = toMapFrame(*frame_in_map, position);
    }
    const double to_goal = std::hypot(position.x - goal_->x, position.y - goal_->y);
    if (to_goal < goal_params_.occupied_radius_m + planner_params_.stuck_block_radius_m) {
      RCLCPP_WARN(
        get_logger(), "行き詰まった所 (%.2f, %.2f) はゴールに近い（%.2f m）ので、塞がずに計画し直します",
        position.x, position.y, to_goal);
    } else {
      stuck_blocks_.push_back(
        StuckBlock{position.x, position.y, now().seconds() + planner_params_.stuck_block_duration_s});
      RCLCPP_WARN(
        get_logger(), "局所計画が (%.2f, %.2f) で行き詰まったので、%.0f 秒の間そこを避けて計画し直します",
        position.x, position.y, planner_params_.stuck_block_duration_s);
    }
    plan();
  }

  void publishStatus(const std::uint8_t state, const double planning_ms, const double distance)
  {
    NavigationStatus status;
    status.header.stamp = now();
    status.header.frame_id = get_parameter("map_frame").as_string();
    status.state = state;
    status.planning_ms = static_cast<float>(planning_ms);
    status.distance_to_goal_m = static_cast<float>(distance);
    status_publisher_->publish(status);
  }

  void publishEmptyPath()
  {
    Path path;
    path.header.stamp = now();
    path.header.frame_id = get_parameter("map_frame").as_string();
    path_publisher_->publish(path);
    current_cells_.clear();
  }

  // frame_id の座標系の map での姿勢。map か空なら原点。直せなければ nullopt（ログを出す）
  std::optional<PlanarPose> frameInMap(const std::string & frame_id, const char * what)
  {
    const std::string map_frame = get_parameter("map_frame").as_string();
    if (frame_id.empty() || frame_id == map_frame) {
      return PlanarPose{};
    }
    // map 以外の座標系（odom など）は、今の TF で map に直す
    const auto frame_in_map = lookupPose(map_frame, frame_id);
    if (!frame_in_map.has_value()) {
      RCLCPP_ERROR(
        get_logger(), "%sの座標系 %s を %s に直せないので、%sを捨てます", what, frame_id.c_str(),
        map_frame.c_str(), what);
    }
    return frame_in_map;
  }

  static PlanarPose planarPose(const geometry_msgs::msg::Pose & pose)
  {
    const auto & q = pose.orientation;
    return PlanarPose{
      pose.position.x, pose.position.y,
      std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
  }

  // 新しいゴールを受けたときに、前のゴールの待ちや迂回の状態を消す
  void resetGoalState()
  {
    goal_wait_start_.reset();
    alternative_.reset();
    goal_free_since_.reset();
    stuck_blocks_.clear();
    blocked_since_.reset();
  }

  void onGoal(const PoseStamped::ConstSharedPtr & goal)
  {
    const auto frame_in_map = frameInMap(goal->header.frame_id, "ゴール");
    if (!frame_in_map.has_value()) {
      return;
    }
    resetGoalState();
    setWaypoints({});
    if (!std::isfinite(goal->pose.position.x) || !std::isfinite(goal->pose.position.y)) {
      goal_.reset();
      publishEmptyPath();
      publishStatus(NavigationStatus::STATE_IDLE, 0.0, 0.0);
      return;
    }
    goal_ = toMapFrame(*frame_in_map, planarPose(goal->pose));
    goal_received_time_ = now();
    plan();
  }

  // 通過点を順に並べ、最後をゴールにした経路
  void onRoute(const Path::ConstSharedPtr & route)
  {
    if (route->poses.empty()) {
      RCLCPP_WARN(get_logger(), "点のない経路を受けたので捨てます");
      return;
    }
    for (const auto & pose : route->poses) {
      if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y)) {
        RCLCPP_WARN(get_logger(), "位置が有限でない点を含む経路を受けたので捨てます");
        return;
      }
    }
    const auto frame_in_map = frameInMap(route->header.frame_id, "経路");
    if (!frame_in_map.has_value()) {
      return;
    }
    resetGoalState();
    std::vector<Waypoint> waypoints;
    waypoints.reserve(route->poses.size() - 1U);
    for (std::size_t i = 0U; i + 1U < route->poses.size(); ++i) {
      const PlanarPose point = toMapFrame(*frame_in_map, planarPose(route->poses[i].pose));
      waypoints.push_back(reachableWaypoint({point.x, point.y}, i + 1U));
    }
    goal_ = toMapFrame(*frame_in_map, planarPose(route->poses.back().pose));
    setWaypoints(std::move(waypoints));
    goal_received_time_ = now();
    RCLCPP_INFO(
      get_logger(), "通過点 %zu 個を通ってゴール (%.2f, %.2f) へ向かいます", waypoints_.size(), goal_->x,
      goal_->y);
    plan();
  }

  // 静止物に近すぎる通過点を、余裕（planner.waypoint_clearance_m）のある一番近い所へ移す。見つからなければそのまま
  Waypoint reachableWaypoint(const Waypoint & waypoint, const std::size_t number)
  {
    const auto moved = withClearance(
      waypoint, planner_params_.waypoint_clearance_m, kNearestFreeRadiusM,
      distance_field_.info().resolution,
      [this](const double x, const double y) {return static_cast<double>(distance_field_.at(x, y));});
    if (!moved.has_value()) {
      RCLCPP_WARN(
        get_logger(), "通過点 %zu (%.2f, %.2f) の近くに静止物から %.2f m 離れた所がないので、そのまま使います",
        number, waypoint.x, waypoint.y, planner_params_.waypoint_clearance_m);
      return waypoint;
    }
    if (std::hypot(moved->x - waypoint.x, moved->y - waypoint.y) > 1e-6) {
      RCLCPP_INFO(
        get_logger(), "通過点 %zu を静止物から離しました (%.2f, %.2f) → (%.2f, %.2f)", number, waypoint.x,
        waypoint.y, moved->x, moved->y);
    }
    return *moved;
  }

  void setWaypoints(std::vector<Waypoint> waypoints)
  {
    waypoints_ = std::move(waypoints);
    publishWaypoints();
  }

  void publishWaypoints()
  {
    geometry_msgs::msg::PoseArray message;
    message.header.stamp = now();
    message.header.frame_id = get_parameter("map_frame").as_string();
    for (const auto & waypoint : waypoints_) {
      message.poses.push_back(makePose(waypoint.x, waypoint.y, 0.0));
    }
    waypoints_publisher_->publish(message);
  }

  void onTracks(const TrackedObjectArray::ConstSharedPtr & tracks)
  {
    latest_tracks_ = tracks;
    last_tracks_time_ = now();
    if (!goal_.has_value() || current_cells_.size() < 2U) {
      return;
    }
    if (last_plan_time_.has_value() &&
      (now() - *last_plan_time_).seconds() < kMinReplanIntervalS)
    {
      return;
    }
    const auto obstacles = currentSlowObstacles();
    if (obstacles.empty()) {
      return;
    }
    const auto grid = withObstacles(inflated_, obstacles, inflated_radius_);
    for (std::size_t i = 1U; i < current_cells_.size(); ++i) {
      if (!hasLineOfSight(grid, current_cells_[i - 1U], current_cells_[i])) {
        plan();
        return;
      }
    }
  }

  void onNavigationStatus(const NavigationStatus::ConstSharedPtr & status)
  {
    if (status->state == NavigationStatus::STATE_FAILED && goal_.has_value() &&
      (now() - goal_received_time_).seconds() > kStaleStatusIgnoreS)
    {
      // 局所計画が、計画し直しても経路をふさがれて進めなかった。ゴールを捨てて止まる
      RCLCPP_ERROR(
        get_logger(), "局所計画が行き詰まりから復帰できなかったので、ゴール (%.2f, %.2f) をやめます",
        goal_->x, goal_->y);
      goal_.reset();
      publishStatus(NavigationStatus::STATE_FAILED, 0.0, 0.0);
      return;
    }
    if (status->state != NavigationStatus::STATE_REACHED || !goal_.has_value() ||
      (now() - goal_received_time_).seconds() <= kStaleStatusIgnoreS)
    {
      return;
    }
    // 前のゴールの到達の通知で、新しいゴールを消さないよう、機体の位置でも確かめる
    const auto robot = lookupPose(
      get_parameter("map_frame").as_string(), get_parameter("base_frame").as_string());
    if (robot.has_value() &&
      std::hypot(
        robot->x - alternative_.value_or(*goal_).x, robot->y - alternative_.value_or(*goal_).y) <=
      planner_params_.reached_clear_radius_m)
    {
      goal_.reset();
      publishStatus(NavigationStatus::STATE_REACHED, 0.0, 0.0);
      std_msgs::msg::Bool reached;
      reached.data = true;
      goal_reached_publisher_->publish(reached);
    }
  }

  // 止まっている相手を避ける経路がない。fail_when_blocked で blocked_wait_s 続いたら、ゴールをやめて true を返す
  bool failWhenBlocked(const PlanarPose & goal)
  {
    if (!goal_params_.fail_when_blocked) {
      return false;
    }
    const rclcpp::Time time = now();
    blocked_since_ = blocked_since_.value_or(time);
    if ((time - *blocked_since_).seconds() < goal_params_.blocked_wait_s) {
      return false;
    }
    RCLCPP_ERROR(
      get_logger(), "ゴール (%.2f, %.2f) への経路が %.1f 秒塞がれたままなので、ゴールをやめます", goal.x, goal.y,
      goal_params_.blocked_wait_s);
    blocked_since_.reset();
    goal_.reset();
    return true;
  }

  // 目標の地点を決める。待つときは nullopt
  std::optional<PlanarPose> selectTarget(
    const PlanarPose & goal, const PlanarPose & robot, const std::vector<SlowObstacle> & obstacles,
    const OccupancyGrid & grid)
  {
    const rclcpp::Time time = now();
    if (!goal_params_.enabled) {
      goal_wait_start_.reset();
      alternative_.reset();
      return goal;
    }
    if (!goalIsOccupied(goal, obstacles, goal_params_.occupied_radius_m)) {
      // 相手が一瞬見えなくなっただけで目標を戻すと、行き来して着かなくなる。
      // しばらく空いていることを確かめてから元のゴールへ戻す
      goal_free_since_ = goal_free_since_.value_or(time);
      if (alternative_.has_value() && (time - *goal_free_since_).seconds() < goal_params_.wait_s) {
        return alternative_;
      }
      goal_wait_start_.reset();
      alternative_.reset();
      return goal;
    }
    goal_free_since_.reset();
    if (!goal_wait_start_.has_value()) {
      goal_wait_start_ = time;
    }
    if ((time - *goal_wait_start_).seconds() < goal_params_.wait_s) {
      return std::nullopt;
    }
    if (goal_params_.fail_when_occupied) {
      goal_occupied_failed_ = true;
      return std::nullopt;
    }
    // 決めた代わりのゴールは変えない。機体の位置で選び直すと、目標が動き続けて到着できない
    if (!alternative_.has_value()) {
      alternative_ = findAlternativeGoal(grid, goal, robot, goal_params_.alternative_radius_m);
    }
    return alternative_;
  }

  // stops（機体 → 通過点 → … → 目標）の区間ごとに経路を探す。どこかの区間で見つからなければ nullopt
  std::optional<std::vector<PlannedRoute>> planLegs(
    const OccupancyGrid & grid, const std::vector<PlanarPose> & stops) const
  {
    std::vector<PlannedRoute> legs;
    legs.reserve(stops.size() - 1U);
    for (std::size_t i = 1U; i < stops.size(); ++i) {
      auto leg = planOnGrid(
        grid, stops[i - 1U], stops[i], planner_params_.search, planner_params_.any_angle_shortcut,
        planner_params_.hybrid);
      if (!leg.has_value()) {
        return std::nullopt;
      }
      legs.push_back(std::move(*leg));
    }
    return legs;
  }

  void plan()
  {
    if (!goal_.has_value()) {
      // 到達・失敗などでゴールを消したら、残りの通過点も消す
      if (!waypoints_.empty()) {
        setWaypoints({});
      }
      return;
    }
    const auto start_time = std::chrono::steady_clock::now();
    const auto robot = lookupPose(
      get_parameter("map_frame").as_string(), get_parameter("base_frame").as_string());
    if (!robot.has_value()) {
      return;
    }
    last_plan_time_ = now();
    const PlanarPose goal = *goal_;
    if (const std::size_t passed = dropReachedWaypoints(
        waypoints_, robot->x, robot->y, planner_params_.waypoint_pass_radius_m))
    {
      RCLCPP_INFO(get_logger(), "通過点を %zu 個通過しました（残り %zu 個）", passed, waypoints_.size());
      publishWaypoints();
    }
    const double distance = routeDistance(robot->x, robot->y, waypoints_, goal.x, goal.y);
    const auto elapsed_ms = [&start_time]() {
        return std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start_time).count();
      };

    const auto obstacles = currentSlowObstacles();
    const OccupancyGrid blocked = withObstacles(inflated_, obstacles, inflated_radius_);
    const auto target = selectTarget(goal, *robot, obstacles, blocked);
    if (goal_occupied_failed_) {
      // ゴールの上の障害物がどかない。代わりのゴールへは行かずに止まる
      RCLCPP_ERROR(
        get_logger(), "ゴール (%.2f, %.2f) が %.1f 秒塞がれたままなので、ゴールをやめます", goal.x, goal.y,
        goal_params_.wait_s);
      goal_occupied_failed_ = false;
      goal_.reset();
      publishEmptyPath();
      publishStatus(NavigationStatus::STATE_FAILED, elapsed_ms(), distance);
      return;
    }
    if (!target.has_value()) {
      publishEmptyPath();
      publishStatus(NavigationStatus::STATE_GOAL_WAIT, elapsed_ms(), distance);
      return;
    }

    // 区間の端: 機体、残りの通過点（向きは次の点へ）、目標
    std::vector<PlanarPose> stops{*robot};
    for (std::size_t i = 0U; i < waypoints_.size(); ++i) {
      const double next_x = i + 1U < waypoints_.size() ? waypoints_[i + 1U].x : target->x;
      const double next_y = i + 1U < waypoints_.size() ? waypoints_[i + 1U].y : target->y;
      stops.push_back(
        {waypoints_[i].x, waypoints_[i].y,
          std::atan2(next_y - waypoints_[i].y, next_x - waypoints_[i].x)});
    }
    stops.push_back(*target);

    // 相手を避ける経路がなければ、固定物だけで計画する。塞がれていれば局所計画が止まる
    const OccupancyGrid * used_grid = &blocked;
    auto legs = planLegs(blocked, stops);
    if (legs.has_value()) {
      blocked_since_.reset();
    } else if (failWhenBlocked(goal)) {
      publishEmptyPath();
      publishStatus(NavigationStatus::STATE_FAILED, elapsed_ms(), distance);
      return;
    }
    if (!legs.has_value()) {
      used_grid = &inflated_;
      legs = planLegs(inflated_, stops);
    }
    if (!legs.has_value()) {
      if (waypoints_.empty()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "ゴール (%.2f, %.2f) への経路がありません", goal.x, goal.y);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "通過点 %zu 個を通ってゴール (%.2f, %.2f) へ行く経路がありません",
          waypoints_.size(), goal.x, goal.y);
      }
      publishEmptyPath();
      publishStatus(NavigationStatus::STATE_FAILED, elapsed_ms(), distance);
      return;
    }

    // 区間ごとに折れ線を作って曲線にし、つなぐ（曲線にしても、区間の端の通過点は通る）
    current_cells_.clear();
    std::vector<std::vector<std::pair<double, double>>> segments;
    segments.reserve(legs->size());
    for (std::size_t i = 0U; i < legs->size(); ++i) {
      const PlannedRoute & leg = (*legs)[i];
      for (const auto & cell : leg.cells) {
        if (current_cells_.empty() || !(current_cells_.back() == cell)) {
          current_cells_.push_back(cell);
        }
      }
      // hybrid_astar の点の列は、始点と終点をセルの中心から区間の端の位置に置き換える
      std::vector<std::pair<double, double>> raw_points;
      if (leg.points.has_value()) {
        raw_points = *leg.points;
        raw_points.front() = {stops[i].x, stops[i].y};
        raw_points.back() = {stops[i + 1U].x, stops[i + 1U].y};
      } else {
        raw_points = toPoints(*used_grid, leg.cells, stops[i], stops[i + 1U]);
      }
      // 固定物だけで計画し直した経路は相手を通るので、固定物だけの距離場で曲線にする
      segments.push_back(
        smoothedPoints(
          raw_points, used_grid == &blocked ? obstacles : std::vector<SlowObstacle>{}));
    }
    const auto points = joinSegments(segments);
    path_publisher_->publish(
      makePath(points, target->yaw, get_parameter("map_frame").as_string(), now()));
    publishStatus(NavigationStatus::STATE_DRIVING, elapsed_ms(), distance);
  }

  PlannerParams planner_params_;
  GoalHandlingParams goal_params_;
  SmoothingParams smoothing_params_;
  // 膨らませる前の静止物（地図と柵）の格子
  OccupancyGrid static_grid_;
  DistanceField distance_field_;
  OccupancyGrid inflated_;
  double inflated_radius_{0.0};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::optional<PlanarPose> goal_;
  rclcpp::Time goal_received_time_{0, 0, RCL_ROS_TIME};
  bool goal_occupied_failed_{false};
  std::optional<rclcpp::Time> blocked_since_;
  std::optional<rclcpp::Time> goal_wait_start_;
  std::optional<PlanarPose> alternative_;
  std::optional<rclcpp::Time> goal_free_since_;
  std::optional<rclcpp::Time> last_plan_time_;
  std::optional<rclcpp::Time> last_tracks_time_;
  TrackedObjectArray::ConstSharedPtr latest_tracks_;
  std::vector<GridCell> current_cells_;
  std::vector<StuckBlock> stuck_blocks_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::Publisher<Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<NavigationStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_publisher_;
  rclcpp::Subscription<PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<Path>::SharedPtr route_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_publisher_;
  // 残りの通過点（map）。先頭から順に通る
  std::vector<Waypoint> waypoints_;
  rclcpp::Subscription<TrackedObjectArray>::SharedPtr tracks_subscription_;
  rclcpp::Subscription<NavigationStatus>::SharedPtr navigation_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr stuck_point_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace core_planning

RCLCPP_COMPONENTS_REGISTER_NODE(core_planning::GlobalPlannerNode)
