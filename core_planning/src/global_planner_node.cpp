// フィールドの占有格子で JPS を解いてゴールまでの経路を出すノード。
//
// 一定周期のほか、止まっている相手で経路が塞がれたときにも計画し直す。
// 柵はセンサで近距離を見られないので、地図と同じ扱いで占有にしてある。
// x が NaN のゴールを受け取ると、ゴールを解除して空のパスを出す。

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
#include "core_planning/jps.hpp"
#include "core_planning/path_smoother.hpp"

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
  // ジャンプポイントの間のセルにも直線で飛ぶ（shortcutPath）。false でジャンプポイントの間だけ（simplifyPath）
  bool any_angle_shortcut{true};
};

struct GoalHandlingParams
{
  bool enabled{true};
  double occupied_radius_m{0.8};
  double wait_s{3.0};
  double alternative_radius_m{2.0};
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

// 格子で経路を探す。始点と終点が占有の中なら、近くの空きセルから探す。any_angle_shortcut なら、
// ジャンプポイントの間のセルにも見通しで飛んで短くする（shortcutPath。false なら simplifyPath）
std::optional<std::vector<GridCell>> planOnGrid(
  const OccupancyGrid & grid, const PlanarPose & robot, const PlanarPose & target,
  const bool any_angle_shortcut)
{
  const GridInfo & info = grid.info();
  const int radius_cells = static_cast<int>(std::ceil(kNearestFreeRadiusM / info.resolution));
  const auto start = nearestFreeCell(grid, info.cellAt(robot.x, robot.y), radius_cells);
  const auto goal = nearestFreeCell(grid, info.cellAt(target.x, target.y), radius_cells);
  if (!start.has_value() || !goal.has_value()) {
    return std::nullopt;
  }
  const auto path = findPath(grid, *start, *goal);
  if (!path.has_value()) {
    return std::nullopt;
  }
  return any_angle_shortcut ? shortcutPath(grid, *path) : simplifyPath(grid, *path);
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
    declareBool(
      *this, "planner.any_angle_shortcut", true,
      "ジャンプポイントの間のセルにも直線で飛び、直線で行ける所の折れを除く");
    declareBool(*this, "planner.smoothing.enabled", true, "静止物の周りで経路を曲線にする");
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
    tracks_subscription_ = create_subscription<TrackedObjectArray>(
      "/perception/tracks", rclcpp::QoS(10),
      std::bind(&GlobalPlannerNode::onTracks, this, std::placeholders::_1));
    navigation_subscription_ = create_subscription<NavigationStatus>(
      "/planning/navigation/status", rclcpp::QoS(10),
      std::bind(&GlobalPlannerNode::onNavigationStatus, this, std::placeholders::_1));
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
    planner_params_.any_angle_shortcut = get_parameter("planner.any_angle_shortcut").as_bool();
    goal_params_.enabled = get_parameter("goal_handling.enabled").as_bool();
    goal_params_.occupied_radius_m = get_parameter("goal_handling.occupied_radius_m").as_double();
    goal_params_.wait_s = get_parameter("goal_handling.wait_s").as_double();
    goal_params_.alternative_radius_m =
      get_parameter("goal_handling.alternative_radius_m").as_double();
    smoothing_params_.enabled = get_parameter("planner.smoothing.enabled").as_bool();
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

  std::vector<SlowObstacle> currentSlowObstacles()
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

  void onGoal(const PoseStamped::ConstSharedPtr & goal)
  {
    const std::string map_frame = get_parameter("map_frame").as_string();
    // map 以外の座標系（odom など）のゴールは、今の TF で map に直す
    std::optional<PlanarPose> frame_in_map;
    if (!goal->header.frame_id.empty() && goal->header.frame_id != map_frame) {
      frame_in_map = lookupPose(map_frame, goal->header.frame_id);
      if (!frame_in_map.has_value()) {
        RCLCPP_ERROR(
          get_logger(), "ゴールの座標系 %s を %s に直せないので、ゴールを捨てます",
          goal->header.frame_id.c_str(), map_frame.c_str());
        return;
      }
    }
    goal_wait_start_.reset();
    alternative_.reset();
    goal_free_since_.reset();
    if (!std::isfinite(goal->pose.position.x) || !std::isfinite(goal->pose.position.y)) {
      goal_.reset();
      publishEmptyPath();
      publishStatus(NavigationStatus::STATE_IDLE, 0.0, 0.0);
      return;
    }
    const auto & q = goal->pose.orientation;
    const PlanarPose pose{
      goal->pose.position.x, goal->pose.position.y,
      std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
    goal_ = frame_in_map.has_value() ? toMapFrame(*frame_in_map, pose) : pose;
    plan();
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
    if (status->state != NavigationStatus::STATE_REACHED || !goal_.has_value()) {
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
    // 決めた代わりのゴールは変えない。機体の位置で選び直すと、目標が動き続けて到着できない
    if (!alternative_.has_value()) {
      alternative_ = findAlternativeGoal(grid, goal, robot, goal_params_.alternative_radius_m);
    }
    return alternative_;
  }

  void plan()
  {
    if (!goal_.has_value()) {
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
    const double distance = std::hypot(robot->x - goal.x, robot->y - goal.y);
    const auto elapsed_ms = [&start_time]() {
        return std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start_time).count();
      };

    const auto obstacles = currentSlowObstacles();
    const OccupancyGrid blocked = withObstacles(inflated_, obstacles, inflated_radius_);
    const auto target = selectTarget(goal, *robot, obstacles, blocked);
    if (!target.has_value()) {
      publishEmptyPath();
      publishStatus(NavigationStatus::STATE_GOAL_WAIT, elapsed_ms(), distance);
      return;
    }

    // 相手を避ける経路がなければ、固定物だけで計画する。塞がれていれば局所計画が止まる
    const OccupancyGrid * used_grid = &blocked;
    auto cells = planOnGrid(blocked, *robot, *target, planner_params_.any_angle_shortcut);
    if (!cells.has_value()) {
      used_grid = &inflated_;
      cells = planOnGrid(inflated_, *robot, *target, planner_params_.any_angle_shortcut);
    }
    if (!cells.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "ゴール (%.2f, %.2f) への経路がありません", goal.x, goal.y);
      publishEmptyPath();
      publishStatus(NavigationStatus::STATE_FAILED, elapsed_ms(), distance);
      return;
    }

    current_cells_ = *cells;
    // 固定物だけで計画し直した経路は相手を通るので、固定物だけの距離場で曲線にする
    const auto points = smoothedPoints(
      toPoints(*used_grid, *cells, *robot, *target),
      used_grid == &blocked ? obstacles : std::vector<SlowObstacle>{});
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
  std::optional<rclcpp::Time> goal_wait_start_;
  std::optional<PlanarPose> alternative_;
  std::optional<rclcpp::Time> goal_free_since_;
  std::optional<rclcpp::Time> last_plan_time_;
  std::optional<rclcpp::Time> last_tracks_time_;
  TrackedObjectArray::ConstSharedPtr latest_tracks_;
  std::vector<GridCell> current_cells_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::Publisher<Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<NavigationStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_publisher_;
  rclcpp::Subscription<PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Subscription<TrackedObjectArray>::SharedPtr tracks_subscription_;
  rclcpp::Subscription<NavigationStatus>::SharedPtr navigation_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace core_planning

RCLCPP_COMPONENTS_REGISTER_NODE(core_planning::GlobalPlannerNode)
