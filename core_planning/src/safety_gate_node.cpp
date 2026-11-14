// 計画の速度指令（/cmd_vel_nav）を、機体へ渡す前に確かめるノード。
//
// 次の順に判定して /cmd_vel を出す。
// - watchdog: 姿勢、追跡結果、速度指令が途絶えたら止める
// - 接触中なら止める
// - 自己位置が失敗状態なら止める
// - safety_stop: 衝突までの時間で減速または停止する。さらに相手の到達範囲（推定の遅れ、速度の
//   不確かさ、加速度の上限の分だけ広げた範囲）で、指令の後に相手の方向へ動いたまま範囲に入らずに
//   止まれるかを確かめ、止まれなければ範囲に入らない候補の速度に替える。替えるときは、局所計画
//   （mppi_lite）と同じ地図から作った静止物の距離の場で、候補を保ってから止まる軌道が静止物に寄らないかも
//   確かめる。候補のうち機体の前から見えない側へ向かうものは、局所計画と同じ上限（heading.blind_side）
//   までに縮める
// 計画とは独立に動くので、計画の不具合で相手へ突っ込むのも止められる。
//
// /cmd_vel_nav を受けたら、すぐに判定して /cmd_vel を出す（タイマーを待つと、指令が機体に届くのが
// 最大 50ms 遅れる）。指令が来ない間も 50ms ごとに判定し直し、途絶えの監視と相手の動きに対応する。
// 指令を受けて出したときはタイマーを始めからにして、判定の回数を増やさない。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <core_msgs/msg/contact_status.hpp>
#include <core_msgs/msg/localization_health.hpp>
#include <core_msgs/msg/safety_status.hpp>
#include <core_msgs/msg/tracked_object.hpp>
#include <core_msgs/msg/tracked_object_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "core_nav_common/field_bounds.hpp"
#include "core_nav_common/parameters.hpp"
#include "core_planning/grid_map.hpp"
#include "core_planning/safety.hpp"
#include "core_planning/track_motion.hpp"

namespace core_planning
{
namespace
{
using geometry_msgs::msg::Twist;
using core_msgs::msg::ContactStatus;
using core_msgs::msg::LocalizationHealth;
using core_msgs::msg::SafetyStatus;
using core_msgs::msg::TrackedObject;
using core_msgs::msg::TrackedObjectArray;
using nav_msgs::msg::Odometry;

constexpr auto kPeriod = std::chrono::milliseconds(50);
constexpr double kPeriodS = 0.05;
// 接触の状態の配信が止まったときに、止まったままにならないようにする余裕
constexpr double kContactStatusMarginS = 1.0;
// 到達範囲に使う追跡結果の古さの上限。時計がずれて時刻が大きく離れても、範囲が際限なく広がって
// 動けなくならないようにする（1 秒で、加速度 1.5m/s^2 の広がりは 0.75m）
constexpr double kMaxTracksAgeS = 1.0;
// 追跡結果の時刻がこれより古ければ警告する
constexpr double kWarnTracksAgeS = 0.5;
// 追跡結果に出てこなくなった id を、動いているかの判定から忘れるまでの時間 [s]
constexpr double kForgetTrackS = 2.0;

struct WatchdogParams
{
  bool enabled{true};
  double pose_timeout_s{0.3};
  double tracks_timeout_s{0.3};
  double cmd_timeout_s{0.3};
};

struct GateParams
{
  bool safety_enabled{true};
  safety::SafetyParams safety;
  safety::Footprint footprint;
  WatchdogParams watchdog;
  double contact_hold_s{1.0};
};

struct GateDecision
{
  std::uint8_t state{SafetyStatus::STATE_PASS};
  std::string reason;
  double speed_scale{1.0};
  double ttc_s{-1.0};
  // 出す速度（減速度の制限の前）。nullopt なら指令をそのまま使う
  std::optional<safety::BodyVelocity> output;
};

safety::ObstacleKind toKind(const std::uint8_t classification)
{
  if (classification == TrackedObject::CLASS_ROBOT) {
    return safety::ObstacleKind::kRobot;
  }
  if (classification == TrackedObject::CLASS_HUMAN) {
    return safety::ObstacleKind::kHuman;
  }
  return safety::ObstacleKind::kUnknown;
}

// age_s は追跡結果の時刻から今までの時間。到達範囲の確かめで、位置を進め、広がりに足す。
// moving は追跡の id ごとの、動いているとみなすか（TrackMotionFilter）。正面の相手と衝撃の緩和の判定に使う。
// 判定はロボットだけにかけ、人と分類できない相手は速度をそのまま信じる（mppi_lite と同じ）
std::vector<safety::Obstacle> toObstacles(
  const TrackedObjectArray & tracks, const double age_s,
  const std::unordered_map<std::uint32_t, bool> & moving)
{
  std::vector<safety::Obstacle> obstacles;
  obstacles.reserve(tracks.objects.size());
  for (const auto & object : tracks.objects) {
    const auto found = moving.find(object.id);
    const safety::ObstacleKind kind = toKind(object.classification);
    obstacles.push_back(
      safety::Obstacle{
        object.position.x, object.position.y, object.velocity.x, object.velocity.y, kind,
        object.position_sigma, object.velocity_sigma, age_s,
        kind != safety::ObstacleKind::kRobot || (found != moving.end() && found->second)});
  }
  return obstacles;
}

safety::PlanarPose toPose(const Odometry & odometry)
{
  const auto & q = odometry.pose.pose.orientation;
  return safety::PlanarPose{
    odometry.pose.pose.position.x, odometry.pose.pose.position.y,
    std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
}

safety::BodyVelocity toVelocity(const Twist & twist)
{
  return safety::BodyVelocity{twist.linear.x, twist.linear.y, twist.angular.z};
}

GateDecision stopDecision(const std::uint8_t state, const std::string & reason)
{
  return GateDecision{state, reason, 0.0, -1.0, safety::BodyVelocity{}};
}

// 静止物の判定に使う距離の場と map→odom。使えなければ field は nullptr で、unavailable に理由
struct StaticView
{
  const DistanceField * field{nullptr};
  safety::PlanarPose map_from_odom;
  std::string unavailable;
};

std::string replacedReason(const safety::SupervisedCommand & supervised)
{
  std::string reason = supervised.feasible ? "相手の到達範囲に入る前に止まれない" :
    "相手の到達範囲に入る前に止まれない（離れる向きを選んだ）";
  if (supervised.static_checked && !supervised.static_ok) {
    reason += "（静止物に寄らない候補がなく、静止物から最も離れる向きを選んだ）";
  }
  return reason;
}

double speedRatio(const safety::BodyVelocity & output, const safety::BodyVelocity & commanded)
{
  const double commanded_speed = std::hypot(commanded.vx, commanded.vy);
  if (commanded_speed < 1e-6) {
    return 1.0;
  }
  return std::clamp(std::hypot(output.vx, output.vy) / commanded_speed, 0.0, 1.0);
}
}  // namespace

class SafetyGateNode final : public rclcpp::Node
{
public:
  explicit SafetyGateNode(const rclcpp::NodeOptions & options)
  : Node("safety_gate", options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    using core_nav_common::declareBool;
    using core_nav_common::declareDouble;
    using core_nav_common::declareInt;
    using core_nav_common::declareString;
    declareBool(*this, "safety_stop.enabled", true, "衝突までの時間で減速、停止する");
    declareDouble(*this, "safety_stop.ttc_slow_s", 1.5, 0.05, 30.0, "減速を始める衝突までの時間");
    declareDouble(*this, "safety_stop.ttc_stop_s", 0.6, 0.0, 30.0, "停止する衝突までの時間");
    declareDouble(*this, "safety_stop.footprint_margin_m", 0.1, 0.0, 5.0, "機体の余裕");
    declareDouble(*this, "safety_stop.max_decel_mps2", 3.0, 0.1, 20.0, "減速度の上限");
    declareBool(
      *this, "safety_stop.reachable.enabled", true, "相手の到達範囲に入らずに止まれるかを確かめる");
    declareDouble(
      *this, "safety_stop.reachable.obstacle_accel_mps2", 1.5, 0.0, 20.0, "相手の加速度の上限");
    declareDouble(
      *this, "safety_stop.reachable.velocity_sigma_gain", 1.0, 0.0, 10.0,
      "相手の速度の不確かさに掛ける倍率");
    declareDouble(
      *this, "safety_stop.reachable.commit_time_s", 0.15, 0.0, 2.0, "止まり始めるまでに指令を保つ時間");
    declareDouble(
      *this, "safety_stop.reachable.escape_horizon_s", 1.0, 0.05, 10.0, "候補の速度を保って確かめる時間");
    declareDouble(
      *this, "safety_stop.reachable.escape_change_s", 0.5, 0.05, 10.0,
      "候補の速度を今の速度から変える大きさ（max_decel に掛ける時間）");
    declareInt(*this, "safety_stop.reachable.directions", 16, 4, 72, "候補の速度の向きの数");
    declareBool(
      *this, "safety_stop.oncoming.brake_only", true,
      "正面から来るロボットだけが原因で指令を替えるとき、今の向きのまま減速する候補に限る（横へ逃がさない）");
    declareBool(
      *this, "safety_stop.mitigation.enabled", true,
      "高速で当てられそうなとき、少し動いて衝撃を和らげる（止まっていても動く）");
    declareDouble(
      *this, "safety_stop.mitigation.min_impact_speed_mps", 1.5, 0.0, 10.0,
      "緩和する、当たるときの近づく速さの下限 [m/s]");
    declareDouble(
      *this, "safety_stop.mitigation.horizon_s", 1.0, 0.05, 5.0, "当たるかを見る時間 [s]");
    declareDouble(
      *this, "safety_stop.mitigation.max_shift_m", 0.5, 0.0, 5.0, "緩和で動く距離の上限 [m]");
    declareDouble(
      *this, "safety_stop.mitigation.max_speed_mps", 1.5, 0.0, 5.0, "緩和で動く速さの上限 [m/s]");
    declareDouble(
      *this, "safety_stop.mitigation.min_reduction_mps", 0.3, 0.0, 10.0,
      "何もしないときより、当たる速さがこれ以上小さくならなければ動かない [m/s]");
    declareInt(
      *this, "safety_stop.mitigation.directions", 16, 4, 72, "緩和の候補の向きの数");
    // mppi_lite と同じ判定（mppi.yaml の track_motion）。設定ファイルは全ノードで共通
    declareBool(
      *this, "track_motion.enabled", true,
      "速度が落ち着いて見えている追跡だけを動いているとみなす（false で速さだけで決める）");
    declareDouble(
      *this, "track_motion.min_speed_mps", 0.3, 0.0, 5.0, "動いているとみなす速さの下限 [m/s]");
    declareDouble(
      *this, "track_motion.max_velocity_sigma_mps", 0.2, 0.0, 10.0,
      "動いているとみなす、追跡の速度の不確かさの上限 [m/s]");
    declareDouble(
      *this, "track_motion.persist_s", 0.5, 0.0, 10.0, "上の条件が続く時間 [s]");
    declareBool(
      *this, "safety_stop.static.enabled", true,
      "到達範囲のために指令を替えるとき、候補が静止物（地図と柵）に寄らないかを確かめる");
    declareDouble(
      *this, "safety_stop.static.margin_m", 0.1, 0.0, 5.0, "機体と静止物の間に保つ余裕");
    declareDouble(
      *this, "safety_stop.static.tolerance_m", 0.05, 0.0, 1.0,
      "今すでに余裕より近いとき、今の距離から近づくのを許す距離（距離の場の格子の段の分）");
    declareDouble(
      *this, "safety_stop.static.grid_resolution_m", 0.05, 0.01, 1.0,
      "静止物の距離の場の格子の大きさ（mppi.yaml の grid_resolution_m と同じにする）", true);
    declareString(*this, "map_path", "", "地図の PCD。空なら静止物の判定をしない", true);
    declareString(*this, "map_frame", "map", "地図のフレーム", true);
    declareString(*this, "odom_frame", "odom", "安全停止のフレーム", true);
    declareDouble(*this, "field.x_min", -12.0, -1000.0, 1000.0, "フィールドの範囲", true);
    declareDouble(*this, "field.x_max", 12.0, -1000.0, 1000.0, "フィールドの範囲", true);
    declareDouble(*this, "field.y_min", -7.5, -1000.0, 1000.0, "フィールドの範囲", true);
    declareDouble(*this, "field.y_max", 7.5, -1000.0, 1000.0, "フィールドの範囲", true);
    declareBool(*this, "watchdog.enabled", true, "入力が途絶えたら止める");
    declareDouble(*this, "watchdog.pose_timeout_s", 0.3, 0.01, 60.0, "姿勢の途絶を判定する時間");
    declareDouble(*this, "watchdog.tracks_timeout_s", 0.3, 0.01, 60.0, "追跡結果の途絶を判定する時間");
    declareDouble(*this, "watchdog.cmd_timeout_s", 0.3, 0.01, 60.0, "速度指令の途絶を判定する時間");
    declareDouble(*this, "contact_detection.hold_s", 1.0, 0.0, 60.0, "接触で止める時間");
    declareDouble(*this, "footprint.length_m", 0.8, 0.1, 5.0, "機体の前後の長さ");
    declareDouble(*this, "footprint.width_m", 0.8, 0.1, 5.0, "機体の左右の幅");
    // 局所計画（mppi_lite、mppi.yaml）と同じ値を使う。設定ファイルは全ノードで共通
    declareBool(
      *this, "heading.blind_side.enabled", true,
      "退避の候補のうち、見えない側（機体の後ろ）へ向かうものの速さを抑える");
    declareDouble(
      *this, "heading.blind_side.visible_half_angle_deg", 96.0, 0.0, 180.0,
      "機体の前から測った、見える範囲の半角 [deg]");
    declareDouble(
      *this, "heading.blind_side.speed_mps", 0.5, 0.0, 10.0, "見えない側へ向かう候補の速さの上限 [m/s]");
    loadParams();
    parameter_callback_ = add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &) {loadParams();});
    buildStaticField();

    cmd_publisher_ = create_publisher<Twist>("/control/cmd_vel", rclcpp::QoS(10));
    status_publisher_ = create_publisher<SafetyStatus>("/planning/safety/status", rclcpp::QoS(10));
    cmd_subscription_ = create_subscription<Twist>(
      "/planning/cmd_vel_nav", rclcpp::QoS(10),
      [this](const Twist::ConstSharedPtr & cmd) {
        latest_cmd_ = toVelocity(*cmd);
        last_cmd_time_ = now();
        publishDecision(*last_cmd_time_);
        if (timer_) {
          timer_->reset();
        }
      });
    odometry_subscription_ = create_subscription<Odometry>(
      "/localization/lio/odom", rclcpp::QoS(10),
      [this](const Odometry::ConstSharedPtr & odometry) {
        latest_odometry_ = odometry;
        last_pose_time_ = now();
      });
    tracks_subscription_ = create_subscription<TrackedObjectArray>(
      "/perception/tracks", rclcpp::QoS(10),
      [this](const TrackedObjectArray::ConstSharedPtr & tracks) {
        latest_tracks_ = tracks;
        last_tracks_time_ = now();
        // 追跡の速度を信じてよいかを、追跡結果ごとに 1 回決める（mppi_lite と同じ判定）
        const double stamp_s = rclcpp::Time(tracks->header.stamp).seconds();
        moving_tracks_.clear();
        for (const auto & object : tracks->objects) {
          moving_tracks_[object.id] = motion_filter_.update(
            object.id, stamp_s, std::hypot(object.velocity.x, object.velocity.y),
            object.velocity_sigma, object.coasting, motion_filter_params_);
        }
        motion_filter_.prune(stamp_s, kForgetTrackS);
      });
    contact_subscription_ = create_subscription<ContactStatus>(
      "/planning/contact/status", rclcpp::QoS(10),
      std::bind(&SafetyGateNode::onContact, this, std::placeholders::_1));
    health_subscription_ = create_subscription<LocalizationHealth>(
      "/localization/health", rclcpp::QoS(10),
      [this](const LocalizationHealth::ConstSharedPtr & health) {latest_health_ = health;});
    timer_ = rclcpp::create_timer(
      this, get_clock(), kPeriod, std::bind(&SafetyGateNode::onTimer, this));
  }

private:
  void loadParams()
  {
    GateParams params;
    params.safety_enabled = get_parameter("safety_stop.enabled").as_bool();
    params.safety.ttc_slow_s = get_parameter("safety_stop.ttc_slow_s").as_double();
    params.safety.ttc_stop_s = get_parameter("safety_stop.ttc_stop_s").as_double();
    params.safety.footprint_margin_m = get_parameter("safety_stop.footprint_margin_m").as_double();
    params.safety.max_decel_mps2 = get_parameter("safety_stop.max_decel_mps2").as_double();
    params.safety.reachable.enabled = get_parameter("safety_stop.reachable.enabled").as_bool();
    params.safety.reachable.obstacle_accel_mps2 =
      get_parameter("safety_stop.reachable.obstacle_accel_mps2").as_double();
    params.safety.reachable.velocity_sigma_gain =
      get_parameter("safety_stop.reachable.velocity_sigma_gain").as_double();
    params.safety.reachable.commit_time_s =
      get_parameter("safety_stop.reachable.commit_time_s").as_double();
    params.safety.reachable.escape_horizon_s =
      get_parameter("safety_stop.reachable.escape_horizon_s").as_double();
    params.safety.reachable.escape_change_s =
      get_parameter("safety_stop.reachable.escape_change_s").as_double();
    params.safety.reachable.directions =
      static_cast<int>(get_parameter("safety_stop.reachable.directions").as_int());
    params.safety.static_check.enabled = get_parameter("safety_stop.static.enabled").as_bool();
    params.safety.static_check.margin_m = get_parameter("safety_stop.static.margin_m").as_double();
    params.safety.static_check.tolerance_m =
      get_parameter("safety_stop.static.tolerance_m").as_double();
    params.safety.oncoming.brake_only = get_parameter("safety_stop.oncoming.brake_only").as_bool();
    params.safety.mitigation.enabled = get_parameter("safety_stop.mitigation.enabled").as_bool();
    params.safety.mitigation.min_impact_speed_mps =
      get_parameter("safety_stop.mitigation.min_impact_speed_mps").as_double();
    params.safety.mitigation.horizon_s =
      get_parameter("safety_stop.mitigation.horizon_s").as_double();
    params.safety.mitigation.max_shift_m =
      get_parameter("safety_stop.mitigation.max_shift_m").as_double();
    params.safety.mitigation.max_speed_mps =
      get_parameter("safety_stop.mitigation.max_speed_mps").as_double();
    params.safety.mitigation.min_reduction_mps =
      get_parameter("safety_stop.mitigation.min_reduction_mps").as_double();
    params.safety.mitigation.directions =
      static_cast<int>(get_parameter("safety_stop.mitigation.directions").as_int());
    motion_filter_params_.enabled = get_parameter("track_motion.enabled").as_bool();
    motion_filter_params_.min_speed_mps = get_parameter("track_motion.min_speed_mps").as_double();
    motion_filter_params_.max_velocity_sigma_mps =
      get_parameter("track_motion.max_velocity_sigma_mps").as_double();
    motion_filter_params_.persist_s = get_parameter("track_motion.persist_s").as_double();
    params.safety.blind_side.enabled = get_parameter("heading.blind_side.enabled").as_bool();
    params.safety.blind_side.visible_half_angle_deg =
      get_parameter("heading.blind_side.visible_half_angle_deg").as_double();
    params.safety.blind_side.speed_mps = get_parameter("heading.blind_side.speed_mps").as_double();
    params.watchdog.enabled = get_parameter("watchdog.enabled").as_bool();
    params.watchdog.pose_timeout_s = get_parameter("watchdog.pose_timeout_s").as_double();
    params.watchdog.tracks_timeout_s = get_parameter("watchdog.tracks_timeout_s").as_double();
    params.watchdog.cmd_timeout_s = get_parameter("watchdog.cmd_timeout_s").as_double();
    params.contact_hold_s = get_parameter("contact_detection.hold_s").as_double();
    params.footprint.half_length_m = 0.5 * get_parameter("footprint.length_m").as_double();
    params.footprint.half_width_m = 0.5 * get_parameter("footprint.width_m").as_double();
    params_ = params;
  }

  // mppi_lite と同じ地図と範囲で、静止物の距離の場を作る。地図がない、読めないときは静止物の判定をしない
  void buildStaticField()
  {
    const std::string map_path = get_parameter("map_path").as_string();
    if (map_path.empty()) {
      static_field_unavailable_ = "map_path が空";
      RCLCPP_WARN(get_logger(), "map_path が空なので、退避の候補で静止物を確かめません");
      return;
    }
    FieldBounds field;
    field.x_min = get_parameter("field.x_min").as_double();
    field.x_max = get_parameter("field.x_max").as_double();
    field.y_min = get_parameter("field.y_min").as_double();
    field.y_max = get_parameter("field.y_max").as_double();
    const auto start = std::chrono::steady_clock::now();
    try {
      distance_field_ = buildDistanceField(
        field, get_parameter("safety_stop.static.grid_resolution_m").as_double(), map_path);
    } catch (const std::exception & error) {
      static_field_unavailable_ = std::string("距離の場を作れない: ") + error.what();
      RCLCPP_ERROR(
        get_logger(), "地図 %s から静止物の距離の場を作れません（%s）。退避の候補で静止物を確かめません",
        map_path.c_str(), error.what());
      return;
    }
    RCLCPP_INFO(
      get_logger(), "静止物の距離の場 %dx%d（%.3fm）を %.1f ms で作りました",
      distance_field_->info().width, distance_field_->info().height,
      distance_field_->info().resolution,
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
  }

  // 静止物の判定に使う距離の場と、最新の map→odom
  StaticView staticView() const
  {
    if (!params_.safety.static_check.enabled) {
      return StaticView{};
    }
    if (!distance_field_.has_value()) {
      return StaticView{nullptr, safety::PlanarPose{}, static_field_unavailable_};
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        get_parameter("map_frame").as_string(), get_parameter("odom_frame").as_string(),
        tf2::TimePointZero);
      const auto & q = transform.transform.rotation;
      return StaticView{
        &*distance_field_,
        safety::PlanarPose{
          transform.transform.translation.x, transform.transform.translation.y,
          std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))},
        ""};
    } catch (const tf2::TransformException & error) {
      return StaticView{
        nullptr, safety::PlanarPose{},
        std::string("map から odom への変換を取得できない: ") + error.what()};
    }
  }

  void onContact(const ContactStatus::ConstSharedPtr & status)
  {
    const rclcpp::Time time = now();
    // 接触の検知が短くても、hold_s の間は止める
    if (status->in_contact && !(latest_contact_ && latest_contact_->in_contact)) {
      contact_stop_until_ = time + rclcpp::Duration::from_seconds(params_.contact_hold_s);
    }
    latest_contact_ = status;
    last_contact_time_ = time;
  }

  bool isStale(
    const std::optional<rclcpp::Time> & last, const double timeout_s, const rclcpp::Time & time) const
  {
    return !last.has_value() || (time - *last).seconds() > timeout_s;
  }

  // 途絶えている入力の名前を返す。すべて届いていれば nullopt
  std::optional<std::string> staleInput(const rclcpp::Time & time) const
  {
    if (isStale(last_pose_time_, params_.watchdog.pose_timeout_s, time)) {
      return "姿勢が途絶えた";
    }
    if (isStale(last_tracks_time_, params_.watchdog.tracks_timeout_s, time)) {
      return "追跡結果が途絶えた";
    }
    if (isStale(last_cmd_time_, params_.watchdog.cmd_timeout_s, time)) {
      return "速度指令が途絶えた";
    }
    return std::nullopt;
  }

  bool inContact(const rclcpp::Time & time) const
  {
    if (contact_stop_until_.has_value() && time < *contact_stop_until_) {
      return true;
    }
    return latest_contact_ && latest_contact_->in_contact && last_contact_time_.has_value() &&
           (time - *last_contact_time_).seconds() <= params_.contact_hold_s + kContactStatusMarginS;
  }

  GateDecision decide(const rclcpp::Time & time)
  {
    if (params_.watchdog.enabled) {
      const auto stale = staleInput(time);
      if (stale.has_value()) {
        return stopDecision(SafetyStatus::STATE_WATCHDOG_STOP, *stale);
      }
    }
    if (inContact(time)) {
      return stopDecision(SafetyStatus::STATE_CONTACT_STOP, "接触を検知した");
    }
    if (latest_health_ && latest_health_->level == LocalizationHealth::LEVEL_FAILED) {
      return stopDecision(
        SafetyStatus::STATE_LOCALIZATION_STOP, "自己位置が失敗状態: " + latest_health_->reason);
    }
    if (!params_.safety_enabled || !latest_odometry_ || !latest_tracks_) {
      return GateDecision{};
    }
    const double tracks_age_s = tracksAge(time);
    const StaticView view = staticView();
    if (!view.unavailable.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "静止物の判定をしていません（%s）。相手の到達範囲のために指令を替えるとき、静止物を見ずに候補を選びます",
        view.unavailable.c_str());
    }
    const auto obstacles = toObstacles(*latest_tracks_, tracks_age_s, moving_tracks_);
    const auto pose = toPose(*latest_odometry_);
    const auto own_velocity = toVelocity(latest_odometry_->twist.twist);
    const auto supervised = safety::superviseCommand(
      obstacles, pose, own_velocity, latest_cmd_, params_.footprint, params_.safety, view.field,
      view.map_from_odom);
    const auto & result = supervised.nominal;
    // 高速で当てられそうなら、少し動いて衝撃を和らげる（止まっていても動く）
    const bool was_mitigating = mitigator_.active();
    const auto mitigation = mitigator_.update(
      time.seconds(), obstacles, pose, own_velocity, params_.footprint, params_.safety, view.field,
      view.map_from_odom);
    if (mitigator_.active() != was_mitigating) {
      RCLCPP_INFO(
        get_logger(), "%s（当たる速さ 何もしない %.2f m/s → %.2f m/s、自機 %.2f, %.2f）",
        mitigator_.active() ? "高速で当てられそうなので、衝撃を和らげる向きへ動きます" :
        "衝撃を和らげる動きをやめます", mitigator_.baselineImpact(), mitigator_.chosenImpact(),
        pose.x, pose.y);
    }
    if (mitigation.has_value()) {
      return GateDecision{
        SafetyStatus::STATE_STOP, "高速で当てられそうなので、衝撃を和らげる向きへ動く",
        speedRatio(*mitigation, latest_cmd_), result.ttc_s, *mitigation};
    }
    if (supervised.replaced) {
      return GateDecision{
        SafetyStatus::STATE_STOP, replacedReason(supervised),
        speedRatio(supervised.output, latest_cmd_), result.ttc_s, supervised.output};
    }
    switch (result.decision) {
      case safety::Decision::kStop:
        return GateDecision{
          SafetyStatus::STATE_STOP, "衝突までの時間が短い", result.speed_scale, result.ttc_s,
          supervised.output};
      case safety::Decision::kSlow:
        return GateDecision{
          SafetyStatus::STATE_SLOW, "衝突の可能性がある", result.speed_scale, result.ttc_s,
          supervised.output};
      case safety::Decision::kPass:
        break;
    }
    return GateDecision{SafetyStatus::STATE_PASS, "", 1.0, result.ttc_s, supervised.output};
  }

  // 追跡結果の時刻から今までの時間。負なら 0、上限を超えれば警告して上限にする
  double tracksAge(const rclcpp::Time & time) const
  {
    const double age =
      (time - rclcpp::Time(latest_tracks_->header.stamp, time.get_clock_type())).seconds();
    if (age > kWarnTracksAgeS) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "追跡結果の時刻が %.2f 秒前です（時計のずれか処理の遅れ）。到達範囲には最大 %.1f 秒として使います",
        age, kMaxTracksAgeS);
    }
    return std::clamp(age, 0.0, kMaxTracksAgeS);
  }

  void warnIfDisabled()
  {
    if (!params_.safety_enabled) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "safety_stop が無効です。衝突を止められません");
    }
    if (!params_.watchdog.enabled) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "watchdog が無効です。入力が途絶えても止まりません");
    }
  }

  void onTimer() {publishDecision(now());}

  // 判定して /cmd_vel と /safety/status を出す。指令を受けたときと、タイマーで呼ぶ
  void publishDecision(const rclcpp::Time & time)
  {
    warnIfDisabled();
    // 指令とタイマーが同じ時刻に重なったときは、前回からの時間を 0 として減速度の制限を二重に緩めない
    const double dt =
      last_tick_.has_value() ? std::max((time - *last_tick_).seconds(), 0.0) : kPeriodS;
    last_tick_ = time;

    const GateDecision decision = decide(time);
    const safety::BodyVelocity desired = decision.output.value_or(latest_cmd_);
    // 止めるときは常に、安全停止が有効なら通常時も、減速度を守る
    const bool limit = params_.safety_enabled || decision.state != SafetyStatus::STATE_PASS;
    const safety::BodyVelocity output = limit ?
      safety::limitDeceleration(last_output_, desired, params_.safety.max_decel_mps2, dt) :
      desired;
    last_output_ = output;

    Twist cmd;
    cmd.linear.x = output.vx;
    cmd.linear.y = output.vy;
    cmd.angular.z = output.wz;
    cmd_publisher_->publish(cmd);

    SafetyStatus status;
    status.header.stamp = time;
    status.state = decision.state;
    status.reason = decision.reason;
    status.ttc_s = static_cast<float>(decision.ttc_s);
    status.speed_scale = static_cast<float>(decision.speed_scale);
    status_publisher_->publish(status);
  }

  GateParams params_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::optional<DistanceField> distance_field_;
  // 距離の場がない理由
  std::string static_field_unavailable_;
  safety::BodyVelocity latest_cmd_;
  safety::BodyVelocity last_output_;
  safety::ImpactMitigator mitigator_;
  TrackMotionFilter motion_filter_;
  MotionFilterParams motion_filter_params_;
  // 最新の追跡結果の、id ごとの動いているとみなすか
  std::unordered_map<std::uint32_t, bool> moving_tracks_;
  Odometry::ConstSharedPtr latest_odometry_;
  TrackedObjectArray::ConstSharedPtr latest_tracks_;
  ContactStatus::ConstSharedPtr latest_contact_;
  LocalizationHealth::ConstSharedPtr latest_health_;
  std::optional<rclcpp::Time> last_cmd_time_;
  std::optional<rclcpp::Time> last_pose_time_;
  std::optional<rclcpp::Time> last_tracks_time_;
  std::optional<rclcpp::Time> last_contact_time_;
  std::optional<rclcpp::Time> contact_stop_until_;
  std::optional<rclcpp::Time> last_tick_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr parameter_callback_;
  rclcpp::Publisher<Twist>::SharedPtr cmd_publisher_;
  rclcpp::Publisher<SafetyStatus>::SharedPtr status_publisher_;
  rclcpp::Subscription<Twist>::SharedPtr cmd_subscription_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<TrackedObjectArray>::SharedPtr tracks_subscription_;
  rclcpp::Subscription<ContactStatus>::SharedPtr contact_subscription_;
  rclcpp::Subscription<LocalizationHealth>::SharedPtr health_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace core_planning

RCLCPP_COMPONENTS_REGISTER_NODE(core_planning::SafetyGateNode)
