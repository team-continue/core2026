// 局所計画 MPPI-lite のノード。
//
// /lio/odom を受けるたびに（10Hz）速度指令を計算して /cmd_vel_nav に出す。検出も同じ /lio/odom を
// 受けてから追跡結果を出すので、同じスキャンの追跡結果が届くまで短い上限時間（mppi.tracks_wait_s）待つ
// （待たないと、いつも 1 つ前のスキャンの追跡結果を使う）。
// 計画は odom 座標系で行い、/plan と柵の距離の場は map→odom で対応させる。
// map→odom は飛ぶことがあるので、相手の追跡と機体の動きは odom のまま扱う。
// mppi.drive.predict_delay なら、機体へ送られた指令（安全停止の後の /cmd_vel）を記録し、姿勢の時刻から、
// 今送る指令が効き始める時刻（今 + 計算の時間 + 駆動の無駄時間）まで、送った指令で状態を進めてから計画する
// （drive_delay.hpp）。false なら前の動き（mppi.latency_compensation_s の間、今の速度で位置を進める）。
// traction.enabled なら、送った指令と測った速度から床の滑り（床が出せる加速度）を traction.update_period_s
// ごとに推定し、計画の加減速度と曲がりの上限を変える（traction_estimator.hpp）。推定は /navigation/traction
// （geometry_msgs/Vector3Stamped）に出す。x: 使っている床の静止摩擦 [m/s2]（滑らない床なら無限大）、y: 今回の
// 推定（滑りが見えれば静止摩擦、見えなければ滑らずに出せた下限。推定しなければ NaN）、z: 使っている加速度の
// 上限 [m/s2]。

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <core_msgs/msg/localization_health.hpp>
#include <core_msgs/msg/navigation_status.hpp>
#include <core_msgs/msg/tracked_object.hpp>
#include <core_msgs/msg/tracked_object_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "core_nav_common/field_bounds.hpp"
#include "core_nav_common/parameter_updates.hpp"
#include "core_nav_common/parameters.hpp"
#include "core_planning/chassis_tracker.hpp"
#include "core_planning/drive_delay.hpp"
#include "core_planning/grid_map.hpp"
#include "core_planning/static_map_params.hpp"
#include "core_planning/mppi.hpp"
#include "core_planning/traction_estimator.hpp"
#include "core_planning/tracks_sync.hpp"

namespace core_planning
{
namespace
{
using core_msgs::msg::LocalizationHealth;
using core_msgs::msg::NavigationStatus;
using core_msgs::msg::TrackedObject;
using core_msgs::msg::TrackedObjectArray;
using nav_msgs::msg::Odometry;
using nav_msgs::msg::Path;

// この距離以上ゴールが動いたら、別のゴールとして扱う
constexpr double kGoalChangedM = 0.05;
// ゴールの向きがこの角度 [rad]（約 3°）以上変わっても、別のゴールとして扱う。着いた後に、同じ位置で
// 向きだけ違うゴールを受けたら回り直す
constexpr double kGoalYawChangedRad = 0.05;
// 送った指令を覚えておく時間 [s]。姿勢はこれより古くならない
constexpr double kSentCommandKeepS = 1.0;
// 姿勢の時刻が今よりこれ以上先か、これ以上古ければ、時計が合っていないとみなして先を読まない [s]
constexpr double kMaxClockAheadS = 0.02;
constexpr double kMaxOdometryAgeS = 0.5;
// 計算の時間（姿勢を受けてから指令を出すまで）として見込む上限 [s]
constexpr double kMaxComputeS = 0.05;

// traction.* の実数のパラメータ
const std::array<std::pair<const char *, double TractionParams::*>, 16> kTractionDoubles{{
  {"traction.update_period_s", &TractionParams::update_period_s},
  {"traction.window_s", &TractionParams::window_s},
  {"traction.horizon_s", &TractionParams::horizon_s},
  {"traction.min_excitation_mps", &TractionParams::min_excitation_mps},
  {"traction.min_improvement_ratio", &TractionParams::min_improvement_ratio},
  {"traction.min_improvement_mps", &TractionParams::min_improvement_mps},
  {"traction.initial_friction_mps2", &TractionParams::initial_friction_mps2},
  {"traction.accel_ratio", &TractionParams::accel_ratio},
  {"traction.decel_ratio", &TractionParams::decel_ratio},
  {"traction.lateral_ratio", &TractionParams::lateral_ratio},
  {"traction.max_accel_mps2", &TractionParams::max_accel_mps2},
  {"traction.max_decel_mps2", &TractionParams::max_decel_mps2},
  {"traction.max_lateral_accel_mps2", &TractionParams::max_lateral_accel_mps2},
  {"traction.min_accel_mps2", &TractionParams::min_accel_mps2},
  {"traction.min_decel_mps2", &TractionParams::min_decel_mps2},
  {"traction.min_lateral_accel_mps2", &TractionParams::min_lateral_accel_mps2},
}};

// traction.* の 1 つを設定する。traction.* でなければ何もしない
void applyTractionParameter(TractionParams & params, const rclcpp::Parameter & parameter)
{
  const std::string & name = parameter.get_name();
  if (name == "traction.enabled") {
    params.enabled = parameter.as_bool();
  } else if (name == "traction.raise_after") {
    params.raise_after = static_cast<int>(parameter.as_int());
  } else {
    for (const auto & [key, member] : kTractionDoubles) {
      if (name == key) {
        params.*member = parameter.as_double();
      }
    }
  }
}

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double toSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

std::int64_t toNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL + static_cast<std::int64_t>(stamp.nanosec);
}

ObjectKind toObjectKind(const std::uint8_t classification)
{
  switch (classification) {
    case TrackedObject::CLASS_ROBOT:
      return ObjectKind::kRobot;
    case TrackedObject::CLASS_HUMAN:
      return ObjectKind::kHuman;
    default:
      return ObjectKind::kUnknown;
  }
}

// odom のメッセージの状態を、処理の遅れの分だけ先に進めて返す。twist は base_link 座標系
MppiState toState(const Odometry & odometry, const double latency_s)
{
  const double yaw = yawOf(odometry.pose.pose.orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const auto & linear = odometry.twist.twist.linear;
  MppiState state;
  state.vx = c * linear.x - s * linear.y;
  state.vy = s * linear.x + c * linear.y;
  state.wz = odometry.twist.twist.angular.z;
  state.x = odometry.pose.pose.position.x + state.vx * latency_s;
  state.y = odometry.pose.pose.position.y + state.vy * latency_s;
  state.yaw = yaw + state.wz * latency_s;
  return state;
}

// 追跡の速度はそのまま使う。止まっているロボットに付く誤った速度で横へ大きく外れるのは、正面・合流・横切りの
// 相手の予測の位置をよけない（OncomingMonitor）ことで防ぐ。速度を 0 にすると、追跡を始めた直後や見えなく
// なった間の、本当に速く動いているロボットの動きを見込めず、横切る相手に当て、見え直した相手を急によけた
std::vector<PredictedObject> toObjects(
  const TrackedObjectArray & tracks, const double age_s)
{
  std::vector<PredictedObject> objects;
  objects.reserve(tracks.objects.size());
  for (const auto & track : tracks.objects) {
    PredictedObject object;
    object.x = track.position.x + track.velocity.x * age_s;
    object.y = track.position.y + track.velocity.y * age_s;
    object.vx = track.velocity.x;
    object.vy = track.velocity.y;
    object.sigma = track.position_sigma;
    object.velocity_sigma = track.velocity_sigma;
    object.kind = toObjectKind(track.classification);
    object.id = track.id;
    objects.push_back(object);
  }
  return objects;
}
}  // namespace

class MppiLiteNode final : public rclcpp::Node
{
public:
  explicit MppiLiteNode(const rclcpp::NodeOptions & options)
  : Node("mppi_lite", options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    chassis_(*this, tf_buffer_)
  {
    declareParameters();
    validateSide(get_parameter("passing_rule.side").as_string());
    if (const auto error = headingModeError(get_parameter("heading.mode").as_string())) {
      throw std::invalid_argument(*error);
    }
    loadParams();
    yaw_mode_ = static_cast<int>(get_parameter("heading.initial_yaw_mode").as_int());
    if (const auto error = staticMarginError(mppi_params_, max_speed_mps_)) {
      throw std::invalid_argument(*error);
    }
    if (const auto error = tractionParamsError(traction_params_)) {
      throw std::invalid_argument(*error);
    }

    parameter_validation_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & parameter : parameters) {
          if (parameter.get_name() == "passing_rule.side" &&
          parameter.as_string() != "right" && parameter.as_string() != "left")
          {
            result.successful = false;
            result.reason = "passing_rule.side は right か left を指定してください: " +
            parameter.as_string();
          }
          if (parameter.get_name() == "heading.mode") {
            if (const auto error = headingModeError(parameter.as_string())) {
              result.successful = false;
              result.reason = *error;
            }
          }
        }
        if (const auto error = staticMarginErrorAfter(parameters)) {
          result.successful = false;
          result.reason = *error;
        }
        TractionParams traction = traction_params_;
        for (const auto & parameter : parameters) {
          applyTractionParameter(traction, parameter);
        }
        if (const auto error = tractionParamsError(traction)) {
          result.successful = false;
          result.reason = *error;
        }
        return result;
      });
    parameter_callback_ = core_nav_common::onParametersSet(
      *this, [this](const std::vector<rclcpp::Parameter> &) {loadParams();});

    FieldBounds field;
    field.x_min = get_parameter("field.x_min").as_double();
    field.x_max = get_parameter("field.x_max").as_double();
    field.y_min = get_parameter("field.y_min").as_double();
    field.y_max = get_parameter("field.y_max").as_double();
    const auto field_start = std::chrono::steady_clock::now();
    distance_field_ = buildDistanceField(
      field, get_parameter("grid_resolution_m").as_double(),
      get_parameter("map_path").as_string(), staticMapOptions(*this));
    RCLCPP_INFO(
      get_logger(), "距離の場 %dx%d（%.3fm）を %.1f ms で作りました", distance_field_.info().width,
      distance_field_.info().height, distance_field_.info().resolution,
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - field_start)
      .count());

    command_publisher_ =
      create_publisher<geometry_msgs::msg::Twist>("/planning/cmd_vel_nav", rclcpp::QoS(10));
    // 同じ指令と、それを上半身の座標に直したときの向きの時刻。cmd_yaw_hold が、この時刻の向きから回った分を戻す
    command_frame_publisher_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/planning/cmd_vel_nav/frame", rclcpp::QoS(10));
    status_publisher_ = create_publisher<NavigationStatus>("/planning/navigation/status", rclcpp::QoS(10));
    local_plan_publisher_ = create_publisher<Path>("/planning/local_plan", rclcpp::QoS(1));
    // 行き詰まった所（map）。大域計画がしばらく通れない所として扱い、計画し直す
    stuck_point_publisher_ =
      create_publisher<geometry_msgs::msg::PointStamped>("/planning/stuck_point", rclcpp::QoS(10));
    plan_subscription_ = create_subscription<Path>(
      "/planning/plan", rclcpp::QoS(1), std::bind(&MppiLiteNode::onPlan, this, std::placeholders::_1));
    // 同じスキャンの追跡結果を待つ上限。待つたびに始めからにし、時間が来たら前の追跡結果で計算する
    tracks_wait_timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(tracks_wait_params_.wait_s > 0.0 ?
      tracks_wait_params_.wait_s : 1.0),
      std::bind(&MppiLiteNode::onTracksWaitTimeout, this));
    tracks_wait_timer_->cancel();
    tracks_subscription_ = create_subscription<TrackedObjectArray>(
      "/perception/tracks", rclcpp::QoS(10),
      std::bind(&MppiLiteNode::onTracks, this, std::placeholders::_1));
    health_subscription_ = create_subscription<LocalizationHealth>(
      "/localization/health", rclcpp::QoS(10),
      [this](const LocalizationHealth::ConstSharedPtr & health) {health_ = health;});
    odometry_subscription_ = create_subscription<Odometry>(
      "/localization/lio/odom", rclcpp::QoS(10),
      std::bind(&MppiLiteNode::onOdometryReceived, this, std::placeholders::_1));
    traction_publisher_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>("/planning/navigation/traction", rclcpp::QoS(10));
    sent_command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/control/cmd_vel", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::ConstSharedPtr & twist) {onSentCommand(*twist);});
    // 腰のモード（操縦の UI が切り替える。target_angle_node と同じ値）
    yaw_mode_subscription_ = create_subscription<std_msgs::msg::Int32>(
      "/control/yaw_mode", rclcpp::QoS(10),
      [this](const std_msgs::msg::Int32::ConstSharedPtr & mode) {onYawMode(mode->data);});
    // 操作が上半身を回しているか（0 でない速さを最後に受けた時刻。target_angle_node と同じ決め方）
    operator_rate_subscription_ = create_subscription<std_msgs::msg::Float64>(
      "/control/yaw_target_rate", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64::ConstSharedPtr & rate) {
        if (std::isfinite(rate->data) && std::abs(rate->data) > kOperatorRateThreshold) {
          last_operator_active_ = std::chrono::steady_clock::now();
        }
      });
    // 操作（UI）が決めた速度の上限。パラメータの上限（mppi.max_speed_mps）より下げるときだけ効く
    max_speed_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/planning/max_speed", rclcpp::QoS(1).reliable().transient_local());
    requested_max_speed_subscription_ = create_subscription<std_msgs::msg::Float64>(
      "/ui/auto/max_speed", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64::ConstSharedPtr & speed) {
        if (!std::isfinite(speed->data) || speed->data <= 0.0) {
          RCLCPP_WARN(get_logger(), "速度の上限 %.3f m/s は正の有限値でないので使いません", speed->data);
          return;
        }
        requested_max_speed_mps_ = speed->data;
        publishMaxSpeed();
      });
    publishMaxSpeed();
    control_mode_subscription_ = create_subscription<std_msgs::msg::String>(
      "/control/mode", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::ConstSharedPtr & mode) {
        auto_mode_ = mode->data == "auto";
      });
    // TURRET で操作がない間に、上半身を回す速さ（target_angle_node が操作の代わりに使う）
    auto_yaw_rate_publisher_ =
      create_publisher<std_msgs::msg::Float64>("/control/auto/yaw_target_rate", rclcpp::QoS(10));
    RCLCPP_INFO(
      get_logger(), "腰のモードは %s から始めます（heading.follow_yaw_mode %s）",
      yaw_mode_ == kTurretYawMode ? "TURRET" : "FRONT", follow_yaw_mode_ ? "true" : "false");
  }

private:
  void declareParameters()
  {
    using core_nav_common::declareBool;
    using core_nav_common::declareDouble;
    using core_nav_common::declareInt;
    using core_nav_common::declareString;

    declareString(*this, "map_path", "", "地図の PCD。空なら柵だけを障害物にする", true);
    declareStaticMapParams(*this);
    declareString(*this, "map_frame", "map", "地図のフレーム", true);
    declareString(*this, "odom_frame", "odom", "計画のフレーム", true);
    declareDouble(*this, "grid_resolution_m", 0.05, 0.01, 1.0, "距離の場の格子の大きさ", true);
    declareDouble(*this, "field.x_min", -12.0, -1000.0, 1000.0, "フィールドの範囲", true);
    declareDouble(*this, "field.x_max", 12.0, -1000.0, 1000.0, "フィールドの範囲", true);
    declareDouble(*this, "field.y_min", -7.5, -1000.0, 1000.0, "フィールドの範囲", true);
    declareDouble(*this, "field.y_max", 7.5, -1000.0, 1000.0, "フィールドの範囲", true);

    declareInt(*this, "mppi.samples", 300, 3, 20000, "サンプル数");
    declareInt(*this, "mppi.horizon_steps", 20, 2, 200, "予測のステップ数");
    declareDouble(*this, "mppi.dt", 0.1, 0.01, 1.0, "予測の刻み [s]");
    declareDouble(*this, "mppi.temperature", 1.0, 0.001, 1000.0, "重み付き平均の温度");
    declareDouble(*this, "mppi.noise_linear", 0.8, 0.0, 10.0, "速度のノイズ [m/s]");
    declareDouble(*this, "mppi.noise_correlation", 0.7, 0.0, 0.999, "ノイズの時間相関");
    declareDouble(*this, "mppi.max_speed_mps", 3.0, 0.0, 10.0, "速度の上限");
    declareDouble(*this, "mppi.max_accel_mps2", 3.0, 0.1, 20.0, "加速度の上限");
    declareDouble(*this, "mppi.max_angular_speed_radps", 1.5, 0.0, 20.0, "角速度の上限");
    declareDouble(*this, "mppi.max_angular_accel_radps2", 6.0, 0.1, 100.0, "角加速度の上限");
    declareDouble(*this, "mppi.yaw_gain_per_s", 1.5, 0.0, 100.0, "ゴールの向きへ回る比 [1/s]");
    declareDouble(
      *this, "mppi.yaw_brake_accel_ratio", 0.7, 0.0, 1.0,
      "向きの目標の手前で回る速さを落とすときに見込む角加速度の、max_angular_accel_radps2 に対する割合。"
      "回る速さを、この角加速度で止まりきれる速さまでに抑える。0 で抑えない（前の動き）");
    declareDouble(*this, "mppi.reference_decel_mps2", 2.0, 0.1, 20.0, "ゴール手前の減速度");
    declareDouble(
      *this, "mppi.curve.max_lateral_accel_mps2", 0.0, 0.0, 20.0,
      "経路の曲がりで出す横の加速度の上限 [m/s2]。参照軌道の速さを落とす。0 で落とさない");
    declareDouble(
      *this, "mppi.curve.window_m", 0.5, 0.05, 5.0, "曲率を測る、経路の前後の距離 [m]");
    declareDouble(*this, "mppi.goal_tolerance_m", 0.15, 0.01, 5.0, "到着とみなす距離");
    declareDouble(*this, "mppi.goal_speed_tolerance_mps", 0.2, 0.01, 5.0, "到着とみなす速度");
    declareDouble(
      *this, "mppi.goal_yaw_tolerance_deg", 5.0, -1.0, 180.0,
      "到着とみなす、ゴールの向きとの差 [deg]。位置に着いた後、ここまでその場で回る。"
      "負なら向きを見ない（位置と速さだけで着いたとする、前の動き）");
    declareDouble(
      *this, "mppi.goal_yaw_rate_tolerance_radps", 0.2, 0.01, 5.0, "向きが合ったとみなす回る速さ [rad/s]");
    declareDouble(
      *this, "mppi.goal_yaw_timeout_s", 5.0, 0.0, 60.0,
      "位置に着いてから、ゴールの向きに合うのを待つ時間の上限 [s]。過ぎたら向きが合わなくても着いたとする");
    declareDouble(*this, "mppi.latency_compensation_s", 0.05, 0.0, 1.0, "遅れの補償");
    declareBool(
      *this, "mppi.drive.predict_delay", false,
      "送った指令で、今送る指令が効き始める時刻まで状態を進めてから計画する（false で前の動き: "
      "latency_compensation_s の間、今の速度で位置を進める）");
    declareDouble(
      *this, "mppi.drive.dead_time_s", 0.0, 0.0, 1.0,
      "/cmd_vel を出してから駆動が応え始めるまでの時間（無駄時間）[s]");
    declareDouble(
      *this, "mppi.drive.time_constant_s", 0.0, 0.0, 1.0, "駆動が指令の速度へ近づく一次遅れの時定数 [s]");
    declareDouble(
      *this, "mppi.drive.max_accel_mps2", 5.0, 0.1, 50.0, "駆動の加速度の上限（先を読むときに使う）[m/s2]");
    declareDouble(
      *this, "mppi.drive.max_angular_accel_radps2", 10.0, 0.1, 200.0,
      "駆動の角加速度の上限（先を読むときに使う）[rad/s2]");
    declareBool(
      *this, "mppi.drive.lag_in_rollout", true,
      "予測にも駆動の一次遅れ（mppi.drive.time_constant_s）を入れる");
    declareBool(
      *this, "traction.enabled", false,
      "送った指令と測った速度から床の滑りを推定し、計画の加減速度と曲がりの横の加速度の上限を変える");
    declareDouble(*this, "traction.update_period_s", 2.0, 0.1, 60.0, "床を推定する間隔 [s]");
    declareDouble(*this, "traction.window_s", 3.0, 0.5, 60.0, "推定に使う記録の長さ [s]");
    declareDouble(*this, "traction.horizon_s", 0.3, 0.05, 5.0, "比べる 2 つの速度のサンプルの間隔 [s]");
    declareDouble(
      *this, "traction.min_excitation_mps", 0.3, 0.0, 10.0,
      "窓の中の速度の変化がこれ未満なら推定しない [m/s]");
    declareDouble(
      *this, "traction.min_improvement_ratio", 0.95, 0.01, 1.0,
      "床の上限を入れて速度の誤差がこの倍以下に減れば滑っているとみなす");
    declareDouble(
      *this, "traction.min_improvement_mps", 0.005, 0.0, 10.0, "滑っているとみなす誤差の減りの下限 [m/s]");
    declareInt(*this, "traction.raise_after", 2, 1, 100, "床の値を上げるのに要る、続いた推定の回数");
    declareDouble(
      *this, "traction.initial_friction_mps2", 2.0, 0.1, 100.0, "推定を始める前の床の静止摩擦 [m/s2]");
    declareDouble(
      *this, "traction.accel_ratio", 2.5 / 3.0, 0.01, 10.0, "床の静止摩擦に掛けて加速度の上限にする比");
    declareDouble(
      *this, "traction.decel_ratio", 0.6, 0.01, 10.0, "床の静止摩擦に掛けて参照軌道の減速度にする比");
    declareDouble(
      *this, "traction.lateral_ratio", 2.0 / 3.0, 0.01, 10.0,
      "床の静止摩擦に掛けて曲がりの横の加速度の上限にする比");
    declareDouble(*this, "traction.max_accel_mps2", 3.0, 0.1, 20.0, "加速度の上限の上限 [m/s2]");
    declareDouble(*this, "traction.max_decel_mps2", 2.0, 0.1, 20.0, "減速度の上限 [m/s2]");
    declareDouble(
      *this, "traction.max_lateral_accel_mps2", 2.5, 0.1, 20.0, "曲がりの横の加速度の上限の上限 [m/s2]");
    declareDouble(*this, "traction.min_accel_mps2", 1.0, 0.1, 20.0, "加速度の上限の下限 [m/s2]");
    declareDouble(*this, "traction.min_decel_mps2", 0.8, 0.1, 20.0, "減速度の下限 [m/s2]");
    declareDouble(
      *this, "traction.min_lateral_accel_mps2", 1.0, 0.1, 20.0, "曲がりの横の加速度の上限の下限 [m/s2]");
    declareBool(
      *this, "mppi.evade_while_stopped", false,
      "到達後や行き詰まりの間も、向かってくる相手すべてを避ける（元の動き）");
    declareBool(
      *this, "mppi.evade_humans_while_stopped", false,
      "到達後や行き詰まりの間に、向かってくる人だけを避ける（false なら人が来ても動き出さない）");
    declareBool(
      *this, "mppi.avoid_being_hit", false,
      "自機が近づいていない、動いているロボットも避ける（false なら当てられても相手の責任とする）");
    declareDouble(*this, "mppi.tracks_max_age_s", 1.0, 0.0, 30.0, "追跡結果を使う時間");
    declareDouble(
      *this, "mppi.tracks_wait_s", 0.02, 0.0, 0.2,
      "姿勢と同じスキャンの追跡結果を待つ上限 [s]。0 なら待たない", true);
    declareDouble(
      *this, "mppi.tracks_stamp_lag_s", 0.0, 0.0, 0.2,
      "追跡結果の時刻（点群のヘッダ）が姿勢の時刻（走査の終わり）よりこれだけ早くても同じスキャンとみなす [s]",
      true);
    declareDouble(*this, "mppi.backoff_speed_mps", 0.5, 0.0, 3.0, "下がるときの速度");
    declareDouble(*this, "mppi.keep_out_range_m", 0.5, 0.01, 5.0, "静的な障害物の罰の範囲");
    declareDouble(
      *this, "mppi.static_margin_time_s", 0.1, 0.0, 1.0,
      "静的な障害物との余裕に足す「速さ × この時間」[s]。速さの上限との積が 4 隅を調べる範囲に"
      "収まる値だけを受け付ける");
    declareBool(
      *this, "mppi.static_margin_approach_only", false,
      "余裕に足す速さを、静的な障害物へ近づく向きの速さだけにする（false で全方向の速さ）");
    declareDouble(*this, "mppi.weight_tracking", 1.0, 0.0, 1e6, "参照軌道の重み");
    declareDouble(*this, "mppi.weight_terminal", 5.0, 0.0, 1e6, "最終ステップの重み");
    declareDouble(*this, "mppi.weight_keep_out", 200.0, 0.0, 1e6, "静的な障害物の罰の重み");
    declareDouble(*this, "mppi.weight_collision", 1000.0, 0.0, 1e9, "衝突の重み");
    declareDouble(*this, "mppi.weight_object", 50.0, 0.0, 1e6, "相手への接近の重み");
    declareDouble(*this, "mppi.weight_effort", 0.1, 0.0, 1e6, "速度指令の変化の重み");
    declareDouble(
      *this, "mppi.weight_collision_course", 10.0, 0.0, 1e6, "動いている相手に向かう速度の重み");
    declareDouble(
      *this, "mppi.collision_course_lookahead_s", 2.5, 0.0, 10.0,
      "衝突コースを判定する最接近までの時間 [s]");
    declareDouble(*this, "mppi.robot_radius_m", 0.55, 0.0, 5.0, "他ロボットの半径");
    declareDouble(*this, "mppi.human_radius_m", 0.3, 0.0, 5.0, "人の半径");
    declareDouble(*this, "mppi.unknown_radius_m", 0.5, 0.0, 5.0, "不明な物体の半径");
    declareDouble(*this, "mppi.robot_margin_m", 0.4, 0.001, 10.0, "他ロボットとの安全距離（目安）");
    declareDouble(
      *this, "mppi.robot_min_margin_m", 0.1, 0.0, 10.0,
      "他ロボットとの最小の安全距離（robot_margin_m より大きければ robot_margin_m）");
    declareDouble(*this, "mppi.human_margin_m", 0.8, 0.001, 10.0, "人との安全距離");
    declareDouble(*this, "mppi.unknown_margin_m", 0.6, 0.001, 10.0, "不明な物体との安全距離");
    declareDouble(
      *this, "mppi.prediction_sigma_growth_mps", 0.3, 0.0, 10.0, "予測の不確かさの増え方");
    declareDouble(
      *this, "mppi.prediction_velocity_sigma_gain", 1.0, 0.0, 10.0,
      "追跡の速度の不確かさを予測の広がりに足す倍率");
    declareDouble(*this, "mppi.object_discount", 0.95, 0.01, 1.0, "将来のステップの割引");
    declareDouble(
      *this, "mppi.object_max_speed_mps", 3.0, 0.1, 10.0,
      "相手のロボットの速さの上限。合流とゴールの手前で待つ判定で、相手が速くなる上限に使う");
    // 安全停止（safety_gate）と同じ値を使う。設定ファイルは全ノードで共通
    declareDouble(
      *this, "safety_stop.reachable.obstacle_accel_mps2", 1.5, 0.0, 20.0,
      "相手の加速度の上限。合流で相手が交点に着く時刻と、ゴールの近くを通るかの予測に使う");

    declareBool(*this, "footprint.enabled", true, "機体の形で衝突を判定する");
    declareDouble(*this, "footprint.length_m", 0.8, 0.0, 5.0, "機体の前後の長さ");
    declareDouble(*this, "footprint.width_m", 0.8, 0.0, 5.0, "機体の左右の幅");
    declareDouble(*this, "footprint.margin_m", 0.1, 0.0, 5.0, "衝突判定の余裕");

    // 大域計画（global_planner）と同じ値を使う。設定ファイルは全ノードで共通
    declareBool(
      *this, "narrow_passage.enabled", true,
      "狭い通路に入る（通路の中では罰の範囲を真ん中まで縮め、速さを下げる）。false で元の動き");
    declareDouble(
      *this, "narrow_passage.min_side_clearance_m", 0.2, 0.0, 2.0,
      "通路に入るのに要る、機体を真ん中に置いたときの片側の余裕 [m]。これ未満なら手前で止まる");
    declareDouble(
      *this, "narrow_passage.lateral_tolerance_m", 0.05, 0.0, 1.0,
      "通路で罰をかけない真ん中からのずれ（格子の丸めの分を含む）[m]。速さの上限もこの分を残す");

    declareBool(*this, "speed_limit.enabled", true, "見通しと混雑で速度を制限する");
    declareDouble(*this, "speed_limit.reaction_time_s", 0.2, 0.0, 5.0, "反応時間");
    declareDouble(*this, "speed_limit.max_decel_mps2", 3.0, 0.1, 20.0, "止まるときの減速度");
    declareDouble(*this, "speed_limit.density_radius_m", 3.0, 0.0, 30.0, "混雑を数える半径");
    declareDouble(*this, "speed_limit.density_speed_scale", 0.9, 0.01, 1.0, "相手 1 体ごとの倍率");
    declareDouble(*this, "speed_limit.min_speed_mps", 0.5, 0.0, 10.0, "制限の下限");
    declareDouble(*this, "speed_limit.stopping_margin_m", 1.0, 0.0, 10.0, "距離から引く分");
    declareBool(*this, "speed_limit.human_enabled", true, "人の近くで速度の上限を下げる");
    declareDouble(
      *this, "speed_limit.human_radius_m", 1.0, 0.0, 30.0,
      "上限を下げる、人の中心から機体の外形までの距離");
    declareDouble(*this, "speed_limit.human_speed_mps", 1.0, 0.0, 10.0, "人の近くでの速度の上限");
    declareDouble(
      *this, "speed_limit.human_approach_time_s", 2.0, 0.0, 10.0,
      "人が向かってくるとみなす、領域が機体にかかるまでの時間");
    declareDouble(
      *this, "speed_limit.human_approach_speed_mps", 0.3, 0.0, 10.0,
      "人が向かってくるとみなす、自機へ向かう速さの下限");

    declareBool(*this, "passing_rule.enabled", true, "すれ違いの側を決める");
    declareString(*this, "passing_rule.side", "right", "自機が避ける側（right か left）");
    declareDouble(*this, "passing_rule.weight", 1.0, 0.0, 1000.0, "逆側を通るときの罰の重み");
    declareDouble(*this, "passing_rule.commit_distance_m", 6.0, 0.0, 50.0, "これより近い相手には規則を適用しない");
    declareDouble(*this, "passing_rule.range_m", 4.0, 0.1, 30.0, "規則を適用する前方の距離");
    declareDouble(*this, "passing_rule.lateral_range_m", 3.0, 0.1, 30.0, "規則を適用する横の距離");

    declareBool(
      *this, "merge_rule.enabled", true, "合流で先に入れる相手には減速せず、垂直二等分線まで寄せる");
    declareDouble(
      *this, "merge_rule.lead_time_s", 0.5, 0.0, 10.0, "交点をこの時間以上早く通れれば自機が先 [s]");
    declareBool(
      *this, "crossing_rule.enabled", true, "前方で進路が交わる横切りの相手は、近づく向きによらず避ける");
    declareBool(
      *this, "oncoming_rule.enabled", true,
      "正面・合流・横切りのロボットに横へ道を開けず、経路の上の停止線で止まる（false で元の動き）");
    declareDouble(
      *this, "oncoming_rule.stop_before_midpoint_m", 0.3, 0.0, 5.0,
      "正面の相手と出会ったときの中点より、自機の前の端を手前に止める距離 [m]");
    declareDouble(
      *this, "oncoming_rule.resume_after_s", 0.5, 0.0, 10.0,
      "止まる理由がなくなってから停止線を保つ時間 [s]");
    declareDouble(
      *this, "oncoming_rule.range_m", 15.0, 1.0, 50.0, "相手を見る、経路に沿った前方の距離 [m]");
    declareDouble(
      *this, "oncoming_rule.pass_goal_margin_m", 3.0, -1.0, 100.0,
      "正面の相手と出会う所からゴールまでがこれ以上なら、止まらずに規則の側へよける [m]。負なら常に止まる");

    declareBool(*this, "stuck_recovery.enabled", true, "行き詰まりから復帰する");
    declareDouble(*this, "stuck_recovery.no_progress_timeout_s", 2.0, 0.1, 120.0, "判定の時間");
    declareDouble(*this, "stuck_recovery.min_progress_m", 0.2, 0.0, 10.0, "進んだとみなす距離");
    declareDouble(*this, "stuck_recovery.wait_s", 1.0, 0.0, 60.0, "止まって待つ時間");
    declareDouble(
      *this, "stuck_recovery.near_goal_accept_m", 0.0, 0.0, 10.0,
      "到達が難しいゴールで、行き詰まったときにゴールまでの残りがこれ以下なら着いたことにする（0 で使わない）");
    declareDouble(
      *this, "stuck_recovery.hard_goal_clearance_m", 0.0, 0.0, 10.0,
      "ゴールから固定物までの距離がこれより小さければ、到達が難しいゴールとみなす");
    declareDouble(
      *this, "stuck_recovery.replan_timeout_s", 0.0, 0.0, 60.0,
      "行き詰まった後、計画し直した経路を待つ時間の上限（0 で待たない）");
    declareDouble(
      *this, "stuck_recovery.give_up_s", 0.0, 0.0, 600.0,
      "ゴールまでの残りの最小値がこの時間よくならなければ FAILED にする（0 で使わない）");
    declareDouble(
      *this, "stuck_recovery.blocked_ahead_m", 1.0, 0.0, 10.0,
      "行き詰まった所として大域計画に知らせる、経路に沿った機体の前方の距離");
    declareDouble(*this, "stuck_recovery.backoff_distance_m", 0.5, 0.0, 10.0, "下がる距離");
    declareInt(*this, "stuck_recovery.max_retries", 3, 0, 100, "復帰を試みる回数");
    declareDouble(
      *this, "stuck_recovery.moving_distance_m", 0.5, 0.0, 10.0,
      "この距離以上動けば、ゴールまでの残りが減らなくても進んでいるとみなす（0 で使わない）");
    declareDouble(
      *this, "stuck_recovery.moving_timeout_s", 10.0, 0.0, 600.0,
      "動いていることで進んでいるとみなすのは、残りの距離が最後に減ってからこの時間まで");

    declareBool(
      *this, "goal_approach.enabled", true,
      "動いているロボットが着くまでの間にゴールの近くを通る予測なら、手前で待つ");
    declareDouble(
      *this, "goal_approach.radius_m", 1.5, 0.0, 10.0, "相手の中心がこの距離より近くを通ればゴールの近くとする");
    declareDouble(
      *this, "goal_approach.hold_distance_m", 2.0, 0.1, 10.0, "待つ位置の、経路に沿ったゴールまでの距離");
    declareDouble(
      *this, "goal_approach.max_hold_s", 6.0, 0.0, 120.0, "1 つのゴールで、待つ位置の近くで待つ時間の上限");
    declareDouble(
      *this, "goal_approach.release_s", 1.0, 0.0, 10.0, "通る予測がなくなっても待ち続ける時間");

    declareDouble(
      *this, "localization_health.degraded_speed_mps", 1.0, 0.0, 10.0,
      "自己位置が不確かなときの速度の上限");

    declareString(
      *this, "heading.mode", "travel",
      "機体の向き。travel: 進む向き（経路の先）へ回り、ゴールの近くでゴールの向きへ回る。"
      "goal: ゴールの向きへ回り、なければ保つ（前の動き）");
    declareDouble(
      *this, "heading.lookahead_m", 1.0, 0.05, 20.0, "進む向きを取る、経路の今の位置から先の距離 [m]");
    declareDouble(
      *this, "heading.goal_align_distance_m", 1.0, 0.0, 20.0,
      "ゴールまでの経路に沿った残りがこれ以下なら、ゴールの向きへ回る [m]");
    // 安全停止（safety_gate）も同じ値を読み、退避の候補にかける。設定ファイルは全ノードで共通
    declareDouble(
      *this, "heading.forward.turn_angle_deg", 45.0, 0.0, 180.0,
      "forward で、向きの目標との差がこの角度を超える間は並進を抑える [deg]");
    declareDouble(
      *this, "heading.forward.turn_speed_mps", 0.5, 0.0, 10.0,
      "forward で、向きの差が大きい間の並進の速さの上限 [m/s]");
    declareBool(
      *this, "heading.blind_side.enabled", true, "見えない側（機体の後ろ）へ進むときの速さを抑える");
    declareDouble(
      *this, "heading.blind_side.visible_half_angle_deg", 96.0, 0.0, 180.0,
      "機体の前から測った、見える範囲の半角 [deg]");
    declareDouble(
      *this, "heading.blind_side.speed_mps", 0.5, 0.0, 10.0, "見えない側へ進むときの速さの上限 [m/s]");
    declareBool(
      *this, "heading.follow_yaw_mode", true,
      "腰のモード（/control/yaw_mode）に従う。TURRET（1）の間は上半身の向きを操作に任せ、向きを指令しない");
    declareDouble(
      *this, "heading.turret_command_lead_s", 0.0, 0.0, 0.5,
      "TURRET で、指令が足回りに効くまでの間に上半身が回る分を見込む時間 [s]。cmd_yaw_hold を使うときは 0"
      "（cmd_yaw_hold が、指令を作った時刻の向きから回った分を戻す）。使わないときは、指令から足回りが動くまでの遅れ");
    declareDouble(
      *this, "heading.operator_hold_s", 1.0, 0.0, 10.0,
      "操作の /control/yaw_target_rate を止めてから、自律移動が上半身の向きを決めるのに戻すまでの時間 [s]"
      "（target_angle_node の operator_hold_sec と同じ値）");
    declareBool(
      *this, "heading.chassis_face.enabled", true,
      "腰の TURRET で、足回りの面を進む向きにそろえる（4 つの面のうち近い面を正面にする）");
    declareDouble(
      *this, "heading.chassis_face.gain_per_s", 1.5, 0.0, 20.0, "足回りの向きの差に対する回る速さの比 [1/s]");
    declareDouble(
      *this, "heading.chassis_face.max_rate_radps", 1.0, 0.0, 10.0, "足回りをそろえる回る速さの上限 [rad/s]");
    declareDouble(
      *this, "heading.chassis_face.min_speed_mps", 0.3, 0.0, 5.0,
      "これより遅いときは足回りを回さない [m/s]");
    declareInt(
      *this, "heading.initial_yaw_mode", 1, 0, 1,
      "/control/yaw_mode を受けるまでの腰のモード（0: FRONT、1: TURRET）。body_controller の yaw_mode と"
      "同じ値にする（腰の無い sim は 0）", true);
  }

  static std::optional<std::string> headingModeError(const std::string & mode)
  {
    if (parseHeadingMode(mode).has_value()) {
      return std::nullopt;
    }
    return "heading.mode は travel・goal・forward・fixed のどれかを指定してください: " + mode;
  }

  // 変更後の値で staticMarginError を確かめる。変更しない値は今の設定を使う
  std::optional<std::string> staticMarginErrorAfter(
    const std::vector<rclcpp::Parameter> & parameters) const
  {
    MppiParams mppi = mppi_params_;
    double max_speed = max_speed_mps_;
    for (const auto & parameter : parameters) {
      const std::string & name = parameter.get_name();
      if (name == "mppi.static_margin_time_s") {
        mppi.static_margin_time_s = parameter.as_double();
      } else if (name == "mppi.max_speed_mps") {
        max_speed = parameter.as_double();
      } else if (name == "mppi.max_angular_speed_radps") {
        mppi.max_angular_speed_radps = parameter.as_double();
      } else if (name == "footprint.enabled") {
        mppi.footprint.enabled = parameter.as_bool();
      } else if (name == "footprint.length_m") {
        mppi.footprint.half_length_m = 0.5 * parameter.as_double();
      } else if (name == "footprint.width_m") {
        mppi.footprint.half_width_m = 0.5 * parameter.as_double();
      }
    }
    return staticMarginError(mppi, max_speed);
  }

  static void validateSide(const std::string & side)
  {
    if (side != "right" && side != "left") {
      throw std::invalid_argument("passing_rule.side は right か left を指定してください: " + side);
    }
  }

  void loadParams()
  {
    MppiParams mppi;
    mppi.samples = static_cast<int>(get_parameter("mppi.samples").as_int());
    mppi.horizon_steps = static_cast<int>(get_parameter("mppi.horizon_steps").as_int());
    mppi.dt = get_parameter("mppi.dt").as_double();
    mppi.temperature = get_parameter("mppi.temperature").as_double();
    mppi.noise_linear = get_parameter("mppi.noise_linear").as_double();
    mppi.noise_correlation = get_parameter("mppi.noise_correlation").as_double();
    mppi.max_accel_mps2 = get_parameter("mppi.max_accel_mps2").as_double();
    mppi.max_angular_speed_radps = get_parameter("mppi.max_angular_speed_radps").as_double();
    mppi.max_angular_accel_radps2 = get_parameter("mppi.max_angular_accel_radps2").as_double();
    mppi.yaw_gain_per_s = get_parameter("mppi.yaw_gain_per_s").as_double();
    mppi.yaw_brake_accel_ratio = get_parameter("mppi.yaw_brake_accel_ratio").as_double();
    mppi.reference_decel_mps2 = get_parameter("mppi.reference_decel_mps2").as_double();
    mppi.curve.max_lateral_accel_mps2 =
      get_parameter("mppi.curve.max_lateral_accel_mps2").as_double();
    mppi.curve.window_m = get_parameter("mppi.curve.window_m").as_double();
    mppi.keep_out_range_m = get_parameter("mppi.keep_out_range_m").as_double();
    mppi.static_margin_time_s = get_parameter("mppi.static_margin_time_s").as_double();
    mppi.static_margin_approach_only = get_parameter("mppi.static_margin_approach_only").as_bool();
    mppi.weight_tracking = get_parameter("mppi.weight_tracking").as_double();
    mppi.weight_terminal = get_parameter("mppi.weight_terminal").as_double();
    mppi.weight_keep_out = get_parameter("mppi.weight_keep_out").as_double();
    mppi.weight_collision = get_parameter("mppi.weight_collision").as_double();
    mppi.weight_object = get_parameter("mppi.weight_object").as_double();
    mppi.weight_effort = get_parameter("mppi.weight_effort").as_double();
    mppi.weight_collision_course = get_parameter("mppi.weight_collision_course").as_double();
    mppi.collision_course_lookahead_s =
      get_parameter("mppi.collision_course_lookahead_s").as_double();
    mppi.objects.robot_radius_m = get_parameter("mppi.robot_radius_m").as_double();
    mppi.objects.human_radius_m = get_parameter("mppi.human_radius_m").as_double();
    mppi.objects.unknown_radius_m = get_parameter("mppi.unknown_radius_m").as_double();
    mppi.objects.robot_margin_m = get_parameter("mppi.robot_margin_m").as_double();
    mppi.objects.robot_min_margin_m = get_parameter("mppi.robot_min_margin_m").as_double();
    mppi.objects.human_margin_m = get_parameter("mppi.human_margin_m").as_double();
    mppi.objects.unknown_margin_m = get_parameter("mppi.unknown_margin_m").as_double();
    mppi.objects.prediction_sigma_growth_mps =
      get_parameter("mppi.prediction_sigma_growth_mps").as_double();
    mppi.objects.velocity_sigma_gain =
      get_parameter("mppi.prediction_velocity_sigma_gain").as_double();
    mppi.objects.discount = get_parameter("mppi.object_discount").as_double();
    mppi.objects.accel_mps2 = get_parameter("safety_stop.reachable.obstacle_accel_mps2").as_double();
    mppi.objects.max_speed_mps = get_parameter("mppi.object_max_speed_mps").as_double();
    mppi.footprint.enabled = get_parameter("footprint.enabled").as_bool();
    mppi.footprint.half_length_m = 0.5 * get_parameter("footprint.length_m").as_double();
    mppi.footprint.half_width_m = 0.5 * get_parameter("footprint.width_m").as_double();
    mppi.footprint.margin_m = get_parameter("footprint.margin_m").as_double();
    mppi.narrow.enabled = get_parameter("narrow_passage.enabled").as_bool();
    mppi.narrow.min_side_clearance_m = get_parameter("narrow_passage.min_side_clearance_m").as_double();
    mppi.narrow.lateral_tolerance_m = get_parameter("narrow_passage.lateral_tolerance_m").as_double();
    mppi.passing.enabled = get_parameter("passing_rule.enabled").as_bool();
    mppi.passing.side_right = get_parameter("passing_rule.side").as_string() == "right";
    mppi.passing.weight = get_parameter("passing_rule.weight").as_double();
    mppi.passing.commit_distance_m = get_parameter("passing_rule.commit_distance_m").as_double();
    mppi.passing.range_m = get_parameter("passing_rule.range_m").as_double();
    mppi.passing.lateral_range_m = get_parameter("passing_rule.lateral_range_m").as_double();
    mppi.avoid_being_hit = get_parameter("mppi.avoid_being_hit").as_bool();
    mppi.merge.enabled = get_parameter("merge_rule.enabled").as_bool();
    mppi.merge.lead_time_s = get_parameter("merge_rule.lead_time_s").as_double();
    mppi.crossing.enabled = get_parameter("crossing_rule.enabled").as_bool();
    mppi.oncoming.enabled = get_parameter("oncoming_rule.enabled").as_bool();
    mppi.oncoming.stop_before_midpoint_m =
      get_parameter("oncoming_rule.stop_before_midpoint_m").as_double();
    mppi.oncoming.resume_after_s = get_parameter("oncoming_rule.resume_after_s").as_double();
    mppi.oncoming.range_m = get_parameter("oncoming_rule.range_m").as_double();
    mppi.oncoming.pass_goal_margin_m = get_parameter("oncoming_rule.pass_goal_margin_m").as_double();
    const std::string heading_mode = get_parameter("heading.mode").as_string();
    if (const auto mode = parseHeadingMode(heading_mode)) {
      mppi.heading.mode = *mode;
    } else {
      // 起動時と変更時に確かめているので、ここへは来ないはず
      RCLCPP_ERROR(
        get_logger(), "%s。前の値を使います", headingModeError(heading_mode).value_or("").c_str());
      mppi.heading.mode = mppi_params_.heading.mode;
    }
    mppi.heading.lookahead_m = get_parameter("heading.lookahead_m").as_double();
    mppi.heading.goal_align_distance_m = get_parameter("heading.goal_align_distance_m").as_double();
    mppi.heading.forward_turn_angle_rad =
      get_parameter("heading.forward.turn_angle_deg").as_double() * M_PI / 180.0;
    mppi.heading.forward_turn_speed_mps = get_parameter("heading.forward.turn_speed_mps").as_double();
    mppi.blind_side.enabled = get_parameter("heading.blind_side.enabled").as_bool();
    mppi.blind_side.visible_half_angle_deg =
      get_parameter("heading.blind_side.visible_half_angle_deg").as_double();
    mppi.blind_side.speed_mps = get_parameter("heading.blind_side.speed_mps").as_double();
    follow_yaw_mode_ = get_parameter("heading.follow_yaw_mode").as_bool();
    operator_hold_s_ = get_parameter("heading.operator_hold_s").as_double();
    turret_command_lead_s_ = get_parameter("heading.turret_command_lead_s").as_double();
    chassis_face_params_.enabled = get_parameter("heading.chassis_face.enabled").as_bool();
    chassis_face_params_.gain_per_s = get_parameter("heading.chassis_face.gain_per_s").as_double();
    chassis_face_params_.max_rate_radps =
      get_parameter("heading.chassis_face.max_rate_radps").as_double();
    chassis_face_params_.min_speed_mps =
      get_parameter("heading.chassis_face.min_speed_mps").as_double();
    DriveParams drive;
    drive.predict_delay = get_parameter("mppi.drive.predict_delay").as_bool();
    drive.dead_time_s = get_parameter("mppi.drive.dead_time_s").as_double();
    drive.time_constant_s = get_parameter("mppi.drive.time_constant_s").as_double();
    drive.max_accel_mps2 = get_parameter("mppi.drive.max_accel_mps2").as_double();
    drive.max_angular_accel_radps2 = get_parameter("mppi.drive.max_angular_accel_radps2").as_double();
    mppi.drive_time_constant_s =
      get_parameter("mppi.drive.lag_in_rollout").as_bool() ? drive.time_constant_s : 0.0;
    drive_params_ = drive;
    TractionParams traction;
    for (const char * name : {"traction.enabled", "traction.raise_after"}) {
      applyTractionParameter(traction, get_parameter(name));
    }
    for (const auto & entry : kTractionDoubles) {
      applyTractionParameter(traction, get_parameter(entry.first));
    }
    if (traction.enabled && !traction_params_.enabled) {
      // 推定を始め直す
      traction_state_.reset(traction.initial_friction_mps2);
      odometry_samples_.clear();
      next_traction_s_.reset();
    }
    traction_params_ = traction;
    base_mppi_params_ = mppi;
    mppi_params_ = traction.enabled ?
      applyTraction(mppi, traction_state_.friction(), traction) : mppi;

    SpeedLimitParams speed_limit;
    speed_limit.enabled = get_parameter("speed_limit.enabled").as_bool();
    speed_limit.reaction_time_s = get_parameter("speed_limit.reaction_time_s").as_double();
    speed_limit.max_decel_mps2 = get_parameter("speed_limit.max_decel_mps2").as_double();
    speed_limit.density_radius_m = get_parameter("speed_limit.density_radius_m").as_double();
    speed_limit.density_speed_scale =
      get_parameter("speed_limit.density_speed_scale").as_double();
    speed_limit.min_speed_mps = get_parameter("speed_limit.min_speed_mps").as_double();
    speed_limit.stopping_margin_m = get_parameter("speed_limit.stopping_margin_m").as_double();
    speed_limit.human_enabled = get_parameter("speed_limit.human_enabled").as_bool();
    speed_limit.human_radius_m = get_parameter("speed_limit.human_radius_m").as_double();
    speed_limit.human_speed_mps = get_parameter("speed_limit.human_speed_mps").as_double();
    speed_limit.human_approach_time_s =
      get_parameter("speed_limit.human_approach_time_s").as_double();
    speed_limit.human_approach_speed_mps =
      get_parameter("speed_limit.human_approach_speed_mps").as_double();
    speed_limit_params_ = speed_limit;

    StuckParams stuck;
    stuck.enabled = get_parameter("stuck_recovery.enabled").as_bool();
    stuck.no_progress_timeout_s =
      get_parameter("stuck_recovery.no_progress_timeout_s").as_double();
    stuck.min_progress_m = get_parameter("stuck_recovery.min_progress_m").as_double();
    stuck.wait_s = get_parameter("stuck_recovery.wait_s").as_double();
    stuck.backoff_distance_m = get_parameter("stuck_recovery.backoff_distance_m").as_double();
    stuck.max_retries = static_cast<int>(get_parameter("stuck_recovery.max_retries").as_int());
    stuck.moving_distance_m = get_parameter("stuck_recovery.moving_distance_m").as_double();
    stuck.moving_timeout_s = get_parameter("stuck_recovery.moving_timeout_s").as_double();
    stuck.near_goal_accept_m = get_parameter("stuck_recovery.near_goal_accept_m").as_double();
    stuck.replan_timeout_s = get_parameter("stuck_recovery.replan_timeout_s").as_double();
    stuck.give_up_s = get_parameter("stuck_recovery.give_up_s").as_double();
    stuck_params_ = stuck;
    blocked_ahead_m_ = get_parameter("stuck_recovery.blocked_ahead_m").as_double();
    hard_goal_clearance_m_ = get_parameter("stuck_recovery.hard_goal_clearance_m").as_double();

    GoalApproachParams goal_approach;
    goal_approach.enabled = get_parameter("goal_approach.enabled").as_bool();
    goal_approach.radius_m = get_parameter("goal_approach.radius_m").as_double();
    goal_approach.hold_distance_m = get_parameter("goal_approach.hold_distance_m").as_double();
    goal_approach.max_hold_s = get_parameter("goal_approach.max_hold_s").as_double();
    goal_approach.release_s = get_parameter("goal_approach.release_s").as_double();
    goal_approach_params_ = goal_approach;

    max_speed_mps_ = get_parameter("mppi.max_speed_mps").as_double();
    publishMaxSpeed();
    GoalArrivalParams arrival;
    arrival.position_tolerance_m = get_parameter("mppi.goal_tolerance_m").as_double();
    arrival.speed_tolerance_mps = get_parameter("mppi.goal_speed_tolerance_mps").as_double();
    arrival.yaw_tolerance_rad =
      get_parameter("mppi.goal_yaw_tolerance_deg").as_double() * M_PI / 180.0;
    arrival.yaw_rate_tolerance_radps =
      get_parameter("mppi.goal_yaw_rate_tolerance_radps").as_double();
    arrival.yaw_timeout_s = get_parameter("mppi.goal_yaw_timeout_s").as_double();
    goal_arrival_params_ = arrival;
    latency_compensation_s_ = get_parameter("mppi.latency_compensation_s").as_double();
    evade_while_stopped_ = get_parameter("mppi.evade_while_stopped").as_bool();
    evade_humans_while_stopped_ = get_parameter("mppi.evade_humans_while_stopped").as_bool();
    tracks_max_age_s_ = get_parameter("mppi.tracks_max_age_s").as_double();
    // 使える古さより古い追跡結果しかないときは、検出が止まっているとみなして待たない
    tracks_wait_params_ = TracksWaitParams{
      get_parameter("mppi.tracks_wait_s").as_double(), tracks_max_age_s_,
      get_parameter("mppi.tracks_stamp_lag_s").as_double()};
    backoff_speed_mps_ = get_parameter("mppi.backoff_speed_mps").as_double();
    degraded_speed_mps_ = get_parameter("localization_health.degraded_speed_mps").as_double();
  }

  // heading.mode: fixed の向きを決めて、MPPI の設定（odom の座標系）に入れる。fixed でなければ忘れる
  void updateFixedYaw(const MppiState & state, const SE2 & map_from_odom)
  {
    if (mppi_params_.heading.mode != HeadingMode::kFixed) {
      fixed_yaw_map_.reset();
      mppi_params_.heading.fixed_yaw.reset();
      return;
    }
    if (!fixed_yaw_map_.has_value()) {
      fixed_yaw_map_ = wrapAngle(state.yaw + map_from_odom.yaw);
      RCLCPP_INFO(
        get_logger(), "走行中は今の向き（地図で %.0f°）を保ちます", *fixed_yaw_map_ * 180.0 / M_PI);
    }
    mppi_params_.heading.fixed_yaw = wrapAngle(*fixed_yaw_map_ - map_from_odom.yaw);
  }

  void onPlan(const Path::ConstSharedPtr & plan)
  {
    // 経路が変わったので、進んだ位置は新しい経路の始点から探し直す
    progress_s_.reset();
    if (plan->poses.empty()) {
      plan_map_.clear();
      goal_yaw_map_.reset();
      fixed_yaw_map_.reset();
      navigation_state_ = NavigationStatus::STATE_IDLE;
      controller_.reset();
      stuck_monitor_.reset();
      arrival_monitor_.reset();
      resetGoalHold();
      return;
    }
    std::vector<Point2> points;
    points.reserve(plan->poses.size());
    for (const auto & pose : plan->poses) {
      points.push_back(Point2{pose.pose.position.x, pose.pose.position.y});
    }
    const Point2 goal = points.back();
    const double goal_yaw = yawOf(plan->poses.back().pose.orientation);
    const bool goal_changed = plan_map_.empty() || !goal_yaw_map_.has_value() ||
      std::hypot(goal.x - plan_map_.back().x, goal.y - plan_map_.back().y) > kGoalChangedM ||
      std::abs(wrapAngle(goal_yaw - *goal_yaw_map_)) > kGoalYawChangedRad;
    plan_map_ = std::move(points);
    goal_yaw_map_ = goal_yaw;

    // 同じゴールへの計画のし直しでは、到着や失敗の状態を変えない
    if (goal_changed) {
      // fixed で保つ向きは、新しいゴールで最初に姿勢を受けたときの向きにする
      fixed_yaw_map_.reset();
      navigation_state_ = NavigationStatus::STATE_DRIVING;
      controller_.reset();
      stuck_monitor_.reset();
      arrival_monitor_.reset();
      resetGoalHold();
    } else {
      // 同じゴールへの計画し直し。行き詰まって待っている間なら、この経路で走行に戻れる
      stuck_monitor_.planReceived(now().seconds());
    }
  }

  std::optional<SE2> lookupMapFromOdom()
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        get_parameter("map_frame").as_string(), get_parameter("odom_frame").as_string(),
        tf2::TimePointZero);
      return SE2{
        transform.transform.translation.x, transform.transform.translation.y,
        yawOf(transform.transform.rotation)};
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "map から odom への変換を取得できません: %s", error.what());
      return std::nullopt;
    }
  }

  // 追跡結果を今の時刻まで進める。古すぎるものは使わない
  std::vector<PredictedObject> currentObjects(const double time_s)
  {
    if (tracks_ == nullptr) {
      return {};
    }
    const double age_s = std::max(time_s - toSeconds(tracks_->header.stamp), 0.0);
    if (age_s > tracks_max_age_s_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "追跡結果が古いので使いません（%.2f 秒前）", age_s);
      return {};
    }
    return toObjects(*tracks_, age_s);
  }

  ReferencePath makeOdomPath(const SE2 & map_from_odom) const
  {
    const SE2 odom_from_map = map_from_odom.inverse();
    std::vector<Point2> points;
    points.reserve(plan_map_.size());
    for (const auto & point : plan_map_) {
      points.push_back(odom_from_map.apply(point.x, point.y));
    }
    std::optional<double> goal_yaw;
    if (goal_yaw_map_.has_value()) {
      goal_yaw = wrapAngle(*goal_yaw_map_ - map_from_odom.yaw);
    }
    return ReferencePath(std::move(points), goal_yaw);
  }

  // 今使っている速度の上限を出す（UI に見せる）。起動中で出す先がまだなければ何もしない
  void publishMaxSpeed()
  {
    if (max_speed_publisher_ == nullptr) {
      return;
    }
    std_msgs::msg::Float64 message;
    message.data = limitedMaxSpeed(max_speed_mps_, requested_max_speed_mps_);
    max_speed_publisher_->publish(message);
  }

  void publishCommand(const MppiState & state, const MppiCommand & command)
  {
    // 指令は上半身の座標で出す。TURRET では、指令が足回りに効くまでの遅れ（安全停止・cmd_yaw_hold を通って
    // 足回りが動くまで）の間も上半身が回り、cmd_yaw_hold は自分が受けてから回った分しか直さないので、
    // その遅れの間に回る分を先に見込んだ向きで直す
    const double lead_s = turret_cycle_ ? turret_command_lead_s_ : 0.0;
    const double c = std::cos(state.yaw + upper_rate_measured_ * lead_s);
    const double s = std::sin(state.yaw + upper_rate_measured_ * lead_s);
    geometry_msgs::msg::Twist twist;
    twist.linear.x = c * command.vx + s * command.vy;
    twist.linear.y = -s * command.vx + c * command.vy;
    // TURRET では wz は足回りだけを回すので、足回りの面をそろえる速さにする。上半身を回す速さは
    // 操作が動いていない自動の運転の間だけ target_angle_node へ出す（それ以外は 0。上半身は向きを保つ）
    twist.angular.z = turret_cycle_ ? chassis_rate_cmd_ : command.wz;
    // 先に時刻つきの指令を出す（cmd_yaw_hold は、安全停止の後の指令を受けた時に、向きの同じものを探す）
    geometry_msgs::msg::TwistStamped frame;
    frame.header.stamp = rclcpp::Time(static_cast<int64_t>((state_time_s_ + lead_s) * 1e9), RCL_ROS_TIME);
    frame.header.frame_id = get_parameter("footprint.base_frame").as_string();
    frame.twist = twist;
    command_frame_publisher_->publish(frame);
    command_publisher_->publish(twist);
    std_msgs::msg::Float64 upper_rate;
    upper_rate.data = turret_auto_cycle_ && auto_mode_ ? command.wz : 0.0;
    auto_yaw_rate_publisher_->publish(upper_rate);
    upper_rate_cmd_ = upper_rate.data;
    // 次の周期で、計算の時間として見込む
    last_compute_s_ = std::clamp(
      std::chrono::duration<double>(std::chrono::steady_clock::now() - odometry_start_).count(), 0.0,
      kMaxComputeS);
  }

  void publishStatus(
    const builtin_interfaces::msg::Time & stamp, const double speed_limit,
    const double distance_to_goal, const std::chrono::steady_clock::time_point & start)
  {
    NavigationStatus status;
    status.header.stamp = stamp;
    status.header.frame_id = get_parameter("odom_frame").as_string();
    status.state = navigation_state_;
    status.retries = static_cast<std::uint32_t>(stuck_monitor_.retries());
    status.speed_limit_mps = static_cast<float>(speed_limit);
    status.planning_ms = std::chrono::duration<float, std::milli>(
      std::chrono::steady_clock::now() - start).count();
    status.distance_to_goal_m = static_cast<float>(distance_to_goal);
    status_publisher_->publish(status);
    publishLocalPlan(stamp);
  }

  // この周期に MPPI が選んだ軌跡（odom 座標系）を出す。MPPI を使わなかった周期は空にして、
  // 古い軌跡を残さない
  void publishLocalPlan(const builtin_interfaces::msg::Time & stamp)
  {
    Path local_plan;
    local_plan.header.stamp = stamp;
    local_plan.header.frame_id = get_parameter("odom_frame").as_string();
    if (controller_used_) {
      const auto & trajectory = controller_.lastTrajectory();
      local_plan.poses.reserve(trajectory.size());
      for (const auto & point : trajectory) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = local_plan.header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.orientation.z = std::sin(0.5 * point.yaw);
        pose.pose.orientation.w = std::cos(0.5 * point.yaw);
        local_plan.poses.push_back(pose);
      }
    }
    local_plan_publisher_->publish(local_plan);
  }

  void onTracks(const TrackedObjectArray::ConstSharedPtr & tracks)
  {
    tracks_ = tracks;
    if (pending_odometry_ != nullptr &&
      isTracksForPose(
        toNanoseconds(tracks->header.stamp), toNanoseconds(pending_odometry_->header.stamp),
        stampLagNs(tracks_wait_params_)))
    {
      computePending();
    }
  }

  // 姿勢を受けたら、同じスキャンの追跡結果がまだなければ待つ。届いていれば（または待たない設定なら）すぐ計算する
  void onOdometryReceived(const Odometry::ConstSharedPtr & odometry)
  {
    if (pending_odometry_ != nullptr) {
      // 待っている間に次の姿勢が来たら、前の姿勢で先に計算して指令を抜かさない
      computePending();
    }
    const std::optional<std::int64_t> tracks_ns = tracks_ != nullptr ?
      std::optional<std::int64_t>(toNanoseconds(tracks_->header.stamp)) : std::nullopt;
    if (shouldWaitForTracks(tracks_ns, toNanoseconds(odometry->header.stamp), tracks_wait_params_)) {
      pending_odometry_ = odometry;
      tracks_wait_timer_->reset();
      return;
    }
    onOdometry(odometry);
  }

  void onTracksWaitTimeout()
  {
    if (pending_odometry_ == nullptr) {
      tracks_wait_timer_->cancel();
      return;
    }
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "同じスキャンの追跡結果が %.0f ms 待っても届かないので、前の追跡結果で計算します",
      tracks_wait_params_.wait_s * 1000.0);
    computePending();
  }

  void computePending()
  {
    tracks_wait_timer_->cancel();
    const Odometry::ConstSharedPtr odometry = std::move(pending_odometry_);
    pending_odometry_ = nullptr;
    onOdometry(odometry);
  }

  // 機体へ送られた指令を記録する。出した時刻は、受けた時刻で代える
  void onSentCommand(const geometry_msgs::msg::Twist & twist)
  {
    const double now_s = get_clock()->now().seconds();
    if (!sent_commands_.empty() && now_s < sent_commands_.back().time_s) {
      // 時計が戻った（シミュレーションのやり直しなど）。前の記録は使えない
      sent_commands_.clear();
    }
    sent_commands_.push_back(SentCommand{now_s, twist.linear.x, twist.linear.y, twist.angular.z});
    // 床の推定には、窓と比べる間隔の分の指令が要る
    const double keep_s = traction_params_.enabled ?
      std::max(kSentCommandKeepS, traction_params_.window_s + traction_params_.horizon_s + 1.0) :
      kSentCommandKeepS;
    pruneSentCommands(sent_commands_, now_s - keep_s, drive_params_.dead_time_s);
  }

  // 姿勢の時刻が今の時刻と合っているか（送った指令の時刻と比べられるか）
  bool clockConsistent(const double stamp_s)
  {
    const double age_s = get_clock()->now().seconds() - stamp_s;
    return age_s >= -kMaxClockAheadS && age_s <= kMaxOdometryAgeS;
  }

  // 床の推定。traction.update_period_s ごとに、記録から床を推定して計画の上限を変える
  void updateTraction(const builtin_interfaces::msg::Time & stamp, const MppiState & measured)
  {
    if (!traction_params_.enabled || !clockConsistent(toSeconds(stamp))) {
      return;
    }
    const double time_s = toSeconds(stamp);
    if (!odometry_samples_.empty() && time_s < odometry_samples_.back().time_s) {
      // 時計が戻った（シミュレーションのやり直しなど）
      odometry_samples_.clear();
      next_traction_s_.reset();
    }
    odometry_samples_.push_back(OdometrySample{time_s, measured});
    const double keep_from_s =
      time_s - (traction_params_.window_s + traction_params_.horizon_s + 1.0);
    const auto keep = std::find_if(
      odometry_samples_.begin(), odometry_samples_.end(),
      [keep_from_s](const OdometrySample & sample) {return sample.time_s >= keep_from_s;});
    odometry_samples_.erase(odometry_samples_.begin(), keep);
    if (!next_traction_s_.has_value()) {
      next_traction_s_ = time_s + traction_params_.update_period_s;
      return;
    }
    if (time_s < *next_traction_s_) {
      return;
    }
    next_traction_s_ = time_s + traction_params_.update_period_s;
    const auto estimate = estimateTraction(
      odometry_samples_, sent_commands_, drive_params_, traction_params_, time_s,
      traction_state_.ratio());
    const double before = traction_state_.friction();
    const double after = traction_state_.update(estimate, traction_params_);
    if (after != before) {
      mppi_params_ = applyTraction(base_mppi_params_, after, traction_params_);
      RCLCPP_INFO(
        get_logger(), "床の推定を %.1f → %.1f m/s2 にしました（加速度 %.2f、減速度 %.2f、曲がり %.2f m/s2）",
        before, after, mppi_params_.max_accel_mps2, mppi_params_.reference_decel_mps2,
        mppi_params_.curve.max_lateral_accel_mps2);
    }
    geometry_msgs::msg::Vector3Stamped message;
    message.header.stamp = stamp;
    message.vector.x = after;
    message.vector.y = estimate.has_value() ?
      estimate->friction_mps2 : std::numeric_limits<double>::quiet_NaN();
    message.vector.z = mppi_params_.max_accel_mps2;
    traction_publisher_->publish(message);
  }

  // 先を読むなら、今送る指令が効き始める時刻を返す。先を読まない設定か、姿勢の時刻が今の時刻と合わない
  // （時計がずれている）ときは nullopt（前の動き）
  std::optional<double> predictionTime(const double stamp_s)
  {
    if (!drive_params_.predict_delay) {
      return std::nullopt;
    }
    const double now_s = get_clock()->now().seconds();
    const double age_s = now_s - stamp_s;
    if (age_s < -kMaxClockAheadS || age_s > kMaxOdometryAgeS) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "姿勢の時刻が今の時刻から %.3f 秒ずれているので、送った指令から先を読まずに、遅れの補償（%.3f 秒）"
        "だけ進めます", age_s, latency_compensation_s_);
      return std::nullopt;
    }
    return std::max(now_s, stamp_s) + last_compute_s_ + drive_params_.dead_time_s;
  }

  void onOdometry(const Odometry::ConstSharedPtr & odometry)
  {
    const auto start = std::chrono::steady_clock::now();
    odometry_start_ = start;
    controller_used_ = false;
    const double time_s = toSeconds(odometry->header.stamp);
    const MppiState measured = toState(*odometry, 0.0);
    updateTraction(odometry->header.stamp, measured);
    // 腰が TURRET の間: 操作が上半身を回していれば向きを指令しない（向きの目標なし、回る速さ 0）。
    // 操作がなければ、上半身の向きは FRONT と同じく決め、その回る速さは target_angle_node へ、足回りは
    // 面を進む向きにそろえる速さを cmd_vel の wz で出す。どちらでも、上半身は自律移動の指令（wz）では
    // 回らないので、向きは駆動の遅れの先読みで進めず、測った向きを使う
    turret_cycle_ = turretMode();
    turret_auto_cycle_ = turret_cycle_ && !operatorActive();
    upper_rate_measured_ = measured.wz;
    // 計画に使う上半身の向きの時刻（指令を上半身の座標に直す向き）。TURRET は yawHeldState と同じく、指令を
    // 出す時刻まで進めた向き。それ以外は遅れの先読み・補償の先の時刻（cmd_yaw_hold は先の時刻を使わない）
    const double ahead_s = std::max(get_clock()->now().seconds() + last_compute_s_ - time_s, 0.0);
    // 遅れの先読みは、送った指令（上半身の座標）を世界の座標に直しながら進める。TURRET では上半身は指令の wz
    // （足回りを回す速さ）ではなく、操作や target_angle_node が回すので、測った上半身の回る速さで向きを進める。
    // 指令の wz で進めると、上半身を回す操作の間、先読みした速度の向きが「回る速さ × 遅れ」だけずれ、
    // 経路から外れて構造物や柵に当たった（sim で操作で上半身を ±0.8rad/s で振ると 15 回中 7 回）
    const std::optional<double> prediction_s = predictionTime(time_s);
    const MppiState predicted = prediction_s.has_value() ?
      predictDelayedState(
      measured, time_s, *prediction_s, sent_commands_, drive_params_,
      traction_params_.enabled ? floorFor(traction_state_.friction()) : FloorTraction{}, 0.005,
      turret_cycle_ ? std::optional<double>(measured.wz) : std::nullopt) :
      toState(*odometry, latency_compensation_s_);
    state_time_s_ = turret_cycle_ ? time_s + std::min(ahead_s, kMaxYawHoldAheadS) :
      prediction_s.value_or(time_s + latency_compensation_s_);
    mppi_params_.heading.yaw_free = turret_cycle_ && !turret_auto_cycle_;
    // 自律移動が上半身を回すときの今の回る速さは、測った値でなく直前に出した指令にする。target_angle_node は
    // 指令の速さを先回りで腰に伝え、ずれを帰還で詰めるので、測った速さには帰還の分が乗る。それを今の速さとして
    // 加速度の上限の分だけ変えた値を指令すると、帰還の分が周期ごとに積み上がり、上限の速さを超えて回り続ける
    const double held_rate = turret_auto_cycle_ ? upper_rate_cmd_ : 0.0;
    const MppiState state = turret_cycle_ ?
      yawHeldState(predicted, measured, time_s, held_rate) : predicted;
    chassis_rate_cmd_ = turret_cycle_ ? chassisFaceRateFor(state) : 0.0;
    mppi_params_.chassis_decoupled = turret_cycle_;
    mppi_params_.chassis_rate_radps = chassis_rate_cmd_;
    const auto stop = [&]() {
        return limitCommand(state, 0.0, 0.0, 0.0, mppi_params_);
      };

    const auto map_from_odom = lookupMapFromOdom();
    // 駆動部の向き（腰の角度）に合わせて、機体の形を回す（回転モードの間は外接円）
    if (mppi_params_.footprint.enabled) {
      // TURRET では上半身の向きを先へ進めて使う（yawHeldState）。腰の角度は姿勢と同じ時刻の値にし、進めた分を
      // 引いて、世界の座標での駆動部の向きを合わせる（上半身を操作で回すと、腰は逆へ同じ速さで回る）
      const auto shape = turret_cycle_ ?
        chassis_.shapeAt(
        0.5 * get_parameter("footprint.length_m").as_double(),
        0.5 * get_parameter("footprint.width_m").as_double(), odometry->header.stamp,
        wrapAngle(state.yaw - measured.yaw)) :
        chassis_.shape(
        0.5 * get_parameter("footprint.length_m").as_double(),
        0.5 * get_parameter("footprint.width_m").as_double());
      if (shape.has_value()) {
        orientFootprint(mppi_params_.footprint, *shape);
      }
    }
    if (!map_from_odom.has_value() || plan_map_.empty()) {
      publishCommand(state, stop());
      publishStatus(odometry->header.stamp, 0.0, 0.0, start);
      return;
    }

    // 経路上の進んだ位置は、前の周期の位置の近くだけで探す（行って戻る経路で、戻りの区間へ飛ばない）。
    // 大域計画の経路は機体の位置から始まるので、新しい経路を受けた直後は始点の近くを探す
    ReferencePath path = makeOdomPath(*map_from_odom);
    if (progress_s_.has_value()) {
      path.setProgressWindow(*progress_s_ - kProgressBackM, *progress_s_ + kProgressAheadM);
    } else {
      path.setProgressWindow(0.0, kProgressInitialAheadM);
    }
    progress_s_ = path.progress(state.x, state.y);
    updateFixedYaw(state, *map_from_odom);
    const double remaining = path.remaining(state.x, state.y);

    const GoalArrival arrival = navigation_state_ != NavigationStatus::STATE_REACHED &&
      navigation_state_ != NavigationStatus::STATE_FAILED ?
      updateArrival(time_s, state, path) : GoalArrival::kApproaching;
    if (navigation_state_ == NavigationStatus::STATE_FAILED ||
      navigation_state_ == NavigationStatus::STATE_IDLE)
    {
      publishCommand(state, stop());
      publishStatus(odometry->header.stamp, 0.0, remaining, start);
      return;
    }

    const auto objects = currentObjects(prediction_s.value_or(time_s + latency_compensation_s_));
    double speed_limit = computeSpeedLimit(
      speed_limit_params_, objects, state, mppi_params_.footprint,
      limitedMaxSpeed(max_speed_mps_, requested_max_speed_mps_));
    const std::uint8_t health_level =
      health_ != nullptr ? health_->level : LocalizationHealth::LEVEL_OK;
    // 到達後に相手を避けるときも、自己位置が怪しい間は速度を抑える
    if (health_level == LocalizationHealth::LEVEL_DEGRADED) {
      speed_limit = std::min(speed_limit, degraded_speed_mps_);
    }
    if (navigation_state_ == NavigationStatus::STATE_REACHED) {
      const auto evasion = health_level == LocalizationHealth::LEVEL_FAILED ? std::nullopt :
        evadeWhileStopped(state, objects, *map_from_odom, speed_limit);
      publishCommand(state, evasion.value_or(stop()));
      publishStatus(odometry->header.stamp, 0.0, remaining, start);
      return;
    }
    if (health_level == LocalizationHealth::LEVEL_FAILED) {
      // 位置が分からないまま動かない。止まっている間は行き詰まりと数えない
      stuck_monitor_.reset();
      controller_.reset();
      navigation_state_ = NavigationStatus::STATE_DRIVING;
      publishCommand(state, stop());
      publishStatus(odometry->header.stamp, 0.0, remaining, start);
      return;
    }
    // 位置に着いて、その場でゴールの向きへ回っている。残りの距離は減らないが行き詰まりではないので、
    // 判定をやり直す（待つ・下がる途中で位置に入ったときも、下がって離れたり失敗にしたりしない）
    if (arrival == GoalArrival::kAligning) {
      stuck_monitor_.reset();
    }
    if (isOnCollisionCourse(state, objects, mppi_params_)) {
      stuck_monitor_.holdProgress(time_s, remaining);
    }
    // ゴールの近くを相手が通る予測なら、手前で待つ。わざと止まるので行き詰まりとは数えない
    const auto hold_length = stuck_monitor_.state() == StuckState::kDriving ?
      goalHold(state, path, objects, speed_limit, time_s) : std::nullopt;
    if (hold_length.has_value()) {
      stuck_monitor_.holdProgress(time_s, remaining);
    }
    // 正面から来る相手と、合流・横切りで先に通す相手には、横へ道を開けず停止線で止まる。
    // わざと止まるので行き詰まりとは数えない
    const YieldPlan yield = oncomingYield(state, path, objects, speed_limit, time_s);
    if (yield.yielding > 0) {
      stuck_monitor_.holdProgress(time_s, remaining);
    }
    std::optional<double> cut_length = hold_length;
    if (yield.stop_length.has_value()) {
      cut_length = std::min(cut_length.value_or(*yield.stop_length), *yield.stop_length);
    }

    MppiCommand command;
    // ゴールが固定物（地図・柵・入らない範囲）に近すぎて機体が入れないなら、いちばん近くまで来て進めないときに
    // 着いたことにする。距離の場は map の座標
    stuck_monitor_.setGoalHardToReach(
      distance_field_.at(plan_map_.back().x, plan_map_.back().y) < hard_goal_clearance_m_);
    const StuckState previous_stuck = stuck_monitor_.state();
    const StuckState stuck_state =
      stuck_monitor_.update(time_s, remaining, state.x, state.y, stuck_params_);
    // 回数を使い切っていれば、待った後に FAILED になるので知らせない
    if (stuck_state == StuckState::kWait && previous_stuck != StuckState::kWait &&
      stuck_monitor_.retries() < stuck_params_.max_retries)
    {
      publishStuckPoint(state, path, *map_from_odom, odometry->header.stamp, remaining);
    }
    switch (stuck_state) {
      case StuckState::kDriving:
        navigation_state_ = NavigationStatus::STATE_DRIVING;
        speed_limit = std::min(speed_limit, yield.speed_limit);
        command = controller_.solve(
          state, cut_length.has_value() ? path.truncated(*cut_length) : path,
          markHeld(objects, yield.held_ids), &distance_field_, *map_from_odom, mppi_params_,
          speed_limit);
        controller_used_ = true;
        break;
      case StuckState::kWait: {
          navigation_state_ = NavigationStatus::STATE_STUCK_WAIT;
          const auto evasion = evadeWhileStopped(state, objects, *map_from_odom, speed_limit);
          command = evasion.value_or(stop());
          break;
        }
      case StuckState::kBackoff: {
          navigation_state_ = NavigationStatus::STATE_BACKOFF;
          const auto evasion = evadeWhileStopped(state, objects, *map_from_odom, speed_limit);
          if (evasion.has_value()) {
            command = *evasion;
            break;
          }
          const Point2 direction = chooseBackoffDirection(
            state, path, objects, &distance_field_, *map_from_odom, mppi_params_);
          // 見えない側へ下がるときは、MPPI と同じ上限を backoff_speed_mps にもかける
          const double speed = backoff_speed_mps_ * BlindSideLimit(mppi_params_.blind_side).scale(
            direction.x * backoff_speed_mps_, direction.y * backoff_speed_mps_,
            std::cos(state.yaw), std::sin(state.yaw));
          command = limitCommand(
            state, direction.x * speed, direction.y * speed, 0.0, mppi_params_);
          break;
        }
      case StuckState::kFailed:
        navigation_state_ = NavigationStatus::STATE_FAILED;
        controller_.reset();
        command = stop();
        RCLCPP_ERROR(get_logger(), "行き詰まりから復帰できませんでした。新しいゴールを待ちます");
        break;
      case StuckState::kNearGoal:
        // ゴールのすぐ手前で進めないので、下がってやり直さずに着いたことにする
        navigation_state_ = NavigationStatus::STATE_REACHED;
        controller_.reset();
        command = stop();
        RCLCPP_WARN(
          get_logger(),
          "ゴールが固定物に近く（%.2f m）、手前 %.2f m より近づけないので、着いたことにします",
          distance_field_.at(plan_map_.back().x, plan_map_.back().y), remaining);
        break;
    }
    publishCommand(state, command);
    publishStatus(odometry->header.stamp, speed_limit, remaining, start);
  }

  // ゴールに着いたかを判定し、着いたら（向きが合わないまま待つ時間が切れたときも）REACHED にする。
  // 位置に着いて回り始めるときと、向きが合わないまま着いたとするときを記録に出す
  // 腰のモードを受ける。0（FRONT）と 1（TURRET）のほかは無視する（target_angle_node と同じ）
  void onYawMode(const int mode)
  {
    if (mode != kFrontYawMode && mode != kTurretYawMode) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "知らない腰のモード %d を無視します（0: FRONT、1: TURRET）", mode);
      return;
    }
    // 回転モード中は TURRET から FRONT へ切り替えない（target_angle_node と同じ）
    if (yaw_mode_ == kTurretYawMode && mode == kFrontYawMode && chassis_.rotating()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "回転モード中なので、FRONT への切り替えを無視します（TURRET のまま）");
      return;
    }
    if (mode != yaw_mode_) {
      RCLCPP_INFO(
        get_logger(), "腰のモードが %s になりました%s", mode == kTurretYawMode ? "TURRET" : "FRONT",
        mode == kTurretYawMode && follow_yaw_mode_ ? "。上半身の向きは指令しません" : "");
    }
    yaw_mode_ = mode;
  }

  // 腰が TURRET で、heading.follow_yaw_mode のとき
  bool turretMode() const {return follow_yaw_mode_ && yaw_mode_ == kTurretYawMode;}

  // 操作が上半身を回しているか（止めてから heading.operator_hold_s の間も含む）
  bool operatorActive() const
  {
    if (!last_operator_active_.has_value()) {
      return false;
    }
    const double since_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - *last_operator_active_).count();
    return since_s <= operator_hold_s_;
  }

  // TURRET で、足回りの面を進む向きにそろえる回る速さ。回転モード中と、腰の角度が分からないときは 0
  double chassisFaceRateFor(const MppiState & state)
  {
    if (chassis_.rotating()) {
      return 0.0;
    }
    const auto waist = chassis_.chassisYaw();
    if (!waist.has_value()) {
      return 0.0;
    }
    return chassisFaceRate(
      wrapAngle(state.yaw + *waist), std::atan2(state.vy, state.vx), std::hypot(state.vx, state.vy),
      chassis_face_params_);
  }

  // 向きを指令しないときの計画の状態。上半身は操作で回るので、計画の間は向きを保つとみなす（回る速さ 0）。
  // 向きは、指令を出す時刻（今 + 計算の時間）の向きにする。cmd_yaw_hold はその時の向きを基準に、並進を
  // 世界の座標に保ち直す。駆動の遅れの先読み（predictDelayedState）で進めた向きは使わない
  // 回る速さは rate_radps（TURRET で自律移動が上半身を回すときは直前の指令、それ以外は 0）
  MppiState yawHeldState(
    MppiState state, const MppiState & measured, const double stamp_s,
    const double rate_radps) const
  {
    const double ahead_s = std::max(now().seconds() + last_compute_s_ - stamp_s, 0.0);
    state.yaw = wrapAngle(measured.yaw + measured.wz * std::min(ahead_s, kMaxYawHoldAheadS));
    state.wz = rate_radps;
    return state;
  }

  GoalArrival updateArrival(
    const double time_s, const MppiState & state, const ReferencePath & path)
  {
    const Point2 & goal = path.points().back();
    // 向きを指令しない間は、ゴールの向きを待たない（位置と速さだけで着いたとする）
    const std::optional<double> goal_yaw =
      mppi_params_.heading.yaw_free ? std::nullopt : path.goalYaw();
    const bool was_at_position = arrival_monitor_.positionReachedAt().has_value();
    const GoalArrival arrival =
      arrival_monitor_.update(time_s, state, goal, goal_yaw, goal_arrival_params_);
    const double yaw_error_deg =
      goal_yaw.has_value() ? wrapAngle(*goal_yaw - state.yaw) * 180.0 / M_PI : 0.0;
    if (arrival == GoalArrival::kAligning && !was_at_position) {
      RCLCPP_INFO(
        get_logger(), "位置に着きました。その場でゴールの向きへ回ります（向きの差 %.0f°）", yaw_error_deg);
    }
    if (arrival != GoalArrival::kReached && arrival != GoalArrival::kTimedOut) {
      return arrival;
    }
    const double aligning_s = arrival_monitor_.positionReachedAt().has_value() ?
      time_s - *arrival_monitor_.positionReachedAt() : 0.0;
    if (arrival == GoalArrival::kTimedOut) {
      RCLCPP_WARN(
        get_logger(),
        "位置に着いてから %.1f 秒でゴールの向きに合わないので、着いたとします（向きの差 %.1f°、"
        "回る速さ %.2f rad/s、位置の差 %.2f m）", aligning_s, yaw_error_deg, state.wz,
        std::hypot(goal.x - state.x, goal.y - state.y));
    } else if (aligning_s > 0.0) {
      RCLCPP_INFO(
        get_logger(), "ゴールの向きに合わせて着きました（位置に着いてから %.1f 秒、向きの差 %.1f°）",
        aligning_s, yaw_error_deg);
    }
    navigation_state_ = NavigationStatus::STATE_REACHED;
    controller_.reset();
    return arrival;
  }

  // 行き詰まった所を大域計画に知らせる。経路に沿って機体の blocked_ahead_m 前の点（ゴールを越えない）を
  // map で出し、大域計画はその周りをしばらく通れない所として計画し直す
  void publishStuckPoint(
    const MppiState & state, const ReferencePath & path, const SE2 & map_from_odom,
    const builtin_interfaces::msg::Time & stamp, const double remaining)
  {
    const double s = std::min(path.progress(state.x, state.y) + blocked_ahead_m_, path.length());
    const Point2 ahead = path.pointAt(s);
    const Point2 in_map = map_from_odom.apply(ahead.x, ahead.y);
    geometry_msgs::msg::PointStamped point;
    point.header.stamp = stamp;
    point.header.frame_id = get_parameter("map_frame").as_string();
    point.point.x = in_map.x;
    point.point.y = in_map.y;
    stuck_point_publisher_->publish(point);
    RCLCPP_WARN(
      get_logger(), "行き詰まりました（ゴールまで %.2f m、%d 回目の復帰）。(%.2f, %.2f) を避けて計画し直します",
      remaining, stuck_monitor_.retries() + 1, in_map.x, in_map.y);
  }

  // ゴールの手前で待つなら、待つ位置（経路の始点からの距離）を返す。待ち始めと終わりを記録に出す
  std::optional<double> goalHold(
    const MppiState & state, const ReferencePath & path,
    const std::vector<PredictedObject> & objects, const double speed_limit, const double time_s)
  {
    const bool was_holding = goal_hold_monitor_.holding();
    const auto hold = goal_hold_monitor_.update(
      time_s, state, path, objects, mppi_params_, goal_approach_params_, speed_limit);
    if (hold.has_value() != was_holding) {
      RCLCPP_INFO(
        get_logger(), "%s（odom のゴール %.2f, %.2f、待つ位置の近くで待った時間 %.1f 秒）",
        hold.has_value() ? "ゴールの近くを相手が通る予測なので、手前で待ちます" :
        "ゴールの手前で待つのをやめます", path.points().back().x, path.points().back().y,
        goal_hold_monitor_.waited());
    }
    return hold;
  }

  void resetGoalHold()
  {
    goal_hold_monitor_.reset();
    oncoming_monitor_.reset();
  }

  // 正面から来る相手と、合流・横切りの相手への応じ方を決める。止まり始めと終わりを記録に出す
  YieldPlan oncomingYield(
    const MppiState & state, const ReferencePath & path,
    const std::vector<PredictedObject> & objects, const double speed_limit, const double time_s)
  {
    const bool was_yielding = oncoming_yielding_;
    const YieldPlan yield =
      oncoming_monitor_.update(time_s, state, path, objects, mppi_params_, speed_limit);
    oncoming_yielding_ = yield.yielding > 0;
    if (oncoming_yielding_ != was_yielding) {
      const double stop_ahead = yield.stop_length.has_value() ?
        *yield.stop_length - path.progress(state.x, state.y) : 0.0;
      RCLCPP_INFO(
        get_logger(), "%s（相手 %d 台、停止線まで %.2f m、速さの上限 %.2f m/s、自機 %.2f, %.2f）",
        oncoming_yielding_ ? "向かってくる相手に道を開けず、停止線で止まります" :
        "停止線で止まるのをやめます", yield.yielding, stop_ahead,
        std::isfinite(yield.speed_limit) ? yield.speed_limit : -1.0, state.x, state.y);
    }
    return yield;
  }

  // 止まっている間や下がっている間に向かってくる相手がいれば、その場を基準に避ける指令を返す。
  // evade_while_stopped なら相手すべて、evade_humans_while_stopped なら人だけを避け、どちらも false
  // （既定）なら避けない。避けなければ次の計画に前回の結果を持ち越さないよう捨てて、nullopt を返す
  std::optional<MppiCommand> evadeWhileStopped(
    const MppiState & state, const std::vector<PredictedObject> & objects,
    const SE2 & map_from_odom, const double speed_limit)
  {
    if (!shouldEvadeWhileStopped(
        state, objects, mppi_params_, evade_while_stopped_, evade_humans_while_stopped_))
    {
      controller_.reset();
      return std::nullopt;
    }
    const ReferencePath here({Point2{state.x, state.y}, Point2{state.x, state.y}});
    const MppiCommand command = controller_.solve(
      state, here, objects, &distance_field_, map_from_odom, mppi_params_, speed_limit);
    controller_used_ = true;
    return command;
  }

  MppiParams mppi_params_;
  SpeedLimitParams speed_limit_params_;
  StuckParams stuck_params_;
  double blocked_ahead_m_{1.0};
  double hard_goal_clearance_m_{0.0};
  GoalApproachParams goal_approach_params_;
  double max_speed_mps_{3.0};
  // 操作（UI）が決めた速度の上限と、今使っている上限を出す先
  std::optional<double> requested_max_speed_mps_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr requested_max_speed_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr max_speed_publisher_;
  bool evade_while_stopped_{false};
  bool evade_humans_while_stopped_{false};
  GoalArrivalParams goal_arrival_params_;
  double latency_compensation_s_{0.05};
  DriveParams drive_params_;
  // 床の推定。base_mppi_params_ は推定で変える前の設定
  TractionParams traction_params_;
  TractionState traction_state_;
  MppiParams base_mppi_params_;
  std::vector<OdometrySample> odometry_samples_;
  std::optional<double> next_traction_s_;
  // 機体へ送られた指令（/cmd_vel を受けた時刻の順）
  std::vector<SentCommand> sent_commands_;
  // 前の周期の計算の時間（姿勢を受けてから指令を出すまで）[s]
  double last_compute_s_{0.0};
  std::chrono::steady_clock::time_point odometry_start_;
  double tracks_max_age_s_{1.0};
  TracksWaitParams tracks_wait_params_;
  double backoff_speed_mps_{0.5};
  double degraded_speed_mps_{1.0};

  DistanceField distance_field_;
  MppiController controller_;
  StuckMonitor stuck_monitor_;
  GoalArrivalMonitor arrival_monitor_;
  std::uint8_t navigation_state_{NavigationStatus::STATE_IDLE};
  // この周期に MPPI で指令を求めたか。ローカルパスを出すかの判断に使う
  bool controller_used_{false};
  std::vector<Point2> plan_map_;
  std::optional<double> goal_yaw_map_;
  // heading.mode: fixed で保つ向き（map の座標系）。ゴールが変わったときと、fixed に切り替えたときに覚え直す
  std::optional<double> fixed_yaw_map_;
  // 腰のモード（/control/yaw_mode）と、それに従うか
  static constexpr int kFrontYawMode = 0;
  static constexpr int kTurretYawMode = 1;
  // 向きを保つときに、姿勢の時刻から先へ進める時間の上限 [s]（時計がずれたときに向きを飛ばさない）
  static constexpr double kMaxYawHoldAheadS = 0.3;
  // 経路上の進んだ位置を探す範囲 [m]。前の周期の位置から後ろへ・先へ。1 周期（約 0.1 秒）に進むのは
  // 速さの上限 3m/s でも 0.3m。新しい経路は機体の位置から始まるが、計画の遅れの間に先へ進んでいる分を見込む
  static constexpr double kProgressBackM = 0.5;
  static constexpr double kProgressAheadM = 1.0;
  static constexpr double kProgressInitialAheadM = 2.0;
  int yaw_mode_{kTurretYawMode};
  bool follow_yaw_mode_{true};
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr yaw_mode_subscription_;
  // 操作の上半身の回し（/control/yaw_target_rate）を見て、動いている間は自律移動が向きを決めない
  static constexpr double kOperatorRateThreshold = 1e-3;
  double operator_hold_s_{1.0};
  std::optional<std::chrono::steady_clock::time_point> last_operator_active_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr operator_rate_subscription_;
  // 運転モード（/control/mode）が auto か。auto でなければ上半身を回す速さを出さない
  bool auto_mode_{false};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_mode_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr auto_yaw_rate_publisher_;
  // 足回りの面をそろえる設定と、この周期の TURRET の状態・足回りを回す速さ
  ChassisFaceParams chassis_face_params_;
  bool turret_cycle_{false};
  bool turret_auto_cycle_{false};
  double chassis_rate_cmd_{0.0};
  // 直前に target_angle_node へ出した上半身の回る速さ（/control/auto/yaw_target_rate）
  double upper_rate_cmd_{0.0};
  // この周期に測った上半身の回る速さと、TURRET で指令の向きに見込む遅れ（heading.turret_command_lead_s）
  double upper_rate_measured_{0.0};
  double turret_command_lead_s_{0.0};
  // 計画に使った上半身の向きの時刻と、時刻つきの指令を出す先
  double state_time_s_{0.0};
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr command_frame_publisher_;
  // 今の経路の上で、機体が進んだ位置（odom の経路の始点からの距離）。経路を受けたら消す
  std::optional<double> progress_s_;
  GoalHoldMonitor goal_hold_monitor_;
  OncomingMonitor oncoming_monitor_;
  bool oncoming_yielding_{false};
  TrackedObjectArray::ConstSharedPtr tracks_;
  LocalizationHealth::ConstSharedPtr health_;
  // 同じスキャンの追跡結果を待っている姿勢
  Odometry::ConstSharedPtr pending_odometry_;
  rclcpp::TimerBase::SharedPtr tracks_wait_timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ChassisTracker chassis_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_validation_;
  core_nav_common::ParameterUpdateHandle parameter_callback_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<NavigationStatus>::SharedPtr status_publisher_;
  rclcpp::Publisher<Path>::SharedPtr local_plan_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr stuck_point_publisher_;
  rclcpp::Subscription<Path>::SharedPtr plan_subscription_;
  rclcpp::Subscription<TrackedObjectArray>::SharedPtr tracks_subscription_;
  rclcpp::Subscription<LocalizationHealth>::SharedPtr health_subscription_;
  rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sent_command_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr traction_publisher_;
};
}  // namespace core_planning

RCLCPP_COMPONENTS_REGISTER_NODE(core_planning::MppiLiteNode)
