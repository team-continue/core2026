#include "core_localization/point_extraction_stage.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer_interface.h>

#include "core_nav_common/field_bounds.hpp"
#include "core_nav_common/parameters.hpp"

namespace core_localization
{
namespace
{
using core_msgs::msg::ProcessStats;
using core_msgs::msg::TrackedObject;
using core_msgs::msg::TrackedObjectArray;
using ParameterLookup = std::function<rclcpp::Parameter(const std::string &)>;

constexpr const char * kPrefix = "point_extraction.";
constexpr const char * kFieldPrefix = "field.";
// 同じ警告を出す間隔 [ms]
constexpr int kWarnPeriodMs = 5000;

void declareDoubleArray(
  rclcpp::Node & node, const std::string & name, const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  node.declare_parameter<std::vector<double>>(name, std::vector<double>{}, descriptor);
}

void declareStringArray(rclcpp::Node & node, const std::string & name, const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  node.declare_parameter<std::vector<std::string>>(name, std::vector<std::string>{}, descriptor);
}

void declareParams(rclcpp::Node & node)
{
  using core_nav_common::declareBool;
  using core_nav_common::declareDouble;
  using core_nav_common::declareInt;
  using core_nav_common::declareString;
  const PointExtractionStageParams defaults;
  const auto & ceiling = defaults.extraction.ceiling;
  const auto & field = defaults.extraction.field_objects;
  const auto & tracks = field.tracks;
  const core_nav_common::FieldBounds bounds;
  declareString(
    node, "point_extraction.mode", "off",
    "点群抽出のモード（off: かけない / ceiling: 床と人の頭より上だけ / field_objects: フィールドとその周りだけ）");
  declareString(
    node, "point_extraction.floor_frame", defaults.floor_frame, "床からの高さを求める座標系（z 軸が鉛直）");
  declareDouble(
    node, "point_extraction.floor_z_m", defaults.floor_z_m, -100.0, 100.0, "床の座標系での床の高さ [m]");
  declareString(
    node, "point_extraction.map_frame", defaults.map_frame, "観客席の箱とフィールドの範囲の座標系");
  declareString(
    node, "point_extraction.odom_frame", defaults.odom_frame,
    "map の変換を求める間は動かないとみなす座標系");
  declareDouble(
    node, "point_extraction.tf_wait_s", defaults.tf_wait_s, 0.0, 1.0,
    "点群の時刻の TF を待つ時間の上限 [s]（実時間）");

  declareDouble(
    node, "point_extraction.ceiling.band_min_m", ceiling.band_min_m, -10.0, 50.0,
    "ceiling: 人の帯の下限 [m]。これ未満の床の点は残す");
  declareDouble(
    node, "point_extraction.ceiling.band_max_m", ceiling.band_max_m, -10.0, 50.0,
    "ceiling: 人の帯の上限 [m]。これを超える人の頭より上の点は残す");
  declareBool(
    node, "point_extraction.ceiling.outside.enabled", ceiling.outside.enabled,
    "ceiling: 柵の外（フィールドの範囲 field.* に余裕を足した長方形の外）では、人の帯の上限を広げる");
  declareDouble(
    node, "point_extraction.ceiling.outside.field_margin_m", ceiling.outside.field_margin_m, 0.0, 50.0,
    "ceiling: フィールドの範囲（field.*）に足す余裕 [m]。これより外を観客がいる所とみなす");
  declareDouble(
    node, "point_extraction.ceiling.outside.band_max_m", ceiling.outside.band_max_m, -10.0, 50.0,
    "ceiling: 柵の外の人の帯の上限 [m]。下限は band_min_m と同じ");
  declareStringArray(node, "point_extraction.ceiling.region.names", "ceiling: 観客席の箱の名前");
  declareDoubleArray(
    node, "point_extraction.ceiling.region.boxes",
    "ceiling: 観客席の箱ごとに 6 個（x_min, x_max, y_min, y_max, z_min, z_max）[m] を並べた値");
  declareInt(
    node, "point_extraction.ceiling.min_points", static_cast<std::int64_t>(ceiling.min_points), 0,
    10000000, "ceiling: 除いた後の点がこれより少なければ、その周期は抽出をかけない");

  declareDouble(
    node, "point_extraction.field_objects.field_margin_m", field.field_margin_m, 0.0, 10.0,
    "field_objects: フィールドの範囲（field.*）に足す余裕 [m]");
  declareDouble(
    node, "point_extraction.field_objects.max_height_m", field.max_height_m, 0.01, 50.0,
    "field_objects: 床からの高さの上限 [m]。これより上を除く");
  declareInt(
    node, "point_extraction.field_objects.min_points", static_cast<std::int64_t>(field.min_points), 0,
    10000000, "field_objects: 除いた後の点がこれより少なければ、その周期は抽出をかけない");
  declareBool(
    node, "point_extraction.field_objects.tracks.enabled", tracks.enabled,
    "field_objects: 追跡した相手の柱の中の点を除く");
  declareString(
    node, "point_extraction.field_objects.tracks.topic", defaults.tracks_topic, "追跡のトピック", true);
  declareDouble(
    node, "point_extraction.field_objects.tracks.human_radius_m", tracks.human_radius_m, 0.0, 5.0,
    "人の半径 [m]");
  declareDouble(
    node, "point_extraction.field_objects.tracks.robot_radius_m", tracks.robot_radius_m, 0.0, 5.0,
    "ロボットの半径 [m]");
  declareDouble(
    node, "point_extraction.field_objects.tracks.unknown_radius_m", tracks.unknown_radius_m, 0.0, 5.0,
    "種類の分からない物の半径 [m]");
  declareDouble(
    node, "point_extraction.field_objects.tracks.margin_m", tracks.margin_m, 0.0, 5.0,
    "半径に足す余裕 [m]");
  declareDouble(
    node, "point_extraction.field_objects.tracks.max_age_s", tracks.max_age_s, 0.01, 10.0,
    "追跡の時刻と点群の時刻の差の上限 [s]。これより離れた追跡は使わない");

  // フィールドの範囲は検出と同じ field.yaml から受け取る
  declareDouble(node, "field.x_min", bounds.x_min, -1000.0, 1000.0, "フィールドの範囲（柵の内側の面）");
  declareDouble(node, "field.x_max", bounds.x_max, -1000.0, 1000.0, "フィールドの範囲（柵の内側の面）");
  declareDouble(node, "field.y_min", bounds.y_min, -1000.0, 1000.0, "フィールドの範囲（柵の内側の面）");
  declareDouble(node, "field.y_max", bounds.y_max, -1000.0, 1000.0, "フィールドの範囲（柵の内側の面）");
}

float asFloat(const rclcpp::Parameter & parameter)
{
  return static_cast<float>(parameter.as_double());
}

PointExtractionStageParams readParams(const ParameterLookup & get)
{
  PointExtractionStageParams params;
  auto & extraction = params.extraction;
  extraction.mode = point_extraction::parseMode(get("point_extraction.mode").as_string());
  params.floor_frame = get("point_extraction.floor_frame").as_string();
  params.floor_z_m = asFloat(get("point_extraction.floor_z_m"));
  params.map_frame = get("point_extraction.map_frame").as_string();
  params.odom_frame = get("point_extraction.odom_frame").as_string();
  params.tf_wait_s = get("point_extraction.tf_wait_s").as_double();

  auto & ceiling = extraction.ceiling;
  ceiling.band_min_m = asFloat(get("point_extraction.ceiling.band_min_m"));
  ceiling.band_max_m = asFloat(get("point_extraction.ceiling.band_max_m"));
  ceiling.regions = crowd::makeRegions(
    get("point_extraction.ceiling.region.names").as_string_array(),
    get("point_extraction.ceiling.region.boxes").as_double_array());
  ceiling.min_points = static_cast<std::size_t>(get("point_extraction.ceiling.min_points").as_int());
  ceiling.outside.enabled = get("point_extraction.ceiling.outside.enabled").as_bool();
  ceiling.outside.field = point_extraction::Area{
    asFloat(get("field.x_min")), asFloat(get("field.x_max")), asFloat(get("field.y_min")),
    asFloat(get("field.y_max"))};
  ceiling.outside.field_margin_m = asFloat(get("point_extraction.ceiling.outside.field_margin_m"));
  ceiling.outside.band_max_m = asFloat(get("point_extraction.ceiling.outside.band_max_m"));

  auto & field = extraction.field_objects;
  field.field = point_extraction::Area{
    asFloat(get("field.x_min")), asFloat(get("field.x_max")), asFloat(get("field.y_min")),
    asFloat(get("field.y_max"))};
  field.field_margin_m = asFloat(get("point_extraction.field_objects.field_margin_m"));
  field.max_height_m = asFloat(get("point_extraction.field_objects.max_height_m"));
  field.min_points =
    static_cast<std::size_t>(get("point_extraction.field_objects.min_points").as_int());
  auto & tracks = field.tracks;
  tracks.enabled = get("point_extraction.field_objects.tracks.enabled").as_bool();
  tracks.human_radius_m = asFloat(get("point_extraction.field_objects.tracks.human_radius_m"));
  tracks.robot_radius_m = asFloat(get("point_extraction.field_objects.tracks.robot_radius_m"));
  tracks.unknown_radius_m = asFloat(get("point_extraction.field_objects.tracks.unknown_radius_m"));
  tracks.margin_m = asFloat(get("point_extraction.field_objects.tracks.margin_m"));
  tracks.max_age_s = get("point_extraction.field_objects.tracks.max_age_s").as_double();
  params.tracks_topic = get("point_extraction.field_objects.tracks.topic").as_string();

  point_extraction::validate(extraction);
  const std::vector<std::pair<const char *, const std::string *>> frames{
    {"point_extraction.floor_frame", &params.floor_frame},
    {"point_extraction.map_frame", &params.map_frame},
    {"point_extraction.odom_frame", &params.odom_frame},
    {"point_extraction.field_objects.tracks.topic", &params.tracks_topic}};
  for (const auto & [name, frame] : frames) {
    if (frame->empty()) {
      throw std::invalid_argument(std::string(name) + " を空にはできません");
    }
  }
  return params;
}

bool touchesParams(const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & parameter : parameters) {
    const std::string & name = parameter.get_name();
    if (name.rfind(kPrefix, 0) == 0U || name.rfind(kFieldPrefix, 0) == 0U) {
      return true;
    }
  }
  return false;
}

std::string regionNames(const std::vector<crowd::Region> & regions)
{
  std::ostringstream names;
  for (std::size_t i = 0; i < regions.size(); ++i) {
    names << (i == 0U ? "" : "、") << regions[i].name;
  }
  return names.str();
}

point_extraction::TrackClass trackClass(const std::uint8_t classification)
{
  switch (classification) {
    case TrackedObject::CLASS_HUMAN:
      return point_extraction::TrackClass::kHuman;
    case TrackedObject::CLASS_ROBOT:
      return point_extraction::TrackClass::kRobot;
    default:
      return point_extraction::TrackClass::kUnknown;
  }
}

bool usesTracks(const point_extraction::Params & params)
{
  return params.mode == point_extraction::Mode::kFieldObjects && params.field_objects.tracks.enabled;
}

bool usesMap(const point_extraction::Params & params)
{
  return params.mode == point_extraction::Mode::kFieldObjects ||
         (params.mode == point_extraction::Mode::kCeiling &&
         (!params.ceiling.regions.empty() || params.ceiling.outside.enabled));
}
}  // namespace

PointExtractionStage::PointExtractionStage(rclcpp::Node & node)
: node_(node)
{
  declareParams(node_);
  try {
    params_ = readParams([this](const std::string & name) {return node_.get_parameter(name);});
  } catch (const std::exception & error) {
    throw std::invalid_argument(std::string("点群抽出の設定が正しくありません: ") + error.what());
  }
  // 変更は、今の値に新しい値を重ねて組み合わせを確かめてから受け入れる
  validate_callback_ = node_.add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      if (!touchesParams(parameters)) {
        return result;
      }
      try {
        (void)readParams(
          [this, &parameters](const std::string & name) {
            for (const auto & parameter : parameters) {
              if (parameter.get_name() == name) {
                return parameter;
              }
            }
            return node_.get_parameter(name);
          });
      } catch (const std::exception & error) {
        result.successful = false;
        result.reason = std::string("点群抽出の設定が正しくありません: ") + error.what();
      }
      return result;
    });
  update_callback_ = core_nav_common::onParametersSet(
    node_, [this](const std::vector<rclcpp::Parameter> & parameters) {
      if (!touchesParams(parameters)) {
        return;
      }
      params_ = readParams([this](const std::string & name) {return node_.get_parameter(name);});
      logSettings();
    });
  stats_publisher_ = node_.create_publisher<ProcessStats>("~/point_extraction_stats", rclcpp::QoS(10));
  logSettings();
}

bool PointExtractionStage::active() const
{
  return params_.extraction.mode != point_extraction::Mode::kOff;
}

point_extraction::Result PointExtractionStage::extract(
  const std::vector<Eigen::Vector3f> & positions, const std::string & cloud_frame,
  const rclcpp::Time & stamp)
{
  ensureTf();
  if (usesTracks(params_.extraction)) {
    ensureTracks();
  }
  if (cloud_frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), warn_clock_, kWarnPeriodMs,
      "点群の座標系の名前が空なので、点群抽出をかけません");
    point_extraction::Params off = params_.extraction;
    off.mode = point_extraction::Mode::kOff;
    return point_extraction::extract(positions, off, {}, {});
  }
  const auto cylinders =
    usesTracks(params_.extraction) ? trackCylinders(stamp) : std::nullopt;
  point_extraction::Transforms transforms = lookupTransforms(cloud_frame, stamp);
  if (cylinders.has_value() && !cylinders->empty()) {
    const std::string & tracks_frame = latest_tracks_->header.frame_id;
    transforms.tracks_from_cloud = lookupAtCloud(tracks_frame, cloud_frame, stamp, "追跡の柱");
  }
  const point_extraction::Result result = point_extraction::extract(
    positions, params_.extraction, transforms,
    cylinders.value_or(std::vector<point_extraction::Cylinder>{}));
  report(result);
  return result;
}

point_extraction::Transforms PointExtractionStage::lookupTransforms(
  const std::string & cloud_frame, const rclcpp::Time & stamp)
{
  // 点群の時刻の odom←点群は FAST-LIO の出力と同時に出るが、点群より少し遅れて届くことがある
  waitForCloudPose(cloud_frame, stamp);
  point_extraction::Transforms transforms;
  const auto floor_from_cloud = lookupAtCloud(params_.floor_frame, cloud_frame, stamp, "床からの高さ");
  if (floor_from_cloud.has_value()) {
    transforms.height_from_cloud =
      Eigen::Translation3f(0.0F, 0.0F, -params_.floor_z_m) * *floor_from_cloud;
  }
  if (usesMap(params_.extraction)) {
    transforms.map_from_cloud = lookupAtCloud(
      params_.map_frame, cloud_frame, stamp,
      params_.extraction.mode == point_extraction::Mode::kCeiling ? "観客席の箱" : "フィールドの範囲");
  }
  return transforms;
}

void PointExtractionStage::waitForCloudPose(
  const std::string & cloud_frame, const rclcpp::Time & stamp) const
{
  const tf2::TimePoint time = tf2_ros::fromRclcpp(stamp);
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(params_.tf_wait_s));
  // 理由の文字列を渡さないと、tf2 が座標系のないことを呼ぶたびに警告する
  std::string reason;
  while (!tf_buffer_->canTransform(params_.odom_frame, cloud_frame, time, &reason) &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

std::optional<Eigen::Isometry3f> PointExtractionStage::lookupAtCloud(
  const std::string & target, const std::string & cloud_frame, const rclcpp::Time & stamp,
  const char * purpose)
{
  // target が odom なら、点群の時刻の変換そのものになる
  const tf2::TimePoint time = tf2_ros::fromRclcpp(stamp);
  try {
    return tf2::transformToEigen(
      tf_buffer_->lookupTransform(
        target, tf2::TimePointZero, cloud_frame, time, params_.odom_frame)).cast<float>();
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), warn_clock_, kWarnPeriodMs,
      "点群抽出の%sに使う、時刻 %.3f の %s から %s への変換を取得できません: %s", purpose,
      stamp.seconds(), cloud_frame.c_str(), target.c_str(), error.what());
    return std::nullopt;
  }
}

std::optional<std::vector<point_extraction::Cylinder>> PointExtractionStage::trackCylinders(
  const rclcpp::Time & stamp)
{
  if (latest_tracks_ == nullptr) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), warn_clock_, kWarnPeriodMs,
      "追跡（%s）をまだ受け取っていないので、点群抽出は追跡の柱をかけません",
      params_.tracks_topic.c_str());
    return std::nullopt;
  }
  const double dt_s = (stamp - rclcpp::Time(latest_tracks_->header.stamp, stamp.get_clock_type()))
    .seconds();
  const double max_age_s = params_.extraction.field_objects.tracks.max_age_s;
  if (!std::isfinite(dt_s) || std::abs(dt_s) > max_age_s) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), warn_clock_, kWarnPeriodMs,
      "追跡の時刻が点群と %.3f 秒離れていて上限 %.3f 秒を超えるので、点群抽出は追跡の柱をかけません",
      dt_s, max_age_s);
    return std::nullopt;
  }
  std::vector<point_extraction::Track> tracks;
  tracks.reserve(latest_tracks_->objects.size());
  for (const auto & object : latest_tracks_->objects) {
    tracks.push_back(
      point_extraction::Track{
        Eigen::Vector2f(static_cast<float>(object.position.x), static_cast<float>(object.position.y)),
        Eigen::Vector2f(static_cast<float>(object.velocity.x), static_cast<float>(object.velocity.y)),
        trackClass(object.classification)});
  }
  return point_extraction::predictCylinders(tracks, dt_s, params_.extraction.field_objects.tracks);
}

void PointExtractionStage::ensureTf()
{
  if (tf_buffer_ != nullptr) {
    return;
  }
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_.get_clock());
  // 専用のスレッドで TF を受けるので、点群の処理の中で待てる
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

void PointExtractionStage::ensureTracks()
{
  if (tracks_subscription_ != nullptr) {
    return;
  }
  // 点群と同じ実行器で受けるので、処理中に書き換わらない
  tracks_subscription_ = node_.create_subscription<TrackedObjectArray>(
    params_.tracks_topic, rclcpp::QoS(10),
    [this](const TrackedObjectArray::ConstSharedPtr & tracks) {latest_tracks_ = tracks;});
}

void PointExtractionStage::report(const point_extraction::Result & result)
{
  if (result.passed_without_transform) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), warn_clock_, kWarnPeriodMs,
      "床（%s）か map（%s）への変換がないので、この周期は点群抽出をかけずに全点を通します",
      params_.floor_frame.c_str(), params_.map_frame.c_str());
  }
  if (result.band_skipped) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), warn_clock_, kWarnPeriodMs,
      "床からの高さを求める変換がないので、この周期は点群抽出の人の帯をかけません");
  }
  if (result.region_skipped) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), warn_clock_, kWarnPeriodMs,
      "%s への変換がないので、この周期は点群抽出の観客席の箱をかけません", params_.map_frame.c_str());
  }
  if (result.outside_band_skipped) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), warn_clock_, kWarnPeriodMs,
      "床か %s への変換がないので、この周期は点群抽出の柵の外の帯をかけません",
      params_.map_frame.c_str());
  }
  if (result.tracks_skipped) {
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), warn_clock_, kWarnPeriodMs,
      "追跡の座標系への変換がないので、この周期は点群抽出の追跡の柱をかけません");
  }
  if (result.fallback) {
    ++fallback_count_;
    const bool ceiling = params_.extraction.mode == point_extraction::Mode::kCeiling;
    const std::size_t removed = result.removed_band + result.removed_region +
      result.removed_outside_band + result.removed_outside + result.removed_above +
      result.removed_track;
    RCLCPP_WARN_THROTTLE(
      node_.get_logger(), warn_clock_, kWarnPeriodMs,
      "点群抽出（%s）で残る点が %zu 点（入力 %zu 点、帯 %zu、観客席 %zu、柵の外の帯 %zu、範囲の外 %zu、"
      "上限より上 %zu、追跡 %zu）で min_points %zu より少ないので、この周期は抽出をかけません（累計 %lu 周期）",
      point_extraction::modeName(params_.extraction.mode).c_str(), result.input - removed,
      result.input, result.removed_band, result.removed_region, result.removed_outside_band,
      result.removed_outside, result.removed_above, result.removed_track,
      ceiling ? params_.extraction.ceiling.min_points : params_.extraction.field_objects.min_points,
      static_cast<unsigned long>(fallback_count_));  // NOLINT(runtime/int)
  }
}

void PointExtractionStage::publishStats(
  const point_extraction::Result & result, const rclcpp::Time & stamp,
  const std::chrono::steady_clock::time_point start)
{
  ProcessStats stats;
  stats.header.stamp = stamp;
  stats.input_points = static_cast<std::uint32_t>(result.input);
  stats.output_points = static_cast<std::uint32_t>(result.kept);
  // TF を待った時間も含む
  stats.processing_ms = std::chrono::duration<float, std::milli>(
    std::chrono::steady_clock::now() - start).count();
  // 抽出をかけなかった周期（点が少ない、変換がない）の累計
  stats.skipped_frames = static_cast<std::uint32_t>(fallback_count_);
  stats_publisher_->publish(stats);
}

void PointExtractionStage::logSettings() const
{
  const auto & extraction = params_.extraction;
  switch (extraction.mode) {
    case point_extraction::Mode::kOff:
      RCLCPP_INFO(node_.get_logger(), "点群抽出: off（かけない）");
      return;
    case point_extraction::Mode::kCeiling: {
        const auto & ceiling = extraction.ceiling;
        RCLCPP_INFO(
          node_.get_logger(),
          "点群抽出: ceiling（床と人の頭より上だけを残す）。人の帯 [%.2f, %.2f]m（%s で床 %.2fm）、"
          "観客席の箱 %zu 個（%s で %s）、min_points %zu",
          ceiling.band_min_m, ceiling.band_max_m, params_.floor_frame.c_str(), params_.floor_z_m,
          ceiling.regions.size(), params_.map_frame.c_str(), regionNames(ceiling.regions).c_str(),
          ceiling.min_points);
        if (ceiling.outside.enabled) {
          const point_extraction::Area boundary = point_extraction::outsideBoundary(ceiling.outside);
          RCLCPP_INFO(
            node_.get_logger(),
            "点群抽出: 柵の外（%s で x [%.2f, %.2f] y [%.2f, %.2f]m の外。field.* に余裕 %.2fm）は、"
            "人の帯を [%.2f, %.2f]m に広げます",
            params_.map_frame.c_str(), boundary.x_min, boundary.x_max, boundary.y_min, boundary.y_max,
            ceiling.outside.field_margin_m, ceiling.band_min_m, ceiling.outside.band_max_m);
        }
        if (ceiling.regions.empty()) {
          RCLCPP_WARN(
            node_.get_logger(),
            "点群抽出の観客席の箱（point_extraction.ceiling.region.boxes）が空なので、観客席の箱は除きません");
        }
        return;
      }
    case point_extraction::Mode::kFieldObjects: {
        const auto & field = extraction.field_objects;
        const point_extraction::Area area = point_extraction::keepArea(field);
        const auto & tracks = field.tracks;
        RCLCPP_INFO(
          node_.get_logger(),
          "点群抽出: field_objects（フィールドとその周りだけを残す）。残す範囲 x [%.2f, %.2f] y [%.2f, %.2f]m"
          "（%s、field.* に余裕 %.2fm）、床から %.2fm まで（%s で床 %.2fm）、追跡の柱 %s（%s、人 %.2fm・"
          "ロボット %.2fm・不明 %.2fm に余裕 %.2fm、時刻の差 %.2f 秒まで）、min_points %zu",
          area.x_min, area.x_max, area.y_min, area.y_max, params_.map_frame.c_str(),
          field.field_margin_m, field.max_height_m, params_.floor_frame.c_str(), params_.floor_z_m,
          tracks.enabled ? "有効" : "無効", params_.tracks_topic.c_str(), tracks.human_radius_m,
          tracks.robot_radius_m, tracks.unknown_radius_m, tracks.margin_m, tracks.max_age_s,
          field.min_points);
        return;
      }
  }
}
}  // namespace core_localization
