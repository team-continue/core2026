#include "core_localization/point_extraction.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace core_localization::point_extraction
{
namespace
{
enum class Reason : std::uint8_t {kKept, kOutside, kAbove, kTrack};

std::string text(const float value)
{
  return std::to_string(value);
}

void requireFinite(const float value, const std::string & what)
{
  if (!std::isfinite(value)) {
    throw std::invalid_argument(what + " が有限の値ではありません（" + text(value) + "）");
  }
}

void requireNonNegative(const float value, const std::string & what)
{
  requireFinite(value, what);
  if (value < 0.0F) {
    throw std::invalid_argument(what + " は 0 以上にしてください（" + text(value) + "）");
  }
}

float radiusOf(const TrackClass classification, const TrackParams & params)
{
  switch (classification) {
    case TrackClass::kHuman:
      return params.human_radius_m;
    case TrackClass::kRobot:
      return params.robot_radius_m;
    case TrackClass::kUnknown:
      break;
  }
  return params.unknown_radius_m;
}

bool insideAnyCylinder(const std::vector<Cylinder> & cylinders, const Eigen::Vector2f & point)
{
  for (const auto & cylinder : cylinders) {
    if ((point - cylinder.center).squaredNorm() <= cylinder.radius_m * cylinder.radius_m) {
      return true;
    }
  }
  return false;
}

Result passAll(const std::size_t count)
{
  Result result;
  result.input = count;
  result.kept = count;
  result.keep.assign(count, 1U);
  return result;
}

bool outsideOf(const Area & area, const Eigen::Vector3f & map)
{
  return map.x() < area.x_min || map.x() > area.x_max || map.y() < area.y_min ||
         map.y() > area.y_max;
}

Result extractCeiling(
  const std::vector<Eigen::Vector3f> & points, const CeilingParams & params,
  const Transforms & transforms)
{
  // 帯と領域の計算は人混みのフィルタ（crowd_filter.hpp）と同じ。自機の点は FAST-LIO の前処理が
  // 機体の箱で除くので、ここでは使わない。柵の外の帯を足してから全体で fallback を決めるので、
  // classify の fallback は使わない（min_points 0）
  crowd::Params crowd_params;
  crowd_params.self_enabled = false;
  crowd_params.height_band_enabled = true;
  crowd_params.height_min_m = params.band_min_m;
  crowd_params.height_max_m = params.band_max_m;
  crowd_params.region_enabled = !params.regions.empty();
  crowd_params.regions = params.regions;
  crowd_params.min_points = 0U;
  crowd::Transforms crowd_transforms;
  crowd_transforms.height_from_cloud = transforms.height_from_cloud;
  crowd_transforms.map_from_cloud = transforms.map_from_cloud;
  const crowd::Result crowd_result = crowd::classify(points, crowd_params, crowd_transforms);

  Result result;
  result.input = crowd_result.input;
  result.removed_band = crowd_result.removed_height;
  result.removed_region = crowd_result.removed_region;
  result.band_skipped = crowd_result.height_skipped;
  result.region_skipped = crowd_result.region_skipped;
  std::vector<std::uint8_t> keep = crowd_result.keep;

  // 柵の外の帯は、床からの高さと map の位置の両方がいる
  const bool use_outside = params.outside.enabled && transforms.height_from_cloud.has_value() &&
    transforms.map_from_cloud.has_value();
  result.outside_band_skipped = params.outside.enabled && !use_outside;
  if (use_outside) {
    const Area boundary = outsideBoundary(params.outside);
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (keep[i] == 0U || !outsideOf(boundary, *transforms.map_from_cloud * points[i])) {
        continue;
      }
      const float height = (*transforms.height_from_cloud * points[i]).z();
      if (height >= params.band_min_m && height <= params.outside.band_max_m) {
        keep[i] = 0U;
        ++result.removed_outside_band;
      }
    }
  }

  const std::size_t removed =
    result.removed_band + result.removed_region + result.removed_outside_band;
  const std::size_t kept = result.input - removed;
  result.fallback = removed > 0U && kept < params.min_points;
  result.keep = result.fallback ? std::vector<std::uint8_t>(points.size(), 1U) : std::move(keep);
  result.kept = result.fallback ? result.input : kept;
  return result;
}

Result extractFieldObjects(
  const std::vector<Eigen::Vector3f> & points, const FieldObjectsParams & params,
  const Transforms & transforms, const std::vector<Cylinder> & cylinders)
{
  // 範囲と高さはこのモードの中心なので、変換がなければその周期は全点を通す
  if (!transforms.height_from_cloud.has_value() || !transforms.map_from_cloud.has_value()) {
    Result result = passAll(points.size());
    result.passed_without_transform = true;
    return result;
  }
  const Area area = keepArea(params);
  const bool has_cylinders = params.tracks.enabled && !cylinders.empty();
  const bool use_tracks = has_cylinders && transforms.tracks_from_cloud.has_value();
  const Eigen::Isometry3f & map_from_cloud = *transforms.map_from_cloud;
  const Eigen::Isometry3f & height_from_cloud = *transforms.height_from_cloud;

  Result result;
  result.input = points.size();
  result.tracks_skipped = has_cylinders && !use_tracks;
  std::vector<Reason> reasons(points.size(), Reason::kKept);
  for (std::size_t i = 0; i < points.size(); ++i) {
    const Eigen::Vector3f & point = points[i];
    const Eigen::Vector3f map = map_from_cloud * point;
    if (map.x() < area.x_min || map.x() > area.x_max || map.y() < area.y_min ||
      map.y() > area.y_max)
    {
      reasons[i] = Reason::kOutside;
      continue;
    }
    if ((height_from_cloud * point).z() > params.max_height_m) {
      reasons[i] = Reason::kAbove;
      continue;
    }
    if (use_tracks &&
      insideAnyCylinder(cylinders, (*transforms.tracks_from_cloud * point).head<2>()))
    {
      reasons[i] = Reason::kTrack;
    }
  }
  for (const Reason reason : reasons) {
    result.removed_outside += reason == Reason::kOutside ? 1U : 0U;
    result.removed_above += reason == Reason::kAbove ? 1U : 0U;
    result.removed_track += reason == Reason::kTrack ? 1U : 0U;
  }
  const std::size_t removed = result.removed_outside + result.removed_above + result.removed_track;
  const std::size_t kept = result.input - removed;
  result.fallback = removed > 0U && kept < params.min_points;
  result.keep.resize(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    result.keep[i] = (result.fallback || reasons[i] == Reason::kKept) ? 1U : 0U;
  }
  result.kept = result.fallback ? result.input : kept;
  return result;
}
}  // namespace

Mode parseMode(const std::string & text)
{
  if (text == "off") {
    return Mode::kOff;
  }
  if (text == "ceiling") {
    return Mode::kCeiling;
  }
  if (text == "field_objects") {
    return Mode::kFieldObjects;
  }
  throw std::invalid_argument(
          "点群抽出のモードは off、ceiling、field_objects のどれかにしてください（" + text + "）");
}

std::string modeName(const Mode mode)
{
  switch (mode) {
    case Mode::kCeiling:
      return "ceiling";
    case Mode::kFieldObjects:
      return "field_objects";
    case Mode::kOff:
      break;
  }
  return "off";
}

Area keepArea(const FieldObjectsParams & params)
{
  const Area & field = params.field;
  for (const float value : {field.x_min, field.x_max, field.y_min, field.y_max}) {
    requireFinite(value, "フィールドの範囲");
  }
  if (!(field.x_min < field.x_max) || !(field.y_min < field.y_max)) {
    throw std::invalid_argument(
            "フィールドの範囲は x_min < x_max、y_min < y_max にしてください（x [" + text(field.x_min) +
            ", " + text(field.x_max) + "]、y [" + text(field.y_min) + ", " + text(field.y_max) + "]）");
  }
  requireNonNegative(params.field_margin_m, "残す範囲の余裕（field_margin_m）");
  return Area{
    field.x_min - params.field_margin_m, field.x_max + params.field_margin_m,
    field.y_min - params.field_margin_m, field.y_max + params.field_margin_m};
}

Area outsideBoundary(const OutsideBand & params)
{
  const Area & field = params.field;
  for (const float value : {field.x_min, field.x_max, field.y_min, field.y_max}) {
    requireFinite(value, "柵の外の帯のフィールドの範囲");
  }
  if (!(field.x_min < field.x_max) || !(field.y_min < field.y_max)) {
    throw std::invalid_argument(
            "柵の外の帯のフィールドの範囲は x_min < x_max、y_min < y_max にしてください（x [" +
            text(field.x_min) + ", " + text(field.x_max) + "]、y [" + text(field.y_min) + ", " +
            text(field.y_max) + "]）");
  }
  requireNonNegative(params.field_margin_m, "柵の外の帯の余裕（ceiling.outside.field_margin_m）");
  return Area{
    field.x_min - params.field_margin_m, field.x_max + params.field_margin_m,
    field.y_min - params.field_margin_m, field.y_max + params.field_margin_m};
}

std::vector<Cylinder> predictCylinders(
  const std::vector<Track> & tracks, const double dt_s, const TrackParams & params)
{
  std::vector<Cylinder> cylinders;
  cylinders.reserve(tracks.size());
  const float dt = static_cast<float>(dt_s);
  for (const auto & track : tracks) {
    cylinders.push_back(
      Cylinder{track.position + track.velocity * dt,
        radiusOf(track.classification, params) + params.margin_m});
  }
  return cylinders;
}

Result extract(
  const std::vector<Eigen::Vector3f> & points, const Params & params,
  const Transforms & transforms, const std::vector<Cylinder> & cylinders)
{
  switch (params.mode) {
    case Mode::kCeiling:
      return extractCeiling(points, params.ceiling, transforms);
    case Mode::kFieldObjects:
      return extractFieldObjects(points, params.field_objects, transforms, cylinders);
    case Mode::kOff:
      break;
  }
  return passAll(points.size());
}

void validate(const Params & params)
{
  const CeilingParams & ceiling = params.ceiling;
  requireFinite(ceiling.band_min_m, "人の帯の下限（ceiling.band_min_m）");
  requireFinite(ceiling.band_max_m, "人の帯の上限（ceiling.band_max_m）");
  if (ceiling.band_min_m > ceiling.band_max_m) {
    throw std::invalid_argument(
            "人の帯の下限 " + text(ceiling.band_min_m) + "m が上限 " + text(ceiling.band_max_m) +
            "m を超えています");
  }
  crowd::Params crowd_params;
  crowd_params.regions = ceiling.regions;
  crowd::validate(crowd_params);
  // 柵の外の帯は、使うときだけ確かめる（使わない値で ceiling の帯を変えられなくならないように）
  if (ceiling.outside.enabled) {
    (void)outsideBoundary(ceiling.outside);
    requireFinite(ceiling.outside.band_max_m, "柵の外の帯の上限（ceiling.outside.band_max_m）");
    // 柵の外ではふつうの帯より広げるための値なので、ふつうの帯の上限より低くはしない
    if (ceiling.outside.band_max_m < ceiling.band_max_m) {
      throw std::invalid_argument(
              "柵の外の帯の上限 " + text(ceiling.outside.band_max_m) + "m が、人の帯の上限 " +
              text(ceiling.band_max_m) + "m より低くなっています");
    }
  }

  const FieldObjectsParams & field = params.field_objects;
  (void)keepArea(field);
  requireFinite(field.max_height_m, "高さの上限（field_objects.max_height_m）");
  if (!(field.max_height_m > 0.0F)) {
    throw std::invalid_argument(
            "高さの上限（field_objects.max_height_m）は正の値にしてください（" +
            text(field.max_height_m) + "）");
  }
  const TrackParams & tracks = field.tracks;
  requireNonNegative(tracks.human_radius_m, "人の半径（tracks.human_radius_m）");
  requireNonNegative(tracks.robot_radius_m, "ロボットの半径（tracks.robot_radius_m）");
  requireNonNegative(tracks.unknown_radius_m, "種類の分からない物の半径（tracks.unknown_radius_m）");
  requireNonNegative(tracks.margin_m, "追跡の柱の余裕（tracks.margin_m）");
  if (!std::isfinite(tracks.max_age_s) || !(tracks.max_age_s > 0.0)) {
    throw std::invalid_argument(
            "追跡の時刻の差の上限（tracks.max_age_s）は正の値にしてください（" +
            std::to_string(tracks.max_age_s) + "）");
  }
}
}  // namespace core_localization::point_extraction
