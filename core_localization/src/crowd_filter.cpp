#include "core_localization/crowd_filter.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace core_localization::crowd
{
namespace
{
enum class Reason : std::uint8_t {kKept, kSelf, kHeight, kRegion};

constexpr std::size_t kValuesPerRegion = 6U;

std::string formatBox(const AxisBox & box)
{
  const auto range = [](const float low, const float high) {
      return "[" + std::to_string(low) + ", " + std::to_string(high) + "]";
    };
  return "x " + range(box.min.x(), box.max.x()) + "、y " + range(box.min.y(), box.max.y()) +
         "、z " + range(box.min.z(), box.max.z());
}

void checkBox(const AxisBox & box, const std::string & what)
{
  if (!box.min.allFinite() || !box.max.allFinite()) {
    throw std::invalid_argument(what + " に有限でない値があります（" + formatBox(box) + "）");
  }
  if ((box.min.array() > box.max.array()).any()) {
    throw std::invalid_argument(
            what + " の下限が上限を超えています（" + formatBox(box) + "）。下限、上限の順に書いてください");
  }
}

bool insideAny(const std::vector<Region> & regions, const Eigen::Vector3f & point)
{
  for (const auto & region : regions) {
    if (contains(region.box, point)) {
      return true;
    }
  }
  return false;
}
}  // namespace

bool contains(const AxisBox & box, const Eigen::Vector3f & point)
{
  return (point.array() >= box.min.array()).all() && (point.array() <= box.max.array()).all();
}

Result classify(
  const std::vector<Eigen::Vector3f> & points, const Params & params,
  const Transforms & transforms)
{
  const bool use_self = params.self_enabled && transforms.base_from_cloud.has_value();
  const bool use_height = params.height_band_enabled && transforms.height_from_cloud.has_value();
  const bool use_region = params.region_enabled && !params.regions.empty() &&
    transforms.map_from_cloud.has_value();

  Result result;
  result.input = points.size();
  result.self_skipped = params.self_enabled && !use_self;
  result.height_skipped = params.height_band_enabled && !use_height;
  result.region_skipped = params.region_enabled && !params.regions.empty() && !use_region;

  std::vector<Reason> reasons(points.size(), Reason::kKept);
  for (std::size_t i = 0; i < points.size(); ++i) {
    const Eigen::Vector3f & point = points[i];
    if (use_self && contains(params.self_box, *transforms.base_from_cloud * point)) {
      reasons[i] = Reason::kSelf;
      continue;
    }
    if (use_height) {
      const float height = (*transforms.height_from_cloud * point).z();
      if (height >= params.height_min_m && height <= params.height_max_m) {
        reasons[i] = Reason::kHeight;
        continue;
      }
    }
    if (use_region && insideAny(params.regions, *transforms.map_from_cloud * point)) {
      reasons[i] = Reason::kRegion;
    }
  }

  for (const Reason reason : reasons) {
    result.removed_self += reason == Reason::kSelf ? 1U : 0U;
    result.removed_height += reason == Reason::kHeight ? 1U : 0U;
    result.removed_region += reason == Reason::kRegion ? 1U : 0U;
  }
  const std::size_t removed_crowd = result.removed_height + result.removed_region;
  const std::size_t kept_all = result.input - result.removed_self - removed_crowd;
  result.fallback = removed_crowd > 0U && kept_all < params.min_points;

  result.keep.resize(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    const bool removed = result.fallback ? reasons[i] == Reason::kSelf : reasons[i] != Reason::kKept;
    result.keep[i] = removed ? 0U : 1U;
  }
  result.kept = result.fallback ? result.input - result.removed_self : kept_all;
  return result;
}

AxisBox makeBox(
  const std::vector<double> & min_xyz, const std::vector<double> & max_xyz,
  const std::string & what)
{
  if (min_xyz.size() != 3U || max_xyz.size() != 3U) {
    throw std::invalid_argument(
            what + " の下限と上限は 3 個ずつ（x, y, z）にしてください（" +
            std::to_string(min_xyz.size()) + " 個と " + std::to_string(max_xyz.size()) + " 個）");
  }
  AxisBox box;
  box.min = Eigen::Vector3d(min_xyz[0], min_xyz[1], min_xyz[2]).cast<float>();
  box.max = Eigen::Vector3d(max_xyz[0], max_xyz[1], max_xyz[2]).cast<float>();
  checkBox(box, what);
  return box;
}

std::vector<Region> makeRegions(
  const std::vector<std::string> & names, const std::vector<double> & boxes)
{
  if (boxes.size() % kValuesPerRegion != 0U) {
    throw std::invalid_argument(
            "領域の値は 1 つにつき 6 個（x_min, x_max, y_min, y_max, z_min, z_max）にしてください（" +
            std::to_string(boxes.size()) + " 個）");
  }
  const std::size_t count = boxes.size() / kValuesPerRegion;
  if (names.size() != count) {
    throw std::invalid_argument(
            "領域の名前の数 " + std::to_string(names.size()) + " が領域の数 " +
            std::to_string(count) + " と違います");
  }
  std::vector<Region> regions;
  regions.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    const double * values = boxes.data() + i * kValuesPerRegion;
    Region region;
    region.name = names[i];
    region.box.min = Eigen::Vector3d(values[0], values[2], values[4]).cast<float>();
    region.box.max = Eigen::Vector3d(values[1], values[3], values[5]).cast<float>();
    checkBox(region.box, "領域 " + names[i]);
    regions.push_back(region);
  }
  return regions;
}

void validate(const Params & params)
{
  checkBox(params.self_box, "自機の箱");
  for (const auto & region : params.regions) {
    checkBox(region.box, "領域 " + region.name);
  }
  if (!std::isfinite(params.height_min_m) || !std::isfinite(params.height_max_m) ||
    params.height_min_m > params.height_max_m)
  {
    throw std::invalid_argument(
            "高さの帯の下限 " + std::to_string(params.height_min_m) + "m が上限 " +
            std::to_string(params.height_max_m) + "m を超えているか、有限でありません");
  }
}
}  // namespace core_localization::crowd
