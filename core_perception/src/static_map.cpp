#include "core_perception/static_map.hpp"

#include <array>
#include <stdexcept>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "core_nav_common/point_filters.hpp"

namespace core_perception
{
StaticMap StaticMap::load(
  const std::string & path, const FieldBounds & field, const double leaf_size_m,
  const double min_height_m, const double max_height_m)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile(path, cloud) != 0) {
    throw std::runtime_error("地図を読めません: " + path);
  }
  StaticMap map;
  map.field_ = field;
  map.leaf_size_m_ = leaf_size_m;
  // 位置のずれを許すよう、周囲 1 ボクセルにも広げる
  for (const auto & point : cloud.points) {
    if (!field.contains(point.x, point.y, -leaf_size_m) || point.z < min_height_m ||
      point.z > max_height_m)
    {
      continue;
    }
    for (const int dx : {-1, 0, 1}) {
      for (const int dy : {-1, 0, 1}) {
        for (const int dz : {-1, 0, 1}) {
          map.voxels_.insert(
            core_nav_common::voxelKey(
              point.x + dx * leaf_size_m, point.y + dy * leaf_size_m, point.z + dz * leaf_size_m,
              leaf_size_m));
        }
      }
    }
  }
  return map;
}

bool StaticMap::contains(const Eigen::Vector3f & map_point) const
{
  return voxels_.count(
    core_nav_common::voxelKey(map_point.x(), map_point.y(), map_point.z(), leaf_size_m_)) > 0U;
}
}  // namespace core_perception
