// フィールドの範囲と、事前地図の固定物の検索。

#ifndef CORE_PERCEPTION__STATIC_MAP_HPP_
#define CORE_PERCEPTION__STATIC_MAP_HPP_

#include <cstdint>
#include <string>
#include <unordered_set>

#include <Eigen/Core>

#include "core_nav_common/field_bounds.hpp"

namespace core_perception
{
using core_nav_common::FieldBounds;

class StaticMap
{
public:
  StaticMap() = default;

  // PCD を読み、フィールド内で床より高い点をボクセルにする。読めなければ std::runtime_error
  static StaticMap load(
    const std::string & path, const FieldBounds & field, double leaf_size_m, double min_height_m,
    double max_height_m);

  bool contains(const Eigen::Vector3f & map_point) const;

  std::size_t size() const {return voxels_.size();}

  const FieldBounds & field() const {return field_;}

  void setField(const FieldBounds & field) {field_ = field;}

private:
  FieldBounds field_;
  double leaf_size_m_{0.2};
  std::unordered_set<std::uint64_t> voxels_;
};
}  // namespace core_perception

#endif  // CORE_PERCEPTION__STATIC_MAP_HPP_
