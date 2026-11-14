// 機体の形（長方形）と点の距離。

#ifndef CORE_PLANNING__FOOTPRINT_HPP_
#define CORE_PLANNING__FOOTPRINT_HPP_

#include <algorithm>
#include <cmath>

namespace core_planning
{
// 機体座標系の点 (x, y) から、中心が原点の長方形までの符号付き距離。内側は負
inline double signedDistanceToRectangle(
  const double x, const double y, const double half_length, const double half_width)
{
  const double dx = std::abs(x) - half_length;
  const double dy = std::abs(y) - half_width;
  const double outside = std::hypot(std::max(dx, 0.0), std::max(dy, 0.0));
  const double inside = std::min(std::max(dx, dy), 0.0);
  return outside + inside;
}
}  // namespace core_planning

#endif  // CORE_PLANNING__FOOTPRINT_HPP_
