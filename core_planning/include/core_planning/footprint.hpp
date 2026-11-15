// 機体の形（長方形）と点の距離。
//
// core2026 は上部（base_link）の下で駆動部（chassis_link）が回る。駆動部の長方形は、base_link から見て
// 腰の角度だけ回っている。回転モードの間は角度が速く変わるので、外接円で判定する

#ifndef CORE_PLANNING__FOOTPRINT_HPP_
#define CORE_PLANNING__FOOTPRINT_HPP_

#include <algorithm>
#include <cmath>
#include <optional>

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

// 駆動部の形。half_length・half_width は駆動部の座標での長方形、yaw は base_link から見た駆動部の向き。
// circle なら向きによらず外接円
struct ChassisShape
{
  double half_length{0.0};
  double half_width{0.0};
  double yaw{0.0};
  bool circle{false};
};

// base_link の点 (x, y) から駆動部までの符号付き距離。内側は負
inline double signedDistanceToChassis(const double x, const double y, const ChassisShape & shape)
{
  if (shape.circle) {
    return std::hypot(x, y) - std::hypot(shape.half_length, shape.half_width);
  }
  const double c = std::cos(shape.yaw);
  const double s = std::sin(shape.yaw);
  return signedDistanceToRectangle(c * x + s * y, -s * x + c * y, shape.half_length, shape.half_width);
}

// 駆動部を base_link の軸に沿って囲む長方形の半分の長さと幅（向きを見ない判定の目安に使う）
struct HalfExtents
{
  double half_length{0.0};
  double half_width{0.0};
};

inline HalfExtents boundingHalfExtents(const ChassisShape & shape)
{
  if (shape.circle) {
    const double radius = std::hypot(shape.half_length, shape.half_width);
    return HalfExtents{radius, radius};
  }
  const double c = std::abs(std::cos(shape.yaw));
  const double s = std::abs(std::sin(shape.yaw));
  return HalfExtents{
    c * shape.half_length + s * shape.half_width, s * shape.half_length + c * shape.half_width};
}

// 腰の角度（分からなければ nullopt）と回転モードから、駆動部の形を決める。
// 角度が分からないとき・回転モードの間は外接円にする
inline ChassisShape chassisShape(
  const double half_length, const double half_width, const std::optional<double> & yaw,
  const bool rotating)
{
  return ChassisShape{half_length, half_width, yaw.value_or(0.0), rotating || !yaw.has_value()};
}

// 機体の形の設定（FootprintParams・safety::Footprint）の長方形を、駆動部の向きに合わせる。
// 判定の目安に使う half_length_m・half_width_m は駆動部を囲む長方形にし、
// 距離を測る所は chassis（signedDistanceToFootprint）で駆動部の形そのものを使う
template<typename FootprintT>
void orientFootprint(FootprintT & footprint, const ChassisShape & chassis)
{
  footprint.chassis = chassis;
  const HalfExtents extents = boundingHalfExtents(chassis);
  footprint.half_length_m = extents.half_length;
  footprint.half_width_m = extents.half_width;
}

// 機体の形の設定から、base_link の点までの符号付き距離。駆動部の形が決まっていなければ
// （chassis の大きさが 0）、half_length_m・half_width_m の長方形を base_link の向きのまま使う
template<typename FootprintT>
double signedDistanceToFootprint(const double x, const double y, const FootprintT & footprint)
{
  if (footprint.chassis.half_length <= 0.0 || footprint.chassis.half_width <= 0.0) {
    return signedDistanceToRectangle(x, y, footprint.half_length_m, footprint.half_width_m);
  }
  return signedDistanceToChassis(x, y, footprint.chassis);
}
}  // namespace core_planning

#endif  // CORE_PLANNING__FOOTPRINT_HPP_
