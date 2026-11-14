// フィールドの範囲。柵の内側の面に合わせる。

#ifndef CORE_NAV_COMMON__FIELD_BOUNDS_HPP_
#define CORE_NAV_COMMON__FIELD_BOUNDS_HPP_

namespace core_nav_common
{
struct FieldBounds
{
  double x_min{-12.0};
  double x_max{12.0};
  double y_min{-7.5};
  double y_max{7.5};

  bool contains(const double x, const double y, const double inset) const
  {
    return x >= x_min + inset && x <= x_max - inset && y >= y_min + inset && y <= y_max - inset;
  }
};
}  // namespace core_nav_common

#endif  // CORE_NAV_COMMON__FIELD_BOUNDS_HPP_
