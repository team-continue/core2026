// 見えない側へ進むときの速さの上限。局所計画（MPPI）と安全停止の退避の候補で同じものを使う。
//
// LiDAR は機体の前の端に付いていて、後ろ側は機体にさえぎられて見えない。進む向きが機体の前から
// visible_half_angle_deg より外なら、速さを speed_mps までに抑える。横（±90°）は見える範囲に入る。

#ifndef CORE_PLANNING__BLIND_SIDE_HPP_
#define CORE_PLANNING__BLIND_SIDE_HPP_

#include <algorithm>
#include <cmath>

namespace core_planning
{
struct BlindSideParams
{
  // false なら上限をかけない（前の動き）
  bool enabled{true};
  // 機体の前から測った、見える範囲の半角 [deg]
  double visible_half_angle_deg{96.0};
  // 見えない側へ進むときの速さの上限 [m/s]
  double speed_mps{0.5};
};

// 見えない側へ向かう速度を縮める倍率を求める。見える範囲の半角の cos を先に求めておく
class BlindSideLimit
{
public:
  explicit BlindSideLimit(const BlindSideParams & params)
  : enabled_(params.enabled),
    visible_cos_(std::cos(params.visible_half_angle_deg * M_PI / 180.0)),
    speed_mps_(std::max(params.speed_mps, 0.0))
  {
  }

  // 機体の前の向きの単位ベクトル (forward_x, forward_y) と同じ座標系の速度 (vx, vy) に掛ける倍率（1 以下）。
  // 見える範囲の速度、上限以下の速さ、無効のときは 1
  double scale(
    const double vx, const double vy, const double forward_x, const double forward_y) const
  {
    if (!enabled_) {
      return 1.0;
    }
    const double speed = std::hypot(vx, vy);
    if (speed <= speed_mps_ || vx * forward_x + vy * forward_y >= visible_cos_ * speed) {
      return 1.0;
    }
    return speed_mps_ / speed;
  }

  // 速度 (vx, vy) の向きが、機体の前から見える範囲の外か。止まっていれば false
  bool isBlind(
    const double vx, const double vy, const double forward_x, const double forward_y) const
  {
    const double speed = std::hypot(vx, vy);
    return speed > 0.0 && vx * forward_x + vy * forward_y < visible_cos_ * speed;
  }

private:
  bool enabled_;
  double visible_cos_;
  double speed_mps_;
};
}  // namespace core_planning

#endif  // CORE_PLANNING__BLIND_SIDE_HPP_
