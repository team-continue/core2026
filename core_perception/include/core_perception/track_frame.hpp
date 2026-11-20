// 追跡結果（odom の位置と速度）を、別の座標系（地図）へ直す。ROS に依存しない。
//
// 追跡は odom の座標で行う（地図照合の補正で位置が飛ばないように）。操縦の UI は地図の上に重ねて描くので、
// map ← odom の変換で直したものも出す。

#ifndef CORE_PERCEPTION__TRACK_FRAME_HPP_
#define CORE_PERCEPTION__TRACK_FRAME_HPP_

#include <Eigen/Geometry>

namespace core_perception
{
struct PlanarTrackState
{
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
};

// target ← source の変換 target_from_source で、位置は点として、速度は向きだけ回して直す（高さは 0 とする）
inline PlanarTrackState transformTrackState(
  const PlanarTrackState & state, const Eigen::Isometry3f & target_from_source)
{
  const Eigen::Vector3f position =
    target_from_source * Eigen::Vector3f(static_cast<float>(state.x), static_cast<float>(state.y),
      0.0F);
  const Eigen::Vector3f velocity =
    target_from_source.linear() *
    Eigen::Vector3f(static_cast<float>(state.vx), static_cast<float>(state.vy), 0.0F);
  return PlanarTrackState{position.x(), position.y(), velocity.x(), velocity.y()};
}
}  // namespace core_perception

#endif  // CORE_PERCEPTION__TRACK_FRAME_HPP_
