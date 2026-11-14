// 大域計画の折れ線を、静止物の周りでだけ曲線にする。ROS に依存しない。
//
// 折れ線を spacing_m おきの点に増やし、端点以外の点を iterations 回動かす。点ごとに次の 2 つを足す。
// - なめらかさ: 両隣の中点へ smooth_weight の割合で引く
// - 余裕: 距離場の距離が「hard_radius_m + extra_clearance_m」より近ければ、不足分の clearance_weight
//   の割合だけ、距離場の勾配の向き（障害物から離れる向き）へ押す
// 障害物から遠い直線の区間は両方の力が 0 なので動かない。動かした先が守るべき距離より近くなる移動は
// 採らない。端点は動かないので、端点から 1m の間は、取りたい距離を端点の距離から少しずつ上げる
// （端点が障害物の近くにあっても、端点で鉤形に曲がらない）。

#ifndef CORE_PLANNING__PATH_SMOOTHER_HPP_
#define CORE_PLANNING__PATH_SMOOTHER_HPP_

#include <optional>
#include <utility>
#include <vector>

#include "core_planning/grid_map.hpp"

namespace core_planning
{
struct SmoothingParams
{
  bool enabled{true};
  // 点の間隔 [m]
  double spacing_m{0.1};
  // 膨らみの外に、さらに取りたい余裕 [m]
  double extra_clearance_m{0.3};
  // 点を動かす回数の上限。動きがなくなれば途中でやめる
  int iterations{200};
  // 両隣の中点へ引く割合（0〜1）
  double smooth_weight{0.25};
  // 余裕の不足分のうち、1 回で押し戻す割合（0〜1）
  double clearance_weight{0.25};
};

// 折れ線を曲線にする。端点は動かさない。
//
// 守る距離は field の格子のセルで判定する（`field.at(x, y) >= hard_radius_m`。inflate で膨らみの外に
// なるセルと同じ）。点と、隣り合う点の中点の両方で確かめる。
// - 最初と最後の区間（機体の位置からと、ゴールへの区間）の点は「hard_radius_m と、元の折れ線の同じ点の
//   距離の小さい方」を守る。機体が柵や障害物の近くにいると、計画は nearestFreeCell で探し始めるので、
//   始点からの区間がもともと膨らみの中にある。ここで膨らみの外を求めると毎回失敗して折れ線に戻るため、
//   元の折れ線より障害物に近づかないことだけを条件にする（押し出す向きには動ける）。ゴールの側も同じ
// - それ以外の点は hard_radius_m 以上を守る。元の折れ線が途中で膨らみに入っている（格子と距離場が
//   合っていない）など、守れないときは nullopt を返す（呼び出し側は折れ線を使う）
//
// enabled が false なら、折れ線をそのまま返す。点が 2 つ未満のときも、そのまま返す。
// 値が範囲外のとき、座標が有限でないときは std::invalid_argument を投げる。
std::optional<std::vector<std::pair<double, double>>> smoothPath(
  const std::vector<std::pair<double, double>> & polyline, const DistanceField & field,
  double hard_radius_m, const SmoothingParams & params);
}  // namespace core_planning

#endif  // CORE_PLANNING__PATH_SMOOTHER_HPP_
