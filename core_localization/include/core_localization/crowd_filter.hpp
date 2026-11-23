// 人混みに強くするための点群フィルタの計算部分。ROS に依存しないので、ノードと分けてテストする。
// 点群抽出（point_extraction.hpp）の ceiling モードの計算に使う。
//
// 人はフィールドの中だけでなく外（観客席、フィールドの周り）にもいて動く。地図照合が頼れるのは、
// 人の頭より上の物（壁の上の方、天井、トラス、柱）と床なので、人がいうる所の点を除く。
// - 人の高さの帯: 床からの高さが [min, max] の点。床の点（min 未満）と頭より上の点（max 超）は残す
// - 領域: map の座標系で与えた箱（観客席など）の中の点。段になった観客席の上の段の人は床から高いので、
//   高さの帯では除けない
// - 自機の除去: base_link の座標系の箱（機体の形と余裕）の中の点。点群抽出では使わない
//   （自機の点は FAST-LIO の前処理が同じ機体の箱（footprint.yaml）で除き、重なるため）。
//   FAST-LIO の前処理を通らない点群にかけるときのために残している
// 1 つの点は、自機、高さの帯、領域の順で最初に当てはまったフィルタで除いたものとして数える。
//
// 除いた後の点が min_points 未満になる周期は、高さの帯と領域を外して自機の除去だけをかける（fallback）。
// 点が少なすぎると照合が退化して発散するおそれがあり、人の点が混ざる方が害が小さいため。
// 自機の点は動く物ではなく常に正しくないので、その周期も除く。

#ifndef CORE_LOCALIZATION__CROWD_FILTER_HPP_
#define CORE_LOCALIZATION__CROWD_FILTER_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>

namespace core_localization::crowd
{
// 座標軸に沿った箱。境界を含む
struct AxisBox
{
  Eigen::Vector3f min{Eigen::Vector3f::Zero()};
  Eigen::Vector3f max{Eigen::Vector3f::Zero()};
};

struct Region
{
  std::string name;
  AxisBox box;
};

struct Params
{
  bool self_enabled{false};
  // 機体 0.8m x 0.8m に 0.1m の余裕。高さは機体の上面 0.8m に余裕 0.05m。下は床の少し下から
  AxisBox self_box{Eigen::Vector3f(-0.5F, -0.5F, -0.1F), Eigen::Vector3f(0.5F, 0.5F, 0.85F)};
  bool height_band_enabled{false};
  float height_min_m{0.1F};
  float height_max_m{2.2F};
  bool region_enabled{false};
  std::vector<Region> regions;
  // 除いた後にこれより少なければ、その周期は高さの帯と領域をかけない
  std::size_t min_points{1000U};
};

// 点群の座標系から各座標系への変換。その周期に得られなかった変換は nullopt にし、使うフィルタを外す
struct Transforms
{
  std::optional<Eigen::Isometry3f> base_from_cloud;
  // 変換後の z が床からの高さになる変換（床の座標系への変換から、床の高さを引いたもの）
  std::optional<Eigen::Isometry3f> height_from_cloud;
  std::optional<Eigen::Isometry3f> map_from_cloud;
};

struct Result
{
  // 残す点は 1。入力と同じ順
  std::vector<std::uint8_t> keep;
  std::size_t input{0U};
  // 各フィルタが除くと判定した点の数。fallback のときも判定した数を入れる
  std::size_t removed_self{0U};
  std::size_t removed_height{0U};
  std::size_t removed_region{0U};
  // 実際に残す点の数
  std::size_t kept{0U};
  // 点が少なすぎて、高さの帯と領域を外した
  bool fallback{false};
  // 有効だが変換がなくて外したフィルタ
  bool self_skipped{false};
  bool height_skipped{false};
  bool region_skipped{false};
};

bool contains(const AxisBox & box, const Eigen::Vector3f & point);

// 点ごとに残すかを決める
Result classify(
  const std::vector<Eigen::Vector3f> & points, const Params & params,
  const Transforms & transforms);

// 3 個ずつの下限と上限から箱を作る。数が違うか下限が上限を超えると std::invalid_argument
AxisBox makeBox(
  const std::vector<double> & min_xyz, const std::vector<double> & max_xyz,
  const std::string & what);

// 名前と、領域ごとに 6 個（x_min, x_max, y_min, y_max, z_min, z_max）並べた値から領域を作る。
// 数が合わないか下限が上限を超えると std::invalid_argument
std::vector<Region> makeRegions(
  const std::vector<std::string> & names, const std::vector<double> & boxes);

// 値の組み合わせを確かめる。正しくなければ理由を付けて std::invalid_argument
void validate(const Params & params);

// keep が 1 の点だけを、順を保って返す。数が違えば std::invalid_argument
template<typename Point>
std::vector<Point> selectKept(
  const std::vector<Point> & points, const std::vector<std::uint8_t> & keep)
{
  if (points.size() != keep.size()) {
    throw std::invalid_argument(
            "点の数 " + std::to_string(points.size()) + " と印の数 " +
            std::to_string(keep.size()) + " が違います");
  }
  std::vector<Point> output;
  output.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (keep[i] != 0U) {
      output.push_back(points[i]);
    }
  }
  return output;
}
}  // namespace core_localization::crowd

#endif  // CORE_LOCALIZATION__CROWD_FILTER_HPP_
