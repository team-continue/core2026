// 点群抽出（地図照合の入力から、照合に使う点だけを取り出す処理）の計算部分。ROS に依存しないので、
// ノードと分けてテストする。
//
// 人はフィールドの中だけでなく外（観客席、フィールドの周り）にもいて動く。地図照合に動く物の点が混ざると
// 照合がずれるので、モードごとに動かない物の点だけを残す。FAST-LIO の手前にはかけない（scan_filter だけ）。
// - ceiling（天井）: 床と、床から band_max より上（壁の上の方、天井、トラス、柱）だけを残す。
//   人の帯（床から band_min〜band_max）と、観客席の箱（map の座標系。段になった上の段の人は床から高いので、
//   帯では除けない）の中を除く。計算は crowd_filter.hpp の classify（高さの帯と領域）。
//   柵の外（フィールドの範囲に余裕を足した長方形の外）では、帯の上限を outside.band_max に広げる
//   （台や低い段の上に立つ観客を除く。会場の観客席を測らずに、フィールドの範囲から決める）
// - field_objects（フィールドオブジェクト）: フィールドとその周り（map の座標系の長方形。field.yaml の範囲に
//   余裕を足す）の、床から max_height までの点だけを残す。外（観客）と max_height より上（屋根）を除く。
//   フィールドの中の相手（他のロボットと人）は、追跡の箱を点群の時刻まで速度で進め、種類ごとの半径に
//   余裕を足した柱の中の点を除く
//
// 除いた後の点が min_points（モードごと）未満になる周期は、その周期は抽出をかけずに全点を通す（fallback）。
// 点が少なすぎると照合が退化して発散するおそれがあり、動く物の点が混ざる方が害が小さいため。

#ifndef CORE_LOCALIZATION__POINT_EXTRACTION_HPP_
#define CORE_LOCALIZATION__POINT_EXTRACTION_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "core_localization/crowd_filter.hpp"

namespace core_localization::point_extraction
{
enum class Mode : std::uint8_t {kOff, kCeiling, kFieldObjects};

// "off"・"ceiling"・"field_objects" をモードにする。ほかの文字列は理由を付けて std::invalid_argument
Mode parseMode(const std::string & text);
std::string modeName(Mode mode);

// map の座標軸に沿った xy の長方形。境界を含む
struct Area
{
  float x_min{0.0F};
  float x_max{0.0F};
  float y_min{0.0F};
  float y_max{0.0F};
};

// 柵の外（観客がいる所）の人の帯。会場の観客席を測らずに、フィールドの範囲から決める
struct OutsideBand
{
  bool enabled{false};
  // フィールドの範囲（field.yaml。柵の内側の面）。既定値は field.yaml と同じ
  Area field{-12.0F, 12.0F, -7.5F, 7.5F};
  // フィールドの範囲にこの余裕を足した長方形の外を、観客がいる所とみなす
  float field_margin_m{1.0F};
  // 柵の外の人の帯の上限 [m]。下限は CeilingParams の band_min_m と同じ
  float band_max_m{3.0F};
};

struct CeilingParams
{
  // 床からの高さがこの帯の中の点（人）を除く。床（band_min 未満）と頭より上（band_max 超）は残す
  float band_min_m{0.1F};
  float band_max_m{2.2F};
  // 観客席などの箱（map の座標系）
  std::vector<crowd::Region> regions;
  OutsideBand outside;
  std::size_t min_points{0U};
};

enum class TrackClass : std::uint8_t {kUnknown, kRobot, kHuman};

// 追跡した相手。座標は追跡の座標系（odom）
struct Track
{
  Eigen::Vector2f position{Eigen::Vector2f::Zero()};
  Eigen::Vector2f velocity{Eigen::Vector2f::Zero()};
  TrackClass classification{TrackClass::kUnknown};
};

struct TrackParams
{
  bool enabled{true};
  // 種類ごとの半径 [m]。人はシミュレーションの円柱（半径 0.25m）、ロボットは 0.8m 角の外接円（0.566m）。
  // 種類が分からない物は大きい方（ロボット）にする
  float human_radius_m{0.25F};
  float robot_radius_m{0.57F};
  float unknown_radius_m{0.57F};
  // 半径に足す余裕。追跡の位置の誤差（約 0.13m）と、見えている面からの中心の推定のずれを含める
  float margin_m{0.2F};
  // 追跡の時刻と点群の時刻の差の上限 [s]。これより古い（新しい）追跡は使わない
  double max_age_s{0.5};
};

struct FieldObjectsParams
{
  // フィールドの範囲（field.yaml。柵の内側の面）。既定値は field.yaml と同じ
  Area field{-12.0F, 12.0F, -7.5F, 7.5F};
  // フィールドの範囲に足す余裕。柵の厚みと、すぐ外の動かない物を含める
  float field_margin_m{0.5F};
  // 床からの高さの上限。これより上（屋根など）を除く
  float max_height_m{2.2F};
  TrackParams tracks;
  std::size_t min_points{0U};
};

struct Params
{
  Mode mode{Mode::kOff};
  CeilingParams ceiling;
  FieldObjectsParams field_objects;
};

// 追跡の相手を除く柱。座標は追跡の座標系
struct Cylinder
{
  Eigen::Vector2f center{Eigen::Vector2f::Zero()};
  float radius_m{0.0F};
};

// 点群の座標系から各座標系への変換。その周期に得られなかった変換は nullopt
struct Transforms
{
  // 変換後の z が床からの高さになる変換（床の座標系への変換から、床の高さを引いたもの）
  std::optional<Eigen::Isometry3f> height_from_cloud;
  std::optional<Eigen::Isometry3f> map_from_cloud;
  // 追跡の座標系（odom）への変換。field_objects の追跡の柱に使う
  std::optional<Eigen::Isometry3f> tracks_from_cloud;
};

struct Result
{
  // 残す点は 1。入力と同じ順
  std::vector<std::uint8_t> keep;
  std::size_t input{0U};
  // 各処理が除くと判定した点の数。fallback のときも判定した数を入れる。1 つの点は最初に当てはまった
  // 処理で数える（ceiling: 帯、領域、柵の外の帯の順。field_objects: 範囲の外、高さの上限、追跡の柱の順）
  std::size_t removed_band{0U};
  std::size_t removed_region{0U};
  // 柵の外で、ふつうの帯より上の広げた帯（band_max 超〜outside.band_max）にあって除いた点
  std::size_t removed_outside_band{0U};
  std::size_t removed_outside{0U};
  std::size_t removed_above{0U};
  std::size_t removed_track{0U};
  // 実際に残す点の数
  std::size_t kept{0U};
  // 除いた後の点が min_points より少ないので、この周期は抽出をかけずに全点を通した
  bool fallback{false};
  // 変換がないので、この周期は抽出をかけずに全点を通した（field_objects の範囲と高さ）
  bool passed_without_transform{false};
  // 有効だが変換か追跡がないので外した処理
  bool band_skipped{false};
  bool region_skipped{false};
  bool outside_band_skipped{false};
  bool tracks_skipped{false};
};

// field.yaml の範囲に余裕を足した、残す範囲。範囲が正しくなければ std::invalid_argument
Area keepArea(const FieldObjectsParams & params);

// ceiling の柵の外の帯をかける境界（field.yaml の範囲に余裕を足した長方形。この外にかける）。
// 範囲が正しくなければ std::invalid_argument
Area outsideBoundary(const OutsideBand & params);

// 追跡を dt_s だけ速度で進め、種類ごとの半径に余裕を足した柱にする
std::vector<Cylinder> predictCylinders(
  const std::vector<Track> & tracks, double dt_s, const TrackParams & params);

// 点ごとに残すかを決める。cylinders は field_objects で除く柱（追跡の座標系）
Result extract(
  const std::vector<Eigen::Vector3f> & points, const Params & params,
  const Transforms & transforms, const std::vector<Cylinder> & cylinders);

// 値の組み合わせを確かめる。正しくなければ理由を付けて std::invalid_argument
void validate(const Params & params);
}  // namespace core_localization::point_extraction

#endif  // CORE_LOCALIZATION__POINT_EXTRACTION_HPP_
