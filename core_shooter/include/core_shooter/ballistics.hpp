#ifndef CORE_SHOOTER__BALLISTICS_HPP_
#define CORE_SHOOTER__BALLISTICS_HPP_

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace core_shooter
{

/// 砲塔座標系の3次元ベクトル。REP-103 に従い x:前方 / y:左 / z:上 [m]。
struct Vector3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

/// 3次元ベクトルが指す方向のヨー・ピッチ角 [rad]。
struct DirectionAngles
{
  double yaw_rad = 0.0;
  double pitch_rad = 0.0;
};

/// 砲塔座標系（x軸前向き）の点を方向角へ変換する。
///
/// yaw   = atan2(y, x)              … x軸前向きからの左右角
/// pitch = atan2(z, hypot(x, y))    … 水平面からの仰角
inline DirectionAngles toDirectionAngles(const Vector3 & point)
{
  return DirectionAngles{
    std::atan2(point.y, point.x),
    std::atan2(point.z, std::hypot(point.x, point.y))};
}

inline double vectorNorm(const Vector3 & point)
{
  return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

inline bool isFinite(const Vector3 & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

/// 実測した曲射弾道の1点。着弾ずれは読み込み時に射角補正へ変換しておく。
struct BallisticSample
{
  double range_m = 0.0;
  double yaw_correction_rad = 0.0;
  double pitch_correction_rad = 0.0;
};

/// フライングディスクの曲射弾道を距離ごとの実測値で表したテーブル。
///
/// 空のまま使うと補正 0（直線弾道）として振る舞う。
class BallisticTable
{
public:
  /// 実測テーブル（config/ballistics_*.yaml）を読み込む。
  ///
  /// 形式:
  ///   ballistics:
  ///     <水平距離[m]>: {y: <横ずれ[m]>, z: <縦ずれ[m]>}
  ///
  /// y/z は砲身を的の中心へ向けて撃ったときの着弾点のずれ。これを打ち消す向きの
  /// 射角補正 atan2(-offset, range) に変換して保持する。
  static bool load(const std::string & path, BallisticTable & table, std::string & reason)
  {
    YAML::Node root;
    try {
      root = YAML::LoadFile(path);
    } catch (const YAML::Exception & e) {
      reason = "failed to load '" + path + "': " + e.what();
      return false;
    }

    const YAML::Node entries = root["ballistics"];
    if (!entries || !entries.IsMap()) {
      reason = "'" + path + "' must contain a 'ballistics' mapping of range[m] -> {y, z}";
      return false;
    }

    std::vector<BallisticSample> samples;
    for (const auto & entry : entries) {
      double range_m = 0.0;
      double offset_y_m = 0.0;
      double offset_z_m = 0.0;
      try {
        range_m = entry.first.as<double>();
        offset_y_m = entry.second["y"].as<double>();
        offset_z_m = entry.second["z"].as<double>();
      } catch (const YAML::Exception &) {
        reason = "invalid entry in '" + path +
          "': every key must be a range[m] mapped to {y: <m>, z: <m>}";
        return false;
      }
      if (!std::isfinite(range_m) || range_m <= 0.0 ||
        !std::isfinite(offset_y_m) || !std::isfinite(offset_z_m))
      {
        reason = "invalid sample in '" + path +
          "': range must be finite and > 0, y/z must be finite";
        return false;
      }
      samples.push_back(
        BallisticSample{
          range_m,
          std::atan2(-offset_y_m, range_m),
          std::atan2(-offset_z_m, range_m)});
    }

    if (samples.empty()) {
      reason = "'" + path + "' contains no ballistics samples";
      return false;
    }

    std::sort(
      samples.begin(), samples.end(),
      [](const BallisticSample & lhs, const BallisticSample & rhs) {
        return lhs.range_m < rhs.range_m;
      });
    for (size_t i = 1; i < samples.size(); ++i) {
      if (samples[i].range_m <= samples[i - 1].range_m) {
        reason = "duplicated range in '" + path + "'";
        return false;
      }
    }

    table.samples_ = std::move(samples);
    return true;
  }

  bool empty() const {return samples_.empty();}
  size_t size() const {return samples_.size();}
  double minRange() const {return samples_.front().range_m;}
  double maxRange() const {return samples_.back().range_m;}

  /// テーブルが覆っていない距離のはみ出し量[m]。範囲内なら 0。
  ///
  /// テーブル未設定（直線弾道）のときは、覆うべき範囲という概念がないので 0。
  double rangeError(double horizontal_range_m) const
  {
    if (samples_.empty()) {
      return 0.0;
    }
    if (horizontal_range_m < minRange()) {
      return minRange() - horizontal_range_m;
    }
    if (horizontal_range_m > maxRange()) {
      return horizontal_range_m - maxRange();
    }
    return 0.0;
  }

  /// 水平距離に対する射角補正[rad]を線形補間して返す。
  ///
  /// テーブル未設定なら直線弾道とみなして 0 を返す。
  /// テーブルが覆っていない距離でも補正は行わず 0 を返し、的の方向へそのまま向ける。
  DirectionAngles correction(double horizontal_range_m) const
  {
    if (samples_.empty() || rangeError(horizontal_range_m) > 0.0) {
      return DirectionAngles{};
    }

    for (size_t i = 1; i < samples_.size(); ++i) {
      const BallisticSample & upper = samples_[i];
      if (horizontal_range_m > upper.range_m) {
        continue;
      }
      const BallisticSample & lower = samples_[i - 1];
      // 読み込み時に距離の重複を弾いているので span は必ず正。
      const double span = upper.range_m - lower.range_m;
      const double ratio = (horizontal_range_m - lower.range_m) / span;
      return DirectionAngles{
        lower.yaw_correction_rad +
        (upper.yaw_correction_rad - lower.yaw_correction_rad) * ratio,
        lower.pitch_correction_rad +
        (upper.pitch_correction_rad - lower.pitch_correction_rad) * ratio};
    }
    return DirectionAngles{
      samples_.front().yaw_correction_rad,
      samples_.front().pitch_correction_rad};
  }

private:
  std::vector<BallisticSample> samples_;
};

}  // namespace core_shooter

#endif  // CORE_SHOOTER__BALLISTICS_HPP_
