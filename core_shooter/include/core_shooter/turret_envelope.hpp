#ifndef CORE_SHOOTER__TURRET_ENVELOPE_HPP_
#define CORE_SHOOTER__TURRET_ENVELOPE_HPP_

// 砲塔の可動包絡線。ヨー角ごとに許されるピッチ範囲を折れ線で表す。
//
// 機構干渉とルール上の制約を、ゾーン分割ではなく連続な包絡線として表現する。
// 同じ包絡線を順引き（ヨー→ピッチ範囲）と逆引き（ピッチ→ヨー範囲）の両方に
// 使うことで、状態を持たずに次の2つを同時に満たす:
//
//   1. 指令する (yaw, pitch) が常に包絡線の内側にある
//   2. 砲身が現在位置から指令位置まで掃く全区間で包絡線を満たす
//
// 2 によって「今のピッチでは入れない領域へヨーが進む」ことを防ぐ。
// ピッチが下がるにつれて進めるヨーが広がるので、従来の
// 「ヨーを止めてピッチを直してから回る」が状態機械なしで出てくる。
//
// ROS に依存しないので実機なしで単体テストできる。

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace core_shooter
{

/// 許容ピッチ範囲 [rad]。empty() なら実現不能。
struct PitchRange
{
  double min = 0.0;
  double max = 0.0;
  bool empty() const {return min > max;}
};

/// 許容ヨー範囲 [rad]。
struct YawRange
{
  double min = 0.0;
  double max = 0.0;
};

/// 包絡線の1点。
struct EnvelopePoint
{
  double yaw_rad = 0.0;
  double pitch_min = 0.0;
  double pitch_max = 0.0;
};

/// モータ側の可動範囲。包絡線とは別に常にかかる。
struct MotorLimits
{
  double yaw_min = -3.14159265359;
  double yaw_max = 3.14159265359;
  double pitch_min = -3.14159265359;
  double pitch_max = 3.14159265359;
};

/// 制限後の指令。
struct EnvelopeResult
{
  double yaw = 0.0;
  double pitch = 0.0;
  /// 実測ピッチが現在ヨーの包絡線から外れており、ヨーを固定して
  /// ピッチを戻している最中。手で動かされた場合などに立つ。
  bool recovering = false;
};

/// 砲塔の可動包絡線。
///
/// 点を持たない状態では制限なし（素通し）として振る舞う。
class TurretEnvelope
{
public:
  /// 包絡線ファイルを読み込む。
  ///
  /// 形式:
  ///   envelope:
  ///     <ヨー角[rad]>: {pitch_min: <rad>, pitch_max: <rad>}
  ///
  /// ヨー角の昇順に並べ替えて保持する。2点以上必要。
  static bool load(const std::string & path, TurretEnvelope & envelope, std::string & reason)
  {
    YAML::Node root;
    try {
      root = YAML::LoadFile(path);
    } catch (const YAML::Exception & e) {
      reason = "failed to load '" + path + "': " + e.what();
      return false;
    }

    const YAML::Node entries = root["envelope"];
    if (!entries || !entries.IsMap()) {
      reason = "'" + path +
        "' must contain an 'envelope' mapping of yaw[rad] -> {pitch_min, pitch_max}";
      return false;
    }

    std::vector<EnvelopePoint> points;
    for (const auto & entry : entries) {
      EnvelopePoint point;
      try {
        point.yaw_rad = entry.first.as<double>();
        point.pitch_min = entry.second["pitch_min"].as<double>();
        point.pitch_max = entry.second["pitch_max"].as<double>();
      } catch (const YAML::Exception &) {
        reason = "invalid entry in '" + path +
          "': every key must be a yaw[rad] mapped to {pitch_min: <rad>, pitch_max: <rad>}";
        return false;
      }
      if (!std::isfinite(point.yaw_rad) || !std::isfinite(point.pitch_min) ||
        !std::isfinite(point.pitch_max))
      {
        reason = "non-finite value in '" + path + "'";
        return false;
      }
      if (point.pitch_min > point.pitch_max) {
        reason = "pitch_min must be <= pitch_max in '" + path + "'";
        return false;
      }
      points.push_back(point);
    }

    if (points.size() < 2) {
      reason = "'" + path + "' needs at least two envelope points";
      return false;
    }

    std::sort(
      points.begin(), points.end(),
      [](const EnvelopePoint & lhs, const EnvelopePoint & rhs) {
        return lhs.yaw_rad < rhs.yaw_rad;
      });
    for (size_t i = 1; i < points.size(); ++i) {
      if (points[i].yaw_rad <= points[i - 1].yaw_rad) {
        reason = "duplicated yaw in '" + path + "'";
        return false;
      }
    }

    envelope.points_ = std::move(points);
    return true;
  }

  bool empty() const {return points_.empty();}
  size_t size() const {return points_.size();}
  double yawMin() const {return points_.front().yaw_rad;}
  double yawMax() const {return points_.back().yaw_rad;}

  void setMotorLimits(const MotorLimits & limits) {motor_ = limits;}
  const MotorLimits & motorLimits() const {return motor_;}

  /// 逆引きに持たせる余裕[rad]。境界上での取りこぼしを防ぐ。
  void setPitchMargin(double margin) {pitch_margin_ = margin;}
  double pitchMargin() const {return pitch_margin_;}

  /// 指定ヨーでの許容ピッチ範囲。定義域外は端点の値を使う。
  PitchRange pitchRangeAt(double yaw) const
  {
    if (points_.empty()) {
      return PitchRange{motor_.pitch_min, motor_.pitch_max};
    }
    if (yaw <= yawMin()) {
      return PitchRange{points_.front().pitch_min, points_.front().pitch_max};
    }
    if (yaw >= yawMax()) {
      return PitchRange{points_.back().pitch_min, points_.back().pitch_max};
    }
    for (size_t i = 1; i < points_.size(); ++i) {
      const EnvelopePoint & upper = points_[i];
      if (yaw > upper.yaw_rad) {
        continue;
      }
      const EnvelopePoint & lower = points_[i - 1];
      const double ratio = (yaw - lower.yaw_rad) / (upper.yaw_rad - lower.yaw_rad);
      return PitchRange{
        lower.pitch_min + (upper.pitch_min - lower.pitch_min) * ratio,
        lower.pitch_max + (upper.pitch_max - lower.pitch_max) * ratio};
    }
    return PitchRange{points_.back().pitch_min, points_.back().pitch_max};
  }

  /// [yaw_a, yaw_b] を掃く間ずっと満たせるピッチ範囲。
  ///
  /// 折れ線なので、両端と区間内の折れ点だけ見れば最小・最大が決まる。
  /// 砲身が通過する全ての角度で包絡線を満たすために使う。
  PitchRange pitchRangeOverSpan(double yaw_a, double yaw_b) const
  {
    const double lo = std::min(yaw_a, yaw_b);
    const double hi = std::max(yaw_a, yaw_b);

    PitchRange range = pitchRangeAt(lo);
    const PitchRange at_hi = pitchRangeAt(hi);
    range.min = std::max(range.min, at_hi.min);
    range.max = std::min(range.max, at_hi.max);

    for (const EnvelopePoint & point : points_) {
      if (point.yaw_rad <= lo || point.yaw_rad >= hi) {
        continue;
      }
      range.min = std::max(range.min, point.pitch_min);
      range.max = std::min(range.max, point.pitch_max);
    }
    return range;
  }

  /// そのピッチのままで居られるヨーの範囲。
  ///
  /// around_yaw を含む連続区間を返す。around_yaw 自体が包絡線から外れていれば
  /// 幅ゼロの区間（= ヨーを動かさない）を返す。
  YawRange yawRangeFor(double pitch, double around_yaw) const
  {
    if (points_.empty()) {
      return YawRange{motor_.yaw_min, motor_.yaw_max};
    }

    const double around = std::clamp(around_yaw, yawMin(), yawMax());
    if (!isPitchAllowedAt(around, pitch)) {
      return YawRange{around, around};
    }

    return YawRange{extend(around, pitch, false), extend(around, pitch, true)};
  }

  /// 指定ヨーでそのピッチが許されるか（余裕込み）。
  bool isPitchAllowedAt(double yaw, double pitch) const
  {
    const PitchRange range = pitchRangeAt(yaw);
    return pitch >= (range.min - pitch_margin_) && pitch <= (range.max + pitch_margin_);
  }

  /// 要求角に制限をかける。
  ///
  /// measured_* には実測角（無ければ現在の指令角）を渡す。
  EnvelopeResult apply(
    double yaw_request, double pitch_request,
    double measured_yaw, double measured_pitch) const
  {
    EnvelopeResult result;
    result.yaw = std::clamp(yaw_request, motor_.yaw_min, motor_.yaw_max);
    result.pitch = std::clamp(pitch_request, motor_.pitch_min, motor_.pitch_max);

    if (points_.empty()) {
      return result;
    }

    // 1. 包絡線の定義域へ収める。
    result.yaw = std::clamp(result.yaw, yawMin(), yawMax());

    // 2. 実測ピッチのままで居られるヨーの範囲に収める。
    //    これが「ピッチが下がるまでヨーを待たせる」を担う。
    const YawRange allowed_yaw = yawRangeFor(measured_pitch, measured_yaw);
    result.yaw = std::clamp(result.yaw, allowed_yaw.min, allowed_yaw.max);
    result.recovering = allowed_yaw.min >= allowed_yaw.max &&
      !isPitchAllowedAt(std::clamp(measured_yaw, yawMin(), yawMax()), measured_pitch);

    // 3. 現在ヨーから指令ヨーまで掃く全区間で満たせるピッチに収める。
    //    戻る向きにヨーが動くとき、砲身がまだ制限領域に居るのに
    //    ピッチを解放してしまうことを防ぐ。
    const PitchRange allowed_pitch =
      pitchRangeOverSpan(std::clamp(measured_yaw, yawMin(), yawMax()), result.yaw);
    if (!allowed_pitch.empty()) {
      result.pitch = std::clamp(result.pitch, allowed_pitch.min, allowed_pitch.max);
    }

    // 4. モータ可動範囲で最終クランプ。
    result.yaw = std::clamp(result.yaw, motor_.yaw_min, motor_.yaw_max);
    result.pitch = std::clamp(result.pitch, motor_.pitch_min, motor_.pitch_max);
    return result;
  }

private:
  /// around から左右どちらかへ、ピッチが許される限り伸ばした端を返す。
  double extend(double around, double pitch, bool towards_max) const
  {
    double edge = around;
    if (towards_max) {
      for (size_t i = 1; i < points_.size(); ++i) {
        if (points_[i].yaw_rad <= edge) {
          continue;
        }
        const double from = std::max(points_[i - 1].yaw_rad, edge);
        const double crossing = findCrossing(from, points_[i].yaw_rad, pitch, true);
        edge = crossing;
        if (crossing < points_[i].yaw_rad) {
          break;
        }
      }
    } else {
      for (size_t i = points_.size(); i-- > 1; ) {
        if (points_[i - 1].yaw_rad >= edge) {
          continue;
        }
        const double from = std::min(points_[i].yaw_rad, edge);
        const double crossing = findCrossing(points_[i - 1].yaw_rad, from, pitch, false);
        edge = crossing;
        if (crossing > points_[i - 1].yaw_rad) {
          break;
        }
      }
    }
    return edge;
  }

  /// [y0, y1] を towards_max 方向へ辿り、ピッチが許されなくなる位置を返す。
  ///
  /// 区間内は線形なので、上限・下限それぞれとの交点を解いて手前を取る。
  double findCrossing(double y0, double y1, double pitch, bool towards_max) const
  {
    if (y1 <= y0) {
      return towards_max ? y1 : y0;
    }
    const PitchRange at0 = pitchRangeAt(y0);
    const PitchRange at1 = pitchRangeAt(y1);

    const double start = towards_max ? y0 : y1;
    const double end = towards_max ? y1 : y0;
    const PitchRange at_start = towards_max ? at0 : at1;
    const PitchRange at_end = towards_max ? at1 : at0;

    if (!allowed(at_start, pitch)) {
      return start;
    }
    if (allowed(at_end, pitch)) {
      return end;
    }

    // 端で許されなくなるので、上限側・下限側それぞれの交点のうち手前を取る。
    double ratio = 1.0;
    ratio = std::min(
      ratio, crossRatio(at_start.max + pitch_margin_, at_end.max + pitch_margin_, pitch, true));
    ratio = std::min(
      ratio, crossRatio(at_start.min - pitch_margin_, at_end.min - pitch_margin_, pitch, false));
    ratio = std::clamp(ratio, 0.0, 1.0);
    return start + (end - start) * ratio;
  }

  static bool allowedWithMargin(const PitchRange & range, double pitch, double margin)
  {
    return pitch >= (range.min - margin) && pitch <= (range.max + margin);
  }

  bool allowed(const PitchRange & range, double pitch) const
  {
    return allowedWithMargin(range, pitch, pitch_margin_);
  }

  /// 境界値が bound_start -> bound_end と変化するとき、pitch が条件を
  /// 満たさなくなる位置の比率 [0,1] を返す。満たし続けるなら 1。
  ///
  /// upper=true なら「pitch <= bound」、false なら「pitch >= bound」。
  static double crossRatio(double bound_start, double bound_end, double pitch, bool upper)
  {
    const auto ok = [upper, pitch](double bound) {
        return upper ? (pitch <= bound) : (pitch >= bound);
      };
    if (ok(bound_end)) {
      return 1.0;
    }
    if (!ok(bound_start)) {
      return 0.0;
    }
    const double denom = bound_end - bound_start;
    if (std::fabs(denom) < 1e-12) {
      return 1.0;
    }
    return (pitch - bound_start) / denom;
  }

  std::vector<EnvelopePoint> points_;
  MotorLimits motor_;
  double pitch_margin_ = 0.01;
};

}  // namespace core_shooter

#endif  // CORE_SHOOTER__TURRET_ENVELOPE_HPP_
