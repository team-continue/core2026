#include "core_planning/path_smoother.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <stdexcept>
#include <string>

namespace core_planning
{
namespace
{
using Point = std::pair<double, double>;

// これより短い区間は、重なった点として除く
constexpr double kDegenerateSegmentM = 1e-9;
// 1 回の移動がこれより小さければ動かさない
constexpr double kMinMoveM = 1e-9;
// 1 周で動いた最大の距離がこれより小さければ、収束したとして終える
constexpr double kConvergedMoveM = 1e-4;
// 増やした点の数の上限。範囲の値なら届かない（60m 四方のフィールドを 0.02m 間隔でも数万点）
constexpr double kMaxPoints = 1e6;
// 端点からこの道のりの間は、取りたい距離を端点の距離から少しずつ上げる。
// 端点は動かないので、目標の距離のまま隣の点を押し出すと、端点で鉤形に曲がる（機体が障害物の近く
// にいるとき、ゴールが障害物の近くにあるとき）。壁と平行に 0.7m の位置から始まる経路は、上げないと
// 最初の 0.1m で 0.19m 横へ出る（約 60°）。1m で上げると最初の区間の向きの変化は約 15° になる
constexpr double kEndpointRampM = 1.0;
// 押し出しが尾根を越えるとき、尾根までのうち押す割合
constexpr double kRidgeApproach = 0.5;
// bending で、余裕が足りない点を押した位置に寄せる重み（足りている点は 1）
constexpr double kBendingClearanceWeight = 50.0;
// bending で、守る距離を下回るとき全体の動く量を半分にする回数の上限
constexpr int kMaxStepHalvings = 4;
// corridor の ADMM の回数（円を作り直す 1 回あたり）と、罰の重み ρ の √λ に対する比
constexpr int kAdmmIterations = 20;
constexpr double kCorridorRhoRatio = 1.0;
// corridor の範囲を作り直す回数は iterations / この値
constexpr int kCorridorOuterDivisor = 4;
// corridor で 1 回に動ける距離（距離場を線形にした近似が成り立つ範囲）[m]
constexpr double kCorridorTrustM = 0.3;
// 半平面と円の重なりへの射影の回数
constexpr int kRegionProjections = 4;

void requireRange(const char * name, const double value, const double min, const double max)
{
  if (!std::isfinite(value) || value < min || value > max) {
    std::ostringstream message;
    message << "経路の曲線化の " << name << " は " << min << "〜" << max << " にしてください: " <<
      value;
    throw std::invalid_argument(message.str());
  }
}

void validate(const double hard_radius_m, const SmoothingParams & params)
{
  requireRange("hard_radius_m", hard_radius_m, 0.0, 1e3);
  requireRange("spacing_m", params.spacing_m, 1e-6, 1e3);
  requireRange("extra_clearance_m", params.extra_clearance_m, 0.0, 1e3);
  requireRange("iterations", static_cast<double>(params.iterations), 0.0, 1e6);
  requireRange("smooth_weight", params.smooth_weight, 0.0, 1.0);
  requireRange("clearance_weight", params.clearance_weight, 0.0, 1.0);
  requireRange("bending_length_m", params.bending_length_m, 1e-3, 100.0);
}

double cellDistance(const DistanceField & field, const Point & point)
{
  return static_cast<double>(field.at(point.first, point.second));
}

Point midpoint(const Point & a, const Point & b)
{
  return {0.5 * (a.first + b.first), 0.5 * (a.second + b.second)};
}

// 距離場をセルの中心の間で双一次補間した距離と、その勾配
struct DistanceSample
{
  double distance{0.0};
  double gradient_x{0.0};
  double gradient_y{0.0};
};

DistanceSample sampleDistance(const DistanceField & field, const Point & point)
{
  const GridInfo & info = field.info();
  const double u = (point.first - info.origin_x) / info.resolution - 0.5;
  const double v = (point.second - info.origin_y) / info.resolution - 0.5;
  const int x = static_cast<int>(std::floor(u));
  const int y = static_cast<int>(std::floor(v));
  const double fx = u - x;
  const double fy = v - y;
  const double d00 = field.at(GridCell{x, y});
  const double d10 = field.at(GridCell{x + 1, y});
  const double d01 = field.at(GridCell{x, y + 1});
  const double d11 = field.at(GridCell{x + 1, y + 1});
  return DistanceSample{
    (1.0 - fx) * (1.0 - fy) * d00 + fx * (1.0 - fy) * d10 + (1.0 - fx) * fy * d01 + fx * fy * d11,
    ((1.0 - fy) * (d10 - d00) + fy * (d11 - d01)) / info.resolution,
    ((1.0 - fx) * (d01 - d00) + fx * (d11 - d10)) / info.resolution};
}

struct Resampled
{
  std::vector<Point> points;
  // 最初の区間の終わりの点と、最後の区間の始まりの点の番号
  std::size_t first_segment_end{0U};
  std::size_t last_segment_begin{0U};
};

// 各区間を spacing 以下の等しい間隔に分ける。元の頂点はそのまま残す
Resampled resample(const std::vector<Point> & polyline, const double spacing)
{
  double total_length = 0.0;
  for (std::size_t i = 1U; i < polyline.size(); ++i) {
    total_length += std::hypot(
      polyline[i].first - polyline[i - 1U].first, polyline[i].second - polyline[i - 1U].second);
  }
  if (total_length / spacing > kMaxPoints) {
    std::ostringstream message;
    message << "経路が長すぎて曲線にできません: 長さ " << total_length << "m、間隔 " << spacing << "m";
    throw std::invalid_argument(message.str());
  }

  Resampled resampled;
  resampled.points.reserve(static_cast<std::size_t>(total_length / spacing) + polyline.size());
  resampled.points.push_back(polyline.front());
  bool first_segment = true;
  for (std::size_t i = 1U; i < polyline.size(); ++i) {
    const auto [x0, y0] = resampled.points.back();
    const auto [x1, y1] = polyline[i];
    const double length = std::hypot(x1 - x0, y1 - y0);
    if (length <= kDegenerateSegmentM) {
      continue;
    }
    const std::size_t segment_begin = resampled.points.size() - 1U;
    const auto count = static_cast<std::size_t>(std::max(1.0, std::ceil(length / spacing - 1e-9)));
    for (std::size_t k = 1U; k < count; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(count);
      resampled.points.emplace_back(x0 + t * (x1 - x0), y0 + t * (y1 - y0));
    }
    resampled.points.push_back(polyline[i]);
    if (first_segment) {
      resampled.first_segment_end = resampled.points.size() - 1U;
      first_segment = false;
    }
    resampled.last_segment_begin = segment_begin;
  }
  // 重なった点を除いても、終点は元の終点と同じにする
  resampled.points.back() = polyline.back();
  return resampled;
}

// 点と、隣り合う点の中点で守る距離
struct Floors
{
  std::vector<double> points;
  // midpoints[i] は points[i] と points[i + 1] の中点
  std::vector<double> midpoints;
};

// 最初と最後の区間は、元の折れ線の距離が hard_radius_m より近ければ、その距離を守る（ヘッダの説明）
Floors makeFloors(const Resampled & resampled, const DistanceField & field, const double hard_radius)
{
  const auto & points = resampled.points;
  const auto in_end_segment = [&resampled](const std::size_t first, const std::size_t last) {
      return last <= resampled.first_segment_end || first >= resampled.last_segment_begin;
    };
  Floors floors;
  floors.points.reserve(points.size());
  floors.midpoints.reserve(points.size());
  for (std::size_t i = 0U; i < points.size(); ++i) {
    floors.points.push_back(
      in_end_segment(i, i) ? std::min(hard_radius, cellDistance(field, points[i])) : hard_radius);
  }
  for (std::size_t i = 0U; i + 1U < points.size(); ++i) {
    floors.midpoints.push_back(
      in_end_segment(i, i + 1U) ?
      std::min(hard_radius, cellDistance(field, midpoint(points[i], points[i + 1U]))) :
      hard_radius);
  }
  return floors;
}

bool satisfiesFloors(
  const std::vector<Point> & points, const Floors & floors, const DistanceField & field)
{
  for (std::size_t i = 0U; i < points.size(); ++i) {
    if (cellDistance(field, points[i]) < floors.points[i]) {
      return false;
    }
  }
  for (std::size_t i = 0U; i + 1U < points.size(); ++i) {
    if (cellDistance(field, midpoint(points[i], points[i + 1U])) < floors.midpoints[i]) {
      return false;
    }
  }
  return true;
}

// 点ごとに取りたい距離。端点の近くでは端点の距離から目標の距離まで道のりに比例して上げる
std::vector<double> makeTargets(
  const std::vector<Point> & points, const DistanceField & field, const double full_target)
{
  std::vector<double> along(points.size(), 0.0);
  for (std::size_t i = 1U; i < points.size(); ++i) {
    along[i] = along[i - 1U] + std::hypot(
      points[i].first - points[i - 1U].first, points[i].second - points[i - 1U].second);
  }
  const double total = along.back();
  const double start_distance = sampleDistance(field, points.front()).distance;
  const double goal_distance = sampleDistance(field, points.back()).distance;
  std::vector<double> targets(points.size());
  std::transform(
    along.begin(), along.end(), targets.begin(),
    [total, start_distance, goal_distance, full_target](const double s) {
      const bool near_start = s <= total - s;
      const double from_end = near_start ? s : total - s;
      const double base = near_start ? start_distance : goal_distance;
      if (base >= full_target || from_end >= kEndpointRampM) {
        return full_target;
      }
      return base + (full_target - base) * from_end / kEndpointRampM;
    });
  return targets;
}

// from から to への線分の上で、補間した距離がいちばん遠い点の位置 t（0〜1。同じなら from に近い方）。
// 双一次補間の距離は、セルの中心を結ぶ格子線の間ではほぼ線形なので、格子線と交わる所と to を比べる
double farthestOnSegment(const DistanceField & field, const Point & from, const Point & to)
{
  const GridInfo & info = field.info();
  const auto grid_u = [&info](const double coordinate, const double origin) {
      return (coordinate - origin) / info.resolution - 0.5;
    };
  std::vector<double> candidates{1.0};
  const auto add_crossings = [&candidates](const double a, const double b) {
      if (std::abs(b - a) < 1e-12) {
        return;
      }
      for (double k = std::ceil(std::min(a, b)); k <= std::floor(std::max(a, b)); k += 1.0) {
        const double t = (k - a) / (b - a);
        if (t > 0.0 && t < 1.0) {
          candidates.push_back(t);
        }
      }
    };
  add_crossings(grid_u(from.first, info.origin_x), grid_u(to.first, info.origin_x));
  add_crossings(grid_u(from.second, info.origin_y), grid_u(to.second, info.origin_y));
  std::sort(candidates.begin(), candidates.end());
  double best = 0.0;
  double best_distance = sampleDistance(field, from).distance;
  for (const double t : candidates) {
    const Point point{
      from.first + t * (to.first - from.first), from.second + t * (to.second - from.second)};
    const double distance = sampleDistance(field, point).distance;
    if (distance > best_distance) {
      best = t;
      best_distance = distance;
    }
  }
  return best;
}

// 点 point を、余裕が足りなければ障害物から離れる向きへ押した先（足りていればそのまま）。狭い通路では
// 真ん中でも余裕が足りず両側から押されるので、そのまま押すと尾根（両側の障害物から等距離の線）を越えて
// 左右に行き来し、1 点ごとに折れる。押す線分の上で距離がいちばん遠い所が途中にあれば（尾根を越える）、
// そこまでのうち kRidgeApproach の割合だけ押す。尾根の上まで押さないのは、斜めの尾根は格子では階段になり、
// 点が段に乗って折れるため
Point pushedPoint(
  const Point & point, const DistanceField & field, const double target_distance,
  const double push_range, const SmoothingParams & params)
{
  // セルの距離が十分遠ければ、補間した距離も目標より遠いので調べない
  if (cellDistance(field, point) >= push_range) {
    return point;
  }
  const DistanceSample sample = sampleDistance(field, point);
  const double deficit = target_distance - sample.distance;
  if (deficit <= 0.0) {
    return point;
  }
  // 勾配の大きさは 1 を超えないようにする
  const double norm = std::hypot(sample.gradient_x, sample.gradient_y);
  const double scale = params.clearance_weight * deficit / std::max(norm, 1.0);
  const Point pushed{
    point.first + scale * sample.gradient_x, point.second + scale * sample.gradient_y};
  const double t = farthestOnSegment(field, point, pushed);
  if (t >= 1.0) {
    return pushed;
  }
  const double approach = kRidgeApproach * t * scale;
  return {point.first + approach * sample.gradient_x, point.second + approach * sample.gradient_y};
}

// 1 点を動かした先（laplacian）。両隣の中点へ引いて（なめらかさ）から、pushedPoint で押す
Point movedPoint(
  const Point & previous, const Point & current, const Point & next, const DistanceField & field,
  const double target_distance, const double push_range, const SmoothingParams & params)
{
  const Point center = midpoint(previous, next);
  const Point smoothed{
    current.first + params.smooth_weight * (center.first - current.first),
    current.second + params.smooth_weight * (center.second - current.second)};
  return pushedPoint(smoothed, field, target_distance, push_range, params);
}

// 端点以外の点を動かす。動かした点で、その点と両隣との中点が守る距離を満たす移動だけを採るので、
// 初めに満たしていれば最後まで満たす
std::vector<Point> relax(
  std::vector<Point> points, const Floors & floors, const DistanceField & field,
  const double target_distance, const SmoothingParams & params)
{
  const std::size_t count = points.size();
  if (count < 3U) {
    return points;
  }
  const std::vector<double> targets = makeTargets(points, field, target_distance);
  // 双一次補間の値は、点のあるセルの値と最大で対角 1 つ分しか違わない
  const double push_range = target_distance + 2.0 * field.info().resolution;
  for (int iteration = 0; iteration < params.iterations; ++iteration) {
    // 動かした点をすぐ次の点に使うので、向きを交互に変えて片側へ寄る偏りをなくす
    const bool forward = iteration % 2 == 0;
    double largest_move = 0.0;
    for (std::size_t k = 1U; k + 1U < count; ++k) {
      const std::size_t i = forward ? k : count - 1U - k;
      const Point candidate = movedPoint(
        points[i - 1U], points[i], points[i + 1U], field, targets[i], push_range, params);
      const double move =
        std::hypot(candidate.first - points[i].first, candidate.second - points[i].second);
      if (move < kMinMoveM ||
        cellDistance(field, candidate) < floors.points[i] ||
        cellDistance(field, midpoint(points[i - 1U], candidate)) < floors.midpoints[i - 1U] ||
        cellDistance(field, midpoint(candidate, points[i + 1U])) < floors.midpoints[i])
      {
        continue;
      }
      points[i] = candidate;
      largest_move = std::max(largest_move, move);
    }
    if (largest_move < kConvergedMoveM) {
      break;
    }
  }
  return points;
}

// 対称で正定値の五重対角の行列（対角 diagonal、1 つ隣 first、2 つ隣 second）のコレスキー分解 L L^T。
// 行列が変わらなければ、分解を 1 度だけして右辺を変えて何度も解ける
class PentadiagonalCholesky
{
public:
  PentadiagonalCholesky(
    const std::vector<double> & diagonal, const std::vector<double> & first,
    const std::vector<double> & second)
  : l0_(diagonal.size()), l1_(diagonal.size(), 0.0), l2_(diagonal.size(), 0.0)
  {
    for (std::size_t i = 0U; i < diagonal.size(); ++i) {
      const double l2 = i >= 2U ? second[i - 2U] / l0_[i - 2U] : 0.0;
      const double l1 = i >= 1U ? (first[i - 1U] - (i >= 2U ? l2 * l1_[i - 1U] : 0.0)) / l0_[i - 1U] :
        0.0;
      const double pivot = diagonal[i] - l1 * l1 - l2 * l2;
      if (!(pivot > 0.0)) {
        throw std::invalid_argument("曲げの平滑化の行列が正定値ではありません");
      }
      l0_[i] = std::sqrt(pivot);
      l1_[i] = l1;
      l2_[i] = l2;
    }
  }

  std::vector<double> solve(std::vector<double> rhs) const
  {
    const std::size_t n = rhs.size();
    for (std::size_t i = 0U; i < n; ++i) {
      double value = rhs[i];
      if (i >= 1U) {
        value -= l1_[i] * rhs[i - 1U];
      }
      if (i >= 2U) {
        value -= l2_[i] * rhs[i - 2U];
      }
      rhs[i] = value / l0_[i];
    }
    for (std::size_t k = n; k-- > 0U; ) {
      double value = rhs[k];
      if (k + 1U < n) {
        value -= l1_[k + 1U] * rhs[k + 1U];
      }
      if (k + 2U < n) {
        value -= l2_[k + 2U] * rhs[k + 2U];
      }
      rhs[k] = value / l0_[k];
    }
    return rhs;
  }

private:
  std::vector<double> l0_;
  std::vector<double> l1_;
  std::vector<double> l2_;
};

// 端点以外の点を、曲げのエネルギー（2 階差分の二乗和、曲率の二乗の和に当たる）が小さくなるように動かす
// （bending）。毎回、余裕が足りない点を pushedPoint で押した位置 q に対して
//   min_p  Σ w_i |p_i − q_i|² + λ Σ|p_{i−1} − 2 p_i + p_{i+1}|²   （端点は固定）
// を解く（λ = (bending_length_m / spacing_m)^4。曲がりはおよそ bending_length_m の長さに広がる）。
// 余裕が足りない点は w_i = kBendingClearanceWeight で押した位置に強く寄せ、曲げの力に負けて障害物へ寄らない
// ようにする。それ以外は w_i = 1。中点へ引く laplacian は線の長さを小さくするので、曲がりが折れ線の角に
// 集まる。こちらは曲がりを広げるので、最大の曲率が下がる。障害物から遠い直線は 2 階差分が 0 なので動かない。
// 動かした先で守る距離を下回る所があれば、全体の動く量を半分ずつにして確かめる（点ごとに戻すと段ができる）。
// 半分を kMaxStepHalvings 回しても下回れば、下回る点だけを前の位置に戻す
std::vector<Point> relaxBending(
  std::vector<Point> points, const Floors & floors, const DistanceField & field,
  const double target_distance, const SmoothingParams & params)
{
  const std::size_t count = points.size();
  if (count < 4U) {
    return relax(std::move(points), floors, field, target_distance, params);
  }
  const double lambda = std::pow(params.bending_length_m / params.spacing_m, 4.0);
  // 全点の DᵀD（D は 2 階差分）を帯で作る
  std::vector<double> dd0(count, 0.0);
  std::vector<double> dd1(count, 0.0);
  std::vector<double> dd2(count, 0.0);
  for (std::size_t k = 1U; k + 1U < count; ++k) {
    dd0[k - 1U] += 1.0;
    dd0[k] += 4.0;
    dd0[k + 1U] += 1.0;
    dd1[k - 1U] += -2.0;
    dd1[k] += -2.0;
    dd2[k - 1U] += 1.0;
  }
  const std::size_t unknowns = count - 2U;
  const std::size_t last = count - 1U;
  std::vector<double> first(unknowns, 0.0);
  std::vector<double> second(unknowns, 0.0);
  for (std::size_t j = 0U; j < unknowns; ++j) {
    const std::size_t i = j + 1U;
    if (j + 1U < unknowns) {
      first[j] = lambda * dd1[i];
    }
    if (j + 2U < unknowns) {
      second[j] = lambda * dd2[i];
    }
  }
  const std::vector<double> targets = makeTargets(points, field, target_distance);
  const double push_range = target_distance + 2.0 * field.info().resolution;
  const Point start = points.front();
  const Point goal = points.back();
  const auto violates = [&](const std::vector<Point> & candidate) {
      for (std::size_t i = 1U; i < last; ++i) {
        if (cellDistance(field, candidate[i]) < floors.points[i]) {
          return true;
        }
      }
      for (std::size_t i = 0U; i < last; ++i) {
        if (cellDistance(field, midpoint(candidate[i], candidate[i + 1U])) < floors.midpoints[i]) {
          return true;
        }
      }
      return false;
    };

  for (int iteration = 0; iteration < params.iterations; ++iteration) {
    std::vector<double> diagonal(unknowns);
    std::vector<double> rhs_x(unknowns);
    std::vector<double> rhs_y(unknowns);
    for (std::size_t j = 0U; j < unknowns; ++j) {
      const std::size_t i = j + 1U;
      const Point q = pushedPoint(points[i], field, targets[i], push_range, params);
      const double weight = q != points[i] ? kBendingClearanceWeight : 1.0;
      diagonal[j] = weight + lambda * dd0[i];
      rhs_x[j] = weight * q.first;
      rhs_y[j] = weight * q.second;
    }
    // 固定した端点の分を右辺へ移す（DᵀD の端点との成分）
    const auto move_fixed = [&](const std::size_t i, const double coefficient, const Point & fixed) {
        rhs_x[i - 1U] -= lambda * coefficient * fixed.first;
        rhs_y[i - 1U] -= lambda * coefficient * fixed.second;
      };
    move_fixed(1U, dd1[0U], start);
    move_fixed(2U, dd2[0U], start);
    move_fixed(last - 1U, dd1[last - 1U], goal);
    move_fixed(last - 2U, dd2[last - 2U], goal);
    const PentadiagonalCholesky solver(diagonal, first, second);
    const std::vector<double> xs = solver.solve(std::move(rhs_x));
    const std::vector<double> ys = solver.solve(std::move(rhs_y));

    std::vector<Point> candidate = points;
    double step = 1.0;
    for (int halving = 0; halving <= kMaxStepHalvings; ++halving, step *= 0.5) {
      for (std::size_t j = 0U; j < unknowns; ++j) {
        const Point & from = points[j + 1U];
        candidate[j + 1U] = Point{
          from.first + step * (xs[j] - from.first), from.second + step * (ys[j] - from.second)};
      }
      if (!violates(candidate)) {
        break;
      }
    }
    // 半分にしても下回る点は前の位置に戻す。戻すと隣との中点が変わるので、下回る所がなくなるまで繰り返す
    // （すべて戻せば前の点の並びで、それは守る距離を満たしている）
    bool reverted = true;
    while (reverted) {
      reverted = false;
      for (std::size_t i = 1U; i < last; ++i) {
        if (candidate[i] != points[i] && cellDistance(field, candidate[i]) < floors.points[i]) {
          candidate[i] = points[i];
          reverted = true;
        }
      }
      for (std::size_t i = 0U; i < last; ++i) {
        if (cellDistance(field, midpoint(candidate[i], candidate[i + 1U])) < floors.midpoints[i]) {
          for (const std::size_t k : {i, i + 1U}) {
            if (k != 0U && k != last && candidate[k] != points[k]) {
              candidate[k] = points[k];
              reverted = true;
            }
          }
        }
      }
    }
    double largest_move = 0.0;
    for (std::size_t i = 1U; i < last; ++i) {
      largest_move = std::max(
        largest_move,
        std::hypot(candidate[i].first - points[i].first, candidate[i].second - points[i].second));
    }
    points = std::move(candidate);
    if (largest_move < kConvergedMoveM) {
      break;
    }
  }
  return points;
}

// 全点の DᵀD（D は 2 階差分）の帯（対角、1 つ隣、2 つ隣）
struct BendingBands
{
  std::vector<double> d0;
  std::vector<double> d1;
  std::vector<double> d2;
};

BendingBands bendingBands(const std::size_t count)
{
  BendingBands bands{std::vector<double>(count, 0.0), std::vector<double>(count, 0.0),
    std::vector<double>(count, 0.0)};
  for (std::size_t k = 1U; k + 1U < count; ++k) {
    bands.d0[k - 1U] += 1.0;
    bands.d0[k] += 4.0;
    bands.d0[k + 1U] += 1.0;
    bands.d1[k - 1U] += -2.0;
    bands.d1[k] += -2.0;
    bands.d2[k - 1U] += 1.0;
  }
  return bands;
}

// corridor の 1 点の範囲。障害物から離れる側の半平面 normal·p >= offset（距離場を今の点で線形にして、守る
// 距離を保つ側）と、今の点を中心とする半径 radius の円（線形にした近似が成り立つ範囲）の重なり
struct CorridorRegion
{
  Point center;
  double radius{0.0};
  // normal は単位ベクトル。勾配がなければ（尾根の上など）半平面を使わない
  bool has_half_plane{false};
  Point normal;
  double offset{0.0};
};

// 点 point を、中心 center・半径 radius の円の中へ射影する
Point projectToDisk(const Point & point, const Point & center, const double radius)
{
  const double dx = point.first - center.first;
  const double dy = point.second - center.second;
  const double distance = std::hypot(dx, dy);
  if (distance <= radius) {
    return point;
  }
  const double scale = radius / distance;
  return {center.first + scale * dx, center.second + scale * dy};
}

// 半平面と円の重なりへの射影（交互に射影する。数回で十分近い）
Point projectToRegion(Point point, const CorridorRegion & region)
{
  for (int k = 0; k < kRegionProjections; ++k) {
    point = projectToDisk(point, region.center, region.radius);
    if (!region.has_half_plane) {
      break;
    }
    const double violation =
      region.offset - (region.normal.first * point.first + region.normal.second * point.second);
    if (violation <= 0.0) {
      break;
    }
    point = {point.first + violation * region.normal.first, point.second + violation * region.normal.second};
  }
  return point;
}

// corridor: bending と同じ目的（曲げのエネルギー + 余裕が足りない点を押した位置へ寄せる）を、各点が
// CorridorRegion の中にあるという条件の下で解く（ADMM。x の更新は bending と同じ五重対角の連立方程式）。
// 範囲は毎回いまの点で作り直す（逐次に凸にする）。半平面は距離場を今の点で線形にし、守る距離に格子の分を
// 足した距離を保つ側。bending は守る距離を下回った所を後から戻すが、こちらは初めから条件として解く
std::vector<Point> relaxCorridor(
  std::vector<Point> points, const Floors & floors, const DistanceField & field,
  const double target_distance, const SmoothingParams & params)
{
  const std::size_t count = points.size();
  if (count < 4U) {
    return relax(std::move(points), floors, field, target_distance, params);
  }
  const double lambda = std::pow(params.bending_length_m / params.spacing_m, 4.0);
  const double rho = kCorridorRhoRatio * std::sqrt(lambda);
  const BendingBands bands = bendingBands(count);
  const std::size_t unknowns = count - 2U;
  const std::size_t last = count - 1U;
  std::vector<double> first(unknowns, 0.0);
  std::vector<double> second(unknowns, 0.0);
  for (std::size_t j = 0U; j < unknowns; ++j) {
    if (j + 1U < unknowns) {
      first[j] = lambda * bands.d1[j + 1U];
    }
    if (j + 2U < unknowns) {
      second[j] = lambda * bands.d2[j + 1U];
    }
  }
  const std::vector<double> targets = makeTargets(points, field, target_distance);
  const double push_range = target_distance + 2.0 * field.info().resolution;
  const double cell_margin = field.info().resolution;
  const Point start = points.front();
  const Point goal = points.back();

  const int outer_iterations = std::max(1, params.iterations / kCorridorOuterDivisor);
  for (int outer = 0; outer < outer_iterations; ++outer) {
    std::vector<CorridorRegion> regions(unknowns);
    std::vector<double> diagonal(unknowns);
    std::vector<double> soft_x(unknowns);
    std::vector<double> soft_y(unknowns);
    for (std::size_t j = 0U; j < unknowns; ++j) {
      const std::size_t i = j + 1U;
      const DistanceSample sample = sampleDistance(field, points[i]);
      CorridorRegion & region = regions[j];
      region.center = points[i];
      region.radius = kCorridorTrustM;
      const double norm = std::hypot(sample.gradient_x, sample.gradient_y);
      if (norm > 0.5) {
        // d(p) ≈ d(c) + n·(p − c) >= floor + margin
        region.has_half_plane = true;
        region.normal = {sample.gradient_x / norm, sample.gradient_y / norm};
        region.offset = region.normal.first * points[i].first + region.normal.second * points[i].second +
          floors.points[i] + cell_margin - sample.distance;
      } else {
        // 尾根の上などで勾配がない。今の点から動ける距離を、守る距離までの余りに抑える
        region.radius = std::min(
          kCorridorTrustM, std::max(sample.distance - floors.points[i] - cell_margin, 0.0));
      }
      const Point q = pushedPoint(points[i], field, targets[i], push_range, params);
      const double weight = q != points[i] ? kBendingClearanceWeight : 1.0;
      diagonal[j] = weight + rho + lambda * bands.d0[i];
      soft_x[j] = weight * q.first;
      soft_y[j] = weight * q.second;
    }
    // 固定した端点の分（DᵀD の端点との成分）
    soft_x[0U] -= lambda * bands.d1[0U] * start.first;
    soft_y[0U] -= lambda * bands.d1[0U] * start.second;
    soft_x[1U] -= lambda * bands.d2[0U] * start.first;
    soft_y[1U] -= lambda * bands.d2[0U] * start.second;
    soft_x[unknowns - 1U] -= lambda * bands.d1[last - 1U] * goal.first;
    soft_y[unknowns - 1U] -= lambda * bands.d1[last - 1U] * goal.second;
    soft_x[unknowns - 2U] -= lambda * bands.d2[last - 2U] * goal.first;
    soft_y[unknowns - 2U] -= lambda * bands.d2[last - 2U] * goal.second;
    const PentadiagonalCholesky solver(diagonal, first, second);

    std::vector<Point> z(unknowns);
    for (std::size_t j = 0U; j < unknowns; ++j) {
      z[j] = regions[j].center;
    }
    std::vector<Point> u(unknowns, Point{0.0, 0.0});
    for (int k = 0; k < kAdmmIterations; ++k) {
      std::vector<double> rhs_x(soft_x);
      std::vector<double> rhs_y(soft_y);
      for (std::size_t j = 0U; j < unknowns; ++j) {
        rhs_x[j] += rho * (z[j].first - u[j].first);
        rhs_y[j] += rho * (z[j].second - u[j].second);
      }
      const std::vector<double> xs = solver.solve(std::move(rhs_x));
      const std::vector<double> ys = solver.solve(std::move(rhs_y));
      for (std::size_t j = 0U; j < unknowns; ++j) {
        const Point x{xs[j], ys[j]};
        z[j] = projectToRegion(Point{x.first + u[j].first, x.second + u[j].second}, regions[j]);
        u[j] = Point{u[j].first + x.first - z[j].first, u[j].second + x.second - z[j].second};
      }
    }
    std::vector<Point> candidate = points;
    for (std::size_t j = 0U; j < unknowns; ++j) {
      candidate[j + 1U] = z[j];
    }
    // 円は点の守る距離を保つが、隣との中点は保たない（通路が細く曲がる所）。下回れば前の点に戻す
    bool reverted = true;
    while (reverted) {
      reverted = false;
      for (std::size_t i = 1U; i < last; ++i) {
        if (candidate[i] != points[i] && cellDistance(field, candidate[i]) < floors.points[i]) {
          candidate[i] = points[i];
          reverted = true;
        }
      }
      for (std::size_t i = 0U; i < last; ++i) {
        if (cellDistance(field, midpoint(candidate[i], candidate[i + 1U])) < floors.midpoints[i]) {
          for (const std::size_t k : {i, i + 1U}) {
            if (k != 0U && k != last && candidate[k] != points[k]) {
              candidate[k] = points[k];
              reverted = true;
            }
          }
        }
      }
    }
    double largest_move = 0.0;
    for (std::size_t i = 1U; i < last; ++i) {
      largest_move = std::max(
        largest_move,
        std::hypot(candidate[i].first - points[i].first, candidate[i].second - points[i].second));
    }
    points = std::move(candidate);
    if (largest_move < kConvergedMoveM) {
      break;
    }
  }
  return points;
}
}  // namespace

std::optional<std::vector<std::pair<double, double>>> smoothPath(
  const std::vector<std::pair<double, double>> & polyline, const DistanceField & field,
  const double hard_radius_m, const SmoothingParams & params)
{
  validate(hard_radius_m, params);
  if (!params.enabled || polyline.size() < 2U) {
    return polyline;
  }
  const bool finite = std::all_of(
    polyline.begin(), polyline.end(),
    [](const Point & point) {return std::isfinite(point.first) && std::isfinite(point.second);});
  if (!finite) {
    throw std::invalid_argument("曲線にする経路に有限でない座標があります");
  }

  const Resampled resampled = resample(polyline, params.spacing_m);
  if (resampled.points.size() < 2U) {
    // すべての点が重なっている
    return polyline;
  }
  const Floors floors = makeFloors(resampled, field, hard_radius_m);
  if (!satisfiesFloors(resampled.points, floors, field)) {
    // 元の折れ線が途中で膨らみに入っている
    return std::nullopt;
  }
  const double target = hard_radius_m + params.extra_clearance_m;
  auto points = params.mode == SmoothingMode::kBending ?
    relaxBending(resampled.points, floors, field, target, params) :
    params.mode == SmoothingMode::kCorridor ?
    relaxCorridor(resampled.points, floors, field, target, params) :
    relax(resampled.points, floors, field, target, params);
  // 移動ごとに確かめているので通るはずだが、返す前に全点を確かめる
  if (!satisfiesFloors(points, floors, field)) {
    return std::nullopt;
  }
  return points;
}
SmoothingMode parseSmoothingMode(const std::string & name)
{
  if (name == "laplacian") {
    return SmoothingMode::kLaplacian;
  }
  if (name == "bending") {
    return SmoothingMode::kBending;
  }
  if (name == "corridor") {
    return SmoothingMode::kCorridor;
  }
  throw std::invalid_argument(
          "曲線化の方法は laplacian / bending / corridor のどれかにしてください: " + name);
}
}  // namespace core_planning
