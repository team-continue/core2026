#include "core_planning/safety.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "core_planning/footprint.hpp"

namespace core_planning::safety
{
namespace
{
constexpr double kStepS = 0.05;
constexpr double kMinSpeedMps = 1e-6;
// これより遅い不明の相手は止まった物とみなし、加速度の広がりを足さない（mppi.cpp の
// kMovingObjectSpeedMps と同じ）。ロボットと人は止まっていても動き出せるので常に足す
constexpr double kMovingObstacleSpeedMps = 0.3;
// 自機の速度の、相手の方向の成分がこれを超えれば、相手へ近づいているとみなす
constexpr double kApproachSpeedMps = 0.05;
// 候補の速度: 今の速度からの変化の大きさ（上限に対する割合）と、指令の向きに減速した倍率
constexpr std::array<double, 2> kChangeFractions{0.5, 1.0};
constexpr std::array<double, 3> kCommandFractions{0.75, 0.5, 0.25};
// 静止物までの距離を比べるときの丸めの分。距離の場は float なので、同じ段の値でも差がわずかにずれる
constexpr double kStaticCompareEpsilonM = 1e-6;
// 軌道の静止物までの距離を調べる間隔の、距離の場の格子に対する割合。軌道の点は 0.05 秒ごとで、2m/s では
// 0.1m 離れ、格子 0.05m の箱の角をまたいでしまうので、点の間を格子の半分ごとに分けて調べる
constexpr double kStaticProbeStepRatio = 0.5;

using Trajectory = std::vector<PlanarPose>;

std::size_t stepCount(const double horizon_s)
{
  return static_cast<std::size_t>(std::ceil(horizon_s / kStepS));
}

// 機体座標系の速度を保って進む軌道
Trajectory commandedTrajectory(
  const PlanarPose & start, const BodyVelocity & velocity, const double horizon_s)
{
  const std::size_t steps = stepCount(horizon_s);
  Trajectory trajectory;
  trajectory.reserve(steps + 1U);
  trajectory.push_back(start);
  PlanarPose pose = start;
  for (std::size_t i = 0U; i < steps; ++i) {
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    pose.x += (c * velocity.vx - s * velocity.vy) * kStepS;
    pose.y += (s * velocity.vx + c * velocity.vy) * kStepS;
    pose.yaw += velocity.wz * kStepS;
    trajectory.push_back(pose);
  }
  return trajectory;
}

// 今の速度から max_decel で止まるまでの軌道
Trajectory brakingTrajectory(
  const PlanarPose & start, const BodyVelocity & velocity, const double max_decel,
  const double horizon_s)
{
  const std::size_t steps = stepCount(horizon_s);
  const double speed = std::hypot(velocity.vx, velocity.vy);
  const double c = std::cos(start.yaw);
  const double s = std::sin(start.yaw);
  const double direction_x = speed > kMinSpeedMps ? (c * velocity.vx - s * velocity.vy) / speed : 0.0;
  const double direction_y = speed > kMinSpeedMps ? (s * velocity.vx + c * velocity.vy) / speed : 0.0;
  const double stop_time = max_decel > 0.0 ? speed / max_decel : 0.0;

  Trajectory trajectory;
  trajectory.reserve(steps + 1U);
  for (std::size_t i = 0U; i <= steps; ++i) {
    const double t = std::min(static_cast<double>(i) * kStepS, stop_time);
    const double travelled = speed * t - 0.5 * max_decel * t * t;
    trajectory.push_back(
      PlanarPose{start.x + direction_x * travelled, start.y + direction_y * travelled, start.yaw});
  }
  return trajectory;
}

// 時刻 t の相手と機体の隙間。重なっていれば負
double gapAt(
  const PlanarPose & pose, const Obstacle & obstacle, const double t, const Footprint & footprint,
  const double margin)
{
  const double dx = obstacle.x + obstacle.vx * t - pose.x;
  const double dy = obstacle.y + obstacle.vy * t - pose.y;
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return signedDistanceToFootprint(c * dx + s * dy, -s * dx + c * dy, footprint) -
         obstacleRadius(obstacle.kind) - margin;
}

// 軌道上で最初に衝突する時刻。すでに重なっていても離れていく動きは衝突に数えない
std::optional<double> firstCollision(
  const Trajectory & trajectory, const Obstacle & obstacle, const Footprint & footprint,
  const double margin)
{
  for (std::size_t i = 0U; i < trajectory.size(); ++i) {
    const double t = static_cast<double>(i) * kStepS;
    const double gap = gapAt(trajectory[i], obstacle, t, footprint, margin);
    if (gap > 0.0) {
      continue;
    }
    if (i + 1U == trajectory.size()) {
      return t;
    }
    const double next_gap = gapAt(trajectory[i + 1U], obstacle, t + kStepS, footprint, margin);
    if (next_gap <= gap) {
      return t;
    }
  }
  return std::nullopt;
}

struct Collision
{
  double t{0.0};
  const Obstacle * obstacle{nullptr};
};

std::optional<Collision> earliestCollision(
  const Trajectory & trajectory, const std::vector<Obstacle> & obstacles,
  const Footprint & footprint, const double margin)
{
  std::optional<Collision> earliest;
  for (const auto & obstacle : obstacles) {
    const auto collision = firstCollision(trajectory, obstacle, footprint, margin);
    if (collision.has_value() && (!earliest.has_value() || *collision < earliest->t)) {
      earliest = Collision{*collision, &obstacle};
    }
  }
  return earliest;
}

// 衝突する時刻の相手の向きを、機体座標系の単位ベクトルで返す
std::pair<double, double> threatDirection(
  const Trajectory & trajectory, const Collision & collision)
{
  const auto index = std::min(
    static_cast<std::size_t>(collision.t / kStepS), trajectory.size() - 1U);
  const PlanarPose & pose = trajectory[index];
  const double dx = collision.obstacle->x + collision.obstacle->vx * collision.t - pose.x;
  const double dy = collision.obstacle->y + collision.obstacle->vy * collision.t - pose.y;
  const double distance = std::hypot(dx, dy);
  if (distance < 1e-6) {
    return {0.0, 0.0};
  }
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {(c * dx + s * dy) / distance, (-s * dx + c * dy) / distance};
}

struct Vector2
{
  double x{0.0};
  double y{0.0};
};

double norm(const Vector2 & v)
{
  return std::hypot(v.x, v.y);
}

// 速度を目標へ max_change だけ近づける
Vector2 stepToward(const Vector2 & velocity, const Vector2 & target, const double max_change)
{
  const Vector2 difference{target.x - velocity.x, target.y - velocity.y};
  const double distance = norm(difference);
  if (distance <= max_change) {
    return target;
  }
  const double ratio = max_change / distance;
  return Vector2{velocity.x + difference.x * ratio, velocity.y + difference.y * ratio};
}

// 速度の向きを保って max_change だけ遅くする
Vector2 stepBrake(const Vector2 & velocity, const double max_change)
{
  const double speed = norm(velocity);
  if (speed <= max_change) {
    return Vector2{};
  }
  const double ratio = (speed - max_change) / speed;
  return Vector2{velocity.x * ratio, velocity.y * ratio};
}

struct OwnSample
{
  double t{0.0};
  Vector2 position;
  Vector2 velocity;
};

// 目標の速度へ max_decel で近づき、hold_s 秒保ったあと止まる軌道（odom 座標系の並進）。
// 機体の向きは変えない。止まった後は相手の方向へ動かないので、そこで打ち切る
std::vector<OwnSample> holdThenBrake(
  const PlanarPose & pose, const Vector2 & velocity, const Vector2 & target, const double hold_s,
  const double horizon_s, const double max_decel)
{
  const double max_change = max_decel * kStepS;
  const std::size_t steps = stepCount(horizon_s);
  std::vector<OwnSample> samples;
  samples.reserve(steps + 1U);
  OwnSample sample{0.0, Vector2{pose.x, pose.y}, velocity};
  samples.push_back(sample);
  for (std::size_t i = 0U; i < steps; ++i) {
    const Vector2 next = sample.t + 0.5 * kStepS < hold_s ?
      stepToward(sample.velocity, target, max_change) : stepBrake(sample.velocity, max_change);
    sample.position.x += 0.5 * (sample.velocity.x + next.x) * kStepS;
    sample.position.y += 0.5 * (sample.velocity.y + next.y) * kStepS;
    sample.velocity = next;
    sample.t += kStepS;
    samples.push_back(sample);
    if (sample.t >= hold_s && norm(next) < kMinSpeedMps) {
      break;
    }
  }
  return samples;
}

struct Clearance
{
  // 相手の方向へ動いたまま到達範囲に入る最初の時刻
  std::optional<double> violation;
  // 相手の方向へ動いている間の、到達範囲との隙間の最小値。近づく間がなければ無限大
  double min_gap{std::numeric_limits<double>::infinity()};
};

Clearance reachableClearance(
  const std::vector<OwnSample> & samples, const std::vector<Obstacle> & obstacles,
  const double yaw, const Footprint & footprint, const SafetyParams & params)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  Clearance clearance;
  // 最初の点は今の状態で、どの候補でも同じなので数えない
  for (std::size_t i = 1U; i < samples.size(); ++i) {
    const OwnSample & sample = samples[i];
    const double speed = norm(sample.velocity);
    if (speed < kMinSpeedMps) {
      continue;
    }
    for (const auto & obstacle : obstacles) {
      const double elapsed = sample.t + obstacle.age_s;
      const double dx = obstacle.x + obstacle.vx * elapsed - sample.position.x;
      const double dy = obstacle.y + obstacle.vy * elapsed - sample.position.y;
      const double distance = std::hypot(dx, dy);
      const double approach = distance > 1e-6 ?
        (sample.velocity.x * dx + sample.velocity.y * dy) / distance : speed;
      if (approach <= kApproachSpeedMps) {
        continue;
      }
      const double gap = signedDistanceToFootprint(c * dx + s * dy, -s * dx + c * dy, footprint) -
        reachableRadius(obstacle, sample.t, params);
      clearance.min_gap = std::min(clearance.min_gap, gap);
      if (gap <= 0.0 && !clearance.violation.has_value()) {
        clearance.violation = sample.t;
      }
    }
  }
  return clearance;
}

// 向きを保って動く機体の、静止物までの最小距離を調べる。4 隅の位置（地図の座標系の中心からの位置）は
// 向きだけで決まるので先に求める
struct StaticProbe
{
  const DistanceField & field;
  const PlanarPose & map_from_odom;
  double map_c;
  double map_s;
  std::array<Vector2, 4> corners;
};

StaticProbe makeStaticProbe(
  const DistanceField & field, const PlanarPose & map_from_odom, const double yaw,
  const Footprint & footprint)
{
  const double c = std::cos(yaw + map_from_odom.yaw);
  const double s = std::sin(yaw + map_from_odom.yaw);
  std::array<Vector2, 4> corners;
  std::size_t index = 0U;
  for (const double along : {-footprint.half_length_m, footprint.half_length_m}) {
    for (const double side : {-footprint.half_width_m, footprint.half_width_m}) {
      corners[index++] = Vector2{c * along - s * side, s * along + c * side};
    }
  }
  return StaticProbe{
    field, map_from_odom, std::cos(map_from_odom.yaw), std::sin(map_from_odom.yaw), corners};
}

// 機体の中心が odom の (x, y) にあるときの、4 隅と中心の静止物までの最小距離
double probeClearance(const StaticProbe & probe, const double x, const double y)
{
  const double map_x = probe.map_from_odom.x + probe.map_c * x - probe.map_s * y;
  const double map_y = probe.map_from_odom.y + probe.map_s * x + probe.map_c * y;
  double closest = static_cast<double>(probe.field.at(map_x, map_y));
  for (const Vector2 & corner : probe.corners) {
    closest = std::min(
      closest, static_cast<double>(probe.field.at(map_x + corner.x, map_y + corner.y)));
  }
  return closest;
}

// 軌道の静止物までの最小距離。点の間は格子の半分ごとに分けて調べる。最初の点は今の状態で、どの候補でも
// 同じなので数えない（点が 1 つならその点を使う）
double trajectoryStaticClearance(
  const std::vector<OwnSample> & samples, const DistanceField & field,
  const PlanarPose & map_from_odom, const double yaw, const Footprint & footprint)
{
  const StaticProbe probe = makeStaticProbe(field, map_from_odom, yaw, footprint);
  if (samples.size() < 2U) {
    return samples.empty() ? std::numeric_limits<double>::infinity() :
           probeClearance(probe, samples.front().position.x, samples.front().position.y);
  }
  const double step = kStaticProbeStepRatio * field.info().resolution;
  double closest = std::numeric_limits<double>::infinity();
  for (std::size_t i = 1U; i < samples.size(); ++i) {
    const Vector2 & from = samples[i - 1U].position;
    const Vector2 & to = samples[i].position;
    const double length = norm(Vector2{to.x - from.x, to.y - from.y});
    const int parts = std::max(1, static_cast<int>(std::ceil(length / step)));
    for (int k = 1; k <= parts; ++k) {
      const double ratio = static_cast<double>(k) / static_cast<double>(parts);
      closest = std::min(
        closest,
        probeClearance(probe, from.x + (to.x - from.x) * ratio, from.y + (to.y - from.y) * ratio));
    }
  }
  return closest;
}

// 候補の速度を確かめた結果
struct CandidateCheck
{
  bool feasible{false};
  // 範囲から離れる距離。止まる軌道と保つ軌道の、良いほうの隙間の最小値
  double score{-std::numeric_limits<double>::infinity()};
  // 止まる軌道での静止物までの最小距離。距離の場がなければ無限大
  double static_clearance{std::numeric_limits<double>::infinity()};
};

// 候補の速度 target を、少し保ってから止まる軌道と、保ち続ける軌道で確かめる。
// どちらかで相手の方向へ動いたまま範囲に入らなければ使える。
// field があれば、止まる軌道の静止物までの最小距離も求める
CandidateCheck checkCandidate(
  const std::vector<Obstacle> & obstacles, const PlanarPose & pose, const Vector2 & velocity,
  const Vector2 & target, const Footprint & footprint, const SafetyParams & params,
  const DistanceField * field = nullptr, const PlanarPose & map_from_odom = PlanarPose{})
{
  const ReachableParams & reachable = params.reachable;
  const double fastest = std::max(norm(velocity), norm(target));
  const double stop_horizon = reachable.commit_time_s +
    (params.max_decel_mps2 > 0.0 ? fastest / params.max_decel_mps2 : 0.0) + kStepS;
  const std::vector<OwnSample> braking_samples = holdThenBrake(
    pose, velocity, target, reachable.commit_time_s, stop_horizon, params.max_decel_mps2);
  const Clearance braking = reachableClearance(
    braking_samples, obstacles, pose.yaw, footprint, params);
  const Clearance holding = reachableClearance(
    holdThenBrake(
      pose, velocity, target, reachable.escape_horizon_s, reachable.escape_horizon_s,
      params.max_decel_mps2),
    obstacles, pose.yaw, footprint, params);
  return CandidateCheck{
    !braking.violation.has_value() || !holding.violation.has_value(),
    std::max(braking.min_gap, holding.min_gap),
    field != nullptr ?
    trajectoryStaticClearance(braking_samples, *field, map_from_odom, pose.yaw, footprint) :
    std::numeric_limits<double>::infinity()};
}

Vector2 clampSpeed(const Vector2 & velocity, const double limit)
{
  const double speed = norm(velocity);
  if (speed <= limit || speed < kMinSpeedMps) {
    return velocity;
  }
  const double ratio = limit / speed;
  return Vector2{velocity.x * ratio, velocity.y * ratio};
}

// 候補の速度（odom 座標系）。指令を指令の向きに減速したもの、止まる、今の速度、今の速度から
// 各向きへ変えたもの。速さは指令と今の速さの大きいほうまでにし、止まっているときに動き出さない。
// 機体の前（向き yaw）から見えない側へ向かう候補は、blind_side の上限までに縮める
std::vector<Vector2> candidateVelocities(
  const Vector2 & reference, const Vector2 & velocity, const double yaw,
  const SafetyParams & params)
{
  const ReachableParams & reachable = params.reachable;
  const double speed_limit = std::max(norm(reference), norm(velocity));
  const double max_change = params.max_decel_mps2 * reachable.escape_change_s;
  const int directions = std::max(reachable.directions, 1);
  std::vector<Vector2> candidates;
  candidates.reserve(
    kCommandFractions.size() + 2U + static_cast<std::size_t>(directions) * kChangeFractions.size());
  for (const double fraction : kCommandFractions) {
    candidates.push_back(Vector2{reference.x * fraction, reference.y * fraction});
  }
  candidates.push_back(Vector2{});
  candidates.push_back(clampSpeed(velocity, speed_limit));
  for (int k = 0; k < directions; ++k) {
    const double angle = 2.0 * M_PI * static_cast<double>(k) / static_cast<double>(directions);
    for (const double fraction : kChangeFractions) {
      candidates.push_back(
        clampSpeed(
          Vector2{
          velocity.x + max_change * fraction * std::cos(angle),
          velocity.y + max_change * fraction * std::sin(angle)},
          speed_limit));
    }
  }
  const BlindSideLimit blind(params.blind_side);
  const double forward_x = std::cos(yaw);
  const double forward_y = std::sin(yaw);
  std::transform(
    candidates.begin(), candidates.end(), candidates.begin(), [&](const Vector2 & candidate) {
      const double scale = blind.scale(candidate.x, candidate.y, forward_x, forward_y);
      return Vector2{candidate.x * scale, candidate.y * scale};
    });
  return candidates;
}

// 正面の相手と判定する、進む向きの差の cos（150°）。局所計画（mppi）と同じ境
constexpr double kHeadOnCos = -0.866;
// 正面の相手だけが原因で替えるときの、今の向きのまま減速する候補（今の速度に掛ける割合）
constexpr std::array<double, 4> kBrakeFractions{0.75, 0.5, 0.25, 0.0};
// 緩和の候補の速さ（mitigation.max_speed_mps に掛ける割合）
constexpr std::array<double, 2> kShiftSpeedFractions{1.0, 0.5};
// 緩和で、horizon_s より先まで当たらないかを見る時間 [s]（動いた後に遅れて当たる候補を除く）
constexpr double kMitigationTailS = 0.5;
// 緩和の指令として、選んだ軌道のこの時刻の速度を出す（周期と指令の遅れの分だけ先）[s]
constexpr double kMitigationCommandS = 0.1;

Vector2 toOdom(const BodyVelocity & velocity, const double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Vector2{c * velocity.vx - s * velocity.vy, s * velocity.vx + c * velocity.vy};
}

BodyVelocity toBody(const Vector2 & velocity, const double yaw, const double wz)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return BodyVelocity{c * velocity.x + s * velocity.y, -s * velocity.x + c * velocity.y, wz};
}

// 速度の目標 target へ max_decel で近づき、基準の点（reference から drift で進む点）からのずれが
// max_shift を越えないうちに止まる軌道（止まるまでに進む距離を見込んで減速を始める）
std::vector<OwnSample> shiftTrajectory(
  const PlanarPose & pose, const Vector2 & velocity, const Vector2 & target,
  const Vector2 & reference, const Vector2 & drift, const double max_shift, const double horizon_s,
  const double max_decel)
{
  const double max_change = max_decel * kStepS;
  const std::size_t steps = stepCount(horizon_s);
  std::vector<OwnSample> samples;
  samples.reserve(steps + 1U);
  OwnSample sample{0.0, Vector2{pose.x, pose.y}, velocity};
  samples.push_back(sample);
  for (std::size_t i = 0U; i < steps; ++i) {
    const Vector2 relative{
      sample.velocity.x - drift.x, sample.velocity.y - drift.y};
    const double relative_speed = norm(relative);
    const double stopping =
      max_decel > 0.0 ? relative_speed * relative_speed / (2.0 * max_decel) : 0.0;
    const double shifted = norm(
      Vector2{
        sample.position.x - (reference.x + drift.x * sample.t),
        sample.position.y - (reference.y + drift.y * sample.t)});
    // ずれの分だけ減速する（drift の速度へ戻る）
    const Vector2 next = shifted + stopping < max_shift ?
      stepToward(sample.velocity, target, max_change) :
      stepToward(sample.velocity, drift, max_change);
    sample.position.x += 0.5 * (sample.velocity.x + next.x) * kStepS;
    sample.position.y += 0.5 * (sample.velocity.y + next.y) * kStepS;
    sample.velocity = next;
    sample.t += kStepS;
    samples.push_back(sample);
  }
  return samples;
}

// 自機が原因の接触とみなす、自機の速さ（run_scenario.py の classify_contact と crowd_model と同じ）
constexpr double kOwnFaultSpeedMps = 0.5;
// 緩和の候補を選ぶときの余裕: 速さの下限に掛ける倍率と、相手から離れているとみなす向きの cos（約 107°）
constexpr double kOwnFaultSpeedMargin = 0.8;
constexpr double kAwayCos = -0.3;

struct Contact
{
  double time_s;
  // 近づく速さ（中心を結ぶ向きの相対速度）[m/s]
  double impact_speed_mps;
  // 当たるとき、自機が kOwnFaultSpeedMps 以上で相手の方向へ動いているか（自機が原因の接触になる）
  bool own_fault;
};

// 軌道 samples（機体の向き yaw は変えない）と、等速で進む相手が、最初に重なる時刻と近づく速さ。
// 機体の余裕と相手の余裕は足さない（実際に当たるか）
std::optional<Contact> firstContact(
  const std::vector<OwnSample> & samples, const Obstacle & obstacle, const double yaw,
  const Footprint & footprint)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  for (const OwnSample & sample : samples) {
    const double elapsed = sample.t + obstacle.age_s;
    const double dx = obstacle.x + obstacle.vx * elapsed - sample.position.x;
    const double dy = obstacle.y + obstacle.vy * elapsed - sample.position.y;
    const double gap = signedDistanceToFootprint(c * dx + s * dy, -s * dx + c * dy, footprint) -
      obstacleRadius(obstacle.kind);
    if (gap > 0.0) {
      continue;
    }
    const double distance = std::hypot(dx, dy);
    const double closing = distance > 1e-6 ?
      ((sample.velocity.x - obstacle.vx) * dx + (sample.velocity.y - obstacle.vy) * dy) / distance :
      norm(Vector2{sample.velocity.x - obstacle.vx, sample.velocity.y - obstacle.vy});
    // 相手の予測や追跡のぶれで当たり方は変わるので、余裕を持って判定する: 速さが下限の
    // kOwnFaultSpeedMargin 倍以上で、相手から離れる向き（向きの差 kAwayCos より大きい）でなければ
    const double own_speed = norm(sample.velocity);
    const bool own_fault = own_speed >= kOwnFaultSpeedMps * kOwnFaultSpeedMargin &&
      sample.velocity.x * dx + sample.velocity.y * dy > kAwayCos * own_speed * distance;
    return Contact{sample.t, std::max(closing, 0.0), own_fault};
  }
  return std::nullopt;
}
}  // namespace

double obstacleRadius(const ObstacleKind kind)
{
  switch (kind) {
    case ObstacleKind::kRobot:
      return 0.45;
    case ObstacleKind::kHuman:
      return 0.3;
    case ObstacleKind::kUnknown:
      return 0.5;
  }
  return 0.5;
}

SafetyResult evaluateSafety(
  const std::vector<Obstacle> & obstacles, const PlanarPose & pose,
  const BodyVelocity & own_velocity, const BodyVelocity & commanded, const Footprint & footprint,
  const SafetyParams & params)
{
  const Trajectory trajectory = commandedTrajectory(pose, commanded, params.ttc_slow_s);
  const auto collision = earliestCollision(
    trajectory, obstacles, footprint, params.footprint_margin_m);
  SafetyResult result;
  if (!collision.has_value()) {
    return result;
  }
  result.ttc_s = collision->t;
  // 止まるほうが早く当たるなら、指令は避ける動きなので止めない。ただし指令でも ttc_stop_s 以内に
  // 当たるなら、避けきれないので通さない
  const auto braking = earliestCollision(
    brakingTrajectory(pose, own_velocity, params.max_decel_mps2, params.ttc_slow_s), obstacles,
    footprint, params.footprint_margin_m);
  const bool evasive = braking.has_value() && braking->t < collision->t &&
    collision->t >= params.ttc_stop_s;
  if (evasive || collision->t >= params.ttc_slow_s) {
    return result;
  }
  const auto [threat_x, threat_y] = threatDirection(trajectory, *collision);
  result.threat_x = threat_x;
  result.threat_y = threat_y;
  if (collision->t < params.ttc_stop_s) {
    result.decision = Decision::kStop;
    result.speed_scale = 0.0;
    return result;
  }
  result.decision = Decision::kSlow;
  result.speed_scale = collision->t / params.ttc_slow_s;
  return result;
}

BodyVelocity applySpeedScale(const BodyVelocity & commanded, const SafetyResult & result)
{
  const double direction_norm = std::hypot(result.threat_x, result.threat_y);
  if (direction_norm < 0.5) {
    return BodyVelocity{
      commanded.vx * result.speed_scale, commanded.vy * result.speed_scale,
      commanded.wz * result.speed_scale};
  }
  const double toward = commanded.vx * result.threat_x + commanded.vy * result.threat_y;
  // 相手から離れる向きの速度は抑えない
  const double removed = toward > 0.0 ? toward * (1.0 - result.speed_scale) : 0.0;
  return BodyVelocity{
    commanded.vx - removed * result.threat_x, commanded.vy - removed * result.threat_y,
    commanded.wz};
}

double reachableRadius(const Obstacle & obstacle, const double t, const SafetyParams & params)
{
  const double elapsed = std::max(t + obstacle.age_s, 0.0);
  const bool can_accelerate = obstacle.kind != ObstacleKind::kUnknown ||
    std::hypot(obstacle.vx, obstacle.vy) >= kMovingObstacleSpeedMps;
  const double acceleration = can_accelerate ? params.reachable.obstacle_accel_mps2 : 0.0;
  return obstacleRadius(obstacle.kind) + params.footprint_margin_m + obstacle.position_sigma +
         params.reachable.velocity_sigma_gain * obstacle.velocity_sigma * elapsed +
         0.5 * acceleration * elapsed * elapsed;
}

std::optional<double> firstReachableViolation(
  const std::vector<Obstacle> & obstacles, const PlanarPose & pose, const double velocity_x,
  const double velocity_y, const double target_x, const double target_y, const double hold_s,
  const double horizon_s, const Footprint & footprint, const SafetyParams & params)
{
  return reachableClearance(
    holdThenBrake(
      pose, Vector2{velocity_x, velocity_y}, Vector2{target_x, target_y}, hold_s, horizon_s,
      params.max_decel_mps2),
    obstacles, pose.yaw, footprint, params).violation;
}

double staticClearance(
  const DistanceField & field, const PlanarPose & map_from_odom, const PlanarPose & pose,
  const Footprint & footprint)
{
  return probeClearance(makeStaticProbe(field, map_from_odom, pose.yaw, footprint), pose.x, pose.y);
}

SupervisedCommand superviseCommand(
  const std::vector<Obstacle> & obstacles, const PlanarPose & pose,
  const BodyVelocity & own_velocity, const BodyVelocity & commanded, const Footprint & footprint,
  const SafetyParams & params, const DistanceField * field, const PlanarPose & map_from_odom)
{
  SupervisedCommand supervised;
  supervised.nominal = evaluateSafety(
    obstacles, pose, own_velocity, commanded, footprint, params);
  supervised.decision = supervised.nominal.decision;
  supervised.output = applySpeedScale(commanded, supervised.nominal);
  if (!params.reachable.enabled || obstacles.empty()) {
    return supervised;
  }
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  const Vector2 velocity{
    c * own_velocity.vx - s * own_velocity.vy, s * own_velocity.vx + c * own_velocity.vy};
  const Vector2 reference{
    c * supervised.output.vx - s * supervised.output.vy,
    s * supervised.output.vx + c * supervised.output.vy};
  if (checkCandidate(obstacles, pose, velocity, reference, footprint, params).feasible) {
    return supervised;
  }
  // 範囲に入る原因が正面から来るロボットだけなら、今の向きのまま減速する候補に限り、横へ逃がさない
  // （道を開けない）。範囲に入らない候補のうち指令に最も近いもの、なければ止まる
  if (params.oncoming.brake_only) {
    std::vector<Obstacle> others;
    others.reserve(obstacles.size());
    bool head_on = false;
    for (const auto & obstacle : obstacles) {
      if (isHeadOnObstacle(obstacle, pose, velocity.x, velocity.y, footprint, params)) {
        head_on = true;
      } else {
        others.push_back(obstacle);
      }
    }
    if (head_on &&
      (others.empty() ||
      checkCandidate(others, pose, velocity, reference, footprint, params).feasible))
    {
      std::optional<Vector2> chosen;
      double chosen_distance = std::numeric_limits<double>::infinity();
      for (const double fraction : kBrakeFractions) {
        const Vector2 candidate{velocity.x * fraction, velocity.y * fraction};
        const double distance = norm(Vector2{candidate.x - reference.x, candidate.y - reference.y});
        if (distance < chosen_distance &&
          checkCandidate(obstacles, pose, velocity, candidate, footprint, params).feasible)
        {
          chosen = candidate;
          chosen_distance = distance;
        }
      }
      supervised.replaced = true;
      supervised.feasible = chosen.has_value();
      supervised.decision = Decision::kStop;
      supervised.output = toBody(chosen.value_or(Vector2{}), pose.yaw, supervised.output.wz);
      return supervised;
    }
  }
  // 指令の後に止まれない。範囲に入らない候補のうち指令に最も近いもの、なければ離れる距離が最大のもの。
  // 距離の場があれば、静止物について使える候補（止まる軌道で静止物に寄らない）からこの順に選び、
  // 1 つもなければ静止物までの最小距離が最大のものにする。指令は計画が静止物を見て決めているので、
  // 替えるときだけ確かめる
  const DistanceField * static_field = params.static_check.enabled ? field : nullptr;
  // 静止物について使える最小距離。今すでに余裕より近ければ（狭い所）、今より近づかない候補を残す
  const double required_clearance = static_field != nullptr ?
    std::min(
    params.static_check.margin_m,
    staticClearance(*static_field, map_from_odom, pose, footprint) -
    params.static_check.tolerance_m) :
    -std::numeric_limits<double>::infinity();
  struct Choice
  {
    Vector2 velocity;
    CandidateCheck check;
  };
  std::optional<Choice> closest;
  double closest_distance = std::numeric_limits<double>::infinity();
  std::optional<Choice> farthest;
  std::optional<Choice> clearest;
  for (const Vector2 & candidate : candidateVelocities(reference, velocity, pose.yaw, params)) {
    const CandidateCheck check = checkCandidate(
      obstacles, pose, velocity, candidate, footprint, params, static_field, map_from_odom);
    // 今の距離が許容以下だと条件が 0 以下になり、静止物のセル（距離 0）に入る候補も通ってしまうので、
    // 静止物に入らないことも条件にする
    const bool static_ok = static_field == nullptr ||
      (check.static_clearance + kStaticCompareEpsilonM >= required_clearance &&
      check.static_clearance > 0.0);
    const double distance = norm(Vector2{candidate.x - reference.x, candidate.y - reference.y});
    if (check.feasible && static_ok && distance < closest_distance) {
      closest = Choice{candidate, check};
      closest_distance = distance;
    }
    if (static_ok && (!farthest.has_value() || check.score > farthest->check.score)) {
      farthest = Choice{candidate, check};
    }
    if (!clearest.has_value() || check.static_clearance > clearest->check.static_clearance) {
      clearest = Choice{candidate, check};
    }
  }
  // 候補は必ず 1 つ以上あるので clearest は空にならない
  const Choice & choice = closest.has_value() ? *closest :
    (farthest.has_value() ? *farthest : clearest.value());
  const Vector2 chosen = choice.velocity;
  supervised.replaced = true;
  supervised.feasible = closest.has_value();
  supervised.static_checked = static_field != nullptr;
  supervised.static_ok = closest.has_value() || farthest.has_value();
  supervised.static_clearance_m = static_field != nullptr ? choice.check.static_clearance : -1.0;
  supervised.decision = Decision::kStop;
  supervised.output = BodyVelocity{
    c * chosen.x + s * chosen.y, -s * chosen.x + c * chosen.y, supervised.output.wz};
  return supervised;
}

BodyVelocity limitDeceleration(
  const BodyVelocity & previous, const BodyVelocity & target, const double max_decel,
  const double dt)
{
  const double previous_speed = std::hypot(previous.vx, previous.vy);
  if (previous_speed < kMinSpeedMps) {
    return target;
  }
  // 今の進行方向の速度が落ちないなら減速ではない
  const double along = (target.vx * previous.vx + target.vy * previous.vy) / previous_speed;
  if (along >= previous_speed) {
    return target;
  }
  const double dvx = target.vx - previous.vx;
  const double dvy = target.vy - previous.vy;
  const double change = std::hypot(dvx, dvy);
  const double max_change = max_decel * dt;
  if (change <= max_change) {
    return target;
  }
  const double ratio = max_change / change;
  return BodyVelocity{previous.vx + dvx * ratio, previous.vy + dvy * ratio, target.wz};
}

BodyVelocity inTurnedFrame(const BodyVelocity & velocity, const double turned)
{
  const double c = std::cos(turned);
  const double s = std::sin(turned);
  return BodyVelocity{c * velocity.vx + s * velocity.vy, -s * velocity.vx + c * velocity.vy, velocity.wz};
}

bool isHeadOnObstacle(
  const Obstacle & obstacle, const PlanarPose & pose, const double velocity_x,
  const double velocity_y, const Footprint & footprint, const SafetyParams & params)
{
  const double own_speed = std::hypot(velocity_x, velocity_y);
  const double speed = std::hypot(obstacle.vx, obstacle.vy);
  if (!obstacle.moving_reliable || obstacle.kind != ObstacleKind::kRobot ||
    own_speed < kMovingObstacleSpeedMps || speed < kMovingObstacleSpeedMps)
  {
    return false;
  }
  if ((velocity_x * obstacle.vx + velocity_y * obstacle.vy) / (own_speed * speed) > kHeadOnCos) {
    return false;
  }
  const double direction_x = velocity_x / own_speed;
  const double direction_y = velocity_y / own_speed;
  const double dx = obstacle.x - pose.x;
  const double dy = obstacle.y - pose.y;
  const double along = dx * direction_x + dy * direction_y;
  const double lateral = std::abs(direction_x * dy - direction_y * dx);
  return along > 0.0 &&
         lateral <= footprint.half_width_m + params.footprint_margin_m + obstacleRadius(obstacle.kind);
}

std::optional<ImpactThreat> findImpactThreat(
  const std::vector<Obstacle> & obstacles, const PlanarPose & pose, const double velocity_x,
  const double velocity_y, const Footprint & footprint, const SafetyParams & params)
{
  const MitigationParams & mitigation = params.mitigation;
  const Vector2 velocity{velocity_x, velocity_y};
  // 今の速度を保つ軌道
  const std::vector<OwnSample> samples = holdThenBrake(
    pose, velocity, velocity, mitigation.horizon_s, mitigation.horizon_s, params.max_decel_mps2);
  std::optional<ImpactThreat> threat;
  for (std::size_t i = 0U; i < obstacles.size(); ++i) {
    const Obstacle & obstacle = obstacles[i];
    if (!obstacle.moving_reliable ||
      std::hypot(obstacle.vx, obstacle.vy) < kMovingObstacleSpeedMps)
    {
      continue;
    }
    const auto contact = firstContact(samples, obstacle, pose.yaw, footprint);
    if (contact.has_value() && contact->impact_speed_mps >= mitigation.min_impact_speed_mps &&
      (!threat.has_value() || contact->time_s < threat->time_s))
    {
      threat = ImpactThreat{i, contact->time_s, contact->impact_speed_mps};
    }
  }
  return threat;
}

std::optional<BodyVelocity> ImpactMitigator::update(
  const double time_s, const std::vector<Obstacle> & obstacles, const PlanarPose & pose,
  const BodyVelocity & own_velocity, const Footprint & footprint, const SafetyParams & params,
  const DistanceField * field, const PlanarPose & map_from_odom)
{
  active_ = false;
  const MitigationParams & mitigation = params.mitigation;
  if (!mitigation.enabled) {
    origin_.reset();
    return std::nullopt;
  }
  const Vector2 velocity = toOdom(own_velocity, pose.yaw);
  if (origin_.has_value() && time_s < origin_->time_s) {
    origin_.reset();
  }
  // 緩和は、止まっているか遅い（kOwnFaultSpeedMps 未満の）ときにだけ始める。走っている間は、計画の
  // 停止線と安全停止の減速で応じる（走りながらの緩和では、緩和の後に相手の方向へ動いて当たり、自機が
  // 原因の接触になった）
  if (!origin_.has_value() && norm(velocity) >= kOwnFaultSpeedMps) {
    return std::nullopt;
  }
  // 緩和を始めた後は、緩和しなかったとき（始めた位置に止まっていたとき）に当たるかで続けるかを
  // 決める。今の位置で見ると、少し動いただけで当たらなくなり、緩和と戻りを繰り返すため
  const Vector2 drift{};
  const Vector2 reference = origin_.has_value() ?
    Vector2{origin_->x, origin_->y} : Vector2{pose.x, pose.y};
  const auto threat = findImpactThreat(
    obstacles, PlanarPose{reference.x, reference.y, pose.yaw},
    origin_.has_value() ? 0.0 : velocity.x, origin_.has_value() ? 0.0 : velocity.y, footprint,
    params);
  if (!threat.has_value()) {
    origin_.reset();
    return std::nullopt;
  }
  if (!origin_.has_value()) {
    origin_ = Origin{pose.x, pose.y, time_s};
  }
  const Obstacle & target_obstacle = obstacles[threat->index];
  std::vector<Obstacle> others;
  others.reserve(obstacles.size());
  for (std::size_t i = 0U; i < obstacles.size(); ++i) {
    if (i != threat->index) {
      others.push_back(obstacles[i]);
    }
  }
  const DistanceField * static_field = params.static_check.enabled ? field : nullptr;
  const double required_clearance = static_field != nullptr ?
    std::min(
    params.static_check.margin_m,
    staticClearance(*static_field, map_from_odom, pose, footprint) -
    params.static_check.tolerance_m) :
    -std::numeric_limits<double>::infinity();
  const double horizon = mitigation.horizon_s + kMitigationTailS;
  const int directions = std::max(mitigation.directions, 1);

  std::optional<Vector2> best_command;
  double best_impact = std::numeric_limits<double>::infinity();
  double best_change = std::numeric_limits<double>::infinity();
  // 相手の今の位置への向き。緩和は相手から離れる向き（向きの差 kAwayCos より大きい）にだけ動く。
  // 横へ動くと、当たり方の予測が少しずれただけで、自機が相手の方向へ動いているうちに当たる
  const double threat_dx = target_obstacle.x + target_obstacle.vx * target_obstacle.age_s - pose.x;
  const double threat_dy = target_obstacle.y + target_obstacle.vy * target_obstacle.age_s - pose.y;
  const double threat_distance = std::max(std::hypot(threat_dx, threat_dy), 1e-6);
  for (int k = 0; k < directions; ++k) {
    const double angle = 2.0 * M_PI * static_cast<double>(k) / static_cast<double>(directions);
    if ((std::cos(angle) * threat_dx + std::sin(angle) * threat_dy) / threat_distance > kAwayCos) {
      continue;
    }
    for (const double fraction : kShiftSpeedFractions) {
      const double speed = mitigation.max_speed_mps * fraction;
      const Vector2 target{
        drift.x + speed * std::cos(angle), drift.y + speed * std::sin(angle)};
      const std::vector<OwnSample> samples = shiftTrajectory(
        pose, velocity, target, reference, drift, mitigation.max_shift_m, horizon,
        params.max_decel_mps2);
      const auto contact = firstContact(samples, target_obstacle, pose.yaw, footprint);
      const double impact = contact.has_value() ? contact->impact_speed_mps : 0.0;
      const double change = norm(Vector2{target.x - velocity.x, target.y - velocity.y});
      // 当たるときに自機が相手の方向へ動いていれば、自機が原因の接触になるので使わない
      if ((contact.has_value() && contact->own_fault) || impact > best_impact + 1e-9 ||
        (std::abs(impact - best_impact) <= 1e-9 && change >= best_change))
      {
        continue;
      }
      // ほかの相手の到達範囲へ向かわない。静止物の余裕を取れる
      if (!others.empty() &&
        reachableClearance(samples, others, pose.yaw, footprint, params).violation.has_value())
      {
        continue;
      }
      if (static_field != nullptr) {
        const double clearance =
          trajectoryStaticClearance(samples, *static_field, map_from_odom, pose.yaw, footprint);
        if (clearance + kStaticCompareEpsilonM < required_clearance || clearance <= 0.0) {
          continue;
        }
      }
      const std::size_t index = std::min(
        static_cast<std::size_t>(std::lround(kMitigationCommandS / kStepS)), samples.size() - 1U);
      best_command = samples[index].velocity;
      best_impact = impact;
      best_change = change;
    }
  }
  if (!best_command.has_value() ||
    threat->impact_speed_mps - best_impact < mitigation.min_reduction_mps)
  {
    return std::nullopt;
  }
  active_ = true;
  baseline_impact_ = threat->impact_speed_mps;
  chosen_impact_ = best_impact;
  return toBody(*best_command, pose.yaw, 0.0);
}
}  // namespace core_planning::safety
