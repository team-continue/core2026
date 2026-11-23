#include "core_perception/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Dense>

#include "core_perception/hungarian.hpp"

namespace core_perception
{
namespace
{
// 追跡と観測の対応に使えない組み合わせのコスト
constexpr double kNoMatchCost = 1e6;

void applyMeasurement(
  Track & track, const Detection & detection, const TrackingParams & params)
{
  Eigen::Matrix<double, 2, 4> h = Eigen::Matrix<double, 2, 4>::Zero();
  h(0, 0) = 1.0;
  h(1, 1) = 1.0;
  const Eigen::Matrix2d r =
    Eigen::Matrix2d::Identity() * (params.measurement_sigma_m * params.measurement_sigma_m);
  const Eigen::Vector2d residual = detection.center.cast<double>() - track.state.head<2>();
  const Eigen::Matrix2d s = h * track.covariance * h.transpose() + r;
  const Eigen::Matrix<double, 4, 2> gain = track.covariance * h.transpose() * s.inverse();
  track.state += gain * residual;
  track.covariance = (Eigen::Matrix4d::Identity() - gain * h) * track.covariance;

  ++track.hits;
  track.coast_s = 0.0;
  track.height.add(detection.height);
  track.width.add(detection.width);
  track.depth.add(detection.depth);
}

void limitSpeed(Track & track, const double max_speed)
{
  const double speed = track.state.tail<2>().norm();
  if (speed > max_speed) {
    track.state.tail<2>() *= max_speed / speed;
  }
}
}  // namespace

double velocitySigma(const Track & track)
{
  return std::sqrt(0.5 * (track.covariance(2, 2) + track.covariance(3, 3)));
}

double uncertainVelocitySigma(const Track & track, const TrackingParams & params)
{
  return std::max(velocitySigma(track) - params.velocity_settled_sigma_mps, 0.0);
}

ObjectClass classify(const Track & track, const ClassificationParams & params)
{
  const float width = std::max(track.width.value(), track.depth.value());
  const float height = track.height.value();
  if (height >= params.human_min_height_m && width <= params.human_max_width_m) {
    return ObjectClass::kHuman;
  }
  if (height <= params.robot_max_height_m && width >= params.robot_min_width_m &&
    width <= params.robot_max_width_m)
  {
    return ObjectClass::kRobot;
  }
  return ObjectClass::kUnknown;
}

void MultiObjectTracker::predict(const double dt, const TrackingParams & params)
{
  Eigen::Matrix4d f = Eigen::Matrix4d::Identity();
  f(0, 2) = dt;
  f(1, 3) = dt;
  const double q = params.accel_sigma_mps2 * params.accel_sigma_mps2;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;
  Eigen::Matrix4d process = Eigen::Matrix4d::Zero();
  process(0, 0) = process(1, 1) = 0.25 * dt4 * q;
  process(0, 2) = process(2, 0) = process(1, 3) = process(3, 1) = 0.5 * dt3 * q;
  process(2, 2) = process(3, 3) = dt2 * q;
  time_s_ += dt;
  for (auto & track : tracks_) {
    track.state = f * track.state;
    track.covariance = f * track.covariance * f.transpose() + process;
    track.age_s += dt;
    track.coast_s += dt;
  }
}

void MultiObjectTracker::createTrack(const Detection & detection, const TrackingParams & params)
{
  Track track;
  track.id = next_id_++;
  track.state << detection.center.cast<double>(), 0.0, 0.0;
  const double position_variance = params.measurement_sigma_m * params.measurement_sigma_m;
  track.covariance = Eigen::Matrix4d::Zero();
  track.covariance(0, 0) = track.covariance(1, 1) = position_variance;
  track.covariance(2, 2) = track.covariance(3, 3) =
    params.initial_velocity_sigma_mps * params.initial_velocity_sigma_mps;
  track.hits = 1U;
  track.birth_s = time_s_;
  track.height.add(detection.height);
  track.width.add(detection.width);
  track.depth.add(detection.depth);
  tracks_.push_back(track);
}

void MultiObjectTracker::update(
  const std::vector<Detection> & detections, const double dt, const TrackingParams & params)
{
  predict(dt, params);

  const std::size_t num_tracks = tracks_.size();
  const std::size_t num_detections = detections.size();
  std::vector<bool> detection_used(num_detections, false);

  if (num_tracks > 0U && num_detections > 0U) {
    // 行を追跡、列を観測にする。追跡が多いときも解けるよう、列にダミーを足す
    std::vector<std::vector<double>> cost(
      num_tracks, std::vector<double>(num_detections + num_tracks, kNoMatchCost));
    for (std::size_t t = 0U; t < num_tracks; ++t) {
      // 隠れている間は位置が不確かなので、対応を探す範囲を広げる
      const double gate =
        params.association_gate_m + params.coast_sigma_growth_mps * tracks_[t].coast_s;
      for (std::size_t d = 0U; d < num_detections; ++d) {
        const double distance =
          (detections[d].center.cast<double>() - tracks_[t].state.head<2>()).norm();
        if (distance <= gate) {
          cost[t][d] = distance;
        }
      }
    }
    const auto assignment = solveAssignment(cost);
    for (std::size_t t = 0U; t < num_tracks; ++t) {
      const std::size_t d = assignment[t];
      if (d < num_detections && cost[t][d] < kNoMatchCost) {
        applyMeasurement(tracks_[t], detections[d], params);
        limitSpeed(tracks_[t], params.max_speed_mps);
        detection_used[d] = true;
      }
    }
  }

  const auto removed = std::stable_partition(
    tracks_.begin(), tracks_.end(),
    [&params](const Track & track) {
      // 一度しか観測していない追跡は誤検出の可能性が高いので、すぐ消す
      const double limit = track.hits >= params.min_hits ? params.max_coast_s : 0.15;
      return track.coast_s <= limit;
    });
  if (params.handover.enabled) {
    for (auto it = removed; it != tracks_.end(); ++it) {
      if (isConfirmed(*it, params)) {
        lost_.push_back(LostTrack{*it, time_s_, time_s_ - it->coast_s});
      }
    }
  }
  tracks_.erase(removed, tracks_.end());

  for (std::size_t d = 0U; d < num_detections; ++d) {
    if (!detection_used[d]) {
      createTrack(detections[d], params);
    }
  }
  if (params.handover.enabled) {
    handOver(params);
  } else {
    lost_.clear();
  }
}

void MultiObjectTracker::handOver(const TrackingParams & params)
{
  const HandoverParams & handover = params.handover;
  lost_.erase(
    std::remove_if(
      lost_.begin(), lost_.end(),
      [this, &handover](const LostTrack & lost) {
        return time_s_ - lost.deleted_s > handover.max_gap_s;
      }),
    lost_.end());
  const double extra_variance =
    handover.extra_velocity_sigma_mps * handover.extra_velocity_sigma_mps;
  for (auto lost = lost_.begin(); lost != lost_.end(); ) {
    const Eigen::Vector2d velocity = lost->track.state.tail<2>();
    const Eigen::Vector2d predicted =
      lost->track.state.head<2>() + velocity * (time_s_ - lost->deleted_s);
    const Eigen::Matrix2d velocity_covariance =
      lost->track.covariance.bottomRightCorner<2, 2>() +
      Eigen::Matrix2d::Identity() * extra_variance;
    const double inherited_sigma = std::sqrt(0.5 * velocity_covariance.trace());
    // 消えた追跡を最後に観測した後に生まれ、引き継ぐ速度より不確かな追跡のうち、最も近いもの
    Track * child = nullptr;
    double best = handover.max_distance_m;
    for (auto & track : tracks_) {
      const double distance = (track.state.head<2>() - predicted).norm();
      if (track.inherited || track.birth_s <= lost->last_seen_s ||
        velocitySigma(track) <= inherited_sigma || distance > best)
      {
        continue;
      }
      child = &track;
      best = distance;
    }
    if (child == nullptr) {
      ++lost;
      continue;
    }
    child->state.tail<2>() = velocity;
    // 位置は新しい追跡の観測のまま、速度だけを消えた追跡の推定にする
    child->covariance.topRightCorner<2, 2>().setZero();
    child->covariance.bottomLeftCorner<2, 2>().setZero();
    child->covariance.bottomRightCorner<2, 2>() = velocity_covariance;
    child->id = lost->track.id;
    child->hits = std::max(child->hits, params.min_hits);
    child->inherited = true;
    lost = lost_.erase(lost);
  }
}
}  // namespace core_perception
