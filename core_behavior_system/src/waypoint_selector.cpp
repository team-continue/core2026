#include "core_behavior_system/waypoint_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace core_behavior_system {

WaypointSelector::WaypointSelector(const WaypointSelectorConfig &config)
    : config_(config), rng_(std::random_device{}()),
      jitter_dist_(0.0, std::max(0.0, config_.random_jitter)) {}

std::optional<int> WaypointSelector::SelectBestWaypoint(
    int current_target_id, double now,
    const std::vector<WaypointCandidate> &candidates) {
  std::optional<int> best_id;
  double best_score = std::numeric_limits<double>::infinity();

  for (const auto &c : candidates) {
    if (c.id == current_target_id) {
      continue;
    }
    if (!c.path_complete) {
      continue;
    }

    if (config_.forward_only && c.first_dir_dot < 0.0) {
      continue;
    }

    const double path_length = std::max(0.0, c.path_length);
    const double angle_cost = 1.0 - c.first_dir_dot;
    const double time_penalty = ComputeTimePenalty(c.id, now);

    double score =
        path_length * config_.w_distance +
        angle_cost * config_.w_angle +
        time_penalty * config_.w_time;

    if (config_.random_jitter > 0.0) {
      score += jitter_dist_(rng_);
    }

    if (score < best_score) {
      best_score = score;
      best_id = c.id;
    }
  }

  return best_id;
}

void WaypointSelector::MarkVisited(int waypoint_id, double now) {
  if (waypoint_id < 0) {
    return;
  }
  last_visit_time_[waypoint_id] = now;
}

double WaypointSelector::ComputeTimePenalty(int waypoint_id, double now) const {
  auto it = last_visit_time_.find(waypoint_id);
  if (it == last_visit_time_.end()) {
    return 0.0;
  }

  const double time_since = std::max(0.0, now - it->second);
  if (config_.revisit_cooldown <= 0.0) {
    return 0.0;
  }

  if (config_.use_exponential_decay) {
    return std::exp(-time_since / config_.revisit_cooldown);
  }

  const double t = 1.0 - (time_since / config_.revisit_cooldown);
  return std::clamp(t, 0.0, 1.0);
}

}  // namespace core_behavior_system
