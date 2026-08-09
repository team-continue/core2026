#pragma once

#include <optional>
#include <random>
#include <unordered_map>
#include <vector>

namespace core_behavior_system {

struct WaypointCandidate {
  int id;
  double path_length;
  double first_dir_dot;
  bool path_complete;
};

struct WaypointSelectorConfig {
  double w_distance{1.0};
  double w_angle{3.0};
  double w_time{5.0};
  double revisit_cooldown{15.0};
  bool use_exponential_decay{true};
  bool forward_only{false};
  double random_jitter{0.01};
};

class WaypointSelector {
public:
  explicit WaypointSelector(const WaypointSelectorConfig &config = {});

  std::optional<int> SelectBestWaypoint(
      int current_target_id, double now,
      const std::vector<WaypointCandidate> &candidates);

  void MarkVisited(int waypoint_id, double now);

  const WaypointSelectorConfig &config() const { return config_; }
  void set_config(const WaypointSelectorConfig &config) { config_ = config; }

private:
  double ComputeTimePenalty(int waypoint_id, double now) const;

  WaypointSelectorConfig config_;
  std::unordered_map<int, double> last_visit_time_;
  std::mt19937 rng_;
  std::uniform_real_distribution<double> jitter_dist_;
};

}  // namespace core_behavior_system
