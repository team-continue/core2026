#pragma once

#include <cstddef>
#include <vector>

namespace core_shooter
{

class ForbiddenRegionPlanner
{
public:
  struct Config
  {
    bool enabled = false;
    std::vector<double> points_yaw;
    std::vector<double> points_pitch;
    double point_radius = 0.0;
    bool connect_as_polyline = true;
    double polyline_margin = 0.0;
    bool closed_loop = false;
    double detour_extra_margin = 0.02;

    double yaw_min_angle = -3.14159265359;
    double yaw_max_angle = 3.14159265359;
    double pitch_min_angle = -3.14159265359;
    double pitch_max_angle = 3.14159265359;
  };

  struct AdjustResult
  {
    double yaw = 0.0;
    double pitch = 0.0;
    bool angle_cap_applied = false;
    bool target_projected_to_safe = false;
    bool escaping_from_forbidden_current = false;
    bool detour_waypoint_used = false;
    bool path_blocked_without_detour = false;
  };

  ForbiddenRegionPlanner() = default;

  void setConfig(const Config & config);

  bool configured() const;
  std::size_t boundaryPointCount() const;
  bool isPointInForbiddenRegion(double yaw, double pitch) const;

  AdjustResult adjustTarget(
    double yaw, double pitch,
    bool has_current_command, double current_yaw, double current_pitch) const;

private:
  struct AnglePoint
  {
    double yaw = 0.0;
    double pitch = 0.0;
  };

  struct ForbiddenHit
  {
    bool inside = false;
    bool is_segment = false;
    std::size_t idx_a = 0;
    std::size_t idx_b = 0;
    AnglePoint nearest{};
    AnglePoint seg_a{};
    AnglePoint seg_b{};
    double clearance = 0.0;
    double signed_distance = 0.0;  // distance - clearance
  };

  static constexpr double kForbiddenEps = 1e-4;

  AnglePoint makeAnglePoint(double yaw, double pitch) const;
  AnglePoint add(const AnglePoint & a, const AnglePoint & b) const;
  AnglePoint sub(const AnglePoint & a, const AnglePoint & b) const;
  AnglePoint scale(const AnglePoint & a, double s) const;
  double dot(const AnglePoint & a, const AnglePoint & b) const;
  double normSq(const AnglePoint & a) const;
  double norm(const AnglePoint & a) const;
  double distance(const AnglePoint & a, const AnglePoint & b) const;
  AnglePoint perpendicular(const AnglePoint & a) const;
  AnglePoint normalizeOr(const AnglePoint & v, const AnglePoint & fallback) const;

  double clampYaw(double angle) const;
  double clampPitch(double angle) const;
  AnglePoint clampAnglePoint(const AnglePoint & p) const;

  AnglePoint lerp(const AnglePoint & a, const AnglePoint & b, double t) const;
  AnglePoint closestPointOnSegment(
    const AnglePoint & p, const AnglePoint & a, const AnglePoint & b) const;
  double distancePointToSegment(
    const AnglePoint & p, const AnglePoint & a, const AnglePoint & b) const;
  double cross2D(const AnglePoint & a, const AnglePoint & b) const;
  bool segmentsIntersect(
    const AnglePoint & a, const AnglePoint & b, const AnglePoint & c, const AnglePoint & d) const;
  double distanceSegmentToSegment(
    const AnglePoint & a, const AnglePoint & b, const AnglePoint & c, const AnglePoint & d) const;

  bool forbiddenSegmentsEnabled() const;
  bool forbiddenPointsEnabled() const;
  std::size_t forbiddenSegmentCount() const;
  void getForbiddenSegment(std::size_t seg_idx, AnglePoint & a, AnglePoint & b) const;

  void loadForbiddenBoundaryPoints();
  bool findMostSevereForbiddenHit(const AnglePoint & p, ForbiddenHit & out_hit) const;
  AnglePoint projectOutsideForbiddenHit(
    const AnglePoint & p, const AnglePoint & reference, const ForbiddenHit & hit) const;
  AnglePoint projectToNearestSafePoint(
    const AnglePoint & target, const AnglePoint & reference, bool & adjusted) const;
  bool isPathBlockedByForbidden(const AnglePoint & start, const AnglePoint & goal) const;
  AnglePoint findLastSafePointOnSegment(const AnglePoint & start, const AnglePoint & goal) const;
  AnglePoint computeEndpointDetourWaypoint(bool use_start_endpoint) const;
  bool tryFindDetourWaypoint(
    const AnglePoint & current, const AnglePoint & goal, AnglePoint & waypoint_out) const;

  Config config_;
  std::vector<AnglePoint> forbidden_boundary_points_;
};

}  // namespace core_shooter
