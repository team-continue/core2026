#include "core_shooter/forbidden_region_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace core_shooter
{

void ForbiddenRegionPlanner::setConfig(const Config & config)
{
  config_ = config;
  loadForbiddenBoundaryPoints();
}

bool ForbiddenRegionPlanner::configured() const
{
  return config_.enabled &&
         (!forbidden_boundary_points_.empty()) &&
         (forbiddenPointsEnabled() || forbiddenSegmentsEnabled());
}

std::size_t ForbiddenRegionPlanner::boundaryPointCount() const
{
  return forbidden_boundary_points_.size();
}

bool ForbiddenRegionPlanner::isPointInForbiddenRegion(double yaw, double pitch) const
{
  if (!configured()) {
    return false;
  }
  ForbiddenHit hit{};
  return findMostSevereForbiddenHit(makeAnglePoint(yaw, pitch), hit);
}

ForbiddenRegionPlanner::AdjustResult ForbiddenRegionPlanner::adjustTarget(
  double yaw, double pitch,
  bool has_current_command, double current_yaw, double current_pitch) const
{
  AdjustResult result{};
  const double yaw_capped = clampYaw(yaw);
  const double pitch_capped = clampPitch(pitch);
  result.yaw = yaw_capped;
  result.pitch = pitch_capped;
  result.angle_cap_applied =
    (std::fabs(yaw - yaw_capped) > 1e-9) || (std::fabs(pitch - pitch_capped) > 1e-9);

  if (!configured()) {
    return result;
  }

  const AnglePoint desired = makeAnglePoint(result.yaw, result.pitch);
  const AnglePoint current =
    has_current_command ? makeAnglePoint(current_yaw, current_pitch) : desired;

  if (has_current_command && isPointInForbiddenRegion(current.yaw, current.pitch)) {
    bool adjusted = false;
    const AnglePoint escape = projectToNearestSafePoint(current, desired, adjusted);
    result.yaw = escape.yaw;
    result.pitch = escape.pitch;
    result.escaping_from_forbidden_current = adjusted;
    return result;
  }

  bool target_adjusted = false;
  const AnglePoint safe_target = projectToNearestSafePoint(desired, current, target_adjusted);
  result.target_projected_to_safe = target_adjusted;

  if (!has_current_command) {
    result.yaw = safe_target.yaw;
    result.pitch = safe_target.pitch;
    return result;
  }

  if (!isPathBlockedByForbidden(current, safe_target)) {
    result.yaw = safe_target.yaw;
    result.pitch = safe_target.pitch;
    return result;
  }

  AnglePoint waypoint{};
  if (tryFindDetourWaypoint(current, safe_target, waypoint)) {
    result.yaw = waypoint.yaw;
    result.pitch = waypoint.pitch;
    result.detour_waypoint_used = true;
    return result;
  }

  const AnglePoint reachable = findLastSafePointOnSegment(current, safe_target);
  result.yaw = reachable.yaw;
  result.pitch = reachable.pitch;
  result.path_blocked_without_detour = true;
  return result;
}

ForbiddenRegionPlanner::AnglePoint ForbiddenRegionPlanner::makeAnglePoint(
  double yaw,
  double pitch) const
{
  return AnglePoint{yaw, pitch};
}

ForbiddenRegionPlanner::AnglePoint ForbiddenRegionPlanner::add(
  const AnglePoint & a, const AnglePoint & b) const
{
  return AnglePoint{a.yaw + b.yaw, a.pitch + b.pitch};
}

ForbiddenRegionPlanner::AnglePoint ForbiddenRegionPlanner::sub(
  const AnglePoint & a, const AnglePoint & b) const
{
  return AnglePoint{a.yaw - b.yaw, a.pitch - b.pitch};
}

ForbiddenRegionPlanner::AnglePoint ForbiddenRegionPlanner::scale(
  const AnglePoint & a, double s) const
{
  return AnglePoint{a.yaw * s, a.pitch * s};
}

double ForbiddenRegionPlanner::dot(const AnglePoint & a, const AnglePoint & b) const
{
  return a.yaw * b.yaw + a.pitch * b.pitch;
}

double ForbiddenRegionPlanner::normSq(const AnglePoint & a) const
{
  return dot(a, a);
}

double ForbiddenRegionPlanner::norm(const AnglePoint & a) const
{
  return std::sqrt(normSq(a));
}

double ForbiddenRegionPlanner::distance(const AnglePoint & a, const AnglePoint & b) const
{
  return norm(sub(a, b));
}

ForbiddenRegionPlanner::AnglePoint ForbiddenRegionPlanner::perpendicular(const AnglePoint & a) const
{
  return AnglePoint{-a.pitch, a.yaw};
}

ForbiddenRegionPlanner::AnglePoint ForbiddenRegionPlanner::normalizeOr(
  const AnglePoint & v, const AnglePoint & fallback) const
{
  const double n = norm(v);
  if (n > 1e-12) {
    return scale(v, 1.0 / n);
  }
  const double fn = norm(fallback);
  if (fn > 1e-12) {
    return scale(fallback, 1.0 / fn);
  }
  return AnglePoint{1.0, 0.0};
}

double ForbiddenRegionPlanner::clampYaw(double angle) const
{
  return std::clamp(angle, config_.yaw_min_angle, config_.yaw_max_angle);
}

double ForbiddenRegionPlanner::clampPitch(double angle) const
{
  return std::clamp(angle, config_.pitch_min_angle, config_.pitch_max_angle);
}

ForbiddenRegionPlanner::AnglePoint ForbiddenRegionPlanner::clampAnglePoint(const AnglePoint & p)
const
{
  return AnglePoint{clampYaw(p.yaw), clampPitch(p.pitch)};
}

ForbiddenRegionPlanner::AnglePoint ForbiddenRegionPlanner::lerp(
  const AnglePoint & a, const AnglePoint & b, double t) const
{
  return AnglePoint{
    a.yaw + (b.yaw - a.yaw) * t,
    a.pitch + (b.pitch - a.pitch) * t
  };
}

ForbiddenRegionPlanner::AnglePoint ForbiddenRegionPlanner::closestPointOnSegment(
  const AnglePoint & p, const AnglePoint & a, const AnglePoint & b) const
{
  const AnglePoint ab = sub(b, a);
  const double ab_norm_sq = normSq(ab);
  if (ab_norm_sq <= 1e-12) {
    return a;
  }
  const double t = std::clamp(dot(sub(p, a), ab) / ab_norm_sq, 0.0, 1.0);
  return lerp(a, b, t);
}

double ForbiddenRegionPlanner::distancePointToSegment(
  const AnglePoint & p, const AnglePoint & a, const AnglePoint & b) const
{
  return distance(p, closestPointOnSegment(p, a, b));
}

double ForbiddenRegionPlanner::cross2D(const AnglePoint & a, const AnglePoint & b) const
{
  return a.yaw * b.pitch - a.pitch * b.yaw;
}

bool ForbiddenRegionPlanner::segmentsIntersect(
  const AnglePoint & a, const AnglePoint & b, const AnglePoint & c, const AnglePoint & d) const
{
  const AnglePoint ab = sub(b, a);
  const AnglePoint ac = sub(c, a);
  const AnglePoint ad = sub(d, a);
  const AnglePoint cd = sub(d, c);
  const AnglePoint ca = sub(a, c);
  const AnglePoint cb = sub(b, c);

  const double c1 = cross2D(ab, ac);
  const double c2 = cross2D(ab, ad);
  const double c3 = cross2D(cd, ca);
  const double c4 = cross2D(cd, cb);

  auto on_segment = [&](const AnglePoint & p, const AnglePoint & q, const AnglePoint & r) {
      return
        std::min(p.yaw, r.yaw) - 1e-12 <= q.yaw && q.yaw <= std::max(p.yaw, r.yaw) + 1e-12 &&
        std::min(p.pitch, r.pitch) - 1e-12 <= q.pitch &&
        q.pitch <= std::max(p.pitch, r.pitch) + 1e-12;
    };

  if (((c1 > 0 && c2 < 0) || (c1 < 0 && c2 > 0)) &&
    ((c3 > 0 && c4 < 0) || (c3 < 0 && c4 > 0)))
  {
    return true;
  }

  if (std::fabs(c1) <= 1e-12 && on_segment(a, c, b)) {return true;}
  if (std::fabs(c2) <= 1e-12 && on_segment(a, d, b)) {return true;}
  if (std::fabs(c3) <= 1e-12 && on_segment(c, a, d)) {return true;}
  if (std::fabs(c4) <= 1e-12 && on_segment(c, b, d)) {return true;}
  return false;
}

double ForbiddenRegionPlanner::distanceSegmentToSegment(
  const AnglePoint & a, const AnglePoint & b, const AnglePoint & c, const AnglePoint & d) const
{
  if (segmentsIntersect(a, b, c, d)) {
    return 0.0;
  }
  return std::min(
    std::min(distancePointToSegment(a, c, d), distancePointToSegment(b, c, d)),
    std::min(distancePointToSegment(c, a, b), distancePointToSegment(d, a, b)));
}

bool ForbiddenRegionPlanner::forbiddenSegmentsEnabled() const
{
  return config_.connect_as_polyline &&
         forbidden_boundary_points_.size() >= 2 &&
         config_.polyline_margin > 0.0;
}

bool ForbiddenRegionPlanner::forbiddenPointsEnabled() const
{
  return !forbidden_boundary_points_.empty() && config_.point_radius > 0.0;
}

std::size_t ForbiddenRegionPlanner::forbiddenSegmentCount() const
{
  if (!forbiddenSegmentsEnabled()) {
    return 0;
  }
  const std::size_t n = forbidden_boundary_points_.size();
  return (n - 1) + ((config_.closed_loop && n >= 3) ? 1 : 0);
}

void ForbiddenRegionPlanner::getForbiddenSegment(
  std::size_t seg_idx, AnglePoint & a,
  AnglePoint & b) const
{
  const std::size_t n = forbidden_boundary_points_.size();
  if (seg_idx < n - 1) {
    a = forbidden_boundary_points_[seg_idx];
    b = forbidden_boundary_points_[seg_idx + 1];
    return;
  }
  a = forbidden_boundary_points_[n - 1];
  b = forbidden_boundary_points_[0];
}

void ForbiddenRegionPlanner::loadForbiddenBoundaryPoints()
{
  forbidden_boundary_points_.clear();
  if (config_.points_yaw.size() != config_.points_pitch.size()) {
    throw std::runtime_error("forbidden_region_points_yaw/pitch size mismatch");
  }
  forbidden_boundary_points_.reserve(config_.points_yaw.size());
  for (std::size_t i = 0; i < config_.points_yaw.size(); ++i) {
    forbidden_boundary_points_.push_back(
      AnglePoint{config_.points_yaw[i],
        config_.points_pitch[i]});
  }
  if (config_.closed_loop && forbidden_boundary_points_.size() < 3) {
    config_.closed_loop = false;
  }
}

bool ForbiddenRegionPlanner::findMostSevereForbiddenHit(
  const AnglePoint & p,
  ForbiddenHit & out_hit) const
{
  bool found = false;
  ForbiddenHit best;
  best.signed_distance = std::numeric_limits<double>::infinity();

  if (forbiddenPointsEnabled()) {
    for (std::size_t i = 0; i < forbidden_boundary_points_.size(); ++i) {
      const AnglePoint & center = forbidden_boundary_points_[i];
      const double d = distance(p, center);
      const double signed_distance = d - config_.point_radius;
      if (signed_distance <= 0.0 && signed_distance < best.signed_distance) {
        best.inside = true;
        best.is_segment = false;
        best.idx_a = i;
        best.idx_b = i;
        best.nearest = center;
        best.clearance = config_.point_radius;
        best.signed_distance = signed_distance;
        found = true;
      }
    }
  }

  if (forbiddenSegmentsEnabled()) {
    for (std::size_t i = 0; i < forbiddenSegmentCount(); ++i) {
      AnglePoint a{};
      AnglePoint b{};
      getForbiddenSegment(i, a, b);
      const AnglePoint nearest = closestPointOnSegment(p, a, b);
      const double d = distance(p, nearest);
      const double signed_distance = d - config_.polyline_margin;
      if (signed_distance <= 0.0 && signed_distance < best.signed_distance) {
        best.inside = true;
        best.is_segment = true;
        best.idx_a = i;
        best.idx_b = i;
        best.nearest = nearest;
        best.seg_a = a;
        best.seg_b = b;
        best.clearance = config_.polyline_margin;
        best.signed_distance = signed_distance;
        found = true;
      }
    }
  }

  if (found) {
    out_hit = best;
  }
  return found;
}

ForbiddenRegionPlanner::AnglePoint ForbiddenRegionPlanner::projectOutsideForbiddenHit(
  const AnglePoint & p, const AnglePoint & reference, const ForbiddenHit & hit) const
{
  AnglePoint direction = sub(p, hit.nearest);
  if (norm(direction) <= 1e-12) {
    if (hit.is_segment) {
      const AnglePoint seg_dir = sub(hit.seg_b, hit.seg_a);
      direction = perpendicular(seg_dir);
      if (dot(direction, sub(reference, hit.nearest)) < 0.0) {
        direction = scale(direction, -1.0);
      }
    } else {
      direction = sub(reference, hit.nearest);
    }
  }
  const AnglePoint unit = normalizeOr(direction, AnglePoint{1.0, 0.0});
  return clampAnglePoint(add(hit.nearest, scale(unit, hit.clearance + kForbiddenEps)));
}

ForbiddenRegionPlanner::AnglePoint ForbiddenRegionPlanner::projectToNearestSafePoint(
  const AnglePoint & target, const AnglePoint & reference, bool & adjusted) const
{
  adjusted = false;
  if (!configured()) {
    return target;
  }

  AnglePoint p = clampAnglePoint(target);
  ForbiddenHit hit{};
  if (!findMostSevereForbiddenHit(p, hit)) {
    return p;
  }
  adjusted = true;

  for (int iter = 0; iter < 8; ++iter) {
    p = projectOutsideForbiddenHit(p, reference, hit);
    if (!findMostSevereForbiddenHit(p, hit)) {
      return p;
    }
  }
  return p;
}

bool ForbiddenRegionPlanner::isPathBlockedByForbidden(
  const AnglePoint & start,
  const AnglePoint & goal) const
{
  if (!configured()) {
    return false;
  }
  if (forbiddenPointsEnabled()) {
    for (const auto & center : forbidden_boundary_points_) {
      if (distancePointToSegment(center, start, goal) <= config_.point_radius) {
        return true;
      }
    }
  }
  if (forbiddenSegmentsEnabled()) {
    for (std::size_t i = 0; i < forbiddenSegmentCount(); ++i) {
      AnglePoint a{};
      AnglePoint b{};
      getForbiddenSegment(i, a, b);
      if (distanceSegmentToSegment(start, goal, a, b) <= config_.polyline_margin) {
        return true;
      }
    }
  }
  return false;
}

ForbiddenRegionPlanner::AnglePoint ForbiddenRegionPlanner::findLastSafePointOnSegment(
  const AnglePoint & start, const AnglePoint & goal) const
{
  if (!configured() || !isPathBlockedByForbidden(start, goal)) {
    return goal;
  }
  double lo = 0.0;
  double hi = 1.0;
  for (int i = 0; i < 24; ++i) {
    const double mid = 0.5 * (lo + hi);
    const AnglePoint p = lerp(start, goal, mid);
    if (isPointInForbiddenRegion(p.yaw, p.pitch)) {
      hi = mid;
    } else {
      lo = mid;
    }
  }
  return clampAnglePoint(lerp(start, goal, lo));
}

ForbiddenRegionPlanner::AnglePoint ForbiddenRegionPlanner::computeEndpointDetourWaypoint(
  bool use_start_endpoint) const
{
  AnglePoint endpoint{};
  AnglePoint neighbor{};
  if (use_start_endpoint) {
    endpoint = forbidden_boundary_points_.front();
    neighbor = forbidden_boundary_points_[1];
  } else {
    endpoint = forbidden_boundary_points_.back();
    neighbor = forbidden_boundary_points_[forbidden_boundary_points_.size() - 2];
  }

  const AnglePoint extend_dir = normalizeOr(
    sub(endpoint, neighbor), AnglePoint{use_start_endpoint ? -1.0 : 1.0, 0.0});
  const double clearance = std::max(config_.point_radius, config_.polyline_margin) +
    config_.detour_extra_margin;
  return clampAnglePoint(add(endpoint, scale(extend_dir, std::max(clearance, kForbiddenEps))));
}

bool ForbiddenRegionPlanner::tryFindDetourWaypoint(
  const AnglePoint & current, const AnglePoint & goal, AnglePoint & waypoint_out) const
{
  if (!forbiddenSegmentsEnabled() || config_.closed_loop || forbidden_boundary_points_.size() < 2) {
    return false;
  }

  bool found = false;
  double best_cost = std::numeric_limits<double>::infinity();

  for (bool use_start : {true, false}) {
    AnglePoint candidate = computeEndpointDetourWaypoint(use_start);
    bool projected = false;
    candidate = projectToNearestSafePoint(candidate, current, projected);

    if (isPointInForbiddenRegion(candidate.yaw, candidate.pitch)) {
      continue;
    }
    if (isPathBlockedByForbidden(current, candidate)) {
      continue;
    }
    if (isPathBlockedByForbidden(candidate, goal)) {
      continue;
    }

    const double cost = distance(current, candidate) + distance(candidate, goal);
    if (!found || cost < best_cost) {
      best_cost = cost;
      waypoint_out = candidate;
      found = true;
    }
  }
  return found;
}

}  // namespace core_shooter
