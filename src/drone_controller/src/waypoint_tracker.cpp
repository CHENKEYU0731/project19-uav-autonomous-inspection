#include "drone_controller/waypoint_tracker.hpp"

#include <cmath>
#include <stdexcept>

namespace drone_controller
{

Position3D make_ned_target(
  const Position3D & home,
  const double north_offset,
  const double east_offset,
  const double altitude_m)
{
  return {
    home.x + north_offset,
    home.y + east_offset,
    home.z - altitude_m,
  };
}

double euclidean_distance(const Position3D & first, const Position3D & second)
{
  const double dx = second.x - first.x;
  const double dy = second.y - first.y;
  const double dz = second.z - first.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool is_finite_position(const Position3D & position)
{
  return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z);
}

bool sample_is_fresh(
  const SteadyClock::time_point received_at,
  const SteadyClock::time_point now,
  const SteadyClock::duration maximum_age)
{
  return maximum_age > SteadyClock::duration::zero() && now >= received_at &&
         now - received_at <= maximum_age;
}

bool landing_is_confirmed(const bool disarmed, const bool landed)
{
  return disarmed && landed;
}

WaypointTracker::WaypointTracker(
  const double acceptance_radius,
  const Clock::duration stable_duration)
: acceptance_radius_(acceptance_radius), stable_duration_(stable_duration)
{
  if (!std::isfinite(acceptance_radius_) || acceptance_radius_ <= 0.0) {
    throw std::invalid_argument("acceptance_radius must be finite and positive");
  }

  if (stable_duration_ <= Clock::duration::zero()) {
    throw std::invalid_argument("stable_duration must be positive");
  }
}

bool WaypointTracker::update(
  const Position3D & position,
  const Position3D & target,
  const Clock::time_point now)
{
  const double distance = euclidean_distance(position, target);

  if (!std::isfinite(distance) || distance > acceptance_radius_) {
    reset();
    return false;
  }

  if (!inside_radius_) {
    inside_radius_ = true;
    entered_radius_at_ = now;
    return false;
  }

  return now - entered_radius_at_ >= stable_duration_;
}

void WaypointTracker::reset()
{
  inside_radius_ = false;
  entered_radius_at_ = Clock::time_point{};
}

}  // namespace drone_controller
