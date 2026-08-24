// Copyright 2026 Project19 contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DRONE_CONTROLLER__WAYPOINT_TRACKER_HPP_
#define DRONE_CONTROLLER__WAYPOINT_TRACKER_HPP_

#include <chrono>

namespace drone_controller
{

struct Position3D
{
  double x;
  double y;
  double z;
};

using SteadyClock = std::chrono::steady_clock;

Position3D make_ned_target(
  const Position3D & home, double north_offset, double east_offset, double altitude_m);

Position3D map_enu_to_ned(const Position3D & map_enu_position);

double euclidean_distance(const Position3D & first, const Position3D & second);
bool is_finite_position(const Position3D & position);
bool sample_is_fresh(
  SteadyClock::time_point received_at, SteadyClock::time_point now,
  SteadyClock::duration maximum_age);
bool landing_is_confirmed(bool disarmed, bool landed);

class WaypointTracker
{
public:
  using Clock = SteadyClock;

  WaypointTracker(double acceptance_radius, Clock::duration stable_duration);

  bool update(const Position3D & position, const Position3D & target, Clock::time_point now);
  void reset();

private:
  double acceptance_radius_;
  Clock::duration stable_duration_;
  bool inside_radius_{false};
  Clock::time_point entered_radius_at_{};
};

}  // namespace drone_controller

#endif  // DRONE_CONTROLLER__WAYPOINT_TRACKER_HPP_
