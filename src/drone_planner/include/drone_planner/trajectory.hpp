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

#ifndef DRONE_PLANNER__TRAJECTORY_HPP_
#define DRONE_PLANNER__TRAJECTORY_HPP_

#include "drone_planner/grid_map.hpp"

#include <vector>

namespace drone_planner
{

struct KinematicLimits
{
  double maximum_velocity_m_s{};
  double maximum_acceleration_m_s2{};
  double sample_period_s{};
};

struct TrajectorySample2D
{
  double time_from_start_s{};
  Point2D position;
  Point2D velocity;
  Point2D acceleration;
};

bool has_line_of_sight(
  const InflatedGrid & grid,
  const GridCell & start,
  const GridCell & goal);

bool remaining_path_is_safe(
  const InflatedGrid & grid,
  const std::vector<GridCell> & path,
  const GridCell & current);

std::vector<GridCell> prune_path(
  const InflatedGrid & grid,
  const std::vector<GridCell> & path);

double quintic_segment_duration(
  double segment_length_m,
  const KinematicLimits & limits);

std::vector<TrajectorySample2D> parameterize_quintic(
  const std::vector<Point2D> & waypoints,
  const KinematicLimits & limits);

std::vector<TrajectorySample2D> parameterize_hold(
  const Point2D & position,
  const KinematicLimits & limits);

}  // namespace drone_planner

#endif  // DRONE_PLANNER__TRAJECTORY_HPP_
