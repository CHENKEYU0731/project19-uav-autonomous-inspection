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

#include "drone_planner/trajectory.hpp"

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <vector>

namespace drone_planner
{
namespace
{

constexpr double kMaximumQuinticFirstDerivative = 1.875;
constexpr double kMaximumQuinticSecondDerivative = 10.0 / 1.7320508075688772935;
constexpr double kGeometryTolerance = 1e-12;
constexpr std::size_t kMaximumTrajectorySampleCount = 1'000'000U;
constexpr double kMaximumTrajectoryDurationSeconds = 3'600.0;

void validate_limits(const KinematicLimits & limits)
{
  if (!std::isfinite(limits.maximum_velocity_m_s) ||
    !std::isfinite(limits.maximum_acceleration_m_s2) ||
    !std::isfinite(limits.sample_period_s) ||
    limits.maximum_velocity_m_s <= 0.0 ||
    limits.maximum_acceleration_m_s2 <= 0.0 ||
    limits.sample_period_s <= 0.0 ||
    limits.sample_period_s > kMaximumTrajectoryDurationSeconds)
  {
    throw std::invalid_argument("kinematic limits must be finite and positive");
  }
}

void require_free_cell(const InflatedGrid & grid, const GridCell & cell)
{
  if (!grid.contains(cell) || grid.blocked(cell)) {
    throw std::invalid_argument("path contains a blocked or out-of-bounds cell");
  }
}

int direction(const int value)
{
  return (value > 0) - (value < 0);
}

}  // namespace

bool has_line_of_sight(
  const InflatedGrid & grid,
  const GridCell & start,
  const GridCell & goal)
{
  if (!grid.contains(start) || !grid.contains(goal) ||
    grid.blocked(start) || grid.blocked(goal))
  {
    return false;
  }
  if (start == goal) {
    return true;
  }

  GridCell current = start;
  const int delta_column = goal.column - start.column;
  const int delta_row = goal.row - start.row;
  const int column_step = direction(delta_column);
  const int row_step = direction(delta_row);
  const double column_delta_t = delta_column == 0 ?
    std::numeric_limits<double>::infinity() :
    1.0 / static_cast<double>(std::abs(delta_column));
  const double row_delta_t = delta_row == 0 ?
    std::numeric_limits<double>::infinity() :
    1.0 / static_cast<double>(std::abs(delta_row));
  double next_column_boundary_t = column_delta_t / 2.0;
  double next_row_boundary_t = row_delta_t / 2.0;

  while (current != goal) {
    if (next_column_boundary_t + kGeometryTolerance < next_row_boundary_t) {
      current.column += column_step;
      next_column_boundary_t += column_delta_t;
    } else if (next_row_boundary_t + kGeometryTolerance < next_column_boundary_t) {
      current.row += row_step;
      next_row_boundary_t += row_delta_t;
    } else {
      const GridCell column_neighbour{current.column + column_step, current.row};
      const GridCell row_neighbour{current.column, current.row + row_step};
      if (grid.blocked(column_neighbour) || grid.blocked(row_neighbour)) {
        return false;
      }
      current.column += column_step;
      current.row += row_step;
      next_column_boundary_t += column_delta_t;
      next_row_boundary_t += row_delta_t;
    }
    if (grid.blocked(current)) {
      return false;
    }
  }
  return true;
}

bool remaining_path_is_safe(
  const InflatedGrid & grid,
  const std::vector<GridCell> & path,
  const GridCell & current)
{
  if (path.empty() || !grid.contains(current)) {
    return false;
  }
  const auto squared_distance = [&current](const GridCell & cell) {
      const std::int64_t delta_column =
        static_cast<std::int64_t>(cell.column) - current.column;
      const std::int64_t delta_row =
        static_cast<std::int64_t>(cell.row) - current.row;
      return delta_column * delta_column + delta_row * delta_row;
    };
  const auto nearest = std::min_element(
    path.begin(), path.end(),
    [&squared_distance](const GridCell & first, const GridCell & second) {
      return squared_distance(first) < squared_distance(second);
    });
  for (auto cell = nearest; cell != path.end(); ++cell) {
    if (grid.blocked(*cell)) {
      return false;
    }
    const auto next = std::next(cell);
    if (next != path.end() && !has_line_of_sight(grid, *cell, *next)) {
      return false;
    }
  }
  return true;
}

std::vector<GridCell> prune_path(
  const InflatedGrid & grid,
  const std::vector<GridCell> & path)
{
  if (path.empty()) {
    throw std::invalid_argument("path must not be empty");
  }
  for (const GridCell & cell : path) {
    require_free_cell(grid, cell);
  }
  for (std::size_t index = 1U; index < path.size(); ++index) {
    if (!has_line_of_sight(grid, path[index - 1U], path[index])) {
      throw std::invalid_argument("input path contains an unsafe segment");
    }
  }
  if (path.size() == 1U) {
    return path;
  }

  std::vector<GridCell> result{path.front()};
  std::size_t anchor = 0U;
  while (anchor + 1U < path.size()) {
    std::size_t candidate = path.size() - 1U;
    while (candidate > anchor + 1U &&
      !has_line_of_sight(grid, path[anchor], path[candidate]))
    {
      --candidate;
    }
    if (!has_line_of_sight(grid, path[anchor], path[candidate])) {
      throw std::invalid_argument("path cannot be pruned without an unsafe segment");
    }
    result.push_back(path[candidate]);
    anchor = candidate;
  }
  return result;
}

double quintic_segment_duration(
  const double segment_length_m,
  const KinematicLimits & limits)
{
  validate_limits(limits);
  if (!std::isfinite(segment_length_m) || segment_length_m <= 0.0) {
    throw std::invalid_argument("segment length must be finite and positive");
  }
  const double velocity_duration =
    kMaximumQuinticFirstDerivative * segment_length_m /
    limits.maximum_velocity_m_s;
  const double acceleration_duration = std::sqrt(
    kMaximumQuinticSecondDerivative * segment_length_m /
    limits.maximum_acceleration_m_s2);
  const double duration_s = std::max(
    {velocity_duration, acceleration_duration, limits.sample_period_s});
  if (!std::isfinite(duration_s) ||
    duration_s > kMaximumTrajectoryDurationSeconds)
  {
    throw std::invalid_argument("segment duration exceeds the safety limit");
  }
  return duration_s;
}

std::vector<TrajectorySample2D> parameterize_quintic(
  const std::vector<Point2D> & waypoints,
  const KinematicLimits & limits)
{
  validate_limits(limits);
  if (waypoints.size() < 2U) {
    throw std::invalid_argument("at least two waypoints are required");
  }
  for (const Point2D & waypoint : waypoints) {
    if (!std::isfinite(waypoint.x) || !std::isfinite(waypoint.y)) {
      throw std::invalid_argument("waypoints must be finite");
    }
  }

  std::vector<TrajectorySample2D> samples;
  samples.push_back(
    TrajectorySample2D{0.0, waypoints.front(), Point2D{}, Point2D{}});
  double segment_start_time_s = 0.0;
  for (std::size_t segment = 1U; segment < waypoints.size(); ++segment) {
    const Point2D start = waypoints[segment - 1U];
    const Point2D goal = waypoints[segment];
    const Point2D displacement{goal.x - start.x, goal.y - start.y};
    const double length_m = std::hypot(displacement.x, displacement.y);
    if (!std::isfinite(length_m) || length_m <= kGeometryTolerance) {
      throw std::invalid_argument("consecutive waypoints must be distinct");
    }
    const double duration_s = quintic_segment_duration(length_m, limits);
    if (duration_s > kMaximumTrajectoryDurationSeconds - segment_start_time_s) {
      throw std::invalid_argument("trajectory duration exceeds the safety limit");
    }
    const double segment_end_time_s = segment_start_time_s + duration_s;
    const double requested_interval_count =
      std::ceil(duration_s / limits.sample_period_s);
    if (!std::isfinite(requested_interval_count) ||
      requested_interval_count >
      static_cast<double>(kMaximumTrajectorySampleCount - samples.size()))
    {
      throw std::invalid_argument("trajectory sample count exceeds the safety limit");
    }
    const std::size_t interval_count = std::max<std::size_t>(
      1U, static_cast<std::size_t>(requested_interval_count));
    for (std::size_t interval = 1U; interval <= interval_count; ++interval) {
      const bool is_endpoint = interval == interval_count;
      const double local_time_s = is_endpoint ?
        duration_s :
        static_cast<double>(interval) * limits.sample_period_s;
      const double normalized_time = local_time_s / duration_s;
      const double squared_time = normalized_time * normalized_time;
      const double cubed_time = squared_time * normalized_time;
      const double fourth_power = cubed_time * normalized_time;
      const double fifth_power = fourth_power * normalized_time;
      const double position_scale =
        10.0 * cubed_time - 15.0 * fourth_power + 6.0 * fifth_power;
      const double velocity_scale =
        (30.0 * squared_time - 60.0 * cubed_time + 30.0 * fourth_power) /
        duration_s;
      const double acceleration_scale =
        (60.0 * normalized_time - 180.0 * squared_time + 120.0 * cubed_time) /
        (duration_s * duration_s);

      TrajectorySample2D sample;
      sample.time_from_start_s = is_endpoint ?
        segment_end_time_s : segment_start_time_s + local_time_s;
      if (!std::isfinite(sample.time_from_start_s) ||
        sample.time_from_start_s > segment_end_time_s)
      {
        throw std::invalid_argument("trajectory timestamp exceeds the safety limit");
      }
      sample.position = Point2D{
        start.x + displacement.x * position_scale,
        start.y + displacement.y * position_scale,
      };
      sample.velocity = Point2D{
        displacement.x * velocity_scale,
        displacement.y * velocity_scale,
      };
      sample.acceleration = Point2D{
        displacement.x * acceleration_scale,
        displacement.y * acceleration_scale,
      };
      if (is_endpoint) {
        sample.position = goal;
        sample.velocity = Point2D{};
        sample.acceleration = Point2D{};
      }
      samples.push_back(sample);
    }
    segment_start_time_s = segment_end_time_s;
  }
  return samples;
}

std::vector<TrajectorySample2D> parameterize_hold(
  const Point2D & position,
  const KinematicLimits & limits)
{
  validate_limits(limits);
  if (!std::isfinite(position.x) || !std::isfinite(position.y)) {
    throw std::invalid_argument("hold position must be finite");
  }

  // Keep two strictly increasing timestamps so consumers can validate and
  // execute a completed goal as a normal, stationary trajectory.
  return {
    TrajectorySample2D{0.0, position, Point2D{}, Point2D{}},
    TrajectorySample2D{limits.sample_period_s, position, Point2D{}, Point2D{}},
  };
}

}  // namespace drone_planner
