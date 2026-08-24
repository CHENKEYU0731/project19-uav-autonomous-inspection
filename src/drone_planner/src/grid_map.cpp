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

#include "drone_planner/grid_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace drone_planner
{
namespace
{

constexpr std::size_t kMaximumGridCellCount = 262'144U;
constexpr std::size_t kMaximumInflationOffsetCandidates = 1'000'000U;
constexpr std::size_t kMaximumInflationApplications = 10'000'000U;
constexpr double kResolutionRelativeTolerance = 1e-12;
constexpr double kHalfCellDiagonalInCells = 0.7071067811865475244;

bool valid_cell_value(const std::int8_t value)
{
  return value == kUnknown || value == kFree || value == kOccupied;
}

std::size_t validated_cell_count(const GridGeometry & geometry)
{
  const double width_m =
    static_cast<double>(geometry.width) * geometry.resolution_m;
  const double height_m =
    static_cast<double>(geometry.height) * geometry.resolution_m;
  const double maximum_x = geometry.origin_x + width_m;
  const double maximum_y = geometry.origin_y + height_m;
  if (!std::isfinite(geometry.resolution_m) || geometry.resolution_m <= 0.0 ||
    !std::isfinite(geometry.origin_x) || !std::isfinite(geometry.origin_y) ||
    geometry.width == 0U || geometry.height == 0U ||
    geometry.width > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
    geometry.height > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
    geometry.width > std::numeric_limits<std::size_t>::max() / geometry.height ||
    !std::isfinite(width_m) || !std::isfinite(height_m) ||
    !std::isfinite(maximum_x) || !std::isfinite(maximum_y) ||
    maximum_x <= geometry.origin_x || maximum_y <= geometry.origin_y ||
    !std::isfinite(std::hypot(width_m, height_m)))
  {
    throw std::invalid_argument("invalid grid geometry");
  }
  const std::size_t cell_count = geometry.width * geometry.height;
  if (cell_count > kMaximumGridCellCount) {
    throw std::invalid_argument("grid exceeds the planner safety limit");
  }
  return cell_count;
}

bool matching_resolution(const double first, const double second)
{
  return std::abs(first - second) <=
         kResolutionRelativeTolerance * std::max(first, second);
}

double map_diagonal(const GridGeometry & geometry)
{
  return std::hypot(
    static_cast<double>(geometry.width) * geometry.resolution_m,
    static_cast<double>(geometry.height) * geometry.resolution_m);
}

}  // namespace

bool GridCell::operator==(const GridCell & other) const
{
  return column == other.column && row == other.row;
}

bool GridCell::operator!=(const GridCell & other) const
{
  return !(*this == other);
}

GridMap::GridMap(GridGeometry geometry)
: GridMap(
    geometry,
    std::vector<std::int8_t>(validated_cell_count(geometry), kUnknown))
{
}

GridMap::GridMap(GridGeometry geometry, std::vector<std::int8_t> cells)
: geometry_(geometry), cells_(std::move(cells))
{
  const std::size_t expected_cell_count = validated_cell_count(geometry_);
  if (cells_.size() != expected_cell_count ||
    !std::all_of(cells_.begin(), cells_.end(), valid_cell_value))
  {
    throw std::invalid_argument("grid cells do not match the geometry contract");
  }
}

const GridGeometry & GridMap::geometry() const
{
  return geometry_;
}

const std::vector<std::int8_t> & GridMap::cells() const
{
  return cells_;
}

std::int8_t GridMap::at(const GridCell & cell) const
{
  return cells_.at(index(cell));
}

void GridMap::set(const GridCell & cell, const std::int8_t value)
{
  if (!valid_cell_value(value)) {
    throw std::invalid_argument("unsupported occupancy value");
  }
  cells_.at(index(cell)) = value;
}

bool GridMap::contains(const GridCell & cell) const
{
  return cell.column >= 0 && cell.row >= 0 &&
         static_cast<std::size_t>(cell.column) < geometry_.width &&
         static_cast<std::size_t>(cell.row) < geometry_.height;
}

std::optional<GridCell> GridMap::world_to_cell(const Point2D & point) const
{
  if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
    return std::nullopt;
  }
  const double column_value = (point.x - geometry_.origin_x) / geometry_.resolution_m;
  const double row_value = (point.y - geometry_.origin_y) / geometry_.resolution_m;
  if (column_value < 0.0 || row_value < 0.0 ||
    column_value >= static_cast<double>(geometry_.width) ||
    row_value >= static_cast<double>(geometry_.height))
  {
    return std::nullopt;
  }
  return GridCell{
    static_cast<int>(std::floor(column_value)),
    static_cast<int>(std::floor(row_value)),
  };
}

Point2D GridMap::cell_center(const GridCell & cell) const
{
  if (!contains(cell)) {
    throw std::out_of_range("grid cell lies outside the map");
  }
  return Point2D{
    geometry_.origin_x +
    (static_cast<double>(cell.column) + 0.5) * geometry_.resolution_m,
    geometry_.origin_y +
    (static_cast<double>(cell.row) + 0.5) * geometry_.resolution_m,
  };
}

void GridMap::integrate(const GridMap & observation)
{
  if (!matching_resolution(
      geometry_.resolution_m, observation.geometry().resolution_m))
  {
    throw std::invalid_argument("rolling-grid resolution does not match the fused grid");
  }

  const GridGeometry & observed_geometry = observation.geometry();
  const double resolution_m = geometry_.resolution_m;
  const double observed_maximum_x = observed_geometry.origin_x +
    static_cast<double>(observed_geometry.width) * resolution_m;
  const double observed_maximum_y = observed_geometry.origin_y +
    static_cast<double>(observed_geometry.height) * resolution_m;

  for (std::size_t target_row = 0; target_row < geometry_.height; ++target_row) {
    const double target_minimum_y = geometry_.origin_y +
      static_cast<double>(target_row) * resolution_m;
    const double target_maximum_y = target_minimum_y + resolution_m;
    for (std::size_t target_column = 0; target_column < geometry_.width;
      ++target_column)
    {
      const double target_minimum_x = geometry_.origin_x +
        static_cast<double>(target_column) * resolution_m;
      const double target_maximum_x = target_minimum_x + resolution_m;
      if (target_maximum_x <= observed_geometry.origin_x ||
        target_minimum_x >= observed_maximum_x ||
        target_maximum_y <= observed_geometry.origin_y ||
        target_minimum_y >= observed_maximum_y)
      {
        continue;
      }

      const double relative_minimum_x =
        (target_minimum_x - observed_geometry.origin_x) / resolution_m;
      const double relative_maximum_x =
        (target_maximum_x - observed_geometry.origin_x) / resolution_m;
      const double relative_minimum_y =
        (target_minimum_y - observed_geometry.origin_y) / resolution_m;
      const double relative_maximum_y =
        (target_maximum_y - observed_geometry.origin_y) / resolution_m;
      const int first_column = std::max(
        0, static_cast<int>(std::floor(relative_minimum_x)) - 1);
      const int last_column = std::min(
        static_cast<int>(observed_geometry.width) - 1,
        static_cast<int>(std::ceil(relative_maximum_x)) + 1);
      const int first_row = std::max(
        0, static_cast<int>(std::floor(relative_minimum_y)) - 1);
      const int last_row = std::min(
        static_cast<int>(observed_geometry.height) - 1,
        static_cast<int>(std::ceil(relative_maximum_y)) + 1);

      bool has_overlap = false;
      bool has_occupied_overlap = false;
      bool all_overlapping_cells_are_free = true;
      for (int observed_row = first_row; observed_row <= last_row; ++observed_row) {
        const double cell_minimum_y = observed_geometry.origin_y +
          static_cast<double>(observed_row) * resolution_m;
        const double cell_maximum_y = cell_minimum_y + resolution_m;
        for (int observed_column = first_column; observed_column <= last_column;
          ++observed_column)
        {
          const double cell_minimum_x = observed_geometry.origin_x +
            static_cast<double>(observed_column) * resolution_m;
          const double cell_maximum_x = cell_minimum_x + resolution_m;
          const bool overlaps =
            std::max(target_minimum_x, cell_minimum_x) <
            std::min(target_maximum_x, cell_maximum_x) &&
            std::max(target_minimum_y, cell_minimum_y) <
            std::min(target_maximum_y, cell_maximum_y);
          if (!overlaps) {
            continue;
          }
          has_overlap = true;
          const std::int8_t value = observation.at(
            GridCell{observed_column, observed_row});
          has_occupied_overlap = has_occupied_overlap || value == kOccupied;
          all_overlapping_cells_are_free =
            all_overlapping_cells_are_free && value == kFree;
        }
      }

      const GridCell target{
        static_cast<int>(target_column), static_cast<int>(target_row)};
      const bool observation_covers_target =
        has_overlap && all_overlapping_cells_are_free &&
        target_minimum_x >= observed_geometry.origin_x &&
        target_maximum_x <= observed_maximum_x &&
        target_minimum_y >= observed_geometry.origin_y &&
        target_maximum_y <= observed_maximum_y;
      if (has_occupied_overlap) {
        set(target, kOccupied);
      } else if (observation_covers_target) {
        set(target, kFree);
      }
    }
  }
}

void GridMap::clear_disk(const Point2D & center, const double radius_m)
{
  if (!std::isfinite(center.x) || !std::isfinite(center.y) ||
    !std::isfinite(radius_m) || radius_m < 0.0 ||
    radius_m > map_diagonal(geometry_))
  {
    throw std::invalid_argument("footprint disk must be finite and non-negative");
  }
  for (std::size_t row = 0; row < geometry_.height; ++row) {
    for (std::size_t column = 0; column < geometry_.width; ++column) {
      const GridCell cell{static_cast<int>(column), static_cast<int>(row)};
      const double minimum_x = geometry_.origin_x +
        static_cast<double>(column) * geometry_.resolution_m;
      const double maximum_x = minimum_x + geometry_.resolution_m;
      const double minimum_y = geometry_.origin_y +
        static_cast<double>(row) * geometry_.resolution_m;
      const double maximum_y = minimum_y + geometry_.resolution_m;
      const double farthest_x = std::max(
        std::abs(minimum_x - center.x), std::abs(maximum_x - center.x));
      const double farthest_y = std::max(
        std::abs(minimum_y - center.y), std::abs(maximum_y - center.y));
      if (std::hypot(farthest_x, farthest_y) <= radius_m) {
        set(cell, kFree);
      }
    }
  }
}

std::size_t GridMap::index(const GridCell & cell) const
{
  if (!contains(cell)) {
    throw std::out_of_range("grid cell lies outside the map");
  }
  return static_cast<std::size_t>(cell.row) * geometry_.width +
         static_cast<std::size_t>(cell.column);
}

InflatedGrid::InflatedGrid(const GridMap & source, const double inflation_radius_m)
: geometry_(source.geometry()), blocked_(source.cells().size(), 0U)
{
  if (!std::isfinite(inflation_radius_m) || inflation_radius_m < 0.0 ||
    inflation_radius_m > map_diagonal(geometry_))
  {
    throw std::invalid_argument("inflation radius must be finite and non-negative");
  }
  for (std::size_t row = 0; row < geometry_.height; ++row) {
    for (std::size_t column = 0; column < geometry_.width; ++column) {
      const GridCell cell{static_cast<int>(column), static_cast<int>(row)};
      blocked_[index(cell)] = source.at(cell) == kFree ? 0U : 1U;
    }
  }
  if (inflation_radius_m == 0.0) {
    return;
  }

  const double maximum_center_distance_cells =
    inflation_radius_m / geometry_.resolution_m + kHalfCellDiagonalInCells;
  if (!std::isfinite(maximum_center_distance_cells) ||
    maximum_center_distance_cells >
    static_cast<double>((std::numeric_limits<int>::max() - 1) / 2))
  {
    throw std::invalid_argument("inflation radius exceeds the cell-index safety limit");
  }
  const int cell_radius = static_cast<int>(
    std::ceil(maximum_center_distance_cells));
  const std::size_t candidate_side =
    2U * static_cast<std::size_t>(cell_radius) + 1U;
  if (candidate_side > kMaximumInflationOffsetCandidates / candidate_side) {
    throw std::invalid_argument("inflation offset template exceeds the safety limit");
  }

  std::vector<GridCell> offsets;
  offsets.reserve(candidate_side * candidate_side);
  for (int row_offset = -cell_radius; row_offset <= cell_radius; ++row_offset) {
    for (int column_offset = -cell_radius; column_offset <= cell_radius;
      ++column_offset)
    {
      if (std::hypot(
          static_cast<double>(column_offset),
          static_cast<double>(row_offset)) <= maximum_center_distance_cells)
      {
        offsets.push_back(GridCell{column_offset, row_offset});
      }
    }
  }

  const std::size_t occupied_count = static_cast<std::size_t>(std::count(
      source.cells().begin(), source.cells().end(), kOccupied));
  if (!offsets.empty() &&
    occupied_count > kMaximumInflationApplications / offsets.size())
  {
    throw std::invalid_argument("inflation work exceeds the online safety budget");
  }

  for (std::size_t source_row = 0; source_row < geometry_.height; ++source_row) {
    for (std::size_t source_column = 0; source_column < geometry_.width; ++source_column) {
      const GridCell obstacle{
        static_cast<int>(source_column), static_cast<int>(source_row)};
      if (source.at(obstacle) != kOccupied) {
        continue;
      }
      for (const GridCell & offset : offsets) {
        const GridCell candidate{
          obstacle.column + offset.column,
          obstacle.row + offset.row,
        };
        if (contains(candidate)) {
          blocked_[index(candidate)] = 1U;
        }
      }
    }
  }
}

const GridGeometry & InflatedGrid::geometry() const
{
  return geometry_;
}

bool InflatedGrid::contains(const GridCell & cell) const
{
  return cell.column >= 0 && cell.row >= 0 &&
         static_cast<std::size_t>(cell.column) < geometry_.width &&
         static_cast<std::size_t>(cell.row) < geometry_.height;
}

bool InflatedGrid::blocked(const GridCell & cell) const
{
  return !contains(cell) || blocked_.at(index(cell)) != 0U;
}

std::size_t InflatedGrid::index(const GridCell & cell) const
{
  if (!contains(cell)) {
    throw std::out_of_range("grid cell lies outside the inflated map");
  }
  return static_cast<std::size_t>(cell.row) * geometry_.width +
         static_cast<std::size_t>(cell.column);
}

}  // namespace drone_planner
