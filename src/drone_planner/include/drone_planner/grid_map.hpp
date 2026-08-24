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

#ifndef DRONE_PLANNER__GRID_MAP_HPP_
#define DRONE_PLANNER__GRID_MAP_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace drone_planner
{

constexpr std::int8_t kUnknown = -1;
constexpr std::int8_t kFree = 0;
constexpr std::int8_t kOccupied = 100;

struct Point2D
{
  double x{};
  double y{};
};

struct GridCell
{
  int column{};
  int row{};

  bool operator==(const GridCell & other) const;
  bool operator!=(const GridCell & other) const;
};

struct GridGeometry
{
  double resolution_m{};
  std::size_t width{};
  std::size_t height{};
  double origin_x{};
  double origin_y{};
};

class GridMap
{
public:
  explicit GridMap(GridGeometry geometry);
  GridMap(GridGeometry geometry, std::vector<std::int8_t> cells);

  const GridGeometry & geometry() const;
  const std::vector<std::int8_t> & cells() const;
  std::int8_t at(const GridCell & cell) const;
  void set(const GridCell & cell, std::int8_t value);
  bool contains(const GridCell & cell) const;
  std::optional<GridCell> world_to_cell(const Point2D & point) const;
  Point2D cell_center(const GridCell & cell) const;
  void integrate(const GridMap & observation);
  void clear_disk(const Point2D & center, double radius_m);

private:
  std::size_t index(const GridCell & cell) const;

  GridGeometry geometry_;
  std::vector<std::int8_t> cells_;
};

class InflatedGrid
{
public:
  InflatedGrid(const GridMap & source, double inflation_radius_m);

  const GridGeometry & geometry() const;
  bool contains(const GridCell & cell) const;
  bool blocked(const GridCell & cell) const;

private:
  std::size_t index(const GridCell & cell) const;

  GridGeometry geometry_;
  std::vector<std::uint8_t> blocked_;
};

}  // namespace drone_planner

#endif  // DRONE_PLANNER__GRID_MAP_HPP_
