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

#include "drone_planner/a_star.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

namespace drone_planner
{
namespace
{

constexpr double kSqrtTwo = 1.4142135623730950488;
constexpr std::size_t kMaximumAStarCellCount = 262'144U;
constexpr std::size_t kMaximumExpandedNodeCount = 100'000U;
constexpr std::size_t kMaximumQueuedNodeCount = 500'000U;

struct Move
{
  int column_offset;
  int row_offset;
  double cost_cells;
};

constexpr std::array<Move, 8> kMoves{{
  {-1, -1, kSqrtTwo},
  {0, -1, 1.0},
  {1, -1, kSqrtTwo},
  {-1, 0, 1.0},
  {1, 0, 1.0},
  {-1, 1, kSqrtTwo},
  {0, 1, 1.0},
  {1, 1, kSqrtTwo},
}};

double octile_distance(const GridCell & first, const GridCell & second)
{
  const int delta_column = std::abs(first.column - second.column);
  const int delta_row = std::abs(first.row - second.row);
  return static_cast<double>(delta_column + delta_row) +
         (kSqrtTwo - 2.0) * static_cast<double>(std::min(delta_column, delta_row));
}

std::size_t cell_index(const GridGeometry & geometry, const GridCell & cell)
{
  return static_cast<std::size_t>(cell.row) * geometry.width +
         static_cast<std::size_t>(cell.column);
}

GridCell cell_from_index(const GridGeometry & geometry, const std::size_t index)
{
  return GridCell{
    static_cast<int>(index % geometry.width),
    static_cast<int>(index / geometry.width),
  };
}

}  // namespace

std::optional<AStarResult> plan_a_star(
  const InflatedGrid & grid,
  const GridCell & start,
  const GridCell & goal)
{
  if (!grid.contains(start) || !grid.contains(goal) ||
    grid.blocked(start) || grid.blocked(goal))
  {
    return std::nullopt;
  }

  const GridGeometry & geometry = grid.geometry();
  const std::size_t cell_count = geometry.width * geometry.height;
  if (cell_count > kMaximumAStarCellCount) {
    return std::nullopt;
  }
  const std::size_t no_parent = std::numeric_limits<std::size_t>::max();
  const std::size_t start_index = cell_index(geometry, start);
  const std::size_t goal_index = cell_index(geometry, goal);
  std::vector<double> best_cost(cell_count, std::numeric_limits<double>::infinity());
  std::vector<std::size_t> parent(cell_count, no_parent);
  std::vector<std::uint8_t> closed(cell_count, 0U);

  using QueueEntry = std::tuple<double, double, double, int, int>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
  const double start_heuristic = octile_distance(start, goal) * geometry.resolution_m;
  best_cost[start_index] = 0.0;
  queue.emplace(start_heuristic, start_heuristic, 0.0, start.row, start.column);
  std::size_t expanded_node_count = 0U;
  std::size_t queued_node_count = 1U;

  while (!queue.empty()) {
    const auto [_priority, _heuristic, cost, row, column] = queue.top();
    queue.pop();
    const GridCell current{column, row};
    const std::size_t current_index = cell_index(geometry, current);
    if (closed[current_index] != 0U || cost > best_cost[current_index] + 1e-12) {
      continue;
    }
    if (expanded_node_count >= kMaximumExpandedNodeCount) {
      return std::nullopt;
    }
    closed[current_index] = 1U;
    ++expanded_node_count;
    if (current == goal) {
      std::vector<GridCell> path;
      for (std::size_t index = goal_index; ; index = parent[index]) {
        path.push_back(cell_from_index(geometry, index));
        if (index == start_index) {
          break;
        }
        if (parent[index] == no_parent) {
          return std::nullopt;
        }
      }
      std::reverse(path.begin(), path.end());
      return AStarResult{path, cost, expanded_node_count};
    }

    for (const Move & move : kMoves) {
      const GridCell next{
        current.column + move.column_offset,
        current.row + move.row_offset,
      };
      if (!grid.contains(next) || grid.blocked(next)) {
        continue;
      }
      if (move.column_offset != 0 && move.row_offset != 0 &&
        (grid.blocked(GridCell{current.column + move.column_offset, current.row}) ||
        grid.blocked(GridCell{current.column, current.row + move.row_offset})))
      {
        continue;
      }
      const std::size_t next_index = cell_index(geometry, next);
      if (closed[next_index] != 0U) {
        continue;
      }
      const double candidate_cost =
        cost + move.cost_cells * geometry.resolution_m;
      if (candidate_cost + 1e-12 >= best_cost[next_index]) {
        continue;
      }
      best_cost[next_index] = candidate_cost;
      parent[next_index] = current_index;
      const double heuristic = octile_distance(next, goal) * geometry.resolution_m;
      if (queued_node_count >= kMaximumQueuedNodeCount) {
        return std::nullopt;
      }
      queue.emplace(
        candidate_cost + heuristic,
        heuristic,
        candidate_cost,
        next.row,
        next.column);
      ++queued_node_count;
    }
  }
  return std::nullopt;
}

}  // namespace drone_planner
