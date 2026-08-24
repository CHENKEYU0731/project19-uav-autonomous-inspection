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

#ifndef DRONE_PLANNER__A_STAR_HPP_
#define DRONE_PLANNER__A_STAR_HPP_

#include "drone_planner/grid_map.hpp"

#include <cstddef>
#include <optional>
#include <vector>

namespace drone_planner
{

struct AStarResult
{
  std::vector<GridCell> path;
  double cost_m{};
  std::size_t expanded_node_count{};
};

std::optional<AStarResult> plan_a_star(
  const InflatedGrid & grid,
  const GridCell & start,
  const GridCell & goal);

}  // namespace drone_planner

#endif  // DRONE_PLANNER__A_STAR_HPP_
