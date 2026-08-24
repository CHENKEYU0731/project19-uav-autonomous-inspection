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
#include "drone_planner/grid_map.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace drone_planner
{
namespace
{

GridMap free_map(const std::size_t width, const std::size_t height)
{
  return GridMap(
    GridGeometry{1.0, width, height, 0.0, 0.0},
    std::vector<std::int8_t>(width * height, kFree));
}

TEST(AStarTest, FindsStraightCardinalPathWithMetricCost)
{
  const InflatedGrid grid(free_map(5U, 3U), 0.0);

  const auto result = plan_a_star(grid, GridCell{0, 1}, GridCell{4, 1});

  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->path.front(), (GridCell{0, 1}));
  ASSERT_EQ(result->path.back(), (GridCell{4, 1}));
  EXPECT_DOUBLE_EQ(result->cost_m, 4.0);
  EXPECT_GT(result->expanded_node_count, 0U);
}

TEST(AStarTest, FindsDetourAroundInflatedObstacle)
{
  GridMap map = free_map(9U, 7U);
  map.set(GridCell{4, 3}, kOccupied);
  const InflatedGrid grid(map, 1.0);

  const auto result = plan_a_star(grid, GridCell{0, 3}, GridCell{8, 3});

  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->path.front() == (GridCell{0, 3}));
  EXPECT_TRUE(result->path.back() == (GridCell{8, 3}));
  EXPECT_TRUE(
    std::any_of(
      result->path.begin(), result->path.end(),
      [](const GridCell & cell) {return cell.row != 3;}));
  for (const GridCell & cell : result->path) {
    EXPECT_FALSE(grid.blocked(cell));
  }
}

TEST(AStarTest, RejectsBlockedOrOutOfBoundsEndpoints)
{
  GridMap map = free_map(3U, 3U);
  map.set(GridCell{0, 0}, kOccupied);
  const InflatedGrid grid(map, 0.0);

  EXPECT_EQ(plan_a_star(grid, GridCell{0, 0}, GridCell{2, 2}), std::nullopt);
  EXPECT_EQ(plan_a_star(grid, GridCell{1, 1}, GridCell{3, 2}), std::nullopt);
}

TEST(AStarTest, RejectsUnreachableGoal)
{
  GridMap map = free_map(5U, 5U);
  for (int row = 0; row < 5; ++row) {
    map.set(GridCell{2, row}, kOccupied);
  }
  const InflatedGrid grid(map, 0.0);

  EXPECT_EQ(plan_a_star(grid, GridCell{0, 2}, GridCell{4, 2}), std::nullopt);
}

TEST(AStarTest, RejectsDiagonalCornerCutting)
{
  GridMap map = free_map(2U, 2U);
  map.set(GridCell{1, 0}, kOccupied);
  map.set(GridCell{0, 1}, kOccupied);
  const InflatedGrid grid(map, 0.0);

  EXPECT_EQ(plan_a_star(grid, GridCell{0, 0}, GridCell{1, 1}), std::nullopt);
}

TEST(AStarTest, RoutesAroundAOneSidedDiagonalCorner)
{
  GridMap map = free_map(2U, 2U);
  map.set(GridCell{1, 0}, kOccupied);
  const InflatedGrid grid(map, 0.0);

  const auto result = plan_a_star(grid, GridCell{0, 0}, GridCell{1, 1});

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(
    result->path,
    (std::vector<GridCell>{{0, 0}, {0, 1}, {1, 1}}));
}

TEST(AStarTest, StopsAtTheOnlineExpansionBudget)
{
  const InflatedGrid grid(free_map(100'001U, 1U), 0.0);

  EXPECT_EQ(
    plan_a_star(grid, GridCell{0, 0}, GridCell{100'000, 0}),
    std::nullopt);
}

TEST(AStarTest, ProducesDeterministicPathForEqualCostAlternatives)
{
  GridMap map = free_map(7U, 5U);
  map.set(GridCell{3, 2}, kOccupied);
  const InflatedGrid grid(map, 0.0);

  const auto first = plan_a_star(grid, GridCell{0, 2}, GridCell{6, 2});
  const auto second = plan_a_star(grid, GridCell{0, 2}, GridCell{6, 2});

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(first->path, second->path);
  EXPECT_DOUBLE_EQ(first->cost_m, second->cost_m);
  EXPECT_EQ(first->expanded_node_count, second->expanded_node_count);
}

}  // namespace
}  // namespace drone_planner
