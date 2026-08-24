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

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace drone_planner
{
namespace
{

GridGeometry geometry(
  const std::size_t width = 5U,
  const std::size_t height = 5U,
  const double resolution_m = 1.0,
  const double origin_x = 0.0,
  const double origin_y = 0.0)
{
  return GridGeometry{resolution_m, width, height, origin_x, origin_y};
}

TEST(GridMapTest, RejectsInvalidGeometryAndCellValues)
{
  EXPECT_THROW(GridMap(GridGeometry{}), std::invalid_argument);
  EXPECT_THROW(
    GridMap(
      GridGeometry{
        std::numeric_limits<double>::quiet_NaN(), 2U, 2U, 0.0, 0.0}),
    std::invalid_argument);
  EXPECT_THROW(
    GridMap(
      GridGeometry{
        std::numeric_limits<double>::max(), 2U, 2U, 0.0, 0.0}),
    std::invalid_argument);
  EXPECT_THROW(
    GridMap(
      GridGeometry{
        std::numeric_limits<double>::max(), 1U, 1U, 0.0, 0.0}),
    std::invalid_argument);
  EXPECT_THROW(
    GridMap(
      GridGeometry{
        1.0, 1U, 1U, std::numeric_limits<double>::max(), 0.0}),
    std::invalid_argument);
  EXPECT_THROW(GridMap(geometry(2U, 2U), {kFree}), std::invalid_argument);
  EXPECT_THROW(
    GridMap(geometry(2U, 2U), {kFree, kUnknown, kOccupied, 42}),
    std::invalid_argument);
}

TEST(GridMapTest, ConvertsWorldCoordinatesAtCellCentersAndBoundaries)
{
  const GridMap map(geometry(3U, 2U, 0.5, -1.0, 2.0));

  EXPECT_EQ(map.world_to_cell(Point2D{-0.75, 2.25}), (GridCell{0, 0}));
  EXPECT_EQ(map.world_to_cell(Point2D{0.49, 2.99}), (GridCell{2, 1}));
  EXPECT_EQ(map.world_to_cell(Point2D{-1.01, 2.25}), std::nullopt);
  EXPECT_EQ(map.world_to_cell(Point2D{0.5, 2.25}), std::nullopt);
  const Point2D center = map.cell_center(GridCell{2, 1});
  EXPECT_DOUBLE_EQ(center.x, 0.25);
  EXPECT_DOUBLE_EQ(center.y, 2.75);
}

TEST(GridMapTest, FusionPreservesUnknownAndOverwritesWithNewKnownEvidence)
{
  GridMap fused(geometry(3U, 1U), {kFree, kOccupied, kUnknown});
  const GridMap observation(
    geometry(3U, 1U), {kUnknown, kFree, kOccupied});

  fused.integrate(observation);

  EXPECT_EQ(fused.cells(), (std::vector<std::int8_t>{kFree, kFree, kOccupied}));
}

TEST(GridMapTest, FusionMapsRollingObservationByWorldCellCenter)
{
  GridMap fused(geometry(4U, 1U), {kUnknown, kUnknown, kUnknown, kUnknown});
  const GridMap observation(
    geometry(2U, 1U, 1.0, 1.0), {kFree, kOccupied});

  fused.integrate(observation);

  EXPECT_EQ(
    fused.cells(),
    (std::vector<std::int8_t>{kUnknown, kFree, kOccupied, kUnknown}));
}

TEST(GridMapTest, FusionRejectsResolutionMismatch)
{
  GridMap fused(geometry());
  const GridMap observation(geometry(5U, 5U, 0.5));

  EXPECT_THROW(fused.integrate(observation), std::invalid_argument);

  GridMap microscopic_fused(geometry(2U, 1U, 1e-12));
  const GridMap microscopic_observation(geometry(2U, 1U, 2e-12));
  EXPECT_THROW(
    microscopic_fused.integrate(microscopic_observation),
    std::invalid_argument);
}

TEST(GridMapTest, FusionMapsOccupiedEvidenceToEveryOverlappingCell)
{
  GridMap fused(geometry(2U, 1U), {kFree, kFree});
  const GridMap observation(geometry(1U, 1U, 1.0, 0.5), {kOccupied});

  fused.integrate(observation);

  EXPECT_EQ(fused.cells(), (std::vector<std::int8_t>{kOccupied, kOccupied}));
}

TEST(GridMapTest, FusionClearsOnlyCellsCompletelyCoveredByFreeEvidence)
{
  GridMap partially_covered(geometry(2U, 1U), {kOccupied, kOccupied});
  const GridMap partial_observation(geometry(1U, 1U, 1.0, 0.5), {kFree});
  partially_covered.integrate(partial_observation);
  EXPECT_EQ(
    partially_covered.cells(),
    (std::vector<std::int8_t>{kOccupied, kOccupied}));

  GridMap completely_covered(
    geometry(3U, 1U), {kOccupied, kOccupied, kOccupied});
  const GridMap complete_observation(
    geometry(2U, 1U, 1.0, 0.5), {kFree, kFree});
  completely_covered.integrate(complete_observation);
  EXPECT_EQ(
    completely_covered.cells(),
    (std::vector<std::int8_t>{kOccupied, kFree, kOccupied}));
}

TEST(GridMapTest, FootprintClearingIsBoundedToRequestedDisk)
{
  GridMap map(geometry(), std::vector<std::int8_t>(25U, kOccupied));

  map.clear_disk(Point2D{2.5, 2.5}, 0.8);

  EXPECT_EQ(map.at(GridCell{2, 2}), kFree);
  EXPECT_EQ(map.at(GridCell{0, 0}), kOccupied);
}

TEST(GridMapTest, FootprintClearingPreservesPartiallyIntersectedCells)
{
  GridMap map(
    geometry(3U, 1U), std::vector<std::int8_t>(3U, kOccupied));

  map.clear_disk(Point2D{0.5, 0.5}, 0.75);

  EXPECT_EQ(map.at(GridCell{0, 0}), kFree);
  EXPECT_EQ(map.at(GridCell{1, 0}), kOccupied);
}

TEST(GridMapTest, FootprintClearingPreservesCellsOutsideTheExactDisk)
{
  GridMap map(
    geometry(4U, 3U), std::vector<std::int8_t>(12U, kOccupied));

  map.clear_disk(Point2D{0.5, 0.5}, 1.55);

  EXPECT_EQ(map.at(GridCell{0, 0}), kFree);
  EXPECT_EQ(map.at(GridCell{2, 1}), kOccupied);
}

TEST(GridMapTest, RejectsDiskRadiiLargerThanTheMapDiagonal)
{
  GridMap map(geometry());

  EXPECT_THROW(map.clear_disk(Point2D{2.5, 2.5}, 100.0), std::invalid_argument);
  EXPECT_THROW(InflatedGrid(map, 100.0), std::invalid_argument);
}

TEST(GridMapTest, InflationBlocksUnknownWithoutExpandingIt)
{
  std::vector<std::int8_t> cells(15U, kUnknown);
  for (std::size_t column = 0; column < 5U; ++column) {
    cells[5U + column] = kFree;
  }
  const GridMap map(geometry(5U, 3U), cells);
  const InflatedGrid inflated(map, 1.0);

  for (int column = 0; column < 5; ++column) {
    EXPECT_FALSE(inflated.blocked(GridCell{column, 1}));
    EXPECT_TRUE(inflated.blocked(GridCell{column, 0}));
    EXPECT_TRUE(inflated.blocked(GridCell{column, 2}));
  }
}

TEST(GridMapTest, InflationExpandsOnlyMeasuredOccupiedCellsConservatively)
{
  std::vector<std::int8_t> cells(25U, kFree);
  cells[12U] = kOccupied;
  const InflatedGrid inflated(GridMap(geometry(), cells), 1.0);

  EXPECT_TRUE(inflated.blocked(GridCell{2, 2}));
  EXPECT_TRUE(inflated.blocked(GridCell{1, 1}));
  EXPECT_TRUE(inflated.blocked(GridCell{3, 3}));
  EXPECT_FALSE(inflated.blocked(GridCell{0, 0}));
}

TEST(GridMapTest, InflationRejectsWorkAboveTheOnlineSafetyBudget)
{
  const GridMap map(
    geometry(100U, 100U),
    std::vector<std::int8_t>(10'000U, kOccupied));

  EXPECT_THROW(InflatedGrid(map, 20.0), std::invalid_argument);
}

}  // namespace
}  // namespace drone_planner
