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
#include "drone_planner/trajectory.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace drone_planner
{
namespace
{

constexpr double kTolerance = 1e-9;

GridMap free_map(const std::size_t width, const std::size_t height)
{
  return GridMap(
    GridGeometry{1.0, width, height, 0.0, 0.0},
    std::vector<std::int8_t>(width * height, kFree));
}

double magnitude(const Point2D & vector)
{
  return std::hypot(vector.x, vector.y);
}

TEST(TrajectoryTest, SupercoverRejectsObstacleTouchedOnlyAtCorner)
{
  GridMap map = free_map(3U, 3U);
  map.set(GridCell{1, 0}, kOccupied);
  const InflatedGrid grid(map, 0.0);

  EXPECT_FALSE(has_line_of_sight(grid, GridCell{0, 0}, GridCell{2, 2}));
}

TEST(TrajectoryTest, SupercoverAcceptsClearCardinalAndDiagonalSegments)
{
  const InflatedGrid grid(free_map(5U, 5U), 0.0);

  EXPECT_TRUE(has_line_of_sight(grid, GridCell{0, 2}, GridCell{4, 2}));
  EXPECT_TRUE(has_line_of_sight(grid, GridCell{0, 0}, GridCell{4, 4}));
  EXPECT_TRUE(has_line_of_sight(grid, GridCell{3, 3}, GridCell{3, 3}));
}

TEST(TrajectoryTest, SupercoverRejectsObstacleOnNonDiagonalSegment)
{
  GridMap map = free_map(5U, 3U);
  map.set(GridCell{2, 1}, kOccupied);
  const InflatedGrid grid(map, 0.0);

  EXPECT_FALSE(has_line_of_sight(grid, GridCell{0, 0}, GridCell{4, 2}));
}

TEST(TrajectoryTest, RemainingPathIgnoresBlockedCellsAlreadyPassed)
{
  GridMap map = free_map(6U, 1U);
  map.set(GridCell{1, 0}, kOccupied);
  const InflatedGrid grid(map, 0.0);
  const std::vector<GridCell> path{
    {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}};

  EXPECT_FALSE(remaining_path_is_safe(grid, path, GridCell{0, 0}));
  EXPECT_TRUE(remaining_path_is_safe(grid, path, GridCell{3, 0}));
}

TEST(TrajectoryTest, RemainingPathRejectsNewObstacleAhead)
{
  GridMap map = free_map(6U, 1U);
  map.set(GridCell{4, 0}, kOccupied);
  const InflatedGrid grid(map, 0.0);
  const std::vector<GridCell> path{
    {0, 0}, {1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}};

  EXPECT_FALSE(remaining_path_is_safe(grid, path, GridCell{2, 0}));
}

TEST(TrajectoryTest, PrunesOpenPathToEndpoints)
{
  const InflatedGrid grid(free_map(6U, 3U), 0.0);
  const std::vector<GridCell> path{
    {0, 1}, {1, 1}, {2, 1}, {3, 1}, {4, 1}, {5, 1}};

  const auto pruned = prune_path(grid, path);

  EXPECT_EQ(pruned, (std::vector<GridCell>{{0, 1}, {5, 1}}));
}

TEST(TrajectoryTest, PruningKeepsCollisionSafeDetourSegments)
{
  GridMap map = free_map(7U, 5U);
  map.set(GridCell{3, 2}, kOccupied);
  const InflatedGrid grid(map, 0.0);
  const std::vector<GridCell> path{
    {0, 2}, {1, 2}, {2, 2}, {2, 3},
    {3, 3}, {4, 3}, {4, 2}, {5, 2}, {6, 2}};

  const auto pruned = prune_path(grid, path);

  ASSERT_GT(pruned.size(), 2U);
  EXPECT_EQ(pruned.front(), path.front());
  EXPECT_EQ(pruned.back(), path.back());
  for (std::size_t index = 1; index < pruned.size(); ++index) {
    EXPECT_TRUE(has_line_of_sight(grid, pruned[index - 1U], pruned[index]));
  }
}

TEST(TrajectoryTest, RejectsInvalidOrUnsafeInputPath)
{
  GridMap map = free_map(3U, 3U);
  map.set(GridCell{1, 1}, kOccupied);
  const InflatedGrid grid(map, 0.0);

  EXPECT_THROW(prune_path(grid, {}), std::invalid_argument);
  EXPECT_THROW(
    prune_path(grid, std::vector<GridCell>{{0, 0}, {1, 1}, {2, 2}}),
    std::invalid_argument);
}

TEST(TrajectoryTest, SegmentDurationUsesAnalyticVelocityAndAccelerationBounds)
{
  const KinematicLimits limits{1.0, 2.0, 0.1};
  const double expected_velocity_duration = 1.875 * 2.0;

  EXPECT_NEAR(
    quintic_segment_duration(2.0, limits),
    expected_velocity_duration,
    kTolerance);
}

TEST(TrajectoryTest, QuinticSamplesPreserveEndpointsAndKinematicLimits)
{
  const KinematicLimits limits{0.8, 0.5, 0.03};

  const auto samples = parameterize_quintic(
    std::vector<Point2D>{{0.0, 0.0}, {2.0, 0.0}}, limits);

  ASSERT_GT(samples.size(), 2U);
  EXPECT_NEAR(samples.front().time_from_start_s, 0.0, kTolerance);
  EXPECT_NEAR(samples.front().position.x, 0.0, kTolerance);
  EXPECT_NEAR(samples.back().position.x, 2.0, kTolerance);
  EXPECT_NEAR(magnitude(samples.front().velocity), 0.0, kTolerance);
  EXPECT_NEAR(magnitude(samples.front().acceleration), 0.0, kTolerance);
  EXPECT_NEAR(magnitude(samples.back().velocity), 0.0, kTolerance);
  EXPECT_NEAR(magnitude(samples.back().acceleration), 0.0, kTolerance);
  for (std::size_t index = 0; index < samples.size(); ++index) {
    EXPECT_LE(magnitude(samples[index].velocity), limits.maximum_velocity_m_s + 1e-9);
    EXPECT_LE(
      magnitude(samples[index].acceleration),
      limits.maximum_acceleration_m_s2 + 1e-9);
    if (index > 0U) {
      EXPECT_GT(
        samples[index].time_from_start_s,
        samples[index - 1U].time_from_start_s);
    }
  }
}

TEST(TrajectoryTest, QuinticSamplesRespectTheAnalyticAccelerationPeak)
{
  constexpr double peak_normalized_time =
    (3.0 - 1.7320508075688772935) / 6.0;
  const KinematicLimits duration_limits{100.0, 1.0, 0.001};
  const double duration_s = quintic_segment_duration(1.0, duration_limits);
  const KinematicLimits sampled_limits{
    100.0, 1.0, peak_normalized_time * duration_s};

  const auto samples = parameterize_quintic(
    std::vector<Point2D>{{0.0, 0.0}, {1.0, 0.0}}, sampled_limits);

  ASSERT_GT(samples.size(), 2U);
  EXPECT_NEAR(magnitude(samples[1].acceleration), 1.0, 1e-9);
}

TEST(TrajectoryTest, PiecewiseTrajectoryDoesNotSpatiallyCutTheCorner)
{
  const KinematicLimits limits{1.0, 1.0, 0.05};
  const auto samples = parameterize_quintic(
    std::vector<Point2D>{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}}, limits);

  ASSERT_GT(samples.size(), 3U);
  const double first_duration = quintic_segment_duration(1.0, limits);
  const auto boundary = std::find_if(
    samples.begin(), samples.end(),
    [first_duration](const TrajectorySample2D & sample) {
      return std::abs(sample.time_from_start_s - first_duration) < 1e-9;
    });
  ASSERT_NE(boundary, samples.end());
  EXPECT_NEAR(magnitude(boundary->velocity), 0.0, kTolerance);
  EXPECT_NEAR(magnitude(boundary->acceleration), 0.0, kTolerance);
  for (const auto & sample : samples) {
    const bool on_first_segment = std::abs(sample.position.y) < 1e-9;
    const bool on_second_segment = std::abs(sample.position.x - 1.0) < 1e-9;
    EXPECT_TRUE(on_first_segment || on_second_segment);
  }
}

TEST(TrajectoryTest, HoldTrajectoryHasAStationaryEndpoint)
{
  const KinematicLimits limits{1.0, 1.0, 0.1};
  const auto samples = parameterize_hold(Point2D{1.5, -2.0}, limits);

  ASSERT_EQ(samples.size(), 2U);
  EXPECT_DOUBLE_EQ(samples.front().time_from_start_s, 0.0);
  EXPECT_DOUBLE_EQ(samples.back().time_from_start_s, limits.sample_period_s);
  for (const auto & sample : samples) {
    EXPECT_DOUBLE_EQ(sample.position.x, 1.5);
    EXPECT_DOUBLE_EQ(sample.position.y, -2.0);
    EXPECT_DOUBLE_EQ(magnitude(sample.velocity), 0.0);
    EXPECT_DOUBLE_EQ(magnitude(sample.acceleration), 0.0);
  }
}

TEST(TrajectoryTest, HoldTrajectoryRejectsInvalidInput)
{
  EXPECT_THROW(
    parameterize_hold(
      Point2D{std::numeric_limits<double>::quiet_NaN(), 0.0},
      KinematicLimits{1.0, 1.0, 0.1}),
    std::invalid_argument);
  EXPECT_THROW(
    parameterize_hold(Point2D{0.0, 0.0}, KinematicLimits{1.0, 1.0, 0.0}),
    std::invalid_argument);
}

TEST(TrajectoryTest, RejectsInvalidLimitsAndDegenerateWaypoints)
{
  EXPECT_THROW(
    parameterize_quintic(
      std::vector<Point2D>{{0.0, 0.0}, {1.0, 0.0}},
      KinematicLimits{0.0, 1.0, 0.1}),
    std::invalid_argument);
  EXPECT_THROW(
    parameterize_quintic(
      std::vector<Point2D>{{0.0, 0.0}, {0.0, 0.0}},
      KinematicLimits{1.0, 1.0, 0.1}),
    std::invalid_argument);
  EXPECT_THROW(
    parameterize_quintic(
      std::vector<Point2D>{{0.0, 0.0}},
      KinematicLimits{1.0, 1.0, 0.1}),
    std::invalid_argument);
  EXPECT_THROW(
    parameterize_quintic(
      std::vector<Point2D>{{0.0, 0.0}, {1.0, 0.0}},
      KinematicLimits{1.0, 1.0, 1e-12}),
    std::invalid_argument);
  EXPECT_THROW(
    quintic_segment_duration(
      std::numeric_limits<double>::max(),
      KinematicLimits{1.0, 1.0, 0.1}),
    std::invalid_argument);
  EXPECT_THROW(
    parameterize_quintic(
      std::vector<Point2D>{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}},
      KinematicLimits{1.0, 1.0, 1e308}),
    std::invalid_argument);
}

}  // namespace
}  // namespace drone_planner
