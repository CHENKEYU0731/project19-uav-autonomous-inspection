#include "drone_perception/depth_grid_mapper.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace drone_perception
{
namespace
{

MapperConfig test_config()
{
  MapperConfig config;
  config.resolution_m = 1.0;
  config.width_m = 8.0;
  config.height_m = 8.0;
  config.min_depth_m = 0.2;
  config.max_depth_m = 6.0;
  config.min_relative_height_m = -0.5;
  config.max_relative_height_m = 0.5;
  config.pixel_stride = 1;
  return config;
}

CameraIntrinsics test_intrinsics()
{
  return CameraIntrinsics{1.0, 1.0, 1.0, 1.0};
}

Pose3D optical_pose(const double x = 0.0)
{
  return Pose3D{x, 0.0, 0.0, -0.5, 0.5, -0.5, 0.5};
}

Pose3D identity_pose(const double x = 0.0)
{
  return Pose3D{x, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
}

std::vector<float> one_center_depth(const float value)
{
  std::vector<float> depth(9, std::numeric_limits<float>::quiet_NaN());
  depth[4] = value;
  return depth;
}

std::int8_t cell_at_world(const GridData & grid, const double x, const double y)
{
  const auto column = static_cast<std::uint32_t>(
    std::floor((x - grid.origin_x) / grid.resolution_m));
  const auto row = static_cast<std::uint32_t>(
    std::floor((y - grid.origin_y) / grid.resolution_m));
  return grid.cells.at(static_cast<std::size_t>(row) * grid.width + column);
}

double occupied_cell_center_x(const GridData & grid)
{
  for (std::uint32_t row = 0; row < grid.height; ++row) {
    for (std::uint32_t column = 0; column < grid.width; ++column) {
      if (grid.cells[static_cast<std::size_t>(row) * grid.width + column] == 100) {
        return grid.origin_x + (static_cast<double>(column) + 0.5) * grid.resolution_m;
      }
    }
  }
  throw std::runtime_error("grid has no occupied cell");
}

TEST(DepthGridMapperTest, ProjectsOpticalAxisToBaseForward)
{
  const auto grid = build_grid(
    one_center_depth(2.0F), 3, 3, test_intrinsics(), optical_pose(), identity_pose(), test_config());

  EXPECT_EQ(cell_at_world(grid, 2.0, 0.0), 100);
  EXPECT_EQ(grid.used_depth_count, 1U);
  EXPECT_EQ(grid.occupied_cell_count, 1U);
}

TEST(DepthGridMapperTest, MarksRayFreeAndEndpointOccupied)
{
  const auto grid = build_grid(
    one_center_depth(2.0F), 3, 3, test_intrinsics(), optical_pose(), identity_pose(), test_config());

  EXPECT_EQ(cell_at_world(grid, 0.0, 0.0), 0);
  EXPECT_EQ(cell_at_world(grid, 1.0, 0.0), 0);
  EXPECT_EQ(cell_at_world(grid, 2.0, 0.0), 100);
}

TEST(DepthGridMapperTest, RejectsInvalidConfigurationIntrinsicsAndDimensions)
{
  auto config = test_config();
  config.resolution_m = 0.0;
  EXPECT_THROW(
    build_grid(
      one_center_depth(2.0F), 3, 3, test_intrinsics(), optical_pose(), identity_pose(), config),
    std::invalid_argument);

  auto intrinsics = test_intrinsics();
  intrinsics.fx = 0.0;
  EXPECT_THROW(
    build_grid(
      one_center_depth(2.0F), 3, 3, intrinsics, optical_pose(), identity_pose(), test_config()),
    std::invalid_argument);

  EXPECT_THROW(
    build_grid(
      std::vector<float>(8, 2.0F), 3, 3, test_intrinsics(), optical_pose(), identity_pose(),
      test_config()),
    std::invalid_argument);

  config = test_config();
  config.width_m = 10000000.0;
  EXPECT_THROW(
    build_grid(
      one_center_depth(2.0F), 3, 3, test_intrinsics(), optical_pose(), identity_pose(), config),
    std::invalid_argument);
}

TEST(DepthGridMapperTest, IgnoresNonFiniteAndOutOfRangeDepth)
{
  auto depth = std::vector<float>{
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::infinity(),
    0.1F,
    7.0F,
  };
  const auto intrinsics = CameraIntrinsics{1.0, 1.0, 0.5, 0.5};

  const auto grid = build_grid(
    depth, 2, 2, intrinsics, optical_pose(), identity_pose(), test_config());

  EXPECT_EQ(grid.used_depth_count, 0U);
  EXPECT_EQ(grid.occupied_cell_count, 0U);
}

TEST(DepthGridMapperTest, FiltersPointsOutsideRelativeHeightSlice)
{
  auto depth = one_center_depth(2.0F);
  depth[1] = 2.0F;
  depth[7] = 2.0F;

  const auto grid = build_grid(
    depth, 3, 3, test_intrinsics(), optical_pose(), identity_pose(), test_config());

  EXPECT_EQ(grid.used_depth_count, 1U);
  EXPECT_EQ(grid.occupied_cell_count, 1U);
}

TEST(DepthGridMapperTest, MovesOriginWithoutMovingWorldObstacle)
{
  const auto first = build_grid(
    one_center_depth(3.0F), 3, 3, test_intrinsics(), optical_pose(), identity_pose(), test_config());
  const auto second = build_grid(
    one_center_depth(2.0F), 3, 3, test_intrinsics(), optical_pose(1.0), identity_pose(1.0),
    test_config());

  EXPECT_DOUBLE_EQ(second.origin_x - first.origin_x, 1.0);
  EXPECT_NEAR(occupied_cell_center_x(first), occupied_cell_center_x(second), 1e-9);
}

}  // namespace
}  // namespace drone_perception
