#include "drone_perception/depth_grid_mapper.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace drone_perception
{
namespace
{

constexpr std::int8_t kUnknown = -1;
constexpr std::int8_t kFree = 0;
constexpr std::int8_t kOccupied = 100;
constexpr std::size_t kMaxGridCells = 10'000'000;

struct Point3D
{
  double x;
  double y;
  double z;
};

struct GridCell
{
  int column;
  int row;
};

bool finite(const double value)
{
  return std::isfinite(value);
}

void validate_pose(const Pose3D & pose)
{
  const std::array<double, 7> values{
    pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw};
  if (!std::all_of(values.begin(), values.end(), finite)) {
    throw std::invalid_argument("pose values must be finite");
  }

  const double quaternion_norm = std::sqrt(
    pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz + pose.qw * pose.qw);
  if (quaternion_norm <= std::numeric_limits<double>::epsilon()) {
    throw std::invalid_argument("pose quaternion norm must be positive");
  }
}

void validate_inputs(
  const std::vector<float> & depth,
  const std::uint32_t image_width,
  const std::uint32_t image_height,
  const CameraIntrinsics & intrinsics,
  const Pose3D & camera_pose,
  const Pose3D & base_pose,
  const MapperConfig & config)
{
  validate_mapper_config(config);
  if (!finite(intrinsics.fx) || !finite(intrinsics.fy) ||
    !finite(intrinsics.cx) || !finite(intrinsics.cy) ||
    intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0)
  {
    throw std::invalid_argument("camera intrinsics must be finite with positive focal lengths");
  }
  if (image_width == 0U || image_height == 0U ||
    image_height > std::numeric_limits<std::size_t>::max() / image_width ||
    depth.size() != static_cast<std::size_t>(image_width) * image_height)
  {
    throw std::invalid_argument("depth buffer dimensions do not match the image");
  }
  validate_pose(camera_pose);
  validate_pose(base_pose);
}

Point3D rotate_and_translate(const Point3D & point, const Pose3D & pose)
{
  const double norm = std::sqrt(
    pose.qx * pose.qx + pose.qy * pose.qy + pose.qz * pose.qz + pose.qw * pose.qw);
  const double qx = pose.qx / norm;
  const double qy = pose.qy / norm;
  const double qz = pose.qz / norm;
  const double qw = pose.qw / norm;

  const double tx = 2.0 * (qy * point.z - qz * point.y);
  const double ty = 2.0 * (qz * point.x - qx * point.z);
  const double tz = 2.0 * (qx * point.y - qy * point.x);

  return Point3D{
    pose.x + point.x + qw * tx + (qy * tz - qz * ty),
    pose.y + point.y + qw * ty + (qz * tx - qx * tz),
    pose.z + point.z + qw * tz + (qx * ty - qy * tx),
  };
}

std::optional<GridCell> world_to_cell(
  const GridData & grid, const double world_x, const double world_y)
{
  const double column_value = (world_x - grid.origin_x) / grid.resolution_m;
  const double row_value = (world_y - grid.origin_y) / grid.resolution_m;
  if (!finite(column_value) || !finite(row_value) ||
    column_value < 0.0 || row_value < 0.0 ||
    column_value >= static_cast<double>(grid.width) ||
    row_value >= static_cast<double>(grid.height))
  {
    return std::nullopt;
  }
  return GridCell{
    static_cast<int>(std::floor(column_value)),
    static_cast<int>(std::floor(row_value)),
  };
}

std::size_t cell_index(const GridData & grid, const int column, const int row)
{
  return static_cast<std::size_t>(row) * grid.width + static_cast<std::size_t>(column);
}

void trace_ray(GridData & grid, const GridCell start, const GridCell endpoint)
{
  int column = start.column;
  int row = start.row;
  const int delta_column = std::abs(endpoint.column - start.column);
  const int delta_row = std::abs(endpoint.row - start.row);
  const int step_column = start.column < endpoint.column ? 1 : -1;
  const int step_row = start.row < endpoint.row ? 1 : -1;
  int error = delta_column - delta_row;

  while (column != endpoint.column || row != endpoint.row) {
    auto & cell = grid.cells[cell_index(grid, column, row)];
    if (cell != kOccupied) {
      cell = kFree;
    }

    const int doubled_error = 2 * error;
    if (doubled_error > -delta_row) {
      error -= delta_row;
      column += step_column;
    }
    if (doubled_error < delta_column) {
      error += delta_column;
      row += step_row;
    }
  }

  auto & endpoint_cell = grid.cells[cell_index(grid, endpoint.column, endpoint.row)];
  if (endpoint_cell != kOccupied) {
    endpoint_cell = kOccupied;
    ++grid.occupied_cell_count;
  }
}

}  // namespace

void validate_mapper_config(const MapperConfig & config)
{
  if (!finite(config.resolution_m) || config.resolution_m <= 0.0 ||
    !finite(config.width_m) || config.width_m <= 0.0 ||
    !finite(config.height_m) || config.height_m <= 0.0)
  {
    throw std::invalid_argument("grid resolution and dimensions must be finite and positive");
  }
  if (!finite(config.min_depth_m) || !finite(config.max_depth_m) ||
    config.min_depth_m <= 0.0 || config.max_depth_m <= config.min_depth_m)
  {
    throw std::invalid_argument("depth range must be finite, positive, and increasing");
  }
  if (!finite(config.min_relative_height_m) || !finite(config.max_relative_height_m) ||
    config.max_relative_height_m < config.min_relative_height_m)
  {
    throw std::invalid_argument("height slice must be finite and increasing");
  }
  if (config.pixel_stride <= 0) {
    throw std::invalid_argument("pixel stride must be positive");
  }

  const double width_cells = std::ceil(config.width_m / config.resolution_m);
  const double height_cells = std::ceil(config.height_m / config.resolution_m);
  if (width_cells > static_cast<double>(std::numeric_limits<int>::max()) ||
    height_cells > static_cast<double>(std::numeric_limits<int>::max()) ||
    width_cells * height_cells > static_cast<double>(kMaxGridCells))
  {
    throw std::invalid_argument("grid dimensions exceed the local-map safety limit");
  }
}

GridData build_grid(
  const std::vector<float> & depth,
  const std::uint32_t image_width,
  const std::uint32_t image_height,
  const CameraIntrinsics & intrinsics,
  const Pose3D & camera_pose_in_map,
  const Pose3D & base_pose_in_map,
  const MapperConfig & config)
{
  validate_inputs(
    depth, image_width, image_height, intrinsics, camera_pose_in_map, base_pose_in_map, config);

  const double width_cells = std::ceil(config.width_m / config.resolution_m);
  const double height_cells = std::ceil(config.height_m / config.resolution_m);
  if (width_cells > static_cast<double>(std::numeric_limits<int>::max()) ||
    height_cells > static_cast<double>(std::numeric_limits<int>::max()))
  {
    throw std::invalid_argument("grid dimensions exceed supported cell counts");
  }

  GridData grid;
  grid.width = static_cast<std::uint32_t>(width_cells);
  grid.height = static_cast<std::uint32_t>(height_cells);
  grid.resolution_m = config.resolution_m;
  grid.origin_x = base_pose_in_map.x - static_cast<double>(grid.width) * grid.resolution_m / 2.0;
  grid.origin_y = base_pose_in_map.y - static_cast<double>(grid.height) * grid.resolution_m / 2.0;
  if (grid.height > std::numeric_limits<std::size_t>::max() / grid.width) {
    throw std::invalid_argument("grid cell count exceeds addressable memory");
  }
  const std::size_t grid_cell_count = static_cast<std::size_t>(grid.width) * grid.height;
  if (grid_cell_count > kMaxGridCells) {
    throw std::invalid_argument("grid cell count exceeds the local-map safety limit");
  }
  grid.cells.assign(grid_cell_count, kUnknown);

  const auto camera_cell = world_to_cell(grid, camera_pose_in_map.x, camera_pose_in_map.y);
  if (!camera_cell.has_value()) {
    throw std::invalid_argument("camera origin lies outside the rolling grid");
  }

  const auto stride = static_cast<std::uint32_t>(config.pixel_stride);
  for (std::uint32_t row = 0; row < image_height; row += stride) {
    for (std::uint32_t column = 0; column < image_width; column += stride) {
      const float range = depth[static_cast<std::size_t>(row) * image_width + column];
      if (!std::isfinite(range) || range < config.min_depth_m || range > config.max_depth_m) {
        continue;
      }

      const Point3D optical_point{
        (static_cast<double>(column) - intrinsics.cx) * range / intrinsics.fx,
        (static_cast<double>(row) - intrinsics.cy) * range / intrinsics.fy,
        range,
      };
      const Point3D endpoint = rotate_and_translate(optical_point, camera_pose_in_map);
      const double relative_height = endpoint.z - base_pose_in_map.z;
      if (relative_height < config.min_relative_height_m ||
        relative_height > config.max_relative_height_m)
      {
        continue;
      }

      const auto endpoint_cell = world_to_cell(grid, endpoint.x, endpoint.y);
      if (!endpoint_cell.has_value()) {
        continue;
      }
      ++grid.used_depth_count;
      trace_ray(grid, camera_cell.value(), endpoint_cell.value());
    }
  }

  return grid;
}

}  // namespace drone_perception
