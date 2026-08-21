#ifndef DRONE_PERCEPTION__DEPTH_GRID_MAPPER_HPP_
#define DRONE_PERCEPTION__DEPTH_GRID_MAPPER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace drone_perception
{

struct CameraIntrinsics
{
  double fx{};
  double fy{};
  double cx{};
  double cy{};
};

struct MapperConfig
{
  double resolution_m{0.1};
  double width_m{12.0};
  double height_m{12.0};
  double min_depth_m{0.2};
  double max_depth_m{10.0};
  double min_relative_height_m{-0.5};
  double max_relative_height_m{0.5};
  int pixel_stride{2};
};

struct Pose3D
{
  double x{};
  double y{};
  double z{};
  double qx{};
  double qy{};
  double qz{};
  double qw{1.0};
};

struct GridData
{
  std::uint32_t width{};
  std::uint32_t height{};
  double resolution_m{};
  double origin_x{};
  double origin_y{};
  std::size_t used_depth_count{};
  std::size_t occupied_cell_count{};
  std::vector<std::int8_t> cells;
};

void validate_mapper_config(const MapperConfig & config);

GridData build_grid(
  const std::vector<float> & depth,
  std::uint32_t image_width,
  std::uint32_t image_height,
  const CameraIntrinsics & intrinsics,
  const Pose3D & camera_pose_in_map,
  const Pose3D & base_pose_in_map,
  const MapperConfig & config);

}  // namespace drone_perception

#endif  // DRONE_PERCEPTION__DEPTH_GRID_MAPPER_HPP_
