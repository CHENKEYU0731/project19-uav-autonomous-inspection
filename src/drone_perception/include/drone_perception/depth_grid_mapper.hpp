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
