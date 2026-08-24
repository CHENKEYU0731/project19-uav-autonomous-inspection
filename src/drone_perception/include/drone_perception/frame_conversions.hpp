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

#ifndef DRONE_PERCEPTION__FRAME_CONVERSIONS_HPP_
#define DRONE_PERCEPTION__FRAME_CONVERSIONS_HPP_

#include "drone_perception/depth_grid_mapper.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace drone_perception
{

constexpr std::uint8_t kPoseFrameNed = 1;

class Px4TimestampAligner
{
public:
  std::optional<std::int64_t> align(
    std::uint64_t timestamp_sample_us, std::int64_t arrival_ros_ns);

private:
  std::optional<std::int64_t> offset_ns_;
  std::optional<std::int64_t> pending_offset_ns_;
  std::uint64_t previous_sample_us_{};
  std::int64_t previous_arrival_ros_ns_{};
};

std::optional<Pose3D> ned_frd_to_enu_flu(
  const std::array<float, 3> & position_ned,
  const std::array<float, 4> & q_ned_frd,
  std::uint8_t pose_frame);

}  // namespace drone_perception

#endif  // DRONE_PERCEPTION__FRAME_CONVERSIONS_HPP_
