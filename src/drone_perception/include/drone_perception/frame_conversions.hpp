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
