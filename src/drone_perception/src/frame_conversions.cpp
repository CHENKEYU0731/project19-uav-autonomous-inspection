#include "drone_perception/frame_conversions.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace drone_perception
{
namespace
{

using Matrix3 = std::array<std::array<double, 3>, 3>;

Matrix3 multiply(const Matrix3 & left, const Matrix3 & right)
{
  Matrix3 result{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      for (std::size_t inner = 0; inner < 3; ++inner) {
        result[row][column] += left[row][inner] * right[inner][column];
      }
    }
  }
  return result;
}

Matrix3 quaternion_to_matrix(const std::array<double, 4> & q_wxyz)
{
  const double w = q_wxyz[0];
  const double x = q_wxyz[1];
  const double y = q_wxyz[2];
  const double z = q_wxyz[3];
  return Matrix3{{
    {{1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)}},
    {{2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)}},
    {{2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)}},
  }};
}

std::array<double, 4> matrix_to_quaternion_xyzw(const Matrix3 & matrix)
{
  double x;
  double y;
  double z;
  double w;
  const double trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
  if (trace > 0.0) {
    const double scale = 2.0 * std::sqrt(trace + 1.0);
    w = 0.25 * scale;
    x = (matrix[2][1] - matrix[1][2]) / scale;
    y = (matrix[0][2] - matrix[2][0]) / scale;
    z = (matrix[1][0] - matrix[0][1]) / scale;
  } else if (matrix[0][0] > matrix[1][1] && matrix[0][0] > matrix[2][2]) {
    const double scale = 2.0 * std::sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]);
    w = (matrix[2][1] - matrix[1][2]) / scale;
    x = 0.25 * scale;
    y = (matrix[0][1] + matrix[1][0]) / scale;
    z = (matrix[0][2] + matrix[2][0]) / scale;
  } else if (matrix[1][1] > matrix[2][2]) {
    const double scale = 2.0 * std::sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]);
    w = (matrix[0][2] - matrix[2][0]) / scale;
    x = (matrix[0][1] + matrix[1][0]) / scale;
    y = 0.25 * scale;
    z = (matrix[1][2] + matrix[2][1]) / scale;
  } else {
    const double scale = 2.0 * std::sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]);
    w = (matrix[1][0] - matrix[0][1]) / scale;
    x = (matrix[0][2] + matrix[2][0]) / scale;
    y = (matrix[1][2] + matrix[2][1]) / scale;
    z = 0.25 * scale;
  }

  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;
  if (w < 0.0) {
    x = -x;
    y = -y;
    z = -z;
    w = -w;
  }
  return {x, y, z, w};
}

}  // namespace

std::optional<Pose3D> ned_frd_to_enu_flu(
  const std::array<float, 3> & position_ned,
  const std::array<float, 4> & q_ned_frd,
  const std::uint8_t pose_frame)
{
  if (pose_frame != kPoseFrameNed) {
    return std::nullopt;
  }
  if (!std::all_of(
      position_ned.begin(), position_ned.end(),
      [](const float value) {return std::isfinite(value);}) ||
    !std::all_of(
      q_ned_frd.begin(), q_ned_frd.end(),
      [](const float value) {return std::isfinite(value);}))
  {
    return std::nullopt;
  }

  const double quaternion_norm = std::sqrt(
    static_cast<double>(q_ned_frd[0]) * q_ned_frd[0] +
    static_cast<double>(q_ned_frd[1]) * q_ned_frd[1] +
    static_cast<double>(q_ned_frd[2]) * q_ned_frd[2] +
    static_cast<double>(q_ned_frd[3]) * q_ned_frd[3]);
  if (!std::isfinite(quaternion_norm) ||
    quaternion_norm <= std::numeric_limits<double>::epsilon())
  {
    return std::nullopt;
  }
  const std::array<double, 4> normalized_q{
    q_ned_frd[0] / quaternion_norm,
    q_ned_frd[1] / quaternion_norm,
    q_ned_frd[2] / quaternion_norm,
    q_ned_frd[3] / quaternion_norm,
  };

  const Matrix3 enu_from_ned{{
    {{0.0, 1.0, 0.0}},
    {{1.0, 0.0, 0.0}},
    {{0.0, 0.0, -1.0}},
  }};
  const Matrix3 frd_from_flu{{
    {{1.0, 0.0, 0.0}},
    {{0.0, -1.0, 0.0}},
    {{0.0, 0.0, -1.0}},
  }};
  const Matrix3 enu_from_flu = multiply(
    multiply(enu_from_ned, quaternion_to_matrix(normalized_q)), frd_from_flu);
  const auto q_enu_flu = matrix_to_quaternion_xyzw(enu_from_flu);

  return Pose3D{
    position_ned[1],
    position_ned[0],
    -position_ned[2],
    q_enu_flu[0],
    q_enu_flu[1],
    q_enu_flu[2],
    q_enu_flu[3],
  };
}

}  // namespace drone_perception
