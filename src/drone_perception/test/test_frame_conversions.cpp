#include "drone_perception/frame_conversions.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

namespace drone_perception
{
namespace
{

constexpr double kSqrtHalf = 0.7071067811865476;

void expect_same_rotation(const Pose3D & actual, const std::array<double, 4> & expected_xyzw)
{
  const double dot =
    actual.qx * expected_xyzw[0] + actual.qy * expected_xyzw[1] +
    actual.qz * expected_xyzw[2] + actual.qw * expected_xyzw[3];
  EXPECT_NEAR(std::abs(dot), 1.0, 1e-6);
}

TEST(FrameConversionsTest, MapsNedPositionAndNorthFacingAttitudeToEnuFlu)
{
  const auto pose = ned_frd_to_enu_flu(
    std::array<float, 3>{1.0F, 2.0F, -3.0F},
    std::array<float, 4>{1.0F, 0.0F, 0.0F, 0.0F},
    kPoseFrameNed);

  ASSERT_TRUE(pose.has_value());
  EXPECT_DOUBLE_EQ(pose->x, 2.0);
  EXPECT_DOUBLE_EQ(pose->y, 1.0);
  EXPECT_DOUBLE_EQ(pose->z, 3.0);
  expect_same_rotation(pose.value(), {0.0, 0.0, kSqrtHalf, kSqrtHalf});
}

TEST(FrameConversionsTest, MapsEastFacingNedAttitudeToZeroEnuYaw)
{
  const auto pose = ned_frd_to_enu_flu(
    std::array<float, 3>{0.0F, 0.0F, 0.0F},
    std::array<float, 4>{
      static_cast<float>(kSqrtHalf), 0.0F, 0.0F, static_cast<float>(kSqrtHalf)},
    kPoseFrameNed);

  ASSERT_TRUE(pose.has_value());
  expect_same_rotation(pose.value(), {0.0, 0.0, 0.0, 1.0});
}

TEST(FrameConversionsTest, RejectsUnsupportedFrameAndNonFiniteInput)
{
  EXPECT_FALSE(
    ned_frd_to_enu_flu(
      std::array<float, 3>{0.0F, 0.0F, 0.0F},
      std::array<float, 4>{1.0F, 0.0F, 0.0F, 0.0F},
      2).has_value());

  EXPECT_FALSE(
    ned_frd_to_enu_flu(
      std::array<float, 3>{std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F},
      std::array<float, 4>{1.0F, 0.0F, 0.0F, 0.0F},
      kPoseFrameNed).has_value());

  EXPECT_FALSE(
    ned_frd_to_enu_flu(
      std::array<float, 3>{0.0F, 0.0F, 0.0F},
      std::array<float, 4>{0.0F, 0.0F, 0.0F, 0.0F},
      kPoseFrameNed).has_value());
}

TEST(FrameConversionsTest, NormalizesInputQuaternion)
{
  const auto unit = ned_frd_to_enu_flu(
    std::array<float, 3>{0.0F, 0.0F, 0.0F},
    std::array<float, 4>{1.0F, 0.0F, 0.0F, 0.0F},
    kPoseFrameNed);
  const auto scaled = ned_frd_to_enu_flu(
    std::array<float, 3>{0.0F, 0.0F, 0.0F},
    std::array<float, 4>{2.0F, 0.0F, 0.0F, 0.0F},
    kPoseFrameNed);

  ASSERT_TRUE(unit.has_value());
  ASSERT_TRUE(scaled.has_value());
  expect_same_rotation(
    scaled.value(), {unit->qx, unit->qy, unit->qz, unit->qw});
}

}  // namespace
}  // namespace drone_perception
