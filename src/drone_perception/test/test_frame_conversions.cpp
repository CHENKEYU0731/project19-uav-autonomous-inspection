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

TEST(Px4TimestampAlignerTest, PreservesSampleTimeAcrossVariableDeliveryDelay)
{
  Px4TimestampAligner aligner;
  const auto first = aligner.align(1'000'000U, 50'000'000'000LL);
  const auto delayed = aligner.align(1'100'000U, 50'300'000'000LL);

  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(delayed.has_value());
  EXPECT_EQ(first.value(), 50'000'000'000LL);
  EXPECT_EQ(delayed.value(), 50'100'000'000LL);
}

TEST(Px4TimestampAlignerTest, RejectsInvalidRosTimeWithoutInitializingOffset)
{
  Px4TimestampAligner aligner;

  EXPECT_FALSE(aligner.align(1'000'000U, 0).has_value());
  const auto first_valid = aligner.align(1'100'000U, 50'100'000'000LL);

  ASSERT_TRUE(first_valid.has_value());
  EXPECT_EQ(first_valid.value(), 50'100'000'000LL);
}

TEST(Px4TimestampAlignerTest, RelocksAfterConfirmedForwardAndImmediateBackwardClockJumps)
{
  Px4TimestampAligner aligner;
  ASSERT_TRUE(aligner.align(1'000'000U, 50'000'000'000LL).has_value());

  EXPECT_FALSE(aligner.align(1'100'000U, 51'100'000'000LL).has_value());
  const auto after_one_second_jump = aligner.align(1'200'000U, 51'200'000'000LL);
  ASSERT_TRUE(after_one_second_jump.has_value());
  EXPECT_EQ(after_one_second_jump.value(), 51'200'000'000LL);

  EXPECT_FALSE(aligner.align(1'300'000U, 56'300'000'000LL).has_value());
  const auto after_five_second_jump = aligner.align(1'400'000U, 56'400'000'000LL);
  ASSERT_TRUE(after_five_second_jump.has_value());
  EXPECT_EQ(after_five_second_jump.value(), 56'400'000'000LL);

  const auto after_backward_jump = aligner.align(1'500'000U, 40'500'000'000LL);
  ASSERT_TRUE(after_backward_jump.has_value());
  EXPECT_EQ(after_backward_jump.value(), 40'500'000'000LL);
}

TEST(Px4TimestampAlignerTest, RejectsTimestampAdditionOverflow)
{
  Px4TimestampAligner aligner;
  const auto maximum = std::numeric_limits<std::int64_t>::max();
  ASSERT_TRUE(aligner.align(1U, maximum - 1'000).has_value());

  const auto overflowed = aligner.align(3U, maximum - 800);

  EXPECT_FALSE(overflowed.has_value());
}

TEST(Px4TimestampAlignerTest, RecoversAfterSingleFutureOrStaleSample)
{
  Px4TimestampAligner aligner;
  ASSERT_TRUE(aligner.align(1'000'000U, 50'000'000'000LL).has_value());

  EXPECT_FALSE(aligner.align(1'600'000U, 50'100'000'000LL).has_value());
  EXPECT_FALSE(aligner.align(1'100'000U, 50'700'000'000LL).has_value());
  const auto recovered = aligner.align(1'200'000U, 50'200'000'000LL);

  ASSERT_TRUE(recovered.has_value());
  EXPECT_EQ(recovered.value(), 50'200'000'000LL);
}

TEST(Px4TimestampAlignerTest, RejectsZeroOutOfOrderFutureAndStaleSamples)
{
  Px4TimestampAligner aligner;
  EXPECT_FALSE(aligner.align(0U, 50'000'000'000LL).has_value());
  ASSERT_TRUE(aligner.align(1'000'000U, 50'000'000'000LL).has_value());
  EXPECT_FALSE(aligner.align(999'999U, 50'010'000'000LL).has_value());
  EXPECT_FALSE(aligner.align(1'600'000U, 50'100'000'000LL).has_value());
  EXPECT_FALSE(aligner.align(1'100'000U, 50'700'000'000LL).has_value());
}

}  // namespace
}  // namespace drone_perception
