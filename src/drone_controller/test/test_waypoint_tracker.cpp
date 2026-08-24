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

#include "drone_controller/waypoint_tracker.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <stdexcept>

namespace drone_controller
{
namespace
{

using namespace std::chrono_literals;

TEST(MissionGeometryTest, AppliesAltitudeRelativeToNonzeroHomePosition)
{
  const Position3D home{10.0, -4.0, 1.25};

  const Position3D target = make_ned_target(home, 2.0, -3.0, 2.5);

  EXPECT_DOUBLE_EQ(target.x, 12.0);
  EXPECT_DOUBLE_EQ(target.y, -7.0);
  EXPECT_DOUBLE_EQ(target.z, -1.25);
}

TEST(MissionGeometryTest, ConvertsMapEnuToPx4Ned)
{
  const Position3D target = map_enu_to_ned({3.0, 2.0, 2.5});

  EXPECT_DOUBLE_EQ(target.x, 2.0);
  EXPECT_DOUBLE_EQ(target.y, 3.0);
  EXPECT_DOUBLE_EQ(target.z, -2.5);
}

TEST(WaypointTrackerTest, RequiresContinuousTimeInsideThreeDimensionalRadius)
{
  WaypointTracker tracker(0.3, 1s);
  const Position3D target{1.0, 2.0, -2.5};
  const auto start = WaypointTracker::Clock::time_point{};

  EXPECT_FALSE(tracker.update({1.2, 2.1, -2.4}, target, start));
  EXPECT_FALSE(tracker.update({1.2, 2.1, -2.4}, target, start + 999ms));
  EXPECT_TRUE(tracker.update({1.2, 2.1, -2.4}, target, start + 1s));
}

TEST(WaypointTrackerTest, LeavingRadiusRestartsStableTimer)
{
  WaypointTracker tracker(0.3, 1s);
  const Position3D target{0.0, 0.0, -2.5};
  const auto start = WaypointTracker::Clock::time_point{};

  EXPECT_FALSE(tracker.update({0.1, 0.1, -2.5}, target, start));
  EXPECT_FALSE(tracker.update({0.4, 0.0, -2.5}, target, start + 750ms));
  EXPECT_FALSE(tracker.update({0.1, 0.1, -2.5}, target, start + 1s));
  EXPECT_FALSE(tracker.update({0.1, 0.1, -2.5}, target, start + 1999ms));
  EXPECT_TRUE(tracker.update({0.1, 0.1, -2.5}, target, start + 2s));
}

TEST(WaypointTrackerTest, ResetForcesACompleteNewStableInterval)
{
  WaypointTracker tracker(0.3, 1s);
  const Position3D target{0.0, 0.0, 0.0};
  const auto start = WaypointTracker::Clock::time_point{};

  EXPECT_FALSE(tracker.update({0.0, 0.0, 0.0}, target, start));
  tracker.reset();
  EXPECT_FALSE(tracker.update({0.0, 0.0, 0.0}, target, start + 2s));
  EXPECT_TRUE(tracker.update({0.0, 0.0, 0.0}, target, start + 3s));
}

TEST(WaypointTrackerTest, RejectsNonPositiveConfiguration)
{
  EXPECT_THROW(WaypointTracker(0.0, 1s), std::invalid_argument);
  EXPECT_THROW(WaypointTracker(0.3, 0s), std::invalid_argument);
}

TEST(MissionSafetyTest, RejectsNonFinitePositionSamples)
{
  const double invalid = std::numeric_limits<double>::quiet_NaN();

  EXPECT_TRUE(is_finite_position({1.0, 2.0, -2.5}));
  EXPECT_FALSE(is_finite_position({invalid, 2.0, -2.5}));
  EXPECT_FALSE(is_finite_position({1.0, invalid, -2.5}));
  EXPECT_FALSE(is_finite_position({1.0, 2.0, invalid}));
}

TEST(MissionSafetyTest, RejectsStaleOrFutureTelemetry)
{
  const auto received_at = SteadyClock::time_point{} + 10s;

  EXPECT_TRUE(sample_is_fresh(received_at, received_at + 500ms, 500ms));
  EXPECT_FALSE(sample_is_fresh(received_at, received_at + 501ms, 500ms));
  EXPECT_FALSE(sample_is_fresh(received_at, received_at - 1ms, 500ms));
  EXPECT_FALSE(sample_is_fresh(received_at, received_at, 0ms));
}

TEST(MissionSafetyTest, RequiresBothLandingAndDisarmForConfirmation)
{
  EXPECT_TRUE(landing_is_confirmed(true, true));
  EXPECT_FALSE(landing_is_confirmed(true, false));
  EXPECT_FALSE(landing_is_confirmed(false, true));
  EXPECT_FALSE(landing_is_confirmed(false, false));
}

}  // namespace
}  // namespace drone_controller
