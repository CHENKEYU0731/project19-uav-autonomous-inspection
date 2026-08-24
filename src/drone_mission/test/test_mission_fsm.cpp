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

#include "drone_mission/mission_fsm.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

namespace
{

using drone_mission::MissionAction;
using drone_mission::MissionFsm;
using drone_mission::MissionState;

TEST(MissionFsm, RejectsEmptyInspectionPlan)
{
  EXPECT_THROW(MissionFsm(0U), std::invalid_argument);
}

TEST(MissionFsm, CompletesMultipleWaypointsThenReturnsAndLands)
{
  MissionFsm fsm(2U);
  EXPECT_EQ(fsm.start().front().to, MissionState::takeoff);
  auto decisions = fsm.takeoff_reached();
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_EQ(decisions.front().action, MissionAction::publish_waypoint);
  EXPECT_EQ(decisions.front().waypoint_index, 0);

  decisions = fsm.goal_reached();
  ASSERT_EQ(decisions.size(), 1U);
  EXPECT_EQ(decisions.front().from, MissionState::inspecting);
  EXPECT_EQ(decisions.front().to, MissionState::inspecting);
  EXPECT_EQ(decisions.front().waypoint_index, 1);

  decisions = fsm.goal_reached();
  EXPECT_EQ(decisions.front().to, MissionState::returning_home);
  EXPECT_EQ(decisions.front().action, MissionAction::publish_home);
  decisions = fsm.goal_reached();
  EXPECT_EQ(decisions.front().to, MissionState::landing);
  EXPECT_EQ(decisions.front().action, MissionAction::command_land);
  EXPECT_EQ(fsm.landed().front().to, MissionState::complete);
  EXPECT_EQ(fsm.reached_waypoint_count(), 2U);
}

TEST(MissionFsm, SkipsUnreachableWaypointWithExplicitExceptionTransitions)
{
  MissionFsm fsm(3U);
  fsm.start();
  fsm.takeoff_reached();
  const auto decisions = fsm.goal_unreachable("planner reported no path");
  ASSERT_EQ(decisions.size(), 2U);
  EXPECT_EQ(decisions[0].to, MissionState::handling_exception);
  EXPECT_EQ(decisions[0].event, "waypoint_unreachable");
  EXPECT_EQ(decisions[1].from, MissionState::handling_exception);
  EXPECT_EQ(decisions[1].to, MissionState::inspecting);
  EXPECT_EQ(decisions[1].action, MissionAction::publish_waypoint);
  EXPECT_EQ(decisions[1].waypoint_index, 1);
}

TEST(MissionFsm, LowBatteryImmediatelyOverridesInspectionWithReturnHome)
{
  MissionFsm fsm(3U);
  fsm.start();
  fsm.takeoff_reached();
  const auto decisions = fsm.low_battery("simulated battery below threshold");
  ASSERT_EQ(decisions.size(), 2U);
  EXPECT_EQ(decisions[0].to, MissionState::handling_exception);
  EXPECT_EQ(decisions[0].event, "low_battery");
  EXPECT_EQ(decisions[1].to, MissionState::returning_home);
  EXPECT_EQ(decisions[1].action, MissionAction::publish_home);
  EXPECT_EQ(fsm.active_waypoint_index(), -1);
}

TEST(MissionFsm, IgnoresEventsThatDoNotApplyToCurrentState)
{
  MissionFsm fsm(1U);
  EXPECT_TRUE(fsm.goal_reached().empty());
  EXPECT_TRUE(fsm.goal_unreachable("stale status").empty());
  EXPECT_TRUE(fsm.landed().empty());
  fsm.start();
  EXPECT_TRUE(fsm.start().empty());
}

}  // namespace
