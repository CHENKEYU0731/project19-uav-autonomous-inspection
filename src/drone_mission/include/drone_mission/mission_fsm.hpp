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

#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace drone_mission
{

enum class MissionState
{
  standby,
  takeoff,
  inspecting,
  handling_exception,
  returning_home,
  landing,
  complete,
};

enum class MissionAction
{
  none,
  publish_waypoint,
  publish_home,
  command_land,
};

struct MissionDecision
{
  MissionState from;
  MissionState to;
  MissionAction action;
  int waypoint_index;
  std::string event;
  std::string reason;
};

class MissionFsm
{
public:
  explicit MissionFsm(std::size_t waypoint_count);

  MissionState state() const noexcept;
  std::size_t reached_waypoint_count() const noexcept;
  int active_waypoint_index() const noexcept;

  std::vector<MissionDecision> start();
  std::vector<MissionDecision> takeoff_reached();
  std::vector<MissionDecision> goal_reached();
  std::vector<MissionDecision> goal_unreachable(const std::string & reason);
  std::vector<MissionDecision> low_battery(const std::string & reason);
  std::vector<MissionDecision> landed();

private:
  MissionDecision transition(
    MissionState next, MissionAction action, const std::string & event,
    const std::string & reason, int waypoint_index = -1);
  MissionDecision event(
    MissionAction action, const std::string & name, const std::string & reason,
    int waypoint_index = -1) const;

  std::size_t waypoint_count_;
  std::size_t reached_waypoint_count_{0U};
  int active_waypoint_index_{-1};
  MissionState state_{MissionState::standby};
};

}  // namespace drone_mission
