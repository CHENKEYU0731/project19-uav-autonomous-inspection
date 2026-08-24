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

#include <stdexcept>

namespace drone_mission
{

MissionFsm::MissionFsm(const std::size_t waypoint_count)
: waypoint_count_(waypoint_count)
{
  if (waypoint_count_ == 0U) {
    throw std::invalid_argument("mission requires at least one inspection waypoint");
  }
}

MissionState MissionFsm::state() const noexcept
{
  return state_;
}

std::size_t MissionFsm::reached_waypoint_count() const noexcept
{
  return reached_waypoint_count_;
}

int MissionFsm::active_waypoint_index() const noexcept
{
  return active_waypoint_index_;
}

std::vector<MissionDecision> MissionFsm::start()
{
  if (state_ != MissionState::standby) {
    return {};
  }
  return {transition(
      MissionState::takeoff, MissionAction::none, "mission_started",
      "waiting for the controller to reach takeoff altitude")};
}

std::vector<MissionDecision> MissionFsm::takeoff_reached()
{
  if (state_ != MissionState::takeoff) {
    return {};
  }
  active_waypoint_index_ = 0;
  return {transition(
      MissionState::inspecting, MissionAction::publish_waypoint,
      "takeoff_completed", "publishing the first inspection waypoint",
      active_waypoint_index_)};
}

std::vector<MissionDecision> MissionFsm::goal_reached()
{
  if (state_ == MissionState::inspecting) {
    ++reached_waypoint_count_;
    if (static_cast<std::size_t>(active_waypoint_index_ + 1) < waypoint_count_) {
      ++active_waypoint_index_;
      return {event(
          MissionAction::publish_waypoint, "waypoint_reached",
          "publishing the next inspection waypoint", active_waypoint_index_)};
    }
    active_waypoint_index_ = -1;
    return {transition(
        MissionState::returning_home, MissionAction::publish_home,
        "inspection_completed", "all inspection waypoints were processed")};
  }
  if (state_ == MissionState::returning_home) {
    return {transition(
        MissionState::landing, MissionAction::command_land,
        "home_reached", "home hover point reached")};
  }
  return {};
}

std::vector<MissionDecision> MissionFsm::goal_unreachable(const std::string & reason)
{
  if (state_ != MissionState::inspecting) {
    return {};
  }
  std::vector<MissionDecision> decisions;
  decisions.push_back(
    transition(
      MissionState::handling_exception, MissionAction::none,
      "waypoint_unreachable", reason, active_waypoint_index_));
  if (static_cast<std::size_t>(active_waypoint_index_ + 1) < waypoint_count_) {
    ++active_waypoint_index_;
    decisions.push_back(
      transition(
        MissionState::inspecting, MissionAction::publish_waypoint,
        "waypoint_skipped", "unreachable waypoint skipped",
        active_waypoint_index_));
  } else {
    active_waypoint_index_ = -1;
    decisions.push_back(
      transition(
        MissionState::returning_home, MissionAction::publish_home,
        "waypoint_skipped", "last waypoint skipped; returning home"));
  }
  return decisions;
}

std::vector<MissionDecision> MissionFsm::low_battery(const std::string & reason)
{
  if (state_ != MissionState::takeoff && state_ != MissionState::inspecting) {
    return {};
  }
  active_waypoint_index_ = -1;
  return {
    transition(
      MissionState::handling_exception, MissionAction::none,
      "low_battery", reason),
    transition(
      MissionState::returning_home, MissionAction::publish_home,
      "inspection_interrupted", "low battery requires immediate return to home")};
}

std::vector<MissionDecision> MissionFsm::landed()
{
  if (state_ != MissionState::landing) {
    return {};
  }
  return {transition(
      MissionState::complete, MissionAction::none,
      "landing_completed", "vehicle landed and disarmed")};
}

MissionDecision MissionFsm::transition(
  const MissionState next, const MissionAction action, const std::string & event_name,
  const std::string & reason, const int waypoint_index)
{
  const MissionState previous = state_;
  state_ = next;
  return {previous, next, action, waypoint_index, event_name, reason};
}

MissionDecision MissionFsm::event(
  const MissionAction action, const std::string & name, const std::string & reason,
  const int waypoint_index) const
{
  return {state_, state_, action, waypoint_index, name, reason};
}

}  // namespace drone_mission
