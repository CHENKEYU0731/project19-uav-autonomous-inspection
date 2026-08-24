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

#include <drone_interfaces/msg/mission_command.hpp>
#include <drone_interfaces/msg/mission_event.hpp>
#include <drone_interfaces/msg/planner_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace drone_mission
{

using namespace std::chrono_literals;
using PlannerStatus = drone_interfaces::msg::PlannerStatus;
using SteadyClock = std::chrono::steady_clock;

struct Goal2D
{
  double x;
  double y;
};

class MissionNode : public rclcpp::Node
{
public:
  MissionNode()
  : Node("mission_node"),
    map_frame_(declare_parameter<std::string>("map_frame", "map")),
    goal_altitude_m_(declare_parameter<double>("goal_altitude_m", 2.5)),
    takeoff_altitude_m_(declare_parameter<double>("takeoff_altitude_m", 2.5)),
    takeoff_tolerance_m_(declare_parameter<double>("takeoff_tolerance_m", 0.25)),
    takeoff_stable_time_s_(declare_parameter<double>("takeoff_stable_time_s", 0.5)),
    unreachable_timeout_s_(declare_parameter<double>("unreachable_timeout_s", 2.0)),
    planner_status_timeout_s_(declare_parameter<double>("planner_status_timeout_s", 0.5)),
    landing_timeout_s_(declare_parameter<double>("landing_timeout_s", 45.0)),
    low_battery_after_reached_(declare_parameter<int>(
        "simulate_low_battery_after_reached_waypoints", -1)),
    home_{
      declare_parameter<double>("home_x_m", 0.0),
      declare_parameter<double>("home_y_m", 0.0)},
    waypoints_(parse_waypoints(declare_parameter<std::vector<double>>(
        "inspection_waypoints_xy", {0.0, 3.0, 0.0, 8.0, -1.0, 3.0}))),
    fsm_(waypoints_.size())
  {
    validate_parameters();
    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/drone_planner/goal", rclcpp::QoS(1).reliable().transient_local());
    command_publisher_ = create_publisher<drone_interfaces::msg::MissionCommand>(
      "/drone_mission/command", rclcpp::QoS(10).reliable());
    event_publisher_ = create_publisher<drone_interfaces::msg::MissionEvent>(
      "/drone_mission/event", rclcpp::QoS(50).reliable().transient_local());

    planner_status_subscription_ = create_subscription<PlannerStatus>(
      "/drone_planner/status", rclcpp::QoS(10).reliable(),
      std::bind(&MissionNode::handle_planner_status, this, std::placeholders::_1));
    position_subscription_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", rclcpp::SensorDataQoS(),
      std::bind(&MissionNode::handle_position, this, std::placeholders::_1));
    vehicle_status_subscription_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr message) {
        arming_state_ = message->arming_state;
        status_received_ = true;
      });
    land_subscription_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr message) {
        landed_ = message->landed;
        land_status_received_ = true;
      });
    timer_ = create_wall_timer(100ms, std::bind(&MissionNode::tick, this));
    RCLCPP_INFO(get_logger(), "M4 mission manager ready in STANDBY");
  }

  bool mission_succeeded() const noexcept
  {
    return mission_succeeded_;
  }

private:
  static std::vector<Goal2D> parse_waypoints(const std::vector<double> & values)
  {
    if (values.empty() || values.size() % 2U != 0U) {
      throw std::invalid_argument("inspection_waypoints_xy must contain x/y pairs");
    }
    std::vector<Goal2D> waypoints;
    waypoints.reserve(values.size() / 2U);
    for (std::size_t index = 0U; index < values.size(); index += 2U) {
      if (!std::isfinite(values[index]) || !std::isfinite(values[index + 1U])) {
        throw std::invalid_argument("inspection waypoints must be finite");
      }
      waypoints.push_back({values[index], values[index + 1U]});
    }
    return waypoints;
  }

  void validate_parameters() const
  {
    if (map_frame_.empty()) {
      throw std::invalid_argument("map_frame must not be empty");
    }
    for (const auto value : {
        goal_altitude_m_, takeoff_altitude_m_, takeoff_tolerance_m_,
        takeoff_stable_time_s_, unreachable_timeout_s_, planner_status_timeout_s_,
        landing_timeout_s_})
    {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("mission numeric parameters must be finite and positive");
      }
    }
    if (takeoff_tolerance_m_ >= takeoff_altitude_m_ ||
      planner_status_timeout_s_ > unreachable_timeout_s_ ||
      !std::isfinite(home_.x) || !std::isfinite(home_.y) ||
      low_battery_after_reached_ == 0)
    {
      throw std::invalid_argument("mission parameters violate the state-machine contract");
    }
  }

  void handle_position(const px4_msgs::msg::VehicleLocalPosition::SharedPtr message)
  {
    if (!message->xy_valid || !message->z_valid || !std::isfinite(message->x) ||
      !std::isfinite(message->y) || !std::isfinite(message->z))
    {
      return;
    }
    current_map_position_ = {message->y, message->x};
    current_down_m_ = message->z;
    if (!initial_down_m_.has_value() &&
      arming_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
    {
      initial_down_m_ = message->z;
    }
    position_received_ = true;
  }

  void handle_planner_status(const PlannerStatus::SharedPtr message)
  {
    if (!awaiting_goal_) {
      return;
    }
    const rclcpp::Time stamp(message->stamp, get_clock()->get_clock_type());
    const rclcpp::Time now = get_clock()->now();
    const double age = (now - stamp).seconds();
    if (stamp < active_goal_stamp_ || age < 0.0 || age > planner_status_timeout_s_) {
      return;
    }
    if (message->state == PlannerStatus::GOAL_REACHED && message->map_fresh &&
      message->trajectory_valid)
    {
      awaiting_goal_ = false;
      no_path_since_.reset();
      const bool returning_to_verified_waypoint =
        fsm_.state() == MissionState::returning_home &&
        active_return_waypoint_index_.has_value();
      if (fsm_.state() == MissionState::inspecting) {
        const int reached_index = fsm_.active_waypoint_index();
        if (reached_index < 0) {
          finish(false, "FSM reported an invalid reached waypoint index");
          return;
        }
        reached_waypoint_indices_.push_back(reached_index);
        process(fsm_.goal_reached());
      } else if (returning_to_verified_waypoint) {
        publish_event(
          {
            MissionState::returning_home,
            MissionState::returning_home,
            MissionAction::none,
            active_return_waypoint_index_.value(),
            "return_waypoint_reached",
            "verified breadcrumb reached during return"});
        ++return_route_cursor_;
        publish_next_return_goal();
      } else {
        process(fsm_.goal_reached());
      }
      return;
    }
    if (message->state == PlannerStatus::NO_PATH && message->map_fresh) {
      if (!no_path_since_.has_value()) {
        no_path_since_ = SteadyClock::now();
      }
      latest_no_path_reason_ = message->reason;
    } else {
      no_path_since_.reset();
      latest_no_path_reason_.clear();
    }
  }

  void tick()
  {
    const auto now = SteadyClock::now();
    if (fsm_.state() == MissionState::standby) {
      if (!position_received_ || !status_received_ || !land_status_received_ ||
        !initial_down_m_.has_value())
      {
        return;
      }
      process(fsm_.start());
    }

    if (fsm_.state() == MissionState::takeoff) {
      const double climbed_m = initial_down_m_.value() - current_down_m_;
      const bool ready = arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
        climbed_m >= takeoff_altitude_m_ - takeoff_tolerance_m_;
      if (!ready) {
        takeoff_ready_since_.reset();
      } else if (!takeoff_ready_since_.has_value()) {
        takeoff_ready_since_ = now;
      } else if (elapsed_seconds(takeoff_ready_since_.value(), now) >= takeoff_stable_time_s_) {
        process(fsm_.takeoff_reached());
      }
    }

    if (fsm_.state() == MissionState::inspecting && no_path_since_.has_value() &&
      elapsed_seconds(no_path_since_.value(), now) >= unreachable_timeout_s_)
    {
      awaiting_goal_ = false;
      no_path_since_.reset();
      process(fsm_.goal_unreachable(latest_no_path_reason_));
    }

    if (!low_battery_triggered_ && low_battery_after_reached_ > 0 &&
      fsm_.state() == MissionState::inspecting &&
      fsm_.reached_waypoint_count() >= static_cast<std::size_t>(low_battery_after_reached_))
    {
      low_battery_triggered_ = true;
      awaiting_goal_ = false;
      no_path_since_.reset();
      process(fsm_.low_battery("simulated battery fell below the return threshold"));
    }

    if (fsm_.state() == MissionState::landing) {
      const bool landing_timed_out =
        landing_started_at_.has_value() &&
        elapsed_seconds(landing_started_at_.value(), now) > landing_timeout_s_;
      const bool should_repeat_land =
        !last_land_command_at_.has_value() ||
        elapsed_seconds(last_land_command_at_.value(), now) >= 1.0;
      if (landed_ && arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
        process(fsm_.landed());
      } else if (landing_timed_out) {
        finish(false, "timed out waiting for landed and disarmed confirmation");
      } else if (should_repeat_land) {
        publish_land_command("repeating LAND while awaiting confirmation");
      }
    }
  }

  void process(const std::vector<MissionDecision> & decisions)
  {
    for (const auto & decision : decisions) {
      publish_event(decision);
      switch (decision.action) {
        case MissionAction::none:
          break;
        case MissionAction::publish_waypoint:
          if (decision.waypoint_index < 0 ||
            static_cast<std::size_t>(decision.waypoint_index) >= waypoints_.size())
          {
            finish(false, "FSM produced an invalid waypoint index");
            return;
          }
          publish_goal(waypoints_[static_cast<std::size_t>(decision.waypoint_index)]);
          break;
        case MissionAction::publish_home:
          start_return_route();
          break;
        case MissionAction::command_land:
          landing_started_at_ = SteadyClock::now();
          publish_land_command(decision.reason);
          break;
      }
      if (decision.to == MissionState::complete) {
        finish(true, decision.reason);
      }
    }
  }

  void publish_goal(const Goal2D goal)
  {
    const double yaw_rad = std::atan2(
      goal.x - current_map_position_.x, goal.y - current_map_position_.y);
    publish_yaw_command(static_cast<float>(yaw_rad), "face the active mission goal");
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = map_frame_;
    message.pose.position.x = goal.x;
    message.pose.position.y = goal.y;
    message.pose.position.z = goal_altitude_m_;
    message.pose.orientation.w = 1.0;
    active_goal_stamp_ = rclcpp::Time(message.header.stamp, get_clock()->get_clock_type());
    awaiting_goal_ = true;
    no_path_since_.reset();
    latest_no_path_reason_.clear();
    goal_publisher_->publish(message);
    RCLCPP_INFO(
      get_logger(), "Published mission goal [%.2f, %.2f, %.2f]", goal.x, goal.y,
      goal_altitude_m_);
  }

  void start_return_route()
  {
    return_route_indices_.clear();
    if (reached_waypoint_indices_.size() > 1U) {
      for (auto iterator = std::next(reached_waypoint_indices_.rbegin());
        iterator != reached_waypoint_indices_.rend(); ++iterator)
      {
        return_route_indices_.push_back(*iterator);
      }
    }
    return_route_cursor_ = 0U;
    publish_next_return_goal();
  }

  void publish_next_return_goal()
  {
    if (return_route_cursor_ < return_route_indices_.size()) {
      const int waypoint_index = return_route_indices_[return_route_cursor_];
      if (waypoint_index < 0 ||
        static_cast<std::size_t>(waypoint_index) >= waypoints_.size())
      {
        finish(false, "return route contains an invalid waypoint index");
        return;
      }
      active_return_waypoint_index_ = waypoint_index;
      publish_goal(waypoints_[static_cast<std::size_t>(waypoint_index)]);
      return;
    }
    active_return_waypoint_index_.reset();
    publish_goal(home_);
  }

  void publish_land_command(const std::string & reason)
  {
    drone_interfaces::msg::MissionCommand message;
    message.command = drone_interfaces::msg::MissionCommand::LAND;
    message.stamp = get_clock()->now();
    message.yaw_rad = 0.0F;
    message.reason = reason;
    command_publisher_->publish(message);
    last_land_command_at_ = SteadyClock::now();
  }

  void publish_yaw_command(const float yaw_rad, const std::string & reason)
  {
    drone_interfaces::msg::MissionCommand message;
    message.command = drone_interfaces::msg::MissionCommand::SET_YAW;
    message.stamp = get_clock()->now();
    message.yaw_rad = yaw_rad;
    message.reason = reason;
    command_publisher_->publish(message);
  }

  void publish_event(const MissionDecision & decision)
  {
    drone_interfaces::msg::MissionEvent message;
    message.stamp = get_clock()->now();
    message.sequence = ++event_sequence_;
    message.from_state = state_code(decision.from);
    message.to_state = state_code(decision.to);
    message.waypoint_index = decision.waypoint_index;
    message.event = decision.event;
    message.reason = decision.reason;
    event_publisher_->publish(message);
    RCLCPP_INFO(
      get_logger(), "[M4 FSM] #%" PRIu64 " %s -> %s event=%s waypoint=%d reason=%s",
      static_cast<std::uint64_t>(message.sequence), state_name(decision.from),
      state_name(decision.to), decision.event.c_str(), decision.waypoint_index,
      decision.reason.c_str());
  }

  void finish(const bool succeeded, const std::string & reason)
  {
    if (mission_finished_) {
      return;
    }
    mission_finished_ = true;
    mission_succeeded_ = succeeded;
    timer_->cancel();
    if (succeeded) {
      RCLCPP_INFO(get_logger(), "M4 mission complete: %s", reason.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "M4 mission failed: %s", reason.c_str());
    }
    rclcpp::shutdown();
  }

  static double elapsed_seconds(
    const SteadyClock::time_point from,
    const SteadyClock::time_point to)
  {
    return std::chrono::duration<double>(to - from).count();
  }

  static std::uint8_t state_code(const MissionState state)
  {
    return static_cast<std::uint8_t>(state);
  }

  static const char * state_name(const MissionState state)
  {
    switch (state) {
      case MissionState::standby: return "STANDBY";
      case MissionState::takeoff: return "TAKEOFF";
      case MissionState::inspecting: return "INSPECTING";
      case MissionState::handling_exception: return "HANDLING_EXCEPTION";
      case MissionState::returning_home: return "RETURNING_HOME";
      case MissionState::landing: return "LANDING";
      case MissionState::complete: return "COMPLETE";
    }
    return "UNKNOWN";
  }

  std::string map_frame_;
  double goal_altitude_m_;
  double takeoff_altitude_m_;
  double takeoff_tolerance_m_;
  double takeoff_stable_time_s_;
  double unreachable_timeout_s_;
  double planner_status_timeout_s_;
  double landing_timeout_s_;
  int low_battery_after_reached_;
  Goal2D home_;
  Goal2D current_map_position_{0.0, 0.0};
  std::vector<Goal2D> waypoints_;
  std::vector<int> reached_waypoint_indices_;
  std::vector<int> return_route_indices_;
  std::size_t return_route_cursor_{0U};
  std::optional<int> active_return_waypoint_index_;
  MissionFsm fsm_;
  std::optional<double> initial_down_m_;
  double current_down_m_{0.0};
  std::uint8_t arming_state_{0U};
  bool landed_{true};
  bool position_received_{false};
  bool status_received_{false};
  bool land_status_received_{false};
  bool awaiting_goal_{false};
  bool low_battery_triggered_{false};
  bool mission_finished_{false};
  bool mission_succeeded_{false};
  std::uint64_t event_sequence_{0U};
  std::string latest_no_path_reason_;
  rclcpp::Time active_goal_stamp_{0, 0, RCL_ROS_TIME};
  std::optional<SteadyClock::time_point> takeoff_ready_since_;
  std::optional<SteadyClock::time_point> no_path_since_;
  std::optional<SteadyClock::time_point> landing_started_at_;
  std::optional<SteadyClock::time_point> last_land_command_at_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::Publisher<drone_interfaces::msg::MissionCommand>::SharedPtr command_publisher_;
  rclcpp::Publisher<drone_interfaces::msg::MissionEvent>::SharedPtr event_publisher_;
  rclcpp::Subscription<PlannerStatus>::SharedPtr planner_status_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drone_mission

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<drone_mission::MissionNode>();
    rclcpp::spin(node);
    return node->mission_succeeded() ? 0 : 1;
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("mission_node"), "Fatal error: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
