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

#include <drone_interfaces/msg/mission_command.hpp>
#include <drone_interfaces/msg/planned_trajectory.hpp>
#include <drone_interfaces/msg/planner_status.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

#include <algorithm>
#include <cinttypes>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace drone_controller
{

using namespace std::chrono_literals;

class WaypointController : public rclcpp::Node
{
public:
  WaypointController()
  : Node("waypoint_controller"),
    takeoff_altitude_m_(declare_parameter<double>("takeoff_altitude_m", 2.5)),
    acceptance_radius_m_(declare_parameter<double>("acceptance_radius_m", 0.3)),
    stable_time_s_(declare_parameter<double>("stable_time_s", 1.0)),
    warmup_setpoint_count_(declare_parameter<int>("warmup_setpoint_count", 10)),
    startup_timeout_s_(declare_parameter<double>("startup_timeout_s", 120.0)),
    command_timeout_s_(declare_parameter<double>("command_timeout_s", 10.0)),
    segment_timeout_s_(declare_parameter<double>("segment_timeout_s", 30.0)),
    landing_timeout_s_(declare_parameter<double>("landing_timeout_s", 30.0)),
    position_timeout_s_(declare_parameter<double>("position_timeout_s", 0.5)),
    status_timeout_s_(declare_parameter<double>("status_timeout_s", 2.0)),
    map_frame_(declare_parameter<std::string>("map_frame", "map")),
    use_planned_trajectory_(declare_parameter<bool>("use_planned_trajectory", false)),
    mission_managed_landing_(declare_parameter<bool>("mission_managed_landing", false)),
    planned_trajectory_topic_(declare_parameter<std::string>(
        "planned_trajectory_topic", "/drone_planner/trajectory")),
    planner_stale_timeout_s_(declare_parameter<double>("planner_stale_timeout_s", 0.5)),
    waypoint_offsets_xy_(declare_parameter<std::vector<double>>(
        "waypoint_offsets_xy", {2.0, 0.0, 2.0, 2.0, -2.0, 2.0, -2.0, 0.0})),
    waypoint_tracker_(
      acceptance_radius_m_,
      std::chrono::duration_cast<WaypointTracker::Clock::duration>(
        std::chrono::duration<double>(stable_time_s_)))
  {
    validate_parameters();

    offboard_mode_publisher_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);
    trajectory_publisher_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", 10);

    position_subscription_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleLocalPosition::UniquePtr message) {
        const Position3D position{message->x, message->y, message->z};
        if (message->xy_valid && message->z_valid && is_finite_position(position)) {
          current_position_ = position;
          position_received_ = true;
          last_position_received_at_ = WaypointTracker::Clock::now();
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "Ignoring invalid PX4 local position sample");
        }
      });

    status_subscription_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleStatus::UniquePtr message) {
        arming_state_ = message->arming_state;
        nav_state_ = message->nav_state;
        failsafe_ = message->failsafe;
        pre_flight_checks_pass_ = message->pre_flight_checks_pass;
        status_received_ = true;
        last_status_received_at_ = WaypointTracker::Clock::now();
      });

    land_detected_subscription_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleLandDetected::UniquePtr message) {
        landed_ = message->landed;
        land_status_received_ = true;
        last_land_status_received_at_ = WaypointTracker::Clock::now();
      });

    command_ack_subscription_ = create_subscription<px4_msgs::msg::VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack", rclcpp::SensorDataQoS(),
      [this](const px4_msgs::msg::VehicleCommandAck::UniquePtr message) {
        handle_command_ack(*message);
      });

    planned_trajectory_subscription_ =
      create_subscription<drone_interfaces::msg::PlannedTrajectory>(
      planned_trajectory_topic_, rclcpp::QoS(1).reliable().transient_local(),
      [this](const drone_interfaces::msg::PlannedTrajectory::UniquePtr message) {
        handle_planned_trajectory(*message);
      });
    planner_status_subscription_ = create_subscription<drone_interfaces::msg::PlannerStatus>(
      "/drone_planner/status", rclcpp::QoS(10).reliable(),
      [this](const drone_interfaces::msg::PlannerStatus::UniquePtr message) {
        handle_planner_status(*message);
      });
    mission_command_subscription_ =
      create_subscription<drone_interfaces::msg::MissionCommand>(
      "/drone_mission/command", rclcpp::QoS(10).reliable(),
      [this](const drone_interfaces::msg::MissionCommand::UniquePtr message) {
        handle_mission_command(*message);
      });
    insertion_hold_subscription_ = create_subscription<std_msgs::msg::Bool>(
      "/drone_m3/insertion_hold", rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::UniquePtr message) {
        if (!use_planned_trajectory_ || insertion_hold_active_ == message->data) {
          return;
        }
        insertion_hold_active_ = message->data;
        if (insertion_hold_active_) {
          insertion_hold_started_at_ = WaypointTracker::Clock::now();
          insertion_hold_target_ = current_position_;
          RCLCPP_INFO(get_logger(), "Holding position for dynamic blocker insertion");
        } else {
          insertion_hold_target_.reset();
          RCLCPP_INFO(get_logger(), "Dynamic blocker insertion hold released");
        }
      });

    timer_ = create_wall_timer(100ms, std::bind(&WaypointController::control_tick, this));
    RCLCPP_INFO(
      get_logger(), "Waiting for valid PX4 local position, vehicle status, and land status");
  }

  bool mission_succeeded() const
  {
    return mission_succeeded_;
  }

private:
  enum class State
  {
    waiting_for_vehicle,
    priming_offboard,
    requesting_offboard,
    taking_off,
    waiting_for_planner,
    inspecting,
    returning_home,
    landing,
    complete,
  };

  struct PlannedTrajectorySnapshot
  {
    drone_interfaces::msg::PlannedTrajectory message;
    WaypointTracker::Clock::time_point received_at{WaypointTracker::Clock::now()};
    WaypointTracker::Clock::time_point execution_started_at{received_at};
    std::size_t point_index{0U};
  };

  void handle_planned_trajectory(
    const drone_interfaces::msg::PlannedTrajectory & message)
  {
    if (!use_planned_trajectory_) {
      return;
    }
    const rclcpp::Time created_at(message.created_at, get_clock()->get_clock_type());
    const rclcpp::Time trajectory_stamp(
      message.trajectory.header.stamp, get_clock()->get_clock_type());
    const rclcpp::Time now = get_clock()->now();
    if (message.trajectory_id == 0U || message.frame_id != map_frame_ ||
      message.trajectory.header.frame_id != map_frame_ ||
      created_at.nanoseconds() <= 0 || trajectory_stamp.nanoseconds() <= 0 ||
      created_at != trajectory_stamp || now < created_at ||
      (now - created_at).seconds() > planner_stale_timeout_s_ ||
      message.trajectory.points.empty())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting planner trajectory with invalid frame, timestamp, or freshness");
      return;
    }

    double previous_time_s = -1.0;
    for (const auto & point : message.trajectory.points) {
      const double time_s = static_cast<double>(point.time_from_start.sec) +
        static_cast<double>(point.time_from_start.nanosec) / 1e9;
      if (point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000U ||
        !std::isfinite(time_s) || time_s < previous_time_s ||
        point.transforms.size() != 1U || point.velocities.size() != 1U ||
        point.accelerations.size() != 1U ||
        !valid_trajectory_vector(point.transforms.front().translation) ||
        !valid_trajectory_vector(point.velocities.front().linear) ||
        !valid_trajectory_vector(point.accelerations.front().linear))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Rejecting planner trajectory with invalid point data");
        return;
      }
      previous_time_s = time_s;
    }

    const auto now_steady = WaypointTracker::Clock::now();
    const bool new_trajectory =
      !latest_planned_trajectory_.has_value() ||
      latest_planned_trajectory_->message.trajectory_id != message.trajectory_id;
    if (new_trajectory) {
      planned_endpoint_reached_ = false;
    }
    PlannedTrajectorySnapshot snapshot;
    snapshot.message = message;
    snapshot.received_at = now_steady;
    snapshot.execution_started_at = new_trajectory ? now_steady :
      latest_planned_trajectory_->execution_started_at;
    snapshot.point_index = new_trajectory ? 0U : std::min(
      latest_planned_trajectory_->point_index, message.trajectory.points.size() - 1U);
    latest_planned_trajectory_ = std::move(snapshot);

    if (planner_status_received_) {
      planner_status_safe_ =
        (planner_state_ == drone_interfaces::msg::PlannerStatus::READY ||
        planner_state_ == drone_interfaces::msg::PlannerStatus::GOAL_REACHED) &&
        planner_map_fresh_ && planner_trajectory_valid_ &&
        planner_status_trajectory_id_ == message.trajectory_id;
    }
  }

  static bool valid_trajectory_vector(const geometry_msgs::msg::Vector3 & vector)
  {
    return std::isfinite(vector.x) && std::isfinite(vector.y) && std::isfinite(vector.z);
  }

  void handle_planner_status(const drone_interfaces::msg::PlannerStatus & message)
  {
    if (!use_planned_trajectory_) {
      return;
    }
    planner_status_received_ = true;
    planner_status_received_at_ = WaypointTracker::Clock::now();
    planner_state_ = message.state;
    planner_map_fresh_ = message.map_fresh;
    planner_trajectory_valid_ = message.trajectory_valid;
    planner_status_trajectory_id_ = message.trajectory_id;
    planner_status_safe_ =
      (message.state == drone_interfaces::msg::PlannerStatus::READY ||
      message.state == drone_interfaces::msg::PlannerStatus::GOAL_REACHED) &&
      message.map_fresh && message.trajectory_valid && latest_planned_trajectory_.has_value() &&
      message.trajectory_id == latest_planned_trajectory_->message.trajectory_id;
  }

  void handle_mission_command(const drone_interfaces::msg::MissionCommand & message)
  {
    if (!mission_managed_landing_ ||
      state_ == State::landing || state_ == State::complete)
    {
      return;
    }
    if (message.command == drone_interfaces::msg::MissionCommand::SET_YAW) {
      constexpr float pi = 3.14159265358979323846F;
      if (!std::isfinite(message.yaw_rad) || message.yaw_rad < -pi || message.yaw_rad > pi) {
        RCLCPP_WARN(get_logger(), "Rejecting invalid mission yaw command");
        return;
      }
      commanded_yaw_rad_ = message.yaw_rad;
      RCLCPP_INFO(
        get_logger(), "Accepted mission yaw command %.2f rad: %s",
        commanded_yaw_rad_, message.reason.c_str());
      return;
    }
    if (message.command != drone_interfaces::msg::MissionCommand::LAND) {
      RCLCPP_WARN(
        get_logger(), "Ignoring unknown mission command %u",
        static_cast<unsigned int>(message.command));
      return;
    }
    const rclcpp::Time stamp(message.stamp, get_clock()->get_clock_type());
    const rclcpp::Time now = get_clock()->now();
    if (stamp.nanoseconds() <= 0 || now < stamp ||
      (now - stamp).seconds() > command_timeout_s_)
    {
      RCLCPP_WARN(get_logger(), "Rejecting stale or invalid mission LAND command");
      return;
    }
    if (arming_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
      RCLCPP_WARN(get_logger(), "Ignoring mission LAND command because vehicle is not armed");
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Accepted mission LAND command: %s", message.reason.c_str());
    request_landing();
  }

  bool planner_execution_is_safe(const WaypointTracker::Clock::time_point now)
  {
    if (!use_planned_trajectory_ || !latest_planned_trajectory_.has_value() ||
      !planner_status_received_ || !planner_status_safe_)
    {
      return false;
    }
    const auto maximum_age = std::chrono::duration_cast<WaypointTracker::Clock::duration>(
      std::chrono::duration<double>(planner_stale_timeout_s_));
    if (!sample_is_fresh(planner_status_received_at_, now, maximum_age) ||
      !sample_is_fresh(latest_planned_trajectory_->received_at, now, maximum_age))
    {
      return false;
    }
    const rclcpp::Time created_at(
      latest_planned_trajectory_->message.created_at, get_clock()->get_clock_type());
    const rclcpp::Time ros_now = get_clock()->now();
    return created_at.nanoseconds() > 0 && ros_now >= created_at &&
           (ros_now - created_at).seconds() <= planner_stale_timeout_s_;
  }

  void begin_planned_execution(const WaypointTracker::Clock::time_point now)
  {
    if (!latest_planned_trajectory_.has_value()) {
      return;
    }
    latest_planned_trajectory_->execution_started_at = now;
    latest_planned_trajectory_->point_index = 0U;
    waypoint_tracker_.reset();
    target_started_at_ = now;
    planner_hold_active_ = false;
    state_ = State::inspecting;
    RCLCPP_INFO(
      get_logger(), "Planner trajectory %" PRIu64 " accepted; beginning map-frame execution",
      static_cast<std::uint64_t>(latest_planned_trajectory_->message.trajectory_id));
  }

  void validate_parameters() const
  {
    if (!std::isfinite(takeoff_altitude_m_) || takeoff_altitude_m_ <= 0.0) {
      throw std::invalid_argument("takeoff_altitude_m must be finite and positive");
    }
    if (!std::isfinite(stable_time_s_) || stable_time_s_ <= 0.0) {
      throw std::invalid_argument("stable_time_s must be finite and positive");
    }
    if (warmup_setpoint_count_ < 10) {
      throw std::invalid_argument("warmup_setpoint_count must be at least 10");
    }
    validate_positive_duration(startup_timeout_s_, "startup_timeout_s");
    validate_positive_duration(command_timeout_s_, "command_timeout_s");
    validate_positive_duration(segment_timeout_s_, "segment_timeout_s");
    validate_positive_duration(landing_timeout_s_, "landing_timeout_s");
    validate_positive_duration(position_timeout_s_, "position_timeout_s");
    validate_positive_duration(status_timeout_s_, "status_timeout_s");
    if (map_frame_.empty()) {
      throw std::invalid_argument("map_frame must not be empty");
    }
    if (planned_trajectory_topic_.empty()) {
      throw std::invalid_argument("planned_trajectory_topic must not be empty");
    }
    validate_positive_duration(planner_stale_timeout_s_, "planner_stale_timeout_s");
    if (planner_stale_timeout_s_ > 10.0) {
      throw std::invalid_argument("planner_stale_timeout_s exceeds the safety limit");
    }
    if (mission_managed_landing_ && !use_planned_trajectory_) {
      throw std::invalid_argument(
              "mission_managed_landing requires use_planned_trajectory");
    }
    if (position_timeout_s_ >= stable_time_s_) {
      throw std::invalid_argument("position_timeout_s must be shorter than stable_time_s");
    }
    if (waypoint_offsets_xy_.size() < 8 || waypoint_offsets_xy_.size() % 2 != 0) {
      throw std::invalid_argument("waypoint_offsets_xy must contain at least four x/y pairs");
    }
    for (const double value : waypoint_offsets_xy_) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument("waypoint_offsets_xy values must be finite");
      }
    }
  }

  static void validate_positive_duration(const double value, const char * name)
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(std::string(name) + " must be finite and positive");
    }
  }

  void control_tick()
  {
    const auto now = WaypointTracker::Clock::now();
    if (!position_received_ || !status_received_ || !land_status_received_) {
      if (elapsed_seconds(node_started_at_, now) > startup_timeout_s_) {
        finish_mission(
          false,
          "timed out waiting for PX4 position, vehicle status, and land status");
      }
      return;
    }

    const std::string telemetry_failure = stale_telemetry_reason(now);
    if (!telemetry_failure.empty()) {
      if (state_ == State::landing) {
        mark_mission_failed(telemetry_failure);
      } else if (state_ != State::complete) {
        abort_or_land(telemetry_failure);
        return;
      }
    }

    if (failsafe_ && state_ != State::complete) {
      if (state_ == State::landing) {
        mark_mission_failed("PX4 entered failsafe during landing");
      } else {
        abort_or_land("PX4 entered failsafe");
        return;
      }
    }

    switch (state_) {
      case State::waiting_for_vehicle:
        start_mission();
        break;
      case State::priming_offboard:
        prime_offboard();
        break;
      case State::requesting_offboard:
        await_offboard_activation(now);
        break;
      case State::taking_off:
        track_takeoff(now);
        break;
      case State::waiting_for_planner:
        wait_for_planner(now);
        break;
      case State::inspecting:
        if (use_planned_trajectory_) {
          if (insertion_hold_active_) {
            if (!insertion_hold_target_.has_value()) {
              abort_or_land("dynamic blocker insertion hold has no valid target");
              break;
            }
            publish_position_setpoint(insertion_hold_target_.value());
            if (elapsed_seconds(insertion_hold_started_at_, now) > command_timeout_s_) {
              abort_or_land("dynamic blocker insertion hold timed out");
            }
          } else {
            track_planned_trajectory(now);
          }
        } else {
          track_inspection_waypoint(now);
        }
        break;
      case State::returning_home:
        track_return_home(now);
        break;
      case State::landing:
        track_landing(now);
        break;
      case State::complete:
        break;
    }
  }

  void start_mission()
  {
    if (arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
      abort_or_land("refusing to start a new mission because vehicle is already armed");
      return;
    }
    if (arming_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
      finish_mission(false, "refusing to start because vehicle arming state is unknown");
      return;
    }
    if (!landed_) {
      finish_mission(false, "refusing to start because vehicle is not confirmed landed");
      return;
    }
    if (!pre_flight_checks_pass_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for PX4 pre-flight checks to pass");
      if (elapsed_seconds(node_started_at_, WaypointTracker::Clock::now()) > startup_timeout_s_) {
        finish_mission(false, "timed out waiting for PX4 pre-flight checks");
      }
      return;
    }

    home_position_ = current_position_;
    takeoff_target_ = make_ned_target(home_position_, 0.0, 0.0, takeoff_altitude_m_);
    inspection_targets_.clear();
    inspection_targets_.reserve(waypoint_offsets_xy_.size() / 2);
    for (std::size_t index = 0; index < waypoint_offsets_xy_.size(); index += 2) {
      inspection_targets_.push_back(
        make_ned_target(
          home_position_, waypoint_offsets_xy_[index], waypoint_offsets_xy_[index + 1],
          takeoff_altitude_m_));
    }

    state_ = State::priming_offboard;
    RCLCPP_INFO(
      get_logger(), "Mission initialized at home [%.2f, %.2f]; priming Offboard",
      home_position_.x, home_position_.y);
  }

  void prime_offboard()
  {
    publish_position_setpoint(takeoff_target_);
    if (++warmup_counter_ < warmup_setpoint_count_) {
      return;
    }

    state_ = State::requesting_offboard;
    state_entered_at_ = WaypointTracker::Clock::now();
    send_activation_commands(state_entered_at_);
    RCLCPP_INFO(get_logger(), "Offboard and arm commands sent; awaiting PX4 confirmation");
  }

  void await_offboard_activation(const WaypointTracker::Clock::time_point now)
  {
    publish_position_setpoint(takeoff_target_);

    if (command_rejected_) {
      abort_or_land("PX4 rejected the Offboard or arm command");
      return;
    }
    if (arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED &&
      nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD)
    {
      waypoint_tracker_.reset();
      target_started_at_ = now;
      state_ = State::taking_off;
      RCLCPP_INFO(
        get_logger(), "PX4 confirmed Offboard and armed; taking off to %.2f m above home",
        takeoff_altitude_m_);
      return;
    }
    if (elapsed_seconds(state_entered_at_, now) > command_timeout_s_) {
      abort_or_land("timed out waiting for Offboard and arm confirmation");
      return;
    }
    if (now - last_command_sent_at_ >= 1s) {
      send_activation_commands(now);
    }
  }

  void track_takeoff(const WaypointTracker::Clock::time_point now)
  {
    if (!flight_control_is_valid()) {
      return;
    }
    publish_position_setpoint(takeoff_target_);
    if (elapsed_seconds(target_started_at_, now) > segment_timeout_s_) {
      abort_or_land("takeoff target timed out");
      return;
    }
    if (!waypoint_tracker_.update(current_position_, takeoff_target_, now)) {
      return;
    }

    record_settled_error(takeoff_target_);
    waypoint_tracker_.reset();
    current_waypoint_index_ = 0;
    target_started_at_ = now;
    if (use_planned_trajectory_) {
      state_ = State::waiting_for_planner;
      RCLCPP_INFO(get_logger(), "Takeoff target reached; waiting for a fresh planner trajectory");
    } else {
      state_ = State::inspecting;
      RCLCPP_INFO(
        get_logger(), "Takeoff target reached; starting waypoint 1/%zu",
        inspection_targets_.size());
    }
  }

  void wait_for_planner(const WaypointTracker::Clock::time_point now)
  {
    if (!flight_control_is_valid()) {
      return;
    }
    if (planner_execution_is_safe(now)) {
      planner_hold_active_ = false;
      planner_hold_target_.reset();
      begin_planned_execution(now);
      return;
    }
    hold_for_planner(now);
    if (elapsed_seconds(planner_hold_started_at_, now) > segment_timeout_s_) {
      abort_or_land("timed out waiting for a fresh planner trajectory");
    }
  }

  void track_planned_trajectory(const WaypointTracker::Clock::time_point now)
  {
    if (!flight_control_is_valid()) {
      return;
    }
    if (!planner_execution_is_safe(now)) {
      hold_for_planner(now);
      if (elapsed_seconds(planner_hold_started_at_, now) > segment_timeout_s_) {
        abort_or_land("planner trajectory or map remained unsafe during hold");
      }
      return;
    }
    planner_hold_active_ = false;
    planner_hold_target_.reset();
    auto & snapshot = latest_planned_trajectory_.value();
    if (snapshot.message.trajectory.points.empty()) {
      abort_or_land("planner supplied an empty trajectory");
      return;
    }

    const double elapsed = elapsed_seconds(snapshot.execution_started_at, now);
    if (!std::isfinite(elapsed) || elapsed < 0.0 ||
      elapsed > trajectory_duration_s(snapshot) + segment_timeout_s_)
    {
      abort_or_land("planned trajectory execution timed out");
      return;
    }

    while (snapshot.point_index + 1U < snapshot.message.trajectory.points.size() &&
      trajectory_point_time_s(snapshot.message.trajectory.points[snapshot.point_index + 1U]) <=
      elapsed)
    {
      ++snapshot.point_index;
    }
    if (snapshot.point_index != tracked_point_index_ ||
      snapshot.message.trajectory_id != tracked_trajectory_id_)
    {
      waypoint_tracker_.reset();
      tracked_point_index_ = snapshot.point_index;
      tracked_trajectory_id_ = snapshot.message.trajectory_id;
      target_started_at_ = now;
    }

    const auto & point = snapshot.message.trajectory.points[snapshot.point_index];
    const Position3D target = map_enu_to_ned(
      {
        point.transforms.front().translation.x,
        point.transforms.front().translation.y,
        point.transforms.front().translation.z});
    publish_planned_setpoint(point);

    if (snapshot.point_index + 1U == snapshot.message.trajectory.points.size() &&
      waypoint_tracker_.update(current_position_, target, now))
    {
      record_settled_error(target);
      if (mission_managed_landing_) {
        if (!planned_endpoint_reached_) {
          planned_endpoint_reached_ = true;
          RCLCPP_INFO(
            get_logger(),
            "Planned trajectory endpoint reached; holding for the mission manager");
        }
      } else {
        RCLCPP_INFO(get_logger(), "Planned trajectory endpoint reached; requesting landing");
        request_landing();
      }
    }
  }

  void hold_for_planner(const WaypointTracker::Clock::time_point now)
  {
    if (!planner_hold_active_) {
      planner_hold_active_ = true;
      planner_hold_started_at_ = now;
      planner_hold_target_ = current_position_;
    }
    publish_position_setpoint(planner_hold_target_.value());
  }

  static double trajectory_point_time_s(
    const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint & point)
  {
    return static_cast<double>(point.time_from_start.sec) +
           static_cast<double>(point.time_from_start.nanosec) / 1e9;
  }

  static double trajectory_duration_s(const PlannedTrajectorySnapshot & snapshot)
  {
    return trajectory_point_time_s(snapshot.message.trajectory.points.back());
  }

  void track_inspection_waypoint(const WaypointTracker::Clock::time_point now)
  {
    if (!flight_control_is_valid()) {
      return;
    }
    const Position3D & target = inspection_targets_.at(current_waypoint_index_);
    publish_position_setpoint(target);
    if (elapsed_seconds(target_started_at_, now) > segment_timeout_s_) {
      abort_or_land("inspection waypoint timed out");
      return;
    }
    if (!waypoint_tracker_.update(current_position_, target, now)) {
      return;
    }

    record_settled_error(target);
    RCLCPP_INFO(
      get_logger(), "Waypoint %zu/%zu reached", current_waypoint_index_ + 1,
      inspection_targets_.size());
    waypoint_tracker_.reset();
    ++current_waypoint_index_;
    target_started_at_ = now;
    if (current_waypoint_index_ == inspection_targets_.size()) {
      state_ = State::returning_home;
      RCLCPP_INFO(get_logger(), "Inspection waypoints complete; returning home");
    }
  }

  void track_return_home(const WaypointTracker::Clock::time_point now)
  {
    if (!flight_control_is_valid()) {
      return;
    }
    publish_position_setpoint(takeoff_target_);
    if (elapsed_seconds(target_started_at_, now) > segment_timeout_s_) {
      abort_or_land("return-to-home target timed out");
      return;
    }
    if (!waypoint_tracker_.update(current_position_, takeoff_target_, now)) {
      return;
    }

    record_settled_error(takeoff_target_);
    RCLCPP_INFO(get_logger(), "Home hover point reached; requesting landing");
    request_landing();
  }

  void request_landing()
  {
    state_ = State::landing;
    state_entered_at_ = WaypointTracker::Clock::now();
    last_command_sent_at_ = state_entered_at_;
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
  }

  void track_landing(const WaypointTracker::Clock::time_point now)
  {
    const bool status_is_fresh = sample_is_fresh(
      last_status_received_at_, now,
      std::chrono::duration_cast<WaypointTracker::Clock::duration>(
        std::chrono::duration<double>(status_timeout_s_)));
    const bool land_status_is_fresh = sample_is_fresh(
      last_land_status_received_at_, now,
      std::chrono::duration_cast<WaypointTracker::Clock::duration>(
        std::chrono::duration<double>(status_timeout_s_)));

    if (status_is_fresh &&
      arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED)
    {
      if (!disarm_observed_) {
        disarm_observed_ = true;
        disarm_observed_at_ = now;
      }
      if (land_status_is_fresh && landing_is_confirmed(true, landed_)) {
        state_ = State::complete;
        if (mission_failed_) {
          finish_mission(false, failure_reason_);
        } else {
          finish_mission(true, "vehicle landed and disarmed");
        }
      } else if (elapsed_seconds(disarm_observed_at_, now) > status_timeout_s_) {
        finish_mission(false, "vehicle disarmed without a fresh landed confirmation");
      }
      return;
    }

    disarm_observed_ = false;
    if (nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
      publish_offboard_heartbeat();
    }
    if (!landing_fallback_active_ &&
      elapsed_seconds(state_entered_at_, now) > landing_timeout_s_)
    {
      mark_mission_failed("landing timed out before disarm; continuing supervised Auto Land");
      landing_fallback_active_ = true;
      publish_auto_land_mode();
      last_command_sent_at_ = now;
    } else if (now - last_command_sent_at_ >= 1s) {
      publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
      if (landing_fallback_active_) {
        publish_auto_land_mode();
      }
      last_command_sent_at_ = now;
    }
  }

  bool flight_control_is_valid()
  {
    if (arming_state_ != px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
      finish_mission(false, "vehicle disarmed before mission completion");
      return false;
    }
    if (nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
      abort_or_land("vehicle left Offboard mode");
      return false;
    }
    return true;
  }

  void send_activation_commands(const WaypointTracker::Clock::time_point now)
  {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
    last_command_sent_at_ = now;
  }

  void handle_command_ack(const px4_msgs::msg::VehicleCommandAck & message)
  {
    const bool activation_command =
      message.command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE ||
      message.command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    const bool landing_command = state_ == State::landing &&
      (message.command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND ||
      (landing_fallback_active_ &&
      message.command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE));
    if ((state_ != State::requesting_offboard || !activation_command) && !landing_command) {
      return;
    }

    if (command_ack_is_accepted(message.result)) {
      return;
    }
    if (message.result ==
      px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED)
    {
      RCLCPP_WARN(
        get_logger(), "PX4 temporarily rejected command %u; retrying",
        static_cast<unsigned int>(message.command));
      return;
    }

    RCLCPP_ERROR(
      get_logger(), "PX4 rejected command %u with result %u",
      static_cast<unsigned int>(message.command), static_cast<unsigned int>(message.result));
    if (landing_command) {
      mark_mission_failed("PX4 rejected a landing command; continuing supervised Auto Land");
      if (!landing_fallback_active_) {
        landing_fallback_active_ = true;
        publish_auto_land_mode();
      }
    } else {
      command_rejected_ = true;
    }
  }

  static bool command_ack_is_accepted(const std::uint8_t result)
  {
    return result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED ||
           result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS;
  }

  void abort_or_land(const std::string & reason)
  {
    if (state_ == State::landing || state_ == State::complete) {
      return;
    }

    mark_mission_failed(reason);
    RCLCPP_ERROR(get_logger(), "%s; aborting mission", reason.c_str());
    if (arming_state_ == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
      request_landing();
    } else {
      finish_mission(false, reason);
    }
  }

  void mark_mission_failed(const std::string & reason)
  {
    if (mission_failed_) {
      return;
    }
    mission_failed_ = true;
    failure_reason_ = reason;
    RCLCPP_ERROR(get_logger(), "Mission marked failed: %s", reason.c_str());
  }

  void record_settled_error(const Position3D & target)
  {
    const double error = euclidean_distance(current_position_, target);
    settled_error_sum_m_ += error;
    settled_error_max_m_ = std::max(settled_error_max_m_, error);
    ++settled_error_count_;
    RCLCPP_INFO(
      get_logger(), "Settled target error: %.3f m (sample %zu)", error, settled_error_count_);
  }

  void finish_mission(const bool succeeded, const std::string & reason)
  {
    if (mission_finished_) {
      return;
    }
    mission_finished_ = true;
    mission_succeeded_ = succeeded;
    state_ = State::complete;
    timer_->cancel();

    if (settled_error_count_ > 0) {
      const double mean_error = settled_error_sum_m_ / static_cast<double>(settled_error_count_);
      RCLCPP_INFO(
        get_logger(), "Settled target error summary: samples=%zu mean=%.3f m max=%.3f m",
        settled_error_count_, mean_error, settled_error_max_m_);
    }
    if (succeeded) {
      RCLCPP_INFO(get_logger(), "Mission complete: %s", reason.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Mission failed: %s", reason.c_str());
    }
    rclcpp::shutdown();
  }

  static double elapsed_seconds(
    const WaypointTracker::Clock::time_point start,
    const WaypointTracker::Clock::time_point end)
  {
    return std::chrono::duration<double>(end - start).count();
  }

  std::string stale_telemetry_reason(const WaypointTracker::Clock::time_point now) const
  {
    const auto position_timeout = std::chrono::duration_cast<WaypointTracker::Clock::duration>(
      std::chrono::duration<double>(position_timeout_s_));
    const auto status_timeout = std::chrono::duration_cast<WaypointTracker::Clock::duration>(
      std::chrono::duration<double>(status_timeout_s_));

    if (!sample_is_fresh(last_position_received_at_, now, position_timeout)) {
      return "PX4 local position telemetry became stale";
    }
    if (!sample_is_fresh(last_status_received_at_, now, status_timeout)) {
      return "PX4 vehicle status telemetry became stale";
    }
    if (!sample_is_fresh(last_land_status_received_at_, now, status_timeout)) {
      return "PX4 land status telemetry became stale";
    }
    return {};
  }

  void publish_auto_land_mode()
  {
    constexpr float px4_custom_main_mode_auto = 4.0F;
    constexpr float px4_custom_sub_mode_auto_land = 6.0F;
    publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F,
      px4_custom_main_mode_auto, px4_custom_sub_mode_auto_land);
  }

  void publish_position_setpoint(const Position3D & target)
  {
    publish_offboard_heartbeat();

    const float unused = std::numeric_limits<float>::quiet_NaN();
    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.position = {
      static_cast<float>(target.x), static_cast<float>(target.y), static_cast<float>(target.z)};
    setpoint.velocity = {unused, unused, unused};
    setpoint.acceleration = {unused, unused, unused};
    setpoint.jerk = {unused, unused, unused};
    setpoint.yaw = commanded_yaw_rad_;
    setpoint.yawspeed = unused;
    setpoint.timestamp = timestamp_us();
    trajectory_publisher_->publish(setpoint);
  }

  void publish_planned_setpoint(
    const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint & point)
  {
    publish_offboard_heartbeat(true, true, true);
    const auto & transform = point.transforms.front();
    const auto & velocity = point.velocities.front().linear;
    const auto & acceleration = point.accelerations.front().linear;
    const Position3D position = map_enu_to_ned(
      {
        transform.translation.x, transform.translation.y, transform.translation.z});
    const Position3D ned_velocity = map_enu_to_ned({velocity.x, velocity.y, velocity.z});
    const Position3D ned_acceleration = map_enu_to_ned(
      {
        acceleration.x, acceleration.y, acceleration.z});
    px4_msgs::msg::TrajectorySetpoint setpoint{};
    setpoint.position = {
      static_cast<float>(position.x), static_cast<float>(position.y),
      static_cast<float>(position.z)};
    setpoint.velocity = {
      static_cast<float>(ned_velocity.x), static_cast<float>(ned_velocity.y),
      static_cast<float>(ned_velocity.z)};
    setpoint.acceleration = {
      static_cast<float>(ned_acceleration.x), static_cast<float>(ned_acceleration.y),
      static_cast<float>(ned_acceleration.z)};
    const float unused = std::numeric_limits<float>::quiet_NaN();
    setpoint.jerk = {unused, unused, unused};
    setpoint.yaw = commanded_yaw_rad_;
    setpoint.yawspeed = unused;
    setpoint.timestamp = timestamp_us();
    trajectory_publisher_->publish(setpoint);
  }

  void publish_offboard_heartbeat(
    const bool position = true, const bool velocity = false, const bool acceleration = false)
  {
    px4_msgs::msg::OffboardControlMode control_mode{};
    control_mode.position = position;
    control_mode.velocity = velocity;
    control_mode.acceleration = acceleration;
    control_mode.timestamp = timestamp_us();
    offboard_mode_publisher_->publish(control_mode);
  }

  void publish_vehicle_command(
    const std::uint32_t command, const float param1 = 0.0F, const float param2 = 0.0F,
    const float param3 = 0.0F)
  {
    px4_msgs::msg::VehicleCommand message{};
    message.param1 = param1;
    message.param2 = param2;
    message.param3 = param3;
    message.command = command;
    message.target_system = 1;
    message.target_component = 1;
    message.source_system = 1;
    message.source_component = 1;
    message.from_external = true;
    message.timestamp = timestamp_us();
    vehicle_command_publisher_->publish(message);
  }

  std::uint64_t timestamp_us()
  {
    return static_cast<std::uint64_t>(get_clock()->now().nanoseconds() / 1000);
  }

  const double takeoff_altitude_m_;
  const double acceptance_radius_m_;
  const double stable_time_s_;
  const int warmup_setpoint_count_;
  const double startup_timeout_s_;
  const double command_timeout_s_;
  const double segment_timeout_s_;
  const double landing_timeout_s_;
  const double position_timeout_s_;
  const double status_timeout_s_;
  const std::string map_frame_;
  const bool use_planned_trajectory_;
  const bool mission_managed_landing_;
  const std::string planned_trajectory_topic_;
  const double planner_stale_timeout_s_;
  const std::vector<double> waypoint_offsets_xy_;

  WaypointTracker waypoint_tracker_;
  State state_{State::waiting_for_vehicle};
  Position3D current_position_{};
  Position3D home_position_{};
  Position3D takeoff_target_{};
  std::vector<Position3D> inspection_targets_;
  std::size_t current_waypoint_index_{0};
  std::uint64_t tracked_trajectory_id_{0U};
  std::size_t tracked_point_index_{std::numeric_limits<std::size_t>::max()};
  int warmup_counter_{0};
  std::uint8_t arming_state_{0};
  std::uint8_t nav_state_{0};
  bool position_received_{false};
  bool status_received_{false};
  bool land_status_received_{false};
  bool landed_{false};
  bool failsafe_{false};
  bool pre_flight_checks_pass_{false};
  bool command_rejected_{false};
  bool mission_failed_{false};
  bool mission_finished_{false};
  bool mission_succeeded_{false};
  bool landing_fallback_active_{false};
  bool disarm_observed_{false};
  double settled_error_sum_m_{0.0};
  double settled_error_max_m_{0.0};
  std::size_t settled_error_count_{0};
  float commanded_yaw_rad_{0.0F};
  std::string failure_reason_;
  bool planner_status_received_{false};
  bool planner_status_safe_{false};
  bool planner_hold_active_{false};
  bool insertion_hold_active_{false};
  bool planner_map_fresh_{false};
  bool planner_trajectory_valid_{false};
  bool planned_endpoint_reached_{false};
  std::uint8_t planner_state_{drone_interfaces::msg::PlannerStatus::WAITING_FOR_MAP};
  std::uint64_t planner_status_trajectory_id_{0U};
  std::optional<PlannedTrajectorySnapshot> latest_planned_trajectory_;
  std::optional<Position3D> planner_hold_target_;
  std::optional<Position3D> insertion_hold_target_;
  WaypointTracker::Clock::time_point node_started_at_{WaypointTracker::Clock::now()};
  WaypointTracker::Clock::time_point state_entered_at_{node_started_at_};
  WaypointTracker::Clock::time_point target_started_at_{node_started_at_};
  WaypointTracker::Clock::time_point last_command_sent_at_{node_started_at_};
  WaypointTracker::Clock::time_point last_position_received_at_{node_started_at_};
  WaypointTracker::Clock::time_point last_status_received_at_{node_started_at_};
  WaypointTracker::Clock::time_point last_land_status_received_at_{node_started_at_};
  WaypointTracker::Clock::time_point disarm_observed_at_{node_started_at_};
  WaypointTracker::Clock::time_point planner_status_received_at_{node_started_at_};
  WaypointTracker::Clock::time_point planner_hold_started_at_{node_started_at_};
  WaypointTracker::Clock::time_point insertion_hold_started_at_{node_started_at_};

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr
    land_detected_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr command_ack_subscription_;
  rclcpp::Subscription<drone_interfaces::msg::PlannedTrajectory>::SharedPtr
    planned_trajectory_subscription_;
  rclcpp::Subscription<drone_interfaces::msg::PlannerStatus>::SharedPtr
    planner_status_subscription_;
  rclcpp::Subscription<drone_interfaces::msg::MissionCommand>::SharedPtr
    mission_command_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr insertion_hold_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drone_controller

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    const auto node = std::make_shared<drone_controller::WaypointController>();
    rclcpp::spin(node);
    const bool mission_succeeded = node->mission_succeeded();
    rclcpp::shutdown();
    return mission_succeeded ? 0 : 1;
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("waypoint_controller"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
}
