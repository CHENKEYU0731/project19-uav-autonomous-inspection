#include "drone_controller/waypoint_tracker.hpp"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
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
    inspecting,
    returning_home,
    landing,
    complete,
  };

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
      case State::inspecting:
        track_inspection_waypoint(now);
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
    state_ = State::inspecting;
    RCLCPP_INFO(
      get_logger(), "Takeoff target reached; starting waypoint 1/%zu",
      inspection_targets_.size());
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
    setpoint.yaw = 0.0F;
    setpoint.yawspeed = unused;
    setpoint.timestamp = timestamp_us();
    trajectory_publisher_->publish(setpoint);
  }

  void publish_offboard_heartbeat()
  {
    px4_msgs::msg::OffboardControlMode control_mode{};
    control_mode.position = true;
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
  const std::vector<double> waypoint_offsets_xy_;

  WaypointTracker waypoint_tracker_;
  State state_{State::waiting_for_vehicle};
  Position3D current_position_{};
  Position3D home_position_{};
  Position3D takeoff_target_{};
  std::vector<Position3D> inspection_targets_;
  std::size_t current_waypoint_index_{0};
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
  std::string failure_reason_;
  WaypointTracker::Clock::time_point node_started_at_{WaypointTracker::Clock::now()};
  WaypointTracker::Clock::time_point state_entered_at_{node_started_at_};
  WaypointTracker::Clock::time_point target_started_at_{node_started_at_};
  WaypointTracker::Clock::time_point last_command_sent_at_{node_started_at_};
  WaypointTracker::Clock::time_point last_position_received_at_{node_started_at_};
  WaypointTracker::Clock::time_point last_status_received_at_{node_started_at_};
  WaypointTracker::Clock::time_point last_land_status_received_at_{node_started_at_};
  WaypointTracker::Clock::time_point disarm_observed_at_{node_started_at_};

  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr
    land_detected_subscription_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr command_ack_subscription_;
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
