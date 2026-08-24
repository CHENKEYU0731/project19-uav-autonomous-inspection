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

#include "drone_planner/a_star.hpp"
#include "drone_planner/grid_map.hpp"
#include "drone_planner/trajectory.hpp"

#include <drone_interfaces/msg/planned_trajectory.hpp>
#include <drone_interfaces/msg/planner_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace drone_planner
{
namespace
{

using namespace std::chrono_literals;
using SteadyClock = std::chrono::steady_clock;
using PlannedTrajectory = drone_interfaces::msg::PlannedTrajectory;
using PlannerStatus = drone_interfaces::msg::PlannerStatus;

constexpr double kIdentityQuaternionTolerance = 1e-6;

std::string normalize_frame(std::string frame)
{
  while (!frame.empty() && frame.front() == '/') {
    frame.erase(frame.begin());
  }
  return frame;
}

bool finite_point(const Point2D & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool identity_orientation(const geometry_msgs::msg::Quaternion & orientation)
{
  return std::abs(orientation.x) <= kIdentityQuaternionTolerance &&
         std::abs(orientation.y) <= kIdentityQuaternionTolerance &&
         std::abs(orientation.z) <= kIdentityQuaternionTolerance &&
         std::abs(orientation.w - 1.0) <= kIdentityQuaternionTolerance;
}

bool valid_duration(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode()
  : Node("planner_node"),
    map_frame_(declare_parameter<std::string>("map_frame", "map")),
    base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
    goal_x_m_(declare_parameter<double>("goal_x_m", 0.0)),
    goal_y_m_(declare_parameter<double>("goal_y_m", 3.0)),
    goal_z_m_(declare_parameter<double>("goal_z_m", 2.5)),
    footprint_radius_m_(declare_parameter<double>("footprint_radius_m", 0.35)),
    inflation_radius_m_(declare_parameter<double>("inflation_radius_m", 0.50)),
    minimum_altitude_m_(declare_parameter<double>("minimum_altitude_m", 2.0)),
    maximum_map_age_s_(declare_parameter<double>("maximum_map_age_s", 0.5)),
    tf_timeout_s_(declare_parameter<double>("tf_timeout_s", 0.1)),
    maximum_velocity_m_s_(declare_parameter<double>("maximum_velocity_m_s", 1.0)),
    maximum_acceleration_m_s2_(
      declare_parameter<double>("maximum_acceleration_m_s2", 1.0)),
    sample_period_s_(declare_parameter<double>("sample_period_s", 0.1))
  {
    validate_parameters();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_occupancy_grid", map_qos,
      std::bind(&PlannerNode::handle_map, this, std::placeholders::_1));
    goal_subscription_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/drone_planner/goal", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&PlannerNode::handle_goal, this, std::placeholders::_1));
    trajectory_publisher_ = create_publisher<PlannedTrajectory>(
      "/drone_planner/trajectory", rclcpp::QoS(1).reliable().transient_local());
    status_publisher_ = create_publisher<PlannerStatus>(
      "/drone_planner/status", rclcpp::QoS(10).reliable());
    timer_ = create_wall_timer(100ms, std::bind(&PlannerNode::plan_tick, this));
    publish_status(PlannerStatus::WAITING_FOR_MAP, false, false, "waiting for a local map");
  }

private:
  void validate_parameters() const
  {
    if (map_frame_.empty() || base_frame_.empty()) {
      throw std::invalid_argument("planner frame names must not be empty");
    }
    if (!std::isfinite(goal_x_m_) || !std::isfinite(goal_y_m_) ||
      !std::isfinite(goal_z_m_) || goal_z_m_ <= 0.0)
    {
      throw std::invalid_argument("planner goal must be finite and above the map plane");
    }
    for (const auto value : {
        footprint_radius_m_, inflation_radius_m_, minimum_altitude_m_,
        maximum_map_age_s_, tf_timeout_s_, maximum_velocity_m_s_,
        maximum_acceleration_m_s2_, sample_period_s_})
    {
      if (!valid_duration(value)) {
        throw std::invalid_argument("planner numeric parameters must be finite and positive");
      }
    }
    if (minimum_altitude_m_ > goal_z_m_) {
      throw std::invalid_argument("minimum_altitude_m must not exceed goal_z_m");
    }
    if (maximum_map_age_s_ > 10.0 || tf_timeout_s_ > 10.0) {
      throw std::invalid_argument("planner freshness and TF timeouts exceed the safety limit");
    }
  }

  void handle_goal(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    if (normalize_frame(message->header.frame_id) != map_frame_ ||
      !std::isfinite(message->pose.position.x) ||
      !std::isfinite(message->pose.position.y) ||
      !std::isfinite(message->pose.position.z) || message->pose.position.z <= 0.0)
    {
      RCLCPP_WARN(get_logger(), "Rejecting goal with an invalid frame or position");
      return;
    }
    goal_x_m_ = message->pose.position.x;
    goal_y_m_ = message->pose.position.y;
    goal_z_m_ = message->pose.position.z;
    goal_received_ = true;
    RCLCPP_INFO(
      get_logger(), "Accepted goal in %s: [%.2f, %.2f, %.2f]",
      map_frame_.c_str(), goal_x_m_, goal_y_m_, goal_z_m_);
  }

  void handle_map(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    if (normalize_frame(message->header.frame_id) != map_frame_ ||
      message->header.stamp.sec < 0 ||
      (message->info.width == 0U || message->info.height == 0U) ||
      message->info.width > std::numeric_limits<std::size_t>::max() / message->info.height ||
      message->data.size() !=
      static_cast<std::size_t>(message->info.width) * message->info.height ||
      !std::isfinite(message->info.resolution) || message->info.resolution <= 0.0 ||
      !std::isfinite(message->info.origin.position.x) ||
      !std::isfinite(message->info.origin.position.y) ||
      !identity_orientation(message->info.origin.orientation))
    {
      publish_status(
        PlannerStatus::INVALID_MAP, false, false,
        "map frame, timestamp, geometry, or origin orientation is invalid");
      return;
    }

    try {
      const GridGeometry geometry{
        message->info.resolution,
        message->info.width,
        message->info.height,
        message->info.origin.position.x,
        message->info.origin.position.y,
      };
      const std::vector<std::int8_t> cells(message->data.begin(), message->data.end());
      GridMap observation(geometry, cells);
      if (!fused_map_) {
        fused_map_ = std::make_unique<GridMap>(geometry, cells);
      } else {
        fused_map_->integrate(observation);
      }
      latest_map_stamp_ = rclcpp::Time(
        message->header.stamp, get_clock()->get_clock_type());
      latest_map_received_at_ = SteadyClock::now();
      map_received_ = true;
    } catch (const std::exception & error) {
      publish_status(PlannerStatus::INVALID_MAP, false, false, error.what());
    }
  }

  bool map_is_fresh()
  {
    if (!map_received_ || latest_map_stamp_.nanoseconds() <= 0) {
      return false;
    }
    const rclcpp::Time now = get_clock()->now();
    if (now < latest_map_stamp_) {
      return false;
    }
    return (now - latest_map_stamp_).seconds() <= maximum_map_age_s_ &&
           std::chrono::duration<double>(SteadyClock::now() - latest_map_received_at_).count() <=
           maximum_map_age_s_ + tf_timeout_s_;
  }

  void plan_tick()
  {
    if (!map_received_) {
      publish_status(PlannerStatus::WAITING_FOR_MAP, false, false, "waiting for a local map");
      return;
    }
    if (!map_is_fresh()) {
      publish_status(PlannerStatus::STALE_MAP, false, false, "local map exceeded age limit");
      return;
    }
    if (!goal_received_) {
      publish_status(PlannerStatus::WAITING_FOR_MAP, true, false, "waiting for a goal");
      return;
    }

    geometry_msgs::msg::TransformStamped base_transform;
    try {
      base_transform = tf_buffer_->lookupTransform(
        map_frame_, base_frame_, latest_map_stamp_,
        rclcpp::Duration::from_seconds(tf_timeout_s_));
    } catch (const tf2::TransformException & error) {
      publish_status(PlannerStatus::WAITING_FOR_TF, true, false, error.what());
      return;
    }

    const Point2D current_position{
      base_transform.transform.translation.x,
      base_transform.transform.translation.y,
    };
    const double current_altitude_m = base_transform.transform.translation.z;
    if (!finite_point(current_position) ||
      !std::isfinite(current_altitude_m))
    {
      publish_status(PlannerStatus::WAITING_FOR_TF, true, false, "TF pose is non-finite");
      return;
    }
    if (!initial_altitude_m_.has_value()) {
      initial_altitude_m_ = current_altitude_m;
    }
    if (current_altitude_m - initial_altitude_m_.value() < minimum_altitude_m_) {
      publish_status(
        PlannerStatus::WAITING_FOR_AIRBORNE, true, false,
        "waiting until the vehicle reaches the minimum relative altitude");
      return;
    }

    try {
      GridMap candidate = *fused_map_;
      candidate.clear_disk(current_position, footprint_radius_m_);
      const InflatedGrid inflated(candidate, inflation_radius_m_);
      const auto start = candidate.world_to_cell(current_position);
      const auto goal = candidate.world_to_cell(Point2D{goal_x_m_, goal_y_m_});
      if (!start.has_value() || !goal.has_value()) {
        publish_status(PlannerStatus::NO_PATH, true, false, "start or goal is outside the map");
        return;
      }
      const bool goal_reached = start.value() == goal.value();
      if (!active_grid_path_.empty() && active_grid_path_.back() == goal.value() &&
        !active_trajectory_.empty() &&
        remaining_path_is_safe(inflated, active_grid_path_, start.value()))
      {
        publish_trajectory(active_trajectory_);
        publish_status(
          goal_reached ? PlannerStatus::GOAL_REACHED : PlannerStatus::READY,
          true, true,
          goal_reached ? "goal cell reached; holding endpoint" :
          "active trajectory remains collision-free");
        return;
      }
      const auto result = plan_a_star(inflated, start.value(), goal.value());
      if (!result.has_value()) {
        publish_status(
          PlannerStatus::NO_PATH, true, false,
          "goal is unreachable in the inflated map");
        return;
      }
      const auto pruned_path = prune_path(inflated, result->path);
      const std::vector<Point2D> waypoints = cell_centers(candidate, pruned_path);
      const KinematicLimits limits{
        maximum_velocity_m_s_, maximum_acceleration_m_s2_, sample_period_s_};
      const auto samples = goal_reached ?
        parameterize_hold(waypoints.front(), limits) :
        parameterize_quintic(waypoints, limits);
      if (samples.empty()) {
        publish_status(
          PlannerStatus::NO_PATH, true, false,
          "trajectory parameterization returned no samples");
        return;
      }
      ++trajectory_id_;
      active_grid_path_ = result->path;
      active_trajectory_ = samples;
      publish_trajectory(active_trajectory_);
      publish_status(
        goal_reached ? PlannerStatus::GOAL_REACHED : PlannerStatus::READY,
        true, true,
        goal_reached ? "goal cell reached; holding endpoint" :
        "fresh collision-checked trajectory");
    } catch (const std::exception & error) {
      publish_status(PlannerStatus::INVALID_MAP, true, false, error.what());
    }
  }

  static std::vector<Point2D> cell_centers(
    const GridMap & map, const std::vector<GridCell> & path)
  {
    std::vector<Point2D> result;
    result.reserve(path.size());
    for (const auto & cell : path) {
      result.push_back(map.cell_center(cell));
    }
    return result;
  }

  void publish_trajectory(const std::vector<TrajectorySample2D> & samples)
  {
    const auto stamp = get_clock()->now();
    PlannedTrajectory message;
    message.trajectory_id = trajectory_id_;
    message.created_at = stamp;
    message.frame_id = map_frame_;
    message.trajectory.header.stamp = stamp;
    message.trajectory.header.frame_id = map_frame_;
    message.trajectory.points.reserve(samples.size());
    for (const auto & sample : samples) {
      trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
      geometry_msgs::msg::Transform transform;
      transform.translation.x = sample.position.x;
      transform.translation.y = sample.position.y;
      transform.translation.z = goal_z_m_;
      transform.rotation.w = 1.0;
      point.transforms.push_back(transform);

      geometry_msgs::msg::Twist velocity;
      velocity.linear.x = sample.velocity.x;
      velocity.linear.y = sample.velocity.y;
      velocity.linear.z = 0.0;
      point.velocities.push_back(velocity);

      geometry_msgs::msg::Twist acceleration;
      acceleration.linear.x = sample.acceleration.x;
      acceleration.linear.y = sample.acceleration.y;
      acceleration.linear.z = 0.0;
      point.accelerations.push_back(acceleration);
      const auto duration_ns = rclcpp::Duration::from_seconds(
        sample.time_from_start_s).nanoseconds();
      point.time_from_start.sec = static_cast<std::int32_t>(duration_ns / 1'000'000'000LL);
      point.time_from_start.nanosec = static_cast<std::uint32_t>(
        duration_ns % 1'000'000'000LL);
      message.trajectory.points.push_back(std::move(point));
    }
    trajectory_publisher_->publish(std::move(message));
  }

  void publish_status(
    const std::uint8_t state, const bool map_fresh, const bool trajectory_valid,
    const std::string & reason)
  {
    PlannerStatus message;
    message.state = state;
    message.trajectory_id = trajectory_id_;
    message.stamp = get_clock()->now();
    message.map_fresh = map_fresh;
    message.trajectory_valid = trajectory_valid;
    message.reason = reason;
    status_publisher_->publish(std::move(message));
  }

  std::string map_frame_;
  std::string base_frame_;
  double goal_x_m_;
  double goal_y_m_;
  double goal_z_m_;
  double footprint_radius_m_;
  double inflation_radius_m_;
  double minimum_altitude_m_;
  double maximum_map_age_s_;
  double tf_timeout_s_;
  double maximum_velocity_m_s_;
  double maximum_acceleration_m_s2_;
  double sample_period_s_;
  bool goal_received_{true};
  bool map_received_{false};
  std::uint64_t trajectory_id_{0U};
  std::optional<double> initial_altitude_m_;
  rclcpp::Time latest_map_stamp_{0, 0, RCL_ROS_TIME};
  SteadyClock::time_point latest_map_received_at_{SteadyClock::now()};
  std::unique_ptr<GridMap> fused_map_;
  std::vector<GridCell> active_grid_path_;
  std::vector<TrajectorySample2D> active_trajectory_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscription_;
  rclcpp::Publisher<PlannedTrajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<PlannerStatus>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace drone_planner

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<drone_planner::PlannerNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("planner_node"), "Fatal error: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
