#include "drone_perception/depth_grid_mapper.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace drone_perception
{

using SteadyClock = std::chrono::steady_clock;
constexpr std::size_t kMaximumDepthPixels = 4'000'000;

std::string fixed_precision(const double value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  return stream.str();
}

diagnostic_msgs::msg::KeyValue diagnostic_value(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

Pose3D pose_from_transform(const geometry_msgs::msg::TransformStamped & transform)
{
  return Pose3D{
    transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z,
    transform.transform.rotation.x,
    transform.transform.rotation.y,
    transform.transform.rotation.z,
    transform.transform.rotation.w,
  };
}

class DepthGridNode : public rclcpp::Node
{
public:
  DepthGridNode()
  : Node("depth_grid_node"),
    map_frame_(declare_parameter<std::string>("map_frame", "map")),
    base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
    tf_timeout_s_(declare_parameter<double>("tf_timeout_s", 0.1))
  {
    mapper_config_.resolution_m = declare_parameter<double>("resolution_m", 0.1);
    mapper_config_.width_m = declare_parameter<double>("width_m", 12.0);
    mapper_config_.height_m = declare_parameter<double>("height_m", 12.0);
    mapper_config_.min_depth_m = declare_parameter<double>("min_depth_m", 0.2);
    mapper_config_.max_depth_m = declare_parameter<double>("max_depth_m", 10.0);
    mapper_config_.min_relative_height_m =
      declare_parameter<double>("min_relative_height_m", -0.5);
    mapper_config_.max_relative_height_m =
      declare_parameter<double>("max_relative_height_m", 0.5);
    mapper_config_.pixel_stride = declare_parameter<int>("pixel_stride", 2);
    validate_mapper_config(mapper_config_);
    if (map_frame_.empty() || base_frame_.empty() ||
      !std::isfinite(tf_timeout_s_) || tf_timeout_s_ <= 0.0)
    {
      throw std::invalid_argument("frame names and TF timeout must be valid");
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    grid_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/local_occupancy_grid", rclcpp::QoS(1).reliable().transient_local());
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/drone_perception/diagnostics", 10);
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/depth/camera_info", rclcpp::SensorDataQoS(),
      std::bind(&DepthGridNode::handle_camera_info, this, std::placeholders::_1));
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&DepthGridNode::handle_depth, this, std::placeholders::_1));
  }

private:
  void handle_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr message)
  {
    const CameraIntrinsics intrinsics{message->k[0], message->k[4], message->k[2], message->k[5]};
    if (message->width == 0U || message->height == 0U ||
      message->height > kMaximumDepthPixels / message->width ||
      message->header.frame_id.empty() ||
      !std::isfinite(intrinsics.fx) || !std::isfinite(intrinsics.fy) ||
      !std::isfinite(intrinsics.cx) || !std::isfinite(intrinsics.cy) ||
      intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Ignoring invalid depth camera intrinsics");
      return;
    }
    intrinsics_ = intrinsics;
    camera_width_ = message->width;
    camera_height_ = message->height;
    camera_frame_ = message->header.frame_id;
  }

  bool image_layout_is_valid(const sensor_msgs::msg::Image & image) const
  {
    const std::uint16_t endian_probe = 1U;
    const bool host_is_big_endian =
      *reinterpret_cast<const std::uint8_t *>(&endian_probe) == 0U;
    const std::size_t row_bytes = static_cast<std::size_t>(image.width) * sizeof(float);
    return image.encoding == "32FC1" &&
           image.width > 0U && image.height > 0U &&
           image.height <= kMaximumDepthPixels / image.width &&
           static_cast<bool>(image.is_bigendian) == host_is_big_endian &&
           image.step >= row_bytes &&
           image.height <= std::numeric_limits<std::size_t>::max() / image.step &&
           image.data.size() >= static_cast<std::size_t>(image.height) * image.step;
  }

  void handle_depth(const sensor_msgs::msg::Image::SharedPtr message)
  {
    const auto processing_started_at = SteadyClock::now();
    if (!intrinsics_.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for valid depth camera info");
      return;
    }
    if (message->width != camera_width_ || message->height != camera_height_ ||
      message->header.frame_id != camera_frame_ || !image_layout_is_valid(*message))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring depth image with mismatched camera contract or unsupported layout");
      return;
    }

    geometry_msgs::msg::TransformStamped camera_transform;
    geometry_msgs::msg::TransformStamped base_transform;
    try {
      const rclcpp::Time image_time(message->header.stamp);
      const rclcpp::Duration timeout = rclcpp::Duration::from_seconds(tf_timeout_s_);
      camera_transform = tf_buffer_->lookupTransform(
        map_frame_, message->header.frame_id, image_time, timeout);
      base_transform = tf_buffer_->lookupTransform(map_frame_, base_frame_, image_time, timeout);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for depth TF: %s", error.what());
      return;
    }

    std::vector<float> depth(static_cast<std::size_t>(message->width) * message->height);
    const std::size_t row_bytes = static_cast<std::size_t>(message->width) * sizeof(float);
    for (std::uint32_t row = 0; row < message->height; ++row) {
      std::memcpy(
        depth.data() + static_cast<std::size_t>(row) * message->width,
        message->data.data() + static_cast<std::size_t>(row) * message->step,
        row_bytes);
    }

    GridData grid;
    try {
      grid = build_grid(
        depth, message->width, message->height, intrinsics_.value(),
        pose_from_transform(camera_transform), pose_from_transform(base_transform), mapper_config_);
    } catch (const std::invalid_argument & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Depth grid rejected input: %s", error.what());
      return;
    }

    nav_msgs::msg::OccupancyGrid output;
    output.header.stamp = message->header.stamp;
    output.header.frame_id = map_frame_;
    output.info.map_load_time = message->header.stamp;
    output.info.resolution = static_cast<float>(grid.resolution_m);
    output.info.width = grid.width;
    output.info.height = grid.height;
    output.info.origin.position.x = grid.origin_x;
    output.info.origin.position.y = grid.origin_y;
    output.info.origin.orientation.w = 1.0;
    output.data = std::move(grid.cells);
    grid_publisher_->publish(output);

    const auto published_at = SteadyClock::now();
    publish_times_.push_back(published_at);
    while (publish_times_.size() > 30U) {
      publish_times_.pop_front();
    }
    const double output_rate_hz = calculate_output_rate();
    const double processing_latency_ms =
      std::chrono::duration<double, std::milli>(published_at - processing_started_at).count();
    publish_diagnostics(
      message->header.stamp, processing_latency_ms, output_rate_hz,
      grid.used_depth_count, grid.occupied_cell_count);
  }

  double calculate_output_rate() const
  {
    if (publish_times_.size() < 2U) {
      return 0.0;
    }
    const double elapsed_seconds =
      std::chrono::duration<double>(publish_times_.back() - publish_times_.front()).count();
    return elapsed_seconds > 0.0 ?
           static_cast<double>(publish_times_.size() - 1U) / elapsed_seconds : 0.0;
  }

  void publish_diagnostics(
    const builtin_interfaces::msg::Time & stamp,
    const double processing_latency_ms,
    const double output_rate_hz,
    const std::size_t used_depth_count,
    const std::size_t occupied_cell_count)
  {
    diagnostic_msgs::msg::DiagnosticArray diagnostics;
    diagnostics.header.stamp = stamp;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "drone_perception/local_grid";
    status.hardware_id = "gazebo_depth_camera";
    if (publish_times_.size() < 2U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "warming up output-rate window";
    } else if (output_rate_hz < 5.0) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "output rate below 5 Hz";
    } else if (occupied_cell_count == 0U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "no occupied cells in current depth slice";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "local grid healthy";
    }
    status.values.push_back(
      diagnostic_value("processing_latency_ms", fixed_precision(processing_latency_ms)));
    status.values.push_back(diagnostic_value("output_rate_hz", fixed_precision(output_rate_hz)));
    status.values.push_back(diagnostic_value("used_depth_count", std::to_string(used_depth_count)));
    status.values.push_back(
      diagnostic_value("occupied_cell_count", std::to_string(occupied_cell_count)));
    diagnostics.status.push_back(std::move(status));
    diagnostics_publisher_->publish(diagnostics);
  }

  std::string map_frame_;
  std::string base_frame_;
  double tf_timeout_s_;
  MapperConfig mapper_config_;
  std::optional<CameraIntrinsics> intrinsics_;
  std::uint32_t camera_width_{};
  std::uint32_t camera_height_{};
  std::string camera_frame_;
  std::deque<SteadyClock::time_point> publish_times_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
};

}  // namespace drone_perception

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<drone_perception::DepthGridNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("depth_grid_node"), "Fatal error: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
