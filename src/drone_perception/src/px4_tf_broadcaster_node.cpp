#include "drone_perception/frame_conversions.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

namespace drone_perception
{

class Px4TfBroadcaster : public rclcpp::Node
{
public:
  Px4TfBroadcaster()
  : Node("px4_tf_broadcaster"),
    map_frame_(declare_parameter<std::string>("map_frame", "map")),
    base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
    camera_frame_(declare_parameter<std::string>("camera_frame", "camera_optical_frame")),
    camera_x_m_(declare_parameter<double>("camera_x_m", 0.13233)),
    camera_y_m_(declare_parameter<double>("camera_y_m", 0.0)),
    camera_z_m_(declare_parameter<double>("camera_z_m", 0.26078))
  {
    if (map_frame_.empty() || base_frame_.empty() || camera_frame_.empty() ||
      !std::isfinite(camera_x_m_) || !std::isfinite(camera_y_m_) ||
      !std::isfinite(camera_z_m_))
    {
      throw std::invalid_argument("TF frame names and camera translation must be valid");
    }

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_transform_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    publish_camera_transform();

    odometry_subscription_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
      std::bind(&Px4TfBroadcaster::handle_odometry, this, std::placeholders::_1));
  }

private:
  void publish_camera_transform()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = base_frame_;
    transform.child_frame_id = camera_frame_;
    transform.transform.translation.x = camera_x_m_;
    transform.transform.translation.y = camera_y_m_;
    transform.transform.translation.z = camera_z_m_;
    transform.transform.rotation.x = -0.5;
    transform.transform.rotation.y = 0.5;
    transform.transform.rotation.z = -0.5;
    transform.transform.rotation.w = 0.5;
    static_transform_broadcaster_->sendTransform(transform);
  }

  void handle_odometry(const px4_msgs::msg::VehicleOdometry::SharedPtr message)
  {
    const auto pose = ned_frd_to_enu_flu(message->position, message->q, message->pose_frame);
    if (!pose.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring invalid PX4 odometry or unsupported pose frame %u",
        static_cast<unsigned int>(message->pose_frame));
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = pose->x;
    transform.transform.translation.y = pose->y;
    transform.transform.translation.z = pose->z;
    transform.transform.rotation.x = pose->qx;
    transform.transform.rotation.y = pose->qy;
    transform.transform.rotation.z = pose->qz;
    transform.transform.rotation.w = pose->qw;
    transform_broadcaster_->sendTransform(transform);
  }

  std::string map_frame_;
  std::string base_frame_;
  std::string camera_frame_;
  double camera_x_m_;
  double camera_y_m_;
  double camera_z_m_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscription_;
};

}  // namespace drone_perception

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<drone_perception::Px4TfBroadcaster>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("px4_tf_broadcaster"), "Fatal error: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
