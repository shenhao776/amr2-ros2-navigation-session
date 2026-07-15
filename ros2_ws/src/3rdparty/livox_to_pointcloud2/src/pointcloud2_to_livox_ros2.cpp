#include <livox_to_pointcloud2/pointcloud2_to_livox_ros2.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace pointcloud2_to_livox {

PointCloud2ToLivox::PointCloud2ToLivox(const rclcpp::NodeOptions& options)
    : rclcpp::Node("pointcloud2_to_livox", options) {
  livox_pub = this->create_publisher<livox_ros_driver2::msg::CustomMsg>(
      "/livox/lidar_custom", rclcpp::SensorDataQoS());
  points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/point", rclcpp::SensorDataQoS(),
      std::bind(&PointCloud2ToLivox::pointcloud2_callback, this,
                std::placeholders::_1));
}

PointCloud2ToLivox::~PointCloud2ToLivox() {}

void PointCloud2ToLivox::pointcloud2_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  const auto livox_msg = converter.convert(msg);
  livox_pub->publish(*livox_msg);
}

}  // namespace pointcloud2_to_livox

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud2_to_livox::PointCloud2ToLivox);