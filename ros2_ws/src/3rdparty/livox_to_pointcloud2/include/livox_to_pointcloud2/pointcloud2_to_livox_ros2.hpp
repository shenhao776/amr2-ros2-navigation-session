#pragma once

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "pointcloud2_to_livox_converter.hpp"

namespace pointcloud2_to_livox {

class PointCloud2ToLivox : public rclcpp::Node {
 public:
  PointCloud2ToLivox(const rclcpp::NodeOptions& options);
  ~PointCloud2ToLivox();

 private:
  void pointcloud2_callback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  PointCloud2ToLivoxConverter converter;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_pub;
};

}  // namespace pointcloud2_to_livox