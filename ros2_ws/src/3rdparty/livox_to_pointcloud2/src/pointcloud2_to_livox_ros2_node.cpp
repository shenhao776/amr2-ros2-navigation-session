#include <iostream>
#include <livox_to_pointcloud2/pointcloud2_to_livox_ros2.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(
      std::make_shared<pointcloud2_to_livox::PointCloud2ToLivox>(options));
  rclcpp::shutdown();
  return 0;
}