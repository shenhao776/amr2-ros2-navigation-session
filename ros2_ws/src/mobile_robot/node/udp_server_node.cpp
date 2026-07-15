#include "../include/udp_server.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mobile_robot::UDPServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}