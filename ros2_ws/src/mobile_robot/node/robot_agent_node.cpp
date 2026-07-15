#include "../include/robot_agent.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mobile_robot::RobotAgentNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}