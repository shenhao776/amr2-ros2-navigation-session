/*
This code is a modified version of the FAST-LIVO2 framework.
Original repository: https://github.com/hku-mars/FAST-LIVO2

Modified by: SHEN HAO
Email: shenhao776@gmail.com
*/
#include <csignal>
#include <rclcpp/rclcpp.hpp>

#include "LIVMapper.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // 全局禁用 stdout 缓冲，所有的 printf 都会立刻输出
  setvbuf(stdout, NULL, _IONBF, 0);
  // 注册信号处理
  signal(SIGINT, LIVMapper::OnSignal);

  auto node = std::make_shared<LIVMapper>();

  node->initializeComponents();

  // [修改] 使用 MultiThreadedExecutor 替代 rclcpp::spin(node)
  // 这样配合 Reentrant CallbackGroup
  // 可以在处理耗时任务（如建图）的同时接收传感器数据
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}