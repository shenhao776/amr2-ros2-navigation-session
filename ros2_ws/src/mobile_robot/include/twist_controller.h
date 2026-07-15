#ifndef TWIST_CONTROLLER_H
#define TWIST_CONTROLLER_H

#include <fcntl.h>
#include <termios.h>

#include <atomic>
#include <cmath>
#include <mutex>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace mobile_robot {

class TwistControllerNode : public rclcpp::Node {
 public:
  TwistControllerNode();
  ~TwistControllerNode();

 private:
  // Parameters
  double cmd_vel_timeout_;
  double max_linear_vel_;
  double max_angular_vel_;
  double yaw_p_gain_;  // 直线行驶时的 P 增益
  std::mutex serial_mutex_;
  std::string serial_port_name_;
  int baudrate_;
  int serial_error_count_ = 0;
  // 错误阈值
  const int MAX_SERIAL_ERRORS = 10;

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Time last_cmd_vel_time_;

  // State
  std::atomic<bool> is_timed_out_{true};
  int serial_fd_ = -1;

  // 直线辅助相关状态
  double current_yaw_ = 0.0;
  double target_yaw_ = 0.0;
  bool driving_straight_ = false;

  // Constants
  const double BASE_WIDTH = 0.4;
  const double WHEEL_SIZE = 0.175;
  const double MOTOR_STEPS = 0.9;  // 9.0/10.0
  const double MOTOR_POLES = 60.0;

  // Serial Reading
  std::thread serial_read_thread_;
  std::atomic<bool> keep_reading_{true};

  void init_serial();
  void close_serial();
  void read_serial_loop();
  void send_command(int right_rpm, int left_rpm);

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void watchdog_callback();

  // 辅助函数
  double get_yaw_from_orientation(const geometry_msgs::msg::Quaternion& q);
};

}  // namespace mobile_robot

#endif