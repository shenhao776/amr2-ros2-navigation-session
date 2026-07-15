#include "../include/twist_controller.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <thread>

namespace mobile_robot {

TwistControllerNode::TwistControllerNode() : Node("twist_controller_node") {
  // Declare parameters
  cmd_vel_timeout_ = this->declare_parameter("cmd_vel_timeout", 10000.0);
  max_linear_vel_ = this->declare_parameter("max_linear_vel", 1.5);
  max_angular_vel_ = this->declare_parameter("max_angular_vel", 0.8);
  serial_port_name_ =
      this->declare_parameter("serial_port", "/dev/mobile_robot_base");
  baudrate_ = this->declare_parameter("baudrate", 115200);
  // 新增：直线修正的 P 参数，如果修正过猛请调小，修正不足请调大
  yaw_p_gain_ = this->declare_parameter("yaw_p_gain", 1.0);

  // 初始化 serial_fd_ 为 -1，表示未连接
  serial_fd_ = -1;

  // Init Serial
  init_serial();

  // 开启读取线程
  keep_reading_ = true;
  serial_read_thread_ =
      std::thread(&TwistControllerNode::read_serial_loop, this);

  // Setup ROS
  rclcpp::QoS twist_qos(10);
  twist_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", twist_qos,
      std::bind(&TwistControllerNode::cmd_vel_callback, this,
                std::placeholders::_1));

  // 新增：IMU 订阅
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/livox/imu", 10,
      std::bind(&TwistControllerNode::imu_callback, this,
                std::placeholders::_1));

  watchdog_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TwistControllerNode::watchdog_callback, this));

  last_cmd_vel_time_ = this->get_clock()->now();

  RCLCPP_INFO(this->get_logger(), "Twist Controller Node initialized.");
  RCLCPP_INFO(this->get_logger(), "Watchdog timeout: %.2fs", cmd_vel_timeout_);
}

TwistControllerNode::~TwistControllerNode() {
  keep_reading_ = false;
  if (serial_read_thread_.joinable()) {
    serial_read_thread_.join();
  }
  send_command(0, 0);  // Stop motors
  close_serial();
}

void TwistControllerNode::init_serial() {
  // 加锁，防止和 read 线程冲突
  std::lock_guard<std::mutex> lock(serial_mutex_);

  if (serial_fd_ != -1) {
    close(serial_fd_);
    serial_fd_ = -1;
  }

  // 打开串口
  // O_RDWR: 读写模式
  // O_NOCTTY: 也就是不把该串口当成控制终端
  // O_NDELAY: 非阻塞打开，防止打开时就卡死
  serial_fd_ = open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

  if (serial_fd_ == -1) {
    // 只有在第一次初始化报错或者调试时输出 Error，防止重连时刷屏
    // 这里使用 WARN 级别，因为我们会不断重试
    RCLCPP_WARN(this->get_logger(),
                "Failed to open serial port: %s, Waiting for connection...",
                serial_port_name_.c_str());
    return;
  }

  // 恢复为阻塞状态（或者后续通过 termios 设置）
  fcntl(serial_fd_, F_SETFL, 0);

  struct termios options;
  tcgetattr(serial_fd_, &options);

  // Set baudrate
  speed_t baud;
  switch (baudrate_) {
    case 9600:
      baud = B9600;
      break;
    case 115200:
      baud = B115200;
      break;
    default:
      baud = B115200;
  }
  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);

  options.c_cflag |= (CLOCAL | CREAD);  // Enable receiver, ignore modem lines
  options.c_cflag &= ~PARENB;           // No parity
  options.c_cflag &= ~CSTOPB;           // 1 stop bit
  options.c_cflag &= ~CSIZE;            // Mask character size bits
  options.c_cflag |= CS8;               // 8 data bits

  // Raw input
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag &= ~(IXON | IXOFF | IXANY);  // Disable software flow control
  options.c_oflag &= ~OPOST;                   // Raw output

  // 设置读取超时
  // VMIN = 0, VTIME = 1 (0.1秒超时) -> 实现非阻塞读取的效果，防止 read 卡死线程
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 1;

  tcsetattr(serial_fd_, TCSANOW, &options);

  RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully.",
              serial_port_name_.c_str());
}

void TwistControllerNode::close_serial() {
  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (serial_fd_ != -1) {
    close(serial_fd_);
    serial_fd_ = -1;
    RCLCPP_INFO(this->get_logger(), "Serial port closed.");
  }
}

void TwistControllerNode::read_serial_loop() {
  char buf[256];
  while (keep_reading_ && rclcpp::ok()) {
    int current_fd;

    // 1. 安全地获取当前的 fd
    {
      std::lock_guard<std::mutex> lock(serial_mutex_);
      current_fd = serial_fd_;
    }

    // 2. 如果 fd 无效，说明断开了，等待重连，不要 read
    if (current_fd == -1) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      continue;
    }

    // 3. 读取数据
    // 注意：这里没有加锁 read，因为 read 是阻塞/耗时操作，加锁会阻塞
    // send_command。 如果在 read 期间 fd 被 close 了，read
    // 通常会返回错误，我们在下面处理。
    int n = read(current_fd, buf, sizeof(buf) - 1);

    if (n > 0) {
      buf[n] = 0;
      std::string s(buf);
      // Removing newlines for cleaner log
      s.erase(std::remove(s.begin(), s.end(), '\n'), s.end());
      s.erase(std::remove(s.begin(), s.end(), '\r'), s.end());
      if (!s.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "Serial received: %s", s.c_str());
      }
    } else if (n < 0) {
      // 读取错误，可能是硬件断开
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        RCLCPP_WARN(this->get_logger(),
                    "Serial read error. Connection might be lost.");
        // 这里不主动 close，交给 send_command
        // 去触发重连机制，或者你可以选择在这里也触发
      }
    }

    // 稍微休眠，释放 CPU
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void TwistControllerNode::send_command(int right_rpm, int left_rpm) {
  // 1. 检查连接状态
  bool need_reconnect = false;
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_fd_ == -1) need_reconnect = true;
  }

  if (need_reconnect) {
    // 尝试重连（每周期尝试一次，不会阻塞太久，因为我们在init里设置了非阻塞open）
    init_serial();

    // 再次检查是否连接成功，如果没有成功，直接返回，等下一个周期再试
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_fd_ == -1) return;
  }

  // 2. 准备命令
  char cmd[64];
  int len = snprintf(cmd, sizeof(cmd), "A,%d,%d\r\n", right_rpm, left_rpm);
  int written = 0;

  // 3. 写入操作
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_fd_ != -1) {
      written = write(serial_fd_, cmd, len);
    } else {
      return;
    }
  }

  // 4. 结果判断与容错逻辑
  if (written < 0) {
    // 写入失败
    serial_error_count_++;
    RCLCPP_WARN(this->get_logger(), "Serial write failed (%d/%d)",
                serial_error_count_, MAX_SERIAL_ERRORS);

    // 只有当连续失败次数超过阈值时，才认为是物理断开
    if (serial_error_count_ > MAX_SERIAL_ERRORS) {
      RCLCPP_ERROR(this->get_logger(),
                   "Serial connection lost! Reconnecting...");
      close_serial();           // 关闭串口，下次循环会触发重连逻辑
      serial_error_count_ = 0;  // 重置计数器
    }
  } else {
    // 写入成功，立刻重置计数器
    // 只要有一次成功，说明连接是好的，之前的失败可能是瞬时干扰
    if (serial_error_count_ > 0) {
      RCLCPP_INFO(this->get_logger(), "Serial recovered.");
    }
    serial_error_count_ = 0;
  }
}

void TwistControllerNode::watchdog_callback() {
  auto now = this->get_clock()->now();
  double time_since_last_cmd = (now - last_cmd_vel_time_).seconds();

  if (time_since_last_cmd > cmd_vel_timeout_) {
    if (!is_timed_out_) {
      RCLCPP_WARN(this->get_logger(),
                  "[WATCHDOG] No 'cmd_vel' for %.2fs. Sending stop command.",
                  cmd_vel_timeout_);
      send_command(0, 0);
      is_timed_out_ = true;
      // 超时也重置直线行驶状态
      driving_straight_ = false;
    }
  }
}

double TwistControllerNode::get_yaw_from_orientation(
    const geometry_msgs::msg::Quaternion& q) {
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void TwistControllerNode::imu_callback(
    const sensor_msgs::msg::Imu::SharedPtr msg) {
  current_yaw_ = get_yaw_from_orientation(msg->orientation);
}

void TwistControllerNode::cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  last_cmd_vel_time_ = this->get_clock()->now();
  if (is_timed_out_) {
    RCLCPP_INFO(this->get_logger(), "[WATCHDOG] Resuming motion.");
    is_timed_out_ = false;
  }

  double cmd_linear_x = msg->linear.x;
  double cmd_angular_z = msg->angular.z;

  // --- 直线保持逻辑 ---
  // 判断条件：有线速度（且不是特别慢），同时角速度指令非常小（认为是想走直线）
  if (std::abs(cmd_linear_x) > 0.01 && std::abs(cmd_angular_z) < 0.001) {
    if (!driving_straight_) {
      // 刚进入直线模式，锁定当前 Yaw 为目标
      target_yaw_ = current_yaw_;
      driving_straight_ = true;
    }

    // 计算偏差
    double yaw_error = target_yaw_ - current_yaw_;

    // 角度归一化 (-PI ~ PI)
    while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

    // P 控制器修正
    double correction = yaw_error * yaw_p_gain_;
    cmd_angular_z += correction;

  } else {
    // 停车或转弯时，重置状态
    driving_straight_ = false;
  }
  // -------------------

  // Limit velocities
  double limited_linear_x =
      std::max(std::min(cmd_linear_x, max_linear_vel_), -max_linear_vel_);
  double limited_angular_z =
      std::max(std::min(cmd_angular_z, max_angular_vel_), -max_angular_vel_);

  // Kinematics
  double vel_right_speed =
      limited_linear_x - (limited_angular_z * BASE_WIDTH / 2.0);
  double vel_left_speed =
      limited_linear_x + (limited_angular_z * BASE_WIDTH / 2.0);

  // RPM Calculation
  double vel_right_rpm =
      vel_right_speed * 60.0 / (WHEEL_SIZE * M_PI) * MOTOR_POLES / MOTOR_STEPS;
  double vel_left_rpm =
      vel_left_speed * 60.0 / (WHEEL_SIZE * M_PI) * MOTOR_POLES / MOTOR_STEPS;

  send_command((int)vel_right_rpm, (int)vel_left_rpm);
}

}  // namespace mobile_robot