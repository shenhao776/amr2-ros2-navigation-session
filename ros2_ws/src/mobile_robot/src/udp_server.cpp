#include "../include/udp_server.h"

#include <arpa/inet.h>
#include <unistd.h>

#include <cstring>

namespace mobile_robot {

UDPServerNode::UDPServerNode() : Node("udp_server_node") {
  this->declare_parameter("udp_port", 8888);
  this->declare_parameter("udp_buffer_size", 4096);
  this->declare_parameter("udp_timeout_sec", 10.0);

  port_ = this->get_parameter("udp_port").as_int();
  buffer_size_ = this->get_parameter("udp_buffer_size").as_int();
  udp_timeout_sec_ = this->get_parameter("udp_timeout_sec").as_double();

  // Internal communication topics
  udp_cmd_pub_ =
      this->create_publisher<std_msgs::msg::String>("udp_read_string", 10);
  udp_reply_sub_ = this->create_subscription<std_msgs::msg::String>(
      "udp_write_string", 10,
      std::bind(&UDPServerNode::reply_callback, this, std::placeholders::_1));

  init_socket();

  timer_ =
      this->create_wall_timer(std::chrono::milliseconds(10),
                              std::bind(&UDPServerNode::check_udp_loop, this));

  RCLCPP_INFO(this->get_logger(), "UDP Server Node started on port %d", port_);
  last_udp_message_time_ = this->get_clock()->now();
}

UDPServerNode::~UDPServerNode() {
  close_socket();
  if (buffer_) delete[] buffer_;
}

void UDPServerNode::init_socket() {
  socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd_ < 0) {
    RCLCPP_FATAL(this->get_logger(), "Unable to create UDP socket");
    return;
  }

  memset(&server_addr_, 0, sizeof(server_addr_));
  server_addr_.sin_family = AF_INET;
  server_addr_.sin_addr.s_addr = INADDR_ANY;
  server_addr_.sin_port = htons(port_);

  if (bind(socket_fd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) <
      0) {
    RCLCPP_FATAL(this->get_logger(), "Unable to bind UDP socket to port %d",
                 port_);
    close_socket();
    return;
  }

  client_addr_len_ = sizeof(client_addr_);
  if (!buffer_) buffer_ = new char[buffer_size_];
}

void UDPServerNode::close_socket() {
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

void UDPServerNode::reinit_socket() {
  RCLCPP_WARN(this->get_logger(), "Reinitializing UDP socket...");
  close_socket();
  rclcpp::sleep_for(std::chrono::seconds(1));
  init_socket();
}

void UDPServerNode::check_udp_loop() {
  if (socket_fd_ < 0) return;

  // 使用 while 循环读取所有缓冲区中的数据，防止积压
  while (true) {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(socket_fd_, &readfds);
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;  // 非阻塞检查

    // 检查是否有数据可读
    int activity = select(socket_fd_ + 1, &readfds, NULL, NULL, &timeout);

    if (activity > 0 && FD_ISSET(socket_fd_, &readfds)) {
      memset(buffer_, 0, buffer_size_);
      int recv_len =
          recvfrom(socket_fd_, buffer_, buffer_size_, 0,
                   (struct sockaddr*)&client_addr_, &client_addr_len_);

      if (recv_len > 0) {
        client_addr_set_ = true;
        last_udp_message_time_ = this->get_clock()->now();

        // 收到数据立即发布（或者你也可以选择只记录最新一条，在while循环结束后再发布）
        // 这里建议直接发布，因为 robot_agent 处理很快
        std::string cmd(buffer_, recv_len);
        std_msgs::msg::String msg;
        msg.data = cmd;
        udp_cmd_pub_->publish(msg);
      }
    } else {
      // 没有更多数据了，跳出循环
      break;
    }
  }
}

void UDPServerNode::reply_callback(const std_msgs::msg::String::SharedPtr msg) {
  if (!client_addr_set_ || socket_fd_ < 0) return;

  sendto(socket_fd_, msg->data.c_str(), msg->data.length(), 0,
         (struct sockaddr*)&client_addr_, client_addr_len_);
}

}  // namespace mobile_robot