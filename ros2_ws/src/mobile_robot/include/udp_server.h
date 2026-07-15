#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include <netinet/in.h>
#include <sys/socket.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace mobile_robot {

class UDPServerNode : public rclcpp::Node {
 public:
  UDPServerNode();
  ~UDPServerNode();

 private:
  // Network
  int socket_fd_ = -1;
  int port_;
  int buffer_size_;
  char* buffer_ = nullptr;
  struct sockaddr_in server_addr_;
  struct sockaddr_in client_addr_;
  socklen_t client_addr_len_;
  bool client_addr_set_ = false;
  double udp_timeout_sec_;
  rclcpp::Time last_udp_message_time_;

  // ROS
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr udp_cmd_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr udp_reply_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void init_socket();
  void close_socket();
  void reinit_socket();
  void check_udp_loop();

  // Callback for reply from Robot Agent
  void reply_callback(const std_msgs::msg::String::SharedPtr msg);
};

}  // namespace mobile_robot

#endif