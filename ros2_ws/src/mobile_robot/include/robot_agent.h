#ifndef ROBOT_AGENT_H
#define ROBOT_AGENT_H

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
// [修改] 引入 FollowPath Action
#include "nav2_msgs/action/follow_path.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mobile_robot {

struct Pose7D {
  double x, y, z, qx, qy, qz, qw;
};

class RobotAgentNode : public rclcpp::Node {
 public:
  RobotAgentNode();
  ~RobotAgentNode() override;

 private:
  // --- Type Aliases ---
  // [修改] 使用 FollowPath Action
  using FollowPath = nav2_msgs::action::FollowPath;
  using GoalHandleFollowPath = rclcpp_action::ClientGoalHandle<FollowPath>;

  // --- ROS 2 Interfaces ---
  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      udp_cmd_sub_;  // From UDP Server

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr received_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_poses_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      udp_reply_pub_;  // To UDP Server

  // Actions
  // [修改] 客户端类型变更
  rclcpp_action::Client<FollowPath>::SharedPtr follow_path_client_;

  // --- Logic State ---
  // [修改] Handle 类型变更
  GoalHandleFollowPath::SharedPtr current_goal_handle_;
  std::atomic<bool> is_goal_active_{false};
  bool is_task_active_{false};
  std::vector<Pose7D> mission_waypoints_;
  size_t current_waypoint_index_{0};
  rclcpp::Time mission_start_time_;

  // [新增] 重试定时器
  rclcpp::TimerBase::SharedPtr retry_timer_;

  // --- Robot Pose ---
  geometry_msgs::msg::PoseStamped current_robot_pose_;
  std::mutex pose_mutex_;

  // --- Parameters & Map Data ---
  std::vector<Pose7D> all_poses_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr all_poses_xyz_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_;
  std::string localization_file_path_;
  double connection_radius_;
  double max_tour_length_;
  double path_density_;
  std::string mission_mode_;
  bool automatic_update_map_;

  std::vector<Pose7D> fixed_waypoints_;

  // --- Bag Recording ---
  bool is_recording_ = false;
  bool enable_bag_recording_ = false;
  std::string navigation_state_ = "IDLE";
  std::string bag_storage_path_;
  std::string bag_record_topics_;
  std::string current_bag_name_;
  pid_t ssh_conversion_pid_ = -1;
  pid_t recording_pid_ = -1;

  // --- Debug Mode ---
  bool debug_mode_ = false;
  rclcpp::TimerBase::SharedPtr debug_timer_;
  void debug_planning_callback();

  // --- Logging ---
  std::ofstream log_file_;
  void log(const std::string& message);
  void init_log_file();

  // --- Logic Methods ---
  void process_command_msg(const std_msgs::msg::String::SharedPtr msg);
  void process_command_string(const std::string& command);
  void handle_target_pose(const std::string& command);
  void handle_cmd_vel(const std::string& command);

  void send_reply(const std::string& message);

  void load_all_poses();
  void publish_all_poses_as_pointcloud();

  // Mission
  void start_mission(const Pose7D& target_pose);
  void execute_next_nav_segment();
  void cancel_all_goals(const std::string& reason);

  // Path Planning
  std::vector<int> find_shortest_path_indices(int start_node_idx,
                                              int end_node_idx);

  // [新增] 路径处理函数
  std::vector<Pose7D> downsample_path(const std::vector<Pose7D>& path,
                                      double min_distance);
  std::vector<Pose7D> interpolate_path(const std::vector<Pose7D>& path,
                                       double resolution);

  double get_path_distance(int start_node_idx, int end_node_idx);
  double get_euclidean_distance_3d(const Pose7D& p1, const Pose7D& p2);
  int get_closest_node_idx(const Pose7D& pose);
  std::vector<Pose7D> smooth_path_data(const std::vector<Pose7D>& path,
                                       double weight_data, double weight_smooth,
                                       double tolerance);

  // Bag & Process
  void start_ros2_bag_record();
  void stop_ros2_bag_record(bool allow_conversion = true);
  void cleanup_ssh_process();

  // Callbacks
  void pose_callback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  // [修改] 回调函数参数类型更新
  void goal_response_callback(
      const GoalHandleFollowPath::SharedPtr& goal_handle);
  void feedback_callback(
      GoalHandleFollowPath::SharedPtr,
      const std::shared_ptr<const FollowPath::Feedback> feedback);
  void result_callback(const GoalHandleFollowPath::WrappedResult& result);
};

}  // namespace mobile_robot

#endif
