#include "../include/robot_agent.h"

#include <pcl_conversions/pcl_conversions.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <numeric>
#include <queue>
#include <sstream>
#include <thread>
#include <unordered_set>

namespace {

bool wait_for_child(pid_t pid, std::chrono::milliseconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    int status = 0;
    const pid_t result = waitpid(pid, &status, WNOHANG);
    if (result == pid) return true;
    if (result < 0 && (errno == ECHILD || errno == ESRCH)) return true;
    if (result < 0 && errno != EINTR) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return false;
}

}  // namespace

namespace mobile_robot {

RobotAgentNode::RobotAgentNode() : Node("robot_agent_node") {
  // --- Parameters ---
  this->declare_parameter(
      "localization_file_path",
      "/root/shared_files/dataset/my_map_data/result/localization_output.txt");
  this->declare_parameter("connection_radius", 2.0);
  this->declare_parameter("max_tour_length", 5.0);
  this->declare_parameter("path_density", 0.5);
  this->declare_parameter("bag_storage_path",
                          "/root/shared_files/dataset/ros2bags");
  this->declare_parameter("bag_record_topics", "-a");
  this->declare_parameter("default_start_pose.x", 0.0);
  this->declare_parameter("default_start_pose.y", 0.0);
  this->declare_parameter("default_start_pose.z", 0.0);
  this->declare_parameter("automatic_update_map", false);
  this->declare_parameter("mission_mode", "patrol");
  this->declare_parameter("enable_bag_recording", false);
  this->declare_parameter("debug_mode", false);
  this->declare_parameter("fixed_waypoints", std::vector<double>());

  localization_file_path_ =
      this->get_parameter("localization_file_path").as_string();
  connection_radius_ = this->get_parameter("connection_radius").as_double();
  max_tour_length_ = this->get_parameter("max_tour_length").as_double();
  path_density_ = this->get_parameter("path_density").as_double();
  bag_storage_path_ = this->get_parameter("bag_storage_path").as_string();
  bag_record_topics_ = this->get_parameter("bag_record_topics").as_string();
  automatic_update_map_ = this->get_parameter("automatic_update_map").as_bool();
  mission_mode_ = this->get_parameter("mission_mode").as_string();
  enable_bag_recording_ = this->get_parameter("enable_bag_recording").as_bool();
  debug_mode_ = this->get_parameter("debug_mode").as_bool();

  // Load Fixed Waypoints from YAML
  std::vector<double> flat_points =
      this->get_parameter("fixed_waypoints").as_double_array();
  fixed_waypoints_.clear();
  for (size_t i = 0; i < flat_points.size(); i += 2) {
    if (i + 1 < flat_points.size()) {
      Pose7D p;
      p.x = flat_points[i];
      p.y = flat_points[i + 1];
      p.z = 0.0;
      p.qx = 0.0;
      p.qy = 0.0;
      p.qz = 0.0;
      p.qw = 1.0;
      fixed_waypoints_.push_back(p);
    }
  }
  RCLCPP_INFO(this->get_logger(), "Loaded %zu fixed waypoints from params.",
              fixed_waypoints_.size());

  // --- Publishers ---
  rclcpp::QoS qos_profile(rclcpp::KeepLast(1));
  qos_profile.transient_local();

  received_path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("received_path", qos_profile);
  all_poses_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "all_poses_cloud", qos_profile);
  target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/received_target_pose", qos_profile);
  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos_profile);
  udp_reply_pub_ =
      this->create_publisher<std_msgs::msg::String>("udp_write_string", 10);

  // --- Subscribers ---
  pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "pcl_pose", 10,
          std::bind(&RobotAgentNode::pose_callback, this,
                    std::placeholders::_1));

  // Subscribe to commands from UDP Server
  udp_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
      "udp_read_string", 10,
      std::bind(&RobotAgentNode::process_command_msg, this,
                std::placeholders::_1));

  // --- Action Client ---
  follow_path_client_ =
      rclcpp_action::create_client<FollowPath>(this, "follow_path");

  // --- Init ---
  init_log_file();
  load_all_poses();

  if (debug_mode_) {
    log("[DEBUG] Debug Mode Enabled: Nav2 checks will be bypassed.");
  }

  log("Robot Agent Node started (Continuous Path Mode).");
}

RobotAgentNode::~RobotAgentNode() {
  if (is_recording_) {
    log("Still recording during node destruction, force stopping.");
    // Never start a post-processing job while the ROS system is shutting down.
    stop_ros2_bag_record(false);
  }
  cleanup_ssh_process();
  if (log_file_.is_open()) {
    log("Robot Agent Node shutting down.");
    log_file_.close();
  }
}

void RobotAgentNode::debug_planning_callback() {
  if (debug_timer_) {
    debug_timer_->cancel();
  }

  // 模拟任务完成
  if (!mission_waypoints_.empty()) {
    Pose7D reached_pose = mission_waypoints_.back();

    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_robot_pose_.header.stamp = this->get_clock()->now();
    current_robot_pose_.header.frame_id = "map";
    current_robot_pose_.pose.position.x = reached_pose.x;
    current_robot_pose_.pose.position.y = reached_pose.y;
    current_robot_pose_.pose.position.z = reached_pose.z;
    current_robot_pose_.pose.orientation.x = reached_pose.qx;
    current_robot_pose_.pose.orientation.y = reached_pose.qy;
    current_robot_pose_.pose.orientation.z = reached_pose.qz;
    current_robot_pose_.pose.orientation.w = reached_pose.qw;

    if (current_robot_pose_.header.stamp.sec == 0) {
      current_robot_pose_.header.stamp.sec = 1;
    }
  }

  log("[DEBUG] Simulated full path traversal complete.");
  stop_ros2_bag_record();
  send_reply("info mission_complete");
  navigation_state_ = "COMPLETE";
  is_task_active_ = false;
}

void RobotAgentNode::log(const std::string& message) {
  RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
  if (log_file_.is_open()) {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
    log_file_ << "[" << ss.str() << "] " << message << std::endl;
  }
}

void RobotAgentNode::init_log_file() {
  try {
    std::string package_share_directory =
        ament_index_cpp::get_package_share_directory("mobile_robot");
    std::filesystem::path log_dir =
        std::filesystem::path(package_share_directory) / "log";
    std::filesystem::create_directories(log_dir);
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "robot_agent_log_"
       << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S")
       << ".txt";
    std::filesystem::path log_file_path = log_dir / ss.str();
    log_file_.open(log_file_path.string(), std::ios::out | std::ios::app);
    if (log_file_.is_open()) {
      log("Log file created successfully: " + log_file_path.string());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open log file: %s",
                   log_file_path.string().c_str());
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error creating log file: %s", e.what());
  }
}

void RobotAgentNode::send_reply(const std::string& message) {
  std_msgs::msg::String msg;
  msg.data = message;
  udp_reply_pub_->publish(msg);
}

void RobotAgentNode::process_command_msg(
    const std_msgs::msg::String::SharedPtr msg) {
  process_command_string(msg->data);
}

void RobotAgentNode::process_command_string(const std::string& command) {
  if (command.substr(0, 11) == "target_pose") {
    handle_target_pose(command);
  } else if (command.substr(0, 7) == "cmd_vel") {
    handle_cmd_vel(command);
  } else if (command == "start_scan_all") {
    if (is_task_active_) {
      log("Warning: Task is in progress.");
      send_reply("error task_already_active");
      return;
    }
    Pose7D dummy_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    log("Received 'start_scan_all' command.");
    start_mission(dummy_pose);
  } else if (command == "cancel_nav") {
    cancel_all_goals("Received UDP cancel command.");
    navigation_state_ = "IDLE";
  } else if (command == "get_pose") {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (current_robot_pose_.header.stamp.sec == 0) {
      send_reply("error no_pose_received_yet");
    } else {
      std::stringstream ss;
      ss << std::fixed << std::setprecision(6) << "pose "
         << current_robot_pose_.pose.position.x << " "
         << current_robot_pose_.pose.position.y << " "
         << current_robot_pose_.pose.position.z << " "
         << current_robot_pose_.pose.orientation.x << " "
         << current_robot_pose_.pose.orientation.y << " "
         << current_robot_pose_.pose.orientation.z << " "
         << current_robot_pose_.pose.orientation.w;
      send_reply(ss.str());
    }
  } else if (command == "record_start") {
    if (!enable_bag_recording_) {
      send_reply("warn recording_feature_disabled");
      return;
    }
    start_ros2_bag_record();
    if (is_recording_) {
      send_reply("info recording_started");
    } else {
      send_reply("warn already_recording");
    }
  } else if (command == "record_stop") {
    if (!enable_bag_recording_) {
      send_reply("warn recording_feature_disabled");
      return;
    }
    if (is_recording_) {
      // [Mod] 先发送回复，再执行耗时的停止操作，防止客户端超时
      send_reply("info recording_stopped_and_converting");
      stop_ros2_bag_record();
    } else {
      send_reply("warn not_currently_recording");
    }
  } else if (command == "get_status") {
    std::string reply = "status " + navigation_state_;
    send_reply(reply);
    // 如果查询时已经是完成状态，查询后是否需要重置为IDLE？
    // 建议由前端发送新的指令来重置，或者开始新任务时自动覆盖。
    // 这里保持状态不变。
  }
}

void RobotAgentNode::handle_cmd_vel(const std::string& command) {
  std::stringstream ss(command);
  std::string keyword;
  double linear_x, angular_z;
  ss >> keyword >> linear_x >> angular_z;
  if (!ss.fail()) {
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = linear_x;
    twist_msg.angular.z = angular_z;
    cmd_vel_pub_->publish(twist_msg);
  }
}

void RobotAgentNode::handle_target_pose(const std::string& command) {
  if (is_task_active_) {
    send_reply("error task_already_active");
    return;
  }
  std::stringstream ss(command);
  std::string keyword;
  Pose7D target_pose;
  ss >> keyword >> target_pose.x >> target_pose.y >> target_pose.z >>
      target_pose.qx >> target_pose.qy >> target_pose.qz >> target_pose.qw;

  if (ss.fail()) {
    send_reply("error invalid_target_pose_format");
    return;
  }

  if (kdtree_ && !all_poses_.empty()) {
    pcl::PointXYZ search_point(target_pose.x, target_pose.y, target_pose.z);
    std::vector<int> nearest_indices(1);
    std::vector<float> nearest_sq_dists(1);

    // 搜索最近的1个点
    if (kdtree_->nearestKSearch(search_point, 1, nearest_indices,
                                nearest_sq_dists) > 0) {
      int idx = nearest_indices[0];
      // nearestKSearch 返回的是平方距离，需要开根号
      double dist = std::sqrt(nearest_sq_dists[0]);

      // 逻辑：如果距离超过 0.2 米，吸附到最近点；否则仅修正高度
      if (dist > 0.2) {
        Pose7D nearest_map_point = all_poses_[idx];

        log("[Info] Target distance " + std::to_string(dist) +
            "m > 0.2m. Snapping position to nearest localization point.");
        log("       Original Pos: (" + std::to_string(target_pose.x) + ", " +
            std::to_string(target_pose.y) + ", " +
            std::to_string(target_pose.z) + ")");

        // 替换位置为最近的地图点
        target_pose.x = nearest_map_point.x;
        target_pose.y = nearest_map_point.y;
        target_pose.z = nearest_map_point.z;

        // 注意：方向 (qx, qy, qz, qw) 保持不变，使用用户指令的方向
        log("       Snapped Pos:  (" + std::to_string(target_pose.x) + ", " +
            std::to_string(target_pose.y) + ", " +
            std::to_string(target_pose.z) + ")");

      } else {
        // 距离在 0.2m 以内，认为位置有效。
        // 但为了防止 Z 轴误差（例如用户发的是 2D 坐标 z=0），仍然进行高度修正
        double original_z = target_pose.z;
        target_pose.z = all_poses_[idx].z;

        if (std::abs(original_z - target_pose.z) > 1e-4) {
          log("[Info] Target within 0.2m. Auto-corrected height from " +
              std::to_string(original_z) + " to " +
              std::to_string(target_pose.z));
        }
      }
    } else {
      log("[Warn] Could not find nearest waypoint for verification.");
    }
  }

  auto target_pose_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
  target_pose_msg->header.stamp = this->get_clock()->now();
  target_pose_msg->header.frame_id = "map";
  target_pose_msg->pose.position.x = target_pose.x;
  target_pose_msg->pose.position.y = target_pose.y;
  target_pose_msg->pose.position.z = target_pose.z;
  target_pose_msg->pose.orientation.x = target_pose.qx;
  target_pose_msg->pose.orientation.y = target_pose.qy;
  target_pose_msg->pose.orientation.z = target_pose.qz;
  target_pose_msg->pose.orientation.w = target_pose.qw;
  target_pose_pub_->publish(std::move(target_pose_msg));

  start_mission(target_pose);
}

void RobotAgentNode::pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(pose_mutex_);
  current_robot_pose_.header = msg->header;
  current_robot_pose_.pose = msg->pose.pose;
}

void RobotAgentNode::load_all_poses() {
  std::ifstream file(localization_file_path_);
  if (!file.is_open()) {
    log("Error: Unable to open localization file.");
    return;
  }
  all_poses_.clear();
  std::string line;
  double timestamp;
  Pose7D pose;
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    ss >> timestamp >> pose.x >> pose.y >> pose.z >> pose.qx >> pose.qy >>
        pose.qz >> pose.qw;
    if (!ss.fail()) {
      all_poses_.push_back(pose);
    }
  }
  if (all_poses_.empty()) {
    log("Error: No valid localization points loaded.");
    return;
  }
  log("Loaded " + std::to_string(all_poses_.size()) + " localization points.");
  all_poses_xyz_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto& p : all_poses_) {
    all_poses_xyz_->push_back(pcl::PointXYZ(p.x, p.y, p.z));
  }
  kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
  kdtree_->setInputCloud(all_poses_xyz_);
  publish_all_poses_as_pointcloud();
}

void RobotAgentNode::publish_all_poses_as_pointcloud() {
  if (!all_poses_xyz_ || all_poses_xyz_->empty()) return;
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*all_poses_xyz_, cloud_msg);
  cloud_msg.header.stamp = this->get_clock()->now();
  cloud_msg.header.frame_id = "map";
  all_poses_pub_->publish(cloud_msg);
}

void RobotAgentNode::start_mission(const Pose7D& target_pose) {
  if (all_poses_.empty() && mission_mode_ != "scan_all_fixed") {
    send_reply("error no_poses_loaded");
    return;
  }

  if (retry_timer_) {
    retry_timer_->cancel();
  }

  mission_start_time_ = this->get_clock()->now();
  navigation_state_ = "RUNNING";
  log("************************************************************");
  log("** Mission started: Beginning path planning (ONE-SHOT)    **");
  log("************************************************************");

  Pose7D pose_o;
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (current_robot_pose_.header.stamp.sec == 0) {
      log("Warning: Using default origin.");
      pose_o = {this->get_parameter("default_start_pose.x").as_double(),
                this->get_parameter("default_start_pose.y").as_double(),
                this->get_parameter("default_start_pose.z").as_double(),
                0.0,
                0.0,
                0.0,
                1.0};
    } else {
      pose_o = {current_robot_pose_.pose.position.x,
                current_robot_pose_.pose.position.y,
                current_robot_pose_.pose.position.z,
                current_robot_pose_.pose.orientation.x,
                current_robot_pose_.pose.orientation.y,
                current_robot_pose_.pose.orientation.z,
                current_robot_pose_.pose.orientation.w};
    }
  }

  mission_waypoints_.clear();

  if (mission_mode_ == "scan_all") {
    log("Mission Mode: scan_all.");
    mission_waypoints_.push_back(pose_o);
    mission_waypoints_.insert(mission_waypoints_.end(),
                              fixed_waypoints_.begin(), fixed_waypoints_.end());
    mission_waypoints_.push_back(pose_o);
    log("Fixed path generated.");

  } else if (mission_mode_ == "single_point") {
    log("Mission Mode: single_point.");
    mission_waypoints_.push_back(pose_o);
    mission_waypoints_.push_back(target_pose);

  } else if (mission_mode_ == "patrol") {
    log("Mission Mode: patrol.");
    Pose7D pose_a = target_pose;
    pcl::PointXYZ target_search_point(pose_a.x, pose_a.y, pose_a.z);

    std::vector<int> coarse_candidates;
    std::vector<float> point_sq_distances;
    double search_radius = max_tour_length_ * 1.5;
    kdtree_->radiusSearch(target_search_point, search_radius, coarse_candidates,
                          point_sq_distances);
    if (coarse_candidates.size() < 10) {
      log("Warning: Patrol candidates found are insufficient: " +
          std::to_string(coarse_candidates.size()));
    }

    std::unordered_set<int> representative_indices;
    double sample_step = 0.2;
    for (double dx = -max_tour_length_; dx <= max_tour_length_;
         dx += sample_step) {
      for (double dy = -max_tour_length_; dy <= max_tour_length_;
           dy += sample_step) {
        if (dx * dx + dy * dy > max_tour_length_ * max_tour_length_) continue;
        pcl::PointXYZ sample_point(pose_a.x + dx, pose_a.y + dy, pose_a.z);
        std::vector<int> nearest_idx(1);
        std::vector<float> nearest_dist_sq(1);
        kdtree_->nearestKSearch(sample_point, 1, nearest_idx, nearest_dist_sq);
        if (!nearest_idx.empty()) {
          representative_indices.insert(nearest_idx[0]);
        }
      }
    }

    int idx_a = get_closest_node_idx(pose_a);
    std::vector<int> final_patrol_points;
    for (int idx : representative_indices) {
      if (get_path_distance(idx_a, idx) <= max_tour_length_) {
        final_patrol_points.push_back(idx);
      }
    }

    if (final_patrol_points.size() < 3) {
      log("Warning: Less than 3 valid patrol points.");
      send_reply("error not_enough_patrol_points_after_verification");
      return;
    }

    int pointB_idx = -1;
    double max_dist_A_to_B = -1.0;
    for (int idx : final_patrol_points) {
      double dist = get_path_distance(idx_a, idx);
      if (dist > max_dist_A_to_B) {
        max_dist_A_to_B = dist;
        pointB_idx = idx;
      }
    }

    int pointC_idx = -1;
    double max_dist_B_to_C = -1.0;
    for (int idx : final_patrol_points) {
      double dist = get_path_distance(pointB_idx, idx);
      if (dist > max_dist_B_to_C) {
        max_dist_B_to_C = dist;
        pointC_idx = idx;
      }
    }

    int idx_o = get_closest_node_idx(pose_o);
    int idx_b = pointB_idx;
    int idx_c = pointC_idx;

    std::vector<int> intermediate_indices = {idx_a, idx_b, idx_c};
    std::vector<int> best_permutation = {0, 1, 2};
    double min_tour_distance = std::numeric_limits<double>::infinity();
    std::vector<int> p = {0, 1, 2};
    std::sort(p.begin(), p.end());

    do {
      double current_dist =
          get_path_distance(idx_o, intermediate_indices[p[0]]) +
          get_path_distance(intermediate_indices[p[0]],
                            intermediate_indices[p[1]]) +
          get_path_distance(intermediate_indices[p[1]],
                            intermediate_indices[p[2]]);
      if (current_dist < min_tour_distance) {
        min_tour_distance = current_dist;
        best_permutation = p;
      }
    } while (std::next_permutation(p.begin(), p.end()));

    mission_waypoints_.push_back(pose_o);
    for (int i = 0; i < 3; ++i) {
      int current_original_idx = intermediate_indices[best_permutation[i]];
      if (current_original_idx == idx_a) {
        mission_waypoints_.push_back(pose_a);
      } else {
        mission_waypoints_.push_back(all_poses_[current_original_idx]);
      }
    }
    log("Patrol mission planned.");

  } else {
    log("Error: Invalid mission_mode specified.");
    send_reply("error invalid_mission_mode");
    return;
  }

  is_task_active_ = true;
  current_waypoint_index_ = 0;
  start_ros2_bag_record();
  execute_next_nav_segment();
}

std::vector<Pose7D> RobotAgentNode::smooth_path_data(
    const std::vector<Pose7D>& path, double weight_data, double weight_smooth,
    double tolerance) {
  if (path.size() < 3) return path;

  std::vector<Pose7D> new_path = path;
  double change = tolerance;
  int max_iter = 1000;
  int iter = 0;

  while (change >= tolerance && iter < max_iter) {
    change = 0.0;
    for (size_t i = 1; i < path.size() - 1; ++i) {
      double aux_x = new_path[i].x;
      double aux_y = new_path[i].y;

      new_path[i].x += weight_data * (path[i].x - new_path[i].x) +
                       weight_smooth * (new_path[i - 1].x + new_path[i + 1].x -
                                        2.0 * new_path[i].x);
      new_path[i].y += weight_data * (path[i].y - new_path[i].y) +
                       weight_smooth * (new_path[i - 1].y + new_path[i + 1].y -
                                        2.0 * new_path[i].y);

      change +=
          std::abs(aux_x - new_path[i].x) + std::abs(aux_y - new_path[i].y);
    }
    iter++;
  }
  return new_path;
}

// [核心修复] 修正插值逻辑，确保保留关键路点的方向
std::vector<Pose7D> RobotAgentNode::interpolate_path(
    const std::vector<Pose7D>& path, double resolution) {
  if (path.empty()) return path;
  std::vector<Pose7D> dense_path;
  dense_path.push_back(path.front());

  for (size_t i = 0; i < path.size() - 1; ++i) {
    double dx = path[i + 1].x - path[i].x;
    double dy = path[i + 1].y - path[i].y;
    double dist = std::sqrt(dx * dx + dy * dy);

    int steps = std::ceil(dist / resolution);
    if (steps < 1) steps = 1;  // 至少插一步，保证终点能加进去

    double dz = path[i + 1].z - path[i].z;
    for (int j = 1; j <= steps; ++j) {
      double t = (double)j / steps;
      Pose7D p;
      p.x = path[i].x + t * dx;
      p.y = path[i].y + t * dy;
      p.z = path[i].z + t * dz;

      // [BUG修复]
      // 之前是 p.qx = path[i].qx (一直沿用起点方向)
      // 现在：如果到达了该段终点(j==steps)，必须继承终点的方向(path[i+1])，
      // 因为这个点可能是整个任务的最终目标点，带有用户的角度要求。
      if (j == steps) {
        p.qx = path[i + 1].qx;
        p.qy = path[i + 1].qy;
        p.qz = path[i + 1].qz;
        p.qw = path[i + 1].qw;
      } else {
        // 中间点：暂时沿用起点方向（会被 execute_next_nav_segment
        // 中的切线计算覆盖，无所谓）
        p.qx = path[i].qx;
        p.qy = path[i].qy;
        p.qz = path[i].qz;
        p.qw = path[i].qw;
      }
      dense_path.push_back(p);
    }
  }
  return dense_path;
}

void RobotAgentNode::execute_next_nav_segment() {
  if (!is_task_active_) return;

  if (retry_timer_) {
    retry_timer_->cancel();
  }

  if (mission_waypoints_.size() < 2) {
    log("Error: Not enough waypoints.");
    stop_ros2_bag_record();
    is_task_active_ = false;
    return;
  }

  if (!debug_mode_ &&
      !follow_path_client_->wait_for_action_server(std::chrono::seconds(10))) {
    log("Error: Action server not ready.");
    cancel_all_goals("Server timeout");
    return;
  }

  std::vector<Pose7D> full_path_poses;
  log("Planning path through " + std::to_string(mission_waypoints_.size()) +
      " waypoints...");

  for (size_t i = 0; i < mission_waypoints_.size() - 1; ++i) {
    Pose7D& start_pose = mission_waypoints_[i];
    Pose7D& end_pose = mission_waypoints_[i + 1];

    int start_node_idx = get_closest_node_idx(start_pose);
    int end_node_idx = get_closest_node_idx(end_pose);

    std::vector<int> path_indices =
        find_shortest_path_indices(start_node_idx, end_node_idx);

    if (path_indices.empty()) {
      log("Error: Path planning failed at segment " + std::to_string(i));
      cancel_all_goals("Planning failed");
      return;
    }

    for (size_t k = 0; k < path_indices.size(); ++k) {
      if (i > 0 && k == 0) continue;
      full_path_poses.push_back(all_poses_[path_indices[k]]);
    }
  }

  if (!full_path_poses.empty() && !mission_waypoints_.empty()) {
    Pose7D final_goal = mission_waypoints_.back();
    double dist_to_goal =
        get_euclidean_distance_3d(full_path_poses.back(), final_goal);
    if (dist_to_goal > 0.05) {
      full_path_poses.push_back(final_goal);
    } else {
      full_path_poses.back() = final_goal;
    }
  }

  if (full_path_poses.empty()) {
    log("Error: Empty path.");
    return;
  }

  // [修改流程] 1. 下采样 -> 2. 插值(加密) -> 3. 平滑
  std::vector<Pose7D> downsampled =
      downsample_path(full_path_poses, path_density_);

  // [关键] 强制加密路径点（0.1m间距），解决卡顿
  std::vector<Pose7D> dense_path = interpolate_path(downsampled, 0.1);

  // 平滑处理
  std::vector<Pose7D> final_path =
      smooth_path_data(dense_path, 0.8, 0.25, 1e-6);

  if (debug_mode_) {
    double total_path_length = 0.0;
    for (size_t i = 0; i < final_path.size() - 1; ++i) {
      total_path_length +=
          get_euclidean_distance_3d(final_path[i], final_path[i + 1]);
    }
    log("[DEBUG] Planned path total length: " +
        std::to_string(total_path_length) + " meters.");
  }

  auto goal_msg = FollowPath::Goal();
  auto path_viz_msg = std::make_shared<nav_msgs::msg::Path>();
  path_viz_msg->header.stamp = this->get_clock()->now();
  path_viz_msg->header.frame_id = "map";

  for (size_t i = 0; i < final_path.size(); ++i) {
    const auto& pose7d = final_path[i];
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = path_viz_msg->header;
    pose_stamped.pose.position.x = pose7d.x;
    pose_stamped.pose.position.y = pose7d.y;
    pose_stamped.pose.position.z = pose7d.z;

    bool is_very_last_point = (i == final_path.size() - 1);
    if (is_very_last_point) {
      // 最后一个点：保留任务要求的朝向
      // 由于 interpolate_path 已经修复，这里的 pose7d.qx 就是正确的用户指定方向
      pose_stamped.pose.orientation.x = pose7d.qx;
      pose_stamped.pose.orientation.y = pose7d.qy;
      pose_stamped.pose.orientation.z = pose7d.qz;
      pose_stamped.pose.orientation.w = pose7d.qw;
    } else {
      // [关键] 中间点：强制使用切线方向，不考虑原始方向
      double yaw = 0.0;
      if (i + 1 < final_path.size()) {
        double dx = final_path[i + 1].x - final_path[i].x;
        double dy = final_path[i + 1].y - final_path[i].y;
        yaw = std::atan2(dy, dx);
      } else if (i > 0) {
        double dx = final_path[i].x - final_path[i - 1].x;
        double dy = final_path[i].y - final_path[i - 1].y;
        yaw = std::atan2(dy, dx);
      }
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose_stamped.pose.orientation = tf2::toMsg(q);
    }
    path_viz_msg->poses.push_back(pose_stamped);
  }

  goal_msg.path = *path_viz_msg;
  goal_msg.controller_id = "FollowPath";
  goal_msg.goal_checker_id = "general_goal_checker";

  received_path_pub_->publish(*path_viz_msg);
  log("Full Mission Path Sent (" + std::to_string(final_path.size()) +
      " pts).");

  if (debug_mode_) {
    debug_timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&RobotAgentNode::debug_planning_callback, this));
    return;
  }

  auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(
      &RobotAgentNode::goal_response_callback, this, std::placeholders::_1);
  send_goal_options.result_callback =
      std::bind(&RobotAgentNode::result_callback, this, std::placeholders::_1);

  follow_path_client_->async_send_goal(goal_msg, send_goal_options);
}

std::vector<int> RobotAgentNode::find_shortest_path_indices(int start_node_idx,
                                                            int end_node_idx) {
  using PQElement = std::pair<double, int>;
  std::priority_queue<PQElement, std::vector<PQElement>,
                      std::greater<PQElement>>
      open_set;
  std::vector<double> g_score(all_poses_.size(),
                              std::numeric_limits<double>::infinity());
  std::vector<int> came_from(all_poses_.size(), -1);

  g_score[start_node_idx] = 0;
  double h_score = get_euclidean_distance_3d(all_poses_[start_node_idx],
                                             all_poses_[end_node_idx]);
  open_set.push({h_score, start_node_idx});

  while (!open_set.empty()) {
    int current_idx = open_set.top().second;
    open_set.pop();

    if (current_idx == end_node_idx) {
      std::vector<int> path_indices;
      while (current_idx != -1) {
        path_indices.push_back(current_idx);
        current_idx = came_from[current_idx];
      }
      std::reverse(path_indices.begin(), path_indices.end());
      return path_indices;
    }

    std::vector<int> neighbor_indices;
    std::vector<float> neighbor_sq_dists;
    pcl::PointXYZ search_point(all_poses_[current_idx].x,
                               all_poses_[current_idx].y,
                               all_poses_[current_idx].z);

    if (kdtree_->radiusSearch(search_point, connection_radius_,
                              neighbor_indices, neighbor_sq_dists) > 0) {
      for (size_t i = 0; i < neighbor_indices.size(); ++i) {
        int neighbor_idx = neighbor_indices[i];
        double tentative_g_score =
            g_score[current_idx] + std::sqrt(neighbor_sq_dists[i]);
        if (tentative_g_score < g_score[neighbor_idx]) {
          came_from[neighbor_idx] = current_idx;
          g_score[neighbor_idx] = tentative_g_score;
          double h_score_neighbor = get_euclidean_distance_3d(
              all_poses_[neighbor_idx], all_poses_[end_node_idx]);
          open_set.push(
              {g_score[neighbor_idx] + h_score_neighbor, neighbor_idx});
        }
      }
    }
  }
  return {};
}

std::vector<Pose7D> RobotAgentNode::downsample_path(
    const std::vector<Pose7D>& path, double min_distance) {
  if (path.size() < 2) return path;
  std::vector<Pose7D> downsampled_path;
  downsampled_path.push_back(path.front());
  for (size_t i = 1; i < path.size(); ++i) {
    if (get_euclidean_distance_3d(path[i], downsampled_path.back()) >=
        min_distance) {
      downsampled_path.push_back(path[i]);
    }
  }
  if (get_euclidean_distance_3d(path.back(), downsampled_path.back()) > 1e-6) {
    downsampled_path.push_back(path.back());
  }
  return downsampled_path;
}

void RobotAgentNode::goal_response_callback(
    const GoalHandleFollowPath::SharedPtr& goal_handle) {
  if (!goal_handle) {
    log("Error: Navigation goal rejected by server.");
    cancel_all_goals("Goal rejected by server");
  } else {
    current_goal_handle_ = goal_handle;
    is_goal_active_ = true;
  }
}

void RobotAgentNode::result_callback(
    const GoalHandleFollowPath::WrappedResult& result) {
  is_goal_active_ = false;
  current_goal_handle_ = nullptr;

  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    // [关键修改] 遇到障碍物中止(Aborted)时，不取消任务，而是尝试重试
    log("Warning: Navigation mission aborted (likely obstacle or timeout). "
        "Retrying in 5 seconds...");

    // 启动定时器，5秒后重新执行路径发送
    if (retry_timer_) retry_timer_->cancel();
    retry_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&RobotAgentNode::execute_next_nav_segment, this));

    // 注意：不要调用 cancel_all_goals()，保持 is_task_active_ 为 true
    return;
  }

  stop_ros2_bag_record();
  log("Mission complete! (Full path traversed)");
  send_reply("info mission_complete");
  navigation_state_ = "COMPLETE";
  is_task_active_ = false;
}

void RobotAgentNode::start_ros2_bag_record() {
  if (is_recording_ || recording_pid_ != -1 || !enable_bag_recording_) {
    return;
  }
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << "nav_bag_"
     << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
  std::string bag_name = ss.str();
  current_bag_name_ = bag_name;

  std::stringstream command_ss;
  command_ss << "mkdir -p " << bag_storage_path_ << " && "
             << "exec ros2 bag record --max-cache-size 1000000000 -o "
             << bag_storage_path_ << "/" << bag_name << " "
             << bag_record_topics_;

  std::string command_to_run = command_ss.str();

  pid_t pid = fork();
  if (pid < 0) {
    log("Error: Failed to fork recording subprocess!");
    return;
  } else if (pid == 0) {
    execl("/bin/sh", "sh", "-c", command_to_run.c_str(), (char*)NULL);
    _exit(1);
  } else {
    log("Successfully created recording process, PID: " + std::to_string(pid));
    recording_pid_ = pid;
    is_recording_ = true;
  }
}

void RobotAgentNode::stop_ros2_bag_record(bool allow_conversion) {
  if (!is_recording_ || recording_pid_ <= 0 || !enable_bag_recording_) {
    return;
  }
  if (current_bag_name_.empty()) {
    is_recording_ = false;
    recording_pid_ = -1;
    return;
  }
  log("Stopping recording process...");
  const pid_t pid = recording_pid_;
  kill(pid, SIGINT);

  bool exited = wait_for_child(pid, std::chrono::seconds(3));
  if (!exited) {
    log("Recording process did not stop after SIGINT; sending SIGTERM.");
    kill(pid, SIGTERM);
    exited = wait_for_child(pid, std::chrono::seconds(2));
  }
  if (!exited) {
    log("Recording process did not stop after SIGTERM; sending SIGKILL.");
    kill(pid, SIGKILL);
    wait_for_child(pid, std::chrono::seconds(1));
  }

  recording_pid_ = -1;
  is_recording_ = false;

  cleanup_ssh_process();

  if (allow_conversion && automatic_update_map_) {
    log("Starting background conversion...");
    std::string ros2_bag_path = bag_storage_path_ + "/" + current_bag_name_;
    std::string ros1_bag_path = "/root/shared_files/rosbag/ros1bag/lvio_bag/" +
                                current_bag_name_ + ".bag";

    std::stringstream command_ss;
    command_ss
        << "docker run --rm "
        << "-e ROS_DOMAIN_ID=1 --net=host --privileged "
        << "-v /home/hao/shared_files:/root/shared_files "
        << "-v /dev:/dev "
        << "--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all "
        << "-e DISPLAY=$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE "
        << "--user root shenhao776/amr_ros1_x86:v0.3 "
        << "/root/shared_files/ros1_ros2_bridge_prj/scripts/bag_converter.py "
        << ros2_bag_path << " " << ros1_bag_path;

    std::string command_to_run = command_ss.str();
    pid_t pid = fork();
    if (pid < 0) {
      current_bag_name_.clear();
      return;
    } else if (pid == 0) {
      setsid();
      execl("/bin/sh", "sh", "-c", command_to_run.c_str(), (char*)NULL);
      _exit(1);
    } else {
      ssh_conversion_pid_ = pid;
      current_bag_name_.clear();
    }
  } else {
    current_bag_name_.clear();
  }
}

void RobotAgentNode::cancel_all_goals(const std::string& reason) {
  if (retry_timer_) {
    retry_timer_->cancel();
  }
  if (is_goal_active_ && current_goal_handle_) {
    follow_path_client_->async_cancel_goal(current_goal_handle_);
  }
  is_goal_active_ = false;
  is_task_active_ = false;
  mission_waypoints_.clear();
}

double RobotAgentNode::get_path_distance(int start_node_idx, int end_node_idx) {
  if (start_node_idx == end_node_idx) return 0.0;
  auto indices = find_shortest_path_indices(start_node_idx, end_node_idx);
  if (indices.size() < 2) return std::numeric_limits<double>::infinity();
  double distance = 0.0;
  for (size_t i = 0; i < indices.size() - 1; ++i) {
    distance += get_euclidean_distance_3d(all_poses_[indices[i]],
                                          all_poses_[indices[i + 1]]);
  }
  return distance;
}

double RobotAgentNode::get_euclidean_distance_3d(const Pose7D& p1,
                                                 const Pose7D& p2) {
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) +
                   std::pow(p1.z - p2.z, 2));
}

int RobotAgentNode::get_closest_node_idx(const Pose7D& pose) {
  pcl::PointXYZ search_point(pose.x, pose.y, pose.z);
  std::vector<int> indices(1);
  std::vector<float> sq_dists(1);
  kdtree_->nearestKSearch(search_point, 1, indices, sq_dists);
  return indices[0];
}

void RobotAgentNode::cleanup_ssh_process() {
  if (ssh_conversion_pid_ != -1) {
    const pid_t pid = ssh_conversion_pid_;
    kill(-pid, SIGTERM);
    if (!wait_for_child(pid, std::chrono::seconds(2))) {
      log("Background conversion did not stop after SIGTERM; sending SIGKILL.");
      kill(-pid, SIGKILL);
      wait_for_child(pid, std::chrono::seconds(1));
    }
    ssh_conversion_pid_ = -1;
  }
}

}  // namespace mobile_robot
