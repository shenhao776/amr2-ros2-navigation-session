#include <lidar_localization/lidar_localization_component.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

PCLLocalization::PCLLocalization(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("lidar_localization", options),
      clock_(RCL_ROS_TIME),
      tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
      tflistener_(tfbuffer_),
      broadcaster_(this) {
  declare_parameter("global_frame_id", "map");
  declare_parameter("odom_frame_id", "livox_odom");
  declare_parameter("base_frame_id", "base_link");
  declare_parameter("imu_topic", "/livox/imu");
  declare_parameter("registration_method", "NDT");
  declare_parameter("score_threshold", 2.0);
  declare_parameter("ndt_resolution", 1.0);
  declare_parameter("ndt_step_size", 0.1);
  declare_parameter("ndt_max_iterations", 35);
  declare_parameter("ndt_num_threads", 4);
  declare_parameter("transform_epsilon", 0.01);
  declare_parameter("voxel_leaf_size", 0.2);
  declare_parameter("scan_max_range", 100.0);
  declare_parameter("scan_min_range", 1.0);
  declare_parameter("scan_period", 0.1);
  declare_parameter("use_pcd_map", false);
  declare_parameter("map_path", "/map/map.pcd");
  declare_parameter("set_initial_pose", false);
  declare_parameter("initial_pose_x", 0.0);
  declare_parameter("initial_pose_y", 0.0);
  declare_parameter("initial_pose_z", 0.0);
  declare_parameter("initial_pose_qx", 0.0);
  declare_parameter("initial_pose_qy", 0.0);
  declare_parameter("initial_pose_qz", 0.0);
  declare_parameter("initial_pose_qw", 1.0);
  declare_parameter("use_odom", false);
  declare_parameter("use_imu", false);
  declare_parameter("enable_debug", false);
}

using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn PCLLocalization::on_configure(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Configuring");

  initializeParameters();
  initializePubSub();
  initializeRegistration();

  path_ptr_ = std::make_shared<nav_msgs::msg::Path>();
  path_ptr_->header.frame_id = global_frame_id_;

  RCLCPP_INFO(get_logger(), "Configuring end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_activate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Activating");

  pose_pub_->on_activate();
  path_pub_->on_activate();
  initial_map_pub_->on_activate();

  if (set_initial_pose_) {
    auto msg =
        std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.pose.position.x = initial_pose_x_;
    msg->pose.pose.position.y = initial_pose_y_;
    msg->pose.pose.position.z = initial_pose_z_;
    msg->pose.pose.orientation.x = initial_pose_qx_;
    msg->pose.pose.orientation.y = initial_pose_qy_;
    msg->pose.pose.orientation.z = initial_pose_qz_;
    msg->pose.pose.orientation.w = initial_pose_qw_;

    geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped(
        new geometry_msgs::msg::PoseStamped);
    pose_stamped->header.stamp = msg->header.stamp;
    pose_stamped->header.frame_id = global_frame_id_;
    pose_stamped->pose = msg->pose.pose;
    path_ptr_->poses.push_back(*pose_stamped);

    initialPoseReceived(msg);
  }

  if (use_pcd_map_) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(map_path_, *map_cloud_ptr);
    RCLCPP_INFO(get_logger(), "Map Size %ld", map_cloud_ptr->size());

    sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(
        new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*map_cloud_ptr, *map_msg_ptr);
    map_msg_ptr->header.frame_id = global_frame_id_;
    initial_map_pub_->publish(*map_msg_ptr);
    RCLCPP_INFO(get_logger(), "Initial Map Published");

    if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(
          new pcl::PointCloud<pcl::PointXYZI>());
      voxel_grid_filter_.setInputCloud(map_cloud_ptr);
      voxel_grid_filter_.filter(*filtered_cloud_ptr);
      registration_->setInputTarget(filtered_cloud_ptr);
    } else {
      registration_->setInputTarget(map_cloud_ptr);
    }

    map_recieved_ = true;
  }

  RCLCPP_INFO(get_logger(), "Activating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_deactivate(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  pose_pub_->on_deactivate();
  path_pub_->on_deactivate();
  initial_map_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_cleanup(const rclcpp_lifecycle::State&) {
  RCLCPP_INFO(get_logger(), "Cleaning Up");
  initial_pose_sub_.reset();
  initial_map_pub_.reset();
  path_pub_.reset();
  pose_pub_.reset();
  odom_sub_.reset();
  cloud_sub_.reset();
  imu_sub_.reset();

  RCLCPP_INFO(get_logger(), "Cleaning Up end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_shutdown(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_error(const rclcpp_lifecycle::State& state) {
  RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

void PCLLocalization::initializeParameters() {
  RCLCPP_INFO(get_logger(), "initializeParameters");
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("imu_topic", imu_topic_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("registration_method", registration_method_);
  get_parameter("score_threshold", score_threshold_);
  get_parameter("ndt_resolution", ndt_resolution_);
  get_parameter("ndt_step_size", ndt_step_size_);
  get_parameter("ndt_num_threads", ndt_num_threads_);
  get_parameter("ndt_max_iterations", ndt_max_iterations_);
  get_parameter("transform_epsilon", transform_epsilon_);
  get_parameter("voxel_leaf_size", voxel_leaf_size_);
  get_parameter("scan_max_range", scan_max_range_);
  get_parameter("scan_min_range", scan_min_range_);
  get_parameter("scan_period", scan_period_);
  get_parameter("use_pcd_map", use_pcd_map_);
  get_parameter("map_path", map_path_);
  get_parameter("set_initial_pose", set_initial_pose_);
  get_parameter("initial_pose_x", initial_pose_x_);
  get_parameter("initial_pose_y", initial_pose_y_);
  get_parameter("initial_pose_z", initial_pose_z_);
  get_parameter("initial_pose_qx", initial_pose_qx_);
  get_parameter("initial_pose_qy", initial_pose_qy_);
  get_parameter("initial_pose_qz", initial_pose_qz_);
  get_parameter("initial_pose_qw", initial_pose_qw_);
  get_parameter("use_odom", use_odom_);
  get_parameter("use_imu", use_imu_);
  get_parameter("enable_debug", enable_debug_);

  RCLCPP_INFO(get_logger(), "global_frame_id: %s", global_frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "odom_frame_id: %s", odom_frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "imu_topic: %s", imu_topic_.c_str());
  RCLCPP_INFO(get_logger(), "base_frame_id: %s", base_frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "registration_method: %s",
              registration_method_.c_str());
  RCLCPP_INFO(get_logger(), "ndt_resolution: %lf", ndt_resolution_);
  RCLCPP_INFO(get_logger(), "ndt_step_size: %lf", ndt_step_size_);
  RCLCPP_INFO(get_logger(), "ndt_num_threads: %d", ndt_num_threads_);
  RCLCPP_INFO(get_logger(), "transform_epsilon: %lf", transform_epsilon_);
  RCLCPP_INFO(get_logger(), "voxel_leaf_size: %lf", voxel_leaf_size_);
  RCLCPP_INFO(get_logger(), "scan_max_range: %lf", scan_max_range_);
  RCLCPP_INFO(get_logger(), "scan_min_range: %lf", scan_min_range_);
  RCLCPP_INFO(get_logger(), "scan_period: %lf", scan_period_);
  RCLCPP_INFO(get_logger(), "use_pcd_map: %d", use_pcd_map_);
  RCLCPP_INFO(get_logger(), "map_path: %s", map_path_.c_str());
  RCLCPP_INFO(get_logger(), "set_initial_pose: %d", set_initial_pose_);
  RCLCPP_INFO(get_logger(), "use_odom: %d", use_odom_);
  RCLCPP_INFO(get_logger(), "use_imu: %d", use_imu_);
  RCLCPP_INFO(get_logger(), "enable_debug: %d", enable_debug_);
}

void PCLLocalization::initializePubSub() {
  RCLCPP_INFO(get_logger(), "initializePubSub");

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "pcl_pose",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "path", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "initial_map",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "initialpose", rclcpp::SystemDefaultsQoS(),
          std::bind(&PCLLocalization::initialPoseReceived, this,
                    std::placeholders::_1));

  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&PCLLocalization::mapReceived, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&PCLLocalization::odomReceived, this, std::placeholders::_1));

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "cloud", rclcpp::SensorDataQoS(),
      std::bind(&PCLLocalization::cloudReceived, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PCLLocalization::imuReceived, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "initializePubSub end");
}

void PCLLocalization::initializeRegistration() {
  RCLCPP_INFO(get_logger(), "initializeRegistration");

  if (registration_method_ == "GICP") {
    boost::shared_ptr<
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>
        gicp(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI,
                                                       pcl::PointXYZI>());
    gicp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp;
  } else if (registration_method_ == "NDT") {
    boost::shared_ptr<
        pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>
        ndt(new pcl::NormalDistributionsTransform<pcl::PointXYZI,
                                                  pcl::PointXYZI>());
    ndt->setStepSize(ndt_step_size_);
    ndt->setResolution(ndt_resolution_);
    ndt->setTransformationEpsilon(transform_epsilon_);
    registration_ = ndt;
  } else if (registration_method_ == "NDT_OMP") {
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr
        ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZI,
                                                         pcl::PointXYZI>());
    ndt_omp->setStepSize(ndt_step_size_);
    ndt_omp->setResolution(ndt_resolution_);
    ndt_omp->setTransformationEpsilon(transform_epsilon_);
    if (ndt_num_threads_ > 0) {
      ndt_omp->setNumThreads(ndt_num_threads_);
    } else {
      ndt_omp->setNumThreads(omp_get_max_threads());
    }
    registration_ = ndt_omp;
  } else if (registration_method_ == "GICP_OMP") {
    pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI,
                                             pcl::PointXYZI>::Ptr
        gicp_omp(
            new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI,
                                                         pcl::PointXYZI>());
    gicp_omp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp_omp;
  } else {
    RCLCPP_ERROR(get_logger(), "Invalid registration method.");
    exit(EXIT_FAILURE);
  }
  registration_->setMaximumIterations(ndt_max_iterations_);

  voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_,
                                 voxel_leaf_size_);
  RCLCPP_INFO(get_logger(), "initializeRegistration end");
}

void PCLLocalization::initialPoseReceived(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "initialPoseReceived");
  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(),
                "initialpose_frame_id does not match global_frame_id");
    return;
  }
  initialpose_recieved_ = true;
  corrent_pose_with_cov_stamped_ptr_ = msg;
  pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);

  cloudReceived(last_scan_ptr_);
  RCLCPP_INFO(get_logger(), "initialPoseReceived end");
}

void PCLLocalization::mapReceived(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  RCLCPP_INFO(get_logger(), "mapReceived");
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);

  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(),
                "map_frame_id does not match　global_frame_id");
    return;
  }

  pcl::fromROSMsg(*msg, *map_cloud_ptr);

  if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
    voxel_grid_filter_.setInputCloud(map_cloud_ptr);
    voxel_grid_filter_.filter(*filtered_cloud_ptr);
    registration_->setInputTarget(filtered_cloud_ptr);

  } else {
    registration_->setInputTarget(map_cloud_ptr);
  }

  map_recieved_ = true;
  RCLCPP_INFO(get_logger(), "mapReceived end");
}

void PCLLocalization::odomReceived(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  if (!use_odom_) {
    return;
  }
  RCLCPP_INFO_ONCE(get_logger(), "odomReceived started (predicting pose)");

  double current_odom_received_time =
      msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  double dt_odom = current_odom_received_time - last_odom_received_time_;
  last_odom_received_time_ = current_odom_received_time;

  if (dt_odom > 1.0 || dt_odom <= 0.0) {
    return;
  }

  // 1. 获取当前预测/校正后的位姿
  Eigen::Vector3d current_pos(
      corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x,
      corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y,
      corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z);

  Eigen::Quaterniond current_quat;
  tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation,
               current_quat);

  // 2. 获取角速度和线速度 (通常在机器人坐标系下)
  Eigen::Vector3d angular_vel(msg->twist.twist.angular.x,
                              msg->twist.twist.angular.y,
                              msg->twist.twist.angular.z);
  Eigen::Vector3d linear_vel(msg->twist.twist.linear.x,
                             msg->twist.twist.linear.y,
                             msg->twist.twist.linear.z);

  // 3. 正确的四元数姿态更新 (角速度积分)
  if (angular_vel.norm() > 1e-6) {
    // 计算旋转增量四元数
    Eigen::Quaterniond delta_quat(Eigen::AngleAxisd(
        angular_vel.norm() * dt_odom, angular_vel.normalized()));
    current_quat = current_quat * delta_quat;
    current_quat.normalize();
  }

  // 4. 位置更新 (将机器人系速度转换到地图系)
  Eigen::Vector3d delta_position = current_quat * (linear_vel * dt_odom);
  current_pos += delta_position;

  // 5. 更新状态变量
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x = current_pos.x();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y = current_pos.y();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z = current_pos.z();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation =
      tf2::toMsg(current_quat);

  // 发布 TF (map -> odom)
  if (initialpose_recieved_ && map_recieved_) {
    Eigen::Affine3d map_to_base;
    tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, map_to_base);
    Eigen::Affine3d odom_to_base;
    tf2::fromMsg(msg->pose.pose, odom_to_base);

    Eigen::Affine3d map_to_odom = map_to_base * odom_to_base.inverse();

    geometry_msgs::msg::TransformStamped map_to_odom_tf;
    map_to_odom_tf.header.stamp = msg->header.stamp;
    map_to_odom_tf.header.frame_id = global_frame_id_;
    map_to_odom_tf.child_frame_id = odom_frame_id_;
    map_to_odom_tf.transform = tf2::eigenToTransform(map_to_odom).transform;
    broadcaster_.sendTransform(map_to_odom_tf);
  }
}

void PCLLocalization::imuReceived(
    const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  if (!use_imu_) {
    return;
  }

  sensor_msgs::msg::Imu tf_converted_imu;

  try {
    const geometry_msgs::msg::TransformStamped transform =
        tfbuffer_.lookupTransform(base_frame_id_, msg->header.frame_id,
                                  tf2::TimePointZero);

    geometry_msgs::msg::Vector3Stamped angular_velocity, linear_acceleration,
        transformed_angular_velocity, transformed_linear_acceleration;
    geometry_msgs::msg::Quaternion transformed_quaternion;

    angular_velocity.header = msg->header;
    angular_velocity.vector = msg->angular_velocity;
    linear_acceleration.header = msg->header;
    linear_acceleration.vector = msg->linear_acceleration;

    tf2::doTransform(angular_velocity, transformed_angular_velocity, transform);
    tf2::doTransform(linear_acceleration, transformed_linear_acceleration,
                     transform);

    tf_converted_imu.angular_velocity = transformed_angular_velocity.vector;
    tf_converted_imu.linear_acceleration =
        transformed_linear_acceleration.vector;
    tf_converted_imu.orientation = transformed_quaternion;

  } catch (tf2::TransformException& ex) {
    std::cout << "Failed to lookup transform" << std::endl;
    RCLCPP_WARN(this->get_logger(), "Failed to lookup transform.");
    return;
  }

  Eigen::Vector3f angular_velo{
      static_cast<float>(tf_converted_imu.angular_velocity.x),
      static_cast<float>(tf_converted_imu.angular_velocity.y),
      static_cast<float>(tf_converted_imu.angular_velocity.z)};
  Eigen::Vector3f acc{
      static_cast<float>(tf_converted_imu.linear_acceleration.x),
      static_cast<float>(tf_converted_imu.linear_acceleration.y),
      static_cast<float>(tf_converted_imu.linear_acceleration.z)};
  Eigen::Quaternionf quat{static_cast<float>(msg->orientation.w),
                          static_cast<float>(msg->orientation.x),
                          static_cast<float>(msg->orientation.y),
                          static_cast<float>(msg->orientation.z)};
  double imu_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

  lidar_undistortion_.getImu(angular_velo, acc, quat, imu_time);
}

void PCLLocalization::cloudReceived(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  if (!map_recieved_ || !initialpose_recieved_) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud_ptr);

  if (use_imu_) {
    double received_time =
        msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    lidar_undistortion_.adjustDistortion(cloud_ptr, received_time);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());
  voxel_grid_filter_.setInputCloud(cloud_ptr);
  voxel_grid_filter_.filter(*filtered_cloud_ptr);

  double r;
  pcl::PointCloud<pcl::PointXYZI> tmp;
  for (const auto& p : filtered_cloud_ptr->points) {
    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (scan_min_range_ < r && r < scan_max_range_) {
      tmp.push_back(p);
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(
      new pcl::PointCloud<pcl::PointXYZI>(tmp));
  registration_->setInputSource(tmp_ptr);

  Eigen::Affine3d affine;
  tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, affine);
  Eigen::Matrix4f init_guess = affine.matrix().cast<float>();

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  registration_->align(*output_cloud, init_guess);

  bool has_converged = registration_->hasConverged();
  double fitness_score = registration_->getFitnessScore();
  if (!has_converged) {
    RCLCPP_WARN(get_logger(), "The registration didn't converge.");
    return;
  }

  // 更新全局位姿（用于下一帧的预测和 odomReceived 中的 TF 计算）
  Eigen::Matrix4f final_transformation =
      registration_->getFinalTransformation();
  Eigen::Matrix3d rot_mat =
      final_transformation.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond quat_eig(rot_mat);

  corrent_pose_with_cov_stamped_ptr_->header.stamp = msg->header.stamp;
  corrent_pose_with_cov_stamped_ptr_->header.frame_id = global_frame_id_;
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x =
      static_cast<double>(final_transformation(0, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y =
      static_cast<double>(final_transformation(1, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z =
      static_cast<double>(final_transformation(2, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation =
      tf2::toMsg(quat_eig);

  // 发布位姿话题（用于可视化）
  double pose_covariance = std::max(0.1, fitness_score);
  for (int i = 0; i < 36; i++)
    corrent_pose_with_cov_stamped_ptr_->pose.covariance[i] = 0;
  corrent_pose_with_cov_stamped_ptr_->pose.covariance[0] =
      corrent_pose_with_cov_stamped_ptr_->pose.covariance[7] =
          corrent_pose_with_cov_stamped_ptr_->pose.covariance[14] =
              corrent_pose_with_cov_stamped_ptr_->pose.covariance[21] =
                  corrent_pose_with_cov_stamped_ptr_->pose.covariance[28] =
                      corrent_pose_with_cov_stamped_ptr_->pose.covariance[35] =
                          pose_covariance;

  pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);

  // [Modification] 如果 use_odom 为 false，则直接发布 map -> base_link 的变换
  if (!use_odom_) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = msg->header.stamp;
    transform_stamped.header.frame_id = global_frame_id_;
    transform_stamped.child_frame_id = base_frame_id_;
    transform_stamped.transform.translation.x =
        corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x;
    transform_stamped.transform.translation.y =
        corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y;
    transform_stamped.transform.translation.z =
        corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z;
    transform_stamped.transform.rotation =
        corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation;
    broadcaster_.sendTransform(transform_stamped);
  }

  // 发布路径
  geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped_ptr(
      new geometry_msgs::msg::PoseStamped);
  pose_stamped_ptr->header = corrent_pose_with_cov_stamped_ptr_->header;
  pose_stamped_ptr->pose = corrent_pose_with_cov_stamped_ptr_->pose.pose;
  path_ptr_->poses.push_back(*pose_stamped_ptr);
  path_pub_->publish(*path_ptr_);

  last_scan_ptr_ = msg;
}
