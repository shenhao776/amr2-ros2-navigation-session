/* This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.

Modified by: SHEN HAO
Email: shenhao776@gmail.com

This code is a modified version of the FAST-LIVO2 framework.
Original repository: https://github.com/hku-mars/FAST-LIVO2
*/

#include "LIVMapper.h"

#include <pcl/common/transforms.h>  // [新增] 用于 transformPointCloud
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <filesystem>
#include <functional>
#include <iomanip>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// 静态指针初始化
LIVMapper* LIVMapper::ptr_ = nullptr;

// 信号处理函数
void LIVMapper::OnSignal(int sig) {
  if (ptr_) {
    ptr_->flg_exit = true;
    std::cout << "catch sig " << sig << std::endl;
    ptr_->sig_buffer.notify_all();
  }
  rclcpp::shutdown();
}

// 读取参数实现
void LIVMapper::readParameters() {
  // 辅助 Lambda：声明并获取参数
  auto get_param = [&](const std::string& name, auto& var, auto default_val) {
    if (!this->has_parameter(name)) {
      this->declare_parameter(name, default_val);
    }
    this->get_parameter(name, var);
  };

  get_param("common.lid_topic", lid_topic, std::string("/livox/lidar"));
  get_param("common.imu_topic", imu_topic, std::string("/livox/imu"));
  get_param("common.ros_driver_bug_fix", ros_driver_fix_en, false);
  get_param("common.img_en", img_en, 1);
  get_param("common.lidar_en", lidar_en, 1);
  get_param("common.img_topic", img_topic, std::string("/left_camera/image"));

  // [新增] NDT 对齐参数
  get_param("common.align_with_ndt", align_with_ndt_en,
            true);  // 默认为 true 开启
  get_param("common.ndt_topic", ndt_topic, std::string("/ndt_pose"));

  get_param("vio.normal_en", normal_en, true);
  get_param("vio.inverse_composition_en", inverse_composition_en, false);
  get_param("vio.max_iterations", max_iterations, 5);
  get_param("vio.img_point_cov", IMG_POINT_COV, 100.0);
  get_param("vio.raycast_en", raycast_en, false);
  get_param("vio.exposure_estimate_en", exposure_estimate_en, true);
  get_param("vio.inv_expo_cov", inv_expo_cov, 0.2);
  get_param("vio.grid_size", grid_size, 5);
  get_param("vio.grid_n_height", grid_n_height, 17);
  get_param("vio.patch_pyrimid_level", patch_pyrimid_level, 3);
  get_param("vio.patch_size", patch_size, 8);
  get_param("vio.outlier_threshold", outlier_threshold, 1000.0);

  get_param("time_offset.exposure_time_init", exposure_time_init, 0.0);
  get_param("time_offset.img_time_offset", img_time_offset, 0.0);
  get_param("time_offset.imu_time_offset", imu_time_offset, 0.0);
  get_param("time_offset.lidar_time_offset", lidar_time_offset, 0.0);
  get_param("time_offset.phone_time_offset", phone_time_offset, 0.0);
  get_param("uav.imu_rate_odom", imu_prop_enable, false);
  get_param("uav.gravity_align_en", gravity_align_en, false);

  get_param("evo.seq_name", seq_name, std::string("01"));
  get_param("evo.pose_output_en", pose_output_en, false);

  get_param("imu.gyr_cov", gyr_cov, 1.0);
  get_param("imu.acc_cov", acc_cov, 1.0);
  get_param("imu.imu_int_frame", imu_int_frame, 3);
  get_param("imu.imu_en", imu_en, false);
  get_param("imu.gravity_est_en", gravity_est_en, true);
  get_param("imu.ba_bg_est_en", ba_bg_est_en, true);

  get_param("preprocess.blind", p_pre->blind, 0.01);
  get_param("preprocess.filter_size_surf", filter_size_surf_min, 0.5);
  get_param("preprocess.hilti_en", hilti_en, false);
  get_param("preprocess.lidar_type", p_pre->lidar_type, (int)AVIA);
  get_param("preprocess.scan_line", p_pre->N_SCANS, 6);
  get_param("preprocess.point_filter_num", p_pre->point_filter_num, 3);
  get_param("preprocess.feature_extract_enabled", p_pre->feature_enabled,
            false);

  get_param("pcd_save.pcd_save_en", pcd_save_en, false);
  get_param("pcd_save.colmap_output_en", colmap_output_en, false);
  get_param("pcd_save.filter_size_pcd", filter_size_pcd, 0.5);
  get_param("pcd_save.pcd_save_distance_thresh", pcd_save_distance_thresh_,
            0.2);
  get_param("pcd_save.map_data_path", map_data_path_,
            std::string(ROOT_DIR) + "Log");

  get_param("extrin_calib.Pil", extrinT, std::vector<double>());
  get_param("extrin_calib.Ril", extrinR, std::vector<double>());
  get_param("extrin_calib.Pcl", cameraextrinT, std::vector<double>());
  get_param("extrin_calib.Rcl", cameraextrinR, std::vector<double>());

  get_param("debug.plot_time", plot_time, -10.0);
  get_param("debug.frame_cnt", frame_cnt, 6);

  get_param("publish.blind_rgb_points", blind_rgb_points, 0.01);
  get_param("publish.pub_scan_num", pub_scan_num, 1);
  get_param("publish.pub_effect_point_en", pub_effect_point_en, false);
  get_param("publish.dense_map_en", dense_map_en, false);

  get_param("auto_mode.enabled", auto_mode_enabled_, false);
  get_param("auto_mode.message_timeout", message_timeout_, 5.0);

  p_pre->blind_sqr = p_pre->blind * p_pre->blind;
}

LIVMapper::LIVMapper(const rclcpp::NodeOptions& options)
    : Node("map_updater", options), extT(0, 0, 0), extR(M3D::Identity()) {
  ptr_ = this;

  // 初始化外参变量
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);
  cameraextrinT.assign(3, 0.0);
  cameraextrinR.assign(9, 0.0);

  last_pcd_save_pos_.setZero();
  p_pre = std::make_shared<Preprocess>();
  p_imu = std::make_shared<ImuProcess>();

  // 初始化回调组为 Reentrant (可重入)，允许并行执行回调
  callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  readParameters();

  VoxelMapConfig voxel_config;
  loadVoxelConfig(this, voxel_config);

  LOG_INFO_F("\033[1;32mRunning in MAPPING mode.\033[0m");

  // 初始化管理器
  voxelmap_manager = std::make_shared<VoxelMapManager>(voxel_config, voxel_map);
  vio_manager = std::make_shared<VIOManager>();

  // 初始化点云指针
  visual_sub_map_ = std::make_shared<PointCloudXYZI>();
  feats_undistort_ = std::make_shared<PointCloudXYZI>();
  feats_down_body_ = std::make_shared<PointCloudXYZI>();
  feats_down_world_ = std::make_shared<PointCloudXYZI>();
  pcl_w_wait_pub_ = std::make_shared<PointCloudXYZI>();
  pcl_wait_pub_ = std::make_shared<PointCloudXYZI>();
  pcl_wait_save_ = std::make_shared<PointCloudXYZRGB>();
  pcl_wait_save_intensity_ = std::make_shared<PointCloudXYZI>();
  root_dir = ROOT_DIR;

  initializeFiles();

  path.header.stamp = this->now();
  path.header.frame_id = "camera_init";

  // 初始化 ROS 2 订阅者和发布者
  initializeSubscribersAndPublishers();
}

LIVMapper::~LIVMapper() {
  if (ptr_ == this) ptr_ = nullptr;
}

void LIVMapper::initializeComponents() {
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min,
                                 filter_size_surf_min);

  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  // 使用 shared_from_this() 传递节点指针
  if (!vk::camera_loader::loadFromRosNs(shared_from_this(), "",
                                        vio_manager->cam))
    throw std::runtime_error("Camera model not correctly specified.");

  vio_manager->grid_size = grid_size;
  vio_manager->patch_size = patch_size;
  vio_manager->outlier_threshold = outlier_threshold;
  vio_manager->setImuToLidarExtrinsic(extT, extR);
  vio_manager->setLidarToCameraExtrinsic(cameraextrinR, cameraextrinT);
  vio_manager->state = &_state;
  vio_manager->state_propagat = &state_propagat;
  vio_manager->max_iterations = max_iterations;
  vio_manager->img_point_cov = IMG_POINT_COV;
  vio_manager->normal_en = normal_en;
  vio_manager->inverse_composition_en = inverse_composition_en;
  vio_manager->raycast_en = raycast_en;
  vio_manager->grid_n_width = grid_n_width;
  vio_manager->grid_n_height = grid_n_height;
  vio_manager->patch_pyrimid_level = patch_pyrimid_level;
  vio_manager->exposure_estimate_en = exposure_estimate_en;
  vio_manager->colmap_output_en = colmap_output_en;
  vio_manager->initializeVIO();

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_inv_expo_cov(inv_expo_cov);
  p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_imu_init_frame_num(imu_int_frame);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();
  if (!exposure_estimate_en) p_imu->disable_exposure_est();

  slam_mode_ = (img_en && lidar_en) ? LIVO : imu_en ? ONLY_LIO : ONLY_LO;
}

void LIVMapper::initializeFiles() {
  LOG_INFO("[WARN] Mapping mode: Cleaning up old data in " << map_data_path_);
  std::filesystem::remove_all(map_data_path_);
  std::filesystem::create_directories(map_data_path_);

  if (pcd_save_en) {
    std::filesystem::create_directories(map_data_path_ + "/PCD/");
  }

  if (pose_output_en) {
    std::filesystem::create_directories(map_data_path_ + "/result/images/");
    std::filesystem::create_directories(map_data_path_ + "/result/keyframes/");
  }
}

void LIVMapper::initializeSubscribersAndPublishers() {
  // 使用多线程回调组
  rclcpp::SubscriptionOptions sub_opt;
  sub_opt.callback_group = callback_group_;

  if (p_pre->lidar_type == AVIA) {
    sub_pcl_livox =
        this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lid_topic, 20,
            std::bind(&LIVMapper::livox_pcl_cbk, this, std::placeholders::_1),
            sub_opt);  // [修改] 添加 sub_opt
  } else {
    sub_pcl_pc = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lid_topic, rclcpp::SensorDataQoS(),
        std::bind(&LIVMapper::standard_pcl_cbk, this, std::placeholders::_1),
        sub_opt);
  }

  sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, 2000,
      std::bind(&LIVMapper::imu_cbk, this, std::placeholders::_1), sub_opt);

  sub_img = this->create_subscription<sensor_msgs::msg::Image>(
      img_topic, 2000,
      std::bind(&LIVMapper::img_cbk, this, std::placeholders::_1), sub_opt);

  // [新增] NDT 订阅
  if (align_with_ndt_en) {
    sub_ndt = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        ndt_topic, 100,
        std::bind(&LIVMapper::ndt_cbk, this, std::placeholders::_1), sub_opt);
    LOG_INFO_F("Subscribed to NDT topic: %s for trajectory alignment.",
               ndt_topic.c_str());
  }

  // 发布者初始化
  pubLaserCloudFullRes = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", 20);
  pubLaserCloudEffect = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_effected", 20);
  pubOdomAftMapped = this->create_publisher<nav_msgs::msg::Odometry>(
      "/aft_mapped_to_init", 10);
  pubPath = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  plane_pub = this->create_publisher<visualization_msgs::msg::Marker>(
      "/planner_normal", 1);
  mavros_pose_publisher =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "/mavros/vision_pose/pose", 10);
  pubImuPropOdom = this->create_publisher<nav_msgs::msg::Odometry>(
      "/LIVO2/imu_propagate", 10000);

  voxelmap_manager->voxel_map_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>("/planes",
                                                                   100);

  // 服务初始化
  save_map_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/save_map", std::bind(&LIVMapper::saveMapCallback, this,
                             std::placeholders::_1, std::placeholders::_2));

  LOG_INFO_F("Service /save_map is ready.");

  // 定时器初始化
  // 注意：如果希望定时器也并行执行，也可以给定时器添加 callback_group_。
  // 但通常为了线程安全，保持默认组与订阅者组分开或共用 Reentrant 组均可。
  // 这里保持默认配置，因为 MultiThreadedExecutor 会自动调度不同组（默认组 +
  // sub_cb_group_）的回调并行运行。
  imu_prop_timer =
      this->create_wall_timer(std::chrono::milliseconds(4),
                              std::bind(&LIVMapper::imu_prop_callback, this));

  // 主循环定时器，频率设为 200us (5000Hz) 以匹配原 while 循环
  main_timer =
      this->create_wall_timer(std::chrono::microseconds(200),
                              std::bind(&LIVMapper::timer_callback, this));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

}

// [新增] NDT 回调函数实现
void LIVMapper::ndt_cbk(geometry_msgs::msg::PoseStamped::UniquePtr msg) {
  std::lock_guard<std::mutex> lock(mtx_ndt_buffer_);

  double timestamp = get_time_sec(msg->header.stamp);
  Eigen::Vector3d t(msg->pose.position.x, msg->pose.position.y,
                    msg->pose.position.z);
  Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                       msg->pose.orientation.y, msg->pose.orientation.z);
  Sophus::SE3d pose(q, t);

  // 记录轨迹
  ndt_path_container_.emplace_back(timestamp, pose);

  // 记录第一帧 NDT 用于粗对齐
  if (!has_first_ndt_) {
    first_ndt_pose_ = pose;
    has_first_ndt_ = true;
    LOG_INFO_F("Received FIRST NDT Pose. Timestamp: %f", timestamp);
  }
}

void LIVMapper::handleFirstFrame() {
  if (!is_first_frame) {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time;
    is_first_frame = true;

    // [修改] 初始粗对齐逻辑
    if (align_with_ndt_en) {
      // 等待一小段时间确保NDT数据到达（如果NDT频率较低）
      // 这里简单检查是否有数据
      {
        std::lock_guard<std::mutex> lock(mtx_ndt_buffer_);
        if (has_first_ndt_) {
          // 将 LIO 的当前状态重置为 NDT 的第一帧位姿
          _state.pos_end = first_ndt_pose_.translation();
          _state.rot_end = first_ndt_pose_.rotationMatrix();

          // 同时更新 IMU 传播状态
          state_propagat.pos_end = _state.pos_end;
          state_propagat.rot_end = _state.rot_end;

          LOG_INFO(
              "\033[1;32m[Alignment] Initialized LIO state with FIRST NDT "
              "Pose!\033[0m");
          std::cout << "Init Pos: " << _state.pos_end.transpose() << std::endl;
        } else {
          LOG_WARN(
              "\033[1;33m[Alignment] Enabled but NO NDT received yet. Starting "
              "at origin.\033[0m");
        }
      }
    }
    cout << "FIRST LIDAR FRAME PROCESSED!" << endl;
  }
}

void LIVMapper::gravityAlignment() {
  if (!p_imu->imu_need_init && !gravity_align_finished) {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu() {
  p_imu->Process2(LidarMeasures, _state, feats_undistort_);
  if (gravity_align_en) gravityAlignment();

  mtx_buffer_imu_prop.lock();
  state_propagat = _state;
  mtx_buffer_imu_prop.unlock();

  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort_;
}

void LIVMapper::stateEstimationAndMapping() {
  switch (LidarMeasures.lio_vio_flg) {
    case VIO:
      handleVIO();
      break;
    case LIO:
    case LO:
      handleLIO();

      // [新增] 记录 FAST-LIVO 的轨迹用于后续对齐
      if (align_with_ndt_en && is_first_frame) {
        Sophus::SE3d current_pose(Eigen::Quaterniond(_state.rot_end),
                                  _state.pos_end);
        // 使用 LidarMeasures.last_lio_update_time 作为时间戳
        livo_path_container_.emplace_back(LidarMeasures.last_lio_update_time,
                                          current_pose);
      }
      break;
    default:
      break;
  }
}

void LIVMapper::handleVIO() {
  euler_cur = RotMtoEuler(_state.rot_end);

  if (pcl_w_wait_pub_->empty() || (pcl_w_wait_pub_ == nullptr)) {
    LOG_WARN("[ VIO ] No point!!!");
    return;
  }

  if (fabs((LidarMeasures.last_lio_update_time - _first_lidar_time) -
           plot_time) < (frame_cnt / 2 * 0.1)) {
    vio_manager->plot_flag = true;
  } else {
    vio_manager->plot_flag = false;
  }

  vio_manager->processFrame(
      LidarMeasures.measures.back().img, _pv_list, voxelmap_manager->voxel_map_,
      LidarMeasures.last_lio_update_time - _first_lidar_time);

  if (imu_prop_enable) {
    mtx_buffer_imu_prop.lock();
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
    mtx_buffer_imu_prop.unlock();
  }

  publish_frame_world(pubLaserCloudFullRes, vio_manager);
  publish_img_rgb(pubImage, vio_manager);

  euler_cur = RotMtoEuler(_state.rot_end);
}

void LIVMapper::handleLIO() {
  euler_cur = RotMtoEuler(_state.rot_end);

  if (feats_undistort_->empty() || (feats_undistort_ == nullptr)) {
    LOG_INFO("[ LIO ]: No point to process, skipping frame!!!");
    return;
  }

  double t0 = omp_get_wtime();

  // 1. 原始降采样
  downSizeFilterSurf.setInputCloud(feats_undistort_);
  downSizeFilterSurf.filter(*feats_down_body_);

  double t_down = omp_get_wtime();

  // =================================================================================
  // [动态物体去除]
  // =================================================================================
  // 仅在点云不为空且是非更新模式（建图模式）时执行过滤
  if (!feats_down_body_->empty()) {
    PointCloudXYZI::Ptr cloud_temp(new PointCloudXYZI());
    cloud_temp->reserve(feats_down_body_->size());

    // -----------------------------------------------------------------------------
    // 步骤 A: 绝对盲区剔除 (Blind Zone Filter)
    // -----------------------------------------------------------------------------
    const float remove_x_min = -5.0;   // 后方最远剔除距离
    const float remove_x_max = -0.3;   // 后方最近保留距离（避开车身）
    const float remove_y_width = 1.0;  // 左右宽度阈值 (即总宽2米)

    for (const auto& p : feats_down_body_->points) {
      // 如果点在“后方方框”内，直接跳过
      if (p.x > remove_x_min && p.x < remove_x_max &&
          std::abs(p.y) < remove_y_width) {
        continue;
      }
      cloud_temp->push_back(p);
    }

    // -----------------------------------------------------------------------------
    // 步骤 B: 统计离群点移除 (SOR)
    // -----------------------------------------------------------------------------
    PointCloudXYZI::Ptr cloud_filtered(new PointCloudXYZI());
    pcl::StatisticalOutlierRemoval<PointType> sor;
    sor.setInputCloud(cloud_temp);
    sor.setMeanK(30);             // 临近点数量：建议 30-50
    sor.setStddevMulThresh(0.5);  // 标准差倍数：建议 1.0，越小切得越狠
    sor.filter(*cloud_filtered);

    // 将处理完的干净点云赋值回 feats_down_body_
    *feats_down_body_ = *cloud_filtered;
  }
  // =================================================================================

  feats_down_size = feats_down_body_->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body_;

  double t1 = 0.0, t2 = 0.0;

  transformLidar(_state.rot_end, _state.pos_end, feats_down_body_,
                 feats_down_world_);
  voxelmap_manager->feats_down_world_ = feats_down_world_;
  voxelmap_manager->feats_down_size_ = feats_down_size;

  if (!lidar_map_inited) {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  t1 = omp_get_wtime();
  voxelmap_manager->StateEstimation(state_propagat);
  _state = voxelmap_manager->state_;
  _pv_list = voxelmap_manager->pv_list_;
  t2 = omp_get_wtime();

  if (imu_prop_enable) {
    mtx_buffer_imu_prop.lock();
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
    mtx_buffer_imu_prop.unlock();
  }

  if (pose_output_en) {
    std::ofstream evoFile;
    std::string pose_file_path =
        map_data_path_ + "/result/localization_output.txt";
    evoFile.open(pose_file_path, std::ios::app);
    if (evoFile.is_open()) {
      Eigen::Quaterniond q(_state.rot_end);
      double timestamp = LidarMeasures.last_lio_update_time;
      evoFile << std::fixed << std::setprecision(9) << timestamp << " "
              << _state.pos_end[0] << " " << _state.pos_end[1] << " "
              << _state.pos_end[2] << " " << q.x() << " " << q.y() << " "
              << q.z() << " " << q.w() << std::endl;
      evoFile.close();
    } else {
      LOG_INFO("[ERROR] Failed to open pose output file: " << pose_file_path);
    }

    double timestamp = LidarMeasures.last_lio_update_time;
    double dist_from_last_save = (_state.pos_end - last_pcd_save_pos_).norm();
    if (!is_first_pcd_saved_ ||
        dist_from_last_save > pcd_save_distance_thresh_) {
      last_pcd_save_pos_ = _state.pos_end;
      is_first_pcd_saved_ = true;

      std::stringstream ss;
      ss << std::fixed << std::setprecision(9) << timestamp;
      std::string ts_str = ss.str();
      std::string pcd_filename =
          map_data_path_ + "/result/keyframes/" + ts_str + ".pcd";
      pcl::io::savePCDFileBinary(pcd_filename, *feats_undistort_);
    }

    mtx_buffer.lock();
    if (!img_buffer.empty()) {
      double min_time_diff = std::numeric_limits<double>::max();
      int best_match_idx = -1;
      for (size_t i = 0; i < img_time_buffer.size(); ++i) {
        double time_diff = std::abs(img_time_buffer[i] - timestamp);
        if (time_diff < min_time_diff) {
          min_time_diff = time_diff;
          best_match_idx = i;
        }
      }
      if (best_match_idx != -1 && min_time_diff < 0.05) {
        cv::Mat image_to_save = img_buffer[best_match_idx];
        if (!image_to_save.empty()) {
          std::stringstream ss;
          ss << std::fixed << std::setprecision(9) << timestamp;
          std::string ts_str = ss.str();
          std::string img_filename =
              map_data_path_ + "/result/images/" + ts_str + ".png";

          // ========================== 修改部分开始 ==========================
          // 使用 std::thread 开启一个新线程进行保存
          // 我们使用 .clone()
          // 确保线程拥有独立的内存副本，防止主线程修改数据（虽然 cv::Mat
          // 是引用计数，但在多线程下 clone 最安全） 同时通过值捕获将 filename
          // 和 image 传入 lambda
          std::thread([img_filename, image_to_save]() {
            try {
              // 建议使用深拷贝后的副本写入，或者依赖 cv::Mat
              // 的引用计数机制（这里直接传值捕获即可增加引用计数）
              cv::imwrite(img_filename, image_to_save);
            } catch (const std::exception& e) {
              std::cerr << "Failed to save image in thread: " << e.what()
                        << std::endl;
            }
          }).detach();  // 分离线程，让其在后台运行，不阻塞主线程
          // ========================== 修改部分结束 ==========================

          LOG_INFO("Saved associated image for frame " << ts_str);
        }
        img_buffer.erase(img_buffer.begin(),
                         img_buffer.begin() + best_match_idx + 1);
        img_time_buffer.erase(img_time_buffer.begin(),
                              img_time_buffer.begin() + best_match_idx + 1);
      }
    }
    mtx_buffer.unlock();
  }

  euler_cur = RotMtoEuler(_state.rot_end);

  tf2::Quaternion my_q;
  my_q.setRPY(euler_cur(0), euler_cur(1), euler_cur(2));
  geoQuat.x = my_q.x();
  geoQuat.y = my_q.y();
  geoQuat.z = my_q.z();
  geoQuat.w = my_q.w();

  publish_odometry(pubOdomAftMapped);

  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar = std::make_shared<PointCloudXYZI>();
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body_, world_lidar);

  std::vector<pointWithVar> pv_list_for_update;
  pv_list_for_update.resize(world_lidar->points.size());
  for (size_t i = 0; i < world_lidar->points.size(); ++i) {
    pv_list_for_update[i].point_w << world_lidar->points[i].x,
        world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) *
              (-point_crossmat).transpose() +
          _state.cov.block<3, 3>(3, 3);
    pv_list_for_update[i].var = var;
  }
  voxelmap_manager->UpdateVoxelMap(pv_list_for_update);
  if (voxelmap_manager->config_setting_.map_sliding_en) {
    voxelmap_manager->mapSliding();
  }

  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort_
                                                     : feats_down_body_);
  size_t size = laserCloudFullRes->points.size();
  auto laserCloudWorld = std::make_shared<PointCloudXYZI>(size, 1);

  for (size_t i = 0; i < size; ++i) {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                        &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub_ = *laserCloudWorld;
  *pcl_wait_save_intensity_ += *laserCloudWorld;

  if (!img_en) publish_frame_world(pubLaserCloudFullRes, vio_manager);
  publish_path(pubPath);

  if (pub_effect_point_en)
    publish_effect_world(pubLaserCloudEffect, voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_)
    voxelmap_manager->pubVoxelMap(this->now());

  publish_mavros(mavros_pose_publisher);

  double t4 = omp_get_wtime();
  frame_num++;
  aver_time_consu =
      aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  printf(
      "\033[1;34m+---------------------------------------------------------"
      "--"
      "--"
      "+\033[0m\n");
  printf(
      "\033[1;34m|                         LIO Mapping Time                "
      "  "
      "  "
      "|\033[0m\n");
  printf(
      "\033[1;34m+---------------------------------------------------------"
      "--"
      "--"
      "+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage",
         "Time (secs)");
  printf(
      "\033[1;34m+---------------------------------------------------------"
      "--"
      "--"
      "+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "State Estimation", t2 - t1);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Publish & Update", t4 - t3);
  printf(
      "\033[1;34m+---------------------------------------------------------"
      "--"
      "--"
      "+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time",
         aver_time_consu);
  printf(
      "\033[1;34m+---------------------------------------------------------"
      "--"
      "--"
      "+\033[0m\n");
}

void LIVMapper::prop_imu_once(StatesGroup& imu_prop_state, const double dt,
                              V3D acc_avr, V3D angvel_avr) {
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;
  V3D acc_imu = imu_prop_state.rot_end * acc_avr +
                V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1],
                    imu_prop_state.gravity[2]);
  imu_prop_state.pos_end = imu_prop_state.pos_end +
                           imu_prop_state.vel_end * dt +
                           0.5 * acc_imu * dt * dt;
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback() {
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once) {
    return;
  }
  mtx_buffer_imu_prop.lock();
  new_imu = false;
  if (imu_prop_enable && !prop_imu_buffer.empty()) {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg) {
      imu_propagate = latest_ekf_state;
      while ((!prop_imu_buffer.empty() &&
              get_time_sec(prop_imu_buffer.front().header.stamp) <
                  latest_ekf_time)) {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (size_t i = 0; i < prop_imu_buffer.size(); ++i) {
        double t_from_lidar_end_time =
            get_time_sec(prop_imu_buffer[i].header.stamp) - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x,
                    prop_imu_buffer[i].linear_acceleration.y,
                    prop_imu_buffer[i].linear_acceleration.z);
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x,
                    prop_imu_buffer[i].angular_velocity.y,
                    prop_imu_buffer[i].angular_velocity.z);
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    } else {
      V3D acc_imu(newest_imu.linear_acceleration.x,
                  newest_imu.linear_acceleration.y,
                  newest_imu.linear_acceleration.z);
      V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y,
                  newest_imu.angular_velocity.z);
      double t_from_lidar_end_time =
          get_time_sec(newest_imu.header.stamp) - latest_ekf_time;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate.pos_end;
    vel_i = imu_propagate.vel_end;
    q = Eigen::Quaterniond(imu_propagate.rot_end);
    imu_prop_odom.header.frame_id = "world";
    imu_prop_odom.header.stamp = newest_imu.header.stamp;
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    pubImuPropOdom->publish(imu_prop_odom);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot,
                               const Eigen::Vector3d t,
                               const PointCloudXYZI::Ptr& input_cloud,
                               PointCloudXYZI::Ptr& trans_cloud) {
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); ++i) {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType& pi, PointType& po) {
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T>
void LIVMapper::pointBodyToWorld(const Eigen::Matrix<T, 3, 1>& pi,
                                 Eigen::Matrix<T, 3, 1>& po) {
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T>
Eigen::Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(
    const Eigen::Matrix<T, 3, 1>& pi) {
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Eigen::Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const* const pi,
                                    PointType* const po) {
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LIVMapper::standard_pcl_cbk(sensor_msgs::msg::PointCloud2::UniquePtr msg) {
  if (!lidar_en) return;
  mtx_buffer.lock();

  double cur_head_time = get_time_sec(msg->header.stamp) + lidar_time_offset;
  if (cur_head_time < last_timestamp_lidar) {
    LOG_ERROR("lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  std::shared_ptr<sensor_msgs::msg::PointCloud2> msg_ptr = std::move(msg);
  std::shared_ptr<PointCloudXYZI> ptr = std::make_shared<PointCloudXYZI>();
  p_pre->process(msg_ptr, ptr);
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(
    livox_ros_driver2::msg::CustomMsg::UniquePtr msg_in) {
  if (!lidar_en) return;
  mtx_buffer.lock();
  std::shared_ptr<livox_ros_driver2::msg::CustomMsg> msg_ptr =
      std::move(msg_in);

  double cur_head_time = get_time_sec(msg_ptr->header.stamp);
  if (cur_head_time < last_timestamp_lidar) {
    LOG_ERROR("lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }

  PointCloudXYZI::Ptr ptr = std::make_shared<PointCloudXYZI>();
  p_pre->process(msg_ptr, ptr);

  if (!ptr || ptr->empty()) {
    LOG_ERROR("Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::imu_cbk(sensor_msgs::msg::Imu::UniquePtr msg_in) {
  if (!imu_en) return;
  if (last_timestamp_lidar < 0.0) return;

  std::shared_ptr<sensor_msgs::msg::Imu> msg = std::move(msg_in);
  msg->header.stamp =
      get_ros_time(get_time_sec(msg->header.stamp) - imu_time_offset);
  double timestamp = get_time_sec(msg->header.stamp);
  if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en)) {
    LOG_WARN_F("IMU and LiDAR not synced! delta time: %lf .\n",
               last_timestamp_lidar - timestamp);
  }
  if (ros_driver_fix_en)
    timestamp += std::round(last_timestamp_lidar - timestamp);
  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu) {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    LOG_ERROR_F("imu loop back, offset: %lf \n",
                last_timestamp_imu - timestamp);
    return;
  }
  last_timestamp_imu = timestamp;
  imu_buffer.push_back(msg);
  mtx_buffer.unlock();

  if (imu_prop_enable) {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init) {
      prop_imu_buffer.push_back(*msg);
    }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}

cv::Mat LIVMapper::getImageFromMsg(
    const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) {
  cv::Mat img;
  try {
    img = cv_bridge::toCvCopy(*img_msg, "bgr8")->image;
  } catch (cv_bridge::Exception& e) {
    LOG_ERROR_F("cv_bridge exception: %s", e.what());
  }
  return img;
}

void LIVMapper::img_cbk(sensor_msgs::msg::Image::UniquePtr msg_in) {
  double msg_header_time = get_time_sec(msg_in->header.stamp) + img_time_offset;

  if (std::abs(msg_header_time - last_timestamp_img) < 1e-4) {
    return;
  }

  if (last_timestamp_lidar > 0 && msg_header_time < last_timestamp_img) {
    LOG_WARN("Image loop back, clearing buffer.");
    mtx_buffer.lock();
    img_buffer.clear();
    img_time_buffer.clear();
    mtx_buffer.unlock();
    last_timestamp_img = msg_header_time;
    return;
  }

  mtx_buffer.lock();
  std::shared_ptr<sensor_msgs::msg::Image> msg = std::move(msg_in);
  cv::Mat img_cur = getImageFromMsg(msg);
  if (!img_cur.empty()) {
    img_buffer.push_back(img_cur);
    img_time_buffer.push_back(msg_header_time);
    last_timestamp_img = msg_header_time;
  }
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool LIVMapper::sync_packages(LidarMeasureGroup& meas) {
  // 检查缓冲区是否为空（根据使能状态）
  // 注意：为了线程安全，建议在访问 size/empty
  // 时也加锁，或者依赖外部调用逻辑保证
  if (lid_raw_data_buffer.empty() && lidar_en) return false;
  if (img_buffer.empty() && img_en) return false;
  if (imu_buffer.empty() && imu_en) return false;

  switch (slam_mode_) {
    case ONLY_LIO: {
      // 初始化 last_lio_update_time
      if (meas.last_lio_update_time < 0.0) {
        if (!lid_header_time_buffer.empty()) {
          meas.last_lio_update_time = lid_header_time_buffer.front();
        } else {
          return false;
        }
      }

      // 推送一帧雷达数据
      if (!lidar_pushed) {
        meas.lidar = lid_raw_data_buffer.front();
        if (meas.lidar->points.size() <= 1) return false;
        meas.lidar_frame_beg_time = lid_header_time_buffer.front();
        meas.lidar_frame_end_time =
            meas.lidar_frame_beg_time +
            meas.lidar->points.back().curvature / double(1000);
        meas.pcl_proc_cur = meas.lidar;
        lidar_pushed = true;
      }
      if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time) {
        return false;
      }
      struct MeasureGroup m;
      m.imu.clear();
      m.lio_time = meas.lidar_frame_end_time;
      mtx_buffer.lock();
      while (!imu_buffer.empty()) {
        double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
        if (imu_time > meas.lidar_frame_end_time) break;
        m.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
      }
      lid_raw_data_buffer.pop_front();
      lid_header_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      meas.lio_vio_flg = LIO;
      meas.measures.push_back(m);
      lidar_pushed = false;
      return true;
      break;
    }
    case LIVO: {
      EKF_STATE last_lio_vio_flg = meas.lio_vio_flg;
      switch (last_lio_vio_flg) {
        case WAIT:
        case VIO: {
          double img_capture_time =
              img_time_buffer.front() + exposure_time_init;
          if (meas.last_lio_update_time < 0.0)
            meas.last_lio_update_time = lid_header_time_buffer.front();
          double lid_newest_time =
              lid_header_time_buffer.back() +
              lid_raw_data_buffer.back()->points.back().curvature /
                  double(1000);

          // [ROS 2] 获取最新 IMU 时间
          double imu_newest_time =
              get_time_sec(imu_buffer.back()->header.stamp);

          // 如果图像时间早于上一次 LIO 更新时间，说明图像过旧，丢弃
          if (img_capture_time < meas.last_lio_update_time + 0.00001) {
            mtx_buffer.lock();
            img_buffer.pop_front();
            img_time_buffer.pop_front();
            mtx_buffer.unlock();
            LOG_ERROR("[ Data Cut ] Throw one image frame!");
            return false;
          }
          if (img_capture_time > lid_newest_time ||
              img_capture_time > imu_newest_time) {
            return false;
          }
          struct MeasureGroup m;
          m.imu.clear();
          m.lio_time = img_capture_time;
          mtx_buffer.lock();
          while (!imu_buffer.empty()) {
            double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
            if (imu_time > m.lio_time) break;
            if (imu_time > meas.last_lio_update_time)
              m.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
          }
          mtx_buffer.unlock();
          sig_buffer.notify_all();
          *(meas.pcl_proc_cur) = *(meas.pcl_proc_next);
          PointCloudXYZI().swap(*meas.pcl_proc_next);
          int lid_frame_num = lid_raw_data_buffer.size();
          int max_size = meas.pcl_proc_cur->size() + 24000 * lid_frame_num;
          meas.pcl_proc_cur->reserve(max_size);
          meas.pcl_proc_next->reserve(max_size);
          while (!lid_raw_data_buffer.empty()) {
            if (lid_header_time_buffer.front() > img_capture_time) break;
            auto pcl_points = lid_raw_data_buffer.front()->points;
            double frame_header_time = lid_header_time_buffer.front();
            float max_offs_time_ms = (m.lio_time - frame_header_time) * 1000.0f;

            for (size_t i = 0; i < pcl_points.size(); i++) {
              auto pt = pcl_points[i];
              if (pt.curvature < max_offs_time_ms) {
                pt.curvature +=
                    (frame_header_time - meas.last_lio_update_time) * 1000.0f;
                meas.pcl_proc_cur->points.push_back(pt);
              } else {
                pt.curvature += (frame_header_time - m.lio_time) * 1000.0f;
                meas.pcl_proc_next->points.push_back(pt);
              }
            }
            mtx_buffer.lock();
            lid_raw_data_buffer.pop_front();
            lid_header_time_buffer.pop_front();
            mtx_buffer.unlock();
          }
          meas.measures.push_back(m);
          meas.lio_vio_flg = LIO;
          return true;
        }
        case LIO: {
          double img_capture_time =
              img_time_buffer.front() + exposure_time_init;
          meas.lio_vio_flg = VIO;
          meas.measures.clear();
          struct MeasureGroup m;
          m.vio_time = img_capture_time;
          m.lio_time = meas.last_lio_update_time;
          m.img = img_buffer.front();
          mtx_buffer.lock();
          img_buffer.pop_front();
          img_time_buffer.pop_front();
          mtx_buffer.unlock();
          sig_buffer.notify_all();
          meas.measures.push_back(m);
          lidar_pushed = false;
          return true;
        }
        default: {
          return false;
        }
      }
      break;
    }
    case ONLY_LO: {
      if (!lidar_pushed) {
        if (lid_raw_data_buffer.empty()) return false;
        meas.lidar = lid_raw_data_buffer.front();
        meas.lidar_frame_beg_time = lid_header_time_buffer.front();
        meas.lidar_frame_end_time =
            meas.lidar_frame_beg_time +
            meas.lidar->points.back().curvature / double(1000);
        lidar_pushed = true;
      }
      struct MeasureGroup m;
      m.lio_time = meas.lidar_frame_end_time;
      mtx_buffer.lock();
      lid_raw_data_buffer.pop_front();
      lid_header_time_buffer.pop_front();
      mtx_buffer.unlock();
      sig_buffer.notify_all();
      lidar_pushed = false;
      meas.lio_vio_flg = LO;
      meas.measures.push_back(m);
      return true;
      break;
    }
    default: {
      printf("!! WRONG SLAM TYPE !!");
      return false;
    }
  }
  LOG_ERROR("out sync");
  return false;
}

// [新增] 核心算法：时间同步 -> 最小二乘(SVD) -> 求解变换矩阵
void LIVMapper::performTrajectoryAlignment() {
  if (ndt_path_container_.empty() || livo_path_container_.empty()) {
    LOG_ERROR("Cannot align trajectories: Path containers are empty!");
    return;
  }

  std::vector<Eigen::Vector3d> src_points;  // LIVO points
  std::vector<Eigen::Vector3d> tgt_points;  // NDT points

  // 1. 数据关联：基于时间戳查找最近邻
  double time_threshold = 0.05;  // 50ms 容差
  int match_count = 0;

  // 简单的双指针或遍历查找
  for (const auto& livo_data : livo_path_container_) {
    double t_livo = livo_data.first;

    double min_dt = std::numeric_limits<double>::max();
    Sophus::SE3d best_ndt_pose;

    // 在 NDT 容器中搜索时间最近的
    // 注意：实际生产中可以使用二分查找优化，这里假设数据量不大直接遍历
    for (const auto& ndt_data : ndt_path_container_) {
      double dt = std::abs(ndt_data.first - t_livo);
      if (dt < min_dt) {
        min_dt = dt;
        best_ndt_pose = ndt_data.second;
      }
    }

    if (min_dt < time_threshold) {
      src_points.push_back(livo_data.second.translation());
      tgt_points.push_back(best_ndt_pose.translation());
      match_count++;
    }
  }

  if (match_count < 10) {
    LOG_ERROR(
        "Not enough matched points between NDT and LIVO trajectories for "
        "alignment.");
    return;
  }

  LOG_INFO_F("Aligned %d trajectory points. Computing transform...",
             match_count);

  // 2. 使用 PCL 的 SVD 估算转换矩阵 (LIVO -> NDT)
  // 转换 vector 到 PCL point cloud 格式以便使用 API，或者手动写 SVD
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt(
      new pcl::PointCloud<pcl::PointXYZ>());

  for (size_t i = 0; i < src_points.size(); ++i) {
    cloud_src->push_back(
        pcl::PointXYZ(src_points[i].x(), src_points[i].y(), src_points[i].z()));
    cloud_tgt->push_back(
        pcl::PointXYZ(tgt_points[i].x(), tgt_points[i].y(), tgt_points[i].z()));
  }

  pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>
      svd;
  Eigen::Matrix4f transformation_matrix;
  svd.estimateRigidTransformation(*cloud_src, *cloud_tgt,
                                  transformation_matrix);

  T_livo_to_ndt_ = transformation_matrix.cast<double>();

  std::cout << "\033[1;32m[Trajectory Alignment] Calculated Correction Matrix "
               "(LIVO -> NDT):\n"
            << T_livo_to_ndt_ << "\033[0m" << std::endl;
}

// ==============================================================================
//                                 发布与工具函数
// ==============================================================================
// ==============================================================================
//                             图像与点云发布函数
// ==============================================================================

void LIVMapper::publish_img_rgb(const image_transport::Publisher& pubImage,
                                VIOManagerPtr vio_manager) {
  // 获取当前帧的图像副本
  cv::Mat img_rgb = vio_manager->img_cp;

  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = this->now();
  out_msg.header.frame_id = "camera_init";
  out_msg.encoding = sensor_msgs::image_encodings::BGR8;
  out_msg.image = img_rgb;

  pubImage.publish(out_msg.toImageMsg());
}

void LIVMapper::publish_frame_world(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr&
        pubLaserCloudFullRes,
    VIOManagerPtr vio_manager) {
  if (pcl_w_wait_pub_->empty()) return;
  PointCloudXYZRGB::Ptr laserCloudWorldRGB =
      std::make_shared<PointCloudXYZRGB>();
  if (img_en) {
    static int pub_num = 1;
    *pcl_wait_pub_ += *pcl_w_wait_pub_;
    if (pub_num == pub_scan_num) {
      pub_num = 1;
      size_t size = pcl_wait_pub_->points.size();
      laserCloudWorldRGB->reserve(size);
      cv::Mat img_rgb = vio_manager->img_rgb;
      for (size_t i = 0; i < size; ++i) {
        PointTypeRGB pointRGB;
        pointRGB.x = pcl_wait_pub_->points[i].x;
        pointRGB.y = pcl_wait_pub_->points[i].y;
        pointRGB.z = pcl_wait_pub_->points[i].z;

        // 将点转换到世界坐标系
        V3D p_w(pcl_wait_pub_->points[i].x, pcl_wait_pub_->points[i].y,
                pcl_wait_pub_->points[i].z);

        // 转换到当前相机坐标系 (frame)
        V3D pf(vio_manager->new_frame_->w2f(p_w));

        // 过滤掉相机后方的点
        if (pf[2] < 0) {
          continue;
        }

        // 投影到像素坐标
        V2D pc(vio_manager->new_frame_->w2c(p_w));

        // 检查点是否在图像视野内
        if (vio_manager->new_frame_->cam_->isInFrame(pc.cast<int>(), 3)) {
          // 获取插值后的像素颜色
          V3F pixel = vio_manager->getInterpolatedPixel(img_rgb, pc);
          pointRGB.r = pixel[2];
          pointRGB.g = pixel[1];
          pointRGB.b = pixel[0];

          // 过滤掉过近的噪点 (blind spot)
          if (pf.norm() > blind_rgb_points)
            laserCloudWorldRGB->push_back(pointRGB);
        }
      }
      *pcl_wait_save_ += *laserCloudWorldRGB;
    } else {
      pub_num++;
    }
  }

  sensor_msgs::msg::PointCloud2 laserCloudmsg;

  // 根据是否启用视觉，发布彩色点云或原始强度点云
  if (img_en) {
    pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
  } else {
    pcl::toROSMsg(*pcl_w_wait_pub_, laserCloudmsg);
  }

  // [ROS 2] 设置时间戳和坐标系
  laserCloudmsg.header.stamp = this->now();
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes->publish(laserCloudmsg);

  // 清理缓冲区
  if (laserCloudWorldRGB->size() > 0) {
    PointCloudXYZI().swap(*pcl_wait_pub_);
  }
  PointCloudXYZI().swap(*pcl_w_wait_pub_);
}

void LIVMapper::publish_effect_world(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr&
        pubLaserCloudEffect,
    const std::vector<PointToPlane>& ptpl_list) {
  size_t effect_feat_num = ptpl_list.size();
  auto laserCloudWorld = std::make_shared<PointCloudXYZI>(effect_feat_num, 1);
  for (size_t i = 0; i < effect_feat_num; ++i) {
    // 将 PointToPlane 中的点转换到 PCL 点云中
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
    // 这里通常可以把残差距离作为强度存入，方便在 RViz 中观察误差分布
    // laserCloudWorld->points[i].intensity = ptpl_list[i].dis_to_plane_;
  }

  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = this->now();  // ROS 2 时间
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudEffect->publish(laserCloudFullRes3);
}

template <typename T>
void LIVMapper::set_posestamp(T& out) {
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry(
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr&
        pubOdomAftMapped) {
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "aft_mapped";
  odomAftMapped.header.stamp = get_ros_time(LidarMeasures.lidar_frame_end_time);
  set_posestamp(odomAftMapped.pose.pose);

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = odomAftMapped.header.stamp;
  transform.header.frame_id = "camera_init";
  transform.child_frame_id = "aft_mapped";
  transform.transform.translation.x = _state.pos_end(0);
  transform.transform.translation.y = _state.pos_end(1);
  transform.transform.translation.z = _state.pos_end(2);
  transform.transform.rotation.w = geoQuat.w;
  transform.transform.rotation.x = geoQuat.x;
  transform.transform.rotation.y = geoQuat.y;
  transform.transform.rotation.z = geoQuat.z;

  tf_broadcaster_->sendTransform(transform);
  pubOdomAftMapped->publish(odomAftMapped);
}
void LIVMapper::publish_mavros(
    const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr&
        mavros_pose_publisher) {
  msg_body_pose.header.stamp = this->now();
  msg_body_pose.header.frame_id = "camera_init";
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher->publish(msg_body_pose);
}

void LIVMapper::publish_path(
    const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath) {
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = this->now();
  msg_body_pose.header.frame_id = "camera_init";
  path.poses.push_back(msg_body_pose);
  path.header.frame_id = "camera_init";
  path.header.stamp = this->now();
  pubPath->publish(path);
}
