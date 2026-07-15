/*
 * Mapping timer and map-saving logic for AMR2.
 *
 * This file intentionally contains mapping-only behavior. Loading and updating
 * an existing map are outside the scope of this project.
 */

#include "LIVMapper.h"

#include <pcl/io/pcd_io.h>

#include <filesystem>

void LIVMapper::timer_callback() {
  if (auto_mode_enabled_ && !auto_save_triggered_) {
    if (start_time_.time_since_epoch().count() == 0) {
      start_time_ = std::chrono::steady_clock::now();
      last_message_ros_time_ = this->now();
    }

    if ((this->now() - last_message_ros_time_).seconds() > message_timeout_) {
      LOG_INFO_F("\033[1;33m[Auto Mode] Timeout. Saving map...\033[0m");
      auto_save_triggered_ = true;
      auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto res = std::make_shared<std_srvs::srv::Trigger::Response>();
      saveMapCallback(req, res);
    }
  }

  if (sync_packages(LidarMeasures)) {
    if (auto_mode_enabled_) {
      last_message_ros_time_ = this->now();
    }
    handleFirstFrame();
    processImu();
    stateEstimationAndMapping();
  }
}

void LIVMapper::savePCD() {
  if (!pcd_save_en) {
    LOG_WARN("Map saving is disabled by pcd_save.pcd_save_en.");
    return;
  }

  const std::string pcd_dir = map_data_path_ + "/PCD/";
  std::filesystem::create_directories(pcd_dir);
  const std::string filtered_path = pcd_dir + "map_final_filtered.pcd";
  const std::string visualization_path =
      pcd_dir + "map_final_visualization.pcd";
  const std::string raw_path = pcd_dir + "map_raw.pcd";
  pcl::PCDWriter writer;

  if (img_en) {
    if (pcl_wait_save_->empty()) {
      LOG_WARN("Map save requested, but the RGB map is empty.");
      return;
    }

    auto filtered = std::make_shared<PointCloudXYZRGB>();
    pcl::VoxelGrid<PointTypeRGB> voxel_filter;
    voxel_filter.setInputCloud(pcl_wait_save_);
    voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd,
                             filter_size_pcd);
    voxel_filter.filter(*filtered);

    writer.writeBinary(filtered_path, *filtered);
    writer.writeBinary(visualization_path, *filtered);
    writer.writeBinary(raw_path, *pcl_wait_save_);
    LOG_INFO_F("[Map Save] RGB map saved with %lu filtered points.",
               filtered->size());
    return;
  }

  if (pcl_wait_save_intensity_->empty()) {
    LOG_WARN("Map save requested, but the intensity map is empty.");
    return;
  }

  auto filtered = std::make_shared<PointCloudXYZI>();
  pcl::VoxelGrid<PointType> voxel_filter;
  voxel_filter.setInputCloud(pcl_wait_save_intensity_);
  voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
  voxel_filter.filter(*filtered);

  writer.writeBinary(filtered_path, *filtered);
  writer.writeBinary(visualization_path, *filtered);
  writer.writeBinary(raw_path, *pcl_wait_save_intensity_);
  LOG_INFO_F("[Map Save] Intensity map saved with %lu filtered points.",
             filtered->size());
}

void LIVMapper::saveMapCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  (void)req;
  LOG_INFO("\033[1;32mService /save_map called. Saving map...\033[0m");

  savePCD();

  const std::string map_path =
      map_data_path_ + "/PCD/map_final_visualization.pcd";
  std::error_code error;
  const bool saved = std::filesystem::exists(map_path, error) &&
                     std::filesystem::file_size(map_path, error) > 0;

  res->success = saved && !error;
  if (res->success) {
    res->message = "Map saved to " + map_path;
  } else {
    res->message = "Map save failed: no non-empty visualization map was created.";
    LOG_ERROR_F("%s", res->message.c_str());
  }
}
