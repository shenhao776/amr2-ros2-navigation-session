#pragma once

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pointcloud2_to_livox {

class PointCloud2ToLivoxConverter {
 public:
  PointCloud2ToLivoxConverter() {}

  livox_ros_driver2::msg::CustomMsg::UniquePtr convert(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pointcloud2_msg) {
    auto livox_msg = std::make_unique<livox_ros_driver2::msg::CustomMsg>();

    livox_msg->header = pointcloud2_msg->header;
    livox_msg->timebase = 0;  // Or get it from somewhere else if available
    livox_msg->point_num = pointcloud2_msg->width * pointcloud2_msg->height;
    livox_msg->lidar_id = 0;  // Or set a specific ID

    livox_msg->points.resize(livox_msg->point_num);

    const unsigned char* ptr = pointcloud2_msg->data.data();
    for (uint32_t i = 0; i < livox_msg->point_num; i++) {
      // Create a map to hold field offsets for clarity
      std::map<std::string, int> field_offsets;
      for (const auto& field : pointcloud2_msg->fields) {
        field_offsets[field.name] = field.offset;
      }

      livox_msg->points[i].x =
          *reinterpret_cast<const float*>(ptr + field_offsets["x"]);
      livox_msg->points[i].y =
          *reinterpret_cast<const float*>(ptr + field_offsets["y"]);
      livox_msg->points[i].z =
          *reinterpret_cast<const float*>(ptr + field_offsets["z"]);
      livox_msg->points[i].offset_time =
          *reinterpret_cast<const std::uint32_t*>(ptr + field_offsets["t"]);
      livox_msg->points[i].reflectivity =
          *reinterpret_cast<const float*>(ptr + field_offsets["intensity"]);
      livox_msg->points[i].tag =
          *reinterpret_cast<const std::uint8_t*>(ptr + field_offsets["tag"]);
      livox_msg->points[i].line =
          *reinterpret_cast<const std::uint8_t*>(ptr + field_offsets["line"]);

      ptr += pointcloud2_msg->point_step;
    }

    return livox_msg;
  }
};

}  // namespace pointcloud2_to_livox