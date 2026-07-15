/*
 * ros_params_helper.h
 *
 * Created on: Feb 22, 2013
 * Author: cforster
 * Update on: Feb 08, 2025
 * Author: StrangeFly
 *
 * from libpointmatcher_ros
 */

#ifndef ROS_PARAMS_HELPER_H_
#define ROS_PARAMS_HELPER_H_

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace vk {

inline bool hasParam(const rclcpp::Node::SharedPtr& nh,
                     const std::string& name) {
  return nh->has_parameter(name);
}

// -----------------------------------------------------------------------------
// 1. Get Param with Default Value (Generic Type)
// -----------------------------------------------------------------------------
template <typename T>
T getParam(const rclcpp::Node::SharedPtr& nh, const std::string& name,
           const T& defaultValue) {
  T v;
  if (nh->has_parameter(name)) {
    v = nh->get_parameter(name).get_value<T>();
    RCLCPP_INFO(nh->get_logger(), "Found parameter: %s, value: %s",
                name.c_str(), std::to_string(v).c_str());
    return v;
  } else {
    // 【修复】declare_parameter 会优先使用 YAML 中的值，如果没有才使用
    // defaultValue 我们必须接收它的返回值，而不是直接返回 defaultValue
    try {
      v = nh->declare_parameter(name, defaultValue);
      RCLCPP_INFO(nh->get_logger(), "Declared parameter: %s, value: %s",
                  name.c_str(), std::to_string(v).c_str());
      return v;
    } catch (const std::exception& e) {
      RCLCPP_WARN(nh->get_logger(),
                  "Failed to declare parameter %s: %s. Returning default.",
                  name.c_str(), e.what());
      return defaultValue;
    }
  }
}

// -----------------------------------------------------------------------------
// 2. Get Param with Default Value (String Specialization)
// -----------------------------------------------------------------------------
template <>
inline std::string getParam<std::string>(const rclcpp::Node::SharedPtr& nh,
                                         const std::string& name,
                                         const std::string& defaultValue) {
  std::string v;
  if (nh->has_parameter(name)) {
    v = nh->get_parameter(name).get_value<std::string>();
    RCLCPP_INFO(nh->get_logger(), "Found parameter: %s, value: %s",
                name.c_str(), v.c_str());
    return v;
  } else {
    // 【修复】同上，接收返回值
    try {
      v = nh->declare_parameter(name, defaultValue);
      RCLCPP_INFO(nh->get_logger(), "Declared parameter: %s, value: %s",
                  name.c_str(), v.c_str());
      return v;
    } catch (const std::exception& e) {
      RCLCPP_WARN(nh->get_logger(),
                  "Failed to declare parameter %s: %s. Returning default.",
                  name.c_str(), e.what());
      return defaultValue;
    }
  }
}

// -----------------------------------------------------------------------------
// 3. Get Param WITHOUT Default Value (Generic Type)
// -----------------------------------------------------------------------------
template <typename T>
typename std::enable_if<!std::is_same<T, std::string>::value, T>::type getParam(
    const rclcpp::Node::SharedPtr& nh, const std::string& name) {
  // 【修复】如果参数不存在，尝试声明它 (不带默认值)
  // 这样如果 YAML 中有该值，它就会被声明并可用
  if (!nh->has_parameter(name)) {
    try {
      nh->declare_parameter<T>(name);
    } catch (...) {
      // 忽略重复声明等错误，继续尝试获取
    }
  }

  T v;
  int i = 0;
  while (!nh->get_parameter(name, v)) {
    RCLCPP_ERROR(nh->get_logger(),
                 "Cannot find value for parameter: %s, will try again.",
                 name.c_str());
    if ((i++) >= 5) return T();
    // 简单的延时，避免疯狂刷屏
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(nh->get_logger(), "Found parameter: %s, value: %s", name.c_str(),
              std::to_string(v).c_str());
  return v;
}

// -----------------------------------------------------------------------------
// 4. Get Param WITHOUT Default Value (String Specialization)
// -----------------------------------------------------------------------------
template <typename T>
typename std::enable_if<std::is_same<T, std::string>::value, T>::type getParam(
    const rclcpp::Node::SharedPtr& nh, const std::string& name) {
  // 【修复】同上，先尝试声明
  if (!nh->has_parameter(name)) {
    try {
      nh->declare_parameter<T>(name);
    } catch (...) {
    }
  }

  T v;
  int i = 0;
  while (!nh->get_parameter(name, v)) {
    RCLCPP_ERROR(nh->get_logger(),
                 "Cannot find value for parameter: %s, will try again.",
                 name.c_str());
    if ((i++) >= 5) return T();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  RCLCPP_INFO(nh->get_logger(), "Found parameter: %s, value: %s", name.c_str(),
              v.c_str());
  return v;
}

}  // namespace vk

#endif  // ROS_PARAMS_HELPER_H_