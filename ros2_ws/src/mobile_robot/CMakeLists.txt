cmake_minimum_required(VERSION 3.8)
project(mobile_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Install launch, rviz, config files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Beginner command-line tool for sending an x/y/yaw goal to Nav2.
install(
  PROGRAMS scripts/send_nav_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

# 1. Twist Controller Node (C++ replacement for odometer.py)
add_executable(twist_controller_node node/twist_controller_node.cpp src/twist_controller.cpp)
ament_target_dependencies(twist_controller_node
  "rclcpp"
  "geometry_msgs"
  "sensor_msgs"
)
install(TARGETS twist_controller_node DESTINATION lib/${PROJECT_NAME})

# 2. UDP Server Node (Networking only)
add_executable(udp_server_node node/udp_server_node.cpp src/udp_server.cpp)
ament_target_dependencies(udp_server_node
  "rclcpp"
  "std_msgs"
)
install(TARGETS udp_server_node DESTINATION lib/${PROJECT_NAME})

# 3. Robot Agent Node (Logic and Control)
add_executable(robot_agent_node node/robot_agent_node.cpp src/robot_agent.cpp)
ament_target_dependencies(robot_agent_node
  "rclcpp"
  "geometry_msgs"
  "nav2_msgs"
  "rclcpp_action"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "std_msgs"
  "pcl_conversions"
  "PCL"
  "ament_index_cpp"
)
target_link_libraries(robot_agent_node ${PCL_LIBRARIES})
install(TARGETS robot_agent_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
