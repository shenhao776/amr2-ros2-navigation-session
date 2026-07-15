cmake_minimum_required(VERSION 3.5.2)
project(livox_to_pointcloud2)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

###########
## Build ##
###########

if($ENV{ROS_VERSION} EQUAL 2)
  ##########################
  ## ament-related (ROS2) ##
  ##########################
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  if(livox_ros_driver2_FOUND)
    message(STATUS "Found livox_ros_driver2")
    add_compile_definitions(LIVOX_ROS_DRIVER2)

    # Fix: livox_interfaces2__rosidl_generator_py may reference a non-existent
    # numpy include path when built in a different environment (e.g. container).
    if(TARGET livox_ros_driver2::livox_interfaces2__rosidl_generator_py)
      execute_process(
        COMMAND python3 -c "import numpy; print(numpy.get_include())"
        OUTPUT_VARIABLE _FIX_NUMPY_INCLUDE
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
      )
      if(_FIX_NUMPY_INCLUDE AND EXISTS "${_FIX_NUMPY_INCLUDE}")
        get_target_property(_fix_inc_dirs livox_ros_driver2::livox_interfaces2__rosidl_generator_py INTERFACE_INCLUDE_DIRECTORIES)
        if(_fix_inc_dirs)
          string(REPLACE "/usr/local/lib/python3.10/dist-packages/numpy/core/include" "${_FIX_NUMPY_INCLUDE}" _fix_inc_dirs "${_fix_inc_dirs}")
          set_target_properties(livox_ros_driver2::livox_interfaces2__rosidl_generator_py PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${_fix_inc_dirs}")
          message(STATUS "Fixed numpy include for livox_interfaces2: ${_FIX_NUMPY_INCLUDE}")
        endif()
      endif()
    endif()
  else()
    message(FATAL_ERROR "livox_ros_driver2 not found! Please source the workspace where it is installed.")
  endif()

  # Component
  ament_auto_add_library(livox_to_pointcloud2 SHARED
    src/livox_to_pointcloud2_ros2.cpp
  )
  rclcpp_components_register_nodes(livox_to_pointcloud2 "livox_to_pointcloud2::LivoxToPointCloud2")

  # Node
  ament_auto_add_executable(livox_to_pointcloud2_node
    src/livox_to_pointcloud2_ros2_node.cpp
  )
  target_include_directories(livox_to_pointcloud2_node PUBLIC
    include
  )
  target_link_libraries(livox_to_pointcloud2_node
    livox_to_pointcloud2
  )

  # New node for PointCloud2 to Livox conversion
  ament_auto_add_library(pointcloud2_to_livox SHARED
    src/pointcloud2_to_livox_ros2.cpp
  )
  rclcpp_components_register_nodes(pointcloud2_to_livox "pointcloud2_to_livox::PointCloud2ToLivox")

  ament_auto_add_executable(pointcloud2_to_livox_node
    src/pointcloud2_to_livox_ros2_node.cpp
  )
  target_include_directories(pointcloud2_to_livox_node PUBLIC
    include
  )
  target_link_libraries(pointcloud2_to_livox_node
    pointcloud2_to_livox
  )
  ament_auto_package()
else()
  # This part is for ROS1, you can ignore it for Humble
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    livox_ros_driver
  )
  catkin_package()

  add_executable(livox_to_pointcloud2_node
    src/livox_to_pointcloud2_ros1.cpp
  )
  target_include_directories(livox_to_pointcloud2_node PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(livox_to_pointcloud2_node
    ${catkin_LIBRARIES}
  )
  add_dependencies(livox_to_pointcloud2_node
    ${catkin_EXPORTED_TARGETS}
  )
endif()