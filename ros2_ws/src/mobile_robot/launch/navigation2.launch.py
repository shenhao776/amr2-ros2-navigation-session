# navigation2.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    # [新增] 接收上层传来的日志配置
    log_level = LaunchConfiguration("log_level", default="warn")

    pkg_indoor_robot_share = get_package_share_directory("mobile_robot")

    param_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            pkg_indoor_robot_share, "config", "nav2_params_3d_scan_outdoor.yaml"
        ),
    )

    nav2_bringup_launch_dir = os.path.join(pkg_indoor_robot_share, "launch", "include")
    rviz_config_file = os.path.join(pkg_indoor_robot_share, "rviz", "nav2.rviz")
    lidar_localization_launch_dir = os.path.join(
        get_package_share_directory("lidar_localization_ros2"), "launch"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=param_file,
                description="Full path to param file to load",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            # [新增] 声明 log_level 参数
            DeclareLaunchArgument(
                "log_level",
                default_value="warn",
                description="Logging level",
            ),
            # 启动Nav2核心堆栈
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        nav2_bringup_launch_dir,
                        "/nav2_bringup.launch.py",
                    ]
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": param_file,
                    "log_level": log_level,
                }.items(),
            ),
            # 启动RViz2 (RViz通常输出较少，这里保持 output="log" 即可，或者也加上参数)
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_file],
                parameters=[{"use_sim_time": use_sim_time}],
                output="log",
            ),
            # 延迟启动定位节点
            TimerAction(
                period=3.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [
                                lidar_localization_launch_dir,
                                "/lidar_localization.launch.py",
                            ]
                        ),
                        # 如果 lidar_localization 支持，也可以传 log_level，不支持则不传
                        launch_arguments={"use_sim_time": use_sim_time}.items(),
                    )
                ],
            ),
        ]
    )
