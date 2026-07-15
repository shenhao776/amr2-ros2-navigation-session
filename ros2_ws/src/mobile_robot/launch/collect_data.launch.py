import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 获取 mobile_robot 包的路径
    pkg_indoor_robot = get_package_share_directory("mobile_robot")

    # 1. 启动机器人底层硬件 (Sensor Bringup)
    # 包含: Livox雷达, Realsense相机, 里程计驱动, 静态TF变换, 点云格式转换等
    robot_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_indoor_robot, "launch", "robot_bringup_launch.py")
        )
    )

    # 2. 启动 UDP 接收器 (Remote Control & Recording)
    # 包含: udp_server_node
    # 功能:
    #   - 接收 Web 端发来的 "cmd_vel" 指令控制机器人移动
    #   - 接收 "record_start"/"record_stop" 指令控制 ros2 bag 录制
    udp_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_indoor_robot, "launch", "udp_server.launch.py")
        )
    )

    return LaunchDescription([robot_bringup_launch, udp_server_launch])
