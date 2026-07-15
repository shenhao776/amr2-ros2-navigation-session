from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取默认参数文件的路径
    pkg_share = get_package_share_directory("mobile_robot")
    default_params_file = os.path.join(pkg_share, "config", "planning_params.yaml")

    return LaunchDescription(
        [
            # 声明一个启动参数，允许在命令行覆盖参数文件路径
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Full path to the ROS2 parameters file to use for the udp nodes.",
            ),
            # [新增] 启动 UDP Server 节点 (负责网络通信)
            Node(
                package="mobile_robot",
                executable="udp_server_node",
                name="udp_server_node",
                output="screen",
                # 从启动配置中加载参数文件 (端口号等)
                parameters=[LaunchConfiguration("params_file")],
            ),
            # [新增] 启动 Robot Agent 节点 (负责机器人控制逻辑)
            Node(
                package="mobile_robot",
                executable="robot_agent_node",
                name="robot_agent_node",
                output="screen",
                # 从启动配置中加载参数文件 (路径、参数等)
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
