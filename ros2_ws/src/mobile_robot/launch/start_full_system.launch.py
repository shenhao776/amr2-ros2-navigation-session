import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mobile_robot")
    default_params_file = os.path.join(
        pkg_share, "config", "nav2_params_3d_scan_outdoor.yaml"
    )

    # --- Launch Configurations ---
    nav2_params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_udp_server = LaunchConfiguration("launch_udp_server")
    log_level = LaunchConfiguration("log_level")

    # --- Include other launch files ---
    robot_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "robot_bringup_launch.py")
        )
    )

    navigation2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "navigation2.launch.py")
        ),
        launch_arguments={
            "params_file": nav2_params_file,
            "use_sim_time": use_sim_time,
            "log_level": log_level,
        }.items(),
    )

    # 定义新的 UDP Server 节点
    udp_server_node = Node(
        package="mobile_robot",
        executable="udp_server_node",
        name="udp_server_node",
        output="screen",
        sigterm_timeout="5.0",
        sigkill_timeout="2.0",
        parameters=[os.path.join(pkg_share, "config", "planning_params.yaml")],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # 定义新的 Robot Agent 节点
    robot_agent_node = Node(
        package="mobile_robot",
        executable="robot_agent_node",
        name="robot_agent_node",
        output="screen",
        sigterm_timeout="5.0",
        sigkill_timeout="2.0",
        parameters=[os.path.join(pkg_share, "config", "planning_params.yaml")],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # --- 条件组 ---
    udp_group = GroupAction(
        actions=[
            udp_server_node,
            robot_agent_node,
        ],
        condition=IfCondition(launch_udp_server),
    )

    # [新增] 创建一个延时动作，将 Nav2 和 业务节点的启动推迟 5 秒
    # 这样可以让 robot_bringup 先建立好 TF 树，避免 RViz 报 queue full 错误
    delayed_launch_group = TimerAction(
        period=5.0,
        actions=[
            navigation2_launch,
            udp_group,
        ],
    )

    return LaunchDescription(
        [
            # --- 启动参数声明 ---
            DeclareLaunchArgument("params_file", default_value=default_params_file),
            DeclareLaunchArgument("use_sim_time", default_value="False"),
            DeclareLaunchArgument(
                "launch_udp_server",
                default_value="True",
                description="Set to 'False' to disable the udp receiver nodes.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="warn",
                description="Logging level (info, warn, error, fatal)",
            ),
            # 1. 首先启动硬件驱动和底层 TF
            robot_bringup_launch,
            # 2. 延时启动上层应用和导航
            delayed_launch_group,
        ]
    )
