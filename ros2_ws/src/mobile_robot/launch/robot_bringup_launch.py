import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Include the livox_ros_driver2 launch file
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("livox_ros_driver2"),
                "launch_ROS2",
                "msg_MID360_launch.py",
            )
        )
    )

    # Start the Realsense D455 camera node with pointcloud enabled and configured settings
    realsense_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        launch_arguments={
            "device_type": "D455",
            "pointcloud.enable": "false",
            "enable_sync": "true",
            "rgb_camera.profile": "1280x720x30",
        }.items(),
    )

    twist_controller_node = Node(
        package="mobile_robot",
        executable="twist_controller_node",
        name="twist_controller_node",
        output="screen",
        # 强行把 Nav2 发出的 cmd_vel_nav 接到底盘的 cmd_vel 接口上
        # 这样就绕过了可能没配置好的 velocity_smoother，直接控制
        # remappings=[("/cmd_vel", "/cmd_vel_nav")],
    )

    # base_link -> livox_frame
    livox_frame_to_base_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="livox_frame_to_base_link",
        arguments=["0", "0", "0.37", "0", "0", "0", "base_link", "livox_frame"],
    )

    # base_link -> camera_link (Camera)
    base_link_to_camera_link_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link2camera_link",
        arguments=[
            "0.038263",  # x
            "-0.019912",  # y
            "0.295476",  # z
            "0.49736165",  # qx
            "-0.50085402",  # qy
            "0.50112831",  # qz
            "-0.50064665",  # qw
            "base_link",
            "camera_link",
        ],
    )

    pointcloud_to_scan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[("cloud_in", "/livox/point"), ("scan", "/scan")],
        parameters=[
            {
                "target_frame": "livox_frame",
                "transform_tolerance": 0.2,
                "min_height": 0.15,
                "max_height": 2.0,
                "angle_min": -3.14,  # -M_PI
                "angle_max": 3.14,  # M_PI
                "angle_increment": 0.0087,  # M_PI/360.0
                "scan_time": 0.1,
                "range_min": 0.1,
                "range_max": 10.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
        name="pointcloud_to_laserscan",
    )

    livox_to_pointcloud2_node = Node(
        package="livox_to_pointcloud2",
        executable="livox_to_pointcloud2_node",
        name="livox_to_pointcloud2",
    )

    return LaunchDescription(
        [
            livox_launch,
            realsense_camera_node,
            livox_to_pointcloud2_node,
            pointcloud_to_scan_node,
            twist_controller_node,
            livox_frame_to_base_link_tf,
            base_link_to_camera_link_tf,
        ]
    )
