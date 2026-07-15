# AMR2 ROS 2 Navigation Learning Session

This repository accompanies **Getting AMR2 Moving**, a technical sharing session for people who are new to Linux, Docker, and ROS 2. An Ubuntu tablet runs on the robot and is operated remotely from macOS Terminal over SSH, while RViz stays visible on the tablet.

The session follows one practical path:

```text
SSH to the robot → Record while driving → Replay the bag to build a map → Navigate
```

## What this project covers

- What the LiDAR, camera, mobile base, and microcontroller each do
- The meaning of a ROS 2 node, topic, service, and launch file
- How to tell a host terminal from a Docker container terminal
- How to start AMR2, build a map, and send a navigation goal in RViz
- How to record a rosbag and rebuild a map by replaying it
- How to send a typed x/y/yaw goal to Nav2

## Start here

1. On a new computer, follow the [complete hands-on guide](ros2_ws/README.md#part-1-one-time-setup).
2. Read [The Minimum ROS 2 Concepts](docs/ros2_basics.md) for the essential background.
3. Use the [command reference](docs/command_reference.md) whenever a command or option is unfamiliar.
4. Follow the [recording, mapping, and navigation workflow](ros2_ws/README.md#part-2-record-a-mapping-rosbag).
5. If something fails, use the [troubleshooting guide](docs/troubleshooting.md).

## Project layout

```text
amr2-ros2-navigation-session/
├── README.md                  # Project entry point
├── docs/                      # Concepts, command reference, and troubleshooting
└── ros2_ws/                   # ROS 2 workspace
    ├── README.md              # Complete setup and hands-on workflow
    ├── docker/                # Reproducible ROS 2 Humble environment
    └── src/
        ├── mobile_robot/      # Sensors, base control, and Nav2 launch files
        ├── map_updater_ros2/  # Mapping and map saving
        └── 3rdparty/          # Dependencies; beginners do not need to edit them
```

## Main workflow commands

| Command | Purpose |
|---|---|
| `ros2 launch mobile_robot collect_data.launch.py` | Start the LiDAR, camera, base control, and remote-control/recording nodes |
| `ros2 launch map_updater mapping_mid360.launch.py` | Start the mapper before rosbag playback and open RViz |
| `ros2 launch mobile_robot start_full_system.launch.py` | Start sensing, localization, Nav2, RViz, and base control |
| `ros2 run mobile_robot send_nav_goal.py` | Ask for x, y, and yaw, then send the goal to Nav2 |

Everything else in `src/` implements or supports these three entry points. You do not need to run the packages one by one.

## Safety

- For the first startup, lift the wheels off the ground or provide a clear test area.
- Keep a terminal ready and press `Ctrl+C` to stop a launch immediately.
- Drive slowly while mapping and stay out of the robot's path.
- The sensor calibration is already configured for AMR2. Do not change it during a normal session.
