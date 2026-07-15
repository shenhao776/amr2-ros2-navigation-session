# Troubleshooting

Put the robot in a safe state first. If anything behaves unexpectedly, press `Ctrl+C` in the terminal that started it. Do not repeatedly start the same launch file.

## Fastest first checks

```bash
whoami
pwd
docker ps
```

- Run `docker ps` on the host to check that the `ros2_ws` container exists.
- Run `ros2 ...` inside the container; its prompt normally starts with `root@ros2_ws`.
- In every new terminal, enter the container with `docker exec -it ros2_ws zsh`.

## SSH from the Mac fails

On the tablet, check the SSH service and IP address:

```bash
sudo systemctl status ssh
hostname -I
```

The Mac and tablet must be on the same reachable network. Use `ssh user@tablet_ip` without `-X` or `-Y`.

## RViz does not appear on the tablet

The tablet must have an active, unlocked Ubuntu desktop session. In a terminal opened directly on the tablet desktop, run:

```bash
echo "$DISPLAY"
export DISPLAY=:0
xhost +
```

Then check the running container from the tablet host:

```bash
docker exec ros2_ws printenv DISPLAY
```

It should match the tablet display, normally `:0`. If it does not, recreate the container with the correct `-e DISPLAY=...` value and the `/tmp/.X11-unix` mount. Do not use SSH X11 forwarding in this setup.

## `ros2: command not found`

You are probably still on the host. Run:

```bash
docker exec -it ros2_ws zsh
```

If you are already in the container, load the environments:

```bash
source /opt/ros/humble/setup.zsh
source /root/thirdparty_ws/install/setup.zsh
source /root/shared_files/amr2-ros2-navigation-session/ros2_ws/install/setup.zsh
```

## `package 'mobile_robot' not found`

Build the workspace and load its result:

```bash
cd /root/shared_files/amr2-ros2-navigation-session/ros2_ws
colcon build --symlink-install
source install/setup.zsh
```

## `docker exec` says the container is missing or stopped

Check on the host:

```bash
docker ps -a
```

The project container uses `--rm`, so it is removed when it stops. Start a fresh container with the guide's `docker run` command. You do not need to rebuild the image unless the Dockerfile changed.

## `start_full_system.launch.py` does not stop immediately

Press `Ctrl+C` once, then wait. Most nodes exit immediately, but the Nav2 component container may need up to about seven seconds while Launch escalates from `SIGINT` to `SIGTERM` and finally `SIGKILL`. Messages about this escalation are expected and prevent an indefinite hang.

If the SSH launch terminal is lost or the process still remains, open a new SSH terminal on the tablet host and stop the whole project container:

```bash
docker stop -t 10 ros2_ws
```

Because the container uses `--rm`, Docker removes it after stopping. Run the documented `docker run` command to start a fresh project container.

## LiDAR or camera data is missing

Check power and cables, then run inside the container:

```bash
ros2 node list
ros2 topic list
```

Expected topics include `/livox/lidar`, `/livox/imu`, and camera topics. A missing topic usually means the driver did not start, the device is disconnected, or its network configuration is wrong.

## The robot does not respond to the controller

1. Confirm `collect_data.launch.py` is still running and no node exited.
2. Check the base control cable or serial connection.
3. Read the first red error in the launch terminal.
4. Press `Ctrl+C`, wait for a complete shutdown, and start it only once.

For keyboard control, keep the teleoperation terminal focused and press `k` to send a stop command. If `teleop_twist_keyboard` is not found, rebuild the current Docker image from the project Dockerfile and recreate the container.

## A recorded bag cannot build a map

Inspect it first:

```bash
ros2 bag info /root/shared_files/dataset/ros2bags/amr2_mapping_01
```

It must contain `/livox/lidar` and `/livox/imu`. Stop the live sensor launch before playback, start `mapping_mid360.launch.py` first, and then play the bag. If processing falls behind, replay with `--rate 0.5`.

## `/save_map` fails

Check that the mapping node and service exist:

```bash
ros2 node list
ros2 service list
```

The list should contain `/save_map`. If not, check whether `mapping_mid360.launch.py` is still running.

If the response is successful but the file is missing, run:

```bash
ls -lh /root/shared_files/dataset/my_map_data/PCD/
```

## Navigation says the map is missing

Navigation reads this fixed path:

```text
/root/shared_files/dataset/my_map_data/PCD/map_raw.pcd
```

Confirm that the save service succeeded and the path matches exactly. If saving failed, rebuild and save the map; do not substitute an empty file.

## `send_nav_goal.py` is not found

Pull the latest project, rebuild, and reload the workspace:

```bash
cd /root/shared_files/amr2-ros2-navigation-session/ros2_ws
chmod +x src/mobile_robot/scripts/send_nav_goal.py
colcon build --symlink-install --packages-select mobile_robot --cmake-clean-cache
source install/setup.zsh
ros2 pkg executables mobile_robot
```

The executable list should contain `mobile_robot send_nav_goal.py`. The `chmod` is important with `--symlink-install`, because ROS 2 ignores scripts that do not have executable permission.

If the script waits for Nav2, keep `start_full_system.launch.py` running and check:

```bash
ros2 action list
```

The list should contain `/navigate_to_pose`.

## The map is doubled or localization is poor

- Drive more slowly and avoid sharp turns while mapping.
- Complete a loop and return to the original position and heading.
- Check that the sensor mounts are tight; sensor positions must not change after calibration.
- Remove large moving objects and build the map again.

## When asking for help

Provide the output of:

```bash
ros2 node list
ros2 topic list
ls -lh /root/shared_files/dataset/my_map_data/PCD/
```

Also include the complete terminal output beginning at the first `ERROR`, and say which step of the hands-on guide failed.
