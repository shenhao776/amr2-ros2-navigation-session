# AMR2 Remote Workflow Guide

In this setup, an Ubuntu tablet rides on the robot and runs Docker, ROS 2, and RViz. A macOS Terminal connects over SSH. Commands are typed on the Mac, while RViz opens on the tablet screen.

```text
macOS Terminal ──SSH──▶ Ubuntu tablet ──Docker──▶ ROS 2 and robot hardware
                                             └──▶ RViz on the tablet screen
```

> Do not use `ssh -X` or `ssh -Y`. X11 forwarding would try to send the window to the Mac. This setup intentionally uses `DISPLAY=:0` so RViz appears on the tablet.

For an explanation of every command and option used below, see the [command reference](../docs/command_reference.md).

## Part 1: One-time setup

### 1. Prepare SSH on the tablet

Connect the tablet and Mac to the same network. On the tablet itself, open a terminal and run:

```bash
sudo apt update
sudo apt install -y openssh-server
sudo systemctl enable --now ssh
hostname -I
```

Write down the tablet username and IP address. The examples below use:

```text
Username: hao
IP:       192.168.10.30
```

Test from macOS Terminal:

```bash
ssh hao@192.168.10.30
```

Type `yes` the first time, then enter the tablet password. Run `exit` to leave SSH.

### 2. Allow container windows on the tablet

The tablet must be logged in to its Ubuntu desktop, with the screen unlocked. In a terminal opened directly on that desktop, run once after login:

```bash
echo "$DISPLAY"
export DISPLAY=:0
xhost +
```

The display normally prints `:0`. If it prints another value, use that value instead of `:0` in the Docker command below.

### 3. Clone and build the Docker image

These commands may be entered either directly on the tablet or through Mac SSH:

```bash
mkdir -p ~/shared_files
cd ~/shared_files
git clone https://github.com/shenhao776/amr2-ros2-navigation-session.git
cd amr2-ros2-navigation-session/ros2_ws/docker
docker build -t amr2-ros2 .
```

### 4. Start the container

Run on the tablet host, directly or through SSH:

```bash
docker run -it --rm \
  --name ros2_ws \
  --hostname ros2_ws \
  --net=host \
  --privileged \
  -e ROS_DOMAIN_ID=2 \
  -e DISPLAY=:0 \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/shared_files:/root/shared_files \
  -v /dev:/dev \
  --gpus all \
  amr2-ros2 zsh
```

On a tablet without an NVIDIA GPU, remove the `--gpus all` and `-e NVIDIA_DRIVER_CAPABILITIES=all` lines.

The container runs in the foreground and uses `--rm`. Keep this SSH terminal open during the session. When the container stops, Docker removes it automatically; start a new one by running the same `docker run` command again.

To check whether it is currently running, open another SSH terminal and use:

```bash
docker ps
```

### 5. Build the ROS 2 workspace

From Mac Terminal, enter the tablet and then the container:

```bash
ssh hao@192.168.10.30
docker exec -it ros2_ws zsh
```

Inside the container, run:

```bash
cd /root/shared_files/amr2-ros2-navigation-session/ros2_ws
colcon build --symlink-install
source install/setup.zsh
```

A successful build ends with `Summary: ... packages finished` and no `Failed` entry.

## Opening a ROS 2 terminal from the Mac

Every task below needs its own macOS Terminal tab. In each new tab, first SSH to the tablet and then enter the running container:

```bash
ssh -t hao@192.168.10.30
docker exec -it ros2_ws zsh
```

The prompt changes to `root@ros2_ws`. Commands now run on the tablet inside Docker. A launch that starts RViz will display RViz on the tablet, not on the Mac.

## Part 2: Record a mapping rosbag

Keep the tablet screen unlocked. Clear the area around the robot and keep a terminal ready for `Ctrl+C`.

### Step 1: Start sensors and base control

Open Mac tab 1, enter the ROS 2 container, and run:

```bash
ros2 launch mobile_robot collect_data.launch.py
```

Leave it running. Success means the Mid360 and D455 topics appear and no node exits immediately.

### Step 2: Record a rosbag

Open Mac tab 2 and run:

```bash
mkdir -p /root/shared_files/dataset/ros2bags
cd /root/shared_files/dataset/ros2bags
ros2 bag record \
  -o amr2_mapping_01 \
  /livox/lidar \
  /livox/imu \
  /camera/camera/color/image_raw/compressed
```

Leave this terminal running while driving. The data is stored on the tablet host at:

```text
~/shared_files/dataset/ros2bags/amr2_mapping_01
```

Use a new output name for every recording. To stop and finalize the bag safely, press `Ctrl+C` once and wait until the command returns to the prompt.

### Step 3: Drive with the Mac keyboard

Open Mac tab 3 and run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keep this tab focused while driving:

| Key | Motion |
|---|---|
| `i` | Forward |
| `,` | Backward |
| `j` | Turn left |
| `l` | Turn right |
| `k` | Stop |
| `q` / `z` | Increase / decrease all speeds |

Drive slowly, avoid sharp turns, complete a loop, and return to the starting position and heading. Press `k` before changing terminal tabs. Press `Ctrl+C` when driving is finished.

### Step 4: Finish the recording

Press `k` in the teleoperation terminal, then press `Ctrl+C` there. In the bag-recording terminal, press `Ctrl+C` once and wait until recording is finalized. Finally, stop the sensor launch with `Ctrl+C`.

Live mapping is also available with `ros2 launch map_updater mapping_mid360.launch.py` while driving. It is useful for a quick preview. The recommended workflow is to record first and build the map from bag playback because replay is more stable and can be repeated without driving the robot again.

## Part 3: Build a map from a recorded rosbag

This workflow provides repeatable, offline mapping without moving the robot again.

### 1. Stop live data first

Confirm that `collect_data.launch.py`, teleoperation, and any previous bag playback are stopped. Live sensor messages must not be mixed with recorded messages.

### 2. Start the mapper

In Mac tab 1, enter the ROS 2 container and run:

```bash
ros2 launch map_updater mapping_mid360.launch.py
```

RViz opens on the tablet and waits for messages.

### 3. Replay the bag

In Mac tab 2, enter the container and run:

```bash
ros2 bag play /root/shared_files/dataset/ros2bags/amr2_mapping_01
```

Let playback reach the end. If the tablet cannot process the bag in real time, replay more slowly:

```bash
ros2 bag play --rate 0.5 /root/shared_files/dataset/ros2bags/amr2_mapping_01
```

Do not start the hardware sensor launch while replaying.

### 4. Save the replayed map

After playback finishes, use Mac tab 3:

```bash
ros2 service call /save_map std_srvs/srv/Trigger
```

Wait for `success: true` or `success=True` before stopping the mapper with `Ctrl+C`. The navigation map is written to:

```text
Tablet host: ~/shared_files/dataset/my_map_data/PCD/map_final_visualization.pcd
Container:   /root/shared_files/dataset/my_map_data/PCD/map_final_visualization.pcd
```

## Part 4: Navigate from a command

### 1. Start navigation

Make sure the saved map exists, then run in Mac tab 1:

```bash
ros2 launch mobile_robot start_full_system.launch.py
```

RViz opens on the tablet. Wait until localization and the laser scan are stable.

To see the robot's current map coordinates in another terminal:

```bash
ros2 topic echo /pcl_pose --once
```

### 2. Enter x, y, and yaw interactively

In Mac tab 2, run:

```bash
ros2 run mobile_robot send_nav_goal.py
```

The script clearly asks for:

```text
Target x [m] (example: 2.0):
Target y [m] (example: -1.5):
Target yaw [degrees] (example: 90):
```

`x` and `y` are metres in the `map` frame. Yaw is in degrees: `0` faces +X, `90` faces +Y, and `-90` faces -Y. Review the summary and type `y` to send the goal. Press `Ctrl+C` in the script terminal to cancel it.

For a reusable non-interactive command:

```bash
ros2 run mobile_robot send_nav_goal.py --x 2.0 --y -1.5 --yaw-deg 90
```

The script prints the remaining distance and reports whether Nav2 succeeded, cancelled, or aborted the goal.

## Completion checklist

- [ ] Mac SSH reaches the tablet without X11 forwarding.
- [ ] RViz launched from the Mac appears on the tablet screen.
- [ ] The robot responds to keyboard teleoperation and stops with `k`.
- [ ] The rosbag finishes cleanly and `ros2 bag info` can read it.
- [ ] A new map can be generated from bag playback alone.
- [ ] The x/y/yaw script sends a goal and reports the result.

If a step fails, stop there and use the [troubleshooting guide](../docs/troubleshooting.md).
