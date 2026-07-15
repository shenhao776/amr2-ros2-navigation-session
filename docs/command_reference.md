# Command Reference

This page explains the commands used by the AMR2 workflow. It is a reference rather than a sequence to run from top to bottom. Follow the [remote workflow guide](../ros2_ws/README.md) for the correct order.

## How to read command examples

Do not type terminal prompts such as `$`, `#`, or `root@ros2_ws`. Text inside angle brackets, such as `<package_name>`, is a placeholder and must be replaced. A backslash (`\`) at the end of a line continues the same command on the next line.

Commands run in one of two places:

| Location | Typical prompt | Commands used there |
|---|---|---|
| Ubuntu tablet host | `hao@tablet` | SSH setup, `docker build`, `docker run`, `docker ps`, `docker stop` |
| ROS 2 container | `root@ros2_ws` | `colcon`, `source`, and every `ros2 ...` command |

`~` means the current user's home directory. On the tablet host, `~/shared_files` normally means `/home/hao/shared_files`. In the container, `/root/shared_files` reaches the same shared directory through Docker's volume mount.

## Ubuntu and SSH

### `sudo apt update`

Downloads the current Ubuntu package index. `sudo` runs the command with administrator privileges. This command does not upgrade installed packages.

### `sudo apt install -y openssh-server`

Installs the SSH server. `-y` automatically accepts the installation confirmation.

### `sudo systemctl enable --now ssh`

Starts the SSH service now and enables it after future reboots. `systemctl status ssh` shows its current state.

### `hostname -I`

Prints the tablet's network addresses. Use the address reachable from the Mac.

### `ssh hao@192.168.10.30`

Opens a shell on the tablet. `hao` is the tablet username and the value after `@` is its IP address. `ssh -t ...` additionally allocates an interactive terminal, which is useful before entering Docker. `-X` and `-Y` are intentionally not used because RViz should open on the tablet.

### `exit`

Leaves the current shell. It can leave the container shell or end the SSH connection, depending on the current level.

## Tablet display and RViz

### `echo "$DISPLAY"`

Prints the display selected for graphical applications. The tablet desktop normally uses `:0`.

### `export DISPLAY=:0`

Sets the display for commands started from the current shell. `export` also passes the variable to child processes.

### `xhost +`

Allows X11 clients, including the Docker container, to open windows on the tablet desktop. This is convenient for the isolated session network but permissive; use `xhost -` later if access should be closed again.

## Files, directories, and Git

### `mkdir -p PATH`

Creates a directory. `-p` also creates missing parent directories and does not fail if the directory already exists.

### `cd PATH`

Changes the current directory. `pwd` prints the current directory, and `ls -lh PATH` lists files with human-readable sizes.

### `git clone URL`

Downloads a Git repository into a new directory. For this project:

```bash
git clone https://github.com/shenhao776/amr2-ros2-navigation-session.git
```

### `chmod +x FILE`

Adds executable permission to a script. ROS 2 can only expose `send_nav_goal.py` as a runnable program when this permission is present.

### Basic diagnostic commands

| Command | Meaning |
|---|---|
| `whoami` | Print the current user; `root` normally means the shell is inside the container |
| `pwd` | Print the current working directory |
| `ls -lh PATH` | List files at a path with permissions and human-readable sizes |
| `printenv DISPLAY` | Print the `DISPLAY` environment variable without any surrounding text |

`docker exec ros2_ws printenv DISPLAY` runs `printenv DISPLAY` inside the container without opening an interactive shell.

## Docker

### `docker build -t amr2-ros2 .`

Builds an image from the Dockerfile in the current directory. `-t amr2-ros2` assigns the image name, and `.` supplies the current directory as the build context.

### `docker run ... amr2-ros2 zsh`

Creates and starts a new container from the `amr2-ros2` image, then opens Z shell. The options used by this project mean:

| Option | Meaning |
|---|---|
| `-it` | Keep standard input interactive and allocate a terminal |
| `--rm` | Remove the container automatically after it stops |
| `--name ros2_ws` | Give the container a stable name used by `docker exec` |
| `--hostname ros2_ws` | Set the hostname visible inside the container |
| `--net=host` | Share the tablet's network, simplifying ROS 2 and sensor discovery |
| `--privileged` | Allow access to hardware devices needed by the robot |
| `-e NAME=value` | Set an environment variable inside the container |
| `-v HOST:CONTAINER` | Mount a host path at a container path |
| `--gpus all` | Expose all NVIDIA GPUs to the container; omit on non-NVIDIA systems |

Important environment variables and mounts:

| Setting | Purpose |
|---|---|
| `ROS_DOMAIN_ID=2` | Keeps ROS 2 discovery within domain 2; all communicating processes must match |
| `DISPLAY=:0` | Sends graphical windows to the tablet desktop |
| `/tmp/.X11-unix:/tmp/.X11-unix` | Shares the X11 display socket |
| `~/shared_files:/root/shared_files` | Makes the repository and datasets available inside Docker |
| `/dev:/dev` | Makes tablet hardware devices visible inside Docker |

### `docker ps` and `docker ps -a`

`docker ps` lists running containers. `docker ps -a` also lists stopped containers. A container started with `--rm` disappears from the second list after it exits.

### `docker exec -it ros2_ws zsh`

Starts another interactive Z shell inside the already-running `ros2_ws` container. It does not create a second container.

### `docker stop -t 10 ros2_ws`

Asks the container to stop and waits up to 10 seconds before forcing it. Because this project retains `--rm`, the container is removed after stopping.

## ROS 2 environment and building

### `source FILE`

Loads environment variables and shell functions from a setup file into the current terminal. It affects only that terminal.

```bash
source /opt/ros/humble/setup.zsh
source /root/thirdparty_ws/install/setup.zsh
source /root/shared_files/amr2-ros2-navigation-session/ros2_ws/install/setup.zsh
```

The first line loads ROS 2 Humble, the second loads third-party packages, and the third loads this workspace. After building from the workspace directory, `source install/setup.zsh` is the shorter equivalent of the third line.

### `colcon build --symlink-install`

Builds every ROS 2 package in the workspace. `--symlink-install` links editable resources and Python scripts into `install/`, which makes development changes easier to test.

Useful build options:

| Option | Meaning |
|---|---|
| `--packages-select mobile_robot` | Build only the named package |
| `--cmake-clean-cache` | Remove that package's CMake cache before rebuilding |

## ROS 2 inspection commands

| Command | What it shows |
|---|---|
| `ros2 node list` | Running ROS 2 nodes |
| `ros2 topic list` | Available continuous data channels |
| `ros2 service list` | Available request/response services |
| `ros2 action list` | Available long-running actions such as navigation |
| `ros2 pkg executables mobile_robot` | Executables installed by `mobile_robot` |
| `ros2 topic echo /pcl_pose --once` | One current localization message, then exits |

## Starting project functions

### `ros2 launch PACKAGE FILE`

Starts a launch file and the group of nodes described by it. Keep the terminal open; press `Ctrl+C` once to request a clean shutdown.

```bash
ros2 launch mobile_robot collect_data.launch.py
ros2 launch map_updater mapping_mid360.launch.py
ros2 launch mobile_robot start_full_system.launch.py
```

- `collect_data.launch.py` starts the robot hardware needed for driving and recording.
- `mapping_mid360.launch.py` starts offline mapping and RViz before bag playback.
- `start_full_system.launch.py` starts localization, Nav2, RViz, sensors, and base control.

### `ros2 run PACKAGE EXECUTABLE`

Starts one executable from an installed package.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run mobile_robot send_nav_goal.py
```

The first command drives the robot from the focused keyboard terminal. The second asks for target `x`, `y`, and yaw. A non-interactive goal can be sent with:

```bash
ros2 run mobile_robot send_nav_goal.py --x 2.0 --y -1.5 --yaw-deg 90
```

`--x` and `--y` are metres in the map frame. `--yaw-deg` is the heading in degrees.

## Recording and replaying rosbag data

### `ros2 bag record`

Records selected topics until `Ctrl+C` is pressed:

```bash
ros2 bag record \
  -o amr2_mapping_01 \
  /livox/lidar \
  /livox/imu \
  /camera/camera/color/image_raw/compressed
```

`-o amr2_mapping_01` chooses the output directory name. The following values are topic names. Use a new output name for each recording so existing data is not overwritten.

### `ros2 bag info BAG_PATH`

Shows bag duration, message counts, storage format, and recorded topics without playing the data.

### `ros2 bag play BAG_PATH`

Publishes recorded messages again. `--rate 0.5` replays at half speed, which gives mapping more processing time.

```bash
ros2 bag play --rate 0.5 /root/shared_files/dataset/ros2bags/amr2_mapping_01
```

Do not run live sensor publishers at the same time as playback because identical live and recorded topics would be mixed.

## Saving a map

```bash
ros2 service call /save_map std_srvs/srv/Trigger
```

Calls the `/save_map` service using the empty `std_srvs/srv/Trigger` request type. Wait for `success: true` before stopping the mapper. The navigation map is written to:

```text
/root/shared_files/dataset/my_map_data/PCD/map_raw.pcd
```

## Keyboard control and interruption

`Ctrl+C` sends an interrupt to the foreground command. For launches and bag recording, press it once and wait for cleanup. In keyboard teleoperation, `k` sends a zero-velocity stop command before the program exits.
