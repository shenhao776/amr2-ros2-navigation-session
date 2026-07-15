# The Minimum ROS 2 Concepts

This page covers the minimum concepts needed for the AMR2 hands-on workflow. The commands can be looked up as needed.

## 1. Four parts of the robot

| Part | What it does | A useful analogy |
|---|---|---|
| Livox Mid360 LiDAR | Measures 3D distances around the robot | Distance sense |
| RealSense D455 | Produces color images | Eyes |
| Wheeled base and battery | Carries the hardware and moves | Body and legs |
| Microcontroller | Receives wheel speeds and drives the motors | A simple muscle controller |

## 2. Frames and calibration

The LiDAR, camera, and robot body each observe the world from their own coordinate frame. A transform says where one frame is relative to another. Chaining transforms lets a wall measured by the LiDAR appear in the correct place on the room map.

Calibration measures the fixed position and orientation between sensors and stores those values in configuration files. Poor calibration causes duplicated walls, bent maps, and inaccurate navigation. This project already contains AMR2's calibration; the standard workflow does not recalibrate it.

## 3. Four ROS 2 words

### Node: one small program with one job

The LiDAR driver reads the sensor, the mapping node joins scans, and Nav2 plans paths. A robot runs many nodes at the same time.

### Topic: a channel for continuous data

A LiDAR node continuously publishes scans to a topic. Mapping and obstacle-avoidance nodes can subscribe to the same data.

```bash
ros2 topic list
```

### Launch file: one command that starts a team of nodes

A launch file starts and configures a group of cooperating nodes. This project uses a few launch commands instead of starting dozens of nodes separately.

```bash
ros2 launch <package_name> <launch_file>
```

### Service: one request followed by one response

Saving a map is a one-time action, so it uses a service:

```bash
ros2 service call /save_map std_srvs/srv/Trigger
```

The mapping node receives the request, writes the map, and returns success or failure.

## 4. From an RViz goal to the wheels

```text
Goal clicked in RViz
        ↓
Nav2 plans a path on the map
        ↓
The controller calculates /cmd_vel
        ↓
twist_controller converts it to left/right wheel commands
        ↓
The microcontroller drives the motors
```

The microcontroller knows nothing about maps or goals. It only understands wheel-speed commands. A watchdog stops the wheels if commands stop arriving.

## 5. Why Docker is used

The Docker container holds ROS 2, sensor drivers, and dependencies in one repeatable environment. The host opens that environment; every `ros2` command runs inside it.

```text
Run docker commands on the host; run ros2 commands in the container.
```

Continue with the [AMR2 hands-on guide](../ros2_ws/README.md), or open the [command reference](command_reference.md) for command details.
