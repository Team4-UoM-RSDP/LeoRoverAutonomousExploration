# Leo Rover Autonomous Exploration

**ROS 2 Jazzy · Ubuntu 24.04 · Gazebo Harmonic · MIT License**

A frontier-based autonomous exploration system for the [Leo Rover](https://www.leorover.tech/) mobile robot. This project uses **Wavefront Frontier Detection (WFD)** to autonomously explore and map unknown environments. It supports both **Gazebo simulation** and **real-robot deployment** with a shared exploration node.

## Features

- **Wavefront Frontier Detection (WFD)** for robust frontier discovery on occupancy grids
- **No in-place rotation policy** to improve lidar stability on the real robot
- **Front-only lidar filtering** (real: 120°, sim: 180°) to ignore false rear-body reflections
- **Dual-layer safety system** with front obstacle checks and full 360° safety perimeter monitoring
- **Sim-to-real consistency** using the same `frontier_explorer` node in both modes
- **Self-contained simulation** with included URDF, Gazebo world, and bridge configuration
- **Automatic map saving** when exploration is complete
- **Anti-revisit scoring** to reduce repeated visits to the same regions

## Requirements

| Component | Version                 |
| --------- | ----------------------- |
| Ubuntu    | 24.04 LTS               |
| ROS 2     | Jazzy                   |
| Gazebo    | Harmonic (`gz-sim 8.x`) |
| Python    | 3.12+                   |

### Required ROS 2 Packages

* `nav2_bringup`
* `slam_toolbox`
* `robot_localization`
* `rplidar_ros`
* `ros_gz_sim`
* `ros_gz_bridge`
* `robot_state_publisher`
* `joint_state_publisher`
* `tf2_ros`

The repository includes an installation script:

```bash
cd leo_exploration_ws/src/leo_exploration/scripts
chmod +x install_sim_deps.sh
./install_sim_deps.sh
```

## Quick Start


```bash

### 1. Clone the Repository
git clone https://github.com/Team4-UoM-RSDP/LeoRoverAutonomousExploration.git
cd LeoRoverAutonomousExploration


### 2. Install Dependencies

cd leo_exploration_ws/src/leo_exploration/scripts
chmod +x install_sim_deps.sh
./install_sim_deps.sh

### 3. Build the Workspace

cd ../../..
colcon build --packages-select leo_exploration
source install/setup.bash

## Run in Simulation

ros2 launch leo_exploration sim_exploration_launch.py
```


### Useful Launch Options

```bash
# Run headless (without Gazebo GUI)
ros2 launch leo_exploration sim_exploration_launch.py gz_gui:=false

# Disable RViz
ros2 launch leo_exploration sim_exploration_launch.py rviz:=false

# Use a custom world
ros2 launch leo_exploration sim_exploration_launch.py world:=/path/to/my.world

# Change spawn position
ros2 launch leo_exploration sim_exploration_launch.py spawn_x:=2.0 spawn_y:=1.0
```

## Run on the Real Robot

This repository uses a split deployment for the physical robot:

* The Leo Rover Pi keeps running the LeoOS base stack for drivetrain, IMU, and `odom -> base_footprint -> base_link`.
* The Pi also runs a lightweight `/cmd_vel_relay -> /cmd_vel` bridge so commands from the development machine reliably reach the firmware.
* The development machine runs the USB RPLidar, SLAM Toolbox, Nav2, `frontier_explorer`, and RViz.

### 1. Confirm the Pi base stack is online

On the Pi, the LeoOS user services should be active and must share the same ROS 2 network settings as the development machine. The supported Pi-side setup is:

```bash
systemctl --user status ros-nodes.service uros-agent.service
```

On the development machine, verify that you can already see the rover base stack before starting this package:

```bash
ros2 topic list | grep -E '/merged_odom|/tf|/cmd_vel_relay'
ros2 topic hz /merged_odom
```

If those commands do not show the Pi-side base nodes and TF topics, fix the Pi ROS service environment first. The Pi should not run its own RPLidar node in this split mode.

### 2. Start the real-robot exploration stack (three terminals)

Build the workspace once, then source it in each terminal:

```bash
cd leo_exploration_ws
colcon build --packages-select leo_exploration
source install/setup.bash
```

Make sure the RPLidar is connected to the development machine and accessible:

```bash
sudo chmod 666 /dev/ttyUSB0
```

**Terminal 1 — Lidar + laser TF**

```bash
cd leo_exploration_ws
source install/setup.bash
ros2 launch leo_exploration step1_lidar.launch.py serial_port:=/dev/ttyUSB0 laser_height:=0.12
```

**Terminal 2 — SLAM Toolbox**

```bash
cd leo_exploration_ws
source install/setup.bash
ros2 launch leo_exploration step2_slam.launch.py
```

**Terminal 3 — Nav2 + frontier explorer + RViz**

```bash
cd leo_exploration_ws
source install/setup.bash
ros2 launch leo_exploration step3_nav_explorer.launch.py
```

## System Overview

The exploration stack combines:

* **SLAM Toolbox** for online mapping
* **Nav2** for path planning and navigation
* **Robot Localization (EKF)** for odometry and IMU fusion
* **Frontier Explorer** for frontier detection, scoring, and state-based control

### Main Data Flow

```text
/scan ──> SLAM Toolbox ──> /map
                  │
                  └──> frontier_explorer ──> /navigate_to_pose
                                               │
                                               └──> Nav2 ──> /cmd_vel_nav
                                                            └──> velocity_smoother
                                                                 └──> collision_monitor
                                                                      └──> /cmd_vel_relay -> Pi /cmd_vel
```

## Exploration Logic

The `frontier_explorer` node runs a state machine with six states:

| State             | Description                                             |
| ----------------- | ------------------------------------------------------- |
| `INIT_FORWARD`    | Drives forward briefly to seed the map without spinning |
| `SELECT_FRONTIER` | Detects, filters, scores, and selects frontiers         |
| `NAVIGATING`      | Sends the selected goal to Nav2 and monitors progress   |
| `AVOIDING`        | Backs up and performs a gentle curve to avoid obstacles |
| `RECOVERING`      | Moves forward slowly to expose new frontiers            |
| `COMPLETE`        | Stops exploration and saves the final map               |

## Frontier Scoring

Each frontier is scored using a weighted heuristic:

```text
score = 0.45 * info_gain + 0.35 * distance_score + 0.15 * direction_score - visit_penalty
```

### Scoring Factors

* **Information gain**: larger frontier clusters are preferred
* **Distance score**: favors frontiers in a practical navigation range
* **Direction score**: prefers frontiers ahead of the robot
* **Visit penalty**: discourages repeatedly selecting recently visited areas

## Safety Design

Two safety mechanisms are used (real-robot defaults shown; simulation uses larger thresholds):

| Layer                     | Coverage    | Real Robot Threshold | Simulation Threshold | Purpose                                  |
| ------------------------- | ----------- | -------------------- | -------------------- | ---------------------------------------- |
| Navigation obstacle check | Front 120°  | `0.45 m`             | `0.55 m`             | Obstacle detection for path execution    |
| Safety perimeter          | Full 360°   | `0.35 m`             | `0.50 m`             | Emergency avoidance in all active states |

## Runtime Controls

### Pause Exploration

```bash
ros2 topic pub /explore/enable std_msgs/msg/Bool '{data: false}' --once
```

### Resume Exploration

```bash
ros2 topic pub /explore/enable std_msgs/msg/Bool '{data: true}' --once
```

### Monitor Exploration Progress

```bash
ros2 topic echo /rosout | grep -E "Progress|Explored|COMPLETE"
```

### Clear Costmaps

```bash
ros2 service call /global_costmap/clear_entirely_global_costmap \
  nav2_msgs/srv/ClearEntireCostmap {}
```

### Check TF Tree

```bash
ros2 run tf2_tools view_frames
```

### Check Lidar Frequency

```bash
ros2 topic hz /scan
```

## Dynamic Obstacles in Simulation

Use the obstacle management script:

```bash
cd leo_exploration_ws/src/leo_exploration/scripts
chmod +x obstacle_manager.sh
```

Examples:

```bash
# Add a box obstacle at (3.0, 2.0)
./obstacle_manager.sh add 3.0 2.0

# Add a 3 m wall
./obstacle_manager.sh wall 0.0 3.0 3.0 0.0

# List all dynamic models
./obstacle_manager.sh list

# Remove all dynamic obstacles
./obstacle_manager.sh clear
```

## Default Simulation World

The default world includes:

* A **12 × 12 m** enclosed indoor environment
* Multiple static and movable obstacles with wider spacing
* A Leo Rover spawn point near the center
* Lighting and physics settings suitable for exploration testing

## Troubleshooting

| Problem                      | Possible Cause                                  | Suggested Fix                                     |
| ---------------------------- | ----------------------------------------------- | ------------------------------------------------- |
| Gazebo does not start        | Wrong Gazebo version                            | Run `gz sim --version` and confirm Harmonic / 8.x |
| Robot not visible            | Spawn failure                                   | Check terminal output from `spawn_leo`            |
| Exploration does not begin   | Nav2 or SLAM not ready                          | Wait for staged startup to complete               |
| TF warnings                  | Missing transform                               | Inspect TF using `ros2 run tf2_tools view_frames` |
| Lidar detects rear obstacles | Real robot body reflections                     | Keep `scan_half_angle` at `60.0` (real) / `90.0` (sim) |
| Robot revisits the same area | Low revisit penalty or limited frontier quality | Review scoring and map conditions                 |

## License

This project is licensed under the **MIT License**.

The Wavefront Frontier Detection implementation in `nav2_wavefront_frontier_exploration-main/` is based on prior work by Sean Regan and related frontier exploration research.
