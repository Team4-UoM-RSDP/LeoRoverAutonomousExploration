# Leo Rover Autonomous Exploration

**ROS 2 Jazzy · Ubuntu 24.04 · Gazebo Harmonic · MIT License**

A complete frontier-based autonomous exploration system for the [Leo Rover](https://www.leorover.tech/) mobile robot. The system uses a **Wavefront Frontier Detection (WFD)** algorithm to autonomously map unknown environments. It works identically in **Gazebo simulation** and on the **real robot** (with RPLidar A2M12), employing a strict **no-rotation policy** to maintain lidar stability.

---

## Table of Contents

- [Features](#features)
- [Architecture](#architecture)
- [Repository Structure](#repository-structure)
- [Prerequisites](#prerequisites)
- [Quick Start (Simulation)](#quick-start-simulation)
- [Real Robot Deployment](#real-robot-deployment)
- [Launch Options](#launch-options)
- [Startup Timeline](#startup-timeline)
- [Exploration Algorithm](#exploration-algorithm)
- [Configuration Reference](#configuration-reference)
- [Runtime Controls](#runtime-controls)
- [Adding Obstacles (Simulation)](#adding-obstacles-simulation)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## Features

- **Wavefront Frontier Detection (WFD)** — BFS-based frontier detection ported from [nav2_wavefront_frontier_exploration](https://github.com/SeanReg/nav2_wavefront_frontier_exploration), providing highly accurate frontier centroid detection.
- **No-Rotation Policy** — The robot never spins in place, preventing lidar mount instability on real hardware. Startup seeds the map by driving forward; avoidance uses back-up and gentle curves; recovery drives forward slowly.
- **180° Front-Only Lidar Filter** — Ignores scans beyond ±90° to prevent false obstacles from the real Leo Rover chassis pillars behind the lidar.
- **Anti Ghost-Wall** — Multi-factor frontier scoring penalises recently visited spots, preventing the robot from endlessly revisiting the same area.
- **Dual-Layer Safety** — A 360° safety perimeter check runs every control-loop iteration on top of the 180° navigational obstacle check, triggering immediate avoidance when any obstacle breaches the safety radius.
- **Sim-to-Real Parity** — A single `frontier_explorer` node works unchanged in both simulation and on the physical robot.
- **Self-Contained Simulation** — No external `leo_gz` packages required; URDF, Gazebo world, and all plugins are included.
- **Automatic Map Saving** — The map is saved to disk when exploration completes.

---

## Architecture

```
                    ┌───────────────────────────────────┐
                    │     frontier_explorer node         │
                    │  (shared between sim & real robot) │
                    └──────┬──────────────┬──────────────┘
                           │ /navigate_to_pose    │ /cmd_vel
                    ┌──────▼──────┐      ┌────────▼────────┐
                    │  Nav2 Stack │      │ Controller /     │
                    │  (planner,  │      │ Gazebo DiffDrive │
                    │  controller,│      │ or Leo firmware   │
                    │  bt_nav)    │      └─────────────────┘
                    └──────┬──────┘
                           │ TF: map → odom
                    ┌──────▼──────┐
                    │ SLAM Toolbox│◄──── /scan
                    └─────────────┘
```

**TF tree (identical in both modes):**

```
map ──(SLAM)──▶ odom ──(EKF / DiffDrive)──▶ base_footprint ──(URDF)──▶ base_link ──▶ laser
```

| Component | Simulation | Real Robot |
|---|---|---|
| Lidar | Gazebo GPU lidar plugin → ros_gz_bridge → `/scan` | RPLidar A2M12 → `/scan` |
| Odometry | Gazebo DiffDrive → ros_gz_bridge → `/odom` + TF | Leo firmware → `/odom` + TF |
| IMU | Gazebo IMU plugin → ros_gz_bridge → `/imu` | Leo firmware → `/imu` |
| TF (static) | `robot_state_publisher` from URDF | Static TF launch node |
| Localisation | EKF (`robot_localization`): fuses `/odom` velocity + `/imu` heading | Same |
| Mapping | SLAM Toolbox (online async) | Same |
| Navigation | Nav2 (NavFn planner + Regulated Pure Pursuit controller) | Same |
| Exploration | `frontier_explorer` (WFD) | Same |

**Communication flow (simulation):**

```
Gazebo Harmonic
 ├─ DiffDrive plugin ──▶ /odom
 ├─ GPU Lidar plugin ──▶ /scan_gz
 └─ IMU plugin ────────▶ /imu_gz
        │
    ros_gz_bridge
        │
 ├─ /odom ──▶ EKF (robot_localization) ──▶ odom→base_footprint TF
 ├─ /scan ──▶ SLAM Toolbox ──▶ /map + map→odom TF
 └─ /imu  ──▶ EKF
        │
  frontier_explorer
   (WFD on /map, scoring, state machine)
        │
  Nav2 (navigate_to_pose action)
        │
  /cmd_vel ──▶ ros_gz_bridge ──▶ Gazebo DiffDrive
```

---

## Repository Structure

```
LeoRoverAutonomousExploration/
├── README.md                              ← This file
├── run_sim.sh                             ← One-command simulation launcher
├── leo_exploration_ws/                    ← ROS 2 workspace
│   └── src/
│       └── leo_exploration/               ← Main ROS 2 package
│           ├── leo_exploration/
│           │   ├── frontier_explorer.py   ← Core exploration node (WFD + state machine)
│           │   └── __init__.py
│           ├── launch/
│           │   ├── sim_exploration_launch.py   ← Simulation launch (Gazebo + full stack)
│           │   └── exploration_launch.py       ← Real robot launch (RPLidar + stack)
│           ├── config/
│           │   ├── nav2_params.yaml       ← Nav2 controller / planner / costmap
│           │   ├── slam_toolbox_params.yaml ← SLAM Toolbox settings
│           │   ├── ekf.yaml               ← EKF odometry fusion
│           │   ├── ros_gz_bridge.yaml     ← Gazebo ↔ ROS topic bridge
│           │   └── rviz2_config.rviz      ← RViz2 visualisation layout
│           ├── urdf/
│           │   └── leo_rover.urdf         ← Robot model + Gazebo plugins
│           ├── worlds/
│           │   └── exploration_test.world ← 12 × 12 m indoor test environment
│           ├── scripts/
│           │   ├── install_sim_deps.sh    ← Dependency installer
│           │   ├── obstacle_manager.sh    ← Runtime obstacle injection tool
│           │   └── run_sim.sh
│           ├── package.xml
│           ├── setup.py
│           └── setup.cfg
└── nav2_wavefront_frontier_exploration-main/  ← Reference WFD library
    ├── nav2_wfd/
    │   └── wavefront_frontier.py          ← Original BFS frontier algorithm
    ├── package.xml
    ├── setup.py
    └── LICENSE                            ← MIT (Sean Regan, 2020)
```

---

## Prerequisites

| Requirement | Version |
|---|---|
| Ubuntu | 24.04 LTS (Noble) |
| ROS 2 | Jazzy |
| Gazebo | Harmonic (gz-sim 8.x) — simulation only |
| Python | 3.12 |

**ROS 2 packages:** `nav2_bringup`, `slam_toolbox`, `robot_localization`, `rplidar_ros`, `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, `joint_state_publisher`, `tf2_ros`.

> The included `install_sim_deps.sh` script installs all dependencies automatically.

---

## Quick Start (Simulation)

### One-command launch

```bash
git clone https://github.com/Team4-UoM-RSDP/LeoRoverAutonomousExploration.git
cd LeoRoverAutonomousExploration
chmod +x run_sim.sh
./run_sim.sh
```

### Step-by-step

```bash
# 1. Clean up any old processes
pkill -f "gz sim" ; pkill -f nav2 ; pkill -f slam ; pkill -f frontier ; pkill -f rviz2

# 2. Install dependencies
cd leo_exploration_ws/src/leo_exploration/scripts
chmod +x install_sim_deps.sh
./install_sim_deps.sh

# 3. Build and source
cd ../../../               # back to leo_exploration_ws/
colcon build --packages-select leo_exploration
source install/setup.bash

# 4. Launch simulation
ros2 launch leo_exploration sim_exploration_launch.py
```

The robot spawns into a 12 × 12 m indoor world and begins autonomous exploration after a 40-second staged startup.

---

## Real Robot Deployment

The `frontier_explorer` node is **100 % shared** between simulation and the real robot. Only the launch file differs.

```bash
# 1. Connect RPLidar A2M12 and set permissions
sudo chmod 666 /dev/ttyUSB0

# 2. Build and source
cd leo_exploration_ws
colcon build --packages-select leo_exploration
source install/setup.bash

# 3. Launch
ros2 launch leo_exploration exploration_launch.py

# Or specify a different serial port
ros2 launch leo_exploration exploration_launch.py serial_port:=/dev/ttyUSB1
```

**RPLidar configuration (defaults):**

| Parameter | Value |
|---|---|
| Serial port | `/dev/ttyUSB0` |
| Baud rate | 256 000 |
| Frame ID | `laser` |
| Scan mode | Standard |

> **Note:** The system only uses the front 180° of the lidar. Make sure the lidar is physically mounted facing forward. If mounted backwards, adjust the static TF in `exploration_launch.py`.

---

## Launch Options

### Simulation

```bash
# Headless — no Gazebo GUI (faster, lower GPU usage)
ros2 launch leo_exploration sim_exploration_launch.py gz_gui:=false

# Custom Gazebo world
ros2 launch leo_exploration sim_exploration_launch.py world:=/path/to/my.world

# Custom spawn position
ros2 launch leo_exploration sim_exploration_launch.py spawn_x:=2.0 spawn_y:=1.0

# Disable RViz
ros2 launch leo_exploration sim_exploration_launch.py rviz:=false
```

### Real Robot

```bash
# Custom serial port
ros2 launch leo_exploration exploration_launch.py serial_port:=/dev/ttyUSB1

# Custom laser height
ros2 launch leo_exploration exploration_launch.py laser_height:=0.12
```

---

## Startup Timeline

All components launch on a timed schedule to guarantee each dependency is ready before its consumers start.

### Simulation

| Time | Component | Purpose |
|---|---|---|
| t = 0 s | Gazebo Harmonic + `robot_state_publisher` | Physics engine, URDF → TF |
| t = 5 s | Spawn Leo Rover | Place robot model in Gazebo world |
| t = 7 s | `ros_gz_bridge` | Bridge `/scan`, `/odom`, `/cmd_vel`, `/tf`, `/clock`, `/imu` |
| t = 9 s | EKF (`robot_localization`) | Fuse `/odom` + `/imu` → odom → base_footprint TF |
| t = 12 s | SLAM Toolbox | Online async mapping from `/scan` |
| t = 15 s | RViz2 | Visualisation |
| t = 22 s | Nav2 stack | Controller, planner, behaviour server, bt_navigator, lifecycle manager |
| t = 40 s | `frontier_explorer` | Initial forward drive → autonomous WFD exploration |

### Real Robot

| Time | Component |
|---|---|
| t = 0 s | RPLidar A2M12 node + static TF + EKF |
| t = 4 s | SLAM Toolbox |
| t = 6 s | RViz2 |
| t = 10 s | Nav2 stack |
| t = 18 s | `frontier_explorer` |

---

## Exploration Algorithm

### State Machine

The `frontier_explorer` node drives exploration through six states:

| State | Behaviour |
|---|---|
| **INIT_FORWARD** | Drive straight forward for 3 s at 0.15 m/s to seed the SLAM map (no spin). |
| **SELECT_FRONTIER** | Run WFD BFS on the occupancy grid. Cluster frontier cells, compute centroids, filter by costmap inflation, score, and dispatch the best frontier to Nav2. |
| **NAVIGATING** | Monitor the Nav2 action for completion or timeout (35 s). The safety perimeter check runs every iteration. |
| **AVOIDING** | Obstacle too close — cancel Nav2, back up for 1.8 s, then execute a gentle forward curve for 2.0 s (no spin). |
| **RECOVERING** | Multiple consecutive failures or few frontiers — drive forward slowly for 4.0 s to expose unseen areas (no spin). |
| **COMPLETE** | No frontiers found for 8 consecutive checks. Automatically saves the map to disk and shuts down timers. |

### Wavefront Frontier Detection (WFD)

1. Convert the robot's map-frame pose to grid coordinates.
2. BFS from the robot cell to find the nearest free cell.
3. Expand a wavefront outward from that free cell, visiting every reachable cell.
4. When a frontier cell is found (free cell adjacent to unknown), trace its connected cluster.
5. Discard clusters smaller than 5 cells; compute the centroid of each remaining cluster.
6. Filter centroids that fall inside costmap lethal zones (cost ≥ 70).

### Frontier Scoring

```
score = 0.45 × info_gain + 0.35 × distance_score + 0.15 × direction_score − visit_penalty
```

| Factor | Description |
|---|---|
| **info_gain** | Cluster size normalised to [0, 1] (larger frontiers = more unexplored area). |
| **distance_score** | Prefers frontiers in the 1–4 m range; penalises very close (< 0.5 m filtered out) and very far targets. |
| **direction_score** | Strongly prefers frontiers ahead of the robot heading (since the robot never rotates in place). |
| **visit_penalty** | 0.40 penalty if another frontier was visited within 0.6 m (anti ghost-wall mechanism). |

### Safety Systems

| Layer | Coverage | Threshold | Purpose |
|---|---|---|---|
| **Navigation obstacle check** | Front 180° (±90°) | 0.55 m | Filters rear readings from chassis pillars; triggers avoidance. |
| **Safety perimeter** | Full 360° | 0.50 m | Runs every control iteration; overrides any state to trigger immediate avoidance. |

---

## Configuration Reference

### Frontier Explorer Parameters

All parameters can be overridden in the launch file or via `ros2 param set`.

| Parameter | Default | Description |
|---|---|---|
| `robot_frame` | `base_link` | Robot body TF frame |
| `map_frame` | `map` | Map TF frame |
| `min_frontier_size` | `5` | Minimum cluster size (cells) to be a valid frontier |
| `obstacle_dist` | `0.55` | Front obstacle detection threshold (m) |
| `scan_half_angle` | `90.0` | Half-angle of the front lidar sector (degrees) |
| `safety_radius` | `0.50` | 360° safety perimeter radius (m) |
| `nav_timeout` | `35.0` | Max time for a single Nav2 goal (s) |
| `init_forward_speed` | `0.15` | Forward speed during INIT_FORWARD (m/s) |
| `init_forward_duration` | `3.0` | Duration of initial forward drive (s) |
| `backup_speed` | `-0.18` | Reverse speed during avoidance (m/s) |
| `backup_duration` | `1.8` | Duration of avoidance backup (s) |
| `avoid_curve_speed` | `0.10` | Linear speed during avoidance curve (m/s) |
| `avoid_curve_angular` | `0.5` | Angular speed during avoidance curve (rad/s) |
| `avoid_curve_duration` | `2.0` | Duration of avoidance curve (s) |
| `recov_forward_speed` | `0.12` | Forward speed during recovery (m/s) |
| `recov_forward_duration` | `4.0` | Duration of recovery forward drive (s) |
| `max_consec_fail` | `4` | Consecutive Nav2 failures before entering recovery |
| `costmap_clear_every` | `3` | Clear costmaps every N failures |
| `complete_no_frontier` | `8` | Consecutive no-frontier cycles before declaring complete |
| `log_interval` | `12.0` | Progress logging interval (s) |
| `save_map_on_complete` | `true` | Whether to save the map on completion |
| `map_save_path` | `/tmp/leo_explored_map` | Path prefix for saved map files |

### Nav2 Configuration Highlights (`nav2_params.yaml`)

| Setting | Value |
|---|---|
| Global planner | NavFn (Dijkstra, `allow_unknown: true`) |
| Local controller | Regulated Pure Pursuit |
| Controller frequency | 10 Hz |
| Desired linear velocity | 0.30 m/s |
| Lookahead distance | 0.6 m |
| Goal tolerance (XY) | 0.30 m |
| Goal tolerance (yaw) | 0.30 rad |

### SLAM Toolbox Configuration Highlights (`slam_toolbox_params.yaml`)

| Setting | Value |
|---|---|
| Mode | Online async |
| Solver | Ceres (SPARSE_NORMAL_CHOLESKY) |
| Map resolution | 0.05 m/cell |
| Max laser range | 12.0 m |
| Minimum travel distance | 0.3 m |
| Loop closure | Enabled |

### EKF Configuration (`ekf.yaml`)

| Setting | Value |
|---|---|
| Frequency | 30 Hz |
| Odom sensor | `/odom` — uses velocity (vx, vy, vyaw) |
| IMU sensor | `/imu` — uses absolute yaw + yaw velocity |
| Output TF | odom → base_footprint |

---

## Runtime Controls

```bash
# Pause exploration
ros2 topic pub /explore/enable std_msgs/msg/Bool '{data: false}' --once

# Resume exploration
ros2 topic pub /explore/enable std_msgs/msg/Bool '{data: true}' --once

# Monitor progress
ros2 topic echo /rosout | grep -E "Progress|Explored|COMPLETE"

# Clear costmaps (if navigation is stuck)
ros2 service call /global_costmap/clear_entirely_global_costmap \
  nav2_msgs/srv/ClearEntireCostmap {}

# Inspect the TF tree
ros2 run tf2_tools view_frames

# Check lidar frequency (should be ~10 Hz)
ros2 topic hz /scan
```

---

## Adding Obstacles (Simulation)

Use `obstacle_manager.sh` to dynamically inject or remove obstacles in the running Gazebo simulation without restarting.

```bash
cd leo_exploration_ws/src/leo_exploration/scripts
chmod +x obstacle_manager.sh

# Add a 0.5 × 0.5 × 0.8 m box at position (3.0, 2.0)
./obstacle_manager.sh add 3.0 2.0

# Add a 3 m wall at position (0, 3) with angle 0
./obstacle_manager.sh wall 0.0 3.0 3.0 0.0

# List all models in the world
./obstacle_manager.sh list

# Remove all dynamically added obstacles
./obstacle_manager.sh clear
```

Alternatively, edit `worlds/exploration_test.world` directly or drag-and-drop objects using the Gazebo Harmonic GUI.

---

## Robot Model

The URDF (`urdf/leo_rover.urdf`) describes the full Leo Rover kinematic and dynamic model.

| Property | Value |
|---|---|
| Footprint | 445 mm (L) × 289 mm (W) × 200 mm (H) |
| Wheelbase | 295 mm |
| Track width | 359 mm (centre-to-centre) |
| Wheel radius | 64.5 mm |
| Wheel width | 65 mm |
| Body mass | 4.5 kg |
| Wheel mass | 0.5 kg each |
| Lidar height | 135.5 mm above `base_link` |

**Gazebo plugins included in the URDF:**

| Plugin | Function |
|---|---|
| `gz-sim-diff-drive-system` | Differential drive: subscribes to `/cmd_vel`, publishes `/odom` + TF |
| `gz-sim-joint-state-publisher-system` | Publishes `/joint_states` |
| IMU sensor | Publishes `/imu_gz` at 50 Hz (with noise) |
| GPU Lidar sensor | Publishes `/scan_gz` — 360 rays, 0.15–12.0 m range, 10 Hz |

---

## Troubleshooting

| Symptom | Diagnosis | Fix |
|---|---|---|
| Gazebo does not open | Version mismatch | Run `gz sim --version` — need 8.x (Harmonic) |
| Robot not visible in Gazebo | Spawn failed | Check logs for `spawn_leo` errors |
| Robot spins indefinitely | Nav2 not active | Run `ros2 lifecycle get /bt_navigator` |
| "TF not ready" warnings | Missing odom → base TF | Run `ros2 topic hz /tf` to confirm publishing |
| Slow simulation | GPU overloaded | Launch with `gz_gui:=false` for headless mode |
| Old processes interfere | Stale nodes | `pkill -f "gz sim" ; pkill -f nav2 ; pkill -f slam ; pkill -f frontier ; pkill -f rviz2` |
| Lidar shows rear obstacles | Chassis pillar reflections (real robot) | Ensure `scan_half_angle` is 90° (default) |
| Robot keeps revisiting same spot | Ghost-wall / visit penalty too low | Increase `complete_no_frontier` or check costmap |

---

## Gazebo Test World

The default world (`worlds/exploration_test.world`) provides:

- **12 × 12 m** walled indoor room
- **2.5 m** high outer walls (0.2 m thick)
- **8 movable boxes** (0.5 × 0.5 × 0.8 m) scattered as obstacles
- Directional sunlight with shadows
- 1 ms physics timestep, real-time factor 1.0
- Robot spawns at the centre (0, 0, 0.1 m)

---

## License

This project is licensed under the **MIT License**.

The Wavefront Frontier Detection algorithm in `nav2_wavefront_frontier_exploration-main/` is based on work by [Sean Regan](https://github.com/SeanReg/nav2_wavefront_frontier_exploration) (MIT License, 2020), which implements the approach described in [Keidar & Kaminka (2012)](https://arxiv.org/abs/1806.03581).
