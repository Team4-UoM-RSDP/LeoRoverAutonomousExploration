# Leo Rover Autonomous Exploration

**ROS 2 Jazzy · Ubuntu 24.04 · Gazebo Harmonic · Nav2 · SLAM Toolbox**

Fully autonomous frontier-based exploration for the [Leo Rover](https://www.leorover.tech/) equipped with an RPLidar A2M12.
A single `frontier_explorer` node drives both simulation and real-robot deployments — no code changes required between the two.

## Features

- **Frontier-based exploration** — automatically discovers and navigates to unexplored map boundaries
- **Sim & real parity** — identical exploration node for Gazebo simulation and physical Leo Rover
- **Self-contained simulation** — includes URDF, Gazebo world, and bridge config (no external Leo packages needed)
- **Performance-optimised** — cached numpy conversions for map/costmap/scan data, vectorised scoring, single-pass map statistics, and batched costmap inflation filtering
- **Pause / resume** — publish to `/explore/enable` at any time
- **Automatic map saving** — final map saved on exploration completion via `map_saver_cli`
- **Runtime obstacle management** — spawn/remove obstacles in Gazebo on the fly

---

## Architecture

```
                ┌──────────────────────────────────────┐
                │      frontier_explorer node           │
                │   (shared between sim & real robot)   │
                └─────┬────────────────────┬────────────┘
                      │ /navigate_to_pose  │ /cmd_vel
               ┌──────▼──────┐      ┌──────▼──────────┐
               │  Nav2 Stack  │      │  Gazebo DiffDrive│
               │  (planner,   │      │  or Leo firmware │
               │   controller,│      └─────────────────┘
               │   bt_nav)    │
               └──────┬───────┘
                      │ TF: map → odom
               ┌──────▼──────┐
               │ SLAM Toolbox │◄──── /scan
               └──────────────┘
```

**TF tree** (identical in simulation and on hardware):

```
map ─(SLAM)─▶ odom ─(DiffDrive / firmware)─▶ base_footprint ─(URDF)─▶ base_link ─▶ laser
```

| Component | Simulation | Real Robot |
|-----------|-----------|------------|
| Lidar | Gazebo `gpu_lidar` plugin → `ros_gz_bridge` → `/scan` | RPLidar A2M12 → `/scan` |
| Odometry | Gazebo `DiffDrive` → `ros_gz_bridge` → `/odom` + TF | Leo firmware → `/odom` + TF |
| Static TF | `robot_state_publisher` (from URDF) | `static_transform_publisher` |
| Exploration | `frontier_explorer` (same code) | `frontier_explorer` (same code) |

---

## Quick Start (Simulation)

### 1. Install dependencies (once)

```bash
cd leo_exploration_ws/src/leo_exploration/scripts
chmod +x install_sim_deps.sh
./install_sim_deps.sh
```

The script installs ROS 2 Jazzy packages (Nav2, SLAM Toolbox, TF2, `ros_gz`), Gazebo Harmonic, and Python dependencies (numpy).

### 2. Build

```bash
cd leo_exploration_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select leo_exploration
source install/setup.bash
```

### 3. Launch

```bash
ros2 launch leo_exploration sim_exploration_launch.py
```

The launch file orchestrates all components with timed delays:

| Time | Component |
|------|-----------|
| t = 0 s | Gazebo Harmonic + `robot_state_publisher` |
| t = 5 s | Spawn Leo Rover into Gazebo |
| t = 7 s | `ros_gz_bridge` (`/scan`, `/odom`, `/cmd_vel`, `/tf`, `/clock`) |
| t = 12 s | SLAM Toolbox (online async mapping) |
| t = 15 s | RViz2 |
| t = 22 s | Nav2 — controller, planner, behavior server, bt\_navigator, lifecycle manager |
| t = 40 s | Frontier Explorer → 360° initial spin → autonomous exploration |

> **Note:** The simulation launch starts Nav2 as individual nodes (not via `navigation_launch.py`) to avoid a TF-remapping issue in ROS 2 Jazzy.

### Launch options

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

---

## Real Robot Deployment

The `frontier_explorer` node is 100% shared between simulation and real hardware.

### Steps

```bash
# 1. Connect RPLidar and set permissions
sudo chmod 666 /dev/ttyUSB0

# 2. Build & source
cd leo_exploration_ws
colcon build --packages-select leo_exploration
source install/setup.bash

# 3. Launch
ros2 launch leo_exploration exploration_launch.py

# Specify a different serial port if needed
ros2 launch leo_exploration exploration_launch.py serial_port:=/dev/ttyUSB1
```

### Simulation vs. real robot

| | Simulation | Real Robot |
|--|-----------|------------|
| `use_sim_time` | `true` | `false` |
| Nav2 params `use_sim_time` | `true` | needs `false` |
| Lidar source | Gazebo plugin | RPLidar A2M12 node |
| Odometry source | Gazebo DiffDrive | Leo firmware |

> **⚠️ `nav2_params.yaml`** ships with `use_sim_time: true`. For real-robot use, change it to `false` or maintain a separate params file.

> **⚠️ Nav2 launch** — the real-robot launch currently uses `IncludeLaunchDescription(navigation_launch.py)`, which has a known TF-remapping issue in Jazzy. If you see "action server inactive" errors, apply the direct-node-launch approach from `sim_exploration_launch.py`.

---

## Runtime Controls

```bash
# Pause exploration
ros2 topic pub /explore/enable std_msgs/msg/Bool '{data: false}' --once

# Resume exploration
ros2 topic pub /explore/enable std_msgs/msg/Bool '{data: true}' --once

# Monitor progress
ros2 topic echo /rosout | grep -E "Progress|Explored|COMPLETE"

# Clear costmaps manually (if navigation gets stuck)
ros2 service call /global_costmap/clear_entirely_global_costmap \
  nav2_msgs/srv/ClearEntireCostmap {}

# Inspect TF tree
ros2 run tf2_tools view_frames

# Verify lidar frequency (expect ~10 Hz)
ros2 topic hz /scan
```

---

## Adding Obstacles (Simulation)

### GUI (Gazebo)

1. Click the shape icon in the toolbar → click to place → resize in the Inspector panel → enable **Static**

### Command line

```bash
chmod +x scripts/obstacle_manager.sh

./scripts/obstacle_manager.sh add 3.0 2.0          # box at (3, 2)
./scripts/obstacle_manager.sh wall 0.0 3.0 3.0 0.0 # wall at (0, 3), 3 m long
./scripts/obstacle_manager.sh list                  # list models
./scripts/obstacle_manager.sh clear                 # remove all added obstacles
```

### Permanent (edit world file)

Add models to `worlds/exploration_test.world` before the closing `</world>` tag:

```xml
<model name="my_box">
  <static>true</static>
  <pose>3.0 2.0 0.4 0 0 0</pose>
  <link name="link">
    <collision name="c">
      <geometry><box><size>0.5 0.5 0.8</size></box></geometry>
    </collision>
    <visual name="v">
      <geometry><box><size>0.5 0.5 0.8</size></box></geometry>
      <material><ambient>1 0 0 1</ambient></material>
    </visual>
  </link>
</model>
```

---

## Exploration Algorithm

The `frontier_explorer` node implements a **six-state finite-state machine**:

```
INIT_SPIN → SELECT_FRONTIER ⇄ NAVIGATING
                 ↕                  ↓
             RECOVERING         AVOIDING
                 ↓
             COMPLETE
```

| State | Description |
|-------|-------------|
| **INIT_SPIN** | 360° in-place rotation to seed the SLAM map |
| **SELECT_FRONTIER** | Detect frontier cells (free cells neighbouring unknown), cluster via 8-connected BFS, score, and dispatch the best goal to Nav2 |
| **NAVIGATING** | Wait for Nav2 result; performs emergency lidar-based obstacle avoidance if an object is dangerously close |
| **AVOIDING** | Two-phase manoeuvre — back up, then spin — to escape an obstacle |
| **RECOVERING** | Slow recovery spin when consecutive failures exceed threshold |
| **COMPLETE** | All frontiers exhausted for a configurable number of consecutive checks; map is auto-saved |

### Frontier scoring

```
score = 0.45 × info_gain + 0.35 × distance_score + 0.15 × direction_score − visit_penalty
```

- **info_gain** — normalised cluster size (`min(1, size / 60)`)
- **distance_score** — prefers goals in the 1–4 m range, decays beyond
- **direction_score** — prefers frontiers ahead of the robot
- **visit_penalty** — 0.40 if any previously visited point is within 0.6 m
- Frontiers inside the Nav2 costmap inflation zone are filtered out before scoring
- Frontiers closer than 0.5 m are discarded (Nav2 reports instant success without moving)

### Key parameters

All parameters are configurable via launch files or ROS 2 CLI overrides:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_frontier_size` | 5 | Minimum cells to consider a cluster |
| `obstacle_dist` | 0.45 m | Emergency obstacle avoidance threshold |
| `nav_timeout` | 35–45 s | Cancel navigation if no result |
| `spin_duration` | 12.5 s | Initial 360° spin duration |
| `max_consec_fail` | 4 | Failures before recovery spin |
| `complete_no_frontier` | 8 | Consecutive empty checks before COMPLETE |
| `save_map_on_complete` | `true` | Auto-save map on completion |
| `log_interval` | 10–12 s | Progress logging period |

---

## Performance Optimisations

| Version | Optimisation | Impact |
|---------|-------------|--------|
| v2.1 | **Map array caching** — `_get_map_array()` converts `OccupancyGrid.data` to numpy once per map update, not every control tick | Avoids O(w×h) conversion at 5 Hz |
| v2.1 | **Vectorised frontier scoring** — batch distance, angle, and penalty computation via numpy | Eliminates per-frontier Python loops |
| v2.1 | **Vectorised centroid calculation** — `np.array().mean()` instead of `sum()/len()` | Faster for large clusters |
| v2.1 | **Optimised BFS** — direct numpy index access instead of `.tolist()` | Reduces object creation |
| v2.2 | **Scan array caching** — `_obstacle_in_sector()` caches ranges/angles numpy arrays, invalidated on new scan | Avoids O(n) allocation every 5 Hz tick |
| v2.2 | **Single-pass map statistics** — `np.bincount` replaces three separate `np.sum` calls in `_map_stats()` | One pass instead of three |
| v2.2 | **Costmap array caching** — `_get_costmap_array()` mirrors the map cache strategy | Avoids redundant numpy conversion |
| v2.2 | **Batched inflation filter** — `_filter_frontiers_by_costmap()` checks all frontier centroids in one vectorised operation | Removes per-frontier Python loop |

---

## File Structure

```
leo_exploration_ws/src/leo_exploration/
├── config/
│   ├── nav2_params.yaml            # Nav2 stack parameters (RPP controller, NavFn planner, costmaps)
│   ├── slam_toolbox_params.yaml    # SLAM Toolbox: Ceres solver, loop closure, 0.05 m resolution
│   ├── rviz2_config.rviz           # RViz2 visualisation layout
│   └── ros_gz_bridge.yaml          # Gazebo ↔ ROS topic bridge (/scan, /odom, /cmd_vel, /tf, /clock)
├── launch/
│   ├── sim_exploration_launch.py   # Simulation launch — direct Nav2 node launch (avoids TF remap bug)
│   └── exploration_launch.py       # Real robot launch — RPLidar, SLAM, Nav2, Explorer
├── leo_exploration/
│   ├── __init__.py
│   └── frontier_explorer.py        # Frontier-based exploration node (state machine, scoring, caching)
├── urdf/
│   └── leo_rover.urdf              # Robot description (DiffDrive + GpuLidar Gazebo plugins)
├── worlds/
│   └── exploration_test.world      # 12 × 12 m indoor environment with 8 box obstacles
├── scripts/
│   ├── install_sim_deps.sh         # One-command dependency installer (ROS 2, Gazebo, Nav2, SLAM)
│   └── obstacle_manager.sh         # Spawn / remove obstacles at runtime via Gazebo CLI
├── resource/
│   └── leo_exploration             # ament resource index marker
├── package.xml                     # ROS 2 package manifest (MIT license)
├── setup.py                        # Python package build configuration
└── setup.cfg                       # Script install paths
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Gazebo won't open | Version mismatch | `gz sim --version` — need Harmonic (8.x) |
| Robot not visible in Gazebo | Spawn timing/failure | Check `spawn_leo` logs; increase spawn delay if needed |
| Robot spins indefinitely | Nav2 not yet active | `ros2 lifecycle get /bt_navigator` — wait for `active` |
| "TF not ready" warnings | Missing `odom → base_link` TF | `ros2 topic hz /tf` — verify DiffDrive or firmware is publishing |
| Slow simulation | GPU overloaded | Launch with `gz_gui:=false` to run headless |
| Stale processes after restart | Old nodes lingering | `pkill -f "gz sim" ; pkill -f nav2 ; pkill -f slam ; pkill -f frontier` |
| "Action server inactive" on real robot | TF remap issue in Jazzy | Use the direct-node-launch approach from `sim_exploration_launch.py` |

---

## License

MIT
