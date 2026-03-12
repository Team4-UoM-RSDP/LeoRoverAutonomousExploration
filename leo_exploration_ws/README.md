# Leo Rover Autonomous Exploration

**ROS 2 Jazzy · Ubuntu 24.04 · Gazebo Harmonic**

Frontier-based autonomous exploration for Leo Rover — works in both simulation and on the real robot with the same exploration code.

---

## Architecture

```
                    ┌───────────────────────────────────┐
                    │     frontier_explorer node         │
                    │  (shared between sim & real robot) │
                    └──────┬──────────────┬──────────────┘
                           │ /navigate_to_pose    │ /cmd_vel
                    ┌──────▼──────┐      ┌────────▼────────┐
                    │   Nav2 Stack │      │ Controller/     │
                    │  (bt_nav,   │      │ Gazebo DiffDrive│
                    │   planner,  │      │ or Leo firmware  │
                    │   controller│      └─────────────────┘
                    └──────┬──────┘
                           │ TF: map→odom
                    ┌──────▼──────┐
                    │ SLAM Toolbox│◄──── /scan
                    └─────────────┘
```

**TF tree (identical in both modes):**
```
map ──(SLAM)──▶ odom ──(DiffDrive/firmware)──▶ base_footprint ──(URDF)──▶ base_link ──▶ laser
```

| Component | Simulation | Real Robot |
|-----------|-----------|------------|
| Lidar | Gazebo gpu_lidar plugin → bridge → `/scan` | RPLidar A2M12 → `/scan` |
| Odometry | Gazebo DiffDrive → bridge → `/odom` + TF | Leo firmware → `/odom` + TF |
| TF (static) | `robot_state_publisher` from URDF | Static TF node |
| Exploration | `frontier_explorer` (same code) | `frontier_explorer` (same code) |

---

## Quick Start (Simulation)

pkill -f "gz sim" ; pkill -f nav2 ; pkill -f slam ; pkill -f frontier ; pkill -f rviz2
sleep 3
cd ~/ros2_ws
colcon build --packages-select leo_exploration
source install/setup.bash
ros2 launch leo_exploration sim_exploration_launch.py


### 1. Install Dependencies (once)

```bash
cd ~/ros2_ws/src/leo_exploration_ws/src/leo_exploration/scripts
chmod +x install_sim_deps.sh
./install_sim_deps.sh
```

### 2. Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select leo_exploration
source install/setup.bash
```

### 3. Launch

```bash
ros2 launch leo_exploration sim_exploration_launch.py
```

**Startup timeline (fully automatic):**

| Time | Component |
|------|-----------|
| t=0s | Gazebo Harmonic + robot_state_publisher |
| t=5s | Spawn Leo Rover into Gazebo |
| t=7s | ros_gz_bridge (scan, odom, cmd_vel, tf, clock) |
| t=12s | SLAM Toolbox (online async mapping) |
| t=15s | RViz2 |
| t=22s | Nav2 (controller, planner, behavior, bt_navigator) |
| t=40s | Frontier Explorer → 360° spin → autonomous exploration |

### Launch Options

```bash
# Headless (no Gazebo GUI — faster, lower GPU)
ros2 launch leo_exploration sim_exploration_launch.py gz_gui:=false

# Custom world file
ros2 launch leo_exploration sim_exploration_launch.py world:=/path/to/my.world

# Custom spawn position
ros2 launch leo_exploration sim_exploration_launch.py spawn_x:=2.0 spawn_y:=1.0

# No RViz
ros2 launch leo_exploration sim_exploration_launch.py rviz:=false
```

---

## Real Robot Deployment

### Can I use this code directly on the real Leo Rover?

**Yes — with one important caveat.** The `frontier_explorer` node is 100% shared between simulation and real robot. However, the real-robot launch file (`exploration_launch.py`) currently uses Nav2's `navigation_launch.py`, which has a known TF remapping issue in Jazzy. You should apply the same direct-node-launch approach used in the simulation launch.

### Steps to deploy on real hardware

```bash
# 1. Connect RPLidar and set permissions
sudo chmod 666 /dev/ttyUSB0

# 2. Build & source
cd ~/ros2_ws
colcon build --packages-select leo_exploration
source install/setup.bash

# 3. Launch (real robot)
ros2 launch leo_exploration exploration_launch.py

# Or specify serial port
ros2 launch leo_exploration exploration_launch.py serial_port:=/dev/ttyUSB1
```

### Key differences from simulation

| | Simulation | Real Robot |
|--|-----------|------------|
| `use_sim_time` | `true` | `false` |
| Nav2 params `use_sim_time` | `true` | needs `false` |
| Lidar source | Gazebo plugin | RPLidar node |
| Odometry source | Gazebo DiffDrive | Leo firmware |

> **⚠️ Important**: The `nav2_params.yaml` currently has `use_sim_time: true` (set for simulation). For real robot use, either change it back to `false` or create a separate params file for the real robot (recommended).

> **⚠️ Nav2 launch**: The real-robot `exploration_launch.py` still uses `IncludeLaunchDescription(navigation_launch.py)`. If you encounter "action server inactive" errors on real hardware, apply the same direct node launch approach from `sim_exploration_launch.py`.

---

## Runtime Controls

```bash
# Pause exploration
ros2 topic pub /explore/enable std_msgs/msg/Bool '{data: false}' --once

# Resume exploration
ros2 topic pub /explore/enable std_msgs/msg/Bool '{data: true}' --once

# Monitor progress
ros2 topic echo /rosout | grep -E "Progress|Explored|COMPLETE"

# Clear costmaps (if navigation gets stuck)
ros2 service call /global_costmap/clear_entirely_global_costmap \
  nav2_msgs/srv/ClearEntireCostmap {}

# Check TF tree
ros2 run tf2_tools view_frames

# Check lidar frequency (should be ~10 Hz)
ros2 topic hz /scan
```

---

## Adding Obstacles (Simulation)

### Method A: Gazebo GUI (drag & drop)

1. Click the shape icon in the Gazebo toolbar
2. Click to place in the world
3. Use the Inspector panel to resize/reposition
4. Enable "Static" to fix in place

### Method B: Command line

```bash
chmod +x scripts/obstacle_manager.sh

# Add a box at (3.0, 2.0)
./scripts/obstacle_manager.sh add 3.0 2.0

# Add a horizontal wall (x=0, y=3, length=3m)
./scripts/obstacle_manager.sh wall 0.0 3.0 3.0 0.0

# List all models
./scripts/obstacle_manager.sh list

# Remove all added obstacles
./scripts/obstacle_manager.sh clear
```

### Method C: Edit world file (permanent)

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

## Troubleshooting

| Symptom | Diagnosis | Fix |
|---------|-----------|-----|
| Gazebo won't open | Version mismatch | `gz sim --version` (need 8.x) |
| Robot not in Gazebo | Spawn failed | Check logs for spawn_leo errors |
| Robot spins forever | Nav2 not active | `ros2 lifecycle get /bt_navigator` |
| "TF not ready" | Missing odom→base TF | `ros2 topic hz /tf` |
| Slow simulation | GPU overloaded | Use `gz_gui:=false` |
| Old processes linger | Stale nodes | `pkill -f "gz sim" ; pkill -f nav2` |

---

## File Structure

```
leo_exploration/
├── config/
│   ├── nav2_params.yaml            # Nav2 parameters (use_sim_time: true)
│   ├── slam_toolbox_params.yaml    # SLAM Toolbox configuration
│   ├── rviz2_config.rviz           # RViz2 display config (with RobotModel)
│   └── ros_gz_bridge.yaml          # Gazebo↔ROS topic bridge mapping
├── launch/
│   ├── exploration_launch.py       # Real robot launch
│   └── sim_exploration_launch.py   # Simulation launch (direct Nav2 nodes)
├── leo_exploration/
│   └── frontier_explorer.py        # Frontier-based exploration node
├── urdf/
│   └── leo_rover.urdf              # Robot description (links, joints, plugins)
├── worlds/
│   └── exploration_test.world      # 12×12m test environment with obstacles
├── scripts/
│   ├── install_sim_deps.sh         # Dependency installer
│   └── obstacle_manager.sh         # Runtime obstacle management
├── package.xml
└── setup.py
```

---

## Exploration Algorithm

The `frontier_explorer` node implements frontier-based exploration:

1. **INIT_SPIN** — 360° spin to get initial map
2. **SELECT_FRONTIER** — Detect frontier cells (free cells adjacent to unknown), cluster them, score by info gain + distance + direction, filter out frontiers < 0.5m
3. **NAVIGATING** — Send goal to Nav2, with emergency obstacle avoidance
4. **RECOVERING** — Recovery spin when navigation fails repeatedly
5. **COMPLETE** — Declared when no frontiers remain for 8 consecutive checks

Scoring formula:
```
score = 0.45 × info_gain + 0.35 × distance_score + 0.15 × direction_score − visit_penalty
```
