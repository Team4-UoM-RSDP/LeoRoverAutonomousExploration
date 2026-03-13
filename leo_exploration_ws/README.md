# Leo Rover Autonomous Exploration (v3.0)

**ROS 2 Jazzy · Ubuntu 24.04 · Gazebo Harmonic**

Frontier-based autonomous exploration for Leo Rover — powered by a **Wavefront Frontier Detection (WFD)** algorithm. Works identically in simulation and on the real robot, using a strict **no-rotation** policy to maintain lidar stability.

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
| Exploration | `frontier_explorer` (WFD algorithm) | `frontier_explorer` (WFD algorithm) |

---

## What's New in v3.0

- **WFD (Wavefront Frontier Detection)**: Implements a BFS-based wavefront expansion algorithm for highly accurate frontier centroid detection (replacing the old NumPy morphology approach). 
- **No-Rotation Policy**: The robot will *never* spin in place. This prevents the lidar from losing tracking:
  - Startup: Drives straight forward briefly (`INIT_FORWARD`) to seed the map instead of a 360° spin.
  - Avoidance: Backs up and takes a gentle curve.
  - Recovery: Drives forward slowly to expose new frontiers.
- **180° Front-Only Lidar Filter**: Ignores scans behind the robot (angles beyond ±90°) to prevent false positives from the real Leo Rover chassis pillars blocking the rear lidar view.
- **Anti Ghost-Wall**: On obstacle encounter, the system backs up and curves *without* regenerating a new arbitrary frontier. Multi-factor frontier scoring heavily penalizes recently visited spots to prevent endless returning/oscillation.

---
## once

```bash
git clone https://github.com/Team4-UoM-RSDP/LeoRoverAutonomousExploration.git
cd ~/LeoRoverAutonomousExploration && \
pkill -f "gz sim" || true; pkill -f nav2 || true; pkill -f slam || true; pkill -f frontier || true; pkill -f rviz2 || true; \
cd leo_exploration_ws/src/leo_exploration/scripts && chmod +x install_sim_deps.sh && ./install_sim_deps.sh && \
cd ../../../ && colcon build --packages-select leo_exploration && \
source install/setup.bash && \
ros2 launch leo_exploration sim_exploration_launch.py
```

## Quick Start (Simulation)

```bash
# Clean up old processes if needed
pkill -f "gz sim" ; pkill -f nav2 ; pkill -f slam ; pkill -f frontier ; pkill -f rviz2

# Install Dependencies
cd ./leo_exploration_ws/src/leo_exploration/scripts
chmod +x install_sim_deps.sh
./install_sim_deps.sh

# Build & Source
cd ./leo_exploration_ws
colcon build --packages-select leo_exploration
source install/setup.bash

# Launch Simulation
ros2 launch leo_exploration sim_exploration_launch.py
```

### Startup timeline (fully automatic):

| Time | Component |
|------|-----------|
| t=0s | Gazebo Harmonic + robot_state_publisher |
| t=5s | Spawn Leo Rover into Gazebo |
| t=7s | ros_gz_bridge (scan, odom, cmd_vel, tf, clock) |
| t=12s | SLAM Toolbox (online async mapping) |
| t=15s | RViz2 |
| t=22s | Nav2 (controller, planner, behavior, bt_navigator) |
| t=40s | Frontier Explorer → Initial forward drive → Autonomous WFD exploration |

### Launch Options

```bash
# Headless (no Gazebo GUI — faster, lower GPU usage)
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

**Yes.** The `frontier_explorer` node is 100% shared between simulation and the real robot. Just launch `exploration_launch.py` instead of the simulation launch.

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

> **⚠️ Important**: Make sure your lidar is physically mounted as configured. The system only looks at the front 180 degrees. If mounted backwards, you will need to adjust the static TF transform logic in the launch file.

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

You can run `scripts/obstacle_manager.sh` to dynamically inject or remove obstacles into the Gazebo sim without restarting.

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

Alternatively, you can edit `worlds/exploration_test.world` or drag-and-drop objects using the Gazebo Harmonic GUI.

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

## Exploration Algorithm

The `frontier_explorer` node implements WFD frontier-based exploration through a robust state machine:

1. **INIT_FORWARD** — Initial short drive forward (no spin) to initialize the lidar scan in the SLAM map.
2. **SELECT_FRONTIER** — Execute WFD BFS on the occupancy grid. Convert frontier clusters to centroids. Apply Costmap lethal-zone filtering, then score by `info_gain + distance + direction - visit_penalty`.
3. **NAVIGATING** — Dispatch Nav2 goal and monitor obstacle proximity.
4. **AVOIDING** — If an obstacle gets too close, cancel Nav2, back up, and execute a mild forward curve (no spin) to get clear.
5. **RECOVERING** — If valid frontiers dwindle or multiple consecutive navigation failures occur, drive forward slowly (no spin) to expose unseen areas.
6. **COMPLETE** — Declared when no frontiers remain for 8 consecutive checks. The map is then automatically saved to disk.

Scoring formula:
```
score = 0.45 × info_gain + 0.35 × distance_score + 0.15 × direction_score − visit_penalty
```
还有要保证车身周围有一个安全距离，一旦发现车的四周小于这个距离就立刻远离防止碰撞，我看到模拟车子会直接开到箱子那边然后检测到有障碍物就开始旋转车身就撞上了，避免这个问题的发生在真机和模拟中
