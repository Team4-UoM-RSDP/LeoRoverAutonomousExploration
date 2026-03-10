# Leo Rover Autonomous Exploration

**ROS 2 Jazzy · Ubuntu 24.04 · Gazebo Harmonic · Nav2 · SLAM Toolbox**

Fully autonomous frontier-based exploration for the [Leo Rover](https://www.leorover.tech/) equipped with an RPLidar A2M12.
A single `frontier_explorer` node (v3.0) drives both simulation and real-robot deployments — no code changes required between the two.

---

## Features

- **Frontier-based exploration** — automatically discovers and navigates to unexplored map boundaries using 8-connected BFS clustering
- **Sim & real parity** — identical exploration node for Gazebo simulation and physical Leo Rover
- **Self-contained simulation** — includes URDF, Gazebo world, and bridge config (no external Leo packages needed)
- **Anti-ghost-wall design (v3.0)** — eliminates all in-place rotations that inject odometry noise into SLAM; uses forward-drive + arc turns instead
- **TF jump detection (v2.5)** — monitors `map → odom` for sudden corrections from SLAM loop closures; cancels navigation and waits for map to stabilise
- **Adaptive blacklisting (v2.4)** — failed navigation goals are blacklisted with a radius that scales with consecutive failures
- **Performance-optimised** — nine optimisations including cached numpy conversions for map/costmap/scan data, vectorised scoring, single-pass map statistics, and batched costmap inflation filtering
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
| Lidar | Gazebo `GpuLidar` plugin → `ros_gz_bridge` → `/scan` | RPLidar A2M12 → `/scan` |
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

The script installs:

- ROS 2 Jazzy packages — Nav2, SLAM Toolbox, TF2, `robot_state_publisher`, `joint_state_publisher`
- Gazebo Harmonic (with fallback to OSRF apt source)
- ROS–Gazebo bridge — `ros_gz`, `ros_gz_bridge`, `ros_gz_sim`, `ros_gz_interfaces`
- RPLidar driver (optional, for real robot)
- Python dependencies — `numpy`, `colcon-common-extensions`

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
| t = 7 s | `ros_gz_bridge` (`/scan`, `/odom`, `/cmd_vel`, `/tf`, `/clock`, `/joint_states`) |
| t = 12 s | SLAM Toolbox (online async mapping) |
| t = 15 s | RViz2 |
| t = 22 s | Nav2 — controller, planner, behavior server, bt\_navigator, lifecycle manager |
| t = 40 s | Frontier Explorer → autonomous exploration begins immediately |

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

The real-robot launch sequence:

| Time | Component |
|------|-----------|
| t = 0 s | RPLidar A2M12 + static TF (`base_link → laser` at z = 0.12 m) |
| t = 4 s | SLAM Toolbox (online async mapping) |
| t = 6 s | RViz2 |
| t = 10 s | Nav2 (via `navigation_launch.py`) |
| t = 18 s | Frontier Explorer |

### Simulation vs. real robot

| | Simulation | Real Robot |
|--|-----------|------------|
| `use_sim_time` | `true` | `false` |
| Nav2 params `use_sim_time` | `true` | needs `false` |
| Lidar source | Gazebo `GpuLidar` plugin | RPLidar A2M12 node |
| Odometry source | Gazebo `DiffDrive` | Leo firmware |

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
./scripts/obstacle_manager.sh wall 0.0 3.0 3.0 0.0 # wall at (0, 3), 3 m long, angle 0
./scripts/obstacle_manager.sh list                  # list models
./scripts/obstacle_manager.sh clear                 # remove all added obstacles
```

Boxes are 0.5 × 0.5 × 0.8 m with cyclic colours (red, blue, green, yellow). Walls accept a length and angle (radians). All obstacles are tracked in `/tmp/leo_obs_counter` for cleanup.

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

The `frontier_explorer` node (v3.0) implements a **six-state finite-state machine**:

```
SELECT_FRONTIER ⇄ NAVIGATING
       ↕                ↓
   RECOVERING       AVOIDING
       ↓
   COMPLETE
```

> **v3.0 change:** `INIT_SPIN` (360° in-place rotation) has been removed. The RPLidar A2M12’s 360° field of view seeds the SLAM map without spinning. The legacy state is retained in the enum but skipped at startup; the node begins directly in `SELECT_FRONTIER`.

| State | Description |
|-------|-------------|
| **SELECT_FRONTIER** | Detect frontier cells (free cells neighbouring unknown), cluster via 8-connected BFS, score, and dispatch the best goal to Nav2. Goal orientation is set to the bearing toward the frontier to avoid unnecessary rotation at arrival. |
| **NAVIGATING** | Wait for Nav2 result; performs emergency lidar-based obstacle avoidance if an object is within `obstacle_dist` in the front `obstacle_half_angle` cone |
| **AVOIDING** | Two-phase manoeuvre — back up (`backup_speed` for `backup_duration`), then forward arc turn (0.05 m/s linear + 0.5 rad/s angular) to escape an obstacle without pure in-place rotation |
| **RECOVERING** | Forward-drive + gentle arc turn when consecutive failures exceed `max_consec_fail`; avoids SLAM-confusing stationary rotation |
| **COMPLETE** | All frontiers exhausted for `complete_no_frontier` consecutive checks; map is auto-saved to `map_save_path` |

### Frontier scoring (v3.0)

```
score = 0.25 × info_gain
      + 0.30 × distance_score
      + 0.35 × direction_score
      − visit_penalty
      − fail_direction_penalty
      ± random_perturbation (if consec_fail ≥ 2)
```

| Component | Formula | Notes |
|-----------|---------|-------|
| **info_gain** | `min(1.0, cluster_size / 60)` | Normalised cluster size |
| **distance_score** | `1.0 − abs(dist − 2.0) / 4.0` if dist ≤ 4 m; decays beyond | Prefers 1–4 m range |
| **direction_score** | `max(0.0, 1.0 − angle_diff / π)` | Strongly prefers frontiers ahead of the robot |
| **visit_penalty** | 0.55 if any visited point within 1.0 m | Discourages circling over explored territory |
| **fail_direction_penalty** | 0.35 if frontier bearing within 30° of a recent failed-goal bearing | Steers away from failed directions |
| **random_perturbation** | ±0.15 | Applied after ≥ 2 consecutive failures to break deterministic re-selection |

**Pre-scoring filters:**

- Frontiers with fewer than `min_frontier_size` cells (default 5) are discarded
- Frontiers closer than 0.5 m are discarded (Nav2 reports instant success without moving)
- Frontiers inside the Nav2 costmap inflation zone (cost ≥ 70) are filtered out
- Frontiers within the adaptive blacklist radius of a previously failed goal are excluded

### Adaptive blacklisting

When a navigation goal fails (rejected, timed out, or triggered emergency avoidance), the goal coordinates are added to a time-decaying blacklist:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `blacklist_radius` | 2.0 m | Base exclusion radius around failed goals |
| `blacklist_duration` | 300.0 s | How long a blacklist entry persists |
| Adaptive scaling | `radius × (1 + 0.3 × consec_fail)` | Radius grows with consecutive failures |
| Capacity | 100 entries | Maximum blacklist size (oldest entries evicted first) |

Failed goal bearings are also tracked; frontiers within 30° of a recently failed bearing receive a penalty, preventing the robot from repeatedly heading in the same unsuccessful direction.

### TF jump detection

The node monitors the `map → odom` transform for sudden corrections caused by SLAM loop closures:

| Threshold | Value |
|-----------|-------|
| Displacement | > 1.0 m |
| Rotation | > 30° (0.524 rad) |
| Cooldown after detection | 5.0 s |

When a jump is detected, the node cancels the current navigation goal, clears costmaps, and waits for the map to stabilise before resuming exploration.

### Key parameters

All parameters are configurable via launch files or ROS 2 CLI overrides (`-p name:=value`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_frame` | `base_link` | Robot body frame |
| `map_frame` | `map` | Map frame |
| `min_frontier_size` | 5 | Minimum cells to consider a frontier cluster |
| `obstacle_dist` | 0.50 m | Emergency obstacle avoidance trigger distance |
| `obstacle_half_angle` | 50.0° | Half-width of the forward detection cone |
| `nav_timeout` | 35.0 s | Cancel navigation goal if no result within this time |
| `spin_speed` | 0.55 rad/s | Angular velocity for arc manoeuvres |
| `backup_speed` | −0.18 m/s | Reverse speed during obstacle avoidance backup |
| `backup_duration` | 1.8 s | Duration of the backup phase |
| `avoid_spin_duration` | 2.5 s | Duration of the arc-turn avoidance phase |
| `recov_spin_duration` | 7.0 s | Duration of the recovery arc-turn phase |
| `max_consec_fail` | 4 | Consecutive failures before entering RECOVERING |
| `costmap_clear_every` | 3 | Clear costmaps every N navigation failures |
| `complete_no_frontier` | 8 | Consecutive empty frontier checks before COMPLETE |
| `log_interval` | 12.0 s | Progress logging period |
| `save_map_on_complete` | `true` | Auto-save map on completion |
| `map_save_path` | `/tmp/leo_explored_map` | File path for the saved map |
| `blacklist_radius` | 2.0 m | Base blacklist exclusion radius |
| `blacklist_duration` | 300.0 s | Blacklist entry lifetime |

---

## Performance Optimisations

The `frontier_explorer` node includes nine targeted performance optimisations to minimise per-cycle latency:

| # | Optimisation | Description |
|---|-------------|-------------|
| 1 | Map array caching | `OccupancyGrid.data` → numpy conversion cached, keyed by `header.stamp` |
| 2 | Vectorised frontier scoring | All score components computed via numpy batch operations |
| 3 | Vectorised centroid calculation | `numpy.sum` replaces Python-level summation |
| 4 | BFS with direct numpy indexing | Frontier detection indexes numpy arrays directly (no `tolist()`) |
| 5 | Scan array caching | Laser ranges and angles cached by sequence |
| 6 | Single-pass map statistics | `np.bincount` replaces three separate `np.sum` calls |
| 7 | Costmap array caching | Global costmap `OccupancyGrid.data` → numpy cached by timestamp |
| 8 | Batched costmap inflation filter | All frontiers checked against costmap in one vectorised pass |
| 9 | Dead code removal | Unreachable visited-list trimming removed (deque `maxlen=24`) |

---

## Robot Specification (URDF)

| Property | Value |
|----------|-------|
| Overall dimensions | 424 × 445 × 303 mm (L × W × H) |
| Chassis (body box) | 424 × 445 × 200 mm |
| Mass (body) | 4.5 kg |
| Wheel radius | 65 mm |
| Wheel width | 65 mm |
| Wheel mass | 0.5 kg each |
| Track width (centre-to-centre) | 380 mm |
| Wheelbase | 295 mm |
| Ground clearance | ~65 mm |
| Lidar mount height | 185 mm above `base_link` |
| Lidar collision radius | 60 mm (covers mounting pillars) |

**Gazebo plugins (simulation only):**

| Plugin | Details |
|--------|---------|
| `DiffDrive` | Wheel separation 0.380 m, radius 0.065 m, odom at 20 Hz |
| `GpuLidar` | 360 rays, 12 m max range, 0.18 m min range, 10 Hz, σ = 0.01 noise |
| `JointStatePublisher` | Publishes `/joint_states` |

---

## Simulation World

The default world (`exploration_test.world`, SDF 1.9) provides a 12 × 12 m enclosed indoor environment:

- **Outer walls** — 2.5 m tall, 0.2 m thick, forming a 12 × 12 m room
- **Interior dividers** — horizontal and vertical walls creating corridors and alcoves
- **8 box obstacles** — 0.5 × 0.5 × 0.8 m static boxes in four colours (red, blue, green, yellow)
- **Physics** — step size 0.001 s, real-time factor 1.0
- **Rendering** — ogre2 engine with directional sun lighting

---

## Nav2 Configuration

Key settings from `nav2_params.yaml`:

| Component | Parameter | Value |
|-----------|-----------|-------|
| Controller | Regulated Pure Pursuit, desired velocity | 0.30 m/s |
| Controller | Lookahead distance | 0.6 m |
| Controller | Rotate-to-heading angular velocity | 0.8 rad/s |
| Planner | NavFn (Dijkstra), `allow_unknown: true` | tolerance 1.5 m |
| Local costmap | Rolling 3 × 3 m window, 0.05 m resolution | update 5.0 Hz |
| Global costmap | Full map, `track_unknown_space: true` | update 2.0 Hz |
| Costmap inflation | Inflation radius 0.45 m, cost scaling 3.0 | footprint padding 0.03 m |
| Velocity smoother | Max velocity | [0.30, 0.0, 1.0] m/s |
| Velocity smoother | Max acceleration | [0.50, 0.0, 3.2] m/s² |
| Robot footprint | Rectangular | `[[0.212, 0.223], [0.212, -0.223], [-0.212, -0.223], [-0.212, 0.223]]` |

## SLAM Configuration

Key settings from `slam_toolbox_params.yaml`:

| Parameter | Value |
|-----------|-------|
| Solver | Ceres (SPARSE_NORMAL_CHOLESKY) |
| Map resolution | 0.05 m |
| Map update interval | 1.0 s |
| TF publish period | 0.02 s (50 Hz) |
| Loop closure | **Disabled** (`do_loop_closing: false`) |
| Minimum travel distance | 0.8 m |
| Minimum travel heading | 0.3 rad |

> Loop closure is intentionally disabled to prevent rotation artifacts in symmetric environments.

---

## File Structure

```
leo_exploration_ws/src/leo_exploration/
├── config/
│   ├── nav2_params.yaml            # Nav2 stack parameters (RPP controller, NavFn planner, costmaps)
│   ├── slam_toolbox_params.yaml    # SLAM Toolbox: Ceres solver, loop closure disabled, 0.05 m resolution
│   ├── rviz2_config.rviz           # RViz2 visualisation layout (map, costmaps, laser, frontiers, paths, TF)
│   └── ros_gz_bridge.yaml          # Gazebo ↔ ROS topic bridge (/scan, /odom, /cmd_vel, /tf, /clock, /joint_states)
├── launch/
│   ├── sim_exploration_launch.py   # Simulation launch — direct Nav2 node launch (avoids TF remap bug)
│   └── exploration_launch.py       # Real robot launch — RPLidar, SLAM, Nav2, Explorer
├── leo_exploration/
│   ├── __init__.py
│   └── frontier_explorer.py        # Frontier-based exploration node v3.0 (state machine, scoring, caching)
├── urdf/
│   └── leo_rover.urdf              # Robot description (DiffDrive + GpuLidar + JointState Gazebo plugins)
├── worlds/
│   └── exploration_test.world      # 12 × 12 m indoor environment with walls, dividers, and 8 box obstacles
├── scripts/
│   ├── install_sim_deps.sh         # One-command dependency installer (ROS 2, Gazebo, Nav2, SLAM, numpy)
│   └── obstacle_manager.sh         # Spawn / remove / list obstacles at runtime via Gazebo CLI
├── resource/
│   └── leo_exploration             # ament resource index marker
├── package.xml                     # ROS 2 package manifest (MIT license)
├── setup.py                        # Python package build configuration (v2.4.0)
└── setup.cfg                       # Script install paths
```

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Gazebo won't open | Version mismatch | `gz sim --version` — need Harmonic (8.x) |
| Robot not visible in Gazebo | Spawn timing / failure | Check `spawn_leo` logs; increase spawn delay if needed |
| Robot does not move | Nav2 not yet active | `ros2 lifecycle get /bt_navigator` — wait for `active` |
| "TF not ready" warnings | Missing `odom → base_link` TF | `ros2 topic hz /tf` — verify DiffDrive or firmware is publishing |
| Slow simulation | GPU overloaded | Launch with `gz_gui:=false` to run headless |
| Stale processes after restart | Old nodes lingering | Terminate stale `gz sim`, `nav2`, `slam`, and `frontier` processes |
| "Action server inactive" on real robot | TF remap issue in Jazzy | Use the direct-node-launch approach from `sim_exploration_launch.py` |
| Map has ghost walls or artefacts | SLAM loop closure mismatch | Verify `do_loop_closing: false` in `slam_toolbox_params.yaml`; the TF jump detector will also cancel navigation if a sudden correction is detected |
| Robot keeps returning to the same area | Weak blacklisting | Increase `blacklist_radius` or `blacklist_duration` |

---

## Version History

| Version | Highlights |
|---------|-----------|
| **v3.0** | Anti-ghost-wall: removed `INIT_SPIN`, replaced all pure in-place spins with forward + arc turns, goal orientation set to frontier bearing, scoring rebalanced to prefer forward frontiers |
| **v2.5** | TF jump detection: monitors `map → odom` for sudden SLAM corrections, cancels navigation and enters cooldown |
| **v2.4** | Adaptive blacklisting (radius scales with failures), failure-direction penalty, stronger revisit penalty, random perturbation, blacklist capacity 100 / duration 300 s |
| **v2.3** | Frontier blacklisting: unreachable goals excluded via time-decaying blacklist |

---

## License

This project is licensed under the **MIT License** — see [`package.xml`](leo_exploration_ws/src/leo_exploration/package.xml) for details.
