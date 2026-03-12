#!/usr/bin/env python3
"""
Leo Rover Simulation Exploration Launch  (Self-Contained)
=========================================================
Ubuntu 24.04  |  ROS2 Jazzy  |  Gazebo Harmonic (gz-sim 8)

NO external Leo Rover packages required (no leo_gz, no leo_gz_bringup).
Everything is provided by this package: own URDF, own world, own bridge config.

Only apt dependencies needed:
  sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-bringup ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher

What this launch does (in order):
  t=0s   Gazebo Harmonic physics server (headless or GUI)
  t=0s   robot_state_publisher  (publishes URDF + static TF base_link->laser)
  t=3s   Spawn Leo Rover URDF into Gazebo
  t=4s   ros_gz_bridge (/scan, /odom, /cmd_vel, /tf, /clock)
  t=8s   SLAM Toolbox online_async
  t=10s  RViz2
  t=16s  Nav2 navigation stack
  t=26s  Frontier Explorer node

Usage:
  ros2 launch leo_exploration sim_exploration_launch.py
  ros2 launch leo_exploration sim_exploration_launch.py gz_gui:=false
  ros2 launch leo_exploration sim_exploration_launch.py world:=/path/to/my.world
  ros2 launch leo_exploration sim_exploration_launch.py spawn_x:=1.0 spawn_y:=0.5

Pause/resume explorer:
  ros2 topic pub /explore/enable std_msgs/msg/Bool '{data: false}' --once
  ros2 topic pub /explore/enable std_msgs/msg/Bool '{data: true}'  --once
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory("leo_exploration")
    pkg_slm  = get_package_share_directory("slam_toolbox")
    pkg_nav2 = get_package_share_directory("nav2_bringup")

    # ── File paths ───────────────────────────────────────────────────────────
    urdf_file    = os.path.join(pkg, "urdf",   "leo_rover.urdf")
    world_file   = os.path.join(pkg, "worlds", "exploration_test.world")
    nav2_params  = os.path.join(pkg, "config", "nav2_params.yaml")
    slam_params  = os.path.join(pkg, "config", "slam_toolbox_params.yaml")
    rviz_config  = os.path.join(pkg, "config", "rviz2_config.rviz")
    bridge_cfg   = os.path.join(pkg, "config", "ros_gz_bridge.yaml")
    ekf_params   = os.path.join(pkg, "config", "ekf.yaml")

    with open(urdf_file, "r") as f:
        robot_description = f.read()

    # ── Launch arguments ─────────────────────────────────────────────────────
    gz_gui_arg   = DeclareLaunchArgument("gz_gui",   default_value="true",
                                         description="Show Gazebo GUI")
    rviz_arg     = DeclareLaunchArgument("rviz",     default_value="true",
                                         description="Launch RViz2")
    world_arg    = DeclareLaunchArgument("world",    default_value=world_file,
                                         description="Gazebo world file path")
    spawn_x_arg  = DeclareLaunchArgument("spawn_x",  default_value="0.0")
    spawn_y_arg  = DeclareLaunchArgument("spawn_y",  default_value="0.0")

    gz_gui   = LaunchConfiguration("gz_gui")
    rviz     = LaunchConfiguration("rviz")
    world    = LaunchConfiguration("world")
    spawn_x  = LaunchConfiguration("spawn_x")
    spawn_y  = LaunchConfiguration("spawn_y")

    use_sim_time = "true"

    # =========================================================================
    # 1.  Gazebo Harmonic  (t = 0 s)
    #     gz_gui:=true  → full GUI (can see robot move in Gazebo)
    #     gz_gui:=false → headless server only (-s flag, faster)
    # =========================================================================
    gz_sim_gui = ExecuteProcess(
        condition=IfCondition(gz_gui),
        cmd=["gz", "sim", "-r", world],
        output="screen",
        additional_env={"GZ_SIM_RESOURCE_PATH": pkg},
    )

    gz_sim_headless = ExecuteProcess(
        condition=UnlessCondition(gz_gui),
        cmd=["gz", "sim", "-r", "-s", world],
        output="screen",
        additional_env={"GZ_SIM_RESOURCE_PATH": pkg},
    )

    # =========================================================================
    # 2.  robot_state_publisher  (t = 0 s)
    #     Reads URDF → broadcasts static TFs:
    #       base_footprint→base_link, base_link→laser, base_link→wheels
    # =========================================================================
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time":      True,
        }],
    )

    # =========================================================================
    # 3.  Spawn robot into Gazebo  (t = 5 s — Gazebo needs time to load world)
    #     Uses -topic to read URDF from robot_state_publisher's /robot_description
    #     topic. The create node handles URDF→SDF conversion internally.
    #     This is more reliable than -file for URDF in Gazebo Harmonic.
    # =========================================================================
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="[SIM] Spawning Leo Rover into Gazebo..."),
            Node(
                package="ros_gz_sim",
                executable="create",
                name="spawn_leo",
                output="screen",
                arguments=[
                    "-name",  "leo_rover",
                    "-topic", "robot_description",
                    "-x",     spawn_x,
                    "-y",     spawn_y,
                    "-z",     "0.1",
                    "-Y",     "0.0",
                ],
            ),
        ],
    )

    # =========================================================================
    # 4.  ros_gz_bridge  (t = 7 s — after robot spawned)
    #     Bridges: /scan, /odom, /cmd_vel, /tf, /clock, /joint_states
    # =========================================================================
    bridge_node = TimerAction(
        period=7.0,
        actions=[
            LogInfo(msg="[SIM] Starting ROS-Gazebo bridge..."),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge",
                output="screen",
                parameters=[{
                    "config_file":   bridge_cfg,
                    "use_sim_time":  True,
                }],
            ),
        ],
    )

    # =========================================================================
    # 4.5. robot_localization (EKF) (t = 9 s — after bridge has sensor data)
    #      Fuses /odom and /imu to output odom -> base_footprint TF
    # =========================================================================
    ekf_node = TimerAction(
        period=9.0,
        actions=[
            LogInfo(msg="[SIM] Starting robot_localization EKF..."),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_params, {"use_sim_time": True}],
                remappings=[("odometry/filtered", "odom_filtered")],
            ),
        ],
    )

    # =========================================================================
    # 5.  SLAM Toolbox  (t = 12 s — needs bridge to be forwarding /scan)
    # =========================================================================
    slam_launch = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg="[SIM] Starting SLAM Toolbox..."),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slm, "launch", "online_async_launch.py")
                ),
                launch_arguments={
                    "use_sim_time":     use_sim_time,
                    "slam_params_file": slam_params,
                }.items(),
            ),
        ],
    )

    # =========================================================================
    # 6.  RViz2  (t = 15 s)
    # =========================================================================
    rviz_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                condition=IfCondition(rviz),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
        ],
    )

    # =========================================================================
    # 7.  Nav2 — launched as individual nodes  (t = 22 s)
    #     We do NOT use nav2_bringup/navigation_launch.py because it:
    #       (a) remaps /tf → tf, /tf_static → tf_static (breaks our bridge)
    #       (b) launches 10 nodes in a chain; any failure blocks bt_navigator
    #     Instead we launch only the 4 essential nodes + lifecycle_manager.
    # =========================================================================
    nav2_nodes = TimerAction(
        period=22.0,
        actions=[
            LogInfo(msg="[SIM] Starting Nav2 (direct node launch)..."),

            # ── Controller Server (follows paths from planner) ──
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                output="screen",
                parameters=[nav2_params, {"use_sim_time": True}],
            ),

            # ── Planner Server (computes paths) ──
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[nav2_params, {"use_sim_time": True}],
            ),

            # ── Behavior Server (recovery behaviors: spin, backup, wait) ──
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[nav2_params, {"use_sim_time": True}],
            ),

            # ── BT Navigator (orchestrates navigation via behavior tree) ──
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                output="screen",
                parameters=[nav2_params, {"use_sim_time": True}],
            ),

            # ── Lifecycle Manager (transitions all nodes to ACTIVE) ──
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{
                    "use_sim_time":  True,
                    "autostart":     True,
                    "bond_timeout":  20.0,
                    "node_names": [
                        "controller_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                    ],
                }],
            ),
        ],
    )

    # =========================================================================
    # 8.  Frontier Explorer  (t = 40 s — needs Nav2 fully activated)
    # =========================================================================
    explorer_node = TimerAction(
        period=40.0,
        actions=[
            LogInfo(msg="[SIM] Starting Frontier Explorer..."),
            Node(
                package="leo_exploration",
                executable="frontier_explorer",
                name="frontier_explorer",
                output="screen",
                parameters=[{
                    "use_sim_time":          True,
                    "robot_frame":           "base_link",
                    "map_frame":             "map",
                    "min_frontier_size":     5,
                    "obstacle_dist":         0.45,
                    "scan_half_angle":       90.0,       # 180° front-only lidar
                    "nav_timeout":           45.0,
                    "init_forward_speed":    0.15,       # no spin, drive forward
                    "init_forward_duration": 3.0,
                    "backup_speed":         -0.18,
                    "backup_duration":       1.8,
                    "avoid_curve_speed":     0.10,       # gentle curve, no spin
                    "avoid_curve_angular":   0.5,
                    "avoid_curve_duration":  2.0,
                    "recov_forward_speed":   0.12,       # forward drive, no spin
                    "recov_forward_duration": 4.0,
                    "max_consec_fail":       4,
                    "costmap_clear_every":   3,
                    "complete_no_frontier":  8,
                    "log_interval":         10.0,
                    "save_map_on_complete":  True,
                    "map_save_path":        "/tmp/leo_sim_explored_map",
                }],
            ),
        ],
    )

    return LaunchDescription([
        gz_gui_arg,
        rviz_arg,
        world_arg,
        spawn_x_arg,
        spawn_y_arg,

        LogInfo(msg="\n"
                "╔══════════════════════════════════════════════════╗\n"
                "║  Leo Rover SIMULATION  v2.2  (Self-Contained)    ║\n"
                "║  No leo_gz package needed - own URDF + world     ║\n"
                "║  t=0  Gazebo  t=5  Spawn  t=7  Bridge            ║\n"
                "║  t=12 SLAM    t=22 Nav2   t=40 Explorer          ║\n"
                "╚══════════════════════════════════════════════════╝"),

        gz_sim_gui,
        gz_sim_headless,
        rsp_node,
        spawn_robot,
        bridge_node,
        ekf_node,
        slam_launch,
        rviz_node,
        nav2_nodes,
        explorer_node,
    ])

