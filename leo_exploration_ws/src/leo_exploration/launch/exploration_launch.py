#!/usr/bin/env python3
"""
Leo Rover Complete Exploration Launch
=====================================
Launches (in order with timed delays):
  1. RPLidar A2M12  ──────────────── t=0 s
  2. Static TF base_link → laser ─── t=0 s
  3. SLAM Toolbox (online_async) ──── t=4 s
  4. Nav2 navigation stack ─────────── t=10 s
  5. Frontier Explorer node ────────── t=18 s
  6. RViz2 ─────────────────────────── t=6 s

Usage:
  ros2 launch leo_exploration exploration_launch.py
  ros2 launch leo_exploration exploration_launch.py serial_port:=/dev/ttyUSB1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Package share directories ────────────────────────────────────────────
    pkg_leo  = get_package_share_directory("leo_exploration")
    pkg_slm  = get_package_share_directory("slam_toolbox")
    pkg_nav2 = get_package_share_directory("nav2_bringup")

    # ── Config paths ─────────────────────────────────────────────────────────
    nav2_params  = os.path.join(pkg_leo, "config", "nav2_params.yaml")
    slam_params  = os.path.join(pkg_leo, "config", "slam_toolbox_params.yaml")
    rviz_config  = os.path.join(pkg_leo, "config", "rviz2_config.rviz")
    ekf_params   = os.path.join(pkg_leo, "config", "ekf.yaml")

    # ── Launch arguments ─────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Use simulation clock")
    serial_port_arg = DeclareLaunchArgument(
        "serial_port", default_value="/dev/ttyUSB0",
        description="RPLidar USB port")
    laser_height_arg = DeclareLaunchArgument(
        "laser_height", default_value="0.12",
        description="Height of laser above base_link (metres)")

    use_sim_time  = LaunchConfiguration("use_sim_time")
    serial_port   = LaunchConfiguration("serial_port")
    laser_height  = LaunchConfiguration("laser_height")

    # =========================================================================
    # 1.  RPLidar A2M12 node  (immediate)
    # =========================================================================
    rplidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        parameters=[{
            "channel_type":    "serial",
            "serial_port":     serial_port,
            "serial_baudrate": 256000,
            "frame_id":        "laser",
            "inverted":        False,
            "angle_compensate": True,
            "scan_mode":       "Standard",
        }],
        output="screen",
    )

    # =========================================================================
    # 2.  Static TF:  base_link → laser  (immediate)
    #     x=0.0  y=0.0  z=laser_height  roll=0 pitch=0 yaw=0
    #     Adjust x/y if your lidar is not centred on the robot.
    # =========================================================================
    tf_base_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_laser",
        # Named-flag style — positional args deprecated in ROS2 Jazzy (Issue 4)
        arguments=[
            "--x",          "0.0",
            "--y",          "0.0",
            "--z",          laser_height,
            "--yaw",        "0.0",
            "--pitch",      "0.0",
            "--roll",       "0.0",
            "--frame-id",   "base_link",
            "--child-frame-id", "laser",
        ],
        output="screen",
    )

    # =========================================================================
    # 2.5. EKF Node (IMU + Odom fusion)  (immediate)
    #      Assumes real robot base publishes /odom and /imu
    # =========================================================================
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params, {"use_sim_time": use_sim_time}],
        remappings=[("odometry/filtered", "odom_filtered")],
    )

    # =========================================================================
    # 3.  SLAM Toolbox  (delay 4 s to let lidar start publishing /scan)
    # =========================================================================
    slam_launch = TimerAction(
        period=4.0,
        actions=[
            LogInfo(msg="[SLAM] Starting slam_toolbox online_async…"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_slm, "launch", "online_async_launch.py")
                ),
                launch_arguments={
                    "use_sim_time":      use_sim_time,
                    "slam_params_file":  slam_params,
                }.items(),
            ),
        ],
    )

    # =========================================================================
    # 4.  Nav2 navigation stack  (delay 10 s to let SLAM initialise TF tree)
    # =========================================================================
    nav2_launch = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg="[Nav2] Starting navigation stack…"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2, "launch", "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time":  use_sim_time,
                    "params_file":   nav2_params,
                    "autostart":     "true",
                }.items(),
            ),
        ],
    )

    # =========================================================================
    # 5.  Frontier Explorer  (delay 18 s to let Nav2 fully initialise)
    # =========================================================================
    explorer_node = TimerAction(
        period=18.0,
        actions=[
            LogInfo(msg="[Explorer] Starting frontier exploration…"),
            Node(
                package="leo_exploration",
                executable="frontier_explorer",
                name="frontier_explorer",
                output="screen",
                parameters=[{
                    "use_sim_time":           use_sim_time,
                    "robot_frame":            "base_link",
                    "map_frame":              "map",
                    "min_frontier_size":      5,
                    "obstacle_dist":          0.45,
                    "scan_half_angle":        90.0,      # 180° front-only lidar
                    "nav_timeout":            35.0,
                    "init_forward_speed":     0.15,      # no spin, drive forward
                    "init_forward_duration":  3.0,
                    "backup_speed":          -0.18,
                    "backup_duration":        1.8,
                    "avoid_curve_speed":      0.10,      # gentle curve, no spin
                    "avoid_curve_angular":    0.5,
                    "avoid_curve_duration":   2.0,
                    "recov_forward_speed":    0.12,      # forward drive, no spin
                    "recov_forward_duration": 4.0,
                    "max_consec_fail":        4,
                    "costmap_clear_every":    3,
                    "complete_no_frontier":   8,
                    "log_interval":          12.0,
                    "save_map_on_complete":   True,
                    "map_save_path":          "/tmp/leo_explored_map",
                }],
            ),
        ],
    )

    # =========================================================================
    # 6.  RViz2  (delay 6 s)
    # =========================================================================
    rviz_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        serial_port_arg,
        laser_height_arg,
        LogInfo(msg="╔══ Leo Rover Exploration System Starting ══╗"),
        rplidar_node,
        tf_base_laser,
        ekf_node,
        slam_launch,
        nav2_launch,
        explorer_node,
        rviz_node,
    ])
