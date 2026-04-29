#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_leo = get_package_share_directory("leo_exploration")
    pkg_nav2 = get_package_share_directory("nav2_bringup")

    nav2_params = os.path.join(pkg_leo, "config", "nav2_params_real.yaml")
    rviz_config = os.path.join(pkg_leo, "config", "rviz2_config.rviz")

    system_monitor_node = Node(
        package="leo_exploration",
        executable="system_monitor",
        name="system_monitor",
        output="screen",
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": nav2_params,
            "autostart": "true",
        }.items(),
    )

    explorer_node = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="[Step 3] Nav2 ready, starting frontier explorer..."),
            Node(
                package="leo_exploration",
                executable="frontier_explorer",
                name="frontier_explorer",
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                    "robot_frame": "base_link",
                    "map_frame": "map",
                    "cmd_vel_topic": "/cmd_vel",
                    "min_frontier_size": 5,
                    "obstacle_dist": 0.45,
                    "scan_half_angle": 60.0,
                    "safety_radius": 0.35,
                    "nav_timeout": 35.0,
                    "init_forward_speed": 0.15,
                    "init_forward_duration": 3.0,
                    "backup_speed": -0.18,
                    "backup_duration": 1.8,
                    "avoid_curve_speed": 0.10,
                    "avoid_curve_angular": 0.5,
                    "avoid_curve_duration": 2.0,
                    "recov_forward_speed": 0.12,
                    "recov_forward_duration": 4.0,
                    "max_consec_fail": 4,
                    "costmap_clear_every": 3,
                    "complete_no_frontier": 8,
                    "log_interval": 12.0,
                    "save_map_on_complete": True,
                    "map_save_path": "/tmp/leo_explored_map",
                }],
            )
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([
        LogInfo(msg="[Step 3] Launching Nav2, Explorer, and RViz..."),
        system_monitor_node,
        nav2_launch,
        explorer_node,
        rviz_node
    ])
