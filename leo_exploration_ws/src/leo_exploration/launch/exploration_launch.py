#!/usr/bin/env python3
"""
Leo Rover real-robot exploration launch for a split deployment:
  - Raspberry Pi runs the LeoOS base stack (odom/imu/base TF/cmd_vel)
  - This development machine runs the USB RPLidar, SLAM, Nav2, explorer, and RViz

Bring-up order:
  1. Local RPLidar and base_link -> laser TF
  2. Startup readiness check waits for /scan and odom -> laser
  3. SLAM Toolbox starts once the combined TF chain is ready
  4. Map readiness check waits for /map and map -> base_link
  5. Nav2 starts after the map TF chain is ready
  6. RViz starts on the development machine
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    pkg_leo = get_package_share_directory("leo_exploration")
    pkg_slam = get_package_share_directory("slam_toolbox")
    slam_params = os.path.join(pkg_leo, "config", "slam_toolbox_real.yaml")
    nav2_params = os.path.join(pkg_leo, "config", "nav2_params_real.yaml")
    rviz_config = os.path.join(pkg_leo, "config", "rviz2_config.rviz")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock.",
    )
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Local RPLidar USB serial device.",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz on this machine.",
    )
    explorer_arg = DeclareLaunchArgument(
        "explorer",
        default_value="true",
        description="Launch autonomous frontier exploration after Nav2 is active.",
    )
    launch_lidar_arg = DeclareLaunchArgument(
        "launch_lidar",
        default_value="true",
        description="Launch the local RPLidar driver on this machine.",
    )
    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="/scan",
        description="LaserScan topic consumed by SLAM/Nav2/explorer.",
    )
    laser_frame_arg = DeclareLaunchArgument(
        "laser_frame",
        default_value="laser",
        description="Child frame published for the locally attached lidar.",
    )
    laser_height_arg = DeclareLaunchArgument(
        "laser_height",
        default_value="0.12",
        description="Height of the lidar above base_link in metres.",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    serial_port = LaunchConfiguration("serial_port")
    rviz = LaunchConfiguration("rviz")
    explorer = LaunchConfiguration("explorer")
    launch_lidar = LaunchConfiguration("launch_lidar")
    scan_topic = LaunchConfiguration("scan_topic")
    laser_frame = LaunchConfiguration("laser_frame")
    laser_height = LaunchConfiguration("laser_height")

    # VS Code installed as a snap can leak snap runtime variables into ROS launch.
    # RViz then tries to load glibc pieces from /snap/core20 and exits before opening.
    rviz_clean_env_command = [
        "unset SNAP SNAP_ARCH SNAP_COMMON SNAP_CONTEXT SNAP_COOKIE SNAP_DATA "
        "SNAP_EUID SNAP_INSTANCE_NAME SNAP_LIBRARY_PATH SNAP_NAME SNAP_REAL_HOME "
        "SNAP_REVISION SNAP_UID SNAP_USER_COMMON SNAP_USER_DATA SNAP_VERSION "
        "GTK_PATH LOCPATH; "
        "export XDG_DATA_HOME=\"$HOME/.local/share\"; "
        "export XDG_DATA_DIRS=\"${XDG_DATA_DIRS_VSCODE_SNAP_ORIG:-"
        "/usr/share/ubuntu:/usr/share/gnome:/usr/local/share:/usr/share:"
        "/var/lib/snapd/desktop}\"; "
        "exec rviz2 -d ",
        rviz_config,
        " --ros-args -r __node:=rviz2 -r scan:=",
        scan_topic,
        " -r /scan:=",
        scan_topic,
    ]

    system_monitor_node = Node(
        package="leo_exploration",
        executable="system_monitor",
        name="system_monitor",
        output="screen",
    )

    odometry_tf_bridge_node = Node(
        package="leo_exploration",
        executable="odometry_tf_bridge",
        name="odometry_tf_bridge",
        output="screen",
        parameters=[{
            "odom_topic": "/merged_odom",
            "fallback_parent_frame": "odom",
            "fallback_child_frame": "base_footprint",
            "publish_initial_transform": False,
            "timestamp_offset_sec": 0.0,
        }],
    )

    rplidar_node = Node(
        condition=IfCondition(launch_lidar),
        package="rplidar_ros",
        executable="rplidar_composition",
        name="rplidar_node",
        output="screen",
        parameters=[{
            "channel_type": "serial",
            "serial_port": serial_port,
            "serial_baudrate": 256000,
            "frame_id": laser_frame,
            "inverted": False,
            "angle_compensate": True,
            "scan_mode": "Standard",
        }],
        remappings=[
            ("scan", scan_topic),
            ("/scan", scan_topic),
        ],
    )

    tf_base_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_laser",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", laser_height,
            "--yaw", "0.0",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", "base_link",
            "--child-frame-id", laser_frame,
        ],
    )

    tf_basefootprint_baselink = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_basefootprint_baselink",
        output="screen",
        arguments=[
            "--x", "0.0",
            "--y", "0.0",
            "--z", "0.0",
            "--yaw", "0.0",
            "--pitch", "0.0",
            "--roll", "0.0",
            "--frame-id", "base_footprint",
            "--child-frame-id", "base_link",
        ],
    )

    startup_check_node = Node(
        package="leo_exploration",
        executable="startup_check",
        name="startup_check",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "scan_topic": scan_topic,
            "target_frame": "odom",
            "source_frame": laser_frame,
        }],
    )

    map_tf_check_node = Node(
        package="leo_exploration",
        executable="startup_check",
        name="map_tf_check",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "topic_type": "map",
            "map_topic": "/map",
            "target_frame": "map",
            "source_frame": "base_link",
        }],
    )

    slam_launch = GroupAction([
        SetRemap(src="scan", dst=scan_topic),
        SetRemap(src="/scan", dst=scan_topic),
        LogInfo(msg="[SLAM] Startup checks passed. Launching slam_toolbox..."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_slam, "launch", "online_async_launch.py")
            ),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "slam_params_file": slam_params,
            }.items(),
        ),
    ])

    nav2_nodes_launch = GroupAction([
        SetRemap(src="scan", dst=scan_topic),
        SetRemap(src="/scan", dst=scan_topic),
        LogInfo(msg="[Nav2] Starting core nodes early so TF buffers can warm up..."),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
        ),
    ])

    nav2_lifecycle_launch = GroupAction([
        LogInfo(msg="[Nav2] Map TF is ready. Activating lifecycle nodes..."),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "bond_timeout": 120.0,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                ],
            }],
        ),
    ])

    nav2_ready_check_node = Node(
        package="leo_exploration",
        executable="nav2_ready_check",
        name="nav2_ready_check",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "action_name": "/navigate_to_pose",
            "lifecycle_nodes": [
                "controller_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
            ],
        }],
    )

    explorer_launch = GroupAction(
        condition=IfCondition(explorer),
        actions=[
            SetRemap(src="scan", dst=scan_topic),
            SetRemap(src="/scan", dst=scan_topic),
            LogInfo(msg="[Explorer] Nav2 is active. Launching frontier explorer..."),
            Node(
                package="leo_exploration",
                executable="frontier_explorer",
                name="frontier_explorer",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
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
            ),
        ],
    )

    rviz_launch = GroupAction(
        condition=IfCondition(rviz),
        actions=[
            ExecuteProcess(
                cmd=["bash", "-lc", rviz_clean_env_command],
                name="rviz2",
                output="screen",
            ),
        ],
    )


    startup_ready_chain = RegisterEventHandler(
        OnProcessExit(
            target_action=startup_check_node,
            on_exit=[
                slam_launch,
                nav2_nodes_launch,
                map_tf_check_node,
            ],
        )
    )

    map_ready_chain = RegisterEventHandler(
        OnProcessExit(
            target_action=map_tf_check_node,
            on_exit=[
                nav2_lifecycle_launch,
                nav2_ready_check_node,
                rviz_launch,
            ],
        )
    )

    nav2_ready_chain = RegisterEventHandler(
        OnProcessExit(
            target_action=nav2_ready_check_node,
            on_exit=[
                explorer_launch,
            ],
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        serial_port_arg,
        rviz_arg,
        explorer_arg,
        launch_lidar_arg,
        scan_topic_arg,
        laser_frame_arg,
        laser_height_arg,
        LogInfo(msg="[RealRobot] Split deployment: Pi base stack + local lidar/SLAM/Nav2/RViz"),
        system_monitor_node,
        odometry_tf_bridge_node,
        rplidar_node,
        tf_basefootprint_baselink,
        tf_base_laser,
        startup_check_node,
        startup_ready_chain,
        map_ready_chain,
        nav2_ready_chain,
    ])
