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
import re
import subprocess
from xml.sax.saxutils import escape

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def _route_to_peer(peer_ip):
    """Return the local interface and IP the OS would use to reach peer_ip."""
    try:
        output = subprocess.check_output(
            ["ip", "-4", "route", "get", peer_ip],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=2.0,
        )
    except Exception:
        return "auto", "auto"

    interface = "auto"
    local_ip = "auto"
    dev_match = re.search(r"\bdev\s+(\S+)", output)
    src_match = re.search(r"\bsrc\s+(\S+)", output)
    if dev_match:
        interface = dev_match.group(1)
    if src_match:
        local_ip = src_match.group(1)
    return interface, local_ip


def _configure_dds(context):
    pi_ip = LaunchConfiguration("pi_ip").perform(context)
    local_ip = LaunchConfiguration("local_ip").perform(context)
    network_interface = LaunchConfiguration("network_interface").perform(context)
    ros_domain_id = LaunchConfiguration("ros_domain_id").perform(context)

    route_interface, route_ip = _route_to_peer(pi_ip)
    if local_ip == "auto":
        local_ip = route_ip
    if network_interface == "auto":
        network_interface = route_interface

    if local_ip == "auto" or network_interface == "auto":
        raise RuntimeError(
            "Could not auto-detect local_ip/network_interface. "
            "Pass local_ip:=<this_machine_ip> network_interface:=<wifi_iface>."
        )

    cyclonedds_config = f"""<?xml version=\"1.0\" encoding=\"UTF-8\"?>
<CycloneDDS xmlns=\"https://cdds.io/config\">
  <Domain Id=\"any\">
    <General>
      <Interfaces>
        <NetworkInterface name=\"{escape(network_interface)}\" priority=\"default\" multicast=\"default\" />
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
      <Peers>
        <Peer Address=\"{escape(pi_ip)}\" />
        <Peer Address=\"{escape(local_ip)}\" />
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
"""
    config_path = f"/tmp/leo_cyclonedds_{ros_domain_id}_{local_ip.replace('.', '_')}_{pi_ip.replace('.', '_')}.xml"
    with open(config_path, "w", encoding="utf-8") as config_file:
        config_file.write(cyclonedds_config)

    return [
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
        SetEnvironmentVariable("ROS_DOMAIN_ID", ros_domain_id),
        SetEnvironmentVariable("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET"),
        SetEnvironmentVariable("ROS_IP", local_ip),
        SetEnvironmentVariable("CYCLONEDDS_URI", "file://" + config_path),
        UnsetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE"),
        UnsetEnvironmentVariable("ROS_DISCOVERY_SERVER"),
        LogInfo(
            msg=(
                f"[DDS] CycloneDDS peers: local {local_ip} on {network_interface}, "
                f"Pi {pi_ip}, domain {ros_domain_id}"
            )
        ),
    ]


def generate_launch_description():
    pkg_leo = get_package_share_directory("leo_exploration")
    pkg_slam = get_package_share_directory("slam_toolbox")
    slam_params = os.path.join(pkg_leo, "config", "slam_toolbox_real.yaml")
    nav2_params = os.path.join(pkg_leo, "config", "nav2_params_real.yaml")
    rviz_config = os.path.join(pkg_leo, "config", "rviz2_config.rviz")

    ros_domain_id_arg = DeclareLaunchArgument(
        "ros_domain_id",
        default_value="42",
        description="ROS_DOMAIN_ID shared with the Leo Rover Pi.",
    )
    pi_ip_arg = DeclareLaunchArgument(
        "pi_ip",
        default_value="192.168.8.2",
        description="Leo Rover Pi WiFi IP address.",
    )
    local_ip_arg = DeclareLaunchArgument(
        "local_ip",
        default_value="auto",
        description="This machine's WiFi IP. Use auto to infer from pi_ip route.",
    )
    network_interface_arg = DeclareLaunchArgument(
        "network_interface",
        default_value="auto",
        description="This machine's WiFi interface. Use auto to infer from pi_ip route.",
    )
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
    odom_tf_timestamp_offset_arg = DeclareLaunchArgument(
        "odom_tf_timestamp_offset",
        default_value="0.10",
        description=(
            "Seconds added to locally republished odom TF stamps to absorb "
            "WiFi clock jitter between the Pi and this machine."
        ),
    )
    cmd_vel_out_arg = DeclareLaunchArgument(
        "cmd_vel_out_topic",
        default_value="/cmd_vel_relay",
        description=(
            "Final velocity topic after smoothing/collision monitoring. "
            "The Pi relay forwards /cmd_vel_relay to the firmware /cmd_vel. "
            "Use /cmd_vel_debug for suspended dry-run tests."
        ),
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    serial_port = LaunchConfiguration("serial_port")
    rviz = LaunchConfiguration("rviz")
    explorer = LaunchConfiguration("explorer")
    launch_lidar = LaunchConfiguration("launch_lidar")
    scan_topic = LaunchConfiguration("scan_topic")
    laser_frame = LaunchConfiguration("laser_frame")
    laser_height = LaunchConfiguration("laser_height")
    odom_tf_timestamp_offset = LaunchConfiguration("odom_tf_timestamp_offset")
    cmd_vel_out_topic = LaunchConfiguration("cmd_vel_out_topic")

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
        "export LIBGL_ALWAYS_SOFTWARE=1; "
        "export QT_OPENGL=software; "
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
            "timestamp_offset_sec": odom_tf_timestamp_offset,
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
                "autostart": "false",
                "use_lifecycle_manager": "true",
            }.items(),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_slam",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "bond_timeout": 120.0,
                "node_names": ["slam_toolbox"],
            }],
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
            remappings=[("cmd_vel", "cmd_vel_nav")],
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
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            output="screen",
            parameters=[
                nav2_params,
                {
                    "use_sim_time": use_sim_time,
                    "cmd_vel_out_topic": cmd_vel_out_topic,
                },
            ],
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
                    "velocity_smoother",
                    "collision_monitor",
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
                "velocity_smoother",
                "collision_monitor",
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
                    "cmd_vel_topic": "/cmd_vel_nav",
                    "min_frontier_size": 5,
                    "obstacle_dist": 0.45,
                    "scan_half_angle": 60.0,
                    "safety_radius": 0.35,
                    "scan_timeout": 0.7,
                    "front_min_points": 4,
                    "safety_min_points": 3,
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
        odom_tf_timestamp_offset_arg,
        cmd_vel_out_arg,
        ros_domain_id_arg,
        pi_ip_arg,
        local_ip_arg,
        network_interface_arg,
        OpaqueFunction(function=_configure_dds),
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
