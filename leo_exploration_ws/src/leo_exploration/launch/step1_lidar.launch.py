#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0")
    laser_height_arg = DeclareLaunchArgument("laser_height", default_value="0.12")
    laser_x_arg = DeclareLaunchArgument("laser_x", default_value="0.1325")
    laser_y_arg = DeclareLaunchArgument("laser_y", default_value="0.0")
    laser_yaw_arg = DeclareLaunchArgument("laser_yaw", default_value="0.541052")

    serial_port = LaunchConfiguration("serial_port")
    laser_height = LaunchConfiguration("laser_height")
    laser_x = LaunchConfiguration("laser_x")
    laser_y = LaunchConfiguration("laser_y")
    laser_yaw = LaunchConfiguration("laser_yaw")

    rplidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
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

    tf_base_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_laser",
        arguments=[
            "--x", laser_x, "--y", laser_y, "--z", laser_height,
            "--yaw", laser_yaw, "--pitch", "0.0", "--roll", "0.0",
            "--frame-id", "base_link", "--child-frame-id", "laser",
        ],
        output="screen",
    )

    # NOTE: odom -> base_link is published by the Leo Rover base driver
    # (leo_bringup). Do NOT add a static fallback here — it conflicts
    # with the dynamic TF and causes SLAM to see a frozen pose.
    return LaunchDescription([
        serial_port_arg,
        laser_height_arg,
        laser_x_arg,
        laser_y_arg,
        laser_yaw_arg,
        LogInfo(msg="[Step 1] Launching Lidar and laser TF..."),
        rplidar_node,
        tf_base_laser,
    ])
