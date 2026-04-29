#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0")
    laser_height_arg = DeclareLaunchArgument("laser_height", default_value="0.12")
    
    serial_port = LaunchConfiguration("serial_port")
    laser_height = LaunchConfiguration("laser_height")

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
            "--x", "0.0", "--y", "0.0", "--z", laser_height,
            "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
            "--frame-id", "base_link", "--child-frame-id", "laser",
        ],
        output="screen",
    )

    # NOTE: odom -> base_link is published by the Leo Rover base driver
    # (leo_bringup). Do NOT add a static fallback here — it conflicts
    # with the dynamic TF and causes SLAM to see a frozen pose.
    #请先阅读我所有代码，主要在/leo_exploration_ws下面,现在我尝试打开我项目的rviz的地图卡死,我用手移动雷达，地图要过很久才会动,我现在是把车子架起来悬空的，请你远程操作机器人 Pi，通过指令 ssh pi@192.168.8.2 ，Leo Rover 默认密码是：
    #raspberry， 请你自己在终端测试整个流程直到他可以完整运行.
    return LaunchDescription([
        serial_port_arg,
        laser_height_arg,
        LogInfo(msg="[Step 1] Launching Lidar and laser TF..."),
        rplidar_node,
        tf_base_laser,
    ])
