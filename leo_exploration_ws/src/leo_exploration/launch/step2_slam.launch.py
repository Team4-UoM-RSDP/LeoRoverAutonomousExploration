#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_leo = get_package_share_directory("leo_exploration")
    pkg_slm = get_package_share_directory("slam_toolbox")
    slam_params = os.path.join(pkg_leo, "config", "slam_toolbox_real.yaml")

    return LaunchDescription([
        LogInfo(msg="[Step 2] Launching SLAM Toolbox..."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_slm, "launch", "online_async_launch.py")
            ),
            launch_arguments={
                "use_sim_time": "false",
                "slam_params_file": slam_params,
            }.items(),
        )
    ])
