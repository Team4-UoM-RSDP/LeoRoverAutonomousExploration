#!/usr/bin/env python3
"""
Front-Facing Lidar Filter Node

Subscribes to `/scan_raw` (full 360-degree lidar scan) and publishes to `/scan`
with only the front 120-degree cone (from -60 to +60 degrees). Values outside
this cone are set to infinity. This prevents the robot from attempting to
navigate based on obstacles or noise behind it, producing a smoother path.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy


class LidarFilter(Node):
    def __init__(self):
        super().__init__("lidar_filter")
        
        # QoS matching typical sensor configurations
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        self.sub = self.create_subscription(
            LaserScan,
            "/scan_raw",
            self.scan_callback,
            qos_profile
        )
        
        self.pub = self.create_publisher(
            LaserScan,
            "/scan",
            qos_profile
        )
        
        # 120 degrees total (60 on each side)
        self.max_half_angle = math.radians(60.0)
        
        self.get_logger().info("Lidar filter started: 120-degree front-facing cone.")

    def scan_callback(self, msg: LaserScan):
        # Create a copy of the incoming message to avoid modifying in-place
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_msg.intensities = msg.intensities
        
        # Ranges filtering
        ranges = np.array(msg.ranges, dtype=np.float32)
        
        # Calculate angle for each index
        num_readings = len(ranges)
        angles = msg.angle_min + np.arange(num_readings) * msg.angle_increment
        
        # Wrap angles to [-pi, pi]
        angles = (angles + np.pi) % (2 * np.pi) - np.pi
        
        # Create mask for outside the allowed cone
        mask_outside = np.abs(angles) > self.max_half_angle
        
        # Set ranges outside cone to infinity
        ranges[mask_outside] = float('inf')
        
        filtered_msg.ranges = ranges.tolist()
        
        self.pub.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
