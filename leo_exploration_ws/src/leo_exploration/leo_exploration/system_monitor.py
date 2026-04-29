#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import psutil

class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__('system_monitor')
        self.timer = self.create_timer(5.0, self.monitor_system)
        self.get_logger().info("System Monitor started.")

    def monitor_system(self):
        cpu_percent = psutil.cpu_percent(interval=None)
        mem = psutil.virtual_memory()
        disk = psutil.disk_usage('/')

        if cpu_percent > 90.0:
            self.get_logger().warn(f"High CPU Usage! {cpu_percent}%")
            
        if mem.percent > 90.0:
            self.get_logger().warn(f"High Memory Usage! {mem.percent}%")
            
        if disk.percent > 90.0:
            self.get_logger().warn(f"Low Disk Space! {disk.percent}% used")

def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
