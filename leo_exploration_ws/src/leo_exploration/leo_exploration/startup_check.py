#!/usr/bin/env python3
import sys

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class StartupCheckNode(Node):
    def __init__(self):
        super().__init__("startup_check")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("topic_type", "scan")
        self.declare_parameter("require_topic", True)
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("source_frame", "laser")

        self.scan_topic = self.get_parameter("scan_topic").value
        self.map_topic = self.get_parameter("map_topic").value
        self.topic_type = self.get_parameter("topic_type").value
        self.require_topic = self.get_parameter("require_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.source_frame = self.get_parameter("source_frame").value

        self.topic_name = self.get_topic_name()
        self.topic_received = not self.require_topic
        self.last_status = None

        if self.require_topic:
            self.create_topic_subscription()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.check_status)
        self.get_logger().info(
            f"Waiting for {self.topic_name} and "
            f"{self.target_frame} -> {self.source_frame} TF..."
        )

    def get_topic_name(self):
        if self.topic_type == "scan":
            return self.scan_topic
        if self.topic_type == "map":
            return self.map_topic
        if self.topic_type == "none":
            return "(topic disabled)"
        raise ValueError(f"Unsupported topic_type: {self.topic_type}")

    def create_topic_subscription(self):
        if self.topic_type == "scan":
            self.create_subscription(
                LaserScan,
                self.scan_topic,
                self.topic_callback,
                qos_profile_sensor_data,
            )
            return

        if self.topic_type == "map":
            map_qos = QoSProfile(depth=1)
            map_qos.reliability = QoSReliabilityPolicy.RELIABLE
            map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(
                OccupancyGrid,
                self.map_topic,
                self.topic_callback,
                map_qos,
            )
            return

        if self.topic_type != "none":
            raise ValueError(f"Unsupported topic_type: {self.topic_type}")

    def topic_callback(self, _msg):
        self.topic_received = True

    def check_status(self):
        topic_ready = self.topic_received
        tf_ready = False
        try:
            self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                rclpy.time.Time(),
            )
            tf_ready = True
        except TransformException:
            pass

        status = (
            f"{self.topic_name}={'ok' if topic_ready else 'waiting'} | "
            f"TF {self.target_frame}->{self.source_frame}="
            f"{'ok' if tf_ready else 'waiting'}"
        )
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status

        if topic_ready and tf_ready:
            self.get_logger().info(
                "\033[92mSystem Ready! Topic and TF chain are active.\033[0m"
            )
            sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = StartupCheckNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
