#!/usr/bin/env python3
from copy import deepcopy

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster


class OdometryTfBridge(Node):
    def __init__(self):
        super().__init__("odometry_tf_bridge")

        self.declare_parameter("odom_topic", "/merged_odom")
        self.declare_parameter("fallback_parent_frame", "odom")
        self.declare_parameter("fallback_child_frame", "base_footprint")
        self.declare_parameter("use_local_timestamp", True)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("publish_initial_transform", True)
        self.declare_parameter("timestamp_offset_sec", 0.10)

        self.odom_topic = self.get_parameter("odom_topic").value
        self.fallback_parent_frame = (
            self.get_parameter("fallback_parent_frame").value
        )
        self.fallback_child_frame = (
            self.get_parameter("fallback_child_frame").value
        )
        self.use_local_timestamp = (
            self.get_parameter("use_local_timestamp").value
        )
        self.publish_rate_hz = (
            self.get_parameter("publish_rate_hz").value
        )
        self.publish_initial_transform = (
            self.get_parameter("publish_initial_transform").value
        )
        self.timestamp_offset_sec = float(
            self.get_parameter("timestamp_offset_sec").value
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.received_odom = False
        self.latest_transform = (
            self.create_identity_transform()
            if self.publish_initial_transform
            else None
        )

        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )
        self.publish_timer = self.create_timer(
            1.0 / max(float(self.publish_rate_hz), 1.0),
            self.publish_latest_transform,
        )

        self.get_logger().info(
            f"Bridging {self.odom_topic} into TF "
            f"({self.fallback_parent_frame} -> {self.fallback_child_frame})"
        )
        if self.publish_initial_transform:
            self.get_logger().info(
                "Publishing initial identity TF until odometry arrives"
            )

    def create_identity_transform(self):
        transform = TransformStamped()
        transform.header.frame_id = self.fallback_parent_frame
        transform.child_frame_id = self.fallback_child_frame
        transform.transform.rotation.w = 1.0
        return transform

    def odom_callback(self, msg: Odometry):
        transform = TransformStamped()
        if self.use_local_timestamp:
            transform.header.stamp = self.get_local_stamp()
        else:
            transform.header.stamp = msg.header.stamp
        transform.header.frame_id = (
            msg.header.frame_id or self.fallback_parent_frame
        )
        transform.child_frame_id = (
            msg.child_frame_id or self.fallback_child_frame
        )
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation

        self.latest_transform = transform
        self.publish_latest_transform()

        if not self.received_odom:
            self.received_odom = True
            self.get_logger().info(
                f"Received odometry and published TF "
                f"{transform.header.frame_id} -> {transform.child_frame_id}"
            )

    def publish_latest_transform(self):
        if self.latest_transform is None:
            return

        transform = deepcopy(self.latest_transform)
        if self.use_local_timestamp:
            transform.header.stamp = self.get_local_stamp()

        self.tf_broadcaster.sendTransform(transform)

    def get_local_stamp(self):
        now_nanoseconds = self.get_clock().now().nanoseconds
        offset_nanoseconds = int(self.timestamp_offset_sec * 1e9)
        return Time(nanoseconds=now_nanoseconds + offset_nanoseconds).to_msg()


def main(args=None):
    rclpy.init(args=args)
    node = OdometryTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
