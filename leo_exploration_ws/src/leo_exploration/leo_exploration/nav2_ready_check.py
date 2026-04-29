#!/usr/bin/env python3
import sys
import time

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class Nav2ReadyCheck(Node):
    def __init__(self):
        super().__init__("nav2_ready_check")

        self.declare_parameter("action_name", "/navigate_to_pose")
        self.declare_parameter(
            "lifecycle_nodes",
            [
                "controller_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
            ],
        )
        self.declare_parameter("check_period", 1.0)

        self.action_name = self.get_parameter("action_name").value
        self.lifecycle_nodes = list(
            self.get_parameter("lifecycle_nodes").value
        )
        self.check_period = float(self.get_parameter("check_period").value)

        self.action_client = ActionClient(
            self,
            NavigateToPose,
            self.action_name,
        )
        self.state_clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in self.lifecycle_nodes
        }
        self.last_status = None

    def wait_until_ready(self):
        self.get_logger().info(
            f"Waiting for Nav2 lifecycle nodes and {self.action_name}..."
        )

        while rclpy.ok():
            ready_nodes, waiting_nodes = self.get_lifecycle_status()
            action_ready = self.action_client.wait_for_server(timeout_sec=0.1)

            status = (
                f"Nav2 active={ready_nodes or 'none'} | "
                f"waiting={waiting_nodes or 'none'} | "
                f"{self.action_name}={'ok' if action_ready else 'waiting'}"
            )
            if status != self.last_status:
                self.get_logger().info(status)
                self.last_status = status

            if not waiting_nodes and action_ready:
                self.get_logger().info(
                    "\033[92mNav2 Ready! Lifecycle nodes are active and "
                    "navigation action server is available.\033[0m"
                )
                return True

            time.sleep(self.check_period)

        return False

    def get_lifecycle_status(self):
        ready_nodes = []
        waiting_nodes = []

        for name, client in self.state_clients.items():
            if not client.wait_for_service(timeout_sec=0.1):
                waiting_nodes.append(f"{name}:service")
                continue

            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            if not future.done() or future.result() is None:
                waiting_nodes.append(f"{name}:state")
                continue

            current_state = future.result().current_state
            if current_state.id == State.PRIMARY_STATE_ACTIVE:
                ready_nodes.append(name)
            else:
                waiting_nodes.append(f"{name}:{current_state.label}")

        return ready_nodes, waiting_nodes


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ReadyCheck()
    exit_code = 0
    try:
        if not node.wait_until_ready():
            exit_code = 1
    except KeyboardInterrupt:
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
