#!/usr/bin/env python3
"""Send batched navigation goals through the fleet-level ROS API.

The script reads ``/fleet/states`` to discover robot names and current
positions, then sends one ``/fleet/navigate`` request containing goals for the
selected robots.
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from pybullet_fleet_msgs.msg import FleetNavigate as FleetNavigateMsg
from pybullet_fleet_msgs.msg import FleetState, RobotGoal2D
from pybullet_fleet_msgs.srv import FleetNavigate
from rclpy.node import Node


class FleetNavGoalClient(Node):
    def __init__(self) -> None:
        super().__init__("send_fleet_nav_goals")
        self.positions: dict[str, tuple[float, float, float]] = {}
        self.create_subscription(FleetState, "/fleet/states", self._on_state, 10)
        self.navigate = self.create_client(FleetNavigate, "/fleet/navigate")
        self.navigate_pub = self.create_publisher(FleetNavigateMsg, "/fleet/navigate", 10)

    def _on_state(self, msg: FleetState) -> None:
        for robot in msg.robots:
            self.positions[robot.name] = (
                float(robot.pose.position.x),
                float(robot.pose.position.y),
                float(robot.pose.position.z),
            )

    def wait_for_states(self, count: int, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.positions) >= count:
                return
        raise TimeoutError(f"/fleet/states reported {len(self.positions)} robot(s), expected at least {count}")

    def call_navigate(self, goals: list[RobotGoal2D], timeout: float) -> None:
        if not self.navigate.wait_for_service(timeout_sec=timeout):
            raise TimeoutError("/fleet/navigate service is not available")

        request = FleetNavigate.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = "odom"
        request.command_id = "fleet-nav-demo"
        request.source = "send_fleet_nav_goals.py"
        request.goals_2d = goals

        future = self.navigate.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError("/fleet/navigate service call timed out")
        response = future.result()
        if response is None:
            raise RuntimeError("/fleet/navigate service call failed")
        ack = response.ack
        accepted = ", ".join(ack.accepted_names)
        if ack.rejected_names:
            rejected = ", ".join(f"{name}:{reason}" for name, reason in zip(ack.rejected_names, ack.reject_reasons))
            raise RuntimeError(f"/fleet/navigate rejected command: accepted=[{accepted}], rejected=[{rejected}]")
        self.get_logger().info(f"/fleet/navigate accepted {len(ack.accepted_names)} robot(s): {accepted}")

    def publish_navigate(self, goals: list[RobotGoal2D], timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and self.navigate_pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.navigate_pub.get_subscription_count() == 0:
            raise TimeoutError("/fleet/navigate topic has no subscribers")

        msg = FleetNavigateMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.command_id = "fleet-nav-demo-topic"
        msg.source = "send_fleet_nav_goals.py"
        msg.goals_2d = goals
        self.navigate_pub.publish(msg)
        self.get_logger().info(f"published /fleet/navigate topic command for {len(goals)} robot(s)")


def _make_goals(
    positions: dict[str, tuple[float, float, float]],
    *,
    count: int,
    dx: float,
    dy: float,
    yaw: float,
) -> list[RobotGoal2D]:
    names = sorted(positions)[:count]
    goals = []
    for name in names:
        x, y, z = positions[name]
        goals.append(RobotGoal2D(name=name, position=[x + dx, y + dy], yaw=yaw, z=z))
    return goals


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robots", type=int, default=20, help="Number of robots to command")
    parser.add_argument("--dx", type=float, default=0.5, help="Goal x offset from each current robot pose")
    parser.add_argument("--dy", type=float, default=0.0, help="Goal y offset from each current robot pose")
    parser.add_argument("--yaw", type=float, default=0.0, help="Goal yaw in radians")
    parser.add_argument("--timeout", type=float, default=10.0, help="ROS wait/call timeout in seconds")
    parser.add_argument(
        "--transport",
        choices=["service", "topic"],
        default="service",
        help="Use /fleet/navigate service with ack, or topic fire-and-forget",
    )
    args = parser.parse_args()
    if args.robots <= 0:
        parser.error("--robots must be positive")
    if not math.isfinite(args.dx) or not math.isfinite(args.dy) or not math.isfinite(args.yaw):
        parser.error("--dx, --dy, and --yaw must be finite")

    rclpy.init()
    node = FleetNavGoalClient()
    try:
        node.wait_for_states(args.robots, args.timeout)
        goals = _make_goals(node.positions, count=args.robots, dx=args.dx, dy=args.dy, yaw=args.yaw)
        if args.transport == "service":
            node.call_navigate(goals, args.timeout)
        else:
            node.publish_navigate(goals, args.timeout)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
