#!/usr/bin/env python3
"""Spawn robots via the SpawnEntity service.

Usage:
    python3 scripts/spawn_robots.py --name arm_0 --urdf robots/arm_robot.urdf --x 1.0 --y 2.0
    python3 scripts/spawn_robots.py --name mobile_5 --x 5.0
"""

import argparse

import rclpy
from rclpy.node import Node
from simulation_interfaces.srv import SpawnEntity


def main():
    parser = argparse.ArgumentParser(description="Spawn a robot via SpawnEntity service")
    parser.add_argument("--name", required=True, help="Entity name")
    parser.add_argument("--urdf", default="robots/mobile_robot.urdf", help="URDF path")
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.05)
    args = parser.parse_args()

    rclpy.init()
    node = Node("spawn_robots_client")
    client = node.create_client(SpawnEntity, "/sim/spawn_entity")

    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("SpawnEntity service not available")
        node.destroy_node()
        rclpy.shutdown()
        return

    request = SpawnEntity.Request()
    request.name = args.name
    request.entity_resource.uri = args.urdf
    request.initial_pose.pose.position.x = args.x
    request.initial_pose.pose.position.y = args.y
    request.initial_pose.pose.position.z = args.z
    request.initial_pose.pose.orientation.w = 1.0

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    if result.result.result == 1:  # RESULT_OK
        node.get_logger().info(f"Spawned: {result.entity_name}")
    else:
        node.get_logger().error(f"Failed: {result.result.error_message}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
