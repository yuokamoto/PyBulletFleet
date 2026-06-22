#!/usr/bin/env python3
"""Query entities and their states via simulation_interfaces services.

Usage:
    python3 scripts/query_entities.py                     # List all entities
    python3 scripts/query_entities.py --name robot_0      # Get specific entity state
"""

import argparse

import rclpy
from rclpy.node import Node
from simulation_interfaces.srv import GetEntities, GetEntityState


def main():
    parser = argparse.ArgumentParser(description="Query simulation entities")
    parser.add_argument("--name", default=None, help="Get state of specific entity")
    args = parser.parse_args()

    rclpy.init()
    node = Node("query_entities_client")

    if args.name:
        client = node.create_client(GetEntityState, "/sim/get_entity_state")
        if not client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error("GetEntityState service not available")
            node.destroy_node()
            rclpy.shutdown()
            return

        request = GetEntityState.Request()
        request.entity = args.name
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        result = future.result()

        if result.result.result == 1:  # RESULT_OK
            p = result.state.pose.position
            o = result.state.pose.orientation
            node.get_logger().info(
                f"Entity '{args.name}': "
                f"pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) "
                f"ori=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})"
            )
        else:
            node.get_logger().error(f"Failed: {result.result.error_message}")
    else:
        client = node.create_client(GetEntities, "/sim/get_entities")
        if not client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error("GetEntities service not available")
            node.destroy_node()
            rclpy.shutdown()
            return

        request = GetEntities.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        result = future.result()

        if result.result.result == 1:  # RESULT_OK
            node.get_logger().info(f"Entities ({len(result.entities)}): {list(result.entities)}")
        else:
            node.get_logger().error(f"Failed: {result.result.error_message}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
