#!/usr/bin/env python3
"""Send a NavigateToPose goal to a robot.

Usage::

    python3 scripts/send_nav_goal.py --robot robot0 --x 5.0 --y 3.0

Uses the ``NavigateToPose`` action server at ``/{robot}/navigate_to_pose``.
Prints feedback (distance_remaining) until the goal succeeds.
"""

import argparse

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def main():
    parser = argparse.ArgumentParser(description="Send NavigateToPose goal")
    parser.add_argument("--robot", default="robot0")
    parser.add_argument("--x", type=float, default=5.0)
    parser.add_argument("--y", type=float, default=3.0)
    parser.add_argument("--z", type=float, default=0.05)
    args = parser.parse_args()

    rclpy.init()
    node = Node("send_nav_goal_client")
    client = ActionClient(node, NavigateToPose, f"/{args.robot}/navigate_to_pose")

    node.get_logger().info(f"Waiting for /{args.robot}/navigate_to_pose action server...")
    client.wait_for_server()

    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "odom"
    goal.pose.pose.position.x = args.x
    goal.pose.pose.position.y = args.y
    goal.pose.pose.position.z = args.z
    goal.pose.pose.orientation.w = 1.0

    node.get_logger().info(f"Sending NavigateToPose goal: ({args.x}, {args.y}, {args.z})")

    def feedback_cb(feedback_msg):
        fb = feedback_msg.feedback
        node.get_logger().info(f"  distance_remaining: {fb.distance_remaining:.2f}")

    future = client.send_goal_async(goal, feedback_callback=feedback_cb)
    rclpy.spin_until_future_complete(node, future)

    goal_handle = future.result()
    if not goal_handle.accepted:
        node.get_logger().error("Goal rejected!")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info("Goal accepted, waiting for result...")
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    node.get_logger().info("NavigateToPose completed!")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
