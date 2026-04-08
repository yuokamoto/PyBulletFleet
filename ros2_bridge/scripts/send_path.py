#!/usr/bin/env python3
"""Publish a square path for a robot to follow.

Usage::

    ros2 run pybullet_fleet_ros send_path          # default robot0
    python3 scripts/send_path.py --robot robot0 --size 3.0

The path is published once to ``/{robot}/path`` as a ``nav_msgs/Path``.
"""

import argparse
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node


def make_square_path(cx: float, cy: float, size: float, num_per_side: int = 5) -> Path:
    """Create a square path centered at (cx, cy)."""
    path = Path()
    path.header.frame_id = "odom"
    half = size / 2.0
    corners = [
        (cx - half, cy - half),
        (cx + half, cy - half),
        (cx + half, cy + half),
        (cx - half, cy + half),
        (cx - half, cy - half),  # close the loop
    ]
    for i in range(len(corners) - 1):
        x0, y0 = corners[i]
        x1, y1 = corners[i + 1]
        for j in range(num_per_side):
            t = j / num_per_side
            ps = PoseStamped()
            ps.header.frame_id = "odom"
            ps.pose.position.x = x0 + (x1 - x0) * t
            ps.pose.position.y = y0 + (y1 - y0) * t
            ps.pose.position.z = 0.05
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
    return path


def main():
    parser = argparse.ArgumentParser(description="Send a square path to a robot")
    parser.add_argument("--robot", default="robot0", help="Robot name (default: robot0)")
    parser.add_argument("--size", type=float, default=2.0, help="Square side length (default: 2.0)")
    parser.add_argument("--cx", type=float, default=0.0, help="Center X")
    parser.add_argument("--cy", type=float, default=0.0, help="Center Y")
    args = parser.parse_args()

    rclpy.init()
    node = Node("send_path_client")
    pub = node.create_publisher(Path, f"/{args.robot}/path", 10)

    # Wait for subscriber
    time.sleep(1.0)

    path = make_square_path(args.cx, args.cy, args.size)
    pub.publish(path)
    node.get_logger().info(f"Published square path ({len(path.poses)} waypoints) to /{args.robot}/path")

    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
