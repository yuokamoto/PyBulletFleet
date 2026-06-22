#!/usr/bin/env python3
"""Send a joint trajectory to an arm robot.

Usage::

    python3 scripts/send_joint_trajectory.py --robot robot0
    python3 scripts/send_joint_trajectory.py --robot robot0 --positions 0.5 1.0 -0.5 0.0

Publishes a ``trajectory_msgs/JointTrajectory`` to ``/{robot}/joint_trajectory``.
"""

import argparse
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main():
    parser = argparse.ArgumentParser(description="Send a joint trajectory")
    parser.add_argument("--robot", default="robot0", help="Robot name")
    parser.add_argument(
        "--joints",
        nargs="+",
        default=["base_to_shoulder", "shoulder_to_elbow", "elbow_to_wrist", "wrist_to_end"],
        help="Joint names",
    )
    parser.add_argument(
        "--positions",
        nargs="+",
        type=float,
        default=[0.5, 1.0, -0.5, 0.0],
        help="Target joint positions",
    )
    args = parser.parse_args()

    rclpy.init()
    node = Node("send_joint_trajectory_client")
    pub = node.create_publisher(JointTrajectory, f"/{args.robot}/joint_trajectory", 10)

    time.sleep(1.0)

    msg = JointTrajectory()
    msg.joint_names = args.joints
    point = JointTrajectoryPoint()
    point.positions = args.positions
    point.time_from_start = Duration(sec=2, nanosec=0)
    msg.points = [point]

    pub.publish(msg)
    node.get_logger().info(
        f"Published JointTrajectory to /{args.robot}/joint_trajectory: " f"joints={args.joints}, positions={args.positions}"
    )

    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
