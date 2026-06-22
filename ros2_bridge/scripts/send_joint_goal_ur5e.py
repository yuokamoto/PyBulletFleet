#!/usr/bin/env python3
"""Send a FollowJointTrajectory goal to a UR5e arm (Tier 2 demo).

Convenience wrapper around ``send_joint_goal.py`` with UR5e-specific
defaults (6-DOF joint names and example poses).

Usage::

    # Home position (all zeros)
    python3 scripts/send_joint_goal_ur5e.py

    # Pre-defined named pose
    python3 scripts/send_joint_goal_ur5e.py --pose ready
    python3 scripts/send_joint_goal_ur5e.py --pose pick
    python3 scripts/send_joint_goal_ur5e.py --pose place

    # Custom joint positions (6 values)
    python3 scripts/send_joint_goal_ur5e.py --positions 0.0 -1.57 1.57 -1.57 -1.57 0.0

    # Different robot name
    python3 scripts/send_joint_goal_ur5e.py --robot my_ur5e --pose ready
"""

import argparse

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# UR5e 6-DOF joint names
UR5E_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Named poses (joint positions in radians)
NAMED_POSES = {
    "home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "ready": [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
    "pick": [0.5, -1.2, 1.8, -2.17, -1.57, 0.0],
    "place": [-0.5, -1.2, 1.8, -2.17, -1.57, 0.0],
    "up": [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
}


def main():
    parser = argparse.ArgumentParser(description="Send FollowJointTrajectory goal to UR5e")
    parser.add_argument("--robot", default="ur5e_0", help="Robot name (default: ur5e_0)")
    parser.add_argument("--pose", choices=list(NAMED_POSES.keys()), help="Named pose to send")
    parser.add_argument("--positions", nargs=6, type=float, help="Custom joint positions (6 values, radians)")
    parser.add_argument("--duration", type=float, default=3.0, help="Trajectory duration (seconds)")
    args = parser.parse_args()

    # Determine target positions
    if args.positions is not None:
        positions = args.positions
    elif args.pose is not None:
        positions = NAMED_POSES[args.pose]
    else:
        positions = NAMED_POSES["home"]

    rclpy.init()
    node = Node("send_joint_goal_ur5e")
    client = ActionClient(node, FollowJointTrajectory, f"/{args.robot}/follow_joint_trajectory")

    node.get_logger().info(f"Waiting for /{args.robot}/follow_joint_trajectory action server...")
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error("Action server not available")
        node.destroy_node()
        rclpy.shutdown()
        return

    # Build trajectory
    traj = JointTrajectory()
    traj.joint_names = UR5E_JOINTS

    point = JointTrajectoryPoint()
    point.positions = positions
    sec = int(args.duration)
    nsec = int((args.duration - sec) * 1e9)
    point.time_from_start = Duration(sec=sec, nanosec=nsec)
    traj.points = [point]

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj

    pose_name = args.pose or "custom"
    node.get_logger().info(f"Sending UR5e goal '{pose_name}': {[f'{p:.2f}' for p in positions]}")

    def feedback_cb(feedback_msg):
        fb = feedback_msg.feedback
        actual = [f"{p:.3f}" for p in fb.actual.positions] if fb.actual.positions else []
        node.get_logger().info(f"Feedback: actual={actual}")

    future = client.send_goal_async(goal, feedback_callback=feedback_cb)
    rclpy.spin_until_future_complete(node, future)

    goal_handle = future.result()
    if not goal_handle.accepted:
        node.get_logger().error("Goal rejected")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info("Goal accepted, waiting for result...")
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)

    result = result_future.result()
    node.get_logger().info(f"Result: error_code={result.result.error_code}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
