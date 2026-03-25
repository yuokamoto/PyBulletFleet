#!/usr/bin/env python3
"""Send a FollowJointTrajectory goal to an arm robot.

Usage::

    # Default joints (arm_robot.urdf) with default positions
    python3 scripts/send_joint_goal.py --robot arm0

    # Custom positions
    python3 scripts/send_joint_goal.py --robot arm0 --positions 0.5 1.0 -0.5 0.0

    # Custom joints and positions
    python3 scripts/send_joint_goal.py --robot arm0 \
        --joints base_to_shoulder shoulder_to_elbow --positions 0.8 -0.3

Uses the ``FollowJointTrajectory`` action server at ``/{robot}/follow_joint_trajectory``.
Prints feedback (actual/desired/error positions) until the goal succeeds.
"""

import argparse

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main():
    parser = argparse.ArgumentParser(description="Send FollowJointTrajectory goal")
    parser.add_argument("--robot", default="arm0", help="Robot name")
    parser.add_argument(
        "--joints",
        nargs="+",
        default=["base_to_shoulder", "shoulder_to_elbow", "elbow_to_wrist", "wrist_to_end"],
        help="Joint names (default: arm_robot.urdf joints)",
    )
    parser.add_argument(
        "--positions",
        nargs="+",
        type=float,
        default=[0.5, 1.0, -0.5, 0.0],
        help="Target joint positions (radians)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=2.0,
        help="Time for trajectory point (seconds)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = Node("send_joint_goal_client")
    client = ActionClient(node, FollowJointTrajectory, f"/{args.robot}/follow_joint_trajectory")

    node.get_logger().info(f"Waiting for /{args.robot}/follow_joint_trajectory action server...")
    client.wait_for_server()

    # Build trajectory
    traj = JointTrajectory()
    traj.joint_names = args.joints

    point = JointTrajectoryPoint()
    point.positions = args.positions
    sec = int(args.duration)
    nsec = int((args.duration - sec) * 1e9)
    point.time_from_start = Duration(sec=sec, nanosec=nsec)
    traj.points = [point]

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj

    node.get_logger().info(f"Sending FollowJointTrajectory goal: joints={args.joints}, positions={args.positions}")

    def feedback_cb(feedback_msg):
        fb = feedback_msg.feedback
        actual = [f"{p:.3f}" for p in fb.actual.positions] if fb.actual.positions else []
        desired = [f"{p:.3f}" for p in fb.desired.positions] if fb.desired.positions else []
        error = [f"{p:.3f}" for p in fb.error.positions] if fb.error.positions else []
        node.get_logger().info(f"  actual={actual}  desired={desired}  error={error}")

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

    result = result_future.result().result
    if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
        node.get_logger().info("FollowJointTrajectory completed successfully!")
    else:
        node.get_logger().error(f"FollowJointTrajectory failed with error_code={result.error_code}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
