#!/usr/bin/env python3
"""Send cmd_vel commands to a robot via ROS 2 topic.

Usage:
    python3 scripts/teleop_cmd_vel.py --robot robot_0 --vx 1.0 --wz 0.2 --duration 5.0
    python3 scripts/teleop_cmd_vel.py --robot robot_0 --vx 0.5  # runs until Ctrl+C
"""

import argparse
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelPublisher(Node):
    def __init__(self, robot_name: str, vx: float, vy: float, vz: float, wz: float, rate: float):
        super().__init__("teleop_cmd_vel")
        self._pub = self.create_publisher(Twist, f"/{robot_name}/cmd_vel", 10)
        self._twist = Twist()
        self._twist.linear.x = vx
        self._twist.linear.y = vy
        self._twist.linear.z = vz
        self._twist.angular.z = wz
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f"Publishing cmd_vel to /{robot_name}/cmd_vel: " f"vx={vx}, vy={vy}, vz={vz}, wz={wz} at {rate}Hz"
        )

    def _publish(self):
        self._pub.publish(self._twist)


def main():
    parser = argparse.ArgumentParser(description="Send cmd_vel to a robot")
    parser.add_argument("--robot", default="robot0", help="Robot name")
    parser.add_argument("--vx", type=float, default=0.5, help="Linear X velocity (m/s)")
    parser.add_argument("--vy", type=float, default=0.0, help="Linear Y velocity (m/s)")
    parser.add_argument("--vz", type=float, default=0.0, help="Linear Z velocity (m/s)")
    parser.add_argument("--wz", type=float, default=0.0, help="Angular Z velocity (rad/s)")
    parser.add_argument("--rate", type=float, default=10.0, help="Publish rate (Hz)")
    parser.add_argument("--duration", type=float, default=0.0, help="Duration in seconds (0=infinite)")
    args = parser.parse_args()

    rclpy.init()
    node = CmdVelPublisher(args.robot, args.vx, args.vy, args.vz, args.wz, args.rate)

    if args.duration > 0:
        start = time.time()
        while rclpy.ok() and (time.time() - start) < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
        # Send zero velocity to stop
        stop_twist = Twist()
        node._pub.publish(stop_twist)
        time.sleep(0.1)
    else:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
