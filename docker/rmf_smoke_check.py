#!/usr/bin/env python3
"""Reliable RMF integration smoke check (run inside the bridge container).

Verifies the RMF stack stands up on the bridge and that the bridge executes RMF
navigation commands — WITHOUT depending on RMF's dispatcher or traffic schedule
(the full dispatch->patrol path is covered separately by test_rmf_dispatch_e2e.sh).
Keeping this gate dispatcher-free makes it a fast, deterministic signal.

Gates (all required):
  1. FLEET: /fleet_states reports every expected robot -> the tinyRobot fleet
     adapter is up and publishing.
  2. NAV: a DIRECT NavigateToPose goal to a robot's own
     ``/<robot>/navigate_to_pose`` action server moves it >= MOVE_THRESHOLD m.
     This exercises the bridge's RMF-facing action server, controller, PyBullet
     execution and odom feedback — deterministically (no bidding / no traffic
     negotiation), so it is stable on both WSL2 and CI.

(Topic presence for the door/lift/dispenser/ingestor handlers is checked in the
wrapper, test_rmf_smoke.sh, via `ros2 topic list`.)

Exit 0 on pass, 1 on failure.
"""

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rmf_fleet_msgs.msg import FleetState

EXPECTED_ROBOTS = ["tinyRobot1", "tinyRobot2"]
NAV_ROBOT = "tinyRobot2"  # idle at its charger -> motion is unambiguous
# ~1.5 m off tinyRobot2_charger (20.42, -5.31) toward the room interior (the
# robot already faces ~pi), comfortably reachable and collision-free.
NAV_GOAL_XY = (18.9, -5.31)
MOVE_THRESHOLD = 0.3  # metres; comfortably above odom noise

FLEET_TIMEOUT = 150.0  # wait for the demo stack + fleet adapter
SERVER_TIMEOUT = 30.0  # wait for the bridge's nav action server
MOVE_TIMEOUT = 60.0  # wait for the robot to drive after the goal is sent


class SmokeChecker(Node):
    def __init__(self):
        super().__init__("rmf_smoke_check")
        self._fleet_robots = set()  # names seen on /fleet_states
        self._pos = None  # latest (x, y) of NAV_ROBOT
        self.create_subscription(FleetState, "/fleet_states", self._on_fleet, 10)
        self.create_subscription(Odometry, f"/{NAV_ROBOT}/odom", self._on_odom, 10)
        self._nav = ActionClient(self, NavigateToPose, f"/{NAV_ROBOT}/navigate_to_pose")

    def _on_fleet(self, msg: FleetState):
        for r in msg.robots:
            self._fleet_robots.add(r.name)

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._pos = (p.x, p.y)

    def spin_for(self, seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_fleet(self, timeout: float) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if set(EXPECTED_ROBOTS).issubset(self._fleet_robots) and self._pos is not None:
                return True
        return False


def main() -> int:
    rclpy.init()
    node = SmokeChecker()
    log = node.get_logger()

    # Gate 1: fleet adapter up and reporting both robots.
    log.info("waiting for /fleet_states to report all robots ...")
    if not node.wait_for_fleet(FLEET_TIMEOUT):
        log.error(
            f"FAIL: fleet not ready. saw robots={sorted(node._fleet_robots)}, "
            f"{NAV_ROBOT} odom={'yes' if node._pos else 'no'}"
        )
        return 1
    log.info(f"FLEET gate: /fleet_states reports {sorted(node._fleet_robots)}")

    # Gate 2: direct NavigateToPose drives the robot (bridge execution contract).
    if not node._nav.wait_for_server(timeout_sec=SERVER_TIMEOUT):
        log.error(f"FAIL: {NAV_ROBOT}/navigate_to_pose action server not available")
        return 1
    start = node._pos
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x = float(NAV_GOAL_XY[0])
    goal.pose.pose.position.y = float(NAV_GOAL_XY[1])
    goal.pose.pose.orientation.w = 1.0
    log.info(f"sending direct NavigateToPose {NAV_GOAL_XY} to {NAV_ROBOT} (start={start})")
    send_future = node._nav.send_goal_async(goal)

    # Wait for the server to accept/reject the goal, so a rejection fails fast and
    # clearly instead of silently timing out on "no motion".
    accept_deadline = time.monotonic() + 15.0
    while time.monotonic() < accept_deadline and rclpy.ok() and not send_future.done():
        rclpy.spin_once(node, timeout_sec=0.2)
    if not send_future.done():
        log.error("FAIL: NavigateToPose goal was not acknowledged by the action server")
        return 1
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        log.error("FAIL: NavigateToPose goal was rejected by the action server")
        return 1

    deadline = time.monotonic() + MOVE_TIMEOUT
    while time.monotonic() < deadline and rclpy.ok():
        node.spin_for(1.0)
        moved = math.hypot(node._pos[0] - start[0], node._pos[1] - start[1])
        if moved >= MOVE_THRESHOLD:
            log.info(f"NAV gate: {NAV_ROBOT} moved {moved:.2f} m on a direct NavigateToPose")
            log.info("PASS: RMF stack is up and the bridge executes nav commands")
            return 0

    log.error(
        f"FAIL: {NAV_ROBOT} did not move >= {MOVE_THRESHOLD} m within "
        f"{MOVE_TIMEOUT:.0f}s of a direct NavigateToPose (pos={node._pos})"
    )
    return 1


if __name__ == "__main__":
    rc = 1
    try:
        rc = main()
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
    sys.exit(rc)
