#!/usr/bin/env python3
"""Pattern 2 (batch API) smoke check — run inside the bridge container.

Assumes a bridge is running with ``fleet_interface: batch``. Verifies the O(1)
fleet endpoints:
  1. /fleet/states (FleetState) publishes one message covering all robots.
  2. /fleet/navigate SERVICE applies a batch of goals (num_set == #robots) and
     the robots actually move toward them.
  3. /fleet/navigate TOPIC (FleetGoal) drives a second batch (shared helper).

Exit 0 on pass, 1 on failure.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from pybullet_fleet_msgs.msg import FleetGoal, FleetState, RobotGoal
from pybullet_fleet_msgs.srv import FleetNavigate

EXPECTED = {"robot0", "robot1", "robot2"}
MOVE_THRESHOLD = 0.3  # metres
READY_TIMEOUT = 30.0
MOVE_TIMEOUT = 30.0


class BatchChecker(Node):
    def __init__(self):
        super().__init__("batch_check")
        self._states = {}  # name -> (x, y)
        self._msg_count = 0
        self.create_subscription(FleetState, "/fleet/states", self._on_states, 10)
        self._goal_pub = self.create_publisher(FleetGoal, "/fleet/navigate", 10)
        self._nav = self.create_client(FleetNavigate, "/fleet/navigate")

    def _on_states(self, msg: FleetState):
        self._msg_count += 1
        for r in msg.robots:
            self._states[r.name] = (r.x, r.y)

    def spin(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_states(self, timeout) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if EXPECTED.issubset(self._states):
                return True
        return False

    def wait_moved(self, start, timeout) -> float:
        """Return the max displacement reached within timeout."""
        end = time.monotonic() + timeout
        best = 0.0
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            for n, (x, y) in self._states.items():
                if n in start:
                    best = max(best, abs(x - start[n][0]) + abs(y - start[n][1]))
            if best >= MOVE_THRESHOLD:
                break
        return best


def goals_plus_x(states, dx=3.0):
    return [RobotGoal(name=n, x=x + dx, y=y, yaw=0.0) for n, (x, y) in states.items()]


def main() -> int:
    rclpy.init()
    node = BatchChecker()
    log = node.get_logger()

    # Gate 1: aggregated /fleet/states with all robots.
    log.info("waiting for /fleet/states to cover all robots ...")
    if not node.wait_states(READY_TIMEOUT):
        log.error(f"FAIL: /fleet/states incomplete; saw {sorted(node._states)} (msgs={node._msg_count})")
        return 1
    log.info(f"STATES gate: /fleet/states reports {sorted(node._states)} (1 aggregated topic)")

    # Gate 2: /fleet/navigate SERVICE moves the fleet.
    if not node._nav.wait_for_service(timeout_sec=10.0):
        log.error("FAIL: /fleet/navigate service unavailable")
        return 1
    start = dict(node._states)
    req = FleetNavigate.Request(goals=goals_plus_x(start, 3.0))
    future = node._nav.call_async(req)
    while rclpy.ok() and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    resp = future.result()
    if not resp.accepted or resp.num_set != len(start):
        log.error(f"FAIL: service applied {resp.num_set}/{len(start)} (accepted={resp.accepted})")
        return 1
    moved = node.wait_moved(start, MOVE_TIMEOUT)
    if moved < MOVE_THRESHOLD:
        log.error(f"FAIL: fleet did not move after service navigate (max {moved:.2f} m)")
        return 1
    log.info(f"SERVICE gate: num_set={resp.num_set}, fleet moved {moved:.2f} m")

    # Gate 3: /fleet/navigate TOPIC drives a second batch.
    start2 = dict(node._states)
    node._goal_pub.publish(FleetGoal(goals=goals_plus_x(start2, 3.0)))
    moved2 = node.wait_moved(start2, MOVE_TIMEOUT)
    if moved2 < MOVE_THRESHOLD:
        log.error(f"FAIL: fleet did not move after topic navigate (max {moved2:.2f} m)")
        return 1
    log.info(f"TOPIC gate: fleet moved {moved2:.2f} m")

    log.info("PASS: batch /fleet/states + /fleet/navigate (service & topic) work")
    return 0


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
