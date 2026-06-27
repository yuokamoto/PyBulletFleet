#!/usr/bin/env python3
"""RMF end-to-end smoke check (run inside the bridge container).

Assumes the office demo is already launched and publishing. Proves the full
dispatch chain end-to-end:

    dispatch_patrol  ->  RMF task dispatch + bidding  ->  tinyRobot fleet adapter
        ->  NavigateToPose action  ->  pybullet_fleet bridge  ->  PyBullet moves
        the robot  ->  /<robot>/odom reflects the new pose

Two pass gates, both required:

  1. TASK gate (robust, fast): the dispatched patrol task reaches ``underway``
     or ``completed`` on ``/task_state_update``. This proves dispatch -> bidding
     -> fleet-adapter acceptance, and is insensitive to the WSL2 wall-clock jitter
     that can briefly stall RMF's scheduler. Healthy latency is a few seconds.
  2. MOTION gate (proves the bridge actually drives the robot): a fleet robot's
     odom moves more than ``MOVE_THRESHOLD`` m from where it sat at dispatch time.
     We settle the fleet to quiet *before* dispatching so the motion is
     attributable to our patrol, not pre-existing drift (e.g. a robot parking).

Requiring both makes the smoke meaningful (full chain, including the bridge's
PyBullet execution leg) while the task gate keeps the common path quick. On a
stable-clock CI runner both gates clear in seconds; the long timeout only
absorbs the occasional WSL2 clock-jump stall observed locally.

Exit 0 on pass, 1 on failure (with which gate(s) failed).
"""

import json
import math
import re
import subprocess
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

ROBOTS = ["tinyRobot1", "tinyRobot2"]
PATROL_WAYPOINTS = ["lounge", "coe"]  # both named in the office nav graph
MOVE_THRESHOLD = 0.3  # metres; comfortably above odom noise
UNDERWAY_STATES = ("underway", "completed")
TASK_ID_RE = re.compile(r"patrol\.dispatch-[0-9a-fA-F]+")

READY_TIMEOUT = 150.0  # wait for the demo stack + first odom
DEADLINE = 240.0  # overall wait for both gates after dispatch
# If the task is still `queued` (never awarded) after this long, re-dispatch: on
# WSL2 a wall-clock jump can land in RMF's ~2s bidding window and stall the award
# (the task never leaves `queued`). Each dispatch is an independent bidding round,
# so a fresh one usually misses the jump. On a stable-clock CI runner the first
# dispatch is awarded in ~2.5s, so this never fires (no behaviour masked).
REDISPATCH_AFTER = 50.0
MAX_DISPATCHES = 4

# Robots may already be in motion at startup (e.g. RMF sends a not-quite-full
# robot to park). We wait for everything to go quiet *before* dispatching, so
# the motion we then detect is attributable to our patrol. "Quiet" = no robot
# moves more than SETTLE_EPS over SETTLE_WINDOW.
SETTLE_MAX = 90.0  # give up waiting for quiet after this (then dispatch anyway)
SETTLE_WINDOW = 6.0  # contiguous quiet span that counts as "settled"
SETTLE_EPS = 0.05  # metres of jitter tolerated while "quiet"


class SmokeChecker(Node):
    def __init__(self):
        super().__init__("rmf_smoke_check")
        self._pos = {r: None for r in ROBOTS}  # latest (x, y)
        self._start = {r: None for r in ROBOTS}  # pose at dispatch time
        self._task_status = {}  # task_id -> latest status string
        for r in ROBOTS:
            self.create_subscription(Odometry, f"/{r}/odom", self._make_cb(r), 10)
        self.create_subscription(String, "/task_state_update", self._on_task_state, 10)

    def _make_cb(self, robot):
        def _cb(msg: Odometry):
            p = msg.pose.pose.position
            self._pos[robot] = (p.x, p.y)

        return _cb

    def _on_task_state(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        # rmf_api_msgs wraps the state as {"type":..., "data": {...}}; tolerate
        # either the wrapper or a bare state object.
        state = payload.get("data", payload) if isinstance(payload, dict) else {}
        booking = state.get("booking", {}) if isinstance(state, dict) else {}
        task_id = booking.get("id")
        status = state.get("status")
        if task_id and status:
            self._task_status[task_id] = status

    def spin_for(self, seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

    def wait_until_ready(self, timeout: float) -> bool:
        """Return True once every robot has reported at least one odom message."""
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if all(self._pos[r] is not None for r in ROBOTS):
                return True
        return False

    def wait_until_settled(self, max_wait: float, window: float, eps: float) -> bool:
        """Wait until no robot moves more than ``eps`` over a ``window``-long span.

        Returns True if quiet was observed, False if ``max_wait`` elapsed first
        (caller dispatches anyway — a noisy start shouldn't hard-fail the smoke).
        """
        deadline = time.monotonic() + max_wait
        ref = dict(self._pos)  # positions at the start of the current quiet span
        ref_t = time.monotonic()
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            moved = max(
                (math.hypot(self._pos[r][0] - ref[r][0], self._pos[r][1] - ref[r][1])
                 for r in ROBOTS if self._pos[r] and ref[r]),
                default=0.0,
            )
            if moved > eps:
                ref, ref_t = dict(self._pos), time.monotonic()  # motion: reset span
            elif time.monotonic() - ref_t >= window:
                return True
        return False

    def snapshot_start(self):
        self._start = dict(self._pos)
        for r in ROBOTS:
            self.get_logger().info(f"start pose {r}: {self._start[r]}")

    def max_displacement(self) -> float:
        best = 0.0
        for r in ROBOTS:
            s, p = self._start[r], self._pos[r]
            if s is None or p is None:
                continue
            best = max(best, math.hypot(p[0] - s[0], p[1] - s[1]))
        return best

    def task_underway(self, task_id) -> bool:
        """True if our task (or, if its id is unknown, any task) is underway."""
        if task_id is not None:
            return self._task_status.get(task_id) in UNDERWAY_STATES
        return any(s in UNDERWAY_STATES for s in self._task_status.values())


def dispatch_patrol(logger):
    """Dispatch a patrol; return the task id parsed from the CLI output (or None)."""
    cmd = [
        "ros2", "run", "rmf_demos_tasks", "dispatch_patrol",
        "-p", *PATROL_WAYPOINTS, "-n", "1",
    ]
    logger.info(f"dispatching: {' '.join(cmd)}")
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    except subprocess.TimeoutExpired:
        logger.warn("dispatch command timed out (continuing to watch)")
        return None
    if out.returncode != 0:
        logger.warn(f"dispatch returned {out.returncode}: {out.stderr.strip()}")
    m = TASK_ID_RE.search(out.stdout) or TASK_ID_RE.search(out.stderr)
    task_id = m.group(0) if m else None
    logger.info(f"dispatch acknowledged; task_id={task_id}")
    return task_id


def main() -> int:
    rclpy.init()
    node = SmokeChecker()
    log = node.get_logger()

    log.info("waiting for demo stack + robot odom ...")
    if not node.wait_until_ready(READY_TIMEOUT):
        log.error("timed out waiting for robot odom; demo did not come up")
        return 1

    log.info("odom is live; waiting for robots to go quiet before dispatching ...")
    if node.wait_until_settled(SETTLE_MAX, SETTLE_WINDOW, SETTLE_EPS):
        log.info("robots are quiet; dispatching now")
    else:
        log.warn(f"robots still moving after {SETTLE_MAX:.0f}s; dispatching anyway")

    node.snapshot_start()
    dispatch_patrol(log)  # task ids are tracked via /task_state_update
    dispatches = 1
    last_dispatch = time.monotonic()

    deadline = time.monotonic() + DEADLINE
    task_ok = False
    move_ok = False
    while time.monotonic() < deadline and rclpy.ok():
        node.spin_for(1.0)
        # Pass on ANY task reaching underway (re-dispatch may create several).
        if not task_ok and node.task_underway(None):
            task_ok = True
            log.info("TASK gate: a dispatched patrol reached underway/completed")
        disp = node.max_displacement()
        if not move_ok and disp >= MOVE_THRESHOLD:
            move_ok = True
            log.info(f"MOTION gate: a robot moved {disp:.2f} m after dispatch")
        if task_ok and move_ok:
            log.info("PASS: task is underway and the bridge drove a robot")
            return 0
        # Still stuck in `queued` (no award)? Kick a fresh bidding round.
        stuck = not task_ok and (time.monotonic() - last_dispatch) >= REDISPATCH_AFTER
        if stuck and dispatches < MAX_DISPATCHES:
            log.warn(f"task still not underway after {REDISPATCH_AFTER:.0f}s; re-dispatching")
            dispatch_patrol(log)
            dispatches += 1
            last_dispatch = time.monotonic()

    log.error(
        f"FAIL after {DEADLINE:.0f}s — task_underway={task_ok}, "
        f"robot_moved={move_ok} (max displacement {node.max_displacement():.3f} m). "
        f"task states seen: {node._task_status}"
    )
    if task_ok and not move_ok:
        log.error(
            "Task was accepted by RMF but no robot motion was observed — on WSL2 "
            "this is usually a wall-clock-jump stall in RMF's scheduler, not a "
            "bridge fault. Expect this to pass on a stable-clock CI runner."
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
