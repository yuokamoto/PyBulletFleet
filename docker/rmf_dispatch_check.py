#!/usr/bin/env python3
"""RMF dispatch end-to-end check (run inside the bridge container).

Assumes an RMF demo (office/hotel/...) is already launched and publishing. Runs
one or more dispatch SCENARIOS sequentially in the single running demo and proves
the full chain for each:

    dispatch_*  ->  RMF bidding  ->  fleet adapter  ->  NavigateToPose / actions
        ->  pybullet_fleet bridge  ->  PyBullet  ->  the task reaches `completed`

Scenarios are given as argv tokens ``type:arg1,arg2,...``:
    patrol:lounge,coe
    delivery:pantry,coke_dispenser,hardware_2,coke_ingestor
    clean:clean_lobby

Each scenario PASSES when its dispatched task reaches ``completed`` on
``/task_state_update`` within ``SCENARIO_TIMEOUT`` (we also log `underway` and
robot motion as progress). Reaching `completed` is the meaningful assertion: for
delivery it proves the dispenser dispensed + ingestor ingested; for a cross-floor
patrol it proves the robot rode the lift; for clean it proves the coverage path
ran. All scenarios must pass for exit 0.

Every dispatch is sent with --use_sim_time: the demos run RMF on the simulated
/clock, so a wall-clock earliest-start would sit in the sim future and the task
would never leave `queued`.
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
from rmf_fleet_msgs.msg import FleetState
from std_msgs.msg import String

MOVE_THRESHOLD = 0.3  # metres; comfortably above odom noise
# Task ids look like "<category>.dispatch-<hex>". Note clean dispatches a *composed*
# task, so its id is "compose.dispatch-..." (not "clean.dispatch-..."); match any.
TASK_ID_RE = re.compile(r"\b[a-z_]+\.dispatch-[0-9a-fA-F]+")

READY_TIMEOUT = 150.0  # wait for the demo stack + fleet adapter
SCENARIO_TIMEOUT = 300.0  # per-scenario wait for the task to reach `completed`
SETTLE_MAX = 60.0  # wait for robots to go quiet before dispatching (best effort)
SETTLE_WINDOW = 5.0  # contiguous quiet span that counts as "settled"
SETTLE_EPS = 0.05  # metres of jitter tolerated while "quiet"
TERMINAL_OK = ("completed",)
TERMINAL_BAD = ("failed", "canceled", "killed")


def build_dispatch(scenario: str):
    """Map a ``type:csv`` scenario token to a dispatch_* command (+ --use_sim_time)."""
    kind, _, rest = scenario.partition(":")
    args = [a for a in rest.split(",") if a]
    if kind == "patrol":
        return ["ros2", "run", "rmf_demos_tasks", "dispatch_patrol", "-p", *args, "-n", "1", "--use_sim_time"]
    if kind == "delivery":
        pickup, pick_h, dropoff, drop_h = args
        return [
            "ros2",
            "run",
            "rmf_demos_tasks",
            "dispatch_delivery",
            "-p",
            pickup,
            "-ph",
            pick_h,
            "-d",
            dropoff,
            "-dh",
            drop_h,
            "--use_sim_time",
        ]
    if kind == "clean":
        return ["ros2", "run", "rmf_demos_tasks", "dispatch_clean", "-cs", args[0], "--use_sim_time"]
    raise ValueError(f"unknown scenario kind: {kind!r}")


class DispatchChecker(Node):
    def __init__(self):
        super().__init__("rmf_dispatch_check")
        self._pos = {}  # robot name -> latest (x, y)
        self._subs = {}  # robot name -> odom subscription
        self._task_status = {}  # task_id -> latest status string
        self.create_subscription(FleetState, "/fleet_states", self._on_fleet, 10)
        self.create_subscription(String, "/task_state_update", self._on_task_state, 10)

    def _on_fleet(self, msg: FleetState):
        # Discover robots across all fleets and subscribe to each one's odom once.
        for r in msg.robots:
            if r.name not in self._subs:
                self._pos.setdefault(r.name, None)
                self._subs[r.name] = self.create_subscription(Odometry, f"/{r.name}/odom", self._make_odom_cb(r.name), 10)

    def _make_odom_cb(self, robot):
        def _cb(msg: Odometry):
            p = msg.pose.pose.position
            self._pos[robot] = (p.x, p.y)

        return _cb

    def _on_task_state(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        # rmf_api_msgs wraps the state as {"type":..., "data": {...}}; tolerate either.
        state = payload.get("data", payload) if isinstance(payload, dict) else {}
        booking = state.get("booking", {}) if isinstance(state, dict) else {}
        task_id, status = booking.get("id"), state.get("status")
        if task_id and status:
            self._task_status[task_id] = status

    def spin_for(self, seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_fleet(self, timeout: float) -> bool:
        """Return True once at least one robot is known and reporting odom."""
        end = time.monotonic() + timeout
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self._pos and all(p is not None for p in self._pos.values()):
                return True
        return False

    def wait_until_settled(self, max_wait, window, eps) -> bool:
        deadline = time.monotonic() + max_wait
        ref = dict(self._pos)
        ref_t = time.monotonic()
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            moved = max(
                (
                    math.hypot(self._pos[r][0] - ref[r][0], self._pos[r][1] - ref[r][1])
                    for r in self._pos
                    if self._pos.get(r) and ref.get(r)
                ),
                default=0.0,
            )
            if moved > eps:
                ref, ref_t = dict(self._pos), time.monotonic()
            elif time.monotonic() - ref_t >= window:
                return True
        return False

    def max_displacement(self, start) -> float:
        best = 0.0
        for r, s in start.items():
            p = self._pos.get(r)
            if s and p:
                best = max(best, math.hypot(p[0] - s[0], p[1] - s[1]))
        return best


def dispatch(node, log, scenario):
    cmd = build_dispatch(scenario)
    log.info(f"dispatching [{scenario}]: {' '.join(cmd)}")
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    except subprocess.TimeoutExpired:
        log.warn("dispatch command timed out (continuing to watch)")
        return None
    if out.returncode != 0:
        log.warn(f"dispatch returned {out.returncode}: {out.stderr.strip()}")
    m = TASK_ID_RE.search(out.stdout) or TASK_ID_RE.search(out.stderr)
    task_id = m.group(0) if m else None
    log.info(f"dispatch acknowledged; task_id={task_id}")
    return task_id


def run_scenario(node, log, scenario) -> bool:
    """Dispatch one scenario and wait for its task to reach `completed`."""
    if node.wait_until_settled(SETTLE_MAX, SETTLE_WINDOW, SETTLE_EPS):
        log.info("robots quiet; dispatching")
    else:
        log.warn("robots still moving; dispatching anyway")
    start = dict(node._pos)
    task_id = dispatch(node, log, scenario)

    deadline = time.monotonic() + SCENARIO_TIMEOUT
    seen_underway = False
    seen_motion = False
    while time.monotonic() < deadline and rclpy.ok():
        node.spin_for(1.0)
        status = node._task_status.get(task_id) if task_id else None
        if not seen_underway and (status in ("underway",) or status in TERMINAL_OK):
            seen_underway = True
            log.info(f"[{scenario}] task underway")
        disp = node.max_displacement(start)
        if not seen_motion and disp >= MOVE_THRESHOLD:
            seen_motion = True
            log.info(f"[{scenario}] a robot moved {disp:.2f} m")
        if status in TERMINAL_OK:
            log.info(f"[{scenario}] PASS: task reached `{status}`")
            return True
        if status in TERMINAL_BAD:
            log.error(f"[{scenario}] FAIL: task reached terminal `{status}`")
            return False
    log.error(
        f"[{scenario}] FAIL after {SCENARIO_TIMEOUT:.0f}s — last status="
        f"{node._task_status.get(task_id)}, underway={seen_underway}, moved={seen_motion}"
    )
    return False


def main(scenarios) -> int:
    rclpy.init()
    node = DispatchChecker()
    log = node.get_logger()

    log.info("waiting for demo stack + fleet odom ...")
    if not node.wait_for_fleet(READY_TIMEOUT):
        log.error("timed out waiting for fleet odom; demo did not come up")
        return 1
    log.info(f"fleet up: robots={sorted(node._pos)}")

    results = {s: run_scenario(node, log, s) for s in scenarios}
    ok = all(results.values())
    log.info(f"RESULTS: {results}")
    return 0 if ok else 1


if __name__ == "__main__":
    scenario_args = sys.argv[1:]
    if not scenario_args:
        print("usage: rmf_dispatch_check.py <type:args> [<type:args> ...]", file=sys.stderr)
        sys.exit(2)
    rc = 1
    try:
        rc = main(scenario_args)
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass
    sys.exit(rc)
